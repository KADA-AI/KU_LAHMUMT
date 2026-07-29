from __future__ import annotations

import json
import math
import threading
import time
from collections import Counter, deque
from copy import deepcopy
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from modules.common import agent_status_snapshot, db_paths
from modules.mission_status_monitoring.footprint_context import (
    build_0401_footprint_context,
    footprint_for_aircraft,
    position_for_aircraft,
)
from modules.monitoring.logic.mission_update import (
    build_uav_mission_view,
    extract_0401_agent_states,
)
from modules.monitoring.logic.fuel_warning import resolve_fuel_capacity_liters
from modules.sim.mission.mission_plan_loader import build_mission_plan_payload
from modules.sim.mission.monitoring_loader import build_monitoring_snapshot
from modules.sim.mission.remaining_area_loader import build_remaining_area_snapshot

try:
    from pyproj import CRS, Transformer
    from shapely.geometry import LineString, mapping, shape
    from shapely.ops import transform, unary_union
except Exception:  # Optional; the dashboard still renders line centerlines.
    CRS = Transformer = LineString = mapping = shape = transform = unary_union = None


INPUT_TYPE_NAMES = {
    1: "협업기동임무",
    2: "협업수색공격임무",
    3: "협업경계임무",
    4: "협업공중부대엄호임무",
    5: "협업지상부대엄호임무",
    6: "협업도심수색공격임무",
}
REGION_TYPE_NAMES = {
    1: "미지정",
    2: "통제권변경지역",
    3: "ACP",
    4: "공격대기지역",
    5: "전투진지",
    6: "목표지역",
    7: "경계지역",
    8: "탑재지대",
    9: "착륙지대",
    10: "비행금지지역",
    11: "도시지역",
}
FLIGHT_MODE_COMMAND_NAMES = {
    0: "미사용",
    1: "자동이륙(미사용)",
    2: "자동착륙(미사용)",
    3: "통제권이양지 이동",
    4: "전술집결지 이동",
    5: "기지복귀(RTB)",
    6: "편대비행",
    7: "경로이동비행",
    8: "점항법비행",
    9: "표적추적비행",
}
FILMING_MODE_COMMAND_NAMES = {
    0: "없음",
    1: "좌표지향 모드",
    2: "구간탐색 모드",
    3: "자동추적 모드",
    4: "기체고정 모드",
    5: "자동주사 모드",
}
SENSOR_TYPE_NAMES = {
    0: "촬영하지 않음",
    1: "EO",
    2: "IR",
}
UAV_COMMAND_TYPE_NAMES = {
    1: "비행모드 통제",
    2: "임무장비 모드 통제",
    3: "비행·임무장비 동시 통제",
}


def _now_ms_2000() -> int:
    epoch = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.now(timezone.utc) - epoch).total_seconds() * 1000.0)


def _to_unix_ms(timestamp_ms_2000: int | None) -> int | None:
    if timestamp_ms_2000 is None:
        return None
    return int(timestamp_ms_2000) + 946_684_800_000


def _signal_age_details(
    integration: Any,
    timestamp_ms_2000: int | None,
) -> tuple[int | None, int | None, int | None]:
    """Prefer packet arrival age over a SIM timestamp that may lag at high load."""

    payload_age = (
        max(0, _now_ms_2000() - int(timestamp_ms_2000))
        if timestamp_ms_2000 is not None
        else None
    )
    arrival_age = None
    resolver = getattr(integration, "latest_0401_arrival_age_ms", None)
    if callable(resolver):
        try:
            value = _as_float(resolver())
            if value is not None:
                arrival_age = max(0, int(round(value)))
        except Exception:
            arrival_age = None
    effective_age = arrival_age if arrival_age is not None else payload_age
    return effective_age, payload_age, arrival_age


def _as_int(value: Any) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _as_float(value: Any) -> float | None:
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except Exception:
        return None


def _as_bool(value: Any, *, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "y", "on"}:
            return True
        if normalized in {"false", "0", "no", "n", "off", ""}:
            return False
    number = _as_float(value)
    if number is not None:
        return number != 0.0
    return bool(value)


def _round(value: Any, digits: int = 2) -> float | None:
    number = _as_float(value)
    return round(number, digits) if number is not None else None


def _first_mapping_value(value: Any, *names: str) -> Any:
    if not isinstance(value, dict):
        return None
    lowered = {str(key).lower(): item for key, item in value.items()}
    for name in names:
        key = str(name).lower()
        if key in lowered:
            return lowered[key]
    return None


def _as_items(value: Any) -> list[Any]:
    if value is None:
        return []
    return list(value) if isinstance(value, (list, tuple, set)) else [value]


def _0402_coordinate(value: Any) -> dict[str, float] | None:
    coordinate = _first_mapping_value(value, "coordinate")
    candidate = coordinate if isinstance(coordinate, dict) else value
    if not isinstance(candidate, dict):
        return None
    latitude = _as_float(_first_mapping_value(candidate, "latitude", "lat"))
    longitude = _as_float(
        _first_mapping_value(candidate, "longitude", "lon", "lng")
    )
    altitude = _as_float(_first_mapping_value(candidate, "altitude", "alt"))
    if latitude is None or longitude is None:
        return None
    result = {"latitude": float(latitude), "longitude": float(longitude)}
    if altitude is not None:
        result["altitude"] = float(altitude)
    return result


def _iter_0402_messages(payload: Any):
    if isinstance(payload, (list, tuple)):
        for item in payload:
            yield from _iter_0402_messages(item)
        return
    if not isinstance(payload, dict):
        return
    raw = payload.get("raw")
    if isinstance(raw, (dict, list, tuple)):
        yield from _iter_0402_messages(raw)
        return
    yield payload


def _iter_0602_messages(payload: Any):
    yield from _iter_0402_messages(payload)


def _kst_time_ms(unix_ms: int | None) -> str | None:
    if unix_ms is None:
        return None
    kst = timezone(timedelta(hours=9))
    value = datetime.fromtimestamp(float(unix_ms) / 1000.0, tz=kst)
    return value.strftime("%H:%M:%S.%f")[:-3]


def _gsd_requirement_satisfied(
    measured_gsd_cm: Any,
    required_gsd_cm: Any,
) -> bool | None:
    """GSD is compliant when the measured cm/px does not exceed the limit."""

    measured = _as_float(measured_gsd_cm)
    required = _as_float(required_gsd_cm)
    if measured is None or required is None:
        return None
    return bool(measured <= required)


def _normalize_coverage_pass_rows(value: Any) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for index, raw in enumerate(value if isinstance(value, list) else [], start=1):
        if not isinstance(raw, dict):
            continue
        pass_name = str(
            raw.get("coverage_pass") or raw.get("coveragePass") or ""
        ).strip().lower()
        if not pass_name:
            continue
        actual = _as_float(
            raw.get("actual_covered_area_m2", raw.get("covered_area_m2", raw.get("coveredAreaM2")))
        ) or 0.0
        required = _as_float(
            raw.get("required_area_m2", raw.get("planned_area_m2", raw.get("plannedAreaM2")))
        ) or 0.0
        actual = max(0.0, min(actual, required)) if required > 0.0 else max(0.0, actual)
        remaining = _as_float(raw.get("remaining_area_m2", raw.get("remainingAreaM2")))
        if remaining is None:
            remaining = max(0.0, required - actual)
        percent = (
            (actual / required * 100.0)
            if required > 0.0
            else (_as_float(raw.get("coverage_percent", raw.get("coveragePercent"))) or 0.0)
        )
        requirement_met = raw.get("requirement_met")
        if requirement_met is None:
            requirement_met = raw.get("is_done", raw.get("isDone"))
        rows.append(
            {
                "coveragePass": pass_name,
                "passIndex": _as_int(raw.get("pass_index", raw.get("passIndex"))) or index,
                "percent": round(max(0.0, min(100.0, float(percent))), 2),
                "actualCoveredM2": round(actual, 2),
                "requiredM2": round(required, 2),
                "remainingM2": round(max(0.0, float(remaining)), 2),
                "requirementsMet": bool(requirement_met),
                "status": str(raw.get("status") or ("completed" if requirement_met else "partial" if actual > 0.0 else "planned")),
            }
        )
    return rows


def _normalize_coverage_depth_rows(value: Any) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for raw in value if isinstance(value, list) else []:
        if not isinstance(raw, dict):
            continue
        depth = _as_int(raw.get("coverage_depth", raw.get("coverageDepth")))
        if depth not in {0, 1, 2}:
            continue
        remaining = _as_int(
            raw.get("remaining_capture_count", raw.get("remainingCaptureCount"))
        )
        if remaining not in {0, 1, 2}:
            remaining = 2 - int(depth)
        aircraft_ids = raw.get("active_aircraft_ids", raw.get("activeAircraftIDs"))
        active_passes = raw.get(
            "active_coverage_passes",
            raw.get("activeCoveragePasses"),
        )
        rows.append(
            {
                "coverageDepth": int(depth),
                "remainingCaptureCount": int(remaining),
                "areaM2": _round(
                    raw.get("area_m2", raw.get("areaM2", raw.get("remainingAreaM2"))),
                    2,
                ),
                "coveragePercent": _round(
                    raw.get("coverage_percent", raw.get("coveragePercent")),
                    2,
                ),
                "isComplete": int(depth) >= 2 or int(remaining) <= 0,
                "activeAircraftIDs": list(aircraft_ids)
                if isinstance(aircraft_ids, list)
                else [],
                "activeCoveragePasses": list(active_passes)
                if isinstance(active_passes, list)
                else [],
            }
        )
    rows.sort(key=lambda row: int(row["coverageDepth"]))
    return rows


def _footprint_coverage_row(value: dict[str, Any], *, kind: str) -> dict[str, Any]:
    """Normalize one footprint row using one assignment-area union.

    OUT/RETURN remain visible as diagnostics, but they are not separate area
    requirements in the mission-status score.
    """
    passes = _normalize_coverage_pass_rows(
        value.get("coverage_pass_details", value.get("coveragePassDetails"))
    )
    depth_rows = _normalize_coverage_depth_rows(
        value.get("coverage_depth_details", value.get("coverageDepthDetails"))
    )
    spatial_covered = _as_float(
        value.get("spatial_covered_area_m2", value.get("covered_area_m2"))
    ) or 0.0
    spatial_required = _as_float(
        value.get("spatial_required_area_m2", value.get("planned_area_m2"))
    ) or 0.0
    work_covered = _as_float(
        value.get("coverage_work_covered_area_m2", value.get("covered_area_m2"))
    ) or 0.0
    work_required = _as_float(
        value.get("coverage_work_required_area_m2", value.get("planned_area_m2"))
    ) or 0.0
    work_covered = max(0.0, min(work_covered, work_required)) if work_required > 0 else max(0.0, work_covered)
    spatial_covered = (
        max(0.0, min(spatial_covered, spatial_required))
        if spatial_required > 0
        else max(0.0, spatial_covered)
    )
    union_required_raw = _as_float(value.get("status_union_required_area_m2"))
    union_covered_raw = _as_float(value.get("status_union_covered_area_m2"))
    has_exact_union = union_required_raw is not None and union_required_raw > 0.0
    if has_exact_union:
        final_required = float(union_required_raw)
        final_covered = float(union_covered_raw or 0.0)
    elif passes:
        # Old persisted rows do not carry geometry union.  Until a fresh
        # tracker sample writes the exact status_union fields, use the largest
        # pass coverage as a conservative lower bound instead of averaging 2A.
        final_required = float(
            spatial_required
            or max((float(row.get("requiredM2") or 0.0) for row in passes), default=0.0)
        )
        final_covered = max(
            float(spatial_covered),
            max((float(row.get("actualCoveredM2") or 0.0) for row in passes), default=0.0),
        )
    else:
        final_required = float(spatial_required or work_required)
        final_covered = float(spatial_covered if spatial_required > 0.0 else work_covered)
    final_covered = (
        max(0.0, min(final_covered, final_required))
        if final_required > 0.0
        else max(0.0, final_covered)
    )
    percent = (
        final_covered / final_required * 100.0
        if final_required > 0.0
        else (_as_float(value.get("status_union_coverage_percent")) or 0.0)
    )
    workload_percent = (
        work_covered / work_required * 100.0
        if work_required > 0.0
        else (_as_float(value.get("coverage_percent")) or 0.0)
    )
    explicit_met = value.get("status_union_requirement_met") if has_exact_union else None
    if explicit_met is None:
        tolerance = _as_float(value.get("status_union_completion_tolerance_m2"))
        if tolerance is None:
            tolerance = max(0.05, final_required * 1e-6)
        explicit_met = bool(
            final_required > 0.0
            and (final_required - final_covered) <= float(tolerance)
        )
    return {
        "kind": kind,
        "source": "footprint",
        "basis": "initial-input-domain",
        "percent": round(max(0.0, min(100.0, float(percent))), 2),
        "covered": round(final_covered, 2),
        "planned": round(final_required, 2),
        "remaining": round(max(0.0, final_required - final_covered), 2),
        "unit": "m²",
        "spatialPercent": round(max(0.0, min(100.0, float(percent))), 2),
        "spatialCovered": round(final_covered, 2),
        "spatialPlanned": round(final_required, 2),
        "passes": passes,
        "coveragePassCount": len(passes),
        "coveragePassPolicy": "single_assignment_union" if passes else None,
        "sourcePassPolicy": value.get("coverage_pass_policy", value.get("coveragePassPolicy")),
        "coveragePassRequirementMode": "all_passes_required" if passes else None,
        "evaluationPolicy": "single_assignment_union",
        "coverageDepthDetails": depth_rows,
        "coverageDepthPolicy": value.get(
            "coverage_depth_policy",
            value.get("coverageDepthPolicy", "spatial_capture_depth" if depth_rows else None),
        ),
        "requiredCoverageDepth": _as_int(
            value.get("required_coverage_depth", value.get("requiredCoverageDepth"))
        )
        or (2 if depth_rows else None),
        "remainingCoverageDepth": _as_int(
            value.get("remaining_coverage_depth", value.get("remainingCoverageDepth"))
        ),
        "completedCoverageDepth": _as_int(
            value.get("completed_coverage_depth", value.get("completedCoverageDepth"))
        ),
        "workloadPercent": round(max(0.0, min(100.0, float(workload_percent))), 2),
        "workloadCovered": round(work_covered, 2),
        "workloadPlanned": round(work_required, 2),
        "requirementsMet": bool(explicit_met),
    }


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except Exception:
        return {}


def _feature_collection(features: list[dict[str, Any]]) -> dict[str, Any]:
    return {"type": "FeatureCollection", "features": features}


def _coord(point: Any) -> list[float] | None:
    if not isinstance(point, dict):
        return None
    lat = _as_float(point.get("latitude"))
    lon = _as_float(point.get("longitude"))
    if lat is None or lon is None:
        return None
    return [lon, lat]


def _coords(points: Any) -> list[list[float]]:
    if not isinstance(points, list):
        return []
    return [value for value in (_coord(point) for point in points) if value is not None]


_OPTION_ASSIGNMENT_COLORS = {
    4: "#37b7a1",
    5: "#e2a43b",
    6: "#de6c58",
}


def _option_payload_object(value: Any) -> dict[str, Any]:
    if isinstance(value, list):
        for item in reversed(value):
            payload = _option_payload_object(item)
            if payload:
                return payload
        return {}
    if not isinstance(value, dict):
        return {}
    for key in ("optionList", "OptionList", "option_list", "optionInfoList"):
        if isinstance(value.get(key), list):
            return value
    for key in ("body", "payload", "data"):
        nested = _option_payload_object(value.get(key))
        if nested:
            return nested
    return {}


def _option_rows(payload: Any) -> list[dict[str, Any]]:
    body = _option_payload_object(payload)
    rows = (
        body.get("optionList")
        or body.get("OptionList")
        or body.get("option_list")
        or body.get("optionInfoList")
        or []
    )
    return [row for row in rows if isinstance(row, dict)]


def _option_payload_timestamp(payload: Any) -> int:
    body = _option_payload_object(payload)
    return _as_int(
        body.get("timestamp")
        or body.get("Timestamp")
        or body.get("timeStamp")
        or body.get("TimeStamp")
    ) or 0


def _latest_option_payload_from_db(db_root: Path) -> dict[str, Any]:
    """Load the newest persisted 0701 so a late-started dashboard has context."""
    option_dir = db_root / "MissionPlanOptionInfo"
    try:
        paths = sorted(
            option_dir.glob("*.json"),
            key=lambda path: (path.stat().st_mtime_ns, path.name),
            reverse=True,
        )
    except OSError:
        return {}
    for path in paths:
        body = _option_payload_object(_read_json(path))
        if body:
            return body
    return {}


def _option_assignment_payload_signature(payload: Any) -> str:
    body = _option_payload_object(payload)
    timestamp = _option_payload_timestamp(body)
    option_keys = []
    for row in _option_rows(body):
        option_id = _as_int(row.get("optionID") or row.get("OptionID") or row.get("optionId"))
        plan_id = _as_int(
            row.get("missionPlanID") or row.get("MissionPlanID") or row.get("missionPlanId")
        )
        recommended = bool(row.get("recommend") or row.get("Recommend") or row.get("recommended"))
        option_keys.append(f"{option_id or 0}:{plan_id or 0}:{int(recommended)}")
    return f"{timestamp or 0}|{'|'.join(option_keys)}"


def _option_assignment_artifact_signature(payload: Any, db_root: Path) -> str:
    stamps: list[str] = []
    for row in _option_rows(payload):
        plan_id = _as_int(
            row.get("missionPlanID") or row.get("MissionPlanID") or row.get("missionPlanId")
        )
        if plan_id is None:
            continue
        plan_path = db_root / "MissionPlan" / f"{int(plan_id)}.json"
        plan = _read_json(plan_path)
        paths = [plan_path]
        for aircraft in plan.get("aircraftList") or []:
            if not isinstance(aircraft, dict):
                continue
            aircraft_id = _as_int(aircraft.get("aircraftID"))
            package_id = _as_int(
                aircraft.get("individualMissionPackageID")
                or aircraft.get("individualMissionPlanPackageID")
            )
            if aircraft_id in (4, 5, 6) and package_id is not None:
                paths.append(db_root / "IndividualMissionPlan" / f"{int(package_id)}.json")
        for path in paths:
            try:
                stat = path.stat()
                stamps.append(f"{path.name}:{stat.st_mtime_ns}:{stat.st_size}")
            except OSError:
                stamps.append(f"{path.name}:missing")
    return "|".join(stamps)


def _coordinate_bounds(coordinates: list[list[float]]) -> list[float] | None:
    clean = [point for point in coordinates if isinstance(point, list) and len(point) >= 2]
    if not clean:
        return None
    lons = [float(point[0]) for point in clean]
    lats = [float(point[1]) for point in clean]
    return [min(lons), min(lats), max(lons), max(lats)]


def _merge_bounds(values: list[list[float]]) -> list[float] | None:
    clean = [value for value in values if isinstance(value, list) and len(value) >= 4]
    if not clean:
        return None
    return [
        min(float(value[0]) for value in clean),
        min(float(value[1]) for value in clean),
        max(float(value[2]) for value in clean),
        max(float(value[3]) for value in clean),
    ]


def _assignment_center(bounds: list[float] | None) -> dict[str, float] | None:
    if not isinstance(bounds, list) or len(bounds) < 4:
        return None
    return {
        "longitude": round((float(bounds[0]) + float(bounds[2])) / 2.0, 6),
        "latitude": round((float(bounds[1]) + float(bounds[3])) / 2.0, 6),
    }


def build_option_assignment_snapshot(payload: Any, *, db_root: Path | str) -> dict[str, Any]:
    """Build per-option UAV assignment summaries and map geometry from 0701."""
    root = Path(db_root)
    body = _option_payload_object(payload)
    timestamp = _as_int(
        body.get("timestamp")
        or body.get("Timestamp")
        or body.get("timeStamp")
        or body.get("TimeStamp")
    )
    options: list[dict[str, Any]] = []

    for option_index, row in enumerate(_option_rows(body), start=1):
        option_id = _as_int(row.get("optionID") or row.get("OptionID") or row.get("optionId"))
        option_name = _as_int(row.get("optionName") or row.get("OptionName") or row.get("option_name"))
        plan_id = _as_int(
            row.get("missionPlanID") or row.get("MissionPlanID") or row.get("missionPlanId")
        )
        recommended = bool(row.get("recommend") or row.get("Recommend") or row.get("recommended"))
        plan_path = root / "MissionPlan" / f"{int(plan_id or 0)}.json"
        plan = _read_json(plan_path) if plan_id is not None else {}
        available = bool(plan and _as_int(plan.get("missionPlanID")) in (None, plan_id))
        aircraft_rows = {
            int(aircraft_id): aircraft
            for aircraft in (plan.get("aircraftList") or [])
            if isinstance(aircraft, dict)
            and (aircraft_id := _as_int(aircraft.get("aircraftID"))) in (4, 5, 6)
        }
        features: list[dict[str, Any]] = []
        aircraft_summaries: list[dict[str, Any]] = []
        option_bounds: list[list[float]] = []
        complete = bool(available)

        for aircraft_id in (4, 5, 6):
            aircraft = aircraft_rows.get(aircraft_id) or {}
            package_id = _as_int(
                aircraft.get("individualMissionPackageID")
                or aircraft.get("individualMissionPlanPackageID")
            )
            individual_plan = (
                _read_json(root / "IndividualMissionPlan" / f"{int(package_id)}.json")
                if package_id is not None
                else {}
            )
            if package_id is None or not individual_plan:
                complete = False
            assignments: list[dict[str, Any]] = []
            aircraft_bounds: list[list[float]] = []
            area_count = 0
            line_count = 0
            color = _OPTION_ASSIGNMENT_COLORS[int(aircraft_id)]

            for mission in individual_plan.get("individualMissionList") or []:
                if not isinstance(mission, dict) or bool(mission.get("isDone")):
                    continue
                info = mission.get("individualMissionInfo") or {}
                if not isinstance(info, dict):
                    continue
                related = mission.get("relatedMission") or {}
                input_id = _as_int(related.get("inputMissionID")) if isinstance(related, dict) else None
                mission_id = _as_int(mission.get("individualMissionID"))
                mission_area_count = 0
                mission_line_count = 0
                mission_bounds: list[list[float]] = []

                for area_index, area in enumerate(info.get("areaList") or [], start=1):
                    if not isinstance(area, dict) or bool(area.get("isHole")):
                        continue
                    ring = _coords(area.get("coordinateList"))
                    if len(ring) < 3:
                        continue
                    if ring[0] != ring[-1]:
                        ring.append(list(ring[0]))
                    bounds = _coordinate_bounds(ring)
                    if bounds:
                        mission_bounds.append(bounds)
                    mission_area_count += 1
                    features.append(
                        {
                            "type": "Feature",
                            "properties": {
                                "optionID": option_id,
                                "missionPlanID": plan_id,
                                "aircraftID": int(aircraft_id),
                                "inputMissionID": input_id,
                                "individualMissionID": mission_id,
                                "assignmentShape": "AREA",
                                "assignmentIndex": int(area_index),
                                "color": color,
                            },
                            "geometry": {"type": "Polygon", "coordinates": [ring]},
                        }
                    )

                for line_index, line in enumerate(info.get("lineList") or [], start=1):
                    if not isinstance(line, dict):
                        continue
                    line_coords = _coords(line.get("coordinateList"))
                    if len(line_coords) < 2:
                        continue
                    bounds = _coordinate_bounds(line_coords)
                    if bounds:
                        mission_bounds.append(bounds)
                    width_m = _as_float(line.get("width")) or _as_float(info.get("width")) or 0.0
                    properties = {
                        "optionID": option_id,
                        "missionPlanID": plan_id,
                        "aircraftID": int(aircraft_id),
                        "inputMissionID": input_id,
                        "individualMissionID": mission_id,
                        "assignmentShape": "LINE",
                        "assignmentIndex": int(line_index),
                        "widthM": round(float(width_m), 1),
                        "color": color,
                    }
                    corridor = _buffer_line(line_coords, float(width_m)) if width_m > 0.0 else None
                    if corridor is not None:
                        features.append(
                            {
                                "type": "Feature",
                                "properties": {**properties, "featureRole": "corridor"},
                                "geometry": corridor,
                            }
                        )
                    features.append(
                        {
                            "type": "Feature",
                            "properties": {**properties, "featureRole": "centerline"},
                            "geometry": {"type": "LineString", "coordinates": line_coords},
                        }
                    )
                    mission_line_count += 1

                assignment_bounds = _merge_bounds(mission_bounds)
                if mission_area_count or mission_line_count:
                    if assignment_bounds:
                        aircraft_bounds.append(assignment_bounds)
                    area_count += mission_area_count
                    line_count += mission_line_count
                    assignments.append(
                        {
                            "inputMissionID": input_id,
                            "individualMissionID": mission_id,
                            "individualMissionType": _as_int(info.get("individualMissionType")),
                            "patternType": _as_int(info.get("patternType")),
                            "areaCount": int(mission_area_count),
                            "lineCount": int(mission_line_count),
                            "bounds": assignment_bounds,
                            "center": _assignment_center(assignment_bounds),
                        }
                    )

            merged_aircraft_bounds = _merge_bounds(aircraft_bounds)
            if merged_aircraft_bounds:
                option_bounds.append(merged_aircraft_bounds)
            aircraft_summaries.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": f"UAV{int(aircraft_id) - 3}",
                    "individualMissionPackageID": package_id,
                    "assignmentCount": len(assignments),
                    "areaCount": int(area_count),
                    "lineCount": int(line_count),
                    "inputMissionIDs": sorted(
                        {int(value) for item in assignments if (value := _as_int(item.get("inputMissionID"))) is not None}
                    ),
                    "bounds": merged_aircraft_bounds,
                    "center": _assignment_center(merged_aircraft_bounds),
                    "color": color,
                    "assignments": assignments,
                }
            )

        options.append(
            {
                "optionIndex": int(option_index),
                "optionID": option_id,
                "optionName": option_name,
                "missionPlanID": plan_id,
                "recommend": bool(recommended),
                "available": bool(available),
                "complete": bool(complete),
                "bounds": _merge_bounds(option_bounds),
                "aircraft": aircraft_summaries,
                "geojson": _feature_collection(features),
            }
        )

    signature = _option_assignment_payload_signature(body)
    return {
        "available": bool(options),
        "timestamp": timestamp,
        "signature": signature,
        "optionCount": len(options),
        "options": options,
    }


def _haversine(left: dict[str, Any], right: dict[str, Any]) -> float | None:
    lat1 = _as_float(left.get("latitude"))
    lon1 = _as_float(left.get("longitude"))
    lat2 = _as_float(right.get("latitude"))
    lon2 = _as_float(right.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    return 6_371_000.0 * 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


def _footprint_size(corners: Any) -> tuple[float | None, float | None]:
    if not isinstance(corners, list) or len(corners) < 4:
        return None, None
    top = _haversine(corners[0], corners[1])
    bottom = _haversine(corners[3], corners[2])
    left = _haversine(corners[0], corners[3])
    right = _haversine(corners[1], corners[2])
    widths = [value for value in (top, bottom) if value is not None]
    heights = [value for value in (left, right) if value is not None]
    return (
        sum(widths) / len(widths) if widths else None,
        sum(heights) / len(heights) if heights else None,
    )


def _buffer_line(coords: list[list[float]], width_m: float) -> dict[str, Any] | None:
    if len(coords) < 2 or width_m <= 0 or LineString is None or Transformer is None:
        return None
    try:
        center_lon = sum(point[0] for point in coords) / len(coords)
        center_lat = sum(point[1] for point in coords) / len(coords)
        zone = max(1, min(60, int((center_lon + 180.0) / 6.0) + 1))
        epsg = (32600 if center_lat >= 0 else 32700) + zone
        to_local = Transformer.from_crs("EPSG:4326", CRS.from_epsg(epsg), always_xy=True)
        to_wgs84 = Transformer.from_crs(CRS.from_epsg(epsg), "EPSG:4326", always_xy=True)
        local = transform(to_local.transform, LineString(coords))
        polygon = local.buffer(width_m / 2.0, cap_style=2, join_style=2)
        return mapping(transform(to_wgs84.transform, polygon))
    except Exception:
        return None


class _JsonCache:
    def __init__(self) -> None:
        self._values: dict[str, tuple[int, dict[str, Any]]] = {}

    def read(self, path: Path) -> dict[str, Any]:
        key = str(path)
        cached = self._values.get(key)
        try:
            stamp = path.stat().st_mtime_ns
        except Exception:
            return cached[1] if cached is not None else {}
        if cached is not None and cached[0] == stamp:
            return cached[1]
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
            if not isinstance(value, dict):
                raise ValueError("JSON root is not an object")
        except Exception:
            return cached[1] if cached is not None else {}
        self._values[key] = (stamp, value)
        return value


class MissionStatusService:
    def __init__(self, integration=None) -> None:
        self.integration = integration
        self._lock = threading.RLock()
        self._json_cache = _JsonCache()
        self._mission_signature = ""
        self._upstream_mission_signature = ""
        self._upstream_plan_source = ""
        self._startup_baseline_captured = False
        self._startup_mission_signature = ""
        self._awaiting_new_mission = True
        self._service_started_unix_ms = int(time.time() * 1000.0)
        self._mission: dict[str, Any] = {}
        self._mission_geometry = self._empty_geometry()
        self._mission_bounds: list[float] | None = None
        self._mission_db_root: str | None = None
        self._history_db_root: str | None = None
        self._completed_mission_history: list[dict[str, Any]] = []
        self._last_coverage_by_input: dict[int, dict[str, Any]] = {}
        self._last_parts_by_input: dict[int, dict[str, Any]] = {}
        self._view: dict[str, Any] = {}
        self._last_view_refresh = 0.0
        self._last_sample_timestamp: int | None = None
        self._quality_by_key: dict[tuple[int, int], dict[str, Any]] = {}
        # Quality is owned by the immutable input-mission domain, not by a
        # generated MissionPlan.  These ledgers therefore survive attack and
        # post-attack replans while the input mission package stays the same.
        self._initial_quality_domain_by_input: dict[int, dict[str, Any]] = {}
        self._spatial_quality_by_input: dict[int, dict[str, Any]] = {}
        self._events: deque[dict[str, Any]] = deque(maxlen=80)
        self._discoveries: deque[dict[str, Any]] = deque(maxlen=200)
        self._discovery_db_root: str | None = None
        self._seen_0402_detection_signatures: set[str] = set()
        self._target_in_frame_state: dict[int, bool] = {}
        self._last_0402_payload_signature = ""
        self._uav_commands: deque[dict[str, Any]] = deque(maxlen=300)
        self._command_db_root: str | None = None
        self._seen_0602_command_signatures: set[str] = set()
        self._last_0602_payload_signature = ""
        self._last_current_by_aircraft: dict[int, int | None] = {}
        self._last_plan_id: int | None = None
        self._last_snapshot_source = ""
        self._option_assignment_cache_key = ""
        self._option_assignment_payload_key = ""
        self._option_assignment_last_probe = 0.0
        self._remaining_area_cache_key: tuple[Any, ...] | None = None
        self._remaining_area_cache_payload: dict[str, Any] | None = None
        self._option_assignment_snapshot: dict[str, Any] = {
            "available": False,
            "timestamp": None,
            "signature": "",
            "optionCount": 0,
            "options": [],
        }
        self._quality_settings = self._load_quality_settings()
        self._fuel_capacity_liters = max(0.1, float(resolve_fuel_capacity_liters()))

    @staticmethod
    def _empty_geometry() -> dict[str, Any]:
        return {
            "inputAreas": _feature_collection([]),
            "inputLines": _feature_collection([]),
            "lineCorridors": _feature_collection([]),
            "paths": _feature_collection([]),
            "remainingAreas": _feature_collection([]),
            "coverageDepth": _feature_collection([]),
            "coveragePassAttribution": _feature_collection([]),
        }

    def _load_quality_settings(self) -> dict[str, float]:
        path = db_paths.PROJECT_ROOT / "modules" / "monitoring" / "quality_monitor_settings.json"
        data = _read_json(path).get("spatial_resolution") or {}
        return {
            "img_w_px": max(1.0, _as_float(data.get("img_w_px")) or 1920.0),
            "img_h_px": max(1.0, _as_float(data.get("img_h_px")) or 1080.0),
            "obj_w_m": max(0.01, _as_float(data.get("obj_w_m")) or 6.0),
            "obj_h_m": max(0.01, _as_float(data.get("obj_h_m")) or 3.2),
            "obj_min_px_x": max(1.0, _as_float(data.get("obj_min_px_x")) or 72.0),
            "obj_min_px_y": max(1.0, _as_float(data.get("obj_min_px_y")) or 38.0),
        }

    def _build_initial_quality_domains(self) -> dict[int, dict[str, Any]]:
        """Project the initial input polygons/corridors into local metre CRS.

        The geometry comes from InputMissionPlan, never from a generated UAV
        path.  It remains fixed while 3->2->3 aircraft ownership changes.
        """

        if any(value is None for value in (shape, unary_union, transform, Transformer, CRS)):
            return {}
        grouped: dict[int, list[Any]] = {}
        for collection_name in ("inputAreas", "lineCorridors"):
            collection = self._mission_geometry.get(collection_name) or {}
            for feature in collection.get("features") or []:
                if not isinstance(feature, dict):
                    continue
                input_id = _as_int((feature.get("properties") or {}).get("inputMissionID"))
                if input_id is None:
                    continue
                try:
                    geometry = shape(feature.get("geometry") or {})
                except Exception:
                    continue
                if geometry is None or geometry.is_empty:
                    continue
                grouped.setdefault(int(input_id), []).append(geometry)

        domains: dict[int, dict[str, Any]] = {}
        for input_id, geometries in grouped.items():
            try:
                wgs84_geometry = unary_union(geometries)
                if wgs84_geometry.is_empty:
                    continue
                center = wgs84_geometry.centroid
                center_lon = float(center.x)
                center_lat = float(center.y)
                zone = max(1, min(60, int((center_lon + 180.0) / 6.0) + 1))
                epsg = (32600 if center_lat >= 0.0 else 32700) + zone
                to_local = Transformer.from_crs(
                    "EPSG:4326",
                    CRS.from_epsg(epsg),
                    always_xy=True,
                )
                local_geometry = transform(to_local.transform, wgs84_geometry)
                area_m2 = float(max(0.0, local_geometry.area or 0.0))
                if local_geometry.is_empty or area_m2 <= 0.0:
                    continue
                domains[int(input_id)] = {
                    "geometry": local_geometry,
                    "toLocal": to_local,
                    "areaM2": area_m2,
                    "basis": "initial-input-domain",
                }
            except Exception:
                continue
        return domains

    def _clip_quality_footprint_to_initial_domain(
        self,
        input_id: int,
        corners: Any,
    ) -> tuple[Any | None, bool]:
        """Return (clipped footprint, accepted).

        A missing optional geometry dependency keeps the legacy sample path.
        When an initial domain exists, a footprint wholly outside it is
        rejected so lead/turn/return filming cannot pollute mission GSD.
        """

        domain = self._initial_quality_domain_by_input.get(int(input_id))
        if not isinstance(domain, dict):
            return None, True
        if shape is None or transform is None:
            return None, True
        raw_coords = _coords(corners)
        if len(raw_coords) < 3:
            return None, False
        if raw_coords[0] != raw_coords[-1]:
            raw_coords.append(raw_coords[0])
        try:
            footprint_wgs84 = shape(
                {"type": "Polygon", "coordinates": [raw_coords]}
            )
            if not footprint_wgs84.is_valid:
                footprint_wgs84 = footprint_wgs84.buffer(0)
            footprint_local = transform(
                domain["toLocal"].transform,
                footprint_wgs84,
            )
            clipped = footprint_local.intersection(domain["geometry"])
            if clipped.is_empty or float(clipped.area or 0.0) <= 1e-6:
                return None, False
            return clipped, True
        except Exception:
            # Do not discard live quality solely because optional geometry
            # processing failed for one malformed footprint.
            return None, True

    def _update_spatial_quality(
        self,
        *,
        input_id: int,
        clipped_footprint: Any,
        eq_gsd: float,
        required_gsd: float,
    ) -> None:
        if clipped_footprint is None or clipped_footprint.is_empty:
            return
        domain = self._initial_quality_domain_by_input.get(int(input_id)) or {}
        state = self._spatial_quality_by_input.setdefault(
            int(input_id),
            {
                "coveredGeometry": None,
                "satisfiedGeometry": None,
                "weightedGsdAreaSum": 0.0,
                "weightedAreaM2": 0.0,
                "requiredAreaM2": float(domain.get("areaM2") or 0.0),
            },
        )
        covered = state.get("coveredGeometry")
        try:
            newly_covered = (
                clipped_footprint
                if covered is None or covered.is_empty
                else clipped_footprint.difference(covered)
            )
            new_area_m2 = float(max(0.0, newly_covered.area or 0.0))
            if new_area_m2 > 1e-6:
                state["weightedGsdAreaSum"] = float(
                    state.get("weightedGsdAreaSum") or 0.0
                ) + (float(eq_gsd) * new_area_m2)
                state["weightedAreaM2"] = float(
                    state.get("weightedAreaM2") or 0.0
                ) + new_area_m2
            state["coveredGeometry"] = (
                clipped_footprint
                if covered is None or covered.is_empty
                else covered.union(clipped_footprint)
            )
            if float(eq_gsd) <= float(required_gsd):
                satisfied = state.get("satisfiedGeometry")
                state["satisfiedGeometry"] = (
                    clipped_footprint
                    if satisfied is None or satisfied.is_empty
                    else satisfied.union(clipped_footprint)
                )
        except Exception:
            return

    def _add_event(self, level: str, category: str, message: str, timestamp: int | None = None) -> None:
        self._events.appendleft(
            {
                "timestamp": _to_unix_ms(timestamp or _now_ms_2000()),
                "level": level,
                "category": category,
                "message": message,
            }
        )

    def _refresh_view(self, plan_id: int | None) -> None:
        now = time.monotonic()
        if plan_id is None:
            self._view = {}
            return
        if now - self._last_view_refresh < 1.0 and _as_int(self._view.get("mission_plan_id")) == plan_id:
            return
        self._last_view_refresh = now
        try:
            self._view = build_uav_mission_view(plan_id, db_root=db_paths.get_active_db_root())
        except Exception:
            self._view = {}

    @staticmethod
    def _db_root_key() -> str:
        try:
            return str(Path(db_paths.get_active_db_root()).resolve())
        except Exception:
            return str(Path(db_paths.get_active_db_root()))

    def _history_path(self) -> Path:
        return Path(db_paths.get_active_db_root()) / "DSS_Internal" / "mission_status_history.json"

    def _discovery_path(self) -> Path:
        return (
            Path(db_paths.get_active_db_root())
            / "DSS_Internal"
            / "mission_status_0402_discoveries.jsonl"
        )

    @staticmethod
    def _discovery_signature(row: dict[str, Any]) -> str:
        coordinate = row.get("coordinate") if isinstance(row.get("coordinate"), dict) else {}
        signature_body = {
            "timestamp": _as_int(row.get("messageTimestamp")),
            "kind": str(row.get("kind") or ""),
            "targetID": _as_int(row.get("targetID")),
            "aircraftID": _as_int(row.get("aircraftID")),
            "watcherID": _as_int(row.get("watcherID")),
            "latitude": _round(coordinate.get("latitude"), 7),
            "longitude": _round(coordinate.get("longitude"), 7),
        }
        return json.dumps(signature_body, ensure_ascii=False, sort_keys=True)

    def _ensure_discoveries_loaded(self, db_root_key: str) -> None:
        if self._discovery_db_root == db_root_key:
            return
        self._discovery_db_root = db_root_key
        self._discoveries.clear()
        self._seen_0402_detection_signatures.clear()
        self._target_in_frame_state.clear()
        self._last_0402_payload_signature = ""
        try:
            path = self._discovery_path()
            if not path.is_file():
                return
            recent_lines: deque[str] = deque(maxlen=200)
            with path.open("r", encoding="utf-8") as handle:
                for line in handle:
                    if line.strip():
                        recent_lines.append(line)
            for line in recent_lines:
                try:
                    row = json.loads(line)
                except Exception:
                    continue
                if not isinstance(row, dict):
                    continue
                signature = self._discovery_signature(row)
                self._seen_0402_detection_signatures.add(signature)
                self._discoveries.appendleft(row)
        except Exception:
            return

    def _persist_discoveries(self, rows: list[dict[str, Any]]) -> None:
        if not rows:
            return
        try:
            path = self._discovery_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8", newline="\n") as handle:
                for row in rows:
                    handle.write(json.dumps(row, ensure_ascii=False, separators=(",", ":")))
                    handle.write("\n")
        except Exception as exc:
            self._add_event("warn", "0402", f"발견 이력 저장 실패: {exc}")

    @staticmethod
    def _0402_packets(integration: Any) -> list[dict[str, Any]]:
        if integration is None:
            return []
        drain = getattr(integration, "drain_0402_events", None)
        if callable(drain):
            try:
                rows = drain()
                return [item for item in rows if isinstance(item, dict)]
            except Exception:
                return []
        latest = getattr(integration, "latest_payload", None)
        if callable(latest):
            try:
                return [
                    {
                        "payload": latest("0402"),
                        "footprintContext": build_0401_footprint_context(
                            latest("0401")
                        ),
                    }
                ]
            except Exception:
                return []
        return []

    def _consume_0402_events(self) -> None:
        persisted_rows: list[dict[str, Any]] = []
        for packet in self._0402_packets(self.integration):
            payload = packet.get("payload")
            try:
                payload_signature = json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                    default=str,
                )
            except Exception:
                payload_signature = repr(payload)
            # The fallback latest-payload API has no drain semantics.  Avoid
            # recording the same latest 0402 on every 400 ms state poll.
            if "arrivalUnixMs" not in packet:
                if payload_signature == self._last_0402_payload_signature:
                    continue
                self._last_0402_payload_signature = payload_signature

            arrival_unix_ms = _as_int(packet.get("arrivalUnixMs"))
            for message in _iter_0402_messages(payload):
                raw_timestamp = _as_int(
                    _first_mapping_value(message, "timestamp", "timeStamp")
                )
                event_unix_ms = (
                    _to_unix_ms(raw_timestamp)
                    if raw_timestamp is not None
                    else arrival_unix_ms or int(time.time() * 1000.0)
                )
                source = str(_first_mapping_value(message, "source") or "0402")

                roi_values = _first_mapping_value(message, "roiInfoList")
                if roi_values is None:
                    roi_values = _first_mapping_value(message, "roiInfo")
                for roi in _as_items(roi_values):
                    coordinate = _0402_coordinate(roi)
                    if coordinate is None:
                        continue
                    aircraft_id = _as_int(
                        _first_mapping_value(roi, "aircraftID", "watcherID")
                    )
                    fov = _as_float(_first_mapping_value(roi, "fov"))
                    footprint, footprint_timestamp = footprint_for_aircraft(
                        packet.get("footprintContext"),
                        aircraft_id,
                    )
                    row = {
                        "timestamp": event_unix_ms,
                        "timeKst": _kst_time_ms(event_unix_ms),
                        "messageTimestamp": raw_timestamp,
                        "kind": "ROI",
                        "source": source,
                        "aircraftID": aircraft_id,
                        "coordinate": coordinate,
                        "fov": _round(fov, 3),
                        "footprint": footprint,
                        "footprintTimestamp": footprint_timestamp,
                        "footprintTimestampUnix": _to_unix_ms(footprint_timestamp),
                        "message": (
                            f"ROI 발견 · UAV {aircraft_id if aircraft_id is not None else '-'} · "
                            f"{coordinate['latitude']:.6f}, {coordinate['longitude']:.6f}"
                        ),
                    }
                    signature = self._discovery_signature(row)
                    if signature in self._seen_0402_detection_signatures:
                        continue
                    self._seen_0402_detection_signatures.add(signature)
                    self._discoveries.appendleft(row)
                    persisted_rows.append(row)
                    self._add_event("info", "0402", row["message"], raw_timestamp)

                target_values = _first_mapping_value(message, "targetList")
                if target_values is None:
                    target_values = _first_mapping_value(message, "targets")
                if target_values is None:
                    target_values = _first_mapping_value(
                        message,
                        "situationAwarenessInfoList",
                    )
                for target in _as_items(target_values):
                    if not isinstance(target, dict):
                        continue
                    target_id = _as_int(
                        _first_mapping_value(target, "targetID", "targetId")
                    )
                    if target_id is None:
                        continue
                    watcher = _first_mapping_value(target, "watcher")
                    watcher_id = _as_int(
                        _first_mapping_value(target, "watcherID", "aircraftID")
                    )
                    if watcher_id is None:
                        watcher_id = _as_int(
                            _first_mapping_value(watcher, "aircraftID", "watcherID")
                        )
                    in_frame_value = _first_mapping_value(target, "targetInFrame")
                    destroyed_value = _first_mapping_value(target, "isDestroyed")
                    in_frame = _as_bool(in_frame_value, default=True)
                    destroyed = _as_bool(destroyed_value)
                    active = bool(in_frame and not destroyed)
                    previously_active = bool(
                        self._target_in_frame_state.get(int(target_id), False)
                    )
                    if not active:
                        self._target_in_frame_state[int(target_id)] = False
                        continue
                    if previously_active:
                        continue
                    coordinate = _0402_coordinate(target)
                    target_type = _as_int(_first_mapping_value(target, "targetType"))
                    threat = _as_float(_first_mapping_value(target, "threat"))
                    footprint, footprint_timestamp = footprint_for_aircraft(
                        packet.get("footprintContext"),
                        watcher_id,
                    )
                    row = {
                        "timestamp": event_unix_ms,
                        "timeKst": _kst_time_ms(event_unix_ms),
                        "messageTimestamp": raw_timestamp,
                        "kind": "TARGET",
                        "source": source,
                        "targetID": int(target_id),
                        "targetType": target_type,
                        "watcherID": watcher_id,
                        "coordinate": coordinate,
                        "threat": _round(threat, 3),
                        "footprint": footprint,
                        "footprintTimestamp": footprint_timestamp,
                        "footprintTimestampUnix": _to_unix_ms(footprint_timestamp),
                        "message": (
                            f"표적 발견 · Target ID {int(target_id)}"
                            + (f" · Type {int(target_type)}" if target_type is not None else "")
                            + (f" · UAV {int(watcher_id)}" if watcher_id is not None else "")
                        ),
                    }
                    signature = self._discovery_signature(row)
                    if signature in self._seen_0402_detection_signatures:
                        continue
                    self._target_in_frame_state[int(target_id)] = True
                    self._seen_0402_detection_signatures.add(signature)
                    self._discoveries.appendleft(row)
                    persisted_rows.append(row)
                    self._add_event("info", "0402", row["message"], raw_timestamp)
        self._persist_discoveries(persisted_rows)

    def _command_path(self) -> Path:
        return (
            Path(db_paths.get_active_db_root())
            / "DSS_Internal"
            / "mission_status_0602_commands.jsonl"
        )

    @staticmethod
    def _command_signature(row: dict[str, Any]) -> str:
        signature_body = {
            "timestamp": _as_int(row.get("messageTimestamp")),
            "aircraftID": _as_int(row.get("aircraftID")),
            "commandModeType": _as_int(row.get("commandModeType")),
            "flightMode": _as_int(row.get("flightMode")),
            "filmingMode": _as_int(row.get("filmingMode")),
            "sensorType": _as_int(row.get("sensorType")),
        }
        return json.dumps(signature_body, ensure_ascii=False, sort_keys=True)

    def _ensure_commands_loaded(self, db_root_key: str) -> None:
        if self._command_db_root == db_root_key:
            return
        self._command_db_root = db_root_key
        self._uav_commands.clear()
        self._seen_0602_command_signatures.clear()
        self._last_0602_payload_signature = ""
        try:
            path = self._command_path()
            if not path.is_file():
                return
            recent_lines: deque[str] = deque(maxlen=300)
            with path.open("r", encoding="utf-8") as handle:
                for line in handle:
                    if line.strip():
                        recent_lines.append(line)
            for line in recent_lines:
                try:
                    row = json.loads(line)
                except Exception:
                    continue
                if not isinstance(row, dict):
                    continue
                self._seen_0602_command_signatures.add(
                    self._command_signature(row)
                )
                self._uav_commands.appendleft(row)
        except Exception:
            return

    def _persist_commands(self, rows: list[dict[str, Any]]) -> None:
        if not rows:
            return
        try:
            path = self._command_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8", newline="\n") as handle:
                for row in rows:
                    handle.write(
                        json.dumps(row, ensure_ascii=False, separators=(",", ":"))
                    )
                    handle.write("\n")
        except Exception as exc:
            self._add_event("warn", "0602", f"통제 명령 이력 저장 실패: {exc}")

    @staticmethod
    def _0602_packets(integration: Any) -> list[dict[str, Any]]:
        if integration is None:
            return []
        drain = getattr(integration, "drain_0602_events", None)
        if callable(drain):
            try:
                rows = drain()
                return [item for item in rows if isinstance(item, dict)]
            except Exception:
                return []
        latest = getattr(integration, "latest_payload", None)
        if callable(latest):
            try:
                return [
                    {
                        "payload": latest("0602"),
                        "aircraftContext": build_0401_footprint_context(
                            latest("0401")
                        ),
                    }
                ]
            except Exception:
                return []
        return []

    def _consume_0602_events(self) -> None:
        persisted_rows: list[dict[str, Any]] = []
        for packet in self._0602_packets(self.integration):
            payload = packet.get("payload")
            try:
                payload_signature = json.dumps(
                    payload,
                    ensure_ascii=False,
                    sort_keys=True,
                    default=str,
                )
            except Exception:
                payload_signature = repr(payload)
            if "arrivalUnixMs" not in packet:
                if payload_signature == self._last_0602_payload_signature:
                    continue
                self._last_0602_payload_signature = payload_signature

            arrival_unix_ms = _as_int(packet.get("arrivalUnixMs"))
            for message in _iter_0602_messages(payload):
                raw_timestamp = _as_int(
                    _first_mapping_value(message, "timestamp", "timeStamp")
                )
                event_unix_ms = (
                    _to_unix_ms(raw_timestamp)
                    if raw_timestamp is not None
                    else arrival_unix_ms or int(time.time() * 1000.0)
                )
                aircraft_id = _as_int(
                    _first_mapping_value(message, "aircraftID", "aircraftId")
                )
                command_type = _as_int(
                    _first_mapping_value(
                        message,
                        "uavCommandModeType",
                        "commandModeType",
                    )
                )
                flight_command = _first_mapping_value(
                    message,
                    "flightModeCommand",
                )
                filming_command = _first_mapping_value(
                    message,
                    "filmingModeCommand",
                )
                flight_mode = _as_int(
                    _first_mapping_value(flight_command, "flightMode")
                )
                filming_mode = _as_int(
                    _first_mapping_value(filming_command, "operationMode")
                )
                sensor_type = _as_int(
                    _first_mapping_value(filming_command, "sensorType")
                )
                field_of_view = _as_float(
                    _first_mapping_value(filming_command, "fieldOfView", "fov")
                )
                position, position_timestamp = position_for_aircraft(
                    packet.get("aircraftContext"),
                    aircraft_id,
                )
                uav_number = (
                    int(aircraft_id) - 3
                    if aircraft_id is not None and 4 <= int(aircraft_id) <= 6
                    else None
                )
                uav_label = f"UAV{uav_number}" if uav_number is not None else f"UAV {aircraft_id or '-'}"
                flight_text = (
                    f"{int(flight_mode)} : "
                    f"{FLIGHT_MODE_COMMAND_NAMES.get(int(flight_mode), '미정의 비행모드')}"
                    if flight_mode is not None
                    else None
                )
                filming_text = (
                    f"{int(filming_mode)} : "
                    f"{FILMING_MODE_COMMAND_NAMES.get(int(filming_mode), '미정의 촬영모드')}"
                    if filming_mode is not None
                    else None
                )
                summary_parts = []
                if flight_text:
                    summary_parts.append(f"비행 {flight_text}")
                if filming_text:
                    summary_parts.append(f"촬영 {filming_text}")
                if not summary_parts:
                    summary_parts.append(
                        UAV_COMMAND_TYPE_NAMES.get(
                            int(command_type or 0),
                            f"통제유형 {command_type or '-'}",
                        )
                    )
                row = {
                    "timestamp": event_unix_ms,
                    "timeKst": _kst_time_ms(event_unix_ms),
                    "messageTimestamp": raw_timestamp,
                    "source": str(_first_mapping_value(message, "source") or "0602"),
                    "aircraftID": aircraft_id,
                    "uavLabel": uav_label,
                    "commandModeType": command_type,
                    "commandModeTypeName": UAV_COMMAND_TYPE_NAMES.get(
                        int(command_type or 0),
                        "미정의 통제",
                    ),
                    "flightMode": flight_mode,
                    "flightModeName": (
                        FLIGHT_MODE_COMMAND_NAMES.get(
                            int(flight_mode),
                            "미정의 비행모드",
                        )
                        if flight_mode is not None
                        else None
                    ),
                    "flightCommandText": flight_text,
                    "filmingMode": filming_mode,
                    "filmingModeName": (
                        FILMING_MODE_COMMAND_NAMES.get(
                            int(filming_mode),
                            "미정의 촬영모드",
                        )
                        if filming_mode is not None
                        else None
                    ),
                    "filmingCommandText": filming_text,
                    "sensorType": sensor_type,
                    "sensorTypeName": (
                        SENSOR_TYPE_NAMES.get(int(sensor_type), "미정의 센서")
                        if sensor_type is not None
                        else None
                    ),
                    "fieldOfView": _round(field_of_view, 3),
                    "position": position,
                    "positionTimestamp": position_timestamp,
                    "positionTimestampUnix": _to_unix_ms(position_timestamp),
                    "message": f"{uav_label} · {' · '.join(summary_parts)}",
                }
                signature = self._command_signature(row)
                if signature in self._seen_0602_command_signatures:
                    continue
                self._seen_0602_command_signatures.add(signature)
                self._uav_commands.appendleft(row)
                persisted_rows.append(row)
        self._persist_commands(persisted_rows)

    def _ensure_history_loaded(self, db_root_key: str) -> None:
        if self._history_db_root == db_root_key:
            return
        self._history_db_root = db_root_key
        # A new viewer session starts clean. History is accumulated only from
        # plan transitions observed by this running process.
        self._completed_mission_history = []

    def _apply_startup_mission_gate(
        self,
        mission: dict[str, Any] | None,
        signature: str,
    ) -> dict[str, Any] | None:
        if not self._startup_baseline_captured:
            self._startup_baseline_captured = True
            self._startup_mission_signature = signature
            self._upstream_mission_signature = signature
            return None
        if not self._awaiting_new_mission:
            return mission
        if not signature or signature == self._startup_mission_signature:
            self._upstream_mission_signature = signature
            return None
        self._awaiting_new_mission = False
        return mission

    def _persist_mission_history(self) -> None:
        try:
            path = self._history_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = Path(f"{path}.tmp")
            temporary.write_text(
                json.dumps(
                    {"version": 1, "entries": self._completed_mission_history[-8:]},
                    ensure_ascii=False,
                    separators=(",", ":"),
                ),
                encoding="utf-8",
            )
            temporary.replace(path)
        except Exception:
            return

    def _archive_current_mission(self) -> None:
        plan_id = _as_int(self._mission.get("missionPlanID"))
        if plan_id is None:
            return
        archived_at = _now_ms_2000()
        archived_geometry = self._empty_geometry()
        archived_parts: list[dict[str, Any]] = []
        for part in self._last_parts_by_input.values():
            if not (bool(part.get("isDone")) or bool(part.get("isCurrent"))):
                continue
            archived_part = deepcopy(part)
            archived_quality = archived_part.get("quality") or {}
            archived_part.setdefault("measuredGsd", archived_part.get("gsd"))
            archived_part.setdefault("targetGsd", archived_part.get("requiredGSD"))
            archived_part.setdefault("qualitySamples", archived_quality.get("samples", 0))
            archived_part.setdefault(
                "qualitySatisfaction",
                archived_quality.get("satisfactionPercent"),
            )
            archived_part.update(
                {
                    "isHistorical": True,
                    "isCurrent": False,
                    "isDone": True,
                    "historyPlanID": int(plan_id),
                    "historyTimestamp": int(archived_at),
                    "status": "완료 이력",
                    "statusTone": "done",
                }
            )
            archived_parts.append(archived_part)
        included = 0
        for collection_name in ("inputAreas", "inputLines", "lineCorridors"):
            features: list[dict[str, Any]] = []
            collection = self._mission_geometry.get(collection_name) or {}
            for feature in collection.get("features") or []:
                if not isinstance(feature, dict):
                    continue
                properties = feature.get("properties") or {}
                input_id = _as_int(properties.get("inputMissionID"))
                part = self._last_parts_by_input.get(input_id or -1, {})
                if not (
                    bool(properties.get("isDone"))
                    or bool(part.get("isDone"))
                    or bool(part.get("isCurrent"))
                ):
                    continue
                coverage = self._last_coverage_by_input.get(input_id or -1, {})
                archived = deepcopy(feature)
                archived_props = archived.setdefault("properties", {})
                archived_props.update(
                    {
                        "isHistorical": True,
                        "isDone": True,
                        "historyPlanID": int(plan_id),
                        "historyTimestamp": int(archived_at),
                        "statusLabel": "\uc644\ub8cc \uc774\ub825",
                        "statusTone": "done",
                        "coverageValue": coverage.get("percent", -1),
                        "coveredValue": coverage.get("covered", -1),
                        "plannedValue": coverage.get("planned", -1),
                        "coverageUnit": coverage.get("unit", ""),
                        "coverageSource": coverage.get("source", ""),
                        "typeLabel": part.get("type", properties.get("inputMissionType", "-")),
                        "regionLabel": part.get("region", properties.get("regionType", "-")),
                        "measuredGsd": part.get("measuredGsd", -1),
                        "targetGsd": part.get("targetGsd", -1),
                        "gsdState": (
                            "pass"
                            if part.get("gsdSatisfied") is True
                            else "fail" if part.get("gsdSatisfied") is False else "unknown"
                        ),
                        "qualitySamples": part.get("qualitySamples", 0),
                        "qualitySatisfaction": part.get("qualitySatisfaction", -1),
                        "activeAircraftCount": 0,
                    }
                )
                features.append(archived)
                included += 1
            archived_geometry[collection_name] = _feature_collection(features)
        if included <= 0 and not archived_parts:
            return
        entry = {
            "planID": int(plan_id),
            "archivedTimestamp": int(archived_at),
            "geometry": archived_geometry,
            "missionParts": archived_parts,
        }
        self._completed_mission_history = [
            item
            for item in self._completed_mission_history
            if _as_int(item.get("planID")) != int(plan_id)
        ]
        self._completed_mission_history.append(entry)
        self._completed_mission_history = self._completed_mission_history[-8:]
        self._persist_mission_history()

    def _mission_part_history(
        self,
        parts: list[dict[str, Any]],
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        latest_by_input: dict[int, dict[str, Any]] = {}
        for entry in self._completed_mission_history:
            for part in entry.get("missionParts") or []:
                if not isinstance(part, dict):
                    continue
                input_id = _as_int(part.get("inputMissionID"))
                if input_id is not None:
                    latest_by_input[input_id] = part

        active_ids: set[int] = set()
        hydrated_parts: list[dict[str, Any]] = []
        for source_part in parts:
            part = deepcopy(source_part)
            input_id = _as_int(part.get("inputMissionID"))
            if input_id is not None:
                active_ids.add(input_id)
            historical = latest_by_input.get(input_id or -1)
            if historical is not None and bool(part.get("isDone")):
                current_coverage = _as_float(part.get("coverage"))
                history_coverage = _as_float(historical.get("coverage"))
                if history_coverage is not None and (
                    current_coverage is None or history_coverage > current_coverage
                ):
                    part["coverage"] = history_coverage
                    part["coverageDetail"] = deepcopy(historical.get("coverageDetail") or {})

                current_quality = part.get("quality") or {}
                history_quality = historical.get("quality") or {}
                current_samples = (
                    _as_int(part.get("qualitySamples"))
                    or _as_int(current_quality.get("samples"))
                    or 0
                )
                history_samples = (
                    _as_int(historical.get("qualitySamples"))
                    or _as_int(history_quality.get("samples"))
                    or 0
                )
                if history_samples > current_samples:
                    for key in (
                        "gsd",
                        "requiredGSD",
                        "measuredGsd",
                        "targetGsd",
                        "gsdSatisfied",
                        "qualitySamples",
                        "qualitySatisfaction",
                        "quality",
                    ):
                        if key in historical:
                            part[key] = deepcopy(historical[key])
                    part["historyPlanID"] = historical.get("historyPlanID")
            hydrated_parts.append(part)

        history_parts = [
            deepcopy(part)
            for input_id, part in latest_by_input.items()
            if input_id not in active_ids
        ]
        history_parts.sort(
            key=lambda item: (
                _as_int(item.get("historyTimestamp")) or 0,
                _as_int(item.get("sequence")) or 0,
            )
        )
        return hydrated_parts, history_parts

    @staticmethod
    def _feature_history_key(collection_name: str, feature: dict[str, Any]) -> str:
        properties = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        return "|".join(
            (
                collection_name,
                str(properties.get("inputMissionID")),
                json.dumps(geometry, sort_keys=True, separators=(",", ":")),
            )
        )

    def _geometry_with_history(self) -> dict[str, Any]:
        result = self._empty_geometry()
        active_plan_id = _as_int(self._mission.get("missionPlanID"))
        for collection_name in ("inputAreas", "inputLines", "lineCorridors"):
            active_features = list(
                ((self._mission_geometry.get(collection_name) or {}).get("features") or [])
            )
            seen = {
                self._feature_history_key(collection_name, feature)
                for feature in active_features
                if isinstance(feature, dict)
            }
            historical_features: list[dict[str, Any]] = []
            for entry in reversed(self._completed_mission_history):
                if _as_int(entry.get("planID")) == active_plan_id:
                    continue
                geometry = entry.get("geometry") or {}
                collection = geometry.get(collection_name) or {}
                for feature in collection.get("features") or []:
                    if not isinstance(feature, dict):
                        continue
                    key = self._feature_history_key(collection_name, feature)
                    if key in seen:
                        continue
                    seen.add(key)
                    historical_features.append(feature)
            result[collection_name] = _feature_collection(
                list(reversed(historical_features)) + active_features
            )
        result["paths"] = deepcopy(self._mission_geometry.get("paths") or _feature_collection([]))
        return result

    def _remaining_coverage_geometry(self, plan_id: int | None) -> dict[str, Any]:
        if plan_id is None:
            return {
                "revision": "",
                "remainingAreas": _feature_collection([]),
                "coverageDepth": _feature_collection([]),
                "coveragePassAttribution": _feature_collection([]),
                "coverageDepthSummaries": [],
                "coveragePassSummaries": [],
                "coveragePassRequirementMode": "all_passes_required",
            }

        # The monitor polls every two seconds.  The Area ledger is file-backed,
        # so a cheap stat token is sufficient to avoid reparsing and rebuilding
        # unchanged polygons on every request.  Mission/root identity keeps the
        # cache isolated across replans and scenario transitions.
        snapshot_path = (
            Path(db_paths.get_active_db_root())
            / "DSS_Internal"
            / "mission_area_replan"
            / f"mission_area_snapshot_{int(plan_id)}.json"
        )
        try:
            stat = snapshot_path.stat()
            file_token: tuple[Any, ...] = (
                True,
                int(stat.st_mtime_ns),
                int(stat.st_size),
            )
        except OSError:
            file_token = (False, 0, 0)
        cache_key = (
            self._db_root_key(),
            int(plan_id),
            self._mission_signature,
            *file_token,
        )
        if (
            cache_key == self._remaining_area_cache_key
            and self._remaining_area_cache_payload is not None
        ):
            return self._remaining_area_cache_payload

        try:
            payload = build_remaining_area_snapshot(int(plan_id))
        except Exception:
            payload = {}
        payload_plan_id = _as_int(payload.get("missionPlanID")) if isinstance(payload, dict) else None
        if (
            not isinstance(payload, dict)
            or payload.get("available", True) is False
            or payload_plan_id != int(plan_id)
        ):
            # Never carry the previous plan's geometry into a new plan while
            # its exact snapshot is still being published.
            payload = {}
        features = (
            ((payload.get("featureCollection") or {}).get("features") or [])
            if isinstance(payload, dict)
            else []
        )
        remaining_features: list[dict[str, Any]] = []
        depth_features: list[dict[str, Any]] = []
        pass_features: list[dict[str, Any]] = []
        for feature in features:
            if not isinstance(feature, dict):
                continue
            properties = feature.get("properties") or {}
            role = str(properties.get("visualizationRole") or "")
            if role == "coverageDepth":
                depth_features.append(feature)
            elif role == "coveragePassAttribution":
                pass_features.append(feature)
            elif (
                str(properties.get("missionKind") or "").strip().lower() == "area"
                and str((feature.get("geometry") or {}).get("type") or "")
                in {"Polygon", "MultiPolygon"}
            ):
                # Legacy/simple snapshots expose their live Area as
                # remainingDetail/areaOwnershipProjection without a
                # visualizationRole.  SIM renders these directly; retain them
                # here as a separate source instead of dropping them.
                remaining_features.append(feature)

        # The central snapshot intentionally persists only outstanding depth
        # obligations.  This dashboard also owns the immutable input-area
        # geometry, so it can render the complementary 2/2-complete band
        # without bloating every replan contract.
        if shape is not None and unary_union is not None and mapping is not None:
            depth_by_input: dict[int, list[dict[str, Any]]] = {}
            for feature in depth_features:
                input_id = _as_int((feature.get("properties") or {}).get("inputMissionID"))
                if input_id is not None:
                    depth_by_input.setdefault(input_id, []).append(feature)
            remaining_input_ids = {
                input_id
                for input_id in (
                    _as_int((feature.get("properties") or {}).get("inputMissionID"))
                    for feature in remaining_features
                )
                if input_id is not None
            }
            for input_feature in (
                (self._mission_geometry.get("inputAreas") or {}).get("features") or []
            ):
                if not isinstance(input_feature, dict):
                    continue
                input_id = _as_int((input_feature.get("properties") or {}).get("inputMissionID"))
                rows = depth_by_input.get(input_id or -1, [])
                if input_id is None or any(
                    _as_int((row.get("properties") or {}).get("coverageDepth")) == 2
                    for row in rows
                ):
                    continue
                if not rows and input_id in remaining_input_ids:
                    # A live legacy remainingDetail is not evidence that the
                    # whole input Area has reached 2/2 coverage.
                    continue
                try:
                    required_geometry = shape(input_feature.get("geometry") or {})
                    incomplete = unary_union(
                        [
                            shape(row.get("geometry") or {})
                            for row in rows
                            if _as_int((row.get("properties") or {}).get("coverageDepth")) in {0, 1}
                        ]
                    ) if rows else None
                    completed = (
                        required_geometry.difference(incomplete)
                        if incomplete is not None and not incomplete.is_empty
                        else required_geometry
                    )
                    if completed.is_empty:
                        continue
                    completed_props = deepcopy(input_feature.get("properties") or {})
                    completed_props.update(
                        {
                            "visualizationRole": "coverageDepth",
                            "coverageDepth": 2,
                            "remainingCaptureCount": 0,
                            "requiredCoverageDepth": 2,
                            "coverageDepthStatus": "complete",
                            "coverageDepthLabel": "2/2 complete",
                            "coverageDepthDerived": 1,
                            "activeAircraftIDs": "",
                            "activeAgents": "",
                            "activeCoveragePasses": "",
                            "isDone": 1,
                        }
                    )
                    depth_features.append(
                        {
                            "type": "Feature",
                            "geometry": mapping(completed),
                            "properties": completed_props,
                        }
                    )
                except Exception:
                    continue
        result = {
            "revision": str(payload.get("dataRevision") or "") if isinstance(payload, dict) else "",
            "remainingAreas": _feature_collection(remaining_features),
            "coverageDepth": _feature_collection(depth_features),
            "coveragePassAttribution": _feature_collection(pass_features),
            "coverageDepthSummaries": list(payload.get("coverageDepthSummaries") or [])
            if isinstance(payload, dict)
            else [],
            "coveragePassSummaries": list(payload.get("coveragePassSummaries") or [])
            if isinstance(payload, dict)
            else [],
            "coveragePassRequirementMode": "all_passes_required",
        }
        self._remaining_area_cache_key = cache_key
        self._remaining_area_cache_payload = result
        return result

    def _install_mission(self, mission: dict[str, Any], signature: str) -> None:
        db_root_key = self._db_root_key()
        self._ensure_history_loaded(db_root_key)
        previous_plan_id = _as_int(self._mission.get("missionPlanID"))
        next_plan_id = _as_int(mission.get("missionPlanID"))
        previous_input_package_id = _as_int(
            self._mission.get("inputMissionPackageID")
        )
        next_input_package_id = _as_int(mission.get("inputMissionPackageID"))
        same_initial_domain = bool(
            self._mission_db_root == db_root_key
            and previous_input_package_id is not None
            and next_input_package_id is not None
            and int(previous_input_package_id) == int(next_input_package_id)
        )
        if (
            self._mission_db_root == db_root_key
            and previous_plan_id is not None
            and next_plan_id is not None
            and int(previous_plan_id) != int(next_plan_id)
        ):
            self._archive_current_mission()
        elif self._mission_db_root != db_root_key:
            self._last_coverage_by_input = {}
            self._last_parts_by_input = {}
        self._mission = mission
        self._mission_signature = signature
        self._mission_db_root = db_root_key
        geometry, bounds = self._build_geometry(mission)
        self._mission_geometry = geometry
        self._mission_bounds = bounds
        plan_id = _as_int(mission.get("missionPlanID"))
        if plan_id != self._last_plan_id:
            if not same_initial_domain:
                self._quality_by_key.clear()
                self._spatial_quality_by_input.clear()
                self._last_sample_timestamp = None
                self._initial_quality_domain_by_input = (
                    self._build_initial_quality_domains()
                )
            elif not self._initial_quality_domain_by_input:
                self._initial_quality_domain_by_input = (
                    self._build_initial_quality_domains()
                )
            self._add_event("info", "PLAN", f"MissionPlan {plan_id or '-'} 로드")
            self._last_plan_id = plan_id
            self._last_current_by_aircraft.clear()

    def _restore_active_progress_mission(
        self,
        snapshot: dict[str, Any],
        mission: dict[str, Any] | None,
    ) -> tuple[dict[str, Any] | None, str | None]:
        """Recover the active plan when this optional viewer starts after 0903."""
        source = mission.get("source") if isinstance(mission, dict) else self._upstream_plan_source
        if source != "db":
            return mission, None

        base = Path(db_paths.get_active_db_root()) / "DSS_Internal"
        progress = self._json_cache.read(base / "coverage_progress.json")
        progress_plan_id = _as_int(progress.get("mission_plan_id"))
        progress_timestamp = _as_int(progress.get("timestamp_ms"))
        snapshot_timestamp = _as_int(snapshot.get("timestamp"))
        db_plan_id = _as_int(mission.get("missionPlanID")) if isinstance(mission, dict) else None
        current_plan_id = _as_int(self._mission.get("missionPlanID"))
        if (
            progress_plan_id is None
            or progress_timestamp is None
            or snapshot_timestamp is None
            or abs(snapshot_timestamp - progress_timestamp) > 15_000
        ):
            return mission, None
        if progress_plan_id == db_plan_id:
            return mission, None
        if (
            progress_plan_id == current_plan_id
            and self._mission.get("source") == "coverage-progress"
        ):
            return None, None

        restored = build_mission_plan_payload(
            progress_plan_id,
            db_root=db_paths.get_active_db_root(),
        )
        if not restored.get("ok"):
            return mission, None
        signature = f"{progress_plan_id}:coverage-progress:{progress_timestamp}"
        return (
            {
                **restored,
                "signature": signature,
                "source": "coverage-progress",
                "selectedTimestamp": progress_timestamp,
            },
            signature,
        )

    def _build_geometry(self, mission: dict[str, Any]) -> tuple[dict[str, Any], list[float] | None]:
        areas: list[dict[str, Any]] = []
        lines: list[dict[str, Any]] = []
        corridors: list[dict[str, Any]] = []
        paths: list[dict[str, Any]] = []
        all_coords: list[list[float]] = []
        input_plans = mission.get("inputMissionPlans") or []
        input_plan = input_plans[0] if input_plans and isinstance(input_plans[0], dict) else {}
        for index, item in enumerate(input_plan.get("inputMissionList") or [], start=1):
            if not isinstance(item, dict):
                continue
            input_id = _as_int(item.get("inputMissionID"))
            props = {
                "inputMissionID": input_id,
                "sequence": index,
                "inputMissionType": _as_int(item.get("inputMissionType")),
                "regionType": _as_int(item.get("regionType")),
                "isDone": bool(item.get("isDone")),
            }
            detail = item.get("missionDetail") or {}
            area_list = detail.get("areaList") or []
            outers: list[list[list[float]]] = []
            holes: list[list[list[float]]] = []
            for area in area_list if isinstance(area_list, list) else []:
                ring = _coords((area or {}).get("coordinateList"))
                if len(ring) < 3:
                    continue
                if ring[0] != ring[-1]:
                    ring.append(ring[0])
                all_coords.extend(ring)
                (holes if bool((area or {}).get("isHole")) else outers).append(ring)
            if outers:
                if len(outers) == 1:
                    polygon_geometry = {"type": "Polygon", "coordinates": [outers[0], *holes]}
                else:
                    polygon_geometry = {"type": "MultiPolygon", "coordinates": [[ring] for ring in outers]}
                areas.append({"type": "Feature", "properties": props, "geometry": polygon_geometry})

            for line_index, line in enumerate(detail.get("lineList") or []):
                line_coords = _coords((line or {}).get("coordinateList"))
                if len(line_coords) < 2:
                    continue
                all_coords.extend(line_coords)
                width = _as_float((line or {}).get("width")) or 0.0
                line_props = {**props, "lineIndex": line_index, "widthM": width}
                lines.append(
                    {
                        "type": "Feature",
                        "properties": line_props,
                        "geometry": {"type": "LineString", "coordinates": line_coords},
                    }
                )
                corridor_geometry = _buffer_line(line_coords, width)
                if corridor_geometry is not None:
                    corridors.append(
                        {"type": "Feature", "properties": line_props, "geometry": corridor_geometry}
                    )

        for feature in mission.get("features") or []:
            if not isinstance(feature, dict):
                continue
            path_id = _as_int(feature.get("pathId"))
            path_index = mission.get("pathMissionIndex") or {}
            path_meta = (
                path_index.get(path_id)
                or path_index.get(str(path_id))
                or {}
            ) if path_id is not None and isinstance(path_index, dict) else {}
            if not isinstance(path_meta, dict):
                path_meta = {}
            path_coords = feature.get("coords") or []
            clean = []
            for point in path_coords if isinstance(path_coords, list) else []:
                if isinstance(point, (list, tuple)) and len(point) >= 2:
                    lon, lat = _as_float(point[0]), _as_float(point[1])
                    if lon is not None and lat is not None:
                        clean.append([lon, lat])
            if len(clean) < 2:
                continue
            all_coords.extend(clean)
            paths.append(
                {
                    "type": "Feature",
                    "properties": {
                        "aircraftID": _as_int(feature.get("aircraftId")),
                        "agent": feature.get("agent"),
                        "pathID": path_id,
                        "inputMissionID": _as_int(path_meta.get("inputMissionID")),
                        "individualMissionID": _as_int(path_meta.get("individualMissionID")),
                        "isDone": bool(feature.get("isDone")),
                    },
                    "geometry": {"type": "LineString", "coordinates": clean},
                }
            )
        bounds = None
        if all_coords:
            lons = [point[0] for point in all_coords]
            lats = [point[1] for point in all_coords]
            bounds = [min(lons), min(lats), max(lons), max(lats)]
        return (
            {
                "inputAreas": _feature_collection(areas),
                "inputLines": _feature_collection(lines),
                "lineCorridors": _feature_collection(corridors),
                "paths": _feature_collection(paths),
            },
            bounds,
        )

    def _raw_0401(self) -> dict[str, Any] | None:
        try:
            direct = self.integration.latest_0401() if self.integration is not None else None
            if isinstance(direct, dict):
                return direct
        except Exception:
            pass
        wrapper = agent_status_snapshot.load_agent_status_snapshot() or {}
        raw = wrapper.get("raw") if isinstance(wrapper, dict) else None
        return raw if isinstance(raw, dict) else None

    def _option_assignment_state(self) -> dict[str, Any]:
        root = Path(db_paths.get_active_db_root())
        try:
            latest = getattr(self.integration, "latest_payload", None)
            live_payload = latest("0701") if callable(latest) else None
        except Exception:
            live_payload = None
        live_body = _option_payload_object(live_payload)

        now = time.monotonic()
        live_key = _option_assignment_payload_signature(live_body) if live_body else ""
        if (
            live_key
            and live_key == self._option_assignment_payload_key
            and now - self._option_assignment_last_probe < 0.8
            and str(root.resolve()) in self._option_assignment_cache_key
        ):
            return deepcopy(self._option_assignment_snapshot)
        if (
            not live_body
            and now - self._option_assignment_last_probe < 0.8
            and str(root.resolve()) in self._option_assignment_cache_key
        ):
            return deepcopy(self._option_assignment_snapshot)

        self._option_assignment_last_probe = now
        disk_body = _latest_option_payload_from_db(root)
        candidates = [body for body in (live_body, disk_body) if body]
        if not candidates:
            empty_key = f"{root.resolve()}|empty"
            if empty_key != self._option_assignment_cache_key:
                self._option_assignment_cache_key = empty_key
                self._option_assignment_payload_key = ""
                self._option_assignment_snapshot = {
                    "available": False,
                    "timestamp": None,
                    "signature": "",
                    "optionCount": 0,
                    "options": [],
                }
            return deepcopy(self._option_assignment_snapshot)

        # Prefer the newest message; on equal timestamps, prefer the live bus copy.
        body = max(
            candidates,
            key=lambda candidate: (
                _option_payload_timestamp(candidate),
                int(candidate is live_body),
            ),
        )
        payload_key = _option_assignment_payload_signature(body)
        artifact_key = _option_assignment_artifact_signature(body, root)
        cache_key = f"{root.resolve()}|{payload_key}|{artifact_key}"
        if cache_key == self._option_assignment_cache_key:
            return deepcopy(self._option_assignment_snapshot)

        previous_payload_key = self._option_assignment_payload_key
        snapshot = build_option_assignment_snapshot(body, db_root=root)
        snapshot["signature"] = f"{payload_key}|{artifact_key}"
        self._option_assignment_cache_key = cache_key
        self._option_assignment_payload_key = payload_key
        self._option_assignment_snapshot = snapshot
        if payload_key and payload_key != previous_payload_key:
            self._add_event(
                "info",
                "OPTION",
                f"후보 옵션 {int(snapshot.get('optionCount') or 0)}개 UAV 할당영역 갱신",
                _as_int(snapshot.get("timestamp")),
            )
        return deepcopy(snapshot)

    def _current_mission_maps(self) -> tuple[dict[int, dict[str, Any]], dict[int, int]]:
        entries: dict[int, dict[str, Any]] = {}
        input_map: dict[int, int] = {}
        for item in self._view.get("uav_entries") or []:
            if not isinstance(item, dict):
                continue
            aircraft_id = _as_int(item.get("aircraft_id"))
            current_id = _as_int(item.get("current_individual_mission_id"))
            if aircraft_id is None:
                continue
            current = next(
                (
                    mission
                    for mission in item.get("missions") or []
                    if _as_int((mission or {}).get("individual_mission_id")) == current_id
                ),
                None,
            )
            if isinstance(current, dict):
                entries[aircraft_id] = current
                input_id = _as_int(current.get("input_id"))
                if input_id is not None:
                    input_map[aircraft_id] = input_id
        return entries, input_map

    def _update_quality(self, raw: dict[str, Any] | None, plan_id: int | None) -> None:
        if plan_id is None:
            return
        timestamp, states = extract_0401_agent_states(raw)
        if timestamp is None or timestamp == self._last_sample_timestamp:
            return
        self._last_sample_timestamp = timestamp
        current_missions, _ = self._current_mission_maps()
        cfg = self._quality_settings
        gamma = cfg["obj_w_m"] * cfg["obj_h_m"] / (cfg["obj_min_px_x"] * cfg["obj_min_px_y"])
        required_gsd = math.sqrt(gamma)
        total_pixels = cfg["img_w_px"] * cfg["img_h_px"]
        for state in states:
            aircraft_id = _as_int(state.get("aircraft_id"))
            mission = current_missions.get(aircraft_id or -1)
            if aircraft_id is None or mission is None:
                continue
            if _as_int(state.get("sensor_operation_mode")) != 2:
                continue
            if _as_int(state.get("flying")) != 1 or _as_int(state.get("filming")) != 1:
                continue
            current_waypoint_id = _as_int(state.get("current_waypoint_id"))
            waypoint = next(
                (
                    item
                    for item in mission.get("waypoints") or []
                    if _as_int((item or {}).get("waypoint_id")) == current_waypoint_id
                ),
                None,
            )
            if not isinstance(waypoint, dict):
                continue
            line_search_points = _as_int(waypoint.get("line_search_point_count")) or 0
            if not bool(waypoint.get("has_line_search")) and line_search_points < 2:
                continue
            waypoint_mode = _as_int(waypoint.get("operation_mode"))
            if waypoint_mode is not None and waypoint_mode != 2:
                continue
            width, height = _footprint_size(state.get("footprint_corners"))
            if width is None or height is None or width <= 0 or height <= 0:
                continue
            footprint_area = width * height
            eq_gsd = math.sqrt(footprint_area / total_pixels)
            input_id = _as_int(mission.get("input_id"))
            if input_id is None:
                continue
            clipped_footprint, accepted = (
                self._clip_quality_footprint_to_initial_domain(
                    int(input_id),
                    state.get("footprint_corners"),
                )
            )
            if not accepted:
                continue
            key = (aircraft_id, input_id)
            row = self._quality_by_key.setdefault(
                key,
                {
                    "aircraftID": aircraft_id,
                    "inputMissionID": input_id,
                    "samples": 0,
                    "satisfied": 0,
                    "eqGsdSum": 0.0,
                    "latestEqGsd": None,
                    "latestFootprintAreaM2": None,
                    "timestamp": None,
                },
            )
            row["samples"] += 1
            row["satisfied"] += int(eq_gsd <= required_gsd)
            row["eqGsdSum"] += eq_gsd
            row["latestEqGsd"] = eq_gsd
            row["latestFootprintAreaM2"] = footprint_area
            row["timestamp"] = timestamp
            self._update_spatial_quality(
                input_id=int(input_id),
                clipped_footprint=clipped_footprint,
                eq_gsd=float(eq_gsd),
                required_gsd=float(required_gsd),
            )

    def _initial_domain_snapshot_coverage(
        self,
        plan_id: int,
    ) -> dict[int, dict[str, Any]]:
        """Read exact current-plan coverage against the original input domain."""

        path = (
            Path(db_paths.get_active_db_root())
            / "DSS_Internal"
            / "mission_area_replan"
            / f"mission_area_snapshot_{int(plan_id)}.json"
        )
        snapshot = self._json_cache.read(path)
        if _as_int(snapshot.get("missionPlanID")) not in (None, int(plan_id)):
            return {}
        result: dict[int, dict[str, Any]] = {}
        for entry in snapshot.get("missions") or []:
            if not isinstance(entry, dict):
                continue
            input_id = _as_int(entry.get("inputMissionID"))
            planned = _as_float(entry.get("plannedAreaM2")) or 0.0
            remaining = _as_float(entry.get("remainingAreaM2"))
            if input_id is None or planned <= 0.0:
                continue
            remaining = max(0.0, min(planned, float(remaining or 0.0)))
            covered = max(0.0, planned - remaining)
            percent = covered / planned * 100.0
            tolerance = max(0.05, planned * 1e-6)
            result[int(input_id)] = {
                "kind": str(entry.get("missionType") or "").strip().lower(),
                "source": "initial-domain-snapshot",
                "basis": "initial-input-domain",
                "percent": round(max(0.0, min(100.0, percent)), 2),
                "covered": round(covered, 2),
                "planned": round(planned, 2),
                "remaining": round(remaining, 2),
                "unit": "m²",
                "spatialPercent": round(max(0.0, min(100.0, percent)), 2),
                "spatialCovered": round(covered, 2),
                "spatialPlanned": round(planned, 2),
                "requirementsMet": bool(
                    bool(entry.get("isDone")) or remaining <= tolerance
                ),
            }
        return result

    def _coverage(self, plan_id: int | None) -> tuple[dict[int, dict[str, Any]], dict[str, Any]]:
        if plan_id is None:
            return {}, {
                "overallPercent": None,
                "overall": None,
                "percent": None,
                "line": None,
                "area": None,
                "trackedMissionCount": 0,
            }
        base = Path(db_paths.get_active_db_root()) / "DSS_Internal"
        area_data = self._json_cache.read(base / "coverage_progress.json")
        line_data = self._json_cache.read(base / "line_scan_progress.json")
        by_input: dict[int, dict[str, Any]] = {}
        shape_by_input: dict[int, str] = {}
        input_plans = self._mission.get("inputMissionPlans") or []
        input_plan = input_plans[0] if input_plans and isinstance(input_plans[0], dict) else {}
        for item in input_plan.get("inputMissionList") or []:
            if not isinstance(item, dict):
                continue
            input_id = _as_int(item.get("inputMissionID"))
            detail = item.get("missionDetail") or {}
            if input_id is not None:
                shape_by_input[input_id] = "area" if detail.get("areaList") else "line"

        area_matches_plan = _as_int(area_data.get("mission_plan_id")) in (None, plan_id)
        if area_matches_plan:
            has_explicit_footprint = "input_footprint_coverage" in area_data
            footprint_rows = (
                area_data.get("input_footprint_coverage")
                if has_explicit_footprint
                else area_data.get("input_coverage")
            ) or {}
            for key, value in footprint_rows.items():
                input_id = _as_int(key)
                if input_id is None or not isinstance(value, dict):
                    continue
                if not has_explicit_footprint and shape_by_input.get(input_id) == "line":
                    # Legacy input_coverage stored line progress in meter fields
                    # named as square meters. Never present that as footprint area.
                    continue
                planned = _as_float(
                    value.get("coverage_work_required_area_m2", value.get("planned_area_m2"))
                ) or 0.0
                enabled = bool(value.get("coverage_enabled"))
                if planned <= 0.0 and not enabled:
                    continue
                by_input[input_id] = _footprint_coverage_row(
                    value,
                    kind=shape_by_input.get(input_id, "area"),
                )
        # If footprint aggregation is temporarily unavailable during a plan
        # handoff, use the exact snapshot whose plannedAreaM2 is still the
        # original input domain.  This prevents a 3->2->3 route split from
        # changing the denominator to the new aircraft path lengths.
        for input_id, snapshot_row in self._initial_domain_snapshot_coverage(
            int(plan_id)
        ).items():
            if input_id in by_input:
                continue
            row = dict(snapshot_row)
            row["kind"] = shape_by_input.get(
                int(input_id),
                str(row.get("kind") or "area"),
            )
            by_input[int(input_id)] = row
        grouped: dict[int, dict[str, float]] = {}
        for entry in line_data.get("entries") or []:
            if not isinstance(entry, dict) or not bool(entry.get("enabled", True)):
                continue
            if plan_id is not None and _as_int(entry.get("missionPlanID")) not in (None, plan_id):
                continue
            input_id = _as_int(entry.get("inputMissionID"))
            if input_id is None:
                continue
            group = grouped.setdefault(input_id, {"planned": 0.0, "covered": 0.0})
            group["planned"] += _as_float(entry.get("plannedLengthM")) or 0.0
            group["covered"] += _as_float(entry.get("coveredLengthM")) or 0.0
        for input_id, values in grouped.items():
            if input_id in by_input:
                continue
            planned = values["planned"]
            covered = min(planned, values["covered"]) if planned > 0 else values["covered"]
            by_input[input_id] = {
                "kind": "line",
                "source": "route-progress",
                "basis": "current-assignment-fallback",
                "percent": round((covered / planned * 100.0) if planned > 0 else 0.0, 2),
                "covered": round(covered, 2),
                "planned": round(planned, 2),
                "unit": "m",
                "requirementsMet": bool(planned > 0.0 and covered >= planned - 1e-6),
            }
        tracked = [item for item in by_input.values() if item.get("planned", 0) > 0]
        footprint = [
            item
            for item in tracked
            if item.get("source") in {"footprint", "initial-domain-snapshot"}
        ]
        plan_coverage = (
            area_data.get("plan_footprint_coverage")
            if "plan_footprint_coverage" in area_data
            else {}
        ) or {}
        plan_covered = _as_float(plan_coverage.get("status_union_covered_area_m2")) or 0.0
        plan_planned = _as_float(plan_coverage.get("status_union_required_area_m2")) or 0.0
        plan_percent = _as_float(plan_coverage.get("status_union_coverage_percent"))
        overall = (
            round(max(0.0, min(100.0, float(plan_percent))), 2)
            if area_matches_plan and plan_percent is not None
            else round(min(plan_covered, plan_planned) / plan_planned * 100.0, 2)
            if area_matches_plan and plan_planned > 0.0
            else None
        )
        if overall is None and footprint:
            total_covered = sum(float(item["covered"]) for item in footprint)
            total_planned = sum(float(item["planned"]) for item in footprint)
            overall = round(min(total_covered, total_planned) / total_planned * 100.0, 2)

        def weighted_kind(kind: str) -> float | None:
            values = [
                item
                for item in footprint
                if item.get("kind") == kind and item.get("planned", 0) > 0
            ]
            if not values and kind == "line":
                values = [
                    item
                    for item in tracked
                    if item.get("kind") == "line" and item.get("source") == "route-progress"
                ]
            planned = sum(float(item["planned"]) for item in values)
            covered = sum(float(item["covered"]) for item in values)
            return round(min(covered, planned) / planned * 100.0, 2) if planned > 0.0 else None

        summary = {
            "overallPercent": overall,
            "overall": overall,
            "percent": overall,
            "line": weighted_kind("line"),
            "area": weighted_kind("area"),
            "areaSpatial": (
                round(
                    sum(
                        float(item.get("spatialCovered") or 0.0)
                        for item in footprint
                        if item.get("kind") == "area"
                    )
                    / max(
                        1e-9,
                        sum(
                            float(item.get("spatialPlanned") or 0.0)
                            for item in footprint
                            if item.get("kind") == "area"
                        ),
                    )
                    * 100.0,
                    2,
                )
                if any(
                    float(item.get("spatialPlanned") or 0.0) > 0.0
                    for item in footprint
                    if item.get("kind") == "area"
                )
                else None
            ),
            "requirementsMet": bool(tracked) and all(
                bool(item.get("requirementsMet", True)) for item in tracked
            ),
            "trackedMissionCount": len(tracked),
        }
        return by_input, summary

    def _quality_rows(self) -> tuple[dict[int, dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
        by_input_raw: dict[int, dict[str, float]] = {}
        vehicles: list[dict[str, Any]] = []
        cfg = self._quality_settings
        required_gsd = math.sqrt(
            cfg["obj_w_m"] * cfg["obj_h_m"] / (cfg["obj_min_px_x"] * cfg["obj_min_px_y"])
        )
        for row in self._quality_by_key.values():
            samples = int(row["samples"])
            satisfied = int(row["satisfied"])
            input_id = int(row["inputMissionID"])
            bucket = by_input_raw.setdefault(input_id, {"samples": 0, "satisfied": 0, "eqGsdSum": 0.0})
            bucket["samples"] += samples
            bucket["satisfied"] += satisfied
            bucket["eqGsdSum"] += float(row["eqGsdSum"])
            vehicles.append(
                {
                    "aircraftID": row["aircraftID"],
                    "inputMissionID": input_id,
                    "samples": samples,
                    "satisfactionPercent": round(satisfied / samples * 100.0, 1) if samples else None,
                    "averageGsd": round(float(row["eqGsdSum"]) / samples, 4) if samples else None,
                    "latestGsd": _round(row.get("latestEqGsd"), 4),
                    "latestFootprintAreaM2": _round(row.get("latestFootprintAreaM2"), 1),
                    "timestamp": row.get("timestamp"),
                }
            )
        by_input: dict[int, dict[str, Any]] = {}
        total_samples = 0
        total_satisfied = 0
        spatial_total_covered_m2 = 0.0
        spatial_total_satisfied_m2 = 0.0
        spatial_total_required_m2 = 0.0
        spatial_weighted_gsd_sum = 0.0
        spatial_weighted_area_m2 = 0.0
        for input_id, row in by_input_raw.items():
            samples = int(row["samples"])
            satisfied = int(row["satisfied"])
            total_samples += samples
            total_satisfied += satisfied
            temporal_average_gsd = row["eqGsdSum"] / samples if samples else None
            quality_row: dict[str, Any] = {
                "samples": samples,
                "satisfactionPercent": round(satisfied / samples * 100.0, 1) if samples else None,
                "averageGsd": round(temporal_average_gsd, 4)
                if temporal_average_gsd is not None
                else None,
                "requiredGsd": round(required_gsd, 4),
                "basis": "initial-input-domain-time-samples",
                "requirementsMet": bool(samples > 0 and satisfied >= samples),
            }
            spatial_state = self._spatial_quality_by_input.get(int(input_id)) or {}
            covered_geometry = spatial_state.get("coveredGeometry")
            satisfied_geometry = spatial_state.get("satisfiedGeometry")
            covered_area_m2 = float(
                max(0.0, covered_geometry.area or 0.0)
            ) if covered_geometry is not None and not covered_geometry.is_empty else 0.0
            satisfied_area_m2 = float(
                max(0.0, satisfied_geometry.area or 0.0)
            ) if satisfied_geometry is not None and not satisfied_geometry.is_empty else 0.0
            satisfied_area_m2 = min(covered_area_m2, satisfied_area_m2)
            required_area_m2 = float(
                spatial_state.get("requiredAreaM2")
                or (self._initial_quality_domain_by_input.get(int(input_id)) or {}).get("areaM2")
                or 0.0
            )
            weighted_area_m2 = float(spatial_state.get("weightedAreaM2") or 0.0)
            weighted_sum = float(spatial_state.get("weightedGsdAreaSum") or 0.0)
            if covered_area_m2 > 0.0:
                spatial_satisfaction = satisfied_area_m2 / covered_area_m2 * 100.0
                spatial_average_gsd = (
                    weighted_sum / weighted_area_m2
                    if weighted_area_m2 > 0.0
                    else temporal_average_gsd
                )
                quality_row.update(
                    {
                        "basis": "initial-input-domain-spatial",
                        "satisfactionPercent": round(
                            max(0.0, min(100.0, spatial_satisfaction)),
                            1,
                        ),
                        "averageGsd": round(spatial_average_gsd, 4)
                        if spatial_average_gsd is not None
                        else None,
                        "evaluatedAreaM2": round(covered_area_m2, 2),
                        "gsdCompliantAreaM2": round(satisfied_area_m2, 2),
                        "requiredAreaM2": round(required_area_m2, 2),
                        "spatialCoveragePercent": round(
                            min(covered_area_m2, required_area_m2)
                            / required_area_m2
                            * 100.0,
                            2,
                        ) if required_area_m2 > 0.0 else None,
                        "qualityCoveragePercent": round(
                            min(satisfied_area_m2, required_area_m2)
                            / required_area_m2
                            * 100.0,
                            2,
                        ) if required_area_m2 > 0.0 else None,
                        "requirementsMet": bool(
                            covered_area_m2 > 0.0
                            and (covered_area_m2 - satisfied_area_m2)
                            <= max(0.05, covered_area_m2 * 1e-6)
                        ),
                    }
                )
                spatial_total_covered_m2 += covered_area_m2
                spatial_total_satisfied_m2 += satisfied_area_m2
                spatial_total_required_m2 += max(0.0, required_area_m2)
                spatial_weighted_gsd_sum += weighted_sum
                spatial_weighted_area_m2 += weighted_area_m2
            by_input[input_id] = quality_row

        temporal_average_gsd = (
            sum(float(row["eqGsdSum"]) for row in self._quality_by_key.values()) / total_samples
            if total_samples
            else None
        )
        average_gsd = (
            spatial_weighted_gsd_sum / spatial_weighted_area_m2
            if spatial_weighted_area_m2 > 0.0
            else temporal_average_gsd
        )
        satisfaction = (
            round(
                spatial_total_satisfied_m2 / spatial_total_covered_m2 * 100.0,
                1,
            )
            if spatial_total_covered_m2 > 0.0
            else round(total_satisfied / total_samples * 100.0, 1)
            if total_samples
            else None
        )
        return by_input, vehicles, {
            "samples": total_samples,
            "satisfactionPercent": satisfaction,
            "satisfaction": satisfaction,
            "score": satisfaction,
            "meanGSD": round(average_gsd * 100.0, 3) if average_gsd is not None else None,
            "requiredGsd": round(required_gsd, 4),
            "requiredGSD": round(required_gsd * 100.0, 3),
            "basis": (
                "initial-input-domain-spatial"
                if spatial_total_covered_m2 > 0.0
                else "initial-input-domain-time-samples"
            ),
            "evaluatedAreaM2": round(spatial_total_covered_m2, 2),
            "gsdCompliantAreaM2": round(spatial_total_satisfied_m2, 2),
            "requiredAreaM2": round(spatial_total_required_m2, 2),
            "qualityCoveragePercent": round(
                min(spatial_total_satisfied_m2, spatial_total_required_m2)
                / spatial_total_required_m2
                * 100.0,
                2,
            ) if spatial_total_required_m2 > 0.0 else None,
            "settings": cfg,
        }

    def _targets(self, snapshot: dict[str, Any]) -> list[dict[str, Any]]:
        raw_targets = snapshot.get("targets")
        if isinstance(raw_targets, list) and raw_targets:
            return raw_targets
        if isinstance(raw_targets, dict) and raw_targets:
            return list(raw_targets.values())
        target_info = self._json_cache.read(Path(db_paths.get_active_db_root()) / "DSS_Internal" / "targetInfo.json")
        values = target_info.get("targetList") or {}
        return list(values.values()) if isinstance(values, dict) else list(values) if isinstance(values, list) else []

    def _mission_parts(
        self,
        coverage: dict[int, dict[str, Any]],
        quality: dict[int, dict[str, Any]],
        current_inputs: dict[int, int],
    ) -> list[dict[str, Any]]:
        input_plans = self._mission.get("inputMissionPlans") or []
        input_plan = input_plans[0] if input_plans and isinstance(input_plans[0], dict) else {}
        active_counts = Counter(current_inputs.values())
        parts: list[dict[str, Any]] = []
        for sequence, item in enumerate(input_plan.get("inputMissionList") or [], start=1):
            if not isinstance(item, dict):
                continue
            input_id = _as_int(item.get("inputMissionID"))
            input_type = _as_int(item.get("inputMissionType"))
            region_type = _as_int(item.get("regionType"))
            detail = item.get("missionDetail") or {}
            shape = "AREA" if detail.get("areaList") else "LINE" if detail.get("lineList") else "POINT"
            coverage_detail = coverage.get(input_id or -1)
            quality_detail = quality.get(input_id or -1)
            measured_gsd_cm = (
                round(float(quality_detail["averageGsd"]) * 100.0, 3)
                if quality_detail and quality_detail.get("averageGsd") is not None
                else None
            )
            required_gsd_cm = (
                round(float(quality_detail["requiredGsd"]) * 100.0, 3)
                if quality_detail and quality_detail.get("requiredGsd") is not None
                else None
            )
            current = bool(input_id in active_counts)
            execution_done = bool(item.get("isDone"))
            coverage_required = bool(
                shape == "AREA"
                and coverage_detail
                and coverage_detail.get("source") == "footprint"
                and (_as_float(coverage_detail.get("planned")) or 0.0) > 0.0
            )
            coverage_done = bool(
                coverage_detail.get("requirementsMet")
                if coverage_required and coverage_detail
                else True
            )
            done = bool(execution_done and coverage_done)
            if execution_done and coverage_required and not coverage_done:
                status_label = "촬영 미충족"
                status_tone = "bad"
            elif done:
                status_label = "완료"
                status_tone = "done"
            elif current:
                status_label = "수행 중"
                status_tone = "active"
            else:
                status_label = "대기"
                status_tone = "pending"
            input_name = INPUT_TYPE_NAMES.get(input_type, "미지정")
            region_name = REGION_TYPE_NAMES.get(region_type, "미지정")
            parts.append(
                {
                    "sequence": sequence,
                    "inputMissionID": input_id,
                    "inputMissionType": input_type,
                    "inputMissionTypeName": INPUT_TYPE_NAMES.get(input_type, f"임무 유형 {input_type}"),
                    "type": f"{input_type} - {input_name}",
                    "regionType": region_type,
                    "regionTypeName": REGION_TYPE_NAMES.get(region_type, f"지역 유형 {region_type}"),
                    "region": f"{region_type} - {region_name}",
                    "shape": shape,
                    "isDone": done,
                    "isExecutionDone": execution_done,
                    "isCoverageDone": coverage_done,
                    "coverageRequired": coverage_required,
                    "isCurrent": current,
                    "status": status_label,
                    "statusTone": status_tone,
                    "activeAircraftCount": active_counts.get(input_id, 0),
                    "coverage": coverage_detail.get("percent") if coverage_detail else None,
                    "coverageDetail": coverage_detail,
                    "gsd": measured_gsd_cm,
                    "requiredGSD": required_gsd_cm,
                    # The card displays the measured average GSD and its maximum
                    # allowed value, so its pass/fail state must use that same
                    # comparison.  requirementsMet is the stricter diagnostic
                    # saying every sampled/spatial cell passed; using it here can
                    # show "fail" even when the displayed 5.41 <= 8.38 cm/px.
                    "gsdSatisfied": _gsd_requirement_satisfied(
                        measured_gsd_cm,
                        required_gsd_cm,
                    ),
                    "quality": {
                        **(quality_detail or {}),
                        "gsd": measured_gsd_cm,
                        "targetGSD": required_gsd_cm,
                    },
                    "measuredGsd": measured_gsd_cm,
                    "targetGsd": required_gsd_cm,
                    "qualitySamples": (
                        _as_int((quality_detail or {}).get("samples")) or 0
                    ),
                    "qualitySatisfaction": (
                        (quality_detail or {}).get("satisfactionPercent")
                    ),
                }
            )
        return parts

    def state(self) -> dict[str, Any]:
        with self._lock:
            db_root_key = self._db_root_key()
            self._ensure_discoveries_loaded(db_root_key)
            self._ensure_commands_loaded(db_root_key)
            self._consume_0402_events()
            self._consume_0602_events()
            snapshot = build_monitoring_snapshot(
                self.integration,
                mission_since=self._upstream_mission_signature or None,
            )
            mission = snapshot.get("mission")
            upstream_signature = str(snapshot.get("missionSignature") or "")
            mission = self._apply_startup_mission_gate(
                mission if isinstance(mission, dict) else None,
                upstream_signature,
            )
            if upstream_signature:
                self._upstream_mission_signature = upstream_signature
            if isinstance(mission, dict):
                self._upstream_plan_source = str(mission.get("source") or self._upstream_plan_source)
            signature = upstream_signature or self._mission_signature
            restored_signature = None
            if not self._awaiting_new_mission:
                mission, restored_signature = self._restore_active_progress_mission(snapshot, mission)
            if restored_signature:
                signature = restored_signature
            if isinstance(mission, dict) and mission:
                self._install_mission(mission, signature or str(mission.get("signature") or ""))
            plan_id = _as_int(self._mission.get("missionPlanID"))
            raw_timestamp = _as_int(snapshot.get("timestamp"))
            raw_timestamp_unix = _to_unix_ms(raw_timestamp)
            telemetry_fresh = (
                raw_timestamp_unix is not None
                and raw_timestamp_unix >= self._service_started_unix_ms
            )
            timestamp = raw_timestamp if telemetry_fresh else None
            self._refresh_view(plan_id)
            current_missions, current_inputs = self._current_mission_maps()
            raw_0401 = self._raw_0401() if plan_id is not None and telemetry_fresh else None
            self._update_quality(raw_0401, plan_id)
            coverage_by_input, coverage_summary = self._coverage(plan_id)
            quality_by_input, quality_vehicles, quality_summary = self._quality_rows()
            quality_by_vehicle_input = {
                (_as_int(item.get("aircraftID")), _as_int(item.get("inputMissionID"))): item
                for item in quality_vehicles
            }

            age_ms, payload_age_ms, arrival_age_ms = _signal_age_details(
                self.integration,
                timestamp,
            )
            direct_enabled = bool(getattr(self.integration, "enabled", False))
            source = str(snapshot.get("source") or "none") if telemetry_fresh else "waiting"
            if source != self._last_snapshot_source:
                self._add_event("info", "SIGNAL", f"0401 데이터 소스: {source}", timestamp)
                self._last_snapshot_source = source

            vehicles = []
            visible_vehicles = (
                snapshot.get("vehicles") or {}
                if plan_id is not None and telemetry_fresh
                else {}
            )
            for label, source_vehicle in visible_vehicles.items():
                if not isinstance(source_vehicle, dict):
                    continue
                aircraft_id = _as_int(str(label).replace("LAH", "").replace("UAV", ""))
                if str(label).startswith("UAV") and aircraft_id is not None:
                    aircraft_id += 3
                current = current_missions.get(aircraft_id or -1, {})
                current_input = current_inputs.get(aircraft_id or -1)
                current_quality = quality_by_vehicle_input.get((aircraft_id, current_input), {})
                previous = self._last_current_by_aircraft.get(aircraft_id or -1)
                if aircraft_id is not None and current_input is not None and previous != current_input:
                    self._add_event("info", "MISSION", f"{label} 입력임무 {current_input} 진입", timestamp)
                if aircraft_id is not None:
                    self._last_current_by_aircraft[aircraft_id] = current_input
                vehicles.append(
                    {
                        "aircraftID": aircraft_id,
                        "label": label,
                        "lat": source_vehicle.get("lat"),
                        "lon": source_vehicle.get("lon"),
                        "alt": source_vehicle.get("alt"),
                        "speed": source_vehicle.get("speed"),
                        "heading": source_vehicle.get("heading"),
                        "roll": source_vehicle.get("roll"),
                        "pitch": source_vehicle.get("pitch"),
                        "yaw": source_vehicle.get("yaw"),
                        "health": source_vehicle.get("health"),
                        "fuel": (
                            round(
                                max(
                                    0.0,
                                    min(
                                        100.0,
                                        float(source_vehicle.get("fuel"))
                                        / self._fuel_capacity_liters
                                        * 100.0,
                                    ),
                                ),
                                1,
                            )
                            if _as_float(source_vehicle.get("fuel")) is not None
                            else None
                        ),
                        "fuelLiters": _round(source_vehicle.get("fuel"), 2),
                        "flying": (
                            True
                            if _as_int(source_vehicle.get("flying")) in (1, 2)
                            else False if _as_int(source_vehicle.get("flying")) == 0 else None
                        ),
                        "filming": (
                            True
                            if _as_int(source_vehicle.get("filming")) == 1
                            else False if _as_int(source_vehicle.get("filming")) == 2 else None
                        ),
                        "flightMode": source_vehicle.get("flightMode"),
                        "payloadHealth": source_vehicle.get("payloadHealth"),
                        "fuelWarning": source_vehicle.get("fuelWarning"),
                        "sensorFov": source_vehicle.get("filmingFov"),
                        "footprint": source_vehicle.get("footprintCorners") or [],
                        "currentWaypointID": (snapshot.get("currentWaypoints") or {}).get(label),
                        "currentInputMissionID": current_input,
                        "currentInputID": current_input,
                        "currentIndividualMissionID": _as_int(current.get("individual_mission_id")),
                        "currentMissionID": _as_int(current.get("individual_mission_id")),
                        "coverage": (coverage_by_input.get(current_input or -1) or {}).get("percent"),
                        "quality": {
                            "gsd": (
                                round(float(current_quality["latestGsd"]) * 100.0, 3)
                                if current_quality.get("latestGsd") is not None
                                else None
                            ),
                            "satisfaction": current_quality.get("satisfactionPercent"),
                        },
                        "status": "LIVE" if age_ms is not None and age_ms <= 2500 else "STALE",
                    }
                )
            parts = self._mission_parts(coverage_by_input, quality_by_input, current_inputs)
            parts, history_parts = self._mission_part_history(parts)
            self._last_coverage_by_input = deepcopy(coverage_by_input)
            self._last_parts_by_input = {
                input_id: deepcopy(item)
                for item in parts
                if (input_id := _as_int(item.get("inputMissionID"))) is not None
            }
            done_count = sum(1 for item in parts if item["isDone"])
            rate = None
            try:
                rate = self.integration.receive_rate_hz() if self.integration is not None else None
            except Exception:
                pass
            option_assignments = self._option_assignment_state()
            return {
                "ok": True,
                "generatedAt": int(time.time() * 1000.0),
                "scenario": {
                    "name": Path(db_paths.get_active_db_root()).parent.name,
                    "dbRoot": str(db_paths.get_active_db_root()),
                    "missionPlanID": plan_id,
                    "inputMissionPackageID": _as_int(self._mission.get("inputMissionPackageID")),
                    "inputMissionPackageType": _as_int(
                        ((self._mission.get("inputMissionPlans") or [{}])[0]).get("inputMissionPackageType")
                    ),
                },
                "signal": {
                    "source": source,
                    "directReceiver": direct_enabled,
                    "receiverError": getattr(self.integration, "error", None),
                    "timestamp": timestamp,
                    "timestampUnix": _to_unix_ms(timestamp),
                    "ageMs": age_ms,
                    "payloadAgeMs": payload_age_ms,
                    "arrivalAgeMs": arrival_age_ms,
                    "rateHz": _round(rate, 2),
                    "status": "LIVE" if age_ms is not None and age_ms <= 2500 else "STALE" if timestamp else "WAIT",
                },
                "summary": {
                    "vehicleCount": len(vehicles),
                    "activeVehicleCount": sum(1 for item in vehicles if item["status"] == "LIVE"),
                    "activeVehicles": sum(1 for item in vehicles if item.get("flying") is True),
                    "filmingVehicles": sum(1 for item in vehicles if item.get("filming") is True),
                    "healthyVehicles": sum(1 for item in vehicles if _as_int(item.get("health")) == 1),
                    "missionCount": len(parts),
                    "doneMissionCount": done_count,
                    "coveragePercent": coverage_summary["overallPercent"],
                    "coverage": coverage_summary["overallPercent"],
                    "qualityPercent": quality_summary["satisfactionPercent"],
                    "planID": plan_id,
                    "currentInputMissionIDs": sorted(set(current_inputs.values())),
                },
                "vehicles": vehicles,
                "missionParts": parts,
                "missionPartHistory": history_parts,
                "coverage": {**coverage_summary, "byInput": coverage_by_input},
                "quality": {**quality_summary, "byInput": quality_by_input, "vehicles": quality_vehicles},
                "optionAssignments": option_assignments,
                "targets": self._targets(snapshot) if plan_id is not None else [],
                "discoveries": list(self._discoveries),
                "uavCommands": list(self._uav_commands),
                "events": list(self._events),
            }

    def mission(self, since: str | None = None) -> dict[str, Any]:
        with self._lock:
            if not self._mission_signature:
                self.state()
            plan_id = _as_int(self._mission.get("missionPlanID"))
            remaining = self._remaining_coverage_geometry(plan_id)
            response_signature = f"{self._mission_signature}:{remaining.get('revision') or '-'}"
            changed = not since or str(since) != response_signature
            response = {
                "ok": True,
                "changed": changed,
                "signature": response_signature,
                "planID": plan_id,
                "bounds": self._mission_bounds,
                "coverageDepthSummaries": remaining.get("coverageDepthSummaries") or [],
                "coveragePassSummaries": remaining.get("coveragePassSummaries") or [],
                "coveragePassRequirementMode": "all_passes_required",
            }
            if changed:
                geometry = self._geometry_with_history()
                geometry["remainingAreas"] = (
                    remaining.get("remainingAreas") or _feature_collection([])
                )
                geometry["coverageDepth"] = remaining.get("coverageDepth") or _feature_collection([])
                geometry["coveragePassAttribution"] = (
                    remaining.get("coveragePassAttribution") or _feature_collection([])
                )
                response["geojson"] = geometry
            return response
