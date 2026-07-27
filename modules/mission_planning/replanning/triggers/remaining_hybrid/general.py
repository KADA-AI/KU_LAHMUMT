from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from shapely.geometry import MultiPolygon, Polygon
from shapely.ops import unary_union

from modules.common import agent_status_snapshot, mission_area_replan_store
from modules.mission_planning.pipelines.line_scan_remaining_adapter import (
    apply_line_scan_remaining_to_input_mission,
)
from modules.mission_planning.pipelines.next_collab_path_builder import (
    build_flight_path_from_planned_row,
    build_mission_info_from_planned_row,
)
from modules.mission_planning.planners.next_collab_division._geo_utils import (
    coords_to_xy,
    meters_to_coord,
)
from modules.mission_planning.runtime.next_collab_division_runner import (
    run_next_collab_division_plan,
)
from modules.mission_planning.runtime.next_collab_line_runner import (
    run_next_collab_line_plan,
)
from modules.mission_planning.replanning.line_entry_context import (
    build_line_entry_context_row,
)
from modules.mission_planning.runtime.cache.source_artifacts import read_json_cached


@dataclass
class RemainingHybridResult:
    applied: bool
    mode: str | None = None
    input_mission_id: int | None = None
    aircraft_ids: List[int] | None = None
    planner_workflow: str | None = None
    reason: str | None = None
    validation: Dict[str, Any] | None = None


@dataclass
class _AgentEntry:
    aircraft_id: int
    coordinate: Dict[str, Any]
    heading_deg: float | None
    context: Dict[str, Any] | None = None


_AREA_COVERAGE_PASS_CONTRACT_KEYS = (
    "areaCoveragePassContractVersion",
    "coveragePassPolicy",
    "coveragePassOrder",
    "coveragePassDetails",
    "coveragePassObligations",
    "remainingCoveragePasses",
    "completedCoveragePasses",
    "currentCoveragePass",
    "activeCoveragePass",
    "areaCoveragePhase",
)
_AREA_COVERAGE_DEPTH_CONTRACT_KEYS = (
    "areaCoverageDepthContractVersion",
    "coverageDepthPolicy",
    "requiredCoverageDepth",
    "coverageDepthDetails",
    "coverageDepthObligations",
    "remainingCoverageDepth",
    "completedCoverageDepth",
    "coverageDepthSatisfied",
    "coverageDepthUnresolvedGeometryCount",
    "coverageObservationDetails",
    "activeCoverageAcquisitionIDs",
    "coverageAcquisitionNamespace",
)


def _to_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _area_coverage_pass_contract_from_input_mission(
    input_mission: Dict[str, Any] | None,
) -> Dict[str, Any]:
    """Normalize a reciprocal-Area contract stored on root and/or detail."""

    mission = input_mission if isinstance(input_mission, dict) else {}
    detail = mission.get("missionDetail")
    source = dict(mission)
    if isinstance(detail, dict):
        source.update(detail)
    normalized = mission_area_replan_store.coverage_pass_replan_contract(source)
    depth_contract = mission_area_replan_store.coverage_depth_replan_contract(source)
    has_explicit_portable_contract = bool(
        source.get("areaCoveragePassContractVersion") is not None
        or isinstance(source.get("remainingCoveragePasses"), list)
        or isinstance(source.get("coveragePassObligations"), list)
    )
    if not has_explicit_portable_contract and not depth_contract:
        return normalized
    contract = {
        key: copy.deepcopy(source[key])
        for key in _AREA_COVERAGE_PASS_CONTRACT_KEYS
        if key in source
    }
    if depth_contract:
        if isinstance(source.get("coveragePassDetails"), list):
            contract["coveragePassAttributionDetails"] = copy.deepcopy(
                source.get("coveragePassDetails") or []
            )
        for key, value in normalized.items():
            contract[key] = copy.deepcopy(value)
    else:
        for key, value in normalized.items():
            if key != "coveragePassDetails":
                contract.setdefault(key, copy.deepcopy(value))
    for key in _AREA_COVERAGE_DEPTH_CONTRACT_KEYS:
        if key in depth_contract:
            contract[key] = copy.deepcopy(depth_contract[key])
    scope_input_id = _to_int(
        source.get("areaTakeoverSourceInputMissionID")
        or source.get("sourceInputMissionID")
        or source.get("inputMissionID")
    )
    if scope_input_id is not None and scope_input_id > 0:
        contract.setdefault(
            "coverageAcquisitionNamespace",
            f"inputMission:{int(scope_input_id)}",
        )
    return contract


def _apply_area_coverage_pass_contract(
    target: Dict[str, Any],
    contract: Dict[str, Any] | None,
) -> None:
    if not isinstance(target, dict) or not isinstance(contract, dict) or not contract:
        return
    for key, value in contract.items():
        target[key] = copy.deepcopy(value)


def _normalize_coordinate(payload: object | None) -> Dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    if lat is None or lon is None:
        return None
    out: Dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    alt = _to_float(payload.get("altitude"))
    if alt is not None:
        out["altitude"] = float(alt)
    return out


def _normalize_coord_list(payload: object | None) -> List[Dict[str, float]]:
    coords = payload if isinstance(payload, list) else []
    out: List[Dict[str, float]] = []
    for item in coords:
        coord = _normalize_coordinate(item)
        if coord is not None:
            out.append(coord)
    return out


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        payload = read_json_cached(path, kind=path.parent.name or "json")
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _has_nonempty_geometry(detail: Dict[str, Any]) -> bool:
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    if line_list:
        return True
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    if area_list:
        return True
    coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []
    return len(coord_list) >= 2


def _is_line_input_mission(mission: Dict[str, Any]) -> bool:
    try:
        mission_type = int(mission.get("inputMissionType"))
    except Exception:
        mission_type = None
    if mission_type in (1, 7):
        return True
    if mission_type in (2, 3, 4, 5, 6):
        return False
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    return bool(detail.get("lineList"))


def _is_area_input_mission(mission: Dict[str, Any]) -> bool:
    try:
        mission_type = int(mission.get("inputMissionType"))
    except Exception:
        mission_type = None
    if mission_type == 2:
        return True
    if mission_type in (1, 7):
        return False
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    return bool(area_list and not line_list)


def _mission_input_id(mission: Dict[str, Any]) -> int | None:
    related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    return _to_int(related.get("inputMissionID"))


def _current_remaining_input_mission(cmpk_payload: Dict[str, Any]) -> Dict[str, Any] | None:
    missions = cmpk_payload.get("inputMissionList") if isinstance(cmpk_payload.get("inputMissionList"), list) else []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        if bool(mission.get("isDone")):
            continue
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        if not _has_nonempty_geometry(detail):
            continue
        return mission
    return None


def _representative_altitude(detail: Dict[str, Any]) -> float:
    for key in ("lineList", "areaList"):
        rows = detail.get(key) if isinstance(detail.get(key), list) else []
        for row in rows:
            if not isinstance(row, dict):
                continue
            for coord in row.get("coordinateList") or []:
                alt = _to_float(coord.get("altitude")) if isinstance(coord, dict) else None
                if alt is not None:
                    return float(alt)
    for coord in detail.get("coordinateList") or []:
        alt = _to_float(coord.get("altitude")) if isinstance(coord, dict) else None
        if alt is not None:
            return float(alt)
    return 0.0


def _build_single_area_polygon(detail: Dict[str, Any]) -> List[Dict[str, float]]:
    raw_area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    area_blocks = [
        block
        for block in raw_area_list
        if isinstance(block, dict) and not bool(block.get("isHole"))
    ]
    hole_blocks = [
        block
        for block in raw_area_list
        if isinstance(block, dict) and bool(block.get("isHole"))
    ]
    if len(area_blocks) == 1:
        coords = _normalize_coord_list(area_blocks[0].get("coordinateList"))
        if len(coords) >= 3:
            return coords
        return []

    polygons_xy: List[Polygon] = []
    for block in area_blocks:
        coords = _normalize_coord_list(block.get("coordinateList"))
        points_xy = coords_to_xy(coords)
        if len(points_xy) < 3:
            continue
        try:
            poly = Polygon(points_xy)
        except Exception:
            continue
        if not poly.is_valid:
            try:
                poly = poly.buffer(0)
            except Exception:
                continue
        if poly.is_empty:
            continue
        if isinstance(poly, MultiPolygon):
            try:
                poly = max(
                    (geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty),
                    key=lambda geom: float(geom.area or 0.0),
                )
            except Exception:
                continue
        if isinstance(poly, Polygon) and not poly.is_empty:
            polygons_xy.append(poly)

    if polygons_xy:
        try:
            merged = unary_union(polygons_xy)
        except Exception:
            merged = polygons_xy[0]
        if isinstance(merged, MultiPolygon):
            try:
                merged = merged.convex_hull
            except Exception:
                merged = max(
                    (geom for geom in merged.geoms if isinstance(geom, Polygon) and not geom.is_empty),
                    key=lambda geom: float(geom.area or 0.0),
                )
        if isinstance(merged, Polygon) and not merged.is_empty:
            exterior = [(float(x), float(y)) for (x, y) in list(merged.exterior.coords)[:-1]]
            if len(exterior) >= 3:
                tolerance = 0.0
                if len(exterior) > 120:
                    min_x = min(point[0] for point in exterior)
                    max_x = max(point[0] for point in exterior)
                    min_y = min(point[1] for point in exterior)
                    max_y = max(point[1] for point in exterior)
                    tolerance = max(max(max_x - min_x, max_y - min_y) * 0.01, 10.0)
                    tolerance = min(tolerance, 120.0)
                if tolerance > 0.0:
                    try:
                        simplified = merged.simplify(tolerance, preserve_topology=True)
                        if isinstance(simplified, Polygon) and not simplified.is_empty:
                            exterior = [
                                (float(x), float(y))
                                for (x, y) in list(simplified.exterior.coords)[:-1]
                            ]
                    except Exception:
                        pass
                if len(exterior) >= 3:
                    alt = _representative_altitude(detail)
                    return [meters_to_coord(x, y, alt_m=alt) for (x, y) in exterior]

    coord_list = _normalize_coord_list(detail.get("coordinateList"))
    if len(coord_list) >= 3:
        return coord_list
    return []


def _summarize_agent_entries() -> Dict[int, _AgentEntry]:
    snapshot_payload = agent_status_snapshot.load_agent_status_snapshot() or {}
    states = snapshot_payload.get("agent_states") or snapshot_payload.get("raw", {}).get("agentStateList") or []
    out: Dict[int, _AgentEntry] = {}
    for entry in states:
        if not isinstance(entry, dict):
            continue
        if not bool(entry.get("isUnmanned")):
            continue
        aircraft_id = _to_int(entry.get("aircraftID"))
        if aircraft_id is None or aircraft_id <= 0:
            continue
        coord = entry.get("coordinate") or {}
        if not coord:
            unmanned_info = entry.get("unmannedInfo") or {}
            coord = unmanned_info.get("coordinate") or {}
        normalized_coord = _normalize_coordinate(coord)
        if normalized_coord is None:
            continue
        heading = None
        velocity = entry.get("velocity") if isinstance(entry.get("velocity"), dict) else {}
        if velocity:
            heading = _to_float(velocity.get("heading"))
        if heading is None:
            heading = _to_float(entry.get("heading"))
        context_row = build_line_entry_context_row(
            aircraft_id=int(aircraft_id),
            state=entry,
            entry_coordinate=normalized_coord,
            heading_deg=heading,
            current_coordinate=normalized_coord,
        )
        out[int(aircraft_id)] = _AgentEntry(
            aircraft_id=int(aircraft_id),
            coordinate=normalized_coord,
            heading_deg=heading,
            context=context_row,
        )
    return out


def _current_template_records(
    missions: List[Dict[str, Any]],
    flight_plans_0303: List[Dict[str, Any]],
    *,
    current_input_id: int,
) -> Dict[int, Dict[str, Any]] | None:
    fp_by_path = {
        int(path_id): fp
        for fp in flight_plans_0303
        if isinstance(fp, dict) and (_to_int(fp.get("pathID")) or 0) > 0
        for path_id in [_to_int(fp.get("pathID"))]
    }
    rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    for index, mission in enumerate(missions):
        if not isinstance(mission, dict):
            continue
        aircraft_id = _to_int(mission.get("aircraftID")) or 0
        if aircraft_id not in (4, 5, 6):
            continue
        if _mission_input_id(mission) != int(current_input_id):
            continue
        path_id = _to_int(mission.get("pathID")) or 0
        rows_by_aircraft.setdefault(int(aircraft_id), []).append(
            {
                "index": int(index),
                "mission": mission,
                "flight_path": copy.deepcopy(fp_by_path.get(int(path_id)) or {}),
            }
        )
    if not rows_by_aircraft:
        return None
    if any(len(items) != 1 for items in rows_by_aircraft.values()):
        return None
    return {int(aid): items[0] for aid, items in rows_by_aircraft.items()}


def validate_remaining_hybrid_source_geometry(
    cmpk_payload: Dict[str, Any],
    *,
    current_input_id: int,
    template_records: Dict[int, Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    source_mission = _find_input_mission(cmpk_payload, int(current_input_id))
    source_detail = (
        source_mission.get("missionDetail")
        if isinstance(source_mission, dict) and isinstance(source_mission.get("missionDetail"), dict)
        else {}
    )
    source_has_geometry = _has_nonempty_geometry(source_detail)
    template_input_ids: List[int] = []
    if isinstance(template_records, dict):
        for record in template_records.values():
            if not isinstance(record, dict):
                continue
            mission = record.get("mission")
            if not isinstance(mission, dict):
                continue
            input_id = _mission_input_id(mission)
            if input_id is not None:
                template_input_ids.append(int(input_id))
    template_matches = (
        not template_input_ids
        or all(int(input_id) == int(current_input_id) for input_id in template_input_ids)
    )
    return {
        "valid": bool(isinstance(source_mission, dict) and source_has_geometry and template_matches),
        "sourceInputMissionID": int(current_input_id),
        "sourceMissionFound": isinstance(source_mission, dict),
        "sourceGeometryPresent": bool(source_has_geometry),
        "sourceGeometryType": "line" if _is_line_input_mission(source_mission or {}) else "area",
        "templateInputMissionIDs": sorted(set(template_input_ids)),
        "templateMatchesSourceInput": bool(template_matches),
    }


def _find_input_mission(cmpk_payload: Dict[str, Any], input_mission_id: int) -> Dict[str, Any] | None:
    missions = cmpk_payload.get("inputMissionList") if isinstance(cmpk_payload.get("inputMissionList"), list) else []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        input_id = _to_int(mission.get("inputMissionID"))
        if input_id is not None and int(input_id) == int(input_mission_id):
            return mission
    return None


def _area_snapshot_remaining_detail(detail: object) -> Dict[str, Any] | None:
    if not isinstance(detail, dict):
        return None
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    area_segment_list = detail.get("areaSegmentList") if isinstance(detail.get("areaSegmentList"), list) else []
    coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []
    if not area_list and not area_segment_list and len(coord_list) < 3:
        return None
    out = copy.deepcopy(detail)
    out["lineList"] = []
    if not isinstance(out.get("areaList"), list):
        out["areaList"] = []
    if not isinstance(out.get("coordinateList"), list):
        out["coordinateList"] = []
    if out.get("areaList") or len(out.get("coordinateList") or []) >= 3:
        out.pop("areaSegmentList", None)
        out.pop("areaSegmentPolicy", None)
    return out


def _apply_area_snapshot_remaining_to_input_mission(
    input_mission: Dict[str, Any],
    *,
    current_input_id: int,
    replan_detail: Dict[str, Any],
    log: Callable[[str], None] | None = None,
) -> tuple[Dict[str, Any], Dict[str, Any] | None]:
    if not _is_area_input_mission(input_mission):
        return input_mission, None

    source_plan_id = _to_int(
        replan_detail.get("sourceMissionPlanID")
        or replan_detail.get("currentMissionPlanID")
    )
    source_input_id = _to_int(
        replan_detail.get("reexecuteSourceInputMissionID")
        or replan_detail.get("sourceInputMissionID")
        or current_input_id
    )
    if source_plan_id is None or source_plan_id <= 0 or source_input_id is None or source_input_id <= 0:
        return input_mission, None

    snapshot_info = mission_area_replan_store.load_replan_ready_snapshot_entry(
        int(source_plan_id),
        int(source_input_id),
        allow_latest=True,
        allow_latest_area=True,
        audit_context="current_remaining_hybrid_area_snapshot",
    )
    entry = snapshot_info.get("entry") if isinstance(snapshot_info, dict) else None
    if not isinstance(entry, dict):
        if int(source_input_id) != int(current_input_id):
            return input_mission, {
                "sourcePlanID": int(source_plan_id),
                "sourceInputMissionID": int(source_input_id),
                "cloneInputMissionID": int(current_input_id),
                "snapshotApplied": False,
                "reason": "snapshot_entry_unavailable",
            }
        return input_mission, None
    if str(entry.get("missionType") or "").strip().lower() != "area":
        if int(source_input_id) != int(current_input_id):
            return input_mission, {
                "sourcePlanID": int(source_plan_id),
                "sourceInputMissionID": int(source_input_id),
                "cloneInputMissionID": int(current_input_id),
                "snapshotApplied": False,
                "reason": "snapshot_entry_not_area",
            }
        return input_mission, None

    pending_detail = mission_area_replan_store.coverage_replan_pending_remaining_detail(entry)
    remaining_detail = _area_snapshot_remaining_detail(pending_detail)
    if not isinstance(remaining_detail, dict):
        if log is not None:
            log(
                "[REMAINING][AREA] snapshot remaining unavailable "
                f"(sourcePlan={int(source_plan_id)}, sourceInput={int(source_input_id)})."
            )
        return input_mission, {
            "sourcePlanID": int(source_plan_id),
            "sourceInputMissionID": int(source_input_id),
            "snapshotApplied": False,
            "reason": "remaining_geometry_unavailable",
        }

    mission = copy.deepcopy(input_mission)
    original_detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    merged_detail = dict(original_detail or {})
    depth_contract = mission_area_replan_store.coverage_depth_replan_contract(entry)
    if depth_contract:
        assignment_detail = mission_area_replan_store.area_assignment_detail(
            entry,
            fallback=original_detail,
        )
        if assignment_detail is not None:
            mission_area_replan_store.apply_area_assignment_geometry(
                merged_detail,
                assignment_detail,
            )
        merged_detail["areaCoverageWorkloadDetail"] = copy.deepcopy(
            remaining_detail
        )
    else:
        merged_detail.update(remaining_detail)
    contracts = mission_area_replan_store.apply_area_coverage_replan_contracts(
        merged_detail,
        entry,
    )
    mission["missionDetail"] = merged_detail
    if contracts.get("passes") or contracts.get("depth"):
        mission_area_replan_store.apply_area_coverage_replan_contracts(mission, entry)
    mission["isDone"] = False
    mission["areaTakeoverSource"] = "mission_area_replan_snapshot"
    mission["areaTakeoverScope"] = "full_remaining"
    mission["areaTakeoverSourceInputMissionID"] = int(source_input_id)
    mission["areaTakeoverCloneInputMissionID"] = int(current_input_id)
    mission["areaTakeoverSnapshotMissionPlanID"] = _to_int(snapshot_info.get("snapshotMissionPlanID"))
    mission["areaTakeoverSnapshotExact"] = bool(snapshot_info.get("exact"))
    if log is not None:
        area_rows = remaining_detail.get("areaList") if isinstance(remaining_detail.get("areaList"), list) else []
        segment_rows = (
            remaining_detail.get("areaSegmentList")
            if isinstance(remaining_detail.get("areaSegmentList"), list)
            else []
        )
        log(
            "[REMAINING][AREA] snapshot remaining injected "
            f"cloneInput={int(current_input_id)} sourceInput={int(source_input_id)} "
            f"snapshotPlan={mission.get('areaTakeoverSnapshotMissionPlanID')} "
            f"areaRows={len(area_rows)} segmentRows={len(segment_rows)} "
            f"remainingAreaM2={float(entry.get('remainingAreaM2') or 0.0):.1f}"
        )
    return mission, {
        "sourcePlanID": int(source_plan_id),
        "sourceInputMissionID": int(source_input_id),
        "cloneInputMissionID": int(current_input_id),
        "snapshotMissionPlanID": _to_int(snapshot_info.get("snapshotMissionPlanID")),
        "snapshotExact": bool(snapshot_info.get("exact")),
        "snapshotApplied": True,
        "remainingAreaM2": float(entry.get("remainingAreaM2") or 0.0),
        "currentCoveragePass": contracts["passes"].get("currentCoveragePass") if contracts.get("passes") else None,
        "remainingCoveragePasses": list(contracts["passes"].get("remainingCoveragePasses") or []) if contracts.get("passes") else [],
        "remainingCoverageDepth": contracts["depth"].get("remainingCoverageDepth") if contracts.get("depth") else None,
    }


def _apply_runtime_meta(path_row: Dict[str, Any], mission_entry: Dict[str, Any]) -> None:
    bearing_deg = _to_float(path_row.get("bearingDeg"))
    if bearing_deg is not None:
        mission_entry["bearing_deg"] = float(bearing_deg)


def _replace_current_missions(
    missions: List[Dict[str, Any]],
    *,
    replacements_by_aircraft: Dict[int, Dict[str, Any]],
) -> None:
    for aircraft_id, replacement in replacements_by_aircraft.items():
        index = _to_int(replacement.get("_replaceIndex"))
        if index is None or not (0 <= int(index) < len(missions)):
            continue
        replacement_copy = copy.deepcopy(replacement)
        replacement_copy.pop("_replaceIndex", None)
        missions[int(index)] = replacement_copy


def _replace_current_flight_paths(
    flight_plans_0303: List[Dict[str, Any]],
    *,
    replacements_by_path: Dict[int, Dict[str, Any]],
) -> None:
    for idx, flight_path in enumerate(flight_plans_0303):
        if not isinstance(flight_path, dict):
            continue
        path_id = _to_int(flight_path.get("pathID")) or 0
        if path_id <= 0:
            continue
        replacement = replacements_by_path.get(int(path_id))
        if replacement is None:
            continue
        flight_plans_0303[idx] = copy.deepcopy(replacement)


def apply_remaining_hybrid_replan(
    *,
    cmpk_source_path: Path,
    replan_detail: Dict[str, Any] | None,
    missions: List[Dict[str, Any]],
    flight_plans_0303: List[Dict[str, Any]],
    timestamp_ms: int,
    log: Callable[[str], None] | None = None,
) -> RemainingHybridResult:
    detail = replan_detail if isinstance(replan_detail, dict) else {}
    trigger = str(detail.get("trigger") or "").strip()
    if trigger not in {"0401", "0802"}:
        return RemainingHybridResult(applied=False, reason=f"trigger {trigger or '-'} not eligible")

    cmpk_payload = _load_json(Path(cmpk_source_path))
    current_input_mission = _current_remaining_input_mission(cmpk_payload)
    if not isinstance(current_input_mission, dict):
        return RemainingHybridResult(applied=False, reason="current remaining input mission unavailable")

    current_input_id = _to_int(current_input_mission.get("inputMissionID"))
    if current_input_id is None or current_input_id <= 0:
        return RemainingHybridResult(applied=False, reason="current inputMissionID unavailable")

    try:
        from modules.mission_planning.runtime.state import branch_ownership as branch_store

        locked_type2_branch = branch_store.is_locked_type2_branch_mission(
            cmpk_payload,
            int(current_input_id),
        )
    except Exception:
        locked_type2_branch = False
    if locked_type2_branch:
        if log is not None:
            log(
                "[REMAINING][TYPE2] generic pooled fallback blocked; immutable "
                "branch assignments remain active."
            )
        return RemainingHybridResult(
            applied=False,
            input_mission_id=int(current_input_id),
            reason="type2 immutable branch ownership preserved",
        )

    source_validation = validate_remaining_hybrid_source_geometry(
        cmpk_payload,
        current_input_id=int(current_input_id),
    )
    if not bool(source_validation.get("valid")):
        return RemainingHybridResult(
            applied=False,
            input_mission_id=int(current_input_id),
            reason="source geometry validation failed",
            validation=source_validation,
        )

    template_records = _current_template_records(
        missions,
        flight_plans_0303,
        current_input_id=int(current_input_id),
    )
    if not template_records:
        return RemainingHybridResult(
            applied=False,
            input_mission_id=int(current_input_id),
            reason="current template missions unavailable or ambiguous",
            validation=source_validation,
        )

    source_validation = validate_remaining_hybrid_source_geometry(
        cmpk_payload,
        current_input_id=int(current_input_id),
        template_records=template_records,
    )
    if not bool(source_validation.get("valid")):
        return RemainingHybridResult(
            applied=False,
            input_mission_id=int(current_input_id),
            reason="source/template geometry validation failed",
            validation=source_validation,
        )
    if log is not None:
        log(f"[REMAINING] source geometry validation: {source_validation}")

    target_aircraft_ids = sorted(int(aid) for aid in template_records.keys())
    agent_entries = _summarize_agent_entries()
    aircraft_entries: List[Dict[str, Any]] = []
    for aircraft_id in target_aircraft_ids:
        agent_entry = agent_entries.get(int(aircraft_id))
        if agent_entry is None:
            return RemainingHybridResult(
                applied=False,
                input_mission_id=int(current_input_id),
                aircraft_ids=target_aircraft_ids,
                reason=f"agent status missing for aircraft {aircraft_id}",
                validation=source_validation,
            )
        row: Dict[str, Any] = dict(agent_entry.context or {})
        row["aircraftID"] = int(aircraft_id)
        row["coordinate"] = dict(agent_entry.coordinate)
        if agent_entry.heading_deg is not None:
            row["headingDeg"] = float(agent_entry.heading_deg)
        aircraft_entries.append(row)
    aircraft_entry_by_id = {
        int(row["aircraftID"]): row
        for row in aircraft_entries
        if _to_int(row.get("aircraftID")) is not None
    }

    current_input_mission, area_snapshot_validation = _apply_area_snapshot_remaining_to_input_mission(
        current_input_mission,
        current_input_id=int(current_input_id),
        replan_detail=detail,
        log=log,
    )
    if isinstance(area_snapshot_validation, dict):
        source_validation = dict(source_validation)
        source_validation["areaSnapshotRemaining"] = area_snapshot_validation
        if not bool(area_snapshot_validation.get("snapshotApplied")):
            return RemainingHybridResult(
                applied=False,
                mode="area",
                input_mission_id=int(current_input_id),
                aircraft_ids=target_aircraft_ids,
                reason=f"area snapshot remaining unavailable: {area_snapshot_validation.get('reason') or 'unknown'}",
                validation=source_validation,
            )

    mission_detail = current_input_mission.get("missionDetail") if isinstance(current_input_mission.get("missionDetail"), dict) else {}
    turn_radius_scale = _to_float(detail.get("turnRadiusScale"))
    if turn_radius_scale is None or turn_radius_scale <= 0.0:
        turn_radius_scale = 1.2

    replacements_by_aircraft: Dict[int, Dict[str, Any]] = {}
    replacement_fp_by_path: Dict[int, Dict[str, Any]] = {}

    if _is_line_input_mission(current_input_mission):
        source_plan_id = _to_int(detail.get("sourceMissionPlanID") or detail.get("currentMissionPlanID"))
        line_target_mission, line_detail = apply_line_scan_remaining_to_input_mission(
            current_input_mission,
            source_plan_id=source_plan_id,
            input_mission_id=int(current_input_id),
        )
        if line_detail and log is not None:
            log(
                "[REMAINING][LINE] line_scan remaining injected "
                f"fragments={int(line_detail.get('lineRemainingFragmentCount') or 0)} "
                f"aircraft={line_detail.get('lineScanContributingAircraftIDs') or []}"
            )
        planner_result = run_next_collab_line_plan(
            target_mission=line_target_mission,
            aircraft_entries=aircraft_entries,
            turn_radius_scale=float(turn_radius_scale),
            log=log,
        )
        expected_rows = [
            dict(row)
            for row in planner_result.expected_paths
            if isinstance(row, dict) and (_to_int(row.get("aircraftID")) or 0) in target_aircraft_ids
        ]
        rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
        for row in expected_rows:
            aircraft_id = _to_int(row.get("aircraftID")) or 0
            rows_by_aircraft.setdefault(int(aircraft_id), []).append(row)
        if sorted(rows_by_aircraft.keys()) != target_aircraft_ids or any(len(rows) != 1 for rows in rows_by_aircraft.values()):
            return RemainingHybridResult(
                applied=False,
                mode="line",
                input_mission_id=int(current_input_id),
                aircraft_ids=target_aircraft_ids,
                planner_workflow=str(planner_result.workflow),
                reason="planned line rows do not match current aircraft templates",
                validation=source_validation,
            )
        for aircraft_id in target_aircraft_ids:
            template_record = template_records[int(aircraft_id)]
            template_mission = copy.deepcopy(template_record["mission"])
            template_path = copy.deepcopy(template_record["flight_path"])
            path_row = rows_by_aircraft[int(aircraft_id)][0]
            mission_info = build_mission_info_from_planned_row(
                path_row,
                template_info=copy.deepcopy(template_mission.get("individualMissionInfo") or {}),
                fallback_polygon_coords=[],
            )
            template_mission["individualMissionInfo"] = mission_info
            template_mission["isDone"] = False
            _apply_runtime_meta(path_row, template_mission)
            path_id = _to_int(template_mission.get("pathID")) or 0
            individual_mission_id = _to_int(template_mission.get("individualMissionID")) or 0
            if path_id <= 0 or individual_mission_id <= 0:
                return RemainingHybridResult(
                    applied=False,
                    mode="line",
                    input_mission_id=int(current_input_id),
                    aircraft_ids=target_aircraft_ids,
                    reason=f"template IDs unavailable for aircraft {aircraft_id}",
                    validation=source_validation,
                )
            replacements_by_aircraft[int(aircraft_id)] = {
                "_replaceIndex": int(template_record["index"]),
                **template_mission,
            }
            replacement_fp_by_path[int(path_id)] = build_flight_path_from_planned_row(
                path_row,
                template_path=template_path,
                mission_info=mission_info,
                individual_mission_id=int(individual_mission_id),
                path_id=int(path_id),
                aircraft_id=int(aircraft_id),
                entry_coord=copy.deepcopy(aircraft_entry_by_id[int(aircraft_id)]["coordinate"]),
                timestamp_ms=int(timestamp_ms),
                source="MMR",
            )
        _replace_current_missions(missions, replacements_by_aircraft=replacements_by_aircraft)
        _replace_current_flight_paths(flight_plans_0303, replacements_by_path=replacement_fp_by_path)
        return RemainingHybridResult(
            applied=True,
            mode="line",
            input_mission_id=int(current_input_id),
            aircraft_ids=target_aircraft_ids,
            planner_workflow=str(planner_result.workflow),
            reason="applied",
            validation=source_validation,
        )

    area_pass_contract = _area_coverage_pass_contract_from_input_mission(
        current_input_mission
    )
    if (
        area_pass_contract.get("areaCoverageDepthContractVersion") is not None
        and not bool(area_pass_contract.get("coverageDepthSatisfied"))
        and int(area_pass_contract.get("coverageDepthUnresolvedGeometryCount") or 0) > 0
    ):
        return RemainingHybridResult(
            applied=False,
            mode="area",
            input_mission_id=int(current_input_id),
            aircraft_ids=target_aircraft_ids,
            reason="area coverage-depth geometry unresolved",
            validation=source_validation,
        )
    if area_pass_contract and not list(
        area_pass_contract.get("remainingCoveragePasses") or []
    ):
        if log is not None:
            log(
                "[REMAINING][AREA] reciprocal coverage contract has no pending "
                "passes; hybrid replacement planning skipped."
            )
        return RemainingHybridResult(
            applied=False,
            mode="area",
            input_mission_id=int(current_input_id),
            aircraft_ids=target_aircraft_ids,
            reason="area reciprocal coverage already complete",
            validation=source_validation,
        )

    if area_pass_contract.get("areaCoverageDepthContractVersion") is not None:
        depth_source = dict(current_input_mission)
        depth_source.update(mission_detail)
        pending_depth_detail = mission_area_replan_store.coverage_depth_pending_remaining_detail(
            depth_source
        )
        assignment_detail = mission_area_replan_store.area_assignment_detail(
            current_input_mission,
            fallback=mission_detail,
        )
        if assignment_detail is not None:
            mission_area_replan_store.apply_area_assignment_geometry(
                mission_detail,
                assignment_detail,
            )
        if isinstance(pending_depth_detail, dict):
            mission_detail["areaCoverageWorkloadDetail"] = copy.deepcopy(
                pending_depth_detail
            )

    mission_polygon = _build_single_area_polygon(mission_detail)
    if len(mission_polygon) < 3:
        return RemainingHybridResult(
            applied=False,
            mode="area",
            input_mission_id=int(current_input_id),
            aircraft_ids=target_aircraft_ids,
            reason="merged area polygon unavailable",
            validation=source_validation,
        )

    planner_result = run_next_collab_division_plan(
        mission_polygon=mission_polygon,
        aircraft_entries=aircraft_entries,
        turn_radius_scale=float(turn_radius_scale),
        log=log,
    )
    expected_rows = [
        dict(row)
        for row in planner_result.expected_paths
        if isinstance(row, dict) and (_to_int(row.get("aircraftID")) or 0) in target_aircraft_ids
    ]
    for path_row in expected_rows:
        _apply_area_coverage_pass_contract(path_row, area_pass_contract)
    rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    for row in expected_rows:
        aircraft_id = _to_int(row.get("aircraftID")) or 0
        rows_by_aircraft.setdefault(int(aircraft_id), []).append(row)
    if sorted(rows_by_aircraft.keys()) != target_aircraft_ids or any(len(rows) != 1 for rows in rows_by_aircraft.values()):
        return RemainingHybridResult(
            applied=False,
            mode="area",
            input_mission_id=int(current_input_id),
            aircraft_ids=target_aircraft_ids,
            planner_workflow=str(planner_result.workflow),
            reason="planned area rows do not match current aircraft templates",
            validation=source_validation,
        )

    piece_polygon_map: Dict[Tuple[int, int], List[Dict[str, Any]]] = {}
    split_result = getattr(planner_result, "split_result", None)
    for piece in getattr(split_result, "pieces", []) or []:
        aircraft_id = _to_int(getattr(piece, "assigned_uav", None)) or 0
        piece_index = _to_int(getattr(piece, "piece_index", None)) or 0
        data = getattr(piece, "data", {}) if getattr(piece, "data", None) is not None else {}
        coords = _normalize_coord_list((data or {}).get("coordinateList") if isinstance(data, dict) else None)
        if aircraft_id > 0 and piece_index > 0 and coords:
            piece_polygon_map[(int(aircraft_id), int(piece_index))] = coords

    for aircraft_id in target_aircraft_ids:
        template_record = template_records[int(aircraft_id)]
        template_mission = copy.deepcopy(template_record["mission"])
        template_path = copy.deepcopy(template_record["flight_path"])
        path_row = rows_by_aircraft[int(aircraft_id)][0]
        piece_index = _to_int(path_row.get("pieceIndex")) or 0
        mission_info = build_mission_info_from_planned_row(
            path_row,
            template_info=copy.deepcopy(template_mission.get("individualMissionInfo") or {}),
            fallback_polygon_coords=piece_polygon_map.get((int(aircraft_id), int(piece_index))) or mission_polygon,
        )
        _apply_area_coverage_pass_contract(mission_info, area_pass_contract)
        template_mission["individualMissionInfo"] = mission_info
        template_mission["isDone"] = False
        _apply_area_coverage_pass_contract(template_mission, area_pass_contract)
        _apply_runtime_meta(path_row, template_mission)
        path_id = _to_int(template_mission.get("pathID")) or 0
        individual_mission_id = _to_int(template_mission.get("individualMissionID")) or 0
        if path_id <= 0 or individual_mission_id <= 0:
            return RemainingHybridResult(
                applied=False,
                mode="area",
                input_mission_id=int(current_input_id),
                aircraft_ids=target_aircraft_ids,
                reason=f"template IDs unavailable for aircraft {aircraft_id}",
                validation=source_validation,
            )
        replacements_by_aircraft[int(aircraft_id)] = {
            "_replaceIndex": int(template_record["index"]),
            **template_mission,
        }
        replacement_fp_by_path[int(path_id)] = build_flight_path_from_planned_row(
            path_row,
            template_path=template_path,
            mission_info=mission_info,
            individual_mission_id=int(individual_mission_id),
            path_id=int(path_id),
            aircraft_id=int(aircraft_id),
            entry_coord=copy.deepcopy(aircraft_entry_by_id[int(aircraft_id)]["coordinate"]),
            timestamp_ms=int(timestamp_ms),
            source="MMR",
        )

    _replace_current_missions(missions, replacements_by_aircraft=replacements_by_aircraft)
    _replace_current_flight_paths(flight_plans_0303, replacements_by_path=replacement_fp_by_path)
    return RemainingHybridResult(
        applied=True,
        mode="area",
        input_mission_id=int(current_input_id),
        aircraft_ids=target_aircraft_ids,
        planner_workflow=str(planner_result.workflow),
        reason="applied",
        validation=source_validation,
    )
