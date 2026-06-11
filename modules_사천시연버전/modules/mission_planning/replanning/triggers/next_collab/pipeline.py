from __future__ import annotations

import concurrent.futures
import json
import math
import sys
import threading
import time
from copy import deepcopy
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set

from modules.common import db_paths
from modules.mission_planning.runtime.debug_artifacts import debug_artifact_mode, write_debug_json
from modules.mission_planning.runtime.json_io import write_json_batch
from modules.mission_planning.runtime.ids.replan_reservation import ReplanIdReservation
from modules.mission_planning.runtime.cache.source_artifacts import read_json_cached
from modules.mission_planning.engine.mission_generation.id_allocation import allocator as id_allocator
from modules.mission_planning.runtime.logging.pipeline_events import (
    PipelinePhaseTimer,
    new_replan_transaction_id,
)
from modules.mission_planning.runtime.validation.replan_payloads import validate_replan_payloads
from modules.mission_planning.runtime import next_collab_replan_store
from modules.mission_planning.runtime.next_collab_division_runner import (
    run_next_collab_division_plan,
)
from modules.mission_planning.runtime.next_collab_line_runner import (
    run_next_collab_line_plan,
)
from modules.mission_planning.runtime.next_collab_replan_runtime import (
    OPTION_NAME as NEXT_COLLAB_OPTION_NAME,
    TRIGGER_TYPE as NEXT_COLLAB_TRIGGER_TYPE,
)
from modules.mission_planning._paths import mission_planner_root, mission_planning_root, project_root
from modules.mission_planning.pipelines.next_collab_path_builder import (
    _coord_with_dem_altitude,
    build_formation_flight_path_from_template,
    build_flight_path_from_planned_row,
    build_mission_info_from_planned_row,
    prewarm_dem_altitudes_for_path_rows,
)
from modules.mission_planning.pipelines.mission_path_trim import reassign_unique_waypoint_ids_inplace
from modules.mission_planning.pipelines.lah_operational_mode import lah_special_info_for_input
from modules.mission_planning.replanning.line_entry_context import (
    build_line_entry_context_map_from_entry_rows,
)
from modules.mission_planning.replanning.triggers.prior.pipeline import (
    _now_ms_since_2000,
    _normalize_altitude_value,
    _to_float,
    _to_int,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.algo import split_algorithms as split_algorithms_module
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.area_review import review_assigned_areas_local
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
    _assign_group_by_takeover_distance,
    run_split_pipeline,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0302 import (
    build_0302_packages_from_split_with_lah,
    _piece_runtime_meta,
    _piece_to_mission_info,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0303_0304 import (
    _apply_runtime_params,
    build_0303_0304_from_0302_packages,
    _import_runtime_modules,
)
from modules.mission_planning.MissionPlanner.runtime_settings import (
    pop_runtime_camera_fov_adjustment_logs,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.models import (
    DirectionDebug,
    SplitPiece,
    SplitRunResult,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.pathing.expected_path import generate_expected_paths
from modules.mission_planning.MissionPlanner.planning_enhanced.pathing.expected_velocity import calculate_expected_velocity
from modules.mission_planning.MissionPlanner.planning_enhanced.type_decider.logic import apply_logic_type_decider
from shapely.geometry import GeometryCollection, LineString, MultiPolygon, Polygon
from shapely.ops import nearest_points, triangulate, unary_union

from modules.mission_planning.planners.next_collab_division._geo_utils import coord_to_xy, meters_to_coord
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        get_runtime_bool,
        get_runtime_area_review_max_segment_m,
        get_runtime_float,
        get_runtime_manual_fov_deg,
        load_runtime_settings,
    )
except Exception:
    from MissionPlanner.runtime_settings import (  # type: ignore
        get_runtime_bool,
        get_runtime_area_review_max_segment_m,
        get_runtime_float,
        get_runtime_manual_fov_deg,
        load_runtime_settings,
    )


DEFAULT_OPTION_NAME = "비행/촬영"
TRIGGER_TYPE = "nextCollaborativeMission"
REPLAN_FLOW_MODE = "next_collab_local_assigned"
ENTRY_FOV_DEG = 10.0
FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M = 30.0
DEFAULT_AREA_REVIEW_MAX_SEGMENT_M = 3000.0

DEFAULT_OPTION_NAME = NEXT_COLLAB_OPTION_NAME
TRIGGER_TYPE = NEXT_COLLAB_TRIGGER_TYPE
AREA_PLANNER_COMPONENT_MAX_COUNT = 64


def _next_collab_replacement_path_build_workers(item_count: int, *, scope: str = "") -> int:
    if int(item_count) < 2:
        return 1
    scope_name = str(scope or "").strip().lower()
    default_workers = 2 if scope_name == "line" else 1
    try:
        runtime_values = load_runtime_settings().get("values") or {}
        scope_key = (
            f"next_collab_{scope_name}_replacement_path_build_workers"
            if scope_name
            else ""
        )
        raw_value = (
            runtime_values.get(scope_key)
            if scope_key and scope_key in runtime_values
            else runtime_values.get(
                "next_collab_replacement_path_build_workers",
                default_workers,
            )
        )
    except Exception:
        raw_value = default_workers
    try:
        requested = int(float(raw_value))
    except Exception:
        requested = default_workers
    return max(1, min(int(item_count), int(requested)))


def _next_collab_replacement_waypoint_block_size() -> int:
    try:
        runtime_values = load_runtime_settings().get("values") or {}
        raw_value = runtime_values.get("next_collab_replacement_waypoint_block_size", 512)
    except Exception:
        raw_value = 512
    try:
        requested = int(float(raw_value))
    except Exception:
        requested = 512
    return max(64, int(requested))


class _ReplacementWaypointIdProvider:
    def __init__(self, *, scope: str) -> None:
        self._scope = str(scope)
        self._lock = threading.Lock()
        self._next_id = 0
        self._end_id = -1
        self._used = 0
        self._blocks: List[tuple[int, int]] = []

    def __call__(self) -> int:
        with self._lock:
            if self._next_id > self._end_id:
                block_size = _next_collab_replacement_waypoint_block_size()
                start = int(id_allocator.reserve_waypoint_block(int(block_size)))
                self._next_id = int(start)
                self._end_id = int(start) + int(block_size) - 1
                self._blocks.append((int(self._next_id), int(self._end_id)))
            value = int(self._next_id)
            self._next_id += 1
            self._used += 1
            return value

    def summary(self) -> Dict[str, Any]:
        return {
            "scope": self._scope,
            "used": int(self._used),
            "blocks": len(self._blocks),
            "reserved": sum((int(end) - int(start) + 1) for start, end in self._blocks),
            "blockRanges": [f"{int(start)}-{int(end)}" for start, end in self._blocks],
        }


def _build_replacement_flight_paths(
    build_items: List[tuple[int, Callable[[], Dict[str, Any]]]],
    *,
    emit: Callable[[str], None],
    scope: str,
    min_workers: int = 1,
) -> Dict[int, Dict[str, Any]]:
    if not build_items:
        return {}
    workers = max(
        int(min_workers),
        _next_collab_replacement_path_build_workers(len(build_items), scope=scope),
    )
    workers = max(1, min(len(build_items), int(workers)))

    def _build_one(item: tuple[int, Callable[[], Dict[str, Any]]]) -> tuple[int, Dict[str, Any]]:
        path_id, build_fn = item
        return int(path_id), build_fn()

    started = time.perf_counter()
    if workers <= 1:
        results = {int(path_id): build_fn() for path_id, build_fn in build_items}
    else:
        try:
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=int(workers),
                thread_name_prefix="NextCollabFP",
            ) as executor:
                results = {
                    int(path_id): payload
                    for path_id, payload in executor.map(_build_one, build_items)
                }
        except Exception as exc:
            emit(
                f"[NEXTCOLLAB][{scope}] parallel FlightPath build failed; "
                f"fallback sequential: {exc}"
            )
            results = {int(path_id): build_fn() for path_id, build_fn in build_items}
            workers = 1
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    emit(
        f"[NEXTCOLLAB][{scope}] replacement FlightPath build "
        f"items={len(build_items)} workers={int(workers)} elapsed={elapsed_ms:.1f} ms"
    )
    return results


def _assign_replacement_waypoint_ids_in_order(
    *,
    generated_fp_by_path: Dict[int, Dict[str, Any]],
    ordered_path_ids: List[int],
    waypoint_id_provider: Callable[[], int],
    emit: Callable[[str], None],
    scope: str,
) -> Dict[str, Any]:
    started = time.perf_counter()
    assigned_paths = 0
    assigned_waypoints = 0
    for path_id in ordered_path_ids:
        payload = generated_fp_by_path.get(int(path_id))
        if not isinstance(payload, dict):
            continue
        waypoints = payload.get("waypointList")
        if not isinstance(waypoints, list) or not waypoints:
            continue
        reassign_unique_waypoint_ids_inplace(
            waypoints,
            waypoint_id_provider=waypoint_id_provider,
        )
        payload["waypointList"] = waypoints
        if "lahWaypointList" in payload:
            payload["lahWaypointList"] = deepcopy(waypoints)
        assigned_paths += 1
        assigned_waypoints += len(waypoints)
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    emit(
        f"[NEXTCOLLAB][{scope}] waypoint IDs assigned after parallel build "
        f"paths={assigned_paths} waypoints={assigned_waypoints} elapsed={elapsed_ms:.1f} ms"
    )
    return {
        "paths": int(assigned_paths),
        "waypoints": int(assigned_waypoints),
        "elapsedMs": round(float(elapsed_ms), 3),
    }


def _emit_replacement_flight_path_metrics(
    metrics_rows: List[Dict[str, Any]],
    *,
    emit: Callable[[str], None],
    scope: str,
) -> None:
    if not metrics_rows:
        return
    rows = sorted(
        [dict(row) for row in metrics_rows if isinstance(row, dict)],
        key=lambda row: (
            int(row.get("aircraftID") or 0),
            int(row.get("pathID") or 0),
        ),
    )
    if not rows:
        return

    def _num(row: Dict[str, Any], key: str) -> float:
        try:
            return float(row.get(key) or 0.0)
        except Exception:
            return 0.0

    def _sum_float(key: str) -> float:
        return round(sum(_num(row, key) for row in rows), 3)

    def _sum_int(key: str) -> int:
        return int(round(sum(_num(row, key) for row in rows)))

    def _max_float(key: str) -> float:
        return round(max((_num(row, key) for row in rows), default=0.0), 3)

    slowest = max(rows, key=lambda row: _num(row, "buildTotalMs"))
    summary = {
        "items": len(rows),
        "nominalBuildTotalMs": _sum_float("buildTotalMs"),
        "maxBuildTotalMs": _max_float("buildTotalMs"),
        "maxPathID": int(slowest.get("pathID") or 0),
        "maxAircraftID": int(slowest.get("aircraftID") or 0),
        "scanLines": _sum_int("scanLines"),
        "scanLinePoints": _sum_int("scanLinePoints"),
        "areaSweepItems": _sum_int("areaSweepItems"),
        "areaGroups": _sum_int("areaGroups"),
        "areaMergedRows": _sum_int("areaMergedRows"),
        "areaMergedCoords": _sum_int("areaMergedCoords"),
        "waypoints": _sum_int("waypoints"),
        "lineSearchWaypoints": _sum_int("lineSearchWaypoints"),
        "lineSearchCoords": _sum_int("lineSearchCoords"),
        "scanLinesMs": _sum_float("scanLinesMs"),
        "lineSweepItemsMs": _sum_float("lineSweepItemsMs"),
        "areaSweepItemsMs": _sum_float("areaSweepItemsMs"),
        "areaGroupMs": _sum_float("areaGroupMs"),
        "areaCollectRowsMs": _sum_float("areaCollectRowsMs"),
        "areaDemMs": _sum_float("areaDemMs"),
        "areaAnchorAltitudeMs": _sum_float("areaAnchorAltitudeMs"),
        "areaSpeedMs": _sum_float("areaSpeedMs"),
        "areaWaypointAppendMs": _sum_float("areaWaypointAppendMs"),
        "postAltitudeMs": _sum_float("postAltitudeMs"),
        "filmingTargetNormalizeMs": _sum_float("filmingTargetNormalizeMs"),
        "waypointIdMs": _sum_float("waypointIdMs"),
        "timelineMs": _sum_float("timelineMs"),
        "runtimeFlyoverMs": _sum_float("runtimeFlyoverMs"),
        "lahCopyMs": _sum_float("lahCopyMs"),
    }
    emit(
        f"[NEXTCOLLAB][{scope}][FP_SUMMARY] "
        + json.dumps(summary, ensure_ascii=False, sort_keys=True)
    )
    for row in rows:
        emit(
            f"[NEXTCOLLAB][{scope}][FP_DETAIL] "
            + json.dumps(row, ensure_ascii=False, sort_keys=True)
        )


def _normalize_search_speed_scale_multiplier(value: Any) -> float:
    parsed = _to_float(value)
    if parsed is None or parsed <= 0.0:
        return 1.0
    return max(float(parsed), 0.1)


def _apply_search_speed_scale_multiplier_to_rows(
    rows: List[Dict[str, Any]],
    *,
    search_speed_scale_multiplier: Any,
    emit: Callable[[str], None],
    scope: str,
) -> float:
    multiplier = _normalize_search_speed_scale_multiplier(search_speed_scale_multiplier)
    if abs(float(multiplier) - 1.0) <= 1e-6:
        return 1.0
    applied = 0
    for row in rows:
        if not isinstance(row, dict):
            continue
        existing = _normalize_search_speed_scale_multiplier(
            row.get("searchSpeedScaleMultiplier")
            if row.get("searchSpeedScaleMultiplier") is not None
            else row.get("_searchSpeedScaleMultiplier")
        )
        row["searchSpeedScaleMultiplier"] = round(float(existing) * float(multiplier), 6)
        applied += 1
    if applied > 0:
        emit(
            f"[NEXTCOLLAB][{scope}] searchSpeed scale multiplier applied "
            f"factor={float(multiplier):.2f} rows={int(applied)}"
        )
    return float(multiplier)


@dataclass
class NextCollabPipelineResult:
    plan_ids: List[int]
    option_names: List[str]
    plan_meta_map: Dict[int, Dict[str, Any]]
    generated_imp_ids: Set[int]
    generated_path_ids: Set[int]
    new_input_package_id: int
    log_path: Path


@dataclass
class _PreparedReplacements:
    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]]
    generated_fp_by_path: Dict[int, Dict[str, Any]]
    generated_path_ids: Set[int]
    planner_workflow: str
    planner_result_text: str
    planned_result_count: int
    review_report: Dict[str, Any]
    mission_mode: str = ""
    timing_ms: Dict[str, float] = field(default_factory=dict)
    source_cache: Dict[str, Any] = field(default_factory=dict)
    id_reservation: Dict[str, Any] = field(default_factory=dict)
    uav_work_summary: Dict[int, int] = field(default_factory=dict)
    runtime_preservation: Dict[str, Any] = field(default_factory=dict)


class _NextCollabPrepareTimer:
    def __init__(self) -> None:
        self._last = time.perf_counter()
        self._timings: Dict[str, float] = {}

    def mark(self, name: str) -> None:
        now = time.perf_counter()
        self._timings[str(name)] = round(max(0.0, (now - self._last) * 1000.0), 3)
        self._last = now

    def snapshot(self) -> Dict[str, float]:
        return dict(self._timings)


@dataclass
class _NextCollabSourceCache:
    flight_paths: Dict[int, Optional[Dict[str, Any]]] = field(default_factory=dict)
    flight_path_loads: int = 0
    flight_path_hits: int = 0

    def summary(self) -> Dict[str, Any]:
        return {
            "flightPathLoads": int(self.flight_path_loads),
            "flightPathHits": int(self.flight_path_hits),
            "flightPathCached": len(self.flight_paths),
        }


def warm_next_collab_replan_pipeline() -> Dict[str, Any]:
    try:
        _ensure_runtime_import_paths()
        d0303, _, search_speed, mp_config = _import_runtime_modules()
        cruise_speed, turn_step = _apply_runtime_params(d0303, search_speed, mp_config)
    except Exception as exc:
        return {"ready": False, "error": str(exc)}
    return {
        "ready": True,
        "cruiseSpeedMps": float(cruise_speed),
        "turnStepDeg": float(turn_step),
    }


def _ensure_runtime_import_paths() -> None:
    candidate_paths = (
        mission_planner_root(),
        mission_planning_root(),
        project_root() / "modules",
        project_root(),
    )
    for path in candidate_paths:
        path_str = str(path)
        if path.exists() and path_str not in sys.path:
            sys.path.insert(0, path_str)


def _ensure_option_names(plan_ids: List[int], option_names: List[str] | None) -> List[str]:
    names = [str(name) for name in (option_names or []) if name is not None]
    if not names:
        names = [DEFAULT_OPTION_NAME]
    while len(names) < len(plan_ids):
        names.append(names[-1])
    return names[: len(plan_ids)]


def _set_source_field(payload: Dict[str, Any], source: str) -> None:
    if "Source" in payload or "source" not in payload:
        payload["Source"] = str(payload.get("Source") or payload.get("source") or source)
    else:
        payload["source"] = str(payload.get("source") or payload.get("Source") or source)


def _normalize_coordinate(payload: object | None) -> dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    alt = _normalize_altitude_value(payload.get("altitude"))
    if lat is None or lon is None:
        return None
    coord: dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        coord["altitude"] = int(alt)
    return coord


def _centroid_coordinate(coords: List[dict[str, float]]) -> dict[str, float] | None:
    if not coords:
        return None
    lat_vals = [float(item["latitude"]) for item in coords if "latitude" in item]
    lon_vals = [float(item["longitude"]) for item in coords if "longitude" in item]
    if not lat_vals or not lon_vals:
        return None
    out: dict[str, float] = {
        "latitude": sum(lat_vals) / float(len(lat_vals)),
        "longitude": sum(lon_vals) / float(len(lon_vals)),
    }
    alt_vals = [float(item["altitude"]) for item in coords if "altitude" in item]
    if alt_vals:
        avg_altitude = _normalize_altitude_value(sum(alt_vals) / float(len(alt_vals)))
        if avg_altitude is not None:
            out["altitude"] = int(avg_altitude)
    return out


def _extract_entry_coordinate_map(detail: Dict[str, Any]) -> Dict[int, dict[str, float]]:
    out: Dict[int, dict[str, float]] = {}
    for item in detail.get("entryAircraftList") or []:
        if not isinstance(item, dict):
            continue
        aid = _to_int(item.get("aircraftID"))
        coord = _normalize_coordinate(item.get("coordinate"))
        if aid is None or aid <= 0 or coord is None:
            continue
        out[int(aid)] = coord
    return out


def _extract_entry_heading_map(detail: Dict[str, Any]) -> Dict[int, float]:
    out: Dict[int, float] = {}
    for item in detail.get("entryAircraftList") or []:
        if not isinstance(item, dict):
            continue
        aid = _to_int(item.get("aircraftID"))
        heading = _to_float(
            item.get("headingDeg")
            if item.get("headingDeg") is not None
            else item.get("heading")
        )
        if aid is None or aid <= 0 or heading is None:
            continue
        out[int(aid)] = float(heading) % 360.0
    return out


def _extract_entry_aircraft_context_map(detail: Dict[str, Any]) -> Dict[int, Dict[str, Any]]:
    return build_line_entry_context_map_from_entry_rows(detail.get("entryAircraftList") or [])


def _build_takeover_info_list(entry_coord_map: Dict[int, dict[str, float]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for aircraft_id, coord in sorted(entry_coord_map.items()):
        rows.append(
            {
                "aircraftID": int(aircraft_id),
                "coordinate": {
                    "latitude": float(coord["latitude"]),
                    "longitude": float(coord["longitude"]),
                    "altitude": int(round(float(coord.get("altitude", 0.0) or 0.0))),
                },
            }
        )
    return rows


def _next_collab_area_path_row_phase_rank(path_row: Dict[str, Any]) -> int:
    source = str(path_row.get("source", "") or "")
    if source == "make_path_0":
        return 0
    if source == "make_waypoint":
        return 1
    if source == "make_path_2":
        return 2
    return 9


def _normalize_coord_list(payload: object | None) -> List[Dict[str, Any]]:
    coords = payload if isinstance(payload, list) else []
    out: List[Dict[str, Any]] = []
    for item in coords:
        coord = _normalize_coordinate(item)
        if coord is None:
            continue
        out.append(coord)
    return out


def _area_anchor_coordinate(area_list: object | None) -> dict[str, float] | None:
    areas = area_list if isinstance(area_list, list) else []
    centers: List[dict[str, float]] = []
    for area in areas:
        if not isinstance(area, dict):
            continue
        coords = _normalize_coord_list(area.get("coordinateList"))
        if not coords:
            continue
        center = _centroid_coordinate(coords)
        if center is not None:
            centers.append(center)
    if not centers:
        return None
    if len(centers) == 1:
        return dict(centers[0])
    return _centroid_coordinate(centers)


def _mission_entry_point(mission: Dict[str, Any]) -> Optional[Dict[str, float]]:
    if not isinstance(mission, dict):
        return None
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []

    if line_list:
        coords = _normalize_coord_list(line_list[0].get("coordinateList"))
        if coords:
            return dict(coords[0])
    if coord_list:
        coords = _normalize_coord_list(coord_list)
        if coords:
            return dict(coords[0])
    if area_list:
        all_centers: List[dict[str, float]] = []
        for area in area_list:
            if not isinstance(area, dict):
                continue
            coords = _normalize_coord_list(area.get("coordinateList"))
            if not coords:
                continue
            center = _centroid_coordinate(coords)
            if center is not None:
                all_centers.append(center)
        if all_centers:
            return _centroid_coordinate(all_centers)
    return None


def _find_input_mission(input_plan: Dict[str, Any], input_mission_id: int) -> Dict[str, Any] | None:
    for item in input_plan.get("inputMissionList") or []:
        if not isinstance(item, dict):
            continue
        if _to_int(item.get("inputMissionID")) == int(input_mission_id):
            return item
    return None


def _find_next_input_entry(input_plan: Dict[str, Any], input_mission_id: int) -> Dict[str, float] | None:
    found = False
    for item in input_plan.get("inputMissionList") or []:
        if not isinstance(item, dict):
            continue
        current_id = _to_int(item.get("inputMissionID"))
        if current_id == int(input_mission_id):
            found = True
            continue
        if not found:
            continue
        entry = _mission_entry_point(item)
        if entry is not None:
            return entry
    return None


def _group_next_collab_path_rows_by_aircraft(
    rows: List[Dict[str, Any]],
) -> Dict[int, List[Dict[str, Any]]]:
    grouped: Dict[int, List[Dict[str, Any]]] = {}
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        aircraft_id = _to_int(row.get("aircraftID")) or 0
        if aircraft_id <= 0:
            continue
        grouped.setdefault(int(aircraft_id), []).append(dict(row))
    return {int(aircraft_id): list(grouped[aircraft_id]) for aircraft_id in sorted(grouped)}


def _reserve_next_collab_replacement_ids(
    *,
    path_rows_by_aircraft: Dict[int, List[Dict[str, Any]]],
    emit: Callable[[str], None],
    id_reservation_summaries: Optional[List[Dict[str, Any]]] = None,
    scope: str,
) -> Optional[ReplanIdReservation]:
    total_rows = sum(len(rows) for rows in path_rows_by_aircraft.values())
    if total_rows <= 0:
        return None
    reservation = ReplanIdReservation.reserve(
        individual_count=int(total_rows),
        path_count_by_aircraft={
            int(aircraft_id): len(rows)
            for aircraft_id, rows in path_rows_by_aircraft.items()
            if rows
        },
    )
    summary = {"scope": str(scope), **reservation.summary()}
    if id_reservation_summaries is not None:
        id_reservation_summaries.append(summary)
    emit(
        "[NEXTCOLLAB][ID] reserved replacement IDs "
        f"scope={scope} individual={total_rows} "
        f"pathAircraft={sorted(int(aid) for aid in path_rows_by_aircraft)}"
    )
    return reservation


def _refresh_next_collab_id_reservation_summary(
    *,
    reservation: ReplanIdReservation,
    scope: str,
    id_reservation_summaries: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    summary = {"scope": str(scope), **reservation.summary()}
    if id_reservation_summaries is not None:
        for idx in range(len(id_reservation_summaries) - 1, -1, -1):
            if str(id_reservation_summaries[idx].get("scope") or "") == str(scope):
                id_reservation_summaries[idx] = summary
                break
        else:
            id_reservation_summaries.append(summary)
    return summary


def _reserve_unique_individual_ids(
    count: int,
    *,
    avoid_ids: Set[int],
) -> List[int]:
    needed = max(0, int(count or 0))
    if needed <= 0:
        return []
    avoid = {int(value) for value in avoid_ids if _to_int(value) is not None and int(value) > 0}
    reserved: List[int] = []
    max_avoid = max(avoid) if avoid else 0
    while len(reserved) < needed:
        request_count = needed - len(reserved)
        reservation = ReplanIdReservation.reserve(individual_count=int(request_count))
        batch = [reservation.next_individual() for _ in range(int(request_count))]
        for value in batch:
            value_int = int(value)
            if value_int in avoid or value_int in reserved:
                continue
            reserved.append(value_int)
        if len(reserved) >= needed:
            break
        max_batch = max([int(value) for value in batch], default=0)
        if max_batch <= max_avoid:
            catch_up_count = max(needed - len(reserved), max_avoid - max_batch + needed)
            catch_up = ReplanIdReservation.reserve(individual_count=int(catch_up_count))
            for _ in range(int(catch_up_count)):
                value_int = int(catch_up.next_individual())
                if value_int in avoid or value_int in reserved:
                    continue
                reserved.append(value_int)
                if len(reserved) >= needed:
                    break
        if not batch and len(reserved) < needed:
            raise RuntimeError("failed to reserve individualMissionIDs")
    return reserved[:needed]


def _deduplicate_generated_individual_mission_ids(
    *,
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    generated_fp_by_path: Dict[int, Dict[str, Any]],
    generated_path_ids: Set[int],
    emit: Callable[[str], None],
) -> int:
    seen: Dict[int, str] = {}
    all_ids: Set[int] = set()
    duplicate_generated: List[tuple[Dict[str, Any], int, int, str]] = []

    for aircraft_index, aircraft_id in enumerate(sorted(packages_by_aircraft)):
        pkg = packages_by_aircraft.get(int(aircraft_id))
        if not isinstance(pkg, dict):
            continue
        imp_id = _to_int(pkg.get("individualMissionPackageID")) or 0
        mission_list = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        for mission_index, mission in enumerate(mission_list):
            if not isinstance(mission, dict):
                continue
            mission_id = _to_int(mission.get("individualMissionID"))
            path_id = _to_int(mission.get("pathID"))
            if mission_id is None or mission_id <= 0:
                continue
            mission_id = int(mission_id)
            all_ids.add(mission_id)
            mission_ctx = (
                f"aircraftIndex={aircraft_index} aircraftID={int(aircraft_id)} "
                f"imp={int(imp_id)} missionIndex={mission_index}"
            )
            if mission_id not in seen:
                seen[mission_id] = mission_ctx
                continue
            if path_id is not None and int(path_id) in generated_fp_by_path:
                duplicate_generated.append((mission, int(path_id), int(mission_id), mission_ctx))
            else:
                emit(
                    "[NEXTCOLLAB][ID] duplicate preserved individualMissionID cannot be remapped "
                    f"without generated FlightPath: id={mission_id} first={seen[mission_id]} "
                    f"duplicate={mission_ctx}"
                )

    if not duplicate_generated:
        return 0

    new_ids = _reserve_unique_individual_ids(
        len(duplicate_generated),
        avoid_ids=set(all_ids),
    )
    for (mission, path_id, old_id, mission_ctx), new_id in zip(duplicate_generated, new_ids):
        mission["individualMissionID"] = int(new_id)
        flight_path = generated_fp_by_path.get(int(path_id))
        if isinstance(flight_path, dict):
            flight_path["individualMissionID"] = int(new_id)
            generated_path_ids.add(int(path_id))
        all_ids.add(int(new_id))
        emit(
            "[NEXTCOLLAB][ID] remapped duplicate generated individualMissionID "
            f"{old_id}->{int(new_id)} pathID={int(path_id)} {mission_ctx}"
        )
    return len(new_ids)


def _is_line_input_mission(mission: Dict[str, Any]) -> bool:
    if not isinstance(mission, dict):
        return False
    try:
        mission_type = int(mission.get("inputMissionType"))
    except Exception:
        mission_type = None
    if mission_type in (1, 4, 5, 7):
        return True
    if mission_type in (2, 3, 6):
        return False
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    return bool(line_list)


def _is_formation_input_mission(mission: Dict[str, Any]) -> bool:
    if not isinstance(mission, dict):
        return False
    return int(_to_int(mission.get("inputMissionType")) or 0) == 7


def _piece_entry_point(piece: SplitPiece) -> Optional[Dict[str, float]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    for key in ("Centerline", "coordinateList", "rawCoordinateList"):
        coords = _normalize_coord_list(data.get(key))
        if coords:
            return dict(coords[0])
    return None


def _bearing_from_coords(start: Dict[str, Any], end: Dict[str, Any]) -> float | None:
    lat1 = _to_float(start.get("latitude"))
    lon1 = _to_float(start.get("longitude"))
    lat2 = _to_float(end.get("latitude"))
    lon2 = _to_float(end.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    return float(split_algorithms_module._bearing_deg(start, end))


def _build_target_direction_debugs(
    mission: Dict[str, Any],
    *,
    prev_pt: Dict[str, Any] | None,
    next_pt: Dict[str, Any] | None,
) -> List[DirectionDebug]:
    mission_id = _to_int(mission.get("inputMissionID")) or 0
    mission_type = _to_int(mission.get("inputMissionType")) or 0
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    directions: List[DirectionDebug] = []

    if mission_type in (1, 4, 5, 7):
        debug = DirectionDebug(
            parent_order=1,
            mission_id=mission_id,
            mission_type=mission_type,
            source_area_index=None,
            prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
            next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
        )
        line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
        coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []
        if line_list:
            coords = _normalize_coord_list(line_list[0].get("coordinateList"))
            if coords:
                debug.line_start = dict(coords[0])
                debug.line_end = dict(coords[-1])
        elif coord_list:
            coords = _normalize_coord_list(coord_list)
            if coords:
                debug.line_start = dict(coords[0])
                debug.line_end = dict(coords[-1])
        directions.append(debug)
        return directions

    if mission_type in (2, 3, 6):
        area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
        for area_idx, area in enumerate(area_list, start=1):
            if not isinstance(area, dict):
                continue
            coords = _normalize_coord_list(area.get("coordinateList"))
            if len(coords) < 3:
                continue
            center, bearing_move, bearing_in, bearing_out = split_algorithms_module._resolve_area_bearing(
                prev_pt,
                next_pt,
                coords,
            )
            debug = DirectionDebug(
                parent_order=1,
                mission_id=mission_id,
                mission_type=mission_type,
                source_area_index=int(area_idx),
                prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
                next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
            )
            if center is not None:
                debug.center_point = {
                    "latitude": float(center["latitude"]),
                    "longitude": float(center["longitude"]),
                    "altitude": int(round(float(center.get("altitude", 0.0) or 0.0))),
                }
            debug.bearing_in_deg = float(bearing_in) if bearing_in is not None else None
            debug.bearing_out_deg = float(bearing_out) if bearing_out is not None else None
            debug.bearing_move_deg = float(bearing_move)
            debug.bearing_split_deg = float((bearing_move + 90.0) % 360.0)
            directions.append(debug)
        if directions:
            return directions

    return [
        DirectionDebug(
            parent_order=1,
            mission_id=mission_id,
            mission_type=mission_type,
            source_area_index=None,
            prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
            next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
        )
    ]


def _apply_piece_template_metadata(
    piece: SplitPiece,
    *,
    template_map: Dict[int, List[Dict[str, Any]]],
) -> None:
    aircraft_id = _to_int(piece.assigned_uav)
    if aircraft_id is None or aircraft_id <= 0:
        return
    templates = template_map.get(int(aircraft_id)) or []
    template = templates[0] if templates else None
    template_info = _template_mission_info(template)
    if not template_info:
        return
    if "individualMissionType" in template_info:
        piece.data["individualMissionType"] = template_info.get("individualMissionType")
    if "patternType" in template_info:
        piece.data["patternType"] = template_info.get("patternType")


def _run_area_review_for_target(
    *,
    pieces: List[SplitPiece],
    target_aircraft_ids: List[int],
    target_input_mission: Dict[str, Any],
    representative_entry: Dict[str, Any],
    next_entry: Dict[str, Any] | None,
    mrpk_data: Dict[str, Any],
    emit: Callable[[str], None],
) -> tuple[List[SplitPiece], Dict[str, Any]]:
    runtime_cfg = load_runtime_settings()
    review_enabled = bool(get_runtime_bool("enhanced_area_review_enabled", True, runtime_cfg))
    review_max_segment_m = float(
        get_runtime_area_review_max_segment_m(
            DEFAULT_AREA_REVIEW_MAX_SEGMENT_M,
            runtime_cfg,
        )
    )
    base_report: Dict[str, Any] = {
        "enabled": review_enabled,
        "mode": REPLAN_FLOW_MODE,
        "maxSegmentM": float(review_max_segment_m),
        "changed": False,
        "overflowRows": 0,
        "targets": 0,
        "localized": 0,
        "oldPieceCount": len(pieces),
        "newPieceCount": len(pieces),
        "details": [],
    }
    if not review_enabled:
        emit("[NEXTCOLLAB] area-review skipped by config.")
        return list(pieces), base_report

    split_result = SplitRunResult(
        uav_count=len(target_aircraft_ids),
        uav_ids=[int(aid) for aid in target_aircraft_ids],
        pieces=list(pieces),
    )
    review_report = dict(base_report)
    review_report.update(
        review_assigned_areas_local(
            split_result,
            mrpk_data if isinstance(mrpk_data, dict) else {},
            max_segment_m=review_max_segment_m,
        )
    )
    emit(
        "[NEXTCOLLAB] area-review(replan-local) done: "
        f"targets={int(review_report.get('targets', 0))} "
        f"localized={int(review_report.get('localized', 0))} "
        f"pieces={int(review_report.get('oldPieceCount', len(pieces)))}->"
        f"{int(review_report.get('newPieceCount', len(split_result.pieces)))} "
        f"maxSegmentM={float(review_max_segment_m):.1f}"
    )
    return list(split_result.pieces), review_report


def _apply_piece_entry_metadata(
    piece: SplitPiece,
    *,
    entry_coord: dict[str, float],
    next_coord: dict[str, float] | None,
) -> None:
    data = piece.data if isinstance(piece.data, dict) else {}
    data["prevPoint"] = {
        "latitude": float(entry_coord["latitude"]),
        "longitude": float(entry_coord["longitude"]),
        "altitude": int(round(float(entry_coord.get("altitude", 0.0) or 0.0))),
    }
    if next_coord is not None:
        data["nextPoint"] = {
            "latitude": float(next_coord["latitude"]),
            "longitude": float(next_coord["longitude"]),
            "altitude": int(round(float(next_coord.get("altitude", 0.0) or 0.0))),
        }

    coords = _normalize_coord_list(data.get("coordinateList"))
    if int(piece.mission_type) in (2, 3, 6) and len(coords) >= 3:
        center, bearing_move, bearing_in, bearing_out = split_algorithms_module._resolve_area_bearing(
            dict(data["prevPoint"]),
            dict(data["nextPoint"]) if "nextPoint" in data else None,
            coords,
        )
        bearing_entry = float(bearing_in) if bearing_in is not None else float(bearing_move)
        bearing_split = (bearing_entry + 90.0) % 360.0
        data["bearing_deg"] = float(bearing_entry)
        data["splitBearing_deg"] = float(bearing_split)
        data["phaseMoveBearing_deg"] = float(bearing_entry)
        data["phaseSplitBearing_deg"] = float(bearing_split)
        data["boundaryAxisBearing_deg"] = float(bearing_split)
        if bearing_in is not None:
            data["bearingIn_deg"] = float(bearing_in)
        if bearing_out is not None:
            data["bearingOut_deg"] = float(bearing_out)
        data.setdefault("sourceAreaIndex", 1)
        if center is not None and "center" not in data:
            data["center"] = center
    else:
        centerline = _normalize_coord_list(data.get("Centerline"))
        if not centerline:
            centerline = coords
        if centerline:
            bearing = _bearing_from_coords(data["prevPoint"], centerline[0])
            if bearing is not None:
                data["bearingFromPrev"] = float(bearing)


def _replace_geometry_from_piece(
    template_info: Dict[str, Any],
    generated_info: Dict[str, Any],
) -> Dict[str, Any]:
    info = deepcopy(template_info or {})
    if not info:
        return deepcopy(generated_info)

    for key in ("individualMissionType", "patternType", "autoZoomIn"):
        if key in generated_info:
            info[key] = deepcopy(generated_info[key])

    for key in ("lineList", "areaList", "coordinateList"):
        if key in generated_info:
            info[key] = deepcopy(generated_info[key])
        else:
            info.pop(key, None)

    for key in ("BEARING", "MOVE_BEARING"):
        if key in generated_info:
            info[key] = generated_info[key]

    for key in ("FOV", "SEP", "SPEED"):
        if key not in info and key in generated_info:
            info[key] = generated_info[key]

    area_anchor = _area_anchor_coordinate(info.get("areaList"))
    if area_anchor is not None and not info.get("lineList"):
        coord_list = _normalize_coord_list(info.get("coordinateList"))
        if len(coord_list) != 1:
            # LAH 0304 treats coordinateList as a route, so area missions must expose one anchor point.
            info["coordinateList"] = [deepcopy(area_anchor)]

    if "targetID" in generated_info and "targetID" not in info:
        info["targetID"] = generated_info["targetID"]
    return info


def _template_mission_info(mission: Dict[str, Any] | None) -> Dict[str, Any]:
    if not isinstance(mission, dict):
        return {}
    info = mission.get("individualMissionInfo")
    return dict(info) if isinstance(info, dict) else {}


def _summarize_next_collab_runtime_preservation(
    template_record_map: Dict[int, List[Dict[str, Any]]],
    *,
    turn_radius_scale: float,
) -> Dict[str, Any]:
    template_fov_values: Set[float] = set()
    template_sep_values: Set[float] = set()
    template_speed_values: Set[float] = set()
    template_count = 0
    for records in template_record_map.values():
        for record in records or []:
            if not isinstance(record, dict):
                continue
            mission = record.get("mission") if isinstance(record.get("mission"), dict) else {}
            info = _template_mission_info(mission)
            if not info:
                continue
            template_count += 1
            fov = _to_float(info.get("FOV"))
            sep = _to_float(info.get("SEP"))
            speed = _to_float(info.get("SPEED"))
            if fov is not None and fov > 0.0:
                template_fov_values.add(round(float(fov), 6))
            if sep is not None and sep > 0.0:
                template_sep_values.add(round(float(sep), 6))
            if speed is not None and speed > 0.0:
                template_speed_values.add(round(float(speed), 6))
    try:
        runtime_values = load_runtime_settings().get("values") or {}
    except Exception:
        runtime_values = {}
    preserved_runtime_keys = (
        "next_collab_turn_radius_scale",
        "line_custom_fov_deg",
        "area_custom_fov_deg",
        "area_nadir_fov_deg",
        "manual_fov_enabled",
        "next_collab_min_sep_m",
        "next_collab_default_sep_m",
    )
    return {
        "turnRadiusScale": float(turn_radius_scale),
        "templateRecordCount": int(template_count),
        "templateFovValues": sorted(template_fov_values),
        "templateSepValues": sorted(template_sep_values),
        "templateSpeedValues": sorted(template_speed_values),
        "runtimeSettings": {
            key: runtime_values.get(key)
            for key in preserved_runtime_keys
            if isinstance(runtime_values, dict) and key in runtime_values
        },
    }


def _mission_input_id(mission: Dict[str, Any]) -> int | None:
    related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    return _to_int(related.get("inputMissionID"))


def _representative_area_altitude_m(detail: Dict[str, Any]) -> float:
    if not isinstance(detail, dict):
        return 0.0
    row_lists = []
    if isinstance(detail.get("areaList"), list):
        row_lists.append(detail.get("areaList") or [])
    if isinstance(detail.get("areaSegmentList"), list):
        row_lists.append(detail.get("areaSegmentList") or [])
    for rows in row_lists:
        for row in rows:
            if not isinstance(row, dict):
                continue
            for coord in row.get("coordinateList") or []:
                if not isinstance(coord, dict):
                    continue
                altitude = _to_float(coord.get("altitude"))
                if altitude is not None:
                    return float(altitude)
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    for row in area_list:
        if not isinstance(row, dict):
            continue
        for coord in row.get("coordinateList") or []:
            if not isinstance(coord, dict):
                continue
            altitude = _to_float(coord.get("altitude"))
            if altitude is not None:
                return float(altitude)
    for coord in detail.get("coordinateList") or []:
        if not isinstance(coord, dict):
            continue
        altitude = _to_float(coord.get("altitude"))
        if altitude is not None:
            return float(altitude)
    return 0.0


def _coord_list_to_polygon_xy(coords: Any) -> Optional[Polygon]:
    coord_list = _normalize_coord_list(coords)
    points_xy: List[tuple[float, float]] = []
    for coord in coord_list:
        point_xy = coord_to_xy(coord)
        if point_xy is None:
            continue
        if points_xy and math.hypot(points_xy[-1][0] - point_xy[0], points_xy[-1][1] - point_xy[1]) < 1.0:
            continue
        points_xy.append((float(point_xy[0]), float(point_xy[1])))
    if len(points_xy) >= 2 and math.hypot(points_xy[0][0] - points_xy[-1][0], points_xy[0][1] - points_xy[-1][1]) < 1.0:
        points_xy = points_xy[:-1]
    if len(points_xy) < 3:
        return None
    try:
        poly = Polygon(points_xy)
    except Exception:
        return None
    if poly.is_empty:
        return None
    if not poly.is_valid:
        try:
            poly = poly.buffer(0)
        except Exception:
            return None
    if poly.is_empty:
        return None
    if isinstance(poly, Polygon):
        return poly
    if isinstance(poly, MultiPolygon):
        polygons = [item for item in poly.geoms if isinstance(item, Polygon) and not item.is_empty]
        return max(polygons, key=lambda item: float(item.area or 0.0)) if polygons else None
    return None


def _iter_polygons_xy(geometry: Any) -> List[Polygon]:
    if geometry is None or bool(getattr(geometry, "is_empty", False)):
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, MultiPolygon):
        return [item for item in geometry.geoms if isinstance(item, Polygon) and not item.is_empty]
    if isinstance(geometry, GeometryCollection):
        return [item for item in geometry.geoms if isinstance(item, Polygon) and not item.is_empty]
    return []


def _polygon_exterior_to_coords(poly: Polygon, *, altitude_m: float) -> List[Dict[str, Any]]:
    if not isinstance(poly, Polygon) or poly.is_empty:
        return []
    coords_out: List[Dict[str, Any]] = []
    for x_val, y_val in list(poly.exterior.coords)[:-1]:
        coords_out.append(meters_to_coord(float(x_val), float(y_val), alt_m=float(altitude_m)))
    return _normalize_coord_list(coords_out)


def _hole_cut_width_m(poly: Polygon) -> float:
    try:
        scale_m = math.sqrt(max(float(poly.area or 0.0), 0.0))
    except Exception:
        scale_m = 0.0
    return max(0.15, min(2.0, float(scale_m) * 0.0005))


def _hole_cut_polygons(poly: Polygon, *, min_area_m2: float = 1.0) -> List[Polygon]:
    if not isinstance(poly, Polygon) or poly.is_empty or not list(poly.interiors):
        return []
    try:
        exterior_line = LineString(poly.exterior.coords)
    except Exception:
        return []
    if exterior_line.is_empty:
        return []

    cut_width_m = _hole_cut_width_m(poly)
    cut_corridors: List[Any] = []
    for interior in list(poly.interiors):
        try:
            hole_poly = Polygon(interior)
        except Exception:
            continue
        if hole_poly.is_empty:
            continue
        try:
            hole_point, exterior_point = nearest_points(hole_poly.boundary, exterior_line)
            nearest_line = LineString(
                [
                    (float(hole_point.x), float(hole_point.y)),
                    (float(exterior_point.x), float(exterior_point.y)),
                ]
            )
        except Exception:
            continue
        if nearest_line.is_empty or float(nearest_line.length or 0.0) <= 0.0:
            continue
        try:
            cut = nearest_line.buffer(float(cut_width_m), cap_style=2, join_style=2)
        except Exception:
            continue
        if not cut.is_empty:
            cut_corridors.append(cut)
    if not cut_corridors:
        return []

    try:
        cut_geometry = unary_union(cut_corridors)
        opened = poly.difference(cut_geometry)
    except Exception:
        return []
    polygons: List[Polygon] = []
    for piece in _iter_polygons_xy(opened):
        if piece.is_empty or float(piece.area or 0.0) < float(min_area_m2):
            continue
        if list(piece.interiors):
            return []
        polygons.append(piece)
    polygons.sort(key=lambda item: float(item.area or 0.0), reverse=True)
    return polygons


def _hole_free_area_components_from_polygon(
    poly: Polygon,
    *,
    altitude_m: float,
    component_source: str,
    min_area_m2: float = 1.0,
) -> List[Dict[str, Any]]:
    if not isinstance(poly, Polygon) or poly.is_empty:
        return []
    if not poly.is_valid:
        try:
            poly = poly.buffer(0)
        except Exception:
            return []
    if poly.is_empty:
        return []

    if not list(poly.interiors):
        coords = _polygon_exterior_to_coords(poly, altitude_m=altitude_m)
        if len(coords) >= 3 and float(poly.area or 0.0) >= float(min_area_m2):
            return [
                {
                    "componentSource": str(component_source),
                    "componentDecomposition": "none",
                    "areaM2": float(poly.area or 0.0),
                    "coordinateList": coords,
                }
            ]
        return []

    components: List[Dict[str, Any]] = []
    cut_polygons = _hole_cut_polygons(poly, min_area_m2=float(min_area_m2))
    if cut_polygons:
        for piece in cut_polygons:
            coords = _polygon_exterior_to_coords(piece, altitude_m=altitude_m)
            if len(coords) < 3:
                continue
            components.append(
                {
                    "componentSource": str(component_source),
                    "componentDecomposition": "hole_cut",
                    "areaM2": float(piece.area or 0.0),
                    "coordinateList": coords,
                }
            )
        if components:
            components.sort(key=lambda item: float(item.get("areaM2") or 0.0), reverse=True)
            return components

    try:
        raw_triangles = triangulate(poly)
    except Exception:
        raw_triangles = []
    for triangle in raw_triangles:
        if not isinstance(triangle, Polygon) or triangle.is_empty:
            continue
        try:
            clipped = triangle.intersection(poly)
        except Exception:
            continue
        for piece in _iter_polygons_xy(clipped):
            if piece.is_empty or float(piece.area or 0.0) < float(min_area_m2):
                continue
            if list(piece.interiors):
                continue
            coords = _polygon_exterior_to_coords(piece, altitude_m=altitude_m)
            if len(coords) < 3:
                continue
            components.append(
                {
                    "componentSource": str(component_source),
                    "componentDecomposition": "triangulated_hole",
                    "areaM2": float(piece.area or 0.0),
                    "coordinateList": coords,
                }
            )
    components.sort(key=lambda item: float(item.get("areaM2") or 0.0), reverse=True)
    return components


def _area_planner_components_from_detail(detail: Dict[str, Any]) -> List[Dict[str, Any]]:
    mission_detail = detail if isinstance(detail, dict) else {}
    altitude_m = _representative_area_altitude_m(mission_detail)
    area_segment_list = (
        mission_detail.get("areaSegmentList")
        if isinstance(mission_detail.get("areaSegmentList"), list)
        else []
    )
    if area_segment_list:
        if len(area_segment_list) > int(AREA_PLANNER_COMPONENT_MAX_COUNT):
            return []
        components: List[Dict[str, Any]] = []
        for row in area_segment_list:
            if not isinstance(row, dict):
                continue
            poly = _coord_list_to_polygon_xy(row.get("coordinateList"))
            if poly is None:
                continue
            row_components = _hole_free_area_components_from_polygon(
                poly,
                altitude_m=float(altitude_m),
                component_source="planned_sweep_row_segment",
            )
            for component in row_components:
                component["sourceLineIndex"] = _to_int(row.get("lineIndex"))
                component["sourceAircraftID"] = _to_int(row.get("aircraftID"))
                component["sourceIndividualMissionID"] = _to_int(row.get("individualMissionID"))
                component["sourceInputMissionID"] = _to_int(row.get("inputMissionID"))
                if _to_int(row.get("pathID")) is not None:
                    component["sourcePathID"] = _to_int(row.get("pathID"))
                components.append(component)
                if len(components) > int(AREA_PLANNER_COMPONENT_MAX_COUNT):
                    return []
        for idx, component in enumerate(components, start=1):
            component["componentIndex"] = int(idx)
        return components

    area_list = mission_detail.get("areaList") if isinstance(mission_detail.get("areaList"), list) else []
    outer_polygons: List[Polygon] = []
    hole_polygons: List[Polygon] = []
    if area_list:
        for row in area_list:
            if not isinstance(row, dict):
                continue
            poly = _coord_list_to_polygon_xy(row.get("coordinateList"))
            if poly is None:
                continue
            if bool(row.get("isHole")):
                hole_polygons.append(poly)
            else:
                outer_polygons.append(poly)
    else:
        poly = _coord_list_to_polygon_xy(mission_detail.get("coordinateList"))
        if poly is not None:
            outer_polygons.append(poly)

    if not outer_polygons:
        return []

    try:
        remaining_geometry = unary_union(outer_polygons)
    except Exception:
        remaining_geometry = outer_polygons[0]
    if hole_polygons:
        try:
            remaining_geometry = remaining_geometry.difference(unary_union(hole_polygons))
        except Exception:
            return []

    source = "single_polygon"
    if len(outer_polygons) > 1 and hole_polygons:
        source = "multi_polygon_with_holes"
    elif len(outer_polygons) > 1:
        source = "multi_polygon"
    elif hole_polygons:
        source = "hole_decomposition"

    components: List[Dict[str, Any]] = []
    for poly in _iter_polygons_xy(remaining_geometry):
        components.extend(
            _hole_free_area_components_from_polygon(
                poly,
                altitude_m=float(altitude_m),
                component_source=source,
            )
        )
    if len(components) > int(AREA_PLANNER_COMPONENT_MAX_COUNT):
        return []
    for idx, component in enumerate(components, start=1):
        component["componentIndex"] = int(idx)
    return components


def _area_planner_component_input_summary(detail: Dict[str, Any]) -> Dict[str, Any]:
    mission_detail = detail if isinstance(detail, dict) else {}
    area_segment_list = (
        mission_detail.get("areaSegmentList")
        if isinstance(mission_detail.get("areaSegmentList"), list)
        else []
    )
    area_list = mission_detail.get("areaList") if isinstance(mission_detail.get("areaList"), list) else []
    coordinate_list = _normalize_coord_list(mission_detail.get("coordinateList"))
    outer_count = sum(1 for row in area_list if isinstance(row, dict) and not bool(row.get("isHole")))
    hole_count = sum(1 for row in area_list if isinstance(row, dict) and bool(row.get("isHole")))
    segment_polygon_count = 0
    for row in area_segment_list:
        if not isinstance(row, dict):
            continue
        if len(_normalize_coord_list(row.get("coordinateList"))) >= 3:
            segment_polygon_count += 1

    if area_segment_list and len(area_segment_list) > int(AREA_PLANNER_COMPONENT_MAX_COUNT):
        reason = "area_segment_component_cap_exceeded"
    elif area_segment_list:
        reason = "area_segment_component_extraction_failed"
    elif not area_list and len(coordinate_list) < 3:
        reason = "area_geometry_unavailable"
    elif hole_count > 0 or outer_count > 1:
        reason = "area_component_extraction_failed_or_component_cap_exceeded"
    else:
        reason = "area_component_extraction_failed"

    return {
        "reason": reason,
        "areaSegmentCount": len(area_segment_list),
        "validAreaSegmentPolygonCount": int(segment_polygon_count),
        "areaBlockCount": len(area_list),
        "areaOuterCount": int(outer_count),
        "areaHoleCount": int(hole_count),
        "coordinatePointCount": len(coordinate_list),
        "maxComponents": int(AREA_PLANNER_COMPONENT_MAX_COUNT),
    }


def _is_piece_only_area_takeover_input(target_input_mission: Dict[str, Any]) -> bool:
    if not isinstance(target_input_mission, dict):
        return False
    if str(target_input_mission.get("areaOwnershipPolicy") or "") != "piece_only_takeover":
        return False
    source_ids = target_input_mission.get("areaTakeoverSourceAircraftIDs")
    if not isinstance(source_ids, list) or not source_ids:
        return False
    return any((_to_int(aid) or 0) > 3 for aid in source_ids)


def _merge_replacements_into_active_missions(
    *,
    active_mission_list: List[Dict[str, Any]],
    replacements: List[Dict[str, Any]],
    target_input_id: int,
    preserve_current_input: bool,
) -> tuple[List[Dict[str, Any]], Dict[str, Any]]:
    if not replacements:
        return list(active_mission_list), {
            "policy": "preserve_existing_no_replacement",
            "preservedTargetMissionCount": sum(
                1
                for mission in active_mission_list
                if isinstance(mission, dict) and _mission_input_id(mission) == int(target_input_id)
            ),
            "insertedReplacementCount": 0,
        }

    if preserve_current_input:
        insert_index = None
        preserved_count = 0
        for idx, mission in enumerate(active_mission_list):
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) == int(target_input_id):
                insert_index = int(idx) + 1
                preserved_count += 1
        if insert_index is None:
            insert_index = len(active_mission_list)
        return (
            list(active_mission_list[:insert_index])
            + list(replacements)
            + list(active_mission_list[insert_index:]),
            {
                "policy": "preserve_current_input_then_takeover_piece",
                "preservedTargetMissionCount": int(preserved_count),
                "insertedReplacementCount": len(replacements),
                "insertIndex": int(insert_index),
            },
        )

    filtered_missions: List[Dict[str, Any]] = []
    insert_index = None
    replaced_count = 0
    for mission in active_mission_list:
        if not isinstance(mission, dict):
            continue
        if _mission_input_id(mission) == int(target_input_id):
            if insert_index is None:
                insert_index = len(filtered_missions)
            replaced_count += 1
            continue
        filtered_missions.append(mission)
    if insert_index is None:
        insert_index = len(filtered_missions)
    return (
        filtered_missions[:insert_index] + list(replacements) + filtered_missions[insert_index:],
        {
            "policy": "replace_current_input",
            "replacedTargetMissionCount": int(replaced_count),
            "insertedReplacementCount": len(replacements),
            "insertIndex": int(insert_index),
        },
    )


def _normalize_bearing_deg(value: Any) -> float | None:
    bearing = _to_float(value)
    if bearing is None:
        return None
    return float(bearing) % 360.0


def _bearing_diff_deg(left: Any, right: Any) -> float | None:
    b0 = _normalize_bearing_deg(left)
    b1 = _normalize_bearing_deg(right)
    if b0 is None or b1 is None:
        return None
    diff = (float(b0) - float(b1) + 180.0) % 360.0 - 180.0
    return abs(float(diff))


def _mission_route_bearing(info: Dict[str, Any] | None) -> float | None:
    if not isinstance(info, dict):
        return None
    coords = _normalize_coord_list(info.get("coordinateList"))
    if len(coords) >= 2:
        return _bearing_from_coords(coords[0], coords[-1])
    line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
    if line_list:
        line_coords = _normalize_coord_list((line_list[0] or {}).get("coordinateList"))
        if len(line_coords) >= 2:
            return _bearing_from_coords(line_coords[0], line_coords[-1])
    return _normalize_bearing_deg(info.get("MOVE_BEARING")) or _normalize_bearing_deg(info.get("BEARING"))


def _representative_replacement_geometry(
    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]],
    *,
    target_input_id: int,
) -> tuple[Dict[str, Any] | None, float | None]:
    candidates: List[tuple[int, Dict[str, Any], float | None]] = []
    for aircraft_id, missions in sorted(replacement_by_aircraft.items()):
        for mission in missions or []:
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) != int(target_input_id):
                continue
            info = _template_mission_info(mission)
            if not info:
                continue
            bearing = (
                _normalize_bearing_deg(mission.get("bearing_deg"))
                or _mission_route_bearing(info)
            )
            candidates.append((int(aircraft_id), info, bearing))
    if not candidates:
        return None, None

    representative_info = deepcopy(candidates[0][1])
    bearing_samples = [float(item[2]) for item in candidates if item[2] is not None]
    if not bearing_samples:
        return representative_info, None

    sin_sum = sum(math.sin(math.radians(float(bearing))) for bearing in bearing_samples)
    cos_sum = sum(math.cos(math.radians(float(bearing))) for bearing in bearing_samples)
    if abs(sin_sum) <= 1e-9 and abs(cos_sum) <= 1e-9:
        representative_bearing = float(bearing_samples[0]) % 360.0
    else:
        representative_bearing = math.degrees(math.atan2(sin_sum, cos_sum)) % 360.0
    return representative_info, float(representative_bearing)


def _rebuild_next_collab_lah_target_paths(
    *,
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]],
    target_input_id: int,
    input_plan: Dict[str, Any] | None = None,
    generated_fp_by_path: Dict[int, Dict[str, Any]],
    generated_path_ids: Set[int],
    emit: Callable[[str], None],
    id_reservation_summaries: Optional[List[Dict[str, Any]]] = None,
) -> int:
    representative_info, representative_bearing = _representative_replacement_geometry(
        replacement_by_aircraft,
        target_input_id=int(target_input_id),
    )
    special_info = lah_special_info_for_input(input_plan, int(target_input_id))
    if isinstance(special_info, dict):
        representative_info = deepcopy(special_info)
        representative_bearing = _mission_route_bearing(representative_info)
        emit(
            "[NEXTCOLLAB][LAH] special operation geometry applied "
            f"inputMissionID={int(target_input_id)}"
        )
    if not isinstance(representative_info, dict):
        return 0

    try:
        d0303, d0304, search_speed, mp_config = _import_runtime_modules()
        _apply_runtime_params(d0303, d0304, search_speed, mp_config)
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to load 0304 runtime modules: {exc}")
        return 0

    try:
        runtime_values = (load_runtime_settings().get("values") or {})
    except Exception:
        runtime_values = {}

    target_indices_by_aircraft: Dict[int, List[int]] = {}
    for aircraft_id in (1, 2, 3):
        pkg = packages_by_aircraft.get(int(aircraft_id))
        if not isinstance(pkg, dict):
            continue
        mission_list = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        target_indices = [
            idx
            for idx, mission in enumerate(mission_list)
            if isinstance(mission, dict) and _mission_input_id(mission) == int(target_input_id)
        ]
        if not target_indices:
            continue
        target_indices_by_aircraft[int(aircraft_id)] = list(target_indices)

    if not target_indices_by_aircraft:
        return 0
    reservation = ReplanIdReservation.reserve(
        path_count_by_aircraft={
            int(aircraft_id): len(indices)
            for aircraft_id, indices in target_indices_by_aircraft.items()
            if indices
        }
    )
    lah_scope = "lah_target_path_refresh"
    id_reservation_summaries = id_reservation_summaries if id_reservation_summaries is not None else []
    id_reservation_summaries.append({"scope": lah_scope, **reservation.summary()})

    candidate_updates: List[tuple[int, int, Dict[str, Any]]] = []
    manned_missions: List[Dict[str, Any]] = []
    manned_mission_by_path: Dict[int, Dict[str, Any]] = {}
    for aircraft_id, target_indices in target_indices_by_aircraft.items():
        pkg = packages_by_aircraft.get(int(aircraft_id))
        if not isinstance(pkg, dict):
            continue
        mission_list = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        for mission_idx in target_indices:
            mission = mission_list[mission_idx]
            if not isinstance(mission, dict):
                continue
            updated_mission = deepcopy(mission)
            updated_info = _replace_geometry_from_piece(
                _template_mission_info(updated_mission),
                representative_info,
            )
            if representative_bearing is not None:
                updated_info["BEARING"] = float(representative_bearing)
                updated_info["MOVE_BEARING"] = float(representative_bearing)
                updated_mission["bearing_deg"] = float(representative_bearing)
            updated_mission["individualMissionInfo"] = updated_info
            updated_mission["pathID"] = int(reservation.next_path(int(aircraft_id)))
            candidate_updates.append((int(aircraft_id), int(mission_idx), updated_mission))
            # d0304.build_lah_flight_plans_fixed expects aircraftID on each mission row.
            mission_for_lah = deepcopy(updated_mission)
            mission_for_lah["aircraftID"] = int(aircraft_id)
            manned_missions.append(mission_for_lah)
            path_id = _to_int(mission_for_lah.get("pathID")) or 0
            if path_id > 0:
                manned_mission_by_path[int(path_id)] = mission_for_lah

    if not manned_missions:
        return 0
    _refresh_next_collab_id_reservation_summary(
        reservation=reservation,
        scope=lah_scope,
        id_reservation_summaries=id_reservation_summaries,
    )

    try:
        manned_plan_mode = str(runtime_values.get("manned_plan_mode") or "normal").strip().lower()
    except Exception:
        manned_plan_mode = "normal"

    try:
        lah_packets = d0304.build_lah_flight_plans_fixed(
            manned_missions,
            cruise_speed=30.0,
            manned_plan_mode=manned_plan_mode,
            lah_path_mode=str(runtime_values.get("lah_path_mode", "linear")),
            lah_rl_hex_step=int(runtime_values.get("lah_rl_hex_step", 50)),
            lah_rl_area_km=float(runtime_values.get("lah_rl_area_km", 10.0)),
        )
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to build LAH 0304 packets: {exc}")
        return 0

    uav_packets = [payload for payload in generated_fp_by_path.values() if isinstance(payload, dict)]
    if lah_packets and uav_packets:
        try:
            lah_packets = d0304.apply_uav_eta_follow_speed_plan(
                list(lah_packets),
                list(uav_packets),
            )
        except Exception as exc:
            emit(f"[NEXTCOLLAB] failed to apply LAH follow-speed plan: {exc}")

    built_packets = {
        int(_to_int(packet.get("pathID")) or 0): packet
        for packet in lah_packets
        if isinstance(packet, dict) and (_to_int(packet.get("pathID")) or 0) > 0
    }
    expected_path_ids = {
        int(_to_int(mission.get("pathID")) or 0)
        for mission in manned_missions
        if isinstance(mission, dict) and (_to_int(mission.get("pathID")) or 0) > 0
    }
    if not expected_path_ids or not expected_path_ids.issubset(set(built_packets.keys())):
        missing = sorted(expected_path_ids.difference(set(built_packets.keys())))
        emit(
            "[NEXTCOLLAB] LAH 0304 regeneration incomplete; "
            f"missing pathIDs={missing}"
        )
        return 0

    for aircraft_id, mission_idx, updated_mission in candidate_updates:
        pkg = packages_by_aircraft.get(int(aircraft_id))
        if not isinstance(pkg, dict):
            continue
        mission_list = pkg.get("individualMissionList")
        if not isinstance(mission_list, list) or not (0 <= mission_idx < len(mission_list)):
            continue
        mission_list[mission_idx] = deepcopy(updated_mission)

    for path_id, packet in built_packets.items():
        source_mission = manned_mission_by_path.get(int(path_id))
        source_mission_id = _to_int((source_mission or {}).get("individualMissionID"))
        if source_mission_id is not None and source_mission_id > 0:
            packet["individualMissionID"] = int(source_mission_id)
        generated_fp_by_path[int(path_id)] = deepcopy(packet)
        generated_path_ids.add(int(path_id))

    emit(
        "[NEXTCOLLAB] regenerated LAH 0304 for target mission -> "
        f"paths={sorted(expected_path_ids)} bearing={representative_bearing if representative_bearing is not None else '-'}"
    )
    return len(expected_path_ids)


def _extract_target_templates(
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    target_input_id: int,
) -> Dict[int, List[Dict[str, Any]]]:
    return _extract_templates_for_input(packages_by_aircraft, target_input_id)


def _extract_templates_for_input(
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    input_mission_id: int,
) -> Dict[int, List[Dict[str, Any]]]:
    out: Dict[int, List[Dict[str, Any]]] = {}
    for aircraft_id, pkg in packages_by_aircraft.items():
        missions = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        for mission in missions:
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) != int(input_mission_id):
                continue
            out.setdefault(int(aircraft_id), []).append(mission)
    return out


def _load_flight_path_payload(
    path_id: int | None,
    *,
    source_cache: Optional[_NextCollabSourceCache] = None,
) -> Dict[str, Any] | None:
    if path_id is None or int(path_id) <= 0:
        return None
    if source_cache is not None and int(path_id) in source_cache.flight_paths:
        source_cache.flight_path_hits += 1
        cached = source_cache.flight_paths.get(int(path_id))
        return deepcopy(cached) if isinstance(cached, dict) else None
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        if not path.exists():
            if source_cache is not None:
                source_cache.flight_paths[int(path_id)] = None
            return None
        payload = read_json_cached(path, kind="FlightPath")
        if source_cache is not None:
            source_cache.flight_path_loads += 1
            source_cache.flight_paths[int(path_id)] = deepcopy(payload) if isinstance(payload, dict) else None
        return payload if isinstance(payload, dict) else None
    except Exception:
        if source_cache is not None:
            source_cache.flight_paths[int(path_id)] = None
        return None


def _extract_target_template_records(
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    target_input_id: int,
    *,
    source_cache: Optional[_NextCollabSourceCache] = None,
) -> Dict[int, List[Dict[str, Any]]]:
    return _extract_template_records_for_input(
        packages_by_aircraft,
        target_input_id,
        source_cache=source_cache,
    )


def _extract_template_records_for_input(
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    input_mission_id: int,
    *,
    source_cache: Optional[_NextCollabSourceCache] = None,
) -> Dict[int, List[Dict[str, Any]]]:
    out: Dict[int, List[Dict[str, Any]]] = {}
    for aircraft_id, pkg in packages_by_aircraft.items():
        missions = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        for mission in missions:
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) != int(input_mission_id):
                continue
            path_id = _to_int(mission.get("pathID"))
            out.setdefault(int(aircraft_id), []).append(
                {
                    "mission": mission,
                    "flightPath": _load_flight_path_payload(path_id, source_cache=source_cache),
                }
            )
    return out


def _resolve_next_collab_target_aircraft_ids(
    entry_coord_map: Dict[int, Dict[str, Any]],
    template_map: Dict[int, List[Dict[str, Any]]],
) -> List[int]:
    target_ids: List[int] = []
    for raw_aircraft_id in entry_coord_map.keys():
        aircraft_id = _to_int(raw_aircraft_id)
        if aircraft_id is None or aircraft_id <= 0:
            continue
        if int(aircraft_id) not in target_ids:
            target_ids.append(int(aircraft_id))
    if target_ids:
        return target_ids
    return sorted(int(aid) for aid in template_map.keys() if int(aid) > 0)


def _missing_target_entry_aircraft_ids(
    target_aircraft_ids: List[int],
    entry_coord_map: Dict[int, Dict[str, Any]],
) -> List[int]:
    missing: List[int] = []
    for aircraft_id in sorted({int(aid) for aid in target_aircraft_ids if _to_int(aid) is not None and int(aid) > 0}):
        if _normalize_coordinate(entry_coord_map.get(int(aircraft_id))) is None:
            missing.append(int(aircraft_id))
    return missing


def _format_aircraft_ids(values: List[int]) -> str:
    return ",".join(str(int(aid)) for aid in values)


def _ensure_target_template_records_for_aircraft(
    *,
    target_aircraft_ids: List[int],
    template_map: Dict[int, List[Dict[str, Any]]],
    template_record_map: Dict[int, List[Dict[str, Any]]],
    emit: Callable[[str], None],
) -> None:
    missing_aircraft_ids = [
        int(aid)
        for aid in target_aircraft_ids
        if not template_record_map.get(int(aid))
    ]
    if not missing_aircraft_ids:
        return

    fallback_aircraft_id: int | None = None
    fallback_records: List[Dict[str, Any]] = []
    fallback_ids = sorted(
        int(aid)
        for aid, records in template_record_map.items()
        if int(aid) >= 4 and records
    )
    for candidate_id in fallback_ids:
        records = template_record_map.get(int(candidate_id)) or []
        if records:
            fallback_aircraft_id = int(candidate_id)
            fallback_records = [deepcopy(record) for record in records]
            break

    for aircraft_id in missing_aircraft_ids:
        if fallback_records:
            template_record_map[int(aircraft_id)] = [deepcopy(record) for record in fallback_records]
            template_map[int(aircraft_id)] = [
                deepcopy(record.get("mission") or {})
                for record in fallback_records
                if isinstance(record, dict)
            ]
            emit(
                "[NEXTCOLLAB] target aircraft has no source template; "
                f"aircraft={aircraft_id}, fallbackTemplateAircraft={fallback_aircraft_id}"
            )
        else:
            emit(
                "[NEXTCOLLAB] target aircraft has no source template; "
                f"aircraft={aircraft_id}, using generated defaults"
            )


def _build_line_takeover_mrpk(
    *,
    target_aircraft_ids: List[int],
    entry_coord_map: Dict[int, Dict[str, Any]],
    heading_map: Dict[int, float],
    representative_entry: Dict[str, Any] | None,
) -> Dict[str, Any]:
    rows: List[Dict[str, Any]] = []
    for aircraft_id in sorted(int(aid) for aid in target_aircraft_ids):
        coord = entry_coord_map.get(int(aircraft_id))
        normalized = _normalize_coordinate(coord)
        if normalized is None:
            continue
        row: Dict[str, Any] = {
            "aircraftID": int(aircraft_id),
            "coordinate": dict(normalized),
        }
        if int(aircraft_id) in heading_map:
            row["headingDeg"] = float(heading_map[int(aircraft_id)])
        rows.append(row)
    return {
        "takeOverInfoList": rows,
    }


def _coord_distance_m(start: Dict[str, Any], end: Dict[str, Any]) -> float:
    lat1 = _to_float(start.get("latitude"))
    lon1 = _to_float(start.get("longitude"))
    lat2 = _to_float(end.get("latitude"))
    lon2 = _to_float(end.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return float("inf")
    deg_m = 111_132.0
    dx = (float(lon2) - float(lon1)) * deg_m * math.cos(math.radians((float(lat1) + float(lat2)) / 2.0))
    dy = (float(lat2) - float(lat1)) * deg_m
    return math.hypot(dx, dy)


def _angle_diff_deg(a: float, b: float) -> float:
    diff = (float(a) - float(b) + 180.0) % 360.0 - 180.0
    return abs(float(diff))


def _line_orientation_cost(
    *,
    entry_coord: Dict[str, Any],
    heading_deg: float | None,
    start_coord: Dict[str, Any],
    end_coord: Dict[str, Any],
) -> float:
    cost = _coord_distance_m(entry_coord, start_coord)
    if heading_deg is None:
        return float(cost)
    try:
        line_bearing = float(split_algorithms_module._bearing_deg(start_coord, end_coord))
    except Exception:
        return float(cost)
    return float(cost) + (_angle_diff_deg(float(heading_deg), float(line_bearing)) * 5.0)


def _orient_line_piece_for_entry(
    piece: SplitPiece,
    *,
    entry_coord: Dict[str, Any],
    heading_deg: float | None,
) -> bool:
    data = piece.data if isinstance(piece.data, dict) else {}
    centerline = _normalize_coord_list(data.get("Centerline"))
    if len(centerline) < 2:
        centerline = _normalize_coord_list(data.get("coordinateList"))
    if len(centerline) < 2:
        return False

    forward_cost = _line_orientation_cost(
        entry_coord=entry_coord,
        heading_deg=heading_deg,
        start_coord=centerline[0],
        end_coord=centerline[-1],
    )
    reverse_cost = _line_orientation_cost(
        entry_coord=entry_coord,
        heading_deg=heading_deg,
        start_coord=centerline[-1],
        end_coord=centerline[0],
    )
    if reverse_cost + 1e-6 >= forward_cost:
        return False

    reversed_centerline = list(reversed(centerline))
    if data.get("Centerline") is not None:
        data["Centerline"] = reversed_centerline
    elif data.get("coordinateList") is not None:
        data["coordinateList"] = reversed_centerline
    return True


def _copy_runtime_meta_fields(source_mission: Dict[str, Any], dest_mission: Dict[str, Any]) -> None:
    for key in (
        "bearing_deg",
        "splitBearing_deg",
        "phaseMoveBearing_deg",
        "phaseSplitBearing_deg",
        "boundaryAxisBearing_deg",
        "bearingIn_deg",
        "bearingOut_deg",
        "prevPoint",
        "nextPoint",
    ):
        if key in source_mission:
            dest_mission[key] = deepcopy(source_mission[key])


def _clone_generated_flight_path(
    generated_path: Dict[str, Any],
    *,
    timestamp_ms: int,
    path_id: int,
    aircraft_id: int,
    individual_mission_id: int,
    source: str = "MMR",
) -> Dict[str, Any]:
    flight_path = deepcopy(generated_path) if isinstance(generated_path, dict) else {}
    flight_path["timestamp"] = int(timestamp_ms)
    flight_path["pathID"] = int(path_id)
    flight_path["aircraftID"] = int(aircraft_id)
    flight_path["individualMissionID"] = int(individual_mission_id)
    _set_source_field(flight_path, str(source))
    waypoint_list = flight_path.get("waypointList") if isinstance(flight_path.get("waypointList"), list) else []
    for waypoint in waypoint_list:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    if "lahWaypointList" in flight_path:
        flight_path["lahWaypointList"] = deepcopy(waypoint_list)
    return flight_path


def _focus_coordinate_from_waypoint(waypoint: Dict[str, Any]) -> dict[str, float] | None:
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    coords = _normalize_coord_list(line_search.get("coordinateList"))
    if coords:
        return dict(coords[0])
    coord_orientation = filming.get("coordinateOrientation") if isinstance(filming.get("coordinateOrientation"), dict) else {}
    coord = _normalize_coordinate(coord_orientation.get("coordinate"))
    if coord is not None:
        return coord
    return _normalize_coordinate(waypoint.get("coordinate"))


def _build_entry_waypoint(
    *,
    entry_coord: dict[str, float],
    focus_coord: dict[str, float] | None,
    template_wp: dict[str, Any] | None,
    waypoint_id: int,
) -> Dict[str, Any]:
    base_coord = _normalize_coordinate((template_wp or {}).get("coordinate")) or {}
    altitude = _normalize_altitude_value(entry_coord.get("altitude"))
    if altitude is not None and (int(altitude) < 700 or int(altitude) > 2200):
        altitude = None
    if altitude is None:
        altitude = _normalize_altitude_value(base_coord.get("altitude"))
    if altitude is None:
        altitude = 0
    speed = _to_float((template_wp or {}).get("speed"))
    if speed is None or speed <= 0.0:
        speed = 40.0
    filming = (template_wp or {}).get("filmingProperty") if isinstance((template_wp or {}).get("filmingProperty"), dict) else {}
    sensor_type = _to_int(filming.get("sensorType")) or 1
    fov = _to_float(filming.get("fieldOfView"))
    if fov is None or fov <= 0.0:
        fov = ENTRY_FOV_DEG
    try:
        fov = float(get_runtime_manual_fov_deg("entry_hold_fov_deg", float(fov)))
    except Exception:
        fov = float(fov)
    entry_wp: Dict[str, Any] = {
        "waypointID": int(waypoint_id),
        "coordinate": {
            "latitude": float(entry_coord["latitude"]),
            "longitude": float(entry_coord["longitude"]),
            "altitude": int(altitude),
        },
        "speed": float(speed),
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": 1,
        "filmingProperty": {
            "fieldOfView": float(fov),
            "sensorType": int(sensor_type),
            "operationMode": 1,
        },
    }
    if focus_coord is not None:
        focus_coord_norm = _coord_with_dem_altitude(focus_coord)
        focus_altitude = _normalize_altitude_value(focus_coord_norm.get("altitude"))
        if focus_altitude is not None:
            minimum_altitude = int(
                math.ceil(float(focus_altitude) + float(FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M))
            )
            if int(entry_wp["coordinate"]["altitude"]) < int(minimum_altitude):
                entry_wp["coordinate"]["altitude"] = int(minimum_altitude)
        entry_wp["filmingProperty"]["coordinateOrientation"] = {
            "coordinate": {
                "latitude": float(focus_coord_norm["latitude"]),
                "longitude": float(focus_coord_norm["longitude"]),
                "altitude": int(round(float(focus_coord_norm.get("altitude", 0.0) or 0.0))),
            }
        }
    return entry_wp


def _midpoint_of_sweep_coords(coords: List[Dict[str, Any]]) -> Dict[str, float] | None:
    if len(coords) < 2:
        return None
    first = coords[0]
    last = coords[-1]
    lat1 = _to_float(first.get("latitude"))
    lon1 = _to_float(first.get("longitude"))
    lat2 = _to_float(last.get("latitude"))
    lon2 = _to_float(last.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    alt1 = _to_float(first.get("altitude"))
    alt2 = _to_float(last.get("altitude"))
    out: Dict[str, float] = {
        "latitude": (float(lat1) + float(lat2)) / 2.0,
        "longitude": (float(lon1) + float(lon2)) / 2.0,
    }
    if alt1 is not None and alt2 is not None:
        midpoint_altitude = _normalize_altitude_value((float(alt1) + float(alt2)) / 2.0)
        if midpoint_altitude is not None:
            out["altitude"] = int(midpoint_altitude)
    elif alt1 is not None:
        out["altitude"] = int(round(float(alt1)))
    elif alt2 is not None:
        out["altitude"] = int(round(float(alt2)))
    return out


def _align_waypoint_to_line_search_center(waypoint: Dict[str, Any]) -> None:
    if not isinstance(waypoint, dict):
        return
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    coords = _normalize_coord_list(line_search.get("coordinateList"))
    if not coords:
        return
    center = _midpoint_of_sweep_coords(coords)
    if center is None:
        return
    base_coord = _normalize_coordinate(waypoint.get("coordinate")) or {}
    altitude = _normalize_altitude_value(base_coord.get("altitude"))
    if altitude is None:
        altitude = _normalize_altitude_value(center.get("altitude"))
    if altitude is None:
        altitude = 0
    waypoint["coordinate"] = {
        "latitude": float(center["latitude"]),
        "longitude": float(center["longitude"]),
        "altitude": int(altitude),
    }

def _prepend_entry_waypoint(
    flight_plan: Dict[str, Any],
    *,
    entry_coord: dict[str, float],
    waypoint_id_provider: Optional[Callable[[], int]] = None,
) -> None:
    waypoints = flight_plan.get("waypointList")
    if not isinstance(waypoints, list) or not waypoints:
        return

    first_wp = waypoints[0] if isinstance(waypoints[0], dict) else {}
    first_filming = first_wp.get("filmingProperty") if isinstance(first_wp.get("filmingProperty"), dict) else {}
    first_line_search = first_filming.get("lineSearch") if isinstance(first_filming.get("lineSearch"), dict) else {}
    if len(waypoints) >= 2 and not first_line_search and _to_int(first_filming.get("operationMode")) == 4:
        second_wp = waypoints[1] if isinstance(waypoints[1], dict) else {}
        second_filming = second_wp.get("filmingProperty") if isinstance(second_wp.get("filmingProperty"), dict) else {}
        second_line_search = second_filming.get("lineSearch") if isinstance(second_filming.get("lineSearch"), dict) else {}
        if second_line_search or _to_int(second_filming.get("operationMode")) == 2:
            waypoints.pop(0)

    if not waypoints:
        return
    _align_waypoint_to_line_search_center(waypoints[0] if isinstance(waypoints[0], dict) else {})
    focus_coord = _focus_coordinate_from_waypoint(waypoints[0])
    entry_wp = _build_entry_waypoint(
        entry_coord=entry_coord,
        focus_coord=focus_coord,
        template_wp=waypoints[0] if isinstance(waypoints[0], dict) else None,
        waypoint_id=0,
    )
    waypoints.insert(0, entry_wp)
    for wp in waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = False
    reassign_unique_waypoint_ids_inplace(
        waypoints,
        waypoint_id_provider=waypoint_id_provider,
    )
    flight_plan["waypointList"] = waypoints
    if "lahWaypointList" in flight_plan:
        flight_plan["lahWaypointList"] = deepcopy(waypoints)


def _target_line_coords_and_width(
    target_input_mission: Dict[str, Any],
) -> tuple[List[Dict[str, Any]], float]:
    detail = (
        target_input_mission.get("missionDetail")
        if isinstance(target_input_mission.get("missionDetail"), dict)
        else {}
    )
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    for line in line_list:
        if not isinstance(line, dict):
            continue
        coords = _normalize_coord_list(line.get("coordinateList"))
        if len(coords) < 2:
            continue
        width = _to_float(line.get("width"))
        return coords, float(width if width is not None and width > 0.0 else 1.0)
    coords = _normalize_coord_list(detail.get("coordinateList"))
    return coords, 1.0


def _build_formation_reference_path_row(
    *,
    target_input_mission: Dict[str, Any],
    leader_aircraft_id: int,
    entry_coord: Dict[str, Any] | None = None,
    heading_deg: float | None = None,
) -> Dict[str, Any] | None:
    coords, width_m = _target_line_coords_and_width(target_input_mission)
    if len(coords) < 2:
        return None
    route_reversed = False
    normalized_entry = _normalize_coordinate(entry_coord)
    if normalized_entry is not None:
        forward_cost = _line_orientation_cost(
            entry_coord=normalized_entry,
            heading_deg=heading_deg,
            start_coord=coords[0],
            end_coord=coords[-1],
        )
        reverse_cost = _line_orientation_cost(
            entry_coord=normalized_entry,
            heading_deg=heading_deg,
            start_coord=coords[-1],
            end_coord=coords[0],
        )
        if reverse_cost + 1e-6 < forward_cost:
            coords = list(reversed(coords))
            route_reversed = True
    xy_rows: List[tuple[float, float]] = []
    for coord in coords:
        point_xy = coord_to_xy(coord)
        if point_xy is None:
            continue
        xy_rows.append((float(point_xy[0]), float(point_xy[1])))
    if len(xy_rows) < 2:
        return None
    bearing_deg = _bearing_from_coords(coords[0], coords[-1])
    return {
        "aircraftID": int(leader_aircraft_id),
        "pieceIndex": 1,
        "source": "formation_reference_route",
        "centerLineXY": list(xy_rows),
        "waypointStartXY": xy_rows[0],
        "waypointEndXY": xy_rows[-1],
        "targetXY": xy_rows[-1],
        "targetFaceXY": xy_rows[-1],
        "partWidthM": float(width_m),
        "sourceLineWidthM": float(width_m),
        "sourceCoordinateList": deepcopy(coords),
        "bearingDeg": float(bearing_deg if bearing_deg is not None else 0.0),
        "routeReversed": bool(route_reversed),
    }


def _formation_info_from_record(record: Dict[str, Any] | None) -> Dict[str, Any] | None:
    if not isinstance(record, dict):
        return None
    path = record.get("flightPath") if isinstance(record.get("flightPath"), dict) else {}
    info = path.get("formationInfo") if isinstance(path.get("formationInfo"), dict) else None
    if isinstance(info, dict):
        return deepcopy(info)
    return None


def _formation_leader_from_templates(
    *,
    target_aircraft_ids: List[int],
    template_record_map: Dict[int, List[Dict[str, Any]]],
) -> int:
    target_set = {int(aid) for aid in target_aircraft_ids if int(aid) > 0}
    for records in template_record_map.values():
        for record in records or []:
            info = _formation_info_from_record(record)
            if not isinstance(info, dict):
                continue
            leader_id = _to_int(info.get("leaderAircraftID"))
            if leader_id is not None and int(leader_id) in target_set:
                return int(leader_id)
    return min(target_set) if target_set else 4


def _template_record_for_aircraft(
    template_record_map: Dict[int, List[Dict[str, Any]]],
    aircraft_id: int,
) -> Dict[str, Any]:
    records = template_record_map.get(int(aircraft_id)) or []
    if records:
        return records[0] if isinstance(records[0], dict) else {}
    for fallback_aircraft_id in sorted(template_record_map):
        fallback_records = template_record_map.get(int(fallback_aircraft_id)) or []
        if fallback_records and isinstance(fallback_records[0], dict):
            return deepcopy(fallback_records[0])
    return {}


def _force_formation_mission_info(
    mission_info: Dict[str, Any],
    *,
    width_m: float,
) -> Dict[str, Any]:
    info = deepcopy(mission_info)
    info["individualMissionType"] = 7
    info["patternType"] = int(_to_int(info.get("patternType")) or 9)
    coords = _normalize_coord_list(info.get("coordinateList"))
    if not coords:
        for line in info.get("lineList") or []:
            if isinstance(line, dict):
                coords = _normalize_coord_list(line.get("coordinateList"))
                if coords:
                    break
    if coords:
        info["coordinateList"] = deepcopy(coords)
        info["lineList"] = [
            {
                "width": float(width_m),
                "coordinateList": deepcopy(coords),
            }
        ]
    info.setdefault("autoZoomIn", True)
    return info


def _prepare_formation_replacements(
    *,
    target_input_mission: Dict[str, Any],
    target_input_id: int,
    target_aircraft_ids: List[int],
    entry_coord_map: Dict[int, Dict[str, Any]],
    heading_map: Dict[int, float] | None,
    representative_entry: Dict[str, Any] | None,
    template_record_map: Dict[int, List[Dict[str, Any]]],
    now_ms: int,
    emit: Callable[[str], None],
    prepare_timer: Optional[_NextCollabPrepareTimer] = None,
    id_reservation_summaries: Optional[List[Dict[str, Any]]] = None,
) -> Optional[_PreparedReplacements]:
    leader_aircraft_id = _formation_leader_from_templates(
        target_aircraft_ids=target_aircraft_ids,
        template_record_map=template_record_map,
    )
    missing_entry_aircraft_ids = _missing_target_entry_aircraft_ids(target_aircraft_ids, entry_coord_map)
    if missing_entry_aircraft_ids:
        emit(
            "[NEXTCOLLAB][FORMATION] target aircraft entry coordinates missing; "
            f"aircraft={_format_aircraft_ids(missing_entry_aircraft_ids)}"
        )
        return None
    coords, width_m = _target_line_coords_and_width(target_input_mission)
    if len(coords) < 2:
        emit("[NEXTCOLLAB][FORMATION] target formation mission has no valid route line.")
        return None
    heading_by_aircraft = dict(heading_map or {})
    orientation_entry = (
        _normalize_coordinate(entry_coord_map.get(int(leader_aircraft_id)))
        or _normalize_coordinate(representative_entry)
    )
    orientation_heading = _to_float(heading_by_aircraft.get(int(leader_aircraft_id)))
    reference_row = _build_formation_reference_path_row(
        target_input_mission=target_input_mission,
        leader_aircraft_id=int(leader_aircraft_id),
        entry_coord=orientation_entry,
        heading_deg=orientation_heading,
    )
    if reference_row is None:
        emit("[NEXTCOLLAB][FORMATION] failed to build reference route row.")
        return None
    if prepare_timer is not None:
        prepare_timer.mark("formation_planner_run")

    aircraft_ids = sorted({int(aid) for aid in target_aircraft_ids if int(aid) > 0})
    if int(leader_aircraft_id) not in aircraft_ids:
        aircraft_ids.insert(0, int(leader_aircraft_id))
    path_rows_by_aircraft = {
        int(aircraft_id): [dict(reference_row, aircraftID=int(aircraft_id))]
        for aircraft_id in aircraft_ids
    }
    reservation = _reserve_next_collab_replacement_ids(
        path_rows_by_aircraft=path_rows_by_aircraft,
        emit=emit,
        id_reservation_summaries=id_reservation_summaries,
        scope="formation_replacements",
    )
    if reservation is None:
        emit("[NEXTCOLLAB][FORMATION] failed to reserve individualMissionIDs.")
        return None
    if prepare_timer is not None:
        prepare_timer.mark("id_reservation")
    individual_ids = [reservation.next_individual() for _aircraft_id in aircraft_ids]

    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    generated_fp_by_path: Dict[int, Dict[str, Any]] = {}
    generated_path_ids: Set[int] = set()
    flight_path_build_items: List[tuple[int, Callable[[], Dict[str, Any]]]] = []
    waypoint_id_provider = _ReplacementWaypointIdProvider(scope="FORMATION")
    bearing_deg = _to_float(reference_row.get("bearingDeg"))

    for idx, aircraft_id in enumerate(aircraft_ids):
        template_record = _template_record_for_aircraft(template_record_map, int(aircraft_id))
        template_mission = deepcopy(template_record.get("mission") or {})
        template_path = deepcopy(template_record.get("flightPath") or {})
        template_info = _template_mission_info(template_mission)
        path_row = dict(reference_row)
        path_row["aircraftID"] = int(aircraft_id)
        mission_info = build_mission_info_from_planned_row(
            path_row,
            template_info=template_info,
        )
        mission_info = _force_formation_mission_info(mission_info, width_m=float(width_m))
        related_mission = deepcopy(template_mission.get("relatedMission") or {})
        related_mission["relatedMissionType"] = _to_int(related_mission.get("relatedMissionType")) or 1
        related_mission["inputMissionID"] = int(target_input_id)
        related_mission["priorMissionID"] = _to_int(related_mission.get("priorMissionID")) or 0
        new_path_id = reservation.next_path(int(aircraft_id))
        generated_path_ids.add(int(new_path_id))
        new_mission_entry: Dict[str, Any] = {
            "individualMissionID": int(individual_ids[idx]),
            "isDone": False,
            "relatedMission": related_mission,
            "individualMissionInfo": mission_info,
            "pathID": int(new_path_id),
        }
        if bearing_deg is not None:
            new_mission_entry["bearing_deg"] = float(bearing_deg)
        replacement_by_aircraft.setdefault(int(aircraft_id), []).append(new_mission_entry)
        entry_coord = entry_coord_map.get(int(aircraft_id)) or representative_entry
        flight_path_build_items.append(
            (
                int(new_path_id),
                lambda *,
                template_path=template_path,
                mission_info=mission_info,
                individual_id=int(individual_ids[idx]),
                path_id=int(new_path_id),
                aircraft_id=int(aircraft_id),
                leader_aircraft_id=int(leader_aircraft_id),
                entry_coord=entry_coord,
                timestamp_ms=int(now_ms): build_formation_flight_path_from_template(
                    template_path=template_path,
                    mission_info=mission_info,
                    individual_mission_id=int(individual_id),
                    path_id=int(path_id),
                    aircraft_id=int(aircraft_id),
                    leader_aircraft_id=int(leader_aircraft_id),
                    entry_coord=entry_coord,
                    timestamp_ms=int(timestamp_ms),
                    source="MMR",
                    waypoint_id_provider=waypoint_id_provider,
                ),
            )
        )
    generated_fp_by_path = _build_replacement_flight_paths(
        flight_path_build_items,
        emit=emit,
        scope="FORMATION",
    )
    waypoint_summary = waypoint_id_provider.summary()
    if int(waypoint_summary.get("used") or 0) > 0:
        emit(
            "[NEXTCOLLAB][FORMATION] waypoint local block "
            f"used={int(waypoint_summary.get('used') or 0)} "
            f"reserved={int(waypoint_summary.get('reserved') or 0)} "
            f"blocks={int(waypoint_summary.get('blocks') or 0)}"
        )

    if not generated_fp_by_path:
        emit("[NEXTCOLLAB][FORMATION] no replacement flight paths prepared.")
        return None

    reservation_summary = _refresh_next_collab_id_reservation_summary(
        reservation=reservation,
        scope="formation_replacements",
        id_reservation_summaries=id_reservation_summaries,
    )
    review_report = {
        "enabled": True,
        "mode": "next_collab_formation_reference_route",
        "plannerWorkflow": "formation_reference_route",
        "changed": True,
        "leaderAircraftID": int(leader_aircraft_id),
        "targets": len(aircraft_ids),
        "routeCoordinateCount": len(coords),
        "routeReversed": bool(reference_row.get("routeReversed")),
        "generatedMissionCount": sum(len(rows) for rows in replacement_by_aircraft.values()),
        "generatedPathCount": len(generated_fp_by_path),
        "uavIndependentWork": {int(aid): len(rows) for aid, rows in path_rows_by_aircraft.items()},
        "idReservation": reservation_summary,
    }
    if prepare_timer is not None:
        prepare_timer.mark("replacement_mission_build")
        prepare_timer.mark("flight_path_build")
    emit(
        "[NEXTCOLLAB][FORMATION] reference route prepared "
        f"leader={int(leader_aircraft_id)} aircraft={','.join(str(aid) for aid in aircraft_ids)} "
        f"routePoints={len(coords)} reversed={bool(reference_row.get('routeReversed'))}"
    )
    return _PreparedReplacements(
        replacement_by_aircraft=replacement_by_aircraft,
        generated_fp_by_path=generated_fp_by_path,
        generated_path_ids=set(int(path_id) for path_id in generated_path_ids),
        planner_workflow="formation_reference_route",
        planner_result_text=(
            f"formation leader UAV{int(leader_aircraft_id)} -> "
            f"{','.join('UAV' + str(aid) for aid in aircraft_ids)}"
        ),
        planned_result_count=len(aircraft_ids),
        review_report=review_report,
        mission_mode="formation",
        timing_ms=prepare_timer.snapshot() if prepare_timer is not None else {},
        id_reservation=dict(reservation_summary),
        uav_work_summary={int(aid): len(rows) for aid, rows in path_rows_by_aircraft.items()},
    )


def _prepare_line_replacements(
    *,
    target_input_mission: Dict[str, Any],
    target_input_id: int,
    target_aircraft_ids: List[int],
    entry_coord_map: Dict[int, Dict[str, Any]],
    heading_map: Dict[int, float],
    entry_aircraft_context_map: Dict[int, Dict[str, Any]] | None,
    representative_entry: Dict[str, Any] | None,
    next_entry: Dict[str, Any] | None,
    template_map: Dict[int, List[Dict[str, Any]]],
    template_record_map: Dict[int, List[Dict[str, Any]]],
    now_ms: int,
    turn_radius_scale: float,
    search_speed_scale_multiplier: float = 1.0,
    emit: Callable[[str], None],
    prepare_timer: Optional[_NextCollabPrepareTimer] = None,
    id_reservation_summaries: Optional[List[Dict[str, Any]]] = None,
) -> Optional[_PreparedReplacements]:
    _ = template_map
    _ = next_entry

    missing_entry_aircraft_ids = _missing_target_entry_aircraft_ids(target_aircraft_ids, entry_coord_map)
    if missing_entry_aircraft_ids:
        emit(
            "[NEXTCOLLAB][LINE] target aircraft entry coordinates missing; "
            f"aircraft={_format_aircraft_ids(missing_entry_aircraft_ids)}"
        )
        return None

    aircraft_entries: List[Dict[str, Any]] = []
    for aircraft_id in sorted(int(aid) for aid in target_aircraft_ids):
        coord = _normalize_coordinate(entry_coord_map.get(int(aircraft_id)))
        if coord is None:
            continue
        context_row = (
            dict(entry_aircraft_context_map.get(int(aircraft_id)) or {})
            if isinstance(entry_aircraft_context_map, dict)
            else {}
        )
        row: Dict[str, Any] = dict(context_row)
        row["aircraftID"] = int(aircraft_id)
        row["coordinate"] = dict(coord)
        if int(aircraft_id) in heading_map:
            row["headingDeg"] = float(heading_map[int(aircraft_id)])
        aircraft_entries.append(row)
    if not aircraft_entries:
        emit("[NEXTCOLLAB][LINE] no planner aircraft entries resolved.")
        return None

    try:
        planner_result = run_next_collab_line_plan(
            target_mission=target_input_mission,
            aircraft_entries=aircraft_entries,
            turn_radius_scale=float(turn_radius_scale),
            log=emit,
        )
    except Exception as exc:
        emit(f"[NEXTCOLLAB][LINE] planner failed: {exc}")
        return None
    if prepare_timer is not None:
        prepare_timer.mark("line_planner_run")

    final_path_rows = [
        dict(row)
        for row in planner_result.expected_paths
        if isinstance(row, dict)
    ]
    ordered_path_rows = sorted(
        [
            row
            for row in final_path_rows
            if (_to_int(row.get("aircraftID")) or 0) > 0
        ],
        key=lambda row: (
            int(_to_int(row.get("aircraftID")) or 0),
            int(_to_int(row.get("pieceIndex")) or 0),
            str(row.get("targetLabel", "") or ""),
            str(row.get("source", "") or ""),
        ),
    )
    if not ordered_path_rows:
        emit("[NEXTCOLLAB][LINE] planner returned no valid path rows.")
        return None
    _apply_search_speed_scale_multiplier_to_rows(
        ordered_path_rows,
        search_speed_scale_multiplier=search_speed_scale_multiplier,
        emit=emit,
        scope="LINE",
    )

    path_rows_by_aircraft = _group_next_collab_path_rows_by_aircraft(ordered_path_rows)
    dem_prewarm_summary = prewarm_dem_altitudes_for_path_rows(ordered_path_rows)
    if int(dem_prewarm_summary.get("uniquePairs") or 0) > 0:
        emit(
            "[NEXTCOLLAB][LINE] DEM prewarm "
            f"xy={int(dem_prewarm_summary.get('xyPoints') or 0)} "
            f"unique={int(dem_prewarm_summary.get('uniquePairs') or 0)} "
            f"elapsed={float(dem_prewarm_summary.get('elapsedMs') or 0.0):.1f} ms"
        )
    if prepare_timer is not None:
        prepare_timer.mark("dem_prewarm")
    reservation = _reserve_next_collab_replacement_ids(
        path_rows_by_aircraft=path_rows_by_aircraft,
        emit=emit,
        id_reservation_summaries=id_reservation_summaries,
        scope="line_replacements",
    )
    if reservation is None:
        emit("[NEXTCOLLAB][LINE] failed to reserve individualMissionIDs.")
        return None
    if prepare_timer is not None:
        prepare_timer.mark("id_reservation")

    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    generated_fp_by_path: Dict[int, Dict[str, Any]] = {}
    generated_path_ids: Set[int] = set()
    template_cursor_by_aircraft: Dict[int, int] = {}
    flight_path_build_items: List[tuple[int, Callable[[], Dict[str, Any]]]] = []
    flight_path_metrics_rows: List[Dict[str, Any]] = []
    flight_path_metrics_lock = threading.Lock()
    waypoint_id_provider = _ReplacementWaypointIdProvider(scope="LINE")

    def _record_line_flight_path_metrics(metrics: Dict[str, Any]) -> None:
        with flight_path_metrics_lock:
            flight_path_metrics_rows.append(dict(metrics))

    for aircraft_id, aircraft_path_rows in path_rows_by_aircraft.items():
        for path_row in aircraft_path_rows:
            row_aircraft_id = _to_int(path_row.get("aircraftID")) or 0
            if row_aircraft_id <= 0:
                continue
            individual_id = reservation.next_individual()
            new_path_id = reservation.next_path(int(row_aircraft_id))
            template_records = template_record_map.get(int(row_aircraft_id)) or []
            template_idx = template_cursor_by_aircraft.get(int(row_aircraft_id), 0)
            template_record = (
                template_records[min(template_idx, max(len(template_records) - 1, 0))]
                if template_records
                else {}
            )
            template_cursor_by_aircraft[int(row_aircraft_id)] = template_idx + 1
            template_mission = deepcopy(template_record.get("mission") or {})
            template_path = deepcopy(template_record.get("flightPath") or {})
            template_info = _template_mission_info(template_mission)
            mission_info = build_mission_info_from_planned_row(
                path_row,
                template_info=template_info,
            )
            related_mission = deepcopy(template_mission.get("relatedMission") or {})
            related_mission["relatedMissionType"] = _to_int(related_mission.get("relatedMissionType")) or 1
            related_mission["inputMissionID"] = int(target_input_id)
            related_mission["priorMissionID"] = _to_int(related_mission.get("priorMissionID")) or 0
            generated_path_ids.add(int(new_path_id))
            new_mission_entry: Dict[str, Any] = {
                "individualMissionID": int(individual_id),
                "isDone": False,
                "relatedMission": related_mission,
                "individualMissionInfo": mission_info,
                "pathID": int(new_path_id),
            }
            bearing_deg = _to_float(path_row.get("bearingDeg"))
            if bearing_deg is not None:
                new_mission_entry["bearing_deg"] = float(bearing_deg)
            replacement_by_aircraft.setdefault(int(row_aircraft_id), []).append(new_mission_entry)
            entry_coord = entry_coord_map.get(int(row_aircraft_id)) or representative_entry
            flight_path_build_items.append(
                (
                    int(new_path_id),
                    lambda *,
                    path_row=path_row,
                    template_path=template_path,
                    mission_info=mission_info,
                    individual_id=int(individual_id),
                    path_id=int(new_path_id),
                    aircraft_id=int(row_aircraft_id),
                    entry_coord=entry_coord,
                    timestamp_ms=int(now_ms): build_flight_path_from_planned_row(
                        path_row,
                        template_path=template_path,
                        mission_info=mission_info,
                        individual_mission_id=int(individual_id),
                        path_id=int(path_id),
                        aircraft_id=int(aircraft_id),
                        entry_coord=entry_coord,
                        timestamp_ms=int(timestamp_ms),
                        source="MMR",
                        assign_waypoint_ids=False,
                        metrics_callback=_record_line_flight_path_metrics,
                    ),
                )
            )
    generated_fp_by_path = _build_replacement_flight_paths(
        flight_path_build_items,
        emit=emit,
        scope="LINE",
        min_workers=2,
    )
    _assign_replacement_waypoint_ids_in_order(
        generated_fp_by_path=generated_fp_by_path,
        ordered_path_ids=[int(path_id) for path_id, _ in flight_path_build_items],
        waypoint_id_provider=waypoint_id_provider,
        emit=emit,
        scope="LINE",
    )
    _emit_replacement_flight_path_metrics(
        flight_path_metrics_rows,
        emit=emit,
        scope="LINE",
    )
    waypoint_summary = waypoint_id_provider.summary()
    if int(waypoint_summary.get("used") or 0) > 0:
        emit(
            "[NEXTCOLLAB][LINE] waypoint local block "
            f"used={int(waypoint_summary.get('used') or 0)} "
            f"reserved={int(waypoint_summary.get('reserved') or 0)} "
            f"blocks={int(waypoint_summary.get('blocks') or 0)}"
        )
    if prepare_timer is not None:
        prepare_timer.mark("replacement_mission_build")
        prepare_timer.mark("flight_path_build")

    if not generated_fp_by_path:
        emit("[NEXTCOLLAB][LINE] no replacement flight paths prepared.")
        return None

    reservation_summary = _refresh_next_collab_id_reservation_summary(
        reservation=reservation,
        scope="line_replacements",
        id_reservation_summaries=id_reservation_summaries,
    )
    assignment_summary: Dict[int, int] = {}
    for piece in planner_result.split_result.pieces:
        aircraft_id = _to_int(piece.assigned_uav) or 0
        if aircraft_id <= 0:
            continue
        assignment_summary[int(aircraft_id)] = int(assignment_summary.get(int(aircraft_id), 0)) + 1
    review_report = {
        "enabled": True,
        "mode": "next_collab_line_prediction_path",
        "plannerWorkflow": str(planner_result.workflow),
        "changed": True,
        "targets": len(target_aircraft_ids),
        "oldPieceCount": len(planner_result.split_result.pieces),
        "newPieceCount": len(planner_result.split_result.pieces),
        "pathRowCount": len(ordered_path_rows),
        "generatedMissionCount": sum(len(rows) for rows in replacement_by_aircraft.values()),
        "assignmentSummary": {int(aid): int(count) for aid, count in sorted(assignment_summary.items())},
        "lineOverlayCount": len(planner_result.mid_line_segments),
        "uavIndependentWork": {int(aid): len(rows) for aid, rows in path_rows_by_aircraft.items()},
        "idReservation": reservation_summary,
    }
    return _PreparedReplacements(
        replacement_by_aircraft=replacement_by_aircraft,
        generated_fp_by_path=generated_fp_by_path,
        generated_path_ids=set(int(path_id) for path_id in generated_path_ids),
        planner_workflow=str(planner_result.workflow),
        planner_result_text=str(planner_result.planner_result_text or ""),
        planned_result_count=len(ordered_path_rows),
        review_report=review_report,
        mission_mode="line",
        timing_ms=prepare_timer.snapshot() if prepare_timer is not None else {},
        id_reservation=dict(reservation_summary),
        uav_work_summary={int(aid): len(rows) for aid, rows in path_rows_by_aircraft.items()},
        runtime_preservation={
            "turnRadiusScale": float(turn_radius_scale),
            "templateRecordCount": sum(len(rows) for rows in template_record_map.values()),
        },
    )


def _prepare_area_replacements(
    *,
    target_input_mission: Dict[str, Any],
    target_input_id: int,
    target_aircraft_ids: List[int],
    entry_coord_map: Dict[int, Dict[str, Any]],
    heading_map: Dict[int, float],
    representative_entry: Dict[str, Any] | None,
    template_record_map: Dict[int, List[Dict[str, Any]]],
    now_ms: int,
    turn_radius_scale: float,
    search_speed_scale_multiplier: float = 1.0,
    emit: Callable[[str], None],
    prepare_timer: Optional[_NextCollabPrepareTimer] = None,
    id_reservation_summaries: Optional[List[Dict[str, Any]]] = None,
) -> Optional[_PreparedReplacements]:
    missing_entry_aircraft_ids = _missing_target_entry_aircraft_ids(target_aircraft_ids, entry_coord_map)
    if missing_entry_aircraft_ids:
        emit(
            "[NEXTCOLLAB][AREA] target aircraft entry coordinates missing; "
            f"aircraft={_format_aircraft_ids(missing_entry_aircraft_ids)}"
        )
        return None

    aircraft_entries: List[Dict[str, Any]] = []
    for aircraft_id in target_aircraft_ids:
        entry_coord = entry_coord_map.get(int(aircraft_id))
        if entry_coord is None:
            continue
        row: Dict[str, Any] = {
            "aircraftID": int(aircraft_id),
            "coordinate": dict(entry_coord),
        }
        if int(aircraft_id) in heading_map:
            row["headingDeg"] = float(heading_map[int(aircraft_id)])
        aircraft_entries.append(row)
    if not aircraft_entries:
        emit("[NEXTCOLLAB] planner aircraft entries unresolved.")
        return None

    mission_detail = (
        target_input_mission.get("missionDetail")
        if isinstance(target_input_mission.get("missionDetail"), dict)
        else {}
    )
    area_components = _area_planner_components_from_detail(mission_detail)
    if not area_components:
        component_summary = _area_planner_component_input_summary(mission_detail)
        emit(
            "[NEXTCOLLAB] area replacement requires valid planner components "
            f"(reason={component_summary.get('reason')}, "
            f"segments={component_summary.get('areaSegmentCount')}, "
            f"outer={component_summary.get('areaOuterCount')}, "
            f"holes={component_summary.get('areaHoleCount')}, "
            f"maxComponents={component_summary.get('maxComponents')})."
        )
        return None

    planner_results: List[tuple[Dict[str, Any], Any]] = []
    for component in area_components:
        component_index = int(component.get("componentIndex") or len(planner_results) + 1)
        mission_polygon = _normalize_coord_list(component.get("coordinateList"))
        if len(mission_polygon) < 3:
            emit(f"[NEXTCOLLAB] area component {component_index} has invalid polygon.")
            return None
        try:
            planner_result = run_next_collab_division_plan(
                mission_polygon=mission_polygon,
                aircraft_entries=aircraft_entries,
                turn_radius_scale=float(turn_radius_scale),
                log=emit,
            )
        except Exception as exc:
            emit(f"[NEXTCOLLAB] division planner failed for area component {component_index}: {exc}")
            return None
        planner_results.append((dict(component), planner_result))
    if len(planner_results) > 1:
        emit(
            "[NEXTCOLLAB][AREA] multi-component remaining area planned "
            f"(components={len(planner_results)})."
        )
    if prepare_timer is not None:
        prepare_timer.mark("area_planner_run")

    final_path_rows: List[Dict[str, Any]] = []
    for component, planner_result in planner_results:
        component_index = int(component.get("componentIndex") or 0)
        for row in planner_result.expected_paths:
            if not isinstance(row, dict):
                continue
            row_copy = dict(row)
            row_copy["areaComponentIndex"] = int(component_index)
            final_path_rows.append(row_copy)
    if not final_path_rows:
        emit("[NEXTCOLLAB] division planner returned no final path rows.")
        return None

    piece_polygon_map: Dict[tuple[int, int, int], List[Dict[str, Any]]] = {}
    total_piece_count = 0
    total_mid_line_count = 0
    component_workflows: List[str] = []
    planner_result_text_parts: List[str] = []
    for component, planner_result in planner_results:
        component_index = int(component.get("componentIndex") or 0)
        component_workflows.append(str(planner_result.workflow))
        total_piece_count += len(planner_result.split_result.pieces)
        total_mid_line_count += len(planner_result.mid_line_segments)
        if str(planner_result.planner_result_text or ""):
            planner_result_text_parts.append(
                f"[component {component_index}] {planner_result.planner_result_text}"
            )
        for piece in planner_result.split_result.pieces:
            if not isinstance(piece, SplitPiece):
                continue
            aircraft_id = _to_int(piece.assigned_uav) or 0
            piece_index = _to_int(piece.piece_index) or 0
            if aircraft_id <= 0 or piece_index <= 0:
                continue
            coords = _normalize_coord_list((piece.data or {}).get("coordinateList"))
            if coords:
                piece_polygon_map[(int(component_index), int(aircraft_id), int(piece_index))] = coords
    planner_workflow = (
        f"multi_component:{','.join(component_workflows)}"
        if len(planner_results) > 1
        else str(component_workflows[0] if component_workflows else "")
    )
    planner_result_text = "\n".join(planner_result_text_parts)

    review_report: Dict[str, Any] = {
        "enabled": True,
        "mode": REPLAN_FLOW_MODE,
        "plannerWorkflow": str(planner_workflow),
        "changed": True,
        "operatorDecision": {
            "category": "planner_redivision",
            "reason": (
                "multi_component_area_division_planner_generated_replacements"
                if len(planner_results) > 1
                else "area_division_planner_generated_replacements"
            ),
            "detail": (
                "Remaining area components were divided by the area planner for the target UAV set."
                if len(planner_results) > 1
                else "Remaining area was divided by the area planner for the target UAV set."
            ),
        },
        "areaPlannerComponentCount": len(planner_results),
        "areaPlannerComponents": [
            {
                "componentIndex": int(component.get("componentIndex") or 0),
                "componentSource": str(component.get("componentSource") or ""),
                "componentDecomposition": str(component.get("componentDecomposition") or ""),
                "areaM2": float(component.get("areaM2") or 0.0),
                "coordinatePointCount": len(component.get("coordinateList") or []),
            }
            for component, _planner_result in planner_results
        ],
        "overflowRows": 0,
        "targets": len(target_aircraft_ids),
        "oldPieceCount": int(total_piece_count),
        "newPieceCount": int(total_piece_count),
        "pathRowCount": len(final_path_rows),
        "lineOverlayCount": int(total_mid_line_count),
        "details": [],
    }

    ordered_path_rows = sorted(
        [
            row
            for row in final_path_rows
            if (_to_int(row.get("aircraftID")) or 0) > 0
        ],
        key=lambda row: (
            int(_to_int(row.get("aircraftID")) or 0),
            int(_to_int(row.get("areaComponentIndex")) or 0),
            int(_to_int(row.get("pieceIndex")) or 0),
            int(_next_collab_area_path_row_phase_rank(row)),
            str(row.get("source", "") or ""),
            str(row.get("targetLabel", "") or ""),
        ),
    )
    if not ordered_path_rows:
        emit("[NEXTCOLLAB] division planner returned no valid area path rows.")
        return None
    _apply_search_speed_scale_multiplier_to_rows(
        ordered_path_rows,
        search_speed_scale_multiplier=search_speed_scale_multiplier,
        emit=emit,
        scope="AREA",
    )
    review_report["generatedMissionCount"] = len(ordered_path_rows)

    path_rows_by_aircraft = _group_next_collab_path_rows_by_aircraft(ordered_path_rows)
    dem_prewarm_summary = prewarm_dem_altitudes_for_path_rows(ordered_path_rows)
    if int(dem_prewarm_summary.get("uniquePairs") or 0) > 0:
        emit(
            "[NEXTCOLLAB][AREA] DEM prewarm "
            f"xy={int(dem_prewarm_summary.get('xyPoints') or 0)} "
            f"unique={int(dem_prewarm_summary.get('uniquePairs') or 0)} "
            f"elapsed={float(dem_prewarm_summary.get('elapsedMs') or 0.0):.1f} ms"
        )
    if prepare_timer is not None:
        prepare_timer.mark("dem_prewarm")
    reservation = _reserve_next_collab_replacement_ids(
        path_rows_by_aircraft=path_rows_by_aircraft,
        emit=emit,
        id_reservation_summaries=id_reservation_summaries,
        scope="area_replacements",
    )
    if reservation is None:
        emit("[NEXTCOLLAB] failed to reserve individualMissionIDs.")
        return None
    if prepare_timer is not None:
        prepare_timer.mark("id_reservation")

    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    generated_fp_by_path: Dict[int, Dict[str, Any]] = {}
    generated_path_ids: Set[int] = set()
    template_cursor_by_aircraft: Dict[int, int] = {}
    flight_path_build_items: List[tuple[int, Callable[[], Dict[str, Any]]]] = []
    flight_path_metrics: List[Dict[str, Any]] = []
    flight_path_metrics_lock = threading.Lock()
    waypoint_id_provider = _ReplacementWaypointIdProvider(scope="AREA")

    def _record_flight_path_metrics(metrics: Dict[str, Any]) -> None:
        with flight_path_metrics_lock:
            flight_path_metrics.append(dict(metrics))

    for aircraft_id, aircraft_path_rows in path_rows_by_aircraft.items():
        for path_row in aircraft_path_rows:
            row_aircraft_id = _to_int(path_row.get("aircraftID")) or 0
            if row_aircraft_id <= 0:
                continue
            individual_id = reservation.next_individual()
            new_path_id = reservation.next_path(int(row_aircraft_id))
            template_records = template_record_map.get(int(row_aircraft_id)) or []
            template_idx = template_cursor_by_aircraft.get(int(row_aircraft_id), 0)
            template_record = (
                template_records[min(template_idx, max(len(template_records) - 1, 0))]
                if template_records
                else {}
            )
            template_cursor_by_aircraft[int(row_aircraft_id)] = template_idx + 1
            template_mission = deepcopy(template_record.get("mission") or {})
            template_path = deepcopy(template_record.get("flightPath") or {})
            template_info = _template_mission_info(template_mission)
            component_index = _to_int(path_row.get("areaComponentIndex")) or 0
            piece_index = _to_int(path_row.get("pieceIndex")) or 0
            mission_info = build_mission_info_from_planned_row(
                path_row,
                template_info=template_info,
                fallback_polygon_coords=piece_polygon_map.get(
                    (int(component_index), int(row_aircraft_id), int(piece_index))
                ) or [],
            )
            related_mission = deepcopy(template_mission.get("relatedMission") or {})
            related_mission["relatedMissionType"] = _to_int(related_mission.get("relatedMissionType")) or 1
            related_mission["inputMissionID"] = int(target_input_id)
            related_mission["priorMissionID"] = _to_int(related_mission.get("priorMissionID")) or 0
            generated_path_ids.add(int(new_path_id))
            new_mission_entry: Dict[str, Any] = {
                "individualMissionID": int(individual_id),
                "isDone": False,
                "relatedMission": related_mission,
                "individualMissionInfo": mission_info,
                "pathID": int(new_path_id),
            }
            bearing_deg = _to_float(path_row.get("bearingDeg"))
            if bearing_deg is not None:
                new_mission_entry["bearing_deg"] = float(bearing_deg)
            replacement_by_aircraft.setdefault(int(row_aircraft_id), []).append(new_mission_entry)
            entry_coord = entry_coord_map.get(int(row_aircraft_id)) or representative_entry
            flight_path_build_items.append(
                (
                    int(new_path_id),
                    lambda *,
                    path_row=path_row,
                    template_path=template_path,
                    mission_info=mission_info,
                    individual_id=int(individual_id),
                    path_id=int(new_path_id),
                    aircraft_id=int(row_aircraft_id),
                    entry_coord=entry_coord,
                    timestamp_ms=int(now_ms): build_flight_path_from_planned_row(
                        path_row,
                        template_path=template_path,
                        mission_info=mission_info,
                        individual_mission_id=int(individual_id),
                        path_id=int(path_id),
                        aircraft_id=int(aircraft_id),
                        entry_coord=entry_coord,
                        timestamp_ms=int(timestamp_ms),
                        source="MMR",
                        assign_waypoint_ids=False,
                        metrics_callback=_record_flight_path_metrics,
                    ),
                )
            )
    generated_fp_by_path = _build_replacement_flight_paths(
        flight_path_build_items,
        emit=emit,
        scope="AREA",
        min_workers=2,
    )
    _assign_replacement_waypoint_ids_in_order(
        generated_fp_by_path=generated_fp_by_path,
        ordered_path_ids=[int(path_id) for path_id, _ in flight_path_build_items],
        waypoint_id_provider=waypoint_id_provider,
        emit=emit,
        scope="AREA",
    )
    _emit_replacement_flight_path_metrics(
        flight_path_metrics,
        emit=emit,
        scope="AREA",
    )
    waypoint_summary = waypoint_id_provider.summary()
    if int(waypoint_summary.get("used") or 0) > 0:
        emit(
            "[NEXTCOLLAB][AREA] waypoint local block "
            f"used={int(waypoint_summary.get('used') or 0)} "
            f"reserved={int(waypoint_summary.get('reserved') or 0)} "
            f"blocks={int(waypoint_summary.get('blocks') or 0)}"
        )
    if prepare_timer is not None:
        prepare_timer.mark("replacement_mission_build")
        prepare_timer.mark("flight_path_build")

    if not generated_fp_by_path:
        emit("[NEXTCOLLAB] no replacement flight paths prepared.")
        return None

    reservation_summary = _refresh_next_collab_id_reservation_summary(
        reservation=reservation,
        scope="area_replacements",
        id_reservation_summaries=id_reservation_summaries,
    )
    review_report["uavIndependentWork"] = {
        int(aid): len(rows) for aid, rows in path_rows_by_aircraft.items()
    }
    review_report["idReservation"] = reservation_summary
    return _PreparedReplacements(
        replacement_by_aircraft=replacement_by_aircraft,
        generated_fp_by_path=generated_fp_by_path,
        generated_path_ids=set(int(path_id) for path_id in generated_path_ids),
        planner_workflow=str(planner_workflow),
        planner_result_text=str(planner_result_text or ""),
        planned_result_count=len(ordered_path_rows),
        review_report=review_report,
        mission_mode="area",
        timing_ms=prepare_timer.snapshot() if prepare_timer is not None else {},
        id_reservation=dict(reservation_summary),
        uav_work_summary={int(aid): len(rows) for aid, rows in path_rows_by_aircraft.items()},
        runtime_preservation={
            "turnRadiusScale": float(turn_radius_scale),
            "templateRecordCount": sum(len(rows) for rows in template_record_map.values()),
        },
    )


def prepare_next_collab_input_replacements(
    *,
    source_plan_id: int,
    target_input_mission: Dict[str, Any],
    entry_coord_map: Dict[int, Dict[str, Any]],
    heading_map: Dict[int, float] | None = None,
    entry_aircraft_context_map: Dict[int, Dict[str, Any]] | None = None,
    representative_entry: Dict[str, Any] | None = None,
    next_input_mission: Dict[str, Any] | None = None,
    turn_radius_scale: float | None = None,
    search_speed_scale_multiplier: float | None = None,
    source_template_input_id: int | None = None,
    now_ms: int | None = None,
    log: Callable[[str], None] | None = None,
) -> Optional[_PreparedReplacements]:
    emit = log or (lambda _msg: None)
    prepare_timer = _NextCollabPrepareTimer()
    source_cache = _NextCollabSourceCache()
    id_reservation_summaries: List[Dict[str, Any]] = []
    target_input_id = _to_int(target_input_mission.get("inputMissionID"))
    if target_input_id is None or target_input_id <= 0:
        emit("[NEXTCOLLAB] target input mission has invalid inputMissionID.")
        return None
    if not entry_coord_map:
        emit("[NEXTCOLLAB] entry coordinate map is empty.")
        return None
    template_input_id = _to_int(source_template_input_id) or int(target_input_id)

    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = read_json_cached(plan_src, kind="MissionPlan")
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to load source MissionPlan {source_plan_id}: {exc}")
        return None

    source_input_pkg_id = _to_int(
        plan_data.get("inputMissionPackageID")
        or plan_data.get("InputMissionPackageID")
        or plan_data.get("inputMissionPackageId")
    )
    if source_input_pkg_id is None or source_input_pkg_id <= 0:
        emit("[NEXTCOLLAB] source MissionPlan missing inputMissionPackageID.")
        return None
    try:
        input_src = db_paths.get_db_subpath("InputMissionPlan", f"{int(source_input_pkg_id)}.json")
        input_data = read_json_cached(input_src, kind="InputMissionPlan")
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to load InputMissionPlan {source_input_pkg_id}: {exc}")
        return None

    aircraft_entries = [entry for entry in plan_data.get("aircraftList") or [] if isinstance(entry, dict)]
    if not aircraft_entries:
        emit("[NEXTCOLLAB] source MissionPlan has no aircraftList.")
        return None

    packages_by_aircraft: Dict[int, Dict[str, Any]] = {}
    for aircraft_entry in aircraft_entries:
        aircraft_id = _to_int(aircraft_entry.get("aircraftID"))
        source_imp_id = _to_int(
            aircraft_entry.get("individualMissionPackageID")
            or aircraft_entry.get("individualMissionPlanPackageID")
            or aircraft_entry.get("individualMissionPackageId")
        )
        if aircraft_id is None or aircraft_id <= 0 or source_imp_id is None or source_imp_id <= 0:
            continue
        try:
            imp_src = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(source_imp_id)}.json")
            imp_data = read_json_cached(imp_src, kind="IndividualMissionPlan")
        except Exception:
            continue
        packages_by_aircraft[int(aircraft_id)] = deepcopy(imp_data)
    prepare_timer.mark("source_load")

    template_map = _extract_templates_for_input(packages_by_aircraft, int(template_input_id))
    if not template_map:
        emit(
            f"[NEXTCOLLAB] no source templates found for inputMissionID={template_input_id}"
            f" (targetInputMissionID={target_input_id})."
        )
        return None

    resolved_heading_map = {
        int(aid): float(val)
        for aid, val in dict(heading_map or {}).items()
        if _to_int(aid) is not None and _to_float(val) is not None
    }
    resolved_entry = _normalize_coordinate(representative_entry)
    if resolved_entry is None:
        resolved_entry = _centroid_coordinate(
            [coord for coord in entry_coord_map.values() if _normalize_coordinate(coord) is not None]
        )
    if resolved_entry is None:
        resolved_entry = _mission_entry_point(target_input_mission)
    if resolved_entry is None:
        emit("[NEXTCOLLAB] representative entry coordinate unavailable.")
        return None
    prepare_timer.mark("target_input_resolve")

    target_aircraft_ids = _resolve_next_collab_target_aircraft_ids(
        entry_coord_map,
        template_map,
    )
    if not target_aircraft_ids:
        emit("[NEXTCOLLAB] no target aircraft IDs resolved.")
        return None
    prepare_timer.mark("entry_coordinate_resolve")

    template_record_map = _extract_template_records_for_input(
        packages_by_aircraft,
        int(template_input_id),
        source_cache=source_cache,
    )
    _ensure_target_template_records_for_aircraft(
        target_aircraft_ids=target_aircraft_ids,
        template_map=template_map,
        template_record_map=template_record_map,
        emit=emit,
    )
    effective_now_ms = int(now_ms if now_ms is not None else _now_ms_since_2000())
    effective_turn_radius_scale = _to_float(turn_radius_scale)
    if effective_turn_radius_scale is None or effective_turn_radius_scale <= 0.0:
        effective_turn_radius_scale = float(get_runtime_float("next_collab_turn_radius_scale", 1.2))
    effective_search_speed_scale_multiplier = _normalize_search_speed_scale_multiplier(
        search_speed_scale_multiplier
    )

    def _with_prepare_metadata(prepared: Optional[_PreparedReplacements]) -> Optional[_PreparedReplacements]:
        if prepared is None:
            return None
        prepared.source_cache = source_cache.summary()
        prepared.timing_ms = prepare_timer.snapshot()
        if id_reservation_summaries:
            prepared.id_reservation = {
                "latest": dict(prepared.id_reservation or id_reservation_summaries[-1]),
                "blocks": [dict(row) for row in id_reservation_summaries],
            }
        prepared.review_report = dict(prepared.review_report or {})
        prepared.review_report["prepareTimingMs"] = dict(prepared.timing_ms)
        prepared.review_report["sourceCache"] = dict(prepared.source_cache)
        prepared.review_report["idReservation"] = dict(prepared.id_reservation or {})
        prepared.review_report["uavIndependentWork"] = {
            int(aid): int(count) for aid, count in prepared.uav_work_summary.items()
        }
        runtime_preservation = _summarize_next_collab_runtime_preservation(
            template_record_map,
            turn_radius_scale=float(effective_turn_radius_scale),
        )
        runtime_preservation.update(dict(prepared.runtime_preservation or {}))
        prepared.runtime_preservation = runtime_preservation
        prepared.review_report["runtimePreservation"] = dict(runtime_preservation)
        return prepared

    if _is_formation_input_mission(target_input_mission):
        return _with_prepare_metadata(_prepare_formation_replacements(
            target_input_mission=target_input_mission,
            target_input_id=int(target_input_id),
            target_aircraft_ids=target_aircraft_ids,
            entry_coord_map=entry_coord_map,
            heading_map=resolved_heading_map,
            representative_entry=resolved_entry,
            template_record_map=template_record_map,
            now_ms=effective_now_ms,
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
        ))

    if _is_line_input_mission(target_input_mission):
        return _with_prepare_metadata(_prepare_line_replacements(
            target_input_mission=target_input_mission,
            target_input_id=int(target_input_id),
            target_aircraft_ids=target_aircraft_ids,
            entry_coord_map=entry_coord_map,
            heading_map=resolved_heading_map,
            entry_aircraft_context_map=entry_aircraft_context_map,
            representative_entry=resolved_entry,
            next_entry=_mission_entry_point(next_input_mission) if isinstance(next_input_mission, dict) else None,
            template_map=template_map,
            template_record_map=template_record_map,
            now_ms=effective_now_ms,
            turn_radius_scale=float(effective_turn_radius_scale),
            search_speed_scale_multiplier=float(effective_search_speed_scale_multiplier),
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
        ))

    return _with_prepare_metadata(_prepare_area_replacements(
        target_input_mission=target_input_mission,
        target_input_id=int(target_input_id),
        target_aircraft_ids=target_aircraft_ids,
        entry_coord_map=entry_coord_map,
        heading_map=resolved_heading_map,
        representative_entry=resolved_entry,
        template_record_map=template_record_map,
        now_ms=effective_now_ms,
        turn_radius_scale=float(effective_turn_radius_scale),
        search_speed_scale_multiplier=float(effective_search_speed_scale_multiplier),
        emit=emit,
        prepare_timer=prepare_timer,
        id_reservation_summaries=id_reservation_summaries,
    ))


def run_next_collab_replan_pipeline(
    ctx: Dict[str, Any],
    detail: Dict[str, Any],
    reason: str,
    *,
    log: Callable[[str], None],
) -> Optional[NextCollabPipelineResult]:
    log_messages: List[str] = []
    transaction_id = new_replan_transaction_id("next-collab")
    phase_timer = PipelinePhaseTimer(
        pipeline="next_collab",
        replan_transaction_id=transaction_id,
        emit_events=True,
    )
    prepare_timer = _NextCollabPrepareTimer()
    source_cache = _NextCollabSourceCache()
    id_reservation_summaries: List[Dict[str, Any]] = []

    def emit(message: str) -> None:
        log_messages.append(message)
        log(message)

    plan_ids_raw = list(ctx.get("plan_ids") or [])
    try:
        plan_ids = [int(value) for value in plan_ids_raw if value is not None]
    except Exception:
        emit(f"[NEXTCOLLAB] invalid plan_ids in context: {plan_ids_raw!r}")
        return None
    if len(plan_ids) != 1:
        emit(f"[NEXTCOLLAB] expected exactly one pending missionPlanID, got {len(plan_ids)}.")
        return None

    stored_detail = next_collab_replan_store.load_detail(int(plan_ids[0])) if plan_ids else None
    if not isinstance(detail, dict) or not detail:
        detail = stored_detail or {}
    elif stored_detail:
        merged_detail = dict(stored_detail)
        merged_detail.update(detail)
        detail = merged_detail
    if not isinstance(detail, dict) or not detail:
        emit("[NEXTCOLLAB] replanDetail missing and store lookup failed.")
        return None

    source_plan_id = _to_int(detail.get("sourceMissionPlanID"))
    current_input_id = _to_int(detail.get("currentInputMissionID"))
    target_input_id = _to_int(detail.get("targetInputMissionID"))
    if source_plan_id is None or source_plan_id <= 0:
        emit("[NEXTCOLLAB] sourceMissionPlanID missing.")
        return None
    if current_input_id is None or current_input_id <= 0:
        emit("[NEXTCOLLAB] currentInputMissionID missing.")
        return None
    if target_input_id is None or target_input_id <= 0:
        emit("[NEXTCOLLAB] targetInputMissionID missing.")
        return None

    entry_coord_map = _extract_entry_coordinate_map(detail)
    if not entry_coord_map:
        emit("[NEXTCOLLAB] entryAircraftList missing/empty.")
        return None
    representative_entry = _centroid_coordinate(list(entry_coord_map.values()))
    if representative_entry is not None:
        detail = dict(detail)
        detail["representativeEntryCoordinate"] = dict(representative_entry)
    entry_aircraft_context_map = _extract_entry_aircraft_context_map(detail)

    new_plan_id = int(plan_ids[0])
    option_names = _ensure_option_names([new_plan_id], ctx.get("option_names"))
    now_ms = _now_ms_since_2000()

    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = read_json_cached(plan_src, kind="MissionPlan")
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to load source MissionPlan {source_plan_id}: {exc}")
        return None

    source_input_pkg_id = _to_int(
        plan_data.get("inputMissionPackageID")
        or plan_data.get("InputMissionPackageID")
        or plan_data.get("inputMissionPackageId")
    )
    if source_input_pkg_id is None or source_input_pkg_id <= 0:
        emit("[NEXTCOLLAB] source MissionPlan missing inputMissionPackageID.")
        return None
    try:
        input_src = db_paths.get_db_subpath("InputMissionPlan", f"{int(source_input_pkg_id)}.json")
        input_data = read_json_cached(input_src, kind="InputMissionPlan")
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to load InputMissionPlan {source_input_pkg_id}: {exc}")
        return None

    mrpk_id = _to_int(
        plan_data.get("missionReferencePackageID")
        or plan_data.get("MissionReferencePackageID")
        or plan_data.get("missionReferencePackageId")
    )
    mrpk_data: Dict[str, Any] = {}
    candidate_mrpk_ids: List[int] = []
    if mrpk_id is not None:
        candidate_mrpk_ids.append(int(mrpk_id))
    if 0 not in candidate_mrpk_ids:
        candidate_mrpk_ids.append(0)
    for candidate_id in candidate_mrpk_ids:
        try:
            mrpk_path = db_paths.get_db_subpath("MissionReferenceInfo", f"{int(candidate_id)}.json")
            if not mrpk_path.exists():
                continue
            mrpk_data = read_json_cached(mrpk_path, kind="MissionReferenceInfo")
            break
        except Exception:
            mrpk_data = {}

    target_input_mission = _find_input_mission(input_data, int(target_input_id))
    if not isinstance(target_input_mission, dict):
        emit(f"[NEXTCOLLAB] target input mission {target_input_id} not found in InputMissionPlan.")
        return None
    phase_timer.mark("load_source")
    prepare_timer.mark("source_load")

    new_plan_data = deepcopy(plan_data)
    new_plan_data["missionPlanID"] = int(new_plan_id)
    new_plan_data["timestamp"] = int(now_ms)
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = int(now_ms)
    new_input_pkg_id = int(source_input_pkg_id)
    new_plan_data["inputMissionPackageID"] = int(new_input_pkg_id)
    _set_source_field(new_plan_data, "MMR")

    aircraft_entries = [entry for entry in new_plan_data.get("aircraftList") or [] if isinstance(entry, dict)]
    if not aircraft_entries:
        emit("[NEXTCOLLAB] source MissionPlan has no aircraftList.")
        return None

    imp_reservation = ReplanIdReservation.reserve(imp_count=len(aircraft_entries))
    new_imp_ids = [imp_reservation.next_imp() for _entry in aircraft_entries]
    id_reservation_summaries.append(
        {
            "scope": "input_package_clone",
            **imp_reservation.summary(),
        }
    )
    if len(new_imp_ids) != len(aircraft_entries):
        emit("[NEXTCOLLAB] failed to reserve IMP package IDs.")
        return None

    packages_by_aircraft: Dict[int, Dict[str, Any]] = {}
    generated_imp_ids: Set[int] = set()
    for idx, aircraft_entry in enumerate(aircraft_entries):
        aircraft_id = _to_int(aircraft_entry.get("aircraftID"))
        source_imp_id = _to_int(
            aircraft_entry.get("individualMissionPackageID")
            or aircraft_entry.get("individualMissionPlanPackageID")
            or aircraft_entry.get("individualMissionPackageId")
        )
        if aircraft_id is None or aircraft_id <= 0 or source_imp_id is None or source_imp_id <= 0:
            emit("[NEXTCOLLAB] MissionPlan aircraft entry missing aircraftID/individualMissionPackageID.")
            return None
        try:
            imp_src = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(source_imp_id)}.json")
            imp_data = read_json_cached(imp_src, kind="IndividualMissionPlan")
        except Exception as exc:
            emit(f"[NEXTCOLLAB] failed to load IndividualMissionPlan {source_imp_id}: {exc}")
            return None
        new_imp_id = int(new_imp_ids[idx])
        new_imp_data = deepcopy(imp_data)
        new_imp_data["individualMissionPackageID"] = int(new_imp_id)
        new_imp_data["timestamp"] = int(now_ms)
        _set_source_field(new_imp_data, "MMR")
        aircraft_entry["individualMissionPackageID"] = int(new_imp_id)
        packages_by_aircraft[int(aircraft_id)] = new_imp_data
        generated_imp_ids.add(int(new_imp_id))

    template_map = _extract_target_templates(packages_by_aircraft, int(target_input_id))
    if not template_map:
        emit(f"[NEXTCOLLAB] no target individual missions found for inputMissionID={target_input_id}.")
        return None

    takeover_list = mrpk_data.get("takeOverInfoList") if isinstance(mrpk_data.get("takeOverInfoList"), list) else []
    if not takeover_list:
        mrpk_data = deepcopy(mrpk_data) if isinstance(mrpk_data, dict) else {}
        mrpk_data["takeOverInfoList"] = _build_takeover_info_list(entry_coord_map)

    target_aircraft_ids = _resolve_next_collab_target_aircraft_ids(
        entry_coord_map,
        template_map,
    )
    if not target_aircraft_ids:
        emit("[NEXTCOLLAB] no target aircraft IDs resolved.")
        return None

    representative_entry = _normalize_coordinate(detail.get("representativeEntryCoordinate"))
    if representative_entry is None:
        representative_entry = _centroid_coordinate(list(entry_coord_map.values()))
    if representative_entry is None:
        emit("[NEXTCOLLAB] representative entry coordinate unavailable.")
        return None
    prepare_timer.mark("target_input_resolve")

    next_entry = _find_next_input_entry(input_data, int(target_input_id))
    heading_map = _extract_entry_heading_map(detail)
    template_record_map = _extract_target_template_records(
        packages_by_aircraft,
        int(target_input_id),
        source_cache=source_cache,
    )
    _ensure_target_template_records_for_aircraft(
        target_aircraft_ids=target_aircraft_ids,
        template_map=template_map,
        template_record_map=template_record_map,
        emit=emit,
    )
    for aircraft_id, pkg in packages_by_aircraft.items():
        mission_list = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        for mission in mission_list:
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) == int(current_input_id):
                mission["isDone"] = True
    planner_aircraft_entries: List[Dict[str, Any]] = []
    missing_entry_aircraft_ids = _missing_target_entry_aircraft_ids(target_aircraft_ids, entry_coord_map)
    if missing_entry_aircraft_ids:
        emit(
            "[NEXTCOLLAB] target aircraft entry coordinates missing; "
            f"aircraft={_format_aircraft_ids(missing_entry_aircraft_ids)}"
        )
        return None
    for aircraft_id in target_aircraft_ids:
        entry_coord = entry_coord_map.get(int(aircraft_id))
        if entry_coord is None:
            continue
        row: Dict[str, Any] = {
            "aircraftID": int(aircraft_id),
            "coordinate": dict(entry_coord),
        }
        if int(aircraft_id) in heading_map:
            row["headingDeg"] = float(heading_map[int(aircraft_id)])
        planner_aircraft_entries.append(row)
    if not planner_aircraft_entries:
        emit("[NEXTCOLLAB] planner aircraft entries unresolved.")
        return None
    prepare_timer.mark("entry_coordinate_resolve")

    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    generated_fp_by_path: Dict[int, Dict[str, Any]] = {}
    generated_path_ids: Set[int] = set()
    planner_workflow = ""
    planner_result_text = ""
    planned_result_count = 0
    area_review_report: Dict[str, Any] = {}
    turn_radius_scale = _to_float(detail.get("turnRadiusScale"))
    if turn_radius_scale is None or turn_radius_scale <= 0.0:
        turn_radius_scale = float(get_runtime_float("next_collab_turn_radius_scale", 1.2))
    prepared: Optional[_PreparedReplacements] = None
    if _is_formation_input_mission(target_input_mission):
        prepared = _prepare_formation_replacements(
            target_input_mission=target_input_mission,
            target_input_id=int(target_input_id),
            target_aircraft_ids=target_aircraft_ids,
            entry_coord_map=entry_coord_map,
            heading_map=heading_map,
            representative_entry=representative_entry,
            template_record_map=template_record_map,
            now_ms=int(now_ms),
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
        )
        if prepared is None:
            return None
        replacement_by_aircraft = {
            int(aid): list(rows)
            for aid, rows in prepared.replacement_by_aircraft.items()
        }
        generated_fp_by_path = {
            int(path_id): payload
            for path_id, payload in prepared.generated_fp_by_path.items()
        }
        generated_path_ids = set(int(path_id) for path_id in prepared.generated_path_ids)
        planner_workflow = str(prepared.planner_workflow)
        planner_result_text = str(prepared.planner_result_text or "")
        planned_result_count = int(prepared.planned_result_count)
        area_review_report = dict(prepared.review_report)
    elif _is_line_input_mission(target_input_mission):
        prepared = _prepare_line_replacements(
            target_input_mission=target_input_mission,
            target_input_id=int(target_input_id),
            target_aircraft_ids=target_aircraft_ids,
            entry_coord_map=entry_coord_map,
            heading_map=heading_map,
            entry_aircraft_context_map=entry_aircraft_context_map,
            representative_entry=representative_entry,
            next_entry=next_entry,
            template_map=template_map,
            template_record_map=template_record_map,
            now_ms=int(now_ms),
            turn_radius_scale=float(turn_radius_scale),
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
        )
        if prepared is None:
            return None
        replacement_by_aircraft = {
            int(aid): list(rows)
            for aid, rows in prepared.replacement_by_aircraft.items()
        }
        generated_fp_by_path = {
            int(path_id): payload
            for path_id, payload in prepared.generated_fp_by_path.items()
        }
        generated_path_ids = set(int(path_id) for path_id in prepared.generated_path_ids)
        planner_workflow = str(prepared.planner_workflow)
        planner_result_text = str(prepared.planner_result_text or "")
        planned_result_count = int(prepared.planned_result_count)
        area_review_report = dict(prepared.review_report)
    else:
        prepared = _prepare_area_replacements(
            target_input_mission=target_input_mission,
            target_input_id=int(target_input_id),
            target_aircraft_ids=target_aircraft_ids,
            entry_coord_map=entry_coord_map,
            heading_map=heading_map,
            representative_entry=representative_entry,
            template_record_map=template_record_map,
            now_ms=int(now_ms),
            turn_radius_scale=float(turn_radius_scale),
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
        )
        if prepared is None:
            return None
        replacement_by_aircraft = {
            int(aid): list(rows)
            for aid, rows in prepared.replacement_by_aircraft.items()
        }
        generated_fp_by_path = {
            int(path_id): payload
            for path_id, payload in prepared.generated_fp_by_path.items()
        }
        generated_path_ids = set(int(path_id) for path_id in prepared.generated_path_ids)
        planner_workflow = str(prepared.planner_workflow)
        planner_result_text = str(prepared.planner_result_text or "")
        planned_result_count = int(prepared.planned_result_count)
        area_review_report = dict(prepared.review_report)

    mission_mode = str(prepared.mission_mode or "")
    prepare_timings_ms = dict(prepared.timing_ms or prepare_timer.snapshot())
    source_cache_summary = source_cache.summary()
    id_reservation_summary: Dict[str, Any] = {
        "latest": dict(prepared.id_reservation or {}),
        "blocks": [dict(row) for row in id_reservation_summaries],
    }
    uav_work_summary = {
        int(aid): int(count)
        for aid, count in dict(prepared.uav_work_summary or {}).items()
    }
    runtime_preservation = _summarize_next_collab_runtime_preservation(
        template_record_map,
        turn_radius_scale=float(turn_radius_scale),
    )
    runtime_preservation.update(dict(prepared.runtime_preservation or {}))
    area_review_report["missionMode"] = mission_mode
    area_review_report["prepareTimingMs"] = dict(prepare_timings_ms)
    area_review_report["sourceCache"] = dict(source_cache_summary)
    area_review_report["idReservation"] = dict(id_reservation_summary)
    area_review_report["uavIndependentWork"] = dict(uav_work_summary)
    area_review_report["runtimePreservation"] = dict(runtime_preservation)

    phase_timer.mark("prepare_replacements")

    missing_replacement_aircraft_ids = sorted(
        int(aid)
        for aid in target_aircraft_ids
        if int(aid) > 0 and not replacement_by_aircraft.get(int(aid))
    )
    if missing_replacement_aircraft_ids:
        area_review_report["missingReplacementAircraftIDs"] = list(missing_replacement_aircraft_ids)
        emit(
            "[NEXTCOLLAB][WARN] partial replacements; preserving original target missions "
            f"for aircraft={missing_replacement_aircraft_ids} inputMissionID={int(target_input_id)}"
        )

    preserve_current_input_for_takeover = bool(
        mission_mode == "area" and _is_piece_only_area_takeover_input(target_input_mission)
    )
    if preserve_current_input_for_takeover:
        area_review_report["areaAssignmentPolicy"] = "piece_only_takeover_preserve_existing"
        area_review_report["areaTakeoverSourceAircraftIDs"] = [
            int(aid)
            for aid in (target_input_mission.get("areaTakeoverSourceAircraftIDs") or [])
            if _to_int(aid) is not None
        ]
        area_review_report["operatorDecision"] = {
            "category": "preserved_assignment",
            "reason": "piece_only_takeover",
            "detail": (
                "Existing current area missions are preserved; generated replacement "
                "missions cover only the unavailable UAV remaining piece."
            ),
        }
        emit(
            "[NEXTCOLLAB][AREA] piece-only takeover: preserving existing current "
            f"inputMissionID={int(target_input_id)} and inserting takeover piece replacements."
        )

    removed_completed_by_aircraft: Dict[int, List[int]] = {}
    replacement_insert_policy_by_aircraft: Dict[int, Dict[str, Any]] = {}
    for aircraft_id, pkg in packages_by_aircraft.items():
        mission_list = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        replacements = replacement_by_aircraft.get(int(aircraft_id)) or []
        active_mission_list: List[Dict[str, Any]] = []
        removed_completed_ids: List[int] = []
        for mission in mission_list:
            if not isinstance(mission, dict):
                continue
            if bool(mission.get("isDone")):
                mission_input_id = _mission_input_id(mission)
                if mission_input_id is not None:
                    removed_completed_ids.append(int(mission_input_id))
                continue
            active_mission_list.append(mission)
        if removed_completed_ids:
            removed_completed_by_aircraft[int(aircraft_id)] = list(removed_completed_ids)
        if not replacements:
            pkg["individualMissionList"] = list(active_mission_list)
            if int(aircraft_id) in missing_replacement_aircraft_ids:
                emit(
                    "[NEXTCOLLAB][WARN] no replacement mission for aircraft "
                    f"{int(aircraft_id)}; keeping original target inputMissionID={int(target_input_id)}"
                )
            continue

        pkg["individualMissionList"], insert_policy = _merge_replacements_into_active_missions(
            active_mission_list=active_mission_list,
            replacements=replacements,
            target_input_id=int(target_input_id),
            preserve_current_input=bool(preserve_current_input_for_takeover),
        )
        replacement_insert_policy_by_aircraft[int(aircraft_id)] = dict(insert_policy)

    if replacement_insert_policy_by_aircraft:
        area_review_report["replacementInsertPolicyByAircraft"] = {
            int(aid): dict(policy)
            for aid, policy in sorted(replacement_insert_policy_by_aircraft.items())
        }

    if removed_completed_by_aircraft:
        area_review_report["removedCompletedInputMissionIDs"] = {
            int(aid): [int(mid) for mid in mids]
            for aid, mids in sorted(removed_completed_by_aircraft.items())
        }
        removed_summary = ", ".join(
            f"UAV{int(aid)}={','.join(str(mid) for mid in mids)}"
            for aid, mids in sorted(removed_completed_by_aircraft.items())
        )
        emit(f"[NEXTCOLLAB] omitted completed missions from cloned IMPs: {removed_summary}")

    if not _is_formation_input_mission(target_input_mission):
        _rebuild_next_collab_lah_target_paths(
            packages_by_aircraft=packages_by_aircraft,
            replacement_by_aircraft=replacement_by_aircraft,
            target_input_id=int(target_input_id),
            input_plan=input_data,
            generated_fp_by_path=generated_fp_by_path,
            generated_path_ids=generated_path_ids,
            emit=emit,
            id_reservation_summaries=id_reservation_summaries,
        )

    if not generated_fp_by_path:
        emit("[NEXTCOLLAB] no replacement flight paths prepared.")
        return None

    _deduplicate_generated_individual_mission_ids(
        packages_by_aircraft=packages_by_aircraft,
        generated_fp_by_path=generated_fp_by_path,
        generated_path_ids=generated_path_ids,
        emit=emit,
    )

    phase_timer.mark("build_artifacts")

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{int(new_plan_id)}.json")
    imp_rows: List[tuple[Path, Dict[str, Any]]] = []
    for aircraft_id, pkg in packages_by_aircraft.items():
        imp_id = _to_int(pkg.get("individualMissionPackageID"))
        if imp_id is None or imp_id <= 0:
            emit(f"[NEXTCOLLAB] cloned IMP missing individualMissionPackageID for aircraft {aircraft_id}.")
            return None
        imp_rows.append(
            (
                db_paths.get_db_subpath("IndividualMissionPlan", f"{int(imp_id)}.json"),
                pkg,
            )
        )

    fp_rows: List[tuple[Path, Dict[str, Any]]] = []
    for path_id in sorted(generated_path_ids):
        fp_rows.append(
            (
                db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json"),
                generated_fp_by_path[int(path_id)],
            )
        )

    validation_summary = validate_replan_payloads(
        mission_plan=new_plan_data,
        individual_mission_plans=[payload for _path, payload in imp_rows],
        flight_paths=[payload for _path, payload in fp_rows],
        scope=f"nextCollab:{new_plan_id}",
        allow_existing_db_artifacts=True,
        log=emit,
    )
    phase_timer.mark("validation")

    started_at = time.perf_counter()
    write_results = write_json_batch(
        [(plan_dest, new_plan_data), *imp_rows, *fp_rows],
        pretty=True,
        ensure_ascii=False,
        skip_if_unchanged=True,
        log=emit,
    )
    elapsed_ms = (time.perf_counter() - started_at) * 1000.0
    write_count = sum(1 for row in write_results if row.get("written"))
    emit(
        "[NEXTCOLLAB] stored replanned artifacts -> "
        f"plan:{plan_dest.name}, input:reuse({new_input_pkg_id}.json), imp:{len(imp_rows)}, fp:{len(fp_rows)} "
        f"(written={write_count}/{len(write_results)}, {elapsed_ms:.1f} ms)"
    )
    phase_timer.mark("write_artifacts")
    prepare_timer.mark("write")
    phase_timings_ms = phase_timer.snapshot()
    prepare_timings_ms = prepare_timer.snapshot()
    source_cache_summary = source_cache.summary()
    id_reservation_summary = {
        "latest": dict(prepared.id_reservation or {}),
        "blocks": [dict(row) for row in id_reservation_summaries],
    }
    area_review_report["prepareTimingMs"] = dict(prepare_timings_ms)
    area_review_report["sourceCache"] = dict(source_cache_summary)
    area_review_report["idReservation"] = dict(id_reservation_summary)
    emit(f"[NEXTCOLLAB][TIME] timingMs={phase_timings_ms}")
    emit(
        "[NEXTCOLLAB][BENCH] "
        f"missionMode={mission_mode or 'unknown'} "
        f"prepareTimingMs={json.dumps(prepare_timings_ms, ensure_ascii=False, sort_keys=True)} "
        f"sourceCache={json.dumps(source_cache_summary, ensure_ascii=False, sort_keys=True)}"
    )
    for fov_adjust_message in pop_runtime_camera_fov_adjustment_logs():
        emit(str(fov_adjust_message))

    log_dir = db_paths.get_db_subpath("DSS_Internal")
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"NextCollab_{int(target_input_id)}_{int(now_ms)}.json"
    log_payload = {
        "timestamp": int(now_ms),
        "reason": str(reason or ""),
        "sourceMissionPlanID": int(source_plan_id),
        "sourceInputMissionPackageID": int(source_input_pkg_id),
        "generatedMissionPlanID": int(new_plan_id),
        "generatedInputMissionPackageID": int(new_input_pkg_id),
        "reusedInputMissionPackage": True,
        "currentInputMissionID": int(current_input_id),
        "targetInputMissionID": int(target_input_id),
        "targetAircraftIDs": [int(aid) for aid in target_aircraft_ids],
        "representativeEntryCoordinate": dict(representative_entry),
        "nextInputEntryCoordinate": dict(next_entry) if isinstance(next_entry, dict) else None,
        "entryAircraftList": [
            {
                "aircraftID": int(aid),
                "coordinate": dict(coord),
            }
            for aid, coord in sorted(entry_coord_map.items())
        ],
        "areaReview": dict(area_review_report),
        "plannerWorkflow": str(planner_workflow),
        "plannerResultText": str(planner_result_text or ""),
        "plannedPathRowCount": int(planned_result_count),
        "missionMode": str(mission_mode or ""),
        "prepareTimingMs": dict(prepare_timings_ms),
        "sourceCache": dict(source_cache_summary),
        "idReservation": dict(id_reservation_summary),
        "uavIndependentWork": dict(uav_work_summary),
        "runtimePreservation": dict(runtime_preservation),
        "generatedIndividualMissionPackageIDs": sorted(int(val) for val in generated_imp_ids),
        "generatedPathIDs": sorted(int(val) for val in generated_path_ids),
        "replanFlowMode": REPLAN_FLOW_MODE,
        "replanTransactionId": transaction_id,
        "writeResults": write_results,
        "validation": validation_summary,
        "timingMs": phase_timings_ms,
        "logMessages": list(log_messages),
        "detail": dict(detail),
        "logArtifactMode": debug_artifact_mode(),
    }
    log_written = write_debug_json(log_path, log_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
    log_payload["logArtifactWritten"] = bool(log_written)
    if log_written:
        emit(f"[NEXTCOLLAB] log captured -> {log_path}")
    else:
        emit("[NEXTCOLLAB] log artifact skipped by runtime artifact mode.")
    try:
        next_collab_replan_store.save_event(
            "mission_pipeline_complete",
            {
                "generatedMissionPlanID": int(new_plan_id),
                "generatedInputMissionPackageID": int(new_input_pkg_id),
                "reusedInputMissionPackage": True,
                "sourceMissionPlanID": int(source_plan_id),
                "currentInputMissionID": int(current_input_id),
                "targetInputMissionID": int(target_input_id),
                "targetAircraftIDs": [int(aid) for aid in target_aircraft_ids],
                "generatedIndividualMissionPackageIDs": sorted(int(val) for val in generated_imp_ids),
                "generatedPathIDs": sorted(int(val) for val in generated_path_ids),
                "areaReview": dict(area_review_report),
                "plannerWorkflow": str(planner_workflow),
                "plannedPathRowCount": int(planned_result_count),
                "missionMode": str(mission_mode or ""),
                "prepareTimingMs": dict(prepare_timings_ms),
                "sourceCache": dict(source_cache_summary),
                "idReservation": dict(id_reservation_summary),
                "uavIndependentWork": dict(uav_work_summary),
                "runtimePreservation": dict(runtime_preservation),
                "replanFlowMode": REPLAN_FLOW_MODE,
                "replanTransactionId": transaction_id,
                "writeResults": write_results,
                "validation": validation_summary,
                "logPath": str(log_path),
                "logArtifactMode": debug_artifact_mode(),
                "logArtifactWritten": bool(log_written),
                "timingMs": phase_timings_ms,
            },
        )
    except Exception:
        pass

    plan_meta_map = dict(ctx.get("_option_meta") or {})
    plan_meta_entry = plan_meta_map.setdefault(int(new_plan_id), {})
    plan_meta_entry.update(
        {
            "triggerType": TRIGGER_TYPE,
            "sourceMissionPlanID": int(source_plan_id),
            "currentInputMissionID": int(current_input_id),
            "targetInputMissionID": int(target_input_id),
            "inputMissionPackageID": int(new_input_pkg_id),
            "reusedInputMissionPackage": True,
            "generatedIndividualMissionPackageIDs": sorted(int(val) for val in generated_imp_ids),
            "generatedPathIDs": sorted(int(val) for val in generated_path_ids),
            "areaReview": dict(area_review_report),
            "plannerWorkflow": str(planner_workflow),
            "plannedPathRowCount": int(planned_result_count),
            "missionMode": str(mission_mode or ""),
            "prepareTimingMs": dict(prepare_timings_ms),
            "sourceCache": dict(source_cache_summary),
            "idReservation": dict(id_reservation_summary),
            "uavIndependentWork": dict(uav_work_summary),
            "runtimePreservation": dict(runtime_preservation),
            "replanFlowMode": REPLAN_FLOW_MODE,
            "replanTransactionId": transaction_id,
            "writeResults": write_results,
            "validation": validation_summary,
            "suppress0702Fallback": True,
            "logPath": str(log_path),
            "logArtifactMode": debug_artifact_mode(),
            "logArtifactWritten": bool(log_written),
            "timingMs": phase_timings_ms,
        }
    )

    return NextCollabPipelineResult(
        plan_ids=[int(new_plan_id)],
        option_names=list(option_names),
        plan_meta_map=plan_meta_map,
        generated_imp_ids=set(int(val) for val in generated_imp_ids),
        generated_path_ids=set(int(val) for val in generated_path_ids),
        new_input_package_id=int(new_input_pkg_id),
        log_path=log_path,
    )
