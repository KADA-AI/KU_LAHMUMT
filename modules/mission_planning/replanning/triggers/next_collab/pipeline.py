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

from modules.common import agent_status_snapshot, db_paths, mission_area_replan_store
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
try:
    from modules.mission_planning.runtime.state import branch_ownership as _branch_ownership_store
except Exception:  # pragma: no cover - defensive optional import
    _branch_ownership_store = None  # type: ignore
try:
    from modules.mission_planning.planners.donut_patrol.production import (
        build_donut_band_pieces as _donut_band_pieces,
        build_donut_wplist as _donut_wplist,
        is_donut_boundary_mission as _is_donut_boundary_mission,
    )
except Exception:  # pragma: no cover - defensive optional import
    _donut_band_pieces = None  # type: ignore
    _donut_wplist = None  # type: ignore
    _is_donut_boundary_mission = None  # type: ignore
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
    _area_sweep_items_xy,
    _area_reciprocal_terrain_profile,
    _dem_alt,
    _make_hold_waypoint,
    _recompute_waypoint_timeline,
    build_formation_flight_path_from_template,
    build_flight_path_from_planned_row,
    build_mission_info_from_planned_row,
    _prewarm_dem_altitudes_for_path_rows_if_enabled,
)
from modules.mission_planning.pipelines.mission_path_trim import reassign_unique_waypoint_ids_inplace
from modules.mission_planning.pipelines.ground_maneuver_mode import (
    TYPE2_SELF_RELIANCE_GUARD_AREA,
    ground_maneuver_lah_info_for_input,
    resolve_type2_self_reliance_phase,
)
from modules.mission_planning.pipelines.type2_boundary_guard_loop import (
    annotate_boundary_guard_set,
    apply_boundary_guard_contract,
    extract_boundary_guard_contract,
    link_boundary_guard_flight_path_sets,
    sync_boundary_guard_contract_from_flight_paths,
)
from modules.mission_planning.pipelines.lah_operational_mode import lah_special_info_for_input
from modules.mission_planning.pipelines.handover_terminal import (
    apply_control_transfer_direct_metadata,
)
from modules.mission_planning.planning_modes import (
    mission_mode_context,
    mode_log_label,
    resolve_mission_planning_mode,
)
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
    _apply_input_order_execution_barrier,
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
from modules.mission_planning.MissionPlanner import capture_physics
from modules.mission_planning.pipelines.area_ownership_stitch import (
    convex_hull_area_fragments_xy,
)
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
DEFAULT_AREA_REVIEW_MAX_SEGMENT_M = 1500.0
_LAH_NEXT_COLLAB_MIN_LOOKAHEAD_S = 10.0

DEFAULT_OPTION_NAME = NEXT_COLLAB_OPTION_NAME

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


def _area_coverage_pass_contract_from_input_mission(
    input_mission: Dict[str, Any] | None,
) -> Dict[str, Any]:
    """AREA now has no reciprocal pass contract.

    Monitoring and the different replan entry points may persist the portable
    contract on either the input-mission root or ``missionDetail``.  Merge both
    views (detail wins) before normalizing it so the downstream path builder
    receives the same obligation regardless of which entry point invoked this
    pipeline.
    """

    return {}

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

    # A portable contract may intentionally contain only obligations and pass
    # state (without coveragePassDetails).  Preserve those authoritative fields
    # verbatim; the legacy normalizer cannot reconstruct them from pass order.
    contract = {
        key: deepcopy(source[key])
        for key in _AREA_COVERAGE_PASS_CONTRACT_KEYS
        if key in source
    }
    if depth_contract:
        if isinstance(source.get("coveragePassDetails"), list):
            contract["coveragePassAttributionDetails"] = deepcopy(
                source.get("coveragePassDetails") or []
            )
        for key, value in normalized.items():
            contract[key] = deepcopy(value)
    else:
        for key, value in normalized.items():
            if key != "coveragePassDetails":
                contract.setdefault(key, deepcopy(value))
    for key in _AREA_COVERAGE_DEPTH_CONTRACT_KEYS:
        if key in depth_contract:
            contract[key] = deepcopy(depth_contract[key])
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
        target[key] = deepcopy(value)


TRIGGER_TYPE = NEXT_COLLAB_TRIGGER_TYPE
AREA_PLANNER_COMPONENT_MAX_COUNT = 64


def _next_collab_replacement_path_build_workers(item_count: int, *, scope: str = "") -> int:
    if int(item_count) < 2:
        return 1
    scope_name = str(scope or "").strip().lower()
    default_workers = 3 if scope_name == "line" else 1
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


_AREA_LINK_SCAN_MODE = 2


def _area_link_scan_coords(waypoint: Dict[str, Any]) -> List[Dict[str, Any]]:
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    coords = line_search.get("coordinateList")
    return [item for item in (coords or []) if isinstance(item, dict) and item.get("latitude") is not None]


def _area_link_waypoint_coord(waypoint: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """The waypoint's own coordinate.

    Deliberately not the scan run's endpoints: d0303 feeds the shared geometry
    plain waypoint coordinates, and both builders have to agree on the input
    convention or the same handover gets two different links.
    """

    coord = waypoint.get("coordinate") if isinstance(waypoint, dict) else None
    return dict(coord) if isinstance(coord, dict) and coord.get("latitude") is not None else None


def _area_link_altitude_fn(
    prev_end: Dict[str, Any],
    next_first: Dict[str, Any],
) -> Callable[[float, float], int]:
    """이웃 WP의 AGL을 이월하는 링크 고도 함수.

    d0303 초기계획은 지면고+임무 오프셋(_mission_wp_alt)으로 링크 고도를 만드는데,
    재계획이 맨 DEM 지면고만 쓰면 링크 WP가 지형에 붙어 버린다.  여기서는 양쪽
    이웃 WP에서 관측된 AGL 중 큰 값을 지면 위에 그대로 이월한다.
    """

    agl_candidates: List[float] = []
    msl_candidates: List[float] = []
    for coord in (prev_end, next_first):
        if not isinstance(coord, dict):
            continue
        alt = _to_float(coord.get("altitude"))
        if alt is None or alt <= 0.0:
            continue
        msl_candidates.append(float(alt))
        try:
            ground = float(_dem_alt(float(coord["latitude"]), float(coord["longitude"])))
        except Exception:
            continue
        if alt > ground:
            agl_candidates.append(float(alt) - ground)
    carried_agl = max(agl_candidates) if agl_candidates else 0.0
    neighbor_floor = max(msl_candidates) if msl_candidates else 0.0

    def _altitude(lat: float, lon: float) -> int:
        try:
            ground = float(_dem_alt(float(lat), float(lon)))
        except Exception:
            ground = 0.0
        altitude = ground + carried_agl
        if carried_agl <= 0.0:
            # AGL을 못 구했으면 이웃 WP의 MSL 고도를 바닥으로 쓴다.
            altitude = max(altitude, neighbor_floor)
        return int(round(altitude))

    return _altitude


def _append_next_collab_area_transition_links(
    *,
    generated_fp_by_path: Dict[int, Dict[str, Any]],
    ordered_path_ids_by_aircraft: Dict[int, List[int]],
    emit: Callable[[str], None],
    suppressed_link_pairs: Set[tuple[int, int]] | None = None,
) -> int:
    """Join consecutive area passes of one aircraft with a flyable turn link.

    The initial-plan builder adds this inside d0303, but a next-collab replan
    never goes through d0303 - it builds each replacement path independently -
    so a rebuilt area pass used to end with the aircraft told to fly straight
    from the end of one lane to the start of the next.  Both builders now share
    ``compute_area_transition_link`` so a handover flies the same way whichever
    produced it.
    """

    try:
        from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0303 import (
            _dubins_link_available,
            compute_area_transition_link,
        )
    except Exception:
        return 0
    # 초기계획(d0303)과 같은 스위치 — out-leg 링크는 기본 방출 안 함 (사용자
    # 결정).  켜면 두 빌더가 같은 링크를 낸다.
    suppressed_pairs = {
        (int(prev_path_id), int(next_path_id))
        for prev_path_id, next_path_id in (suppressed_link_pairs or set())
    }
    globally_enabled = bool(_dubins_link_available())
    if not globally_enabled:
        return 0

    linked = 0
    for aircraft_id, path_ids in sorted(ordered_path_ids_by_aircraft.items()):
        ordered = [int(path_id) for path_id in path_ids]
        for prev_path_id, next_path_id in zip(ordered[:-1], ordered[1:]):
            # Sequential two-area routes use this geometry only to select the
            # second capture direction. Their public mission intentionally
            # omits out-leg/turn helpers and starts at the next capture WP.
            if (int(prev_path_id), int(next_path_id)) in suppressed_pairs:
                continue
            prev_payload = generated_fp_by_path.get(int(prev_path_id))
            next_payload = generated_fp_by_path.get(int(next_path_id))
            if not isinstance(prev_payload, dict) or not isinstance(next_payload, dict):
                continue
            prev_wps = prev_payload.get("waypointList")
            next_wps = next_payload.get("waypointList")
            if not isinstance(prev_wps, list) or len(prev_wps) < 2:
                continue
            if not isinstance(next_wps, list) or len(next_wps) < 2:
                continue
            prev_start = _area_link_waypoint_coord(prev_wps[-2])
            prev_end = _area_link_waypoint_coord(prev_wps[-1])
            next_first = _area_link_waypoint_coord(next_wps[0])
            next_second = _area_link_waypoint_coord(next_wps[1])
            if not all((prev_start, prev_end, next_first, next_second)):
                continue
            cruise_speed_mps = _to_float(prev_wps[-1].get("speed")) or 40.0
            try:
                turn_radius_m = float(
                    _turn_radius_m_for_area_link(float(cruise_speed_mps))
                )
            except Exception:
                continue
            try:
                coords, link_speed_mps = compute_area_transition_link(
                    prev_start_coord=prev_start,
                    prev_end_coord=prev_end,
                    next_first_coord=next_first,
                    next_second_coord=next_second,
                    turn_radius_m=turn_radius_m,
                    cruise_speed_mps=float(cruise_speed_mps),
                    # 맨 DEM 지면고를 쓰면 링크 WP가 지형에 붙는다 — 이웃 WP의
                    # AGL을 이월한다 (d0303의 지면고+오프셋 규약과 같은 효과).
                    altitude_fn=_area_link_altitude_fn(prev_end, next_first),
                    min_link_gap_m=max(80.0, turn_radius_m * 0.2),
                )
            except Exception:
                continue
            if not coords:
                continue
            # The camera stays on the last captured point of the finished pass,
            # so the link never leaves an observation gap.
            stare_coord = None
            for waypoint in reversed(prev_wps):
                scan = _area_link_scan_coords(waypoint)
                if scan:
                    stare_coord = dict(scan[-1])
                    break
            if stare_coord is None:
                stare_coord = dict(next_first)
            template_filming = (
                prev_wps[-1].get("filmingProperty")
                if isinstance(prev_wps[-1].get("filmingProperty"), dict)
                else {}
            )
            field_of_view_deg = _to_float(template_filming.get("fieldOfView")) or 7.2
            sensor_type = _to_int(template_filming.get("sensorType")) or 1
            for coord in coords:
                # 진출점(out leg)은 실제로 통과해야 하는 선회 접점 — FLYOVER.
                # 링크는 경로별 flyover 정규화 이후에 덧붙으므로 여기 값이
                # 최종값이고, 마커를 남기면 0303 출력으로 새기만 한다.
                link_wp = _make_hold_waypoint(
                    coordinate=coord,
                    speed_mps=float(link_speed_mps),
                    sensor_type=int(sensor_type),
                    field_of_view_deg=float(field_of_view_deg),
                    orientation_coordinate=stare_coord,
                    waypoint_pass_type=3,
                )
                prev_wps.append(link_wp)
            prev_payload["waypointList"] = prev_wps
            # Link waypoints are created after the replacement path builder has
            # finalized its timeline.  Continue the cumulative ETA through the
            # appended turn instead of leaving the new points at their factory
            # default (eta=0), which makes ETA decrease after the last sweep.
            _recompute_waypoint_timeline(
                prev_wps,
                default_speed_mps=float(cruise_speed_mps),
            )
            linked += len(coords)
            emit(
                "[NEXTCOLLAB][AREA] area transition link "
                f"aircraft={int(aircraft_id)} {int(prev_path_id)}->{int(next_path_id)} "
                f"points={len(coords)} radius_speed={float(link_speed_mps):.1f}m/s"
            )
    return linked


def _turn_radius_m_for_area_link(speed_mps: float) -> float:
    from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0303 import (
        _turn_radius_m_for_speed,
    )

    return float(_turn_radius_m_for_speed(float(speed_mps)))


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
        "lineDemMs": _sum_float("lineDemMs"),
        "lineAnchorContextMs": _sum_float("lineAnchorContextMs"),
        "lineSquashMs": _sum_float("lineSquashMs"),
        "lineSquashReanchorMs": _sum_float("lineSquashReanchorMs"),
        "lineSquashSimplifyMs": _sum_float("lineSquashSimplifyMs"),
        "lineSquashSnapMs": _sum_float("lineSquashSnapMs"),
        "lineSquashSpeedMs": _sum_float("lineSquashSpeedMs"),
        "lineSquashClampFovMs": _sum_float("lineSquashClampFovMs"),
        "lineSearchGeometryCacheMs": _sum_float("lineSearchGeometryCacheMs"),
        "lineSearchGeometryCacheHits": _sum_int("lineSearchGeometryCacheHits"),
        "lineSearchGeometryCacheMisses": _sum_int("lineSearchGeometryCacheMisses"),
        "lineSearchGeometryCacheSkips": _sum_int("lineSearchGeometryCacheSkips"),
        "lineSearchGeometryCacheRows": _sum_int("lineSearchGeometryCacheRows"),
        "lineSearchGeometryCacheCoords": _sum_int("lineSearchGeometryCacheCoords"),
        "lineSearchEstimateWarnCount": _sum_int("lineSearchEstimateWarn"),
        "lineSearchEstimateHeavyCount": _sum_int("lineSearchEstimateHeavy"),
        "maxLineSearchEstimatedCriticalPathMs": _max_float("lineSearchEstimatedCriticalPathMs"),
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


def _effective_type2_three_branch_search_speed_scale_multiplier(
    value: Any,
    *,
    is_type2_branch_mission: bool,
    locked_type2_ownership: Dict[int, List[int]] | None,
    emit: Callable[[str], None] | None = None,
) -> float:
    """Add an execution margin only to the Type-2 three-branch span.

    AREA already has its general scan-completion margin, while LINE is normally
    synchronized almost exactly to the incoming waypoint leg. The independent
    branches have additional activation/turn hand-off latency, so both
    geometries need one shared margin after branch ownership is resolved.
    """

    base_multiplier = _normalize_search_speed_scale_multiplier(value)
    ownership = (
        locked_type2_ownership
        if isinstance(locked_type2_ownership, dict)
        else {}
    )
    if not bool(is_type2_branch_mission) or len(ownership) != 3:
        return float(base_multiplier)
    try:
        branch_multiplier = float(
            get_runtime_float(
                "next_collab_type2_three_branch_search_speed_scale",
                1.10,
            )
        )
    except Exception:
        branch_multiplier = 1.10
    branch_multiplier = max(0.10, min(5.0, float(branch_multiplier)))
    effective_multiplier = float(base_multiplier) * float(branch_multiplier)
    if emit is not None and abs(float(branch_multiplier) - 1.0) > 1e-6:
        emit(
            "[NEXTCOLLAB][TYPE2] three-branch AREA/LINE searchSpeed margin "
            f"applied factor={float(branch_multiplier):.2f} "
            f"effective={float(effective_multiplier):.2f}."
        )
    return float(effective_multiplier)


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
    planning_mode: Dict[str, Any] = field(default_factory=dict)


def _apply_type2_boundary_guard_loop_to_prepared(
    prepared: _PreparedReplacements,
    *,
    input_data: Dict[str, Any],
    input_package_id: int,
    target_input_id: int,
) -> _PreparedReplacements:
    """Finalize the strict Type-2 guard AREA contract after waypoint IDs exist."""

    if (
        resolve_type2_self_reliance_phase(input_data, int(target_input_id))
        != TYPE2_SELF_RELIANCE_GUARD_AREA
    ):
        return prepared

    duration_s = float(
        get_runtime_float("type2_boundary_guard_duration_s", 600.0)
    )
    annotated_missions: List[Dict[str, Any]] = []
    for aircraft_id in sorted(prepared.replacement_by_aircraft):
        owner_rows = [
            mission
            for mission in prepared.replacement_by_aircraft.get(int(aircraft_id), [])
            if isinstance(mission, dict)
            and _mission_input_id(mission) == int(target_input_id)
        ]
        if not owner_rows:
            continue
        annotate_boundary_guard_set(
            owner_rows,
            set_id=(
                f"type2-boundary:{int(input_package_id)}:{int(target_input_id)}:"
                f"aircraft-{int(aircraft_id)}"
            ),
            duration_s=duration_s,
            include_individual_mission_info=True,
        )
        annotated_missions.extend(owner_rows)
        for mission in owner_rows:
            path_id = _to_int(mission.get("pathID"))
            flight_path = (
                prepared.generated_fp_by_path.get(int(path_id))
                if path_id is not None
                else None
            )
            if isinstance(flight_path, dict):
                apply_boundary_guard_contract(
                    flight_path,
                    extract_boundary_guard_contract(
                        mission,
                        mission.get("individualMissionInfo"),
                    ),
                )

    link_boundary_guard_flight_path_sets(
        prepared.generated_fp_by_path.values(),
        strict=True,
    )
    sync_boundary_guard_contract_from_flight_paths(
        annotated_missions,
        prepared.generated_fp_by_path.values(),
    )
    prepared.review_report = dict(prepared.review_report or {})
    prepared.review_report["boundaryGuardLoop"] = {
        "enabled": bool(annotated_missions),
        "durationS": float(duration_s),
        "ownerMissionCount": len(annotated_missions),
    }
    return prepared


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
    flight_path_directory: Optional[Path] = None

    def flight_path_json_path(self, path_id: int) -> Path:
        directory = self.flight_path_directory
        if directory is None:
            directory = Path(db_paths.get_db_subpath("FlightPath"))
            self.flight_path_directory = directory
        return directory / f"{int(path_id)}.json"

    def summary(self) -> Dict[str, Any]:
        return {
            "flightPathLoads": int(self.flight_path_loads),
            "flightPathHits": int(self.flight_path_hits),
            "flightPathCached": len(self.flight_paths),
        }


def warm_next_collab_replan_pipeline() -> Dict[str, Any]:
    try:
        _ensure_runtime_import_paths()
        d0303, d0304, search_speed, mp_config = _import_runtime_modules()
        cruise_speed, turn_step = _apply_runtime_params(
            d0303,
            d0304,
            search_speed,
            mp_config,
        )
    except Exception as exc:
        return {"ready": False, "error": str(exc)}
    terrain_warmup: Dict[str, Any] = {}
    try:
        from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
            warm_terrain_cache,
        )

        terrain_warmup = warm_terrain_cache()
    except Exception as exc:
        terrain_warmup = {"ready": False, "error": str(exc)}
    return {
        "ready": True,
        "cruiseSpeedMps": float(cruise_speed),
        "turnStepDeg": float(turn_step),
        "terrainWarmup": terrain_warmup,
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


def _load_lah_route_start_coordinates(
    entry_coord_map: Dict[int, Dict[str, Any]] | None = None,
    entry_context_map: Dict[int, Dict[str, Any]] | None = None,
) -> Dict[int, Dict[str, Any]]:
    """Resolve 10-second-ahead LAH positions for next-collaboration transit."""

    starts: Dict[int, Dict[str, Any]] = {}
    for aircraft_id, raw_coord in dict(entry_coord_map or {}).items():
        aid = _to_int(aircraft_id)
        coord = _normalize_coordinate(raw_coord)
        if aid in (1, 2, 3) and coord is not None:
            starts[int(aid)] = coord
    try:
        snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    except Exception:
        snapshot = {}
    agent_states = snapshot.get("agent_states") or (snapshot.get("raw") or {}).get("agentStateList") or []
    state_by_aircraft: Dict[int, Dict[str, Any]] = {}
    for entry in agent_states if isinstance(agent_states, list) else []:
        if not isinstance(entry, dict):
            continue
        aid = _to_int(entry.get("aircraftID") or entry.get("aircraftId"))
        if aid not in (1, 2, 3):
            continue
        state_by_aircraft[int(aid)] = dict(entry)
        if int(aid) in starts:
            continue
        manned_info = entry.get("mannedInfo") if isinstance(entry.get("mannedInfo"), dict) else {}
        coord = _normalize_coordinate(entry.get("coordinate")) or _normalize_coordinate(manned_info.get("coordinate"))
        if coord is not None:
            starts[int(aid)] = coord

    contexts = dict(entry_context_map or {})
    predicted_starts: Dict[int, Dict[str, Any]] = {}
    for aircraft_id, start_coord in starts.items():
        state = dict(state_by_aircraft.get(int(aircraft_id)) or {})
        context = contexts.get(int(aircraft_id), contexts.get(str(int(aircraft_id))))
        if isinstance(context, dict):
            state.update(context)
        velocity = state.get("velocity") if isinstance(state.get("velocity"), dict) else {}
        heading = _to_float(
            state.get("headingDeg")
            if state.get("headingDeg") is not None
            else state.get("heading")
        )
        if heading is None:
            heading = _to_float(velocity.get("heading"))
        speed_mps = _to_float(state.get("speedMps") or state.get("speed_mps"))
        if speed_mps is None:
            speed_mps = _to_float(state.get("speed") or velocity.get("speed"))
            if speed_mps is not None and float(speed_mps) > 100.0:
                speed_mps = float(speed_mps) / 3.6
        if heading is None or speed_mps is None or float(speed_mps) <= 0.0:
            predicted_starts[int(aircraft_id)] = dict(start_coord)
            continue
        lookahead_s = max(
            _LAH_NEXT_COLLAB_MIN_LOOKAHEAD_S,
            float(get_runtime_float("lah_next_collab_route_start_lookahead_s", _LAH_NEXT_COLLAB_MIN_LOOKAHEAD_S)),
        )
        distance_m = float(speed_mps) * float(lookahead_s)
        radius_m = 6_371_000.0
        lat1 = math.radians(float(start_coord["latitude"]))
        lon1 = math.radians(float(start_coord["longitude"]))
        bearing = math.radians(float(heading) % 360.0)
        angular = float(distance_m) / radius_m
        lat2 = math.asin(
            math.sin(lat1) * math.cos(angular)
            + math.cos(lat1) * math.sin(angular) * math.cos(bearing)
        )
        lon2 = lon1 + math.atan2(
            math.sin(bearing) * math.sin(angular) * math.cos(lat1),
            math.cos(angular) - math.sin(lat1) * math.sin(lat2),
        )
        predicted = {
            "latitude": math.degrees(lat2),
            "longitude": math.degrees(lon2),
        }
        if start_coord.get("altitude") is not None:
            predicted["altitude"] = int(round(float(start_coord["altitude"])))
        predicted_starts[int(aircraft_id)] = predicted
    return predicted_starts


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


def _normalize_area_assignment_pass(value: Any) -> Optional[str]:
    text = str(value or "").strip().lower()
    return text if text in {"forward", "reverse"} else None


def _ordered_area_assignment_passes(contract: Dict[str, Any] | None) -> List[str]:
    source = contract if isinstance(contract, dict) else {}
    found: Set[str] = set()
    for value in source.get("remainingCoveragePasses") or []:
        pass_name = _normalize_area_assignment_pass(value)
        if pass_name is not None:
            found.add(pass_name)
    for row in source.get("coveragePassObligations") or []:
        if not isinstance(row, dict):
            continue
        pass_name = _normalize_area_assignment_pass(row.get("coveragePass"))
        if pass_name is not None:
            found.add(pass_name)
    # OUT is always allocated first.  RETURN is a second, independent division
    # over the same currently available aircraft pool.
    return [pass_name for pass_name in ("forward", "reverse") if pass_name in found]


def _single_area_assignment_pass_contract(
    contract: Dict[str, Any] | None,
    pass_name: str,
    *,
    default_full: bool = False,
) -> Dict[str, Any]:
    normalized_pass = _normalize_area_assignment_pass(pass_name)
    if normalized_pass is None:
        return {}
    source = deepcopy(contract) if isinstance(contract, dict) else {}
    obligation: Dict[str, Any] = {}
    for key in ("coveragePassObligations", "coveragePassDetails"):
        for row in source.get(key) or []:
            if not isinstance(row, dict):
                continue
            if _normalize_area_assignment_pass(row.get("coveragePass")) != normalized_pass:
                continue
            obligation = deepcopy(row)
            break
        if obligation:
            break
    obligation["coveragePass"] = str(normalized_pass)
    obligation["passIndex"] = 1 if normalized_pass == "forward" else 2
    obligation["isDone"] = False
    if default_full and not obligation.get("obligationKind"):
        obligation["obligationKind"] = "full"
    source["areaCoveragePassContractVersion"] = int(
        _to_int(source.get("areaCoveragePassContractVersion")) or 2
    )
    source["coveragePassPolicy"] = "independent_pass_assignment"
    source["coveragePassOrder"] = [str(normalized_pass)]
    source["coveragePassDetails"] = [deepcopy(obligation)]
    source["coveragePassObligations"] = [deepcopy(obligation)]
    source["remainingCoveragePasses"] = [str(normalized_pass)]
    source["currentCoveragePass"] = str(normalized_pass)
    source["activeCoveragePass"] = str(normalized_pass)
    source["areaCoveragePhase"] = (
        "outbound" if normalized_pass == "forward" else "return"
    )
    source["areaPassAssignmentMode"] = "independent_available_uav_division"
    source["areaAssignedCoveragePass"] = str(normalized_pass)
    return source


def _area_pass_planning_detail(
    mission_detail: Dict[str, Any],
    contract: Dict[str, Any] | None,
    pass_name: str,
) -> Dict[str, Any]:
    detail = deepcopy(mission_detail) if isinstance(mission_detail, dict) else {}
    stable_assignment = mission_area_replan_store.area_assignment_detail(
        detail,
        fallback=detail,
    )
    if isinstance(stable_assignment, dict):
        mission_area_replan_store.apply_area_assignment_geometry(
            detail,
            stable_assignment,
        )
    pass_contract = _single_area_assignment_pass_contract(contract, pass_name)
    obligation_rows = pass_contract.get("coveragePassObligations") or []
    obligation = obligation_rows[0] if obligation_rows and isinstance(obligation_rows[0], dict) else {}
    remaining_detail = obligation.get("remainingDetail")
    if isinstance(remaining_detail, dict):
        # Divide the stable Area once per pass into available-UAV pieces.  The
        # small remaining fragments are capture workload and clip sweep rows
        # inside those pieces; they must not become allocation polygons.
        detail["areaCoverageWorkloadDetail"] = deepcopy(remaining_detail)
    return detail


def _area_rows_need_independent_return_assignment(
    planner_results: List[tuple[Dict[str, Any], Any]],
) -> bool:
    for _component, planner_result in planner_results:
        for row in getattr(planner_result, "expected_paths", []) or []:
            if not isinstance(row, dict):
                continue
            scan_lines = row.get("sweepLineListXY") or []
            try:
                profile = _area_reciprocal_terrain_profile(row, scan_lines)
            except Exception:
                continue
            if bool((profile or {}).get("active")):
                return True
    return False


def _area_entries_after_planned_pass(
    aircraft_entries: List[Dict[str, Any]],
    planner_results: List[tuple[Dict[str, Any], Any]],
) -> List[Dict[str, Any]]:
    updated = {
        int(_to_int(row.get("aircraftID")) or 0): deepcopy(row)
        for row in aircraft_entries
        if isinstance(row, dict) and int(_to_int(row.get("aircraftID")) or 0) > 0
    }
    ordered_rows: List[Dict[str, Any]] = []
    for component, planner_result in planner_results:
        component_index = int(_to_int(component.get("componentIndex")) or 0)
        for row in getattr(planner_result, "expected_paths", []) or []:
            if isinstance(row, dict):
                row_copy = dict(row)
                row_copy["_componentIndex"] = int(component_index)
                ordered_rows.append(row_copy)
    ordered_rows.sort(
        key=lambda row: (
            int(_to_int(row.get("aircraftID")) or 0),
            int(_to_int(row.get("_componentIndex")) or 0),
            int(_next_collab_area_path_row_phase_rank(row)),
        )
    )
    for row in ordered_rows:
        aircraft_id = int(_to_int(row.get("aircraftID")) or 0)
        if aircraft_id not in updated:
            continue
        route_xy = [
            (float(point[0]), float(point[1]))
            for point in (row.get("routeXY") or [])
            if isinstance(point, (tuple, list)) and len(point) >= 2
        ]
        end_xy_raw = row.get("waypointEndXY")
        end_xy = (
            (float(end_xy_raw[0]), float(end_xy_raw[1]))
            if isinstance(end_xy_raw, (tuple, list)) and len(end_xy_raw) >= 2
            else route_xy[-1] if route_xy else None
        )
        if end_xy is None:
            continue
        previous_xy = route_xy[-2] if len(route_xy) >= 2 else None
        if previous_xy is None:
            start_xy_raw = row.get("waypointStartXY")
            if isinstance(start_xy_raw, (tuple, list)) and len(start_xy_raw) >= 2:
                previous_xy = (float(start_xy_raw[0]), float(start_xy_raw[1]))
        altitude_m = _to_float((updated[aircraft_id].get("coordinate") or {}).get("altitude")) or 0.0
        updated[aircraft_id]["coordinate"] = meters_to_coord(
            float(end_xy[0]),
            float(end_xy[1]),
            alt_m=float(altitude_m),
        )
        if previous_xy is not None:
            dx = float(end_xy[0]) - float(previous_xy[0])
            dy = float(end_xy[1]) - float(previous_xy[1])
            if math.hypot(dx, dy) > 1.0:
                updated[aircraft_id]["headingDeg"] = (
                    math.degrees(math.atan2(dx, dy)) + 360.0
                ) % 360.0
        updated[aircraft_id]["activationPredictionUsed"] = False
        updated[aircraft_id].pop("linePredictedEntryCoordinate", None)
    return [updated[aircraft_id] for aircraft_id in sorted(updated)]


def _single_aircraft_sequential_area_entries(
    aircraft_entries: List[Dict[str, Any]],
    mission_polygon: List[Dict[str, Any]],
    *,
    enabled: bool,
    stage_count: int = 2,
) -> tuple[List[Dict[str, Any]], Dict[str, Any] | None]:
    """Provide planning-only aircraft so one UAV receives stage 1..N pieces.

    Synthetic aircraft never leave this Area-planner call. Their assigned
    pieces are folded back onto the real aircraft as sequential missions by
    ``_collapse_single_aircraft_sequential_area_result``.
    """

    valid_entries = [deepcopy(row) for row in aircraft_entries if isinstance(row, dict)]
    stage_count = max(2, int(stage_count))
    if not enabled or len(valid_entries) != 1:
        return valid_entries, None

    real_entry = valid_entries[0]
    real_aircraft_id = int(_to_int(real_entry.get("aircraftID")) or 0)
    real_coord = _normalize_coordinate(real_entry.get("coordinate"))
    polygon_xy = [
        point_xy
        for point_xy in (coord_to_xy(coord) for coord in mission_polygon)
        if point_xy is not None
    ]
    real_xy = coord_to_xy(real_coord) if real_coord is not None else None
    if real_aircraft_id <= 0 or real_coord is None or real_xy is None or len(polygon_xy) < 3:
        return valid_entries, None

    center_x = sum(float(point[0]) for point in polygon_xy) / float(len(polygon_xy))
    center_y = sum(float(point[1]) for point in polygon_xy) / float(len(polygon_xy))
    span_x = max(float(point[0]) for point in polygon_xy) - min(float(point[0]) for point in polygon_xy)
    span_y = max(float(point[1]) for point in polygon_xy) - min(float(point[1]) for point in polygon_xy)
    placement_radius_m = max(500.0, 0.75 * math.hypot(span_x, span_y))

    outward_x = float(real_xy[0]) - center_x
    outward_y = float(real_xy[1]) - center_y
    outward_norm = math.hypot(outward_x, outward_y)
    if outward_norm <= 1.0:
        heading_deg = float(_to_float(real_entry.get("headingDeg")) or 0.0)
        heading_rad = math.radians(heading_deg)
        outward_x = math.sin(heading_rad)
        outward_y = math.cos(heading_rad)
        outward_norm = 1.0
    unit_x = outward_x / outward_norm
    unit_y = outward_y / outward_norm
    opposite_xy = (
        center_x - unit_x * placement_radius_m,
        center_y - unit_y * placement_radius_m,
    )

    existing_ids = {
        int(_to_int(row.get("aircraftID")) or 0)
        for row in valid_entries
        if int(_to_int(row.get("aircraftID")) or 0) > 0
    }
    planner_entries = [real_entry]
    virtual_aircraft_ids: List[int] = []
    next_virtual_id = max(existing_ids | {real_aircraft_id}) + 1_000_000
    for virtual_index in range(1, stage_count):
        while next_virtual_id in existing_ids:
            next_virtual_id += 1
        virtual_aircraft_id = int(next_virtual_id)
        next_virtual_id += 1
        existing_ids.add(virtual_aircraft_id)
        virtual_aircraft_ids.append(virtual_aircraft_id)

        fraction = float(virtual_index) / float(stage_count - 1)
        virtual_xy = (
            float(real_xy[0]) + (float(opposite_xy[0]) - float(real_xy[0])) * fraction,
            float(real_xy[1]) + (float(opposite_xy[1]) - float(real_xy[1])) * fraction,
        )
        virtual_entry = deepcopy(real_entry)
        virtual_entry["aircraftID"] = int(virtual_aircraft_id)
        virtual_entry["coordinate"] = meters_to_coord(
            float(virtual_xy[0]),
            float(virtual_xy[1]),
            alt_m=float(real_coord.get("altitude", 0.0) or 0.0),
        )
        virtual_entry["headingDeg"] = (
            math.degrees(
                math.atan2(center_x - virtual_xy[0], center_y - virtual_xy[1])
            )
            + 360.0
        ) % 360.0
        # A copied live prediction still points at the real UAV and would
        # collapse every planning entry back onto the same live position.
        for key in (
            "linePredictedEntryCoordinate",
            "linePredictedHeadingDeg",
            "linePredictionLeadS",
            "lineTurnDirectionConfidence",
            "lineTurnDataAgeS",
            "lineTrendTurnSign",
            "lineTrendTurnRateDps",
            "lineTrendSampleCount",
            "turnSign",
            "turnRateDps",
        ):
            virtual_entry.pop(key, None)
        planner_entries.append(virtual_entry)

    return planner_entries, {
        "realAircraftID": int(real_aircraft_id),
        "virtualAircraftID": int(virtual_aircraft_ids[0]),
        "virtualAircraftIDs": list(virtual_aircraft_ids),
        "plannerAircraftIDs": [int(real_aircraft_id), *virtual_aircraft_ids],
        "stageCount": int(stage_count),
        "realEntryCoordinate": deepcopy(real_coord),
        "realEntryXY": (float(real_xy[0]), float(real_xy[1])),
    }


def _branch_aircraft_sequential_area_entries(
    aircraft_entries: List[Dict[str, Any]],
    mission_polygon: List[Dict[str, Any]],
    *,
    enabled: bool,
    stage_count: int = 2,
) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Create N-1 planning-only partners for every sticky branch owner.

    With several owners, each partner stays next to its real UAV instead of on
    the opposite side of the whole polygon.  The division planner therefore
    assigns adjacent strips to each real/virtual pair; the virtual strip is
    folded back onto that same real owner after planning.
    """

    valid_entries = [deepcopy(row) for row in aircraft_entries if isinstance(row, dict)]
    stage_count = max(2, int(stage_count))
    if not enabled or not valid_entries:
        return valid_entries, []
    if len(valid_entries) == 1:
        pair, context = _single_aircraft_sequential_area_entries(
            valid_entries,
            mission_polygon,
            enabled=True,
            stage_count=stage_count,
        )
        return pair, [context] if isinstance(context, dict) else []

    planner_entries: List[Dict[str, Any]] = []
    contexts: List[Dict[str, Any]] = []
    used_ids = {
        int(_to_int(row.get("aircraftID")) or 0)
        for row in valid_entries
        if int(_to_int(row.get("aircraftID")) or 0) > 0
    }
    for pair_index, real_entry in enumerate(valid_entries):
        pair, context = _single_aircraft_sequential_area_entries(
            [real_entry],
            mission_polygon,
            enabled=True,
            stage_count=stage_count,
        )
        if len(pair) != stage_count or not isinstance(context, dict):
            return valid_entries, []

        real_xy = context.get("realEntryXY")
        real_coord = context.get("realEntryCoordinate")
        if not (
            isinstance(real_xy, (tuple, list))
            and len(real_xy) >= 2
            and isinstance(real_coord, dict)
        ):
            return valid_entries, []
        remapped_virtual_ids: List[int] = []
        for virtual_index, virtual_entry in enumerate(pair[1:], start=1):
            virtual_id = int(_to_int(virtual_entry.get("aircraftID")) or 0)
            while virtual_id <= 0 or virtual_id in used_ids:
                virtual_id += 1
            virtual_entry["aircraftID"] = int(virtual_id)
            used_ids.add(int(virtual_id))
            remapped_virtual_ids.append(int(virtual_id))
            # Tiny deterministic local offsets resolve equal-position ties
            # while keeping every synthetic stage in its owner's neighbourhood.
            angle_rad = math.radians(
                float(
                    (
                        pair_index * 137.507764
                        + virtual_index * (360.0 / stage_count)
                    )
                    % 360.0
                )
            )
            offset_m = 2.0 * float(virtual_index)
            virtual_entry["coordinate"] = meters_to_coord(
                float(real_xy[0]) + math.cos(angle_rad) * offset_m,
                float(real_xy[1]) + math.sin(angle_rad) * offset_m,
                alt_m=float(real_coord.get("altitude", 0.0) or 0.0),
            )
            virtual_entry["headingDeg"] = float(
                _to_float(real_entry.get("headingDeg")) or 0.0
            )
        context["virtualAircraftID"] = int(remapped_virtual_ids[0])
        context["virtualAircraftIDs"] = list(remapped_virtual_ids)
        context["plannerAircraftIDs"] = [
            int(_to_int(real_entry.get("aircraftID")) or 0),
            *remapped_virtual_ids,
        ]
        context["stageCount"] = int(stage_count)
        planner_entries.extend(pair)
        contexts.append(context)

    return planner_entries, contexts


def _area_path_row_start_xy(row: Dict[str, Any]) -> tuple[float, float] | None:
    for key in (
        "areaSweepRouteStartXY",
        "waypointStartXY",
        "entryTPrimeXY",
        "targetXY",
    ):
        raw_xy = row.get(key)
        if isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2:
            return (float(raw_xy[0]), float(raw_xy[1]))
    route_xy = row.get("routeXY")
    if isinstance(route_xy, list) and route_xy:
        raw_xy = route_xy[0]
        if isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2:
            return (float(raw_xy[0]), float(raw_xy[1]))
    return None


def _area_path_row_end_xy(row: Dict[str, Any]) -> tuple[float, float] | None:
    for key in ("areaSweepRouteEndXY", "waypointEndXY"):
        raw_xy = row.get(key)
        if isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2:
            return (float(raw_xy[0]), float(raw_xy[1]))
    route_xy = row.get("routeXY")
    if isinstance(route_xy, list) and route_xy:
        raw_xy = route_xy[-1]
        if isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2:
            return (float(raw_xy[0]), float(raw_xy[1]))
    return None


def _area_path_row_capture_anchors_xy(
    row: Dict[str, Any],
) -> List[tuple[float, float]]:
    """Return the commanded capture anchors, excluding route-offset helpers."""

    scan_lines_xy: List[List[tuple[float, float]]] = []
    for raw_line in row.get("sweepLineListXY") or []:
        if not isinstance(raw_line, list):
            continue
        line_xy = [
            (float(raw_xy[0]), float(raw_xy[1]))
            for raw_xy in raw_line
            if isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2
        ]
        if len(line_xy) >= 2:
            scan_lines_xy.append(line_xy)
    if not scan_lines_xy:
        return []
    try:
        items = _area_sweep_items_xy(
            row,
            scan_lines_xy,
            deduped_scan_lines_xy=scan_lines_xy,
        )
    except Exception:
        return []
    anchors: List[tuple[float, float]] = []
    for item in items:
        raw_anchor = item.get("anchorXY") if isinstance(item, dict) else None
        if not (isinstance(raw_anchor, (tuple, list)) and len(raw_anchor) >= 2):
            continue
        anchor_xy = (float(raw_anchor[0]), float(raw_anchor[1]))
        if anchors and math.hypot(
            anchor_xy[0] - anchors[-1][0],
            anchor_xy[1] - anchors[-1][1],
        ) <= 1.0:
            continue
        anchors.append(anchor_xy)
    return anchors


def _area_path_row_capture_exit_xy(
    row: Dict[str, Any],
) -> tuple[float, float] | None:
    anchors = _area_path_row_capture_anchors_xy(row)
    if anchors:
        return anchors[-1]
    return _area_path_row_end_xy(row)


def _area_path_row_turn_speed_radius(
    row: Dict[str, Any],
) -> tuple[float, float] | None:
    """Return the production AREA speed/radius pair when it is explicit.

    ``resolvedVelMps`` is the historical planner field name, but its value is
    km/h.  Under-specified rows deliberately return ``None`` so old/fallback
    payloads retain the proven nearest-endpoint behavior.
    """

    resolved_vel_kmh = _to_float(row.get("resolvedVelMps"))
    if resolved_vel_kmh is None or resolved_vel_kmh <= 0.0:
        return None
    speed_mps = float(resolved_vel_kmh) / 3.6
    try:
        radius_m = float(_turn_radius_m_for_area_link(float(speed_mps)))
    except Exception:
        return None
    if not math.isfinite(radius_m) or radius_m <= 1.0:
        return None
    return float(speed_mps), float(radius_m)


def _area_path_row_planned_turn_approach_cost_m(
    row: Dict[str, Any],
) -> float | None:
    """Return the planner's fixed-wing approach cost to one AREA stage.

    Every width stage was independently solved from the same live/predicted
    aircraft state.  Its mission phase start therefore contains the turn arc
    and any straight ingress generated with that aircraft's turn radius.  The
    value is converted back to metres so the two terminal stages can be
    compared without mixing speed and time units.
    """

    origin_xy = row.get("originXY")
    origin_heading_deg = _to_float(row.get("originHeadingDeg"))
    speed_radius = _area_path_row_turn_speed_radius(row)
    if (
        not isinstance(origin_xy, (tuple, list))
        or len(origin_xy) < 2
        or origin_heading_deg is None
        or speed_radius is None
    ):
        return None
    speed_mps, _radius_m = speed_radius

    horizon_sec = _to_float(row.get("horizonSec"))
    tangent_xy = row.get("tangentXY")
    waypoint_start_xy = row.get("waypointStartXY")
    if (
        horizon_sec is not None
        and math.isfinite(horizon_sec)
        and horizon_sec >= 0.0
        and isinstance(tangent_xy, (tuple, list))
        and len(tangent_xy) >= 2
        and isinstance(waypoint_start_xy, (tuple, list))
        and len(waypoint_start_xy) >= 2
    ):
        straight_ingress_m = math.hypot(
            float(waypoint_start_xy[0]) - float(tangent_xy[0]),
            float(waypoint_start_xy[1]) - float(tangent_xy[1]),
        )
        explicit_turn_speed_mps = _to_float(row.get("turnSpeedMps"))
        turn_speed_mps = (
            float(explicit_turn_speed_mps)
            if explicit_turn_speed_mps is not None
            and math.isfinite(explicit_turn_speed_mps)
            and explicit_turn_speed_mps > 0.0
            else float(speed_mps)
        )
        return (
            float(horizon_sec) * float(turn_speed_mps)
        ) + float(straight_ingress_m)

    # Older planner rows expose only the capture/waypoint phase boundary.
    for phase_row in row.get("phaseRows") or []:
        if not isinstance(phase_row, dict):
            continue
        if str(phase_row.get("kind") or "").strip().lower() != "waypoint":
            continue
        start_sec = _to_float(phase_row.get("startSec"))
        if start_sec is not None and math.isfinite(start_sec) and start_sec >= 0.0:
            return float(start_sec) * float(speed_mps)
    return None


def _reverse_single_owner_width_stage_execution_order(
    sequence_rows: Dict[int, Dict[str, Any]],
    split_count: int,
) -> bool:
    """Choose either end of one owner's strip chain from the turn solution.

    Only the complete order is reversed, so the aircraft still consumes
    adjacent pieces (N..1) and never jumps through the middle.  Physical
    ``splitStage`` lineage remains untouched; ``areaSingleAircraftSequence`` is
    the execution order consumed by the path emitter.
    """

    split_count = int(split_count)
    if split_count < 2 or any(
        not isinstance(sequence_rows.get(sequence), dict)
        for sequence in range(1, split_count + 1)
    ):
        return False
    owner_counts = {
        int(_to_int(row.get("areaSequentialOwnerCount")) or 0)
        for row in sequence_rows.values()
        if isinstance(row, dict)
    }
    if owner_counts != {1}:
        return False

    first_row = sequence_rows[1]
    last_row = sequence_rows[split_count]
    first_origin = first_row.get("originXY")
    last_origin = last_row.get("originXY")
    first_heading_deg = _to_float(first_row.get("originHeadingDeg"))
    last_heading_deg = _to_float(last_row.get("originHeadingDeg"))
    if (
        not isinstance(first_origin, (tuple, list))
        or len(first_origin) < 2
        or not isinstance(last_origin, (tuple, list))
        or len(last_origin) < 2
        or first_heading_deg is None
        or last_heading_deg is None
        or math.hypot(
            float(first_origin[0]) - float(last_origin[0]),
            float(first_origin[1]) - float(last_origin[1]),
        )
        > 1.0
        or abs(
            (
                float(first_heading_deg)
                - float(last_heading_deg)
                + 180.0
            )
            % 360.0
            - 180.0
        )
        > 1.0
    ):
        # Costs produced from different synthetic/live starts are not
        # comparable; preserve the planner's canonical adjacency order.
        return False

    first_cost_m = _area_path_row_planned_turn_approach_cost_m(first_row)
    last_cost_m = _area_path_row_planned_turn_approach_cost_m(last_row)
    first_speed_radius = _area_path_row_turn_speed_radius(first_row)
    last_speed_radius = _area_path_row_turn_speed_radius(last_row)
    switch_margin_m = max(
        25.0,
        min(
            float(first_speed_radius[0]) if first_speed_radius is not None else 0.0,
            float(last_speed_radius[0]) if last_speed_radius is not None else 0.0,
        ),
    )
    if (
        first_cost_m is None
        or last_cost_m is None
        # About one second of nominal flight prevents numerical noise from
        # flipping the whole stage chain when both terminal approaches tie.
        or float(last_cost_m) + float(switch_margin_m) >= float(first_cost_m)
    ):
        return False

    for physical_sequence, row in sequence_rows.items():
        row["areaSingleAircraftSequence"] = (
            int(split_count) - int(physical_sequence) + 1
        )
        row["areaSequentialExecutionOrderReversed"] = True
    return True


def _area_path_row_execution_state(
    row: Dict[str, Any],
    entry_xy: tuple[float, float] | None,
) -> tuple[tuple[float, float] | None, float | None, bool]:
    """Return the actual capture exit, exit bearing, and route direction.

    AREA rows are stored in canonical sweep order, while the path builder
    reverses that order when the live/sequential entry is closer to the
    canonical end.  Sequential hand-over must make the same decision before
    assigning the following stage; otherwise stage 3 starts from stage 2's
    unused canonical end after stage 2 has actually flown in reverse.
    """

    anchors = _area_path_row_capture_anchors_xy(row)
    # Sequential AREA ingress legs are planning aids and can be oblique to the
    # actual sweep. Direction selection and hand-over must compare the live
    # entry with the capture route itself; otherwise an unrelated T0 -> WP_E
    # diagonal can reverse a stage and seed the next one from the wrong side.
    route_start_xy = anchors[0] if anchors else _area_path_row_start_xy(row)
    route_end_xy = anchors[-1] if anchors else _area_path_row_end_xy(row)
    reverse = bool(
        entry_xy is not None
        and route_start_xy is not None
        and route_end_xy is not None
        and math.hypot(
            float(entry_xy[0]) - float(route_end_xy[0]),
            float(entry_xy[1]) - float(route_end_xy[1]),
        )
        + 1.0e-6
        < math.hypot(
            float(entry_xy[0]) - float(route_start_xy[0]),
            float(entry_xy[1]) - float(route_start_xy[1]),
        )
    )
    ordered_anchors = list(reversed(anchors)) if reverse else list(anchors)
    exit_xy = (
        ordered_anchors[-1]
        if ordered_anchors
        else (route_start_xy if reverse else route_end_xy)
    )
    previous_xy: tuple[float, float] | None = (
        ordered_anchors[-2] if len(ordered_anchors) >= 2 else None
    )
    route_points_xy = [
        (float(raw_xy[0]), float(raw_xy[1]))
        for raw_xy in (row.get("routeXY") or [])
        if isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2
    ]
    ordered_route_xy = (
        list(reversed(route_points_xy)) if reverse else route_points_xy
    )
    if previous_xy is None and exit_xy is not None:
        for candidate_xy in reversed(ordered_route_xy[:-1]):
            if math.hypot(
                float(candidate_xy[0]) - float(exit_xy[0]),
                float(candidate_xy[1]) - float(exit_xy[1]),
            ) > 1.0:
                previous_xy = candidate_xy
                break
    if previous_xy is None:
        previous_xy = route_end_xy if reverse else route_start_xy
    exit_bearing_deg: float | None = None
    if exit_xy is not None and previous_xy is not None:
        delta_x = float(exit_xy[0]) - float(previous_xy[0])
        delta_y = float(exit_xy[1]) - float(previous_xy[1])
        if math.hypot(delta_x, delta_y) > 1.0e-6:
            exit_bearing_deg = (
                math.degrees(math.atan2(delta_x, delta_y)) + 360.0
            ) % 360.0
    return exit_xy, exit_bearing_deg, bool(reverse)


def _area_path_row_exit_bearing_deg(row: Dict[str, Any]) -> float | None:
    """Heading the UAV still holds as it leaves this piece."""

    capture_anchors = _area_path_row_capture_anchors_xy(row)
    end_xy = capture_anchors[-1] if capture_anchors else _area_path_row_end_xy(row)
    if end_xy is None:
        return None
    previous_xy: tuple[float, float] | None = (
        capture_anchors[-2] if len(capture_anchors) >= 2 else None
    )
    route_xy = row.get("routeXY")
    if previous_xy is None and isinstance(route_xy, list) and len(route_xy) >= 2:
        for raw_xy in reversed(route_xy[:-1]):
            if not (isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2):
                continue
            candidate = (float(raw_xy[0]), float(raw_xy[1]))
            if math.hypot(candidate[0] - end_xy[0], candidate[1] - end_xy[1]) > 1.0:
                previous_xy = candidate
                break
    if previous_xy is None:
        previous_xy = _area_path_row_start_xy(row)
    if previous_xy is None:
        return None
    delta_x = float(end_xy[0]) - float(previous_xy[0])
    delta_y = float(end_xy[1]) - float(previous_xy[1])
    if math.hypot(delta_x, delta_y) <= 1e-6:
        return None
    return (math.degrees(math.atan2(delta_x, delta_y)) + 360.0) % 360.0


def _sequential_area_ordered_pair(
    planner_result: Any,
    context: Dict[str, Any],
) -> List[Dict[str, Any]] | None:
    """Return this owner's planner rows ordered in their flown sequence."""

    real_aircraft_id = int(_to_int(context.get("realAircraftID")) or 0)
    planner_aircraft_ids = {
        int(_to_int(value) or 0)
        for value in (
            context.get("plannerAircraftIDs")
            or [
                real_aircraft_id,
                *(context.get("virtualAircraftIDs") or []),
                context.get("virtualAircraftID"),
            ]
        )
        if int(_to_int(value) or 0) > 0
    }
    real_entry_xy = context.get("realEntryXY")
    if real_aircraft_id <= 0 or len(planner_aircraft_ids) < 2:
        return None
    if not (isinstance(real_entry_xy, (tuple, list)) and len(real_entry_xy) >= 2):
        return None
    reference_xy = (float(real_entry_xy[0]), float(real_entry_xy[1]))
    rows = [
        row
        for row in (getattr(planner_result, "expected_paths", None) or [])
        if isinstance(row, dict)
        and int(_to_int(row.get("aircraftID")) or 0)
        in planner_aircraft_ids
    ]
    if len(rows) != len(planner_aircraft_ids):
        return None
    rows.sort(
        key=lambda row: math.hypot(
            float((_area_path_row_start_xy(row) or reference_xy)[0]) - reference_xy[0],
            float((_area_path_row_start_xy(row) or reference_xy)[1]) - reference_xy[1],
        )
    )
    return rows


def _sequential_area_second_pass_entries(
    planner_result: Any,
    planner_aircraft_entries: List[Dict[str, Any]],
    contexts: List[Dict[str, Any]],
) -> List[Dict[str, Any]] | None:
    """Re-seed every planning-only partner at its owner's first-piece exit.

    The first planner pass parks the partner beside the real UAV, so the second
    piece's ingress (T0/T'/WP_S) is solved from wherever the aircraft happened
    to sit at replan time - typically far outside the Area, which is why the
    second piece films from a long standoff.  Moving the partner to the first
    piece's exit makes the planner build that ingress as a turn back in from
    the path the UAV has just flown.
    """

    if not contexts or not planner_aircraft_entries:
        return None
    updated = [deepcopy(row) for row in planner_aircraft_entries if isinstance(row, dict)]
    updated_by_id = {
        int(_to_int(row.get("aircraftID")) or 0): row for row in updated
    }
    moved = 0
    for context in contexts:
        rows = _sequential_area_ordered_pair(planner_result, context)
        if rows is None:
            return None
        real_entry_xy = context.get("realEntryXY")
        if not (
            isinstance(real_entry_xy, (tuple, list))
            and len(real_entry_xy) >= 2
        ):
            return None
        current_entry_xy = (
            float(real_entry_xy[0]),
            float(real_entry_xy[1]),
        )
        previous_exit_bearing_deg: float | None = None
        altitude_m = float(
            ((context.get("realEntryCoordinate") or {}).get("altitude", 0.0)) or 0.0
        )
        stage_entry_xy: Dict[int, tuple[float, float]] = {}
        for sequence, current_row in enumerate(rows, start=1):
            if sequence >= 2:
                planner_aircraft_id = int(
                    _to_int(current_row.get("aircraftID")) or 0
                )
                virtual_entry = updated_by_id.get(planner_aircraft_id)
                if virtual_entry is None:
                    return None
                virtual_entry["coordinate"] = meters_to_coord(
                    float(current_entry_xy[0]),
                    float(current_entry_xy[1]),
                    alt_m=float(altitude_m),
                )
                if previous_exit_bearing_deg is not None:
                    virtual_entry["headingDeg"] = float(
                        previous_exit_bearing_deg
                    )
                # A live prediction copied from the real UAV would drag the
                # partner back onto the aircraft's current track instead of
                # the preceding stage's actual capture exit.
                for key in (
                    "linePredictedEntryCoordinate",
                    "linePredictedHeadingDeg",
                    "linePredictionLeadS",
                    "lineTurnDirectionConfidence",
                    "lineTurnDataAgeS",
                    "lineTrendTurnSign",
                    "lineTrendTurnRateDps",
                    "lineTrendSampleCount",
                    "turnSign",
                    "turnRateDps",
                ):
                    virtual_entry.pop(key, None)
                stage_entry_xy[int(sequence)] = (
                    float(current_entry_xy[0]),
                    float(current_entry_xy[1]),
                )
            (
                current_exit_xy,
                current_exit_bearing_deg,
                _current_reversed,
            ) = _area_path_row_execution_state(
                current_row,
                current_entry_xy,
            )
            if current_exit_xy is None:
                return None
            current_entry_xy = (
                float(current_exit_xy[0]),
                float(current_exit_xy[1]),
            )
            previous_exit_bearing_deg = current_exit_bearing_deg
        if 2 in stage_entry_xy:
            context["secondEntryXY"] = stage_entry_xy[2]
        context["stageEntryXY"] = stage_entry_xy
        moved += 1
    return updated if moved else None


def _sequential_area_second_pass_is_consistent(
    planner_result: Any,
    contexts: List[Dict[str, Any]],
) -> bool:
    """Reject a re-plan that reshuffled which piece each owner flies first.

    Ordering still keys off the real UAV entry, so the near piece must remain
    the one the planner handed to the real aircraft.  Anything else means the
    re-seeded partner stole the first piece and the sequence would invert.
    """

    for context in contexts:
        rows = _sequential_area_ordered_pair(planner_result, context)
        if rows is None:
            return False
        real_aircraft_id = int(_to_int(context.get("realAircraftID")) or 0)
        if int(_to_int(rows[0].get("aircraftID")) or 0) != real_aircraft_id:
            return False
    return True


def _collapse_single_aircraft_sequential_area_result(
    planner_result: Any,
    context: Dict[str, Any] | None,
) -> bool:
    """Fold the planning-only owner into two ordered missions for one UAV."""

    if not isinstance(context, dict):
        return True
    real_aircraft_id = int(_to_int(context.get("realAircraftID")) or 0)
    planner_aircraft_ids = {
        int(_to_int(value) or 0)
        for value in (
            context.get("plannerAircraftIDs")
            or [
                real_aircraft_id,
                *(context.get("virtualAircraftIDs") or []),
                context.get("virtualAircraftID"),
            ]
        )
        if int(_to_int(value) or 0) > 0
    }
    real_entry_xy = context.get("realEntryXY")
    rows = [
        row
        for row in (getattr(planner_result, "expected_paths", None) or [])
        if isinstance(row, dict)
        and int(_to_int(row.get("aircraftID")) or 0)
        in planner_aircraft_ids
    ]
    if (
        real_aircraft_id <= 0
        or len(planner_aircraft_ids) < 2
        or len(rows) != len(planner_aircraft_ids)
    ):
        return False

    if isinstance(real_entry_xy, (tuple, list)) and len(real_entry_xy) >= 2:
        reference_xy = (float(real_entry_xy[0]), float(real_entry_xy[1]))
        rows.sort(
            key=lambda row: math.hypot(
                float((_area_path_row_start_xy(row) or reference_xy)[0]) - reference_xy[0],
                float((_area_path_row_start_xy(row) or reference_xy)[1]) - reference_xy[1],
            )
        )

    real_entry_xy = context.get("realEntryXY")
    current_entry_xy: tuple[float, float] | None = (
        (float(real_entry_xy[0]), float(real_entry_xy[1]))
        if isinstance(real_entry_xy, (tuple, list)) and len(real_entry_xy) >= 2
        else None
    )
    previous_exit_bearing_deg: float | None = None
    altitude_m = float(
        ((context.get("realEntryCoordinate") or {}).get("altitude", 0.0)) or 0.0
    )
    for sequence, row in enumerate(rows, start=1):
        original_owner = int(_to_int(row.get("aircraftID")) or 0)
        row["aircraftID"] = int(real_aircraft_id)
        row["areaSingleAircraftSequentialSplit"] = True
        row["areaSingleAircraftSequence"] = int(sequence)
        row["areaSingleAircraftOriginalPlannerOwner"] = int(original_owner)
        if sequence >= 2 and current_entry_xy is not None:
            row["areaSingleAircraftEntryCoordinate"] = meters_to_coord(
                float(current_entry_xy[0]),
                float(current_entry_xy[1]),
                alt_m=float(altitude_m),
            )
            if previous_exit_bearing_deg is not None:
                row["areaSingleAircraftEntryHeadingDeg"] = float(
                    previous_exit_bearing_deg
                )
        (
            current_exit_xy,
            current_exit_bearing_deg,
            execution_reversed,
        ) = _area_path_row_execution_state(
            row,
            current_entry_xy,
        )
        row["areaSingleAircraftExecutionReversed"] = bool(execution_reversed)
        if current_exit_xy is not None:
            current_entry_xy = (
                float(current_exit_xy[0]),
                float(current_exit_xy[1]),
            )
        previous_exit_bearing_deg = current_exit_bearing_deg

    split_result = getattr(planner_result, "split_result", None)
    for piece in getattr(split_result, "pieces", None) or []:
        if (
            int(_to_int(getattr(piece, "assigned_uav", None)) or 0)
            in planner_aircraft_ids
        ):
            piece.assigned_uav = int(real_aircraft_id)
        data = getattr(piece, "data", None)
        if isinstance(data, dict):
            for key in ("aircraftID", "assignedUAV", "assigned_uav"):
                if int(_to_int(data.get(key)) or 0) in planner_aircraft_ids:
                    data[key] = int(real_aircraft_id)
    return True


def _apply_width_split_sequence_metadata(
    path_rows: List[Dict[str, Any]],
    split_pieces: List[SplitPiece],
    entry_coordinate_by_aircraft: Dict[int, Dict[str, Any]] | None = None,
) -> int:
    """Carry stage 1..N sequencing from split pieces into final path rows.

    The headless planner keeps ``splitStage`` on ``SplitPiece.data`` but does
    not copy it to ``expected_paths``. Without this bridge, downstream path
    building sees unrelated AREA missions and cannot enter each next stage from
    the previous stage's capture exit.
    """

    sequence_by_piece: Dict[tuple[int, int], tuple[int, int]] = {}
    outer_meta_by_piece: Dict[tuple[int, int], Dict[str, Any]] = {}
    for piece in split_pieces:
        if not isinstance(piece, SplitPiece):
            continue
        data = piece.data if isinstance(piece.data, dict) else {}
        aircraft_id = int(_to_int(piece.assigned_uav) or 0)
        piece_index = int(_to_int(piece.piece_index) or 0)
        if aircraft_id > 0 and piece_index > 0:
            outer_meta_by_piece[(aircraft_id, piece_index)] = {
                key: deepcopy(data.get(key))
                for key in (
                    "areaSequentialOwnerSlot",
                    "areaSequentialOwnerCount",
                    "areaOuterOwner",
                    "areaOuterSide",
                    "areaOuterFirstSweep",
                )
                if key in data
            }
        split_count = int(_to_int(data.get("splitCount")) or 0)
        if not bool(data.get("areaSequentialWidthSplit")) or split_count < 2:
            continue
        sequence = int(_to_int(data.get("splitStage")) or 0)
        if (
            aircraft_id > 0
            and piece_index > 0
            and 1 <= sequence <= split_count
        ):
            sequence_by_piece[(aircraft_id, piece_index)] = (
                sequence,
                split_count,
            )

    sequenced_rows: Dict[int, Dict[int, Dict[str, Any]]] = {}
    split_count_by_aircraft: Dict[int, int] = {}
    for row in path_rows:
        if not isinstance(row, dict):
            continue
        aircraft_id = int(_to_int(row.get("aircraftID")) or 0)
        piece_index = int(_to_int(row.get("pieceIndex")) or 0)
        outer_meta = outer_meta_by_piece.get((aircraft_id, piece_index))
        if isinstance(outer_meta, dict):
            row.update(deepcopy(outer_meta))
        sequence_info = sequence_by_piece.get((aircraft_id, piece_index))
        if sequence_info is None:
            continue
        sequence, split_count = sequence_info
        row["areaSequentialWidthSplit"] = True
        row["splitStage"] = int(sequence)
        row["splitCount"] = int(split_count)
        row["areaSingleAircraftSequentialSplit"] = True
        row["areaSingleAircraftSequence"] = int(sequence)
        row["areaSingleAircraftOriginalPlannerOwner"] = int(aircraft_id)
        sequenced_rows.setdefault(int(aircraft_id), {})[int(sequence)] = row
        split_count_by_aircraft[int(aircraft_id)] = max(
            int(split_count_by_aircraft.get(int(aircraft_id), 0)),
            int(split_count),
        )

    completed_sequences = 0
    for aircraft_id, sequence_rows in sequenced_rows.items():
        split_count = int(split_count_by_aircraft.get(int(aircraft_id), 0))
        if split_count < 2 or any(
            not isinstance(sequence_rows.get(sequence), dict)
            for sequence in range(1, split_count + 1)
        ):
            continue
        if _reverse_single_owner_width_stage_execution_order(
            sequence_rows,
            split_count,
        ):
            sequence_rows = {
                int(_to_int(row.get("areaSingleAircraftSequence")) or 0): row
                for row in sequence_rows.values()
                if isinstance(row, dict)
            }
            sequenced_rows[int(aircraft_id)] = sequence_rows
        entry_coord = (
            (entry_coordinate_by_aircraft or {}).get(int(aircraft_id))
            if isinstance(entry_coordinate_by_aircraft, dict)
            else None
        )
        altitude_m = float(
            _to_float(
                (entry_coord or {}).get("altitude")
                if isinstance(entry_coord, dict)
                else None
            )
            or 0.0
        )
        first_entry_xy = (
            coord_to_xy(entry_coord)
            if isinstance(entry_coord, dict)
            else None
        )
        current_entry_xy: tuple[float, float] | None = (
            (float(first_entry_xy[0]), float(first_entry_xy[1]))
            if first_entry_xy is not None
            else _area_path_row_start_xy(sequence_rows[1])
        )
        previous_exit_bearing_deg: float | None = None
        sequence_complete = current_entry_xy is not None
        for sequence in range(1, split_count + 1):
            current_row = sequence_rows[sequence]
            if sequence >= 2 and current_entry_xy is not None:
                next_entry = meters_to_coord(
                    float(current_entry_xy[0]),
                    float(current_entry_xy[1]),
                    alt_m=float(altitude_m),
                )
                current_row["areaSingleAircraftEntryCoordinate"] = deepcopy(
                    next_entry
                )
                current_row["areaPassEntryCoordinate"] = deepcopy(next_entry)
                if previous_exit_bearing_deg is not None:
                    current_row["areaSingleAircraftEntryHeadingDeg"] = float(
                        previous_exit_bearing_deg
                    )
            (
                current_exit_xy,
                current_exit_bearing_deg,
                execution_reversed,
            ) = _area_path_row_execution_state(
                current_row,
                current_entry_xy,
            )
            current_row["areaSingleAircraftExecutionReversed"] = bool(
                execution_reversed
            )
            if current_exit_xy is None:
                sequence_complete = False
                break
            current_entry_xy = (
                float(current_exit_xy[0]),
                float(current_exit_xy[1]),
            )
            previous_exit_bearing_deg = current_exit_bearing_deg
        if sequence_complete:
            completed_sequences += 1
    return int(completed_sequences)


def _area_assignment_pass_rank(path_row: Dict[str, Any]) -> int:
    pass_name = _normalize_area_assignment_pass(
        path_row.get("areaAssignedCoveragePass")
        or path_row.get("activeCoveragePass")
    )
    if pass_name == "forward":
        return 0
    if pass_name == "reverse":
        return 1
    return 2


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


def _input_mission_index(
    input_plan: Dict[str, Any] | None,
    input_mission_id: int,
) -> Optional[int]:
    """Position of an inputMissionID in the plan's own ordering."""

    if not isinstance(input_plan, dict):
        return None
    for index, mission in enumerate(input_plan.get("inputMissionList") or []):
        if not isinstance(mission, dict):
            continue
        if _to_int(mission.get("inputMissionID")) == int(input_mission_id):
            return int(index)
    return None


def _lah_operation_zones_from_input_plan(
    input_plan: Dict[str, Any] | None,
    *,
    max_mission_index: int | None = None,
) -> List[Dict[str, Any]]:
    """Mission corridors/areas the manned route may be flown inside.

    ``max_mission_index`` drops every mission past the one being flown to.  The
    route is a shortest path through the *union* of these zones, so a corridor
    belonging to a far-future mission is otherwise a free highway: the manned
    aircraft would ride the last mission's line out and back rather than
    stepping through the regions in order.
    """

    zones: List[Dict[str, Any]] = []
    if not isinstance(input_plan, dict):
        return zones
    limit = _to_int(max_mission_index)
    for mission_index, mission in enumerate(input_plan.get("inputMissionList") or []):
        if not isinstance(mission, dict):
            continue
        if limit is not None and int(mission_index) > int(limit):
            continue
        input_mission_id = _to_int(mission.get("inputMissionID"))
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        for row in detail.get("lineList") or []:
            if not isinstance(row, dict):
                continue
            coords = _normalize_coord_list(row.get("coordinateList"))
            width_m = _to_float(row.get("width") or detail.get("sourceLineWidthM"))
            if len(coords) < 2 or width_m is None or float(width_m) <= 0.0:
                continue
            zones.append(
                {
                    "zoneType": "line",
                    "coordinateList": coords,
                    "widthM": float(width_m),
                    "inputMissionID": input_mission_id,
                    "missionIndex": int(mission_index),
                }
            )
        for row in detail.get("areaList") or []:
            if not isinstance(row, dict):
                continue
            coords = _normalize_coord_list(row.get("coordinateList"))
            if len(coords) < 3:
                continue
            zones.append(
                {
                    "zoneType": "area",
                    "coordinateList": coords,
                    "isHole": bool(row.get("isHole")),
                    "inputMissionID": input_mission_id,
                    "missionIndex": int(mission_index),
                }
            )
    return zones


def _lah_route_coordinates_from_info(info: Dict[str, Any]) -> List[Dict[str, Any]]:
    coords = _normalize_coord_list(info.get("coordinateList"))
    if coords:
        return coords
    for row in info.get("lineList") or []:
        if not isinstance(row, dict):
            continue
        coords = _normalize_coord_list(row.get("coordinateList"))
        if coords:
            return coords
    anchor = _area_anchor_coordinate(info.get("areaList"))
    return [anchor] if anchor is not None else []


def _lah_formation_target_coordinate(
    coord: Dict[str, Any],
    aircraft_id: int,
) -> Dict[str, Any]:
    target = dict(coord)
    north_m = 100.0 if int(aircraft_id) == 2 else -100.0 if int(aircraft_id) == 3 else 0.0
    if north_m:
        target["latitude"] = float(target["latitude"]) + (float(north_m) / 111_132.92)
    return target


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
    reserved_seen: Set[int] = set()
    max_avoid = max(avoid) if avoid else 0
    while len(reserved) < needed:
        request_count = needed - len(reserved)
        reservation = ReplanIdReservation.reserve(individual_count=int(request_count))
        batch = [reservation.next_individual() for _ in range(int(request_count))]
        for value in batch:
            value_int = int(value)
            if value_int in avoid or value_int in reserved_seen:
                continue
            reserved.append(value_int)
            reserved_seen.add(value_int)
        if len(reserved) >= needed:
            break
        max_batch = max([int(value) for value in batch], default=0)
        if max_batch <= max_avoid:
            catch_up_count = max(needed - len(reserved), max_avoid - max_batch + needed)
            catch_up = ReplanIdReservation.reserve(individual_count=int(catch_up_count))
            for _ in range(int(catch_up_count)):
                value_int = int(catch_up.next_individual())
                if value_int in avoid or value_int in reserved_seen:
                    continue
                reserved.append(value_int)
                reserved_seen.add(value_int)
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
    if mission_type in (1, 7):
        return True
    if mission_type in (2, 3, 4, 5, 6):
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

    if mission_type in (1, 7):
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

    if mission_type in (2, 3, 4, 5, 6):
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
    if int(piece.mission_type) in (2, 3, 4, 5, 6) and len(coords) >= 3:
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

    for key in ("_lahPreserveLineEndpoints", "_lahConstraintLineList"):
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


def _branch_area_ownership_for_target(
    planning_mode_ctx: Dict[str, Any] | None,
    target_input_id: int,
) -> Optional[Dict[int, List[int]]]:
    """Return sticky ``branch_index -> [aircraftID]`` if this is a Type 2/3 branch
    area mission, else ``None``.

    각자도생 area missions (경계 areaList) must keep each area monitored by its
    single owner UAV rather than being re-divided across the whole pool, so a
    next-collab replan reads the persisted branch ownership by package ID.
    """
    if _branch_ownership_store is None or not isinstance(planning_mode_ctx, dict):
        return None
    # 2/3 = 각자도생 브랜치, 4 = 도넛 밴드(각자도생 소유 공유).
    if _to_int(planning_mode_ctx.get("package_type")) not in (2, 3, 4):
        return None
    pkg_id = _to_int(planning_mode_ctx.get("inputMissionPackageID"))
    if pkg_id is None or pkg_id <= 0:
        return None
    try:
        meta = _branch_ownership_store.get_branch_meta(pkg_id)
    except Exception:
        return None
    branch_ids = set(int(x) for x in (meta.get("branch_mission_ids") or []))
    ownership = meta.get("ownership") or {}
    if int(target_input_id) not in branch_ids or not ownership:
        return None
    return {int(k): [int(a) for a in v] for k, v in ownership.items()}


def _line_geometry_xy(coords: Any) -> Optional[LineString]:
    points_xy: List[tuple[float, float]] = []
    for coord in _normalize_coord_list(coords):
        point_xy = coord_to_xy(coord)
        if point_xy is None:
            continue
        point = (float(point_xy[0]), float(point_xy[1]))
        if points_xy and math.hypot(
            points_xy[-1][0] - point[0],
            points_xy[-1][1] - point[1],
        ) < 0.01:
            continue
        points_xy.append(point)
    if len(points_xy) < 2:
        return None
    try:
        line = LineString(points_xy)
    except Exception:
        return None
    return line if not line.is_empty and float(line.length or 0.0) > 0.01 else None


def _mission_artifact_branch_index(
    missions: List[Dict[str, Any]],
    *,
    branch_count: int,
) -> Optional[int]:
    """Read the durable branch marker emitted by newer 0302 artifacts."""

    indices: Set[int] = set()
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        info = _template_mission_info(mission)
        for raw_index in (
            mission.get("branchIndex"),
            mission.get("sourceBranchIndex"),
            info.get("branchIndex"),
            info.get("sourceBranchIndex"),
        ):
            branch_index = _to_int(raw_index)
            if branch_index is None:
                continue
            if not (0 <= int(branch_index) < int(branch_count)):
                return None
            indices.add(int(branch_index))
    if len(indices) != 1:
        return None
    return next(iter(indices))


def _individual_mission_artifact_geometries(
    missions: List[Dict[str, Any]],
    *,
    kind: str,
) -> List[Any]:
    geometries: List[Any] = []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        info = _template_mission_info(mission)
        if kind == "area":
            area_rows = info.get("areaList") if isinstance(info.get("areaList"), list) else []
            for row in area_rows:
                if not isinstance(row, dict) or bool(row.get("isHole")):
                    continue
                poly = _coord_list_to_polygon_xy(row.get("coordinateList"))
                if poly is not None and not poly.is_empty and float(poly.area or 0.0) > 1.0:
                    geometries.append(poly)
            continue

        # Prefer the immutable deployment/source centerline when it exists.
        # Older initial plans only contain lineList, so retain that fallback.
        coordinate_candidates = (
            info.get("sourceCoordinateList"),
            info.get("lineDeploymentCoordinateList"),
        )
        selected_lines: List[LineString] = []
        for candidate in coordinate_candidates:
            line = _line_geometry_xy(candidate)
            if line is not None:
                selected_lines = [line]
                break
        if not selected_lines:
            line_rows = info.get("lineList") if isinstance(info.get("lineList"), list) else []
            selected_lines = [
                line
                for row in line_rows
                if isinstance(row, dict)
                for line in [_line_geometry_xy(row.get("coordinateList"))]
                if line is not None
            ]
        geometries.extend(selected_lines)
    return geometries


def _sampled_line_distance_m(candidate: LineString, reference: LineString) -> float:
    if candidate.is_empty or reference.is_empty:
        return math.inf
    length_m = float(candidate.length or 0.0)
    if length_m <= 0.01:
        return math.inf
    distances: List[float] = []
    for sample_index in range(9):
        point = candidate.interpolate(length_m * float(sample_index) / 8.0)
        distances.append(float(point.distance(reference)))
    return sum(distances) / float(len(distances)) if distances else math.inf


def _match_artifact_geometries_to_branch(
    geometries: List[Any],
    source_branches: List[Dict[str, Any]],
    *,
    kind: str,
) -> Optional[int]:
    if not geometries or not source_branches:
        return None

    if kind == "area":
        try:
            artifact_geometry = unary_union(geometries)
        except Exception:
            return None
        artifact_area = float(getattr(artifact_geometry, "area", 0.0) or 0.0)
        if artifact_geometry.is_empty or artifact_area <= 1.0:
            return None
        overlap_scores: List[tuple[float, int]] = []
        for branch_index, branch in enumerate(source_branches):
            source_poly = _coord_list_to_polygon_xy(branch.get("coordinateList"))
            if source_poly is None or source_poly.is_empty:
                overlap_scores.append((0.0, int(branch_index)))
                continue
            try:
                overlap = float(artifact_geometry.intersection(source_poly).area or 0.0)
            except Exception:
                overlap = 0.0
            overlap_scores.append((max(0.0, min(1.0, overlap / artifact_area)), int(branch_index)))
        overlap_scores.sort(reverse=True)
        best_score, best_index = overlap_scores[0]
        runner_up = overlap_scores[1][0] if len(overlap_scores) > 1 else 0.0
        if best_score < 0.55:
            return None
        if len(overlap_scores) > 1 and best_score - runner_up < 0.20:
            return None
        return int(best_index)

    artifact_lines = [
        geometry
        for geometry in geometries
        if isinstance(geometry, LineString)
        and not geometry.is_empty
        and float(geometry.length or 0.0) > 0.01
    ]
    if not artifact_lines:
        return None
    distance_scores: List[tuple[float, int, float]] = []
    for branch_index, branch in enumerate(source_branches):
        source_line = _line_geometry_xy(branch.get("coordinateList"))
        if source_line is None:
            continue
        weighted_distance = 0.0
        total_weight = 0.0
        for artifact_line in artifact_lines:
            weight = max(1.0, float(artifact_line.length or 0.0))
            weighted_distance += _sampled_line_distance_m(artifact_line, source_line) * weight
            total_weight += weight
        average_distance = weighted_distance / total_weight if total_weight > 0.0 else math.inf
        source_width_m = max(0.0, float(_to_float(branch.get("width")) or 0.0))
        distance_scores.append((float(average_distance), int(branch_index), source_width_m))
    if not distance_scores:
        return None
    distance_scores.sort(key=lambda item: (item[0], item[1]))
    best_distance, best_index, source_width_m = distance_scores[0]
    tolerance_m = max(250.0, source_width_m * 0.75)
    if not math.isfinite(best_distance) or best_distance > tolerance_m:
        return None
    if len(distance_scores) > 1:
        runner_up_distance = distance_scores[1][0]
        required_margin_m = max(50.0, source_width_m * 0.10)
        if runner_up_distance - best_distance < required_margin_m:
            return None
    return int(best_index)


def _recover_type2_branch_ownership_from_source_artifacts(
    *,
    input_data: Dict[str, Any],
    target_input_id: int,
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    target_aircraft_ids: List[int],
) -> tuple[Dict[int, List[int]], Optional[int], str]:
    """Rebuild immutable ownership only from already-assigned source IMP geometry.

    This deliberately never consults current aircraft positions.  A current
    position is valid for route entry, but it is not evidence that ownership may
    move to a different self-reliance branch.
    """

    try:
        from modules.mission_planning.pipelines.ground_maneuver_mode import (
            detect_ground_maneuver_profile,
        )

        profile = detect_ground_maneuver_profile(input_data, package_type=2)
    except Exception:
        profile = None
    if not isinstance(profile, dict):
        return {}, None, ""

    branch_count = _to_int(profile.get("branchCount")) or 0
    branch_ids = [
        int(mission_id)
        for mission_id in (profile.get("branchInputMissionIDs") or [])
        if _to_int(mission_id) is not None
    ]
    if branch_count <= 0 or int(target_input_id) not in set(branch_ids):
        return {}, None, ""

    missions_by_order = profile.get("missionsByOrder")
    if not isinstance(missions_by_order, dict):
        return {}, None, ""
    profile_entries: Dict[int, Dict[str, Any]] = {}
    for raw_entry in missions_by_order.values():
        if not isinstance(raw_entry, dict):
            continue
        mission_id = _to_int(raw_entry.get("inputMissionID"))
        branches = raw_entry.get("branches")
        if (
            mission_id is None
            or not isinstance(branches, list)
            or len(branches) != int(branch_count)
        ):
            continue
        profile_entries[int(mission_id)] = raw_entry

    phase_ids = [int(target_input_id)]
    anchor_id = _to_int(profile.get("anchorInputMissionID"))
    if anchor_id is not None and int(anchor_id) not in phase_ids:
        phase_ids.append(int(anchor_id))
    phase_ids.extend(
        mission_id
        for mission_id in branch_ids
        if int(mission_id) not in set(phase_ids)
    )

    required_ids = {
        int(aircraft_id)
        for aircraft_id in target_aircraft_ids
        if _to_int(aircraft_id) is not None and int(aircraft_id) > 3
    }
    candidate_ids = sorted(
        int(aircraft_id)
        for aircraft_id in packages_by_aircraft
        if _to_int(aircraft_id) is not None and int(aircraft_id) > 3
    )
    if not required_ids or not candidate_ids:
        return {}, None, ""

    for phase_id in phase_ids:
        profile_entry = profile_entries.get(int(phase_id))
        if not isinstance(profile_entry, dict):
            continue
        kind = str(profile_entry.get("kind") or "").strip().lower()
        if kind not in {"line", "area"}:
            continue
        source_branches = profile_entry.get("branches")
        if not isinstance(source_branches, list):
            continue

        aircraft_to_branch: Dict[int, int] = {}
        evidence_mode = "geometry"
        for aircraft_id in candidate_ids:
            package = packages_by_aircraft.get(int(aircraft_id))
            missions = (
                package.get("individualMissionList")
                if isinstance(package, dict)
                and isinstance(package.get("individualMissionList"), list)
                else []
            )
            phase_missions = [
                mission
                for mission in missions
                if isinstance(mission, dict)
                and _mission_input_id(mission) == int(phase_id)
            ]
            if not phase_missions:
                continue
            branch_index = _mission_artifact_branch_index(
                phase_missions,
                branch_count=int(branch_count),
            )
            if branch_index is not None:
                evidence_mode = "marker"
            else:
                geometries = _individual_mission_artifact_geometries(
                    phase_missions,
                    kind=kind,
                )
                branch_index = _match_artifact_geometries_to_branch(
                    geometries,
                    source_branches,
                    kind=kind,
                )
            if branch_index is not None:
                aircraft_to_branch[int(aircraft_id)] = int(branch_index)

        if not required_ids.issubset(set(aircraft_to_branch)):
            continue
        if set(aircraft_to_branch.values()) != set(range(int(branch_count))):
            continue
        ownership: Dict[int, List[int]] = {
            int(branch_index): [] for branch_index in range(int(branch_count))
        }
        for aircraft_id, branch_index in sorted(aircraft_to_branch.items()):
            ownership[int(branch_index)].append(int(aircraft_id))
        if all(ownership.values()):
            return ownership, int(phase_id), str(evidence_mode)
    return {}, None, ""


def _resolve_locked_type2_ownership_with_artifact_recovery(
    *,
    input_data: Dict[str, Any],
    target_input_id: int,
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    target_aircraft_ids: List[int],
    emit: Callable[[str], None],
) -> tuple[bool, Dict[int, List[int]]]:
    if _branch_ownership_store is None:
        return False, {}
    try:
        is_branch_mission = _branch_ownership_store.is_locked_type2_branch_mission(
            input_data,
            int(target_input_id),
        )
        ownership = _branch_ownership_store.get_locked_type2_branch_ownership(
            input_data,
            int(target_input_id),
        )
    except Exception:
        return False, {}
    if not is_branch_mission or ownership:
        return bool(is_branch_mission), ownership

    recovered, evidence_mission_id, evidence_mode = (
        _recover_type2_branch_ownership_from_source_artifacts(
            input_data=input_data,
            target_input_id=int(target_input_id),
            packages_by_aircraft=packages_by_aircraft,
            target_aircraft_ids=target_aircraft_ids,
        )
    )
    if not recovered:
        return True, {}

    try:
        from modules.mission_planning.pipelines.ground_maneuver_mode import (
            detect_ground_maneuver_profile,
        )

        profile = detect_ground_maneuver_profile(input_data, package_type=2)
        package_id = _to_int(
            input_data.get("inputMissionPackageID")
            or input_data.get("InputMissionPackageID")
            or input_data.get("inputMissionPackageId")
        )
        if not isinstance(profile, dict) or package_id is None or package_id <= 0:
            return True, {}
        _branch_ownership_store.register_branch_ownership(
            package_id=int(package_id),
            branch_count=int(profile.get("branchCount") or len(recovered)),
            ownership=recovered,
            branch_mission_ids=profile.get("branchInputMissionIDs") or [],
            anchor_input_mission_id=profile.get("anchorInputMissionID"),
            source="source_mission_plan_artifact_recovery",
            immutable=True,
        )
        persisted = _branch_ownership_store.get_locked_type2_branch_ownership(
            input_data,
            int(target_input_id),
        )
    except Exception:
        persisted = {}
    if persisted:
        emit(
            "[NEXTCOLLAB][TYPE2] restored immutable branch ownership from "
            f"source IMP {evidence_mode} evidence "
            f"(inputMissionID={evidence_mission_id}, ownership={persisted})."
        )
        return True, persisted
    return True, {}


def _coords_centroid_ll(coords: Any) -> Optional[tuple[float, float]]:
    rows = coords if isinstance(coords, list) else []
    lats = [float(c["latitude"]) for c in rows if isinstance(c, dict) and "latitude" in c]
    lons = [float(c["longitude"]) for c in rows if isinstance(c, dict) and "longitude" in c]
    if not lats or not lons:
        return None
    return (sum(lats) / len(lats), sum(lons) / len(lons))


def _branch_area_components_in_order(
    mission_detail: Dict[str, Any],
    ownership: Dict[int, List[int]],
) -> List[Dict[str, Any]]:
    """Build one component per areaList element in STRICT list order.

    각자도생 requires areaList[k] to be owned by the same UAV as lineList[k] of
    the neighbouring branch missions (the "line-area-line" set by index). The
    generic component builder runs the areas through unary_union, which reorders
    them; to guarantee the set never breaks we keep each areaList element as its
    own component with branchIndex = list index -> ownership[index]. Holes are
    skipped (branch monitoring areas are simple polygons).
    """
    area_list = mission_detail.get("areaList") if isinstance(mission_detail.get("areaList"), list) else []
    if not area_list:
        return []
    altitude_m = _representative_area_altitude_m(mission_detail)
    components: List[Dict[str, Any]] = []
    branch_idx = 0
    for row in area_list:
        if not isinstance(row, dict) or bool(row.get("isHole")):
            continue
        coords = _normalize_coord_list(row.get("coordinateList"))
        if len(coords) < 3:
            continue
        components.append(
            {
                "coordinateList": coords,
                "componentIndex": branch_idx + 1,
                "branchIndex": branch_idx,
                "ownerAircraftIDs": [int(a) for a in (ownership.get(branch_idx) or [])],
                "altitudeM": float(altitude_m),
                "componentSource": "branch_area_ordered",
            }
        )
        branch_idx += 1
    return components


def _branch_area_owner_for_component(
    component: Dict[str, Any],
    mission_detail: Dict[str, Any],
    ownership: Dict[int, List[int]],
) -> Optional[List[int]]:
    """Owner aircraft(s) for one area component of a branch mission.

    Priority: an explicit strict-index owner (``ownerAircraftIDs``, set by
    :func:`_branch_area_components_in_order`) → a monitoring remaining-area
    snapshot's ``sourceAircraftID`` → last-resort centroid match to the areaList
    branch. The first path is the guaranteed order-preserving one.
    """
    explicit = component.get("ownerAircraftIDs")
    if isinstance(explicit, list) and explicit:
        branch_index = _to_int(component.get("branchIndex"))
        authoritative = ownership.get(int(branch_index)) if branch_index is not None else None
        return [int(a) for a in (authoritative or [])] or None

    src_aid = _to_int(component.get("sourceAircraftID"))
    if src_aid is not None and src_aid > 0:
        # Remaining row segments stay with their exact original owner. An old
        # or malformed segment carrying a non-owner is rejected rather than
        # allowed to seed a cross-branch assignment.
        if any(int(src_aid) in {int(a) for a in owners} for owners in ownership.values()):
            return [int(src_aid)]
        return None

    area_list = mission_detail.get("areaList") if isinstance(mission_detail.get("areaList"), list) else []
    branch_centroids: List[tuple[int, tuple[float, float]]] = []
    branch_idx = 0
    for row in area_list:
        if not isinstance(row, dict) or bool(row.get("isHole")):
            continue
        coords = _normalize_coord_list(row.get("coordinateList"))
        if len(coords) < 3:
            continue
        centroid = _coords_centroid_ll(coords)
        if centroid is not None:
            branch_centroids.append((branch_idx, centroid))
        branch_idx += 1

    comp_centroid = _coords_centroid_ll(_normalize_coord_list(component.get("coordinateList")))
    if comp_centroid is None or not branch_centroids:
        return None
    best_idx = min(
        branch_centroids,
        key=lambda item: (item[1][0] - comp_centroid[0]) ** 2 + (item[1][1] - comp_centroid[1]) ** 2,
    )[0]
    owners = [int(a) for a in (ownership.get(int(best_idx)) or [])]
    return owners or None


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
                component["branchIndex"] = _to_int(row.get("branchIndex"))
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


def _area_ownership_limit_polygon(detail: Dict[str, Any]) -> Optional[Polygon]:
    """Return the original/stable single AREA outer boundary when available."""

    mission_detail = detail if isinstance(detail, dict) else {}
    assignment_detail = mission_detail.get("areaAssignmentDetail")
    candidates: List[Any] = []
    if isinstance(assignment_detail, dict):
        assignment_coordinate_polygon = _coord_list_to_polygon_xy(
            assignment_detail.get("coordinateList")
        )
        if assignment_coordinate_polygon is not None:
            candidates.append(assignment_coordinate_polygon)

        assignment_outer_polygons = [
            polygon
            for row in (assignment_detail.get("areaList") or [])
            if isinstance(row, dict) and not bool(row.get("isHole"))
            for polygon in [_coord_list_to_polygon_xy(row.get("coordinateList"))]
            if polygon is not None
        ]
        if assignment_outer_polygons:
            try:
                candidates.append(unary_union(assignment_outer_polygons))
            except Exception:
                pass

    # Legacy payloads often retain the original outer boundary in
    # coordinateList while areaList contains the fragmented remaining work.
    coordinate_polygon = _coord_list_to_polygon_xy(
        mission_detail.get("coordinateList")
    )
    if coordinate_polygon is not None:
        candidates.append(coordinate_polygon)

    for candidate in candidates:
        polygons = _iter_polygons_xy(candidate)
        if len(polygons) != 1:
            continue
        shell = Polygon(polygons[0].exterior)
        if not shell.is_empty and float(shell.area or 0.0) > 1.0:
            return shell
    return None


def _single_area_ownership_component(detail: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Build one connected, hole-free polygon for ownership division only.

    Exact 0/1-capture fragments can contain holes and disconnected islands.  A
    division planner must never interpret those fragments as separate UAV
    ownership missions.  Filming lines are clipped to the exact obligations
    later by ``next_collab_path_builder``; this envelope is used only to create
    one stable strip per aircraft.
    """

    mission_detail = detail if isinstance(detail, dict) else {}
    altitude_m = _representative_area_altitude_m(mission_detail)
    polygons: List[Polygon] = []

    area_list = mission_detail.get("areaList")
    if isinstance(area_list, list):
        for row in area_list:
            if not isinstance(row, dict) or bool(row.get("isHole")):
                continue
            poly = _coord_list_to_polygon_xy(row.get("coordinateList"))
            if poly is not None:
                polygons.append(poly)
    if not polygons:
        poly = _coord_list_to_polygon_xy(mission_detail.get("coordinateList"))
        if poly is not None:
            polygons.append(poly)
    if not polygons:
        for row in mission_detail.get("areaSegmentList") or []:
            if not isinstance(row, dict):
                continue
            poly = _coord_list_to_polygon_xy(row.get("coordinateList"))
            if poly is not None:
                polygons.append(poly)
    if not polygons:
        return None

    try:
        merged = unary_union(polygons)
    except Exception:
        merged = polygons[0]
    merged_polygons = _iter_polygons_xy(merged)
    if not merged_polygons:
        return None
    hull_summary: Optional[Dict[str, Any]] = None
    # 연결된 계단형/좁은-neck 잔여도 이미 Polygon 하나라는 이유로 외곽선을
    # 그대로 통과시키지 않는다. 비-branch AREA는 조각 수와 무관하게 항상
    # convex hull 소유영역을 만들고 안정 assignment 경계로 크기를 제한한다.
    limit_polygon = _area_ownership_limit_polygon(mission_detail)
    hull_polygon, hull_info = convex_hull_area_fragments_xy(
        merged_polygons,
        limit_geometry=limit_polygon,
    )
    if hull_polygon is not None:
        ownership_polygon = hull_polygon
        decomposition = "convex_hull_ownership_envelope"
        hull_summary = {
            "partsBefore": int(hull_info.get("partsBefore") or 0),
            "sourceAreaM2": round(float(hull_info.get("sourceAreaM2") or 0.0), 1),
            "rawHullAreaM2": round(float(hull_info.get("rawHullAreaM2") or 0.0), 1),
            "resultAreaM2": round(float(hull_info.get("resultAreaM2") or 0.0), 1),
            "limitAreaM2": round(float(hull_info.get("limitAreaM2") or 0.0), 1),
            "clippedToLimit": bool(hull_info.get("clippedToLimit")),
            "usedLimitFallback": bool(hull_info.get("usedLimitFallback")),
        }
    else:
        # 제한형 hull 생성이 실패한 경우에도 원래 경계가 있으면 그보다
        # 커질 수 없는 경계를 사용한다. 경계 자체가 없는 legacy payload만
        # 마지막으로 raw convex hull에 의존한다.
        if limit_polygon is not None:
            ownership_polygon = limit_polygon
            decomposition = "original_boundary_ownership_fallback"
        else:
            try:
                ownership_polygon = unary_union(merged_polygons).convex_hull
            except Exception:
                ownership_polygon = max(
                    merged_polygons,
                    key=lambda item: float(item.area or 0.0),
                )
            decomposition = "convex_hull_ownership_fallback"
    if not isinstance(ownership_polygon, Polygon) or ownership_polygon.is_empty:
        return None
    if not ownership_polygon.is_valid:
        try:
            ownership_polygon = ownership_polygon.buffer(0)
        except Exception:
            return None
    if not isinstance(ownership_polygon, Polygon) or ownership_polygon.is_empty:
        return None
    coords = _polygon_exterior_to_coords(ownership_polygon, altitude_m=float(altitude_m))
    if len(coords) < 3:
        return None
    component: Dict[str, Any] = {
        "componentIndex": 1,
        "componentSource": "stable_area_assignment",
        "componentDecomposition": str(decomposition),
        "areaM2": float(ownership_polygon.area or 0.0),
        "coordinateList": coords,
    }
    if hull_summary is not None:
        component["componentHull"] = hull_summary
    return component


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
    lah_route_start_by_aircraft: Optional[Dict[int, Dict[str, Any]]] = None,
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
    else:
        # Type 2/3 place the manned aircraft by the package ladder, one leg
        # behind the UAVs.  Without this the replan would fall back to the UAVs'
        # replacement geometry, whose area centroid parks the manned aircraft in
        # the middle of the region they are working.
        ladder_info = ground_maneuver_lah_info_for_input(
            input_plan, int(target_input_id)
        )
        if isinstance(ladder_info, dict):
            representative_info = deepcopy(ladder_info)
            representative_bearing = _mission_route_bearing(representative_info)
            emit(
                "[NEXTCOLLAB][LAH] ground maneuver hold applied "
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
    # Only the corridors up to the mission being flown to. Anything further on
    # is a region this leg has no business entering.
    destination_mission_index = _input_mission_index(input_plan, int(target_input_id))
    operation_zones = _lah_operation_zones_from_input_plan(
        input_plan,
        max_mission_index=destination_mission_index,
    )
    if destination_mission_index is None:
        emit(
            "[NEXTCOLLAB][LAH] target inputMissionID "
            f"{int(target_input_id)} not found in the input plan; routing over "
            "every mission zone."
        )
    lah_route_prefix_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    try:
        from modules.mission_planning.replanning.triggers.attack.pipeline import (
            _build_lah_mission_constrained_attack_route as _build_lah_mission_route,
        )
    except Exception:
        _build_lah_mission_route = None
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
            if int(aircraft_id) not in lah_route_prefix_by_aircraft:
                route_start = _normalize_coordinate(
                    (lah_route_start_by_aircraft or {}).get(
                        int(aircraft_id),
                        (lah_route_start_by_aircraft or {}).get(str(int(aircraft_id))),
                    )
                )
                target_route = _lah_route_coordinates_from_info(updated_info)
                if route_start is not None and target_route:
                    designated_target = _lah_formation_target_coordinate(
                        target_route[0],
                        int(aircraft_id),
                    )
                    route_meta: Dict[str, Any] = {
                        "constrained": False,
                        "reason": "mission_route_helper_unavailable",
                    }
                    route_prefix = [dict(route_start), dict(designated_target)]
                    if _build_lah_mission_route is not None:
                        route_prefix, route_meta = _build_lah_mission_route(
                            start_coord=route_start,
                            attack_coord=designated_target,
                            source_plan_id=None,
                            operation_zones=operation_zones,
                        )
                    lah_route_prefix_by_aircraft[int(aircraft_id)] = [
                        dict(coord)
                        for coord in route_prefix
                        if _normalize_coordinate(coord) is not None
                    ]
                    emit(
                        "[NEXTCOLLAB][LAH] designated return route "
                        f"aircraft={int(aircraft_id)} points={len(route_prefix)} "
                        f"constrained={bool(route_meta.get('constrained'))} "
                        f"startInside={route_meta.get('startInside')} "
                        f"reason={route_meta.get('reason')}"
                    )
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
            route_start_by_aircraft=(lah_route_prefix_by_aircraft or lah_route_start_by_aircraft),
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
                lah_missions=list(manned_missions),
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
        path = (
            source_cache.flight_path_json_path(int(path_id))
            if source_cache is not None
            else db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        )
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


def _line_deployment_coords_from_flight_path(
    flight_path: Dict[str, Any] | None,
) -> List[Dict[str, Any]]:
    """Read the longitudinal order actually flown by a generated LINE path."""

    payload = flight_path if isinstance(flight_path, dict) else {}
    waypoints = payload.get("waypointList") if isinstance(payload.get("waypointList"), list) else []
    centers: List[Dict[str, Any]] = []
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        sweep_coords = _normalize_coord_list(line_search.get("coordinateList"))
        if len(sweep_coords) < 2:
            continue
        center = _midpoint_of_sweep_coords(sweep_coords)
        if center is not None:
            centers.append(center)
    distinct_centers: List[Dict[str, Any]] = []
    for center in centers:
        if not distinct_centers:
            distinct_centers.append(center)
            continue
        prev = distinct_centers[-1]
        prev_lat = _to_float(prev.get("latitude"))
        prev_lon = _to_float(prev.get("longitude"))
        center_lat = _to_float(center.get("latitude"))
        center_lon = _to_float(center.get("longitude"))
        if None in (prev_lat, prev_lon, center_lat, center_lon):
            continue
        if abs(float(center_lat) - float(prev_lat)) > 1e-10 or abs(float(center_lon) - float(prev_lon)) > 1e-10:
            distinct_centers.append(center)
    return deepcopy(distinct_centers) if len(distinct_centers) >= 2 else []


def _resolve_line_deployment_coordinate_list_from_templates(
    template_record_map: Dict[int, List[Dict[str, Any]]],
    target_aircraft_ids: List[int] | None = None,
) -> List[Dict[str, Any]]:
    """Resolve the first actual LINE deployment direction across replans.

    New plans carry an explicit immutable reference.  Older artifacts fall
    back to their generated FlightPath order, then to their oriented lineList.
    """

    available_ids = sorted(
        {
            int(parsed_id)
            for raw_id in template_record_map.keys()
            for parsed_id in [_to_int(raw_id)]
            if parsed_id is not None and int(parsed_id) > 0
        }
    )
    requested_ids = [
        int(parsed_id)
        for raw_id in (target_aircraft_ids or [])
        for parsed_id in [_to_int(raw_id)]
        if parsed_id is not None and int(parsed_id) in available_ids
    ]
    aircraft_ids = list(dict.fromkeys(requested_ids + available_ids))
    records = [
        record
        for aircraft_id in aircraft_ids
        for record in (template_record_map.get(int(aircraft_id)) or [])
        if isinstance(record, dict)
    ]

    for record in records:
        mission = record.get("mission") if isinstance(record.get("mission"), dict) else {}
        info = _template_mission_info(mission)
        explicit = _normalize_coord_list(info.get("lineDeploymentCoordinateList"))
        if len(explicit) >= 2:
            return deepcopy(explicit)

    for record in records:
        actual = _line_deployment_coords_from_flight_path(
            record.get("flightPath") if isinstance(record.get("flightPath"), dict) else None
        )
        if len(actual) >= 2:
            return actual

    for record in records:
        mission = record.get("mission") if isinstance(record.get("mission"), dict) else {}
        info = _template_mission_info(mission)
        for line in info.get("lineList") or []:
            if not isinstance(line, dict):
                continue
            coords = _normalize_coord_list(line.get("coordinateList"))
            if len(coords) >= 2:
                return deepcopy(coords)
    return []


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


def _retire_unavailable_uav_missions_from_target(
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    *,
    target_aircraft_ids: List[int],
    target_input_id: int,
) -> Dict[int, Dict[str, Any]]:
    """Remove stale target/downstream work from UAVs outside the live pool.

    ``entryAircraftList`` is the authoritative available-UAV set for a
    next-collaborative replan.  The source MissionPlan still contains cloned
    IMPs for a lost or otherwise unavailable UAV.  Leaving its original target
    Area mission in that clone makes monitoring recreate the retired UAV as an
    OUT/RETURN owner even though the replacement geometry was divided only
    among the live aircraft.  Drop the target mission and its downstream
    suffix for unavailable UAVs; a later recovery replan can recreate those
    templates from an available peer via the existing template fallback.
    """

    active_ids = {
        int(aid)
        for aid in target_aircraft_ids
        if _to_int(aid) is not None and int(aid) > 3
    }
    summary: Dict[int, Dict[str, Any]] = {}
    for raw_aircraft_id, package in packages_by_aircraft.items():
        aircraft_id = _to_int(raw_aircraft_id)
        if (
            aircraft_id is None
            or int(aircraft_id) <= 3
            or int(aircraft_id) in active_ids
            or not isinstance(package, dict)
        ):
            continue
        missions = package.get("individualMissionList")
        if not isinstance(missions, list):
            continue
        first_target_index: Optional[int] = None
        for index, mission in enumerate(missions):
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) == int(target_input_id):
                first_target_index = int(index)
                break
        if first_target_index is None:
            continue

        retired_rows = [
            mission
            for mission in missions[int(first_target_index) :]
            if isinstance(mission, dict)
        ]
        package["individualMissionList"] = list(missions[: int(first_target_index)])
        summary[int(aircraft_id)] = {
            "droppedMissionCount": len(retired_rows),
            "droppedInputMissionIDs": sorted(
                {
                    int(input_id)
                    for input_id in (
                        _mission_input_id(mission) for mission in retired_rows
                    )
                    if input_id is not None and int(input_id) > 0
                }
            ),
            "droppedIndividualMissionIDs": sorted(
                {
                    int(mission_id)
                    for mission_id in (
                        _to_int(mission.get("individualMissionID"))
                        for mission in retired_rows
                    )
                    if mission_id is not None and int(mission_id) > 0
                }
            ),
        }
    return summary


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
    if altitude is not None and int(altitude) <= 0:
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
                "width": max(0, min(50000, int(round(float(width_m))))),
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
    planning_mode: Dict[str, Any] | None = None,
) -> Optional[_PreparedReplacements]:
    planning_mode_ctx = mission_mode_context(mode=planning_mode)
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
    waypoint_id_provider = _ReplacementWaypointIdProvider(
        scope="FORMATION",
    )
    bearing_deg = _to_float(reference_row.get("bearingDeg"))

    for idx, aircraft_id in enumerate(aircraft_ids):
        template_record = _template_record_for_aircraft(template_record_map, int(aircraft_id))
        template_mission = template_record.get("mission") if isinstance(template_record.get("mission"), dict) else {}
        template_path = template_record.get("flightPath") if isinstance(template_record.get("flightPath"), dict) else {}
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
        "planningMode": dict(planning_mode_ctx),
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
        planning_mode=dict(planning_mode_ctx),
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
    planning_mode: Dict[str, Any] | None = None,
    handover_coord_map: Dict[int, Dict[str, Any]] | None = None,
) -> Optional[_PreparedReplacements]:
    planning_mode_ctx = mission_mode_context(mode=planning_mode)
    locked_type2_ownership = (
        _branch_area_ownership_for_target(planning_mode_ctx, int(target_input_id))
        if _to_int(planning_mode_ctx.get("package_type")) == 2
        else None
    )
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

    planner_target_mission = target_input_mission
    # 각자도생 branches are independent lines with their own headings, and the
    # resolver returns a single direction for the whole mission.  Imposing it
    # would reverse any branch sitting more than 90 degrees off it, which is how
    # one UAV ended up sweeping its corridor backwards while the rest were fine.
    # Each branch already carries its own orientation in the input lineList.
    deployment_coordinate_list = (
        []
        if locked_type2_ownership
        else _resolve_line_deployment_coordinate_list_from_templates(
            template_record_map,
            target_aircraft_ids,
        )
    )
    if locked_type2_ownership:
        emit(
            "[NEXTCOLLAB][TYPE2] per-branch deployment direction kept; "
            "no mission-wide LINE direction imposed."
        )
    if len(deployment_coordinate_list) >= 2:
        planner_target_mission = deepcopy(target_input_mission)
        mission_detail = planner_target_mission.get("missionDetail")
        if isinstance(mission_detail, dict):
            mission_detail = deepcopy(mission_detail)
            mission_detail["lineDeploymentCoordinateList"] = deepcopy(deployment_coordinate_list)
            mission_detail["lineDeploymentDirectionLocked"] = True
            planner_target_mission["missionDetail"] = mission_detail
        else:
            planner_target_mission["lineDeploymentCoordinateList"] = deepcopy(deployment_coordinate_list)
            planner_target_mission["lineDeploymentDirectionLocked"] = True
        emit("[NEXTCOLLAB][LINE] first execution deployment direction restored from prior plan.")

    try:
        planner_result = run_next_collab_line_plan(
            target_mission=planner_target_mission,
            aircraft_entries=aircraft_entries,
            turn_radius_scale=float(turn_radius_scale),
            planning_mode=planning_mode_ctx,
            log=emit,
        )
    except Exception as exc:
        emit(f"[NEXTCOLLAB][LINE] planner failed: {exc}")
        return None
    if locked_type2_ownership:
        split_result = getattr(planner_result, "split_result", None)
        split_ownership = getattr(split_result, "branch_ownership", None)
        normalized_split_ownership = {
            int(branch_index): [int(aircraft_id) for aircraft_id in owners]
            for branch_index, owners in (
                split_ownership if isinstance(split_ownership, dict) else {}
            ).items()
        }
        if normalized_split_ownership != locked_type2_ownership:
            emit(
                "[NEXTCOLLAB][TYPE2] LINE planner ownership mismatch; "
                "existing branch assignments preserved."
            )
            return None
        for piece in (getattr(split_result, "pieces", None) or []):
            data = piece.data if isinstance(getattr(piece, "data", None), dict) else {}
            branch_index = _to_int(data.get("branchIndex"))
            aircraft_id = _to_int(getattr(piece, "assigned_uav", None))
            if (
                branch_index is None
                or aircraft_id is None
                or int(aircraft_id)
                not in {int(aid) for aid in locked_type2_ownership.get(int(branch_index), [])}
            ):
                emit(
                    "[NEXTCOLLAB][TYPE2] LINE piece escaped its immutable branch; "
                    "existing assignments preserved."
                )
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
    required_piece_ids = {
        int(piece.piece_index or 0)
        for piece in (getattr(planner_result.split_result, "pieces", None) or [])
        if int(piece.piece_index or 0) > 0
    }
    planned_piece_ids = {
        int(_to_int(row.get("pieceIndex")) or 0)
        for row in ordered_path_rows
        if int(_to_int(row.get("pieceIndex")) or 0) > 0
    }
    if planned_piece_ids != required_piece_ids:
        emit(
            "[NEXTCOLLAB][LINE] partial piece output rejected; "
            f"required={sorted(required_piece_ids)}, planned={sorted(planned_piece_ids)}. "
            "Existing non-overlapping LINE assignments will be preserved."
        )
        return None
    planned_aircraft_ids = {
        int(_to_int(row.get("aircraftID")) or 0)
        for row in ordered_path_rows
        if int(_to_int(row.get("aircraftID")) or 0) > 0
    }
    required_aircraft_ids = {
        int(aircraft_id)
        for aircraft_id in target_aircraft_ids
        if int(aircraft_id) > 0
    }
    if planned_aircraft_ids != required_aircraft_ids:
        emit(
            "[NEXTCOLLAB][LINE] partial aircraft output rejected; "
            f"required={sorted(required_aircraft_ids)}, planned={sorted(planned_aircraft_ids)}. "
            "Existing non-overlapping LINE assignments will be preserved."
        )
        return None
    if locked_type2_ownership:
        required_owner_ids = {
            int(aircraft_id)
            for owners in locked_type2_ownership.values()
            for aircraft_id in owners
        }
        planned_owner_ids = {
            int(_to_int(row.get("aircraftID")) or 0) for row in ordered_path_rows
        }
        if planned_owner_ids != required_owner_ids:
            emit(
                "[NEXTCOLLAB][TYPE2] LINE output owner set mismatch; "
                f"expected={sorted(required_owner_ids)}, planned={sorted(planned_owner_ids)}."
            )
            return None
    _apply_search_speed_scale_multiplier_to_rows(
        ordered_path_rows,
        search_speed_scale_multiplier=search_speed_scale_multiplier,
        emit=emit,
        scope="LINE",
    )

    path_rows_by_aircraft = _group_next_collab_path_rows_by_aircraft(ordered_path_rows)
    dem_prewarm_summary = _prewarm_dem_altitudes_for_path_rows_if_enabled(ordered_path_rows)
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
    waypoint_id_provider = _ReplacementWaypointIdProvider(
        scope="LINE",
    )

    def _record_line_flight_path_metrics(metrics: Dict[str, Any]) -> None:
        with flight_path_metrics_lock:
            flight_path_metrics_rows.append(dict(metrics))

    for aircraft_id, aircraft_path_rows in path_rows_by_aircraft.items():
        for path_index, path_row in enumerate(aircraft_path_rows):
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
            template_mission = template_record.get("mission") if isinstance(template_record.get("mission"), dict) else {}
            template_path = template_record.get("flightPath") if isinstance(template_record.get("flightPath"), dict) else {}
            template_info = _template_mission_info(template_mission)
            mission_info = build_mission_info_from_planned_row(
                path_row,
                template_info=template_info,
            )
            apply_control_transfer_direct_metadata(mission_info, target_input_mission)
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
            handover_coord = (
                deepcopy((handover_coord_map or {}).get(int(row_aircraft_id)))
                if path_index == len(aircraft_path_rows) - 1
                else None
            )
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
                    handover_coord=handover_coord,
                    timestamp_ms=int(now_ms): build_flight_path_from_planned_row(
                        path_row,
                        template_path=template_path,
                        mission_info=mission_info,
                        individual_mission_id=int(individual_id),
                        path_id=int(path_id),
                        aircraft_id=int(aircraft_id),
                        entry_coord=entry_coord,
                        handover_coord=handover_coord,
                        timestamp_ms=int(timestamp_ms),
                        source="MMR",
                        assign_waypoint_ids=False,
                        metrics_callback=_record_line_flight_path_metrics,
                        # Type-2/3 independent branch LINE sweeps can be much
                        # longer than 16x one route leg.  Capping their virtual
                        # camera at 40*16 m/s makes the aircraft leave the WP
                        # before filming completes.  Sticky branch ownership is
                        # the narrow contract that identifies this case.
                        line_search_multiplier_cap_enabled=not bool(
                            locked_type2_ownership
                        ),
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
        "planningMode": dict(planning_mode_ctx),
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
        planning_mode=dict(planning_mode_ctx),
    )


def _is_donut_replan_mission(
    target_input_mission: Dict[str, Any],
    planning_mode_ctx: Dict[str, Any],
) -> bool:
    """True for a Type-4 donut boundary mission being replanned via next-collab."""
    if _is_donut_boundary_mission is None or _donut_band_pieces is None:
        return False
    if _to_int(planning_mode_ctx.get("package_type")) != 4:
        return False
    try:
        return bool(_is_donut_boundary_mission(target_input_mission))
    except Exception:
        return False


def _donut_template_info_for_owner(
    template_record_map: Dict[int, List[Dict[str, Any]]],
    owner: int,
) -> Dict[str, Any]:
    """Prior-mission info for the owner (prefer the donut band record)."""
    fallback: Dict[str, Any] = {}
    for record in template_record_map.get(int(owner)) or []:
        if not isinstance(record, dict):
            continue
        mission = record.get("mission") if isinstance(record.get("mission"), dict) else {}
        info = mission.get("individualMissionInfo")
        if not isinstance(info, dict) or not info:
            continue
        if isinstance(info.get("_donutPatrol"), dict):
            return info
        if not fallback:
            fallback = info
    return fallback


def _apply_donut_capture_fields(
    mission_info: Dict[str, Any],
    template_info: Dict[str, Any],
    wpl: List[Dict[str, Any]],
) -> None:
    """Fill FOV/SEP/SPEED(+bearings) on a rebuilt donut band mission info.

    The 공간해상도/촬영품질 monitor drops missions without a positive SEP, so a
    replacement info missing capture metadata never opens the sweep gate
    (footprint/GSD stay blank while the band is filmed). Sticky bands keep
    their geometry across replans, so carry the prior mission's values; if the
    prior info also lacks them (replans stacked on a pre-fix replacement),
    derive them from the rebuilt wplist - FOV/speed from the sweep rows, SEP
    from the mean ring-leg length (stations sit one radial sweep apart, so the
    leg length is the sweep separation).
    """
    for key in ("BEARING", "MOVE_BEARING"):
        if key not in mission_info and template_info.get(key) is not None:
            mission_info[key] = template_info[key]
    for key in ("FOV", "SEP", "SPEED"):
        value = _to_float(template_info.get(key))
        if value is not None and value > 0.0:
            mission_info[key] = template_info[key]
    if _to_float(mission_info.get("FOV")) is None:
        for row in wpl:
            fp = row.get("filmingProperty") if isinstance(row, dict) else None
            if isinstance(fp, dict) and _to_int(fp.get("operationMode")) == 2:
                fov = _to_float(fp.get("fieldOfView"))
                if fov is not None and fov > 0.0:
                    mission_info["FOV"] = round(float(fov), 3)
                    break
    if _to_float(mission_info.get("SPEED")) is None:
        speeds = [
            speed
            for speed in (
                _to_float(row.get("speed")) for row in wpl if isinstance(row, dict)
            )
            if speed is not None and speed > 0.0
        ]
        if speeds:
            # wplist speed is m/s; the 0302 SPEED field is km/h by contract.
            mission_info["SPEED"] = round(sum(speeds) / len(speeds) * 3.6, 3)
    if _to_float(mission_info.get("SEP")) is None:
        legs: List[float] = []
        for prev_row, row in zip(wpl, wpl[1:]):
            if not isinstance(prev_row, dict) or not isinstance(row, dict):
                continue
            coord0 = prev_row.get("coordinate")
            coord1 = row.get("coordinate")
            if not isinstance(coord0, dict) or not isinstance(coord1, dict):
                continue
            try:
                dist = float(_coord_distance_m(coord0, coord1))
            except Exception:
                continue
            if dist > 1.0:
                legs.append(dist)
        if legs:
            mission_info["SEP"] = round(sum(legs) / len(legs), 1)


def _prepare_donut_replacements(
    *,
    target_input_mission: Dict[str, Any],
    target_input_id: int,
    target_aircraft_ids: List[int],
    entry_coord_map: Dict[int, Dict[str, Any]],
    template_record_map: Dict[int, List[Dict[str, Any]]],
    now_ms: int,
    turn_radius_scale: float,
    emit: Callable[[str], None],
    prepare_timer: Optional[_NextCollabPrepareTimer] = None,
    id_reservation_summaries: Optional[List[Dict[str, Any]]] = None,
    planning_mode: Dict[str, Any] | None = None,
) -> Optional[_PreparedReplacements]:
    """Replan a Type-4 donut boundary mission as per-owner concentric bands.

    Bypasses the generic area division planner (which would re-slice the annulus
    geometrically and break the onion-peel band ownership). Each UAV re-plans its
    OWN sticky band with the donut planner; ownership comes from the branch store
    (re-anchored by current position only when there is none yet).
    """
    planning_mode_ctx = mission_mode_context(mode=planning_mode)
    pkg_id = _to_int(planning_mode_ctx.get("inputMissionPackageID"))
    stored: Dict[int, List[int]] = {}
    if _branch_ownership_store is not None and pkg_id:
        try:
            stored = _branch_ownership_store.get_branch_ownership(pkg_id)
        except Exception:
            stored = {}

    takeover_map: Dict[int, Dict[str, Any]] = {}
    for aid in target_aircraft_ids:
        coord = entry_coord_map.get(int(aid))
        if isinstance(coord, dict):
            takeover_map[int(aid)] = coord
    uav_ids = [int(a) for a in target_aircraft_ids]
    try:
        pieces, band_ownership = _donut_band_pieces(
            target_input_mission, uav_ids, takeover_map, ownership_override=stored or None
        )
    except Exception as exc:
        emit(f"[NEXTCOLLAB][DONUT] band split failed: {exc}")
        return None
    if not pieces:
        return None
    if _branch_ownership_store is not None and pkg_id and band_ownership:
        try:
            _branch_ownership_store.register_branch_ownership(
                package_id=pkg_id,
                branch_count=len(band_ownership),
                ownership=band_ownership,
                branch_mission_ids=[int(target_input_id)],
                anchor_input_mission_id=int(target_input_id),
                source="next_collab_donut",
            )
        except Exception:
            pass

    per_owner: List[tuple[int, Dict[str, Any], List[Dict[str, Any]]]] = []
    for pc in pieces:
        band_idx = int(pc.get("branchIndex") or 0)
        owners = band_ownership.get(band_idx) or []
        if not owners:
            continue
        owner = int(owners[0])
        marker = pc.get("_donutPatrol") if isinstance(pc.get("_donutPatrol"), dict) else None
        if not marker:
            continue
        try:
            wpl = _donut_wplist(marker, owner)
        except Exception:
            wpl = []
        if not wpl:
            continue
        outer = _normalize_coord_list(pc.get("coordinateList"))
        inner = _normalize_coord_list(pc.get("_donutBandInner"))
        area_list = [{"isHole": False, "coordinateList": outer}]
        if len(inner) >= 3:
            area_list.append({"isHole": True, "coordinateList": inner})
        mission_info = {
            "individualMissionType": 4,
            "patternType": 6,
            "autoZoomIn": True,
            "areaList": area_list,
            "targetID": None,
            "_donutPatrol": deepcopy(marker),
        }
        # Carry FOV/SEP/SPEED forward (monitor sweep gate needs SEP; band
        # geometry is sticky so the prior values stay valid).
        _apply_donut_capture_fields(
            mission_info,
            _donut_template_info_for_owner(template_record_map, owner),
            wpl,
        )
        per_owner.append((owner, mission_info, wpl))

    if not per_owner:
        emit("[NEXTCOLLAB][DONUT] no owner bands produced.")
        return None

    path_rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    for owner, _mi, _wpl in per_owner:
        path_rows_by_aircraft.setdefault(int(owner), []).append({"aircraftID": int(owner)})
    reservation = _reserve_next_collab_replacement_ids(
        path_rows_by_aircraft=path_rows_by_aircraft,
        emit=emit,
        id_reservation_summaries=id_reservation_summaries,
        scope="donut_replacements",
    )
    if reservation is None:
        emit("[NEXTCOLLAB][DONUT] failed to reserve individualMissionIDs.")
        return None

    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    generated_fp_by_path: Dict[int, Dict[str, Any]] = {}
    generated_path_ids: Set[int] = set()
    ordered_path_ids: List[int] = []
    waypoint_id_provider = _ReplacementWaypointIdProvider(scope="AREA")
    for owner, mission_info, wpl in per_owner:
        individual_id = reservation.next_individual()
        path_id = reservation.next_path(int(owner))
        template_records = template_record_map.get(int(owner)) or []
        template_mission = (
            template_records[0].get("mission") if template_records and isinstance(template_records[0], dict) else {}
        ) or {}
        related = deepcopy(template_mission.get("relatedMission") or {})
        related["relatedMissionType"] = _to_int(related.get("relatedMissionType")) or 1
        related["inputMissionID"] = int(target_input_id)
        related["priorMissionID"] = _to_int(related.get("priorMissionID")) or 0
        replacement_by_aircraft.setdefault(int(owner), []).append(
            {
                "individualMissionID": int(individual_id),
                "isDone": False,
                "relatedMission": related,
                "individualMissionInfo": mission_info,
                "pathID": int(path_id),
            }
        )
        wps = [dict(w) for w in wpl]
        for w in wps:
            w["waypointID"] = 0
        generated_fp_by_path[int(path_id)] = {
            "timestamp": int(now_ms),
            "Source": "MMR",
            "pathID": int(path_id),
            "aircraftID": int(owner),
            "individualMissionID": int(individual_id),
            "isFormationFlight": False,
            "waypointList": wps,
        }
        generated_path_ids.add(int(path_id))
        ordered_path_ids.append(int(path_id))

    _assign_replacement_waypoint_ids_in_order(
        generated_fp_by_path=generated_fp_by_path,
        ordered_path_ids=ordered_path_ids,
        waypoint_id_provider=waypoint_id_provider,
        emit=emit,
        scope="AREA",
    )
    reservation_summary = _refresh_next_collab_id_reservation_summary(
        reservation=reservation,
        scope="donut_replacements",
        id_reservation_summaries=id_reservation_summaries,
    )
    review_report = {
        "enabled": True,
        "mode": REPLAN_FLOW_MODE,
        "plannerWorkflow": "donut_band",
        "changed": True,
        "operatorDecision": {
            "category": "donut_band_reassign",
            "reason": "donut_boundary_band_replan",
            "detail": "Donut patrol bands re-planned per sticky owner (no annulus re-division).",
        },
        "targets": len(target_aircraft_ids),
        "generatedMissionCount": len(ordered_path_ids),
        "idReservation": reservation_summary,
        "planningMode": dict(planning_mode_ctx),
        "uavIndependentWork": {int(a): len(r) for a, r in replacement_by_aircraft.items()},
    }
    emit(
        "[NEXTCOLLAB][DONUT] band replan "
        f"bands={len(per_owner)} owners={sorted(int(a) for a in replacement_by_aircraft)}"
    )
    return _PreparedReplacements(
        replacement_by_aircraft=replacement_by_aircraft,
        generated_fp_by_path=generated_fp_by_path,
        generated_path_ids=generated_path_ids,
        planner_workflow="donut_band",
        planner_result_text="",
        planned_result_count=len(ordered_path_ids),
        review_report=review_report,
        mission_mode="area",
        timing_ms=prepare_timer.snapshot() if prepare_timer is not None else {},
        id_reservation=dict(reservation_summary),
        uav_work_summary={int(a): len(r) for a, r in replacement_by_aircraft.items()},
        runtime_preservation={"turnRadiusScale": float(turn_radius_scale or 1.0)},
        planning_mode=dict(planning_mode_ctx),
    )


def _prepare_area_replacements(
    *,
    target_input_mission: Dict[str, Any],
    target_input_id: int,
    target_aircraft_ids: List[int],
    entry_coord_map: Dict[int, Dict[str, Any]],
    heading_map: Dict[int, float],
    entry_aircraft_context_map: Dict[int, Dict[str, Any]] | None,
    representative_entry: Dict[str, Any] | None,
    template_record_map: Dict[int, List[Dict[str, Any]]],
    now_ms: int,
    turn_radius_scale: float,
    search_speed_scale_multiplier: float = 1.0,
    emit: Callable[[str], None],
    prepare_timer: Optional[_NextCollabPrepareTimer] = None,
    id_reservation_summaries: Optional[List[Dict[str, Any]]] = None,
    planning_mode: Dict[str, Any] | None = None,
    split_single_aircraft_into_two: bool = False,
    handover_coord_map: Dict[int, Dict[str, Any]] | None = None,
) -> Optional[_PreparedReplacements]:
    planning_mode_ctx = mission_mode_context(mode=planning_mode)
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
        context_row = (
            entry_aircraft_context_map.get(int(aircraft_id))
            if isinstance(entry_aircraft_context_map, dict)
            else None
        )
        row: Dict[str, Any] = (
            deepcopy(context_row)
            if isinstance(context_row, dict)
            else {}
        )
        # Area used to discard the live speed/turn/prediction fields here and
        # plan only from the request-time coordinate.  Keep the authoritative
        # coordinate/heading while forwarding the rest of the 0401 context so
        # the headless Area planner can use the expected activation position.
        row["aircraftID"] = int(aircraft_id)
        row["coordinate"] = dict(entry_coord)
        if int(aircraft_id) in heading_map:
            row["headingDeg"] = float(heading_map[int(aircraft_id)])
        aircraft_entries.append(row)
    if not aircraft_entries:
        emit("[NEXTCOLLAB] planner aircraft entries unresolved.")
        return None

    raw_mission_detail = (
        target_input_mission.get("missionDetail")
        if isinstance(target_input_mission.get("missionDetail"), dict)
        else {}
    )
    mission_detail = dict(raw_mission_detail)
    area_pass_contract = _area_coverage_pass_contract_from_input_mission(
        target_input_mission
    )
    depth_assignment_mode = bool(
        area_pass_contract.get("areaCoverageDepthContractVersion") is not None
    )
    if (
        area_pass_contract.get("areaCoverageDepthContractVersion") is not None
        and not bool(area_pass_contract.get("coverageDepthSatisfied"))
        and int(area_pass_contract.get("coverageDepthUnresolvedGeometryCount") or 0) > 0
    ):
        emit(
            "[NEXTCOLLAB][AREA] coverage-depth geometry unresolved; "
            "replacement planning blocked without marking the mission complete."
        )
        return None
    if area_pass_contract and not list(
        area_pass_contract.get("remainingCoveragePasses") or []
    ):
        emit(
            "[NEXTCOLLAB][AREA] reciprocal coverage contract has no pending "
            "passes; replacement planning skipped."
        )
        return None
    branch_ownership_map = _branch_area_ownership_for_target(
        planning_mode_ctx,
        int(target_input_id),
    )
    if depth_assignment_mode:
        depth_source = dict(target_input_mission)
        depth_source.update(raw_mission_detail)
        pending_depth_detail = mission_area_replan_store.coverage_depth_pending_remaining_detail(
            depth_source
        )
        assignment_detail = mission_area_replan_store.area_assignment_detail(
            target_input_mission,
            fallback=raw_mission_detail,
        )
        if assignment_detail is not None and branch_ownership_map is None:
            mission_area_replan_store.apply_area_assignment_geometry(
                mission_detail,
                assignment_detail,
            )
        if isinstance(pending_depth_detail, dict):
            mission_detail["areaCoverageWorkloadDetail"] = deepcopy(
                pending_depth_detail
            )
        emit(
            "[NEXTCOLLAB][AREA] stable ownership geometry separated from "
            "exact spatial-depth workload."
        )
    requested_assignment_passes = _ordered_area_assignment_passes(area_pass_contract)
    primary_assignment_pass = (
        requested_assignment_passes[0] if requested_assignment_passes else None
    )
    primary_planning_detail = (
        _area_pass_planning_detail(
            mission_detail,
            area_pass_contract,
            primary_assignment_pass,
        )
        if primary_assignment_pass is not None
        else mission_detail
    )
    if branch_ownership_map is None:
        # One AREA input is one logical single-capture region.  Footprint
        # subtraction may leave several exact polygon fragments, but those are
        # not independent planning areas; reconnect them once, then divide that
        # region across the available UAVs exactly once.
        ownership_component = _single_area_ownership_component(primary_planning_detail)
        area_components = [ownership_component] if ownership_component is not None else []
    else:
        area_components = _area_planner_components_from_detail(primary_planning_detail)
    if not area_components:
        component_summary = _area_planner_component_input_summary(primary_planning_detail)
        emit(
            "[NEXTCOLLAB] area replacement requires valid planner components "
            f"(reason={component_summary.get('reason')}, "
            f"segments={component_summary.get('areaSegmentCount')}, "
            f"outer={component_summary.get('areaOuterCount')}, "
            f"holes={component_summary.get('areaHoleCount')}, "
            f"maxComponents={component_summary.get('maxComponents')})."
        )
        return None

    # Type 2/3 각자도생: each 경계 area is one UAV's own monitoring area, so a
    # replan must plan each component only for its sticky owner(s) instead of
    # re-dividing across the whole pool. Order is critical - areaList[k] must go
    # to the same UAV as lineList[k] of the neighbouring branch missions - so for
    # a fresh areaList we rebuild the components in strict list order (the generic
    # unary_union builder reorders them). A monitoring remaining-area snapshot
    # already carries the owner per segment, so keep those components as-is.
    if branch_ownership_map is not None:
        has_area_segments = bool(
            isinstance(primary_planning_detail.get("areaSegmentList"), list)
            and primary_planning_detail.get("areaSegmentList")
        )
        # 도넛(isHole 보유) 임무의 areaList는 [외곽, 구멍]이라 원소=브랜치 규칙이
        # 성립하지 않음 - 잔여 스냅샷(areaSegmentList)의 sourceAircraftID로만 소유를
        # 유지하고, 순서 재구성은 hole-free 브랜치 임무에만 적용한다.
        has_hole_areas = any(
            isinstance(row, dict) and bool(row.get("isHole"))
            for row in (primary_planning_detail.get("areaList") or [])
            if isinstance(row, dict)
        )
        if not has_area_segments and not has_hole_areas:
            ordered_components = _branch_area_components_in_order(
                primary_planning_detail,
                branch_ownership_map,
            )
            if ordered_components:
                area_components = ordered_components
        emit(
            "[NEXTCOLLAB][AREA] 각자도생 branch area replan; components assigned by "
            f"sticky ownership in list order (branches={len(branch_ownership_map)}, "
            f"segments={has_area_segments})."
        )
    elif depth_assignment_mode:
        emit(
            "[NEXTCOLLAB][AREA] one-shot ownership division enabled "
            f"(targets={len(aircraft_entries)}, plannerComponents=1)."
        )

    planner_results: List[tuple[Dict[str, Any], Any]] = []
    component_sequence = 0

    def _run_area_pass_components(
        components: List[Dict[str, Any]],
        entries: List[Dict[str, Any]],
        *,
        planning_detail: Dict[str, Any],
        pass_name: Optional[str],
        pass_contract: Dict[str, Any] | None,
    ) -> bool:
        nonlocal component_sequence
        for raw_component in components:
            component = dict(raw_component)
            component_sequence += 1
            component_index = int(component_sequence)
            component["componentIndex"] = int(component_index)
            component["areaPassEntryCoordinateByAircraft"] = {
                int(_to_int(entry.get("aircraftID")) or 0): deepcopy(entry.get("coordinate"))
                for entry in entries
                if isinstance(entry, dict)
                and int(_to_int(entry.get("aircraftID")) or 0) > 0
                and isinstance(entry.get("coordinate"), dict)
            }
            if pass_name is not None:
                component["areaAssignedCoveragePass"] = str(pass_name)
                component["areaPassContract"] = deepcopy(pass_contract or {})
            mission_polygon = _normalize_coord_list(component.get("coordinateList"))
            if len(mission_polygon) < 3:
                emit(f"[NEXTCOLLAB] area component {component_index} has invalid polygon.")
                return False
            component_aircraft_entries = entries
            if branch_ownership_map is not None:
                owner_ids = _branch_area_owner_for_component(
                    component,
                    planning_detail,
                    branch_ownership_map,
                )
                if not owner_ids:
                    emit(
                        f"[NEXTCOLLAB][TYPE2] branch component {component_index} has no "
                        "authoritative owner; preserving the existing assignment."
                    )
                    return False
                required_owner_ids = {int(aircraft_id) for aircraft_id in owner_ids}
                owned = [
                    entry
                    for entry in entries
                    if int(entry.get("aircraftID") or 0) in required_owner_ids
                ]
                present_owner_ids = {
                    int(entry.get("aircraftID") or 0) for entry in owned
                }
                if present_owner_ids != required_owner_ids:
                    emit(
                        f"[NEXTCOLLAB][TYPE2] branch component {component_index} deferred; "
                        f"missing owner UAVs={sorted(required_owner_ids - present_owner_ids)}. "
                        "No co-owner or other branch UAV will take its unfinished work."
                    )
                    return False
                component_aircraft_entries = owned
                component["areaPassEntryCoordinateByAircraft"] = {
                    int(entry["aircraftID"]): deepcopy(entry.get("coordinate"))
                    for entry in owned
                    if isinstance(entry.get("coordinate"), dict)
                }
                emit(
                    f"[NEXTCOLLAB][AREA] branch component {component_index} -> "
                    f"owner UAV {sorted(present_owner_ids)}"
                )
            # Plan first with only the real surviving aircraft. The shared AREA
            # splitter now emits stage 1..N from the 700 m target / 900 m cap.
            # with a synthetic partner here used to force attack replans to two
            # pieces before the actual width could be considered.
            split_branch_owner_group = bool(
                branch_ownership_map is not None
                and int(_to_int(planning_mode_ctx.get("package_type")) or 0) in (2, 3)
                and component_aircraft_entries
            )
            split_normal_owner_group = bool(
                branch_ownership_map is None
                and split_single_aircraft_into_two
                and component_aircraft_entries
            )
            minimum_stages_per_owner = (
                2
                if split_branch_owner_group or split_normal_owner_group
                else 1
            )
            planner_aircraft_entries = [
                deepcopy(entry)
                for entry in component_aircraft_entries
                if isinstance(entry, dict)
            ]
            sequential_area_contexts: List[Dict[str, Any]] = []
            sequential_split_requested = False
            try:
                planner_result = run_next_collab_division_plan(
                    mission_polygon=mission_polygon,
                    aircraft_entries=planner_aircraft_entries,
                    turn_radius_scale=float(turn_radius_scale),
                    planning_mode=planning_mode_ctx,
                    log=emit,
                )
            except Exception as exc:
                emit(
                    f"[NEXTCOLLAB] division planner failed for area component "
                    f"{component_index}: {exc}"
                )
                return False
            try:
                width_target_m = float(
                    capture_physics.area_sequential_split_width_m()
                )
                width_threshold_m = float(
                    capture_physics.area_sequential_split_max_width_m()
                )
            except Exception:
                width_target_m = 0.0
                width_threshold_m = 0.0
            widest_part_m = 0.0
            current_count_by_owner: Dict[int, int] = {}
            for path_row in getattr(planner_result, "expected_paths", None) or []:
                if not isinstance(path_row, dict):
                    continue
                row_owner = int(_to_int(path_row.get("aircraftID")) or 0)
                if row_owner > 0:
                    current_count_by_owner[row_owner] = (
                        int(current_count_by_owner.get(row_owner, 0)) + 1
                    )
                if width_threshold_m <= 0.0:
                    continue
                try:
                    part_span_m = float(
                        capture_physics.max_sweep_row_chord_m_xy(
                            path_row.get("partPolygonXY"),
                            path_row.get("bearingDeg"),
                        )
                    )
                except Exception:
                    continue
                widest_part_m = max(widest_part_m, part_span_m)
            # Production expected-path rows do not always retain
            # ``partPolygonXY``. Split pieces are the authoritative geometry
            # for the final allowed-width verification.
            if width_threshold_m > 0.0:
                split_result = getattr(planner_result, "split_result", None)
                for piece in getattr(split_result, "pieces", None) or []:
                    data = (
                        piece.data
                        if isinstance(piece, SplitPiece)
                        and isinstance(piece.data, dict)
                        else {}
                    )
                    try:
                        part_span_m = float(
                            capture_physics.max_sweep_row_chord_m_llh(
                                data.get("coordinateList"),
                                data.get(
                                    "bearing_deg",
                                    data.get("phaseMoveBearing_deg"),
                                ),
                            )
                        )
                    except Exception:
                        continue
                    widest_part_m = max(widest_part_m, part_span_m)

            real_owner_ids = {
                int(_to_int(entry.get("aircraftID")) or 0)
                for entry in component_aircraft_entries
                if isinstance(entry, dict)
                and int(_to_int(entry.get("aircraftID")) or 0) > 0
            }
            minimum_stage_missing = any(
                int(current_count_by_owner.get(owner_id, 0))
                < int(minimum_stages_per_owner)
                for owner_id in real_owner_ids
            )
            width_stage_missing = bool(
                width_threshold_m > 0.0
                and widest_part_m > width_threshold_m + 1.0
            )
            if minimum_stage_missing or width_stage_missing:
                width_stage_count = (
                    int(math.ceil(widest_part_m / width_threshold_m))
                    if width_threshold_m > 0.0
                    and widest_part_m > width_threshold_m
                    else 1
                )
                requested_stage_count = max(
                    int(minimum_stages_per_owner),
                    int(width_stage_count),
                )
                width_entries, width_contexts = (
                    _branch_aircraft_sequential_area_entries(
                        component_aircraft_entries,
                        mission_polygon,
                        enabled=True,
                        stage_count=int(requested_stage_count),
                    )
                )
                if width_contexts and len(width_entries) == (
                    int(requested_stage_count)
                    * len(component_aircraft_entries)
                ):
                    try:
                        width_result = run_next_collab_division_plan(
                            mission_polygon=mission_polygon,
                            aircraft_entries=width_entries,
                            turn_radius_scale=float(turn_radius_scale),
                            planning_mode=planning_mode_ctx,
                            log=None,
                        )
                    except Exception as exc:
                        width_result = None
                        emit(
                            "[NEXTCOLLAB][AREA][WARN] sequential AREA re-plan "
                            f"failed for component {component_index}: {exc}. "
                            "Keeping the real-aircraft division."
                        )
                    if width_result is not None:
                        planner_result = width_result
                        planner_aircraft_entries = width_entries
                        sequential_area_contexts = width_contexts
                        sequential_split_requested = True
                        emit(
                            "[NEXTCOLLAB][AREA] sequential split applied "
                            f"(component {component_index}, stagesPerOwner="
                            f"{requested_stage_count}, widthTarget="
                            f"{width_target_m:,.0f}m, widthLimit="
                            f"{width_threshold_m:,.0f}m, owners="
                            f"{sorted(int(c['realAircraftID']) for c in width_contexts)})."
                        )
            # Synthetic stages are initially solved from replan-time positions.
            # Re-seed every later stage from its preceding capture exit.
            if sequential_area_contexts and bool(
                get_runtime_bool(
                    "next_collab_area_sequential_second_entry_from_first_end_enabled",
                    True,
                )
            ):
                second_pass_entries = _sequential_area_second_pass_entries(
                    planner_result,
                    planner_aircraft_entries,
                    sequential_area_contexts,
                )
                rerun_result = None
                if second_pass_entries is not None:
                    try:
                        rerun_result = run_next_collab_division_plan(
                            mission_polygon=mission_polygon,
                            aircraft_entries=second_pass_entries,
                            turn_radius_scale=float(turn_radius_scale),
                            planning_mode=planning_mode_ctx,
                            log=None,
                        )
                    except Exception as exc:
                        rerun_result = None
                        emit(
                            "[NEXTCOLLAB][AREA][WARN] second-piece entry re-plan failed for "
                            f"component {component_index}: {exc}. Keeping the first-pass entry."
                        )
                else:
                    emit(
                        "[NEXTCOLLAB][AREA][WARN] second-piece entry re-plan skipped for "
                        f"component {component_index}; first-piece exit unavailable."
                    )
                if rerun_result is not None:
                    if _sequential_area_second_pass_is_consistent(
                        rerun_result,
                        sequential_area_contexts,
                    ):
                        planner_result = rerun_result
                        emit(
                            "[NEXTCOLLAB][AREA] sequential stages re-entered from preceding exits "
                            f"(component {component_index}, owners="
                            f"{sorted(int(context['realAircraftID']) for context in sequential_area_contexts)})."
                        )
                    else:
                        emit(
                            "[NEXTCOLLAB][AREA][WARN] second-piece entry re-plan inverted the "
                            f"piece order for component {component_index}; keeping the "
                            "first-pass result."
                        )
            collapse_ok = bool(sequential_area_contexts) or not sequential_split_requested
            for sequential_context in sequential_area_contexts:
                if not _collapse_single_aircraft_sequential_area_result(
                    planner_result,
                    sequential_context,
                ):
                    collapse_ok = False
                    break
            if not collapse_ok:
                emit(
                    "[NEXTCOLLAB][AREA][WARN] sequential split did not produce "
                    "the requested path count per owner; preserving the "
                    "real-aircraft width-based plan."
                )
                try:
                    planner_result = run_next_collab_division_plan(
                        mission_polygon=mission_polygon,
                        aircraft_entries=component_aircraft_entries,
                        turn_radius_scale=float(turn_radius_scale),
                        planning_mode=planning_mode_ctx,
                        log=emit,
                    )
                except Exception as exc:
                    emit(
                        "[NEXTCOLLAB] one-piece Area fallback failed for component "
                        f"{component_index}: {exc}"
                    )
                    return False
                sequential_area_contexts = []
            # Validate the final owner IDs after the planning-only virtual UAV
            # has been folded back to the sticky branch owner. Checking before
            # the collapse would reject every valid sequential branch result.
            if branch_ownership_map is not None:
                planned_aircraft_ids = {
                    int(_to_int(row.get("aircraftID")) or 0)
                    for row in (getattr(planner_result, "expected_paths", None) or [])
                    if isinstance(row, dict)
                    and int(_to_int(row.get("aircraftID")) or 0) > 0
                }
                if planned_aircraft_ids != required_owner_ids:
                    emit(
                        f"[NEXTCOLLAB][TYPE2] branch component {component_index} output "
                        f"owner mismatch: expected={sorted(required_owner_ids)}, "
                        f"planned={sorted(planned_aircraft_ids)}. Existing assignment preserved."
                    )
                    return False
            if sequential_area_contexts:
                if split_branch_owner_group:
                    emit(
                        "[NEXTCOLLAB][TYPE2][AREA] boundary component "
                        f"{component_index} assigned as sequential missions "
                        "per sticky owner UAV "
                        f"{sorted(int(context['realAircraftID']) for context in sequential_area_contexts)}."
                    )
                else:
                    emit(
                        "[NEXTCOLLAB][AREA] remaining UAVs received sequential "
                        "area missions (aircraft="
                        f"{sorted(int(context['realAircraftID']) for context in sequential_area_contexts)})."
                    )
            planner_results.append((component, planner_result))
        return True

    initial_pass_contract = (
        _single_area_assignment_pass_contract(
            area_pass_contract,
            primary_assignment_pass,
            default_full=True,
        )
        if primary_assignment_pass is not None
        else {}
    )
    if not _run_area_pass_components(
        area_components,
        aircraft_entries,
        planning_detail=primary_planning_detail,
        pass_name=primary_assignment_pass,
        pass_contract=initial_pass_contract,
    ):
        return None

    # A fresh rugged Area has no persisted pass contract yet.  Detect the same
    # terrain condition used by the path builder, then convert the preliminary
    # division into an explicit OUT assignment and plan RETURN independently
    # from the predicted OUT exits.
    if (
        not requested_assignment_passes
        and branch_ownership_map is None
        and _area_rows_need_independent_return_assignment(planner_results)
    ):
        requested_assignment_passes = ["forward", "reverse"]
        primary_assignment_pass = "forward"
        initial_pass_contract = _single_area_assignment_pass_contract(
            {},
            "forward",
            default_full=True,
        )
        for component, _planner_result in planner_results:
            component["areaAssignedCoveragePass"] = "forward"
            component["areaPassContract"] = deepcopy(initial_pass_contract)
        emit(
            "[NEXTCOLLAB][AREA] rugged two-pass mission switched to independent "
            "OUT/RETURN available-UAV divisions."
        )

    if len(requested_assignment_passes) > 1:
        out_results = list(planner_results)
        return_entries = _area_entries_after_planned_pass(
            aircraft_entries,
            out_results,
        )
        return_detail = _area_pass_planning_detail(
            mission_detail,
            area_pass_contract,
            "reverse",
        )
        return_components = _area_planner_components_from_detail(return_detail)
        if not return_components:
            emit(
                "[NEXTCOLLAB][AREA] RETURN obligation has no valid planner geometry; "
                "independent pass planning blocked."
            )
            return None
        return_contract = _single_area_assignment_pass_contract(
            area_pass_contract,
            "reverse",
            default_full=True,
        )
        if not _run_area_pass_components(
            return_components,
            return_entries,
            planning_detail=return_detail,
            pass_name="reverse",
            pass_contract=return_contract,
        ):
            return None
        emit(
            "[NEXTCOLLAB][AREA] OUT allocated first, then RETURN independently "
            f"allocated from predicted OUT exits (uavs={len(return_entries)})."
        )
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
        component_pass = _normalize_area_assignment_pass(
            component.get("areaAssignedCoveragePass")
        )
        component_contract = (
            component.get("areaPassContract")
            if isinstance(component.get("areaPassContract"), dict)
            else area_pass_contract
        )
        component_path_rows: List[Dict[str, Any]] = []
        for row in planner_result.expected_paths:
            if not isinstance(row, dict):
                continue
            row_copy = dict(row)
            row_copy["areaComponentIndex"] = int(component_index)
            entry_coordinate_by_aircraft = component.get(
                "areaPassEntryCoordinateByAircraft"
            )
            if isinstance(entry_coordinate_by_aircraft, dict):
                planned_entry = entry_coordinate_by_aircraft.get(
                    int(_to_int(row_copy.get("aircraftID")) or 0)
                )
                if isinstance(planned_entry, dict):
                    row_copy["areaPassEntryCoordinate"] = deepcopy(planned_entry)
            sequential_entry = row_copy.get("areaSingleAircraftEntryCoordinate")
            if isinstance(sequential_entry, dict):
                row_copy["areaPassEntryCoordinate"] = deepcopy(sequential_entry)
            if component_pass is not None:
                row_copy["areaAssignedCoveragePass"] = str(component_pass)
                row_copy["areaPassAssignmentSequence"] = (
                    1 if component_pass == "forward" else 2
                )
            _apply_area_coverage_pass_contract(row_copy, component_contract)
            component_path_rows.append(row_copy)
        entry_coordinate_by_aircraft = component.get(
            "areaPassEntryCoordinateByAircraft"
        )
        _apply_width_split_sequence_metadata(
            component_path_rows,
            list(planner_result.split_result.pieces or []),
            (
                entry_coordinate_by_aircraft
                if isinstance(entry_coordinate_by_aircraft, dict)
                else None
            ),
        )
        final_path_rows.extend(component_path_rows)
    if not final_path_rows:
        emit("[NEXTCOLLAB] division planner returned no final path rows.")
        return None
    if depth_assignment_mode and branch_ownership_map is None:
        planned_counts: Dict[int, int] = {}
        for row in final_path_rows:
            aircraft_id = _to_int(row.get("aircraftID")) or 0
            if aircraft_id > 0:
                planned_counts[int(aircraft_id)] = planned_counts.get(int(aircraft_id), 0) + 1
        expected_aircraft = sorted(int(row["aircraftID"]) for row in aircraft_entries)
        expected_rows_per_aircraft = max(1, len(requested_assignment_passes))
        if (
            sorted(planned_counts) != expected_aircraft
            or any(
                int(planned_counts.get(aircraft_id, 0)) < expected_rows_per_aircraft
                for aircraft_id in expected_aircraft
            )
        ):
            emit(
                "[NEXTCOLLAB][AREA] ownership invariant failed; expected at least "
                f"{expected_rows_per_aircraft} pass row(s) per UAV "
                f"(expected={expected_aircraft}, actual={planned_counts})."
            )
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
                "stable_assignment_one_shot_depth_replan"
                if depth_assignment_mode and branch_ownership_map is None
                else "multi_component_area_division_planner_generated_replacements"
                if len(planner_results) > 1
                else "area_division_planner_generated_replacements"
            ),
            "detail": (
                "Stable ownership was divided once; exact capture-depth workload is clipped inside each owner."
                if depth_assignment_mode and branch_ownership_map is None
                else "Remaining area components were divided by the area planner for the target UAV set."
                if len(planner_results) > 1
                else "Remaining area was divided by the area planner for the target UAV set."
            ),
        },
        "areaPlannerComponentCount": len(planner_results),
        "areaPlannerComponents": [
            {
                "componentIndex": int(component.get("componentIndex") or 0),
                "coveragePass": str(
                    component.get("areaAssignedCoveragePass") or ""
                ),
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
            int(_area_assignment_pass_rank(row)),
            int(_to_int(row.get("areaComponentIndex")) or 0),
            int(_to_int(row.get("areaSingleAircraftSequence")) or 0),
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
    dem_prewarm_summary = _prewarm_dem_altitudes_for_path_rows_if_enabled(ordered_path_rows)
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
    waypoint_id_provider = _ReplacementWaypointIdProvider(
        scope="AREA",
    )
    # Emission order per aircraft, so consecutive area passes can be joined by
    # a turn link once every path has been built.
    area_path_ids_by_aircraft: Dict[int, List[int]] = {}
    sequential_path_ids_by_owner_component: Dict[
        tuple[int, int], Dict[int, int]
    ] = {}

    def _record_flight_path_metrics(metrics: Dict[str, Any]) -> None:
        with flight_path_metrics_lock:
            flight_path_metrics.append(dict(metrics))

    for aircraft_id, aircraft_path_rows in path_rows_by_aircraft.items():
        for path_index, path_row in enumerate(aircraft_path_rows):
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
            template_mission = template_record.get("mission") if isinstance(template_record.get("mission"), dict) else {}
            template_path = template_record.get("flightPath") if isinstance(template_record.get("flightPath"), dict) else {}
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
            row_area_pass_contract = {
                key: deepcopy(path_row[key])
                for key in (
                    *_AREA_COVERAGE_PASS_CONTRACT_KEYS,
                    *_AREA_COVERAGE_DEPTH_CONTRACT_KEYS,
                    "areaPassAssignmentMode",
                    "areaAssignedCoveragePass",
                )
                if key in path_row
            }
            effective_area_pass_contract = (
                row_area_pass_contract or area_pass_contract
            )
            _apply_area_coverage_pass_contract(
                mission_info,
                effective_area_pass_contract,
            )
            related_mission = deepcopy(template_mission.get("relatedMission") or {})
            related_mission["relatedMissionType"] = _to_int(related_mission.get("relatedMissionType")) or 1
            related_mission["inputMissionID"] = int(target_input_id)
            related_mission["priorMissionID"] = _to_int(related_mission.get("priorMissionID")) or 0
            generated_path_ids.add(int(new_path_id))
            area_path_ids_by_aircraft.setdefault(int(row_aircraft_id), []).append(
                int(new_path_id)
            )
            if bool(path_row.get("areaSingleAircraftSequentialSplit")):
                sequence = int(
                    _to_int(path_row.get("areaSingleAircraftSequence")) or 0
                )
                if sequence > 0:
                    sequential_path_ids_by_owner_component.setdefault(
                        (int(row_aircraft_id), int(component_index)), {}
                    )[int(sequence)] = int(new_path_id)
            new_mission_entry: Dict[str, Any] = {
                "individualMissionID": int(individual_id),
                "isDone": False,
                "relatedMission": related_mission,
                "individualMissionInfo": mission_info,
                "pathID": int(new_path_id),
            }
            _apply_area_coverage_pass_contract(
                new_mission_entry,
                effective_area_pass_contract,
            )
            bearing_deg = _to_float(path_row.get("bearingDeg"))
            if bearing_deg is not None:
                new_mission_entry["bearing_deg"] = float(bearing_deg)
            replacement_by_aircraft.setdefault(int(row_aircraft_id), []).append(new_mission_entry)
            entry_coord = (
                path_row.get("areaPassEntryCoordinate")
                if isinstance(path_row.get("areaPassEntryCoordinate"), dict)
                else entry_coord_map.get(int(row_aircraft_id)) or representative_entry
            )
            handover_coord = (
                deepcopy((handover_coord_map or {}).get(int(row_aircraft_id)))
                if path_index == len(aircraft_path_rows) - 1
                else None
            )
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
                    handover_coord=handover_coord,
                    timestamp_ms=int(now_ms): build_flight_path_from_planned_row(
                        path_row,
                        template_path=template_path,
                        mission_info=mission_info,
                        individual_mission_id=int(individual_id),
                        path_id=int(path_id),
                        aircraft_id=int(aircraft_id),
                        entry_coord=entry_coord,
                        handover_coord=handover_coord,
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
    capture_only_transition_pairs = {
        (
            int(sequence_map[sequence]),
            int(sequence_map[sequence + 1]),
        )
        for sequence_map in sequential_path_ids_by_owner_component.values()
        for sequence in sorted(sequence_map)
        if sequence + 1 in sequence_map
    }
    # Before IDs are handed out, so the link waypoints get numbered with the
    # rest of the path they belong to.
    _append_next_collab_area_transition_links(
        generated_fp_by_path=generated_fp_by_path,
        ordered_path_ids_by_aircraft=area_path_ids_by_aircraft,
        emit=emit,
        suppressed_link_pairs=capture_only_transition_pairs,
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
    review_report["planningMode"] = dict(planning_mode_ctx)
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
        planning_mode=dict(planning_mode_ctx),
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
    planning_mode: Dict[str, Any] | None = None,
    split_single_aircraft_area_into_two: bool = False,
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
    planning_mode_ctx = mission_mode_context(
        mode=planning_mode or resolve_mission_planning_mode(input_data)
    )
    # Carry the real package ID so the split pipeline can read the sticky Type 2
    # 각자도생 branch ownership store during this single-mission replan.
    planning_mode_ctx["inputMissionPackageID"] = int(source_input_pkg_id)
    emit(f"[NEXTCOLLAB][MODE] {mode_log_label(planning_mode_ctx)}")

    aircraft_entries = [entry for entry in plan_data.get("aircraftList") or [] if isinstance(entry, dict)]
    if not aircraft_entries:
        emit("[NEXTCOLLAB] source MissionPlan has no aircraftList.")
        return None

    packages_by_aircraft: Dict[int, Dict[str, Any]] = {}
    individual_mission_plan_dir = db_paths.get_db_subpath("IndividualMissionPlan")
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
            imp_src = individual_mission_plan_dir / f"{int(source_imp_id)}.json"
            imp_data = read_json_cached(imp_src, kind="IndividualMissionPlan")
        except Exception:
            continue
        # imp_data는 read_json_cached 반환본(호출자 소유)이고 이후 재사용 없음
        packages_by_aircraft[int(aircraft_id)] = imp_data
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
    is_type2_branch_mission, locked_type2_ownership = (
        _resolve_locked_type2_ownership_with_artifact_recovery(
            input_data=input_data,
            target_input_id=int(target_input_id),
            packages_by_aircraft=packages_by_aircraft,
            target_aircraft_ids=target_aircraft_ids,
            emit=emit,
        )
    )
    if is_type2_branch_mission and not locked_type2_ownership:
        emit(
            "[NEXTCOLLAB][TYPE2] immutable branch state unavailable; "
            "replan deferred and existing assignments preserved."
        )
        return None
    if locked_type2_ownership:
        required_owner_ids = {
            int(aircraft_id)
            for owners in locked_type2_ownership.values()
            for aircraft_id in owners
        }
        available_owner_ids = {int(aircraft_id) for aircraft_id in target_aircraft_ids}
        missing_owner_ids = sorted(required_owner_ids - available_owner_ids)
        if missing_owner_ids:
            # A temporary attack/prior detour is not permission to hand the
            # absent UAV's unfinished strip to another owner (including a
            # co-owner of the same area). Preserve the current IMP and let the
            # absent UAV resume its own suffix when it returns.
            emit(
                "[NEXTCOLLAB][TYPE2] immutable branch replan deferred; "
                f"missing owner UAVs={missing_owner_ids}. Existing assignments remain active."
            )
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
    effective_search_speed_scale_multiplier = (
        _effective_type2_three_branch_search_speed_scale_multiplier(
            search_speed_scale_multiplier,
            is_type2_branch_mission=bool(is_type2_branch_mission),
            locked_type2_ownership=locked_type2_ownership,
            emit=emit,
        )
    )

    def _with_prepare_metadata(prepared: Optional[_PreparedReplacements]) -> Optional[_PreparedReplacements]:
        if prepared is None:
            return None
        prepared = _apply_type2_boundary_guard_loop_to_prepared(
            prepared,
            input_data=input_data,
            input_package_id=int(source_input_pkg_id),
            target_input_id=int(target_input_id),
        )
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
        prepared.planning_mode = dict(prepared.planning_mode or planning_mode_ctx)
        prepared.review_report["planningMode"] = dict(prepared.planning_mode)
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
            planning_mode=planning_mode_ctx,
        ))

    if _is_donut_replan_mission(target_input_mission, planning_mode_ctx):
        return _with_prepare_metadata(_prepare_donut_replacements(
            target_input_mission=target_input_mission,
            target_input_id=int(target_input_id),
            target_aircraft_ids=target_aircraft_ids,
            entry_coord_map=entry_coord_map,
            template_record_map=template_record_map,
            now_ms=effective_now_ms,
            turn_radius_scale=float(effective_turn_radius_scale),
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
            planning_mode=planning_mode_ctx,
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
            planning_mode=planning_mode_ctx,
        ))

    return _with_prepare_metadata(_prepare_area_replacements(
        target_input_mission=target_input_mission,
        target_input_id=int(target_input_id),
        target_aircraft_ids=target_aircraft_ids,
        entry_coord_map=entry_coord_map,
        heading_map=resolved_heading_map,
        entry_aircraft_context_map=entry_aircraft_context_map,
        representative_entry=resolved_entry,
        template_record_map=template_record_map,
        now_ms=effective_now_ms,
        turn_radius_scale=float(effective_turn_radius_scale),
        search_speed_scale_multiplier=float(effective_search_speed_scale_multiplier),
        emit=emit,
        prepare_timer=prepare_timer,
        id_reservation_summaries=id_reservation_summaries,
        planning_mode=planning_mode_ctx,
        split_single_aircraft_into_two=bool(split_single_aircraft_area_into_two),
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
    planning_mode_ctx = mission_mode_context(mode=resolve_mission_planning_mode(input_data))
    # Carry the real package ID so the split pipeline can read the sticky Type 2
    # 각자도생 branch ownership store during this single-mission replan.
    planning_mode_ctx["inputMissionPackageID"] = int(source_input_pkg_id)
    emit(f"[NEXTCOLLAB][MODE] {mode_log_label(planning_mode_ctx)}")

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
    # HO points stay available as 0203 references/visualization, but they are
    # not implicit mission endpoints. Replanning ends at the selected input
    # mission's own final point.
    target_handover_coord_map: Dict[int, Dict[str, Any]] = {}
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
    source_individual_mission_plan_dir = db_paths.get_db_subpath("IndividualMissionPlan")
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
            imp_src = source_individual_mission_plan_dir / f"{int(source_imp_id)}.json"
            imp_data = read_json_cached(imp_src, kind="IndividualMissionPlan")
        except Exception as exc:
            emit(f"[NEXTCOLLAB] failed to load IndividualMissionPlan {source_imp_id}: {exc}")
            return None
        new_imp_id = int(new_imp_ids[idx])
        # imp_data는 read_json_cached 반환본(호출자 소유)이고 이후 재사용 없음
        new_imp_data = imp_data
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

    is_type2_branch_mission, locked_type2_ownership = (
        _resolve_locked_type2_ownership_with_artifact_recovery(
            input_data=input_data,
            target_input_id=int(target_input_id),
            packages_by_aircraft=packages_by_aircraft,
            target_aircraft_ids=target_aircraft_ids,
            emit=emit,
        )
    )
    if is_type2_branch_mission and not locked_type2_ownership:
        emit(
            "[NEXTCOLLAB][TYPE2] immutable branch state unavailable; "
            "replan deferred and existing assignments preserved."
        )
        return None
    if locked_type2_ownership:
        required_owner_ids = {
            int(aircraft_id)
            for owners in locked_type2_ownership.values()
            for aircraft_id in owners
        }
        missing_owner_ids = sorted(
            required_owner_ids - {int(aircraft_id) for aircraft_id in target_aircraft_ids}
        )
        if missing_owner_ids:
            emit(
                "[NEXTCOLLAB][TYPE2] immutable branch replan deferred; "
                f"missing owner UAVs={missing_owner_ids}. Existing assignments remain active."
            )
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
    retired_unavailable_uav_missions = _retire_unavailable_uav_missions_from_target(
        packages_by_aircraft,
        target_aircraft_ids=target_aircraft_ids,
        target_input_id=int(target_input_id),
    )
    if retired_unavailable_uav_missions:
        emit(
            "[NEXTCOLLAB] unavailable UAV target/downstream missions retired "
            f"(targetInputMissionID={int(target_input_id)}, "
            f"aircraft={sorted(retired_unavailable_uav_missions)})."
        )
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
    search_speed_scale_multiplier = (
        _effective_type2_three_branch_search_speed_scale_multiplier(
            1.0,
            is_type2_branch_mission=bool(is_type2_branch_mission),
            locked_type2_ownership=locked_type2_ownership,
            emit=emit,
        )
    )
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
            planning_mode=planning_mode_ctx,
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
    elif _is_donut_replan_mission(target_input_mission, planning_mode_ctx):
        prepared = _prepare_donut_replacements(
            target_input_mission=target_input_mission,
            target_input_id=int(target_input_id),
            target_aircraft_ids=target_aircraft_ids,
            entry_coord_map=entry_coord_map,
            template_record_map=template_record_map,
            now_ms=int(now_ms),
            turn_radius_scale=float(turn_radius_scale),
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
            planning_mode=planning_mode_ctx,
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
            search_speed_scale_multiplier=float(search_speed_scale_multiplier),
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
            planning_mode=planning_mode_ctx,
            handover_coord_map=target_handover_coord_map,
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
            entry_aircraft_context_map=entry_aircraft_context_map,
            representative_entry=representative_entry,
            template_record_map=template_record_map,
            now_ms=int(now_ms),
            turn_radius_scale=float(turn_radius_scale),
            search_speed_scale_multiplier=float(search_speed_scale_multiplier),
            emit=emit,
            prepare_timer=prepare_timer,
            id_reservation_summaries=id_reservation_summaries,
            planning_mode=planning_mode_ctx,
            handover_coord_map=target_handover_coord_map,
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

    prepared = _apply_type2_boundary_guard_loop_to_prepared(
        prepared,
        input_data=input_data,
        input_package_id=int(source_input_pkg_id),
        target_input_id=int(target_input_id),
    )
    replacement_by_aircraft = {
        int(aid): list(rows)
        for aid, rows in prepared.replacement_by_aircraft.items()
    }
    generated_fp_by_path = {
        int(path_id): payload
        for path_id, payload in prepared.generated_fp_by_path.items()
    }
    area_review_report = dict(prepared.review_report or {})

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
    area_review_report["planningMode"] = dict(prepared.planning_mode or planning_mode_ctx)
    area_review_report["prepareTimingMs"] = dict(prepare_timings_ms)
    area_review_report["sourceCache"] = dict(source_cache_summary)
    area_review_report["idReservation"] = dict(id_reservation_summary)
    area_review_report["uavIndependentWork"] = dict(uav_work_summary)
    area_review_report["runtimePreservation"] = dict(runtime_preservation)
    if retired_unavailable_uav_missions:
        area_review_report["retiredUnavailableUavMissions"] = {
            int(aid): dict(row)
            for aid, row in sorted(retired_unavailable_uav_missions.items())
        }

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

    execution_barrier = _apply_input_order_execution_barrier(
        list(packages_by_aircraft.values()),
        input_plan=input_data,
        target_input_id=int(target_input_id),
    )
    area_review_report["executionBarrier"] = dict(execution_barrier)
    emit(
        "[NEXTCOLLAB] input-order execution barrier applied "
        f"targetInputMissionID={int(target_input_id)} "
        f"targetUnblocked={int(execution_barrier['targetUnblocked'])} "
        f"laterBlocked={int(execution_barrier['laterBlocked'])}"
    )

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
            lah_route_start_by_aircraft=_load_lah_route_start_coordinates(
                entry_coord_map,
                _extract_entry_aircraft_context_map(detail),
            ),
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

    mission_plan_dir = db_paths.get_db_subpath("MissionPlan")
    individual_mission_plan_dir = db_paths.get_db_subpath("IndividualMissionPlan")
    flight_path_dir = db_paths.get_db_subpath("FlightPath")
    plan_dest = mission_plan_dir / f"{int(new_plan_id)}.json"
    imp_rows: List[tuple[Path, Dict[str, Any]]] = []
    for aircraft_id, pkg in packages_by_aircraft.items():
        imp_id = _to_int(pkg.get("individualMissionPackageID"))
        if imp_id is None or imp_id <= 0:
            emit(f"[NEXTCOLLAB] cloned IMP missing individualMissionPackageID for aircraft {aircraft_id}.")
            return None
        imp_rows.append(
            (
                individual_mission_plan_dir / f"{int(imp_id)}.json",
                pkg,
            )
        )

    fp_rows: List[tuple[Path, Dict[str, Any]]] = []
    for path_id in sorted(generated_path_ids):
        fp_rows.append(
            (
                flight_path_dir / f"{int(path_id)}.json",
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
