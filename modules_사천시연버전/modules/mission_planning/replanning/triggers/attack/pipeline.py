from __future__ import annotations

import json
import concurrent.futures
import os
import subprocess
from copy import deepcopy
import importlib.util
import math
import time
from collections import OrderedDict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple
import sys
from types import ModuleType

from modules.common import agent_status_snapshot, db_paths, mission_area_replan_store
from modules.common.eta import annotate_eta_flight_plan
from modules.mission_planning._paths import mission_planner_root, mission_planning_root, project_root
from modules.mission_planning.MissionPlanner.runtime_settings import (
    get_runtime_effective_fov_deg,
    get_runtime_float,
    get_runtime_attack_float,
    get_runtime_attack_int,
    get_runtime_attack_int_list,
    get_runtime_attack_target_type_priority,
    get_runtime_attack_weapon_type,
    get_runtime_attack_weapon_type_for_target_type,
    pop_runtime_camera_fov_adjustment_logs,
)
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    sanitize_flight_path_payload_filming_altitudes,
)
from modules.mission_planning.engine.mission_generation.id_allocation.allocator import (
    reserve_mission_plan_ids,
)
from modules.mission_planning.MissionPlanner.dynamics.lah_op_envlp import DEFAULT_ENVELOPE
from modules.mission_planning.runtime.debug_artifacts import debug_artifact_mode, write_debug_json
from modules.mission_planning.runtime.json_io import write_json, write_json_batch
from modules.mission_planning.runtime.cache.source_artifacts import read_json_cached
from modules.mission_planning.runtime.logging.pipeline_events import (
    PipelinePhaseTimer,
    new_replan_transaction_id,
)
from modules.mission_planning.runtime.ids.replan_reservation import ReplanIdReservation
from modules.mission_planning.runtime.validation.replan_payloads import (
    ReplanValidationError,
    validate_generated_artifact_payloads,
    validate_replan_payloads,
)
from modules.mission_planning.runtime.state.attack_assignment import (
    get_last_assigned_manned_id,
    get_used_manned_ids,
    set_pending_manned_assignment,
    set_pending_manned_assignments,
    set_last_assigned_manned_id,
)
from modules.mission_planning.runtime.state.attack_tracking import (
    clear_tracking_assignment,
    get_tracking_assignment,
    list_active_tracking_assignments,
    register_tracking_assignment,
)

_ATTACK_ROOT = mission_planning_root()
_MP_DIR = mission_planner_root()
_PROJECT_ROOT = project_root()
_RECEIVE_DB_MOD: Optional[ModuleType] = None
for _candidate in (_PROJECT_ROOT, _ATTACK_ROOT, _MP_DIR):
    _candidate_str = str(_candidate)
    if _candidate.exists() and _candidate_str not in sys.path:
        sys.path.insert(0, _candidate_str)

from modules.mission_planning.replanning.triggers.prior.pipeline import (
    CollaborativeResumeReplanResult,
    PlanMissionArtifacts,
    _apply_release_resume_mission_info,
    _build_other_uav_resume_package,
    _build_done_reference_mission,
    _build_uav_release_resume_waypoints,
    _RELEASE_RESUME_FAST_SPEED_MPS,
    _clone_follow_up_replan_artifacts,
    _extract_final_uav_coordinate,
    _extract_related_input_mission_id,
    _load_input_plan_for_source_plan,
    _load_done_input_ids_for_plan,
    _load_latest_mission_progress_plan_id,
    _normalize_altitude_value,
    _prepare_uav_collaborative_resume_replan,
    _project_coordinate,
    _resolve_plan_artifacts,
    _scan_latest_source_plan_id,
    _bearing_between,
    _reserve_waypoint_block,
    warm_prior_mission_pipeline,
    _sync_resume_mission_info_with_waypoints,
)
from modules.mission_planning.pipelines.mission_planning_attack_helpers import (
    choose_attack_weapon_type,
    extract_attack_weapon_inventory,
    get_attack_standoff_distances,
    select_attack_standoff_coordinate,
)
from modules.mission_planning.pipelines.lah_operational_mode import (
    detect_lah_special_operation,
    special_attack_coordinate,
    special_force_battle_attack,
    special_target_contains_coordinate,
)
from modules.mission_planning.pipelines.mission_path_trim import (
    DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
    count_sweep_points_in_waypoints,
    estimate_sweep_buffer_points,
    load_sweep_progress,
    merge_small_adjacent_line_search_waypoints,
    preserve_first_waypoint_altitude_from_reference,
    reassign_unique_waypoint_ids_inplace,
    realign_line_search_waypoints_to_first_sweep,
    relink_waypoints,
    recompute_line_search_speed_from_geometry,
    scale_line_search_speed,
    sweep_cut_points,
    trim_waypoints_by_sweep_points,
)
from modules.mission_planning.pipelines.line_search_speed_guard import (
    clamp_line_search_speed_mps,
    effective_line_search_transit_m,
)
LogCallback = Callable[[str], None]

LOG_FILENAME = "log_attack_algorithm.json"
_MISSION_PLAN_START = 700_000_001
_ATTACK_POINT_CACHE: "OrderedDict[Tuple[float, ...], Dict[str, Any]]" = OrderedDict()
_ATTACK_ASSIST_MODULE: Optional[ModuleType] = None
_ATTACK_ASSIST_IMPORT_ERROR: Optional[str] = None
_ATTACK_ASSIST_RASTER_PATHS_CACHE: Dict[str, List[Any]] = {}
_ATTACK_POINT_SUBPROCESS_DISABLED_REASON: Optional[str] = None
_UAV_TRACKING_MIN_FLIGHT_ALTITUDE_M = 700.0
_UAV_TRACKING_MAX_FLIGHT_ALTITUDE_M = 2200.0
_LAH_ATTACK_MAX_SPEED_KMH_FALLBACK = 265.0
_LAH_RESUME_PRESERVE_TWO_POINT_MIN_LENGTH_M = 2000.0
_ATTACK_POINT_SUBPROCESS_TIMEOUT_S = 2.0
_ATTACK_LOS_MAX_RAYS = 360
_ATTACK_LOS_MIN_RAYS = 36
_ATTACK_LOS_MAX_RADIUS_M = 2000.0
_ATTACK_LOS_MIN_RADIUS_M = 500.0
_ATTACK_POINT_META_KEYS = (
    "selection_mode",
    "los_area",
    "raster_sources",
    "terrain_altitude_m",
    "altitude_offset_m",
    "analysis_radius_m",
    "num_rays",
    "lah_altitude_floor_m",
    "altitude_floor_applied",
)
ATTACK_FAILURE_NOTICES: Dict[str, str] = {
    "attack_weapon_unavailable": "공격 불가: 공격기 탄약 부족",
    "manned_unavailable": "공격 불가: 가용 유인기 없음",
    "missing_friendly_coordinate": "공격 불가: 공격기 위치 없음",
    "missing_target_coordinate": "공격 불가: 표적 좌표 없음",
    "attack_point_failed": "공격 불가: 공격 지점 계산 실패",
    "attack_override_failed": "공격 불가: 산출물 생성 실패",
    "attack_override_metadata_missing": "공격 불가: 메타데이터 부족",
    "attack_override_aircraft_id_missing": "공격 불가: 항공기 ID 부족",
    "attack_override_manned_coordinate_missing": "공격 불가: 공격기 위치 없음",
    "attack_override_source_plan_missing": "공격 불가: 기준 MP 없음",
    "attack_override_source_plan_load_failed": "공격 불가: 기준 MP 로드 실패",
    "attack_tracking_assignment_failed": "공격 불가: 추적 UAV 배정 실패",
    "attack_assignment_failed": "공격 불가: 무장/편대 배정 실패",
    "attack_override_artifacts_empty": "공격 불가: 산출물 없음",
    "attack_validation_failed": "공격 불가: 산출물 검증 실패",
}


def attack_failure_notice(code: str) -> str:
    return ATTACK_FAILURE_NOTICES.get(str(code or "").strip(), ATTACK_FAILURE_NOTICES["attack_override_failed"])


def _lah_max_attack_speed_mps() -> float:
    try:
        max_speed_kmh = float(getattr(DEFAULT_ENVELOPE, "max_speed_kmh", _LAH_ATTACK_MAX_SPEED_KMH_FALLBACK))
    except Exception:
        max_speed_kmh = _LAH_ATTACK_MAX_SPEED_KMH_FALLBACK
    if not math.isfinite(max_speed_kmh) or max_speed_kmh <= 0.0:
        max_speed_kmh = _LAH_ATTACK_MAX_SPEED_KMH_FALLBACK
    return round(float(max_speed_kmh) / 3.6, 2)


def _attack_manned_candidates() -> tuple[int, ...]:
    candidates = tuple(get_runtime_attack_int_list("manned_candidate_ids", [2, 3]))
    return candidates or (2, 3)


def warm_attack_plan_pipeline() -> Dict[str, Any]:
    """Preload lazy dependencies used by the attack replan path."""
    status: Dict[str, Any] = {"prior_pipeline": warm_prior_mission_pipeline()}
    status["attack_assist_mode"] = "adaptive_standoff"
    status["compute_attack_point_available"] = True
    assist, import_error = _load_attack_assist_module()
    status["attack_assist_loaded"] = assist is not None
    if import_error:
        status["attack_assist_import_error"] = import_error
    if assist is not None:
        try:
            raster_paths = _detect_attack_raster_paths_cached(assist)
            status["attack_raster_path_count"] = len(raster_paths)
        except Exception as exc:
            status["attack_raster_warm_error"] = str(exc)
    return status


def _attack_assist_script_path() -> Path:
    return _MP_DIR / "data_def" / "lah_attack_assistance.py"


def _load_attack_assist_module() -> Tuple[Optional[ModuleType], Optional[str]]:
    global _ATTACK_ASSIST_MODULE, _ATTACK_ASSIST_IMPORT_ERROR
    if _ATTACK_ASSIST_MODULE is not None:
        return _ATTACK_ASSIST_MODULE, None
    if _ATTACK_ASSIST_IMPORT_ERROR:
        return None, _ATTACK_ASSIST_IMPORT_ERROR

    try:
        from modules.mission_planning.MissionPlanner.data_def import lah_attack_assistance
    except SystemExit as exc:
        _ATTACK_ASSIST_IMPORT_ERROR = str(exc)
        return None, _ATTACK_ASSIST_IMPORT_ERROR
    except Exception as exc:
        _ATTACK_ASSIST_IMPORT_ERROR = str(exc)
        return None, _ATTACK_ASSIST_IMPORT_ERROR

    _ATTACK_ASSIST_MODULE = lah_attack_assistance
    return _ATTACK_ASSIST_MODULE, None


def _detect_attack_raster_paths_cached(assist: ModuleType) -> List[Any]:
    raster_root = _PROJECT_ROOT / "resource"
    root_key = str(raster_root) if raster_root.exists() else ""
    cached = _ATTACK_ASSIST_RASTER_PATHS_CACHE.get(root_key)
    if cached is not None:
        return list(cached)
    raster_paths = assist.detect_raster_paths(root_key or None)
    normalized = list(raster_paths or [])
    _ATTACK_ASSIST_RASTER_PATHS_CACHE[root_key] = list(normalized)
    return list(normalized)


def _attack_descriptor_worker_context(ctx: Dict[str, Any]) -> Dict[str, Any]:
    worker_ctx = dict(ctx or {})
    for key in (
        "mission_ids",
        "missionIDs",
        "option_names",
        "plan_ids",
        "_attack_target_list",
        "_selected_manned_aircraft",
    ):
        value = worker_ctx.get(key)
        if isinstance(value, list):
            worker_ctx[key] = list(value)
    for key in (
        "replan_detail",
        "_selected_attack_weapon_choice",
        "_lah_special_operation",
    ):
        value = worker_ctx.get(key)
        if isinstance(value, dict):
            worker_ctx[key] = deepcopy(value)
    return worker_ctx


def _attack_los_enabled() -> bool:
    return bool(get_runtime_attack_int("attack_los_enabled", 1))


def _attack_los_num_rays() -> int:
    value = get_runtime_attack_int("fast_num_arc_rays", 180)
    try:
        rays = int(value)
    except Exception:
        rays = 180
    return max(_ATTACK_LOS_MIN_RAYS, min(_ATTACK_LOS_MAX_RAYS, rays))


def _attack_los_analysis_radius_m(preferred_standoff_m: Optional[float]) -> float:
    preferred = _to_float(preferred_standoff_m)
    if preferred is None or not math.isfinite(preferred) or preferred <= 0.0:
        preferred = _ATTACK_LOS_MAX_RADIUS_M
    return max(_ATTACK_LOS_MIN_RADIUS_M, min(_ATTACK_LOS_MAX_RADIUS_M, float(preferred)))


def _attack_point_cache_decimals(key: str, default: int, *, minimum: int, maximum: int) -> int:
    try:
        value = int(get_runtime_attack_int(key, default))
    except Exception:
        value = int(default)
    return max(int(minimum), min(int(maximum), int(value)))


def _build_attack_point_cache_key(
    friendly_norm: Dict[str, Any],
    enemy_norm: Dict[str, Any],
    *,
    min_standoff_m: float,
    preferred_standoff_m: float,
    altitude_offset_m: float,
    los_enabled: bool,
    los_num_rays: int,
    los_analysis_radius_m: float,
) -> Tuple[float, ...]:
    friendly_decimals = _attack_point_cache_decimals(
        "attack_point_cache_friendly_decimals",
        4,
        minimum=4,
        maximum=7,
    )
    target_decimals = _attack_point_cache_decimals(
        "attack_point_cache_target_decimals",
        5,
        minimum=4,
        maximum=7,
    )
    return (
        round(float(friendly_norm["latitude"]), int(friendly_decimals)),
        round(float(friendly_norm["longitude"]), int(friendly_decimals)),
        round(float(enemy_norm["latitude"]), int(target_decimals)),
        round(float(enemy_norm["longitude"]), int(target_decimals)),
        round(float(min_standoff_m), 1),
        round(float(preferred_standoff_m), 1),
        round(float(altitude_offset_m), 1),
        1.0 if los_enabled else 0.0,
        float(los_num_rays),
        round(float(los_analysis_radius_m), 1),
    )


def _cache_attack_point(cache_key: Tuple[float, ...], result: Dict[str, Any]) -> int:
    _ATTACK_POINT_CACHE[cache_key] = dict(result)
    _ATTACK_POINT_CACHE.move_to_end(cache_key)
    cache_limit = max(1, int(get_runtime_attack_int("point_cache_max", 16)))
    while len(_ATTACK_POINT_CACHE) > cache_limit:
        _ATTACK_POINT_CACHE.popitem(last=False)
    return cache_limit


def _copy_cached_attack_point(cached: Dict[str, Any], friendly_norm: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(cached)
    terrain_altitude = _normalize_altitude_value(result.get("terrain_altitude_m"))
    altitude_offset = _to_float(result.get("altitude_offset_m"))
    if terrain_altitude is not None and altitude_offset is not None:
        result["altitude"] = _normalize_altitude_value(float(terrain_altitude) + float(altitude_offset))
        result.pop("lah_altitude_floor_m", None)
        result.pop("altitude_floor_applied", None)
    _apply_lah_altitude_floor(result, friendly_norm)
    return result


def _attack_point_los_missing_dependency(error: Any) -> bool:
    return "GDAL for Python is required" in str(error or "")


def _attack_point_selection_mode(value: Any) -> str:
    if not isinstance(value, dict):
        return ""
    return str(value.get("selection_mode") or value.get("attack_point_selection_mode") or "").strip()


def _is_los_attack_point(value: Any) -> bool:
    mode = _attack_point_selection_mode(value).lower()
    return bool(mode.startswith("los_area") or (isinstance(value, dict) and value.get("los_area") is True))


def _preserve_attack_point_altitude(value: Any) -> bool:
    mode = _attack_point_selection_mode(value).lower()
    return bool(mode.startswith("los_area") or mode.startswith("special_"))


def _attach_attack_point_metadata(coord: Optional[Dict[str, Any]], source: Any) -> Optional[Dict[str, Any]]:
    if coord is None:
        return None
    if not isinstance(source, dict):
        return coord
    for key in _ATTACK_POINT_META_KEYS:
        if key in source:
            coord[key] = deepcopy(source.get(key))
    return coord


def _apply_lah_altitude_floor(
    coord: Optional[Dict[str, Any]],
    lah_coord: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    if not isinstance(coord, dict):
        return coord
    if not isinstance(lah_coord, dict):
        return coord
    lah_altitude = _normalize_altitude_value(lah_coord.get("altitude") or lah_coord.get("alt"))
    if lah_altitude is None:
        return coord
    current_altitude = _normalize_altitude_value(coord.get("altitude") or coord.get("alt"))
    if current_altitude is None or int(current_altitude) < int(lah_altitude):
        coord["altitude"] = int(lah_altitude)
        coord["lah_altitude_floor_m"] = int(lah_altitude)
        coord["altitude_floor_applied"] = True
    elif current_altitude is not None:
        coord["altitude"] = int(current_altitude)
    return coord


def _lah_special_battle_anchor_for_input(
    ctx: Dict[str, Any],
    *,
    aircraft_id: int,
    input_mission_id: object | None,
) -> Optional[Dict[str, Any]]:
    if int(aircraft_id) not in (1, 2, 3):
        return None
    profile = ctx.get("_lah_special_operation") if isinstance(ctx, dict) else None
    if not special_force_battle_attack(profile, input_mission_id):
        return None
    return special_attack_coordinate(profile)


def _compute_attack_point_subprocess(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    *,
    min_standoff_m: float,
    preferred_standoff_m: float,
) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    global _ATTACK_POINT_SUBPROCESS_DISABLED_REASON
    if _ATTACK_POINT_SUBPROCESS_DISABLED_REASON:
        return None, f"Attack point subprocess skipped: {_ATTACK_POINT_SUBPROCESS_DISABLED_REASON}"

    script_path = _attack_assist_script_path()
    if not script_path.exists():
        return None, f"Attack assistance script not found: {script_path}"

    try:
        friendly_lat = float(friendly_coord.get("latitude"))
        friendly_lon = float(friendly_coord.get("longitude"))
        enemy_lat = float(enemy_coord.get("latitude"))
        enemy_lon = float(enemy_coord.get("longitude"))
    except Exception as exc:
        return None, f"Invalid coordinate data for subprocess attack calculation: {exc}"

    num_rays = _attack_los_num_rays()
    analysis_radius_m = _attack_los_analysis_radius_m(preferred_standoff_m)
    cmd = [
        sys.executable or "python",
        str(script_path),
        "--friendly-lat",
        str(friendly_lat),
        "--friendly-lon",
        str(friendly_lon),
        "--enemy-lat",
        str(enemy_lat),
        "--enemy-lon",
        str(enemy_lon),
        "--radius-m",
        str(analysis_radius_m),
        "--num-rays",
        str(num_rays),
        "--output-json",
    ]
    try:
        timeout_s = float(
            get_runtime_attack_float(
                "attack_point_subprocess_timeout_s",
                _ATTACK_POINT_SUBPROCESS_TIMEOUT_S,
            )
        )
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            timeout_s = _ATTACK_POINT_SUBPROCESS_TIMEOUT_S
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=False,
            cwd=str(_PROJECT_ROOT),
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return None, f"Attack point subprocess timed out after {timeout_s:.1f}s"
    except Exception as exc:
        return None, f"Attack point subprocess launch failed: {exc}"

    if result.returncode != 0:
        stderr_msg = (result.stderr or "").strip()
        stdout_msg = (result.stdout or "").strip()
        detail = stderr_msg or stdout_msg or f"exit={result.returncode}"
        if _attack_point_los_missing_dependency(detail):
            _ATTACK_POINT_SUBPROCESS_DISABLED_REASON = detail
        return None, f"Attack point subprocess failed: {detail}"

    try:
        payload = json.loads(result.stdout or "{}")
    except Exception as exc:
        return None, f"Attack point subprocess returned invalid JSON: {exc}"

    attack_point = payload.get("attack_point") or {}
    lat_val = attack_point.get("lat", attack_point.get("latitude"))
    lon_val = attack_point.get("lon", attack_point.get("longitude"))
    altitude_val = attack_point.get("alt_m", attack_point.get("altitude"))
    if lat_val is None or lon_val is None:
        return None, "Attack point subprocess returned no coordinates."

    altitude_offset_m = get_runtime_attack_float("attack_point_altitude_offset_m", 300.0)
    fallback_base_altitude = _to_float(enemy_coord.get("altitude"))
    base_altitude = float(altitude_val) if altitude_val is not None else float(fallback_base_altitude or 0.0)
    raster_sources = payload.get("raster_sources") or []
    raster_path = payload.get("raster_path")
    if raster_path and raster_path not in raster_sources:
        raster_sources = [raster_path, *raster_sources]
    los_coord = {
        "latitude": float(lat_val),
        "longitude": float(lon_val),
        "altitude": _normalize_altitude_value(float(base_altitude + altitude_offset_m)),
    }
    _apply_lah_altitude_floor(los_coord, friendly_coord)
    enemy_distance_m = _haversine_distance_m(enemy_coord, los_coord)
    if enemy_distance_m is None:
        return None, "LOS attack point distance check failed."
    if float(enemy_distance_m) < float(min_standoff_m):
        return None, (
            "LOS attack point rejected: "
            f"distance={float(enemy_distance_m):.1f}m < minStandoff={float(min_standoff_m):.1f}m"
        )
    friendly_distance_m = _haversine_distance_m(friendly_coord, los_coord)
    current_distance_m = _haversine_distance_m(friendly_coord, enemy_coord)
    result = {
        "latitude": float(los_coord["latitude"]),
        "longitude": float(los_coord["longitude"]),
        "altitude": _normalize_altitude_value(los_coord.get("altitude")),
        "friendly_distance_m": friendly_distance_m,
        "enemy_distance_m": enemy_distance_m,
        "current_distance_m": current_distance_m,
        "candidate_distance_m": enemy_distance_m,
        "min_standoff_m": float(min_standoff_m),
        "preferred_standoff_m": float(preferred_standoff_m),
        "raster_sources": raster_sources,
        "selection_mode": "los_area",
        "los_area": True,
        "terrain_altitude_m": _normalize_altitude_value(base_altitude),
        "altitude_offset_m": float(altitude_offset_m),
        "analysis_radius_m": float(analysis_radius_m),
        "num_rays": int(num_rays),
    }
    _apply_lah_altitude_floor(result, friendly_coord)
    return (result, None)


def _compute_attack_point_inprocess(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    *,
    min_standoff_m: float,
    preferred_standoff_m: float,
) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    if not bool(get_runtime_attack_int("attack_point_inprocess_enabled", 1)):
        return None, "in-process attack point calculation disabled"

    assist, import_error = _load_attack_assist_module()
    if assist is None:
        return None, f"Attack assistance module unavailable: {import_error or 'unknown'}"

    try:
        friendly_lat = float(friendly_coord.get("latitude"))
        friendly_lon = float(friendly_coord.get("longitude"))
        enemy_lat = float(enemy_coord.get("latitude"))
        enemy_lon = float(enemy_coord.get("longitude"))
    except Exception as exc:
        return None, f"Invalid coordinate data for in-process attack calculation: {exc}"

    num_rays = _attack_los_num_rays()
    analysis_radius_m = _attack_los_analysis_radius_m(preferred_standoff_m)
    friendly_world = (float(friendly_lon), float(friendly_lat))
    enemy_world = (float(enemy_lon), float(enemy_lat))
    try:
        raster_paths = _detect_attack_raster_paths_cached(assist)
        elevation, geotransform, used_rasters = assist.load_elevation(
            raster_paths,
            enemy_world,
            radius_m=float(analysis_radius_m),
        )
        if not used_rasters:
            return None, "No GeoTIFF tiles overlapped the requested analysis bounds."
        if geotransform is None:
            return None, "GeoTIFF mosaic is missing georeferencing."

        enemy_px = assist.ensure_point_inside(enemy_world, geotransform, elevation)
        arc = assist.compute_cover_disk(
            elevation,
            geotransform,
            enemy_pixel=enemy_px,
            radius_m=float(analysis_radius_m),
            num_rays=int(num_rays),
        )
        cell_data = assist.compute_cell_data(arc)
        polygons = assist.build_danger_polygons(
            cell_data,
            arc.world_x,
            arc.world_y,
            arc,
            geotransform,
        )
        if not polygons:
            return None, "No attack candidate areas detected."
        best = assist.choose_attack_point(polygons, friendly_world, enemy_world, geotransform)
        if not best:
            return None, "Failed to derive a centroid-based recommendation."

        best_point = best.get("centroid")
        if not isinstance(best_point, (tuple, list)) or len(best_point) < 2:
            return None, "Attack assistance module returned invalid centroid."
        altitude = assist.sample_elevation_at_world(elevation, (float(best_point[0]), float(best_point[1])), geotransform)
    except Exception as exc:
        return None, f"In-process attack point calculation failed: {exc}"

    altitude_offset_m = get_runtime_attack_float("attack_point_altitude_offset_m", 300.0)
    fallback_base_altitude = _to_float(enemy_coord.get("altitude"))
    base_altitude = float(altitude) if math.isfinite(float(altitude)) else float(fallback_base_altitude or 0.0)
    los_coord = {
        "latitude": float(best_point[1]),
        "longitude": float(best_point[0]),
        "altitude": _normalize_altitude_value(float(base_altitude + altitude_offset_m)),
    }
    _apply_lah_altitude_floor(los_coord, friendly_coord)
    enemy_distance_m = _haversine_distance_m(enemy_coord, los_coord)
    if enemy_distance_m is None:
        return None, "LOS attack point distance check failed."
    if float(enemy_distance_m) < float(min_standoff_m):
        return None, (
            "LOS attack point rejected: "
            f"distance={float(enemy_distance_m):.1f}m < minStandoff={float(min_standoff_m):.1f}m"
        )

    raster_sources = [os.path.abspath(str(path)) for path in (used_rasters or [])]
    friendly_distance_m = _haversine_distance_m(friendly_coord, los_coord)
    current_distance_m = _haversine_distance_m(friendly_coord, enemy_coord)
    result = {
        "latitude": float(los_coord["latitude"]),
        "longitude": float(los_coord["longitude"]),
        "altitude": _normalize_altitude_value(los_coord.get("altitude")),
        "friendly_distance_m": friendly_distance_m,
        "enemy_distance_m": enemy_distance_m,
        "current_distance_m": current_distance_m,
        "candidate_distance_m": enemy_distance_m,
        "min_standoff_m": float(min_standoff_m),
        "preferred_standoff_m": float(preferred_standoff_m),
        "raster_sources": raster_sources,
        "selection_mode": "los_area",
        "los_area": True,
        "terrain_altitude_m": _normalize_altitude_value(base_altitude),
        "altitude_offset_m": float(altitude_offset_m),
        "analysis_radius_m": float(analysis_radius_m),
        "num_rays": int(num_rays),
        "execution_mode": "inprocess",
    }
    _apply_lah_altitude_floor(result, friendly_coord)
    return (result, None)


def _allocate_fresh_plan_id() -> int:
    try:
        reserved_ids = reserve_mission_plan_ids(1)
        if reserved_ids:
            return int(reserved_ids[0])
    except Exception:
        pass
    raise RuntimeError("missionPlanID reservation failed")


def _recent_mission_plan_ids_before(
    anchor_plan_id: Optional[int],
    *,
    limit: int = 24,
) -> List[int]:
    try:
        plan_dir = db_paths.get_db_subpath("MissionPlan")
    except Exception:
        return []

    anchor = _to_int(anchor_plan_id)
    ids: List[int] = []
    try:
        for item in plan_dir.glob("*.json"):
            if not item.stem.isdigit():
                continue
            plan_id = int(item.stem)
            if anchor is not None and plan_id > anchor:
                continue
            ids.append(plan_id)
    except Exception:
        return []

    ids = sorted(set(ids), reverse=True)
    return ids[: max(1, int(limit))]


def _resolve_attack_exclusion_source_plan_id(
    *,
    source_plan_id: Optional[int],
    aircraft_id: Optional[int],
    current_waypoint_id: Optional[int],
    emit: LogCallback,
) -> Optional[int]:
    base_source_plan_id = _to_int(source_plan_id)
    if aircraft_id is None or current_waypoint_id is None:
        return base_source_plan_id

    candidate_plan_ids: List[int] = []
    seen_plan_ids: set[int] = set()
    for value in (
        base_source_plan_id,
        _load_latest_mission_progress_plan_id(),
        _scan_latest_source_plan_id(),
    ):
        plan_id = _to_int(value)
        if plan_id is None or plan_id in seen_plan_ids:
            continue
        seen_plan_ids.add(plan_id)
        candidate_plan_ids.append(plan_id)

    # Right after 0702 application the monitor may already regard the new plan as
    # active while 0401/currentWaypointID still points to the previously applied
    # plan. Search recent prior plans by the live WP before falling back.
    for plan_id in _recent_mission_plan_ids_before(base_source_plan_id):
        if plan_id in seen_plan_ids:
            continue
        seen_plan_ids.add(plan_id)
        candidate_plan_ids.append(plan_id)

    silent_emit: LogCallback = lambda _message: None
    for candidate_plan_id in candidate_plan_ids:
        artifacts = _resolve_plan_artifacts(
            source_plan_id=candidate_plan_id,
            aircraft_id=aircraft_id,
            current_waypoint_id=current_waypoint_id,
            emit=silent_emit,
            allow_first_mission_fallback=False,
        )
        if artifacts is None:
            continue
        if base_source_plan_id is not None and candidate_plan_id != base_source_plan_id:
            emit(
                f"UAV {aircraft_id} resume source adjusted "
                f"{base_source_plan_id} -> {candidate_plan_id} "
                f"(matched waypoint {current_waypoint_id})."
            )
        return candidate_plan_id

    return base_source_plan_id


def run_attack_plan_pipeline(
    ctx: Dict[str, Any],
    log_callback: Optional[LogCallback] = None,
) -> Dict[str, Any]:
    """
    Execute the specialized attack-planning pre-processing flow.
    Returns a dictionary that is also persisted to DSS_Internal/log_attack_algorithm.json.
    """

    log_messages: List[str] = []
    pipeline_started = time.perf_counter()
    replan_transaction_id = new_replan_transaction_id("attack")
    phase_timer = PipelinePhaseTimer(
        pipeline="attack_plan",
        replan_transaction_id=replan_transaction_id,
        emit_events=True,
    )
    attack_log: Dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "replanTransactionId": replan_transaction_id,
        "context": _json_safe(
            {
                "reason": ctx.get("reason"),
                "replan_level": ctx.get("replan_level"),
                "mission_ids": ctx.get("mission_ids"),
                "option_names": ctx.get("option_names"),
                "replan_detail": ctx.get("replan_detail"),
            }
        ),
        "steps": [],
        "timingMs": {},
        "logMessages": log_messages,
        "log_text": "",
        "result": {},
    }

    def _emit(message: str) -> None:
        log_messages.append(message)
        attack_log["log_text"] = "\n".join(log_messages)
        if log_callback:
            log_callback(f"[ATTACK] {message}")

    def _record_timing(name: str, started_at: float) -> int:
        elapsed_ms = _elapsed_ms(started_at)
        attack_log.setdefault("timingMs", {})[name] = elapsed_ms
        _emit(f"[TIME] {name}={elapsed_ms}ms")
        return elapsed_ms

    def _finish() -> Dict[str, Any]:
        if "delivery_wait" not in attack_log.setdefault("timingMs", {}):
            phase_timer.mark("delivery_wait")
        phase_timing = phase_timer.snapshot(include_total=False)
        if phase_timing:
            attack_log.setdefault("timingMs", {}).update(phase_timing)
            _emit(f"[TIME] timingMs={json.dumps(_json_safe(phase_timing), ensure_ascii=False)}")
        _record_timing("total", pipeline_started)
        return _persist_attack_log(attack_log)

    def _set_failure(
        code: str,
        notice: str,
        *,
        detail: Optional[Dict[str, Any]] = None,
        override: bool = False,
    ) -> None:
        result = attack_log.setdefault("result", {})
        if not override and str(result.get("failure_notice") or "").strip():
            return
        result["failure_code"] = str(code or "").strip()
        result["failure_notice"] = str(notice or "").strip()
        if detail:
            result["failure_detail"] = _json_safe(detail)

    # Step 1) Select the manned aircraft (aircraft 2 or 3) with the greatest fuel.
    select_started = time.perf_counter()
    input_pkg_id = _to_int(
        ctx.get("inputMissionPackageID")
        or ctx.get("inputMissionPackageId")
        or ctx.get("input_mission_package_id")
    )
    detail_payload = _normalize_replan_detail(ctx.get("replan_detail")) or {}
    allow_previously_used_manned = _detail_requests_follow_up_attack(detail_payload)
    agent_snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    agent_states = agent_snapshot.get("agent_states") or []
    best_aircraft, candidates, manned_select_reason = _select_preferred_manned_aircraft(
        agent_states,
        input_package_id=input_pkg_id if not allow_previously_used_manned else None,
    )
    attack_log["result"]["manned_candidates"] = candidates
    attack_log["result"]["selected_aircraft"] = best_aircraft
    select_elapsed_ms = _record_timing("select_manned_aircraft", select_started)
    if best_aircraft:
        _emit(
            "STEP1 Selected manned aircraft: "
            f"aircraft {best_aircraft['aircraft_id']} "
            f"(fuel={best_aircraft.get('fuel')}, "
            f"coord={_coord_text(best_aircraft.get('coordinate'))})"
        )
        attack_log["steps"].append(
            {
                "name": "select_manned_aircraft",
                "status": "ok",
                "selected": best_aircraft,
                "candidates": candidates,
                "durationMs": select_elapsed_ms,
            }
        )
    else:
        if candidates and input_pkg_id is not None:
            message = (
                "all candidates already used "
                f"(inputMissionPackageID={input_pkg_id})"
            )
            _emit(f"STEP1 Manned-aircraft selection failed: {message}")
        else:
            message = "latest_0401_agent_status.json unavailable"
            _emit(f"STEP1 Manned-aircraft selection failed: {message}")
        attack_log["steps"].append(
            {
                "name": "select_manned_aircraft",
                "status": "error",
                "message": message,
                "candidates": candidates,
                "durationMs": select_elapsed_ms,
            }
        )
        attack_log["result"]["manned_unavailable"] = True
        if manned_select_reason == "all_unarmed":
            _set_failure(
                "attack_weapon_unavailable",
                attack_failure_notice("attack_weapon_unavailable"),
            )
        else:
            _set_failure(
                "manned_unavailable",
                attack_failure_notice("manned_unavailable"),
            )

    # Step 2) Determine which UAVs are currently tracking targets.
    target_started = time.perf_counter()
    target_entries, target_error = _load_target_entries()
    attack_log["result"]["target_tracking"] = target_entries
    target_elapsed_ms = _record_timing("load_target_entries", target_started)
    phase_timer.mark("read_source")
    if target_entries:
        tracking_summary = ", ".join(
            f"watcher {entry.get('watcher_id')} -> target {entry.get('target_id') or entry.get('key')}"
            for entry in target_entries
        )
        _emit(f"STEP2 UAV tracking summary: {tracking_summary or 'no active tracking'}")
        attack_log["steps"].append(
            {
                "name": "analyze_uav_tracking",
                "status": "ok",
                "entries": target_entries,
                "durationMs": target_elapsed_ms,
            }
        )
    else:
        _emit(f"STEP2 UAV tracking info unavailable: {target_error or 'targetInfo.json missing'}")
        attack_log["steps"].append(
            {
                "name": "analyze_uav_tracking",
                "status": "warn",
                "message": target_error or "targetInfo.json missing",
                "durationMs": target_elapsed_ms,
            }
        )

    # Step 3) Attempt to build an attack mission snapshot using lah_attack_assistance.
    friendly_coord = (best_aircraft or {}).get("coordinate")
    detail_override = _build_primary_target_from_detail(detail_payload, target_entries)
    bundle_targets = _build_attack_targets_from_detail(detail_payload, target_entries)
    if bundle_targets:
        selected_targets = [dict(item) for item in bundle_targets[:3]]
        primary_target = dict(selected_targets[0])
        _emit(
            "STEP2.5 Using bundled 0402 targets: "
            + ", ".join(
                f"target={item.get('target_id')} watcher={item.get('watcher_id')}"
                for item in selected_targets
            )
        )
    elif detail_override:
        primary_target = detail_override
        selected_targets = [dict(primary_target)]
        _emit(
            "STEP2.5 Using 0402 target override: "
            f"target={primary_target.get('target_id')} watcher={primary_target.get('watcher_id')}"
        )
    else:
        primary_target = _pick_primary_target(target_entries)
        selected_targets = [dict(primary_target)] if isinstance(primary_target, dict) else []

    if detail_override and selected_targets and not any(
        _same_target_identity(detail_override, item) for item in selected_targets
    ):
        selected_targets.insert(0, dict(detail_override))
    if len(selected_targets) > 3:
        selected_targets = selected_targets[:3]

    attack_log["result"]["primary_target"] = primary_target
    attack_log["result"]["attack_targets"] = selected_targets
    attack_log["result"]["target_count"] = len(selected_targets)

    multi_target_mode = len(selected_targets) > 1
    if multi_target_mode:
        selected_manned_aircraft = _select_manned_aircraft_list(
            agent_states,
            input_package_id=input_pkg_id,
            required_count=min(len(selected_targets), len(_attack_manned_candidates())),
            allow_previously_used=True,
        )
        if selected_manned_aircraft:
            best_aircraft = dict(selected_manned_aircraft[0])
            friendly_coord = best_aircraft.get("coordinate")
            attack_log["result"]["selected_manned_aircraft"] = selected_manned_aircraft
            _emit(
                "STEP2.6 Multi-target manned selection: "
                + ", ".join(str(item.get("aircraft_id")) for item in selected_manned_aircraft)
            )
        else:
            selected_manned_aircraft = [best_aircraft] if best_aircraft else []
    else:
        selected_manned_aircraft = [best_aircraft] if best_aircraft else []

    selected_weapon_choice = _resolve_attack_weapon_choice(primary_target, best_aircraft)
    selected_weapon_type = int(selected_weapon_choice.get("selectedWeaponType", 1))
    attack_log["result"]["selected_weapon_type"] = selected_weapon_type
    attack_log["result"]["weapon_choice"] = selected_weapon_choice
    phase_timer.mark("descriptor_build")
    if not friendly_coord:
        _set_failure(
            "missing_friendly_coordinate",
            attack_failure_notice("missing_friendly_coordinate"),
        )
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "error",
                "message": "Missing friendly coordinate (cannot derive aircraft position)",
            }
        )
        _emit("STEP3 Attack plan failed: manned aircraft coordinate missing")
        return _finish()
    if not primary_target or not primary_target.get("coordinate"):
        _set_failure(
            "missing_target_coordinate",
            attack_failure_notice("missing_target_coordinate"),
        )
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "warn",
                "message": "No active target with coordinates found.",
            }
        )
        _emit("STEP3 Attack plan deferred: no active target with coordinates")
        return _finish()
    if not bool(selected_weapon_choice.get("ammoAvailable")):
        _set_failure(
            "attack_weapon_unavailable",
            attack_failure_notice("attack_weapon_unavailable"),
            detail={"weaponChoice": selected_weapon_choice},
        )
        attack_log["steps"].append(
            {
                "name": "resolve_attack_weapon",
                "status": "error",
                "message": "No ammunition available for any attack weapon slot.",
                "weaponChoice": _json_safe(selected_weapon_choice),
            }
        )
        _emit("STEP2.7 Attack plan failed: no ammunition available for selected aircraft")
        return _finish()
    _emit(
        "[ATTACK] Selected target "
        f"id={primary_target.get('target_id')} type={primary_target.get('target_type')} "
        f"weaponType={selected_weapon_type} "
        f"(preferred={selected_weapon_choice.get('preferredWeaponType')}, ammo={selected_weapon_choice.get('weaponInventory')})"
    )

    attack_point_started = time.perf_counter()
    attack_point_cache_stats: Dict[str, Any] = {}
    attack_point, attack_error = _compute_attack_point(
        friendly_coord,
        primary_target["coordinate"],
        friendly_heading_deg=(best_aircraft or {}).get("heading"),
        friendly_speed_mps=(best_aircraft or {}).get("speed"),
        cache_stats=attack_point_cache_stats,
    )
    attack_log["result"]["attack_point"] = attack_point
    attack_log["result"]["attack_point_cache"] = attack_point_cache_stats
    attack_point_elapsed_ms = _record_timing("compute_attack_point", attack_point_started)
    mission_updates: Optional[Dict[str, Any]] = None
    if attack_point:
        altitude_display = (
            f"alt={attack_point['altitude']}m" if attack_point.get("altitude") is not None else "alt=unknown"
        )
        _emit(
            "STEP3 Attack plan completed: "
            f"lat={attack_point['latitude']:.6f}, lon={attack_point['longitude']:.6f}, "
            f"{altitude_display}, mode={attack_point.get('selection_mode') or 'unknown'}"
        )
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "ok",
                "attack_point": attack_point,
                "durationMs": attack_point_elapsed_ms,
            }
        )
        override_started = time.perf_counter()
        ctx.pop("_attack_failure_code", None)
        ctx.pop("_attack_failure_notice", None)
        ctx["_attack_target_list"] = [dict(item) for item in selected_targets]
        ctx["_selected_manned_aircraft"] = [dict(item) for item in selected_manned_aircraft]
        mission_updates = _apply_attack_plan_overrides(
            ctx=ctx,
            attack_point=attack_point,
            manned_aircraft=best_aircraft,
            primary_target=primary_target,
            agent_states=agent_states,
            waypoint_memory=agent_snapshot.get("last_nonzero_waypoint_by_aircraft"),
            emit=_emit,
        )
        override_elapsed_ms = _record_timing("apply_attack_plan_overrides", override_started)
        phase_timer.mark("write_artifacts")
        if mission_updates:
            mission_updates["timingMs"] = dict(mission_updates.get("timingMs") or {})
            mission_updates["timingMs"]["pipelineOverride"] = override_elapsed_ms
            attack_log["result"]["missionUpdates"] = mission_updates
            manned_ids = _extract_assigned_manned_ids(mission_updates)
            if manned_ids:
                set_last_assigned_manned_id(manned_ids[-1])
                attack_plan_id = _to_int(mission_updates.get("mission_plan_id"))
                if attack_plan_id is not None:
                    set_pending_manned_assignments(attack_plan_id, input_pkg_id, manned_ids)
    else:
        _set_failure(
            "attack_point_failed",
            attack_failure_notice("attack_point_failed"),
            detail={"attack_error": attack_error},
        )
        _emit(f"STEP3 Attack plan failed: {attack_error}")
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "error",
                "message": attack_error,
                "durationMs": attack_point_elapsed_ms,
            }
        )

    if attack_point and not mission_updates:
        failure_notice = str(ctx.get("_attack_failure_notice") or "").strip()
        failure_code = str(ctx.get("_attack_failure_code") or "attack_override_failed").strip()
        if failure_notice:
            _set_failure(
                failure_code,
                failure_notice,
                detail={"weaponChoice": ctx.get("_selected_attack_weapon_choice")},
            )
        else:
            _set_failure(
                "attack_override_failed",
                attack_failure_notice("attack_override_failed"),
            )

    return _finish()


def run_attack_exclusion_pipeline(
    ctx: Dict[str, Any],
    log_callback: Optional[LogCallback] = None,
) -> Dict[str, Any]:
    """
    Build an attack-exclusion plan by trimming each UAV mission to its
    current resume portion, using the same logic as the non-selected UAV
    branch of the prior-mission pipeline.
    """

    log_messages: List[str] = []
    pipeline_started = time.perf_counter()
    replan_transaction_id = new_replan_transaction_id("attack_exclusion")
    phase_timer = PipelinePhaseTimer(
        pipeline="attack_exclusion",
        replan_transaction_id=replan_transaction_id,
        emit_events=True,
    )
    result_payload: Dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "replanTransactionId": replan_transaction_id,
        "context": _json_safe(
            {
                "reason": ctx.get("reason"),
                "replan_level": ctx.get("replan_level"),
                "mission_ids": ctx.get("mission_ids"),
                "option_names": ctx.get("option_names"),
            }
        ),
        "result": {},
        "logMessages": log_messages,
        "timingMs": {},
    }

    def _emit(message: str) -> None:
        log_messages.append(message)
        if log_callback:
            log_callback(f"[ATTACK-EXCLUDE] {message}")

    def _finish_result() -> Dict[str, Any]:
        result_payload["timingMs"] = phase_timer.snapshot(include_total=True)
        result_payload["timingMs"]["total"] = _elapsed_ms(pipeline_started)
        return result_payload

    source_resolve_started = time.perf_counter()
    source_plan_id = _resolve_attack_source_plan_id(
        ctx,
        _normalize_replan_detail(ctx.get("replan_detail")),
    )
    phase_timer.mark("source_plan_resolve")
    if source_plan_id is None:
        _emit("원본 MissionPlan을 찾지 못해 공격 배제 계획을 생성할 수 없습니다.")
        result_payload["result"] = {"error": "source_plan_not_found"}
        return _finish_result()

    source_load_started = time.perf_counter()
    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = read_json_cached(plan_src, kind="MissionPlan")
    except Exception as exc:
        phase_timer.mark("source_plan_load_failed")
        _emit(f"원본 MissionPlan {source_plan_id} 로드 실패: {exc}")
        result_payload["result"] = {
            "error": "source_plan_load_failed",
            "sourcePlanID": int(source_plan_id),
        }
        return _finish_result()
    phase_timer.mark("source_plan_load")

    agent_started = time.perf_counter()
    agent_snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    agent_states = agent_snapshot.get("agent_states") or []
    agent_index = _index_agent_states(
        agent_states,
        waypoint_memory=agent_snapshot.get("last_nonzero_waypoint_by_aircraft"),
    )
    sweep_progress = load_sweep_progress()
    phase_timer.mark("agent_sweep_state_load")

    plan_id_started = time.perf_counter()
    requested_plan_id = _resolve_requested_plan_id(
        ctx,
        preferred_option_names={"공격 배제"},
    )
    new_plan_id = requested_plan_id or _allocate_fresh_plan_id()
    phase_timer.mark("plan_id_resolve")
    if requested_plan_id is not None:
        _emit(
            f"ATTACK-EXCLUDE using requested missionPlanID {new_plan_id} "
            f"(sourcePlanID={source_plan_id})"
        )
    else:
        _emit(
            f"ATTACK-EXCLUDE allocated fresh missionPlanID {new_plan_id} "
            f"(sourcePlanID={source_plan_id})"
        )
    now_ms = _now_timestamp_ms()
    new_plan_data = deepcopy(plan_data)
    new_plan_data["missionPlanID"] = new_plan_id
    new_plan_data["timestamp"] = now_ms
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = now_ms

    resume_index_started = time.perf_counter()
    attack_exclusion_aircraft_ids: List[int] = []
    resume_candidate_plan_ids: List[Any] = [int(source_plan_id)]
    for entry in new_plan_data.get("aircraftList", []):
        aircraft_id = _to_int((entry or {}).get("aircraftID")) if isinstance(entry, dict) else None
        if aircraft_id is None or aircraft_id <= 3:
            continue
        attack_exclusion_aircraft_ids.append(int(aircraft_id))
        progress_state = _load_latest_mission_progress_state(int(aircraft_id)) or {}
        progress_plan_id = _to_int(progress_state.get("currentMissionPlanID"))
        if progress_plan_id is not None:
            resume_candidate_plan_ids.append(int(progress_plan_id))
    resume_candidate_plan_ids.extend(
        [
            _load_latest_mission_progress_plan_id(),
            _scan_latest_source_plan_id(),
        ]
    )
    resume_index = _build_attack_exclusion_resume_index(
        resume_candidate_plan_ids,
        attack_exclusion_aircraft_ids,
    )
    phase_timer.mark("resume_index_build")
    _emit(
        "ATTACK-EXCLUDE resume index built "
        f"(plans={len({int(pid) for pid in resume_candidate_plan_ids if _to_int(pid) is not None})}, "
        f"aircraft={len(attack_exclusion_aircraft_ids)}, "
        f"rows={sum(len(rows or []) for rows in resume_index.values())}, "
        f"elapsedMs={_elapsed_ms(resume_index_started):.3f})"
    )

    aircraft_updates: List[Dict[str, Any]] = []
    unchanged_aircraft: List[int] = []
    deferred_tracking_clear_aircraft_ids: set[int] = set()
    aircraft_loop_started = time.perf_counter()
    for entry in new_plan_data.get("aircraftList", []):
        aircraft_id = _to_int(entry.get("aircraftID"))
        if aircraft_id is None:
            continue
        if aircraft_id <= 3:
            unchanged_aircraft.append(aircraft_id)
            continue

        state = agent_index.get(aircraft_id) or {}
        current_wp = _to_int(state.get("current_waypoint_id"))
        current_coord = state.get("coordinate") if isinstance(state, dict) else None
        aircraft_update_started = time.perf_counter()
        _emit(
            "ATTACK-EXCLUDE aircraft update start "
            f"(aircraft={aircraft_id}, currentWP={current_wp}, sourcePlanHint={source_plan_id})."
        )
        recovery = _resolve_attack_tracking_recovery(
            aircraft_id=int(aircraft_id),
            source_plan_id=int(source_plan_id),
            current_coord=current_coord,
            emit=_emit,
        )
        if recovery is not None:
            update = _build_other_uav_resume_package(
                source_plan_id=int(recovery["source_plan_id"]),
                aircraft_id=int(aircraft_id),
                current_waypoint_id=_to_int(recovery["split_waypoint_id"]),
                current_coord=recovery.get("done_anchor_coord"),
                emit=_emit,
                now_ms=now_ms,
                sweep_progress=sweep_progress,
                clone_follow_up_artifacts=True,
                drop_prefix_missions=True,
                allow_first_mission_fallback=False,
                include_done_reference_mission=False,
            )
            if not update:
                _emit(
                    f"UAV {aircraft_id} tracking recovery resume mission generation failed; "
                    "keeping existing individual mission."
                )
                _emit(
                    "ATTACK-EXCLUDE aircraft update done "
                    f"(aircraft={aircraft_id}, status=recovery_failed, "
                    f"elapsedMs={_elapsed_ms_detail(aircraft_update_started):.3f})."
                )
                unchanged_aircraft.append(aircraft_id)
                continue

            deferred_tracking_clear_aircraft_ids.add(int(aircraft_id))
            _emit(
                f"UAV {aircraft_id} attack tracking clear deferred until "
                f"attack-exclusion plan apply (attackPlan={source_plan_id})."
            )
            entry["individualMissionPackageID"] = int(update["individualMissionPackageID"])
            aircraft_updates.append(update)
            _emit(
                "ATTACK-EXCLUDE aircraft update done "
                f"(aircraft={aircraft_id}, status=recovery_updated, "
                f"elapsedMs={_elapsed_ms_detail(aircraft_update_started):.3f}, "
                f"timingMs={json.dumps(update.get('timingMs') or {}, ensure_ascii=False, default=str)})."
            )
            continue

        resume_source_hint = int(source_plan_id)
        if current_wp is None:
            inferred_plan_id, inferred_wp = _infer_attack_exclusion_resume_state(
                source_plan_id=int(source_plan_id),
                aircraft_id=int(aircraft_id),
                current_coord=current_coord,
                emit=_emit,
                resume_index=resume_index,
            )
            if inferred_wp is not None:
                current_wp = inferred_wp
                if inferred_plan_id is not None:
                    resume_source_hint = int(inferred_plan_id)
        if current_wp is None:
            _emit(f"UAV {aircraft_id} currentWaypointID가 없어 기존 개별임무를 유지합니다.")
            unchanged_aircraft.append(aircraft_id)
            continue

        resume_source_plan_id = _resolve_attack_exclusion_source_plan_id(
            source_plan_id=int(resume_source_hint),
            aircraft_id=int(aircraft_id),
            current_waypoint_id=current_wp,
            emit=_emit,
        )
        if resume_source_plan_id is None:
            _emit(
                f"UAV {aircraft_id} resume source MissionPlan을 찾지 못해 "
                "기존 개별임무를 유지합니다."
            )
            unchanged_aircraft.append(aircraft_id)
            continue

        update = _build_other_uav_resume_package(
            source_plan_id=int(resume_source_plan_id),
            aircraft_id=int(aircraft_id),
            current_waypoint_id=current_wp,
            current_coord=current_coord,
            emit=_emit,
            now_ms=now_ms,
            sweep_progress=sweep_progress,
            clone_follow_up_artifacts=True,
            drop_prefix_missions=True,
            allow_first_mission_fallback=False,
            include_done_reference_mission=False,
        )
        if not update:
            _emit(f"UAV {aircraft_id} resume 임무 생성에 실패하여 기존 개별임무를 유지합니다.")
            unchanged_aircraft.append(aircraft_id)
            continue

        entry["individualMissionPackageID"] = int(update["individualMissionPackageID"])
        aircraft_updates.append(update)
        if _clear_attack_tracking_assignment_if_attached_to_plan(
            aircraft_id=int(aircraft_id),
            source_plan_id=int(source_plan_id),
            emit=_emit,
            mutate_state=False,
        ):
            deferred_tracking_clear_aircraft_ids.add(int(aircraft_id))
        _emit(
            "ATTACK-EXCLUDE aircraft update done "
            f"(aircraft={aircraft_id}, status=resume_updated, "
            f"resumeSourcePlanID={resume_source_plan_id}, currentWP={current_wp}, "
            f"elapsedMs={_elapsed_ms_detail(aircraft_update_started):.3f}, "
            f"timingMs={json.dumps(update.get('timingMs') or {}, ensure_ascii=False, default=str)})."
        )
    phase_timer.mark("aircraft_update_loop")

    no_update_plan = False
    if not aircraft_updates:
        no_update_plan = True
        _emit(
            "공격 배제용 UAV 재개 임무가 생성되지 않아 "
            "기존 개별임무를 유지한 변경 없음 MissionPlan을 저장합니다."
        )

    validation_started = time.perf_counter()
    try:
        validation_summary = validate_replan_payloads(
            mission_plan=new_plan_data,
            scope="attack_exclusion",
            allow_existing_db_artifacts=True,
            log=_emit,
        )
    except ReplanValidationError as exc:
        phase_timer.mark("validation_failed")
        _emit(f"[VALIDATION][ERR] {'; '.join(exc.errors[:4])}")
        result_payload["result"] = {
            "error": "validation_failed",
            "sourcePlanID": int(source_plan_id),
            "missionPlanID": int(new_plan_id),
            "validationErrors": exc.errors[:12],
        }
        return _finish_result()
    result_payload["result"]["validation"] = validation_summary
    phase_timer.mark("validation")

    write_started = time.perf_counter()
    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(plan_dest, new_plan_data)
    _emit(f"공격 배제 MissionPlan 저장 -> {plan_dest.name} (planID={new_plan_id})")
    phase_timer.mark("write_artifacts")

    carried_snapshot = mission_area_replan_store.carry_forward_snapshot(
        int(source_plan_id),
        int(new_plan_id),
        reason="attack_exclusion",
    )
    if carried_snapshot is not None:
        _emit(
            "ATTACK-EXCLUDE carried area remaining snapshot -> "
            f"{carried_snapshot.name} (sourcePlanID={source_plan_id}, planID={new_plan_id})"
        )

    result_payload["result"] = {
        "sourcePlanID": int(source_plan_id),
        "missionPlanID": int(new_plan_id),
        "planPath": str(plan_dest),
        "missionUpdates": {
            "mode": "attack_exclusion",
            "noOp": bool(no_update_plan),
            "aircraft": aircraft_updates,
            "unchangedAircraft": unchanged_aircraft,
            "trackingClearAircraftIDs": sorted(deferred_tracking_clear_aircraft_ids),
        },
    }
    result_payload["result"]["validation"] = validation_summary
    return _finish_result()


def _select_preferred_manned_aircrafts(
    agent_states: List[Any],
    *,
    input_package_id: Optional[int] = None,
    max_count: int = 1,
    allow_used_reuse: bool = False,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], Optional[str]]:
    candidates: List[Dict[str, Any]] = []
    allowed_candidates = set(_attack_manned_candidates())
    for state in agent_states:
        aircraft_id = _to_int(
            (state.get("aircraftID") if isinstance(state, dict) else None)
            or (state.get("aircraftId") if isinstance(state, dict) else None)
        )
        if aircraft_id not in allowed_candidates:
            continue
        is_unmanned = _to_bool(state.get("isUnmanned")) if isinstance(state, dict) else None
        if is_unmanned:
            continue
        fuel = _to_float(state.get("fuel")) if isinstance(state, dict) else None
        coordinate = _normalize_coordinate(state.get("coordinate")) if isinstance(state, dict) else None
        heading = _to_float(state.get("heading")) if isinstance(state, dict) else None
        speed = _to_float(state.get("speed")) if isinstance(state, dict) else None
        weapon_inventory = extract_attack_weapon_inventory(state)
        candidates.append(
            {
                "aircraft_id": aircraft_id,
                "fuel": fuel,
                "coordinate": coordinate,
                "heading": heading,
                "speed": speed,
                "weapon_inventory": weapon_inventory,
            }
        )
    if not candidates:
        return None, candidates, "no_candidates"
    candidates.sort(
        key=lambda item: (
            item["fuel"] is not None,
            item["fuel"] if item["fuel"] is not None else float("-inf"),
        ),
        reverse=True,
    )
    used = get_used_manned_ids(input_package_id)
    if used and not allow_used_reuse:
        unused = [c for c in candidates if c["aircraft_id"] not in used]
        if not unused:
            return [], candidates, "all_used"
        candidates = unused
    armed_candidates = [
        candidate
        for candidate in candidates
        if sum(int((candidate.get("weapon_inventory") or {}).get(key, 0)) for key in ("type1", "type2", "type3")) > 0
    ]
    if armed_candidates:
        candidates = armed_candidates
    else:
        return [], candidates, "all_unarmed"

    last_assigned = get_last_assigned_manned_id()
    if last_assigned is not None:
        preferred = [candidate for candidate in candidates if candidate["aircraft_id"] != last_assigned]
        deferred = [candidate for candidate in candidates if candidate["aircraft_id"] == last_assigned]
        if preferred:
            candidates = preferred + deferred
    return candidates[: max(1, int(max_count))], candidates, None


def _select_preferred_manned_aircraft(
    agent_states: List[Any],
    *,
    input_package_id: Optional[int] = None,
) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]], Optional[str]]:
    selected, candidates, reason = _select_preferred_manned_aircrafts(
        agent_states,
        input_package_id=input_package_id,
        max_count=1,
        allow_used_reuse=False,
    )
    return (selected[0] if selected else None), candidates, reason


def _select_manned_aircraft_list(
    agent_states: List[Any],
    *,
    input_package_id: Optional[int],
    required_count: int,
    allow_previously_used: bool,
) -> List[Dict[str, Any]]:
    selected, candidates, _reason = _select_preferred_manned_aircraft(
        agent_states,
        input_package_id=input_package_id if not allow_previously_used else None,
    )
    if not candidates:
        return []

    required = max(1, int(required_count or 1))
    used = get_used_manned_ids(input_package_id)
    last_assigned = get_last_assigned_manned_id()

    def _sort_key(item: Dict[str, Any]) -> Tuple[int, int, float, int]:
        aircraft_id = _to_int(item.get("aircraft_id")) or 0
        used_rank = 0 if (allow_previously_used and aircraft_id in used) else 1
        selected_rank = 0 if (selected and aircraft_id == _to_int(selected.get("aircraft_id"))) else 1
        last_rank = 0 if (last_assigned is not None and aircraft_id == int(last_assigned)) else 1
        fuel = _to_float(item.get("fuel")) or 0.0
        return (used_rank, selected_rank + last_rank, -float(fuel), aircraft_id)

    ordered = sorted(candidates, key=_sort_key)
    out: List[Dict[str, Any]] = []
    seen: set[int] = set()
    for candidate in ordered:
        aircraft_id = _to_int(candidate.get("aircraft_id"))
        if aircraft_id is None or aircraft_id in seen:
            continue
        if not allow_previously_used and aircraft_id in used:
            continue
        seen.add(int(aircraft_id))
        out.append(dict(candidate))
        if len(out) >= required:
            break
    return out


def _assign_targets_to_manned_aircraft(
    targets: List[Dict[str, Any]],
    manned_aircraft: List[Dict[str, Any]],
) -> Dict[int, List[Dict[str, Any]]]:
    assignments: Dict[int, List[Dict[str, Any]]] = {}
    if not targets or not manned_aircraft:
        return assignments

    remaining_inventory: Dict[int, Dict[str, int]] = {}
    for aircraft in manned_aircraft:
        aircraft_id = _to_int(aircraft.get("aircraft_id"))
        if aircraft_id is None:
            continue
        remaining_inventory[int(aircraft_id)] = extract_attack_weapon_inventory(aircraft)

    next_index = 0
    for target in targets:
        assigned = False
        for offset in range(len(manned_aircraft)):
            aircraft = manned_aircraft[(next_index + offset) % len(manned_aircraft)]
            aircraft_id = _to_int(aircraft.get("aircraft_id"))
            if aircraft_id is None:
                continue
            inventory = remaining_inventory.setdefault(int(aircraft_id), extract_attack_weapon_inventory(aircraft))
            choice = _resolve_attack_weapon_choice(target, {"weapon_inventory": inventory})
            if not bool(choice.get("ammoAvailable")):
                continue

            target_payload = dict(target)
            target_payload["selected_weapon_type"] = int(choice.get("selectedWeaponType", 1))
            target_payload["weapon_choice"] = dict(choice)
            assignments.setdefault(int(aircraft_id), []).append(target_payload)

            selected_weapon_type = _to_int(choice.get("selectedWeaponType"))
            slot_map = {1: "type1", 2: "type2", 3: "type3"}
            slot_name = slot_map.get(int(selected_weapon_type or 0))
            if slot_name:
                inventory[slot_name] = max(0, int(inventory.get(slot_name, 0)) - 1)

            next_index = (next_index + offset + 1) % len(manned_aircraft)
            assigned = True
            break
        if not assigned:
            continue
    return assignments


def _assign_targets_to_tracking_uavs(
    targets: List[Dict[str, Any]],
    plan_uav_ids: List[int],
    agent_index: Dict[int, Dict[str, Any]],
) -> Dict[int, Dict[str, Any]]:
    assignments: Dict[int, Dict[str, Any]] = {}
    if not targets or not plan_uav_ids:
        return assignments

    tracking_by_target: Dict[int, int] = {}
    active_tracking_by_aircraft: Dict[int, int] = {}
    for entry in list_active_tracking_assignments():
        if not isinstance(entry, dict) or not bool(entry.get("active")):
            continue
        aircraft_id = _to_int(entry.get("aircraft_id"))
        target_id = _to_int(entry.get("target_id"))
        if aircraft_id is None or target_id is None or aircraft_id not in plan_uav_ids:
            continue
        tracking_by_target[int(target_id)] = int(aircraft_id)
        active_tracking_by_aircraft[int(aircraft_id)] = int(target_id)

    remaining_uavs = [int(aid) for aid in plan_uav_ids if _to_int(aid) is not None]
    used_uavs: set[int] = set()

    def _choose_nearest_available(target: Dict[str, Any]) -> Optional[int]:
        target_coord = _normalize_coordinate(target.get("coordinate"))
        if target_coord is None:
            return None
        best_aircraft_id: Optional[int] = None
        best_distance: Optional[float] = None
        for aircraft_id in remaining_uavs:
            if aircraft_id in used_uavs:
                continue
            target_id = _to_int(target.get("target_id") or target.get("targetID"))
            if not _tracking_aircraft_available_for_target(
                active_tracking_by_aircraft,
                int(aircraft_id),
                target_id,
            ):
                continue
            state = agent_index.get(int(aircraft_id)) or {}
            coord = _normalize_coordinate(state.get("coordinate"))
            distance = _haversine_distance_m(coord, target_coord) if coord and target_coord else None
            if distance is None:
                distance = float("inf")
            if best_distance is None or distance < best_distance:
                best_distance = float(distance)
                best_aircraft_id = int(aircraft_id)
        return best_aircraft_id

    for target in targets:
        target_id = _to_int(target.get("target_id") or target.get("targetID"))
        watcher_id = _to_int(target.get("watcher_id") or target.get("watcherID"))

        chosen_aircraft_id: Optional[int] = None
        for candidate in (
            tracking_by_target.get(int(target_id)) if target_id is not None else None,
            watcher_id,
        ):
            if candidate is None:
                continue
            if int(candidate) not in remaining_uavs or int(candidate) in used_uavs:
                continue
            if not _tracking_aircraft_available_for_target(
                active_tracking_by_aircraft,
                int(candidate),
                target_id,
            ):
                continue
            chosen_aircraft_id = int(candidate)
            break

        if chosen_aircraft_id is None:
            chosen_aircraft_id = _choose_nearest_available(target)
        if chosen_aircraft_id is None:
            continue

        assignments[int(chosen_aircraft_id)] = dict(target)
        used_uavs.add(int(chosen_aircraft_id))

    return assignments


def _load_target_entries() -> Tuple[List[Dict[str, Any]], Optional[str]]:
    target_entries: List[Dict[str, Any]] = []
    target_path = db_paths.get_db_subpath("DSS_Internal") / "targetInfo.json"
    if not target_path.exists():
        return [], f"{target_path} does not exist"
    try:
        raw = json.loads(target_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return [], f"Failed to parse targetInfo.json: {exc}"
    target_map = raw.get("targetList") if isinstance(raw, dict) else None
    if not isinstance(target_map, dict):
        return [], "targetInfo.json lacks targetList"
    for key, value in target_map.items():
        entry = value or {}
        target_id = _to_int(entry.get("targetID"))
        watcher_id = _to_int(entry.get("watcherID"))
        if watcher_id is None:
            watcher_field = entry.get("watcher")
            if isinstance(watcher_field, dict):
                watcher_id = _to_int(
                    watcher_field.get("aircraftID")
                    or watcher_field.get("watcherID")
                    or watcher_field.get("id")
                )
            else:
                watcher_id = _to_int(watcher_field)
        if watcher_id is None:
            watcher_id = _extract_watcher_from_key(str(key))
        is_destroyed = bool(entry.get("isDestroyed"))
        target_entries.append(
            {
                "key": str(key),
                "target_id": target_id,
                "target_type": _to_int(entry.get("targetType")),
                "watcher_id": watcher_id,
                "coordinate": _normalize_coordinate(entry.get("coordinate")),
                "is_destroyed": is_destroyed,
                "is_used": _to_int(entry.get("isUsed")),
                "target_in_frame": bool(entry.get("targetInFrame")),
                "threat": _to_float(entry.get("threat")),
                "first_detected": _to_int(entry.get("firstDetected")),
                "last_updated": _to_int(entry.get("lastUpdated")),
                "raw": entry,
            }
        )
    target_entries.sort(
        key=lambda item: (
            1 if item["is_destroyed"] else 0,
            0 if item["target_in_frame"] else 1,
            item["target_id"] if item["target_id"] is not None else 10**9,
            _tracking_entry_preference_key(item),
        ),
    )
    return target_entries, None


def _tracking_entry_preference_key(entry: Dict[str, Any]) -> Tuple[int, int, int, int, int]:
    watcher_id = _to_int(entry.get("watcher_id"))
    first_detected = _to_int(entry.get("first_detected"))
    last_updated = _to_int(entry.get("last_updated")) or first_detected
    return (
        0 if bool(entry.get("target_in_frame")) else 1,
        0 if watcher_id is not None else 1,
        int(first_detected) if first_detected is not None else 10**18,
        -(int(last_updated) if last_updated is not None else -1),
        int(watcher_id) if watcher_id is not None else 10**9,
    )


def _select_preferred_tracking_entry(
    target_entries: List[Dict[str, Any]],
    *,
    target_id: int | None = None,
    target_key: str | None = None,
) -> Optional[Dict[str, Any]]:
    normalized_key = str(target_key or "").strip()
    candidates: List[Dict[str, Any]] = []
    for entry in target_entries:
        if not isinstance(entry, dict):
            continue
        entry_target_id = _to_int(entry.get("target_id"))
        entry_key = str(entry.get("key") or "").strip()
        matches_id = target_id is not None and entry_target_id == int(target_id)
        matches_key = bool(normalized_key and entry_key == normalized_key)
        if not matches_id and not matches_key:
            continue
        candidates.append(entry)
    if not candidates:
        return None
    preferred = min(
        candidates,
        key=lambda item: (
            1 if bool(item.get("is_destroyed")) else 0,
            0 if bool(item.get("target_in_frame")) else 1,
            _tracking_entry_preference_key(item),
        ),
    )
    return dict(preferred)


def _normalize_replan_detail(detail: Any) -> Optional[Dict[str, Any]]:
    if isinstance(detail, dict):
        return detail
    if isinstance(detail, (bytes, bytearray)):
        try:
            detail = detail.decode("utf-8", "ignore")
        except Exception:
            return None
    if isinstance(detail, str):
        try:
            parsed = json.loads(detail)
        except Exception:
            return None
        return parsed if isinstance(parsed, dict) else None
    return None


def _build_primary_target_from_detail(
    detail: Any,
    target_entries: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    detail = _normalize_replan_detail(detail)
    if not detail:
        return None
    trigger = detail.get("trigger")
    has_watcher = detail.get("watcherID") is not None or detail.get("watcherId") is not None
    has_target = detail.get("targetID") is not None or detail.get("targetId") is not None
    has_coord = detail.get("coordinate") is not None or detail.get("targetCoordinate") is not None
    if not (isinstance(trigger, str) and trigger.strip() == "0402") and not (has_watcher and (has_target or has_coord)):
        return None
    target_id = _to_int(detail.get("targetID") or detail.get("targetId"))
    target_key = str(detail.get("targetKey") or detail.get("key") or "").strip()
    target_type = _to_int(detail.get("targetType"))
    watcher_id = _to_int(detail.get("watcherID") or detail.get("watcherId"))
    coord = _normalize_coordinate(detail.get("coordinate") or detail.get("targetCoordinate"))
    if coord is None and (target_id is not None or target_key):
        preferred_entry = _select_preferred_tracking_entry(
            target_entries,
            target_id=target_id,
            target_key=target_key,
        )
        if preferred_entry is not None:
            if preferred_entry.get("is_destroyed"):
                return None
            coord = preferred_entry.get("coordinate")
            if target_type is None:
                target_type = _to_int(preferred_entry.get("target_type"))
            if watcher_id is None:
                watcher_id = _to_int(preferred_entry.get("watcher_id"))
    if target_id is None and coord is None and watcher_id is None:
        return None
    return {
        "key": target_key or detail.get("targetID"),
        "target_id": target_id,
        "target_type": target_type,
        "watcher_id": watcher_id,
        "coordinate": coord,
        "is_destroyed": False,
        "is_used": 0,
        "target_in_frame": True,
        "raw": detail,
    }


def _same_target_identity(left: Dict[str, Any], right: Dict[str, Any]) -> bool:
    left_id = _to_int(left.get("target_id") or left.get("targetID"))
    right_id = _to_int(right.get("target_id") or right.get("targetID"))
    if left_id is not None and right_id is not None:
        return int(left_id) == int(right_id)
    left_key = str(left.get("key") or left.get("targetKey") or "").strip()
    right_key = str(right.get("key") or right.get("targetKey") or "").strip()
    return bool(left_key and right_key and left_key == right_key)


def _build_attack_targets_from_detail(
    detail: Any,
    target_entries: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    detail = _normalize_replan_detail(detail)
    if not detail:
        return []

    raw_items = (
        detail.get("targetBundle")
        or detail.get("attackTargetList")
        or detail.get("targetList")
        or detail.get("targets")
        or []
    )
    if not isinstance(raw_items, list):
        return []

    indexed_entries: Dict[int, Dict[str, Any]] = {}
    keyed_entries: Dict[str, Dict[str, Any]] = {}
    for entry in target_entries:
        if not isinstance(entry, dict):
            continue
        target_id = _to_int(entry.get("target_id"))
        if target_id is not None:
            current = indexed_entries.get(int(target_id))
            if current is None or _tracking_entry_preference_key(entry) < _tracking_entry_preference_key(current):
                indexed_entries[int(target_id)] = dict(entry)
        key = str(entry.get("key") or "").strip()
        if key:
            current = keyed_entries.get(key)
            if current is None or _tracking_entry_preference_key(entry) < _tracking_entry_preference_key(current):
                keyed_entries[key] = dict(entry)

    out: List[Dict[str, Any]] = []
    for raw in raw_items:
        if not isinstance(raw, dict):
            continue
        target_id = _to_int(raw.get("targetID") or raw.get("targetId"))
        key = str(raw.get("targetKey") or raw.get("key") or "").strip()
        base = {}
        if target_id is not None and target_id in indexed_entries:
            base = dict(indexed_entries[int(target_id)])
        elif key and key in keyed_entries:
            base = dict(keyed_entries[key])

        coordinate = _normalize_coordinate(raw.get("coordinate") or raw.get("targetCoordinate"))
        if coordinate is None:
            coordinate = _normalize_coordinate(base.get("coordinate"))
        if coordinate is None:
            continue

        normalized = dict(base)
        normalized.update(
            {
                "key": key or base.get("key") or (str(target_id) if target_id is not None else ""),
                "target_id": target_id if target_id is not None else _to_int(base.get("target_id")),
                "target_type": _to_int(raw.get("targetType") if raw.get("targetType") is not None else base.get("target_type")),
                "watcher_id": _to_int(
                    raw.get("watcherID")
                    or raw.get("watcherId")
                    or base.get("watcher_id")
                ),
                "coordinate": coordinate,
                "is_destroyed": bool(raw.get("isDestroyed") if raw.get("isDestroyed") is not None else base.get("is_destroyed")),
                "is_used": _to_int(raw.get("isUsed") if raw.get("isUsed") is not None else base.get("is_used")) or 0,
                "target_in_frame": bool(
                    raw.get("targetInFrame") if raw.get("targetInFrame") is not None else base.get("target_in_frame")
                ),
                "threat": _to_float(raw.get("threat") if raw.get("threat") is not None else base.get("threat")),
                "raw": dict(raw),
            }
        )
        if normalized.get("target_id") is None:
            continue
        if normalized.get("is_destroyed"):
            continue
        if normalized.get("coordinate") is None:
            continue
        out.append(normalized)

    deduped: List[Dict[str, Any]] = []
    for item in out:
        if any(_same_target_identity(item, existing) for existing in deduped):
            continue
        deduped.append(item)
    return deduped


def _resolve_attack_source_plan_id(
    ctx: Dict[str, Any],
    detail: Dict[str, Any] | None = None,
) -> Optional[int]:
    detail = detail if isinstance(detail, dict) else {}
    for value in (
        ctx.get("sourceMissionPlanID"),
        ctx.get("currentMissionPlanID"),
        detail.get("sourceMissionPlanID"),
        detail.get("currentMissionPlanID"),
        ctx.get("source_plan_id"),
        ctx.get("missionPlanID"),
        _load_latest_mission_progress_plan_id(),
        _scan_latest_source_plan_id(),
    ):
        plan_id = _to_int(value)
        if plan_id is not None and plan_id > 0:
            return int(plan_id)
    return None


def _detail_requests_follow_up_attack(detail: Dict[str, Any] | None) -> bool:
    if not isinstance(detail, dict):
        return False
    if bool(detail.get("followUpAttackMode")):
        return True
    bundle_mode = str(detail.get("targetBundleMode") or "").strip().lower()
    bundle_raw = detail.get("targetBundle")
    if bundle_mode == "follow_up" and isinstance(bundle_raw, list) and len(bundle_raw) > 1:
        return True
    return False


def _pick_primary_target(target_entries: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    collapsed: Dict[str, Dict[str, Any]] = {}
    for entry in target_entries:
        if not isinstance(entry, dict):
            continue
        if bool(entry.get("is_destroyed")) or not entry.get("coordinate"):
            continue
        target_id = _to_int(entry.get("target_id"))
        key = str(entry.get("key") or "").strip()
        identity = f"id:{target_id}" if target_id is not None else f"key:{key}"
        current = collapsed.get(identity)
        if current is None or _tracking_entry_preference_key(entry) < _tracking_entry_preference_key(current):
            collapsed[identity] = dict(entry)
    candidates = list(collapsed.values())
    if not candidates:
        return None

    priority_order = get_runtime_attack_target_type_priority()
    priority_rank = {int(target_type): idx for idx, target_type in enumerate(priority_order)}

    def _score(entry: Dict[str, Any]) -> Tuple[int, int, float, int]:
        target_type = _to_int(entry.get("target_type"))
        threat = _to_float(entry.get("threat")) or 0.0
        target_id = _to_int(entry.get("target_id")) or 0
        return (
            0 if bool(entry.get("target_in_frame")) else 1,
            priority_rank.get(int(target_type), len(priority_order) + 1) if target_type is not None else len(priority_order) + 1,
            -float(threat),
            -int(target_id),
        )

    return min(candidates, key=_score)


def _normalize_attack_batch_entry(
    raw: Any,
    target_entries: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    if not isinstance(raw, dict):
        return None
    target_id = _to_int(raw.get("targetID") or raw.get("targetId") or raw.get("target_id"))
    target_key = raw.get("targetKey") or raw.get("key")
    target_type = _to_int(raw.get("targetType") or raw.get("target_type"))
    watcher_id = _to_int(raw.get("watcherID") or raw.get("watcherId") or raw.get("watcher_id"))
    coord = _normalize_coordinate(raw.get("coordinate") or raw.get("targetCoordinate"))
    is_destroyed = bool(raw.get("isDestroyed") or raw.get("is_destroyed"))
    is_ignored = _to_int(raw.get("isIgnored") or raw.get("is_ignored")) or 0
    is_used = _to_int(raw.get("isUsed") or raw.get("is_used")) or 0
    target_in_frame = bool(raw.get("targetInFrame") or raw.get("target_in_frame"))
    threat = _to_float(raw.get("threat")) or 0.0

    if coord is None and (target_id is not None or target_key is not None):
        preferred_entry = _select_preferred_tracking_entry(
            target_entries,
            target_id=target_id,
            target_key=str(target_key or ""),
        )
        if preferred_entry is not None:
            coord = preferred_entry.get("coordinate")
            target_id = target_id if target_id is not None else _to_int(preferred_entry.get("target_id"))
            target_type = target_type if target_type is not None else _to_int(preferred_entry.get("target_type"))
            watcher_id = watcher_id if watcher_id is not None else _to_int(preferred_entry.get("watcher_id"))
            is_destroyed = bool(preferred_entry.get("is_destroyed"))
            is_used = _to_int(preferred_entry.get("is_used")) or is_used
            target_in_frame = bool(preferred_entry.get("target_in_frame"))
            threat = _to_float(preferred_entry.get("threat")) or threat
            target_key = target_key or preferred_entry.get("key")

    if coord is None or is_destroyed or is_ignored != 0:
        return None

    return {
        "key": str(target_key or target_id or ""),
        "target_id": target_id,
        "target_type": target_type,
        "watcher_id": watcher_id,
        "coordinate": coord,
        "is_destroyed": False,
        "is_used": int(is_used),
        "target_in_frame": bool(target_in_frame),
        "threat": float(threat),
        "selection_order": _to_int(raw.get("selectionOrder")),
        "raw": dict(raw),
    }


def _extract_attack_target_batch_from_detail(
    detail: Any,
    target_entries: List[Dict[str, Any]],
    *,
    limit: int = 3,
) -> list[Dict[str, Any]]:
    detail = _normalize_replan_detail(detail)
    if not detail:
        return []
    raw_items = (
        detail.get("attackTargetList")
        or detail.get("targetBundle")
        or detail.get("targetList")
        or detail.get("targets")
        or []
    )
    if not isinstance(raw_items, list):
        return []

    batch: list[Dict[str, Any]] = []
    seen_keys: set[str] = set()
    ordered_items = sorted(
        [item for item in raw_items if isinstance(item, dict)],
        key=lambda item: (
            _to_int(item.get("selectionOrder")) if _to_int(item.get("selectionOrder")) is not None else 10**9,
            0 if (_to_int(item.get("isUsed")) or 0) != 0 else 1,
            _to_int(item.get("targetID") or item.get("targetId") or item.get("target_id")) or 0,
        ),
    )
    for item in ordered_items:
        normalized = _normalize_attack_batch_entry(item, target_entries)
        if not isinstance(normalized, dict):
            continue
        dedupe_key = str(normalized.get("target_id") or normalized.get("key") or "")
        if not dedupe_key or dedupe_key in seen_keys:
            continue
        seen_keys.add(dedupe_key)
        batch.append(normalized)
        if len(batch) >= max(1, int(limit)):
            break
    return batch


def _resolve_attack_target_batch(
    ctx: Dict[str, Any],
    target_entries: List[Dict[str, Any]],
    *,
    limit: int = 3,
) -> list[Dict[str, Any]]:
    detail = ctx.get("replan_detail")
    batch = _extract_attack_target_batch_from_detail(detail, target_entries, limit=limit)
    if batch:
        return batch
    detail_override = _build_primary_target_from_detail(detail, target_entries)
    if detail_override:
        return [detail_override]
    primary_target = _pick_primary_target(target_entries)
    return [primary_target] if primary_target else []


def _resolve_attack_weapon_choice(
    primary_target: Optional[Dict[str, Any]],
    aircraft_state: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    target_type = _to_int(primary_target.get("target_type")) if isinstance(primary_target, dict) else None
    preferred_weapon_type = get_runtime_attack_weapon_type_for_target_type(
        target_type,
        default=get_runtime_attack_weapon_type(2),
    )
    weapon_inventory = extract_attack_weapon_inventory(aircraft_state or {})
    choice = choose_attack_weapon_type(preferred_weapon_type, weapon_inventory)
    choice["targetType"] = target_type
    return choice


def _resolve_attack_weapon_type(
    primary_target: Optional[Dict[str, Any]],
    aircraft_state: Optional[Dict[str, Any]] = None,
) -> int:
    return int(_resolve_attack_weapon_choice(primary_target, aircraft_state).get("selectedWeaponType", 1))


def _resolve_attack_weapon_choice_for_inventory(
    target: Optional[Dict[str, Any]],
    weapon_inventory: Optional[Dict[str, int]],
) -> Dict[str, Any]:
    target_type = _to_int(target.get("target_type")) if isinstance(target, dict) else None
    preferred_weapon_type = get_runtime_attack_weapon_type_for_target_type(
        target_type,
        default=get_runtime_attack_weapon_type(2),
    )
    choice = choose_attack_weapon_type(preferred_weapon_type, weapon_inventory or {})
    choice["targetType"] = target_type
    return choice


def _active_tracking_assignments_by_target() -> Dict[int, int]:
    mapping: Dict[int, int] = {}
    for entry in list_active_tracking_assignments():
        target_id = _to_int(entry.get("target_id"))
        aircraft_id = _to_int(entry.get("aircraft_id"))
        if target_id is None or aircraft_id is None:
            continue
        mapping[int(target_id)] = int(aircraft_id)
    return mapping


def _active_tracking_assignments_by_aircraft() -> Dict[int, int]:
    mapping: Dict[int, int] = {}
    for entry in list_active_tracking_assignments():
        target_id = _to_int(entry.get("target_id"))
        aircraft_id = _to_int(entry.get("aircraft_id"))
        if target_id is None or aircraft_id is None:
            continue
        mapping[int(aircraft_id)] = int(target_id)
    return mapping


def _tracking_aircraft_available_for_target(
    active_tracking_by_aircraft: Dict[int, int],
    aircraft_id: int,
    target_id: Optional[int],
) -> bool:
    active_target_id = active_tracking_by_aircraft.get(int(aircraft_id))
    if active_target_id is None:
        return True
    requested_target_id = _to_int(target_id)
    return requested_target_id is not None and int(active_target_id) == int(requested_target_id)


def _choose_best_available_watcher(
    available_ids: List[int],
    used_ids: set[int],
    *,
    preferred_ids: List[int],
    target_coord: Optional[Dict[str, Any]],
    agent_index: Dict[int, Dict[str, Any]],
    active_tracking_by_aircraft: Optional[Dict[int, int]] = None,
    target_id: Optional[int] = None,
) -> Optional[int]:
    active_by_aircraft = active_tracking_by_aircraft or {}
    for aircraft_id in preferred_ids:
        if (
            aircraft_id in available_ids
            and aircraft_id not in used_ids
            and _tracking_aircraft_available_for_target(active_by_aircraft, int(aircraft_id), target_id)
        ):
            return int(aircraft_id)
    candidates = [
        aircraft_id
        for aircraft_id in available_ids
        if aircraft_id not in used_ids
        and _tracking_aircraft_available_for_target(active_by_aircraft, int(aircraft_id), target_id)
    ]
    if not candidates:
        return None
    target_coord_norm = _normalize_coordinate(target_coord)
    if target_coord_norm is None:
        return int(candidates[0])
    best_aircraft_id: Optional[int] = None
    best_score: Optional[Tuple[float, int]] = None
    for aircraft_id in candidates:
        state = agent_index.get(int(aircraft_id)) or {}
        coord = _normalize_coordinate(state.get("coordinate"))
        distance_m = _haversine_distance_m(coord, target_coord_norm) if coord else None
        score = (
            float(distance_m) if isinstance(distance_m, (int, float)) else 1.0e12,
            int(aircraft_id),
        )
        if best_score is None or score < best_score:
            best_score = score
            best_aircraft_id = int(aircraft_id)
    return best_aircraft_id


def _assign_targets_to_uav_watchers(
    target_batch: List[Dict[str, Any]],
    *,
    plan_data: Dict[str, Any],
    agent_index: Dict[int, Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], set[int]]:
    available_ids: List[int] = []
    for entry in plan_data.get("aircraftList", []):
        aircraft_id = _to_int((entry or {}).get("aircraftID"))
        if aircraft_id is None or aircraft_id <= 3:
            continue
        if aircraft_id in available_ids:
            continue
        available_ids.append(int(aircraft_id))

    tracking_by_target = _active_tracking_assignments_by_target()
    active_tracking_by_aircraft = _active_tracking_assignments_by_aircraft()
    used_ids: set[int] = set()
    assigned_batch: List[Dict[str, Any]] = []
    for target in target_batch:
        target_copy = dict(target)
        preferred_ids: List[int] = []
        target_id = _to_int(target_copy.get("target_id") or target_copy.get("targetID"))
        if target_id is not None:
            tracked_aircraft = tracking_by_target.get(int(target_id))
            if tracked_aircraft is not None:
                preferred_ids.append(int(tracked_aircraft))
        watcher_id = _to_int(target_copy.get("watcher_id"))
        if watcher_id is not None and watcher_id not in preferred_ids:
            preferred_ids.append(int(watcher_id))
        selected_watcher = _choose_best_available_watcher(
            available_ids,
            used_ids,
            preferred_ids=preferred_ids,
            target_coord=target_copy.get("coordinate"),
            agent_index=agent_index,
            active_tracking_by_aircraft=active_tracking_by_aircraft,
            target_id=target_id,
        )
        if selected_watcher is None:
            continue
        target_copy["watcher_id"] = int(selected_watcher)
        used_ids.add(int(selected_watcher))
        assigned_batch.append(target_copy)
    return assigned_batch, used_ids


def _assign_targets_to_manned_sequences(
    target_batch: List[Dict[str, Any]],
    manned_aircrafts: List[Dict[str, Any]],
) -> Tuple[Dict[int, List[Dict[str, Any]]], Optional[str]]:
    if not target_batch or not manned_aircrafts:
        return {}, "no_manned_candidates"
    inventories: Dict[int, Dict[str, int]] = {}
    sequences: Dict[int, List[Dict[str, Any]]] = {}
    ordered_ids: List[int] = []
    candidate_by_id: Dict[int, Dict[str, Any]] = {}
    assignment_counts: Dict[int, int] = {}
    last_attack_coord_by_id: Dict[int, Dict[str, Any]] = {}
    for candidate in manned_aircrafts:
        aircraft_id = _to_int(candidate.get("aircraft_id"))
        if aircraft_id is None:
            continue
        ordered_ids.append(int(aircraft_id))
        candidate_by_id[int(aircraft_id)] = dict(candidate)
        inventories[int(aircraft_id)] = extract_attack_weapon_inventory(candidate)
        sequences[int(aircraft_id)] = []
        assignment_counts[int(aircraft_id)] = 0
    if not ordered_ids:
        return {}, "no_manned_candidates"

    for idx, target in enumerate(target_batch):
        target_coord = _normalize_coordinate(target.get("coordinate") or target.get("attack_coord"))
        scored_candidates: List[Tuple[Tuple[float, int, int], int, Dict[str, Any]]] = []
        zero_load_available = False

        for order_index, aircraft_id in enumerate(ordered_ids):
            inventory = inventories.get(int(aircraft_id))
            choice = _resolve_attack_weapon_choice_for_inventory(target, inventory)
            if not bool(choice.get("ammoAvailable")):
                continue

            load_count = int(assignment_counts.get(int(aircraft_id), 0))
            if load_count == 0:
                zero_load_available = True

            origin_coord = _normalize_coordinate(last_attack_coord_by_id.get(int(aircraft_id)))
            if origin_coord is None:
                origin_coord = _normalize_coordinate((candidate_by_id.get(int(aircraft_id)) or {}).get("coordinate"))
            distance_m = _haversine_distance_m(origin_coord, target_coord) if origin_coord and target_coord else None
            if not isinstance(distance_m, (int, float)):
                distance_m = 1.0e12

            score = (
                float(distance_m),
                int(load_count),
                int(order_index),
            )
            scored_candidates.append((score, int(aircraft_id), dict(choice)))

        if not scored_candidates:
            target_id = _to_int(target.get("target_id"))
            return {}, f"no_weapon_available_for_target_{target_id or idx + 1}"

        if zero_load_available:
            unassigned_only = [
                item
                for item in scored_candidates
                if int(assignment_counts.get(item[1], 0)) == 0
            ]
            if unassigned_only:
                scored_candidates = unassigned_only

        scored_candidates.sort(key=lambda item: item[0])
        _score, assigned_aircraft_id, assigned_choice = scored_candidates[0]
        slot_key = f"type{int(assigned_choice.get('selectedWeaponType', 1))}"
        inventories[int(assigned_aircraft_id)][slot_key] = max(
            0,
            int(inventories[int(assigned_aircraft_id)].get(slot_key, 0)) - 1,
        )

        target_copy = dict(target)
        target_copy["assigned_manned_aircraft_id"] = int(assigned_aircraft_id)
        target_copy["weapon_choice"] = dict(assigned_choice)
        target_copy["selected_weapon_type"] = int(assigned_choice.get("selectedWeaponType", 1))
        target_copy["weapon_type"] = int(assigned_choice.get("selectedWeaponType", 1))
        sequences.setdefault(int(assigned_aircraft_id), []).append(target_copy)
        assignment_counts[int(assigned_aircraft_id)] = int(assignment_counts.get(int(assigned_aircraft_id), 0)) + 1
        assigned_attack_coord = _normalize_coordinate(
            target_copy.get("attack_coord") or target_copy.get("coordinate")
        )
        if assigned_attack_coord is not None:
            last_attack_coord_by_id[int(assigned_aircraft_id)] = dict(assigned_attack_coord)
    return sequences, None


_DEFAULT_SPEED_MPS = 40.0
def _interpolate_coordinate(
    start_coord: Dict[str, Any],
    end_coord: Dict[str, Any],
    ratio: float,
) -> Dict[str, Any]:
    clamped = max(0.0, min(1.0, float(ratio)))
    start_alt = _normalize_altitude_value(start_coord.get("altitude"))
    end_alt = _normalize_altitude_value(end_coord.get("altitude"))
    if start_alt is None:
        start_alt = end_alt
    if end_alt is None:
        end_alt = start_alt
    altitude = None
    if start_alt is not None and end_alt is not None:
        altitude = _normalize_altitude_value(start_alt + (end_alt - start_alt) * clamped)
    return {
        "latitude": float(start_coord["latitude"]) + (float(end_coord["latitude"]) - float(start_coord["latitude"])) * clamped,
        "longitude": float(start_coord["longitude"]) + (float(end_coord["longitude"]) - float(start_coord["longitude"])) * clamped,
        "altitude": altitude,
    }


def _build_attack_collab_agent_state_map(
    agent_index: Dict[int, Dict[str, Any]],
    *,
    source_plan_id: Optional[int],
    source_artifact_cache: Dict[str, Any],
    emit: Callable[[str], None],
) -> Dict[int, Dict[str, Any]]:
    state_map: Dict[int, Dict[str, Any]] = {}
    current_count = 0
    for aid, state in agent_index.items():
        if aid is None:
            continue
        coord = _normalize_coordinate((state or {}).get("coordinate")) if isinstance(state, dict) else None
        heading = _to_float((state or {}).get("heading")) if isinstance(state, dict) else None
        speed = _to_float((state or {}).get("speed")) if isinstance(state, dict) else None
        if coord is not None:
            current_count += 1
        state_map[int(aid)] = {
            "coordinate": dict(coord or {}),
            "heading": heading,
            "speed": speed,
        }
    emit(
        "[ATTACK][COLLAB] Remaining UAV entry uses current coordinates "
        f"(count={current_count})."
    )
    return state_map


def _boost_attack_collab_first_sweep_search_speed(
    aircraft_id: int,
    path_id: int,
    payload: Dict[str, Any],
    *,
    emit: LogCallback,
    speed_scale: float | None = None,
    reference_coord: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        return payload
    waypoints = payload.get("waypointList")
    if not isinstance(waypoints, list) or not waypoints:
        return payload

    if speed_scale is None:
        scale = get_runtime_float(
            "replan_sweep_speed_scale",
            1.3,
        )
    else:
        try:
            scale = float(speed_scale)
        except Exception:
            scale = 1.0
    if scale <= 0.0 or abs(scale - 1.0) <= 1e-9:
        return payload

    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty")
        if not isinstance(filming, dict):
            continue
        line_search = filming.get("lineSearch")
        if not isinstance(line_search, dict):
            continue
        search_speed = _to_float(line_search.get("searchSpeed"))
        if search_speed is None or search_speed <= 0.0:
            continue
        reference_speed, reference_distance_m = _estimate_attack_collab_first_sweep_search_speed_from_reference(
            waypoint,
            reference_coord,
        )
        base_speed = float(search_speed)
        used_reference_base = False
        if reference_speed is not None and float(reference_speed) > base_speed:
            base_speed = float(reference_speed)
            used_reference_base = True
        cruise_speed_mps = max(
            float(_to_float(waypoint.get("speed")) or _DEFAULT_SPEED_MPS),
            float(base_speed),
        )
        boosted_speed = round(
            clamp_line_search_speed_mps(
                base_speed * float(scale),
                cruise_speed_mps=float(cruise_speed_mps),
                speed_scale=float(scale),
                minimum_speed_mps=float(search_speed),
            ),
            2,
        )
        line_search["searchSpeed"] = float(boosted_speed)
        filming["lineSearch"] = line_search
        waypoint["filmingProperty"] = filming
        payload["waypointList"] = waypoints
        if "lahWaypointList" in payload:
            payload["lahWaypointList"] = deepcopy(waypoints)
        try:
            annotate_eta_flight_plan(payload, default_speed_mps=_DEFAULT_SPEED_MPS, waypoint_list_keys=("waypointList",))
        except Exception:
            pass
        if used_reference_base:
            emit(
                "[ATTACK][COLLAB] First sweep searchSpeed boosted "
                f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
                f"waypointID={_to_int(waypoint.get('waypointID'))}, "
                f"factor={scale:.2f}, old={float(search_speed):.2f}, "
                f"refBase={base_speed:.2f}, refDist={float(reference_distance_m or 0.0):.1f}m, "
                f"new={float(boosted_speed):.2f})."
            )
        else:
            emit(
                "[ATTACK][COLLAB] First sweep searchSpeed boosted "
                f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
                f"waypointID={_to_int(waypoint.get('waypointID'))}, "
                f"factor={scale:.2f}, old={float(search_speed):.2f}, new={float(boosted_speed):.2f})."
            )
        return payload
    return payload


def _drop_attack_collab_leading_entry_waypoint(
    aircraft_id: int,
    path_id: int,
    payload: Dict[str, Any],
    *,
    emit: LogCallback,
) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        return payload
    waypoints = payload.get("waypointList")
    if not isinstance(waypoints, list) or len(waypoints) <= 1:
        return payload

    copied = [deepcopy(item) for item in waypoints if isinstance(item, dict)]
    if len(copied) <= 1:
        return payload
    if not _is_attack_collab_entry_waypoint(copied[0]):
        return payload
    if not _is_attack_collab_sweep_waypoint(copied[1]):
        return payload

    removed = copied[0]
    trimmed = copied[1:]
    relink_waypoints(trimmed)
    payload["waypointList"] = trimmed
    if "lahWaypointList" in payload:
        payload["lahWaypointList"] = deepcopy(trimmed)
    try:
        annotate_eta_flight_plan(payload, default_speed_mps=_DEFAULT_SPEED_MPS, waypoint_list_keys=("waypointList",))
    except Exception:
        pass

    emit(
        "[ATTACK][COLLAB] Leading entry waypoint removed before remaining LINE sweep "
        f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
        f"removedWaypointID={_to_int(removed.get('waypointID'))}, "
        f"firstSweepWaypointID={_to_int((trimmed[0] if trimmed else {}).get('waypointID'))})."
    )
    return payload


def _is_attack_collab_entry_waypoint(waypoint: Dict[str, Any]) -> bool:
    if not isinstance(waypoint, dict):
        return False
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return False
    if isinstance(filming.get("lineSearch"), dict):
        return False
    if _to_int(filming.get("operationMode")) != 1:
        return False
    return isinstance(filming.get("coordinateOrientation"), dict)


def _is_attack_collab_sweep_waypoint(waypoint: Dict[str, Any]) -> bool:
    if not isinstance(waypoint, dict):
        return False
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return False
    line_search = filming.get("lineSearch")
    if isinstance(line_search, dict):
        coords = line_search.get("coordinateList")
        return not isinstance(coords, list) or len(coords) >= 1
    return _to_int(filming.get("operationMode")) == 2


def _estimate_attack_collab_first_sweep_search_speed_from_reference(
    waypoint: Dict[str, Any],
    reference_coord: Dict[str, Any] | None,
) -> tuple[Optional[float], Optional[float]]:
    ref = _normalize_coordinate(reference_coord)
    if ref is None or not isinstance(waypoint, dict):
        return None, None
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return None, None
    line_search = filming.get("lineSearch")
    if not isinstance(line_search, dict):
        return None, None
    raw_coords = line_search.get("coordinateList")
    if not isinstance(raw_coords, list) or len(raw_coords) < 2:
        return None, None
    sweep_coords = [_normalize_coordinate(coord) for coord in raw_coords]
    sweep_coords = [coord for coord in sweep_coords if coord is not None]
    if len(sweep_coords) < 2:
        return None, None

    anchor_coord = _normalize_coordinate(waypoint.get("coordinate"))
    transit_distance_m = _haversine_distance_m(ref, anchor_coord) if anchor_coord is not None else None
    if transit_distance_m is None or transit_distance_m <= 1e-6:
        transit_distance_m = _haversine_distance_m(ref, sweep_coords[0])
    if transit_distance_m is None or transit_distance_m <= 1e-6:
        return None, None

    sweep_distance_m = 0.0
    prev_coord: Optional[Dict[str, Any]] = None
    for coord in sweep_coords:
        if prev_coord is not None:
            segment_m = _haversine_distance_m(prev_coord, coord)
            if segment_m is not None and segment_m > 0.0:
                sweep_distance_m += float(segment_m)
        prev_coord = coord
    if sweep_distance_m <= 1e-6:
        return None, None

    transit_speed_mps = _to_float(waypoint.get("speed")) or _DEFAULT_SPEED_MPS
    if transit_speed_mps <= 0.0:
        transit_speed_mps = _DEFAULT_SPEED_MPS
    effective_transit_distance_m = effective_line_search_transit_m(transit_distance_m)
    if effective_transit_distance_m <= 1e-6:
        return None, None
    transit_time_s = float(effective_transit_distance_m) / float(transit_speed_mps)
    if transit_time_s <= 1e-6:
        return None, None
    search_speed_weight = get_runtime_float("search_speed_weight", 1.1)
    try:
        search_speed_weight = max(0.1, float(search_speed_weight))
    except Exception:
        search_speed_weight = 1.1
    estimated_speed = float(sweep_distance_m) / float(transit_time_s) * float(search_speed_weight)
    current_search_speed = _to_float(line_search.get("searchSpeed")) or 0.0
    cap_cruise_speed_mps = max(float(transit_speed_mps), float(current_search_speed))
    return (
        clamp_line_search_speed_mps(
            estimated_speed,
            cruise_speed_mps=float(cap_cruise_speed_mps),
            speed_scale=float(search_speed_weight),
            minimum_speed_mps=float(current_search_speed),
        ),
        float(transit_distance_m),
    )


def _compute_attack_point(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    friendly_heading_deg: Optional[float] = None,
    friendly_speed_mps: Optional[float] = None,
    cache_stats: Optional[Dict[str, Any]] = None,
) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    started_at = time.perf_counter()
    friendly_norm = _normalize_coordinate(friendly_coord)
    enemy_norm = _normalize_coordinate(enemy_coord)
    if not friendly_norm or not enemy_norm:
        if cache_stats is not None:
            cache_stats.update({"hit": False, "elapsedMs": _elapsed_ms_detail(started_at), "error": "invalid_coordinate"})
        return None, "Insufficient coordinate data for attack calculation."

    min_standoff_m, preferred_standoff_m = get_attack_standoff_distances()
    altitude_offset_m = get_runtime_attack_float("attack_point_altitude_offset_m", 300.0)
    lah_altitude_floor_m = _normalize_altitude_value(friendly_norm.get("altitude"))
    los_enabled = _attack_los_enabled()
    los_num_rays = _attack_los_num_rays()
    los_analysis_radius_m = _attack_los_analysis_radius_m(preferred_standoff_m)
    cache_key = _build_attack_point_cache_key(
        friendly_norm,
        enemy_norm,
        min_standoff_m=float(min_standoff_m),
        preferred_standoff_m=float(preferred_standoff_m),
        altitude_offset_m=float(altitude_offset_m),
        los_enabled=bool(los_enabled),
        los_num_rays=int(los_num_rays),
        los_analysis_radius_m=float(los_analysis_radius_m),
    )
    cached = _ATTACK_POINT_CACHE.get(cache_key)
    if isinstance(cached, dict):
        _ATTACK_POINT_CACHE.move_to_end(cache_key)
        if cache_stats is not None:
            cache_stats.update({"hit": True, "elapsedMs": _elapsed_ms_detail(started_at)})
        return _copy_cached_attack_point(cached, friendly_norm), None

    try:
        if cache_stats is not None:
            cache_stats["hit"] = False
        if los_enabled:
            los_result, los_error = _compute_attack_point_inprocess(
                friendly_norm,
                enemy_norm,
                min_standoff_m=float(min_standoff_m),
                preferred_standoff_m=float(preferred_standoff_m),
            )
            los_method = "los_area_inprocess"
            if los_result is None:
                if _attack_point_los_missing_dependency(los_error):
                    fallback_result = None
                    fallback_error = "subprocess skipped after GDAL dependency failure"
                else:
                    fallback_result, fallback_error = _compute_attack_point_subprocess(
                        friendly_norm,
                        enemy_norm,
                        min_standoff_m=float(min_standoff_m),
                        preferred_standoff_m=float(preferred_standoff_m),
                    )
                if fallback_result is not None:
                    los_result = fallback_result
                    los_method = "los_area_subprocess"
                else:
                    los_error = f"{los_error}; fallback={fallback_error}"
            if los_result is not None:
                cache_limit = _cache_attack_point(cache_key, los_result)
                if cache_stats is not None:
                    cache_stats.update(
                        {
                            "elapsedMs": _elapsed_ms_detail(started_at),
                            "cacheSize": len(_ATTACK_POINT_CACHE),
                            "cacheLimit": cache_limit,
                            "method": los_method,
                            "numRays": int(los_num_rays),
                            "analysisRadiusM": float(los_analysis_radius_m),
                        }
                    )
                return dict(los_result), None
            if cache_stats is not None:
                cache_stats.update(
                    {
                        "losError": str(los_error or "unknown"),
                        "numRays": int(los_num_rays),
                        "analysisRadiusM": float(los_analysis_radius_m),
                    }
                )
        base_altitude = _to_float(enemy_norm.get("altitude"))
        altitude_int = _normalize_altitude_value((base_altitude or 0.0) + altitude_offset_m)
        standoff_selection = select_attack_standoff_coordinate(
            friendly_norm,
            enemy_norm,
            min_distance_m=min_standoff_m,
            preferred_distance_m=preferred_standoff_m,
            fallback_heading_deg=friendly_heading_deg,
        )
        standoff_coord = dict(standoff_selection.get("coordinate") or {})
        result = {
            "latitude": float(standoff_coord["latitude"]),
            "longitude": float(standoff_coord["longitude"]),
            "altitude": altitude_int,
            "friendly_distance_m": _haversine_distance_m(friendly_norm, standoff_coord),
            "enemy_distance_m": _haversine_distance_m(enemy_norm, standoff_coord),
            "current_distance_m": standoff_selection.get("current_distance_m"),
            "candidate_distance_m": standoff_selection.get("candidate_distance_m"),
            "min_standoff_m": standoff_selection.get("min_standoff_m"),
            "preferred_standoff_m": standoff_selection.get("preferred_standoff_m"),
            "raster_sources": [],
            "selection_mode": standoff_selection.get("mode") or "adaptive_standoff",
        }
        _apply_lah_altitude_floor(result, friendly_norm)
        cache_limit = _cache_attack_point(cache_key, result)
        if cache_stats is not None:
            cache_stats.update(
                {
                    "elapsedMs": _elapsed_ms_detail(started_at),
                    "cacheSize": len(_ATTACK_POINT_CACHE),
                    "cacheLimit": cache_limit,
                    "method": "adaptive_standoff",
                }
            )
        return (result, None)
    except Exception as exc:
        if cache_stats is not None:
            cache_stats.update({"hit": False, "elapsedMs": _elapsed_ms_detail(started_at), "error": str(exc)})
        return None, f"Attack point computation error: {exc}"


def _persist_attack_log(payload: Dict[str, Any]) -> Dict[str, Any]:
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    timestamp_token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%f")
    target_path = directory / f"log_attack_algorithm_{timestamp_token}.json"
    log_messages = payload.setdefault("logMessages", [])
    if isinstance(log_messages, list):
        for fov_adjust_message in pop_runtime_camera_fov_adjustment_logs():
            log_messages.append(str(fov_adjust_message))
    payload["log_text"] = "\n".join(payload.get("logMessages") or [])
    try:
        if not write_debug_json(target_path, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False):
            payload["log_artifact_mode"] = debug_artifact_mode()
            payload["log_artifact_written"] = False
            return payload
    except Exception as exc:
        if isinstance(payload.setdefault("steps", []), list):
            payload["steps"].append(
                {
                    "name": "persist_log",
                    "status": "error",
                    "message": f"Failed to write {target_path}: {exc}",
                }
        )
        return payload
    payload["log_path"] = str(target_path)
    payload["log_artifact_mode"] = debug_artifact_mode()
    payload["log_artifact_written"] = True
    payload.setdefault("logMessages", []).append(f"[LOG] Saved attack analysis -> {target_path}")
    payload["log_text"] = "\n".join(payload.get("logMessages") or [])
    return payload


def _normalize_coordinate(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    lat = _to_float(value.get("latitude") or value.get("lat"))
    lon = _to_float(value.get("longitude") or value.get("lon"))
    alt = _normalize_altitude_value(value.get("altitude") or value.get("alt"))
    if lat is None or lon is None:
        return None
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def _coordinate_to_world(coord: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    if not coord:
        return None
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    return (lon, lat)


def _coord_text(coord: Optional[Dict[str, Any]]) -> str:
    if not coord:
        return "-"
    return f"({coord.get('latitude')}, {coord.get('longitude')}, alt={coord.get('altitude')})"


def _extract_watcher_from_key(key: str) -> Optional[int]:
    if "-" not in key:
        return None
    try:
        _, watcher = key.split("-", 1)
        return int(watcher)
    except Exception:
        return None


def _haversine_distance_m(a: Dict[str, Any], b: Dict[str, Any]) -> Optional[float]:
    lat1 = _to_float(a.get("latitude"))
    lon1 = _to_float(a.get("longitude"))
    lat2 = _to_float(b.get("latitude"))
    lon2 = _to_float(b.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    r = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a_val = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a_val), math.sqrt(1.0 - a_val))
    return r * c


def _reasonable_uav_tracking_flight_altitude(value: Any) -> Optional[int]:
    altitude = _normalize_altitude_value(value)
    if altitude is None:
        return None
    if float(altitude) < float(_UAV_TRACKING_MIN_FLIGHT_ALTITUDE_M):
        return None
    if float(altitude) > float(_UAV_TRACKING_MAX_FLIGHT_ALTITUDE_M):
        return None
    return int(altitude)


def _resolve_uav_tracking_flight_altitude(
    *,
    agent_coord: Optional[Dict[str, Any]],
    fp_data: Dict[str, Any],
    artifacts: Any,
) -> int:
    waypoints = [wp for wp in (fp_data.get("waypointList") or []) if isinstance(wp, dict)]
    current_wp_id = _to_int(getattr(artifacts, "current_waypoint_id", None))
    previous_wp_id = _to_int(getattr(artifacts, "previous_waypoint_id", None))
    candidates: List[Any] = []
    for preferred_wp_id in (current_wp_id, previous_wp_id):
        if preferred_wp_id is None:
            continue
        for waypoint in waypoints:
            if _to_int(waypoint.get("waypointID")) != int(preferred_wp_id):
                continue
            coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else {}
            candidates.append(coord.get("altitude"))
            break
    if isinstance(agent_coord, dict):
        candidates.append(agent_coord.get("altitude"))
    for waypoint in reversed(waypoints):
        coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else {}
        candidates.append(coord.get("altitude"))
    for candidate in candidates:
        altitude = _reasonable_uav_tracking_flight_altitude(candidate)
        if altitude is not None:
            return int(altitude)
    for candidate in candidates:
        altitude = _normalize_altitude_value(candidate)
        if altitude is not None and altitude > 0:
            return int(
                max(
                    float(_UAV_TRACKING_MIN_FLIGHT_ALTITUDE_M),
                    min(float(_UAV_TRACKING_MAX_FLIGHT_ALTITUDE_M), float(altitude)),
                )
            )
    return int(_UAV_TRACKING_MIN_FLIGHT_ALTITUDE_M)



def _json_safe(value: Any) -> Any:
    try:
        json.dumps(value, ensure_ascii=False)
        return value
    except TypeError:
        if isinstance(value, dict):
            return {str(k): _json_safe(v) for k, v in value.items()}
        if isinstance(value, (list, tuple, set)):
            return [_json_safe(v) for v in value]
        return str(value)


def _to_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _normalize_option_name(value: Any) -> str:
    return "".join(str(value or "").split()).lower()


def _resolve_requested_plan_id(
    ctx: Dict[str, Any],
    *,
    preferred_option_names: Optional[set[str]] = None,
) -> Optional[int]:
    plan_ids = list(ctx.get("plan_ids") or [])
    option_names = list(ctx.get("option_names") or [])
    normalized_pref = {
        _normalize_option_name(name)
        for name in (preferred_option_names or set())
        if str(name or "").strip()
    }

    if normalized_pref and option_names:
        for idx, name in enumerate(option_names):
            if _normalize_option_name(name) not in normalized_pref:
                continue
            if idx >= len(plan_ids):
                continue
            plan_id = _to_int(plan_ids[idx])
            if plan_id is not None and plan_id > 0:
                return int(plan_id)

    for value in plan_ids:
        plan_id = _to_int(value)
        if plan_id is not None and plan_id > 0:
            return int(plan_id)

    fallback = _to_int(
        ctx.get("missionPlanID")
        or ctx.get("mission_plan_id")
    )
    if fallback is not None and fallback > 0:
        return int(fallback)
    return None


def _to_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "y", "yes"}:
            return True
        if lowered in {"false", "0", "n", "no"}:
            return False
    return None


def _extract_assigned_manned_id(mission_updates: Dict[str, Any]) -> Optional[int]:
    aircraft_entries = mission_updates.get("aircraft") if isinstance(mission_updates, dict) else None
    if not isinstance(aircraft_entries, list):
        return None
    for entry in aircraft_entries:
        if not isinstance(entry, dict):
            continue
        if str(entry.get("role") or "") != "manned":
            continue
        return _to_int(entry.get("aircraft_id"))
    return None


def _extract_assigned_manned_ids(mission_updates: Dict[str, Any]) -> List[int]:
    aircraft_entries = mission_updates.get("aircraft") if isinstance(mission_updates, dict) else None
    if not isinstance(aircraft_entries, list):
        return []
    assigned: List[int] = []
    for entry in aircraft_entries:
        if not isinstance(entry, dict):
            continue
        if str(entry.get("role") or "") != "manned":
            continue
        aircraft_id = _to_int(entry.get("aircraft_id"))
        if aircraft_id is None or aircraft_id in assigned:
            continue
        assigned.append(int(aircraft_id))
    return assigned


def _get_received_db():
    global _RECEIVE_DB_MOD
    for module_name in ("receive.database", "modules.common.receive.database", "database"):
        module = sys.modules.get(module_name)
        if module is not None:
            received = getattr(module, "received_db", None)
            if received is not None:
                return received

    if _RECEIVE_DB_MOD is None:
        database_path = _PROJECT_ROOT / "modules" / "common" / "receive" / "database.py"
        spec = importlib.util.spec_from_file_location(
            "ku_receive_database_standalone",
            database_path,
        )
        if spec is None or spec.loader is None:
            return None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _RECEIVE_DB_MOD = module

    return getattr(_RECEIVE_DB_MOD, "received_db", None)


def _normalize_waypoint_id(value: Any) -> Optional[int]:
    waypoint_id = _to_int(value)
    if waypoint_id is not None and waypoint_id <= 0:
        return None
    return waypoint_id


def _safe_get_value(obj: Any, *names: str) -> Any:
    if obj is None:
        return None
    if isinstance(obj, dict):
        for name in names:
            if name in obj:
                return obj.get(name)
    for name in names:
        try:
            if hasattr(obj, name):
                return getattr(obj, name)
        except Exception:
            pass
    return None


def _iter_safe_items(coll: Any):
    if coll is None:
        return
    try:
        for item in coll:
            yield item
    except Exception:
        return


def _load_latest_mission_progress_state(aircraft_id: int) -> Optional[Dict[str, Optional[int]]]:
    received_db = _get_received_db()
    if received_db is None:
        return None
    try:
        raw = received_db.get_received_0501()
    except Exception:
        raw = None
    if raw is None:
        return None

    current_plan_id = _to_int(_safe_get_value(raw, "currentMissionPlanID", "CurrentMissionPlanID"))
    progress_items = _safe_get_value(
        raw,
        "individualMissionProgressStatusList",
        "IndividualMissionProgressStatusList",
    )
    for item in _iter_safe_items(progress_items):
        aid = _to_int(_safe_get_value(item, "aircraftID", "AircraftID"))
        if aid != aircraft_id:
            continue
        mission = _safe_get_value(item, "currentIndividualMission", "CurrentIndividualMission")
        return {
            "currentMissionPlanID": current_plan_id,
            "individualMissionID": _to_int(
                _safe_get_value(mission, "individualMissionID", "IndividualMissionID")
            ),
            "progress": _to_int(
                _safe_get_value(
                    item,
                    "currentIndividualMissionProgress",
                    "CurrentIndividualMissionProgress",
                )
            ),
        }
    return None


def _load_path_waypoints(path_id: Optional[int]) -> List[Dict[str, Any]]:
    pid = _to_int(path_id)
    if pid is None:
        return []
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{pid}.json")
        data = read_json_cached(path, kind="FlightPath")
    except Exception:
        return []
    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        items = data.get(key)
        if isinstance(items, list):
            return [item for item in items if isinstance(item, dict)]
    return []


def _calc_uav_remaining_seconds(
    aircraft_id: int,
    agent_index: Dict[int, Dict[str, Any]],
    source_plan_id: Optional[int],
    source_artifact_cache: Dict[str, Any],
    emit: Callable[[str], None],
) -> Optional[float]:
    """UAV의 현재 waypoint부터 마지막 waypoint까지 남은 비행시간(초)을 계산한다."""
    from modules.common.eta import annotate_eta_flight_plan

    state = agent_index.get(int(aircraft_id)) or {}
    current_wp = _to_int(state.get("current_waypoint_id"))
    if current_wp is None:
        return None

    artifacts = _resolve_plan_artifacts_cached(
        source_plan_id=source_plan_id,
        aircraft_id=aircraft_id,
        current_waypoint_id=current_wp,
        cache=source_artifact_cache,
        emit=emit,
    )
    if artifacts is None:
        return None

    fp_data = _load_attack_cached_fp_data(
        source_artifact_cache,
        int(artifacts.path_id),
        emit=emit,
    )
    waypoints: List[Dict[str, Any]] = []
    if isinstance(fp_data, dict):
        for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
            items = fp_data.get(key)
            if isinstance(items, list):
                waypoints = [item for item in items if isinstance(item, dict)]
                break
    if not waypoints:
        waypoints = _load_path_waypoints(artifacts.path_id)
    if not waypoints:
        return None

    eta_fp_data = {"waypointList": deepcopy(waypoints)}
    try:
        annotate_eta_flight_plan(eta_fp_data, waypoint_list_keys=("waypointList",))
    except Exception:
        return None

    annotated = eta_fp_data.get("waypointList") or []
    current_eta = None
    final_eta = 0.0
    for wp in annotated:
        eta_val = wp.get("eta")
        if eta_val is None:
            continue
        eta_val = float(eta_val)
        wp_id = _to_int(wp.get("waypointID"))
        if wp_id == current_wp:
            current_eta = eta_val
        if eta_val > final_eta:
            final_eta = eta_val

    if current_eta is None:
        return None
    remaining = max(0.0, final_eta - current_eta)
    return remaining


def _compute_tracking_eta_from_uav_remaining(
    all_uav_ids: List[int],
    used_tracking_uav_ids: set,
    agent_index: Dict[int, Dict[str, Any]],
    source_plan_id: Optional[int],
    source_artifact_cache: Dict[str, Any],
    computed_etas: List[int],
    emit: Callable[[str], None],
) -> Optional[int]:
    """
    비추적 UAV가 있으면 → 남은 시간이 가장 긴 UAV 기준 ETA 반환.
    모든 UAV가 추적 중이면 → 기존 계산 ETA의 평균 반환.
    """
    non_tracking = [aid for aid in all_uav_ids if aid not in used_tracking_uav_ids]

    if non_tracking:
        remaining_map: Dict[int, float] = {}
        for aid in non_tracking:
            remaining = _calc_uav_remaining_seconds(
                aid, agent_index, source_plan_id, source_artifact_cache, emit,
            )
            if remaining is not None:
                remaining_map[aid] = remaining
                emit(f"[ATTACK][ETA] UAV {aid} remaining mission time: {remaining:.1f}s")

        if remaining_map:
            best_id = max(remaining_map, key=remaining_map.get)
            best_remaining = int(round(remaining_map[best_id]))
            emit(
                f"[ATTACK][ETA] Non-tracking UAV ETA selected: "
                f"UAV {best_id} with {best_remaining}s remaining"
            )
            return best_remaining
        emit("[ATTACK][ETA] Non-tracking UAV remaining time unavailable; keeping computed ETA.")
        return None

    # 모든 UAV가 추적 중 → 기존 계산 ETA 평균
    if computed_etas:
        avg = int(round(sum(computed_etas) / len(computed_etas)))
        emit(
            f"[ATTACK][ETA] All UAVs tracking; using average ETA: "
            f"{avg}s (from {len(computed_etas)} values: {computed_etas})"
        )
        return avg

    return None


def _load_attack_exclusion_plan_context(
    plan_id: Optional[int],
    aircraft_id: int,
) -> Optional[Dict[str, Any]]:
    resolved_plan_id = _to_int(plan_id)
    if resolved_plan_id is None:
        return None
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{resolved_plan_id}.json")
        plan_data = read_json_cached(plan_path, kind="MissionPlan")
    except Exception:
        return None

    aircraft_entry = None
    for entry in plan_data.get("aircraftList", []):
        if _to_int((entry or {}).get("aircraftID")) == aircraft_id:
            aircraft_entry = entry
            break
    if not isinstance(aircraft_entry, dict):
        return None

    package_id = _to_int(aircraft_entry.get("individualMissionPackageID"))
    if package_id is None:
        return None

    try:
        imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{package_id}.json")
        imp_data = read_json_cached(imp_path, kind="IndividualMissionPlan")
    except Exception:
        return None

    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        return None

    return {
        "plan_id": resolved_plan_id,
        "individual_mission_package_id": package_id,
        "individualMissionList": mission_list,
    }


def _build_attack_exclusion_resume_index(
    candidate_plan_ids: List[Any],
    aircraft_ids: List[Any],
) -> Dict[int, List[Dict[str, Any]]]:
    unique_plan_ids: List[int] = []
    seen_plan_ids: set[int] = set()
    for value in candidate_plan_ids or []:
        plan_id = _to_int(value)
        if plan_id is None or plan_id in seen_plan_ids:
            continue
        seen_plan_ids.add(int(plan_id))
        unique_plan_ids.append(int(plan_id))

    unique_aircraft_ids: List[int] = []
    seen_aircraft_ids: set[int] = set()
    for value in aircraft_ids or []:
        aircraft_id = _to_int(value)
        if aircraft_id is None or aircraft_id in seen_aircraft_ids:
            continue
        seen_aircraft_ids.add(int(aircraft_id))
        unique_aircraft_ids.append(int(aircraft_id))

    rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {int(aid): [] for aid in unique_aircraft_ids}
    path_waypoints_cache: Dict[int, List[Dict[str, Any]]] = {}
    for plan_order, candidate_plan_id in enumerate(unique_plan_ids):
        for aircraft_id in unique_aircraft_ids:
            context = _load_attack_exclusion_plan_context(candidate_plan_id, aircraft_id)
            if context is None:
                continue
            mission_list = context.get("individualMissionList") or []
            for mission_index, mission in enumerate(mission_list):
                if not isinstance(mission, dict):
                    continue
                mission_id = _to_int(mission.get("individualMissionID"))
                path_id = _to_int(mission.get("pathID"))
                if mission_id is None or path_id is None:
                    continue
                pid = int(path_id)
                if pid not in path_waypoints_cache:
                    path_waypoints_cache[pid] = _load_path_waypoints(pid)
                waypoints = path_waypoints_cache.get(pid) or []
                if not waypoints:
                    continue
                active_idx = next(
                    (idx for idx, wp in enumerate(waypoints) if isinstance(wp, dict) and not bool(wp.get("isDone"))),
                    0,
                )
                rows_by_aircraft.setdefault(int(aircraft_id), []).append(
                    {
                        "plan_id": int(candidate_plan_id),
                        "plan_order": int(plan_order),
                        "mission_index": int(mission_index),
                        "mission_id": int(mission_id),
                        "path_id": int(pid),
                        "mission_is_done": bool(mission.get("isDone")),
                        "active_idx": int(active_idx),
                        "waypoints": waypoints,
                    }
                )
    return rows_by_aircraft


def _infer_attack_exclusion_resume_state(
    *,
    source_plan_id: Optional[int],
    aircraft_id: int,
    current_coord: Optional[Dict[str, Any]],
    emit: LogCallback,
    resume_index: Optional[Dict[int, List[Dict[str, Any]]]] = None,
) -> Tuple[Optional[int], Optional[int]]:
    coord_norm = _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None
    progress_state = _load_latest_mission_progress_state(aircraft_id) or {}
    progress_plan_id = _to_int(progress_state.get("currentMissionPlanID"))
    progress_individual_id = _to_int(progress_state.get("individualMissionID"))

    indexed_rows = list((resume_index or {}).get(int(aircraft_id)) or [])
    if not indexed_rows:
        candidate_plan_ids: List[int] = []
        seen_plan_ids: set[int] = set()
        for value in (
            source_plan_id,
            progress_plan_id,
            _load_latest_mission_progress_plan_id(),
            _scan_latest_source_plan_id(),
        ):
            plan_id = _to_int(value)
            if plan_id is None or plan_id in seen_plan_ids:
                continue
            seen_plan_ids.add(plan_id)
            candidate_plan_ids.append(plan_id)
        indexed_rows = list(
            _build_attack_exclusion_resume_index(candidate_plan_ids, [int(aircraft_id)]).get(int(aircraft_id)) or []
        )

    best_match: Optional[Dict[str, Any]] = None
    for row in indexed_rows:
        if not isinstance(row, dict):
            continue
        candidate_plan_id = _to_int(row.get("plan_id"))
        mission_id = _to_int(row.get("mission_id"))
        path_id = _to_int(row.get("path_id"))
        waypoints = row.get("waypoints") or []
        if candidate_plan_id is None or mission_id is None or path_id is None or not waypoints:
            continue

        mission_priority = 2
        if progress_individual_id is not None and mission_id == progress_individual_id:
            mission_priority = 0
        elif not bool(row.get("mission_is_done")):
            mission_priority = 1

        active_idx = _to_int(row.get("active_idx")) or 0
        plan_order = _to_int(row.get("plan_order")) or 0
        for wp_index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                continue
            waypoint_id = _to_int(waypoint.get("waypointID"))
            if waypoint_id is None:
                continue
            done_priority = 1 if bool(waypoint.get("isDone")) else 0
            coord = _normalize_coordinate(waypoint.get("coordinate"))
            if coord_norm:
                distance_m = _haversine_distance_m(coord_norm, coord) if coord else None
                distance_score = float(distance_m) if isinstance(distance_m, (int, float)) else 1.0e9
                tie_break = abs(wp_index - active_idx)
                score = (
                    mission_priority,
                    done_priority,
                    distance_score,
                    plan_order,
                    tie_break,
                    wp_index,
                )
            else:
                if wp_index != active_idx and done_priority > 0:
                    continue
                distance_m = None
                score = (
                    mission_priority,
                    done_priority,
                    abs(wp_index - active_idx),
                    plan_order,
                    wp_index,
                )

            candidate = {
                "score": score,
                "plan_id": candidate_plan_id,
                "individual_mission_id": mission_id,
                "path_id": path_id,
                "waypoint_id": waypoint_id,
                "distance_m": distance_m,
            }
            if best_match is None or candidate["score"] < best_match["score"]:
                best_match = candidate

    if best_match is None:
        return None, None

    distance_m = best_match.get("distance_m")
    distance_text = f", distance~{distance_m:.1f}m" if isinstance(distance_m, (int, float)) else ""
    emit(
        f"UAV {aircraft_id} currentWaypointID missing -> inferred waypoint "
        f"{best_match['waypoint_id']} from MissionPlan {best_match['plan_id']} "
        f"(mission={best_match['individual_mission_id']}, path={best_match['path_id']}{distance_text})."
    )
    return _to_int(best_match.get("plan_id")), _to_int(best_match.get("waypoint_id"))


def _resolve_attack_tracking_recovery(
    *,
    aircraft_id: int,
    source_plan_id: Optional[int],
    current_coord: Optional[Dict[str, Any]],
    emit: LogCallback,
) -> Optional[Dict[str, Any]]:
    assignment = get_tracking_assignment(aircraft_id)
    if not isinstance(assignment, dict) or not bool(assignment.get("active")):
        return None

    attack_plan_id = _to_int(assignment.get("attack_plan_id"))
    source_plan_id_int = _to_int(source_plan_id)
    if attack_plan_id is None or source_plan_id_int is None or attack_plan_id != source_plan_id_int:
        return None

    tracked_source_plan_id = _to_int(assignment.get("source_plan_id"))
    split_wp = (
        _normalize_waypoint_id(assignment.get("handoff_waypoint_id"))
        or _normalize_waypoint_id(assignment.get("last_nonzero_waypoint_id"))
        or _normalize_waypoint_id(assignment.get("original_current_waypoint_id"))
    )
    if tracked_source_plan_id is None or split_wp is None:
        return None

    handoff_coord = _normalize_coordinate(assignment.get("handoff_coordinate"))
    if handoff_coord is None:
        handoff_coord = _normalize_coordinate(assignment.get("last_nonzero_coordinate"))
    if handoff_coord is None:
        handoff_coord = _normalize_coordinate(current_coord)

    emit(
        f"UAV {aircraft_id} attack-exclusion recovery -> "
        f"detach tracking branch in candidate and resume source mission "
        f"(sourcePlan={tracked_source_plan_id}, splitWP={split_wp})."
    )
    return {
        "source_plan_id": tracked_source_plan_id,
        "split_waypoint_id": split_wp,
        "done_anchor_coord": handoff_coord,
        "tracking_assignment": assignment,
    }


def _clear_attack_tracking_assignment_if_attached_to_plan(
    *,
    aircraft_id: int,
    source_plan_id: Optional[int],
    emit: LogCallback,
    mutate_state: bool = True,
) -> bool:
    assignment = get_tracking_assignment(aircraft_id)
    if not isinstance(assignment, dict) or not bool(assignment.get("active")):
        return False

    attack_plan_id = _to_int(assignment.get("attack_plan_id"))
    source_plan_id_int = _to_int(source_plan_id)
    if attack_plan_id is None or source_plan_id_int is None or attack_plan_id != source_plan_id_int:
        return False

    if mutate_state:
        clear_tracking_assignment(aircraft_id)
        emit(
            f"UAV {aircraft_id} attack tracking assignment cleared after exclusion "
            f"(attackPlan={attack_plan_id})."
        )
    else:
        emit(
            f"UAV {aircraft_id} attack tracking assignment clear deferred until "
            f"attack-exclusion plan apply (attackPlan={attack_plan_id})."
        )
    return True


def _apply_resume_capture_buffer(
    resume_waypoints: List[Dict[str, Any]],
    *,
    emit: Callable[[str], None],
    log_prefix: str,
) -> None:
    return


def _mark_resume_waypoints_not_done(waypoints: Any) -> None:
    for waypoint in waypoints or []:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False


def _elapsed_ms(started_at: float) -> int:
    return int(round((time.perf_counter() - started_at) * 1000))


def _elapsed_ms_detail(started_at: float) -> float:
    return round((time.perf_counter() - started_at) * 1000.0, 3)


def _build_attack_source_artifact_cache(
    *,
    source_plan_id: int,
    plan_data: Dict[str, Any],
) -> Dict[str, Any]:
    aircraft_entries: Dict[int, Dict[str, Any]] = {}
    for entry in plan_data.get("aircraftList", []):
        if not isinstance(entry, dict):
            continue
        aircraft_id = _to_int(entry.get("aircraftID"))
        if aircraft_id is None or aircraft_id in aircraft_entries:
            continue
        aircraft_entries[aircraft_id] = entry
    return {
        "source_plan_id": int(source_plan_id),
        "aircraft_entries": aircraft_entries,
        "imp_payloads": {},
        "fp_payloads": {},
        "waypoint_ids": {},
    }


def _load_attack_cached_imp_data(
    cache: Dict[str, Any],
    package_id: int,
    *,
    emit: Callable[[str], None],
) -> Optional[Dict[str, Any]]:
    imp_payloads = cache.setdefault("imp_payloads", {})
    cached = imp_payloads.get(int(package_id))
    if isinstance(cached, dict):
        return cached

    try:
        imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(package_id)}.json")
        imp_data = read_json_cached(imp_path, kind="IndividualMissionPlan")
    except FileNotFoundError:
        emit(f"[PRIOR] IndividualMissionPlan {package_id} not found.")
        return None
    except Exception as exc:
        emit(f"[PRIOR] IndividualMissionPlan {package_id} load failed: {exc}")
        return None

    imp_payloads[int(package_id)] = imp_data
    return imp_data


def _load_attack_cached_fp_data(
    cache: Dict[str, Any],
    path_id: int,
    *,
    emit: Callable[[str], None],
) -> Optional[Dict[str, Any]]:
    fp_payloads = cache.setdefault("fp_payloads", {})
    cached = fp_payloads.get(int(path_id))
    if isinstance(cached, dict):
        return cached

    try:
        fp_path = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        fp_data = read_json_cached(fp_path, kind="FlightPath")
    except FileNotFoundError:
        emit(f"[ATTACK] FlightPath {path_id} not found.")
        return None
    except Exception as exc:
        emit(f"[ATTACK] FlightPath {path_id} load failed: {exc}")
        return None

    fp_payloads[int(path_id)] = fp_data
    return fp_data


def _load_attack_cached_waypoint_ids(
    cache: Dict[str, Any],
    path_id: int,
    *,
    emit: Callable[[str], None],
) -> List[int]:
    waypoint_ids = cache.setdefault("waypoint_ids", {})
    cached = waypoint_ids.get(int(path_id))
    if isinstance(cached, list):
        return list(cached)

    fp_data = _load_attack_cached_fp_data(cache, int(path_id), emit=emit)
    if fp_data is None:
        waypoint_ids[int(path_id)] = []
        return []

    resolved_ids: List[int] = []
    for wp in fp_data.get("waypointList", []):
        waypoint_id = _to_int((wp or {}).get("waypointID")) if isinstance(wp, dict) else None
        if waypoint_id is None:
            continue
        resolved_ids.append(waypoint_id)
    waypoint_ids[int(path_id)] = list(resolved_ids)
    return resolved_ids


def _resolve_plan_artifacts_cached(
    *,
    source_plan_id: Optional[int],
    aircraft_id: Optional[int],
    current_waypoint_id: Optional[int],
    cache: Dict[str, Any],
    emit: Callable[[str], None],
    allow_first_mission_fallback: bool = True,
) -> Optional[PlanMissionArtifacts]:
    if source_plan_id is None or aircraft_id is None:
        return None

    aircraft_entry = (cache.get("aircraft_entries") or {}).get(int(aircraft_id))
    if aircraft_entry is None:
        emit(f"[PRIOR] Aircraft {aircraft_id} not present in MissionPlan {source_plan_id}.")
        return None

    package_id = _to_int(aircraft_entry.get("individualMissionPackageID"))
    if package_id is None:
        emit(f"[PRIOR] Aircraft {aircraft_id} missing IndividualMissionPackageID.")
        return None

    imp_data = _load_attack_cached_imp_data(cache, int(package_id), emit=emit)
    if imp_data is None:
        return None

    missions = imp_data.get("individualMissionList") or []
    target_mission: Optional[Tuple[int, int]] = None
    previous_wp: Optional[int] = None
    resolved_current_wp = current_waypoint_id

    for mission in missions:
        path_id = _to_int((mission or {}).get("pathID"))
        individual_mission_id = _to_int((mission or {}).get("individualMissionID"))
        if path_id is None or individual_mission_id is None:
            continue
        waypoints = _load_attack_cached_waypoint_ids(cache, int(path_id), emit=emit)
        if not waypoints:
            continue
        if current_waypoint_id in waypoints:
            idx = waypoints.index(current_waypoint_id)
            previous_wp = waypoints[idx - 1] if idx > 0 else None
            target_mission = (individual_mission_id, path_id)
            break

    if target_mission is None and missions and allow_first_mission_fallback:
        fallback = missions[0] or {}
        mission_id = _to_int(fallback.get("individualMissionID")) or 0
        path_id = _to_int(fallback.get("pathID")) or 0
        waypoints = _load_attack_cached_waypoint_ids(cache, int(path_id), emit=emit)
        if waypoints:
            resolved_current_wp = waypoints[0]
            previous_wp = None
        target_mission = (mission_id, path_id)
        emit(
            "[PRIOR] Falling back to first mission for aircraft "
            f"{aircraft_id} (current waypoint not found in plan)."
        )

    if target_mission is None:
        return None

    mission_id, path_id = target_mission
    return PlanMissionArtifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        individual_mission_package_id=int(package_id),
        individual_mission_id=int(mission_id),
        path_id=int(path_id),
        current_waypoint_id=resolved_current_wp,
        previous_waypoint_id=previous_wp,
    )


def _select_unique_collab_replacement(
    descriptor: Dict[str, Any],
    collaborative_resume_by_input: Dict[int, CollaborativeResumeReplanResult],
) -> Tuple[Optional[int], Optional[CollaborativeResumeReplanResult], List[int]]:
    if descriptor.get("mode") != "UAV_RESUME" or not collaborative_resume_by_input:
        return None, None, []
    aircraft_id = _to_int(descriptor.get("aircraft_id"))
    if aircraft_id is None:
        return None, None, []

    matches: List[Tuple[int, CollaborativeResumeReplanResult]] = []
    for input_id, collab in collaborative_resume_by_input.items():
        if collab is None:
            continue
        replacement_ids = {
            int(aid)
            for aid in (getattr(collab, "replacement_aircraft_ids", set()) or set())
            if _to_int(aid) is not None
        }
        if int(aircraft_id) in replacement_ids:
            matches.append((int(input_id), collab))

    if len(matches) == 1:
        return matches[0][0], matches[0][1], [matches[0][0]]
    return None, None, [int(input_id) for input_id, _ in matches]


def _clone_attack_imp_shell(imp_data: Dict[str, Any]) -> Dict[str, Any]:
    cloned = dict(imp_data or {})
    missions = imp_data.get("individualMissionList") if isinstance(imp_data, dict) else None
    cloned["individualMissionList"] = list(missions) if isinstance(missions, list) else []
    return cloned


def _attack_follow_up_skip_reason(
    mission: Dict[str, Any],
    *,
    excluded_input_ids: set[int],
) -> Optional[str]:
    if bool(mission.get("isDone")):
        return "individual mission already done"
    input_id = _extract_related_input_mission_id(mission)
    if input_id is not None and int(input_id) in excluded_input_ids:
        return f"input mission {int(input_id)} already done"
    return None


def _attack_follow_up_requires_clone(
    mission: Dict[str, Any],
    *,
    current_input_id: Optional[int],
) -> bool:
    current_input = _to_int(current_input_id)
    if current_input is None or int(current_input) <= 0:
        return True
    return _extract_related_input_mission_id(mission) == int(current_input)


def _attack_follow_up_existing_path_safe_to_preserve(
    mission: Dict[str, Any],
    *,
    aircraft_id: int,
    emit: Optional[Callable[[str], None]] = None,
    log_prefix: str = "[ATTACK]",
) -> bool:
    source_path_id = _to_int(mission.get("pathID"))
    mission_id = _to_int(mission.get("individualMissionID"))
    if source_path_id is None or mission_id is None:
        return False
    try:
        src = db_paths.get_db_subpath("FlightPath", f"{int(source_path_id)}.json")
        fp_data = read_json_cached(src, kind="FlightPath")
    except Exception as exc:
        if emit is not None:
            emit(
                f"{log_prefix} Follow-up path {source_path_id} cannot be verified for preservation; "
                f"falling back to clone ({exc})."
            )
        return False
    if not isinstance(fp_data, dict):
        return False
    fp_path_id = _to_int(fp_data.get("pathID"))
    if fp_path_id is not None and int(fp_path_id) != int(source_path_id):
        return False
    fp_aircraft_id = _to_int(fp_data.get("aircraftID"))
    if fp_aircraft_id is not None and int(fp_aircraft_id) != int(aircraft_id):
        return False
    fp_mission_id = _to_int(fp_data.get("individualMissionID"))
    if fp_mission_id is not None and int(fp_mission_id) != int(mission_id):
        return False

    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        waypoints = fp_data.get(key)
        if not isinstance(waypoints, list):
            continue
        if any(isinstance(wp, dict) and bool(wp.get("isDone")) for wp in waypoints):
            if emit is not None:
                emit(
                    f"{log_prefix} Follow-up path {source_path_id} has completed waypoint state; "
                    "falling back to clone."
                )
            return False
    return True


def _attack_follow_up_can_preserve(
    mission: Dict[str, Any],
    *,
    aircraft_id: int,
    current_input_id: Optional[int],
    emit: Optional[Callable[[str], None]] = None,
    log_prefix: str = "[ATTACK]",
) -> bool:
    if _attack_follow_up_requires_clone(mission, current_input_id=current_input_id):
        return False
    return _attack_follow_up_existing_path_safe_to_preserve(
        mission,
        aircraft_id=aircraft_id,
        emit=emit,
        log_prefix=log_prefix,
    )


def _count_attack_follow_up_clone_missions(
    *,
    missions: List[Dict[str, Any]],
    aircraft_id: int,
    target_index: Optional[int],
    current_input_id: Optional[int],
    excluded_input_ids: Optional[set[int]],
) -> int:
    if not isinstance(missions, list) or target_index is None or int(target_index) < 0:
        return 0
    excluded_inputs = {
        int(value)
        for value in (excluded_input_ids or set())
        if _to_int(value) is not None
    }
    clone_count = 0
    for mission in missions[int(target_index) + 1 :]:
        if not isinstance(mission, dict):
            continue
        if _attack_follow_up_skip_reason(mission, excluded_input_ids=excluded_inputs) is not None:
            continue
        if not _attack_follow_up_can_preserve(
            mission,
            aircraft_id=aircraft_id,
            current_input_id=current_input_id,
        ):
            clone_count += 1
    return clone_count


def _collect_attack_follow_up_replan_artifacts(
    *,
    missions: List[Dict[str, Any]],
    aircraft_id: int,
    now_ms: int,
    emit: Callable[[str], None],
    log_prefix: str,
    current_input_id: Optional[int],
    excluded_input_ids: Optional[set[int]] = None,
    individual_id_provider: Optional[Callable[[], int]] = None,
    path_id_provider: Optional[Callable[[int], int]] = None,
    waypoint_id_provider: Optional[Callable[[], int]] = None,
) -> Optional[Tuple[List[Dict[str, Any]], List[Tuple[Path, Dict[str, Any]]], Dict[str, Any]]]:
    excluded_inputs = {
        int(value)
        for value in (excluded_input_ids or set())
        if _to_int(value) is not None
    }
    clone_sources: List[Dict[str, Any]] = []
    assembly: List[Tuple[str, Any]] = []
    skipped_count = 0
    preserved_count = 0

    for mission in missions or []:
        if not isinstance(mission, dict):
            continue
        skip_reason = _attack_follow_up_skip_reason(mission, excluded_input_ids=excluded_inputs)
        if skip_reason is not None:
            skipped_count += 1
            emit(
                f"{log_prefix} Skipping follow-up mission "
                f"{_to_int(mission.get('individualMissionID'))} ({skip_reason})."
            )
            continue
        source_path_id = _to_int(mission.get("pathID"))
        if source_path_id is None:
            emit(
                f"{log_prefix} Follow-up mission pathID missing for aircraft {aircraft_id}; "
                "aborting artifact preservation."
            )
            return None
        if not _attack_follow_up_can_preserve(
            mission,
            aircraft_id=aircraft_id,
            current_input_id=current_input_id,
            emit=emit,
            log_prefix=log_prefix,
        ):
            clone_sources.append(mission)
            assembly.append(("clone", len(clone_sources) - 1))
            continue
        preserved = deepcopy(mission)
        preserved["isDone"] = False
        assembly.append(("preserve", preserved))
        preserved_count += 1

    if not assembly:
        return [], [], {
            "candidateCount": 0,
            "clonedCount": 0,
            "preservedCount": 0,
            "skippedCount": int(skipped_count),
        }

    cloned_missions: List[Dict[str, Any]] = []
    cloned_paths: List[Tuple[Path, Dict[str, Any]]] = []
    if clone_sources:
        cloned_artifacts = _clone_follow_up_replan_artifacts(
            missions=clone_sources,
            aircraft_id=aircraft_id,
            now_ms=now_ms,
            emit=emit,
            log_prefix=log_prefix,
            excluded_input_ids=set(),
            individual_id_provider=individual_id_provider,
            path_id_provider=path_id_provider,
            waypoint_id_provider=waypoint_id_provider,
        )
        if cloned_artifacts is None:
            return None
        cloned_missions, cloned_paths = cloned_artifacts

    follow_up_missions: List[Dict[str, Any]] = []
    for mode, value in assembly:
        if mode == "preserve":
            follow_up_missions.append(value)
            continue
        clone_index = int(value)
        if clone_index >= len(cloned_missions):
            emit(
                f"{log_prefix} Follow-up clone count mismatch for aircraft {aircraft_id}; "
                "aborting artifact preservation."
            )
            return None
        follow_up_missions.append(cloned_missions[clone_index])

    stats = {
        "candidateCount": len(assembly),
        "clonedCount": len(cloned_missions),
        "preservedCount": int(preserved_count),
        "skippedCount": int(skipped_count),
    }
    if preserved_count:
        emit(
            f"{log_prefix} Preserved {preserved_count} follow-up mission(s) by existing ID/path "
            f"and cloned {len(cloned_missions)} follow-up mission(s) requiring rewrite."
        )
    return follow_up_missions, cloned_paths, stats


class AttackIdReservation:
    """Descriptor-local ID reservation for attack plan builders."""

    def __init__(self, reservation: ReplanIdReservation):
        self._reservation = reservation

    @classmethod
    def reserve_for_descriptor(
        cls,
        *,
        descriptor: Dict[str, Any],
        target_index: Optional[int],
        source_mission_count: int,
        source_waypoint_count: int = 0,
        attack_target_count: int = 0,
        follow_up_clone_count: Optional[int] = None,
    ) -> "AttackIdReservation":
        mode = str(descriptor.get("mode") or "")
        aircraft_id = _to_int(descriptor.get("aircraft_id")) or 0
        if follow_up_clone_count is not None:
            follow_up_count = max(0, int(follow_up_clone_count or 0))
        elif target_index is not None and int(target_index) >= 0:
            follow_up_count = max(0, int(source_mission_count or 0) - int(target_index) - 1)
        else:
            follow_up_count = 0

        imp_count = 1
        path_count = follow_up_count
        individual_count = follow_up_count
        source_waypoint_budget = max(0, int(source_waypoint_count or 0))
        # Follow-up clone paths may carry dense sweep waypoints. Reserve a large
        # per-mission budget so descriptor builders do not fall back to global ID calls.
        waypoint_count = max(8, source_waypoint_budget * 2 + follow_up_count * 128)

        if mode == "LAH_ATTACK":
            attack_count = max(1, int(attack_target_count or 1))
            path_count += attack_count + 1
            individual_count += attack_count + 1
            waypoint_count += attack_count + 2
        elif mode == "LAH_HOLD_RESUME":
            path_count += 2
            individual_count += 2
            waypoint_count += 3
        elif mode == "UAV_TRACK":
            path_count += 3
            individual_count += 2
            waypoint_count += 4
        else:
            path_count += 2
            individual_count += 1
            waypoint_count += 3

        reservation = ReplanIdReservation.reserve(
            imp_count=imp_count,
            individual_count=individual_count,
            path_count_by_aircraft={int(aircraft_id): path_count} if aircraft_id > 0 else {},
            waypoint_count=waypoint_count,
        )
        return cls(reservation)

    def next_imp(self) -> int:
        return self._reservation.next_imp()

    def next_path(self, aircraft_id: int) -> int:
        return self._reservation.next_path(int(aircraft_id))

    def next_paths(self, aircraft_id: int, count: int) -> List[int]:
        return [self.next_path(int(aircraft_id)) for _ in range(int(count or 0))]

    def next_individual(self) -> int:
        return self._reservation.next_individual()

    def next_individuals(self, count: int) -> List[int]:
        return [self.next_individual() for _ in range(int(count or 0))]

    def next_waypoint(self) -> int:
        return self._reservation.next_waypoint()

    def summary(self) -> Dict[str, Any]:
        return self._reservation.summary()



def _apply_attack_plan_overrides(
    *,
    ctx: Dict[str, Any],
    attack_point: Dict[str, Any],
    manned_aircraft: Optional[Dict[str, Any]],
    primary_target: Optional[Dict[str, Any]],
    agent_states: List[Any],
    waypoint_memory: Optional[Dict[str, Any]] = None,
    emit: Callable[[str], None],
) -> Optional[Dict[str, Any]]:
    def _set_override_failure(code: str, notice: str) -> None:
        ctx["_attack_failure_code"] = str(code or "").strip()
        ctx["_attack_failure_notice"] = str(notice or "").strip()

    override_detail_timing: Dict[str, Any] = {
        "phases": {},
        "collabRuns": [],
        "descriptorDetails": [],
    }

    def _record_phase(name: str, started_at: float, **extra: Any) -> float:
        elapsed_ms = _elapsed_ms_detail(started_at)
        row: Dict[str, Any] = {"elapsedMs": elapsed_ms}
        if extra:
            row.update(_json_safe(extra))
        override_detail_timing.setdefault("phases", {})[str(name)] = row
        return elapsed_ms

    metadata_started = time.perf_counter()
    attack_targets = [dict(item) for item in (ctx.get("_attack_target_list") or []) if isinstance(item, dict)]
    if not attack_targets and isinstance(primary_target, dict):
        attack_targets = [dict(primary_target)]
    if attack_targets and not isinstance(primary_target, dict):
        primary_target = dict(attack_targets[0])

    selected_manned_aircraft = [
        dict(item) for item in (ctx.get("_selected_manned_aircraft") or []) if isinstance(item, dict)
    ]
    if not selected_manned_aircraft and isinstance(manned_aircraft, dict):
        selected_manned_aircraft = [dict(manned_aircraft)]
    if selected_manned_aircraft and not isinstance(manned_aircraft, dict):
        manned_aircraft = dict(selected_manned_aircraft[0])
    _record_phase(
        "metadata_normalize",
        metadata_started,
        attackTargetCount=len(attack_targets),
        selectedMannedCount=len(selected_manned_aircraft),
    )

    if not attack_point or not primary_target or not attack_targets or not selected_manned_aircraft:
        _set_override_failure(
            "attack_override_metadata_missing",
            attack_failure_notice("attack_override_metadata_missing"),
        )
        emit("[ATTACK] Mission override skipped (insufficient attack metadata).")
        return None

    detail = _normalize_replan_detail(ctx.get("replan_detail")) or {}
    manned_id = _to_int((selected_manned_aircraft[0] or {}).get("aircraft_id"))
    watcher_id = _to_int(primary_target.get("watcher_id"))
    if manned_id is None:
        _set_override_failure(
            "attack_override_aircraft_id_missing",
            attack_failure_notice("attack_override_aircraft_id_missing"),
        )
        emit("[ATTACK] Mission override skipped (aircraft IDs missing).")
        return None

    agent_index_started = time.perf_counter()
    agent_index = _index_agent_states(agent_states, waypoint_memory=waypoint_memory)
    manned_state = agent_index.get(manned_id)
    uav_state = agent_index.get(watcher_id) if watcher_id is not None else None
    _record_phase("agent_index", agent_index_started, aircraftCount=len(agent_index))
    if not manned_state or not manned_state.get("coordinate"):
        _set_override_failure(
            "attack_override_manned_coordinate_missing",
            attack_failure_notice("attack_override_manned_coordinate_missing"),
        )
        emit(f"[ATTACK] Coordinate unavailable for manned aircraft {manned_id}.")
        return None
    if (
        watcher_id is not None
        and isinstance(uav_state, dict)
        and uav_state.get("current_waypoint_id") is not None
        and uav_state.get("current_waypoint_id_source") == "remembered"
    ):
        emit(
            f"[ATTACK] Using remembered waypoint {uav_state.get('current_waypoint_id')} "
            f"for watcher {watcher_id} while auto-tracking."
        )
    if len(attack_targets) == 1 and (not uav_state or not uav_state.get("current_waypoint_id")):
        emit(
            f"[ATTACK] Current waypoint unavailable for watcher {watcher_id}; "
            "attempting fallback tracking-UAV assignment."
        )
    weapon_started = time.perf_counter()
    selected_weapon_choice = _resolve_attack_weapon_choice(primary_target, manned_state)
    ctx["_selected_attack_weapon_type"] = int(selected_weapon_choice.get("selectedWeaponType", 1))
    ctx["_selected_attack_weapon_choice"] = selected_weapon_choice
    _record_phase(
        "weapon_choice",
        weapon_started,
        selectedWeaponType=ctx["_selected_attack_weapon_type"],
        ammoAvailable=bool(selected_weapon_choice.get("ammoAvailable")),
    )
    emit(
        "[ATTACK] Weapon choice "
        f"aircraft={manned_id} preferred={selected_weapon_choice.get('preferredWeaponType')} "
        f"selected={selected_weapon_choice.get('selectedWeaponType')} "
        f"ammo={selected_weapon_choice.get('weaponInventory')}"
    )

    trigger = str(detail.get("trigger") or "").strip()
    has_watcher = detail.get("watcherID") is not None or detail.get("watcherId") is not None
    has_target = detail.get("targetID") is not None or detail.get("targetId") is not None
    has_coord = detail.get("coordinate") is not None or detail.get("targetCoordinate") is not None
    use_detection_tracking = bool(trigger == "0402" or (has_watcher and (has_target or has_coord)))
    if not use_detection_tracking:
        replan_level = _to_int(ctx.get("replan_level") or ctx.get("replanLevel"))
        watcher_fallback = _to_int(primary_target.get("watcher_id")) if isinstance(primary_target, dict) else None
        if replan_level == 2 and watcher_fallback is not None:
            use_detection_tracking = True
            emit(
                f"[ATTACK][ETA] Using detection fallback (replan_level=2, watcher={watcher_fallback})."
            )

    tracking_eta_started = time.perf_counter()
    tracking_eta_s: Optional[int] = None
    if use_detection_tracking:
        manned_coord = manned_state.get("coordinate") if isinstance(manned_state, dict) else None
        if manned_coord and attack_point:
            distance_m = _haversine_distance_m(manned_coord, attack_point)
            speed_mps = _lah_max_attack_speed_mps()
            if distance_m is not None:
                tracking_eta_s = int(round(distance_m / speed_mps + 30.0))
                emit(
                    f"[ATTACK][ETA] Tracking ETA computed with LAH max attack speed "
                    f"(dist={distance_m:.1f}m, speed={speed_mps:.2f}m/s, +30s) -> {tracking_eta_s}s"
                )
            else:
                emit("[ATTACK][ETA] Distance calc failed; using default 30s.")
                tracking_eta_s = 30
        else:
            emit("[ATTACK][ETA] Manned/attack coordinate missing; using default 30s.")
            tracking_eta_s = 30
    _record_phase(
        "initial_tracking_eta",
        tracking_eta_started,
        useDetectionTracking=bool(use_detection_tracking),
        trackingEtaS=tracking_eta_s,
    )

    source_plan_started = time.perf_counter()
    source_plan_id = _resolve_attack_source_plan_id(ctx, detail)
    _record_phase("resolve_source_plan", source_plan_started, sourcePlanID=source_plan_id)
    if source_plan_id is None:
        _set_override_failure(
            "attack_override_source_plan_missing",
            attack_failure_notice("attack_override_source_plan_missing"),
        )
        emit("[ATTACK] Mission override skipped (no MissionPlan found).")
        return None
    override_started = time.perf_counter()

    source_load_started = time.perf_counter()
    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = read_json_cached(plan_src, kind="MissionPlan")
    except Exception as exc:
        _set_override_failure(
            "attack_override_source_plan_load_failed",
            attack_failure_notice("attack_override_source_plan_load_failed"),
        )
        emit(f"[ATTACK] MissionPlan {source_plan_id} load failed: {exc}")
        return None
    _record_phase(
        "source_plan_load",
        source_load_started,
        sourcePlanID=source_plan_id,
        aircraftCount=len(plan_data.get("aircraftList") or []),
    )
    ctx.pop("_lah_special_operation", None)
    lah_special_profile = None
    source_input_pkg_id = _to_int(
        plan_data.get("inputMissionPackageID")
        or plan_data.get("InputMissionPackageID")
        or plan_data.get("inputMissionPackageId")
    )
    if source_input_pkg_id is not None and source_input_pkg_id > 0:
        lah_profile_started = time.perf_counter()
        try:
            input_src = db_paths.get_db_subpath("InputMissionPlan", f"{int(source_input_pkg_id)}.json")
            input_data = read_json_cached(input_src, kind="InputMissionPlan")
            lah_special_profile = detect_lah_special_operation(input_data)
            if isinstance(lah_special_profile, dict):
                ctx["_lah_special_operation"] = deepcopy(lah_special_profile)
                emit(
                    "[ATTACK][LAH] special operation profile active "
                    f"(attackWait={lah_special_profile.get('attackWaitInputMissionID')}, "
                    f"battle={lah_special_profile.get('battlePositionInputMissionID')}, "
                    f"target={lah_special_profile.get('targetInputMissionID')})"
                )
        except Exception as exc:
            emit(f"[ATTACK][LAH] special operation profile unavailable: {exc}")
        _record_phase(
            "lah_special_profile",
            lah_profile_started,
            active=bool(isinstance(lah_special_profile, dict)),
            inputMissionPackageID=int(source_input_pkg_id),
        )

    sweep_progress_started = time.perf_counter()
    sweep_progress = load_sweep_progress()
    _record_phase("sweep_progress_load", sweep_progress_started, entries=len(sweep_progress or {}))

    plan_id_started = time.perf_counter()
    requested_plan_id = _resolve_requested_plan_id(
        ctx,
        preferred_option_names={"공격 특화", "공격특화", "공격추천"},
    )
    new_plan_id = requested_plan_id or _allocate_fresh_plan_id()
    _record_phase(
        "plan_id_resolve",
        plan_id_started,
        requested=bool(requested_plan_id is not None),
        missionPlanID=new_plan_id,
    )
    if requested_plan_id is not None:
        emit(
            "[ATTACK] Using requested missionPlanID "
            f"{new_plan_id} (sourcePlanID={source_plan_id})"
        )
    else:
        emit(
            "[ATTACK] Allocated fresh missionPlanID "
            f"{new_plan_id} (sourcePlanID={source_plan_id})"
        )

    source_cache_started = time.perf_counter()
    source_artifact_cache = _build_attack_source_artifact_cache(
        source_plan_id=int(source_plan_id),
        plan_data=plan_data,
    )
    _record_phase(
        "source_artifact_cache_build",
        source_cache_started,
        aircraftEntries=len(source_artifact_cache.get("aircraft_entries") or {}),
    )
    done_input_started = time.perf_counter()
    done_input_ids = _load_done_input_ids_for_plan(int(source_plan_id))
    _record_phase("done_input_ids_load", done_input_started, count=len(done_input_ids or set()))
    plan_copy_started = time.perf_counter()
    new_plan_data = deepcopy(plan_data)
    now_ms = _now_timestamp_ms()
    new_plan_data["missionPlanID"] = new_plan_id
    new_plan_data["timestamp"] = now_ms
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = now_ms
    _record_phase("plan_deepcopy", plan_copy_started)

    attack_coord = {
        "latitude": attack_point.get("latitude"),
        "longitude": attack_point.get("longitude"),
        "altitude": attack_point.get("altitude"),
    }
    attack_coord = _attach_attack_point_metadata(attack_coord, attack_point) or attack_coord

    target_assign_started = time.perf_counter()
    assigned_targets, used_tracking_uav_ids = _assign_targets_to_uav_watchers(
        [dict(item) for item in attack_targets[:3]],
        plan_data=plan_data,
        agent_index=agent_index,
    )
    _record_phase(
        "target_uav_assignment",
        target_assign_started,
        assignedTargetCount=len(assigned_targets or []),
        usedTrackingUavCount=len(used_tracking_uav_ids or set()),
    )
    if not assigned_targets:
        _set_override_failure(
            "attack_override_tracking_unavailable",
            attack_failure_notice("attack_tracking_assignment_failed"),
        )
        emit("[ATTACK] Mission override skipped (tracking UAV unavailable).")
        return None

    manned_assign_started = time.perf_counter()
    manned_sequences, manned_assignment_error = _assign_targets_to_manned_sequences(
        assigned_targets,
        selected_manned_aircraft,
    )
    _record_phase(
        "manned_sequence_assignment",
        manned_assign_started,
        aircraftCount=len(manned_sequences or {}),
        error=manned_assignment_error,
    )
    if not manned_sequences:
        _set_override_failure(
            "attack_override_weapon_assignment_failed",
            attack_failure_notice("attack_assignment_failed"),
        )
        emit(
            f"[ATTACK] Mission override skipped (manned assignment failed: {manned_assignment_error or 'unknown'})."
        )
        return None

    attack_coord_started = time.perf_counter()
    resolved_attack_coord_count = 0
    for aircraft_id, sequence in manned_sequences.items():
        aircraft_state = agent_index.get(int(aircraft_id)) or {}
        aircraft_coord = _normalize_coordinate(aircraft_state.get("coordinate"))
        aircraft_heading = _to_float(aircraft_state.get("heading"))
        current_aircraft_speed = _to_float(aircraft_state.get("speed")) or 40.0
        attack_speed_mps = _lah_max_attack_speed_mps()
        previous_attack_coord: Optional[Dict[str, Any]] = None
        accumulated_eta_s = 0.0

        for seq_index, sequence_target in enumerate(sequence):
            merged_target: Optional[Dict[str, Any]] = None
            for original_target in assigned_targets:
                if _same_target_identity(original_target, sequence_target):
                    merged_target = original_target
                    break
            if merged_target is None:
                continue

            target_coord = _normalize_coordinate(
                sequence_target.get("coordinate") or merged_target.get("coordinate")
            )
            if target_coord is None:
                continue

            resolved_attack_coord: Optional[Dict[str, Any]] = None
            resolved_attack_source: Any = None
            special_lah_attack_coord = special_attack_coordinate(ctx.get("_lah_special_operation"))
            target_in_special_target_region = (
                special_lah_attack_coord is not None
                and int(aircraft_id) in (1, 2, 3)
                and special_target_contains_coordinate(ctx.get("_lah_special_operation"), target_coord)
            )
            use_special_target_region_attack = False
            if special_lah_attack_coord is not None and int(aircraft_id) in (1, 2, 3):
                emit(
                    "[ATTACK][LAH] special battle-position anchor will be phase-checked "
                    f"(aircraft={int(aircraft_id)}, target={sequence_target.get('target_id')})"
                )
            if (
                resolved_attack_coord is None
                and _is_los_attack_point(attack_coord)
                and seq_index == 0
                and manned_id is not None
                and int(aircraft_id) == int(manned_id)
                and isinstance(primary_target, dict)
                and _same_target_identity(sequence_target, primary_target)
            ):
                resolved_attack_coord = _normalize_coordinate(attack_coord)
                resolved_attack_source = attack_coord
                resolved_attack_coord = _attach_attack_point_metadata(resolved_attack_coord, attack_coord)
                emit(
                    "[ATTACK][POINT] Primary target LOS-area point selected "
                    f"enemyDist={(attack_coord or {}).get('enemy_distance_m')}m "
                    f"rays={(attack_coord or {}).get('num_rays')}."
                )

            if (
                resolved_attack_coord is None
                and
                seq_index == 0
                and manned_id is not None
                and int(aircraft_id) == int(manned_id)
                and isinstance(primary_target, dict)
                and _same_target_identity(sequence_target, primary_target)
            ):
                selection = select_attack_standoff_coordinate(
                    aircraft_coord,
                    target_coord,
                    candidate_coord=attack_coord or target_coord,
                    fallback_heading_deg=aircraft_heading,
                )
                resolved_attack_coord = _normalize_coordinate(selection.get("coordinate"))
                resolved_attack_source = {"selection_mode": selection.get("mode") or "adaptive_standoff"}
                resolved_attack_coord = _attach_attack_point_metadata(resolved_attack_coord, resolved_attack_source)
                emit(
                    "[ATTACK][POINT] Primary target adaptive standoff "
                    f"mode={selection.get('mode')} currentDist={selection.get('current_distance_m')}m "
                    f"candidateDist={selection.get('candidate_distance_m')}m "
                    f"standoff={selection.get('min_standoff_m')}/{selection.get('preferred_standoff_m')}m."
                )

            if resolved_attack_coord is None and aircraft_coord is not None:
                start_coord = previous_attack_coord or aircraft_coord
                computed_attack_coord, _attack_error = _compute_attack_point(
                    start_coord,
                    target_coord,
                    friendly_heading_deg=aircraft_heading if previous_attack_coord is None else None,
                    friendly_speed_mps=current_aircraft_speed,
                )
                resolved_attack_coord = _normalize_coordinate(computed_attack_coord)
                resolved_attack_source = computed_attack_coord
                resolved_attack_coord = _attach_attack_point_metadata(resolved_attack_coord, computed_attack_coord)
                if _is_los_attack_point(computed_attack_coord):
                    emit(
                        "[ATTACK][POINT] LOS-area point selected "
                        f"(aircraft={int(aircraft_id)}, target={sequence_target.get('target_id')}, "
                        f"enemyDist={(computed_attack_coord or {}).get('enemy_distance_m')}m, "
                        f"rays={(computed_attack_coord or {}).get('num_rays')})."
                    )

            if resolved_attack_coord is None:
                selection = select_attack_standoff_coordinate(
                    previous_attack_coord or aircraft_coord or target_coord,
                    target_coord,
                    fallback_heading_deg=aircraft_heading if previous_attack_coord is None else None,
                )
                resolved_attack_coord = _normalize_coordinate(selection.get("coordinate"))
                resolved_attack_source = {"selection_mode": selection.get("mode") or "adaptive_standoff"}
                resolved_attack_coord = _attach_attack_point_metadata(resolved_attack_coord, resolved_attack_source)
                emit(
                    "[ATTACK][POINT] Fallback adaptive standoff "
                    f"mode={selection.get('mode')} currentDist={selection.get('current_distance_m')}m "
                    f"standoff={selection.get('min_standoff_m')}/{selection.get('preferred_standoff_m')}m."
                )

            _apply_lah_altitude_floor(resolved_attack_coord, aircraft_coord)
            if bool((resolved_attack_coord or {}).get("altitude_floor_applied")):
                emit(
                    "[ATTACK][POINT] Attack altitude raised to current LAH altitude "
                    f"(aircraft={int(aircraft_id)}, target={sequence_target.get('target_id')}, "
                    f"floor={resolved_attack_coord.get('lah_altitude_floor_m')}m)."
                )

            if resolved_attack_coord.get("altitude") is None:
                resolved_attack_coord["altitude"] = (
                    _normalize_altitude_value(target_coord.get("altitude"))
                    or _normalize_altitude_value((aircraft_coord or {}).get("altitude"))
                    or 800
                )

            start_coord = previous_attack_coord or aircraft_coord or target_coord
            segment_distance_m = _haversine_distance_m(start_coord, resolved_attack_coord)
            if isinstance(segment_distance_m, (int, float)):
                accumulated_eta_s += float(segment_distance_m) / max(float(attack_speed_mps), 1.0)

            tracking_eta_value = int(round(accumulated_eta_s + 30.0))
            selected_weapon_type = _to_int(
                sequence_target.get("selected_weapon_type") or sequence_target.get("weapon_type")
            )

            sequence_target["attack_coord"] = dict(resolved_attack_coord)
            sequence_target["tracking_eta_s"] = tracking_eta_value
            sequence_target["_lah_special_target_region"] = bool(target_in_special_target_region)
            sequence_target["_lah_special_target_region_attack"] = bool(use_special_target_region_attack)
            selection_mode = _attack_point_selection_mode(resolved_attack_coord or resolved_attack_source)
            if selection_mode:
                sequence_target["attack_point_selection_mode"] = selection_mode
            if isinstance(resolved_attack_coord, dict) and resolved_attack_coord.get("raster_sources") is not None:
                sequence_target["attack_point_raster_sources"] = list(resolved_attack_coord.get("raster_sources") or [])
            if selected_weapon_type is not None:
                sequence_target["selected_weapon_type"] = int(selected_weapon_type)
                sequence_target["weapon_type"] = int(selected_weapon_type)

            merged_target.update(
                {
                    "assigned_manned_aircraft_id": int(aircraft_id),
                    "weapon_choice": dict(sequence_target.get("weapon_choice") or {}),
                    "selected_weapon_type": int(selected_weapon_type) if selected_weapon_type is not None else None,
                    "weapon_type": int(selected_weapon_type) if selected_weapon_type is not None else None,
                    "attack_coord": dict(resolved_attack_coord),
                    "tracking_eta_s": tracking_eta_value,
                    "_lah_special_target_region_attack": bool(use_special_target_region_attack),
                    "attack_point_selection_mode": selection_mode or None,
                    "attack_point_raster_sources": list(resolved_attack_coord.get("raster_sources") or [])
                    if isinstance(resolved_attack_coord, dict)
                    else [],
                }
            )
            previous_attack_coord = dict(resolved_attack_coord)
            resolved_attack_coord_count += 1
    _record_phase(
        "attack_coordinate_resolution",
        attack_coord_started,
        resolvedCount=resolved_attack_coord_count,
        mannedSequenceCount=sum(len(seq or []) for seq in manned_sequences.values()),
    )

    # ── ETA 재계산: 비추적 UAV 기준 또는 전체 평균 ──
    all_plan_uav_ids: List[int] = []
    for entry in plan_data.get("aircraftList", []):
        aid = _to_int((entry or {}).get("aircraftID"))
        if aid is not None and aid > 3 and aid not in all_plan_uav_ids:
            all_plan_uav_ids.append(int(aid))

    computed_etas: List[int] = []
    for target in assigned_targets:
        eta_val = _to_int(target.get("tracking_eta_s"))
        if eta_val is not None:
            computed_etas.append(eta_val)

    remaining_eta_started = time.perf_counter()
    adjusted_eta = _compute_tracking_eta_from_uav_remaining(
        all_uav_ids=all_plan_uav_ids,
        used_tracking_uav_ids=used_tracking_uav_ids,
        agent_index=agent_index,
        source_plan_id=source_plan_id,
        source_artifact_cache=source_artifact_cache,
        computed_etas=computed_etas,
        emit=emit,
    )
    _record_phase(
        "remaining_eta",
        remaining_eta_started,
        allUavCount=len(all_plan_uav_ids),
        trackingUavCount=len(used_tracking_uav_ids or set()),
        computedEtaCount=len(computed_etas),
        adjustedEtaS=adjusted_eta,
    )
    if adjusted_eta is not None:
        tracking_eta_s = adjusted_eta
        for target in assigned_targets:
            target["tracking_eta_s"] = adjusted_eta
        emit(f"[ATTACK][ETA] All tracking ETAs unified to {adjusted_eta}s")

    for target in assigned_targets:
        if isinstance(primary_target, dict) and _same_target_identity(target, primary_target):
            primary_target = dict(target)
            break
    if isinstance(primary_target, dict):
        primary_weapon_type = _to_int(
            primary_target.get("selected_weapon_type") or primary_target.get("weapon_type")
        )
        if primary_weapon_type is not None:
            ctx["_selected_attack_weapon_type"] = int(primary_weapon_type)
        if isinstance(primary_target.get("weapon_choice"), dict):
            ctx["_selected_attack_weapon_choice"] = dict(primary_target.get("weapon_choice") or {})

    descriptor_list_started = time.perf_counter()
    descriptors: List[Dict[str, Any]] = []
    active_manned_ids = set(manned_sequences.keys())
    for aircraft in selected_manned_aircraft:
        aircraft_id = _to_int(aircraft.get("aircraft_id"))
        if aircraft_id is None or aircraft_id not in active_manned_ids:
            continue
        descriptors.append(
            {
                "label": "manned",
                "aircraft_id": int(aircraft_id),
                "state": agent_index.get(int(aircraft_id)),
                "attack_targets": [dict(item) for item in manned_sequences.get(int(aircraft_id), [])],
                "mode": "LAH_ATTACK",
            }
        )

    for target in assigned_targets:
        target_copy = dict(target)
        watcher_id = _to_int(target_copy.get("watcher_id"))
        if watcher_id is None:
            continue
        assigned_manned_id = _to_int(target_copy.get("assigned_manned_aircraft_id"))
        tracking_eta_value = _to_int(target_copy.get("tracking_eta_s")) or tracking_eta_s
        assigned_manned_state = agent_index.get(int(assigned_manned_id)) if assigned_manned_id is not None else None
        if tracking_eta_value is None and isinstance(assigned_manned_state, dict):
            manned_coord = _normalize_coordinate(assigned_manned_state.get("coordinate"))
            target_coord = _normalize_coordinate(target_copy.get("coordinate"))
            speed_mps = _lah_max_attack_speed_mps()
            distance_m = _haversine_distance_m(manned_coord, target_coord) if manned_coord and target_coord else None
            if isinstance(distance_m, (int, float)):
                tracking_eta_value = int(round(distance_m / max(speed_mps, 1.0) + 30.0))
        descriptors.append(
            {
                "label": f"uav_tracking_{watcher_id}",
                "aircraft_id": int(watcher_id),
                "state": agent_index.get(int(watcher_id)),
                "target_coord": target_copy.get("coordinate"),
                "attack_coord": target_copy.get("attack_coord") or target_copy.get("coordinate"),
                "target_id": target_copy.get("target_id"),
                "target_type": target_copy.get("target_type"),
                "tracking_eta_s": tracking_eta_value,
                "mode": "UAV_TRACK",
            }
        )

    other_lah_ids: List[int] = []
    for entry in plan_data.get("aircraftList", []):
        aid = _to_int((entry or {}).get("aircraftID"))
        if aid is None or aid > 3 or aid in active_manned_ids:
            continue
        if aid not in other_lah_ids:
            other_lah_ids.append(int(aid))
    for aid in other_lah_ids:
        descriptors.append(
            {
                "label": f"lah_hold_{aid}",
                "aircraft_id": int(aid),
                "state": agent_index.get(int(aid)),
                "target_coord": None,
                "target_id": None,
                "mode": "LAH_HOLD_RESUME",
            }
        )

    other_uav_ids: List[int] = []
    for entry in plan_data.get("aircraftList", []):
        aid = _to_int((entry or {}).get("aircraftID"))
        if aid is None or aid <= 3 or aid in used_tracking_uav_ids:
            continue
        if aid not in other_uav_ids:
            other_uav_ids.append(int(aid))
    for aid in other_uav_ids:
        descriptors.append(
            {
                "label": f"uav_resume_{aid}",
                "aircraft_id": int(aid),
                "state": agent_index.get(int(aid)),
                "target_coord": None,
                "target_id": None,
                "mode": "UAV_RESUME",
            }
        )
    _record_phase(
        "descriptor_list_build",
        descriptor_list_started,
        descriptorCount=len(descriptors),
        activeMannedCount=len(active_manned_ids),
        otherLahCount=len(other_lah_ids),
        otherUavCount=len(other_uav_ids),
    )

    collaborative_resume_by_input: Dict[int, CollaborativeResumeReplanResult] = {}
    if descriptors:
        collab_scan_started = time.perf_counter()
        tracking_unavailable_by_input: Dict[int, set[int]] = {}
        for descriptor in descriptors:
            if descriptor.get("mode") != "UAV_TRACK":
                continue
            aircraft_id = _to_int(descriptor.get("aircraft_id"))
            current_wp = _to_int((descriptor.get("state") or {}).get("current_waypoint_id"))
            if aircraft_id is None:
                continue
            artifacts = _resolve_plan_artifacts_cached(
                source_plan_id=source_plan_id,
                aircraft_id=int(aircraft_id),
                current_waypoint_id=current_wp,
                cache=source_artifact_cache,
                emit=emit,
            )
            if artifacts is None:
                continue
            imp_data = _load_attack_cached_imp_data(
                source_artifact_cache,
                int(artifacts.individual_mission_package_id),
                emit=emit,
            )
            if not isinstance(imp_data, dict):
                continue
            mission_list = imp_data.get("individualMissionList") if isinstance(imp_data.get("individualMissionList"), list) else []
            target_mission = next(
                (
                    mission
                    for mission in mission_list
                    if isinstance(mission, dict)
                    and _to_int(mission.get("individualMissionID")) == int(artifacts.individual_mission_id)
                ),
                None,
            )
            if not isinstance(target_mission, dict):
                continue
            input_mission_id = _extract_related_input_mission_id(target_mission)
            if input_mission_id is None or input_mission_id <= 0:
                continue
            tracking_unavailable_by_input.setdefault(int(input_mission_id), set()).add(int(aircraft_id))
        _record_phase(
            "collab_tracking_scan",
            collab_scan_started,
            inputCount=len(tracking_unavailable_by_input),
            unavailableUavCount=sum(len(vals) for vals in tracking_unavailable_by_input.values()),
        )

        if tracking_unavailable_by_input:
            collab_state_started = time.perf_counter()
            agent_state_map = _build_attack_collab_agent_state_map(
                agent_index,
                source_plan_id=source_plan_id,
                source_artifact_cache=source_artifact_cache,
                emit=emit,
            )
            _record_phase("collab_agent_state_map", collab_state_started, aircraftCount=len(agent_state_map))
            collab_parallel_enabled = (
                str(os.environ.get("REPLAN_ATTACK_COLLAB_PARALLEL", "1") or "").strip().lower()
                not in {"0", "false", "no", "off"}
            )
            collab_future_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
            collab_future_map: Dict[concurrent.futures.Future, tuple[int, set[int]]] = {}
            collab_futures_consumed = False

            def _build_collab_resume_payload(input_mission_id: int, unavailable_ids: set[int]) -> Dict[str, Any]:
                messages: List[str] = []

                def _thread_emit(message: str) -> None:
                    messages.append(str(message))

                collab_input_is_line = _source_input_mission_is_line(
                    source_plan_id=int(source_plan_id),
                    input_mission_id=int(input_mission_id),
                )

                def _attack_collab_flight_path_transform(
                    aircraft_id: int,
                    path_id: int,
                    payload: Dict[str, Any],
                ) -> Dict[str, Any]:
                    transformed = payload
                    if collab_input_is_line:
                        transformed = _drop_attack_collab_leading_entry_waypoint(
                            int(aircraft_id),
                            int(path_id),
                            transformed,
                            emit=_thread_emit,
                        )
                    return _boost_attack_collab_first_sweep_search_speed(
                        int(aircraft_id),
                        int(path_id),
                        transformed,
                        emit=_thread_emit,
                        reference_coord=(agent_state_map.get(int(aircraft_id)) or {}).get("coordinate"),
                    )

                collab_started = time.perf_counter()
                collab = _prepare_uav_collaborative_resume_replan(
                    source_plan_id=int(source_plan_id),
                    current_input_id=int(input_mission_id),
                    unavailable_aircraft_ids={int(aid) for aid in unavailable_ids},
                    agent_state_map=agent_state_map,
                    now_ms=int(now_ms),
                    emit=_thread_emit,
                    log_prefix="[ATTACK][COLLAB]",
                    drop_prefix_missions=False,
                    area_takeover_scope="full_remaining",
                    audit_context="attack_collaborative_resume_remaining_input",
                    flight_path_transform=_attack_collab_flight_path_transform,
                )
                collab_row = {
                    "inputMissionID": int(input_mission_id),
                    "unavailableAircraftIDs": sorted(int(aid) for aid in unavailable_ids),
                    "elapsedMs": _elapsed_ms_detail(collab_started),
                    "status": "ok" if collab is not None else "skipped",
                }
                if collab is not None:
                    collab_row.update(
                        {
                            "replacementAircraftIDs": sorted(int(aid) for aid in collab.replacement_aircraft_ids),
                            "generatedPathCount": len(collab.generated_path_ids or set()),
                            "finishEtaS": int(collab.finish_eta_s),
                            "workflow": str(collab.planner_workflow or ""),
                        }
                    )
                return {
                    "inputMissionID": int(input_mission_id),
                    "collab": collab,
                    "row": collab_row,
                    "messages": messages,
                }

            def _consume_collab_resume_futures(*, wait_reason: str) -> None:
                nonlocal collab_future_executor, collab_futures_consumed
                if collab_futures_consumed or not collab_future_map:
                    return
                join_started = time.perf_counter()
                try:
                    for future in concurrent.futures.as_completed(list(collab_future_map.keys())):
                        input_mission_id, unavailable_ids = collab_future_map[future]
                        try:
                            payload = future.result()
                        except Exception as exc:
                            emit(
                                "[ATTACK][COLLAB][WARN] Collaborative remaining replan failed "
                                f"(inputMissionID={input_mission_id}): {exc}"
                            )
                            override_detail_timing.setdefault("collabRuns", []).append(
                                {
                                    "inputMissionID": int(input_mission_id),
                                    "unavailableAircraftIDs": sorted(int(aid) for aid in unavailable_ids),
                                    "elapsedMs": _elapsed_ms_detail(join_started),
                                    "status": "failed",
                                    "error": str(exc),
                                }
                            )
                            continue
                        for message in payload.get("messages") or []:
                            if message:
                                emit(str(message))
                        collab_row = dict(payload.get("row") or {})
                        override_detail_timing.setdefault("collabRuns", []).append(collab_row)
                        collab = payload.get("collab")
                        resolved_input_id = int(payload.get("inputMissionID") or input_mission_id)
                        if collab is None:
                            continue
                        collaborative_resume_by_input[resolved_input_id] = collab
                        for aid, imp_id in collab.aircraft_imp_ids.items():
                            if int(aid) in {int(item) for item in (collab.unavailable_aircraft_ids or set())}:
                                emit(
                                    "[ATTACK][COLLAB][WARN] skipped applying replacement to unavailable aircraft "
                                    f"(aircraft={int(aid)}, inputMissionID={resolved_input_id})."
                                )
                                continue
                            _update_plan_aircraft_entry(new_plan_data, int(aid), int(imp_id), emit)
                finally:
                    collab_futures_consumed = True
                    _record_phase(
                        "collab_parallel_join",
                        join_started,
                        waitReason=wait_reason,
                        inputCount=len(collab_future_map),
                    )
                    if collab_future_executor is not None:
                        collab_future_executor.shutdown(wait=False, cancel_futures=False)
                        collab_future_executor = None

            collab_items = [
                (int(input_mission_id), {int(aid) for aid in unavailable_ids})
                for input_mission_id, unavailable_ids in tracking_unavailable_by_input.items()
            ]
            if collab_items and collab_parallel_enabled:
                parallel_started = time.perf_counter()
                collab_future_executor = concurrent.futures.ThreadPoolExecutor(
                    max_workers=min(2, len(collab_items)),
                    thread_name_prefix="AttackCollab",
                )
                for input_mission_id, unavailable_ids in collab_items:
                    future = collab_future_executor.submit(
                        _build_collab_resume_payload,
                        int(input_mission_id),
                        set(unavailable_ids),
                    )
                    collab_future_map[future] = (int(input_mission_id), set(unavailable_ids))
                _record_phase(
                    "collab_parallel_started",
                    parallel_started,
                    inputCount=len(collab_items),
                    workers=min(2, len(collab_items)),
                )
            else:
                for input_mission_id, unavailable_ids in collab_items:
                    payload = _build_collab_resume_payload(int(input_mission_id), set(unavailable_ids))
                    for message in payload.get("messages") or []:
                        if message:
                            emit(str(message))
                    override_detail_timing.setdefault("collabRuns", []).append(dict(payload.get("row") or {}))
                    collab = payload.get("collab")
                    if collab is None:
                        continue
                    collaborative_resume_by_input[int(input_mission_id)] = collab
                    for aid, imp_id in collab.aircraft_imp_ids.items():
                        if int(aid) in {int(item) for item in (collab.unavailable_aircraft_ids or set())}:
                            emit(
                                "[ATTACK][COLLAB][WARN] skipped applying replacement to unavailable aircraft "
                                f"(aircraft={int(aid)}, inputMissionID={int(input_mission_id)})."
                            )
                            continue
                        _update_plan_aircraft_entry(new_plan_data, int(aid), int(imp_id), emit)

    aircraft_updates: List[Dict[str, Any]] = []
    descriptor_timings: List[Dict[str, Any]] = []
    descriptor_loop_started = time.perf_counter()
    if "collab_future_map" in locals() and collab_future_map:
        independent_modes = {"LAH_ATTACK", "LAH_HOLD_RESUME"}
        descriptors = [
            descriptor for descriptor in descriptors if descriptor.get("mode") in independent_modes
        ] + [
            descriptor for descriptor in descriptors if descriptor.get("mode") not in independent_modes
        ]
    descriptor_parallel_enabled = (
        str(os.environ.get("REPLAN_ATTACK_DESCRIPTOR_PARALLEL", "1") or "").strip().lower()
        not in {"0", "false", "no", "off"}
    )
    descriptor_worker_limit = _to_int(os.environ.get("REPLAN_ATTACK_DESCRIPTOR_WORKERS"))
    if descriptor_worker_limit is None or descriptor_worker_limit <= 0:
        descriptor_worker_limit = 4
    descriptor_worker_count = min(int(descriptor_worker_limit), max(1, len(descriptors)))
    descriptor_future_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
    descriptor_future_map: Dict[concurrent.futures.Future, Dict[str, Any]] = {}
    descriptor_inline_results: List[Dict[str, Any]] = []

    def _run_attack_descriptor_builder_job(
        *,
        descriptor_index: int,
        descriptor_payload: Dict[str, Any],
        descriptor_detail_payload: Dict[str, Any],
        descriptor_started_at: float,
        aircraft_id_value: int,
        state_payload: Dict[str, Any],
        new_imp_id_value: int,
        new_imp_data_payload: Dict[str, Any],
        fp_data_payload: Dict[str, Any],
        target_mission_payload: Dict[str, Any],
        target_index_value: Optional[int],
        artifacts_payload: Any,
        id_reservation_payload: AttackIdReservation,
        collaborative_resume_payload: Optional[CollaborativeResumeReplanResult],
        execution_mode: str,
    ) -> Dict[str, Any]:
        messages: List[str] = []

        def _thread_emit(message: str) -> None:
            messages.append(str(message))

        worker_ctx = _attack_descriptor_worker_context(ctx)
        builder_started = time.perf_counter()
        mode = str(descriptor_payload.get("mode") or "")
        update: Optional[Dict[str, Any]]
        if mode == "LAH_ATTACK":
            update = _build_lah_attack_sequence_package(
                descriptor=descriptor_payload,
                assigned_targets=[dict(item) for item in descriptor_payload.get("attack_targets") or []],
                new_imp_id=new_imp_id_value,
                imp_data=new_imp_data_payload,
                fp_data=fp_data_payload,
                target_mission=target_mission_payload,
                target_index=target_index_value,
                ctx=worker_ctx,
                state=state_payload,
                aircraft_id=aircraft_id_value,
                artifacts=artifacts_payload,
                emit=_thread_emit,
                now_ms=now_ms,
                done_input_ids=done_input_ids,
                id_reservation=id_reservation_payload,
            )
        elif mode == "LAH_HOLD_RESUME":
            update = _build_lah_hold_resume_package(
                descriptor=descriptor_payload,
                new_imp_id=new_imp_id_value,
                imp_data=new_imp_data_payload,
                fp_data=fp_data_payload,
                target_mission=target_mission_payload,
                target_index=target_index_value,
                ctx=worker_ctx,
                state=state_payload,
                aircraft_id=aircraft_id_value,
                artifacts=artifacts_payload,
                emit=_thread_emit,
                now_ms=now_ms,
                done_input_ids=done_input_ids,
                id_reservation=id_reservation_payload,
            )
        elif mode == "UAV_TRACK":
            update = _build_uav_attack_tracking_package(
                descriptor=descriptor_payload,
                new_imp_id=new_imp_id_value,
                imp_data=new_imp_data_payload,
                fp_data=fp_data_payload,
                target_mission_template=target_mission_payload,
                target_index=target_index_value,
                attack_coord=descriptor_payload.get("attack_coord") or attack_coord,
                ctx=worker_ctx,
                state=state_payload,
                artifacts=artifacts_payload,
                emit=_thread_emit,
                now_ms=now_ms,
                force_start_at_current=use_detection_tracking,
                tracking_eta_s=_to_int(descriptor_payload.get("tracking_eta_s")) or tracking_eta_s,
                sweep_progress=sweep_progress,
                done_input_ids=done_input_ids,
                collaborative_resume=collaborative_resume_payload,
                id_reservation=id_reservation_payload,
            )
        else:
            update = _build_uav_attack_resume_package(
                descriptor=descriptor_payload,
                new_imp_id=new_imp_id_value,
                imp_data=new_imp_data_payload,
                fp_data=fp_data_payload,
                target_mission_template=target_mission_payload,
                target_index=target_index_value,
                ctx=worker_ctx,
                state=state_payload,
                artifacts=artifacts_payload,
                emit=_thread_emit,
                now_ms=now_ms,
                sweep_progress=sweep_progress,
                done_input_ids=done_input_ids,
                id_reservation=id_reservation_payload,
            )

        tracking_assignment: Optional[Dict[str, Any]] = None
        if update and mode == "UAV_TRACK":
            tracking_meta = update.get("tracking") if isinstance(update, dict) else {}
            resume_meta = update.get("resume") if isinstance(update, dict) else {}
            tracking_assignment = {
                "aircraft_id": int(aircraft_id_value),
                "source_plan_id": int(source_plan_id),
                "attack_plan_id": int(new_plan_id),
                "current_input_mission_id": _to_int(
                    (((target_mission_payload or {}).get("relatedMission") or {}).get("inputMissionID"))
                ),
                "original_path_id": int(artifacts_payload.path_id),
                "original_individual_mission_id": int(artifacts_payload.individual_mission_id),
                "original_current_waypoint_id": _normalize_waypoint_id(artifacts_payload.current_waypoint_id),
                "original_coordinate": state_payload.get("coordinate") if isinstance(state_payload, dict) else None,
                "tracking_path_id": _to_int((tracking_meta or {}).get("pathID")),
                "tracking_individual_mission_id": _to_int((tracking_meta or {}).get("individualMissionID")),
                "resume_path_id": _to_int((resume_meta or {}).get("pathID")),
                "resume_individual_mission_id": _to_int((resume_meta or {}).get("individualMissionID")),
                "target_id": _to_int(descriptor_payload.get("target_id")),
            }

        return {
            "descriptorIndex": int(descriptor_index),
            "aircraftID": int(aircraft_id_value),
            "label": descriptor_payload.get("label"),
            "mode": mode,
            "newImpID": int(new_imp_id_value),
            "status": "ok" if update else "builder_skipped",
            "elapsedMs": _elapsed_ms(descriptor_started_at),
            "descriptorDetail": dict(descriptor_detail_payload),
            "builderStage": {
                "elapsedMs": _elapsed_ms_detail(builder_started),
                "builderMode": mode,
                "executionMode": str(execution_mode),
            },
            "builderDetail": dict(update.get("timingMs") or {}) if isinstance(update, dict) else None,
            "update": update,
            "trackingAssignment": tracking_assignment,
            "messages": messages,
            "executionMode": str(execution_mode),
        }

    def _submit_attack_descriptor_builder_job(**job_kwargs: Any) -> None:
        nonlocal descriptor_future_executor
        if descriptor_parallel_enabled and descriptor_worker_count > 1:
            if descriptor_future_executor is None:
                parallel_start_started = time.perf_counter()
                descriptor_future_executor = concurrent.futures.ThreadPoolExecutor(
                    max_workers=int(descriptor_worker_count),
                    thread_name_prefix="AttackDescriptor",
                )
                _record_phase(
                    "descriptor_parallel_started",
                    parallel_start_started,
                    workers=int(descriptor_worker_count),
                    descriptorCount=len(descriptors),
                )
            job_kwargs["execution_mode"] = "parallel"
            future = descriptor_future_executor.submit(_run_attack_descriptor_builder_job, **job_kwargs)
            descriptor_future_map[future] = {
                "descriptorIndex": int(job_kwargs.get("descriptor_index", -1)),
                "aircraftID": int(job_kwargs.get("aircraft_id_value", 0) or 0),
                "label": (job_kwargs.get("descriptor_payload") or {}).get("label"),
                "mode": (job_kwargs.get("descriptor_payload") or {}).get("mode"),
            }
        else:
            job_kwargs["execution_mode"] = "serial"
            descriptor_inline_results.append(_run_attack_descriptor_builder_job(**job_kwargs))

    for descriptor_index, descriptor in enumerate(descriptors):
        if (
            "collab_future_map" in locals()
            and collab_future_map
            and not collab_futures_consumed
            and descriptor.get("mode") not in {"LAH_ATTACK", "LAH_HOLD_RESUME"}
        ):
            _consume_collab_resume_futures(wait_reason=str(descriptor.get("mode") or "dependent_descriptor"))
        descriptor_started = time.perf_counter()
        aircraft_id = descriptor["aircraft_id"]
        state = descriptor["state"] or {}
        current_wp = _to_int(state.get("current_waypoint_id"))
        descriptor_detail: Dict[str, Any] = {
            "aircraftID": int(aircraft_id) if aircraft_id is not None else None,
            "label": descriptor.get("label"),
            "mode": descriptor.get("mode"),
        }

        def _record_descriptor_stage(name: str, started_at: float, **extra: Any) -> None:
            row: Dict[str, Any] = {"elapsedMs": _elapsed_ms_detail(started_at)}
            if extra:
                row.update(_json_safe(extra))
            descriptor_detail[str(name)] = row

        def _finish_descriptor_detail(status: str) -> None:
            descriptor_detail["status"] = str(status)
            descriptor_detail["elapsedMs"] = _elapsed_ms_detail(descriptor_started)
            override_detail_timing.setdefault("descriptorDetails", []).append(dict(descriptor_detail))

        if aircraft_id is None:
            emit(f"[ATTACK] {descriptor['label']} aircraft lacks identifier; skipping.")
            _finish_descriptor_detail("skipped_missing_aircraft_id")
            descriptor_timings.append(
                {
                    "aircraftID": None,
                    "label": descriptor["label"],
                    "mode": descriptor["mode"],
                    "status": "skipped_missing_aircraft_id",
                    "elapsedMs": _elapsed_ms(descriptor_started),
                }
            )
            continue
        if descriptor["mode"] == "UAV_TRACK" and current_wp is None:
            emit(f"[ATTACK] {descriptor['label']} aircraft lacks waypoint context; skipping.")
            _finish_descriptor_detail("skipped_missing_waypoint")
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": descriptor["label"],
                    "mode": descriptor["mode"],
                    "status": "skipped_missing_waypoint",
                    "elapsedMs": _elapsed_ms(descriptor_started),
                }
            )
            continue

        early_collab_input_id, early_collab, early_collab_candidates = _select_unique_collab_replacement(
            descriptor,
            collaborative_resume_by_input,
        )
        if early_collab is not None and early_collab_input_id is not None:
            descriptor_detail["collabEarlySkip"] = {
                "currentInputMissionID": int(early_collab_input_id),
                "replacementAircraftIDs": sorted(
                    int(aid) for aid in (early_collab.replacement_aircraft_ids or set())
                ),
            }
            emit(
                f"[ATTACK][COLLAB] Aircraft {aircraft_id} handled by collaborative remaining replan "
                f"before artifact load (inputMissionID={early_collab_input_id})."
            )
            _finish_descriptor_detail("handled_by_collab_early")
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": descriptor["label"],
                    "mode": descriptor["mode"],
                    "status": "handled_by_collab",
                    "earlySkip": True,
                    "currentInputMissionID": int(early_collab_input_id),
                    "elapsedMs": _elapsed_ms(descriptor_started),
                }
            )
            continue
        if early_collab_candidates:
            descriptor_detail["collabEarlySkip"] = {
                "status": "ambiguous",
                "candidateInputMissionIDs": sorted(int(input_id) for input_id in early_collab_candidates),
            }

        resolve_artifacts_started = time.perf_counter()
        artifacts = _resolve_plan_artifacts_cached(
            source_plan_id=source_plan_id,
            aircraft_id=aircraft_id,
            current_waypoint_id=current_wp,
            cache=source_artifact_cache,
            emit=emit,
        )
        _record_descriptor_stage(
            "resolveArtifacts",
            resolve_artifacts_started,
            currentWaypointID=current_wp,
            sourcePathID=getattr(artifacts, "path_id", None) if artifacts is not None else None,
            sourceImpID=getattr(artifacts, "individual_mission_package_id", None) if artifacts is not None else None,
        )
        if artifacts is None:
            _finish_descriptor_detail("artifact_resolve_failed")
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": descriptor["label"],
                    "mode": descriptor["mode"],
                    "status": "artifact_resolve_failed",
                    "elapsedMs": _elapsed_ms(descriptor_started),
                }
            )
            continue

        load_artifacts_started = time.perf_counter()
        imp_data = _load_attack_cached_imp_data(
            source_artifact_cache,
            int(artifacts.individual_mission_package_id),
            emit=emit,
        )
        fp_data = _load_attack_cached_fp_data(
            source_artifact_cache,
            int(artifacts.path_id),
            emit=emit,
        )
        _record_descriptor_stage(
            "loadArtifacts",
            load_artifacts_started,
            impLoaded=isinstance(imp_data, dict),
            fpLoaded=isinstance(fp_data, dict),
        )
        if imp_data is None or fp_data is None:
            emit(f"[ATTACK] Failed to load artifacts for aircraft {aircraft_id}.")
            _finish_descriptor_detail("artifact_load_failed")
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": descriptor["label"],
                    "mode": descriptor["mode"],
                    "status": "artifact_load_failed",
                    "elapsedMs": _elapsed_ms(descriptor_started),
                }
            )
            continue

        locate_started = time.perf_counter()
        new_imp_data = _clone_attack_imp_shell(imp_data)
        mission_list = new_imp_data.get("individualMissionList", [])
        target_mission = None
        target_index = None
        for idx, mission in enumerate(mission_list):
            if _to_int(mission.get("individualMissionID")) == artifacts.individual_mission_id:
                target_mission = mission
                target_index = idx
                break
        _record_descriptor_stage(
            "cloneAndLocateMission",
            locate_started,
            cloneMode="imp_shell_and_mission_list",
            missionCount=len(mission_list) if isinstance(mission_list, list) else 0,
            targetIndex=target_index,
        )
        if target_mission is None:
            emit(
                f"[ATTACK] Individual mission {artifacts.individual_mission_id} "
                f"not found for aircraft {aircraft_id}."
            )
            _finish_descriptor_detail("target_mission_missing")
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": descriptor["label"],
                    "mode": descriptor["mode"],
                    "status": "target_mission_missing",
                    "elapsedMs": _elapsed_ms(descriptor_started),
                }
            )
            continue

        source_waypoint_count = sum(
            len(fp_data.get(key) or [])
            for key in ("waypointList", "uavWaypointList", "lahWaypointList")
            if isinstance(fp_data.get(key), list)
        )
        current_input_id = _extract_related_input_mission_id(target_mission) or 0
        follow_up_clone_count = _count_attack_follow_up_clone_missions(
            missions=mission_list if isinstance(mission_list, list) else [],
            aircraft_id=int(aircraft_id),
            target_index=target_index,
            current_input_id=current_input_id,
            excluded_input_ids=done_input_ids,
        )
        reservation_started = time.perf_counter()
        attack_id_reservation = AttackIdReservation.reserve_for_descriptor(
            descriptor=descriptor,
            target_index=target_index,
            source_mission_count=len(mission_list) if isinstance(mission_list, list) else 0,
            source_waypoint_count=source_waypoint_count,
            attack_target_count=len(descriptor.get("attack_targets") or []),
            follow_up_clone_count=follow_up_clone_count,
        )
        _record_descriptor_stage(
            "attackIdReservation",
            reservation_started,
            summary=attack_id_reservation.summary(),
            followUpCloneCount=int(follow_up_clone_count),
        )

        collaborative_resume = (
            collaborative_resume_by_input.get(int(current_input_id))
            if int(current_input_id) > 0
            else None
        )
        if (
            descriptor["mode"] == "UAV_RESUME"
            and collaborative_resume is not None
            and int(aircraft_id) in collaborative_resume.replacement_aircraft_ids
        ):
            emit(
                f"[ATTACK][COLLAB] Aircraft {aircraft_id} handled by collaborative remaining replan "
                f"(inputMissionID={current_input_id})."
            )
            _finish_descriptor_detail("handled_by_collab")
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": descriptor["label"],
                    "mode": descriptor["mode"],
                    "status": "handled_by_collab",
                    "elapsedMs": _elapsed_ms(descriptor_started),
                }
            )
            continue

        allocate_imp_started = time.perf_counter()
        new_imp_id = attack_id_reservation.next_imp()
        _record_descriptor_stage("allocateImpId", allocate_imp_started, newImpID=new_imp_id)
        plan_update_started = time.perf_counter()
        if not _mission_plan_has_aircraft_entry(new_plan_data, aircraft_id):
            _record_descriptor_stage("planAircraftEntryCheck", plan_update_started, newImpID=new_imp_id)
            _finish_descriptor_detail("plan_update_failed")
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": descriptor["label"],
                    "mode": descriptor["mode"],
                    "status": "plan_update_failed",
                    "elapsedMs": _elapsed_ms(descriptor_started),
                }
            )
            continue
        _record_descriptor_stage(
            "planAircraftEntryDeferred",
            plan_update_started,
            newImpID=new_imp_id,
            mergeMode="serial_after_descriptor_build",
        )
        _submit_attack_descriptor_builder_job(
            descriptor_index=descriptor_index,
            descriptor_payload=dict(descriptor),
            descriptor_detail_payload=dict(descriptor_detail),
            descriptor_started_at=descriptor_started,
            aircraft_id_value=int(aircraft_id),
            state_payload=deepcopy(state) if isinstance(state, dict) else {},
            new_imp_id_value=int(new_imp_id),
            new_imp_data_payload=new_imp_data,
            fp_data_payload=fp_data,
            target_mission_payload=target_mission,
            target_index_value=target_index,
            artifacts_payload=artifacts,
            id_reservation_payload=attack_id_reservation,
            collaborative_resume_payload=collaborative_resume,
        )
        continue
    if (
        "collab_future_map" in locals()
        and collab_future_map
        and not collab_futures_consumed
    ):
        _consume_collab_resume_futures(wait_reason="descriptor_loop_complete")
    descriptor_results: List[Dict[str, Any]] = list(descriptor_inline_results)
    if descriptor_future_map:
        descriptor_join_started = time.perf_counter()
        try:
            for future in concurrent.futures.as_completed(list(descriptor_future_map.keys())):
                descriptor_results.append(future.result())
        finally:
            if descriptor_future_executor is not None:
                descriptor_future_executor.shutdown(wait=True, cancel_futures=False)
                descriptor_future_executor = None
        _record_phase(
            "descriptor_parallel_join",
            descriptor_join_started,
            jobCount=len(descriptor_future_map),
            workers=int(descriptor_worker_count),
        )

    for descriptor_result in sorted(descriptor_results, key=lambda row: int(row.get("descriptorIndex", 0))):
        for message in descriptor_result.get("messages") or []:
            if message:
                emit(str(message))
        descriptor_detail = dict(descriptor_result.get("descriptorDetail") or {})
        if isinstance(descriptor_result.get("builderStage"), dict):
            descriptor_detail["builder"] = dict(descriptor_result.get("builderStage") or {})
        if isinstance(descriptor_result.get("builderDetail"), dict):
            descriptor_detail["builderDetail"] = dict(descriptor_result.get("builderDetail") or {})

        aircraft_id = int(descriptor_result.get("aircraftID"))
        new_imp_id = int(descriptor_result.get("newImpID"))
        mode = str(descriptor_result.get("mode") or "")
        label = descriptor_result.get("label")
        update = descriptor_result.get("update")
        status = str(descriptor_result.get("status") or "")
        if status == "ok" and isinstance(update, dict):
            plan_update_started = time.perf_counter()
            if not _update_plan_aircraft_entry(new_plan_data, aircraft_id, new_imp_id, emit):
                descriptor_detail["planAircraftEntryUpdate"] = {
                    "elapsedMs": _elapsed_ms_detail(plan_update_started),
                    "newImpID": int(new_imp_id),
                    "mergeMode": "serial_after_descriptor_build",
                }
                descriptor_detail["status"] = "plan_update_failed"
                descriptor_detail["elapsedMs"] = descriptor_result.get("elapsedMs")
                override_detail_timing.setdefault("descriptorDetails", []).append(dict(descriptor_detail))
                descriptor_timings.append(
                    {
                        "aircraftID": int(aircraft_id),
                        "label": label,
                        "mode": mode,
                        "status": "plan_update_failed",
                        "elapsedMs": descriptor_result.get("elapsedMs"),
                    }
                )
                continue
            descriptor_detail["planAircraftEntryUpdate"] = {
                "elapsedMs": _elapsed_ms_detail(plan_update_started),
                "newImpID": int(new_imp_id),
                "mergeMode": "serial_after_descriptor_build",
            }

            tracking_assignment = descriptor_result.get("trackingAssignment")
            if isinstance(tracking_assignment, dict):
                tracking_state_started = time.perf_counter()
                register_tracking_assignment(**tracking_assignment)
                descriptor_detail["trackingState"] = {
                    "elapsedMs": _elapsed_ms_detail(tracking_state_started),
                    "mergeMode": "serial_after_descriptor_build",
                }
                emit(
                    f"[ATTACK][UAV] Tracking assignment state saved "
                    f"(aircraft={aircraft_id}, sourcePlan={source_plan_id}, attackPlan={new_plan_id})."
                )

            aircraft_updates.append(update)
            descriptor_detail["status"] = "ok"
            descriptor_detail["elapsedMs"] = descriptor_result.get("elapsedMs")
            override_detail_timing.setdefault("descriptorDetails", []).append(dict(descriptor_detail))
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": label,
                    "mode": mode,
                    "status": "ok",
                    "elapsedMs": descriptor_result.get("elapsedMs"),
                }
            )
        else:
            descriptor_detail["status"] = "builder_skipped"
            descriptor_detail["elapsedMs"] = descriptor_result.get("elapsedMs")
            override_detail_timing.setdefault("descriptorDetails", []).append(dict(descriptor_detail))
            descriptor_timings.append(
                {
                    "aircraftID": int(aircraft_id),
                    "label": label,
                    "mode": mode,
                    "status": "builder_skipped",
                    "elapsedMs": descriptor_result.get("elapsedMs"),
                }
            )
    _record_phase(
        "descriptor_loop",
        descriptor_loop_started,
        descriptorCount=len(descriptors),
        updateCount=len(aircraft_updates),
    )

    if not aircraft_updates:
        _set_override_failure(
            "attack_override_artifacts_empty",
            attack_failure_notice("attack_override_artifacts_empty"),
        )
        override_total_ms = _elapsed_ms(override_started)
        override_detail_timing["totalMs"] = override_total_ms
        emit(
            "[ATTACK][TIME] override_detail="
            f"{json.dumps(_json_safe(override_detail_timing), ensure_ascii=False)}"
        )
        emit(f"[ATTACK][TIME] override_total={override_total_ms}ms")
        emit("[ATTACK] Mission override produced no artifacts.")
        return None

    if collaborative_resume_by_input:
        final_collab_plan_update_started = time.perf_counter()
        for collab in collaborative_resume_by_input.values():
            for aid, imp_id in collab.aircraft_imp_ids.items():
                if int(aid) in {int(item) for item in (collab.unavailable_aircraft_ids or set())}:
                    emit(
                        "[ATTACK][COLLAB][WARN] skipped final replacement for unavailable aircraft "
                        f"(aircraft={int(aid)}, inputMissionID={int(collab.current_input_id)})."
                    )
                    continue
                _update_plan_aircraft_entry(new_plan_data, int(aid), int(imp_id), emit)
        _record_phase(
            "final_collab_plan_update",
            final_collab_plan_update_started,
            collabInputCount=len(collaborative_resume_by_input),
        )

    validation_started = time.perf_counter()
    try:
        validation_summary = validate_replan_payloads(
            mission_plan=new_plan_data,
            scope="attack_plan",
            allow_existing_db_artifacts=True,
            log=emit,
        )
    except ReplanValidationError as exc:
        _record_phase("validation", validation_started, status="failed", errorCount=len(exc.errors))
        _set_override_failure(
            "attack_validation_failed",
            attack_failure_notice("attack_validation_failed"),
        )
        emit(f"[ATTACK][VALIDATION][ERR] {'; '.join(exc.errors[:4])}")
        return None
    _record_phase(
        "validation",
        validation_started,
        status="ok",
        missionPlanID=validation_summary.get("missionPlanID"),
        flightPathCount=validation_summary.get("flightPaths"),
    )

    plan_write_started = time.perf_counter()
    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(plan_dest, new_plan_data)
    carried_snapshot = mission_area_replan_store.carry_forward_snapshot(
        int(source_plan_id),
        int(new_plan_id),
        reason="attack_replan",
    )
    if carried_snapshot is not None:
        emit(
            "[ATTACK][PLAN] carried area remaining snapshot -> "
            f"{carried_snapshot.name} (sourcePlanID={source_plan_id}, planID={new_plan_id})"
        )
    _record_phase("mission_plan_write", plan_write_started, missionPlanID=new_plan_id)
    emit(f"[ATTACK][PLAN] MissionPlan saved -> {plan_dest.name} (planID={new_plan_id})")

    meta_started = time.perf_counter()
    plan_meta_map = dict(ctx.get("_option_meta") or {})
    plan_meta_entry = plan_meta_map.setdefault(new_plan_id, {})
    attack_target_meta: List[Dict[str, Any]] = []
    for target in assigned_targets:
        attack_target_meta.append(
            {
                "targetID": _to_int(target.get("target_id")),
                "targetType": _to_int(target.get("target_type")),
                "watcherID": _to_int(target.get("watcher_id")),
                "trackingAircraftID": _to_int(target.get("watcher_id")),
                "attackAircraftID": _to_int(target.get("assigned_manned_aircraft_id")),
                "weaponType": _to_int(
                    target.get("selected_weapon_type") or target.get("weapon_type")
                ),
                "weaponChoice": dict(target.get("weapon_choice") or {}),
                "coordinate": _normalize_coordinate(target.get("coordinate")),
                "attackPoint": _normalize_coordinate(target.get("attack_coord")) or attack_coord,
                "attackPointSelectionMode": target.get("attack_point_selection_mode")
                or _attack_point_selection_mode(target.get("attack_coord")),
                "attackPointRasterSources": list(target.get("attack_point_raster_sources") or []),
                "trackingEtaS": _to_int(target.get("tracking_eta_s")),
            }
        )
    plan_meta_entry.update(
        {
            "attack": True,
            "sourcePlanID": int(source_plan_id),
            "followUpAttackMode": len(attack_target_meta) > 1,
            "attackTargetCount": len(attack_target_meta),
            "attackTargets": attack_target_meta,
            "primaryTarget": attack_target_meta[0] if attack_target_meta else None,
        }
    )
    if collaborative_resume_by_input:
        plan_meta_entry["collaborativeRemainingReplan"] = [
            {
                "currentInputMissionID": int(input_id),
                "replacementAircraftIDs": sorted(int(aid) for aid in collab.replacement_aircraft_ids),
                "unavailableAircraftIDs": sorted(int(aid) for aid in collab.unavailable_aircraft_ids),
                "finishEtaS": int(collab.finish_eta_s),
                "plannerWorkflow": str(collab.planner_workflow or ""),
            }
            for input_id, collab in sorted(collaborative_resume_by_input.items())
        ]
    ctx["_option_meta"] = plan_meta_map
    _record_phase(
        "option_meta_build",
        meta_started,
        targetCount=len(attack_target_meta),
        collabInputCount=len(collaborative_resume_by_input),
    )
    override_total_ms = _elapsed_ms(override_started)
    override_detail_timing["totalMs"] = override_total_ms
    emit(
        "[ATTACK][TIME] override_detail="
        f"{json.dumps(_json_safe(override_detail_timing), ensure_ascii=False)}"
    )
    emit(f"[ATTACK][TIME] override_total={override_total_ms}ms")

    return {
        "source_plan_id": source_plan_id,
        "mission_plan_id": new_plan_id,
        "plan_path": str(plan_dest),
        "target_id": primary_target.get("target_id"),
        "target_type": primary_target.get("target_type"),
        "weapon_type": _to_int(ctx.get("_selected_attack_weapon_type")),
        "weapon_choice": dict(ctx.get("_selected_attack_weapon_choice") or {}),
        "attack_targets": [dict(item) for item in assigned_targets],
        "aircraft": aircraft_updates,
        "collaborativeRemainingReplan": [
            {
                "currentInputMissionID": int(input_id),
                "replacementAircraftIDs": sorted(int(aid) for aid in collab.replacement_aircraft_ids),
                "finishEtaS": int(collab.finish_eta_s),
            }
            for input_id, collab in sorted(collaborative_resume_by_input.items())
        ],
        "timingMs": {
            "overrideTotal": override_total_ms,
            "descriptorBuilds": descriptor_timings,
            "overrideDetail": override_detail_timing,
        },
    }


def _index_agent_states(
    agent_states: List[Any],
    *,
    waypoint_memory: Optional[Dict[str, Any]] = None,
) -> Dict[int, Dict[str, Any]]:
    index: Dict[int, Dict[str, Any]] = {}
    for entry in agent_states:
        if not isinstance(entry, dict):
            continue
        aircraft_id = _to_int(entry.get("aircraftID") or entry.get("aircraftId"))
        if aircraft_id is None:
            continue
        coord = (
            _normalize_coordinate(entry.get("coordinate"))
            or _normalize_coordinate((entry.get("mannedInfo") or {}).get("coordinate"))
            or _normalize_coordinate((entry.get("unmannedInfo") or {}).get("coordinate"))
        )
        wp_block = entry.get("currentWaypointID") or {}
        if not wp_block:
            unm_info = entry.get("unmannedInfo") or {}
            wp_block = unm_info.get("currentWaypointID") or {}
        current_wp = _normalize_waypoint_id(wp_block.get("waypointID"))
        waypoint_source = "live" if current_wp is not None else None
        if current_wp is None and isinstance(waypoint_memory, dict):
            remembered_wp = _normalize_waypoint_id(
                waypoint_memory.get(str(aircraft_id)) if str(aircraft_id) in waypoint_memory else waypoint_memory.get(aircraft_id)
            )
            if remembered_wp is not None:
                current_wp = remembered_wp
                waypoint_source = "remembered"
        velocity = entry.get("velocity") or {}
        heading = _to_float(velocity.get("heading"))
        speed = _to_float(velocity.get("speed"))
        if heading is not None:
            heading = heading % 360.0
        weapon_inventory = extract_attack_weapon_inventory(entry)
        index[aircraft_id] = {
            "aircraft_id": aircraft_id,
            "coordinate": coord,
            "current_waypoint_id": current_wp,
            "current_waypoint_id_source": waypoint_source,
            "is_unmanned": bool(entry.get("isUnmanned")),
            "heading": heading,
            "speed": speed,
            "weapon_inventory": weapon_inventory,
        }
    return index


def _insert_lah_attack_waypoint(
    flight_path: Dict[str, Any],
    current_waypoint_id: int,
    attack_coord: Dict[str, Any],
    new_waypoint_id: int,
    target_id: Optional[int],
    weapon_type: Optional[int] = None,
) -> Dict[str, Any]:
    waypoints = list(flight_path.get("lahWaypointList") or [])
    if not waypoints:
        raise ValueError("LAH flight path is empty.")
    current_index = next(
        (idx for idx, wp in enumerate(waypoints) if _to_int(wp.get("waypointID")) == current_waypoint_id),
        None,
    )
    if current_index is None:
        raise ValueError(f"Current waypoint {current_waypoint_id} not found in LAH flight path.")

    template = deepcopy(waypoints[current_index])
    prev_index = current_index - 1
    if prev_index >= 0 and "nextWaypointID" in waypoints[prev_index]:
        waypoints[prev_index]["nextWaypointID"] = new_waypoint_id

    altitude = _normalize_altitude_value(attack_coord.get("altitude"))
    if altitude is None:
        altitude = _normalize_altitude_value((template.get("coordinate") or {}).get("altitude"))
    if altitude is None:
        altitude = 800
    coordinate = {
        "latitude": attack_coord.get("latitude"),
        "longitude": attack_coord.get("longitude"),
        "altitude": altitude,
    }
    new_wp = {
        "waypointID": new_waypoint_id,
        "coordinate": coordinate,
        "speed": template.get("speed", 40),
        "eta": template.get("eta", 0),
        "ecf": template.get("ecf", 0.0),
        "nextWaypointID": current_waypoint_id,
        "hovering": {"time": 0},
        "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
        "attack": deepcopy(template.get("attack") or {}),
    }
    attack_block = new_wp["attack"] or {}
    attack_block["targetID"] = target_id or 0
    resolved_weapon_type = (
        max(0, min(3, int(weapon_type)))
        if weapon_type is not None
        else int(get_runtime_attack_weapon_type(2))
    )
    attack_block["weaponType"] = attack_block.get("weaponType") or resolved_weapon_type
    new_wp["attack"] = attack_block

    waypoints.insert(current_index, new_wp)
    flight_path["lahWaypointList"] = waypoints
    return new_wp


def _extract_path_source(fp_data: Dict[str, Any]) -> str:
    return str(fp_data.get("Source") or fp_data.get("source") or "MMR")


def _extract_lah_waypoint_coordinate(waypoint: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if not isinstance(waypoint, dict):
        return None
    return _normalize_coordinate(waypoint.get("coordinate"))


def _infer_lah_current_waypoint_id(
    waypoints: List[Dict[str, Any]],
    current_coord: Optional[Dict[str, Any]],
) -> Optional[int]:
    coord_norm = _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None
    if coord_norm is None:
        return None

    best_waypoint_id: Optional[int] = None
    best_score: Optional[Tuple[int, float, int]] = None
    for idx, waypoint in enumerate(waypoints):
        waypoint_id = _to_int(waypoint.get("waypointID"))
        waypoint_coord = _extract_lah_waypoint_coordinate(waypoint)
        if waypoint_id is None or waypoint_coord is None:
            continue
        distance_m = _haversine_distance_m(coord_norm, waypoint_coord)
        if distance_m is None:
            continue
        score = (
            1 if bool(waypoint.get("isDone")) else 0,
            float(distance_m),
            idx,
        )
        if best_score is None or score < best_score:
            best_score = score
            best_waypoint_id = waypoint_id
    return best_waypoint_id


def _lah_waypoints_to_coordinate_list(
    waypoints: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    coordinate_list: List[Dict[str, Any]] = []
    for waypoint in waypoints:
        coord = _extract_lah_waypoint_coordinate(waypoint)
        if coord:
            coordinate_list.append(coord)
    return coordinate_list


def _lah_resume_two_point_preserve_threshold_m() -> float:
    return max(
        0.0,
        get_runtime_attack_float(
            "lah_resume_preserve_two_point_min_length_m",
            _LAH_RESUME_PRESERVE_TWO_POINT_MIN_LENGTH_M,
        ),
    )


def _lah_waypoint_path_length_m(waypoints: List[Dict[str, Any]]) -> Optional[float]:
    coords = [
        coord
        for coord in (_extract_lah_waypoint_coordinate(item) for item in (waypoints or []))
        if coord is not None
    ]
    if len(coords) < 2:
        return None
    total_m = 0.0
    for start, end in zip(coords, coords[1:]):
        distance_m = _haversine_distance_m(start, end)
        if distance_m is None:
            return None
        total_m += float(distance_m)
    return float(total_m)


def _long_two_point_lah_resume_length_m(waypoints: List[Dict[str, Any]]) -> Optional[float]:
    resume = [item for item in (waypoints or []) if isinstance(item, dict)]
    if len(resume) != 2:
        return None
    path_length_m = _lah_waypoint_path_length_m(resume)
    if path_length_m is None:
        return None
    threshold_m = _lah_resume_two_point_preserve_threshold_m()
    if float(path_length_m) <= float(threshold_m):
        return None
    return float(path_length_m)


def _build_lah_anchor_waypoint(
    template_wp: Dict[str, Any],
    *,
    coord: Dict[str, Any],
    next_id: int = 0,
    hovering_time: int = 0,
    waypoint_id: Optional[int] = None,
) -> Dict[str, Any]:
    anchor_wp = _build_lah_waypoint_from_template(
        template_wp,
        int(waypoint_id) if waypoint_id is not None else _reserve_waypoint_block(1),
        coord,
        next_id,
        mark_attack=False,
        target_id=None,
    )
    anchor_wp["isDone"] = False
    if hovering_time > 0:
        anchor_wp["hovering"] = {"time": int(hovering_time)}
    elif "hovering" in anchor_wp:
        anchor_wp["hovering"] = {"time": 0}
    return anchor_wp


def _same_normalized_coordinate(
    left: Optional[Dict[str, Any]],
    right: Optional[Dict[str, Any]],
) -> bool:
    if left is None or right is None:
        return False
    left_lat = _to_float(left.get("latitude"))
    left_lon = _to_float(left.get("longitude"))
    right_lat = _to_float(right.get("latitude"))
    right_lon = _to_float(right.get("longitude"))
    if None in (left_lat, left_lon, right_lat, right_lon):
        return False
    if abs(float(left_lat) - float(right_lat)) > 1e-7:
        return False
    if abs(float(left_lon) - float(right_lon)) > 1e-7:
        return False
    return (_normalize_altitude_value(left.get("altitude")) or 0) == (
        _normalize_altitude_value(right.get("altitude")) or 0
    )


def _append_lah_done_anchor(
    done_waypoints: List[Dict[str, Any]],
    *,
    template_wp: Dict[str, Any],
    anchor_coord: Optional[Dict[str, Any]],
    waypoint_id_provider: Optional[Callable[[], int]] = None,
) -> None:
    anchor = _normalize_coordinate(anchor_coord) if isinstance(anchor_coord, dict) else None
    if anchor is None:
        return
    last_coord = _extract_lah_waypoint_coordinate(done_waypoints[-1]) if done_waypoints else None
    if _same_normalized_coordinate(last_coord, anchor):
        return
    done_waypoints.append(
        _build_lah_anchor_waypoint(
            template_wp,
            coord=anchor,
            next_id=0,
            hovering_time=0,
            waypoint_id=int(waypoint_id_provider()) if waypoint_id_provider is not None else None,
        )
    )


def _prepend_lah_transition_waypoint(
    waypoints: List[Dict[str, Any]],
    *,
    template_wp: Dict[str, Any],
    anchor_coord: Optional[Dict[str, Any]],
    waypoint_id_provider: Optional[Callable[[], int]] = None,
) -> List[Dict[str, Any]]:
    if not waypoints:
        return waypoints
    anchor = _normalize_coordinate(anchor_coord) if isinstance(anchor_coord, dict) else None
    if anchor is None:
        return waypoints
    first_coord = _extract_lah_waypoint_coordinate(waypoints[0])
    if _same_normalized_coordinate(first_coord, anchor):
        return waypoints
    anchored = [
        _build_lah_anchor_waypoint(
            template_wp,
            coord=anchor,
            next_id=_to_int((waypoints[0] or {}).get("waypointID")) or 0,
            hovering_time=0,
            waypoint_id=int(waypoint_id_provider()) if waypoint_id_provider is not None else None,
        )
    ]
    anchored.extend(deepcopy(waypoints))
    relink_waypoints(anchored)
    return anchored


def _lah_resume_transition_altitude(
    *,
    attack_coord: Optional[Dict[str, Any]],
    resume_target_coord: Optional[Dict[str, Any]],
    special_resume_coord: Optional[Dict[str, Any]],
) -> int:
    if special_resume_coord is not None:
        special_alt = (
            _normalize_altitude_value((resume_target_coord or {}).get("altitude"))
            or _normalize_altitude_value((special_resume_coord or {}).get("altitude"))
        )
        if special_alt is not None:
            return int(special_alt)
    return int(
        _normalize_altitude_value((attack_coord or {}).get("altitude"))
        or _normalize_altitude_value((resume_target_coord or {}).get("altitude"))
        or 800
    )


def _coordinate_local_xy_m(
    coord: Dict[str, Any],
    origin: Dict[str, Any],
) -> Optional[Tuple[float, float]]:
    coord_norm = _normalize_coordinate(coord)
    origin_norm = _normalize_coordinate(origin)
    if coord_norm is None or origin_norm is None:
        return None
    origin_lat = float(origin_norm["latitude"])
    lat_scale = 111_132.0
    lon_scale = 111_320.0 * max(math.cos(math.radians(origin_lat)), 0.01)
    return (
        (float(coord_norm["longitude"]) - float(origin_norm["longitude"])) * lon_scale,
        (float(coord_norm["latitude"]) - float(origin_norm["latitude"])) * lat_scale,
    )


def _predict_lah_followup_anchor(
    anchor_coord: Optional[Dict[str, Any]],
    state: Dict[str, Any],
    *,
    enable_prediction: bool,
) -> Optional[Dict[str, Any]]:
    anchor = _normalize_coordinate(anchor_coord) if isinstance(anchor_coord, dict) else None
    if anchor is None or not enable_prediction:
        return anchor

    velocity = (state or {}).get("velocity") if isinstance(state, dict) else {}
    heading = _to_float((state or {}).get("heading"))
    if heading is None and isinstance(velocity, dict):
        heading = _to_float(velocity.get("heading"))
    speed_mps = _to_float((state or {}).get("speed"))
    if speed_mps is None and isinstance(velocity, dict):
        speed_mps = _to_float(velocity.get("speed"))
    if heading is None:
        return anchor
    if speed_mps is None or speed_mps <= 0:
        return anchor

    lookahead_s = max(0.0, get_runtime_attack_float("lah_followup_trim_lookahead_s", 7.0))
    max_distance_m = max(0.0, get_runtime_attack_float("lah_followup_trim_max_lookahead_m", 400.0))
    distance_m = float(speed_mps) * lookahead_s
    if max_distance_m > 0.0:
        distance_m = min(distance_m, max_distance_m)
    if distance_m <= 0.0:
        return anchor

    projected = _project_coordinate(anchor, heading, distance_m)
    if projected is None:
        return anchor
    projected["altitude"] = anchor.get("altitude")
    return _normalize_coordinate(projected) or anchor


def _predict_replan_resume_anchor(
    anchor_coord: Optional[Dict[str, Any]],
    state: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    anchor = _normalize_coordinate(anchor_coord) if isinstance(anchor_coord, dict) else None
    if anchor is None:
        return None

    velocity = (state or {}).get("velocity") if isinstance(state, dict) else {}
    heading = _to_float((state or {}).get("heading"))
    if heading is None and isinstance(velocity, dict):
        heading = _to_float(velocity.get("heading"))
    speed_mps = _to_float((state or {}).get("speed"))
    if speed_mps is None and isinstance(velocity, dict):
        speed_mps = _to_float(velocity.get("speed"))
    if heading is None or speed_mps is None or speed_mps <= 0:
        return anchor

    lookahead_s = max(0.0, get_runtime_attack_float("replan_start_trim_lookahead_s", 8.0))
    max_distance_m = max(0.0, get_runtime_attack_float("replan_start_trim_max_lookahead_m", 600.0))
    distance_m = float(speed_mps) * lookahead_s
    if max_distance_m > 0.0:
        distance_m = min(distance_m, max_distance_m)
    if distance_m <= 0.0:
        return anchor

    projected = _project_coordinate(anchor, heading, distance_m)
    if projected is None:
        return anchor
    projected["altitude"] = anchor.get("altitude")
    return _normalize_coordinate(projected) or anchor


def _waypoint_keep_start_index_before_anchor(
    waypoints: List[Dict[str, Any]],
    anchor_coord: Dict[str, Any],
    *,
    radius_m: float,
    projection_snap_m: float,
) -> int:
    valid_points: List[Tuple[int, Tuple[float, float]]] = []
    for idx, waypoint in enumerate(waypoints):
        wp_coord = _extract_lah_waypoint_coordinate(waypoint)
        if wp_coord is None:
            continue
        local_xy = _coordinate_local_xy_m(wp_coord, anchor_coord)
        if local_xy is not None:
            valid_points.append((idx, local_xy))

    if len(valid_points) < 2:
        return 0

    projection_keep = 0
    best: Optional[Tuple[float, int, float]] = None
    for local_idx in range(len(valid_points) - 1):
        _, start_xy = valid_points[local_idx]
        _, end_xy = valid_points[local_idx + 1]
        seg_x = end_xy[0] - start_xy[0]
        seg_y = end_xy[1] - start_xy[1]
        seg_len_sq = seg_x * seg_x + seg_y * seg_y
        if seg_len_sq <= 1e-6:
            t = 0.0
        else:
            t = max(0.0, min(1.0, -((start_xy[0] * seg_x + start_xy[1] * seg_y) / seg_len_sq)))
        proj_x = start_xy[0] + t * seg_x
        proj_y = start_xy[1] + t * seg_y
        dist_sq = proj_x * proj_x + proj_y * proj_y
        if best is None or dist_sq < best[0]:
            best = (dist_sq, local_idx, t)

    if best is not None:
        dist_sq, local_idx, t = best
        if math.sqrt(float(dist_sq)) <= max(float(projection_snap_m), 0.0):
            if local_idx >= len(valid_points) - 2 and t >= 0.85:
                projection_keep = int(valid_points[-1][0])
            elif t <= 0.15:
                projection_keep = int(valid_points[local_idx][0])
            else:
                projection_keep = int(valid_points[min(local_idx + 1, len(valid_points) - 1)][0])

    near_keep = 0
    if radius_m > 0.0:
        scan_until = max(1, projection_keep + 1)
        for idx, waypoint in enumerate(waypoints[:scan_until]):
            wp_coord = _extract_lah_waypoint_coordinate(waypoint)
            if wp_coord is None:
                continue
            dist_m = _haversine_distance_m(anchor_coord, wp_coord)
            if isinstance(dist_m, (int, float)) and float(dist_m) <= float(radius_m):
                near_keep = idx + 1
                continue
            if projection_keep <= 0 or idx >= projection_keep:
                break

    return max(0, min(max(projection_keep, near_keep), len(waypoints) - 1))


def _trim_waypoints_before_replan_anchor(
    waypoints: List[Dict[str, Any]],
    anchor_coord: Optional[Dict[str, Any]],
    *,
    emit: Callable[[str], None],
    log_prefix: str,
    aircraft_id: int,
    path_id: Optional[int],
    reassign_ids: bool,
) -> Tuple[List[Dict[str, Any]], int]:
    anchor = _normalize_coordinate(anchor_coord) if isinstance(anchor_coord, dict) else None
    source_waypoints = [deepcopy(item) for item in (waypoints or []) if isinstance(item, dict)]
    if anchor is None or len(source_waypoints) < 2:
        return source_waypoints, 0

    radius_m = max(0.0, get_runtime_attack_float("replan_start_trim_radius_m", 250.0))
    projection_snap_m = max(
        radius_m * 2.0,
        get_runtime_attack_float("replan_start_trim_projection_snap_m", 600.0),
    )
    keep_start_idx = _waypoint_keep_start_index_before_anchor(
        source_waypoints,
        anchor,
        radius_m=radius_m,
        projection_snap_m=projection_snap_m,
    )
    keep_start_idx = max(0, min(int(keep_start_idx), len(source_waypoints) - 1))
    if keep_start_idx <= 0:
        return source_waypoints, 0

    suffix = [deepcopy(item) for item in source_waypoints[keep_start_idx:]]
    for waypoint in suffix:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    if reassign_ids and suffix:
        reassign_unique_waypoint_ids_inplace(suffix)
    elif suffix:
        relink_waypoints(suffix)

    emit(
        f"{log_prefix} Trimmed stale start waypoint(s) before replan anchor "
        f"(aircraft={aircraft_id}, pathID={path_id}, removedWaypoints={keep_start_idx}, "
        f"firstWP={_to_int((suffix[0] or {}).get('waypointID')) if suffix else None})."
    )
    return suffix, keep_start_idx


def _trim_lah_waypoints_before_anchor(
    waypoints: List[Dict[str, Any]],
    anchor_coord: Optional[Dict[str, Any]],
    *,
    emit: Callable[[str], None],
    log_prefix: str,
    aircraft_id: int,
    path_id: Optional[int],
) -> Tuple[List[Dict[str, Any]], int]:
    return _trim_waypoints_before_replan_anchor(
        waypoints,
        anchor_coord,
        emit=emit,
        log_prefix=log_prefix,
        aircraft_id=aircraft_id,
        path_id=path_id,
        reassign_ids=True,
    )


def _trim_lah_resume_waypoints_after_attack_anchor(
    resume_waypoints: List[Dict[str, Any]],
    *,
    attack_anchor_coord: Optional[Dict[str, Any]],
    source_fp_data: Dict[str, Any],
    aircraft_id: int,
    path_id: Optional[int],
    emit: Callable[[str], None],
    log_prefix: str,
    removed_wp_id: Optional[int] = None,
    waypoint_id_provider: Optional[Callable[[], int]] = None,
) -> Tuple[List[Dict[str, Any]], Optional[int]]:
    anchor = _normalize_coordinate(attack_anchor_coord) if isinstance(attack_anchor_coord, dict) else None
    source_waypoints = [deepcopy(item) for item in (source_fp_data.get("lahWaypointList") or []) if isinstance(item, dict)]
    resume = [deepcopy(item) for item in (resume_waypoints or []) if isinstance(item, dict)]
    if anchor is None or len(resume) < 2:
        return resume, removed_wp_id
    preserve_length_m = _long_two_point_lah_resume_length_m(resume)
    if preserve_length_m is not None:
        emit(
            f"{log_prefix} Preserved long two-point LAH resume path; attack-anchor trim skipped "
            f"(aircraft={aircraft_id}, pathLength={float(preserve_length_m):.1f}m, "
            f"threshold={_lah_resume_two_point_preserve_threshold_m():.1f}m)."
        )
        return resume, removed_wp_id

    before_trim = [deepcopy(item) for item in resume]
    trimmed, removed_count = _trim_waypoints_before_replan_anchor(
        resume,
        anchor,
        emit=emit,
        log_prefix=log_prefix,
        aircraft_id=int(aircraft_id),
        path_id=path_id,
        reassign_ids=False,
    )
    if removed_count > 0:
        removed_prefix = before_trim[:removed_count]
        new_removed_wp_id = _to_int((removed_prefix[-1] or {}).get("waypointID")) if removed_prefix else None
        if trimmed:
            reassign_unique_waypoint_ids_inplace(
                trimmed,
                waypoint_id_provider=waypoint_id_provider,
            )
        emit(
            f"{log_prefix} Resume path trimmed against attack anchor "
            f"(aircraft={aircraft_id}, removedWaypoints={removed_count})."
        )
        return trimmed, new_removed_wp_id if new_removed_wp_id is not None else removed_wp_id

    source_first_coord = _extract_lah_waypoint_coordinate(source_waypoints[0]) if source_waypoints else None
    first_resume_coord = _extract_lah_waypoint_coordinate(resume[0]) if resume else None
    if source_first_coord is None or first_resume_coord is None:
        return resume, removed_wp_id

    same_start_tolerance_m = max(
        0.0,
        get_runtime_attack_float("lah_resume_stale_start_same_tolerance_m", 100.0),
    )
    first_is_source_start_dist = _haversine_distance_m(source_first_coord, first_resume_coord)
    if first_is_source_start_dist is None or float(first_is_source_start_dist) > same_start_tolerance_m:
        return resume, removed_wp_id

    first_dist = _haversine_distance_m(anchor, first_resume_coord)
    stale_guard_m = max(
        0.0,
        get_runtime_attack_float("lah_resume_stale_start_guard_m", 1000.0),
    )
    if first_dist is None or float(first_dist) <= stale_guard_m:
        return resume, removed_wp_id

    best_idx: Optional[int] = None
    best_dist: Optional[float] = None
    for idx, waypoint in enumerate(resume[1:], start=1):
        coord = _extract_lah_waypoint_coordinate(waypoint)
        if coord is None:
            continue
        distance_m = _haversine_distance_m(anchor, coord)
        if not isinstance(distance_m, (int, float)):
            continue
        if best_dist is None or float(distance_m) < float(best_dist):
            best_idx = int(idx)
            best_dist = float(distance_m)

    if best_idx is None or best_dist is None or best_idx <= 0:
        return resume, removed_wp_id

    improvement_ratio = max(
        0.05,
        min(0.95, get_runtime_attack_float("lah_resume_stale_start_improvement_ratio", 0.65)),
    )
    improvement_m = max(
        0.0,
        get_runtime_attack_float("lah_resume_stale_start_improvement_m", 300.0),
    )
    if not (float(best_dist) <= float(first_dist) * improvement_ratio or float(first_dist) - float(best_dist) >= improvement_m):
        return resume, removed_wp_id

    removed_prefix = [deepcopy(item) for item in resume[:best_idx]]
    guarded = [deepcopy(item) for item in resume[best_idx:]]
    if guarded:
        reassign_unique_waypoint_ids_inplace(
            guarded,
            waypoint_id_provider=waypoint_id_provider,
        )
    new_removed_wp_id = _to_int((removed_prefix[-1] or {}).get("waypointID")) if removed_prefix else None
    emit(
        f"{log_prefix} Dropped stale LAH resume start after attack anchor "
        f"(aircraft={aircraft_id}, removedWaypoints={best_idx}, "
        f"firstDistanceM={float(first_dist):.1f}, keptDistanceM={float(best_dist):.1f})."
    )
    return guarded, new_removed_wp_id if new_removed_wp_id is not None else removed_wp_id


def _trim_lah_follow_up_paths_after_anchor(
    *,
    follow_up_missions: List[Dict[str, Any]],
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]],
    current_input_id: Optional[int],
    anchor_coord: Optional[Dict[str, Any]],
    state: Dict[str, Any],
    emit: Callable[[str], None],
    log_prefix: str,
    predict_anchor: bool,
) -> int:
    if not follow_up_missions or not follow_up_paths or current_input_id is None:
        return 0
    trim_anchor = _predict_lah_followup_anchor(
        anchor_coord,
        state,
        enable_prediction=bool(predict_anchor),
    )
    if trim_anchor is None:
        return 0

    payload_by_path_id: Dict[int, Dict[str, Any]] = {}
    for _, payload in follow_up_paths:
        path_id = _to_int((payload or {}).get("pathID")) if isinstance(payload, dict) else None
        if path_id is not None:
            payload_by_path_id[int(path_id)] = payload

    for mission in follow_up_missions:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        path_id = _to_int(mission.get("pathID"))
        if path_id is None:
            continue
        payload = payload_by_path_id.get(int(path_id))
        if not isinstance(payload, dict):
            continue
        waypoints = payload.get("lahWaypointList")
        if not isinstance(waypoints, list) or not waypoints:
            continue

        trimmed_waypoints, removed_count = _trim_waypoints_before_replan_anchor(
            waypoints,
            trim_anchor,
            emit=emit,
            log_prefix=log_prefix,
            aircraft_id=_to_int(payload.get("aircraftID")) or 0,
            path_id=int(path_id),
            reassign_ids=True,
        )
        payload["lahWaypointList"] = trimmed_waypoints
        mission_info = mission.get("individualMissionInfo")
        if isinstance(mission_info, dict):
            mission["individualMissionInfo"] = deepcopy(mission_info)
            mission["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(trimmed_waypoints)
        return removed_count

    return 0


def _waypoints_to_coordinate_list(
    waypoints: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    coordinates: List[Dict[str, Any]] = []
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        coord = _normalize_coordinate(waypoint.get("coordinate"))
        if coord is not None:
            coordinates.append(coord)
    return coordinates


def _trim_uav_follow_up_paths_after_anchor(
    *,
    follow_up_missions: List[Dict[str, Any]],
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]],
    current_input_id: Optional[int],
    anchor_coord: Optional[Dict[str, Any]],
    emit: Callable[[str], None],
    log_prefix: str,
) -> int:
    if not follow_up_missions or not follow_up_paths or current_input_id is None:
        return 0
    anchor = _normalize_coordinate(anchor_coord) if isinstance(anchor_coord, dict) else None
    if anchor is None:
        return 0

    payload_by_path_id: Dict[int, Dict[str, Any]] = {}
    for _, payload in follow_up_paths:
        path_id = _to_int((payload or {}).get("pathID")) if isinstance(payload, dict) else None
        if path_id is not None:
            payload_by_path_id[int(path_id)] = payload

    for mission in follow_up_missions:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        path_id = _to_int(mission.get("pathID"))
        if path_id is None:
            continue
        payload = payload_by_path_id.get(int(path_id))
        if not isinstance(payload, dict):
            continue
        waypoints = payload.get("waypointList")
        if not isinstance(waypoints, list) or not waypoints:
            continue

        trimmed_waypoints, removed_count = _trim_waypoints_before_replan_anchor(
            waypoints,
            anchor,
            emit=emit,
            log_prefix=log_prefix,
            aircraft_id=_to_int(payload.get("aircraftID")) or 0,
            path_id=int(path_id),
            reassign_ids=True,
        )
        if removed_count <= 0:
            return 0
        payload["waypointList"] = trimmed_waypoints
        if "uavWaypointList" in payload:
            payload["uavWaypointList"] = deepcopy(trimmed_waypoints)
        mission_info = mission.get("individualMissionInfo")
        if isinstance(mission_info, dict):
            if _sync_resume_mission_info_with_waypoints(mission, trimmed_waypoints):
                emit(
                    f"{log_prefix} Follow-up missionInfo synced with trimmed lineSearch "
                    f"(pathID={path_id})."
                )
            else:
                mission["individualMissionInfo"] = deepcopy(mission_info)
                mission["individualMissionInfo"]["coordinateList"] = _waypoints_to_coordinate_list(trimmed_waypoints)
        return removed_count

    return 0


def _build_lah_hold_coordinate_near_resume(
    *,
    resume_waypoints: List[Dict[str, Any]],
    current_coord: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    first_resume = _extract_lah_waypoint_coordinate(resume_waypoints[0]) if resume_waypoints else None
    if first_resume is None:
        return _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None

    second_resume = (
        _extract_lah_waypoint_coordinate(resume_waypoints[1])
        if len(resume_waypoints) >= 2
        else None
    )

    base_bearing: Optional[float] = None
    if second_resume is not None:
        base_bearing = _bearing_between(
            float(first_resume["latitude"]),
            float(first_resume["longitude"]),
            float(second_resume["latitude"]),
            float(second_resume["longitude"]),
        )
    elif isinstance(current_coord, dict):
        current_norm = _normalize_coordinate(current_coord)
        if current_norm is not None:
            base_bearing = _bearing_between(
                float(current_norm["latitude"]),
                float(current_norm["longitude"]),
                float(first_resume["latitude"]),
                float(first_resume["longitude"]),
            )

    if base_bearing is None:
        base_bearing = 90.0

    hold_coord = dict(first_resume)
    projected = _project_coordinate(
        first_resume,
        (float(base_bearing) + 90.0) % 360.0,
        get_runtime_attack_float("lah_hold_near_resume_offset_m", 30.0),
    )
    if projected:
        hold_coord.update(
            {
                "latitude": projected.get("latitude", hold_coord.get("latitude")),
                "longitude": projected.get("longitude", hold_coord.get("longitude")),
            }
        )
    hold_coord["altitude"] = first_resume.get("altitude")
    return _normalize_coordinate(hold_coord)


def _is_lah_line_or_hold_standby_mission(mission: Dict[str, Any]) -> bool:
    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo") if isinstance(mission.get("individualMissionInfo"), dict) else {}
    line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
    if line_list:
        return True
    try:
        mission_type = int(info.get("individualMissionType", 0) or 0)
    except Exception:
        mission_type = 0
    if mission_type == 6:
        return True
    if mission_type == 9:
        coords = info.get("coordinateList") if isinstance(info.get("coordinateList"), list) else []
        return bool(coords)
    return False


def _input_mission_payload_is_line(mission: Dict[str, Any]) -> bool:
    if not isinstance(mission, dict):
        return False
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    if line_list:
        return True
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    coordinate_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []
    mission_type = _to_int(mission.get("inputMissionType"))
    return bool(mission_type == 1 and not area_list and len(coordinate_list) >= 2)


def _source_input_mission_is_line(
    *,
    source_plan_id: Optional[int],
    input_mission_id: Optional[int],
) -> bool:
    source_id = _to_int(source_plan_id)
    input_id = _to_int(input_mission_id)
    if source_id is None or input_id is None:
        return False
    input_data = _load_input_plan_for_source_plan(int(source_id))
    if not isinstance(input_data, dict):
        return False
    for mission in input_data.get("inputMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _to_int(mission.get("inputMissionID")) == int(input_id):
            return _input_mission_payload_is_line(mission)
    return False


def _build_lah_standby_hold_coordinate_from_path_end(
    *,
    target_mission: Dict[str, Any],
    fp_data: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    if not _is_lah_line_or_hold_standby_mission(target_mission):
        return None
    return _normalize_coordinate(_extract_final_lah_coordinate(fp_data))


def _split_done_resume_lah_path(
    source_fp_data: Dict[str, Any],
    *,
    artifacts: Any,
    current_coord: Optional[Dict[str, Any]],
    emit: Callable[[str], None],
    force_nonempty_resume: bool = False,
    exclude_current_from_resume: bool = False,
    resume_trim_anchor_coord: Optional[Dict[str, Any]] = None,
    waypoint_id_provider: Optional[Callable[[], int]] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], Optional[int]]:
    waypoints = list(source_fp_data.get("lahWaypointList") or [])
    done_waypoints: List[Dict[str, Any]] = []
    resume_waypoints: List[Dict[str, Any]] = []
    removed_wp_id: Optional[int] = None

    if not waypoints:
        return done_waypoints, resume_waypoints, removed_wp_id

    curr_wp = _to_int(getattr(artifacts, "current_waypoint_id", None))
    prev_wp = _to_int(getattr(artifacts, "previous_waypoint_id", None))
    if curr_wp is None:
        inferred_wp = _infer_lah_current_waypoint_id(waypoints, current_coord)
        if inferred_wp is not None:
            curr_wp = inferred_wp
            emit(f"[ATTACK][LAH] Inferred current waypoint from position -> {curr_wp}.")

    curr_idx = next(
        (idx for idx, waypoint in enumerate(waypoints) if _to_int(waypoint.get("waypointID")) == curr_wp),
        None,
    )

    if curr_idx is not None:
        done_waypoints = deepcopy(waypoints[:curr_idx]) if curr_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[curr_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_wp_id = prev_wp

        done_prefix_idx = 0
        while done_prefix_idx < len(waypoints) and bool(waypoints[done_prefix_idx].get("isDone")):
            done_prefix_idx += 1
        if done_prefix_idx != curr_idx:
            emit(
                "[ATTACK][LAH] isDone/currentWP mismatch; "
                f"using currentWP split (isDonePrefix={done_prefix_idx}, currentIdx={curr_idx})."
            )
    elif any(bool(waypoint.get("isDone")) for waypoint in waypoints):
        idx = 0
        while idx < len(waypoints) and bool((waypoints[idx] or {}).get("isDone")):
            idx += 1
        done_waypoints = deepcopy(waypoints[:idx]) if idx > 0 else []
        resume_waypoints = deepcopy(waypoints[idx:]) if idx > 0 else deepcopy(waypoints)
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
    else:
        resume_waypoints = deepcopy(waypoints)
        removed_wp_id = prev_wp

    if force_nonempty_resume and not resume_waypoints and waypoints:
        forced_start_idx: Optional[int] = None
        if prev_wp is not None:
            for idx, waypoint in enumerate(waypoints):
                if _to_int(waypoint.get("waypointID")) == prev_wp:
                    forced_start_idx = min(idx + 1, len(waypoints) - 1)
                    break
        if forced_start_idx is None:
            for idx, waypoint in enumerate(waypoints):
                if not bool(waypoint.get("isDone")):
                    forced_start_idx = idx
                    break
        if forced_start_idx is None:
            forced_start_idx = len(waypoints) - 1
        done_waypoints = deepcopy(waypoints[:forced_start_idx]) if forced_start_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[forced_start_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_wp_id = prev_wp
        emit(
            "[ATTACK][LAH] Resume fallback applied "
            f"(forcedStartWP={_to_int((resume_waypoints[0] or {}).get('waypointID'))})."
        )

    if exclude_current_from_resume and resume_waypoints:
        first_resume_wp_id = _to_int((resume_waypoints[0] or {}).get("waypointID"))
        if curr_wp is not None and first_resume_wp_id == curr_wp:
            preserve_length_m = _long_two_point_lah_resume_length_m(resume_waypoints)
            if preserve_length_m is not None:
                emit(
                    "[ATTACK][LAH] Preserved current waypoint in long two-point resume path "
                    f"(currentWP={curr_wp}, pathLength={float(preserve_length_m):.1f}m, "
                    f"threshold={_lah_resume_two_point_preserve_threshold_m():.1f}m)."
                )
            else:
                removed_wp_id = int(curr_wp)
                resume_waypoints = deepcopy(resume_waypoints[1:]) if len(resume_waypoints) > 1 else []
                emit(
                    "[ATTACK][LAH] Dropped current waypoint from resume path "
                    f"(currentWP={curr_wp}, nextWP={_to_int((resume_waypoints[0] or {}).get('waypointID')) if resume_waypoints else None})."
                )

    if resume_waypoints and resume_trim_anchor_coord is not None:
        trim_anchor_started = time.perf_counter()
        resume_before_trim = [deepcopy(item) for item in resume_waypoints if isinstance(item, dict)]
        resume_waypoints, stale_removed = _trim_waypoints_before_replan_anchor(
            resume_waypoints,
            resume_trim_anchor_coord,
            emit=emit,
            log_prefix="[ATTACK][LAH]",
            aircraft_id=_to_int(source_fp_data.get("aircraftID")) or 0,
            path_id=_to_int(source_fp_data.get("pathID")),
            reassign_ids=False,
        )
        if stale_removed > 0:
            removed_prefix = resume_before_trim[:stale_removed]
            done_waypoints.extend(deepcopy(removed_prefix))
            removed_wp_id = _to_int((removed_prefix[-1] or {}).get("waypointID"))

    template_wp = deepcopy((waypoints or [None])[0]) if waypoints else _default_lah_waypoint_template()
    if not done_waypoints:
        current_anchor = _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None
        if current_anchor is not None:
            done_waypoints = [
                _build_lah_anchor_waypoint(
                    template_wp,
                    coord=current_anchor,
                    next_id=0,
                    hovering_time=0,
                    waypoint_id=int(waypoint_id_provider()) if waypoint_id_provider is not None else None,
                )
            ]
    else:
        _append_lah_done_anchor(
            done_waypoints,
            template_wp=template_wp,
            anchor_coord=current_coord,
            waypoint_id_provider=waypoint_id_provider,
        )

    for waypoint in done_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = True
    for waypoint in resume_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False

    if done_waypoints:
        reassign_unique_waypoint_ids_inplace(
            done_waypoints,
            waypoint_id_provider=waypoint_id_provider,
        )
    if resume_waypoints:
        reassign_unique_waypoint_ids_inplace(
            resume_waypoints,
            waypoint_id_provider=waypoint_id_provider,
        )
    return done_waypoints, resume_waypoints, removed_wp_id


def _split_done_resume_path(
    source_fp_data: Dict[str, Any],
    *,
    artifacts: Any,
    sweep_progress: Dict[int, Dict[str, Any]] | None,
    emit: Callable[[str], None],
    force_nonempty_resume: bool = False,
    append_replan_anchor: bool = False,
    replan_coordinate: Optional[Dict[str, Any]] = None,
    resume_trim_anchor_coord: Optional[Dict[str, Any]] = None,
    waypoint_id_provider: Optional[Callable[[], int]] = None,
    timing: Optional[Dict[str, Any]] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], Optional[int]]:
    split_started_total = time.perf_counter()
    waypoints = list(source_fp_data.get("waypointList") or [])
    done_waypoints: List[Dict[str, Any]] = []
    resume_waypoints: List[Dict[str, Any]] = []
    removed_wp_id: Optional[int] = None

    def _record_split_stage(name: str, started_at: float, **extra: Any) -> None:
        if timing is None:
            return
        row: Dict[str, Any] = {"elapsedMs": _elapsed_ms_detail(started_at)}
        if extra:
            row.update(_json_safe(extra))
        timing[str(name)] = row

    if not waypoints:
        if timing is not None:
            timing["totalMs"] = _elapsed_ms_detail(split_started_total)
        return done_waypoints, resume_waypoints, removed_wp_id

    initial_split_started = time.perf_counter()
    curr_wp = _to_int(getattr(artifacts, "current_waypoint_id", None))
    prev_wp = _to_int(getattr(artifacts, "previous_waypoint_id", None))
    curr_idx = next(
        (i for i, wp in enumerate(waypoints) if _to_int(wp.get("waypointID")) == curr_wp),
        None,
    )

    # Prefer current waypoint based split whenever available:
    # [done ...] + [current ... remaining]
    if curr_idx is not None:
        done_waypoints = deepcopy(waypoints[:curr_idx]) if curr_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[curr_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp:
            removed_wp_id = prev_wp

        # Diagnostic when waypoint isDone flags disagree with current waypoint progress.
        done_prefix_idx = 0
        while done_prefix_idx < len(waypoints) and bool(waypoints[done_prefix_idx].get("isDone")):
            done_prefix_idx += 1
        if done_prefix_idx != curr_idx:
            emit(
                "[ATTACK][UAV] isDone/currentWP mismatch; "
                f"using currentWP split (isDonePrefix={done_prefix_idx}, currentIdx={curr_idx})."
            )
        if removed_wp_id is not None:
            emit(f"[ATTACK][UAV] Resume trimmed by currentWP (lastRemovedWP={removed_wp_id}).")
    elif any(bool(wp.get("isDone")) for wp in waypoints):
        idx = 0
        while idx < len(waypoints) and bool(waypoints[idx].get("isDone")):
            idx += 1
        done_waypoints = deepcopy(waypoints[:idx]) if idx > 0 else []
        resume_waypoints = deepcopy(waypoints[idx:]) if idx > 0 else deepcopy(waypoints)
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        if removed_wp_id is not None and not (force_nonempty_resume and idx >= len(waypoints)):
            emit(f"[ATTACK][UAV] Resume trimmed by isDone (lastRemovedWP={removed_wp_id}).")
    else:
        done_waypoints = []
        resume_waypoints = deepcopy(waypoints)
        removed_wp_id = prev_wp

    if force_nonempty_resume and not resume_waypoints and waypoints:
        forced_start_idx: Optional[int] = None

        # 1) Prefer a split after previous waypoint when available.
        if prev_wp is not None:
            for idx, waypoint in enumerate(waypoints):
                if _to_int(waypoint.get("waypointID")) == prev_wp:
                    forced_start_idx = min(idx + 1, len(waypoints) - 1)
                    break

        # 2) Otherwise keep the first not-done waypoint (if any).
        if forced_start_idx is None:
            for idx, waypoint in enumerate(waypoints):
                if not bool(waypoint.get("isDone")):
                    forced_start_idx = idx
                    break

        # 3) Last fallback: keep the terminal waypoint as resume anchor.
        if forced_start_idx is None:
            forced_start_idx = len(waypoints) - 1

        done_waypoints = deepcopy(waypoints[:forced_start_idx]) if forced_start_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[forced_start_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_wp_id = prev_wp
        emit(
            "[ATTACK][UAV] Resume fallback applied "
            f"(forcedStartWP={_to_int((resume_waypoints[0] or {}).get('waypointID'))})."
        )
    _record_split_stage(
        "split_current_progress",
        initial_split_started,
        waypointCount=len(waypoints),
        currentWaypointID=curr_wp,
        previousWaypointID=prev_wp,
        currentIndex=curr_idx,
        doneWaypointCount=len(done_waypoints),
        resumeWaypointCount=len(resume_waypoints),
        removedWaypointID=removed_wp_id,
    )

    if resume_waypoints and resume_trim_anchor_coord is not None:
        trim_anchor_started = time.perf_counter()
        resume_before_trim = [deepcopy(item) for item in resume_waypoints if isinstance(item, dict)]
        resume_waypoints, stale_removed = _trim_waypoints_before_replan_anchor(
            resume_waypoints,
            resume_trim_anchor_coord,
            emit=emit,
            log_prefix="[ATTACK][UAV]",
            aircraft_id=_to_int(source_fp_data.get("aircraftID")) or 0,
            path_id=_to_int(source_fp_data.get("pathID")),
            reassign_ids=False,
        )
        if stale_removed > 0:
            removed_prefix = resume_before_trim[:stale_removed]
            done_waypoints.extend(deepcopy(removed_prefix))
            removed_wp_id = _to_int((removed_prefix[-1] or {}).get("waypointID"))
        _record_split_stage(
            "trim_waypoints_before_replan_anchor",
            trim_anchor_started,
            staleRemoved=stale_removed,
            doneWaypointCount=len(done_waypoints),
            resumeWaypointCount=len(resume_waypoints),
        )
    else:
        _record_split_stage(
            "trim_waypoints_before_replan_anchor",
            time.perf_counter(),
            skipped=True,
            hasResumeWaypoints=bool(resume_waypoints),
            hasResumeTrimAnchor=resume_trim_anchor_coord is not None,
        )

    done_count_started = time.perf_counter()
    done_sweep_points = count_sweep_points_in_waypoints(done_waypoints)
    _record_split_stage(
        "count_done_sweep_points",
        done_count_started,
        doneSweepPoints=done_sweep_points,
        doneWaypointCount=len(done_waypoints),
    )

    if append_replan_anchor and done_waypoints and resume_waypoints:
        append_anchor_started = time.perf_counter()
        anchor_added = False
        anchor_coord_src = replan_coordinate if isinstance(replan_coordinate, dict) else {}
        anchor_lat = _to_float(anchor_coord_src.get("latitude"))
        anchor_lon = _to_float(anchor_coord_src.get("longitude"))
        anchor_alt = _normalize_altitude_value(anchor_coord_src.get("altitude"))
        if anchor_alt is None:
            anchor_alt = _normalize_altitude_value((done_waypoints[-1].get("coordinate") or {}).get("altitude"))
        if anchor_alt is None:
            anchor_alt = _normalize_altitude_value((resume_waypoints[0].get("coordinate") or {}).get("altitude"))
        if anchor_alt is None:
            anchor_alt = 0

        if anchor_lat is not None and anchor_lon is not None:
            prev_coord = done_waypoints[-1].get("coordinate") if isinstance(done_waypoints[-1], dict) else {}
            prev_lat = _to_float((prev_coord or {}).get("latitude")) if isinstance(prev_coord, dict) else None
            prev_lon = _to_float((prev_coord or {}).get("longitude")) if isinstance(prev_coord, dict) else None
            prev_alt = _normalize_altitude_value((prev_coord or {}).get("altitude")) if isinstance(prev_coord, dict) else None
            same_as_prev = (
                prev_lat is not None
                and prev_lon is not None
                and abs(prev_lat - anchor_lat) <= 1e-7
                and abs(prev_lon - anchor_lon) <= 1e-7
                and (prev_alt or 0) == anchor_alt
            )
            if not same_as_prev:
                template_wp = done_waypoints[-1] if isinstance(done_waypoints[-1], dict) else {}
                anchor_wp: Dict[str, Any] = {
                    "waypointID": int(waypoint_id_provider()) if waypoint_id_provider is not None else _reserve_waypoint_block(1),
                    "coordinate": {
                        "latitude": anchor_lat,
                        "longitude": anchor_lon,
                        "altitude": anchor_alt,
                    },
                    "speed": template_wp.get("speed", 30.0),
                    "eta": template_wp.get("eta", 0),
                    "ecf": template_wp.get("ecf", 0.0),
                    "nextWaypointID": 0,
                }
                if "waypointPassType" in template_wp:
                    anchor_wp["waypointPassType"] = template_wp.get("waypointPassType")
                if "filmingProperty" in template_wp:
                    template_fp = template_wp.get("filmingProperty")
                    if isinstance(template_fp, dict) and template_fp:
                        anchor_fp = deepcopy(template_fp)
                        coord_orient = anchor_fp.get("coordinateOrientation")
                        if isinstance(coord_orient, dict):
                            coord_orient = dict(coord_orient)
                            coord_orient["coordinate"] = {
                                "latitude": anchor_lat,
                                "longitude": anchor_lon,
                                "altitude": anchor_alt,
                            }
                            anchor_fp["coordinateOrientation"] = coord_orient
                        anchor_wp["filmingProperty"] = anchor_fp
                    else:
                        anchor_fov_deg = get_runtime_effective_fov_deg("entry_hold_fov_deg", 10.0)
                        anchor_wp["filmingProperty"] = {
                            "fieldOfView": float(anchor_fov_deg),
                            "sensorType": 1,
                            "operationMode": 1,
                            "coordinateOrientation": {
                                "coordinate": {
                                    "latitude": anchor_lat,
                                    "longitude": anchor_lon,
                                    "altitude": anchor_alt,
                                }
                            },
                        }
                if "loiterProperty" in template_wp:
                    template_loiter = template_wp.get("loiterProperty")
                    if isinstance(template_loiter, dict) and template_loiter:
                        anchor_wp["loiterProperty"] = deepcopy(template_loiter)
                if "hovering" in template_wp:
                    anchor_wp["hovering"] = {"time": 0}
                if "loiter" in template_wp:
                    anchor_wp["loiter"] = {"radius": 0, "direction": 0, "time": 0, "speed": 0}
                if "attack" in template_wp:
                    anchor_wp["attack"] = {"targetID": 0, "weaponType": 0}

                done_waypoints.append(anchor_wp)
                anchor_added = True
                emit(
                    "[ATTACK][UAV] Added replan anchor waypoint to done path "
                    f"(anchorWP={anchor_wp.get('waypointID')})."
                )
        _record_split_stage(
            "append_replan_anchor",
            append_anchor_started,
            anchorAdded=bool(anchor_added),
            doneWaypointCount=len(done_waypoints),
            resumeWaypointCount=len(resume_waypoints),
        )
    else:
        _record_split_stage(
            "append_replan_anchor",
            time.perf_counter(),
            skipped=True,
            appendRequested=bool(append_replan_anchor),
            doneWaypointCount=len(done_waypoints),
            resumeWaypointCount=len(resume_waypoints),
        )

    cut_started = time.perf_counter()
    progress_entry = None
    if sweep_progress and artifacts.path_id is not None:
        progress_entry = sweep_progress.get(int(artifacts.path_id))
    resume_offset_reference_coord = (
        _normalize_coordinate(replan_coordinate)
        or _normalize_coordinate(resume_trim_anchor_coord)
    )
    raw_cut_points = sweep_cut_points(
        progress_entry,
        default_buffer_seconds=DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
    )
    cut_points = max(0, int(raw_cut_points) - int(done_sweep_points))
    _record_split_stage(
        "resolve_sweep_cut_points",
        cut_started,
        pathID=artifacts.path_id,
        hasProgressEntry=isinstance(progress_entry, dict),
        rawCutPoints=raw_cut_points,
        doneSweepPoints=done_sweep_points,
        cutPoints=cut_points,
    )
    resume_count_started = time.perf_counter()
    total_resume_sweep_points = count_sweep_points_in_waypoints(resume_waypoints)
    _record_split_stage(
        "count_resume_sweep_points",
        resume_count_started,
        totalResumeSweepPoints=total_resume_sweep_points,
        resumeWaypointCount=len(resume_waypoints),
    )
    if (
        force_nonempty_resume
        and cut_points > 0
        and total_resume_sweep_points > 0
        and cut_points >= total_resume_sweep_points
        and isinstance(progress_entry, dict)
    ):
        progress_points_raw = estimate_sweep_buffer_points(
            progress_entry,
            DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
        )
        progress_cut_points = max(0, int(progress_points_raw or 0) - int(done_sweep_points))
        if 0 < progress_cut_points < total_resume_sweep_points:
            emit(
                "[ATTACK][UAV] Resume sweep trim fallback applied "
                f"(rawCutPoints={raw_cut_points}, estimatedCutPoints={progress_points_raw}, "
                f"usingCutPoints={progress_cut_points}, pathID={artifacts.path_id})."
            )
            raw_cut_points = int(progress_points_raw or raw_cut_points)
            cut_points = progress_cut_points
    if cut_points > 0 and resume_waypoints:
        trim_sweep_started = time.perf_counter()
        resume_waypoints, removed_points = trim_waypoints_by_sweep_points(
            resume_waypoints,
            cut_points,
            preserve_waypoints=True,
            reference_coord_for_offset=resume_offset_reference_coord,
        )
        _record_split_stage(
            "trim_waypoints_by_sweep_points",
            trim_sweep_started,
            requestedCutPoints=cut_points,
            removedPoints=removed_points,
            resumeWaypointCount=len(resume_waypoints),
        )
        if removed_points > 0:
            emit(
                f"[ATTACK][UAV] Resume sweep trim applied "
                f"(cutPoints={removed_points}, rawCutPoints={raw_cut_points}, "
                f"doneSweepPoints={done_sweep_points}, pathID={artifacts.path_id})."
            )
    else:
        _record_split_stage(
            "trim_waypoints_by_sweep_points",
            time.perf_counter(),
            skipped=True,
            requestedCutPoints=cut_points,
            resumeWaypointCount=len(resume_waypoints),
        )
    if resume_waypoints:
        merge_started = time.perf_counter()
        resume_waypoints, merged_groups = merge_small_adjacent_line_search_waypoints(
            resume_waypoints,
            max_sweeps=2,
            reference_coord_for_offset=resume_offset_reference_coord,
        )
        _record_split_stage(
            "merge_small_adjacent_line_search_waypoints",
            merge_started,
            mergedWaypoints=merged_groups,
            resumeWaypointCount=len(resume_waypoints),
        )
        if merged_groups > 0:
            emit(
                "[ATTACK][UAV] Resume lineSearch tail groups merged "
                f"(mergedWaypoints={merged_groups})."
            )
    else:
        _record_split_stage(
            "merge_small_adjacent_line_search_waypoints",
            time.perf_counter(),
            skipped=True,
            resumeWaypointCount=0,
        )

    for wp in done_waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = True
    for wp in resume_waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = False

    if done_waypoints:
        reassign_done_started = time.perf_counter()
        reassign_unique_waypoint_ids_inplace(
            done_waypoints,
            waypoint_id_provider=waypoint_id_provider,
        )
        _record_split_stage(
            "reassign_done_waypoint_ids",
            reassign_done_started,
            doneWaypointCount=len(done_waypoints),
        )
    if resume_waypoints:
        capture_started = time.perf_counter()
        _apply_resume_capture_buffer(
            resume_waypoints,
            emit=emit,
            log_prefix="[ATTACK][UAV]",
        )
        _record_split_stage(
            "apply_resume_capture_buffer",
            capture_started,
            resumeWaypointCount=len(resume_waypoints),
        )
        resume_reference_coord = (
            _normalize_coordinate(replan_coordinate)
            or _normalize_coordinate(resume_trim_anchor_coord)
            or _normalize_coordinate((resume_waypoints[0] or {}).get("coordinate"))
        )
        realign_started = time.perf_counter()
        reanchored = realign_line_search_waypoints_to_first_sweep(
            resume_waypoints,
            reference_coord_for_offset=resume_reference_coord,
        )
        _record_split_stage(
            "realign_line_search_waypoints_to_first_sweep",
            realign_started,
            reanchoredWaypoints=reanchored,
            resumeWaypointCount=len(resume_waypoints),
        )
        if reanchored > 0:
            emit(
                "[ATTACK][UAV] Resume lineSearch anchors reoriented from UAV entry "
                f"(waypoints={reanchored})."
            )
        preserve_alt_started = time.perf_counter()
        altitude_preserved = preserve_first_waypoint_altitude_from_reference(resume_waypoints, resume_reference_coord)
        _record_split_stage(
            "preserve_first_waypoint_altitude",
            preserve_alt_started,
            preserved=bool(altitude_preserved),
        )
        if altitude_preserved:
            emit("[ATTACK][UAV] Resume first waypoint altitude preserved from current UAV.")
        search_speed_weight = get_runtime_float("search_speed_weight", 1.1)
        recompute_started = time.perf_counter()
        recomputed = recompute_line_search_speed_from_geometry(
            resume_waypoints,
            first_reference_coord=resume_reference_coord,
            speed_scale=search_speed_weight,
            only_increase=True,
        )
        _record_split_stage(
            "recompute_line_search_speed_from_geometry",
            recompute_started,
            weight=float(search_speed_weight),
            recomputedWaypoints=recomputed,
        )
        if recomputed > 0:
            emit(
                "[ATTACK][UAV] Resume searchSpeed geometry recomputed "
                f"(weight={float(search_speed_weight):.2f}, waypoints={recomputed})."
            )
        resume_speed_scale = get_runtime_attack_float("resume_search_speed_scale", 1.3)
        scale_started = time.perf_counter()
        scaled = scale_line_search_speed(resume_waypoints, resume_speed_scale)
        _record_split_stage(
            "scale_line_search_speed",
            scale_started,
            factor=float(resume_speed_scale),
            scaledWaypoints=scaled,
        )
        if scaled > 0:
            emit(
                f"[ATTACK][UAV] Resume searchSpeed scaled "
                f"(factor={resume_speed_scale:.2f}, waypoints={scaled})."
            )
        reassign_resume_started = time.perf_counter()
        reassign_unique_waypoint_ids_inplace(
            resume_waypoints,
            waypoint_id_provider=waypoint_id_provider,
        )
        _record_split_stage(
            "reassign_resume_waypoint_ids",
            reassign_resume_started,
            resumeWaypointCount=len(resume_waypoints),
        )
        _mark_resume_waypoints_not_done(resume_waypoints)
    elif timing is not None:
        timing["resume_finalize"] = {"elapsedMs": 0.0, "skipped": True, "resumeWaypointCount": 0}
    if timing is not None:
        timing["totalMs"] = _elapsed_ms_detail(split_started_total)
    return done_waypoints, resume_waypoints, removed_wp_id


def _resolve_path_start_waypoint_id(path_id: Optional[int]) -> Optional[int]:
    pid = _to_int(path_id)
    if pid is None:
        return None
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{pid}.json")
        data = read_json_cached(path, copy_result=False, kind="FlightPath")
    except Exception:
        return None
    waypoints = []
    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        lst = data.get(key)
        if isinstance(lst, list):
            waypoints = lst
            break
    if not waypoints:
        return None
    fallback: Optional[int] = None
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        wid = _to_int(wp.get("waypointID"))
        if wid is None:
            continue
        if fallback is None:
            fallback = wid
        if not bool(wp.get("isDone")):
            return wid
    return fallback


def _build_uav_attack_tracking_package(
    *,
    descriptor: Dict[str, Any],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission_template: Dict[str, Any],
    target_index: Optional[int],
    attack_coord: Dict[str, Any],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
    force_start_at_current: bool = False,
    tracking_eta_s: Optional[int] = None,
    sweep_progress: Dict[int, Dict[str, Any]] | None = None,
    done_input_ids: Optional[set[int]] = None,
    collaborative_resume: Optional[CollaborativeResumeReplanResult] = None,
    id_reservation: AttackIdReservation | None = None,
) -> Optional[Dict[str, Any]]:
    if id_reservation is None:
        raise RuntimeError("AttackIdReservation is required for UAV tracking builder")
    if target_index is None:
        emit("[ATTACK][UAV] Target mission index unavailable; skipping UAV override.")
        return None
    agent_coord = _normalize_coordinate(state.get("coordinate"))
    if not agent_coord:
        emit(f"[ATTACK][UAV] Coordinate missing for aircraft {descriptor['aircraft_id']}.")
        return None

    target_coord_norm = _normalize_coordinate(descriptor.get("target_coord") or attack_coord)
    if not target_coord_norm:
        emit(f"[ATTACK][UAV] Target coordinate unavailable for aircraft {descriptor['aircraft_id']}.")
        return None
    if target_coord_norm.get("altitude") is None:
        target_coord_norm["altitude"] = _normalize_altitude_value(agent_coord.get("altitude")) or 700
    target_sensor_coord = deepcopy(target_coord_norm)
    tracking_flight_coord = deepcopy(target_coord_norm)
    tracking_flight_altitude = _resolve_uav_tracking_flight_altitude(
        agent_coord=agent_coord,
        fp_data=fp_data,
        artifacts=artifacts,
    )
    target_altitude = _normalize_altitude_value(tracking_flight_coord.get("altitude"))
    if (
        target_altitude is None
        or float(target_altitude) < float(_UAV_TRACKING_MIN_FLIGHT_ALTITUDE_M)
        or float(target_altitude) > float(_UAV_TRACKING_MAX_FLIGHT_ALTITUDE_M)
    ):
        tracking_flight_coord["altitude"] = int(tracking_flight_altitude)
        emit(
            "[ATTACK][UAV] Tracking target altitude kept for sensor only; "
            f"flight altitude replaced {target_altitude if target_altitude is not None else 'n/a'}m "
            f"-> {int(tracking_flight_altitude)}m."
        )
    replan_resume_anchor = _predict_replan_resume_anchor(agent_coord, state)

    builder_started_total = time.perf_counter()
    builder_timing: Dict[str, Any] = {}

    def _record_builder_stage(name: str, started_at: float, **extra: Any) -> None:
        row: Dict[str, Any] = {"elapsedMs": _elapsed_ms_detail(started_at)}
        if extra:
            row.update(_json_safe(extra))
        builder_timing[str(name)] = row

    allocate_started = time.perf_counter()
    include_done_reference_mission = False
    path_alloc_started = time.perf_counter()
    if include_done_reference_mission:
        done_path_id, attack_path_id, resume_path_id = id_reservation.next_paths(descriptor["aircraft_id"], 3)
    else:
        done_path_id = None
        attack_path_id, resume_path_id = id_reservation.next_paths(descriptor["aircraft_id"], 2)
    _record_builder_stage(
        "allocate_path_ids",
        path_alloc_started,
        donePathID=done_path_id,
        attackPathID=attack_path_id,
        resumePathID=resume_path_id,
    )
    individual_alloc_started = time.perf_counter()
    tracking_individual_id, resume_individual_id = id_reservation.next_individuals(2)
    _record_builder_stage(
        "allocate_individual_ids",
        individual_alloc_started,
        trackingIndividualID=tracking_individual_id,
        resumeIndividualID=resume_individual_id,
    )
    waypoint_alloc_started = time.perf_counter()
    target_wp_id = id_reservation.next_waypoint()
    _record_builder_stage("allocate_waypoint_ids", waypoint_alloc_started, targetWaypointID=target_wp_id)
    _record_builder_stage(
        "allocate_ids",
        allocate_started,
        donePathID=done_path_id,
        attackPathID=attack_path_id,
        resumePathID=resume_path_id,
        trackingIndividualID=tracking_individual_id,
        resumeIndividualID=resume_individual_id,
        targetWaypointID=target_wp_id,
    )
    tracking_target_id = _to_int(descriptor.get("target_id"))
    if tracking_target_id is None:
        detail = ctx.get("replan_detail") if isinstance(ctx, dict) else {}
        if isinstance(detail, dict):
            tracking_target_id = _to_int(detail.get("targetID") or detail.get("targetId"))
            if tracking_target_id is None:
                orient = detail.get("targetOrientation") or {}
                if isinstance(orient, dict):
                    tracking_target_id = _to_int(orient.get("targetID") or orient.get("targetId"))
    tracking_target_id_value = tracking_target_id if tracking_target_id is not None else 0

    original_entry = deepcopy(target_mission_template)
    base_rel_block = dict(original_entry.get("relatedMission") or {})
    input_mission_id = _to_int(base_rel_block.get("inputMissionID")) or _to_int((ctx.get("mission_ids") or [None])[0]) or 0
    prior_mission_id = _to_int(base_rel_block.get("priorMissionID")) or 0
    attack_reason = ctx.get("reason")

    tracking_rel = dict(base_rel_block)
    tracking_rel_type = _to_int(base_rel_block.get("relatedMissionType")) or 1
    if tracking_rel_type not in (1, 2):
        tracking_rel_type = 1
    tracking_rel["relatedMissionType"] = tracking_rel_type
    tracking_rel["inputMissionID"] = input_mission_id
    tracking_rel["priorMissionID"] = prior_mission_id
    tracking_rel["attackReason"] = attack_reason
    tracking_rel["targetID"] = tracking_target_id_value

    resume_rel = dict(base_rel_block)
    resume_rel["relatedMissionType"] = base_rel_block.get("relatedMissionType", 1)
    resume_rel["inputMissionID"] = input_mission_id
    resume_rel["priorMissionID"] = prior_mission_id

    mission_attack = {
        "individualMissionID": tracking_individual_id,
        "isDone": False,
        "relatedMission": tracking_rel,
        "individualMissionInfo": {
            "individualMissionType": 1,
            "patternType": 1,
            "autoZoomIn": True,
            "coordinateList": [
                {
                    "latitude": tracking_flight_coord["latitude"],
                    "longitude": tracking_flight_coord["longitude"],
                    "altitude": tracking_flight_coord["altitude"],
                },
            ],
            "lineList": [],
            "areaList": [],
            "targetID": tracking_target_id_value,
        },
        "pathID": attack_path_id,
    }

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = resume_rel
    mission_resume["isDone"] = False
    source_waypoints = list(fp_data.get("waypointList") or [])
    source_single_point = len(source_waypoints) <= 1
    split_started = time.perf_counter()
    split_timing: Dict[str, Any] = {}
    if source_single_point:
        done_waypoints = deepcopy(source_waypoints)
        for wp in done_waypoints:
            if isinstance(wp, dict):
                wp["isDone"] = True
        resume_waypoints = []
        removed_wp_id = _to_int((done_waypoints[-1] or {}).get("waypointID")) if done_waypoints else None
        emit(
            "[ATTACK][UAV] Source path has a single waypoint; "
            "preserving it as done and skipping done/resume split."
        )
    else:
        done_waypoints, resume_waypoints, removed_wp_id = _split_done_resume_path(
            fp_data,
            artifacts=artifacts,
            sweep_progress=sweep_progress,
            emit=emit,
            force_nonempty_resume=True,
            append_replan_anchor=True,
            replan_coordinate=agent_coord,
            resume_trim_anchor_coord=replan_resume_anchor,
            waypoint_id_provider=id_reservation.next_waypoint,
            timing=split_timing,
        )
    _record_builder_stage(
        "split_done_resume",
        split_started,
        sourceSinglePoint=bool(source_single_point),
        doneWaypointCount=len(done_waypoints or []),
        resumeWaypointCount=len(resume_waypoints or []),
        removedWaypointID=removed_wp_id,
        detail=split_timing,
    )

    preserved_individual_id = _to_int(original_entry.get("individualMissionID"))
    mission_done: Optional[Dict[str, Any]] = None
    done_fp_data: Optional[Dict[str, Any]] = None
    if done_path_id is not None:
        done_fp_data = deepcopy(fp_data)
        done_fp_data["pathID"] = int(done_path_id)
        done_fp_data["timestamp"] = now_ms
        done_fp_data["Source"] = done_fp_data.get("Source") or "MMR"
        done_fp_data["aircraftID"] = descriptor["aircraft_id"]
    if done_fp_data is not None and preserved_individual_id is not None:
        done_fp_data["individualMissionID"] = preserved_individual_id
        if done_waypoints:
            mission_done = _build_done_reference_mission(
                original_entry,
                path_id=int(done_path_id),
                done_waypoints=done_waypoints,
            )
            mission_done["individualMissionID"] = preserved_individual_id
        done_fp_data["waypointList"] = done_waypoints
    elif done_waypoints:
        emit(
            "[ATTACK][UAV] Done-reference mission skipped for tracking branch "
            f"(aircraft={descriptor['aircraft_id']}, removedWaypointID={removed_wp_id})."
        )

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["waypointList"] = resume_waypoints

    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = resume_fp_data.get("Source") or "MMR"
    resume_fp_data["aircraftID"] = descriptor["aircraft_id"]
    resume_fp_data["individualMissionID"] = resume_individual_id

    has_resume = bool(resume_waypoints)
    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    effective_done_input_ids = (
        done_input_ids
        if done_input_ids is not None
        else _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    )
    source_mission_list = imp_data.get("individualMissionList")
    if (
        isinstance(source_mission_list, list)
        and target_index is not None
        and 0 <= target_index < len(source_mission_list)
    ):
        clone_started = time.perf_counter()
        follow_up_artifacts = _collect_attack_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][UAV]",
            current_input_id=input_mission_id,
            excluded_input_ids=effective_done_input_ids,
            individual_id_provider=id_reservation.next_individual,
            path_id_provider=id_reservation.next_path,
            waypoint_id_provider=id_reservation.next_waypoint,
        )
        if follow_up_artifacts is None:
            return None
        follow_up_missions, follow_up_paths, follow_up_stats = follow_up_artifacts
        _record_builder_stage(
            "clone_followups",
            clone_started,
            followUpMissionCount=len(follow_up_missions),
            followUpPathCount=len(follow_up_paths),
            preservedFollowUpCount=follow_up_stats.get("preservedCount"),
            clonedFollowUpCount=follow_up_stats.get("clonedCount"),
            skippedFollowUpCount=follow_up_stats.get("skippedCount"),
        )

    _trim_uav_follow_up_paths_after_anchor(
        follow_up_missions=follow_up_missions,
        follow_up_paths=follow_up_paths,
        current_input_id=input_mission_id,
        anchor_coord=replan_resume_anchor,
        emit=emit,
        log_prefix="[ATTACK][UAV]",
    )

    target_eta = int(tracking_eta_s) if isinstance(tracking_eta_s, int) and tracking_eta_s >= 0 else 30
    target_loiter_time = target_eta
    if collaborative_resume is not None:
        release_started = time.perf_counter()
        release_end_coord = _extract_final_uav_coordinate(fp_data)
        release_start_coord = _normalize_coordinate(tracking_flight_coord)
        if release_end_coord is None and resume_waypoints:
            release_end_coord = _normalize_coordinate((resume_waypoints[-1] or {}).get("coordinate"))
        if release_start_coord is not None and release_end_coord is not None:
            release_waypoints, release_speed_mps = _build_uav_release_resume_waypoints(
                start_coord=release_start_coord,
                end_coord=release_end_coord,
                release_eta_s=int(target_eta),
                target_finish_eta_s=int(collaborative_resume.finish_eta_s),
                default_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
                min_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
                max_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
                force_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
            )
            if release_waypoints:
                resume_waypoints = release_waypoints
                resume_fp_data["waypointList"] = resume_waypoints
                _apply_release_resume_mission_info(
                    mission_resume,
                    start_coord=release_start_coord,
                    end_coord=release_end_coord,
                )
                has_resume = True
                emit(
                    "[ATTACK][COLLAB] Tracking UAV resume replaced with release transit "
                    f"(aircraft={descriptor['aircraft_id']}, targetFinishEta={collaborative_resume.finish_eta_s}, "
                    f"speed={release_speed_mps:.1f}m/s)."
                )
        _record_builder_stage(
            "release_resume",
            release_started,
            releaseWaypointCount=len(resume_waypoints or []),
            targetFinishEtaS=int(collaborative_resume.finish_eta_s),
        )

    if resume_waypoints:
        _mark_resume_waypoints_not_done(resume_waypoints)
        resume_fp_data["waypointList"] = resume_waypoints

    if has_resume and _sync_resume_mission_info_with_waypoints(mission_resume, resume_waypoints):
        emit(
            "[ATTACK][UAV] Resume missionInfo synced with trimmed lineSearch "
            f"(aircraft={descriptor['aircraft_id']}, pathID={resume_path_id})."
        )

    tracking_fov_deg = get_runtime_effective_fov_deg("global_manual_fov_deg", 5.0)
    target_wp = {
        "waypointID": target_wp_id,
        "coordinate": {
            "latitude": tracking_flight_coord["latitude"],
            "longitude": tracking_flight_coord["longitude"],
            "altitude": tracking_flight_coord["altitude"],
        },
        "speed": 30.0,
        "eta": target_eta,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": 2,
        "filmingProperty": {
            "fieldOfView": float(tracking_fov_deg),
            "sensorType": 1,
            "operationMode": 3,
            "coordinateOrientation": {
                "coordinate": {
                    "latitude": target_sensor_coord["latitude"],
                    "longitude": target_sensor_coord["longitude"],
                    "altitude": target_sensor_coord["altitude"],
                }
            },
        },
    }
    # Keep the loiter payload present whenever the pass type is Loiter.
    # Some attack handoff paths intentionally use 0s dwell, and missing this block
    # causes downstream validators/SIM normalization to treat it as a default 30s hold.
    target_wp["loiterProperty"] = {
        "radius": 400,
        "direction": 1,
        "time": int(max(0, target_loiter_time)),
        "speed": 30,
    }
    if tracking_target_id is not None:
        filming = target_wp.get("filmingProperty") or {}
        filming["autoTracking"] = {"targetID": tracking_target_id}
        if "coordinateOrientation" in filming:
            del filming["coordinateOrientation"]
        target_wp["filmingProperty"] = filming

    tracking_fp_data = {
        "timestamp": now_ms,
        "Source": fp_data.get("Source") or "MMR",
        "pathID": attack_path_id,
        "aircraftID": descriptor["aircraft_id"],
        "individualMissionID": tracking_individual_id,
        "isFormationFlight": fp_data.get("isFormationFlight", False),
        "waypointList": [target_wp],
    }

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    write_done_path = False
    write_resume_path = False
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    if 0 <= target_index < len(mission_list):
        prefix = [deepcopy(mission) for mission in mission_list[:target_index] if isinstance(mission, dict)]
        rebuilt = list(prefix)
        if mission_done is not None:
            rebuilt.append(mission_done)
            write_done_path = True
        rebuilt.append(mission_attack)
        if has_resume:
            rebuilt.append(mission_resume)
            write_resume_path = True
        rebuilt.extend(follow_up_missions)
        mission_list[:] = rebuilt
        emit(
            f"[ATTACK][UAV] Preserved completed prefix, inserted tracking, and reattached "
            f"{len(follow_up_missions)} follow-up mission(s)."
        )
    else:
        mission_list.insert(0, mission_attack)
        if has_resume:
            mission_list.insert(1, mission_resume)
            write_resume_path = True
        emit("[ATTACK][UAV] Target mission index invalid; appended tracking branch at head.")

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    done_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{done_path_id}.json") if write_done_path else None
    )
    tracking_fp_dest = db_paths.get_db_subpath("FlightPath", f"{attack_path_id}.json")
    resume_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json") if write_resume_path else None
    )
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [(imp_dest, imp_data)]
    if done_fp_dest is not None:
        write_entries.append((done_fp_dest, done_fp_data))
    write_entries.append((tracking_fp_dest, tracking_fp_data))
    if resume_fp_dest is not None:
        write_entries.append((resume_fp_dest, resume_fp_data))
    write_entries.extend((dest, payload) for dest, payload in follow_up_paths)
    _validate_generated_artifact_write_entries(
        scope=f"attack_uav_tracking:{new_imp_id}",
        individual_mission_plans=[imp_data],
        entries=write_entries,
        log=emit,
    )
    write_started = time.perf_counter()
    write_results = _write_json_files_batch(write_entries)
    _record_builder_stage(
        "write_json",
        write_started,
        fileCount=len(write_results),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
    )

    if resume_fp_dest is not None:
        emit(
            f"[ATTACK][UAV] Generated tracking/resume missions -> "
            f"IMP:{imp_dest.name} PATHS:{tracking_fp_dest.name}/{resume_fp_dest.name} "
            f"(followUps={len(follow_up_missions)})"
        )
    else:
        emit(
            f"[ATTACK][UAV] Generated tracking-only mission -> "
            f"IMP:{imp_dest.name} PATH:{tracking_fp_dest.name} "
            f"(followUps={len(follow_up_missions)}, resumeSkipped=true)"
        )

    result: Dict[str, Any] = {
        "aircraft_id": descriptor["aircraft_id"],
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "tracking": {
            "individualMissionID": tracking_individual_id,
            "pathID": attack_path_id,
            "targetWaypointID": target_wp_id,
        },
        "removedWaypointID": removed_wp_id,
        "donePath": str(done_fp_dest) if done_fp_dest is not None else None,
        "trackingPath": str(tracking_fp_dest),
    }
    if write_resume_path and resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
        result["resumePath"] = str(resume_fp_dest)
    result["followUpMissionCount"] = len(follow_up_missions)
    builder_timing["totalMs"] = _elapsed_ms_detail(builder_started_total)
    result["timingMs"] = builder_timing
    return result


def _build_uav_attack_resume_package(
    *,
    descriptor: Dict[str, Any],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission_template: Dict[str, Any],
    target_index: Optional[int],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
    sweep_progress: Dict[int, Dict[str, Any]] | None = None,
    done_input_ids: Optional[set[int]] = None,
    id_reservation: AttackIdReservation | None = None,
) -> Optional[Dict[str, Any]]:
    if id_reservation is None:
        raise RuntimeError("AttackIdReservation is required for UAV resume builder")
    if target_index is None:
        emit("[ATTACK][UAV] Target mission index unavailable; skipping UAV resume.")
        return None

    done_path_id, resume_path_id = id_reservation.next_paths(descriptor["aircraft_id"], 2)
    [resume_individual_id] = id_reservation.next_individuals(1)

    original_entry = deepcopy(target_mission_template)
    base_rel_block = dict(original_entry.get("relatedMission") or {})
    input_mission_id = (
        _to_int(base_rel_block.get("inputMissionID"))
        or _to_int((ctx.get("mission_ids") or [None])[0])
        or 0
    )
    prior_mission_id = _to_int(base_rel_block.get("priorMissionID")) or 0

    resume_rel = dict(base_rel_block)
    resume_rel["relatedMissionType"] = base_rel_block.get("relatedMissionType", 1)
    resume_rel["inputMissionID"] = input_mission_id
    resume_rel["priorMissionID"] = prior_mission_id

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = resume_rel
    mission_resume["isDone"] = False

    replan_coord = _normalize_coordinate((state or {}).get("coordinate")) or {}
    replan_resume_anchor = _predict_replan_resume_anchor(replan_coord, state)

    split_started = time.perf_counter()
    split_timing: Dict[str, Any] = {}
    done_waypoints, resume_waypoints, removed_wp_id = _split_done_resume_path(
        fp_data,
        artifacts=artifacts,
        sweep_progress=sweep_progress,
        emit=emit,
        append_replan_anchor=True,
        replan_coordinate=replan_coord,
        resume_trim_anchor_coord=replan_resume_anchor,
        waypoint_id_provider=id_reservation.next_waypoint,
        timing=split_timing,
    )
    split_timing_summary = {
        "elapsedMs": _elapsed_ms_detail(split_started),
        "doneWaypointCount": len(done_waypoints or []),
        "resumeWaypointCount": len(resume_waypoints or []),
        "removedWaypointID": removed_wp_id,
        "detail": split_timing,
    }

    preserved_individual_id = _to_int(original_entry.get("individualMissionID"))
    mission_done: Optional[Dict[str, Any]] = None
    done_fp_data = deepcopy(fp_data)
    done_fp_data["pathID"] = done_path_id
    done_fp_data["timestamp"] = now_ms
    done_fp_data["Source"] = done_fp_data.get("Source") or "MMR"
    done_fp_data["aircraftID"] = descriptor["aircraft_id"]
    if preserved_individual_id is not None:
        done_fp_data["individualMissionID"] = preserved_individual_id
        if done_waypoints:
            mission_done = _build_done_reference_mission(
                original_entry,
                path_id=int(done_path_id),
                done_waypoints=done_waypoints,
            )
            mission_done["individualMissionID"] = preserved_individual_id
    done_fp_data["waypointList"] = done_waypoints

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["waypointList"] = resume_waypoints

    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = resume_fp_data.get("Source") or "MMR"
    resume_fp_data["aircraftID"] = descriptor["aircraft_id"]
    resume_fp_data["individualMissionID"] = resume_individual_id

    has_resume = bool(resume_waypoints)
    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    effective_done_input_ids = (
        done_input_ids
        if done_input_ids is not None
        else _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    )
    source_mission_list = imp_data.get("individualMissionList")
    if isinstance(source_mission_list, list) and 0 <= target_index < len(source_mission_list):
        follow_up_artifacts = _collect_attack_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][UAV]",
            current_input_id=input_mission_id,
            excluded_input_ids=effective_done_input_ids,
            individual_id_provider=id_reservation.next_individual,
            path_id_provider=id_reservation.next_path,
            waypoint_id_provider=id_reservation.next_waypoint,
        )
        if follow_up_artifacts is None:
            return None
        follow_up_missions, follow_up_paths, _follow_up_stats = follow_up_artifacts

    _trim_uav_follow_up_paths_after_anchor(
        follow_up_missions=follow_up_missions,
        follow_up_paths=follow_up_paths,
        current_input_id=input_mission_id,
        anchor_coord=replan_resume_anchor,
        emit=emit,
        log_prefix="[ATTACK][UAV]",
    )

    if resume_waypoints:
        _mark_resume_waypoints_not_done(resume_waypoints)
        resume_fp_data["waypointList"] = resume_waypoints

    if has_resume and _sync_resume_mission_info_with_waypoints(mission_resume, resume_waypoints):
        emit(
            "[ATTACK][UAV] Resume missionInfo synced with trimmed lineSearch "
            f"(aircraft={descriptor['aircraft_id']}, pathID={resume_path_id})."
        )

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    write_done_path = False
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    if 0 <= target_index < len(mission_list):
        prefix = [deepcopy(mission) for mission in mission_list[:target_index] if isinstance(mission, dict)]
        rebuilt = list(prefix)
        if mission_done is not None:
            rebuilt.append(mission_done)
            write_done_path = True
        if has_resume:
            rebuilt.append(mission_resume)
        else:
            emit(
                "[ATTACK][UAV] Resume path empty after trimming; "
                "skipping current resume mission and keeping follow-up missions only."
            )
        rebuilt.extend(follow_up_missions)
        mission_list[:] = rebuilt
        if has_resume:
            emit(
                "[ATTACK][UAV] Preserved completed prefix, inserted resume, "
                f"and reattached {len(follow_up_missions)} follow-up mission(s)."
            )
        else:
            emit(
                "[ATTACK][UAV] Preserved completed prefix, skipped empty resume, "
                f"and reattached {len(follow_up_missions)} follow-up mission(s)."
            )
    else:
        if has_resume:
            mission_list.insert(0, mission_resume)
            emit("[ATTACK][UAV] Target mission index invalid; appended resume at head.")
        else:
            emit(
                "[ATTACK][UAV] Target mission index invalid and resume path empty; "
                "skipping resume insertion."
            )

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    done_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{done_path_id}.json") if write_done_path else None
    )
    resume_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json") if has_resume else None
    )
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [(imp_dest, imp_data)]
    if resume_fp_dest is not None:
        write_entries.append((resume_fp_dest, resume_fp_data))
    if done_fp_dest is not None:
        write_entries.append((done_fp_dest, done_fp_data))
    write_entries.extend((dest, payload) for dest, payload in follow_up_paths)
    _validate_generated_artifact_write_entries(
        scope=f"attack_uav_resume:{new_imp_id}",
        individual_mission_plans=[imp_data],
        entries=write_entries,
        log=emit,
    )
    write_started = time.perf_counter()
    write_results = _write_json_files_batch(write_entries)
    write_timing = {
        "elapsedMs": _elapsed_ms_detail(write_started),
        "fileCount": len(write_results),
        "writtenCount": sum(1 for row in write_results if row.get("written")),
        "skippedCount": sum(1 for row in write_results if row.get("skipped")),
    }

    if resume_fp_dest is not None:
        emit(
            f"[ATTACK][UAV] Generated resume-only mission -> "
            f"IMP:{imp_dest.name} PATH:{resume_fp_dest.name} "
            f"(followUps={len(follow_up_missions)})"
        )
    else:
        emit(
            f"[ATTACK][UAV] Generated follow-up-only mission -> "
            f"IMP:{imp_dest.name} (followUps={len(follow_up_missions)}, resumeSkipped=true)"
        )

    result = {
        "aircraft_id": descriptor["aircraft_id"],
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "removedWaypointID": removed_wp_id,
        "donePath": str(done_fp_dest) if done_fp_dest is not None else None,
        "resumePath": str(resume_fp_dest) if resume_fp_dest is not None else None,
        "followUpMissionCount": len(follow_up_missions),
        "timingMs": {"split_done_resume": split_timing_summary, "write_json": write_timing},
    }
    if resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
    return result


def _build_lah_attack_sequence_package(
    *,
    descriptor: Dict[str, Any],
    assigned_targets: List[Dict[str, Any]],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission: Dict[str, Any],
    target_index: Optional[int],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    aircraft_id: int,
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
    done_input_ids: Optional[set[int]] = None,
    id_reservation: AttackIdReservation | None = None,
) -> Optional[Dict[str, Any]]:
    if id_reservation is None:
        raise RuntimeError("AttackIdReservation is required for LAH attack sequence builder")
    valid_targets = [
        dict(item)
        for item in assigned_targets
        if isinstance(item, dict) and _normalize_coordinate(item.get("attack_coord") or item.get("coordinate"))
    ]
    if not valid_targets:
        emit(f"[ATTACK][LAH] No valid assigned targets for aircraft {aircraft_id}.")
        return None
    if len(valid_targets) == 1:
        single_target = dict(valid_targets[0])
        single_descriptor = dict(descriptor)
        single_descriptor["target_id"] = _to_int(single_target.get("target_id") or single_target.get("targetID"))
        single_descriptor["target_type"] = _to_int(single_target.get("target_type") or single_target.get("targetType"))
        single_target_coord_source = single_target.get("attack_coord") or single_target.get("coordinate")
        single_target_coord = _normalize_coordinate(single_target_coord_source)
        single_descriptor["target_coord"] = _attach_attack_point_metadata(
            single_target_coord,
            single_target_coord_source,
        )
        single_descriptor["_lah_special_target_region_attack"] = bool(
            single_target.get("_lah_special_target_region_attack")
        )
        if single_target.get("selected_weapon_type") is not None:
            ctx["_selected_attack_weapon_type"] = _to_int(single_target.get("selected_weapon_type"))
        if isinstance(single_target.get("weapon_choice"), dict):
            ctx["_selected_attack_weapon_choice"] = dict(single_target.get("weapon_choice") or {})
        return _build_lah_attack_package(
            descriptor=single_descriptor,
            new_imp_id=new_imp_id,
            imp_data=imp_data,
            fp_data=fp_data,
            target_mission=target_mission,
            target_index=target_index,
            attack_coord=single_descriptor.get("target_coord") or {},
            ctx=ctx,
            state=state,
            aircraft_id=aircraft_id,
            artifacts=artifacts,
            emit=emit,
            now_ms=now_ms,
            done_input_ids=done_input_ids,
            id_reservation=id_reservation,
        )

    if target_index is None:
        emit(f"[ATTACK][LAH] Target mission index unavailable for aircraft {aircraft_id}.")
        return None

    current_coord = _normalize_coordinate(state.get("coordinate"))
    if not current_coord:
        emit(f"[ATTACK][LAH] Coordinate missing for aircraft {aircraft_id}.")
        return None
    heading = _to_float(state.get("heading"))
    if heading is None:
        emit(f"[ATTACK][LAH] Heading missing for aircraft {aircraft_id}; defaulting to north.")
        heading = 0.0

    template_wp = (
        deepcopy((fp_data.get("lahWaypointList") or [None])[0])
        if fp_data.get("lahWaypointList")
        else _default_lah_waypoint_template()
    )
    replan_resume_anchor = _predict_replan_resume_anchor(current_coord, state)
    _, resume_waypoints, removed_wp_id = _split_done_resume_lah_path(
        fp_data,
        artifacts=artifacts,
        current_coord=current_coord,
        emit=emit,
        force_nonempty_resume=True,
        exclude_current_from_resume=True,
        resume_trim_anchor_coord=replan_resume_anchor,
        waypoint_id_provider=id_reservation.next_waypoint,
    )
    original_entry = deepcopy(target_mission)
    rel_info = dict(target_mission.get("relatedMission") or {})
    input_mission_id = _to_int(rel_info.get("inputMissionID")) or _to_int((ctx.get("mission_ids") or [None])[0]) or 0
    prior_mission_id = _to_int(rel_info.get("priorMissionID")) or 0
    related_template = {
        "relatedMissionType": 1,
        "inputMissionID": input_mission_id,
        "priorMissionID": prior_mission_id,
    }
    special_lah_attack_coord = _lah_special_battle_anchor_for_input(
        ctx,
        aircraft_id=int(aircraft_id),
        input_mission_id=input_mission_id,
    )
    special_resume_coord = dict(special_lah_attack_coord) if special_lah_attack_coord is not None else None
    if special_lah_attack_coord is not None:
        emit(
            "[ATTACK][LAH] current phase forces battle-position attack/resume anchor "
            f"(aircraft={int(aircraft_id)}, inputMissionID={int(input_mission_id)})."
        )

    builder_started_total = time.perf_counter()
    builder_timing: Dict[str, Any] = {}

    def _record_builder_stage(name: str, started_at: float, **extra: Any) -> None:
        row: Dict[str, Any] = {"elapsedMs": _elapsed_ms_detail(started_at)}
        if extra:
            row.update(_json_safe(extra))
        builder_timing[str(name)] = row

    path_alloc_started = time.perf_counter()
    attack_path_ids = list(id_reservation.next_paths(aircraft_id, len(valid_targets)))
    [resume_path_id] = id_reservation.next_paths(aircraft_id, 1)
    _record_builder_stage(
        "allocate_path_ids",
        path_alloc_started,
        attackPathIDs=[int(value) for value in attack_path_ids],
        resumePathID=int(resume_path_id),
    )
    individual_alloc_started = time.perf_counter()
    attack_individual_ids = list(id_reservation.next_individuals(len(valid_targets)))
    [resume_individual_id] = id_reservation.next_individuals(1)
    _record_builder_stage(
        "allocate_individual_ids",
        individual_alloc_started,
        attackIndividualIDs=[int(value) for value in attack_individual_ids],
        resumeIndividualID=int(resume_individual_id),
    )

    attack_missions: List[Dict[str, Any]] = []
    attack_path_payloads: List[Tuple[Path, Dict[str, Any]]] = []
    attack_sequence_meta: List[Dict[str, Any]] = []
    attack_speed_mps = _lah_max_attack_speed_mps()
    emit(
        f"[ATTACK][LAH] Using max attack speed "
        f"{attack_speed_mps:.2f}m/s ({attack_speed_mps * 3.6:.1f}km/h) for aircraft {aircraft_id}."
    )

    previous_attack_coord: Optional[Dict[str, Any]] = None
    waypoint_alloc_elapsed_ms = 0.0
    allocated_waypoint_ids: List[int] = []
    first_entry_coord = _project_coordinate(
        current_coord,
        heading,
        get_runtime_attack_float("entry_offset_m", 100.0),
    ) or dict(current_coord)
    first_target_uses_special_anchor = special_lah_attack_coord is not None
    if special_lah_attack_coord is not None:
        first_entry_coord = dict(current_coord)
    first_entry_alt = (
        (
            _normalize_altitude_value((special_lah_attack_coord or {}).get("altitude"))
            if first_target_uses_special_anchor
            else None
        )
        or
        _normalize_altitude_value(first_entry_coord.get("altitude"))
        or _normalize_altitude_value(current_coord.get("altitude"))
        or 800
    )
    current_altitude_floor = _normalize_altitude_value(current_coord.get("altitude"))
    if current_altitude_floor is not None and int(first_entry_alt) < int(current_altitude_floor):
        first_entry_alt = int(current_altitude_floor)
    first_entry_coord["altitude"] = first_entry_alt

    for idx, assigned in enumerate(valid_targets):
        attack_coord = _normalize_coordinate(assigned.get("attack_coord") or assigned.get("coordinate"))
        if not attack_coord:
            continue
        if special_lah_attack_coord is not None:
            attack_coord = dict(special_lah_attack_coord)
        _apply_lah_altitude_floor(attack_coord, current_coord)
        preserve_attack_altitude = (
            _preserve_attack_point_altitude(assigned.get("attack_coord"))
            or special_lah_attack_coord is not None
        )
        attack_alt = (
            _normalize_altitude_value(attack_coord.get("altitude"))
            if preserve_attack_altitude
            else None
        )
        if attack_alt is None:
            attack_alt = first_entry_alt
        attack_coord["altitude"] = attack_alt

        attack_path_id = int(attack_path_ids[idx])
        attack_individual_id = int(attack_individual_ids[idx])
        waypoint_alloc_started = time.perf_counter()
        attack_wp_id = id_reservation.next_waypoint()
        waypoint_alloc_elapsed_ms += (time.perf_counter() - waypoint_alloc_started) * 1000.0
        allocated_waypoint_ids.append(int(attack_wp_id))

        attack_target_id = _to_int(assigned.get("target_id") or assigned.get("targetID")) or 0
        attack_target_type = _to_int(assigned.get("target_type") or assigned.get("targetType"))
        selected_weapon_type = _to_int(assigned.get("selected_weapon_type")) or _resolve_attack_weapon_type(
            {"target_type": attack_target_type, "target_id": attack_target_id},
            {"weapon_inventory": state.get("weapon_inventory")},
        )

        mission_attack = {
            "individualMissionID": attack_individual_id,
            "isDone": False,
            "relatedMission": dict(related_template),
            "individualMissionInfo": {
                "individualMissionType": 2,
                "patternType": 2,
                "autoZoomIn": False,
                "targetID": int(attack_target_id),
                "coordinateList": [
                    {
                        "latitude": attack_coord["latitude"],
                        "longitude": attack_coord["longitude"],
                        "altitude": attack_coord["altitude"],
                    },
                ],
            },
            "pathID": attack_path_id,
        }
        attack_missions.append(mission_attack)

        attack_wp = _build_lah_waypoint_from_template(
            template_wp,
            attack_wp_id,
            attack_coord,
            0,
            mark_attack=True,
            target_id=attack_target_id,
            weapon_type=selected_weapon_type,
            speed_override_mps=attack_speed_mps,
        )
        attack_fp_data = {
            "timestamp": now_ms,
            "Source": _extract_path_source(fp_data),
            "pathID": attack_path_id,
            "aircraftID": aircraft_id,
            "individualMissionID": attack_individual_id,
            "lahWaypointList": [attack_wp],
        }
        attack_fp_dest = db_paths.get_db_subpath("FlightPath", f"{attack_path_id}.json")
        attack_path_payloads.append((attack_fp_dest, attack_fp_data))
        attack_sequence_meta.append(
            {
                "targetID": int(attack_target_id),
                "targetType": int(attack_target_type) if attack_target_type is not None else None,
                "weaponType": int(selected_weapon_type) if selected_weapon_type is not None else None,
                "pathID": int(attack_path_id),
                "individualMissionID": int(attack_individual_id),
                "attackCoordinate": dict(attack_coord),
                "attackSpeedMps": float(attack_speed_mps),
            }
        )
        previous_attack_coord = dict(attack_coord)

    builder_timing["allocate_waypoint_ids"] = {
        "elapsedMs": round(float(waypoint_alloc_elapsed_ms), 3),
        "waypointIDs": [int(value) for value in allocated_waypoint_ids],
    }

    if not attack_missions or previous_attack_coord is None:
        emit(f"[ATTACK][LAH] Failed to build attack sequence for aircraft {aircraft_id}.")
        return None

    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    source_mission_list = imp_data.get("individualMissionList")
    effective_done_input_ids = (
        done_input_ids
        if done_input_ids is not None
        else _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    )
    if isinstance(source_mission_list, list) and 0 <= target_index < len(source_mission_list):
        follow_up_artifacts = _collect_attack_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][LAH]",
            current_input_id=input_mission_id,
            excluded_input_ids=effective_done_input_ids,
            individual_id_provider=id_reservation.next_individual,
            path_id_provider=id_reservation.next_path,
            waypoint_id_provider=id_reservation.next_waypoint,
        )
        if follow_up_artifacts is None:
            return None
        follow_up_missions, follow_up_paths, _follow_up_stats = follow_up_artifacts

    _trim_lah_follow_up_paths_after_anchor(
        follow_up_missions=follow_up_missions,
        follow_up_paths=follow_up_paths,
        current_input_id=input_mission_id,
        anchor_coord=special_resume_coord or previous_attack_coord,
        state=state,
        emit=emit,
        log_prefix="[ATTACK][LAH]",
        predict_anchor=False,
    )

    if special_resume_coord is not None:
        resume_waypoints = [
            _build_lah_anchor_waypoint(
                template_wp,
                coord=special_resume_coord,
                hovering_time=get_runtime_attack_int("lah_hold_seconds", 300),
                waypoint_id=int(id_reservation.next_waypoint()),
            )
        ]
        emit("[ATTACK][LAH] special operation sequence resume forced to battle-position anchor.")

    resume_waypoints, removed_wp_id = _trim_lah_resume_waypoints_after_attack_anchor(
        resume_waypoints,
        attack_anchor_coord=previous_attack_coord,
        source_fp_data=fp_data,
        aircraft_id=int(aircraft_id),
        path_id=_to_int(fp_data.get("pathID")),
        emit=emit,
        log_prefix="[ATTACK][LAH]",
        removed_wp_id=removed_wp_id,
        waypoint_id_provider=id_reservation.next_waypoint,
    )
    resume_target_coord = _extract_lah_waypoint_coordinate(resume_waypoints[0]) if resume_waypoints else None
    if resume_target_coord is not None:
        resume_bearing = _bearing_between(
            float(previous_attack_coord["latitude"]),
            float(previous_attack_coord["longitude"]),
            float(resume_target_coord["latitude"]),
            float(resume_target_coord["longitude"]),
        )
        resume_start_coord = dict(previous_attack_coord)
        projected_resume = _project_coordinate(
            previous_attack_coord,
            resume_bearing,
            get_runtime_attack_float("resume_offset_m", 20.0),
        )
        if projected_resume:
            resume_start_coord.update(
                {
                    "latitude": projected_resume.get("latitude", resume_start_coord.get("latitude")),
                    "longitude": projected_resume.get("longitude", resume_start_coord.get("longitude")),
                }
            )
        resume_start_coord["altitude"] = _lah_resume_transition_altitude(
            attack_coord=previous_attack_coord,
            resume_target_coord=resume_target_coord,
            special_resume_coord=special_resume_coord,
        )
        resume_waypoints = _prepend_lah_transition_waypoint(
            resume_waypoints,
            template_wp=template_wp,
            anchor_coord=resume_start_coord,
            waypoint_id_provider=id_reservation.next_waypoint,
        )

    has_resume = bool(resume_waypoints)
    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = int(resume_individual_id)
    mission_resume["pathID"] = int(resume_path_id)
    mission_resume["relatedMission"] = dict(related_template)
    mission_resume["isDone"] = False
    mission_resume_info = mission_resume.get("individualMissionInfo")
    if isinstance(mission_resume_info, dict):
        mission_resume["individualMissionInfo"] = deepcopy(mission_resume_info)
        mission_resume["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = _extract_path_source(fp_data)
    resume_fp_data["pathID"] = int(resume_path_id)
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = int(resume_individual_id)
    resume_fp_data["lahWaypointList"] = resume_waypoints

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    prefix = [
        deepcopy(mission)
        for mission in mission_list[:target_index]
        if isinstance(mission, dict)
    ] if 0 <= target_index < len(mission_list) else []
    rebuilt = prefix + list(attack_missions)
    if has_resume:
        rebuilt.append(mission_resume)
    rebuilt.extend(follow_up_missions)
    mission_list[:] = rebuilt
    emit(
        "[ATTACK][LAH] Preserved completed prefix, built sequential attack missions, and reattached "
        f"{len(follow_up_missions)} follow-up mission(s) for aircraft {aircraft_id}."
    )

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    resume_fp_dest = db_paths.get_db_subpath("FlightPath", f"{int(resume_path_id)}.json") if has_resume else None
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [(imp_dest, imp_data)]
    write_entries.extend((dest, payload) for dest, payload in attack_path_payloads)
    if resume_fp_dest is not None:
        write_entries.append((resume_fp_dest, resume_fp_data))
    write_entries.extend((dest, payload) for dest, payload in follow_up_paths)
    _validate_generated_artifact_write_entries(
        scope=f"attack_lah_sequence:{new_imp_id}",
        individual_mission_plans=[imp_data],
        entries=write_entries,
        log=emit,
    )
    write_started = time.perf_counter()
    write_results = _write_json_files_batch(write_entries)
    _record_builder_stage(
        "write_json",
        write_started,
        fileCount=len(write_results),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
    )

    emit(
        "[ATTACK][LAH] Generated sequential attack/resume missions -> "
        f"IMP:{imp_dest.name} attacks={len(attack_path_payloads)} "
        f"{'resume=' + resume_fp_dest.name if resume_fp_dest is not None else 'resume=none'} "
        f"(followUps={len(follow_up_missions)})"
    )

    result: Dict[str, Any] = {
        "aircraft_id": aircraft_id,
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "attack": {
            "individualMissionID": int(attack_individual_ids[0]),
            "pathID": int(attack_path_ids[0]),
        },
        "attackSequence": attack_sequence_meta,
        "removedWaypointID": removed_wp_id,
        "resumePath": str(resume_fp_dest) if resume_fp_dest is not None else None,
        "followUpMissionCount": len(follow_up_missions),
    }
    if has_resume:
        result["resume"] = {
            "individualMissionID": int(resume_individual_id),
            "pathID": int(resume_path_id),
        }
    builder_timing["totalMs"] = _elapsed_ms_detail(builder_started_total)
    result["timingMs"] = builder_timing
    return result


def _build_lah_attack_package(
    *,
    descriptor: Dict[str, Any],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission: Dict[str, Any],
    target_index: Optional[int],
    attack_coord: Dict[str, Any],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    aircraft_id: int,
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
    done_input_ids: Optional[set[int]] = None,
    id_reservation: AttackIdReservation | None = None,
) -> Optional[Dict[str, Any]]:
    if id_reservation is None:
        raise RuntimeError("AttackIdReservation is required for LAH attack builder")
    if target_index is None:
        emit(f"[ATTACK][LAH] Target mission index unavailable for aircraft {aircraft_id}.")
        return None

    current_coord = _normalize_coordinate(state.get("coordinate"))
    if not current_coord:
        emit(f"[ATTACK][LAH] Coordinate missing for aircraft {aircraft_id}.")
        return None
    heading = _to_float(state.get("heading"))
    if heading is None:
        emit(f"[ATTACK][LAH] Heading missing for aircraft {aircraft_id}; defaulting to north.")
        heading = 0.0

    entry_coord = _project_coordinate(
        current_coord,
        heading,
        get_runtime_attack_float("entry_offset_m", 100.0),
    ) or dict(current_coord)
    entry_alt = _normalize_altitude_value(entry_coord.get("altitude")) or _normalize_altitude_value(current_coord.get("altitude")) or 800
    current_altitude_floor = _normalize_altitude_value(current_coord.get("altitude"))
    if current_altitude_floor is not None and int(entry_alt) < int(current_altitude_floor):
        entry_alt = int(current_altitude_floor)
    entry_coord["altitude"] = entry_alt

    attack_coord_norm = _normalize_coordinate(attack_coord)
    if not attack_coord_norm:
        emit("[ATTACK][LAH] Attack coordinate unavailable for manned aircraft.")
        return None
    rel_info = dict(target_mission.get("relatedMission") or {})
    input_mission_id = _to_int(rel_info.get("inputMissionID")) or _to_int((ctx.get("mission_ids") or [None])[0]) or 0
    prior_mission_id = _to_int(rel_info.get("priorMissionID")) or 0
    related_template = {
        "relatedMissionType": 1,
        "inputMissionID": input_mission_id,
        "priorMissionID": prior_mission_id,
    }
    preserve_attack_altitude = _preserve_attack_point_altitude(attack_coord)
    special_lah_attack_coord = _lah_special_battle_anchor_for_input(
        ctx,
        aircraft_id=int(aircraft_id),
        input_mission_id=input_mission_id,
    )
    special_resume_coord = dict(special_lah_attack_coord) if special_lah_attack_coord is not None else None
    if special_lah_attack_coord is not None:
        attack_coord_norm = dict(special_lah_attack_coord)
        entry_coord = dict(current_coord)
        entry_coord["altitude"] = (
            _normalize_altitude_value(attack_coord_norm.get("altitude"))
            or _normalize_altitude_value(current_coord.get("altitude"))
            or entry_alt
        )
        entry_alt = int(entry_coord["altitude"])
        if current_altitude_floor is not None and int(entry_alt) < int(current_altitude_floor):
            entry_alt = int(current_altitude_floor)
            entry_coord["altitude"] = int(entry_alt)
        preserve_attack_altitude = True
        emit(
            "[ATTACK][LAH] current phase forces battle-position attack/resume anchor "
            f"(aircraft={int(aircraft_id)}, inputMissionID={int(input_mission_id)})."
        )
    _apply_lah_altitude_floor(attack_coord_norm, current_coord)
    attack_alt = (
        _normalize_altitude_value(attack_coord_norm.get("altitude"))
        if preserve_attack_altitude
        else None
    )
    if attack_alt is None:
        attack_alt = entry_alt
    attack_coord_norm["altitude"] = attack_alt

    builder_started_total = time.perf_counter()
    builder_timing: Dict[str, Any] = {}

    def _record_builder_stage(name: str, started_at: float, **extra: Any) -> None:
        row: Dict[str, Any] = {"elapsedMs": _elapsed_ms_detail(started_at)}
        if extra:
            row.update(_json_safe(extra))
        builder_timing[str(name)] = row

    allocate_started = time.perf_counter()
    path_alloc_started = time.perf_counter()
    attack_path_id, resume_path_id = id_reservation.next_paths(aircraft_id, 2)
    _record_builder_stage(
        "allocate_path_ids",
        path_alloc_started,
        attackPathID=attack_path_id,
        resumePathID=resume_path_id,
    )
    individual_alloc_started = time.perf_counter()
    attack_individual_id, resume_individual_id = id_reservation.next_individuals(2)
    _record_builder_stage(
        "allocate_individual_ids",
        individual_alloc_started,
        attackIndividualID=attack_individual_id,
        resumeIndividualID=resume_individual_id,
    )
    waypoint_alloc_started = time.perf_counter()
    attack_wp_id = id_reservation.next_waypoint()
    _record_builder_stage(
        "allocate_waypoint_ids",
        waypoint_alloc_started,
        attackWaypointID=attack_wp_id,
    )
    _record_builder_stage(
        "allocate_ids",
        allocate_started,
        attackPathID=attack_path_id,
        resumePathID=resume_path_id,
        attackIndividualID=attack_individual_id,
        resumeIndividualID=resume_individual_id,
        attackWaypointID=attack_wp_id,
    )

    attack_target_id = _to_int(descriptor.get("target_id"))
    attack_target_type = _to_int(descriptor.get("target_type"))
    if attack_target_id is None:
        detail = ctx.get("replan_detail") if isinstance(ctx, dict) else {}
        if isinstance(detail, dict):
            attack_target_id = _to_int(detail.get("targetID") or detail.get("targetId"))
            if attack_target_type is None:
                attack_target_type = _to_int(detail.get("targetType"))
            if attack_target_id is None:
                orient = detail.get("targetOrientation") or {}
                if isinstance(orient, dict):
                    attack_target_id = _to_int(orient.get("targetID") or orient.get("targetId"))
    attack_target_id_value = attack_target_id if attack_target_id is not None else 0
    selected_weapon_type = _to_int(ctx.get("_selected_attack_weapon_type")) or _resolve_attack_weapon_type(
        {"target_type": attack_target_type, "target_id": attack_target_id_value}
    )

    template_wp = deepcopy((fp_data.get("lahWaypointList") or [None])[0]) if fp_data.get("lahWaypointList") else _default_lah_waypoint_template()
    attack_speed_mps = _lah_max_attack_speed_mps()
    emit(
        f"[ATTACK][LAH] Using max attack speed "
        f"{attack_speed_mps:.2f}m/s ({attack_speed_mps * 3.6:.1f}km/h) for aircraft {aircraft_id}."
    )
    split_started = time.perf_counter()
    replan_resume_anchor = _predict_replan_resume_anchor(current_coord, state)
    _, resume_waypoints, removed_wp_id = _split_done_resume_lah_path(
        fp_data,
        artifacts=artifacts,
        current_coord=current_coord,
        emit=emit,
        force_nonempty_resume=True,
        exclude_current_from_resume=True,
        resume_trim_anchor_coord=replan_resume_anchor,
        waypoint_id_provider=id_reservation.next_waypoint,
    )
    _record_builder_stage(
        "split_lah_resume",
        split_started,
        resumeWaypointCount=len(resume_waypoints or []),
        removedWaypointID=removed_wp_id,
    )

    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    source_mission_list = imp_data.get("individualMissionList")
    effective_done_input_ids = (
        done_input_ids
        if done_input_ids is not None
        else _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    )
    if (
        isinstance(source_mission_list, list)
        and 0 <= target_index < len(source_mission_list)
    ):
        clone_started = time.perf_counter()
        follow_up_artifacts = _collect_attack_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][LAH]",
            current_input_id=input_mission_id,
            excluded_input_ids=effective_done_input_ids,
            individual_id_provider=id_reservation.next_individual,
            path_id_provider=id_reservation.next_path,
            waypoint_id_provider=id_reservation.next_waypoint,
        )
        if follow_up_artifacts is None:
            return None
        follow_up_missions, follow_up_paths, follow_up_stats = follow_up_artifacts
        _record_builder_stage(
            "clone_followups",
            clone_started,
            followUpMissionCount=len(follow_up_missions),
            followUpPathCount=len(follow_up_paths),
            preservedFollowUpCount=follow_up_stats.get("preservedCount"),
            clonedFollowUpCount=follow_up_stats.get("clonedCount"),
            skippedFollowUpCount=follow_up_stats.get("skippedCount"),
        )

    _trim_lah_follow_up_paths_after_anchor(
        follow_up_missions=follow_up_missions,
        follow_up_paths=follow_up_paths,
        current_input_id=input_mission_id,
        anchor_coord=special_resume_coord or attack_coord_norm,
        state=state,
        emit=emit,
        log_prefix="[ATTACK][LAH]",
        predict_anchor=False,
    )

    if special_resume_coord is not None:
        resume_waypoints = [
            _build_lah_anchor_waypoint(
                template_wp,
                coord=special_resume_coord,
                hovering_time=get_runtime_attack_int("lah_hold_seconds", 300),
                waypoint_id=int(id_reservation.next_waypoint()),
            )
        ]
        emit("[ATTACK][LAH] special operation resume forced to battle-position anchor.")

    resume_waypoints, removed_wp_id = _trim_lah_resume_waypoints_after_attack_anchor(
        resume_waypoints,
        attack_anchor_coord=attack_coord_norm,
        source_fp_data=fp_data,
        aircraft_id=int(aircraft_id),
        path_id=_to_int(fp_data.get("pathID")),
        emit=emit,
        log_prefix="[ATTACK][LAH]",
        removed_wp_id=removed_wp_id,
        waypoint_id_provider=id_reservation.next_waypoint,
    )
    payload_started = time.perf_counter()
    resume_target_coord = _extract_lah_waypoint_coordinate(resume_waypoints[0]) if resume_waypoints else None
    if resume_target_coord is not None:
        resume_bearing = _bearing_between(
            float(attack_coord_norm["latitude"]),
            float(attack_coord_norm["longitude"]),
            float(resume_target_coord["latitude"]),
            float(resume_target_coord["longitude"]),
        )
        resume_start_coord = dict(attack_coord_norm)
        projected_resume = _project_coordinate(
            attack_coord_norm,
            resume_bearing,
            get_runtime_attack_float("resume_offset_m", 20.0),
        )
        if projected_resume:
            resume_start_coord.update(
                {
                    "latitude": projected_resume.get("latitude", resume_start_coord.get("latitude")),
                    "longitude": projected_resume.get("longitude", resume_start_coord.get("longitude")),
                }
            )
        resume_start_coord["altitude"] = _lah_resume_transition_altitude(
            attack_coord=attack_coord_norm,
            resume_target_coord=resume_target_coord,
            special_resume_coord=special_resume_coord,
        )
        resume_waypoints = _prepend_lah_transition_waypoint(
            resume_waypoints,
            template_wp=template_wp,
            anchor_coord=resume_start_coord,
            waypoint_id_provider=id_reservation.next_waypoint,
        )
    has_resume = bool(resume_waypoints)

    mission_attack = {
        "individualMissionID": attack_individual_id,
        "isDone": False,
        "relatedMission": dict(related_template),
        "individualMissionInfo": {
            "individualMissionType": 2,
            "patternType": 2,
            "autoZoomIn": False,
            "targetID": attack_target_id_value,
            "coordinateList": [
                {
                    "latitude": attack_coord_norm["latitude"],
                    "longitude": attack_coord_norm["longitude"],
                    "altitude": attack_coord_norm["altitude"],
                },
            ],
        },
        "pathID": attack_path_id,
    }

    attack_wp = _build_lah_waypoint_from_template(
        template_wp,
        attack_wp_id,
        attack_coord_norm,
        0,
        mark_attack=True,
        target_id=attack_target_id,
        weapon_type=selected_weapon_type,
        speed_override_mps=attack_speed_mps,
    )
    attack_fp_data = {
        "timestamp": now_ms,
        "Source": _extract_path_source(fp_data),
        "pathID": attack_path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": attack_individual_id,
        "lahWaypointList": [attack_wp],
    }
    _record_builder_stage(
        "payload_build",
        payload_started,
        hasResume=bool(resume_waypoints),
        followUpMissionCount=len(follow_up_missions),
        attackSpeedMps=float(attack_speed_mps),
    )

    original_entry = deepcopy(target_mission)

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = dict(related_template)
    mission_resume["isDone"] = False
    mission_resume_info = mission_resume.get("individualMissionInfo")
    if isinstance(mission_resume_info, dict):
        mission_resume["individualMissionInfo"] = deepcopy(mission_resume_info)
        mission_resume["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = _extract_path_source(fp_data)
    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = resume_individual_id
    resume_fp_data["lahWaypointList"] = resume_waypoints

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    if 0 <= target_index < len(mission_list):
        prefix = [deepcopy(mission) for mission in mission_list[:target_index] if isinstance(mission, dict)]
        rebuilt = prefix + [mission_attack]
        if has_resume:
            rebuilt.append(mission_resume)
        rebuilt.extend(follow_up_missions)
        mission_list[:] = rebuilt
        emit(
            "[ATTACK][LAH] Preserved completed prefix, inserted attack, and reattached "
            f"{len(follow_up_missions)} follow-up mission(s)."
        )
    else:
        mission_list.insert(0, mission_attack)
        if has_resume:
            mission_list.append(mission_resume)
        mission_list.extend(follow_up_missions)
        emit(f"[ATTACK][LAH] Target mission index invalid; appended attack branch at head (aircraft {aircraft_id}).")

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    attack_fp_dest = db_paths.get_db_subpath("FlightPath", f"{attack_path_id}.json")
    resume_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json") if has_resume else None
    )
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [
        (imp_dest, imp_data),
        (attack_fp_dest, attack_fp_data),
    ]
    if resume_fp_dest is not None:
        write_entries.append((resume_fp_dest, resume_fp_data))
    write_entries.extend((dest, payload) for dest, payload in follow_up_paths)
    _validate_generated_artifact_write_entries(
        scope=f"attack_lah_attack:{new_imp_id}",
        individual_mission_plans=[imp_data],
        entries=write_entries,
        log=emit,
    )
    write_started = time.perf_counter()
    write_results = _write_json_files_batch(write_entries)
    _record_builder_stage(
        "write_json",
        write_started,
        fileCount=len(write_results),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
    )

    emit(
        "[ATTACK][LAH] Generated attack/resume missions -> "
        f"IMP:{imp_dest.name} PATHS:{attack_fp_dest.name}"
        f"{'/' + resume_fp_dest.name if resume_fp_dest is not None else ''} "
        f"(followUps={len(follow_up_missions)})"
    )

    result: Dict[str, Any] = {
        "aircraft_id": aircraft_id,
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "attack": {
            "individualMissionID": attack_individual_id,
            "pathID": attack_path_id,
            "waypointIDs": [attack_wp_id],
        },
        "removedWaypointID": removed_wp_id,
        "attackPath": str(attack_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
        "attackCoordinate": dict(attack_coord_norm),
    }
    if resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
        result["resumePath"] = str(resume_fp_dest)
    builder_timing["totalMs"] = _elapsed_ms_detail(builder_started_total)
    result["timingMs"] = builder_timing
    return result


def _build_lah_hold_resume_package(
    *,
    descriptor: Dict[str, Any],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission: Dict[str, Any],
    target_index: Optional[int],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    aircraft_id: int,
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
    done_input_ids: Optional[set[int]] = None,
    id_reservation: AttackIdReservation | None = None,
) -> Optional[Dict[str, Any]]:
    if id_reservation is None:
        raise RuntimeError("AttackIdReservation is required for LAH hold/resume builder")
    if target_index is None:
        emit(f"[ATTACK][LAH] Target mission index unavailable for aircraft {aircraft_id}.")
        return None

    current_coord = _normalize_coordinate(state.get("coordinate"))
    if current_coord is None:
        current_coord = _extract_final_lah_coordinate(fp_data)
    if current_coord is None:
        emit(f"[ATTACK][LAH] Hold/resume coordinate missing for aircraft {aircraft_id}.")
        return None

    builder_started_total = time.perf_counter()
    builder_timing: Dict[str, Any] = {}

    def _record_builder_stage(name: str, started_at: float, **extra: Any) -> None:
        row: Dict[str, Any] = {"elapsedMs": _elapsed_ms_detail(started_at)}
        if extra:
            row.update(_json_safe(extra))
        builder_timing[str(name)] = row

    allocate_started = time.perf_counter()
    path_alloc_started = time.perf_counter()
    hold_path_id, resume_path_id = id_reservation.next_paths(aircraft_id, 2)
    _record_builder_stage(
        "allocate_path_ids",
        path_alloc_started,
        holdPathID=hold_path_id,
        resumePathID=resume_path_id,
    )
    individual_alloc_started = time.perf_counter()
    hold_individual_id, resume_individual_id = id_reservation.next_individuals(2)
    _record_builder_stage(
        "allocate_individual_ids",
        individual_alloc_started,
        holdIndividualID=hold_individual_id,
        resumeIndividualID=resume_individual_id,
    )
    _record_builder_stage(
        "allocate_ids",
        allocate_started,
        holdPathID=hold_path_id,
        resumePathID=resume_path_id,
        holdIndividualID=hold_individual_id,
        resumeIndividualID=resume_individual_id,
    )
    template_wp = deepcopy((fp_data.get("lahWaypointList") or [None])[0]) if fp_data.get("lahWaypointList") else _default_lah_waypoint_template()

    split_started = time.perf_counter()
    replan_resume_anchor = _predict_replan_resume_anchor(current_coord, state)
    _, resume_waypoints, removed_wp_id = _split_done_resume_lah_path(
        fp_data,
        artifacts=artifacts,
        current_coord=current_coord,
        emit=emit,
        force_nonempty_resume=True,
        exclude_current_from_resume=True,
        resume_trim_anchor_coord=replan_resume_anchor,
        waypoint_id_provider=id_reservation.next_waypoint,
    )
    _record_builder_stage(
        "split_lah_resume",
        split_started,
        resumeWaypointCount=len(resume_waypoints or []),
        removedWaypointID=removed_wp_id,
    )
    has_resume = bool(resume_waypoints)
    payload_started = time.perf_counter()
    hold_coord = _build_lah_standby_hold_coordinate_from_path_end(
        target_mission=target_mission,
        fp_data=fp_data,
    )
    if hold_coord is not None:
        emit("[ATTACK][LAH] standby hold anchored to current LINE/hold path endpoint.")
    if hold_coord is None:
        hold_coord = _build_lah_hold_coordinate_near_resume(
            resume_waypoints=resume_waypoints,
            current_coord=current_coord,
        )
    if hold_coord is None:
        hold_coord = current_coord

    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    source_mission_list = imp_data.get("individualMissionList")
    original_entry = deepcopy(target_mission)
    rel_info = dict(original_entry.get("relatedMission") or {})
    input_mission_id = _to_int(rel_info.get("inputMissionID")) or _to_int((ctx.get("mission_ids") or [None])[0]) or 0
    prior_mission_id = _to_int(rel_info.get("priorMissionID")) or 0
    related_template = {
        "relatedMissionType": rel_info.get("relatedMissionType", 1),
        "inputMissionID": input_mission_id,
        "priorMissionID": prior_mission_id,
    }
    effective_done_input_ids = (
        done_input_ids
        if done_input_ids is not None
        else _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    )
    if (
        isinstance(source_mission_list, list)
        and 0 <= target_index < len(source_mission_list)
    ):
        clone_started = time.perf_counter()
        follow_up_artifacts = _collect_attack_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][LAH]",
            current_input_id=input_mission_id,
            excluded_input_ids=effective_done_input_ids,
            individual_id_provider=id_reservation.next_individual,
            path_id_provider=id_reservation.next_path,
            waypoint_id_provider=id_reservation.next_waypoint,
        )
        if follow_up_artifacts is None:
            return None
        follow_up_missions, follow_up_paths, follow_up_stats = follow_up_artifacts
        _record_builder_stage(
            "clone_followups",
            clone_started,
            followUpMissionCount=len(follow_up_missions),
            followUpPathCount=len(follow_up_paths),
            preservedFollowUpCount=follow_up_stats.get("preservedCount"),
            clonedFollowUpCount=follow_up_stats.get("clonedCount"),
            skippedFollowUpCount=follow_up_stats.get("skippedCount"),
        )

    _trim_lah_follow_up_paths_after_anchor(
        follow_up_missions=follow_up_missions,
        follow_up_paths=follow_up_paths,
        current_input_id=input_mission_id,
        anchor_coord=current_coord,
        state=state,
        emit=emit,
        log_prefix="[ATTACK][LAH]",
        predict_anchor=True,
    )

    mission_hold = {
        "individualMissionID": hold_individual_id,
        "isDone": False,
        "relatedMission": dict(related_template),
        "individualMissionInfo": {
            "individualMissionType": 9,
            "patternType": 12,
            "autoZoomIn": False,
            "coordinateList": [dict(hold_coord)],
            "targetID": None,
        },
        "pathID": hold_path_id,
    }

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = dict(related_template)
    mission_resume["isDone"] = False
    mission_resume_info = mission_resume.get("individualMissionInfo")
    if isinstance(mission_resume_info, dict):
        mission_resume["individualMissionInfo"] = deepcopy(mission_resume_info)
        mission_resume["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)

    hold_wp = _build_lah_anchor_waypoint(
        template_wp,
        coord=hold_coord,
        next_id=0,
        hovering_time=get_runtime_attack_int("lah_hold_seconds", 300),
        waypoint_id=id_reservation.next_waypoint(),
    )
    hold_fp_data = {
        "timestamp": now_ms,
        "Source": _extract_path_source(fp_data),
        "pathID": hold_path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": hold_individual_id,
        "lahWaypointList": [hold_wp],
    }

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = _extract_path_source(fp_data)
    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = resume_individual_id
    resume_fp_data["lahWaypointList"] = resume_waypoints
    _record_builder_stage(
        "payload_build",
        payload_started,
        hasResume=bool(has_resume),
        followUpMissionCount=len(follow_up_missions),
    )

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    if 0 <= target_index < len(mission_list):
        prefix = [deepcopy(mission) for mission in mission_list[:target_index] if isinstance(mission, dict)]
        rebuilt = prefix + [mission_hold]
        if has_resume:
            rebuilt.append(mission_resume)
        rebuilt.extend(follow_up_missions)
        mission_list[:] = rebuilt
        emit(
            "[ATTACK][LAH] Preserved completed prefix, inserted hold, and reattached "
            f"{len(follow_up_missions)} follow-up mission(s)."
        )
    else:
        mission_list.insert(0, mission_hold)
        if has_resume:
            mission_list.append(mission_resume)
        mission_list.extend(follow_up_missions)
        emit(
            f"[ATTACK][LAH] Target mission index invalid; appended hold branch at head (aircraft {aircraft_id})."
        )

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    hold_fp_dest = db_paths.get_db_subpath("FlightPath", f"{hold_path_id}.json")
    resume_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json") if has_resume else None
    )
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [
        (imp_dest, imp_data),
        (hold_fp_dest, hold_fp_data),
    ]
    if resume_fp_dest is not None:
        write_entries.append((resume_fp_dest, resume_fp_data))
    write_entries.extend((dest, payload) for dest, payload in follow_up_paths)
    _validate_generated_artifact_write_entries(
        scope=f"attack_lah_hold_resume:{new_imp_id}",
        individual_mission_plans=[imp_data],
        entries=write_entries,
        log=emit,
    )
    write_started = time.perf_counter()
    write_results = _write_json_files_batch(write_entries)
    _record_builder_stage(
        "write_json",
        write_started,
        fileCount=len(write_results),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
    )

    emit(
        "[ATTACK][LAH] Generated hold/resume missions -> "
        f"IMP:{imp_dest.name} PATHS:{hold_fp_dest.name}"
        f"{'/' + resume_fp_dest.name if resume_fp_dest is not None else ''} "
        f"(followUps={len(follow_up_missions)})"
    )

    result: Dict[str, Any] = {
        "aircraft_id": aircraft_id,
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "hold": {
            "individualMissionID": hold_individual_id,
            "pathID": hold_path_id,
            "waypointID": _to_int(hold_wp.get("waypointID")),
            "durationSeconds": get_runtime_attack_int("lah_hold_seconds", 300),
        },
        "removedWaypointID": removed_wp_id,
        "holdPath": str(hold_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
    }
    if resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
        result["resumePath"] = str(resume_fp_dest)
    builder_timing["totalMs"] = _elapsed_ms_detail(builder_started_total)
    result["timingMs"] = builder_timing
    return result


def _build_lah_waypoint_from_template(
    template: Dict[str, Any],
    waypoint_id: int,
    coord: Dict[str, Any],
    next_id: int,
    *,
    mark_attack: bool,
    target_id: Optional[int],
    weapon_type: Optional[int] = None,
    speed_override_mps: Optional[float] = None,
) -> Dict[str, Any]:
    waypoint = deepcopy(template)
    waypoint["waypointID"] = waypoint_id
    waypoint["nextWaypointID"] = next_id or 0
    coordinate = dict(template.get("coordinate") or {})
    coordinate["latitude"] = coord.get("latitude")
    coordinate["longitude"] = coord.get("longitude")
    coordinate["altitude"] = _normalize_altitude_value(coord.get("altitude")) or coordinate.get("altitude") or 800
    waypoint["coordinate"] = coordinate
    speed_value = _to_float(speed_override_mps)
    if speed_value is None or speed_value <= 0.0:
        speed_value = _to_float(template.get("speed")) or 30.0
    waypoint["speed"] = round(float(speed_value), 2)
    waypoint["hovering"] = {"time": 0}
    if "loiter" in waypoint:
        waypoint["loiter"] = {"radius": 0, "direction": 0, "time": 0, "speed": 0}
    attack_block = dict(template.get("attack") or {"targetID": 0, "weaponType": 0})
    if mark_attack:
        attack_block["targetID"] = _to_int(target_id) or 0
        attack_block["weaponType"] = (
            max(0, min(3, int(weapon_type)))
            if weapon_type is not None
            else int(get_runtime_attack_weapon_type(2))
        )
    else:
        attack_block["targetID"] = 0
        attack_block["weaponType"] = 0
    waypoint["attack"] = attack_block
    return waypoint


def _default_lah_waypoint_template() -> Dict[str, Any]:
    return {
        "waypointID": 0,
        "coordinate": {"latitude": 0.0, "longitude": 0.0, "altitude": 800},
        "speed": 40.0,
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "hovering": {"time": 0},
        "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
        "attack": {"targetID": 0, "weaponType": 0},
    }


def _extract_final_lah_coordinate(fp_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    waypoints = fp_data.get("lahWaypointList") or []
    if not waypoints:
        return None
    last = waypoints[-1].get("coordinate")
    return _normalize_coordinate(last)


def _project_coordinate(
    coord: Optional[Dict[str, Any]],
    heading_deg: Optional[float],
    distance_m: float,
) -> Optional[Dict[str, float]]:
    if not coord:
        return None
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    heading = heading_deg if heading_deg is not None else 0.0
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    heading_rad = math.radians(heading)
    angular_distance = distance_m / 6_371_000.0
    new_lat = math.asin(
        math.sin(lat_rad) * math.cos(angular_distance)
        + math.cos(lat_rad) * math.sin(angular_distance) * math.cos(heading_rad)
    )
    new_lon = lon_rad + math.atan2(
        math.sin(heading_rad) * math.sin(angular_distance) * math.cos(lat_rad),
        math.cos(angular_distance) - math.sin(lat_rad) * math.sin(new_lat),
    )
    return {
        "latitude": math.degrees(new_lat),
        "longitude": math.degrees(new_lon),
    }


def _update_plan_aircraft_entry(
    plan_data: Dict[str, Any],
    aircraft_id: int,
    new_package_id: int,
    emit: Callable[[str], None],
) -> bool:
    for entry in plan_data.get("aircraftList", []):
        if _to_int(entry.get("aircraftID")) == aircraft_id:
            entry["individualMissionPackageID"] = new_package_id
            return True
    emit(f"[ATTACK] Aircraft {aircraft_id} not found in mission plan.")
    return False


def _mission_plan_has_aircraft_entry(plan_data: Dict[str, Any], aircraft_id: int) -> bool:
    for entry in plan_data.get("aircraftList", []):
        if not isinstance(entry, dict):
            continue
        if _to_int(entry.get("aircraftID")) == int(aircraft_id):
            return True
    return False


def _prepare_attack_json_payload(path: Path, data: Dict[str, Any]) -> Dict[str, Any]:
    waypoint_list = data.get("waypointList") if isinstance(data, dict) else None
    lah_waypoint_list = data.get("lahWaypointList") if isinstance(data, dict) else None
    has_waypoints = isinstance(waypoint_list, list) and bool(waypoint_list)
    has_lah_waypoints = isinstance(lah_waypoint_list, list) and bool(lah_waypoint_list)
    if has_waypoints:
        try:
            from modules.mission_planning.replanning.triggers.prior.pipeline import (
                _apply_runtime_flyover_to_flight_path_payload,
            )
            _apply_runtime_flyover_to_flight_path_payload(data)
        except Exception:
            pass
    if has_waypoints or has_lah_waypoints:
        sanitize_flight_path_payload_filming_altitudes(data)
    return data


def _write_json_file(path: Path, data: Dict[str, Any]) -> None:
    data = _prepare_attack_json_payload(path, data)
    write_json(path, data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)


def _attack_json_write_workers(file_count: int) -> int:
    try:
        requested = int(get_runtime_attack_int("json_write_workers", 4))
    except Exception:
        requested = 4
    return max(1, min(int(file_count or 0), int(requested)))


def _validate_generated_artifact_write_entries(
    *,
    scope: str,
    individual_mission_plans: List[Dict[str, Any]],
    entries: List[Tuple[Path, Dict[str, Any]]],
    log: Callable[[str], None],
) -> Dict[str, Any]:
    flight_paths = [
        payload
        for _path, payload in entries or []
        if isinstance(payload, dict) and _to_int(payload.get("pathID")) is not None
    ]
    return validate_generated_artifact_payloads(
        individual_mission_plans=individual_mission_plans,
        flight_paths=flight_paths,
        scope=scope,
        allow_existing_db_artifacts=True,
        log=log,
    )


def _write_json_files_batch(entries: List[Tuple[Path, Dict[str, Any]]]) -> List[Dict[str, Any]]:
    normalized_entries = [(Path(path), payload) for path, payload in entries]
    unique_paths = {str(path.resolve()) for path, _payload in normalized_entries}
    unique_payloads = {id(payload) for _path, payload in normalized_entries}
    workers = (
        _attack_json_write_workers(len(normalized_entries))
        if len(unique_paths) == len(normalized_entries)
        and len(unique_payloads) == len(normalized_entries)
        else 1
    )
    if workers <= 1:
        prepared_entries = [
            (path, _prepare_attack_json_payload(path, payload))
            for path, payload in normalized_entries
        ]
        return write_json_batch(
            prepared_entries,
            pretty=True,
            ensure_ascii=False,
            skip_if_unchanged=True,
        )

    def _write_one(entry: Tuple[Path, Dict[str, Any]]) -> Dict[str, Any]:
        path, payload = entry
        prepared = _prepare_attack_json_payload(path, payload)
        written = write_json(
            path,
            prepared,
            pretty=True,
            ensure_ascii=False,
            skip_if_unchanged=True,
        )
        return {
            "path": str(path),
            "name": path.name,
            "written": bool(written),
            "skipped": not bool(written),
        }

    with concurrent.futures.ThreadPoolExecutor(
        max_workers=int(workers),
        thread_name_prefix="AttackJsonWrite",
    ) as executor:
        return list(executor.map(_write_one, normalized_entries))


def _now_timestamp_ms() -> int:
    epoch = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.now(timezone.utc) - epoch).total_seconds() * 1000)
