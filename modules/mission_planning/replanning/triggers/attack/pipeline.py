from __future__ import annotations

import json
import concurrent.futures
import os
import subprocess
import threading
from copy import deepcopy
import importlib.util
import math
import time
from collections import OrderedDict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple
import sys
from types import ModuleType

from modules.common import agent_status_snapshot, db_paths, mission_area_replan_store
from modules.common.eta import annotate_eta_flight_plan
from modules.common.terrain_los import ENEMY_OBSERVER_HEIGHT_M, LOS_CLEARANCE_M
from modules.monitoring.logic.dem_cover.los_api import evaluate_regional_los
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
from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
    get_terrain_elev_many_metrics,
    reset_terrain_elev_many_metrics,
    terrain_elev_many,
)
from modules.mission_planning.MissionPlanner.data_def.lah_terrain_path import (
    LAH_LOW_TERRAIN_CORRIDOR_M,
    LAH_LOW_TERRAIN_MIN_LEG_M,
    LAH_VERTICAL_RATE_USE_RATIO,
    build_lah_terrain_following_path,
)
from modules.mission_planning.engine.mission_generation.id_allocation.allocator import (
    mark_waypoint_files_written,
    reserve_mission_plan_ids,
)
from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0304 import (
    normalize_lah_eta_seconds_inplace,
)
from modules.mission_planning.MissionPlanner.dynamics.lah_op_envlp import DEFAULT_ENVELOPE
from modules.mission_planning.runtime.debug_artifacts import debug_artifact_mode, write_debug_json
from modules.mission_planning.runtime.json_io import write_json
from modules.mission_planning.runtime.replan_transaction import ReplanTransaction
from modules.mission_planning.runtime.cache.source_artifacts import (
    SourceArtifactCache,
    call_with_source_artifact_cache,
    get_active_source_artifact_cache,
    read_json_cached,
    use_source_artifact_cache,
)
from modules.mission_planning.runtime.logging.pipeline_events import (
    PipelinePhaseTimer,
    new_replan_transaction_id,
)
from modules.mission_planning.runtime.ids.replan_reservation import ReplanIdReservation, ReservedIdBlock
from modules.mission_planning.runtime.validation.replan_payloads import (
    ReplanValidationError,
    normalize_flight_path_waypoint_altitudes_inplace,
    normalize_flight_path_waypoint_speeds_inplace,
    validate_generated_artifact_payloads,
    validate_replan_payloads,
)
from modules.mission_planning.runtime.validation.attack_continuity import (
    collect_lah_attack_rows,
    evaluate_candidate_attack_continuity,
    missing_attack_identities,
)
from modules.mission_planning.pipelines.attack_los_altitude import profile_with_batch_dem
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
    resolve_plan_lineage_ids,
)

_ATTACK_ROOT = mission_planning_root()
_MP_DIR = mission_planner_root()
_PROJECT_ROOT = project_root()
_RECEIVE_DB_MOD: Optional[ModuleType] = None
_ATTACK_FOLLOW_UP_PRESERVE_CACHE_LOCK = threading.RLock()
_ATTACK_FOLLOW_UP_PRESERVE_CACHE_MAX = 4096
_ATTACK_FOLLOW_UP_PRESERVE_CACHE: "OrderedDict[Tuple[str, int, int, int, int], bool]" = OrderedDict()
_ATTACK_SNAPSHOT_CARRY_EXECUTOR = concurrent.futures.ThreadPoolExecutor(
    max_workers=1,
    thread_name_prefix="AttackSnapshotCarry",
)
for _candidate in (_PROJECT_ROOT, _ATTACK_ROOT, _MP_DIR):
    _candidate_str = str(_candidate)
    if _candidate.exists() and _candidate_str not in sys.path:
        sys.path.insert(0, _candidate_str)


def _queue_attack_snapshot_carry(
    source_plan_id: int,
    target_plan_id: int,
    *,
    reason: str,
) -> concurrent.futures.Future:
    """Run post-commit snapshot bookkeeping outside the planning critical path."""
    return _ATTACK_SNAPSHOT_CARRY_EXECUTOR.submit(
        mission_area_replan_store.carry_forward_snapshot,
        int(source_plan_id),
        int(target_plan_id),
        reason=str(reason),
    )


def _attack_fast_path_key(path: Path) -> str:
    try:
        return os.path.normcase(os.path.abspath(os.fspath(path)))
    except Exception:
        return str(path)


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
from modules.mission_planning.pipelines.ground_maneuver_mode import (
    TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
    TYPE2_SELF_RELIANCE_RETURN_LINE,
    detect_ground_maneuver_attack_profile,
    ground_maneuver_target_attack_anchor,
    resolve_type2_self_reliance_phase,
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
from modules.mission_planning.pipelines.line_scan_remaining_adapter import (
    _normalize_coord_list as _normalize_line_coord_list,
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
_UAV_ATTACK_COMPLETION_HOLD_SECONDS = 300
_UAV_ATTACK_COMPLETION_HOLD_RADIUS_M = 400
_UAV_ATTACK_COMPLETION_HOLD_SPEED_MPS = 30.0
_LAH_ATTACK_MAX_SPEED_KMH_FALLBACK = 265.0
_LAH_ATTACK_ROUTE_MIN_LOOKAHEAD_S = 10.0
_LAH_RESUME_PRESERVE_TWO_POINT_MIN_LENGTH_M = 2000.0
_ATTACK_POINT_SUBPROCESS_TIMEOUT_S = 2.0
_ATTACK_LOS_MAX_RAYS = 360
_ATTACK_LOS_MIN_RAYS = 36
_ATTACK_LOS_MAX_RADIUS_M = 9000.0
_ATTACK_LOS_MIN_RADIUS_M = 500.0
_ATTACK_POINT_META_KEYS = (
    "selection_mode",
    "los_area",
    "raster_sources",
    "terrain_altitude_m",
    "altitude_offset_m",
    "los_verified",
    "los_required_altitude_m",
    "los_selected_altitude_m",
    "los_distance_m",
    "los_profile_sample_count",
    "los_dem_resolved_sample_count",
    "los_profile_step_m",
    "los_clearance_m",
    "los_target_height_m",
    "los_altitude_adjusted",
    "los_controlling_distance_m",
    "los_controlling_terrain_m",
    "los_profile_error",
    "analysis_radius_m",
    "num_rays",
    "lah_altitude_floor_m",
    "altitude_floor_applied",
    "friendly_target_direction_constrained",
    "mission_zone_constrained",
    "mission_zone_count",
    "mission_zone_source_plan_id",
    "mission_zone_input_mission_id",
    "mission_zone_watcher_id",
    "attack_point_at_hide_endpoint",
    "attack_point_vertical_popup",
    "attack_point_popup_offset_m",
    "attack_other_enemy_exposure_fallback",
    "attack_other_enemy_los_checked",
    "attack_other_enemy_considered_count",
    "attack_other_enemy_visible_count",
    "attack_other_enemy_unknown_count",
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
    "attack_tactical_no_engageable_target": "공격 보류: LOS 공격점 계산 불가",
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
    command_aircraft_id = get_runtime_attack_int("command_aircraft_id", 1)
    candidates = tuple(
        aircraft_id
        for aircraft_id in get_runtime_attack_int_list("manned_candidate_ids", [2, 3])
        if int(aircraft_id) != int(command_aircraft_id)
    )
    return candidates or (2, 3)


def warm_attack_plan_pipeline() -> Dict[str, Any]:
    """Preload lazy dependencies used by the attack replan path."""
    status: Dict[str, Any] = {"prior_pipeline": warm_prior_mission_pipeline()}
    try:
        from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
            warm_next_collab_replan_pipeline,
        )

        status["next_collab_pipeline"] = warm_next_collab_replan_pipeline()
    except Exception as exc:
        status["next_collab_pipeline"] = {"ready": False, "error": str(exc)}
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


def _attack_source_cache_enabled() -> bool:
    return (
        str(os.environ.get("REPLAN_ATTACK_SOURCE_CACHE", "1") or "").strip().lower()
        not in {"0", "false", "no", "off"}
    )


def _attack_collab_remaining_replan_mode() -> str:
    raw = os.environ.get("REPLAN_ATTACK_COLLAB_REMAINING_REPLAN")
    if raw is None:
        return "auto"
    lowered = str(raw or "").strip().lower()
    if lowered in {"1", "true", "yes", "on", "always"}:
        return "always"
    if lowered in {"0", "false", "no", "off", "disabled"}:
        return "off"
    return "auto"


def _attack_reuse_unaffected_uav_enabled() -> bool:
    return (
        str(os.environ.get("REPLAN_ATTACK_REUSE_UNAFFECTED_UAV", "1") or "").strip().lower()
        not in {"0", "false", "no", "off"}
    )


def _attack_resume_descriptor_uav_ids(
    *,
    configured_reuse: bool,
    other_uav_ids: Iterable[int],
    current_input_by_aircraft: Dict[int, Optional[int]],
    collaborative_input_ids: Iterable[int],
    type2_branch_line_aircraft_ids: Iterable[int],
) -> set[int]:
    """Select only UAVs whose individual package must be regenerated.

    With reuse enabled, collaborative redistribution touches only UAVs on the
    affected input, while Type-2 suffix refresh touches only aircraft currently
    executing an outbound/return branch LINE.  This prevents one aircraft's
    attack from rewriting a different aircraft that is already filming a guard
    AREA (or is on any ordinary input mission).
    """

    all_other_ids = {
        int(value) for value in other_uav_ids if _to_int(value) is not None
    }
    if not configured_reuse:
        return all_other_ids

    selected = {
        int(value)
        for value in type2_branch_line_aircraft_ids
        if _to_int(value) is not None and int(value) in all_other_ids
    }
    collab_ids = {
        int(value) for value in collaborative_input_ids if _to_int(value) is not None
    }
    if collab_ids:
        for aircraft_id in all_other_ids:
            current_input_id = _to_int(current_input_by_aircraft.get(int(aircraft_id)))
            # A missing lookup must retain the old safe fallback for a genuine
            # collaborative redivision; known unrelated inputs remain reused.
            if current_input_id is None or int(current_input_id) in collab_ids:
                selected.add(int(aircraft_id))
    return selected


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


def _attack_los_clearance_m() -> float:
    # This is a physical LOS policy shared with concealment and SIM, not a
    # deployment-PC tuning knob.  Letting a persisted local setting override
    # it made identical targets produce different attack altitudes and could
    # make one side of the same sightline report the opposite result.
    return float(LOS_CLEARANCE_M)


def _attack_los_target_height_m() -> float:
    return float(ENEMY_OBSERVER_HEIGHT_M)


def _attack_los_profile_step_m() -> float:
    value = get_runtime_attack_float("attack_los_profile_step_m", 0.0)
    return max(0.0, min(500.0, float(value))) if math.isfinite(float(value)) else 0.0


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
    los_clearance_m: float,
    los_target_height_m: float,
    los_profile_step_m: float,
    require_inside_mission_zone: bool = False,
    line_coverage_signature: Tuple[float, ...] = (),
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
        round(float(los_clearance_m), 1),
        round(float(los_target_height_m), 1),
        round(float(los_profile_step_m), 1),
        1.0 if require_inside_mission_zone else 0.0,
        *tuple(float(value) for value in line_coverage_signature),
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
        altitude_candidates = [float(terrain_altitude) + float(altitude_offset)]
        los_required = _to_float(result.get("los_required_altitude_m"))
        if bool(result.get("los_verified")) and los_required is not None and math.isfinite(los_required):
            altitude_candidates.append(float(los_required))
        result["altitude"] = int(math.ceil(max(altitude_candidates)))
        result["los_selected_altitude_m"] = int(result["altitude"])
        result.pop("lah_altitude_floor_m", None)
        result.pop("altitude_floor_applied", None)
    _apply_lah_altitude_floor(result, friendly_norm)
    return result


def _apply_attack_los_profile_altitude(
    result: Dict[str, Any],
    *,
    base_altitude_m: float,
    altitude_offset_m: float,
    los_profile: Any,
) -> Dict[str, Any]:
    """Apply the DEM-profile LOS requirement without another terrain lookup."""
    baseline_altitude_m = float(base_altitude_m) + float(altitude_offset_m)
    profile = los_profile if isinstance(los_profile, dict) else {}
    verified = bool(profile.get("verified"))
    required_altitude_m = _to_float(profile.get("required_altitude_m"))
    if required_altitude_m is None or not math.isfinite(required_altitude_m):
        verified = False
        required_altitude_m = None

    selected_altitude_m = float(baseline_altitude_m)
    if verified and required_altitude_m is not None:
        selected_altitude_m = max(selected_altitude_m, float(required_altitude_m))
    selected_altitude = int(math.ceil(selected_altitude_m))

    result["altitude"] = int(selected_altitude)
    result["terrain_altitude_m"] = _normalize_altitude_value(base_altitude_m)
    result["altitude_offset_m"] = float(altitude_offset_m)
    result["los_verified"] = bool(verified)
    result["los_required_altitude_m"] = (
        float(required_altitude_m) if required_altitude_m is not None else None
    )
    result["los_selected_altitude_m"] = int(selected_altitude)
    result["los_altitude_adjusted"] = bool(
        verified
        and required_altitude_m is not None
        and float(required_altitude_m) > float(baseline_altitude_m) + 1e-6
    )
    result["los_distance_m"] = _to_float(profile.get("distance_m"))
    result["los_profile_sample_count"] = _to_int(profile.get("sample_count"))
    result["los_profile_step_m"] = _to_float(profile.get("sample_step_m"))
    result["los_clearance_m"] = _to_float(profile.get("clearance_m"))
    result["los_target_height_m"] = _to_float(profile.get("target_height_m"))
    result["los_controlling_distance_m"] = _to_float(
        profile.get("controlling_distance_m")
    )
    result["los_controlling_terrain_m"] = _to_float(
        profile.get("controlling_terrain_m")
    )
    if not verified:
        result["los_profile_error"] = str(profile.get("reason") or "profile_unavailable")
    else:
        result.pop("los_profile_error", None)
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
    return bool(
        mode.startswith("los_area")
        or mode.startswith("special_")
        or (isinstance(value, dict) and value.get("los_verified") is True)
    )


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
    if special_force_battle_attack(profile, input_mission_id):
        return special_attack_coordinate(profile, input_mission_id=input_mission_id)
    # Type 2 각자도생: while the manned aircraft holds at 목표지역, anchor the
    # attack inside it; outside that window fall through to per-position default.
    gm_profile = ctx.get("_ground_maneuver_operation") if isinstance(ctx, dict) else None
    gm_anchor = ground_maneuver_target_attack_anchor(gm_profile, input_mission_id, aircraft_id)
    if gm_anchor is not None:
        return gm_anchor
    return None


def _compute_attack_point_subprocess(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    *,
    min_standoff_m: float,
    preferred_standoff_m: float,
    line_coverage_corridors: Optional[List[Dict[str, Any]]] = None,
    require_inside_mission_zone: bool = True,
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
    los_clearance_m = _attack_los_clearance_m()
    los_target_height_m = _attack_los_target_height_m()
    los_profile_step_m = _attack_los_profile_step_m()
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
        "--los-clearance-m",
        str(los_clearance_m),
        "--los-target-height-m",
        str(los_target_height_m),
        "--los-profile-step-m",
        str(los_profile_step_m),
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
    los_profile = payload.get("los_profile") or {}
    raster_sources = payload.get("raster_sources") or []
    raster_path = payload.get("raster_path")
    if raster_path and raster_path not in raster_sources:
        raster_sources = [raster_path, *raster_sources]
    los_coord = {
        "latitude": float(lat_val),
        "longitude": float(lon_val),
    }
    _apply_attack_los_profile_altitude(
        los_coord,
        base_altitude_m=float(base_altitude),
        altitude_offset_m=float(altitude_offset_m),
        los_profile=los_profile,
    )
    if not bool(los_coord.get("los_verified")):
        batch_los, _batch_los_error = _compute_attack_los_altitude_batch_dem(
            los_coord,
            enemy_coord,
            lah_floor_coord=friendly_coord,
        )
        if batch_los is not None:
            los_coord.update(batch_los)
    _apply_lah_altitude_floor(los_coord, friendly_coord)
    enemy_distance_m = _haversine_distance_m(enemy_coord, los_coord)
    if enemy_distance_m is None:
        return None, "LOS attack point distance check failed."
    if float(enemy_distance_m) < float(min_standoff_m):
        return None, (
            "LOS attack point rejected: "
            f"distance={float(enemy_distance_m):.1f}m < minStandoff={float(min_standoff_m):.1f}m"
        )
    if not _attack_point_between_friendly_and_enemy(los_coord, friendly_coord, enemy_coord):
        return None, "LOS attack point rejected: outside friendly-target segment."
    corridors = list(line_coverage_corridors or [])
    if (
        corridors
        and require_inside_mission_zone
        and not _attack_point_inside_line_coverage(los_coord, corridors, tolerance_m=0.0)
    ):
        return None, "LOS attack point rejected: outside photographed LINE coverage."
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
        "friendly_target_direction_constrained": True,
        "mission_zone_constrained": bool(corridors),
        "mission_zone_requirement": (
            "inside" if corridors and require_inside_mission_zone else "before_or_inside"
        ),
        "mission_zone_count": len(corridors),
    }
    for key in _ATTACK_POINT_META_KEYS:
        if key in los_coord:
            result[key] = deepcopy(los_coord.get(key))
    _apply_lah_altitude_floor(result, friendly_coord)
    return (result, None)


def _compute_attack_point_inprocess(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    *,
    min_standoff_m: float,
    preferred_standoff_m: float,
    line_coverage_corridors: Optional[List[Dict[str, Any]]] = None,
    require_inside_mission_zone: bool = True,
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
    los_clearance_m = _attack_los_clearance_m()
    los_target_height_m = _attack_los_target_height_m()
    los_profile_step_m = _attack_los_profile_step_m()
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
        los_profile = assist.minimum_attack_altitude_for_los(
            elevation,
            geotransform,
            (float(best_point[0]), float(best_point[1])),
            enemy_world,
            target_height_m=float(los_target_height_m),
            clearance_m=float(los_clearance_m),
            sample_step_m=(float(los_profile_step_m) if los_profile_step_m > 0.0 else None),
        )
    except Exception as exc:
        return None, f"In-process attack point calculation failed: {exc}"

    altitude_offset_m = get_runtime_attack_float("attack_point_altitude_offset_m", 300.0)
    fallback_base_altitude = _to_float(enemy_coord.get("altitude"))
    base_altitude = float(altitude) if math.isfinite(float(altitude)) else float(fallback_base_altitude or 0.0)
    los_coord = {
        "latitude": float(best_point[1]),
        "longitude": float(best_point[0]),
    }
    _apply_attack_los_profile_altitude(
        los_coord,
        base_altitude_m=float(base_altitude),
        altitude_offset_m=float(altitude_offset_m),
        los_profile=los_profile,
    )
    if not bool(los_coord.get("los_verified")):
        batch_los, _batch_los_error = _compute_attack_los_altitude_batch_dem(
            los_coord,
            enemy_coord,
            lah_floor_coord=friendly_coord,
        )
        if batch_los is not None:
            los_coord.update(batch_los)
    _apply_lah_altitude_floor(los_coord, friendly_coord)
    enemy_distance_m = _haversine_distance_m(enemy_coord, los_coord)
    if enemy_distance_m is None:
        return None, "LOS attack point distance check failed."
    if float(enemy_distance_m) < float(min_standoff_m):
        return None, (
            "LOS attack point rejected: "
            f"distance={float(enemy_distance_m):.1f}m < minStandoff={float(min_standoff_m):.1f}m"
        )
    if not _attack_point_between_friendly_and_enemy(los_coord, friendly_coord, enemy_coord):
        return None, "LOS attack point rejected: outside friendly-target segment."
    corridors = list(line_coverage_corridors or [])
    if (
        corridors
        and require_inside_mission_zone
        and not _attack_point_inside_line_coverage(los_coord, corridors, tolerance_m=0.0)
    ):
        return None, "LOS attack point rejected: outside photographed LINE coverage."

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
        "friendly_target_direction_constrained": True,
        "mission_zone_constrained": bool(corridors),
        "mission_zone_requirement": (
            "inside" if corridors and require_inside_mission_zone else "before_or_inside"
        ),
        "mission_zone_count": len(corridors),
    }
    for key in _ATTACK_POINT_META_KEYS:
        if key in los_coord:
            result[key] = deepcopy(los_coord.get(key))
    _apply_lah_altitude_floor(result, friendly_coord)
    return (result, None)


def _compute_attack_los_altitude_batch_dem(
    attack_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    *,
    lah_floor_coord: Optional[Dict[str, Any]] = None,
    altitude_offset_m: Optional[float] = None,
) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    """GDAL-independent LOS altitude path using the shared batched DEM cache."""
    attack_norm = _normalize_coordinate(attack_coord)
    enemy_norm = _normalize_coordinate(enemy_coord)
    if not attack_norm or not enemy_norm:
        return None, "Insufficient coordinate data for batched DEM LOS calculation."
    configured_step_m = _attack_los_profile_step_m()
    sample_step_m = float(configured_step_m) if configured_step_m > 0.0 else 10.0
    try:
        reset_terrain_elev_many_metrics()
        los_profile = profile_with_batch_dem(
            attack_norm,
            enemy_norm,
            terrain_elev_many,
            target_height_m=float(_attack_los_target_height_m()),
            clearance_m=float(_attack_los_clearance_m()),
            sample_step_m=float(sample_step_m),
            max_samples=1024,
        )
        dem_metrics = get_terrain_elev_many_metrics(reset=True)
    except Exception as exc:
        return None, f"Batched DEM LOS calculation failed: {exc}"

    sample_count = _to_int(los_profile.get("sample_count")) or 0
    resolved_count = _to_int(dem_metrics.get("demResolvedByTile")) or 0
    if sample_count <= 0 or resolved_count < sample_count:
        return None, (
            "Batched DEM LOS profile was not fully covered "
            f"(resolved={resolved_count}, samples={sample_count})."
        )
    if not bool(los_profile.get("verified")):
        return None, str(los_profile.get("reason") or "Batched DEM LOS profile unavailable.")
    base_altitude_m = _to_float(los_profile.get("attack_ground_m"))
    if base_altitude_m is None:
        return None, "Attack-point terrain altitude unavailable in batched DEM profile."

    result: Dict[str, Any] = {
        "latitude": float(attack_norm["latitude"]),
        "longitude": float(attack_norm["longitude"]),
        "raster_sources": [],
        "execution_mode": "shared_batch_dem_profile",
        "los_dem_resolved_sample_count": int(resolved_count),
    }
    _apply_attack_los_profile_altitude(
        result,
        base_altitude_m=float(base_altitude_m),
        altitude_offset_m=float(
            altitude_offset_m
            if altitude_offset_m is not None
            else get_runtime_attack_float("attack_point_altitude_offset_m", 300.0)
        ),
        los_profile=los_profile,
    )
    _apply_lah_altitude_floor(result, lah_floor_coord or attack_norm)
    return result, None


def _compute_direct_attack_los_altitude_inprocess(
    attack_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    *,
    lah_floor_coord: Optional[Dict[str, Any]] = None,
) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    """LOS-elevate a fixed attack point without running the radial point search."""
    assist, import_error = _load_attack_assist_module()
    if assist is None:
        fallback, fallback_error = _compute_attack_los_altitude_batch_dem(
            attack_coord,
            enemy_coord,
            lah_floor_coord=lah_floor_coord,
        )
        if fallback is not None:
            return fallback, None
        return None, (
            f"Attack assistance module unavailable: {import_error or 'unknown'}; "
            f"batchDem={fallback_error}"
        )
    attack_norm = _normalize_coordinate(attack_coord)
    enemy_norm = _normalize_coordinate(enemy_coord)
    if not attack_norm or not enemy_norm:
        return None, "Insufficient coordinate data for direct LOS altitude calculation."

    attack_world = (float(attack_norm["longitude"]), float(attack_norm["latitude"]))
    enemy_world = (float(enemy_norm["longitude"]), float(enemy_norm["latitude"]))
    distance_m = _haversine_distance_m(attack_norm, enemy_norm)
    if distance_m is None or not math.isfinite(distance_m) or distance_m <= 0.0:
        return None, "Invalid target-to-attack distance for direct LOS altitude calculation."
    analysis_radius_m = max(
        _ATTACK_LOS_MIN_RADIUS_M,
        min(_ATTACK_LOS_MAX_RADIUS_M, float(distance_m) + 300.0),
    )
    try:
        raster_paths = _detect_attack_raster_paths_cached(assist)
        elevation, geotransform, used_rasters = assist.load_elevation(
            raster_paths,
            enemy_world,
            radius_m=float(analysis_radius_m),
        )
        if not used_rasters or geotransform is None:
            raise RuntimeError("DEM unavailable for direct LOS altitude calculation.")
        base_altitude_m = assist.sample_elevation_at_world(
            elevation,
            attack_world,
            geotransform,
        )
        if not math.isfinite(float(base_altitude_m)):
            raise RuntimeError("Attack-point terrain altitude unavailable for direct LOS calculation.")
        profile_step_m = _attack_los_profile_step_m()
        los_profile = assist.minimum_attack_altitude_for_los(
            elevation,
            geotransform,
            attack_world,
            enemy_world,
            target_height_m=float(_attack_los_target_height_m()),
            clearance_m=float(_attack_los_clearance_m()),
            sample_step_m=(float(profile_step_m) if profile_step_m > 0.0 else None),
        )
    except Exception as exc:
        fallback, fallback_error = _compute_attack_los_altitude_batch_dem(
            attack_norm,
            enemy_norm,
            lah_floor_coord=lah_floor_coord,
        )
        if fallback is not None:
            return fallback, None
        return None, f"Direct LOS altitude calculation failed: {exc}; batchDem={fallback_error}"

    result: Dict[str, Any] = {
        "latitude": float(attack_norm["latitude"]),
        "longitude": float(attack_norm["longitude"]),
        "raster_sources": [os.path.abspath(str(path)) for path in (used_rasters or [])],
        "analysis_radius_m": float(analysis_radius_m),
        "execution_mode": "inprocess_direct_profile",
    }
    _apply_attack_los_profile_altitude(
        result,
        base_altitude_m=float(base_altitude_m),
        altitude_offset_m=float(
            get_runtime_attack_float("attack_point_altitude_offset_m", 300.0)
        ),
        los_profile=los_profile,
    )
    if not bool(result.get("los_verified")):
        fallback, fallback_error = _compute_attack_los_altitude_batch_dem(
            attack_norm,
            enemy_norm,
            lah_floor_coord=lah_floor_coord,
        )
        if fallback is not None:
            return fallback, None
        result["los_profile_error"] = str(
            result.get("los_profile_error") or fallback_error or "profile_unavailable"
        )
    _apply_lah_altitude_floor(result, lah_floor_coord or attack_norm)
    return result, None


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
    for transient_key in (
        "_preserved_source_attack_rows",
        "_preserved_lah_attack_aircraft_ids",
        "_incremental_attack_append",
    ):
        ctx.pop(transient_key, None)
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
    attack_source_plan_id = _resolve_attack_source_plan_id(ctx, detail_payload)
    committed_attack_rows, committed_attack_scan_errors = _load_committed_lah_attack_rows(
        attack_source_plan_id
    )
    if committed_attack_scan_errors:
        attack_log["result"]["attackContinuityScan"] = {
            "ok": False,
            "sourcePlanID": attack_source_plan_id,
            "errors": list(committed_attack_scan_errors),
        }
        _set_failure(
            "attack_source_continuity_unavailable",
            attack_failure_notice("attack_override_failed"),
            detail={"errors": committed_attack_scan_errors},
        )
        _emit(
            "[ATTACK][CONTINUITY][ERR] Source attack scan failed; preserving the "
            f"currently applied plan instead of risking target loss: {committed_attack_scan_errors[:3]}"
        )
        return _finish()
    committed_attack_aircraft_ids = {
        int(row["aircraftID"])
        for row in committed_attack_rows
        if _to_int(row.get("aircraftID")) is not None
    }
    # Attack assignment is intentionally limited to three simultaneous target
    # tasks, but concealment must account for every active threat supplied in
    # the contact bundle.  Keep the uncapped list separate from the attackers'
    # assignment list so a fourth enemy cannot disappear from LAH LOS checks.
    enemy_contact_targets = [dict(item) for item in bundle_targets]
    if detail_override and not any(
        _same_target_identity(detail_override, item) for item in enemy_contact_targets
    ):
        enemy_contact_targets.insert(0, dict(detail_override))
    # The monitor bundle is the freshest source, but targetInfo can contain
    # additional still-active contacts observed by another UAV.  Tactical LOS
    # must not silently collapse to the last/assigned target, so merge every
    # known live coordinate while keeping the attack assignment cap separate.
    for tracked_target in target_entries:
        if not isinstance(tracked_target, dict):
            continue
        if bool(tracked_target.get("is_destroyed")):
            continue
        if _normalize_coordinate(tracked_target.get("coordinate")) is None:
            continue
        if any(
            _same_target_identity(tracked_target, existing)
            for existing in enemy_contact_targets
        ):
            continue
        enemy_contact_targets.append(dict(tracked_target))

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

    already_committed_targets: List[Dict[str, Any]] = []
    capacity_deferred_targets: List[Dict[str, Any]] = []
    if committed_attack_rows:
        (
            selected_targets,
            already_committed_targets,
            capacity_deferred_targets,
        ) = _partition_novel_attack_targets(
            selected_targets,
            committed_attack_rows,
            maximum_target_count=3,
        )
        attack_log["result"]["attackContinuityScan"] = {
            "ok": True,
            "sourcePlanID": attack_source_plan_id,
            "committedAttacks": [dict(row) for row in committed_attack_rows],
            "busyMannedAircraftIDs": sorted(committed_attack_aircraft_ids),
            "alreadyCommittedTargetIDs": [
                _to_int(item.get("target_id") or item.get("targetID"))
                for item in already_committed_targets
            ],
            "capacityDeferredTargetIDs": [
                _to_int(item.get("target_id") or item.get("targetID"))
                for item in capacity_deferred_targets
            ],
        }
        _emit(
            "[ATTACK][CONTINUITY] Preserving committed attack identity "
            f"(sourcePlan={attack_source_plan_id}, "
            f"attacks={[(row.get('aircraftID'), row.get('targetID')) for row in committed_attack_rows]})."
        )
        if capacity_deferred_targets:
            _emit(
                "[ATTACK][CONTINUITY] New target(s) deferred because the three-target "
                "capacity is already reserved -> "
                f"{[_to_int(item.get('target_id') or item.get('targetID')) for item in capacity_deferred_targets]}."
            )
        if not selected_targets:
            attack_log["result"]["status"] = "preserved_existing_attacks"
            attack_log["result"]["deferred_attack_targets"] = [
                dict(item) for item in capacity_deferred_targets
            ]
            _emit(
                "[ATTACK][CONTINUITY] No safe novel attack remains; current attack "
                "IMP/path/WP graph stays applied without regeneration."
            )
            return _finish()

    primary_target = dict(selected_targets[0]) if selected_targets else None

    attack_log["result"]["primary_target"] = primary_target
    attack_log["result"]["attack_targets"] = selected_targets
    attack_log["result"]["target_count"] = len(selected_targets)

    multi_target_mode = len(selected_targets) > 1
    incremental_append: Optional[Dict[str, Any]] = None
    if committed_attack_rows:
        _safe_selected, safe_candidates, _safe_reason = _select_preferred_manned_aircrafts(
            agent_states,
            input_package_id=None,
            max_count=max(1, len(_attack_manned_candidates())),
            allow_used_reuse=True,
        )
        selected_manned_aircraft = [
            dict(item)
            for item in safe_candidates
            if _to_int(item.get("aircraft_id")) not in committed_attack_aircraft_ids
        ]
        if not selected_manned_aircraft:
            (
                append_candidate,
                append_committed_row,
                append_reason,
            ) = _select_incremental_attack_append_candidate(
                committed_attack_rows,
                selected_targets,
                safe_candidates,
            )
            if append_candidate is None or append_committed_row is None:
                attack_log["result"]["status"] = "deferred_until_attack_slot_free"
                attack_log["result"]["deferred_attack_targets"] = [
                    dict(item) for item in selected_targets
                ]
                attack_log["result"]["incrementalAppend"] = {
                    "eligible": False,
                    "reason": str(append_reason),
                }
                _emit(
                    "[ATTACK][CONTINUITY] Both armed LAH attack slots are carrying "
                    "committed paths and append safety was not proven "
                    f"(reason={append_reason}); preserving the applied graph and "
                    "deferring the novel target."
                )
                return _finish()
            selected_manned_aircraft = [dict(append_candidate)]
            append_aircraft_id = int(append_candidate["aircraft_id"])
            incremental_append = {
                "aircraftID": int(append_aircraft_id),
                "sourcePlanID": int(attack_source_plan_id or 0),
                "committedRow": dict(append_committed_row),
            }
            attack_log["result"]["incrementalAppend"] = {
                "eligible": True,
                "aircraftID": int(append_aircraft_id),
                "committedTargetID": _to_int(append_committed_row.get("targetID")),
                "novelTargetID": _to_int(
                    selected_targets[0].get("target_id")
                    or selected_targets[0].get("targetID")
                ),
            }
            _emit(
                "[ATTACK][CONTINUITY] Third target will be appended behind the "
                "existing attack without regenerating either committed path "
                f"(aircraft={append_aircraft_id}, committedTarget="
                f"{append_committed_row.get('targetID')})."
            )
        best_aircraft = dict(selected_manned_aircraft[0])
        friendly_coord = best_aircraft.get("coordinate")
        attack_log["result"]["selected_manned_aircraft"] = selected_manned_aircraft
        if incremental_append is None:
            _emit(
                "[ATTACK][CONTINUITY] Novel attack assigned only to free LAH(s): "
                + ", ".join(str(item.get("aircraft_id")) for item in selected_manned_aircraft)
            )
    elif multi_target_mode:
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

    attack_watcher_id = _to_int(primary_target.get("watcher_id"))
    attack_mission_zones, attack_mission_zone_summary = _load_attack_line_coverage_corridors(
        source_plan_id=attack_source_plan_id,
        watcher_id=attack_watcher_id,
        target_coord=primary_target.get("coordinate"),
    )
    attack_log["result"]["attack_mission_zone_constraint"] = attack_mission_zone_summary
    _emit(
        "[ATTACK][POINT] Mission-zone constraint "
        f"sourcePlan={attack_source_plan_id} watcher={attack_watcher_id} "
        f"zones={len(attack_mission_zones)} "
        f"reason={attack_mission_zone_summary.get('reason')}."
    )

    attack_point_started = time.perf_counter()
    attack_point_cache_stats: Dict[str, Any] = {}
    attack_point, attack_error = _compute_attack_point(
        friendly_coord,
        primary_target["coordinate"],
        friendly_heading_deg=(best_aircraft or {}).get("heading"),
        friendly_speed_mps=(best_aircraft or {}).get("speed"),
        cache_stats=attack_point_cache_stats,
        line_coverage_corridors=attack_mission_zones,
        line_coverage_metadata=attack_mission_zone_summary,
    )
    attack_log["result"]["attack_point"] = attack_point
    attack_log["result"]["attack_point_cache"] = attack_point_cache_stats
    attack_point_elapsed_ms = _record_timing("compute_attack_point", attack_point_started)
    mission_updates: Optional[Dict[str, Any]] = None
    if attack_point:
        altitude_display = (
            f"alt={attack_point['altitude']}m" if attack_point.get("altitude") is not None else "alt=unknown"
        )
        los_display = (
            f", los=verified, losDistance={float(attack_point.get('los_distance_m')):.1f}m, "
            f"losRequiredAlt={float(attack_point.get('los_required_altitude_m')):.1f}m"
            if attack_point.get("los_verified") is True
            and _to_float(attack_point.get("los_distance_m")) is not None
            and _to_float(attack_point.get("los_required_altitude_m")) is not None
            else f", los=unverified({attack_point.get('los_profile_error') or 'disabled_or_unavailable'})"
        )
        _emit(
            "STEP3 Attack plan completed: "
            f"lat={attack_point['latitude']:.6f}, lon={attack_point['longitude']:.6f}, "
            f"{altitude_display}, mode={attack_point.get('selection_mode') or 'unknown'}"
            f"{los_display}"
        )
        if attack_point.get("mission_zone_requirement_relaxed") or attack_point_cache_stats.get(
            "missionZoneRequirementRelaxed"
        ):
            standoff_m = _to_float(attack_point.get("enemy_distance_m"))
            standoff_text = (
                f"standoff={standoff_m:.0f}m" if standoff_m is not None else "standoff=unknown"
            )
            _emit(
                "[ATTACK][POINT][WARN] No point met the minimum standoff inside the "
                f"photographed corridor; the attack point sits before it ({standoff_text})."
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
        ctx["_preserved_source_attack_rows"] = [
            dict(item) for item in committed_attack_rows
        ]
        ctx["_preserved_lah_attack_aircraft_ids"] = sorted(
            committed_attack_aircraft_ids
        )
        if incremental_append is not None:
            ctx["_incremental_attack_append"] = deepcopy(incremental_append)
        else:
            ctx.pop("_incremental_attack_append", None)
        ctx["_enemy_contact_target_list"] = [
            dict(item)
            for item in (enemy_contact_targets or selected_targets)
            if isinstance(item, dict)
        ]
        ctx["_selected_manned_aircraft"] = [dict(item) for item in selected_manned_aircraft]
        attack_source_cache_stats: Dict[str, Any] = {}
        if _attack_source_cache_enabled():
            active_source_cache = get_active_source_artifact_cache()
            attack_source_cache = active_source_cache or SourceArtifactCache()
            if active_source_cache is not None:
                mission_updates = _apply_attack_plan_overrides(
                    ctx=ctx,
                    attack_point=attack_point,
                    manned_aircraft=best_aircraft,
                    primary_target=primary_target,
                    agent_states=agent_states,
                    waypoint_memory=agent_snapshot.get("last_nonzero_waypoint_by_aircraft"),
                    emit=_emit,
                )
            else:
                with use_source_artifact_cache(attack_source_cache):
                    mission_updates = _apply_attack_plan_overrides(
                        ctx=ctx,
                        attack_point=attack_point,
                        manned_aircraft=best_aircraft,
                        primary_target=primary_target,
                        agent_states=agent_states,
                        waypoint_memory=agent_snapshot.get("last_nonzero_waypoint_by_aircraft"),
                        emit=_emit,
                    )
            attack_source_cache_stats = attack_source_cache.stats()
        else:
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
            if attack_source_cache_stats:
                mission_updates["timingMs"]["sourceArtifactCache"] = dict(attack_source_cache_stats)
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
    source_artifact_cache = SourceArtifactCache()

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
        plan_data = source_artifact_cache.read_json(plan_src, kind="MissionPlan")
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
    # plan_data는 read_json_cached 반환본(호출자 소유)이고 이후 재사용 없음
    # Keep the cached source MissionPlan immutable.  The exclusion candidate is
    # rewired to a completely fresh artifact graph before it is published.
    new_plan_data = deepcopy(plan_data)
    new_plan_data["missionPlanID"] = new_plan_id
    new_plan_data["timestamp"] = now_ms
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = now_ms

    resume_index_started = time.perf_counter()
    attack_exclusion_aircraft_ids: List[int] = []
    resume_candidate_plan_ids: List[Any] = [int(source_plan_id)]
    missing_resume_aircraft_ids: List[int] = []
    for entry in new_plan_data.get("aircraftList", []):
        aircraft_id = _to_int((entry or {}).get("aircraftID")) if isinstance(entry, dict) else None
        if aircraft_id is None or aircraft_id <= 3:
            continue
        attack_exclusion_aircraft_ids.append(int(aircraft_id))
        state = agent_index.get(int(aircraft_id)) or {}
        if _to_int(state.get("current_waypoint_id")) is not None:
            continue
        missing_resume_aircraft_ids.append(int(aircraft_id))
        progress_state = _load_latest_mission_progress_state(int(aircraft_id)) or {}
        progress_plan_id = _to_int(progress_state.get("currentMissionPlanID"))
        if progress_plan_id is not None:
            resume_candidate_plan_ids.append(int(progress_plan_id))
    resume_index: Dict[int, List[Dict[str, Any]]] = {}
    if missing_resume_aircraft_ids:
        resume_candidate_plan_ids.extend(
            [
                _load_latest_mission_progress_plan_id(),
                _scan_latest_source_plan_id(),
            ]
        )
        resume_index = _build_attack_exclusion_resume_index(
            resume_candidate_plan_ids,
            missing_resume_aircraft_ids,
        )
    phase_timer.mark("resume_index_build")
    _emit(
        "ATTACK-EXCLUDE resume index built "
        f"(plans={len({int(pid) for pid in resume_candidate_plan_ids if _to_int(pid) is not None})}, "
        f"aircraft={len(missing_resume_aircraft_ids)}, "
        f"rows={sum(len(rows or []) for rows in resume_index.values())}, "
        f"elapsedMs={_elapsed_ms(resume_index_started):.3f})"
    )

    aircraft_updates: List[Dict[str, Any]] = []
    unchanged_aircraft: List[int] = []
    deferred_tracking_clear_aircraft_ids: set[int] = set()
    manned_return_aircraft_ids: set[int] = set()
    failed_manned_return_aircraft_ids: set[int] = set()
    aircraft_loop_started = time.perf_counter()
    recovery_by_aircraft: Dict[int, Optional[Dict[str, Any]]] = {}
    update_future_by_aircraft: Dict[int, concurrent.futures.Future] = {}
    aircraft_update_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
    parallel_entries = [
        entry
        for entry in new_plan_data.get("aircraftList", [])
        if isinstance(entry, dict) and (_to_int(entry.get("aircraftID")) or 0) > 3
    ]
    if len(parallel_entries) > 1:
        parallel_job_specs: List[Dict[str, Any]] = []
        for parallel_entry in parallel_entries:
            parallel_aircraft_id = int(_to_int(parallel_entry.get("aircraftID")) or 0)
            parallel_state = agent_index.get(parallel_aircraft_id) or {}
            parallel_current_wp = _to_int(parallel_state.get("current_waypoint_id"))
            parallel_current_coord = (
                parallel_state.get("coordinate") if isinstance(parallel_state, dict) else None
            )
            parallel_recovery = _resolve_attack_tracking_recovery(
                aircraft_id=parallel_aircraft_id,
                source_plan_id=int(source_plan_id),
                current_coord=parallel_current_coord,
                emit=_emit,
            )
            recovery_by_aircraft[parallel_aircraft_id] = parallel_recovery
            if parallel_recovery is not None:
                parallel_job_specs.append(
                    {
                        "aircraft_id": int(parallel_aircraft_id),
                        "source_plan_id": int(parallel_recovery["source_plan_id"]),
                        "current_waypoint_id": _to_int(parallel_recovery["split_waypoint_id"]),
                        "current_coord": parallel_recovery.get("done_anchor_coord"),
                    }
                )
                continue
            if parallel_current_wp is None:
                continue
            parallel_resume_plan_id = _resolve_attack_exclusion_source_plan_id(
                source_plan_id=int(source_plan_id),
                aircraft_id=parallel_aircraft_id,
                current_waypoint_id=parallel_current_wp,
                emit=_emit,
            )
            if parallel_resume_plan_id is None:
                continue
            parallel_job_specs.append(
                {
                    "aircraft_id": int(parallel_aircraft_id),
                    "source_plan_id": int(parallel_resume_plan_id),
                    "current_waypoint_id": parallel_current_wp,
                    "current_coord": parallel_current_coord,
                }
            )

        # Every parallel resume package needs one IMP, one mission, one path,
        # and N+2 waypoint IDs.  Reserve those blocks together before workers
        # start so they never serialize on the global ID file lock.
        reservation_specs: List[Tuple[Dict[str, Any], int]] = []
        for job_spec in parallel_job_specs:
            try:
                prepass_artifacts = call_with_source_artifact_cache(
                    source_artifact_cache,
                    _resolve_plan_artifacts,
                    source_plan_id=int(job_spec["source_plan_id"]),
                    aircraft_id=int(job_spec["aircraft_id"]),
                    current_waypoint_id=_to_int(job_spec.get("current_waypoint_id")),
                    emit=lambda _message: None,
                    allow_first_mission_fallback=False,
                )
                if prepass_artifacts is None:
                    continue
                prepass_fp = source_artifact_cache.read_json(
                    db_paths.get_db_subpath("FlightPath", f"{int(prepass_artifacts.path_id)}.json"),
                    kind="FlightPath",
                )
                waypoint_count = max(1, len(prepass_fp.get("waypointList") or []) + 2)
            except Exception:
                continue
            reservation_specs.append((job_spec, int(waypoint_count)))

        if reservation_specs:
            bulk_reservation = ReplanIdReservation.reserve(
                imp_count=len(reservation_specs),
                individual_count=len(reservation_specs),
                path_count_by_aircraft={
                    int(job_spec["aircraft_id"]): 1
                    for job_spec, _waypoint_count in reservation_specs
                },
                waypoint_count=sum(
                    int(waypoint_count)
                    for _job_spec, waypoint_count in reservation_specs
                ),
            )
            for job_spec, waypoint_count in reservation_specs:
                aircraft_id = int(job_spec["aircraft_id"])
                job_spec["id_reservation"] = ReplanIdReservation(
                    imp_ids=ReservedIdBlock(
                        "individualMissionPackage",
                        [bulk_reservation.next_imp()],
                    ),
                    individual_ids=ReservedIdBlock(
                        "individualMission",
                        [bulk_reservation.next_individual()],
                    ),
                    waypoint_ids=ReservedIdBlock(
                        "waypoint",
                        [
                            bulk_reservation.next_waypoint()
                            for _ in range(int(waypoint_count))
                        ],
                    ),
                    path_ids_by_aircraft={
                        int(aircraft_id): ReservedIdBlock(
                            f"pathID[{int(aircraft_id)}]",
                            [bulk_reservation.next_path(int(aircraft_id))],
                        )
                    },
                )

        if parallel_job_specs:
            aircraft_update_executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=min(3, len(parallel_job_specs)),
                thread_name_prefix="AttackExcludeUAV",
            )
            for job_spec in parallel_job_specs:
                parallel_aircraft_id = int(job_spec["aircraft_id"])
                update_future_by_aircraft[parallel_aircraft_id] = aircraft_update_executor.submit(
                    call_with_source_artifact_cache,
                    source_artifact_cache,
                    _build_other_uav_resume_package,
                    source_plan_id=int(job_spec["source_plan_id"]),
                    aircraft_id=parallel_aircraft_id,
                    current_waypoint_id=_to_int(job_spec.get("current_waypoint_id")),
                    current_coord=job_spec.get("current_coord"),
                    emit=_emit,
                    now_ms=now_ms,
                    sweep_progress=sweep_progress,
                    clone_follow_up_artifacts=True,
                    preserve_follow_up_artifacts=True,
                    drop_prefix_missions=True,
                    allow_first_mission_fallback=False,
                    include_done_reference_mission=False,
                    id_reservation=job_spec.get("id_reservation"),
                )

    def _take_parallel_update(aircraft_id: int) -> Optional[Dict[str, Any]]:
        future = update_future_by_aircraft.get(int(aircraft_id))
        if future is None:
            return None
        try:
            return future.result()
        except Exception as exc:
            _emit(
                "ATTACK-EXCLUDE parallel aircraft update failed "
                f"(aircraft={aircraft_id}, error={exc!r})."
            )
            return None

    # Excluding the target this option was raised for must not cancel an
    # engagement already under way against a different target: that aircraft
    # would drop its attack, the other target would stay tracked but never shot,
    # and the operator sees the attack simply vanish.
    excluded_target_ids = _attack_exclusion_requested_target_ids(ctx)
    retained_engagement_target_ids = sorted(
        target_id
        for target_id in _actively_tracked_target_ids()
        if target_id not in excluded_target_ids
    )
    if retained_engagement_target_ids:
        _emit(
            "ATTACK-EXCLUDE keeping engagements on still-tracked targets "
            f"{retained_engagement_target_ids} (excluding {sorted(excluded_target_ids) or 'all'})."
        )

    for entry in new_plan_data.get("aircraftList", []):
        aircraft_id = _to_int(entry.get("aircraftID"))
        if aircraft_id is None:
            continue
        if aircraft_id <= 3:
            lah_exclusion_context = _resolve_global_attack_exclusion_lah_context(
                source_plan_id=int(source_plan_id),
                aircraft_id=int(aircraft_id),
            )
            if lah_exclusion_context is None:
                unchanged_aircraft.append(aircraft_id)
                continue
            try:
                from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
                    _build_post_attack_lah_resume_update,
                )

                lah_return_update = _build_post_attack_lah_resume_update(
                    source_plan_id=int(source_plan_id),
                    current_input_id=int(lah_exclusion_context["current_input_id"]),
                    target_id=int(lah_exclusion_context["target_ids"][0]),
                    aircraft_id=int(aircraft_id),
                    current_state=agent_index.get(int(aircraft_id)) or {},
                    now_ms=int(now_ms),
                    emit=_emit,
                    log_prefix="[ATTACK-EXCLUDE][LAH-RETURN]",
                    exclude_all_target_missions=True,
                    retained_target_ids=retained_engagement_target_ids,
                )
            except Exception as exc:
                _emit(
                    "ATTACK-EXCLUDE LAH return generation failed "
                    f"(aircraft={aircraft_id}, error={exc!r})."
                )
                lah_return_update = None
            if not isinstance(lah_return_update, dict):
                failed_manned_return_aircraft_ids.add(int(aircraft_id))
                continue
            new_imp_id = _to_int(lah_return_update.get("individualMissionPackageID"))
            if new_imp_id is None or new_imp_id <= 0:
                failed_manned_return_aircraft_ids.add(int(aircraft_id))
                continue
            entry["individualMissionPackageID"] = int(new_imp_id)
            aircraft_updates.append(lah_return_update)
            manned_return_aircraft_ids.add(int(aircraft_id))
            _emit(
                "ATTACK-EXCLUDE manned attack branch removed and return mission applied "
                f"(aircraft={aircraft_id}, targets={lah_exclusion_context['target_ids']})."
            )
            continue

        state = agent_index.get(aircraft_id) or {}
        current_wp = _to_int(state.get("current_waypoint_id"))
        current_coord = state.get("coordinate") if isinstance(state, dict) else None
        aircraft_update_started = time.perf_counter()
        _emit(
            "ATTACK-EXCLUDE aircraft update start "
            f"(aircraft={aircraft_id}, currentWP={current_wp}, sourcePlanHint={source_plan_id})."
        )
        recovery = recovery_by_aircraft.get(int(aircraft_id))
        if int(aircraft_id) not in recovery_by_aircraft:
            recovery = _resolve_attack_tracking_recovery(
                aircraft_id=int(aircraft_id),
                source_plan_id=int(source_plan_id),
                current_coord=current_coord,
                emit=_emit,
            )
        if recovery is not None:
            update = _take_parallel_update(int(aircraft_id))
            if int(aircraft_id) not in update_future_by_aircraft:
                update = call_with_source_artifact_cache(
                    source_artifact_cache,
                    _build_other_uav_resume_package,
                    source_plan_id=int(recovery["source_plan_id"]),
                    aircraft_id=int(aircraft_id),
                    current_waypoint_id=_to_int(recovery["split_waypoint_id"]),
                    current_coord=recovery.get("done_anchor_coord"),
                    emit=_emit,
                    now_ms=now_ms,
                    sweep_progress=sweep_progress,
                    clone_follow_up_artifacts=True,
                    preserve_follow_up_artifacts=True,
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

        update = _take_parallel_update(int(aircraft_id))
        if int(aircraft_id) not in update_future_by_aircraft:
            update = call_with_source_artifact_cache(
                source_artifact_cache,
                _build_other_uav_resume_package,
                source_plan_id=int(resume_source_plan_id),
                aircraft_id=int(aircraft_id),
                current_waypoint_id=current_wp,
                current_coord=current_coord,
                emit=_emit,
                now_ms=now_ms,
                sweep_progress=sweep_progress,
                clone_follow_up_artifacts=True,
                preserve_follow_up_artifacts=True,
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
    if aircraft_update_executor is not None:
        aircraft_update_executor.shutdown(wait=True, cancel_futures=False)
    phase_timer.mark("aircraft_update_loop")

    if failed_manned_return_aircraft_ids:
        _emit(
            "공격 배제 계획에서 유인기 복귀 임무를 완전하게 만들지 못해 부분 배제 계획을 폐기합니다. "
            f"(aircraft={sorted(failed_manned_return_aircraft_ids)})"
        )
        result_payload["result"] = {
            "error": "manned_return_generation_failed",
            "sourcePlanID": int(source_plan_id),
            "missionPlanID": int(new_plan_id),
            "failedMannedAircraftIDs": sorted(failed_manned_return_aircraft_ids),
        }
        return _finish_result()

    no_update_plan = False
    if not aircraft_updates:
        no_update_plan = True
        _emit(
            "공격 배제용 UAV 재개 임무가 생성되지 않아 "
            "기존 개별임무를 유지한 변경 없음 MissionPlan을 저장합니다."
        )

    # External consumers treat each exclusion option as a wholly new artifact
    # graph. Resume builders intentionally preserve unchanged follow-up IDs for
    # speed, and unchanged LAHs used to retain their source package IDs. Clone
    # the finished candidate once here so all published IDs are fresh while
    # mission content, ordering, and completion state remain unchanged.
    fresh_id_started = time.perf_counter()
    try:
        fresh_id_summary = _freshen_attack_exclusion_artifact_ids(
            new_plan_data,
            now_ms=int(now_ms),
            emit=_emit,
        )
    except Exception as exc:
        phase_timer.mark("artifact_id_refresh_failed")
        _emit(f"ATTACK-EXCLUDE fresh artifact ID allocation failed: {exc}")
        result_payload["result"] = {
            "error": "artifact_id_refresh_failed",
            "sourcePlanID": int(source_plan_id),
            "missionPlanID": int(new_plan_id),
            "detail": str(exc),
        }
        return _finish_result()
    final_package_by_aircraft = {
        int(row["aircraftID"]): int(row["individualMissionPackageID"])
        for row in fresh_id_summary.get("aircraftArtifacts", [])
        if isinstance(row, dict)
        and _to_int(row.get("aircraftID")) is not None
        and _to_int(row.get("individualMissionPackageID")) is not None
    }
    for update in aircraft_updates:
        if not isinstance(update, dict):
            continue
        aircraft_id = _to_int(update.get("aircraft_id") or update.get("aircraftID"))
        final_package_id = final_package_by_aircraft.get(int(aircraft_id or 0))
        if final_package_id is None:
            continue
        previous_package_id = _to_int(update.get("individualMissionPackageID"))
        if previous_package_id is not None and previous_package_id != final_package_id:
            update["intermediateIndividualMissionPackageID"] = int(previous_package_id)
        update["individualMissionPackageID"] = int(final_package_id)
    phase_timer.mark("artifact_id_refresh")
    _emit(
        "ATTACK-EXCLUDE refreshed every aircraft artifact ID "
        f"(aircraft={len(fresh_id_summary.get('aircraftArtifacts') or [])}, "
        f"missions={fresh_id_summary.get('individualMissionCount', 0)}, "
        f"paths={fresh_id_summary.get('pathCount', 0)}, "
        f"waypoints={fresh_id_summary.get('waypointCount', 0)}, "
        f"elapsedMs={_elapsed_ms_detail(fresh_id_started):.3f})."
    )

    # An exclusion plan is defined by having no tracking in it.  The per-aircraft
    # detach above only fires when the tracking assignment's attack_plan_id
    # matches the plan being excluded from, so tracking inherited from an earlier
    # replan could survive.  Sweep the finished plan and drop whatever is left.
    stripped_tracking = _strip_tracking_from_exclusion_plan(new_plan_data, emit=_emit)
    if stripped_tracking:
        result_payload["result"]["strippedTrackingMissions"] = stripped_tracking

    validation_started = time.perf_counter()
    try:
        validation_summary = call_with_source_artifact_cache(
            source_artifact_cache,
            validate_replan_payloads,
            mission_plan=new_plan_data,
            scope="attack_exclusion",
            allow_existing_db_artifacts=True,
            validate_existing_flight_path_waypoints=False,
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

    _queue_attack_snapshot_carry(
        int(source_plan_id),
        int(new_plan_id),
        reason="attack_exclusion",
    )
    _emit(
        "ATTACK-EXCLUDE area remaining snapshot carry queued "
        f"(sourcePlanID={source_plan_id}, planID={new_plan_id})."
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
            "mannedReturnAircraftIDs": sorted(manned_return_aircraft_ids),
        },
        "freshArtifactIDs": fresh_id_summary,
    }
    if stripped_tracking:
        result_payload["result"]["strippedTrackingMissions"] = stripped_tracking
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


def _partition_novel_attack_targets(
    requested_targets: List[Dict[str, Any]],
    committed_attack_rows: List[Dict[str, Any]],
    *,
    maximum_target_count: int = 3,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Keep current attack commands intact and select only genuinely new work.

    Returns ``(novel, already_committed, deferred_by_capacity)``.  Existing
    target IDs consume the package's three-target capacity first because their
    IMP/path/WP identity must survive a later detection unchanged.
    """

    committed_target_ids = {
        int(target_id)
        for target_id in (
            _to_int(row.get("targetID"))
            for row in committed_attack_rows or []
            if isinstance(row, dict)
        )
        if target_id is not None and int(target_id) > 0
    }
    capacity = max(0, int(maximum_target_count) - len(committed_target_ids))
    novel: List[Dict[str, Any]] = []
    already_committed: List[Dict[str, Any]] = []
    deferred: List[Dict[str, Any]] = []

    for raw_target in requested_targets or []:
        if not isinstance(raw_target, dict):
            continue
        target = dict(raw_target)
        target_id = _to_int(target.get("target_id") or target.get("targetID"))
        if target_id is not None and int(target_id) in committed_target_ids:
            already_committed.append(target)
            continue
        if any(_same_target_identity(target, existing) for existing in novel):
            already_committed.append(target)
            continue
        if len(novel) < capacity:
            novel.append(target)
        else:
            deferred.append(target)
    return novel, already_committed, deferred


def _select_incremental_attack_append_candidate(
    committed_attack_rows: List[Dict[str, Any]],
    novel_targets: List[Dict[str, Any]],
    manned_candidates: List[Dict[str, Any]],
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]], str]:
    """Choose one busy LAH for the only safe incremental-append shape.

    Appending is deliberately narrower than ordinary attack assignment.  It is
    allowed only when LAH2 and LAH3 each own exactly one unfinished attack and
    exactly one novel target fills the third (and final) simultaneous slot.
    The already-committed weapon is reserved from the live inventory before
    the new target is tested, so an in-flight shot can never be double-spent.

    Returns ``(candidate, committed_row, reason)``.  ``candidate`` carries the
    post-reservation inventory that the normal manned assignment code consumes.
    """

    rows = [dict(row) for row in committed_attack_rows or [] if isinstance(row, dict)]
    targets = [dict(target) for target in novel_targets or [] if isinstance(target, dict)]
    if len(targets) != 1:
        return None, None, "append_requires_exactly_one_novel_target"
    if len(rows) != 2:
        return None, None, "append_requires_exactly_two_committed_attacks"

    rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    committed_target_ids: set[int] = set()
    for row in rows:
        aircraft_id = _to_int(row.get("aircraftID"))
        target_id = _to_int(row.get("targetID"))
        if aircraft_id not in (2, 3) or target_id is None or target_id <= 0:
            return None, None, "append_committed_identity_invalid"
        rows_by_aircraft.setdefault(int(aircraft_id), []).append(row)
        committed_target_ids.add(int(target_id))
    if set(rows_by_aircraft) != {2, 3} or any(
        len(rows_by_aircraft.get(aircraft_id, [])) != 1 for aircraft_id in (2, 3)
    ):
        return None, None, "append_requires_one_committed_attack_per_lah"
    if len(committed_target_ids) != 2:
        return None, None, "append_committed_targets_not_unique"

    novel_target = targets[0]
    novel_target_id = _to_int(novel_target.get("target_id") or novel_target.get("targetID"))
    if novel_target_id is None or novel_target_id <= 0 or int(novel_target_id) in committed_target_ids:
        return None, None, "append_novel_target_identity_invalid"

    scored: List[Tuple[Tuple[float, int, int], Dict[str, Any], Dict[str, Any]]] = []
    target_coord = _normalize_coordinate(
        novel_target.get("coordinate") or novel_target.get("attack_coord")
    )
    for order_index, raw_candidate in enumerate(manned_candidates or []):
        if not isinstance(raw_candidate, dict):
            continue
        aircraft_id = _to_int(raw_candidate.get("aircraft_id") or raw_candidate.get("aircraftID"))
        if aircraft_id not in (2, 3) or int(aircraft_id) not in rows_by_aircraft:
            continue
        committed_row = rows_by_aircraft[int(aircraft_id)][0]
        committed_weapon_type = _to_int(committed_row.get("weaponType"))
        if committed_weapon_type not in (1, 2, 3):
            # Without weapon provenance we cannot know whether the live count
            # already includes the pending shot.  Defer instead of guessing.
            continue
        remaining_inventory = extract_attack_weapon_inventory(raw_candidate)
        committed_slot = f"type{int(committed_weapon_type)}"
        if int(remaining_inventory.get(committed_slot, 0)) <= 0:
            continue
        remaining_inventory[committed_slot] = max(
            0, int(remaining_inventory.get(committed_slot, 0)) - 1
        )
        weapon_choice = _resolve_attack_weapon_choice_for_inventory(
            novel_target,
            remaining_inventory,
        )
        if not bool(weapon_choice.get("ammoAvailable")):
            continue

        candidate = dict(raw_candidate)
        candidate["aircraft_id"] = int(aircraft_id)
        candidate["weapon_inventory"] = dict(remaining_inventory)
        candidate["_reserved_committed_weapon_type"] = int(committed_weapon_type)
        candidate["_append_weapon_choice"] = dict(weapon_choice)
        distance_m = _haversine_distance_m(
            _normalize_coordinate(candidate.get("coordinate")),
            target_coord,
        )
        if not isinstance(distance_m, (int, float)):
            distance_m = 1.0e12
        # Prefer the nearer future attacker, then the candidate order already
        # established by operational/fuel selection, then a stable aircraft ID.
        score = (float(distance_m), int(order_index), int(aircraft_id))
        scored.append((score, candidate, dict(committed_row)))

    if not scored:
        return None, None, "append_no_candidate_with_unreserved_ammunition"
    scored.sort(key=lambda item: item[0])
    _score, selected, committed_row = scored[0]
    return selected, committed_row, "ok"


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
    seen_target_ids: Set[int] = set()
    seen_target_keys: Set[str] = set()
    seen_key_without_id: Set[str] = set()
    for item in out:
        target_id = _to_int(item.get("target_id") or item.get("targetID"))
        target_key = str(item.get("key") or item.get("targetKey") or "").strip()
        if target_id is not None and int(target_id) in seen_target_ids:
            continue
        if target_key:
            if target_id is None and target_key in seen_target_keys:
                continue
            if target_id is not None and target_key in seen_key_without_id:
                continue
        deduped.append(item)
        if target_id is not None:
            seen_target_ids.add(int(target_id))
        elif target_key:
            seen_key_without_id.add(target_key)
        if target_key:
            seen_target_keys.add(target_key)
    return deduped


def _resolve_attack_source_plan_id(
    ctx: Dict[str, Any],
    detail: Dict[str, Any] | None = None,
) -> Optional[int]:
    detail = detail if isinstance(detail, dict) else {}
    # A detection request can wait in the replanning queue while another
    # attack plan is selected and starts executing.  In that case the IDs
    # captured in the request/context are stale.  mission_progress is the
    # applied-plan view, so prefer it when its MissionPlan artifact is already
    # present.  If progress is missing or briefly points at an unavailable
    # artifact, retain the established request/context fallback order.
    applied_plan_id = _to_int(_load_latest_mission_progress_plan_id())
    if applied_plan_id is not None and applied_plan_id > 0:
        try:
            applied_plan_path = db_paths.get_db_subpath(
                "MissionPlan", f"{int(applied_plan_id)}.json"
            )
            if Path(applied_plan_path).is_file():
                return int(applied_plan_id)
        except Exception:
            pass
    # ``sourceMissionPlanID`` can intentionally retain the previous attack
    # plan as lineage metadata during a follow-up attack.  The executable
    # mission must always branch from the currently applied plan, otherwise
    # completed LINE progress from that older plan can be planned again.
    for value in (
        detail.get("currentMissionPlanID"),
        ctx.get("currentMissionPlanID"),
        detail.get("sourceMissionPlanID"),
        ctx.get("sourceMissionPlanID"),
        ctx.get("source_plan_id"),
        ctx.get("missionPlanID"),
        _scan_latest_source_plan_id(),
    ):
        plan_id = _to_int(value)
        if plan_id is not None and plan_id > 0:
            return int(plan_id)
    return None


def _load_committed_lah_attack_rows(
    source_plan_id: Optional[int],
) -> Tuple[List[Dict[str, int]], List[str]]:
    plan_id = _to_int(source_plan_id)
    if plan_id is None or plan_id <= 0:
        return [], []
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(plan_id)}.json")
        plan_data = read_json_cached(
            plan_path,
            copy_result=False,
            kind="MissionPlan",
        )
    except Exception as exc:
        return [], [f"MissionPlan {int(plan_id)} load failed: {exc}"]
    return collect_lah_attack_rows(plan_data)


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


def _active_tracking_unavailable_by_input_for_plan(
    *,
    source_plan_id: int,
    plan_data: Dict[str, Any],
) -> Tuple[Dict[int, set[int]], set[int]]:
    """Return active tracking UAVs attached to the executable plan lineage.

    A follow-up attack request does not have to repeat every target that is
    already being tracked.  Those omitted trackers must still stay out of the
    collaborative-search availability pool.
    """

    plan_uav_ids = {
        int(aircraft_id)
        for entry in (plan_data.get("aircraftList") or [])
        if isinstance(entry, dict)
        and (aircraft_id := _to_int(entry.get("aircraftID"))) is not None
        and int(aircraft_id) > 3
    }
    if not plan_uav_ids:
        return {}, set()

    try:
        plan_lineage = resolve_plan_lineage_ids(int(source_plan_id))
    except Exception:
        plan_lineage = set()
    if not plan_lineage:
        plan_lineage = {int(source_plan_id)}

    unavailable_by_input: Dict[int, set[int]] = {}
    unavailable_ids: set[int] = set()
    for assignment in list_active_tracking_assignments():
        if not isinstance(assignment, dict):
            continue
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        attack_plan_id = _to_int(assignment.get("attack_plan_id"))
        input_mission_id = _to_int(assignment.get("current_input_mission_id"))
        if (
            aircraft_id is None
            or int(aircraft_id) not in plan_uav_ids
            or attack_plan_id is None
            or int(attack_plan_id) not in plan_lineage
        ):
            continue
        unavailable_ids.add(int(aircraft_id))
        if input_mission_id is not None and int(input_mission_id) > 0:
            unavailable_by_input.setdefault(int(input_mission_id), set()).add(
                int(aircraft_id)
            )
    return unavailable_by_input, unavailable_ids


def _remaining_plan_uav_ids(
    plan_data: Dict[str, Any],
    unavailable_aircraft_ids: set[int],
) -> List[int]:
    unavailable_ids = {int(aid) for aid in unavailable_aircraft_ids}
    remaining: List[int] = []
    for entry in plan_data.get("aircraftList", []):
        if not isinstance(entry, dict):
            continue
        aircraft_id = _to_int(entry.get("aircraftID"))
        if (
            aircraft_id is None
            or int(aircraft_id) <= 3
            or int(aircraft_id) in unavailable_ids
            or int(aircraft_id) in remaining
        ):
            continue
        remaining.append(int(aircraft_id))
    return remaining


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
        scored_candidates: List[Tuple[Tuple[int, float, int], int, Dict[str, Any]]] = []
        for order_index, aircraft_id in enumerate(ordered_ids):
            inventory = inventories.get(int(aircraft_id))
            choice = _resolve_attack_weapon_choice_for_inventory(target, inventory)
            if not bool(choice.get("ammoAvailable")):
                continue

            load_count = int(assignment_counts.get(int(aircraft_id), 0))
            origin_coord = _normalize_coordinate(last_attack_coord_by_id.get(int(aircraft_id)))
            if origin_coord is None:
                origin_coord = _normalize_coordinate((candidate_by_id.get(int(aircraft_id)) or {}).get("coordinate"))
            distance_m = _haversine_distance_m(origin_coord, target_coord) if origin_coord and target_coord else None
            if not isinstance(distance_m, (int, float)):
                distance_m = 1.0e12

            # Keep the first two contacts distributed across both attackers,
            # then give a third contact to the better-positioned aircraft as a
            # second sequential strike.  LAH1 remains the command/relay ship;
            # tracking three contacts must not force the third one into a later
            # replan merely because only LAH2/LAH3 are firing aircraft.
            score = (int(load_count), float(distance_m), int(order_index))
            scored_candidates.append((score, int(aircraft_id), dict(choice)))

        if not scored_candidates:
            if any(sequences.values()):
                # Keep the attacks that are already valid.  This contact stays
                # active/tracked and can be assigned after the next kill or an
                # inventory/state update.
                continue
            target_id = _to_int(target.get("target_id"))
            return {}, f"no_weapon_available_for_target_{target_id or idx + 1}"

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
    assigned_sequences = {
        int(aircraft_id): list(sequence)
        for aircraft_id, sequence in sequences.items()
        if sequence
    }
    return assigned_sequences, None


def _expected_attack_pairs_from_manned_sequences(
    manned_sequences: Dict[int, List[Dict[str, Any]]],
) -> set[Tuple[int, int]]:
    """Return every LAH/target pair requested by the final assignment."""

    return {
        (int(aircraft_id), int(target_id))
        for aircraft_id, sequence in manned_sequences.items()
        for target in sequence
        if isinstance(target, dict)
        for target_id in [
            _to_int(target.get("target_id") or target.get("targetID"))
        ]
        if target_id is not None and int(target_id) > 0
    }


_DEFAULT_SPEED_MPS = 40.0
# Fallback cover hold when the strike geometry cannot be measured. The old
# five minutes left the manned aircraft parked long after the strike, so this
# is only a floor to sit on until the follow-up replan moves them.
_LAH_COVER_HOLD_DEFAULT_SECONDS = 60
# Smallest pop-up that still counts as leaving cover. A firing point level
# with the hide point produces a zero-length leg the aircraft never flies.
_LAH_MIN_POPUP_CLIMB_M = 5.0
# Slowest speed a manned waypoint may be emitted with.  Zero is not a valid
# transit command - the aircraft never leaves the point - and stops are carried
# by ``hovering``/``loiter`` instead.
_LAH_MIN_TRANSIT_SPEED_MPS = 24.0
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
        if str(waypoint.get("coverageAcquisitionID") or "").startswith(
            "areaMission:"
        ):
            emit(
                "[ATTACK][COLLAB] AREA first sweep searchSpeed kept "
                f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
                f"waypointID={_to_int(waypoint.get('waypointID'))}, "
                f"speed={float(search_speed):.2f})."
            )
            return payload
        reference_speed, reference_distance_m = _estimate_attack_collab_first_sweep_search_speed_from_reference(
            waypoint,
            reference_coord,
        )
        base_speed = float(search_speed)
        used_reference_base = False
        if reference_speed is not None and float(reference_speed) > 0.0:
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
                multiplier_cap_enabled=False,
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
            annotate_eta_flight_plan(
                payload,
                default_speed_mps=_DEFAULT_SPEED_MPS,
                waypoint_list_keys=("waypointList",),
                line_search_timing="incoming",
            )
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
        annotate_eta_flight_plan(
            payload,
            default_speed_mps=_DEFAULT_SPEED_MPS,
            waypoint_list_keys=("waypointList",),
            line_search_timing="incoming",
        )
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
    return (
        clamp_line_search_speed_mps(
            estimated_speed,
            cruise_speed_mps=float(transit_speed_mps),
            speed_scale=float(search_speed_weight),
            multiplier_cap_enabled=False,
        ),
        float(transit_distance_m),
    )


def _compute_attack_point(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    friendly_heading_deg: Optional[float] = None,
    friendly_speed_mps: Optional[float] = None,
    cache_stats: Optional[Dict[str, Any]] = None,
    line_coverage_corridors: Optional[List[Dict[str, Any]]] = None,
    line_coverage_metadata: Optional[Dict[str, Any]] = None,
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
    los_clearance_m = _attack_los_clearance_m()
    los_target_height_m = _attack_los_target_height_m()
    los_profile_step_m = _attack_los_profile_step_m()
    constrained_corridors = list(line_coverage_corridors or [])
    coverage_meta = dict(line_coverage_metadata or {})
    require_inside_mission_zone = bool(
        constrained_corridors
        and coverage_meta.get("requireInsideDiscoveredMissionZone", True)
    )
    current_target_distance_m = _haversine_distance_m(friendly_norm, enemy_norm)
    if (
        current_target_distance_m is not None
        and float(current_target_distance_m) < float(min_standoff_m)
    ):
        # Never send an LAH that is already close to the target backwards just
        # to recover the nominal standoff.  The current position becomes the
        # attack point.  Skip the radial point search, but still run the cheap
        # one-dimensional DEM profile so its altitude has verified target LOS.
        result = dict(friendly_norm)
        direct_los_error: Optional[str] = None
        if los_enabled:
            direct_los, direct_los_error = _compute_direct_attack_los_altitude_inprocess(
                friendly_norm,
                enemy_norm,
                lah_floor_coord=friendly_norm,
            )
            if direct_los is not None:
                result.update(direct_los)
            else:
                result["los_verified"] = False
                result["los_profile_error"] = str(direct_los_error or "profile_unavailable")
        result.update(
            {
                "friendly_distance_m": 0.0,
                "enemy_distance_m": float(current_target_distance_m),
                "current_distance_m": float(current_target_distance_m),
                "candidate_distance_m": float(current_target_distance_m),
                "min_standoff_m": float(min_standoff_m),
                "preferred_standoff_m": float(preferred_standoff_m),
                "raster_sources": [],
                "selection_mode": "current_position_inside_min_standoff_no_retreat",
                "minimum_standoff_bypassed": True,
                "no_retreat": True,
                "friendly_target_direction_constrained": True,
                "mission_zone_constrained": bool(constrained_corridors),
                "mission_zone_requirement": (
                    "inside"
                    if constrained_corridors and require_inside_mission_zone
                    else "before_or_inside"
                ),
                "mission_zone_count": len(constrained_corridors),
            }
        )
        if cache_stats is not None:
            cache_stats.update(
                {
                    "hit": False,
                    "elapsedMs": _elapsed_ms_detail(started_at),
                    "method": "current_position_no_retreat",
                    "losSkipped": not bool(los_enabled),
                    "losVerified": bool(result.get("los_verified")),
                    "losError": direct_los_error,
                }
            )
        return result, None
    cache_key = _build_attack_point_cache_key(
        friendly_norm,
        enemy_norm,
        min_standoff_m=float(min_standoff_m),
        preferred_standoff_m=float(preferred_standoff_m),
        altitude_offset_m=float(altitude_offset_m),
        los_enabled=bool(los_enabled),
        los_num_rays=int(los_num_rays),
        los_analysis_radius_m=float(los_analysis_radius_m),
        los_clearance_m=float(los_clearance_m),
        los_target_height_m=float(los_target_height_m),
        los_profile_step_m=float(los_profile_step_m),
        require_inside_mission_zone=bool(require_inside_mission_zone),
        line_coverage_signature=_attack_line_coverage_signature(constrained_corridors),
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
        los_error: Optional[str] = None
        if los_enabled:
            los_result, los_error = _compute_attack_point_inprocess(
                friendly_norm,
                enemy_norm,
                min_standoff_m=float(min_standoff_m),
                preferred_standoff_m=float(preferred_standoff_m),
                line_coverage_corridors=constrained_corridors,
                require_inside_mission_zone=bool(require_inside_mission_zone),
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
                        line_coverage_corridors=constrained_corridors,
                        require_inside_mission_zone=bool(require_inside_mission_zone),
                    )
                if fallback_result is not None:
                    los_result = fallback_result
                    los_method = "los_area_subprocess"
                else:
                    los_error = f"{los_error}; fallback={fallback_error}"
            if los_result is not None:
                if constrained_corridors:
                    los_result["mission_zone_source_plan_id"] = _to_int(
                        coverage_meta.get("sourcePlanID")
                    )
                    los_result["mission_zone_watcher_id"] = _to_int(
                        coverage_meta.get("watcherID")
                    )
                    input_ids = [
                        int(value)
                        for value in (coverage_meta.get("inputMissionIDs") or [])
                        if _to_int(value) is not None
                    ]
                    if len(input_ids) == 1:
                        los_result["mission_zone_input_mission_id"] = int(input_ids[0])
                    los_result["mission_zone_requirement"] = (
                        "inside" if require_inside_mission_zone else "before_or_inside"
                    )
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
        constrained_selection = _select_attack_standoff_inside_constraints(
            friendly_norm,
            enemy_norm,
            min_standoff_m=float(min_standoff_m),
            preferred_standoff_m=float(preferred_standoff_m),
            line_coverage_corridors=constrained_corridors,
            require_inside_mission_zone=bool(require_inside_mission_zone),
        )
        coverage_relaxed = False
        if constrained_selection is None and require_inside_mission_zone:
            # The first discovered corridor is short and the manned aircraft is
            # still approaching from unphotographed ground, so the segment
            # towards it leaves the corridor well inside the minimum standoff.
            # Requiring both then has no solution at all.  Prefer a point
            # before the corridor over cancelling the attack outright.
            constrained_selection = _select_attack_standoff_inside_constraints(
                friendly_norm,
                enemy_norm,
                min_standoff_m=float(min_standoff_m),
                preferred_standoff_m=float(preferred_standoff_m),
                line_coverage_corridors=constrained_corridors,
                require_inside_mission_zone=False,
            )
            coverage_relaxed = constrained_selection is not None
        if constrained_selection is None:
            # Mission-zone metadata is advisory for an attack option.  It must
            # never erase the strike when the target and aircraft coordinates
            # are otherwise valid.  Retry on the direct friendly-target segment
            # before falling back to a deterministic point below.
            constrained_selection = _select_attack_standoff_inside_constraints(
                friendly_norm,
                enemy_norm,
                min_standoff_m=float(min_standoff_m),
                preferred_standoff_m=float(preferred_standoff_m),
                line_coverage_corridors=[],
                require_inside_mission_zone=False,
            )
            coverage_relaxed = constrained_selection is not None
        if constrained_selection is None:
            total_distance_m = _haversine_distance_m(friendly_norm, enemy_norm) or 0.0
            ratio = (
                max(
                    0.0,
                    min(
                        1.0,
                        (float(total_distance_m) - float(preferred_standoff_m))
                        / float(total_distance_m),
                    ),
                )
                if float(total_distance_m) > 1.0
                else 0.0
            )
            emergency_coord = _interpolate_coordinate(
                friendly_norm,
                enemy_norm,
                ratio,
            )
            enemy_altitude_m = _normalize_altitude_value(enemy_norm.get("altitude")) or 0
            friendly_altitude_m = _normalize_altitude_value(friendly_norm.get("altitude")) or 0
            emergency_coord["altitude"] = max(
                int(friendly_altitude_m),
                int(enemy_altitude_m + altitude_offset_m),
            )
            emergency_result = {
                **emergency_coord,
                "friendly_distance_m": _haversine_distance_m(friendly_norm, emergency_coord),
                "enemy_distance_m": _haversine_distance_m(enemy_norm, emergency_coord),
                "current_distance_m": float(total_distance_m),
                "candidate_distance_m": _haversine_distance_m(enemy_norm, emergency_coord),
                "min_standoff_m": float(min_standoff_m),
                "preferred_standoff_m": float(preferred_standoff_m),
                "raster_sources": [],
                "selection_mode": "best_effort_direct_segment",
                "los_verified": False,
                "los_profile_error": str(los_error or "terrain_profile_unavailable"),
                "best_effort_attack_point": True,
                "mission_zone_constrained": bool(constrained_corridors),
                "mission_zone_requirement_relaxed": True,
                "mission_zone_count": len(constrained_corridors),
            }
            if cache_stats is not None:
                cache_stats.update(
                    {
                        "elapsedMs": _elapsed_ms_detail(started_at),
                        "method": "best_effort_direct_segment",
                        "missionZoneRequirementRelaxed": True,
                        "losError": str(los_error or "terrain_profile_unavailable"),
                    }
                )
            return emergency_result, None
        if coverage_relaxed:
            constrained_selection = dict(constrained_selection)
            constrained_selection["mission_zone_requirement_relaxed"] = True
            constrained_selection["mission_zone_requirement_relaxed_from"] = "inside"
            if cache_stats is not None:
                cache_stats["missionZoneRequirementRelaxed"] = True
        base_altitude = _to_float(enemy_norm.get("altitude"))
        altitude_int = _normalize_altitude_value((base_altitude or 0.0) + altitude_offset_m)
        standoff_coord = dict(constrained_selection.get("coordinate") or {})
        direct_los: Optional[Dict[str, Any]] = None
        direct_los_error: Optional[str] = None
        if los_enabled:
            direct_los, direct_los_error = _compute_direct_attack_los_altitude_inprocess(
                standoff_coord,
                enemy_norm,
                lah_floor_coord=friendly_norm,
            )
        direct_los_verified = bool(direct_los and direct_los.get("los_verified"))
        result = {
            "latitude": float(standoff_coord["latitude"]),
            "longitude": float(standoff_coord["longitude"]),
            "altitude": (
                _normalize_altitude_value((direct_los or {}).get("altitude"))
                if direct_los is not None
                else altitude_int
            ),
            "friendly_distance_m": _haversine_distance_m(friendly_norm, standoff_coord),
            "enemy_distance_m": _haversine_distance_m(enemy_norm, standoff_coord),
            "current_distance_m": constrained_selection.get("current_distance_m"),
            "candidate_distance_m": constrained_selection.get("enemy_distance_m"),
            "min_standoff_m": float(min_standoff_m),
            "preferred_standoff_m": float(preferred_standoff_m),
            "raster_sources": list((direct_los or {}).get("raster_sources") or []),
            "selection_mode": (
                "los_area_direct_profile"
                if direct_los_verified
                else constrained_selection.get("selection_mode") or "friendly_target_segment"
            ),
            "los_area": bool(direct_los_verified),
            "friendly_target_direction_constrained": True,
            "mission_zone_constrained": bool(constrained_corridors),
            "mission_zone_requirement": (
                "inside"
                if constrained_corridors and require_inside_mission_zone
                else "before_or_inside"
            ),
            "mission_zone_count": len(constrained_corridors),
        }
        if direct_los is not None:
            for key in _ATTACK_POINT_META_KEYS:
                if key in direct_los:
                    result[key] = deepcopy(direct_los.get(key))
        elif los_enabled:
            result["los_verified"] = False
            result["los_profile_error"] = str(direct_los_error or "profile_unavailable")
        if constrained_corridors:
            result["mission_zone_source_plan_id"] = _to_int(coverage_meta.get("sourcePlanID"))
            result["mission_zone_watcher_id"] = _to_int(coverage_meta.get("watcherID"))
            input_ids = [
                int(value)
                for value in (coverage_meta.get("inputMissionIDs") or [])
                if _to_int(value) is not None
            ]
            if len(input_ids) == 1:
                result["mission_zone_input_mission_id"] = int(input_ids[0])
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
        # Valid aircraft/target coordinates must still produce an option when
        # DEM coverage or an optional geometry dependency is unavailable.
        # Reuse the current aircraft position and climb conservatively; the
        # emitted metadata makes the degraded LOS status explicit.
        enemy_altitude_m = _normalize_altitude_value(enemy_norm.get("altitude")) or 0
        friendly_altitude_m = _normalize_altitude_value(friendly_norm.get("altitude")) or 0
        fallback = dict(friendly_norm)
        fallback["altitude"] = max(
            int(friendly_altitude_m),
            int(enemy_altitude_m + altitude_offset_m),
        )
        fallback.update(
            {
                "friendly_distance_m": 0.0,
                "enemy_distance_m": _haversine_distance_m(enemy_norm, fallback),
                "min_standoff_m": float(min_standoff_m),
                "preferred_standoff_m": float(preferred_standoff_m),
                "raster_sources": [],
                "selection_mode": "best_effort_current_position",
                "los_verified": False,
                "los_profile_error": f"Attack point computation error: {exc}",
                "best_effort_attack_point": True,
                "mission_zone_constrained": bool(constrained_corridors),
                "mission_zone_requirement_relaxed": True,
                "mission_zone_count": len(constrained_corridors),
            }
        )
        return fallback, None


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


def _attack_local_xy_m(
    coord: Dict[str, Any],
    *,
    origin: Dict[str, Any],
) -> Optional[Tuple[float, float]]:
    point = _normalize_coordinate(coord)
    ref = _normalize_coordinate(origin)
    if point is None or ref is None:
        return None
    mean_lat_rad = math.radians((float(point["latitude"]) + float(ref["latitude"])) * 0.5)
    return (
        (float(point["longitude"]) - float(ref["longitude"]))
        * 111_320.0
        * math.cos(mean_lat_rad),
        (float(point["latitude"]) - float(ref["latitude"])) * 111_320.0,
    )


def _attack_point_between_friendly_and_enemy(
    candidate_coord: Dict[str, Any],
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    *,
    projection_tolerance: float = 0.01,
    lateral_tolerance_m: float = 50.0,
) -> bool:
    """Return true only on the finite, narrow friendly-to-enemy corridor."""
    friendly_xy = _attack_local_xy_m(friendly_coord, origin=enemy_coord)
    candidate_xy = _attack_local_xy_m(candidate_coord, origin=enemy_coord)
    if friendly_xy is None or candidate_xy is None:
        return False
    fx, fy = friendly_xy
    cx, cy = candidate_xy
    length_sq = (float(fx) * float(fx)) + (float(fy) * float(fy))
    if length_sq <= 1.0:
        return False
    projection = ((float(cx) * float(fx)) + (float(cy) * float(fy))) / float(length_sq)
    tolerance = max(0.0, float(projection_tolerance))
    if not ((-tolerance) <= float(projection) <= (1.0 + tolerance)):
        return False
    lateral_distance_m = abs((float(cx) * float(fy)) - (float(cy) * float(fx))) / math.sqrt(
        float(length_sq)
    )
    return float(lateral_distance_m) <= max(0.0, float(lateral_tolerance_m))


def _attack_point_to_segment_distance_m(
    point_coord: Dict[str, Any],
    left_coord: Dict[str, Any],
    right_coord: Dict[str, Any],
) -> Optional[float]:
    point_xy = _attack_local_xy_m(point_coord, origin=left_coord)
    right_xy = _attack_local_xy_m(right_coord, origin=left_coord)
    if point_xy is None or right_xy is None:
        return None
    px, py = point_xy
    rx, ry = right_xy
    length_sq = (float(rx) * float(rx)) + (float(ry) * float(ry))
    if length_sq <= 1e-6:
        return math.hypot(float(px), float(py))
    ratio = ((float(px) * float(rx)) + (float(py) * float(ry))) / float(length_sq)
    ratio = max(0.0, min(1.0, float(ratio)))
    nearest_x = float(rx) * float(ratio)
    nearest_y = float(ry) * float(ratio)
    return math.hypot(float(px) - nearest_x, float(py) - nearest_y)


def _attack_point_inside_line_coverage(
    candidate_coord: Dict[str, Any],
    line_coverage_corridors: List[Dict[str, Any]],
    *,
    tolerance_m: float = 50.0,
) -> bool:
    if not line_coverage_corridors:
        return True
    for corridor in line_coverage_corridors:
        zone_type = str(corridor.get("zoneType") or "line").strip().lower()
        coords = _normalize_line_coord_list(
            corridor.get("coordinateList"),
            min_len=3 if zone_type == "area" else 2,
        )
        if zone_type == "area" and len(coords) >= 3:
            point = _normalize_coordinate(candidate_coord)
            if point is None:
                continue
            point_x = float(point["longitude"])
            point_y = float(point["latitude"])
            inside = False
            previous = coords[-1]
            for current in coords:
                x0, y0 = float(previous["longitude"]), float(previous["latitude"])
                x1, y1 = float(current["longitude"]), float(current["latitude"])
                crosses = (y0 > point_y) != (y1 > point_y)
                if crosses:
                    intersect_x = ((x1 - x0) * (point_y - y0) / (y1 - y0)) + x0
                    if point_x < intersect_x:
                        inside = not inside
                previous = current
            if inside:
                return True
            closed_coords = [*coords, coords[0]]
            if any(
                (distance_m := _attack_point_to_segment_distance_m(
                    candidate_coord,
                    closed_coords[idx],
                    closed_coords[idx + 1],
                )) is not None
                and float(distance_m) <= max(0.0, float(tolerance_m))
                for idx in range(len(closed_coords) - 1)
            ):
                return True
            continue
        width_m = _to_float(corridor.get("widthM") or corridor.get("width"))
        if len(coords) < 2 or width_m is None or float(width_m) <= 0.0:
            continue
        allowed_distance_m = (float(width_m) * 0.5) + max(0.0, float(tolerance_m))
        for idx in range(len(coords) - 1):
            distance_m = _attack_point_to_segment_distance_m(
                candidate_coord,
                coords[idx],
                coords[idx + 1],
            )
            if distance_m is not None and float(distance_m) <= float(allowed_distance_m):
                return True
    return False


def _attack_line_coverage_signature(
    line_coverage_corridors: List[Dict[str, Any]],
) -> Tuple[float, ...]:
    signature: List[float] = [float(len(line_coverage_corridors))]
    for corridor in line_coverage_corridors:
        coords = _normalize_line_coord_list(corridor.get("coordinateList"), min_len=2)
        if len(coords) < 2:
            continue
        signature.extend(
            [
                2.0 if str(corridor.get("zoneType") or "line").lower() == "area" else 1.0,
                round(float(coords[0]["latitude"]), 7),
                round(float(coords[0]["longitude"]), 7),
                round(float(coords[-1]["latitude"]), 7),
                round(float(coords[-1]["longitude"]), 7),
                round(float(corridor.get("widthM") or corridor.get("width") or 0.0), 1),
            ]
        )
    return tuple(signature)


def _load_attack_line_coverage_corridors(
    *,
    source_plan_id: Optional[int],
    watcher_id: Optional[int],
    target_coord: Optional[Dict[str, Any]] = None,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    source_plan = _to_int(source_plan_id)
    watcher = _to_int(watcher_id)
    summary: Dict[str, Any] = {
        "sourcePlanID": source_plan,
        "watcherID": watcher,
        "corridorCount": 0,
    }
    if source_plan is None:
        summary["reason"] = "source_plan_missing"
        return [], summary

    matching_entries: List[Dict[str, Any]] = []
    for progress_entry in load_sweep_progress().values():
        if not isinstance(progress_entry, dict):
            continue
        line_entry = progress_entry.get("line_scan")
        if not isinstance(line_entry, dict):
            line_entry = progress_entry
        entry_plan = _to_int(line_entry.get("missionPlanID") or line_entry.get("mission_plan_id"))
        if entry_plan != int(source_plan):
            continue
        if not bool(line_entry.get("enabled", True)):
            continue
        matching_entries.append(line_entry)

    watcher_input_ids = {
        int(input_id)
        for entry in matching_entries
        if _to_int(entry.get("aircraftID") or entry.get("aircraft_id")) == int(watcher)
        and (input_id := _to_int(entry.get("inputMissionID") or entry.get("input_mission_id"))) is not None
    }

    input_plan = _load_input_plan_for_source_plan(int(source_plan))
    if not isinstance(input_plan, dict):
        summary["reason"] = "source_input_mission_plan_missing"
        return [], summary

    zones_by_input_id: Dict[int, List[Dict[str, Any]]] = {}
    mission_index_by_input_id: Dict[int, int] = {}
    target_input_ids: List[int] = []
    input_missions = [
        mission
        for mission in (input_plan.get("inputMissionList") or [])
        if isinstance(mission, dict)
    ]
    for mission_index, mission in enumerate(input_missions):
        if not isinstance(mission, dict):
            continue
        input_id = _to_int(mission.get("inputMissionID"))
        if input_id is None:
            continue
        mission_index_by_input_id[int(input_id)] = int(mission_index)
        detail = mission.get("missionDetail")
        if not isinstance(detail, dict):
            continue
        mission_rows: List[Dict[str, Any]] = []
        for row in detail.get("lineList") or []:
            if not isinstance(row, dict):
                continue
            coords = _normalize_line_coord_list(row.get("coordinateList"), min_len=2)
            width_m = _to_float(row.get("width") or detail.get("sourceLineWidthM"))
            if len(coords) < 2 or width_m is None or float(width_m) <= 0.0:
                continue
            mission_rows.append(
                {
                    "zoneType": "line",
                    "coordinateList": coords,
                    "widthM": float(width_m),
                    "inputMissionID": int(input_id),
                }
            )
        for row in detail.get("areaList") or []:
            if not isinstance(row, dict):
                continue
            coords = _normalize_line_coord_list(row.get("coordinateList"), min_len=3)
            if len(coords) < 3:
                continue
            mission_rows.append(
                {
                    "zoneType": "area",
                    "coordinateList": coords,
                    "inputMissionID": int(input_id),
                }
            )
        if mission_rows:
            zones_by_input_id[int(input_id)] = mission_rows
            if isinstance(target_coord, dict) and _attack_point_inside_line_coverage(
                target_coord,
                mission_rows,
                tolerance_m=100.0,
            ):
                target_input_ids.append(int(input_id))

    if not target_input_ids:
        summary["inputMissionIDs"] = []
        summary["currentWatcherInputMissionIDs"] = sorted(watcher_input_ids)
        summary["rawCorridorCount"] = 0
        summary["reason"] = "discovered_target_mission_zone_missing"
        return [], summary

    # A target can lie on a shared boundary.  Prefer the earliest operational
    # mission in that case; importantly, never union every completed/future
    # mission zone because that admits unrelated far-away attack points.
    target_input_id = min(
        set(target_input_ids),
        key=lambda value: int(mission_index_by_input_id.get(int(value), 1_000_000)),
    )
    target_mission_index = int(mission_index_by_input_id.get(int(target_input_id), 0))
    mission_zones = list(zones_by_input_id.get(int(target_input_id), []))
    first_mission = target_mission_index == 0

    summary["inputMissionIDs"] = [int(target_input_id)]
    summary["targetInputMissionID"] = int(target_input_id)
    summary["targetMissionIndex"] = int(target_mission_index)
    summary["firstMission"] = bool(first_mission)
    summary["requireInsideDiscoveredMissionZone"] = bool(first_mission)
    summary["allowBeforeDiscoveredMissionZone"] = not bool(first_mission)
    summary["currentWatcherInputMissionIDs"] = sorted(watcher_input_ids)
    summary["rawCorridorCount"] = len(mission_zones)
    summary["corridorCount"] = len(mission_zones)
    summary["reason"] = (
        "first_discovered_mission_zone_inside"
        if first_mission
        else "discovered_mission_zone_before_or_inside"
    )
    return mission_zones, summary


def _load_attack_operation_zones(source_plan_id: Optional[int]) -> List[Dict[str, Any]]:
    """Load every LINE corridor and AREA polygon in operational input order."""

    source_plan = _to_int(source_plan_id)
    if source_plan is None:
        return []
    input_plan = _load_input_plan_for_source_plan(int(source_plan))
    if not isinstance(input_plan, dict):
        return []

    zones: List[Dict[str, Any]] = []
    for mission_index, mission in enumerate(input_plan.get("inputMissionList") or []):
        if not isinstance(mission, dict):
            continue
        input_mission_id = _to_int(mission.get("inputMissionID"))
        detail = mission.get("missionDetail")
        if not isinstance(detail, dict):
            continue
        for row in detail.get("lineList") or []:
            if not isinstance(row, dict):
                continue
            coords = _normalize_line_coord_list(row.get("coordinateList"), min_len=2)
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
            coords = _normalize_line_coord_list(row.get("coordinateList"), min_len=3)
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


def _select_attack_standoff_inside_constraints(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    *,
    min_standoff_m: float,
    preferred_standoff_m: float,
    line_coverage_corridors: Optional[List[Dict[str, Any]]] = None,
    require_inside_mission_zone: bool = True,
) -> Optional[Dict[str, Any]]:
    friendly = _normalize_coordinate(friendly_coord)
    enemy = _normalize_coordinate(enemy_coord)
    if friendly is None or enemy is None:
        return None
    current_distance_m = _haversine_distance_m(friendly, enemy)
    if current_distance_m is None:
        return None
    minimum_m = max(0.0, float(min_standoff_m))
    if float(current_distance_m) < minimum_m:
        return {
            "coordinate": dict(friendly),
            "current_distance_m": float(current_distance_m),
            "enemy_distance_m": float(current_distance_m),
            "friendly_distance_m": 0.0,
            "selection_mode": "current_position_inside_min_standoff_no_retreat",
            "friendly_target_direction_constrained": True,
            "mission_zone_constrained": bool(line_coverage_corridors),
            "mission_zone_requirement": "no_retreat_override",
            "mission_zone_count": len(line_coverage_corridors or []),
            "minimum_standoff_bypassed": True,
            "no_retreat": True,
        }
    maximum_between_m = max(0.0, float(current_distance_m) - 1.0)
    if maximum_between_m + 1e-6 < minimum_m:
        return None
    preferred_m = max(minimum_m, float(preferred_standoff_m))
    if preferred_m >= maximum_between_m:
        # When the preferred standoff is farther than the LAH itself, do not
        # collapse the attack waypoint onto the LAH endpoint.  Keep a visible,
        # useful point between the LAH and target.
        desired_m = (float(minimum_m) + float(maximum_between_m)) * 0.5
    else:
        desired_m = float(preferred_m)
    bearing_deg = _bearing_between(
        float(enemy["latitude"]),
        float(enemy["longitude"]),
        float(friendly["latitude"]),
        float(friendly["longitude"]),
    )
    distances_m: List[float] = [float(minimum_m), float(maximum_between_m), float(desired_m)]
    cursor_m = float(minimum_m)
    while cursor_m < float(maximum_between_m) - 1e-6:
        distances_m.append(float(cursor_m))
        cursor_m += 25.0
    distances_m = sorted(
        set(round(float(value), 6) for value in distances_m),
        key=lambda value: abs(float(value) - float(desired_m)),
    )
    corridors = list(line_coverage_corridors or [])
    for distance_m in distances_m:
        candidate = _project_coordinate(enemy, float(bearing_deg), float(distance_m))
        candidate = _normalize_coordinate(candidate)
        if candidate is None:
            continue
        if not _attack_point_between_friendly_and_enemy(candidate, friendly, enemy):
            continue
        if (
            corridors
            and require_inside_mission_zone
            and not _attack_point_inside_line_coverage(candidate, corridors, tolerance_m=0.0)
        ):
            continue
        return {
            "coordinate": candidate,
            "current_distance_m": float(current_distance_m),
            "enemy_distance_m": _haversine_distance_m(enemy, candidate),
            "friendly_distance_m": _haversine_distance_m(friendly, candidate),
            "selection_mode": (
                "friendly_target_segment_mission_inside"
                if corridors and require_inside_mission_zone
                else (
                    "friendly_target_segment_mission_before_or_inside"
                    if corridors
                    else "friendly_target_segment"
                )
            ),
            "friendly_target_direction_constrained": True,
            "mission_zone_constrained": bool(corridors),
            "mission_zone_requirement": (
                "inside" if corridors and require_inside_mission_zone else "before_or_inside"
            ),
            "mission_zone_count": len(corridors),
        }
    return None


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
        data = read_json_cached(path, copy_result=False, kind="FlightPath")
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

    current_eta = None
    final_eta = 0.0
    has_missing_eta = False
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        eta_raw = wp.get("eta")
        if eta_raw is None:
            has_missing_eta = True
            break
        try:
            eta_val = float(eta_raw)
        except Exception:
            has_missing_eta = True
            break
        wp_id = _to_int(wp.get("waypointID"))
        if wp_id == current_wp:
            current_eta = eta_val
        if eta_val > final_eta:
            final_eta = eta_val
    if not has_missing_eta and current_eta is not None:
        return max(0.0, float(final_eta) - float(current_eta))

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


def _attack_exclusion_requested_target_ids(ctx: Dict[str, Any]) -> set[int]:
    """Targets this exclusion request is actually about.

    An empty set means the operator asked to drop every attack, so nothing is
    carved out of the sweep.
    """

    detail = _normalize_replan_detail((ctx or {}).get("replan_detail"))
    out: set[int] = set()
    if not isinstance(detail, dict):
        return out
    for key in ("targetID", "targetId", "target_id"):
        value = _to_int(detail.get(key))
        if value is not None and value > 0:
            out.add(int(value))
    for list_key in ("targetIDList", "targetList", "targets"):
        rows = detail.get(list_key)
        if not isinstance(rows, list):
            continue
        for row in rows:
            value = (
                _to_int(row)
                if not isinstance(row, dict)
                else _to_int(
                    row.get("targetID") or row.get("targetId") or row.get("target_id")
                )
            )
            if value is not None and value > 0:
                out.add(int(value))
    return out


def _actively_tracked_target_ids() -> set[int]:
    """Targets a UAV is still watching, i.e. engagements not yet finished."""

    out: set[int] = set()
    try:
        entries = list_active_tracking_assignments()
    except Exception:
        return out
    for entry in entries or []:
        if not isinstance(entry, dict) or not bool(entry.get("active")):
            continue
        target_id = _to_int(entry.get("target_id") or entry.get("targetID"))
        if target_id is not None and target_id > 0:
            out.add(int(target_id))
    return out


def _resolve_global_attack_exclusion_lah_context(
    *,
    source_plan_id: int,
    aircraft_id: int,
) -> Optional[Dict[str, Any]]:
    """Return all target-bound LAH attack/support missions on the source plan."""

    context = _load_attack_exclusion_plan_context(int(source_plan_id), int(aircraft_id))
    if not isinstance(context, dict):
        return None
    target_ids: List[int] = []
    current_input_id: Optional[int] = None
    for mission in context.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if bool(mission.get("postAttackResume")):
            continue
        info = mission.get("individualMissionInfo")
        if not isinstance(info, dict):
            continue
        mission_type = _to_int(info.get("individualMissionType"))
        target_id = _to_int(info.get("targetID") or info.get("targetId"))
        if mission_type not in {2, 9} or target_id is None or target_id <= 0:
            continue
        if int(target_id) not in target_ids:
            target_ids.append(int(target_id))
        if current_input_id is None:
            related = mission.get("relatedMission")
            if isinstance(related, dict):
                current_input_id = _to_int(
                    related.get("inputMissionID") or related.get("inputMissionId")
                )
    if not target_ids or current_input_id is None or current_input_id <= 0:
        return None
    return {
        "current_input_id": int(current_input_id),
        "target_ids": list(target_ids),
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


_ATTACK_EXCLUSION_WAYPOINT_LIST_KEYS = (
    "waypointList",
    "uavWaypointList",
    "lahWaypointList",
)


def _freshen_attack_exclusion_artifact_ids(
    plan_data: Dict[str, Any],
    *,
    now_ms: int,
    emit: LogCallback,
    id_reservation: Optional[ReplanIdReservation] = None,
) -> Dict[str, Any]:
    """Clone an exclusion candidate into a completely fresh artifact graph.

    Attack-exclusion used to mix freshly generated resume artifacts with
    unchanged packages and follow-up paths referenced by their old IDs.  Some
    consumers cache artifacts by ID, so a new MissionPlan that retains any of
    those references can be interpreted as an unchanged command.  This pass is
    deliberately content-neutral: it only updates IDs and artifact timestamps.
    """

    aircraft_entries = [
        entry
        for entry in (plan_data.get("aircraftList") or [])
        if isinstance(entry, dict)
    ]
    package_rows: List[Dict[str, Any]] = []
    path_count_by_aircraft: Dict[int, int] = {}
    total_missions = 0
    total_waypoints = 0

    # Load and validate the complete source graph before reserving or writing
    # anything.  This keeps a missing package/path from producing a half-cloned
    # exclusion option.
    for entry in aircraft_entries:
        aircraft_id = _to_int(entry.get("aircraftID"))
        source_imp_id = _to_int(entry.get("individualMissionPackageID"))
        if aircraft_id is None or aircraft_id <= 0:
            raise RuntimeError(f"invalid aircraftID in exclusion plan: {entry!r}")
        if source_imp_id is None or source_imp_id <= 0:
            raise RuntimeError(
                f"aircraft {aircraft_id} has no valid individualMissionPackageID"
            )
        imp_path = db_paths.get_db_subpath(
            "IndividualMissionPlan", f"{int(source_imp_id)}.json"
        )
        imp_data = deepcopy(read_json_cached(imp_path, kind="IndividualMissionPlan"))
        missions = imp_data.get("individualMissionList")
        if not isinstance(missions, list):
            raise RuntimeError(
                f"IndividualMissionPlan {source_imp_id} individualMissionList is invalid"
            )

        mission_rows: List[Dict[str, Any]] = []
        for mission in missions:
            if not isinstance(mission, dict):
                raise RuntimeError(
                    f"IndividualMissionPlan {source_imp_id} contains a non-object mission"
                )
            source_mission_id = _to_int(mission.get("individualMissionID"))
            source_path_id = _to_int(mission.get("pathID"))
            if source_mission_id is None or source_mission_id <= 0:
                raise RuntimeError(
                    f"IndividualMissionPlan {source_imp_id} contains an invalid mission ID"
                )
            if source_path_id is None or source_path_id <= 0:
                raise RuntimeError(
                    f"mission {source_mission_id} contains an invalid pathID"
                )
            path_src = db_paths.get_db_subpath("FlightPath", f"{int(source_path_id)}.json")
            path_data = deepcopy(read_json_cached(path_src, kind="FlightPath"))
            if not isinstance(path_data, dict):
                raise RuntimeError(f"FlightPath {source_path_id} payload is invalid")
            waypoint_count = sum(
                sum(1 for waypoint in (path_data.get(key) or []) if isinstance(waypoint, dict))
                for key in _ATTACK_EXCLUSION_WAYPOINT_LIST_KEYS
                if isinstance(path_data.get(key), list)
            )
            total_waypoints += int(waypoint_count)
            mission_rows.append(
                {
                    "mission": mission,
                    "sourceMissionID": int(source_mission_id),
                    "path": path_data,
                    "sourcePathID": int(source_path_id),
                }
            )

        total_missions += len(mission_rows)
        path_count_by_aircraft[int(aircraft_id)] = (
            path_count_by_aircraft.get(int(aircraft_id), 0) + len(mission_rows)
        )
        package_rows.append(
            {
                "entry": entry,
                "aircraftID": int(aircraft_id),
                "sourcePackageID": int(source_imp_id),
                "package": imp_data,
                "missions": mission_rows,
            }
        )

    reservation = id_reservation or ReplanIdReservation.reserve(
        imp_count=len(package_rows),
        individual_count=int(total_missions),
        path_count_by_aircraft=path_count_by_aircraft,
        waypoint_count=int(total_waypoints),
    )
    imp_payloads: List[Dict[str, Any]] = []
    path_entries: List[Tuple[Path, Dict[str, Any]]] = []
    write_entries: List[Tuple[Path, Dict[str, Any]]] = []
    artifact_rows: List[Dict[str, Any]] = []

    for package_row in package_rows:
        aircraft_id = int(package_row["aircraftID"])
        source_imp_id = int(package_row["sourcePackageID"])
        new_imp_id = int(reservation.next_imp())
        imp_copy = deepcopy(package_row["package"])
        imp_copy["individualMissionPackageID"] = int(new_imp_id)
        imp_copy["aircraftID"] = int(aircraft_id)
        imp_copy["timestamp"] = int(now_ms)
        cloned_missions: List[Dict[str, Any]] = []
        mission_id_rows: List[Dict[str, int]] = []

        for mission_row in package_row["missions"]:
            new_mission_id = int(reservation.next_individual())
            new_path_id = int(reservation.next_path(int(aircraft_id)))
            mission_copy = deepcopy(mission_row["mission"])
            mission_copy["individualMissionID"] = int(new_mission_id)
            mission_copy["pathID"] = int(new_path_id)
            cloned_missions.append(mission_copy)

            path_copy = deepcopy(mission_row["path"])
            path_copy["pathID"] = int(new_path_id)
            path_copy["aircraftID"] = int(aircraft_id)
            path_copy["individualMissionID"] = int(new_mission_id)
            path_copy["timestamp"] = int(now_ms)
            for key in _ATTACK_EXCLUSION_WAYPOINT_LIST_KEYS:
                waypoints = path_copy.get(key)
                if not isinstance(waypoints, list) or not waypoints:
                    continue
                reassign_unique_waypoint_ids_inplace(
                    waypoints,
                    waypoint_id_provider=reservation.next_waypoint,
                )
            path_dest = db_paths.get_db_subpath("FlightPath", f"{int(new_path_id)}.json")
            path_entries.append((path_dest, path_copy))
            mission_id_rows.append(
                {
                    "sourceIndividualMissionID": int(mission_row["sourceMissionID"]),
                    "individualMissionID": int(new_mission_id),
                    "sourcePathID": int(mission_row["sourcePathID"]),
                    "pathID": int(new_path_id),
                }
            )

        imp_copy["individualMissionList"] = cloned_missions
        imp_dest = db_paths.get_db_subpath(
            "IndividualMissionPlan", f"{int(new_imp_id)}.json"
        )
        imp_payloads.append(imp_copy)
        write_entries.append((imp_dest, imp_copy))
        package_row["entry"]["individualMissionPackageID"] = int(new_imp_id)
        artifact_rows.append(
            {
                "aircraftID": int(aircraft_id),
                "sourceIndividualMissionPackageID": int(source_imp_id),
                "individualMissionPackageID": int(new_imp_id),
                "missionCount": len(cloned_missions),
                "missionIDs": mission_id_rows,
            }
        )

    write_entries.extend(path_entries)
    if write_entries:
        _validate_generated_artifact_write_entries(
            scope=f"attackExclusionFreshIds:{_to_int(plan_data.get('missionPlanID')) or 0}",
            individual_mission_plans=imp_payloads,
            entries=path_entries,
            log=emit,
        )
        _write_json_files_batch(write_entries)

    return {
        "policy": "all_artifact_ids_fresh",
        "aircraftArtifacts": artifact_rows,
        "individualMissionCount": int(total_missions),
        "pathCount": int(total_missions),
        "waypointCount": int(total_waypoints),
        "reservedIds": reservation.summary(),
    }


_TRACKING_OPERATION_MODE = 3


def _strip_tracking_from_exclusion_plan(
    plan_data: Dict[str, Any],
    *,
    emit: LogCallback,
) -> List[Dict[str, Any]]:
    """Remove any surviving target-tracking mission from an exclusion plan.

    ``_resolve_attack_tracking_recovery`` only detaches tracking whose
    assignment names the plan being excluded from; tracking carried over from an
    earlier replan slips through and lands in a plan whose entire purpose is to
    have none.  This is the backstop: it reads the finished plan and drops every
    individual mission whose waypoints film in tracking mode.  IDs and ordering
    of the surviving missions are untouched.
    """

    removed: List[Dict[str, Any]] = []
    for entry in plan_data.get("aircraftList") or []:
        if not isinstance(entry, dict):
            continue
        aircraft_id = _to_int(entry.get("aircraftID"))
        imp_id = _to_int(entry.get("individualMissionPackageID"))
        if aircraft_id is None or imp_id is None:
            continue
        try:
            imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(imp_id)}.json")
            imp_data = read_json_cached(imp_path, kind="IndividualMissionPlan")
        except Exception:
            continue
        missions = imp_data.get("individualMissionList")
        if not isinstance(missions, list) or not missions:
            continue
        kept: List[Dict[str, Any]] = []
        dropped_here: List[Dict[str, Any]] = []
        for mission in missions:
            if not isinstance(mission, dict) or not _mission_films_in_tracking_mode(mission):
                kept.append(mission)
                continue
            dropped_here.append(
                {
                    "aircraftID": int(aircraft_id),
                    "individualMissionID": _to_int(mission.get("individualMissionID")),
                    "pathID": _to_int(mission.get("pathID")),
                }
            )
        if not dropped_here:
            continue
        if not kept:
            # Never hand back an empty package; a tracking-only aircraft keeps
            # what it has rather than losing its mission list entirely.
            emit(
                "[ATTACK-EXCLUDE][WARN] aircraft "
                f"{aircraft_id} has only tracking missions; leaving them in place."
            )
            continue
        imp_data["individualMissionList"] = kept
        try:
            _write_json_file(imp_path, imp_data)
        except Exception as exc:
            emit(f"[ATTACK-EXCLUDE][WARN] could not rewrite IMP {imp_id}: {exc}")
            continue
        removed.extend(dropped_here)
        emit(
            "[ATTACK-EXCLUDE] removed leftover tracking mission(s) "
            f"(aircraft={aircraft_id}, missions={[row['individualMissionID'] for row in dropped_here]})."
        )
    return removed


def _mission_films_in_tracking_mode(mission: Dict[str, Any]) -> bool:
    """Whether any waypoint of this mission films a tracked target."""

    path_id = _to_int(mission.get("pathID"))
    if path_id is None or path_id <= 0:
        return False
    try:
        path_data = read_json_cached(
            db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json"),
            kind="FlightPath",
        )
    except Exception:
        return False
    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        rows = path_data.get(key)
        if not isinstance(rows, list):
            continue
        for row in rows:
            if not isinstance(row, dict):
                continue
            filming = row.get("filmingProperty") or row.get("filming")
            if not isinstance(filming, dict):
                continue
            if _to_int(filming.get("operationMode")) == _TRACKING_OPERATION_MODE:
                return True
    return False


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


def _build_uav_attack_completion_hold_waypoint(
    fp_data: Dict[str, Any],
    *,
    waypoint_id: int,
    fallback_coordinate: Any = None,
    hold_seconds: int = _UAV_ATTACK_COMPLETION_HOLD_SECONDS,
) -> Optional[Dict[str, Any]]:
    """Build a terminal loiter when the current collaborative sweep is complete.

    An attack replan can arrive after the last LINE/AREA capture point has already
    been consumed.  In that case the split operation has no resume geometry.  The
    current collaborative mission has not been handed off yet, so exposing the
    next input mission would let the simulator advance prematurely.  Keep one
    executable hold at the final capture point instead.
    """

    waypoints = (
        fp_data.get("waypointList")
        if isinstance(fp_data, dict) and isinstance(fp_data.get("waypointList"), list)
        else []
    )
    template_wp = next(
        (deepcopy(item) for item in reversed(waypoints) if isinstance(item, dict)),
        {},
    )
    final_coord = (
        _normalize_coordinate(_extract_final_uav_coordinate(fp_data))
        or _normalize_coordinate(template_wp.get("coordinate"))
        or _normalize_coordinate(fallback_coordinate)
    )
    if final_coord is None:
        return None

    hold_seconds = max(1, int(hold_seconds or _UAV_ATTACK_COMPLETION_HOLD_SECONDS))
    marker_wp = template_wp
    marker_wp["waypointID"] = int(waypoint_id)
    marker_wp["coordinate"] = deepcopy(final_coord)
    marker_wp["speed"] = float(_UAV_ATTACK_COMPLETION_HOLD_SPEED_MPS)
    marker_wp["eta"] = int(hold_seconds)
    marker_wp["ecf"] = float(_to_float(marker_wp.get("ecf")) or 0.0)
    marker_wp["nextWaypointID"] = 0
    marker_wp["waypointPassType"] = 2
    marker_wp["isDone"] = False
    marker_wp["loiterProperty"] = {
        "radius": int(_UAV_ATTACK_COMPLETION_HOLD_RADIUS_M),
        "direction": 1,
        "time": int(hold_seconds),
        "speed": int(round(_UAV_ATTACK_COMPLETION_HOLD_SPEED_MPS)),
    }

    filming = marker_wp.get("filmingProperty")
    filming = deepcopy(filming) if isinstance(filming, dict) else {}
    filming["operationMode"] = 1
    filming["sensorType"] = _to_int(filming.get("sensorType")) or 1
    filming["fieldOfView"] = float(
        _to_float(filming.get("fieldOfView"))
        or get_runtime_effective_fov_deg("global_manual_fov_deg", 5.0)
    )
    filming.pop("lineSearch", None)
    filming.pop("areaSearch", None)
    filming.pop("autoTracking", None)
    filming["coordinateOrientation"] = {"coordinate": deepcopy(final_coord)}
    marker_wp["filmingProperty"] = filming
    marker_wp["attackCompletionBoundaryHold"] = True
    return marker_wp


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
        fp_data = read_json_cached(fp_path, copy_result=False, kind="FlightPath")
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
        waypoint_index_by_id: Dict[int, int] = {}
        for idx, waypoint_id in enumerate(waypoints):
            waypoint_index_by_id.setdefault(int(waypoint_id), int(idx))
        current_index = waypoint_index_by_id.get(int(current_waypoint_id))
        if current_index is not None:
            idx = int(current_index)
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


def _resolve_source_aircraft_input_mission_id(
    *,
    source_plan_id: int,
    aircraft_id: int,
    state: Optional[Dict[str, Any]],
    cache: Dict[str, Any],
    emit: Callable[[str], None],
) -> Optional[int]:
    """Resolve one aircraft's currently executing related input mission."""

    current_waypoint_id = _to_int((state or {}).get("current_waypoint_id"))
    artifacts = _resolve_plan_artifacts_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        current_waypoint_id=current_waypoint_id,
        cache=cache,
        emit=emit,
        allow_first_mission_fallback=False,
    )
    if artifacts is None:
        return None
    imp_data = _load_attack_cached_imp_data(
        cache,
        int(artifacts.individual_mission_package_id),
        emit=emit,
    )
    if not isinstance(imp_data, dict):
        return None
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _to_int(mission.get("individualMissionID")) != int(
            artifacts.individual_mission_id
        ):
            continue
        input_mission_id = _extract_related_input_mission_id(mission)
        return int(input_mission_id) if input_mission_id is not None else None
    return None


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


def _drop_stale_lah_tactical_follow_ups(
    missions: List[Dict[str, Any]],
    *,
    current_input_id: Optional[int],
    emit: Optional[Callable[[str], None]] = None,
) -> List[Dict[str, Any]]:
    """Remove attack/target-hold branches superseded by this tactical replan.

    The current input's ordinary resume is rebuilt separately.  Reattaching an
    older type-2 attack or target-bound type-9 hold after the new branch makes a
    stale target reappear and is the source of several apparent path U-turns.
    Future input missions are intentionally untouched.
    """

    current_input = _to_int(current_input_id)
    kept: List[Dict[str, Any]] = []
    dropped_ids: List[int] = []
    for mission in missions or []:
        if not isinstance(mission, dict):
            continue
        info = mission.get("individualMissionInfo")
        mission_type = _to_int(
            info.get("individualMissionType") if isinstance(info, dict) else None
        )
        target_id = _to_int(info.get("targetID") if isinstance(info, dict) else None)
        stale = bool(
            current_input is not None
            and _extract_related_input_mission_id(mission) == int(current_input)
            and (
                bool(mission.get("postAttackResume"))
                # The run-to-cover leg belongs to the engagement it was built
                # for; leaving it behind would strand a movement path pointing
                # at a hide point the next replan has already superseded.
                or bool(mission.get("lahCoverIngress"))
                or (
                    mission_type in {2, 9}
                    and target_id is not None
                    and int(target_id) > 0
                )
            )
        )
        if stale:
            mission_id = _to_int(mission.get("individualMissionID"))
            if mission_id is not None:
                dropped_ids.append(int(mission_id))
            continue
        kept.append(mission)
    if dropped_ids and emit is not None:
        emit(
            "[ATTACK][LAH] Dropped superseded target-bound follow-up mission(s) "
            f"for inputMissionID={current_input}: {dropped_ids}."
        )
    return kept


def _attack_follow_up_requires_clone(
    mission: Dict[str, Any],
    *,
    current_input_id: Optional[int],
) -> bool:
    current_input = _to_int(current_input_id)
    if current_input is None or int(current_input) <= 0:
        return True
    return _extract_related_input_mission_id(mission) == int(current_input)


def _attack_follow_up_preserve_cache_key(
    path: Path,
    *,
    aircraft_id: int,
    mission_id: int,
) -> Tuple[str, int, int, int, int] | None:
    try:
        path_obj = Path(path)
        stat = path_obj.stat()
    except Exception:
        return None
    return (
        _attack_fast_path_key(path_obj),
        int(stat.st_mtime_ns),
        int(stat.st_size),
        int(aircraft_id),
        int(mission_id),
    )


def _attack_follow_up_preserve_cache_get(
    key: Tuple[str, int, int, int, int] | None,
) -> bool | None:
    if key is None:
        return None
    with _ATTACK_FOLLOW_UP_PRESERVE_CACHE_LOCK:
        cached = _ATTACK_FOLLOW_UP_PRESERVE_CACHE.get(key)
        if cached is None:
            return None
        _ATTACK_FOLLOW_UP_PRESERVE_CACHE.move_to_end(key)
        return bool(cached)


def _attack_follow_up_preserve_cache_store_true(
    key: Tuple[str, int, int, int, int] | None,
) -> None:
    if key is None:
        return
    with _ATTACK_FOLLOW_UP_PRESERVE_CACHE_LOCK:
        _ATTACK_FOLLOW_UP_PRESERVE_CACHE[key] = True
        _ATTACK_FOLLOW_UP_PRESERVE_CACHE.move_to_end(key)
        while len(_ATTACK_FOLLOW_UP_PRESERVE_CACHE) > _ATTACK_FOLLOW_UP_PRESERVE_CACHE_MAX:
            _ATTACK_FOLLOW_UP_PRESERVE_CACHE.popitem(last=False)


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
    src = db_paths.get_db_subpath("FlightPath", f"{int(source_path_id)}.json")
    cache_key = _attack_follow_up_preserve_cache_key(
        src,
        aircraft_id=int(aircraft_id),
        mission_id=int(mission_id),
    )
    cached = _attack_follow_up_preserve_cache_get(cache_key)
    if cached is True:
        return True
    try:
        fp_data = read_json_cached(src, copy_result=False, kind="FlightPath")
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
    _attack_follow_up_preserve_cache_store_true(cache_key)
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


def _copy_preserved_attack_follow_up_mission(mission: Dict[str, Any]) -> Dict[str, Any]:
    preserved = dict(mission or {})
    related = mission.get("relatedMission") if isinstance(mission, dict) else None
    if isinstance(related, dict):
        preserved["relatedMission"] = dict(related)
    info = mission.get("individualMissionInfo") if isinstance(mission, dict) else None
    if isinstance(info, dict):
        preserved["individualMissionInfo"] = dict(info)
    preserved["isDone"] = False
    return preserved


def _mark_attack_followups_execution_blocked(
    missions: List[Dict[str, Any]],
    *,
    current_input_id: int,
) -> int:
    """Retain future artifacts at a collaboration boundary without executing them."""

    blocked = 0
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        input_id = _extract_related_input_mission_id(mission)
        if input_id is not None and int(input_id) == int(current_input_id):
            mission.pop("executionBlockedUntilNextCollab", None)
            continue
        mission["executionBlockedUntilNextCollab"] = True
        blocked += 1
    return int(blocked)


def _attack_completion_boundary_follow_up_sources(
    missions: List[Dict[str, Any]],
    *,
    current_input_id: int,
) -> List[Dict[str, Any]]:
    """Drop only stale current-input artifacts; never delete future inputs."""

    return [
        mission
        for mission in missions
        if isinstance(mission, dict)
        and _extract_related_input_mission_id(mission) != int(current_input_id)
    ]


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
        assembly.append(("preserve", _copy_preserved_attack_follow_up_mission(mission)))
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

    @staticmethod
    def counts_for_descriptor(
        *,
        descriptor: Dict[str, Any],
        target_index: Optional[int],
        source_mission_count: int,
        source_waypoint_count: int = 0,
        attack_target_count: int = 0,
        follow_up_clone_count: Optional[int] = None,
    ) -> Dict[str, Any]:
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

        if mode in {"LAH_ATTACK", "LAH_ATTACK_APPEND"}:
            attack_count = max(1, int(attack_target_count or 1))
            if mode == "LAH_ATTACK_APPEND":
                # Append-only preserves every source mission/path and adds one
                # attack mission/path.  There is no rebuilt resume/follow-up.
                follow_up_count = 0
                path_count = attack_count
                individual_count = attack_count
            else:
                # attack(s) + resume + the run-to-cover leg, which is emitted as
                # its own individual mission so the movement and the engagement
                # at cover stay separately addressable.
                path_count += attack_count + 2
                individual_count += attack_count + 2
            # Each attack path now contains a dense-sampled/simplified low-level
            # approach plus the vertical high attack point.  Reserve enough IDs
            # without falling back to the global allocator during a transaction.
            # In addition to the established low-level approach/resume paths,
            # the first attack may carry a native DEM-certified hide prelude.
            waypoint_count += attack_count * 128 + 128 + 256
        elif mode == "LAH_RELAY":
            path_count += 3
            individual_count += 3
            # Native route planner is bounded at 256, while the normal result
            # is typically under ten points.  Keep the reservation local.
            waypoint_count += 256
        elif mode == "LAH_HOLD_RESUME":
            # ingress + hold + resume.
            path_count += 3
            individual_count += 3
            waypoint_count += 4
        elif mode == "UAV_TRACK":
            path_count += 3
            individual_count += 2
            waypoint_count += 4
        else:
            path_count += 2
            individual_count += 1
            waypoint_count += 3

        return {
            "imp_count": int(imp_count),
            "individual_count": int(individual_count),
            "path_count_by_aircraft": {int(aircraft_id): int(path_count)} if aircraft_id > 0 else {},
            "waypoint_count": int(waypoint_count),
        }

    @classmethod
    def from_reserved_slice(
        cls,
        parent: ReplanIdReservation,
        counts: Dict[str, Any],
    ) -> "AttackIdReservation":
        imp_count = max(0, int(counts.get("imp_count") or 0))
        individual_count = max(0, int(counts.get("individual_count") or 0))
        waypoint_count = max(0, int(counts.get("waypoint_count") or 0))
        path_counts = {
            int(aircraft_id): max(0, int(count or 0))
            for aircraft_id, count in dict(counts.get("path_count_by_aircraft") or {}).items()
            if int(count or 0) > 0
        }
        # These per-descriptor slices are carved out of the parent reservation
        # from counts estimated before the builders run.  When a builder needs
        # one more than estimated, draw it from the parent rather than failing
        # the whole attack replan; the parent extends from the global allocator
        # in turn, so every ID stays unique.
        reservation = ReplanIdReservation(
            imp_ids=ReservedIdBlock(
                "individualMissionPackage",
                [parent.next_imp() for _ in range(imp_count)],
                refill=lambda n: [parent.next_imp() for _ in range(int(n))],
            ),
            individual_ids=ReservedIdBlock(
                "individualMission",
                [parent.next_individual() for _ in range(individual_count)],
                refill=lambda n: [parent.next_individual() for _ in range(int(n))],
            ),
            waypoint_ids=ReservedIdBlock(
                "waypoint",
                [parent.next_waypoint() for _ in range(waypoint_count)],
                refill=lambda n: [parent.next_waypoint() for _ in range(int(n))],
            ),
            path_ids_by_aircraft={
                int(aircraft_id): ReservedIdBlock(
                    f"pathID[{int(aircraft_id)}]",
                    [parent.next_path(int(aircraft_id)) for _ in range(count)],
                    refill=(
                        lambda n, _aid=int(aircraft_id): [
                            parent.next_path(_aid) for _ in range(int(n))
                        ]
                    ),
                )
                for aircraft_id, count in sorted(path_counts.items())
            },
        )
        return cls(reservation)

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
        counts = cls.counts_for_descriptor(
            descriptor=descriptor,
            target_index=target_index,
            source_mission_count=source_mission_count,
            source_waypoint_count=source_waypoint_count,
            attack_target_count=attack_target_count,
            follow_up_clone_count=follow_up_clone_count,
        )
        reservation = ReplanIdReservation.reserve(
            imp_count=int(counts.get("imp_count") or 0),
            individual_count=int(counts.get("individual_count") or 0),
            path_count_by_aircraft=dict(counts.get("path_count_by_aircraft") or {}),
            waypoint_count=int(counts.get("waypoint_count") or 0),
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

    preload_started = time.perf_counter()
    preload_counts: Dict[str, Any] = {}
    active_preload_cache = get_active_source_artifact_cache()
    preload_enabled = (
        str(os.environ.get("REPLAN_ATTACK_SOURCE_TREE_PRELOAD", "0") or "").strip().lower()
        in {"1", "true", "yes", "on"}
    )
    if preload_enabled and active_preload_cache is not None:
        try:
            preload_counts = active_preload_cache.preload_mission_plan_tree(int(source_plan_id))
        except Exception as exc:
            preload_counts = {"error": str(exc)}
            emit(f"[ATTACK][CACHE][WARN] source tree preload failed: {exc}")
    _record_phase(
        "source_tree_preload",
        preload_started,
        enabled=bool(preload_enabled),
        activeCache=bool(active_preload_cache is not None),
        **({"counts": preload_counts} if preload_counts else {}),
    )

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
    ctx.pop("_ground_maneuver_operation", None)
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
            else:
                # Type 2 (지상작전부대 기동여건 보장): 목표지역 attack anchor profile.
                gm_attack_profile = detect_ground_maneuver_attack_profile(input_data)
                if isinstance(gm_attack_profile, dict):
                    ctx["_ground_maneuver_operation"] = deepcopy(gm_attack_profile)
                    emit(
                        "[ATTACK][GM] Type-2 target-region attack profile active "
                        f"(targetHoldInputMissionID={gm_attack_profile.get('targetHoldInputMissionID')})"
                    )
        except Exception as exc:
            emit(f"[ATTACK][LAH] special operation profile unavailable: {exc}")
        _record_phase(
            "lah_special_profile",
            lah_profile_started,
            active=bool(isinstance(lah_special_profile, dict)),
            inputMissionPackageID=int(source_input_pkg_id),
        )

    sweep_progress_lock = threading.Lock()
    sweep_progress_loaded = False
    sweep_progress_cache: Dict[int, Dict[str, Any]] = {}

    def _get_sweep_progress_once() -> Dict[int, Dict[str, Any]]:
        nonlocal sweep_progress_loaded, sweep_progress_cache
        if sweep_progress_loaded:
            return sweep_progress_cache
        with sweep_progress_lock:
            if sweep_progress_loaded:
                return sweep_progress_cache
            sweep_progress_started = time.perf_counter()
            loaded_progress = load_sweep_progress()
            sweep_progress_cache = loaded_progress if isinstance(loaded_progress, dict) else {}
            sweep_progress_loaded = True
            _record_phase(
                "sweep_progress_load",
                sweep_progress_started,
                entries=len(sweep_progress_cache or {}),
                lazy=True,
            )
            return sweep_progress_cache

    sweep_progress = _get_sweep_progress_once

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
        assignedTargetCount=sum(len(sequence or []) for sequence in manned_sequences.values()),
        sequenceLengths={
            str(aircraft_id): len(sequence or [])
            for aircraft_id, sequence in sorted(manned_sequences.items())
        },
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
    deferred_attack_count = max(
        0,
        len(assigned_targets)
        - sum(len(sequence or []) for sequence in manned_sequences.values()),
    )
    if deferred_attack_count > 0:
        emit(
            "[ATTACK][LAH] Deferred additional contact(s) to the next replan "
            f"(deferred={deferred_attack_count}, reason=no-compatible-ammunition)."
        )
        ctx["_deferred_attack_target_count"] = int(deferred_attack_count)

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
            special_lah_attack_coord = special_attack_coordinate(
                ctx.get("_lah_special_operation"),
                target_coord=target_coord,
            )
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
                resolved_attack_source = dict(attack_coord or {})
                resolved_attack_source["selection_mode"] = (
                    _attack_point_selection_mode(attack_coord)
                    or selection.get("mode")
                    or "adaptive_standoff"
                )
                resolved_attack_coord = _attach_attack_point_metadata(resolved_attack_coord, resolved_attack_source)
                emit(
                    "[ATTACK][POINT] Primary target adaptive standoff "
                    f"mode={selection.get('mode')} currentDist={selection.get('current_distance_m')}m "
                    f"candidateDist={selection.get('candidate_distance_m')}m "
                    f"standoff={selection.get('min_standoff_m')}/{selection.get('preferred_standoff_m')}m."
                )

            if resolved_attack_coord is None and aircraft_coord is not None:
                start_coord = previous_attack_coord or aircraft_coord
                sequence_mission_zones, sequence_zone_summary = _load_attack_line_coverage_corridors(
                    source_plan_id=int(source_plan_id),
                    watcher_id=_to_int(sequence_target.get("watcher_id")),
                    target_coord=target_coord,
                )
                computed_attack_coord, _attack_error = _compute_attack_point(
                    start_coord,
                    target_coord,
                    friendly_heading_deg=aircraft_heading if previous_attack_coord is None else None,
                    friendly_speed_mps=current_aircraft_speed,
                    line_coverage_corridors=sequence_mission_zones,
                    line_coverage_metadata=sequence_zone_summary,
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
    contact_uav_states = [
        {
            "aircraft_id": int(contact_aircraft_id),
            "coordinate": dict(contact_state.get("coordinate") or {}),
        }
        for contact_aircraft_id, contact_state in sorted(agent_index.items())
        if int(contact_aircraft_id) > 3
        and isinstance(contact_state, dict)
        and isinstance(contact_state.get("coordinate"), dict)
    ]
    contact_threat_targets = [
        dict(item)
        for item in (
            ctx.get("_enemy_contact_target_list")
            or assigned_targets
        )
        if isinstance(item, dict)
    ]
    contact_enemy_coordinates = [
        dict(contact_target.get("coordinate") or {})
        for contact_target in contact_threat_targets
        if isinstance(contact_target, dict)
        and isinstance(contact_target.get("coordinate"), dict)
    ]
    enemy_contact_context = {
        "uav_states": contact_uav_states,
        "enemy_coordinates": contact_enemy_coordinates,
        # Keep identity/type metadata for the firing-point safety gate.  The
        # concealment planner still receives the compact coordinate list.
        "enemy_targets": deepcopy(contact_threat_targets),
        "enemy_input_count": len(contact_threat_targets),
        "enemy_coordinate_count": len(contact_enemy_coordinates),
    }
    descriptors: List[Dict[str, Any]] = []
    active_manned_ids = set(manned_sequences.keys())
    incremental_append = (
        dict(ctx.get("_incremental_attack_append") or {})
        if isinstance(ctx.get("_incremental_attack_append"), dict)
        else {}
    )
    incremental_append_aircraft_id = _to_int(incremental_append.get("aircraftID"))
    for aircraft in selected_manned_aircraft:
        aircraft_id = _to_int(aircraft.get("aircraft_id"))
        if aircraft_id is None or aircraft_id not in active_manned_ids:
            continue
        append_mode = bool(
            incremental_append_aircraft_id is not None
            and int(aircraft_id) == int(incremental_append_aircraft_id)
        )
        descriptors.append(
            {
                "label": "manned",
                "aircraft_id": int(aircraft_id),
                "state": agent_index.get(int(aircraft_id)),
                "attack_targets": [dict(item) for item in manned_sequences.get(int(aircraft_id), [])],
                "mode": "LAH_ATTACK_APPEND" if append_mode else "LAH_ATTACK",
                "enemy_contact": deepcopy(enemy_contact_context),
                **(
                    {
                        "committed_attack_row": deepcopy(
                            incremental_append.get("committedRow") or {}
                        ),
                        "append_source_plan_id": _to_int(
                            incremental_append.get("sourcePlanID")
                        ),
                    }
                    if append_mode
                    else {}
                ),
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

    preserved_lah_attack_aircraft_ids = {
        int(value)
        for value in (ctx.get("_preserved_lah_attack_aircraft_ids") or [])
        if _to_int(value) is not None
    }
    other_lah_ids: List[int] = []
    for entry in plan_data.get("aircraftList", []):
        aid = _to_int((entry or {}).get("aircraftID"))
        if aid is None or aid > 3 or aid in active_manned_ids:
            continue
        if int(aid) in preserved_lah_attack_aircraft_ids:
            # A committed package may only be reused while its cover point is
            # still masked from the contact set as it stands NOW.  A newly
            # discovered enemy can see straight onto a point that was concealed
            # from the previous ones, so an aircraft whose inherited cover no
            # longer certifies is re-planned instead: it drops into the hold
            # pool below and is given a freshly certified hide point against the
            # current enemies.  Being seen is worse than deferring its attack.
            if _preserved_lah_cover_still_masked(
                aircraft_id=int(aid),
                ctx=ctx,
                enemy_contact=enemy_contact_context,
                emit=emit,
            ):
                emit(
                    "[ATTACK][CONTINUITY] Reusing committed LAH package unchanged "
                    f"(aircraft={int(aid)}); no hold/resume descriptor generated."
                )
                continue
            emit(
                "[ATTACK][CONTINUITY][WARN] Committed LAH cover is no longer "
                f"concealed from the current contacts (aircraft={int(aid)}); "
                "discarding the inherited hide point and re-planning cover. "
                "Its committed attack is deferred this cycle."
            )
            preserved_lah_attack_aircraft_ids.discard(int(aid))
            # The continuity contract requires every preserved row to survive
            # verbatim in the new plan.  This aircraft is deliberately no longer
            # preserved, so withdraw its rows too - leaving them would make the
            # downstream check demand a package we are intentionally replacing.
            ctx["_preserved_lah_attack_aircraft_ids"] = [
                value
                for value in (ctx.get("_preserved_lah_attack_aircraft_ids") or [])
                if _to_int(value) is not None and int(value) != int(aid)
            ]
            ctx["_preserved_source_attack_rows"] = [
                row
                for row in (ctx.get("_preserved_source_attack_rows") or [])
                if not (
                    isinstance(row, dict)
                    and _to_int(row.get("aircraftID")) == int(aid)
                )
            ]
            if isinstance(ctx.get("_incremental_attack_append"), dict) and _to_int(
                ctx["_incremental_attack_append"].get("aircraftID")
            ) == int(aid):
                # An append transaction anchors its firing point on exactly the
                # cover endpoint we just rejected.
                ctx.pop("_incremental_attack_append", None)
        if aid not in other_lah_ids:
            other_lah_ids.append(int(aid))
    support_target_id: Optional[int] = None
    for assigned_target in assigned_targets:
        candidate_target_id = _to_int(
            assigned_target.get("target_id") or assigned_target.get("targetID")
        )
        if candidate_target_id is not None and candidate_target_id > 0:
            support_target_id = int(candidate_target_id)
            break
    for aid in other_lah_ids:
        command_aircraft_id = get_runtime_attack_int("command_aircraft_id", 1)
        is_command_relay = int(aid) == int(command_aircraft_id)
        descriptors.append(
            {
                "label": f"lah_relay_{aid}" if is_command_relay else f"lah_hold_{aid}",
                "aircraft_id": int(aid),
                "state": agent_index.get(int(aid)),
                "target_coord": None,
                # Even non-firing LAHs belong to this attack branch. Preserve
                # the attacked target in their generated hold/resume 0302 data.
                "target_id": support_target_id,
                "mode": "LAH_RELAY" if is_command_relay else "LAH_HOLD_RESUME",
                # Every manned aircraft in this branch is exposed to the same
                # contacts, so a wingman waiting out someone else's attack takes
                # cover too - it used to sit wherever it happened to be.
                "enemy_contact": deepcopy(enemy_contact_context),
            }
        )

    collab_scan_started = time.perf_counter()
    (
        tracking_unavailable_by_input,
        active_tracking_uav_ids,
    ) = _active_tracking_unavailable_by_input_for_plan(
        source_plan_id=int(source_plan_id),
        plan_data=plan_data,
    )
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

    unavailable_tracking_uav_ids = {
        int(aid) for aid in active_tracking_uav_ids
    } | {
        int(aid) for aid in used_tracking_uav_ids
    }

    collab_remaining_mode = _attack_collab_remaining_replan_mode()
    # Shared LINE/AREA work remains centrally managed and is redivided around
    # the temporarily unavailable tracker.  Only the locked Type-2
    # self-reliance suffix keeps immutable per-aircraft ownership.
    auto_collab_input_ids: set[int] = set()
    preserved_area_input_ids: set[int] = set()
    if collab_remaining_mode != "off":
        for input_mission_id in tracking_unavailable_by_input:
            remaining_policy = _attack_tracking_collab_remaining_policy(
                source_plan_id=int(source_plan_id),
                input_mission_id=int(input_mission_id),
            )
            if remaining_policy == "redivide":
                auto_collab_input_ids.add(int(input_mission_id))
            elif remaining_policy == "preserve":
                preserved_area_input_ids.add(int(input_mission_id))
    collab_input_ids = set(auto_collab_input_ids)
    if collab_remaining_mode == "always":
        collab_input_ids.update(
            int(input_mission_id)
            for input_mission_id in tracking_unavailable_by_input
            if int(input_mission_id) not in preserved_area_input_ids
        )
    collab_remaining_replan_enabled = bool(collab_input_ids)
    if preserved_area_input_ids:
        emit(
            "[ATTACK][OWNERSHIP] Type-2 self-reliance branch keeps per-aircraft assignments "
            f"(inputMissionIDs={sorted(preserved_area_input_ids)})."
        )
    _record_phase(
        "collab_tracking_scan",
        collab_scan_started,
        inputCount=len(tracking_unavailable_by_input),
        unavailableUavCount=sum(len(vals) for vals in tracking_unavailable_by_input.values()),
        activeTrackingUavCount=len(active_tracking_uav_ids),
        mode=collab_remaining_mode,
        autoCollabInputCount=len(auto_collab_input_ids),
        preservedAreaInputCount=len(preserved_area_input_ids),
        enabled=bool(collab_remaining_replan_enabled),
    )

    other_uav_ids = _remaining_plan_uav_ids(
        plan_data,
        unavailable_tracking_uav_ids,
    )
    current_input_by_other_uav: Dict[int, Optional[int]] = {}
    type2_branch_line_aircraft_ids: set[int] = set()
    type2_branch_line_input_ids: set[int] = set()
    for aircraft_id in other_uav_ids:
        current_input_id = _resolve_source_aircraft_input_mission_id(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
            state=agent_index.get(int(aircraft_id)),
            cache=source_artifact_cache,
            emit=emit,
        )
        current_input_by_other_uav[int(aircraft_id)] = current_input_id
        phase = _source_type2_self_reliance_phase(
            source_plan_id=int(source_plan_id),
            input_mission_id=current_input_id,
        )
        if phase in {
            TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
            TYPE2_SELF_RELIANCE_RETURN_LINE,
        }:
            type2_branch_line_aircraft_ids.add(int(aircraft_id))
            if current_input_id is not None:
                type2_branch_line_input_ids.add(int(current_input_id))

    configured_reuse_unaffected_uav = _attack_reuse_unaffected_uav_enabled()
    resume_descriptor_uav_ids = _attack_resume_descriptor_uav_ids(
        configured_reuse=bool(configured_reuse_unaffected_uav),
        other_uav_ids=other_uav_ids,
        current_input_by_aircraft=current_input_by_other_uav,
        collaborative_input_ids=collab_input_ids,
        type2_branch_line_aircraft_ids=type2_branch_line_aircraft_ids,
    )
    reused_unchanged_uav_ids = set(other_uav_ids) - set(resume_descriptor_uav_ids)
    if type2_branch_line_aircraft_ids:
        emit(
            "[ATTACK][TYPE2-LINE-SUFFIX] Immutable branch ownership retained; "
            "only UAVs currently on branch LINEs receive trimmed suffixes "
            f"(inputMissionIDs={sorted(type2_branch_line_input_ids)}, "
            f"aircraft={','.join(str(aid) for aid in sorted(type2_branch_line_aircraft_ids))})."
        )
    if collab_remaining_replan_enabled and resume_descriptor_uav_ids:
        emit(
            "[ATTACK][COLLAB] Resume descriptors retained only for affected input members; "
            f"collaborative remaining replan will handle shared input "
            f"(aircraft={','.join(str(aid) for aid in sorted(resume_descriptor_uav_ids))})."
        )
    if reused_unchanged_uav_ids:
        emit(
            "[ATTACK][REUSE] Unrelated UAV plans kept unchanged "
            f"(aircraft={','.join(str(aid) for aid in sorted(reused_unchanged_uav_ids))})."
        )
    for aid in sorted(resume_descriptor_uav_ids):
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
        reusedUnchangedUavCount=len(reused_unchanged_uav_ids),
        otherUavDescriptorCount=len(resume_descriptor_uav_ids),
        type2BranchLineInputCount=len(type2_branch_line_input_ids),
        type2IndividualLineSuffixRefreshCount=len(type2_branch_line_aircraft_ids),
        collabRemainingMode=collab_remaining_mode,
        autoCollabInputCount=len(auto_collab_input_ids),
    )

    descriptor_write_batch_requested = (
        str(os.environ.get("REPLAN_ATTACK_DESCRIPTOR_WRITE_BATCH", "1") or "").strip().lower()
        not in {"0", "false", "no", "off"}
    )
    tactical_transaction_required = any(
        str(descriptor.get("mode") or "") in {"LAH_ATTACK", "LAH_ATTACK_APPEND", "LAH_RELAY"}
        for descriptor in descriptors
        if isinstance(descriptor, dict)
    )
    # Tactical branches may be rejected after parallel route construction.
    # Never allow an environment override to write partial IMP/FP artifacts
    # before every required LAH branch has passed certification.
    descriptor_write_deferred_enabled = bool(
        descriptor_write_batch_requested or tactical_transaction_required
    )
    if tactical_transaction_required and not descriptor_write_batch_requested:
        emit(
            "[ATTACK][TACTICAL] Descriptor writes forced to deferred commit "
            "for fail-closed LAH certification."
        )
    collaborative_resume_by_input: Dict[int, CollaborativeResumeReplanResult] = {}
    collab_deferred_write_entries: List[Tuple[Path, Dict[str, Any]]] = []
    if descriptors and collab_remaining_replan_enabled:
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
                    defer_writes=descriptor_write_deferred_enabled,
                    split_single_aircraft_area_into_two=not bool(collab_input_is_line),
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
                            "timingMs": dict(getattr(collab, "timing_ms", {}) or {}),
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
                        collab_deferred_write_entries.extend(
                            (Path(path), payload)
                            for path, payload in (getattr(collab, "deferred_write_entries", None) or [])
                            if isinstance(payload, dict)
                        )
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
                if int(input_mission_id) in collab_input_ids
            ]
            if collab_items and collab_parallel_enabled:
                parallel_started = time.perf_counter()
                collab_future_executor = concurrent.futures.ThreadPoolExecutor(
                    max_workers=min(2, len(collab_items)),
                    thread_name_prefix="AttackCollab",
                )
                for input_mission_id, unavailable_ids in collab_items:
                    active_source_cache = get_active_source_artifact_cache()
                    if active_source_cache is not None:
                        future = collab_future_executor.submit(
                            call_with_source_artifact_cache,
                            active_source_cache,
                            _build_collab_resume_payload,
                            int(input_mission_id),
                            set(unavailable_ids),
                        )
                    else:
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
                    collab_deferred_write_entries.extend(
                        (Path(path), payload)
                        for path, payload in (getattr(collab, "deferred_write_entries", None) or [])
                        if isinstance(payload, dict)
                    )
                    for aid, imp_id in collab.aircraft_imp_ids.items():
                        if int(aid) in {int(item) for item in (collab.unavailable_aircraft_ids or set())}:
                            emit(
                                "[ATTACK][COLLAB][WARN] skipped applying replacement to unavailable aircraft "
                                f"(aircraft={int(aid)}, inputMissionID={int(input_mission_id)})."
                            )
                            continue
                        _update_plan_aircraft_entry(new_plan_data, int(aid), int(imp_id), emit)
    elif descriptors:
        policy_started = time.perf_counter()
        emit(
            "[ATTACK][COLLAB] Remaining collaborative replan skipped; "
            "unaffected UAVs keep individual resume/reuse paths."
        )
        _record_phase(
            "collab_remaining_replan_policy",
            policy_started,
            enabled=False,
            mode="reuse_individual_resume",
            descriptorCount=len(descriptors),
        )

    aircraft_updates: List[Dict[str, Any]] = []
    pending_tracking_assignments: List[Tuple[int, Dict[str, Any]]] = []
    descriptor_timings: List[Dict[str, Any]] = []
    descriptor_loop_started = time.perf_counter()
    if "collab_future_map" in locals() and collab_future_map:
        independent_modes = {"LAH_ATTACK", "LAH_ATTACK_APPEND", "LAH_RELAY", "LAH_HOLD_RESUME"}
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
    deferred_descriptor_write_entries: List[Tuple[Path, Dict[str, Any]]] = collab_deferred_write_entries

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
                defer_write=descriptor_write_deferred_enabled,
            )
        elif mode == "LAH_ATTACK_APPEND":
            update = _build_lah_incremental_attack_append_package(
                descriptor=descriptor_payload,
                assigned_targets=[dict(item) for item in descriptor_payload.get("attack_targets") or []],
                new_imp_id=new_imp_id_value,
                imp_data=new_imp_data_payload,
                ctx=worker_ctx,
                state=state_payload,
                aircraft_id=aircraft_id_value,
                artifacts=artifacts_payload,
                emit=_thread_emit,
                now_ms=now_ms,
                id_reservation=id_reservation_payload,
                defer_write=descriptor_write_deferred_enabled,
            )
        elif mode in {"LAH_RELAY", "LAH_HOLD_RESUME"}:
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
                defer_write=descriptor_write_deferred_enabled,
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
                defer_write=descriptor_write_deferred_enabled,
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
                defer_write=descriptor_write_deferred_enabled,
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
            active_source_cache = get_active_source_artifact_cache()
            if active_source_cache is not None:
                future = descriptor_future_executor.submit(
                    call_with_source_artifact_cache,
                    active_source_cache,
                    _run_attack_descriptor_builder_job,
                    **job_kwargs,
                )
            else:
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

    pending_descriptor_jobs: List[Dict[str, Any]] = []

    def _flush_pending_descriptor_jobs(*, reason: str) -> None:
        if not pending_descriptor_jobs:
            return
        flush_jobs = list(pending_descriptor_jobs)
        pending_descriptor_jobs.clear()
        reservation_started = time.perf_counter()
        total_imp_count = 0
        total_individual_count = 0
        total_waypoint_count = 0
        total_path_counts: Dict[int, int] = {}
        for job in flush_jobs:
            counts = dict(job.get("reservationCounts") or {})
            total_imp_count += max(0, int(counts.get("imp_count") or 0))
            total_individual_count += max(0, int(counts.get("individual_count") or 0))
            total_waypoint_count += max(0, int(counts.get("waypoint_count") or 0))
            for raw_aircraft_id, raw_count in dict(counts.get("path_count_by_aircraft") or {}).items():
                aircraft_id = int(raw_aircraft_id)
                count = max(0, int(raw_count or 0))
                if count > 0:
                    total_path_counts[aircraft_id] = total_path_counts.get(aircraft_id, 0) + count

        parent_reservation = ReplanIdReservation.reserve(
            imp_count=int(total_imp_count),
            individual_count=int(total_individual_count),
            path_count_by_aircraft=total_path_counts,
            waypoint_count=int(total_waypoint_count),
        )
        for job in flush_jobs:
            counts = dict(job.get("reservationCounts") or {})
            attack_id_reservation = AttackIdReservation.from_reserved_slice(parent_reservation, counts)
            descriptor_detail = dict(job.get("descriptorDetail") or {})
            descriptor_detail["attackIdReservation"] = {
                "elapsedMs": _elapsed_ms_detail(reservation_started),
                "summary": attack_id_reservation.summary(),
                "followUpCloneCount": int(job.get("followUpCloneCount") or 0),
                "bulk": True,
                "bulkReason": str(reason),
            }

            allocate_imp_started = time.perf_counter()
            new_imp_id = attack_id_reservation.next_imp()
            descriptor_detail["allocateImpId"] = {
                "elapsedMs": _elapsed_ms_detail(allocate_imp_started),
                "newImpID": int(new_imp_id),
            }

            plan_update_started = time.perf_counter()
            aircraft_id = int(job.get("aircraftID") or 0)
            if not _mission_plan_has_aircraft_entry(new_plan_data, aircraft_id):
                descriptor_detail["planAircraftEntryCheck"] = {
                    "elapsedMs": _elapsed_ms_detail(plan_update_started),
                    "newImpID": int(new_imp_id),
                }
                descriptor_detail["status"] = "plan_update_failed"
                descriptor_detail["elapsedMs"] = _elapsed_ms_detail(float(job.get("descriptorStartedAt") or time.perf_counter()))
                override_detail_timing.setdefault("descriptorDetails", []).append(dict(descriptor_detail))
                descriptor_timings.append(
                    {
                        "aircraftID": int(aircraft_id),
                        "label": job.get("label"),
                        "mode": job.get("mode"),
                        "status": "plan_update_failed",
                        "elapsedMs": descriptor_detail.get("elapsedMs"),
                    }
                )
                continue
            descriptor_detail["planAircraftEntryDeferred"] = {
                "elapsedMs": _elapsed_ms_detail(plan_update_started),
                "newImpID": int(new_imp_id),
                "mergeMode": "serial_after_descriptor_build",
            }

            _submit_attack_descriptor_builder_job(
                descriptor_index=int(job.get("descriptorIndex") or 0),
                descriptor_payload=dict(job.get("descriptorPayload") or {}),
                descriptor_detail_payload=descriptor_detail,
                descriptor_started_at=float(job.get("descriptorStartedAt") or time.perf_counter()),
                aircraft_id_value=int(aircraft_id),
                state_payload=deepcopy(job.get("statePayload")) if isinstance(job.get("statePayload"), dict) else {},
                new_imp_id_value=int(new_imp_id),
                new_imp_data_payload=job.get("newImpData"),
                fp_data_payload=job.get("fpData"),
                target_mission_payload=job.get("targetMission"),
                target_index_value=job.get("targetIndex"),
                artifacts_payload=job.get("artifacts"),
                id_reservation_payload=attack_id_reservation,
                collaborative_resume_payload=job.get("collaborativeResume"),
            )

        override_detail_timing.setdefault("descriptorBulkReservations", []).append(
            {
                "reason": str(reason),
                "elapsedMs": _elapsed_ms_detail(reservation_started),
                "jobCount": len(flush_jobs),
                "impCount": int(total_imp_count),
                "individualCount": int(total_individual_count),
                "waypointCount": int(total_waypoint_count),
                "pathCountByAircraft": {int(k): int(v) for k, v in sorted(total_path_counts.items())},
                "summary": parent_reservation.summary(),
            }
        )

    for descriptor_index, descriptor in enumerate(descriptors):
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
        reservation_counts = AttackIdReservation.counts_for_descriptor(
            descriptor=descriptor,
            target_index=target_index,
            source_mission_count=len(mission_list) if isinstance(mission_list, list) else 0,
            source_waypoint_count=source_waypoint_count,
            attack_target_count=len(descriptor.get("attack_targets") or []),
            follow_up_clone_count=follow_up_clone_count,
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

        pending_descriptor_jobs.append(
            {
                "descriptorIndex": int(descriptor_index),
                "descriptorPayload": dict(descriptor),
                "descriptorDetail": dict(descriptor_detail),
                "descriptorStartedAt": float(descriptor_started),
                "aircraftID": int(aircraft_id),
                "label": descriptor.get("label"),
                "mode": descriptor.get("mode"),
                "statePayload": deepcopy(state) if isinstance(state, dict) else {},
                "newImpData": new_imp_data,
                "fpData": fp_data,
                "targetMission": target_mission,
                "targetIndex": target_index,
                "artifacts": artifacts,
                "reservationCounts": reservation_counts,
                "followUpCloneCount": int(follow_up_clone_count),
                "collaborativeResume": collaborative_resume,
            }
        )
        continue
    if (
        "collab_future_map" in locals()
        and collab_future_map
        and not collab_futures_consumed
    ):
        # LAH builders do not depend on the collaborative UAV result.  Start
        # them before joining that future so their path construction overlaps
        # the remaining collaborative line/area planning work.
        independent_modes = {"LAH_ATTACK", "LAH_ATTACK_APPEND", "LAH_RELAY", "LAH_HOLD_RESUME"}
        independent_jobs = [
            job for job in pending_descriptor_jobs
            if str(job.get("mode") or "") in independent_modes
        ]
        dependent_jobs = [
            job for job in pending_descriptor_jobs
            if str(job.get("mode") or "") not in independent_modes
        ]
        if independent_jobs:
            pending_descriptor_jobs[:] = independent_jobs
            _flush_pending_descriptor_jobs(reason="before_collab_join_independent")
            pending_descriptor_jobs[:] = dependent_jobs

        # Resolve every descriptor's source artifacts before joining.  The
        # collaborative planner has therefore overlapped all read/clone work,
        # while only UAV-dependent jobs remain to be filtered below.
        _consume_collab_resume_futures(wait_reason="descriptor_prepare_complete")
        filtered_pending_jobs: List[Dict[str, Any]] = []
        for job in pending_descriptor_jobs:
            target_mission = job.get("targetMission")
            current_input_id = _extract_related_input_mission_id(target_mission) or 0
            collaborative_resume = (
                collaborative_resume_by_input.get(int(current_input_id))
                if int(current_input_id) > 0
                else None
            )
            aircraft_id = int(job.get("aircraftID") or 0)
            if (
                str(job.get("mode") or "") == "UAV_RESUME"
                and collaborative_resume is not None
                and aircraft_id in collaborative_resume.replacement_aircraft_ids
            ):
                descriptor_detail = dict(job.get("descriptorDetail") or {})
                descriptor_detail["collabEarlySkip"] = {
                    "currentInputMissionID": int(current_input_id),
                    "replacementAircraftIDs": sorted(
                        int(aid) for aid in collaborative_resume.replacement_aircraft_ids
                    ),
                }
                descriptor_detail["status"] = "handled_by_collab"
                descriptor_detail["elapsedMs"] = _elapsed_ms_detail(
                    float(job.get("descriptorStartedAt") or time.perf_counter())
                )
                override_detail_timing.setdefault("descriptorDetails", []).append(
                    dict(descriptor_detail)
                )
                descriptor_timings.append(
                    {
                        "aircraftID": int(aircraft_id),
                        "label": job.get("label"),
                        "mode": job.get("mode"),
                        "status": "handled_by_collab",
                        "earlySkip": True,
                        "currentInputMissionID": int(current_input_id),
                        "elapsedMs": descriptor_detail["elapsedMs"],
                    }
                )
                emit(
                    f"[ATTACK][COLLAB] Aircraft {aircraft_id} handled by collaborative "
                    f"remaining replan (inputMissionID={current_input_id})."
                )
                continue
            job["collaborativeResume"] = collaborative_resume
            filtered_pending_jobs.append(job)
        pending_descriptor_jobs[:] = filtered_pending_jobs
    _flush_pending_descriptor_jobs(reason="end_descriptor_prepare")
    if (
        "collab_future_map" in locals()
        and collab_future_map
        and not collab_futures_consumed
    ):
        _flush_pending_descriptor_jobs(reason="before_final_collab_join")
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

    mandatory_tactical_failures = _mandatory_tactical_descriptor_failures(
        descriptors,
        descriptor_results,
    )
    if mandatory_tactical_failures:
        for failed in mandatory_tactical_failures:
            row = failed.get("row")
            if not isinstance(row, dict):
                continue
            for message in row.get("messages") or []:
                if message:
                    emit(str(message))
        failure_summary = [
            {
                "aircraftID": int(item["aircraftID"]),
                "mode": str(item["mode"]),
                "status": str(item["status"]),
            }
            for item in mandatory_tactical_failures
        ]
        _set_override_failure(
            "attack_tactical_certification_failed",
            attack_failure_notice("attack_override_failed"),
        )
        emit(
            "[ATTACK][TACTICAL][ERR] Attack option rejected before state/file "
            "commit because a required LAH branch was not certified "
            f"(failures={failure_summary})."
        )
        return None

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
            raw_deferred_entries = update.pop("_deferredWriteEntries", None)
            if isinstance(raw_deferred_entries, list):
                for item in raw_deferred_entries:
                    if not (isinstance(item, tuple) and len(item) >= 2):
                        continue
                    path, payload = item[0], item[1]
                    if isinstance(payload, dict):
                        deferred_descriptor_write_entries.append((Path(path), payload))
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
                pending_tracking_assignments.append(
                    (int(aircraft_id), dict(tracking_assignment))
                )
                descriptor_detail["trackingState"] = {
                    "deferred": True,
                    "mergeMode": "after_validation_before_plan_commit",
                }

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

    continuity_imp_payloads = [
        payload
        for path, payload in deferred_descriptor_write_entries
        if isinstance(payload, dict) and Path(path).parent.name == "IndividualMissionPlan"
    ]
    continuity_fp_payloads = [
        payload
        for path, payload in deferred_descriptor_write_entries
        if isinstance(payload, dict) and Path(path).parent.name == "FlightPath"
    ]
    candidate_attack_rows, candidate_attack_scan_errors = collect_lah_attack_rows(
        new_plan_data,
        individual_mission_plans=continuity_imp_payloads,
        flight_paths=continuity_fp_payloads,
    )
    preserved_source_attack_rows = [
        dict(item)
        for item in (ctx.get("_preserved_source_attack_rows") or [])
        if isinstance(item, dict)
    ]
    incremental_append_meta = (
        dict(ctx.get("_incremental_attack_append") or {})
        if isinstance(ctx.get("_incremental_attack_append"), dict)
        else {}
    )
    append_aircraft_id = _to_int(incremental_append_meta.get("aircraftID"))
    append_candidate_imp_id: Optional[int] = None
    if append_aircraft_id is not None:
        for aircraft_entry in new_plan_data.get("aircraftList") or []:
            if not isinstance(aircraft_entry, dict):
                continue
            if _to_int(aircraft_entry.get("aircraftID")) != int(append_aircraft_id):
                continue
            append_candidate_imp_id = _to_int(
                aircraft_entry.get("individualMissionPackageID")
            )
            break
    # The append transaction is allowed to place the unchanged mission array
    # in a fresh IMP shell.  Normalize only that declared owner's package ID;
    # mission/path/WP/target identity remains exact and all other aircraft keep
    # the ordinary strict continuity check.
    expected_preserved_attack_rows: List[Dict[str, Any]] = []
    for source_row in preserved_source_attack_rows:
        expected_row = dict(source_row)
        if (
            append_aircraft_id is not None
            and append_candidate_imp_id is not None
            and _to_int(expected_row.get("aircraftID")) == int(append_aircraft_id)
        ):
            expected_row["individualMissionPackageID"] = int(append_candidate_imp_id)
        expected_preserved_attack_rows.append(expected_row)
    missing_committed_identities = missing_attack_identities(
        expected_preserved_attack_rows,
        candidate_attack_rows,
    )
    committed_order_mismatches: List[Dict[str, Any]] = []
    for source_row, expected_row in zip(
        preserved_source_attack_rows,
        expected_preserved_attack_rows,
    ):
        stable_identity = (
            _to_int(expected_row.get("aircraftID")),
            _to_int(expected_row.get("individualMissionID")),
            _to_int(expected_row.get("pathID")),
            _to_int(expected_row.get("waypointID")),
            _to_int(expected_row.get("targetID")),
        )
        matches = [
            row
            for row in candidate_attack_rows
            if (
                _to_int(row.get("aircraftID")),
                _to_int(row.get("individualMissionID")),
                _to_int(row.get("pathID")),
                _to_int(row.get("waypointID")),
                _to_int(row.get("targetID")),
            )
            == stable_identity
        ]
        if len(matches) != 1:
            continue
        candidate_row = matches[0]
        if (
            _to_int(candidate_row.get("missionIndex"))
            != _to_int(source_row.get("missionIndex"))
            or _to_int(candidate_row.get("waypointIndex"))
            != _to_int(source_row.get("waypointIndex"))
        ):
            committed_order_mismatches.append(
                {
                    "identity": list(stable_identity),
                    "sourceMissionIndex": _to_int(source_row.get("missionIndex")),
                    "candidateMissionIndex": _to_int(candidate_row.get("missionIndex")),
                    "sourceWaypointIndex": _to_int(source_row.get("waypointIndex")),
                    "candidateWaypointIndex": _to_int(candidate_row.get("waypointIndex")),
                }
            )
    candidate_attack_pairs = {
        (_to_int(row.get("aircraftID")), _to_int(row.get("targetID")))
        for row in candidate_attack_rows
        if _to_int(row.get("aircraftID")) is not None
        and _to_int(row.get("targetID")) is not None
    }
    expected_new_attack_pairs = _expected_attack_pairs_from_manned_sequences(
        manned_sequences
    )
    certified_deferred_attack_pairs = {
        (int(descriptor_result.get("aircraftID")), int(target_id))
        for descriptor_result in descriptor_results
        if isinstance(descriptor_result, dict)
        and isinstance(descriptor_result.get("update"), dict)
        for target_id in (
            descriptor_result.get("update", {}).get("deferredAttackTargetIDs") or []
        )
        if _to_int(descriptor_result.get("aircraftID")) is not None
        and _to_int(target_id) is not None
        and int(_to_int(target_id) or 0) > 0
    }
    continuity_decision = evaluate_candidate_attack_continuity(
        expected_new_pairs=expected_new_attack_pairs,
        candidate_pairs=candidate_attack_pairs,
        certified_deferred_pairs=certified_deferred_attack_pairs,
        scan_errors=candidate_attack_scan_errors,
        missing_committed_identities=missing_committed_identities,
        committed_order_mismatches=committed_order_mismatches,
    )
    successful_new_attack_pairs = list(
        continuity_decision.get("successfulNewPairs") or []
    )
    missing_new_attack_pairs = list(
        continuity_decision.get("deferredNewPairs") or []
    )
    continuity_ok = bool(continuity_decision.get("ok"))
    override_detail_timing["attackContinuityInvariant"] = {
        "ok": bool(continuity_ok),
        "structuralOk": bool(continuity_decision.get("structuralOk")),
        "sourceAttackCount": len(preserved_source_attack_rows),
        "candidateAttackCount": len(candidate_attack_rows),
        "missingCommittedIdentities": [list(item) for item in missing_committed_identities],
        "committedOrderMismatches": committed_order_mismatches,
        "successfulNewAttackPairs": [list(item) for item in successful_new_attack_pairs],
        "missingNewAttackPairs": [list(item) for item in missing_new_attack_pairs],
        "deferredNewAttackPairs": [list(item) for item in missing_new_attack_pairs],
        "certifiedDeferredAttackPairs": [
            list(item)
            for item in continuity_decision.get("certifiedDeferredPairs") or []
        ],
        "uncertifiedMissingAttackPairs": [
            list(item)
            for item in continuity_decision.get("uncertifiedMissingPairs") or []
        ],
        "partialSuccess": bool(continuity_decision.get("partialSuccess")),
        "scanErrors": list(candidate_attack_scan_errors),
    }
    if not continuity_ok:
        if bool(continuity_decision.get("allNewUnengageable")) and bool(
            continuity_decision.get("structuralOk")
        ):
            _set_override_failure(
                "attack_tactical_no_engageable_target",
                attack_failure_notice("attack_tactical_no_engageable_target"),
            )
        else:
            _set_override_failure(
                "attack_continuity_invariant_failed",
                attack_failure_notice("attack_override_failed"),
            )
        emit(
            "[ATTACK][CONTINUITY][ERR] Candidate plan rejected before MissionPlan/"
            "tracking-state commit; an existing or requested attack disappeared "
            f"(missingCommitted={missing_committed_identities}, "
            f"orderMismatches={committed_order_mismatches}, "
            f"missingNew={missing_new_attack_pairs}, errors={candidate_attack_scan_errors[:3]})."
        )
        return None
    deferred_new_attack_pair_set = {
        (int(aircraft_id), int(target_id))
        for aircraft_id, target_id in missing_new_attack_pairs
    }
    ctx.pop("_deferred_attack_pairs", None)
    ctx.pop("_deferred_attack_target_count", None)
    if deferred_new_attack_pair_set:
        # Builders already emitted certified paths for the successful subset.
        # Keep those paths and report/return only targets that really have a
        # firing waypoint.  Deferred contacts stay tracked and can be retried
        # by the next monitoring request instead of cancelling this strike.
        assigned_targets = [
            target
            for target in assigned_targets
            if (
                int(_to_int(target.get("assigned_manned_aircraft_id")) or 0),
                int(_to_int(target.get("target_id") or target.get("targetID")) or 0),
            )
            not in deferred_new_attack_pair_set
        ]
        ctx["_deferred_attack_pairs"] = [
            [int(aircraft_id), int(target_id)]
            for aircraft_id, target_id in missing_new_attack_pairs
        ]
        ctx["_deferred_attack_target_count"] = len(missing_new_attack_pairs)
        if assigned_targets:
            primary_target = dict(assigned_targets[0])
        emit(
            "[ATTACK][TACTICAL][WARN] Committing certified partial strike; "
            f"successful={successful_new_attack_pairs}, "
            f"deferred={missing_new_attack_pairs}."
        )
    emit(
        "[ATTACK][CONTINUITY] Candidate attack graph certified "
        f"(preserved={len(preserved_source_attack_rows)}, "
        f"new={len(successful_new_attack_pairs)}/{len(expected_new_attack_pairs)}, "
        f"deferred={len(missing_new_attack_pairs)}, "
        f"total={len(candidate_attack_rows)})."
    )

    # These files are already fully materialized in memory.  Their disk write
    # is independent from payload validation, so overlap the two and join
    # before committing the MissionPlan reference graph.
    deferred_write_started: Optional[float] = None
    deferred_write_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
    deferred_write_future: Optional[concurrent.futures.Future] = None
    if deferred_descriptor_write_entries and not tactical_transaction_required:
        deferred_write_started = time.perf_counter()
        deferred_write_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="AttackDescriptorWrite",
        )
        deferred_write_future = deferred_write_executor.submit(
            _write_json_files_batch,
            deferred_descriptor_write_entries,
        )

    validation_imp_payloads: List[Dict[str, Any]] = []
    validation_fp_payloads: List[Dict[str, Any]] = []
    for path, payload in deferred_descriptor_write_entries:
        if not isinstance(payload, dict):
            continue
        parent_name = Path(path).parent.name
        if parent_name == "IndividualMissionPlan":
            validation_imp_payloads.append(payload)
        elif parent_name == "FlightPath":
            validation_fp_payloads.append(payload)

    validation_started = time.perf_counter()
    try:
        for payload in validation_fp_payloads:
            normalize_flight_path_waypoint_altitudes_inplace(payload)
            normalize_flight_path_waypoint_speeds_inplace(payload)
        validation_summary = validate_replan_payloads(
            mission_plan=new_plan_data,
            individual_mission_plans=validation_imp_payloads,
            flight_paths=validation_fp_payloads,
            scope="attack_plan",
            allow_existing_db_artifacts=True,
            validate_existing_flight_path_waypoints=False,
            validate_existing_flight_path_links=False,
            log=emit,
        )
    except ReplanValidationError as exc:
        if deferred_write_executor is not None:
            deferred_write_executor.shutdown(wait=True, cancel_futures=False)
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
        suppliedImpPayloadCount=len(validation_imp_payloads),
        suppliedFlightPathPayloadCount=len(validation_fp_payloads),
    )

    if deferred_descriptor_write_entries and tactical_transaction_required:
        # Tactical transactions validate entirely in memory first.  Starting
        # the writer here prevents rejected concealment/LOS options from
        # leaving partial artifacts even when builders ran in parallel.
        deferred_write_started = time.perf_counter()
        deferred_write_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="AttackTacticalCommit",
        )
        deferred_write_future = deferred_write_executor.submit(
            _write_json_files_batch,
            deferred_descriptor_write_entries,
        )

    if deferred_write_future is not None:
        try:
            deferred_write_results = deferred_write_future.result()
        except Exception as exc:
            deferred_write_results = [{"error": f"{type(exc).__name__}: {exc}"}]
        finally:
            if deferred_write_executor is not None:
                deferred_write_executor.shutdown(wait=True, cancel_futures=False)
                deferred_write_executor = None
        deferred_write_errors = [
            row
            for row in deferred_write_results
            if isinstance(row, dict) and row.get("error")
        ]
        _record_phase(
            "descriptor_deferred_write_batch",
            deferred_write_started or validation_started,
            fileCount=len(deferred_write_results),
            writtenCount=sum(1 for row in deferred_write_results if row.get("written")),
            skippedCount=sum(1 for row in deferred_write_results if row.get("skipped")),
            errorCount=len(deferred_write_errors),
            overlappedWithValidation=not bool(tactical_transaction_required),
        )
        if deferred_write_errors:
            _set_override_failure(
                "attack_descriptor_write_failed",
                attack_failure_notice("attack_override_failed"),
            )
            emit(
                "[ATTACK][WRITE][ERR] descriptor deferred write failed: "
                f"{'; '.join(str(row.get('error')) for row in deferred_write_errors[:4])}"
            )
            return None

    if pending_tracking_assignments:
        tracking_state_started = time.perf_counter()
        committed_tracking_aircraft: List[int] = []
        try:
            for tracking_aircraft_id, tracking_assignment in pending_tracking_assignments:
                register_tracking_assignment(**tracking_assignment)
                committed_tracking_aircraft.append(int(tracking_aircraft_id))
                emit(
                    "[ATTACK][UAV] Tracking assignment state saved after "
                    "tactical validation "
                    f"(aircraft={int(tracking_aircraft_id)}, "
                    f"sourcePlan={source_plan_id}, attackPlan={new_plan_id})."
                )
        except Exception as exc:
            _set_override_failure(
                "attack_tracking_state_commit_failed",
                attack_failure_notice("attack_override_failed"),
            )
            emit(
                "[ATTACK][UAV][ERR] Tracking state commit failed before "
                f"MissionPlan commit: {type(exc).__name__}: {exc}"
            )
            return None
        _record_phase(
            "tracking_state_commit",
            tracking_state_started,
            aircraftIDs=committed_tracking_aircraft,
        )

    plan_write_started = time.perf_counter()
    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(plan_dest, new_plan_data)
    _queue_attack_snapshot_carry(
        int(source_plan_id),
        int(new_plan_id),
        reason="attack_replan",
    )
    emit(
        "[ATTACK][PLAN] area remaining snapshot carry queued "
        f"(sourcePlanID={source_plan_id}, planID={new_plan_id})."
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
            "targetCount": len(
                {
                    int(item["targetID"])
                    for item in attack_target_meta
                    if _to_int(item.get("targetID")) is not None
                    and int(item["targetID"]) > 0
                }
            ),
            "targetIDList": [
                {"targetID": int(target_id)}
                for target_id in dict.fromkeys(
                    int(item["targetID"])
                    for item in attack_target_meta
                    if _to_int(item.get("targetID")) is not None
                    and int(item["targetID"]) > 0
                )
            ],
            "targetID": next(
                (
                    int(item["targetID"])
                    for item in attack_target_meta
                    if _to_int(item.get("targetID")) is not None
                    and int(item["targetID"]) > 0
                ),
                None,
            ),
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


def _plan_lah_enemy_contact_response(
    descriptor: Dict[str, Any],
    state: Dict[str, Any],
    *,
    role: str,
    emit: Callable[[str], None],
) -> Optional[Dict[str, Any]]:
    """Run the production hide/communication adapter and log every fallback."""

    if get_runtime_attack_int("tactical_cover_enabled", 1) <= 0:
        emit(f"[ATTACK][TACTICAL] disabled for aircraft {descriptor.get('aircraft_id')}.")
        return None
    contact = descriptor.get("enemy_contact")
    if not isinstance(contact, dict):
        emit(
            f"[ATTACK][TACTICAL][WARN] contact context missing for aircraft "
            f"{descriptor.get('aircraft_id')}; tactical branch will fail closed."
        )
        return None
    # Keep the target coordinate for diagnostics, but do not constrain hide
    # selection by an attack-altitude ceiling.  The firing point is solved
    # afterwards at the minimum LOS altitude, even when that altitude exceeds
    # the ordinary LAH mission envelope.
    attack_target_coordinate: Optional[Dict[str, Any]] = None
    attack_ceiling_m: Optional[float] = None
    if str(role) == "attacker":
        assigned = descriptor.get("attack_targets")
        if isinstance(assigned, list) and assigned:
            first_target = assigned[0]
            if isinstance(first_target, dict):
                attack_target_coordinate = _normalize_coordinate(
                    first_target.get("coordinate")
                )

    # Concealment outranks link count.  The solver treats the required UAV link
    # count as hard, so a command aircraft that could hide with two links is
    # told to stay exposed with three.  Walk the requirement down and take the
    # first concealed answer; only if every rung fails does the caller fall back
    # to the exposed live-position hold.
    requested_links = get_runtime_attack_int(
        "tactical_relay_min_uav_links" if role == "relay" else "tactical_attacker_min_uav_links",
        3 if role == "relay" else 1,
    )
    link_floor = max(
        1,
        get_runtime_attack_int("tactical_min_uav_links_floor", 1),
    )
    link_ladder = [
        value for value in range(int(requested_links), int(link_floor) - 1, -1) if value >= 1
    ] or [1]

    try:
        from modules.mission_planning.pipelines.lah_enemy_contact import (
            plan_enemy_contact_response,
        )

        result = None
        for rung_links in link_ladder:
            result = plan_enemy_contact_response(
                int(descriptor.get("aircraft_id")),
                state,
                contact.get("uav_states") or [],
                contact.get("enemy_coordinates") or [],
                role=str(role),
                deadline_s=get_runtime_attack_float("tactical_hide_deadline_s", 10.0),
                reconnect_deadline_s=get_runtime_attack_float(
                    "tactical_reconnect_deadline_s", 60.0
                ),
                # Friendly engagement range - firing feasibility only.
                enemy_range_m=get_runtime_attack_float("tactical_enemy_range_m", 5000.0),
                # How far an enemy still constrains concealment.  Unlimited by
                # default: an enemy dropped for being "out of range" is excluded
                # from the masking analysis entirely, and the result then reports
                # enemyVisibleCount=0 for a point with no cover at all.  Detection
                # saturates with exposure time rather than distance - a 300 s hold
                # is seen well beyond weapon range, and live geometry has put every
                # aircraft 5.3-6.3 km from its enemy, i.e. outside the old cap.
                enemy_observation_range_m=get_runtime_attack_float(
                    "tactical_enemy_observation_range_m", 0.0
                ),
                communication_range_m=get_runtime_attack_float(
                    "tactical_communication_range_m", 10000.0
                ),
                min_uav_links=int(rung_links),
                hide_agl_m=get_runtime_attack_float("tactical_hide_agl_m", 50.0),
                max_hide_agl_m=get_runtime_attack_float("tactical_max_hide_agl_m", 1000.0),
                analysis_max_dim=get_runtime_attack_int("tactical_analysis_max_dim", 340),
                refinement_radius_m=get_runtime_attack_float(
                    "tactical_refinement_radius_m", 300.0
                ),
                max_precision_candidates=get_runtime_attack_int(
                    "tactical_max_precision_candidates", 24
                ),
                allow_best_effort=get_runtime_attack_int("tactical_allow_best_effort", 1) > 0,
                degraded_min_uav_links=(
                    get_runtime_attack_int("tactical_relay_degraded_min_uav_links", 2)
                    if role == "relay"
                    else None
                ),
                timing_guard_s=get_runtime_attack_float("tactical_timing_guard_s", 1.0),
                attack_target_coordinate=attack_target_coordinate,
                attack_ceiling_m=attack_ceiling_m,
            )
            if isinstance(result, dict) and result.get("applied"):
                if int(rung_links) < int(requested_links):
                    emit(
                        "[ATTACK][TACTICAL] Concealment took priority over link count "
                        f"(aircraft={descriptor.get('aircraft_id')}, role={role}, "
                        f"links={rung_links}/{requested_links})."
                    )
                break
    except Exception as exc:
        emit(
            f"[ATTACK][TACTICAL][WARN] planner invocation failed for aircraft "
            f"{descriptor.get('aircraft_id')}: {type(exc).__name__}: {exc}; preserving legacy route."
        )
        return None

    timing = result.get("timingMs") if isinstance(result, dict) else {}
    emit(
        "[ATTACK][TACTICAL] "
        f"aircraft={descriptor.get('aircraft_id')} role={role} "
        f"status={(result or {}).get('status')} applied={bool((result or {}).get('applied'))} "
        f"hide={(result or {}).get('hideAchievedS')}s "
        f"relayEta={(result or {}).get('responseEtaS', (result or {}).get('etaS'))}s "
        f"hideDeadlineMet={(result or {}).get('hideDeadlineMet', (result or {}).get('deadlineMet'))} "
        f"reconnectDeadlineMet={(result or {}).get('reconnectDeadlineMet')} "
        f"enemyVisible={(result or {}).get('enemyVisibleCount')} "
        f"uavLinks={(result or {}).get('uavLinkCount')}/{(result or {}).get('requiredUavLinks')} "
        f"warnings={(result or {}).get('warningCodes')} "
        f"rejectedUavs={(result or {}).get('rejectedUavs')} "
        f"timingMs={json.dumps(_json_safe(timing), ensure_ascii=False)} "
        f"failures={(result or {}).get('failureCodes')} "
        f"detail={(result or {}).get('detail')}."
    )
    if not isinstance(result, dict) or not result.get("applied"):
        fallback_text = (
            "fail-closed live-position hold is required"
            if str(role) == "relay"
            else "attack route must not bypass tactical certification"
        )
        emit(
            f"[ATTACK][TACTICAL][WARN] no certified {role} prelude for aircraft "
            f"{descriptor.get('aircraft_id')}; {fallback_text}."
        )
    return result if isinstance(result, dict) else None


def _lah_tactical_endpoint_coordinate(
    plan: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    if not isinstance(plan, dict) or not plan.get("applied"):
        return None
    endpoint = plan.get("endpoint")
    return _normalize_coordinate(endpoint) if isinstance(endpoint, dict) else None


def _manned_group_coordinates_from_ctx(
    ctx: Optional[Dict[str, Any]],
    *,
    exclude_aircraft_id: Optional[int] = None,
) -> List[Dict[str, Any]]:
    """Live positions of the other manned aircraft in this replan."""

    rows = (ctx or {}).get("_selected_manned_aircraft")
    out: List[Dict[str, Any]] = []
    for row in rows if isinstance(rows, list) else []:
        if not isinstance(row, dict):
            continue
        identifier = _to_int(row.get("aircraft_id") or row.get("aircraftID"))
        if identifier is None or identifier not in (1, 2, 3):
            continue
        if exclude_aircraft_id is not None and int(identifier) == int(exclude_aircraft_id):
            continue
        coord = _normalize_coordinate(row.get("coordinate"))
        if coord is not None:
            out.append(coord)
    return out


def _keep_lah_with_group(
    coord: Optional[Dict[str, Any]],
    *,
    group_coords: List[Dict[str, Any]],
    target_coord: Optional[Dict[str, Any]] = None,
    emit: Optional[Callable[[str], None]] = None,
    aircraft_id: Optional[int] = None,
) -> Optional[Dict[str, Any]]:
    """Hold a manned aircraft with its flight, and never ahead of it.

    Attack and post-attack points are solved per aircraft, so one of them can
    be sent kilometres forward on its own.  Two bounds fix that: stay within a
    spread of where the other manned aircraft are, and stay no closer to the
    target than they are.  Altitude is untouched - it is owned by the LOS and
    terrain passes.
    """

    candidate = _normalize_coordinate(coord)
    if candidate is None or not group_coords:
        return coord
    if not bool(get_runtime_attack_int("lah_group_cohesion_enabled", 1)):
        return coord

    centroid = _average_coordinate(group_coords)
    if centroid is None:
        return coord

    adjusted = dict(candidate)
    reasons: List[str] = []

    spread_m = max(0.0, get_runtime_attack_float("lah_group_max_spread_m", 1500.0))
    if spread_m > 0.0:
        distance_m = _haversine_distance_m(centroid, adjusted)
        if distance_m is not None and float(distance_m) > spread_m:
            bearing = _bearing_between(
                float(centroid["latitude"]),
                float(centroid["longitude"]),
                float(adjusted["latitude"]),
                float(adjusted["longitude"]),
            )
            pulled = _normalize_coordinate(
                _project_coordinate(centroid, bearing, spread_m)
            )
            if pulled is not None:
                reasons.append(f"spread {float(distance_m):.0f}m>{spread_m:.0f}m")
                pulled["altitude"] = adjusted.get("altitude")
                adjusted = pulled

    target = _normalize_coordinate(target_coord)
    if target is not None:
        group_standoff_m = _haversine_distance_m(target, centroid)
        own_standoff_m = _haversine_distance_m(target, adjusted)
        if (
            group_standoff_m is not None
            and own_standoff_m is not None
            and float(own_standoff_m) < float(group_standoff_m)
        ):
            bearing = _bearing_between(
                float(target["latitude"]),
                float(target["longitude"]),
                float(adjusted["latitude"]),
                float(adjusted["longitude"]),
            )
            pushed = _normalize_coordinate(
                _project_coordinate(target, bearing, float(group_standoff_m))
            )
            if pushed is not None:
                reasons.append(
                    f"ahead of flight {float(own_standoff_m):.0f}m<{float(group_standoff_m):.0f}m"
                )
                pushed["altitude"] = adjusted.get("altitude")
                adjusted = pushed

    if reasons and emit is not None:
        emit(
            "[ATTACK][LAH] Kept with the flight "
            f"(aircraft={aircraft_id}, {', '.join(reasons)})."
        )
    for key, value in candidate.items():
        adjusted.setdefault(key, value)
    return adjusted


def _attack_wait_hold_seconds(
    ctx: Optional[Dict[str, Any]],
    state: Optional[Dict[str, Any]],
) -> Optional[int]:
    """How long a wingman should sit in cover while the strike happens.

    Estimated as the time for the flight to close on the target at attack
    speed, plus the pop-up dwell at each end.  ``None`` when the geometry is
    unknown, so the caller keeps its configured hold.
    """

    target = _attack_group_target_coordinate(ctx)
    group = _manned_group_coordinates_from_ctx(ctx)
    origin = _average_coordinate(group) or _normalize_coordinate((state or {}).get("coordinate"))
    if target is None or origin is None:
        return None
    distance_m = _haversine_distance_m(origin, target)
    if distance_m is None:
        return None
    speed_mps = _lah_max_attack_speed_mps()
    if not math.isfinite(speed_mps) or speed_mps <= 0.0:
        return None
    travel_s = float(distance_m) / float(speed_mps)
    dwell_s = 2.0 * float(_attack_cover_hold_seconds())
    minimum_s = max(0, get_runtime_attack_int("lah_wait_hold_min_seconds", 30))
    maximum_s = max(minimum_s, get_runtime_attack_int("lah_wait_hold_max_seconds", 600))
    return int(min(maximum_s, max(minimum_s, math.ceil(travel_s + dwell_s))))


def _attack_group_target_coordinate(ctx: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """The direction 'forward' means for this replan: the discovered targets."""

    rows = (ctx or {}).get("_attack_target_list")
    coords: List[Dict[str, Any]] = []
    for row in rows if isinstance(rows, list) else []:
        if not isinstance(row, dict):
            continue
        coord = _normalize_coordinate(row.get("coordinate"))
        if coord is not None:
            coords.append(coord)
    return _average_coordinate(coords)


def _average_coordinate(coords: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    rows = [_normalize_coordinate(item) for item in coords or []]
    rows = [item for item in rows if item is not None]
    if not rows:
        return None
    return {
        "latitude": sum(float(item["latitude"]) for item in rows) / float(len(rows)),
        "longitude": sum(float(item["longitude"]) for item in rows) / float(len(rows)),
    }


def _offset_coordinate_m(
    coord: Dict[str, Any],
    east_m: float,
    north_m: float,
) -> Dict[str, Any]:
    """Shift a coordinate by a local east/north displacement in metres."""

    lat_scale = 111_132.0
    lon_scale = 111_320.0 * max(math.cos(math.radians(float(coord["latitude"]))), 0.01)
    shifted = dict(coord)
    shifted["latitude"] = float(coord["latitude"]) + (float(north_m) / lat_scale)
    shifted["longitude"] = float(coord["longitude"]) + (float(east_m) / lon_scale)
    return shifted


def _attack_popup_candidates(hide: Dict[str, Any]) -> List[Tuple[Dict[str, Any], float]]:
    """Where the aircraft may pop up from, nearest first.

    A straight-up climb is measured against the one DEM column the hide point
    stands on, so a ridge a few tens of metres away forces the whole climb.
    Stepping aside a little - a diagonal pop-up - often clears the same ridge
    hundreds of metres lower, which is the difference between a survivable
    exposure and a long one.
    """

    radius_m = max(0.0, get_runtime_attack_float("attack_popup_search_radius_m", 600.0))
    step_m = max(10.0, get_runtime_attack_float("attack_popup_search_step_m", 150.0))
    bearings = max(4, get_runtime_attack_int("attack_popup_search_bearings", 8))

    candidates: List[Tuple[Dict[str, Any], float]] = [(dict(hide), 0.0)]
    if radius_m <= 0.0:
        return candidates
    ring = step_m
    while ring <= radius_m + 1e-6:
        for index in range(int(bearings)):
            bearing_rad = (2.0 * math.pi * float(index)) / float(bearings)
            east_m = ring * math.sin(bearing_rad)
            north_m = ring * math.cos(bearing_rad)
            candidates.append((_offset_coordinate_m(hide, east_m, north_m), float(ring)))
        ring += step_m
    return candidates


def _attack_popup_other_enemy_exposure(
    attack_coord: Dict[str, Any],
    attack_target_coord: Dict[str, Any],
    threat_targets: Any,
    *,
    attack_target_id: Optional[int] = None,
) -> Dict[str, Any]:
    """Verify that a firing point stays masked from every non-target threat.

    The designated target is expected to see the aircraft during the shot;
    all other currently supplied enemies inside the tactical threat range must
    remain terrain-masked. Unknown DEM/LOS is counted separately and therefore
    fails closed at the caller.
    """

    rows = list(threat_targets) if isinstance(threat_targets, (list, tuple)) else []
    if not rows:
        return {
            "checked": False,
            "consideredCount": 0,
            "visibleCount": 0,
            "unknownCount": 0,
            "targetExcluded": False,
        }

    attack = _normalize_coordinate(attack_coord)
    designated_target = _normalize_coordinate(attack_target_coord)
    if attack is None or designated_target is None:
        return {
            "checked": True,
            "consideredCount": 0,
            "visibleCount": 0,
            "unknownCount": 1,
            "targetExcluded": False,
            "reason": "invalid_popup_or_target_coordinate",
        }

    max_range_m = max(
        0.0,
        float(get_runtime_attack_float("tactical_enemy_range_m", 5000.0)),
    )
    visible_count = 0
    unknown_count = 0
    considered_count = 0
    target_excluded = False
    reasons: List[str] = []
    for raw in rows:
        if not isinstance(raw, dict):
            unknown_count += 1
            reasons.append("invalid_enemy_entry")
            continue
        enemy = _normalize_coordinate(raw.get("coordinate") or raw)
        enemy_id = _to_int(
            raw.get("target_id")
            or raw.get("targetID")
            or raw.get("id")
        )
        same_designated_id = bool(
            attack_target_id is not None
            and enemy_id is not None
            and int(enemy_id) == int(attack_target_id)
        )
        same_designated_position = False
        if enemy is not None and not target_excluded:
            distance_to_designated_m = _haversine_distance_m(
                enemy,
                designated_target,
            )
            same_designated_position = bool(
                distance_to_designated_m is not None
                and float(distance_to_designated_m) <= 10.0
            )
        if same_designated_id or (
            attack_target_id is None and same_designated_position
        ):
            target_excluded = True
            continue
        # Some legacy bundles strip target IDs. Exclude exactly one coordinate
        # matching the designated target, but never hide a second co-located
        # threat once that target has already been removed.
        if enemy_id is None and same_designated_position and not target_excluded:
            target_excluded = True
            continue
        if enemy is None:
            unknown_count += 1
            reasons.append("invalid_enemy_coordinate")
            continue

        distance_m = _haversine_distance_m(enemy, attack)
        if distance_m is None:
            unknown_count += 1
            reasons.append("enemy_range_unavailable")
            continue
        if max_range_m > 0.0 and float(distance_m) > float(max_range_m):
            continue
        considered_count += 1
        try:
            assessment = evaluate_regional_los(
                resource_dir=_PROJECT_ROOT / "resource",
                observer_latitude=float(enemy["latitude"]),
                observer_longitude=float(enemy["longitude"]),
                observer_altitude_m=float(enemy.get("altitude") or 0.0),
                target_latitude=float(attack["latitude"]),
                target_longitude=float(attack["longitude"]),
                target_altitude_m=float(attack.get("altitude") or 0.0),
                observer_height_m=float(ENEMY_OBSERVER_HEIGHT_M),
                target_height_m=0.0,
                max_range_m=float(max_range_m) if max_range_m > 0.0 else None,
                reject_nodata=False,
            )
        except Exception as exc:
            assessment = {
                "visible": None,
                "reason": f"LOS_EXCEPTION:{type(exc).__name__}",
            }
        visible = assessment.get("visible") if isinstance(assessment, dict) else None
        if visible is True:
            visible_count += 1
        elif visible is not False:
            unknown_count += 1
        reason = assessment.get("reason") if isinstance(assessment, dict) else None
        if reason:
            reasons.append(str(reason))

    return {
        "checked": True,
        "consideredCount": int(considered_count),
        "visibleCount": int(visible_count),
        "unknownCount": int(unknown_count),
        "targetExcluded": bool(target_excluded),
        "reasons": list(dict.fromkeys(reasons)),
    }


# The enemy observes from its own height; matching the shared policy keeps the
# certification identical to the one the simulator applies.
_ENEMY_OBSERVER_HEIGHT_M = 5.0
_ATTACK_VISIBILITY_RESOLUTION_M = 5.0


def _attack_los_resource_dir() -> Optional[Path]:
    """The repository ``resource`` directory that actually holds the DEM tiles.

    Nested package directories can carry an empty ``resource`` stub, so the
    presence of a tile - not of the directory - is what identifies it.
    """

    return next(
        (
            parent / "resource"
            for parent in Path(__file__).resolve().parents
            if any((parent / "resource").glob("*.tif"))
        ),
        None,
    )


def _lowest_firing_altitude_m(
    *,
    latitude: float,
    longitude: float,
    floor_m: float,
    ceiling_m: Optional[float],
    target: Dict[str, Any],
) -> Optional[float]:
    """Lowest altitude over this point from which the shot actually clears.

    Certified with the evaluator that gates the shot, with the enemy as the
    observer, so "the plan says fire" and "the round can reach" cannot disagree.
    ``ceiling_m`` is retained as an initial probe for compatibility, not as a
    hard operational limit.  If that probe is still masked, the search expands
    upward until the canonical LOS evaluator opens and then bisects downward.
    """

    target_norm = _normalize_coordinate(target)
    if target_norm is None:
        return float(floor_m)
    resource_dir = _attack_los_resource_dir()
    if resource_dir is None:
        return float(floor_m)
    try:
        from modules.monitoring.logic.dem_cover.los_api import evaluate_regional_los
    except Exception:
        return float(floor_m)

    def visible(altitude_m: float) -> Optional[bool]:
        try:
            assessment = evaluate_regional_los(
                resource_dir=resource_dir,
                observer_latitude=float(target_norm["latitude"]),
                observer_longitude=float(target_norm["longitude"]),
                observer_altitude_m=float(target_norm.get("altitude") or 0.0),
                observer_height_m=float(_ENEMY_OBSERVER_HEIGHT_M),
                target_latitude=float(latitude),
                target_longitude=float(longitude),
                target_altitude_m=float(altitude_m),
            )
        except Exception:
            return None
        if not isinstance(assessment, dict) or not assessment.get("demAvailable", True):
            return None
        return bool(assessment.get("visible"))

    low = float(floor_m)
    at_floor = visible(low)
    if at_floor is None:
        return float(floor_m)  # evaluator unusable; keep the solver's answer
    if at_floor:
        return low
    try:
        initial_high = float(ceiling_m) if ceiling_m is not None else float("nan")
    except (TypeError, ValueError):
        initial_high = float("nan")
    high = (
        initial_high
        if math.isfinite(initial_high) and initial_high > low
        else low + 250.0
    )
    step_m = max(250.0, high - low)
    high_visible = visible(high)
    if high_visible is None:
        return low
    # This is a numerical convergence guard, not an altitude ceiling.  The
    # target endpoint contribution grows monotonically, so a finite terrain
    # profile will open long before the guard is exhausted.
    for _attempt in range(20):
        if high_visible:
            break
        high += step_m
        step_m *= 2.0
        high_visible = visible(high)
        if high_visible is None:
            return low
    if not high_visible:
        # The analytical DEM solver already supplied a firing altitude.  Do
        # not delete the attack merely because secondary certification could
        # not find its crossing; retain that closest computed point.
        return low
    # Terrain masking is monotone in altitude, so bisect for the crossing.
    while (high - low) > _ATTACK_VISIBILITY_RESOLUTION_M:
        mid = (low + high) * 0.5
        if visible(mid):
            high = mid
        else:
            low = mid
    return high


def _attack_coordinate_at_hide_endpoint(
    hide_coord: Optional[Dict[str, Any]],
    target_coord: Optional[Dict[str, Any]],
    *,
    threat_targets: Any = None,
    attack_target_id: Optional[int] = None,
    emit: Optional[Callable[[str], None]] = None,
    aircraft_id: Optional[int] = None,
) -> Optional[Dict[str, Any]]:
    """Turn the certified hide point into the firing point above it.

    A gunship engages from cover: it holds behind terrain and climbs just
    enough to see the target, it does not fly to a separate firing position and
    back.  The concealment endpoint already sits on terrain that masks it, so
    the attack point is that same latitude/longitude with the altitude the LOS
    profile says is needed.  There is deliberately no attack-altitude ceiling:
    proximity to the certified hide point wins, and the selected point climbs
    to the lowest altitude at which the target LOS is open.
    """

    hide = _normalize_coordinate(hide_coord)
    target = _normalize_coordinate(target_coord)
    if hide is None or target is None:
        return None
    if not bool(get_runtime_attack_int("attack_point_at_hide_endpoint", 1)):
        return None

    # Climb only as far as the sightline needs.  The ordinary attack point sits
    # ground+300 m because it is chosen without regard to cover; popping up out
    # of a hide position must not throw that concealment away.
    popup_margin_m = get_runtime_attack_float("attack_point_hide_popup_margin_m", 30.0)

    best: Optional[Dict[str, Any]] = None
    best_altitude_m: Optional[int] = None
    best_score: Optional[Tuple[float, int]] = None
    best_offset_m = 0.0
    best_exposure: Optional[Dict[str, Any]] = None
    fallback_best: Optional[Dict[str, Any]] = None
    fallback_altitude_m: Optional[int] = None
    fallback_offset_m = 0.0
    fallback_exposure: Optional[Dict[str, Any]] = None
    fallback_score: Optional[Tuple[int, float, int]] = None
    first_error: Optional[str] = None
    solved = 0
    # Search the hide point first and its immediate neighbourhood afterwards.
    # With no attack ceiling, the nearest valid point is preferred even when a
    # farther point could save climb altitude.
    vertical_only = bool(get_runtime_attack_int("attack_popup_vertical_only", 0))
    candidates = (
        [(dict(hide), 0.0)]
        if vertical_only
        else _attack_popup_candidates(hide)
    )
    for candidate, offset_m in candidates:
        result, error = _compute_attack_los_altitude_batch_dem(
            candidate,
            target,
            # The hide altitude is the floor: an aircraft that already sees the
            # target has nothing to gain by descending for the shot.
            lah_floor_coord=hide,
            altitude_offset_m=float(popup_margin_m),
        )
        altitude_m = (
            _normalize_altitude_value(result.get("altitude"))
            if isinstance(result, dict)
            else None
        )
        if altitude_m is None:
            if first_error is None:
                first_error = error
            continue
        # The solver and the evaluator that gates the shot read the enemy's own
        # ground cell differently, so certify each candidate at the altitude the
        # shot really needs.  Comparing candidates on the solver's number picked
        # points the aircraft could reach but never fire from.
        certified_m = _lowest_firing_altitude_m(
            latitude=float(candidate["latitude"]),
            longitude=float(candidate["longitude"]),
            floor_m=float(altitude_m),
            ceiling_m=None,
            target=target,
        )
        if certified_m is None:
            if first_error is None:
                first_error = (
                    "no certifiable altitude here is visible to the target "
                    f"(solver wanted {int(altitude_m)}m)"
                )
            continue
        altitude_m = int(round(float(certified_m)))
        exposure = _attack_popup_other_enemy_exposure(
            {**candidate, "altitude": int(altitude_m)},
            target,
            threat_targets,
            attack_target_id=attack_target_id,
        )
        exposure_failed = bool(exposure.get("checked")) and (
            int(exposure.get("visibleCount") or 0) > 0
            or int(exposure.get("unknownCount") or 0) > 0
        )
        if exposure_failed:
            if first_error is None:
                first_error = (
                    "popup exposed to non-target enemies or LOS was unknown "
                    f"(visible={int(exposure.get('visibleCount') or 0)}, "
                    f"unknown={int(exposure.get('unknownCount') or 0)})"
                )
            # Prefer a point hidden from the other enemies, but never delete
            # the designated attack if none exists.  Keep the least-exposed,
            # spatially closest LOS point as an explicit degraded fallback.
            degraded_score = (
                int(exposure.get("visibleCount") or 0)
                + int(exposure.get("unknownCount") or 0),
                float(offset_m),
                int(altitude_m),
            )
            if fallback_score is None or degraded_score < fallback_score:
                fallback_best = dict(result)
                fallback_best["latitude"] = float(candidate["latitude"])
                fallback_best["longitude"] = float(candidate["longitude"])
                fallback_best["altitude"] = int(altitude_m)
                fallback_altitude_m = int(altitude_m)
                fallback_offset_m = float(offset_m)
                fallback_exposure = dict(exposure)
                fallback_score = degraded_score
            continue
        solved += 1
        # With the altitude ceiling removed, the point closest to cover wins.
        # Altitude only breaks ties between points at the same lateral offset.
        score = (float(offset_m), int(altitude_m))
        if best_score is None or score < best_score or (
            score == best_score and float(offset_m) < float(best_offset_m)
        ):
            best = dict(result)
            best["latitude"] = float(candidate["latitude"])
            best["longitude"] = float(candidate["longitude"])
            best["altitude"] = int(altitude_m)
            best_altitude_m = int(altitude_m)
            best_score = score
            best_offset_m = float(offset_m)
            best_exposure = dict(exposure)

    if best is None and fallback_best is not None and fallback_altitude_m is not None:
        best = fallback_best
        best_altitude_m = int(fallback_altitude_m)
        best_offset_m = float(fallback_offset_m)
        best_exposure = dict(fallback_exposure or {})
        best["attack_other_enemy_exposure_fallback"] = True
        solved += 1
        if emit is not None:
            emit(
                "[ATTACK][TACTICAL][WARN] No popup is masked from every other "
                "enemy; keeping the least-exposed nearest LOS attack point "
                f"(aircraft={aircraft_id}, visible="
                f"{int((fallback_exposure or {}).get('visibleCount') or 0)}, "
                f"unknown={int((fallback_exposure or {}).get('unknownCount') or 0)})."
            )

    if best is None or best_altitude_m is None:
        if emit is not None:
            emit(
                "[ATTACK][TACTICAL][WARN] Could not solve a climb needed "
                f"to see the target from the hide point (aircraft={aircraft_id}); "
                f"the tactical attack must fail closed. detail={first_error or 'unknown'}"
            )
        return None

    coordinate = dict(best)
    # A firing point that sits exactly on the hide point is a contradiction:
    # concealment means the sightline is blocked, and a shot means it is open.
    # It happens when the sightline grazes the terrain by well under a metre, so
    # the two tests read the same ridge differently.  Always leave cover by a
    # real, flyable margin.
    hide_altitude_m = _normalize_altitude_value(hide.get("altitude"))
    if hide_altitude_m is not None and best_offset_m <= 0.5:
        floor_altitude_m = int(hide_altitude_m) + int(_LAH_MIN_POPUP_CLIMB_M)
        if int(best_altitude_m) < floor_altitude_m:
            best_altitude_m = floor_altitude_m
            coordinate["altitude"] = int(floor_altitude_m)
    coordinate["attack_point_at_hide_endpoint"] = True
    coordinate["attack_point_vertical_popup"] = bool(best_offset_m <= 0.5)
    if isinstance(best_exposure, dict):
        coordinate["attack_other_enemy_los_checked"] = bool(
            best_exposure.get("checked")
        )
        coordinate["attack_other_enemy_considered_count"] = int(
            best_exposure.get("consideredCount") or 0
        )
        coordinate["attack_other_enemy_visible_count"] = int(
            best_exposure.get("visibleCount") or 0
        )
        coordinate["attack_other_enemy_unknown_count"] = int(
            best_exposure.get("unknownCount") or 0
        )
    if best_offset_m > 0.0:
        coordinate["attack_point_popup_offset_m"] = float(best_offset_m)
    if emit is not None:
        hide_altitude = _normalize_altitude_value(hide.get("altitude"))
        climb_text = (
            f"climb={int(best_altitude_m) - int(hide_altitude)}m"
            if hide_altitude is not None
            else "climb=unknown"
        )
        emit(
            "[ATTACK][TACTICAL] Attack point placed above the hide endpoint "
            f"(aircraft={aircraft_id}, alt={coordinate.get('altitude')}m, {climb_text}, "
            f"lateral={int(best_offset_m)}m, candidates={solved}/{len(candidates)}, "
            f"otherEnemyVisible={coordinate.get('attack_other_enemy_visible_count', 0)}, "
            f"otherEnemyUnknown={coordinate.get('attack_other_enemy_unknown_count', 0)})."
        )
    return coordinate


def _lah_tactical_cover_required(descriptor: Dict[str, Any]) -> bool:
    """Whether this enemy-contact descriptor must fail closed without cover."""

    if get_runtime_attack_int("tactical_cover_enabled", 1) <= 0:
        return False
    # Missing/malformed contact data is itself a certification failure.  It
    # must not turn the safety feature off and let a legacy attack route pass.
    return str(descriptor.get("mode") or "").strip().upper() in {
        "LAH_ATTACK",
        "LAH_ATTACK_APPEND",
        "LAH_RELAY",
    }


def _mandatory_tactical_descriptor_failures(
    descriptors: Iterable[Dict[str, Any]],
    descriptor_results: Iterable[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Return required LAH branches missing a usable certified update."""

    required_modes = {"LAH_ATTACK", "LAH_ATTACK_APPEND", "LAH_RELAY"}
    expected: set[Tuple[int, str]] = set()
    for descriptor in descriptors:
        if not isinstance(descriptor, dict):
            continue
        mode = str(descriptor.get("mode") or "")
        aircraft_id = _to_int(descriptor.get("aircraft_id"))
        if mode in required_modes and aircraft_id is not None:
            expected.add((int(aircraft_id), mode))

    result_by_key: Dict[Tuple[int, str], Dict[str, Any]] = {}
    for row in descriptor_results:
        if not isinstance(row, dict):
            continue
        mode = str(row.get("mode") or "")
        aircraft_id = _to_int(row.get("aircraftID"))
        if mode in required_modes and aircraft_id is not None:
            result_by_key[(int(aircraft_id), mode)] = row

    failures: List[Dict[str, Any]] = []
    for aircraft_id, mode in sorted(expected):
        row = result_by_key.get((int(aircraft_id), mode))
        if (
            row is not None
            and str(row.get("status") or "") == "ok"
            and isinstance(row.get("update"), dict)
        ):
            continue
        failures.append(
            {
                "aircraftID": int(aircraft_id),
                "mode": str(mode),
                "status": str((row or {}).get("status") or "missing_result"),
                "row": row,
            }
        )
    return failures


def _build_lah_tactical_route_waypoints(
    *,
    template_wp: Dict[str, Any],
    plan: Optional[Dict[str, Any]],
    waypoint_id_provider: Callable[[], int],
    terminal_hover_seconds: int = 0,
) -> List[Dict[str, Any]]:
    """Serialize certified 3-D/timestamped WPs using ICD metre altitudes."""

    if not isinstance(plan, dict) or not plan.get("applied"):
        return []
    raw_waypoints = [
        item for item in (plan.get("routeWaypoints") or []) if isinstance(item, dict)
    ]
    normalized: List[Dict[str, Any]] = []
    previous_eta_s = 0
    for raw in raw_waypoints:
        coord = _normalize_coordinate(raw)
        altitude = _to_float(raw.get("altitude"))
        speed_mps = _to_float(raw.get("speedMps"))
        eta_s = _to_float(raw.get("etaS"))
        if coord is None or altitude is None or speed_mps is None or eta_s is None:
            continue
        waypoint = deepcopy(template_wp)
        waypoint["waypointID"] = int(waypoint_id_provider())
        waypoint["isDone"] = False
        # Keep the certified MSL altitude, but quantize only at the 0304 ICD
        # boundary.  The tactical solver retains sub-metre precision internally.
        waypoint["coordinate"] = {
            "latitude": round(float(coord["latitude"]), 7),
            "longitude": round(float(coord["longitude"]), 7),
            "altitude": int(round(float(altitude))),
        }
        # A stop is expressed by ``hovering``/``loiter``, never by speed: every
        # ordinary waypoint carries its transit speed even when the aircraft
        # will hold there.  A zero here is unflyable - the aircraft simply never
        # departs - so the solver's leading zero-speed nodes get the transit
        # floor instead.
        waypoint["speed"] = round(
            max(_LAH_MIN_TRANSIT_SPEED_MPS, float(speed_mps)), 2
        )
        serialized_eta_s = max(
            previous_eta_s,
            int(math.ceil(max(0.0, float(eta_s)) - 1e-9)),
        )
        if normalized and serialized_eta_s <= previous_eta_s:
            previous_coord = _extract_lah_waypoint_coordinate(normalized[-1])
            if not _same_lah_3d_coordinate(previous_coord, coord):
                serialized_eta_s = min(0xFFFFFFFF, previous_eta_s + 1)
        waypoint["eta"] = serialized_eta_s
        previous_eta_s = int(waypoint["eta"])
        waypoint["ecf"] = 0.0
        waypoint["nextWaypointID"] = 0
        waypoint["hovering"] = {"time": 0}
        waypoint["loiter"] = {"radius": 0, "direction": 0, "time": 0, "speed": 0}
        waypoint["attack"] = {"targetID": 0, "weaponType": 0}
        normalized.append(waypoint)

    if not normalized:
        return []
    for index, waypoint in enumerate(normalized[:-1]):
        waypoint["nextWaypointID"] = int(normalized[index + 1]["waypointID"])
    distances = [max(0.0, _to_float(item.get("distanceM")) or 0.0) for item in raw_waypoints]
    total_distance = max(distances[-1] if distances else 0.0, 0.0)
    # ICD ecf is litres burned on the leg into each waypoint, not progress.
    _apply_leg_fuel(normalized)
    if terminal_hover_seconds > 0:
        normalized[-1]["hovering"] = {"time": int(terminal_hover_seconds)}
    return normalized


def _split_lah_cover_ingress_waypoints(
    route_waypoints: List[Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Cut a certified ingress route into (movement, cover-entry) waypoint lists.

    The run to cover and the purpose action at cover are separate individual
    missions, but they share exactly one waypoint: the certified hide endpoint.
    The cut therefore falls *before* that endpoint, which becomes the first
    waypoint of the purpose path.  Cutting after it instead would leave the
    purpose path starting where the aircraft already is - the zero-length leg
    this module documents as a live failure mode, where the aircraft never
    registers the arrival.

    A route with fewer than two waypoints has no movement leg to separate, so
    the whole route is returned as the cover entry.
    """

    route = [deepcopy(item) for item in (route_waypoints or []) if isinstance(item, dict)]
    if len(route) < 2:
        return [], route
    ingress, cover_entry = route[:-1], route[-1:]
    # The movement path ends here: it may not link into another path's IDs, and
    # any dwell belongs to the cover entry that follows it.
    ingress[-1]["nextWaypointID"] = 0
    ingress[-1]["hovering"] = {"time": 0}
    _apply_leg_fuel(ingress)
    _apply_leg_fuel(cover_entry)
    return ingress, cover_entry


def _rebase_lah_path_etas(
    waypoints: List[Dict[str, Any]],
    *,
    base_eta_s: int,
) -> None:
    """Shift a standalone path's ETAs so they start from its own departure.

    ETAs accumulate across the whole certified route, so once the movement legs
    leave for their own individual mission the remaining path still carries
    their elapsed time.  Subtracting the movement's final ETA preserves every
    leg duration - the first waypoint keeps the true flight time of the leg into
    cover - while re-zeroing outright would erase it.
    """

    offset = max(0, int(base_eta_s or 0))
    previous_eta_s = 0
    for index, waypoint in enumerate(waypoints or []):
        if not isinstance(waypoint, dict):
            continue
        eta_s = max(0, (_to_int(waypoint.get("eta")) or 0) - offset)
        eta_s = min(0xFFFFFFFF, eta_s)
        if index and eta_s <= previous_eta_s:
            previous_coord = _extract_lah_waypoint_coordinate(waypoints[index - 1])
            current_coord = _extract_lah_waypoint_coordinate(waypoint)
            if not _same_lah_3d_coordinate(previous_coord, current_coord):
                eta_s = min(0xFFFFFFFF, previous_eta_s + 1)
        waypoint["eta"] = int(eta_s)
        previous_eta_s = int(eta_s)


def _lah_attack_window_seconds_from_waypoints(
    popup_waypoints: List[Dict[str, Any]],
) -> Optional[int]:
    """Measure the hide -> fire -> regain-hide window of a built attack path.

    A wingman holding in cover must wait out the shooter's whole exposure, so
    the number is taken from the emitted waypoints rather than re-estimated:
    the ETA span covers climb, shot and descent, and the terminal dwells cover
    the settle time at either end.
    """

    waypoints = [item for item in (popup_waypoints or []) if isinstance(item, dict)]
    if len(waypoints) < 2:
        return None
    first_eta_s = _to_int(waypoints[0].get("eta")) or 0
    last_eta_s = _to_int(waypoints[-1].get("eta")) or 0
    span_s = max(0, int(last_eta_s) - int(first_eta_s))
    dwell_s = 0
    for waypoint in (waypoints[0], waypoints[-1]):
        hovering = waypoint.get("hovering") if isinstance(waypoint.get("hovering"), dict) else {}
        dwell_s += max(0, _to_int(hovering.get("time")) or 0)
    return int(span_s + dwell_s)


def _record_lah_tactical_points(
    *,
    path_id: Optional[int],
    waypoints: List[Dict[str, Any]],
    plan: Optional[Dict[str, Any]],
    role: str,
    conceal_coordinate: Optional[Dict[str, Any]] = None,
) -> None:
    """Publish concealment/hold waypoint roles for SIM rendering.

    Display metadata only.  A failure here must never affect the emitted plan,
    so every error is swallowed.
    """

    if not waypoints:
        return
    try:
        from modules.mission_planning.pipelines.lah_tactical_point_log import (
            record_tactical_points,
        )

        if conceal_coordinate is not None:
            # A pop-up attack touches the concealment ground twice: before the
            # climb and again on the way down.  The climb and the shot share
            # that ground position but sit above it, so altitude is what
            # separates "in cover" from "exposed".
            conceal_altitude_m = _normalize_altitude_value(
                (_normalize_coordinate(conceal_coordinate) or {}).get("altitude")
            )
            conceal_ids = []
            for item in waypoints:
                item_coord = _extract_lah_waypoint_coordinate(item)
                if not _same_lah_ground_position(item_coord, conceal_coordinate):
                    continue
                item_altitude_m = _normalize_altitude_value(
                    (item_coord or {}).get("altitude")
                )
                if (
                    conceal_altitude_m is not None
                    and item_altitude_m is not None
                    and int(item_altitude_m) > int(conceal_altitude_m)
                ):
                    continue
                conceal_ids.append(_to_int(item.get("waypointID")))
            if not conceal_ids:
                conceal_ids = [_to_int(waypoints[-1].get("waypointID"))]
        else:
            conceal_ids = [_to_int(waypoints[-1].get("waypointID"))]
        hold_ids: List[int] = []
        for item in waypoints:
            hovering = item.get("hovering") if isinstance(item.get("hovering"), dict) else {}
            hover_seconds = _to_int(hovering.get("time")) or 0
            waypoint_id = _to_int(item.get("waypointID"))
            if hover_seconds > 0 and waypoint_id:
                hold_ids.append(int(waypoint_id))
        record_tactical_points(
            path_id,
            conceal_waypoint_ids=[value for value in conceal_ids if value],
            hold_waypoint_ids=hold_ids,
            role=role,
            plan=plan,
        )
    except Exception:
        return


def _same_lah_ground_position(
    left: Optional[Dict[str, Any]],
    right: Optional[Dict[str, Any]],
    *,
    tolerance_deg: float = 1e-6,
) -> bool:
    """Same latitude/longitude, altitude ignored.

    A pop-up climbs and sinks over one spot, so the concealment ground position
    is shared by waypoints at different altitudes.
    """

    a = _normalize_coordinate(left)
    b = _normalize_coordinate(right)
    if a is None or b is None:
        return False
    return (
        abs(float(a["latitude"]) - float(b["latitude"])) <= tolerance_deg
        and abs(float(a["longitude"]) - float(b["longitude"])) <= tolerance_deg
    )


def _same_lah_3d_coordinate(
    left: Optional[Dict[str, Any]],
    right: Optional[Dict[str, Any]],
) -> bool:
    if left is None or right is None:
        return False
    distance_m = _haversine_distance_m(left, right)
    left_alt = _to_float(left.get("altitude"))
    right_alt = _to_float(right.get("altitude"))
    return bool(
        distance_m is not None
        and float(distance_m) <= 0.5
        and left_alt is not None
        and right_alt is not None
        and abs(float(left_alt) - float(right_alt)) <= 0.5
    )


def _apply_leg_fuel(waypoints: List[Dict[str, Any]]) -> None:
    """Write the ICD per-leg ECF (litres) onto an ordered waypoint list."""

    try:
        from modules.common.ecf import apply_leg_fuel_inplace

        apply_leg_fuel_inplace(waypoints)
    except Exception:
        return


def _prepend_lah_tactical_waypoints(
    tactical_waypoints: List[Dict[str, Any]],
    mission_waypoints: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Join a certified hide prelude to the established attack approach."""

    prefix = [deepcopy(item) for item in tactical_waypoints if isinstance(item, dict)]
    suffix = [deepcopy(item) for item in mission_waypoints if isinstance(item, dict)]
    if not prefix:
        return suffix
    if not suffix:
        return prefix
    if _same_lah_3d_coordinate(
        _extract_lah_waypoint_coordinate(prefix[-1]),
        _extract_lah_waypoint_coordinate(suffix[0]),
    ) and not any((suffix[0].get("attack") or {}).get(key) for key in ("targetID", "weaponType")):
        suffix.pop(0)
    eta_offset = max(0, _to_int(prefix[-1].get("eta")) or 0)
    for waypoint in suffix:
        waypoint["eta"] = min(0xFFFFFFFF, eta_offset + max(0, _to_int(waypoint.get("eta")) or 0))
    combined = prefix + suffix
    for index, waypoint in enumerate(combined):
        waypoint["nextWaypointID"] = (
            int(combined[index + 1]["waypointID"]) if index + 1 < len(combined) else 0
        )
    _apply_leg_fuel(combined)
    return combined


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


def _lah_planned_vertical_rate_mps(altitude_delta_m: float) -> float:
    rate_attr = "climb_rate_mps" if float(altitude_delta_m) >= 0.0 else "descent_rate_mps"
    fallback_rate_mps = 8.9 if float(altitude_delta_m) >= 0.0 else 7.0
    try:
        rate_mps = float(getattr(DEFAULT_ENVELOPE, rate_attr, fallback_rate_mps))
    except Exception:
        rate_mps = float(fallback_rate_mps)
    if not math.isfinite(rate_mps) or rate_mps <= 0.0:
        rate_mps = float(fallback_rate_mps)
    return max(0.1, rate_mps * float(LAH_VERTICAL_RATE_USE_RATIO))


def _lah_profile_leg_time_s(
    left_sample: Dict[str, Any],
    right_sample: Dict[str, Any],
    *,
    horizontal_m: float,
    speed_mps: float,
) -> int:
    horizontal_time_s = max(0.0, float(horizontal_m)) / max(1e-6, float(speed_mps))
    altitude_delta_m = float(right_sample.get("altitude", 0.0) or 0.0) - float(
        left_sample.get("altitude", 0.0) or 0.0
    )
    vertical_time_s = abs(altitude_delta_m) / _lah_planned_vertical_rate_mps(altitude_delta_m)
    return max(0, int(math.ceil(max(horizontal_time_s, vertical_time_s) - 1e-9)))


def _build_lah_low_level_waypoint_route(
    *,
    template_wp: Dict[str, Any],
    route_coordinates: List[Dict[str, Any]],
    waypoint_id_provider: Callable[[], int],
    speed_mps: float,
    terminal_template: Optional[Dict[str, Any]] = None,
) -> List[Dict[str, Any]]:
    """Build an adaptive DEM-clearance route with a feasible vertical ETA.

    Terrain following alone climbs whatever ridge lies on the straight line,
    which puts the aircraft on high ground exactly where it is most visible.
    ``prefer_low_terrain`` first routes the leg horizontally through the low
    ground inside a corridor either side of it, so the aircraft goes around a
    hill rather than over it.  Leg endpoints are untouched, so the destination
    and every mission corner stay exactly where they were.
    """

    profile = build_lah_terrain_following_path(
        route_coordinates,
        prefer_low_terrain=bool(get_runtime_attack_int("lah_route_prefer_low_terrain", 1)),
        low_terrain_corridor_m=get_runtime_attack_float(
            "lah_route_low_terrain_corridor_m", LAH_LOW_TERRAIN_CORRIDOR_M
        ),
        low_terrain_min_leg_m=get_runtime_attack_float(
            "lah_route_low_terrain_min_leg_m", LAH_LOW_TERRAIN_MIN_LEG_M
        ),
    )
    if not profile:
        return []
    first_input = _normalize_coordinate(route_coordinates[0]) if route_coordinates else None
    if first_input is not None:
        try:
            reported_altitude_m = float(first_input.get("altitude"))
        except Exception:
            reported_altitude_m = float("nan")
        profile_altitude_m = float(profile[0].get("altitude", 0.0) or 0.0)
        if (
            math.isfinite(reported_altitude_m)
            and reported_altitude_m > 0.0
            and abs(reported_altitude_m - profile_altitude_m) >= 1.0
        ):
            # The packet starts at the reported/live altitude (ETA 0), then
            # explicitly climbs or descends at the same horizontal coordinate.
            profile = [
                {
                    **dict(profile[0]),
                    "altitude": int(round(reported_altitude_m)),
                    "cum_m": 0.0,
                }
            ] + profile
    waypoint_ids = [int(waypoint_id_provider()) for _ in profile]
    total_length_m = max(float(profile[-1].get("cum_m", 0.0) or 0.0), 1.0)
    waypoints: List[Dict[str, Any]] = []
    cumulative_eta_s = 0
    for index, (sample, waypoint_id) in enumerate(zip(profile, waypoint_ids)):
        next_id = waypoint_ids[index + 1] if index + 1 < len(waypoint_ids) else 0
        waypoint = _build_lah_waypoint_from_template(
            template_wp,
            waypoint_id,
            sample,
            next_id,
            mark_attack=False,
            target_id=None,
            speed_override_mps=speed_mps,
        )
        waypoint["isDone"] = False
        cumulative_m = float(sample.get("cum_m", 0.0) or 0.0)
        if index > 0:
            previous_sample = profile[index - 1]
            previous_cumulative_m = float(previous_sample.get("cum_m", 0.0) or 0.0)
            cumulative_eta_s = min(
                0xFFFFFFFF,
                cumulative_eta_s
                + _lah_profile_leg_time_s(
                    previous_sample,
                    sample,
                    horizontal_m=max(0.0, cumulative_m - previous_cumulative_m),
                    speed_mps=speed_mps,
                ),
            )
        waypoint["eta"] = int(cumulative_eta_s)
        waypoints.append(waypoint)

    if waypoints and isinstance(terminal_template, dict):
        terminal = waypoints[-1]
        for field in ("hovering", "loiter"):
            if isinstance(terminal_template.get(field), dict):
                terminal[field] = deepcopy(terminal_template[field])
        if terminal_template.get("_allowSingleLahWaypoint"):
            terminal["_allowSingleLahWaypoint"] = True
    _apply_leg_fuel(waypoints)
    return waypoints


def _build_lah_low_level_attack_waypoints(
    *,
    template_wp: Dict[str, Any],
    start_coord: Dict[str, Any],
    attack_coord: Dict[str, Any],
    attack_waypoint_id: int,
    waypoint_id_provider: Callable[[], int],
    target_id: Optional[int],
    weapon_type: Optional[int],
    speed_mps: float,
    route_coordinates: Optional[List[Dict[str, Any]]] = None,
    regain_cover_coord: Optional[Dict[str, Any]] = None,
) -> List[Dict[str, Any]]:
    """Pop up from cover, fire, sink back - or approach low when far away.

    ``regain_cover_coord`` appends the descent back behind terrain after the
    shot.  A pop-up attack is climb, fire, sink: without it the aircraft holds
    the firing altitude - where, line of sight being symmetric, it is visible -
    until some later replan happens to move it.
    """

    cover_norm = _normalize_coordinate(regain_cover_coord)
    attack_norm = _normalize_coordinate(attack_coord)
    # A firing point solved from the hide endpoint sits inside its immediate
    # neighbourhood, so the aircraft simply climbs towards it - diagonally when
    # the search stepped aside for a lower sightline.  Routing that through the
    # terrain follower would fly a separate low-level approach for a couple of
    # hundred metres, which is neither faster nor more covered, and it replaces
    # the certified hide altitude with the router's own DEM floor.
    direct_popup = bool(
        cover_norm is not None
        and attack_norm is not None
        and (
            _same_lah_ground_position(cover_norm, attack_norm)
            or bool((attack_coord or {}).get("attack_point_at_hide_endpoint"))
        )
    )
    if direct_popup:
        cover_start = _build_lah_waypoint_from_template(
            template_wp,
            int(waypoint_id_provider()),
            cover_norm,
            0,
            mark_attack=False,
            target_id=None,
            speed_override_mps=speed_mps,
        )
        cover_start["isDone"] = False
        cover_start["eta"] = 0
        cover_start["hovering"] = {"time": int(_attack_cover_hold_seconds())}
        approach = [cover_start]
    else:
        approach_coordinates = [
            coord
            for coord in (route_coordinates or [start_coord, attack_coord])
            if _normalize_coordinate(coord) is not None
        ]
        if not approach_coordinates:
            approach_coordinates = [start_coord, attack_coord]
        elif len(approach_coordinates) == 1:
            approach_coordinates.append(attack_coord)
        else:
            approach_coordinates[-1] = attack_coord
        approach = _build_lah_low_level_waypoint_route(
            template_wp=template_wp,
            route_coordinates=approach_coordinates,
            waypoint_id_provider=waypoint_id_provider,
            speed_mps=speed_mps,
        )
    attack_wp = _build_lah_waypoint_from_template(
        template_wp,
        int(attack_waypoint_id),
        attack_coord,
        0,
        mark_attack=True,
        target_id=target_id,
        weapon_type=weapon_type,
        speed_override_mps=speed_mps,
    )
    attack_wp["isDone"] = False
    # LAH 0304 ETA values are cumulative uint32 seconds.  The attack waypoint is
    # appended after the terrain-following approach, so resetting it to zero
    # breaks the flight-plan timeline.  Account for the final vertical leg and
    # keep the terminal ETA cumulative.
    attack_eta_s = 0
    if approach:
        approach_eta_s = max(0, _to_int(approach[-1].get("eta")) or 0)
        approach_coord = _extract_lah_waypoint_coordinate(approach[-1])
        attack_coord_norm = _normalize_coordinate(attack_coord)
        vertical_duration_s = 0
        if approach_coord is not None and attack_coord_norm is not None:
            altitude_delta_m = float(attack_coord_norm.get("altitude", 0.0) or 0.0) - float(
                approach_coord.get("altitude", 0.0) or 0.0
            )
            vertical_rate_mps = _lah_planned_vertical_rate_mps(altitude_delta_m)
            vertical_duration_s = int(
                math.ceil(abs(altitude_delta_m) / vertical_rate_mps - 1e-9)
            )
            # A diagonal pop-up also covers ground.  The climb and the traverse
            # happen together, so the leg takes whichever is slower; charging
            # only the climb would understate the exposure window.
            horizontal_m = _haversine_distance_m(approach_coord, attack_coord_norm)
            if horizontal_m is not None and float(horizontal_m) > 0.0:
                cruise_mps = max(1.0, float(speed_mps))
                horizontal_duration_s = int(
                    math.ceil(float(horizontal_m) / cruise_mps - 1e-9)
                )
                vertical_duration_s = max(vertical_duration_s, horizontal_duration_s)
        attack_eta_s = min(0xFFFFFFFF, approach_eta_s + max(0, vertical_duration_s))
    attack_wp["eta"] = int(attack_eta_s)
    attack_wp["ecf"] = 1.0
    # When the solver certifies a firing altitude at the hide point itself
    # (climb 0, lateral 0) the pop-up degenerates into a leg of zero length: the
    # aircraft is told to fly to where it already is, never registers the
    # arrival, and sits at cover without shooting.  Collapse the pair onto the
    # firing waypoint instead, carrying the cover dwell with it, so the aircraft
    # holds and then fires from the same point.
    if approach and _same_lah_3d_coordinate(
        _extract_lah_waypoint_coordinate(approach[-1]),
        _normalize_coordinate(attack_coord),
    ):
        merged_hold = (approach[-1].get("hovering") or {}).get("time")
        if _to_int(merged_hold):
            attack_wp["hovering"] = {"time": int(merged_hold)}
        attack_wp["eta"] = int(max(0, _to_int(approach[-1].get("eta")) or 0))
        approach.pop()
        if approach:
            approach[-1]["nextWaypointID"] = int(attack_waypoint_id)
    elif approach:
        approach[-1]["nextWaypointID"] = int(attack_waypoint_id)

    cover_wp = _build_lah_regain_cover_waypoint(
        template_wp=template_wp,
        attack_wp=attack_wp,
        attack_coord=attack_coord,
        regain_cover_coord=regain_cover_coord,
        waypoint_id_provider=waypoint_id_provider,
        speed_mps=speed_mps,
        attack_eta_s=int(attack_eta_s),
    )
    if cover_wp is None:
        legacy = approach + [attack_wp]
        _apply_leg_fuel(legacy)
        return legacy
    # ICD 0304 ecf is per-leg fuel, not a terminal marker; the only terminal
    # contract is nextWaypointID, which now belongs to the descent.
    attack_wp["nextWaypointID"] = int(cover_wp["waypointID"])
    combined = approach + [attack_wp, cover_wp]
    _apply_leg_fuel(combined)
    return combined


def _build_lah_regain_cover_waypoint(
    *,
    template_wp: Dict[str, Any],
    attack_wp: Dict[str, Any],
    attack_coord: Dict[str, Any],
    regain_cover_coord: Optional[Dict[str, Any]],
    waypoint_id_provider: Callable[[], int],
    speed_mps: float,
    attack_eta_s: int,
) -> Optional[Dict[str, Any]]:
    """Return to the exact certified cover coordinate after the shot.

    Returns ``None`` when there is nothing to descend to, so the attack path
    keeps its previous shape rather than emitting a degenerate waypoint.
    """

    if not bool(get_runtime_attack_int("attack_regain_cover_enabled", 1)):
        return None
    cover = _normalize_coordinate(regain_cover_coord)
    attack_norm = _normalize_coordinate(attack_coord)
    if cover is None or attack_norm is None:
        return None
    cover_altitude_m = _normalize_altitude_value(cover.get("altitude"))
    attack_altitude_m = _normalize_altitude_value(attack_norm.get("altitude"))
    if cover_altitude_m is None or attack_altitude_m is None:
        return None
    if int(cover_altitude_m) >= int(attack_altitude_m):
        # No pop-up happened, so there is nothing to sink back from.
        return None

    # The concealment certificate belongs to this exact XY and altitude.  A
    # lateral popup must come back to it; copying only the cover altitude onto
    # the firing XY creates an uncertified (and potentially below-DEM) point.
    descend_coord = {
        "latitude": float(cover["latitude"]),
        "longitude": float(cover["longitude"]),
        "altitude": int(cover_altitude_m),
    }
    cover_wp = _build_lah_waypoint_from_template(
        template_wp,
        int(waypoint_id_provider()),
        descend_coord,
        0,
        mark_attack=False,
        target_id=None,
        speed_override_mps=speed_mps,
    )
    cover_wp["isDone"] = False
    descent_m = float(attack_altitude_m) - float(cover_altitude_m)
    vertical_rate_mps = _lah_planned_vertical_rate_mps(-descent_m)
    vertical_s = max(0.0, descent_m) / vertical_rate_mps
    horizontal_m = _haversine_distance_m(attack_norm, cover) or 0.0
    horizontal_s = float(horizontal_m) / max(0.1, float(speed_mps))
    descent_s = int(math.ceil(max(vertical_s, horizontal_s) - 1e-9))
    cover_wp["eta"] = int(min(0xFFFFFFFF, int(attack_eta_s) + max(1, descent_s)))
    cover_wp["ecf"] = 1.0
    cover_wp["nextWaypointID"] = 0
    # Dwell belongs in hovering.time, never in eta: eta is arrival time.
    cover_wp["hovering"] = {"time": int(_attack_cover_hold_seconds())}
    return cover_wp


def _attack_cover_hold_seconds() -> int:
    """Seconds the manned aircraft sits in cover either side of the pop-up."""

    return max(0, get_runtime_attack_int("attack_cover_hold_seconds", 10))


def _rebuild_lah_low_level_resume_waypoints(
    *,
    attack_coord: Dict[str, Any],
    resume_waypoints: List[Dict[str, Any]],
    template_wp: Dict[str, Any],
    waypoint_id_provider: Callable[[], int],
) -> List[Dict[str, Any]]:
    """Descend over the attack point, then return along a DEM-following route."""

    resume_coordinates = [
        coordinate
        for coordinate in (
            _extract_lah_waypoint_coordinate(waypoint)
            for waypoint in resume_waypoints or []
        )
        if coordinate is not None
    ]
    if not resume_coordinates:
        return []
    terminal_template = resume_waypoints[-1] if resume_waypoints else None
    return _build_lah_low_level_waypoint_route(
        template_wp=template_wp,
        route_coordinates=[attack_coord] + resume_coordinates,
        waypoint_id_provider=waypoint_id_provider,
        speed_mps=_to_float(template_wp.get("speed")) or 40.0,
        terminal_template=terminal_template,
    )


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
        # ETA is arrival time within the flight plan; hold duration remains in
        # the hovering property and a single waypoint therefore starts at 0.
        anchor_wp["eta"] = 0
        anchor_wp["ecf"] = 1.0
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


def _predict_lah_attack_route_start(
    current_coord: Optional[Dict[str, Any]],
    state: Dict[str, Any],
) -> Optional[Dict[str, Any]]:
    """Project the first attack WP beyond the plan-delivery latency window."""

    current = _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None
    if current is None:
        return None
    velocity = (state or {}).get("velocity") if isinstance(state, dict) else {}
    heading = _to_float((state or {}).get("heading"))
    if heading is None and isinstance(velocity, dict):
        heading = _to_float(velocity.get("heading"))
    speed_mps = _to_float((state or {}).get("speed"))
    if speed_mps is None and isinstance(velocity, dict):
        speed_mps = _to_float(velocity.get("speed"))
    if heading is None or speed_mps is None or speed_mps <= 0.0:
        return current

    lookahead_s = max(
        _LAH_ATTACK_ROUTE_MIN_LOOKAHEAD_S,
        get_runtime_attack_float(
            "lah_attack_route_start_lookahead_s",
            _LAH_ATTACK_ROUTE_MIN_LOOKAHEAD_S,
        ),
    )
    projected = _project_coordinate(current, float(heading), float(speed_mps) * float(lookahead_s))
    if projected is None:
        return current
    projected["altitude"] = current.get("altitude")
    return _normalize_coordinate(projected) or current


def _build_lah_mission_constrained_attack_route(
    *,
    start_coord: Dict[str, Any],
    attack_coord: Dict[str, Any],
    source_plan_id: Optional[int],
    operation_zones: Optional[List[Dict[str, Any]]] = None,
) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """Route through the union of input LINE widths and AREA polygons.

    If an aircraft is already outside the union, only the shortest recovery leg
    to the nearest mission boundary is allowed outside it.  The remainder is a
    visibility-graph shortest path covered by the mission geometry.
    """

    start = _normalize_coordinate(start_coord)
    attack = _normalize_coordinate(attack_coord)
    fallback = [coord for coord in (start, attack) if coord is not None]
    source_plan = _to_int(source_plan_id)
    metadata: Dict[str, Any] = {
        "sourcePlanID": source_plan,
        "zoneCount": 0,
        "routePointCount": len(fallback),
        "constrained": False,
        "reason": "coordinate_missing" if len(fallback) < 2 else "mission_zone_missing",
    }
    if start is None or attack is None:
        return fallback, metadata

    zones = (
        [dict(zone) for zone in operation_zones if isinstance(zone, dict)]
        if operation_zones is not None
        else _load_attack_operation_zones(source_plan)
    )
    metadata["zoneCount"] = len(zones)
    if not zones:
        return [start, attack], metadata

    try:
        import heapq

        from shapely.geometry import LineString, Point, Polygon
        from shapely.ops import nearest_points, unary_union
    except Exception as exc:
        metadata["reason"] = f"geometry_dependency_unavailable:{type(exc).__name__}"
        return [start, attack], metadata

    origin_lat = (float(start["latitude"]) + float(attack["latitude"])) * 0.5
    origin_lon = (float(start["longitude"]) + float(attack["longitude"])) * 0.5
    lat_scale = 111_132.0
    lon_scale = 111_320.0 * max(math.cos(math.radians(origin_lat)), 0.01)

    def _to_xy(coord: Dict[str, Any]) -> Tuple[float, float]:
        return (
            (float(coord["longitude"]) - origin_lon) * lon_scale,
            (float(coord["latitude"]) - origin_lat) * lat_scale,
        )

    route_altitude = _normalize_altitude_value(start.get("altitude"))

    def _to_coord(point: Any) -> Dict[str, Any]:
        coord: Dict[str, Any] = {
            "latitude": origin_lat + (float(point.y) / lat_scale),
            "longitude": origin_lon + (float(point.x) / lon_scale),
        }
        if route_altitude is not None:
            coord["altitude"] = int(route_altitude)
        return coord

    allowed_geometries: List[Any] = []
    hole_geometries: List[Any] = []
    try:
        for zone in zones:
            zone_type = str(zone.get("zoneType") or "line").strip().lower()
            min_len = 3 if zone_type == "area" else 2
            coords = _normalize_line_coord_list(zone.get("coordinateList"), min_len=min_len)
            if len(coords) < min_len:
                continue
            points_xy = [_to_xy(coord) for coord in coords]
            if zone_type == "area":
                geometry = Polygon(points_xy).buffer(0)
                if bool(zone.get("isHole")):
                    hole_geometries.append(geometry)
                else:
                    allowed_geometries.append(geometry)
                continue
            width_m = _to_float(zone.get("widthM") or zone.get("width"))
            if width_m is None or float(width_m) <= 0.0:
                continue
            allowed_geometries.append(
                LineString(points_xy).buffer(
                    float(width_m) * 0.5,
                    cap_style=2,
                    join_style=2,
                )
            )
        if not allowed_geometries:
            metadata["reason"] = "mission_geometry_empty"
            return [start, attack], metadata
        allowed = unary_union(allowed_geometries).buffer(0)
        if hole_geometries:
            allowed = allowed.difference(unary_union(hole_geometries)).buffer(0)
    except Exception as exc:
        metadata["reason"] = f"mission_geometry_invalid:{type(exc).__name__}"
        return [start, attack], metadata

    polygons: List[Any] = []
    if getattr(allowed, "geom_type", "") == "Polygon":
        polygons = [allowed]
    elif getattr(allowed, "geom_type", "") == "MultiPolygon":
        polygons = [geom for geom in allowed.geoms if not geom.is_empty]
    if not polygons:
        metadata["reason"] = "mission_geometry_non_polygonal"
        return [start, attack], metadata

    start_point = Point(_to_xy(start))
    attack_point = Point(_to_xy(attack))
    component = min(polygons, key=lambda polygon: float(polygon.distance(attack_point)))
    tolerance_m = max(
        0.05,
        get_runtime_attack_float("lah_attack_mission_route_tolerance_m", 0.5),
    )
    covered_component = component.buffer(float(tolerance_m))
    start_inside = bool(covered_component.covers(start_point))
    attack_inside = bool(covered_component.covers(attack_point))
    entry_point = start_point if start_inside else nearest_points(component, start_point)[0]
    exit_point = attack_point if attack_inside else nearest_points(component, attack_point)[0]

    internal_points: List[Any] = []
    direct_internal = LineString([entry_point, exit_point])
    if covered_component.covers(direct_internal):
        internal_points = [entry_point, exit_point]
    else:
        simplify_m = max(
            0.0,
            get_runtime_attack_float("lah_attack_mission_route_simplify_m", 2.0),
        )
        simplified = component.simplify(float(simplify_m), preserve_topology=True)
        for extra_tolerance in (5.0, 10.0, 20.0, 40.0):
            vertex_count = len(list(simplified.exterior.coords)) + sum(
                len(list(ring.coords)) for ring in simplified.interiors
            )
            if vertex_count <= 160:
                break
            simplified = component.simplify(float(extra_tolerance), preserve_topology=True)

        nodes: List[Any] = [entry_point, exit_point]
        nodes.extend(Point(value) for value in list(simplified.exterior.coords)[:-1])
        for ring in simplified.interiors:
            nodes.extend(Point(value) for value in list(ring.coords)[:-1])

        adjacency: List[List[Tuple[int, float]]] = [[] for _ in nodes]
        for left_index in range(len(nodes)):
            for right_index in range(left_index + 1, len(nodes)):
                segment = LineString([nodes[left_index], nodes[right_index]])
                if not covered_component.covers(segment):
                    continue
                distance_m = float(segment.length)
                adjacency[left_index].append((right_index, distance_m))
                adjacency[right_index].append((left_index, distance_m))

        distances = [math.inf] * len(nodes)
        previous: List[Optional[int]] = [None] * len(nodes)
        distances[0] = 0.0
        queue: List[Tuple[float, int]] = [(0.0, 0)]
        while queue:
            current_distance, current_index = heapq.heappop(queue)
            if current_distance != distances[current_index]:
                continue
            if current_index == 1:
                break
            for neighbor, edge_distance in adjacency[current_index]:
                candidate_distance = current_distance + edge_distance
                if candidate_distance >= distances[neighbor]:
                    continue
                distances[neighbor] = candidate_distance
                previous[neighbor] = current_index
                heapq.heappush(queue, (candidate_distance, neighbor))

        if math.isfinite(distances[1]):
            indices: List[int] = []
            cursor: Optional[int] = 1
            while cursor is not None:
                indices.append(cursor)
                cursor = previous[cursor]
            internal_points = [nodes[index] for index in reversed(indices)]
        else:
            metadata["reason"] = "mission_internal_route_unreachable"
            return [start, attack], metadata

    route: List[Dict[str, Any]] = [dict(start)]

    def _append_point(point: Any) -> None:
        coord = _to_coord(point)
        previous_coord = route[-1]
        if (_haversine_distance_m(previous_coord, coord) or 0.0) > 1.0:
            route.append(coord)

    if not start_inside:
        _append_point(entry_point)
    for point in internal_points[1:]:
        _append_point(point)
    if not attack_inside:
        _append_point(exit_point)
    if (_haversine_distance_m(route[-1], attack) or 0.0) <= 1.0:
        route[-1] = dict(attack)
    else:
        route.append(dict(attack))

    metadata.update(
        {
            "routePointCount": len(route),
            "constrained": True,
            "fullyInsideAfterStart": bool(attack_inside),
            "startInside": bool(start_inside),
            "attackInside": bool(attack_inside),
            "recoveryDistanceM": round(float(start_point.distance(entry_point)), 1),
            "exitDistanceM": round(float(exit_point.distance(attack_point)), 1),
            "componentCount": len(polygons),
            "reason": "mission_internal_route",
        }
    )
    return route, metadata


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

    suffix = source_waypoints[keep_start_idx:]
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


def _input_mission_payload_is_area(mission: Dict[str, Any]) -> bool:
    if not isinstance(mission, dict):
        return False
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    return bool(area_list)


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


def _source_input_mission_is_area(
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
            return _input_mission_payload_is_area(mission)
    return False


def _source_input_mission_is_locked_type2_branch(
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
    try:
        from modules.mission_planning.runtime.state import branch_ownership as store

        return store.is_locked_type2_branch_mission(input_data, int(input_id))
    except Exception:
        return False


def _source_type2_self_reliance_phase(
    *,
    source_plan_id: Optional[int],
    input_mission_id: Optional[int],
) -> Optional[str]:
    """Resolve a fresh, exact Type-2 branch phase for one source-plan input.

    The persisted owner map alone is deliberately insufficient here: an old or
    malformed map must never broaden suffix handling into ordinary target-area
    missions.  Both the current input package shape and the locked branch guard
    therefore have to agree.
    """

    source_id = _to_int(source_plan_id)
    input_id = _to_int(input_mission_id)
    if source_id is None or input_id is None:
        return None
    input_data = _load_input_plan_for_source_plan(int(source_id))
    if not isinstance(input_data, dict):
        return None
    if not _source_input_mission_is_locked_type2_branch(
        source_plan_id=int(source_id),
        input_mission_id=int(input_id),
    ):
        return None
    return resolve_type2_self_reliance_phase(input_data, int(input_id))


def _attack_tracking_collab_remaining_policy(
    *,
    source_plan_id: Optional[int],
    input_mission_id: Optional[int],
) -> str:
    """Choose ownership handling for the input interrupted by tracking.

    Only the locked Type-2 self-reliance suffix has immutable per-aircraft
    ownership.  Ordinary collaborative LINE/AREA missions must retain their
    original behavior: merge the measured remaining work and redistribute it
    among the currently available UAVs.  The tracking UAV is converted to a
    release/transit resume by the collaborative attack path, so its old suffix
    is not kept as duplicate ownership.
    """

    if _source_input_mission_is_locked_type2_branch(
        source_plan_id=source_plan_id,
        input_mission_id=input_mission_id,
    ):
        return "preserve"

    if _source_input_mission_is_area(
        source_plan_id=source_plan_id,
        input_mission_id=input_mission_id,
    ):
        return "redivide"
    if _source_input_mission_is_line(
        source_plan_id=source_plan_id,
        input_mission_id=input_mission_id,
    ):
        return "redivide"
    return "ignore"


def _source_input_mission_is_line_or_area(
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
            return _input_mission_payload_is_line(mission) or _input_mission_payload_is_area(mission)
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


def _resolve_sweep_progress_payload(
    sweep_progress: Any,
) -> Dict[int, Dict[str, Any]] | None:
    if callable(sweep_progress):
        loaded = sweep_progress()
    else:
        loaded = sweep_progress
    return loaded if isinstance(loaded, dict) else None


def _attack_sweep_progress_entry_state(entry: Any) -> Tuple[bool, bool]:
    """Return ``(authoritative, has_remaining)`` with conflict-safe semantics."""

    if not isinstance(entry, dict):
        return False, False
    signals: List[bool] = []
    total = _to_int(
        entry.get("sweep_point_count")
        if "sweep_point_count" in entry
        else entry.get("sweepPointCount")
    )
    progress = _to_int(
        entry.get("progress_points")
        if "progress_points" in entry
        else entry.get("progressPoints")
    )
    if total is not None and total > 0 and progress is not None:
        signals.append(int(progress) < int(total))
    percent = _to_float(
        entry.get("progress_percent")
        if "progress_percent" in entry
        else entry.get("progressPercent")
    )
    if percent is not None:
        signals.append(float(percent) < 100.0)
    remaining_seconds = _to_float(
        entry.get("remaining_seconds")
        if "remaining_seconds" in entry
        else entry.get("remainingSeconds")
    )
    if remaining_seconds is not None:
        signals.append(float(remaining_seconds) > 0.0)
    return bool(signals), any(signals)


def _type2_branch_line_completion_confirmed(
    *,
    source_plan_id: Optional[int],
    input_mission_id: Optional[int],
    path_id: Optional[int],
    sweep_progress: Any,
) -> bool:
    phase = _source_type2_self_reliance_phase(
        source_plan_id=source_plan_id,
        input_mission_id=input_mission_id,
    )
    if phase not in {
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
        TYPE2_SELF_RELIANCE_RETURN_LINE,
    }:
        return False
    resolved_path_id = _to_int(path_id)
    payload = _resolve_sweep_progress_payload(sweep_progress)
    progress_entry = (
        payload.get(int(resolved_path_id))
        or payload.get(str(int(resolved_path_id)))
        if isinstance(payload, dict) and resolved_path_id is not None
        else None
    )
    authoritative, has_remaining = _attack_sweep_progress_entry_state(progress_entry)
    return bool(authoritative and not has_remaining)


def _split_done_resume_path(
    source_fp_data: Dict[str, Any],
    *,
    artifacts: Any,
    sweep_progress: Any,
    emit: Callable[[str], None],
    force_nonempty_resume: bool = False,
    append_replan_anchor: bool = False,
    replan_coordinate: Optional[Dict[str, Any]] = None,
    resume_trim_anchor_coord: Optional[Dict[str, Any]] = None,
    waypoint_id_provider: Optional[Callable[[], int]] = None,
    timing: Optional[Dict[str, Any]] = None,
    synchronize_line_search_to_geometry: bool = False,
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
    if artifacts.path_id is not None:
        sweep_progress_payload = _resolve_sweep_progress_payload(sweep_progress)
        if sweep_progress_payload:
            progress_entry = sweep_progress_payload.get(int(artifacts.path_id))
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
        resume_speed_scale = get_runtime_attack_float("resume_search_speed_scale", 1.3)
        geometry_speed_scale = float(search_speed_weight)
        if synchronize_line_search_to_geometry:
            # Type-2 independent LINE routes may legitimately require more than
            # the generic 16x camera-speed cap.  Recompute the final (including
            # the replan margin) value from the trimmed geometry in one pass so
            # repeated attack replans cannot accumulate another 1.3x each time.
            geometry_speed_scale *= float(resume_speed_scale)
        recompute_started = time.perf_counter()
        recomputed = recompute_line_search_speed_from_geometry(
            resume_waypoints,
            first_reference_coord=resume_reference_coord,
            speed_scale=float(geometry_speed_scale),
            only_increase=not bool(synchronize_line_search_to_geometry),
            multiplier_cap_enabled=not bool(synchronize_line_search_to_geometry),
        )
        _record_split_stage(
            "recompute_line_search_speed_from_geometry",
            recompute_started,
            weight=float(geometry_speed_scale),
            recomputedWaypoints=recomputed,
            synchronized=bool(synchronize_line_search_to_geometry),
        )
        if recomputed > 0:
            emit(
                "[ATTACK][UAV] Resume searchSpeed geometry recomputed "
                f"(weight={float(geometry_speed_scale):.2f}, waypoints={recomputed}, "
                f"synchronized={bool(synchronize_line_search_to_geometry)})."
            )
        scale_started = time.perf_counter()
        scaled = (
            0
            if synchronize_line_search_to_geometry
            else scale_line_search_speed(resume_waypoints, resume_speed_scale)
        )
        _record_split_stage(
            "scale_line_search_speed",
            scale_started,
            factor=float(resume_speed_scale),
            scaledWaypoints=scaled,
            skipped=bool(synchronize_line_search_to_geometry),
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
    sweep_progress: Any = None,
    done_input_ids: Optional[set[int]] = None,
    collaborative_resume: Optional[CollaborativeResumeReplanResult] = None,
    id_reservation: AttackIdReservation | None = None,
    defer_write: bool = False,
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
    mission_resume.pop("executionBlockedUntilNextCollab", None)
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
            synchronize_line_search_to_geometry=(
                _source_type2_self_reliance_phase(
                    source_plan_id=_to_int(getattr(artifacts, "source_plan_id", None)),
                    input_mission_id=int(input_mission_id),
                )
                in {
                    TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
                    TYPE2_SELF_RELIANCE_RETURN_LINE,
                }
            ),
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

    type2_branch_line_complete = _type2_branch_line_completion_confirmed(
        source_plan_id=_to_int(getattr(artifacts, "source_plan_id", None)),
        input_mission_id=int(input_mission_id),
        path_id=_to_int(getattr(artifacts, "path_id", None)),
        sweep_progress=sweep_progress,
    )
    completion_boundary_hold = False
    if not resume_waypoints and not type2_branch_line_complete:
        hold_waypoint = _build_uav_attack_completion_hold_waypoint(
            fp_data,
            waypoint_id=id_reservation.next_waypoint(),
            fallback_coordinate=agent_coord,
        )
        if hold_waypoint is not None:
            resume_waypoints = [hold_waypoint]
            resume_fp_data["waypointList"] = resume_waypoints
            resume_fp_data["attackCompletionBoundaryHold"] = True
            mission_resume["attackCompletionBoundaryHold"] = True
            mission_info = mission_resume.get("individualMissionInfo")
            mission_info = deepcopy(mission_info) if isinstance(mission_info, dict) else {}
            mission_info["individualMissionType"] = 7
            mission_info["patternType"] = 10
            mission_info["autoZoomIn"] = False
            mission_info["coordinateList"] = [deepcopy(hold_waypoint["coordinate"])]
            mission_info["lineList"] = []
            mission_info["areaList"] = []
            mission_info["targetID"] = None
            mission_info["SPEED"] = float(_UAV_ATTACK_COMPLETION_HOLD_SPEED_MPS)
            mission_resume["individualMissionInfo"] = mission_info
            completion_boundary_hold = True
            emit(
                "[ATTACK][UAV] Current collaborative sweep has no remaining geometry; "
                "inserted final capture-point loiter; future input missions will be retained "
                "but execution-blocked until the next collaborative handoff "
                f"(aircraft={descriptor['aircraft_id']}, inputMissionID={input_mission_id})."
            )
    elif not resume_waypoints:
        emit(
            "[ATTACK][TYPE2-LINE-SUFFIX] Completed branch LINE removed; "
            "following AREA/LINE missions remain executable "
            f"(aircraft={descriptor['aircraft_id']}, inputMissionID={input_mission_id})."
        )

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
        follow_up_sources = [
            mission
            for mission in source_mission_list[target_index + 1 :]
            if isinstance(mission, dict)
        ]
        if completion_boundary_hold:
            follow_up_sources = _attack_completion_boundary_follow_up_sources(
                follow_up_sources,
                current_input_id=int(input_mission_id),
            )
        follow_up_artifacts = _collect_attack_follow_up_replan_artifacts(
            missions=follow_up_sources,
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
        blocked_follow_up_count = 0
        if completion_boundary_hold:
            blocked_follow_up_count = _mark_attack_followups_execution_blocked(
                follow_up_missions,
                current_input_id=int(input_mission_id),
            )
            emit(
                "[ATTACK][UAV] Retained future mission artifacts at completed collaboration "
                f"boundary (aircraft={descriptor['aircraft_id']}, "
                f"blockedFollowUps={int(blocked_follow_up_count)})."
            )
        _record_builder_stage(
            "clone_followups",
            clone_started,
            followUpMissionCount=len(follow_up_missions),
            followUpPathCount=len(follow_up_paths),
            preservedFollowUpCount=follow_up_stats.get("preservedCount"),
            clonedFollowUpCount=follow_up_stats.get("clonedCount"),
            skippedFollowUpCount=follow_up_stats.get("skippedCount"),
            blockedFollowUpCount=int(blocked_follow_up_count),
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
    write_results, deferred_write_entries = _write_or_defer_attack_json_entries(
        write_entries,
        defer_write=bool(defer_write),
    )
    _record_builder_stage(
        "write_json",
        write_started,
        fileCount=len(write_results),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
        deferred=bool(defer_write),
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
    result["completionBoundaryHold"] = bool(completion_boundary_hold)
    if deferred_write_entries:
        result["_deferredWriteEntries"] = deferred_write_entries
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
    sweep_progress: Any = None,
    done_input_ids: Optional[set[int]] = None,
    id_reservation: AttackIdReservation | None = None,
    defer_write: bool = False,
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
    mission_resume.pop("executionBlockedUntilNextCollab", None)

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
        synchronize_line_search_to_geometry=(
            _source_type2_self_reliance_phase(
                source_plan_id=_to_int(getattr(artifacts, "source_plan_id", None)),
                input_mission_id=int(input_mission_id),
            )
            in {
                TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
                TYPE2_SELF_RELIANCE_RETURN_LINE,
            }
        ),
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

    type2_branch_line_complete = _type2_branch_line_completion_confirmed(
        source_plan_id=_to_int(getattr(artifacts, "source_plan_id", None)),
        input_mission_id=int(input_mission_id),
        path_id=_to_int(getattr(artifacts, "path_id", None)),
        sweep_progress=sweep_progress,
    )
    completion_boundary_hold = False
    if not resume_waypoints and not type2_branch_line_complete:
        hold_waypoint = _build_uav_attack_completion_hold_waypoint(
            fp_data,
            waypoint_id=id_reservation.next_waypoint(),
            fallback_coordinate=replan_coord,
        )
        if hold_waypoint is not None:
            resume_waypoints = [hold_waypoint]
            resume_fp_data["waypointList"] = resume_waypoints
            resume_fp_data["attackCompletionBoundaryHold"] = True
            mission_resume["attackCompletionBoundaryHold"] = True
            mission_info = mission_resume.get("individualMissionInfo")
            mission_info = deepcopy(mission_info) if isinstance(mission_info, dict) else {}
            mission_info["individualMissionType"] = 7
            mission_info["patternType"] = 10
            mission_info["autoZoomIn"] = False
            mission_info["coordinateList"] = [deepcopy(hold_waypoint["coordinate"])]
            mission_info["lineList"] = []
            mission_info["areaList"] = []
            mission_info["targetID"] = None
            mission_info["SPEED"] = float(_UAV_ATTACK_COMPLETION_HOLD_SPEED_MPS)
            mission_resume["individualMissionInfo"] = mission_info
            completion_boundary_hold = True
            emit(
                "[ATTACK][UAV] Current collaborative sweep has no remaining geometry; "
                "inserted final capture-point loiter; future input missions will be retained "
                "but execution-blocked until the next collaborative handoff "
                f"(aircraft={descriptor['aircraft_id']}, inputMissionID={input_mission_id})."
            )
    elif not resume_waypoints:
        emit(
            "[ATTACK][TYPE2-LINE-SUFFIX] Completed branch LINE removed; "
            "following AREA/LINE missions remain executable "
            f"(aircraft={descriptor['aircraft_id']}, inputMissionID={input_mission_id})."
        )

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
        and 0 <= target_index < len(source_mission_list)
    ):
        follow_up_sources = [
            mission
            for mission in source_mission_list[target_index + 1 :]
            if isinstance(mission, dict)
        ]
        if completion_boundary_hold:
            follow_up_sources = _attack_completion_boundary_follow_up_sources(
                follow_up_sources,
                current_input_id=int(input_mission_id),
            )
        follow_up_artifacts = _collect_attack_follow_up_replan_artifacts(
            missions=follow_up_sources,
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
        if completion_boundary_hold:
            blocked_follow_up_count = _mark_attack_followups_execution_blocked(
                follow_up_missions,
                current_input_id=int(input_mission_id),
            )
            emit(
                "[ATTACK][UAV] Retained future mission artifacts at completed collaboration "
                f"boundary (aircraft={descriptor['aircraft_id']}, "
                f"blockedFollowUps={int(blocked_follow_up_count)})."
            )

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
    write_results, deferred_write_entries = _write_or_defer_attack_json_entries(
        write_entries,
        defer_write=bool(defer_write),
    )
    write_timing = {
        "elapsedMs": _elapsed_ms_detail(write_started),
        "fileCount": len(write_results),
        "writtenCount": sum(1 for row in write_results if row.get("written")),
        "skippedCount": sum(1 for row in write_results if row.get("skipped")),
        "deferred": bool(defer_write),
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
        "completionBoundaryHold": bool(completion_boundary_hold),
        "timingMs": {"split_done_resume": split_timing_summary, "write_json": write_timing},
    }
    if deferred_write_entries:
        result["_deferredWriteEntries"] = deferred_write_entries
    if resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
    return result


def _build_lah_tactical_abort_hold_package(
    *,
    descriptor: Dict[str, Any],
    tactical_plan: Optional[Dict[str, Any]],
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
    done_input_ids: Optional[set[int]],
    id_reservation: AttackIdReservation,
    defer_write: bool,
) -> Optional[Dict[str, Any]]:
    """Replace an uncertified attack with a live-position hold transaction."""

    abort_descriptor = dict(descriptor)
    abort_descriptor["mode"] = "LAH_TACTICAL_ABORT"
    abort_descriptor["label"] = f"{descriptor.get('label') or 'manned'}_tactical_abort"
    if isinstance(tactical_plan, dict) and tactical_plan.get("applied"):
        # An attack can fail because no firing altitude fits the envelope even
        # though the ingress-to-cover route is valid.  Preserve that certified
        # endpoint so failure means "hide and wait", not "stay exposed".
        abort_descriptor["_certified_tactical_plan"] = deepcopy(tactical_plan)
    result = _build_lah_hold_resume_package(
        descriptor=abort_descriptor,
        new_imp_id=new_imp_id,
        imp_data=imp_data,
        fp_data=fp_data,
        target_mission=target_mission,
        target_index=target_index,
        ctx=ctx,
        state=state,
        aircraft_id=aircraft_id,
        artifacts=artifacts,
        emit=emit,
        now_ms=now_ms,
        done_input_ids=done_input_ids,
        id_reservation=id_reservation,
        defer_write=defer_write,
    )
    if isinstance(result, dict):
        result["tacticalPlan"] = _json_safe(tactical_plan)
        deferred_target_ids = {
            int(target_id)
            for target in descriptor.get("attack_targets") or []
            if isinstance(target, dict)
            for target_id in [
                _to_int(target.get("target_id") or target.get("targetID"))
            ]
            if target_id is not None and int(target_id) > 0
        }
        descriptor_target_id = _to_int(
            descriptor.get("target_id") or descriptor.get("targetID")
        )
        if descriptor_target_id is not None and int(descriptor_target_id) > 0:
            deferred_target_ids.add(int(descriptor_target_id))
        result["deferredAttackTargetIDs"] = sorted(deferred_target_ids)
    return result


def _enemy_set_fingerprint(enemy_rows: Any) -> Tuple[Tuple[int, int, int], ...]:
    """Identity of a contact set, for detecting that it changed.

    Rounded to ~1 m so a re-reported contact with jittered coordinates does not
    read as a new enemy, while a genuinely new or moved contact does.
    """

    out: List[Tuple[int, int, int]] = []
    for raw in enemy_rows if isinstance(enemy_rows, list) else []:
        if not isinstance(raw, dict):
            continue
        coord = _normalize_coordinate(raw.get("coordinate") or raw)
        if coord is None:
            continue
        out.append(
            (
                int(round(float(coord["latitude"]) * 1e5)),
                int(round(float(coord["longitude"]) * 1e5)),
                int(round(float(coord.get("altitude") or 0.0))),
            )
        )
    return tuple(sorted(out))


def _hide_point_masked_from_every_enemy(
    hide: Dict[str, Any],
    enemy_rows: Any,
    *,
    resource_dir: Any,
    emit: Callable[[str], None],
    tag: str,
) -> Optional[int]:
    """Prove a hide point is masked from EVERY supplied enemy. Fails closed.

    Returns the number of enemies actually ray-traced, or ``None`` when any one
    of them can see the point, cannot be evaluated, or is malformed.  A point
    that cannot be proven concealed is treated exactly like an exposed one.
    """

    hide_altitude_m = _to_float(hide.get("altitude"))
    if hide_altitude_m is None or not math.isfinite(float(hide_altitude_m)):
        # Substituting 0 m MSL puts the point under terrain, which returns
        # ENDPOINT_NOT_ABOVE_TERRAIN -> visible=False for every enemy, i.e. an
        # unknown altitude would certify as concealed from all of them.
        emit(f"{tag} hide altitude unavailable; treated as exposed.")
        return None
    if not isinstance(enemy_rows, list) or not enemy_rows:
        emit(f"{tag} no enemy set available; treated as exposed.")
        return None

    enemy_checked = 0
    for raw_enemy in enemy_rows:
        if not isinstance(raw_enemy, dict):
            emit(f"{tag} malformed enemy entry; treated as exposed.")
            return None
        enemy = _normalize_coordinate(raw_enemy.get("coordinate") or raw_enemy)
        if enemy is None:
            emit(f"{tag} enemy coordinate unavailable; treated as exposed.")
            return None
        try:
            assessment = evaluate_regional_los(
                resource_dir=resource_dir,
                observer_latitude=float(enemy["latitude"]),
                observer_longitude=float(enemy["longitude"]),
                observer_altitude_m=float(enemy.get("altitude") or 0.0),
                observer_height_m=float(ENEMY_OBSERVER_HEIGHT_M),
                target_latitude=float(hide["latitude"]),
                target_longitude=float(hide["longitude"]),
                target_altitude_m=float(hide_altitude_m),
                target_height_m=0.0,
                # No range gate: an enemy skipped for distance returns
                # visible=False without tracing a ray, which would read here as
                # proof of masking.  Concealment is proven against every
                # contact regardless of how far away it is.
                max_range_m=None,
                reject_nodata=False,
            )
        except Exception as exc:
            emit(f"{tag} enemy LOS failed ({type(exc).__name__}: {exc}); treated as exposed.")
            return None
        if (
            not isinstance(assessment, dict)
            or assessment.get("visible") is not False
            or assessment.get("evaluated") is not True
        ):
            emit(
                f"{tag} point is visible, unknown, or was never ray-traced for a "
                f"current enemy (reason={(assessment or {}).get('reason')}); treated as exposed."
            )
            return None
        enemy_checked += 1
    return int(enemy_checked)


def _preserved_lah_cover_still_masked(
    *,
    aircraft_id: int,
    ctx: Dict[str, Any],
    enemy_contact: Any,
    emit: Callable[[str], None],
) -> bool:
    """Is a committed LAH's inherited cover point still masked from every enemy?

    A committed package is normally reused verbatim.  That is only safe while
    the contact set it was certified against still holds: a newly discovered
    enemy can see straight onto a point that was masked from the old ones, and
    reusing it would leave a manned aircraft parked in the open.  Anything that
    cannot be proven - missing identity, unreadable path, unevaluated ray -
    fails closed and forces the aircraft to be re-planned.
    """

    tag = f"[ATTACK][CONTINUITY][WARN] aircraft={int(aircraft_id)}"
    contact = enemy_contact if isinstance(enemy_contact, dict) else {}
    enemy_rows = contact.get("enemy_targets") or contact.get("enemy_coordinates") or []
    if not isinstance(enemy_rows, list) or not enemy_rows:
        emit(f"{tag} no current contact set to re-certify the inherited cover against.")
        return False
    resource_dir = _attack_los_resource_dir()
    if resource_dir is None:
        emit(f"{tag} DEM resource unavailable for inherited-cover re-certification.")
        return False

    committed_row = next(
        (
            row
            for row in (ctx.get("_preserved_source_attack_rows") or [])
            if isinstance(row, dict)
            and _to_int(row.get("aircraftID")) == int(aircraft_id)
        ),
        None,
    )
    if committed_row is None:
        emit(f"{tag} committed attack identity is missing.")
        return False
    committed_path_id = _to_int(committed_row.get("pathID"))
    if committed_path_id is None:
        emit(f"{tag} committed pathID is missing.")
        return False
    try:
        committed_path = read_json_cached(
            db_paths.get_db_subpath("FlightPath", f"{int(committed_path_id)}.json"),
            copy_result=False,
            kind="FlightPath",
        )
    except Exception as exc:
        emit(f"{tag} committed path load failed ({type(exc).__name__}: {exc}).")
        return False
    if not isinstance(committed_path, dict) or _to_int(
        committed_path.get("aircraftID")
    ) != int(aircraft_id):
        emit(f"{tag} committed path ownership/identity mismatch.")
        return False
    waypoints = committed_path.get("lahWaypointList")
    if not isinstance(waypoints, list) or not waypoints:
        emit(f"{tag} committed path has no waypoints.")
        return False
    hide = _normalize_coordinate(_extract_lah_waypoint_coordinate(waypoints[-1]))
    if hide is None:
        emit(f"{tag} committed path endpoint coordinate is missing.")
        return False

    enemy_checked = _hide_point_masked_from_every_enemy(
        hide,
        enemy_rows,
        resource_dir=resource_dir,
        emit=emit,
        tag=f"{tag} inherited cover:",
    )
    if enemy_checked is None:
        return False
    emit(
        "[ATTACK][CONTINUITY] Inherited cover re-certified for "
        f"aircraft={int(aircraft_id)} against {int(enemy_checked)} current "
        f"enem{'y' if enemy_checked == 1 else 'ies'}."
    )
    return True


def _certify_incremental_append_hide_endpoint(
    hide_coord: Optional[Dict[str, Any]],
    descriptor: Dict[str, Any],
    *,
    emit: Callable[[str], None],
) -> Optional[Dict[str, Any]]:
    """Recheck the inherited cover point against the current full contact set.

    An append-only transaction may not move or rebuild the already-committed
    attack/resume graph.  Therefore its only safe firing origin is the exact
    cover endpoint at which that attack path terminates.  Revalidate that point
    with the same regional DEM LOS evaluator used by the simulator: every
    supplied enemy must remain masked and at least the configured attacker UAV
    link count must remain visible.  Any unknown result fails closed.
    """

    hide = _normalize_coordinate(hide_coord)
    contact = descriptor.get("enemy_contact")
    if hide is None or not isinstance(contact, dict):
        emit("[ATTACK][APPEND][WARN] inherited hide/contact context unavailable.")
        return None
    resource_dir = _attack_los_resource_dir()
    if resource_dir is None:
        emit("[ATTACK][APPEND][WARN] DEM resource unavailable for inherited-hide certification.")
        return None

    enemy_rows = contact.get("enemy_targets") or contact.get("enemy_coordinates") or []
    enemy_checked = _hide_point_masked_from_every_enemy(
        hide,
        enemy_rows,
        resource_dir=resource_dir,
        emit=emit,
        tag="[ATTACK][APPEND][WARN] inherited endpoint:",
    )
    if enemy_checked is None:
        return None

    uav_rows = contact.get("uav_states") or []
    required_links = max(
        1, get_runtime_attack_int("tactical_attacker_min_uav_links", 1)
    )
    communication_range_m = max(
        0.0, get_runtime_attack_float("tactical_communication_range_m", 10000.0)
    )
    visible_uav_ids: List[int] = []
    for raw_uav in uav_rows if isinstance(uav_rows, list) else []:
        if not isinstance(raw_uav, dict):
            continue
        uav = _normalize_coordinate(raw_uav.get("coordinate") or raw_uav)
        if uav is None:
            continue
        try:
            assessment = evaluate_regional_los(
                resource_dir=resource_dir,
                observer_latitude=float(uav["latitude"]),
                observer_longitude=float(uav["longitude"]),
                observer_altitude_m=float(uav.get("altitude") or 0.0),
                observer_height_m=0.0,
                target_latitude=float(hide["latitude"]),
                target_longitude=float(hide["longitude"]),
                target_altitude_m=float(hide_altitude_m),
                target_height_m=0.0,
                max_range_m=(
                    float(communication_range_m)
                    if communication_range_m > 0.0
                    else None
                ),
                reject_nodata=True,
            )
        except Exception:
            continue
        if (
            isinstance(assessment, dict)
            and assessment.get("visible") is True
            and assessment.get("evaluated") is True
        ):
            uav_id = _to_int(raw_uav.get("aircraft_id") or raw_uav.get("aircraftID"))
            visible_uav_ids.append(int(uav_id or 0))
    if len(visible_uav_ids) < int(required_links):
        emit(
            "[ATTACK][APPEND][WARN] inherited endpoint lost required UAV LOS "
            f"(links={len(visible_uav_ids)}/{required_links}); append deferred."
        )
        return None
    return {
        "enemyCheckedCount": int(enemy_checked),
        "uavLinkCount": len(visible_uav_ids),
        "requiredUavLinks": int(required_links),
        "visibleUavIDs": visible_uav_ids,
    }


def _append_attack_mission_preserving_graph(
    imp_data: Dict[str, Any],
    *,
    committed_row: Dict[str, Any],
    new_attack_mission: Dict[str, Any],
    new_imp_id: int,
    now_ms: int,
) -> Optional[Tuple[Dict[str, Any], int]]:
    """Insert one mission after its committed attack without rewriting peers."""

    source_missions = imp_data.get("individualMissionList")
    if not isinstance(source_missions, list):
        return None
    committed_mission_id = _to_int(committed_row.get("individualMissionID"))
    committed_path_id = _to_int(committed_row.get("pathID"))
    committed_target_id = _to_int(committed_row.get("targetID"))
    matching_indices: List[int] = []
    for index, mission in enumerate(source_missions):
        if not isinstance(mission, dict) or bool(mission.get("isDone")):
            continue
        info = mission.get("individualMissionInfo")
        mission_target_id = _to_int(
            info.get("targetID") if isinstance(info, dict) else None
        )
        if (
            _to_int(mission.get("individualMissionID")) == committed_mission_id
            and _to_int(mission.get("pathID")) == committed_path_id
            and mission_target_id == committed_target_id
        ):
            matching_indices.append(int(index))
    if len(matching_indices) != 1:
        return None

    new_mission_id = _to_int(new_attack_mission.get("individualMissionID"))
    new_path_id = _to_int(new_attack_mission.get("pathID"))
    if new_mission_id is None or new_path_id is None:
        return None
    if any(
        isinstance(mission, dict)
        and (
            _to_int(mission.get("individualMissionID")) == int(new_mission_id)
            or _to_int(mission.get("pathID")) == int(new_path_id)
        )
        for mission in source_missions
    ):
        return None

    committed_index = matching_indices[0]
    insert_index = int(committed_index) + 1
    candidate = dict(imp_data)
    candidate["individualMissionPackageID"] = int(new_imp_id)
    candidate["timestamp"] = int(now_ms)
    candidate_missions = list(source_missions)
    candidate_missions.insert(int(insert_index), new_attack_mission)
    candidate["individualMissionList"] = candidate_missions

    # Artifact-level invariant: deleting only the inserted object must restore
    # the source mission array byte-for-byte in value and in the same order.
    restored = list(candidate_missions)
    restored.pop(int(insert_index))
    if restored != list(source_missions):
        return None
    return candidate, int(insert_index)


def _build_lah_incremental_attack_append_package(
    *,
    descriptor: Dict[str, Any],
    assigned_targets: List[Dict[str, Any]],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    aircraft_id: int,
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
    id_reservation: AttackIdReservation | None = None,
    defer_write: bool = False,
) -> Optional[Dict[str, Any]]:
    """Append one third-target attack while retaining the active graph intact."""

    if id_reservation is None:
        raise RuntimeError("AttackIdReservation is required for LAH attack append builder")
    valid_targets = [
        dict(item)
        for item in assigned_targets or []
        if isinstance(item, dict)
        and _normalize_coordinate(item.get("coordinate") or item.get("attack_coord"))
    ]
    if len(valid_targets) != 1:
        emit("[ATTACK][APPEND][WARN] append transaction requires exactly one valid target.")
        return None
    committed_row = descriptor.get("committed_attack_row")
    if not isinstance(committed_row, dict):
        emit("[ATTACK][APPEND][WARN] committed attack identity is missing.")
        return None
    if _to_int(committed_row.get("aircraftID")) != int(aircraft_id):
        emit("[ATTACK][APPEND][WARN] committed attack owner mismatch.")
        return None

    committed_path_id = _to_int(committed_row.get("pathID"))
    committed_mission_id = _to_int(committed_row.get("individualMissionID"))
    committed_waypoint_id = _to_int(committed_row.get("waypointID"))
    committed_target_id = _to_int(committed_row.get("targetID"))
    if None in (
        committed_path_id,
        committed_mission_id,
        committed_waypoint_id,
        committed_target_id,
    ):
        emit("[ATTACK][APPEND][WARN] committed attack identity is incomplete.")
        return None

    try:
        committed_path_src = db_paths.get_db_subpath(
            "FlightPath", f"{int(committed_path_id)}.json"
        )
        committed_path = read_json_cached(
            committed_path_src,
            copy_result=False,
            kind="FlightPath",
        )
    except Exception as exc:
        emit(f"[ATTACK][APPEND][WARN] committed path load failed: {exc}")
        return None
    if not isinstance(committed_path, dict):
        return None
    if (
        _to_int(committed_path.get("pathID")) != int(committed_path_id)
        or _to_int(committed_path.get("aircraftID")) != int(aircraft_id)
        or _to_int(committed_path.get("individualMissionID")) != int(committed_mission_id)
    ):
        emit("[ATTACK][APPEND][WARN] committed path ownership/identity mismatch.")
        return None
    committed_waypoints = committed_path.get("lahWaypointList")
    if not isinstance(committed_waypoints, list) or not committed_waypoints:
        emit("[ATTACK][APPEND][WARN] committed LAH path has no waypoints.")
        return None
    matching_attack_waypoints = [
        waypoint
        for waypoint in committed_waypoints
        if isinstance(waypoint, dict)
        and not bool(waypoint.get("isDone"))
        and _to_int(waypoint.get("waypointID")) == int(committed_waypoint_id)
        and _to_int((waypoint.get("attack") or {}).get("targetID"))
        == int(committed_target_id)
    ]
    if len(matching_attack_waypoints) != 1:
        emit("[ATTACK][APPEND][WARN] committed attack waypoint is no longer uniquely active.")
        return None
    append_start = _extract_lah_waypoint_coordinate(committed_waypoints[-1])
    if append_start is None:
        emit("[ATTACK][APPEND][WARN] committed path endpoint coordinate is missing.")
        return None

    hide_certificate = _certify_incremental_append_hide_endpoint(
        append_start,
        descriptor,
        emit=emit,
    )
    if hide_certificate is None:
        return None

    assigned = valid_targets[0]
    target_coord = _normalize_coordinate(
        assigned.get("coordinate") or assigned.get("attack_coord")
    )
    target_id = _to_int(assigned.get("target_id") or assigned.get("targetID"))
    if target_coord is None or target_id is None or target_id <= 0:
        emit("[ATTACK][APPEND][WARN] novel target identity/coordinate is invalid.")
        return None
    contact = descriptor.get("enemy_contact") if isinstance(descriptor.get("enemy_contact"), dict) else {}
    attack_coord = _attack_coordinate_at_hide_endpoint(
        append_start,
        target_coord,
        threat_targets=contact.get("enemy_targets") or contact.get("enemy_coordinates"),
        attack_target_id=int(target_id),
        emit=emit,
        aircraft_id=int(aircraft_id),
    )
    if attack_coord is None:
        emit(
            "[ATTACK][APPEND][WARN] no firing point exists above "
            "the inherited cover endpoint; append deferred."
        )
        return None

    new_path_id = int(id_reservation.next_path(int(aircraft_id)))
    new_individual_id = int(id_reservation.next_individual())
    attack_waypoint_id = int(id_reservation.next_waypoint())
    selected_weapon_type = _to_int(
        assigned.get("selected_weapon_type") or assigned.get("weapon_type")
    )
    if selected_weapon_type not in (1, 2, 3):
        selected_weapon_type = _resolve_attack_weapon_type(
            {
                "target_type": _to_int(assigned.get("target_type") or assigned.get("targetType")),
                "target_id": int(target_id),
            },
            {"weapon_inventory": state.get("weapon_inventory")},
        )
    template_wp = (
        deepcopy(committed_waypoints[0])
        if isinstance(committed_waypoints[0], dict)
        else _default_lah_waypoint_template()
    )
    attack_waypoints = _build_lah_low_level_attack_waypoints(
        template_wp=template_wp,
        start_coord=append_start,
        attack_coord=attack_coord,
        attack_waypoint_id=int(attack_waypoint_id),
        waypoint_id_provider=id_reservation.next_waypoint,
        target_id=int(target_id),
        weapon_type=_to_int(selected_weapon_type),
        speed_mps=_lah_max_attack_speed_mps(),
        route_coordinates=[dict(append_start), dict(attack_coord)],
        regain_cover_coord=dict(append_start),
    )
    if not attack_waypoints or not any(
        isinstance(waypoint, dict)
        and _to_int(waypoint.get("waypointID")) == int(attack_waypoint_id)
        and _to_int((waypoint.get("attack") or {}).get("targetID")) == int(target_id)
        for waypoint in attack_waypoints
    ):
        emit("[ATTACK][APPEND][WARN] appended attack waypoint generation failed.")
        return None

    source_missions = imp_data.get("individualMissionList")
    if not isinstance(source_missions, list):
        emit("[ATTACK][APPEND][WARN] source IMP mission list is unavailable.")
        return None
    committed_mission = next(
        (
            mission
            for mission in source_missions
            if isinstance(mission, dict)
            and _to_int(mission.get("individualMissionID")) == int(committed_mission_id)
            and _to_int(mission.get("pathID")) == int(committed_path_id)
        ),
        None,
    )
    if not isinstance(committed_mission, dict):
        emit("[ATTACK][APPEND][WARN] committed mission is absent from source IMP.")
        return None
    related_mission = deepcopy(committed_mission.get("relatedMission") or {})
    new_attack_mission = {
        "individualMissionID": int(new_individual_id),
        "isDone": False,
        "relatedMission": related_mission,
        "individualMissionInfo": {
            "individualMissionType": 2,
            "patternType": 2,
            "autoZoomIn": False,
            "targetID": int(target_id),
            "coordinateList": [
                {
                    "latitude": attack_coord["latitude"],
                    "longitude": attack_coord["longitude"],
                    "altitude": attack_coord["altitude"],
                }
            ],
        },
        "pathID": int(new_path_id),
        "incrementalAttackAppend": True,
        "appendAfterIndividualMissionID": int(committed_mission_id),
    }
    merged = _append_attack_mission_preserving_graph(
        imp_data,
        committed_row=committed_row,
        new_attack_mission=new_attack_mission,
        new_imp_id=int(new_imp_id),
        now_ms=int(now_ms),
    )
    if merged is None:
        emit("[ATTACK][APPEND][WARN] source IMP graph changed or append point is ambiguous.")
        return None
    candidate_imp, insert_index = merged

    new_path_payload = {
        "timestamp": int(now_ms),
        "Source": _extract_path_source(committed_path),
        "pathID": int(new_path_id),
        "aircraftID": int(aircraft_id),
        "individualMissionID": int(new_individual_id),
        "incrementalAttackAppend": True,
        "appendAfterPathID": int(committed_path_id),
        "lahWaypointList": attack_waypoints,
    }
    imp_dest = db_paths.get_db_subpath(
        "IndividualMissionPlan", f"{int(new_imp_id)}.json"
    )
    path_dest = db_paths.get_db_subpath("FlightPath", f"{int(new_path_id)}.json")
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [
        (imp_dest, candidate_imp),
        (path_dest, new_path_payload),
    ]
    _validate_generated_artifact_write_entries(
        scope=f"attack_lah_incremental_append:{int(new_imp_id)}",
        individual_mission_plans=[candidate_imp],
        entries=write_entries,
        log=emit,
    )
    _write_results, deferred_write_entries = _write_or_defer_attack_json_entries(
        write_entries,
        defer_write=bool(defer_write),
    )
    emit(
        "[ATTACK][APPEND] Preserved the committed IMP mission order/IDs and "
        f"inserted one attack (aircraft={int(aircraft_id)}, target={int(target_id)}, "
        f"missionIndex={int(insert_index)}, path={int(new_path_id)})."
    )
    result: Dict[str, Any] = {
        "aircraft_id": int(aircraft_id),
        "role": descriptor.get("label") or "manned",
        "individualMissionPackageID": int(new_imp_id),
        "appendOnly": True,
        "preservedAttack": dict(committed_row),
        "attack": {
            "individualMissionID": int(new_individual_id),
            "pathID": int(new_path_id),
        },
        "attackSequence": [
            {
                "targetID": int(target_id),
                "pathID": int(new_path_id),
                "individualMissionID": int(new_individual_id),
                "attackCoordinate": dict(attack_coord),
                "appendAfterPathID": int(committed_path_id),
                "hideCertificate": dict(hide_certificate),
            }
        ],
        "timingMs": {"appendOnly": True},
    }
    if deferred_write_entries:
        result["_deferredWriteEntries"] = deferred_write_entries
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
    defer_write: bool = False,
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

    tactical_plan = _plan_lah_enemy_contact_response(
        descriptor,
        state,
        role="attacker",
        emit=emit,
    )
    tactical_endpoint = _lah_tactical_endpoint_coordinate(tactical_plan)
    uncovered_direct_attack = bool(
        _lah_tactical_cover_required(descriptor) and tactical_endpoint is None
    )
    # Terrain sometimes offers no cover at all: where the aircraft stands, the
    # enemy can see below its own minimum safe altitude, so no hide interval
    # exists to certify.  Refusing to plan then produces an option with no
    # attack in it, which is worse than an uncovered attack the operator can
    # see and judge.  Plan it, and say plainly that cover is not certified.
    if _lah_tactical_cover_required(descriptor) and tactical_endpoint is None:
        emit(
            "[ATTACK][TACTICAL][WARN] No certified concealment route exists for "
            f"aircraft {int(aircraft_id)}; retaining the precomputed attack point "
            "and generating a direct terrain-safe attack/resume route. "
            f"detail={(tactical_plan or {}).get('detail')}"
        )

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
    if uncovered_direct_attack and special_lah_attack_coord is not None:
        # A phase anchor is not a firing solution.  When concealment planning
        # degrades, retain the already computed per-target attack point instead
        # of replacing it with an unrelated hold/battle coordinate.
        emit(
            "[ATTACK][TACTICAL][WARN] Precomputed attack point overrides the "
            f"phase battle anchor during direct fallback (aircraft={int(aircraft_id)}, "
            f"inputMissionID={int(input_mission_id)})."
        )
        special_lah_attack_coord = None
    if tactical_endpoint is not None and special_lah_attack_coord is not None:
        # Enemy-contact safety takes precedence over a phase battle anchor.
        # Keeping the special coordinate here bypassed the hide-point popup and
        # recreated the long hide -> remote anchor -> return excursion.
        emit(
            "[ATTACK][TACTICAL] Certified concealment overrides the phase "
            f"battle-position anchor (aircraft={int(aircraft_id)}, "
            f"inputMissionID={int(input_mission_id)})."
        )
        special_lah_attack_coord = None
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
    predicted_attack_start = _predict_lah_attack_route_start(current_coord, state) or dict(current_coord)
    predicted_start_distance_m = _haversine_distance_m(current_coord, predicted_attack_start) or 0.0
    if tactical_endpoint is not None:
        emit(
            "[ATTACK][TACTICAL] First attack route starts at the certified hide endpoint "
            f"(aircraft={int(aircraft_id)}, eta={(tactical_plan or {}).get('etaS')}s)."
        )
    else:
        emit(
            "[ATTACK][LAH] First attack route starts at the 10s-ahead position "
            f"(aircraft={int(aircraft_id)}, projected={float(predicted_start_distance_m):.1f}m)."
        )
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
    enemy_contact = descriptor.get("enemy_contact")
    if not isinstance(enemy_contact, dict):
        enemy_contact = {}
    popup_threat_targets = (
        enemy_contact.get("enemy_targets")
        or enemy_contact.get("enemy_coordinates")
    )

    # Solve every firing point first.  A target the aircraft physically cannot
    # engage from its cover must not cancel the targets it can: aborting the
    # whole aircraft on the first unsolvable one is what left an attack option
    # with no attack in it.  Only a clean sweep of failures aborts.
    firing_point_by_index: Dict[int, Dict[str, Any]] = {}
    unengageable_targets: List[Dict[str, Any]] = []
    if tactical_endpoint is not None and special_lah_attack_coord is None:
        engageable_targets: List[Dict[str, Any]] = []
        for probe_idx, probe in enumerate(valid_targets):
            if not _normalize_coordinate(probe.get("attack_coord") or probe.get("coordinate")):
                engageable_targets.append(probe)
                continue
            solved = _attack_coordinate_at_hide_endpoint(
                tactical_endpoint,
                _normalize_coordinate(probe.get("coordinate")),
                threat_targets=popup_threat_targets,
                attack_target_id=_to_int(probe.get("target_id") or probe.get("targetID")),
                emit=emit,
                aircraft_id=int(aircraft_id),
            )
            if solved is None:
                unengageable_targets.append(probe)
                # The global attack-point solver already selected the nearest
                # available firing point for this target.  Keep the target and
                # use that point when a vertical pop-up from this particular
                # hide endpoint cannot be certified.
                engageable_targets.append(probe)
                continue
            firing_point_by_index[len(engageable_targets)] = solved
            engageable_targets.append(probe)
        if unengageable_targets:
            emit(
                "[ATTACK][TACTICAL][WARN] No vertical pop-up solution for "
                f"{len(unengageable_targets)} target(s) at the certified hide point "
                f"(aircraft={int(aircraft_id)}, targets="
                f"{[_to_int(item.get('target_id') or item.get('targetID')) for item in unengageable_targets]}); "
                "using each target's precomputed closest attack point instead."
            )
        valid_targets = engageable_targets

    for idx, assigned in enumerate(valid_targets):
        attack_coord = _normalize_coordinate(assigned.get("attack_coord") or assigned.get("coordinate"))
        if not attack_coord:
            continue
        assigned_attack_target_id = _to_int(
            assigned.get("target_id") or assigned.get("targetID")
        )
        if special_lah_attack_coord is not None:
            attack_coord = dict(special_lah_attack_coord)
        # Fire from cover.  Every tactical attack uses the same certified
        # anchor; additional contacts are normally deferred to the next replan,
        # but keeping this true for every item also makes the builder safe when
        # called directly with more than one target.
        attack_from_hide = (
            firing_point_by_index.get(int(idx))
            if tactical_endpoint is not None and special_lah_attack_coord is None
            else None
        )
        if (
            tactical_endpoint is not None
            and special_lah_attack_coord is None
            and attack_from_hide is None
            and _normalize_coordinate(assigned.get("attack_coord") or assigned.get("coordinate")) is not None
        ):
            emit(
                "[ATTACK][TACTICAL][WARN] Vertical pop-up unavailable; departing "
                "cover for the precomputed closest attack point and returning "
                f"to cover (aircraft={int(aircraft_id)}, target="
                f"{_to_int(assigned.get('target_id') or assigned.get('targetID'))})."
            )
        if attack_from_hide is not None:
            # Do not move a LOS-certified popup after solving it.  Formation
            # cohesion is already reflected in the independently selected hide
            # anchors; an XY mutation here invalidates LOS and creates a detour.
            attack_coord = dict(attack_from_hide)
        elif uncovered_direct_attack:
            # The tactical solver failed, but this per-target coordinate came
            # from the attack-point solver.  Formation adjustment would move
            # its XY after LOS selection and can invalidate the only available
            # firing solution.
            attack_coord = dict(attack_coord)
        else:
            attack_coord = _keep_lah_with_group(
                attack_coord,
                group_coords=_manned_group_coordinates_from_ctx(
                    ctx, exclude_aircraft_id=int(aircraft_id)
                ),
                target_coord=_normalize_coordinate(assigned.get("coordinate")),
                emit=emit,
                aircraft_id=int(aircraft_id),
            ) or attack_coord
        if attack_from_hide is not None:
            pass
        else:
            # The floor keeps an attack point from commanding a descent to a
            # stale altitude.  A hide-endpoint attack point is deliberately the
            # lowest altitude that still sees the target, so the floor must not
            # drag it back up to wherever the aircraft happens to be.
            _apply_lah_altitude_floor(attack_coord, current_coord)
        preserve_attack_altitude = (
            attack_from_hide is not None
            or _preserve_attack_point_altitude(assigned.get("attack_coord"))
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

        attack_target_id = assigned_attack_target_id or 0
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

        attack_start_coord = (
            dict(previous_attack_coord)
            if previous_attack_coord is not None
            else dict(tactical_endpoint or predicted_attack_start)
        )
        if attack_from_hide is not None:
            # A local pop-up is not a transit mission.  Sending this two-point
            # leg through the general mission-zone router made an aircraft enter
            # a distant corridor and U-turn before firing.
            attack_route_coordinates = [dict(attack_start_coord), dict(attack_coord)]
            attack_route_meta = {
                "sourcePlanID": _to_int(getattr(artifacts, "source_plan_id", None)),
                "zoneCount": 0,
                "routePointCount": 2,
                "constrained": False,
                "reason": "tactical_vertical_popup",
            }
        else:
            attack_route_coordinates, attack_route_meta = _build_lah_mission_constrained_attack_route(
                start_coord=attack_start_coord,
                attack_coord=attack_coord,
                source_plan_id=_to_int(getattr(artifacts, "source_plan_id", None)),
            )
        emit(
            "[ATTACK][LAH] Mission-zone attack route "
            f"(aircraft={int(aircraft_id)}, target={int(attack_target_id)}, "
            f"points={len(attack_route_coordinates)}, constrained={bool(attack_route_meta.get('constrained'))}, "
            f"startInside={attack_route_meta.get('startInside')}, "
            f"reason={attack_route_meta.get('reason')})."
        )
        attack_waypoints = _build_lah_low_level_attack_waypoints(
            template_wp=template_wp,
            start_coord=attack_start_coord,
            attack_coord=attack_coord,
            attack_waypoint_id=int(attack_wp_id),
            waypoint_id_provider=id_reservation.next_waypoint,
            target_id=attack_target_id,
            weapon_type=selected_weapon_type,
            speed_mps=attack_speed_mps,
            route_coordinates=attack_route_coordinates,
            # Sink back behind the same terrain the pop-up came from.
            regain_cover_coord=tactical_endpoint if tactical_endpoint is not None else None,
        )
        if idx == 0 and tactical_endpoint is not None:
            tactical_waypoints = _build_lah_tactical_route_waypoints(
                template_wp=template_wp,
                plan=tactical_plan,
                waypoint_id_provider=id_reservation.next_waypoint,
                # Settle in cover before exposing the aircraft for the shot.
                terminal_hover_seconds=_attack_cover_hold_seconds(),
            )
            if tactical_waypoints:
                attack_waypoints = _prepend_lah_tactical_waypoints(
                    tactical_waypoints,
                    attack_waypoints,
                )
        allocated_waypoint_ids.extend(
            int(waypoint.get("waypointID"))
            for waypoint in attack_waypoints
            if _to_int(waypoint.get("waypointID")) is not None
        )
        attack_fp_data = {
            "timestamp": now_ms,
            "Source": _extract_path_source(fp_data),
            "pathID": attack_path_id,
            "aircraftID": aircraft_id,
            "individualMissionID": attack_individual_id,
            "lahWaypointList": attack_waypoints,
        }
        attack_fp_dest = db_paths.get_db_subpath("FlightPath", f"{attack_path_id}.json")
        attack_path_payloads.append((attack_fp_dest, attack_fp_data))
        if idx == 0 and tactical_endpoint is not None:
            # SIM display metadata: which of these waypoints are the concealment
            # ground, so the map can call them out from ordinary holds.
            _record_lah_tactical_points(
                path_id=attack_path_id,
                waypoints=attack_waypoints,
                plan=tactical_plan,
                role="attacker",
                conceal_coordinate=tactical_endpoint,
            )
        attack_sequence_meta.append(
            {
                "targetID": int(attack_target_id),
                "targetType": int(attack_target_type) if attack_target_type is not None else None,
                "weaponType": int(selected_weapon_type) if selected_weapon_type is not None else None,
                "pathID": int(attack_path_id),
                "individualMissionID": int(attack_individual_id),
                "attackCoordinate": dict(attack_coord),
                "attackSpeedMps": float(attack_speed_mps),
                "missionZoneRoute": dict(attack_route_meta),
                **(
                    {"hidePrelude": _json_safe(tactical_plan)}
                    if idx == 0 and tactical_endpoint is not None
                    else {}
                ),
            }
        )
        previous_attack_coord = dict(
            tactical_endpoint if tactical_endpoint is not None else attack_coord
        )

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
            missions=_drop_stale_lah_tactical_follow_ups(
                source_mission_list[target_index + 1 :],
                current_input_id=input_mission_id,
                emit=emit,
            ),
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
                hovering_time=get_runtime_attack_int("lah_hold_seconds", _LAH_COVER_HOLD_DEFAULT_SECONDS),
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
    if resume_waypoints:
        resume_waypoints = _rebuild_lah_low_level_resume_waypoints(
            attack_coord=previous_attack_coord,
            resume_waypoints=resume_waypoints,
            template_wp=template_wp,
            waypoint_id_provider=id_reservation.next_waypoint,
        )

    has_resume = bool(resume_waypoints)
    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = int(resume_individual_id)
    mission_resume["pathID"] = int(resume_path_id)
    mission_resume["relatedMission"] = dict(related_template)
    mission_resume["isDone"] = False
    mission_resume["postAttackResume"] = True
    last_attack_target_id = _to_int(
        attack_sequence_meta[-1].get("targetID") if attack_sequence_meta else None
    )
    mission_resume_info = mission_resume.get("individualMissionInfo")
    if isinstance(mission_resume_info, dict):
        mission_resume["individualMissionInfo"] = deepcopy(mission_resume_info)
        mission_resume["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)
        mission_resume["individualMissionInfo"]["targetID"] = None
        if last_attack_target_id is not None and last_attack_target_id > 0:
            mission_resume["postAttackSourceTargetID"] = int(last_attack_target_id)

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = _extract_path_source(fp_data)
    resume_fp_data["pathID"] = int(resume_path_id)
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = int(resume_individual_id)
    resume_fp_data["postAttackResume"] = True
    if last_attack_target_id is not None and last_attack_target_id > 0:
        resume_fp_data["postAttackSourceTargetID"] = int(last_attack_target_id)
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
    write_results, deferred_write_entries = _write_or_defer_attack_json_entries(
        write_entries,
        defer_write=bool(defer_write),
    )
    _record_builder_stage(
        "write_json",
        write_started,
        fileCount=len(write_results),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
        deferred=bool(defer_write),
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
    if unengageable_targets:
        result["degradedCoverAttackTargetIDs"] = sorted(
            {
                int(target_id)
                for target in unengageable_targets
                for target_id in [
                    _to_int(target.get("target_id") or target.get("targetID"))
                ]
                if target_id is not None and int(target_id) > 0
            }
        )
    if tactical_endpoint is not None:
        result["hidePrelude"] = _json_safe(tactical_plan)
    if has_resume:
        result["resume"] = {
            "individualMissionID": int(resume_individual_id),
            "pathID": int(resume_path_id),
        }
    if deferred_write_entries:
        result["_deferredWriteEntries"] = deferred_write_entries
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
    defer_write: bool = False,
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

    tactical_plan = _plan_lah_enemy_contact_response(
        descriptor,
        state,
        role="attacker",
        emit=emit,
    )
    tactical_endpoint = _lah_tactical_endpoint_coordinate(tactical_plan)
    uncovered_direct_attack = bool(
        _lah_tactical_cover_required(descriptor) and tactical_endpoint is None
    )
    if uncovered_direct_attack:
        emit(
            "[ATTACK][TACTICAL][WARN] No certified concealment/communication route; "
            "retaining the precomputed attack point and generating a direct "
            f"attack/resume route (aircraft={int(aircraft_id)})."
        )

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
    if uncovered_direct_attack and special_lah_attack_coord is not None:
        emit(
            "[ATTACK][TACTICAL][WARN] Precomputed attack point overrides the "
            f"phase battle anchor during direct fallback (aircraft={int(aircraft_id)}, "
            f"inputMissionID={int(input_mission_id)})."
        )
        special_lah_attack_coord = None
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
            missions=_drop_stale_lah_tactical_follow_ups(
                source_mission_list[target_index + 1 :],
                current_input_id=input_mission_id,
                emit=emit,
            ),
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
                hovering_time=get_runtime_attack_int("lah_hold_seconds", _LAH_COVER_HOLD_DEFAULT_SECONDS),
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
    if resume_waypoints:
        resume_waypoints = _rebuild_lah_low_level_resume_waypoints(
            attack_coord=attack_coord_norm,
            resume_waypoints=resume_waypoints,
            template_wp=template_wp,
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

    predicted_attack_start = _predict_lah_attack_route_start(current_coord, state) or dict(current_coord)
    predicted_start_distance_m = _haversine_distance_m(current_coord, predicted_attack_start) or 0.0
    attack_start_coord = dict(tactical_endpoint or predicted_attack_start)
    attack_route_coordinates, attack_route_meta = _build_lah_mission_constrained_attack_route(
        start_coord=attack_start_coord,
        attack_coord=attack_coord_norm,
        source_plan_id=_to_int(getattr(artifacts, "source_plan_id", None)),
    )
    if tactical_endpoint is not None:
        emit(
            "[ATTACK][TACTICAL] Attack approach begins at certified hide endpoint "
            f"(aircraft={int(aircraft_id)}, eta={(tactical_plan or {}).get('etaS')}s)."
        )
    else:
        emit(
            "[ATTACK][LAH] First attack route starts at the 10s-ahead position "
            f"(aircraft={int(aircraft_id)}, projected={float(predicted_start_distance_m):.1f}m)."
        )
    emit(
        "[ATTACK][LAH] Mission-zone attack route "
        f"(aircraft={int(aircraft_id)}, target={int(attack_target_id_value)}, "
        f"points={len(attack_route_coordinates)}, constrained={bool(attack_route_meta.get('constrained'))}, "
        f"startInside={attack_route_meta.get('startInside')}, "
        f"reason={attack_route_meta.get('reason')})."
    )
    attack_waypoints = _build_lah_low_level_attack_waypoints(
        template_wp=template_wp,
        start_coord=attack_start_coord,
        attack_coord=attack_coord_norm,
        attack_waypoint_id=int(attack_wp_id),
        waypoint_id_provider=id_reservation.next_waypoint,
        target_id=attack_target_id,
        weapon_type=selected_weapon_type,
        speed_mps=attack_speed_mps,
        route_coordinates=attack_route_coordinates,
    )
    if tactical_endpoint is not None:
        tactical_waypoints = _build_lah_tactical_route_waypoints(
            template_wp=template_wp,
            plan=tactical_plan,
            waypoint_id_provider=id_reservation.next_waypoint,
        )
        if tactical_waypoints:
            attack_waypoints = _prepend_lah_tactical_waypoints(
                tactical_waypoints,
                attack_waypoints,
            )
    attack_fp_data = {
        "timestamp": now_ms,
        "Source": _extract_path_source(fp_data),
        "pathID": attack_path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": attack_individual_id,
        "lahWaypointList": attack_waypoints,
    }
    _record_builder_stage(
        "payload_build",
        payload_started,
        hasResume=bool(resume_waypoints),
        followUpMissionCount=len(follow_up_missions),
        attackSpeedMps=float(attack_speed_mps),
        attackRoutePointCount=len(attack_route_coordinates),
        attackRouteConstrained=bool(attack_route_meta.get("constrained")),
    )

    original_entry = deepcopy(target_mission)

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = dict(related_template)
    mission_resume["isDone"] = False
    mission_resume["postAttackResume"] = True
    mission_resume_info = mission_resume.get("individualMissionInfo")
    if isinstance(mission_resume_info, dict):
        mission_resume["individualMissionInfo"] = deepcopy(mission_resume_info)
        mission_resume["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)
        mission_resume["individualMissionInfo"]["targetID"] = None
        if attack_target_id is not None and attack_target_id > 0:
            mission_resume["postAttackSourceTargetID"] = int(attack_target_id)

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = _extract_path_source(fp_data)
    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = resume_individual_id
    resume_fp_data["postAttackResume"] = True
    if attack_target_id is not None and attack_target_id > 0:
        resume_fp_data["postAttackSourceTargetID"] = int(attack_target_id)
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
    write_results, deferred_write_entries = _write_or_defer_attack_json_entries(
        write_entries,
        defer_write=bool(defer_write),
    )
    _record_builder_stage(
        "write_json",
        write_started,
        fileCount=len(write_results),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
        deferred=bool(defer_write),
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
            "waypointIDs": [
                int(waypoint["waypointID"])
                for waypoint in attack_waypoints
                if _to_int(waypoint.get("waypointID")) is not None
            ],
        },
        "removedWaypointID": removed_wp_id,
        "attackPath": str(attack_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
        "attackCoordinate": dict(attack_coord_norm),
    }
    if tactical_endpoint is not None:
        result["hidePrelude"] = _json_safe(tactical_plan)
    if resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
        result["resumePath"] = str(resume_fp_dest)
    if deferred_write_entries:
        result["_deferredWriteEntries"] = deferred_write_entries
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
    defer_write: bool = False,
) -> Optional[Dict[str, Any]]:
    if id_reservation is None:
        raise RuntimeError("AttackIdReservation is required for LAH hold/resume builder")
    if target_index is None:
        emit(f"[ATTACK][LAH] Target mission index unavailable for aircraft {aircraft_id}.")
        return None

    live_current_coord = _normalize_coordinate(state.get("coordinate"))
    current_coord = dict(live_current_coord) if live_current_coord is not None else None
    if current_coord is None:
        current_coord = _extract_final_lah_coordinate(fp_data)
    if current_coord is None:
        emit(f"[ATTACK][LAH] Hold/resume coordinate missing for aircraft {aircraft_id}.")
        return None

    descriptor_mode = str(descriptor.get("mode") or "")
    is_command_relay = descriptor_mode == "LAH_RELAY"
    is_tactical_abort = descriptor_mode == "LAH_TACTICAL_ABORT"
    is_waiting_wingman = descriptor_mode == "LAH_HOLD_RESUME"
    precomputed_abort_plan = (
        descriptor.get("_certified_tactical_plan")
        if is_tactical_abort
        and isinstance(descriptor.get("_certified_tactical_plan"), dict)
        else None
    )
    relay_plan = precomputed_abort_plan or (
        _plan_lah_enemy_contact_response(
            descriptor,
            state,
            # The command aircraft must keep the datalink; a wingman waiting
            # out someone else's attack only has to stay hidden.
            role="relay" if is_command_relay else "attacker",
            emit=emit,
        )
        if (is_command_relay or is_waiting_wingman)
        else None
    )
    relay_endpoint = _lah_tactical_endpoint_coordinate(relay_plan)
    relay_failsafe_hold = bool(is_command_relay and relay_endpoint is None)
    live_position_failsafe = bool(
        relay_failsafe_hold or (is_tactical_abort and relay_endpoint is None)
    )
    if live_position_failsafe and live_current_coord is None:
        emit(
            "[ATTACK][TACTICAL][ERR] Safe hold has no certified route and no "
            f"live 0401 coordinate (aircraft={int(aircraft_id)}); refusing an "
            "uncertified mission-end fallback."
        )
        return None

    support_target_id = _to_int(descriptor.get("target_id"))
    if support_target_id is None or support_target_id <= 0:
        detail = ctx.get("replan_detail") if isinstance(ctx, dict) else {}
        if isinstance(detail, dict):
            support_target_id = _to_int(detail.get("targetID") or detail.get("targetId"))
            if support_target_id is None:
                orientation = detail.get("targetOrientation") or {}
                if isinstance(orientation, dict):
                    support_target_id = _to_int(
                        orientation.get("targetID") or orientation.get("targetId")
                    )
    if support_target_id is None or support_target_id <= 0:
        emit(
            f"[ATTACK][LAH] Target ID missing for hold/resume aircraft {aircraft_id}; "
            "attack option generation stopped."
        )
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
    if relay_endpoint is not None:
        replan_resume_anchor = dict(relay_endpoint)
    elif live_position_failsafe:
        replan_resume_anchor = dict(live_current_coord or current_coord)
    else:
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
    hold_coord = dict(relay_endpoint) if relay_endpoint is not None else None
    if hold_coord is not None:
        emit(
            "[ATTACK][TACTICAL] Command aircraft relay hold uses the certified "
            f"concealment/communication endpoint (aircraft={int(aircraft_id)}, "
            f"eta={(relay_plan or {}).get('etaS')}s)."
        )
    if live_position_failsafe:
        hold_coord = dict(live_current_coord or current_coord)
        if is_tactical_abort:
            emit(
                "[ATTACK][TACTICAL][WARN] Attack suppressed because no certified "
                "concealment/communication route exists; hold is fixed to the "
                f"live 0401 position (aircraft={int(aircraft_id)})."
            )
        else:
            emit(
                "[ATTACK][TACTICAL][WARN] No certified relay concealment route; "
                f"fail-closed hold is fixed to the live 0401 position "
                f"(aircraft={int(aircraft_id)}). Existing LINE/mission endpoints "
                "will not be used."
            )
    elif hold_coord is None:
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
        # A standby anchored to a LINE endpoint can sit far ahead of the rest
        # of the flight; keep this aircraft with the others and behind them.
        hold_coord = _keep_lah_with_group(
            hold_coord,
            group_coords=_manned_group_coordinates_from_ctx(
                ctx, exclude_aircraft_id=int(aircraft_id)
            ),
            target_coord=_attack_group_target_coordinate(ctx),
            emit=emit,
            aircraft_id=int(aircraft_id),
        ) or hold_coord

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
            missions=_drop_stale_lah_tactical_follow_ups(
                source_mission_list[target_index + 1 :],
                current_input_id=input_mission_id,
                emit=emit,
            ),
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
        anchor_coord=(
            relay_endpoint
            or (live_current_coord if live_position_failsafe else current_coord)
        ),
        state=state,
        emit=emit,
        log_prefix="[ATTACK][LAH]",
        predict_anchor=bool(relay_endpoint is None and not live_position_failsafe),
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
            "targetID": int(support_target_id),
        },
        "pathID": hold_path_id,
    }

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = dict(related_template)
    mission_resume["isDone"] = False
    mission_resume["postAttackResume"] = True
    mission_resume_info = mission_resume.get("individualMissionInfo")
    if isinstance(mission_resume_info, dict):
        mission_resume["individualMissionInfo"] = deepcopy(mission_resume_info)
        mission_resume["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)
        mission_resume["individualMissionInfo"]["targetID"] = None
        mission_resume["postAttackSourceTargetID"] = int(support_target_id)

    # Sit in cover for as long as the strike actually needs, not a flat block of
    # time.  Every manned aircraft in the engagement gets the same treatment -
    # the relay and the covering wingman were previously left on the configured
    # default and simply sat there long after the strike was over.  The
    # follow-up replan moves them on once the attack leaves the plan; the
    # configured value is only the fallback for when the geometry is unknown.
    hold_seconds = get_runtime_attack_int(
        "lah_hold_seconds", _LAH_COVER_HOLD_DEFAULT_SECONDS
    )
    waiting_seconds = _attack_wait_hold_seconds(ctx, state)
    if waiting_seconds is not None:
        hold_seconds = int(waiting_seconds)
        emit(
            "[ATTACK][LAH] Holds in cover for the strike "
            f"(aircraft={int(aircraft_id)}, role="
            f"{'relay' if is_command_relay else 'wingman'}, hold={hold_seconds}s)."
        )
    hold_waypoints = (
        _build_lah_tactical_route_waypoints(
            template_wp=template_wp,
            plan=relay_plan,
            waypoint_id_provider=id_reservation.next_waypoint,
            terminal_hover_seconds=hold_seconds,
        )
        if relay_endpoint is not None
        else []
    )
    if not hold_waypoints:
        hold_waypoints = [
            _build_lah_anchor_waypoint(
                template_wp,
                coord=hold_coord,
                next_id=0,
                hovering_time=hold_seconds,
                waypoint_id=id_reservation.next_waypoint(),
            )
        ]
    else:
        # The certified endpoint is the concealment point.  It has no ICD
        # field, so record it out of band for SIM display only.
        _record_lah_tactical_points(
            path_id=hold_path_id,
            waypoints=hold_waypoints,
            plan=relay_plan,
            role="relay" if is_command_relay else "hold",
        )
    hold_wp = hold_waypoints[-1]
    hold_fp_data = {
        "timestamp": now_ms,
        "Source": _extract_path_source(fp_data),
        "pathID": hold_path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": hold_individual_id,
        "lahWaypointList": hold_waypoints,
    }

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = _extract_path_source(fp_data)
    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = resume_individual_id
    resume_fp_data["postAttackResume"] = True
    resume_fp_data["postAttackSourceTargetID"] = int(support_target_id)
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
    write_results, deferred_write_entries = _write_or_defer_attack_json_entries(
        write_entries,
        defer_write=bool(defer_write),
    )
    _record_builder_stage(
        "write_json",
        write_started,
        fileCount=len(write_results),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
        deferred=bool(defer_write),
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
            "durationSeconds": hold_seconds,
            "waypointCount": len(hold_waypoints),
        },
        "removedWaypointID": removed_wp_id,
        "holdPath": str(hold_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
    }
    if is_command_relay:
        result["relayMode"] = True
        result["relayPlan"] = _json_safe(relay_plan)
        result["relayFallbackPolicy"] = (
            "hold_live_current_no_certified_route"
            if relay_failsafe_hold
            else "certified_concealment_route"
        )
        result["relayFallbackCertified"] = bool(relay_endpoint is not None)
    if is_tactical_abort:
        result["tacticalAbort"] = True
        result["attackSuppressed"] = True
        result["tacticalAbortPolicy"] = "hold_live_current_no_certified_route"
    if resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
        result["resumePath"] = str(resume_fp_dest)
    if deferred_write_entries:
        result["_deferredWriteEntries"] = deferred_write_entries
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
    if has_lah_waypoints:
        normalize_lah_eta_seconds_inplace(lah_waypoint_list)
    if has_waypoints or has_lah_waypoints:
        sanitize_flight_path_payload_filming_altitudes(data)
    if isinstance(data, dict) and Path(path).parent.name == "FlightPath":
        normalize_flight_path_waypoint_altitudes_inplace(data)
        normalize_flight_path_waypoint_speeds_inplace(data)
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
    for payload in flight_paths:
        normalize_flight_path_waypoint_altitudes_inplace(payload)
        normalize_flight_path_waypoint_speeds_inplace(payload)
    return validate_generated_artifact_payloads(
        individual_mission_plans=individual_mission_plans,
        flight_paths=flight_paths,
        scope=scope,
        allow_existing_db_artifacts=True,
        log=log,
    )


def _write_or_defer_attack_json_entries(
    entries: List[Tuple[Path, Dict[str, Any]]],
    *,
    defer_write: bool,
) -> Tuple[List[Dict[str, Any]], List[Tuple[Path, Dict[str, Any]]]]:
    if not defer_write:
        return _write_json_files_batch(entries), []
    deferred_entries = [(Path(path), payload) for path, payload in entries]
    return [
        {
            "path": str(path),
            "name": Path(path).name,
            "written": False,
            "skipped": False,
            "deferred": True,
        }
        for path, _payload in deferred_entries
    ], deferred_entries


def _write_json_files_batch(entries: List[Tuple[Path, Dict[str, Any]]]) -> List[Dict[str, Any]]:
    normalized_entries = [(Path(path), payload) for path, payload in entries]
    unique_paths = {_attack_fast_path_key(path) for path, _payload in normalized_entries}
    unique_payloads = {id(payload) for _path, payload in normalized_entries}
    workers = (
        _attack_json_write_workers(len(normalized_entries))
        if len(unique_paths) == len(normalized_entries)
        and len(unique_payloads) == len(normalized_entries)
        else 1
    )
    prepared_entries = [
        (path, _prepare_attack_json_payload(path, payload))
        for path, payload in normalized_entries
    ]
    tx = ReplanTransaction(
        f"attack-json-batch-{os.getpid()}-{threading.get_ident()}-{int(time.time() * 1000)}"
    )
    tx.stage_json_batch(
        prepared_entries,
        # Runtime artifacts are machine-consumed and can be large (lineSearch
        # coordinate arrays). Compact JSON cuts serialization and disk I/O on
        # the replan critical path without changing payload semantics.
        pretty=False,
        ensure_ascii=False,
        label=lambda path, _payload: path.parent.name,
        max_workers=int(workers),
    )
    commit_report = tx.commit(max_workers=int(workers), dry_run=False, skip_if_unchanged=True)
    results = [
        {
            "path": str(row.get("path") or ""),
            "name": str(row.get("name") or Path(str(row.get("path") or "")).name),
            "written": bool(row.get("written")),
            "skipped": bool(row.get("skipped")),
            **({"error": row.get("error")} if row.get("error") else {}),
        }
        for row in (commit_report.get("entries") or [])
        if isinstance(row, dict)
    ]
    _mark_attack_flight_path_writes(prepared_entries, results)
    return results


def _max_waypoint_id_in_flight_path_payload(path: Path, payload: Dict[str, Any]) -> Optional[int]:
    if Path(path).parent.name != "FlightPath" or not isinstance(payload, dict):
        return None
    max_waypoint_id: Optional[int] = None
    for list_key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        waypoint_list = payload.get(list_key)
        if not isinstance(waypoint_list, list):
            continue
        for waypoint in waypoint_list:
            if not isinstance(waypoint, dict):
                continue
            waypoint_id = _to_int(waypoint.get("waypointID"))
            if waypoint_id is None or waypoint_id <= 0:
                continue
            if max_waypoint_id is None or int(waypoint_id) > max_waypoint_id:
                max_waypoint_id = int(waypoint_id)
    return max_waypoint_id


def _mark_attack_flight_path_writes(
    entries: List[Tuple[Path, Dict[str, Any]]],
    results: List[Dict[str, Any]],
) -> None:
    if str(os.environ.get("REPLAN_ATTACK_MARK_WAYPOINT_WRITES", "1") or "").strip().lower() in {
        "0",
        "false",
        "no",
        "off",
    }:
        return
    wrote_flight_path = False
    max_waypoint_id: Optional[int] = None
    for (path, payload), row in zip(entries or [], results or []):
        if not isinstance(row, dict) or not bool(row.get("written")):
            continue
        if Path(path).parent.name != "FlightPath":
            continue
        wrote_flight_path = True
        payload_max = _max_waypoint_id_in_flight_path_payload(Path(path), payload)
        if payload_max is not None and (max_waypoint_id is None or int(payload_max) > max_waypoint_id):
            max_waypoint_id = int(payload_max)
    if not wrote_flight_path:
        return
    try:
        mark_waypoint_files_written(max_waypoint_id=max_waypoint_id)
    except Exception:
        pass


def _now_timestamp_ms() -> int:
    epoch = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.now(timezone.utc) - epoch).total_seconds() * 1000)
