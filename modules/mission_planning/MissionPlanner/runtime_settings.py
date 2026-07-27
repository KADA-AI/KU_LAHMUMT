from __future__ import annotations

import csv
import copy
import json
import math
import threading
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict

from modules.common.settings_paths import uav_params_path


_CACHE_LOCK = threading.Lock()
_CACHE_SIG: tuple[int, int] | None = None
_CACHE_DATA: Dict[str, Any] | None = None
_FOV_DB_CACHE_SIG: tuple[str, int, int] | None = None
_FOV_DB_MAX_WIDTH: float | None = None
_FOV_DB_ROWS_CACHE_SIG: tuple[str, int, int] | None = None
_FOV_DB_ROWS: list[Dict[str, float]] | None = None
_FOV_DB_ROWS_BY_PATH_SIG: dict[tuple[str, int, int], list[Dict[str, float]]] = {}
_THREAD_LOCAL = threading.local()

DEFAULT_AREA_SWEEP_MODE = "vertical"
DEFAULT_AREA_SPLIT_MODE = "single_stage"
DEFAULT_UAV_PLAN_MODE = "dub_path"
MIN_LINE_FOV_DEG = 1.2
MANUAL_FOV_ROLLBACK_KEY = "_manual_fov_rollback_values"
MANUAL_FOV_SYNC_KEYS = (
    "line_override_fov_deg",
    "area_override_fov_deg",
    "area_nadir_override_fov_deg",
    "recon_override_fixed_fov_deg",
    "point_fov_deg",
    "entry_hold_fov_deg",
)
MANUAL_FOV_DERIVED_KEYS = (
    "line_custom_fov_deg",
    "area_custom_fov_deg",
    "area_nadir_fov_deg",
    "recon_area_fixed_fov_deg",
)
UAV_SPEED_WEIGHT_KEY = "uav_speed_weight"
UAV_SPEED_MIN_MPS = 30.0
UAV_SPEED_MAX_MPS = 58.0
DEFAULT_FOV_DB_RELATIVE_PATH = "resource/db/fov_db.csv"
DEFAULT_AREA_REVIEW_MAX_SEGMENT_M = 1500.0
DEFAULT_FOV_DB_SMALLER_FOV_STEPS = 4
DEFAULT_AREA_FOV_DB_SMALLER_FOV_STEPS = 3
LEGACY_DEFAULT_FOV_DB_SMALLER_FOV_STEPS = 3
LEGACY_DEFAULT_AREA_FOV_DB_SMALLER_FOV_STEPS = 1

DEFAULT_RUNTIME_VALUES: Dict[str, Any] = {
    "camera_adjust_enabled": False,
    "camera_adjust_percent": 10.0,
    MANUAL_FOV_ROLLBACK_KEY: {},
    "manual_fov_global_sync_enabled": True,
    "global_manual_fov_deg": 2.4,
    "capture_mission_param_mode": "common",
    "global_density_scale": 1.2,
    "global_search_speed_weight": 1.0,
    "global_route_offset_scale": 1.0,
    "line_override_enabled": False,
    "area_override_enabled": False,
    "area_nadir_override_enabled": False,
    "recon_override_enabled": False,
    "line_override_fov_deg": 2.4,
    "line_override_density_scale": 1.2,
    "line_override_search_speed_weight": 1.0,
    "line_override_route_offset_scale": 1.0,
    "area_override_fov_deg": 2.4,
    "area_override_density_scale": 1.2,
    "area_override_search_speed_weight": 1.0,
    "area_override_route_offset_scale": 1.0,
    "area_nadir_override_fov_deg": 31.2,
    "recon_override_split_width_m": 600.0,
    "recon_override_fixed_fov_deg": 15.0,
    "recon_override_sweep_separation_scale": 0.50,
    "search_speed_weight": 1.0,
    "area_search_speed_weight": 1.0,
    "cruise_speed_mps": 40.0,
    "uav_speed_weight": 1.0,
    "turn_step_deg": 15.0,
    "altitude_m": 1000,
    "altitude_layer_step_m": 10.0,
    "capture_horizontal_overlap_ratio": 0.20,
    "capture_vertical_overlap_ratio": 0.20,
    "capture_fps": 30.0,
    "capture_scan_min_aircraft_speed_mps": 40.0,
    "capture_scan_max_aircraft_speed_mps": 55.0,
    "capture_scan_preferred_interval_s": 1.0,
    "capture_spacing_scale": 1.0,
    "capture_area_spacing_scale": 1.0,
    "capture_image_width_px": 1920.0,
    "capture_image_height_px": 1080.0,
    "line_custom_fov_deg": 2.4,
    "area_custom_fov_deg": 2.4,
    "area_nadir_fov_deg": 31.2,
    "point_fov_deg": 31.2,
    "entry_hold_fov_deg": 10.0,
    "area_output_fov_scale": 1.0,
    "default_sweep_separation_m": 1000.0,
    "fov_db_sep_safety_factor": 1.7,
    "fov_db_path": DEFAULT_FOV_DB_RELATIVE_PATH,
    "db_fov_weight": 1.0,
    "fov_db_smaller_fov_steps": DEFAULT_FOV_DB_SMALLER_FOV_STEPS,
    "area_fov_db_smaller_fov_steps": DEFAULT_AREA_FOV_DB_SMALLER_FOV_STEPS,
    "line_density_scale": 1.2,
    "area_density_scale": 1.2,
    "next_collab_area_density_scale": 2.4,
    "area_search_speed_max_mps": 800.0,
    "line_route_offset_scale": 1.0,
    "area_route_offset_scale": 1.0,
    "area_first_packet_search_speed_scale": 1.2,
    "area_first_packet_sweep_group_scale": 1.0,
    "uav_wp_interval_m": 2000.0,
    "area_wp_interval_m": 1000.0,
    "lah_wp_interval_m": 3000.0,
    "dubins_turn_radius_m": 500.0,
    "default_search_speed_multiplier": 16.0,
    "uav_climb_rate_mps": 5.0,
    "sweep_merge_heading_deg": 5.0,
    "line_sweep_interpolation_enabled": False,
    "sweep_line_interp_points": 3,
    "sweep_interp_max_gap_m": 500.0,
    "ground_required_sample_step_m": 120.0,
    "dense_linesearch_ground_sample_step_m": 240.0,
    "dense_linesearch_ground_sample_min_interp_points": 3,
    "linesearch_inner_parallel_min_coords": 512,
    "linesearch_inner_parallel_workers": 2,
    "formation_follower_postprocess_parallel_enabled": True,
    "formation_follower_postprocess_parallel_min_followers": 2,
    "formation_follower_postprocess_workers": 2,
    "min_sweep_len_m": 3.0,
    "min_route_spacing_m": 200.0,
    "line_search_speed_min_transit_m": 1.0,
    "line_search_speed_max_mps": 0.0,
    "enhanced_area_review_max_segment_m": DEFAULT_AREA_REVIEW_MAX_SEGMENT_M,
    "area_review_max_segment_force_enabled": False,
    "recon_area_review_max_split_count": 0,
    "recon_area_review_min_segment_m": 0.0,
    "post_attack_return_first_fov_scale": 0.70,
    "enhanced_auto_fov_from_db": True,
    "area_dubins_entry_links_enabled": True,
    "recon_area_split_width_m": 600.0,
    "recon_area_fixed_fov_deg": 15.0,
    "recon_sweep_separation_scale": 0.50,
    "recon_linesearch_interp_points_enabled": True,
    "recon_linesearch_interp_points": 2,
    "replan_collab_reexecute_schedule_delay_ms": 30,
    "replan_next_collab_schedule_delay_ms": 1,
    "entry_hold_gimbal_pitch": -90.0,
    "entry_hold_gimbal_yaw": 0.0,
    "loiter_radius_m": 800.0,
    "loiter_direction": 1,
    "loiter_time_s": 30.0,
    "loiter_speed_mps": 30.0,
    "next_collab_default_entry_strategy": "turn_projection",
    "next_collab_sweep_step_ratio": 0.60,
    "next_collab_entry_tprime_target_sep_ratio": 0.30,
    "next_collab_entry_tprime_ratio_scale": 0.50,
    "next_collab_area_path0_trigger_sep_m": 1500.0,
    "next_collab_area_path0_target_sep_ratio": 0.20,
    "next_collab_turn_radius_scale": 1.20,
    "next_collab_takeover_first_step_ratio": 0.40,
    "next_collab_area_fov_scale": 1.00,
    "next_collab_area_search_speed_scale": 1.30,
    "next_collab_area_gsd_margin_ratio": 0.90,
    "next_collab_area_scan_completion_speed_scale": 1.10,
    "next_collab_area_reciprocal_pass_enabled": False,
    "next_collab_area_reciprocal_min_terrain_score": 0.45,
    "next_collab_area_reciprocal_min_relief_m": 100.0,
    "next_collab_area_reciprocal_min_robust_relief_m": 75.0,
    "next_collab_area_reciprocal_min_grade_p90": 0.08,
    "next_collab_area_reciprocal_terrain_sample_cap": 192,
    "next_collab_area_reciprocal_turn_gate_radius_scale": 1.0,
    "next_collab_area_reciprocal_compact_gate_scale": 0.50,
    "next_collab_area_reciprocal_max_turn_waypoints": 3,
    "next_collab_area_reciprocal_max_extra_search_coords": 50000,
    "next_collab_area_activation_prediction_enabled": True,
    "next_collab_area_activation_prediction_confidence_min": 0.70,
    "next_collab_area_activation_prediction_max_age_s": 2.0,
    "next_collab_area_activation_straight_turn_rate_max_dps": 1.0,
    "next_collab_area_activation_straight_min_samples": 4,
    "next_collab_auto_sweep_points": False,
    "next_collab_sweep_points_per_leg": 3,
    "next_collab_line_db_width_weight": 0.30,
    "next_collab_line_db_sep_weight": 0.25,
    "next_collab_line_db_fov_weight": 0.45,
    "next_collab_first_line_fov_scale": 1.35,
    "next_collab_first_line_fov_max_deg": 15.4,
    "next_collab_replacement_path_build_workers": 1,
    "next_collab_line_replacement_path_build_workers": 3,
    "replan_sweep_speed_scale": 1.3,
    "replan_variant_parallel_enabled": True,
    "replan_current_remaining_variant_parallel_enabled": True,
    "replan_reexecute_current_fast_path_enabled": True,
    "replan_reexecute_line_recon_hybrid_share_enabled": True,
    "replan_variant_workers": 3,
    "replan_variant_waypoint_block_size": 5000,
    "replan_recon_worker_cap": 0,
    "replan_current_remaining_precompute_workers": 2,
    "replan_store_prepare_workers": 2,
    "replan_store_prepare_out_of_order": True,
    "replan_store_commit_workers": 2,
    "replan_store_json_write_workers": 2,
    "replan_store_path_id_cross_variant_bulk": True,
    "replan_store_snapshot_post_delivery": True,
    "replan_0303_aircraft_workers": 3,
    "replan_0303_aircraft_process_parallel_enabled": False,
    "replan_0303_aircraft_process_workers": 3,
    "replan_0303_persistent_process_pool_enabled": True,
    "replan_0303_dependency_parallel_enabled": True,
    "replan_0303_dependency_workers": 3,
    "replan_0303_altitude_precompute_enabled": True,
    "ground_required_final_prepass_fresh_skip_enabled": True,
    "ground_required_process_cache_waiter_batch_local_fast_path_enabled": False,
    "ground_required_process_cache_waiter_batch_local_fast_path_grace_ms": 0.0,
    "replan_0303_recompute_line_search_speed": False,
    "dem_alt_cache_round_decimals": 7,
    "input_refresh_fast_dem_enabled": True,
    "input_refresh_dem_alt_cache_round_decimals": 0,
    "input_refresh_ground_required_sample_step_m": 4000.0,
    "input_refresh_dense_linesearch_ground_sample_step_m": 4000.0,
    "input_refresh_dense_linesearch_ground_sample_min_interp_points": 2,
    "input_refresh_ground_required_line_coord_fast_path_min_count": 2,
    "input_refresh_ground_required_line_coord_fast_path_step_tolerance": 5000.0,
    "input_refresh_preserve_initial_geometry_enabled": False,
    "input_refresh_flightpath_carry_forward_enabled": True,
    "input_refresh_0303_process_parallel_enabled": True,
    "input_refresh_0303_process_workers": 3,
    "next_collab_area_replacement_path_build_workers": 3,
    "filming_altitude_batch_enabled": True,
    "replan_dem_batch_enabled": True,
    "lah_path_mode": "linear",
    "lah_rl_hex_step": 50,
    "lah_rl_area_km": 10.0,
}
DEFAULT_PRIOR_MISSION_VALUES: Dict[str, Any] = {
    "tracking_loiter_seconds": 300,
    "default_loiter_seconds": 50,
    "reinsert_loiter_seconds": 100,
    "approach_base_offset_m": 250.0,
    "approach_far_offset_m": 450.0,
    "approach_far_trigger_distance_m": 400.0,
    "orientation_offset_m": 100.0,
    "approach_speed_mps": 40.0,
    "target_speed_mps": 30.0,
    "resume_search_speed_scale": 1.3,
}
ATTACK_TARGET_TYPE_NAMES: Dict[int, str] = {
    1: "전차",
    2: "장갑차",
    3: "방사포",
    4: "곡사포",
    5: "고정고사포",
    6: "군인",
}
ATTACK_WEAPON_TYPE_NAMES: Dict[int, str] = {
    0: "Unknown",
    1: "기관포",
    2: "유도탄",
    3: "로켓",
}
DEFAULT_ATTACK_TARGET_TYPE_PRIORITY: list[int] = [1, 2, 3, 4, 5, 6]
DEFAULT_ATTACK_MISSION_VALUES: Dict[str, Any] = {
    # LAH1 is the command/relay aircraft and is never a shooter, even if a
    # persisted candidate list is edited incorrectly.
    "command_aircraft_id": 1,
    "manned_candidate_ids": [2, 3],
    "target_type_priority": list(DEFAULT_ATTACK_TARGET_TYPE_PRIORITY),
    "entry_offset_m": 100.0,
    "resume_offset_m": 20.0,
    "weapon_type": 2,
    "weapon_for_target_type_1": 2,
    "weapon_for_target_type_2": 2,
    "weapon_for_target_type_3": 3,
    "weapon_for_target_type_4": 3,
    "weapon_for_target_type_5": 2,
    "weapon_for_target_type_6": 1,
    "lah_hold_seconds": 300,
    "lah_hold_near_resume_offset_m": 30.0,
    "resume_search_speed_scale": 1.3,
    "json_write_workers": 4,
    "fast_num_arc_rays": 180,
    "point_cache_max": 16,
    "attack_point_inprocess_enabled": 1,
    "attack_point_cache_friendly_decimals": 4,
    "attack_point_cache_target_decimals": 5,
    "attack_min_standoff_m": 3000.0,
    "attack_preferred_standoff_m": 9000.0,
    "attack_point_altitude_offset_m": 300.0,
    # Keep attack visibility identical to the shared planning/SIM terrain-LOS
    # contract.  Diverging values here previously produced 3 km+ LAH pop-ups
    # even when SIM already had a clear reciprocal sightline below 1 km.
    "attack_los_clearance_m": 1.0,
    "attack_los_target_height_m": 5.0,
    "attack_los_profile_step_m": 0.0,
    # Enemy-contact concealment/relay prelude (native DEM + live 0401 state).
    "tactical_cover_enabled": 1,
    "tactical_hide_deadline_s": 10.0,
    "tactical_reconnect_deadline_s": 60.0,
    "tactical_timing_guard_s": 1.0,
    # A discovered enemy beyond this range is dropped from the masking analysis
    # entirely, and the result then reports enemyVisibleCount=0 for a point with
    # no cover at all.  Detection saturates with exposure time rather than
    # distance - a 300 s hold is seen from well past weapon range (longest SIM
    # weapon is 2.6 km) - so this must cover wherever contacts are actually
    # discovered.  Live geometry put every aircraft 5.3-6.3 km from its enemy.
    # Measured cost of widening it: none; the analysed enemy count drives the
    # work, not the cap.
    "tactical_enemy_range_m": 5000.0,
    "tactical_communication_range_m": 10000.0,
    "tactical_relay_min_uav_links": 3,
    "tactical_relay_degraded_min_uav_links": 2,
    "tactical_attacker_min_uav_links": 1,
    "tactical_hide_agl_m": 50.0,
    "tactical_max_hide_agl_m": 1000.0,
    "tactical_analysis_max_dim": 340,
    "tactical_refinement_radius_m": 300.0,
    "tactical_max_precision_candidates": 24,
    "tactical_allow_best_effort": 1,
}
PERSISTED_RUNTIME_VALUE_KEYS = tuple(DEFAULT_RUNTIME_VALUES.keys())
PERSISTED_PRIOR_MISSION_KEYS = tuple(DEFAULT_PRIOR_MISSION_VALUES.keys())
PERSISTED_ATTACK_MISSION_KEYS = tuple(DEFAULT_ATTACK_MISSION_VALUES.keys())
PERSISTED_FLYOVER_KEYS = ("entry_offset", "dubins_prefix", "last_point", "all_wps")


def _project_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "run.py").is_file() and (
            parent / "modules" / "common" / "db_paths.py"
        ).is_file():
            return parent
    return current.parents[3]


def _module_root() -> Path:
    return (_project_root() / Path(__file__).resolve().parents[2].name).resolve()


def _legacy_settings_path() -> Path:
    return Path(__file__).resolve().parent / "uav_params.json"


def _ensure_root_settings_file(path: Path) -> None:
    legacy_candidates = (
        _project_root() / "settings" / "uav_params.json",
        _project_root() / "uav_params.json",
        _legacy_settings_path(),
    )
    try:
        if path.exists():
            return
        legacy_path = next((candidate for candidate in legacy_candidates if candidate.exists()), None)
        if legacy_path is None:
            return
        if legacy_path.resolve() == path.resolve():
            return
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(legacy_path.read_text(encoding="utf-8"), encoding="utf-8")
    except Exception:
        return


def settings_path() -> Path:
    path = uav_params_path()
    _ensure_root_settings_file(path)
    return path


def default_fov_db_path() -> Path:
    return _module_root() / "resource" / "db" / "fov_db.csv"


def _resolve_runtime_path(raw_path: Any, default_path: Path) -> Path:
    text = str(raw_path or "").strip()
    if not text:
        return Path(default_path)
    try:
        path = Path(text).expanduser()
    except Exception:
        return Path(default_path)
    if path.is_absolute():
        return path
    normalized = text.replace("\\", "/").lstrip("./")
    normalized_lower = normalized.lower()
    if normalized_lower.startswith("modules/resource/db/"):
        return _module_root() / normalized[len("modules/") :]
    if normalized_lower.startswith("resource/db/"):
        return _module_root() / normalized
    return _project_root() / path


def _settings_path_value(path: Path) -> str:
    try:
        resolved = Path(path).expanduser().resolve()
    except Exception:
        resolved = Path(path)
    try:
        module_relative = resolved.relative_to(_module_root())
        return (Path("modules") / module_relative).as_posix()
    except Exception:
        pass
    try:
        return resolved.relative_to(_project_root().resolve()).as_posix()
    except Exception:
        return str(resolved)


def get_runtime_fov_db_path(payload: Dict[str, Any] | None = None) -> Path:
    raw_path = get_runtime_value("fov_db_path", DEFAULT_FOV_DB_RELATIVE_PATH, payload)
    return _resolve_runtime_path(raw_path, default_fov_db_path())


def fov_db_path() -> Path:
    return get_runtime_fov_db_path()


def _path_sig(path: Path) -> tuple[str, int, int] | None:
    try:
        resolved = Path(path).resolve()
        stat = resolved.stat()
    except Exception:
        return None
    return str(resolved), int(stat.st_mtime_ns), int(stat.st_size)


def clear_runtime_settings_cache() -> None:
    global _CACHE_SIG, _CACHE_DATA, _FOV_DB_CACHE_SIG, _FOV_DB_MAX_WIDTH, _FOV_DB_ROWS_CACHE_SIG, _FOV_DB_ROWS
    with _CACHE_LOCK:
        _CACHE_SIG = None
        _CACHE_DATA = None
        _FOV_DB_CACHE_SIG = None
        _FOV_DB_MAX_WIDTH = None
        _FOV_DB_ROWS_CACHE_SIG = None
        _FOV_DB_ROWS = None
        _FOV_DB_ROWS_BY_PATH_SIG.clear()


def set_runtime_fov_db_path(path: str | Path) -> Path:
    selected_path = Path(path).expanduser()
    stored_value = _settings_path_value(selected_path)
    cfg_path = settings_path()
    try:
        payload = json.loads(cfg_path.read_text(encoding="utf-8"))
    except Exception:
        payload = {}
    if not isinstance(payload, dict):
        payload = {}
    values = payload.get("values")
    if not isinstance(values, dict):
        values = {}
    values["fov_db_path"] = stored_value
    payload["values"] = values
    cfg_path.parent.mkdir(parents=True, exist_ok=True)
    cfg_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    clear_runtime_settings_cache()
    return get_runtime_fov_db_path(payload)


def load_runtime_settings() -> Dict[str, Any]:
    override = getattr(_THREAD_LOCAL, "override_payload", None)
    if isinstance(override, dict):
        return copy.deepcopy(override)
    global _CACHE_SIG, _CACHE_DATA
    path = settings_path()
    try:
        stat = path.stat()
        sig = (int(stat.st_mtime_ns), int(stat.st_size))
    except Exception:
        sig = None

    if sig is not None:
        with _CACHE_LOCK:
            if _CACHE_SIG == sig and isinstance(_CACHE_DATA, dict):
                return copy.deepcopy(_CACHE_DATA)

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        payload = {}
    data = canonicalize_runtime_payload(payload if isinstance(payload, dict) else None)
    with _CACHE_LOCK:
        _CACHE_SIG = sig
        _CACHE_DATA = copy.deepcopy(data)
    return data


def get_runtime_override() -> Dict[str, Any] | None:
    override = getattr(_THREAD_LOCAL, "override_payload", None)
    return copy.deepcopy(override) if isinstance(override, dict) else None


def set_runtime_override(payload: Dict[str, Any] | None) -> None:
    if isinstance(payload, dict):
        _THREAD_LOCAL.override_payload = canonicalize_runtime_payload(copy.deepcopy(payload))
    else:
        try:
            delattr(_THREAD_LOCAL, "override_payload")
        except Exception:
            pass


@contextmanager
def runtime_override(payload: Dict[str, Any] | None):
    prev = getattr(_THREAD_LOCAL, "override_payload", None)
    try:
        set_runtime_override(payload)
        yield
    finally:
        if isinstance(prev, dict):
            _THREAD_LOCAL.override_payload = prev
        else:
            try:
                delattr(_THREAD_LOCAL, "override_payload")
            except Exception:
                pass


def canonicalize_runtime_payload(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    base: Dict[str, Any] = {
        "values": copy.deepcopy(DEFAULT_RUNTIME_VALUES),
        "flyover": {key: False for key in PERSISTED_FLYOVER_KEYS},
        "prior_mission": copy.deepcopy(DEFAULT_PRIOR_MISSION_VALUES),
        "attack_mission": copy.deepcopy(DEFAULT_ATTACK_MISSION_VALUES),
    }
    if not isinstance(payload, dict):
        payload = {}

    def _as_float(value: Any, default: float) -> float:
        try:
            return float(value)
        except Exception:
            return float(default)

    def _as_bool(value: Any, default: bool) -> bool:
        try:
            return bool(value)
        except Exception:
            return bool(default)

    def _differs(value: Any, reference: float, tol: float = 1e-6) -> bool:
        try:
            return abs(float(value) - float(reference)) > tol
        except Exception:
            return False

    raw_values = payload.get("values")
    if isinstance(raw_values, dict):
        legacy_line_fov = raw_values.get("fov_deg")
        for key in PERSISTED_RUNTIME_VALUE_KEYS:
            if key in raw_values:
                base["values"][key] = raw_values[key]
        values = base["values"]
        if "global_manual_fov_deg" not in raw_values:
            values["global_manual_fov_deg"] = _as_float(
                raw_values.get("line_custom_fov_deg", legacy_line_fov),
                _as_float(values.get("global_manual_fov_deg"), 2.4),
            )
        if "global_density_scale" not in raw_values:
            values["global_density_scale"] = _as_float(
                raw_values.get("line_density_scale"),
                _as_float(values.get("global_density_scale"), 1.2),
            )
        if "global_search_speed_weight" not in raw_values:
            values["global_search_speed_weight"] = _as_float(
                raw_values.get("search_speed_weight"),
                _as_float(values.get("global_search_speed_weight"), 1.0),
            )
        if "global_route_offset_scale" not in raw_values:
            values["global_route_offset_scale"] = _as_float(
                raw_values.get("line_route_offset_scale"),
                _as_float(values.get("global_route_offset_scale"), 1.0),
            )
        if "line_override_fov_deg" not in raw_values:
            values["line_override_fov_deg"] = _as_float(
                raw_values.get("line_custom_fov_deg", legacy_line_fov),
                _as_float(values.get("global_manual_fov_deg"), 2.4),
            )
        if "line_override_density_scale" not in raw_values:
            values["line_override_density_scale"] = _as_float(
                raw_values.get("line_density_scale"),
                _as_float(values.get("global_density_scale"), 1.2),
            )
        if "line_override_search_speed_weight" not in raw_values:
            values["line_override_search_speed_weight"] = _as_float(
                raw_values.get("search_speed_weight"),
                _as_float(values.get("global_search_speed_weight"), 1.0),
            )
        if "line_override_route_offset_scale" not in raw_values:
            values["line_override_route_offset_scale"] = _as_float(
                raw_values.get("line_route_offset_scale"),
                _as_float(values.get("global_route_offset_scale"), 1.0),
            )
        if "area_override_fov_deg" not in raw_values:
            values["area_override_fov_deg"] = _as_float(
                raw_values.get("area_custom_fov_deg", legacy_line_fov),
                _as_float(values.get("global_manual_fov_deg"), 2.4),
            )
        if "area_override_density_scale" not in raw_values:
            values["area_override_density_scale"] = _as_float(
                raw_values.get("area_density_scale"),
                _as_float(values.get("global_density_scale"), 1.2),
            )
        if "area_override_search_speed_weight" not in raw_values:
            values["area_override_search_speed_weight"] = _as_float(
                raw_values.get("area_search_speed_weight"),
                _as_float(values.get("global_search_speed_weight"), 1.0),
            )
        if "area_override_route_offset_scale" not in raw_values:
            values["area_override_route_offset_scale"] = _as_float(
                raw_values.get("area_route_offset_scale"),
                _as_float(values.get("global_route_offset_scale"), 1.0),
            )
        if "area_nadir_override_fov_deg" not in raw_values:
            values["area_nadir_override_fov_deg"] = _as_float(
                raw_values.get("area_nadir_fov_deg"),
                _as_float(values.get("area_override_fov_deg"), _as_float(values.get("global_manual_fov_deg"), 31.2)),
            )
        if "recon_override_split_width_m" not in raw_values:
            values["recon_override_split_width_m"] = _as_float(
                raw_values.get("recon_area_split_width_m"),
                _as_float(values.get("recon_override_split_width_m"), 600.0),
            )
        if "recon_override_fixed_fov_deg" not in raw_values:
            values["recon_override_fixed_fov_deg"] = _as_float(
                raw_values.get("recon_area_fixed_fov_deg"),
                _as_float(values.get("area_override_fov_deg"), _as_float(values.get("global_manual_fov_deg"), 15.0)),
            )
        if "recon_override_sweep_separation_scale" not in raw_values:
            values["recon_override_sweep_separation_scale"] = _as_float(
                raw_values.get("recon_sweep_separation_scale"),
                _as_float(values.get("recon_override_sweep_separation_scale"), 0.50),
            )
        if "line_override_enabled" not in raw_values:
            values["line_override_enabled"] = (
                _differs(raw_values.get("line_custom_fov_deg", legacy_line_fov), values["global_manual_fov_deg"])
                or _differs(raw_values.get("line_density_scale"), values["global_density_scale"])
                or _differs(raw_values.get("search_speed_weight"), values["global_search_speed_weight"])
                or _differs(raw_values.get("line_route_offset_scale"), values["global_route_offset_scale"])
            )
        if "area_override_enabled" not in raw_values:
            values["area_override_enabled"] = (
                _differs(raw_values.get("area_custom_fov_deg", legacy_line_fov), values["global_manual_fov_deg"])
                or _differs(raw_values.get("area_density_scale"), values["global_density_scale"])
                or _differs(raw_values.get("area_search_speed_weight"), values["global_search_speed_weight"])
                or _differs(raw_values.get("area_route_offset_scale"), values["global_route_offset_scale"])
            )
        if "capture_mission_param_mode" not in raw_values and (
            _as_bool(values.get("line_override_enabled"), False)
            or _as_bool(values.get("area_override_enabled"), False)
        ):
            values["capture_mission_param_mode"] = "manual"
        if "area_nadir_override_enabled" not in raw_values:
            area_fov_reference = (
                _as_float(values.get("area_override_fov_deg"), values["global_manual_fov_deg"])
                if _as_bool(values.get("area_override_enabled"), False)
                else _as_float(values.get("global_manual_fov_deg"), 2.4)
            )
            values["area_nadir_override_enabled"] = _differs(
                raw_values.get("area_nadir_fov_deg"),
                area_fov_reference,
            )
        if "recon_override_enabled" not in raw_values:
            area_fov_reference = (
                _as_float(values.get("area_override_fov_deg"), values["global_manual_fov_deg"])
                if _as_bool(values.get("area_override_enabled"), False)
                else _as_float(values.get("global_manual_fov_deg"), 2.4)
            )
            values["recon_override_enabled"] = (
                _differs(raw_values.get("recon_area_fixed_fov_deg"), area_fov_reference)
                or _differs(raw_values.get("recon_sweep_separation_scale"), 1.0)
                or _differs(raw_values.get("recon_area_split_width_m"), 600.0)
            )
        if "enhanced_auto_fov_from_db" not in raw_values and "enhanced_area_auto_fov_from_db" in raw_values:
            try:
                base["values"]["enhanced_auto_fov_from_db"] = bool(raw_values.get("enhanced_area_auto_fov_from_db"))
            except Exception:
                pass

    values = base["values"]
    global_fov = _as_float(values.get("global_manual_fov_deg"), 2.4)
    global_density = _as_float(values.get("global_density_scale"), 1.2)
    global_speed = _as_float(values.get("global_search_speed_weight"), 1.0)
    global_route = _as_float(values.get("global_route_offset_scale"), 1.0)

    line_override_enabled = _as_bool(values.get("line_override_enabled"), False)
    area_override_enabled = _as_bool(values.get("area_override_enabled"), False)
    area_nadir_override_enabled = _as_bool(values.get("area_nadir_override_enabled"), False)
    recon_override_enabled = _as_bool(values.get("recon_override_enabled"), False)
    capture_mission_param_mode = str(values.get("capture_mission_param_mode", "common") or "common").strip().lower()
    if capture_mission_param_mode not in {"common", "manual"}:
        capture_mission_param_mode = "common"
    values["capture_mission_param_mode"] = capture_mission_param_mode
    capture_param_manual = capture_mission_param_mode == "manual"
    line_param_override_active = capture_param_manual and line_override_enabled
    area_param_override_active = capture_param_manual and area_override_enabled

    line_override_fov = max(MIN_LINE_FOV_DEG, _as_float(values.get("line_override_fov_deg"), global_fov))
    values["line_override_fov_deg"] = line_override_fov
    line_fov = line_override_fov if line_override_enabled else max(MIN_LINE_FOV_DEG, global_fov)
    area_fov = _as_float(values.get("area_override_fov_deg"), global_fov) if area_override_enabled else global_fov
    values["line_custom_fov_deg"] = line_fov
    values["area_custom_fov_deg"] = area_fov
    values["line_density_scale"] = (
        _as_float(values.get("line_override_density_scale"), global_density) if line_param_override_active else global_density
    )
    values["area_density_scale"] = (
        _as_float(values.get("area_override_density_scale"), global_density) if area_param_override_active else global_density
    )
    values["search_speed_weight"] = (
        _as_float(values.get("line_override_search_speed_weight"), global_speed) if line_param_override_active else global_speed
    )
    values["area_search_speed_weight"] = (
        _as_float(values.get("area_override_search_speed_weight"), global_speed) if area_param_override_active else global_speed
    )
    values["line_route_offset_scale"] = (
        _as_float(values.get("line_override_route_offset_scale"), global_route) if line_param_override_active else global_route
    )
    values["area_route_offset_scale"] = (
        _as_float(values.get("area_override_route_offset_scale"), global_route) if area_param_override_active else global_route
    )
    values["area_nadir_fov_deg"] = (
        _as_float(values.get("area_nadir_override_fov_deg"), area_fov) if area_nadir_override_enabled else area_fov
    )
    values["recon_area_fixed_fov_deg"] = (
        _as_float(values.get("recon_override_fixed_fov_deg"), area_fov) if recon_override_enabled else area_fov
    )
    values["recon_sweep_separation_scale"] = (
        _as_float(values.get("recon_override_sweep_separation_scale"), 1.0) if recon_override_enabled else 1.0
    )
    values["recon_area_split_width_m"] = (
        _as_float(values.get("recon_override_split_width_m"), 600.0) if recon_override_enabled else 600.0
    )
    values["fov_db_sep_safety_factor"] = max(
        1.0,
        _as_float(values.get("fov_db_sep_safety_factor"), 1.7),
    )
    raw_fov_db_steps = values.get("fov_db_smaller_fov_steps")
    raw_area_fov_db_steps = values.get("area_fov_db_smaller_fov_steps")
    try:
        if int(float(raw_fov_db_steps)) == LEGACY_DEFAULT_FOV_DB_SMALLER_FOV_STEPS:
            raw_fov_db_steps = DEFAULT_FOV_DB_SMALLER_FOV_STEPS
    except Exception:
        raw_fov_db_steps = DEFAULT_FOV_DB_SMALLER_FOV_STEPS
    try:
        if int(float(raw_area_fov_db_steps)) == LEGACY_DEFAULT_AREA_FOV_DB_SMALLER_FOV_STEPS:
            raw_area_fov_db_steps = DEFAULT_AREA_FOV_DB_SMALLER_FOV_STEPS
    except Exception:
        raw_area_fov_db_steps = DEFAULT_AREA_FOV_DB_SMALLER_FOV_STEPS
    values["fov_db_smaller_fov_steps"] = max(
        3,
        min(10, int(_as_float(raw_fov_db_steps, DEFAULT_FOV_DB_SMALLER_FOV_STEPS))),
    )
    values["area_fov_db_smaller_fov_steps"] = max(
        3,
        min(10, int(_as_float(raw_area_fov_db_steps, DEFAULT_AREA_FOV_DB_SMALLER_FOV_STEPS))),
    )
    values["next_collab_area_density_scale"] = max(
        0.2,
        min(
            10.0,
            _as_float(
                values.get("next_collab_area_density_scale"),
                max(_as_float(values.get("area_density_scale"), 1.2), 2.4),
            ),
        ),
    )
    values["next_collab_area_gsd_margin_ratio"] = max(
        0.10,
        min(1.0, _as_float(values.get("next_collab_area_gsd_margin_ratio"), 0.90)),
    )
    values["next_collab_area_search_speed_scale"] = max(
        0.10,
        min(5.0, _as_float(values.get("next_collab_area_search_speed_scale"), 1.30)),
    )
    values["next_collab_area_scan_completion_speed_scale"] = max(
        1.0,
        min(
            1.5,
            _as_float(
                values.get("next_collab_area_scan_completion_speed_scale"),
                1.10,
            ),
        ),
    )
    values["next_collab_area_reciprocal_pass_enabled"] = _as_bool(
        values.get("next_collab_area_reciprocal_pass_enabled"),
        False,
    )
    values["next_collab_area_reciprocal_min_terrain_score"] = max(
        0.0,
        min(
            1.0,
            _as_float(
                values.get("next_collab_area_reciprocal_min_terrain_score"),
                0.45,
            ),
        ),
    )
    values["next_collab_area_reciprocal_min_relief_m"] = max(
        0.0,
        min(
            5000.0,
            _as_float(values.get("next_collab_area_reciprocal_min_relief_m"), 100.0),
        ),
    )
    values["next_collab_area_reciprocal_min_robust_relief_m"] = max(
        0.0,
        min(
            5000.0,
            _as_float(
                values.get("next_collab_area_reciprocal_min_robust_relief_m"),
                75.0,
            ),
        ),
    )
    values["next_collab_area_reciprocal_min_grade_p90"] = max(
        0.0,
        min(
            2.0,
            _as_float(values.get("next_collab_area_reciprocal_min_grade_p90"), 0.08),
        ),
    )
    values["next_collab_area_reciprocal_terrain_sample_cap"] = max(
        24,
        min(
            768,
            int(
                _as_float(
                    values.get("next_collab_area_reciprocal_terrain_sample_cap"),
                    192,
                )
            ),
        ),
    )
    values["next_collab_area_reciprocal_turn_gate_radius_scale"] = max(
        1.0,
        min(
            1.5,
            _as_float(
                values.get("next_collab_area_reciprocal_turn_gate_radius_scale"),
                1.0,
            ),
        ),
    )
    values["next_collab_area_reciprocal_compact_gate_scale"] = max(
        0.45,
        min(
            0.75,
            _as_float(
                values.get("next_collab_area_reciprocal_compact_gate_scale"),
                0.50,
            ),
        ),
    )
    # The reciprocal return contract is deliberately fixed to two outside
    # fly-by targets plus one non-filming re-entry target.
    values["next_collab_area_reciprocal_max_turn_waypoints"] = 3
    values["next_collab_area_reciprocal_max_extra_search_coords"] = max(
        1000,
        min(
            500000,
            int(
                _as_float(
                    values.get("next_collab_area_reciprocal_max_extra_search_coords"),
                    50000,
                )
            ),
        ),
    )
    values["next_collab_area_activation_prediction_enabled"] = _as_bool(
        values.get("next_collab_area_activation_prediction_enabled"),
        True,
    )
    values["next_collab_area_activation_prediction_confidence_min"] = max(
        0.0,
        min(
            1.0,
            _as_float(
                values.get("next_collab_area_activation_prediction_confidence_min"),
                0.70,
            ),
        ),
    )
    values["next_collab_area_activation_prediction_max_age_s"] = max(
        0.0,
        min(
            10.0,
            _as_float(
                values.get("next_collab_area_activation_prediction_max_age_s"),
                2.0,
            ),
        ),
    )
    values["next_collab_area_activation_straight_turn_rate_max_dps"] = max(
        0.1,
        min(
            5.0,
            _as_float(
                values.get("next_collab_area_activation_straight_turn_rate_max_dps"),
                1.0,
            ),
        ),
    )
    values["next_collab_area_activation_straight_min_samples"] = max(
        2,
        min(
            20,
            int(
                _as_float(
                    values.get("next_collab_area_activation_straight_min_samples"),
                    4,
                )
            ),
        ),
    )

    manual_fov_keys = tuple(MANUAL_FOV_SYNC_KEYS) + tuple(MANUAL_FOV_DERIVED_KEYS)
    snapshot_raw = values.get(MANUAL_FOV_ROLLBACK_KEY)
    snapshot = copy.deepcopy(snapshot_raw) if isinstance(snapshot_raw, dict) else {}
    auto_fov_from_db = _as_bool(values.get("enhanced_auto_fov_from_db"), True)
    manual_fov_global_sync_enabled = _as_bool(values.get("manual_fov_global_sync_enabled"), True)
    if not auto_fov_from_db and manual_fov_global_sync_enabled:
        if not snapshot:
            snapshot = {
                key: copy.deepcopy(values.get(key))
                for key in manual_fov_keys
                if key in values
            }
        manual_fov = max(0.1, min(120.0, _as_float(values.get("global_manual_fov_deg"), global_fov)))
        line_manual_fov = max(MIN_LINE_FOV_DEG, manual_fov)
        for key in MANUAL_FOV_SYNC_KEYS:
            values[key] = float(line_manual_fov if key == "line_override_fov_deg" else manual_fov)
        values["line_custom_fov_deg"] = float(line_manual_fov)
        values["area_custom_fov_deg"] = float(manual_fov)
        values["area_nadir_fov_deg"] = float(manual_fov)
        values["recon_area_fixed_fov_deg"] = float(manual_fov)
        values[MANUAL_FOV_ROLLBACK_KEY] = snapshot
    elif snapshot:
        for key in manual_fov_keys:
            if key in snapshot:
                values[key] = copy.deepcopy(snapshot[key])
        values[MANUAL_FOV_ROLLBACK_KEY] = {}

    values["line_override_fov_deg"] = max(
        MIN_LINE_FOV_DEG,
        min(120.0, _as_float(values.get("line_override_fov_deg"), MIN_LINE_FOV_DEG)),
    )
    values["line_custom_fov_deg"] = max(
        MIN_LINE_FOV_DEG,
        min(120.0, _as_float(values.get("line_custom_fov_deg"), values["line_override_fov_deg"])),
    )

    raw_flyover = payload.get("flyover")
    if isinstance(raw_flyover, dict):
        for key in PERSISTED_FLYOVER_KEYS:
            if key in raw_flyover:
                base["flyover"][key] = bool(raw_flyover.get(key))

    raw_prior = payload.get("prior_mission")
    if isinstance(raw_prior, dict):
        for key in PERSISTED_PRIOR_MISSION_KEYS:
            if key in raw_prior:
                base["prior_mission"][key] = copy.deepcopy(raw_prior[key])

    raw_attack = payload.get("attack_mission")
    if isinstance(raw_attack, dict):
        for key in PERSISTED_ATTACK_MISSION_KEYS:
            if key in raw_attack:
                base["attack_mission"][key] = copy.deepcopy(raw_attack[key])

    raw_values_for_migration = raw_values if isinstance(raw_values, dict) else {}
    if "replan_sweep_speed_scale" not in raw_values_for_migration:
        for source, old_key in (
            (raw_values_for_migration, "post_attack_collab_first_sweep_speed_scale"),
            (raw_attack if isinstance(raw_attack, dict) else {}, "collab_first_sweep_speed_scale"),
            (raw_prior if isinstance(raw_prior, dict) else {}, "collab_first_sweep_speed_scale"),
        ):
            if isinstance(source, dict) and old_key in source:
                base["values"]["replan_sweep_speed_scale"] = _as_float(
                    source.get(old_key),
                    _as_float(base["values"].get("replan_sweep_speed_scale"), 1.3),
                )
                break

    return base


def load_runtime_values(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    values = data.get("values") if isinstance(data.get("values"), dict) else {}
    return values if isinstance(values, dict) else {}


def load_runtime_flyover(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    flyover = data.get("flyover") if isinstance(data.get("flyover"), dict) else {}
    return flyover if isinstance(flyover, dict) else {}


def load_runtime_prior_values(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    prior = data.get("prior_mission") if isinstance(data.get("prior_mission"), dict) else {}
    return prior if isinstance(prior, dict) else {}


def load_runtime_attack_values(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    attack = data.get("attack_mission") if isinstance(data.get("attack_mission"), dict) else {}
    return attack if isinstance(attack, dict) else {}


def get_runtime_value(key: str, default: Any, payload: Dict[str, Any] | None = None) -> Any:
    return load_runtime_values(payload).get(key, default)


def get_runtime_float(key: str, default: float, payload: Dict[str, Any] | None = None) -> float:
    try:
        return float(get_runtime_value(key, default, payload))
    except Exception:
        return float(default)


def get_runtime_uav_speed_weight(
    default: float = 1.0,
    payload: Dict[str, Any] | None = None,
) -> float:
    try:
        value = float(get_runtime_value(UAV_SPEED_WEIGHT_KEY, default, payload))
    except Exception:
        value = float(default)
    if not math.isfinite(value) or value <= 0.0:
        return float(default) if float(default) > 0.0 else 1.0
    return float(value)


def clamp_runtime_uav_speed_mps(speed_mps: float) -> float:
    try:
        speed = float(speed_mps)
    except Exception:
        speed = float(UAV_SPEED_MIN_MPS)
    if not math.isfinite(speed):
        speed = float(UAV_SPEED_MIN_MPS)
    return max(float(UAV_SPEED_MIN_MPS), min(float(UAV_SPEED_MAX_MPS), float(speed)))


def clamp_runtime_uav_speed_kmh(speed_kmh: float) -> float:
    try:
        speed_mps = float(speed_kmh) / 3.6
    except Exception:
        speed_mps = float(UAV_SPEED_MIN_MPS)
    return round(clamp_runtime_uav_speed_mps(speed_mps) * 3.6, 2)


def apply_runtime_uav_speed_weight_mps(
    speed_mps: float,
    payload: Dict[str, Any] | None = None,
) -> float:
    try:
        speed = float(speed_mps)
    except Exception:
        return 0.0
    if not math.isfinite(speed) or speed <= 0.0:
        return 0.0
    weighted = speed * get_runtime_uav_speed_weight(1.0, payload)
    return float(clamp_runtime_uav_speed_mps(weighted))


def apply_runtime_uav_speed_weight_kmh(
    speed_kmh: float,
    payload: Dict[str, Any] | None = None,
) -> float:
    try:
        speed = float(speed_kmh)
    except Exception:
        return 0.0
    if not math.isfinite(speed) or speed <= 0.0:
        return 0.0
    return round(apply_runtime_uav_speed_weight_mps(speed / 3.6, payload) * 3.6, 2)


def get_runtime_int(key: str, default: int, payload: Dict[str, Any] | None = None) -> int:
    try:
        return int(float(get_runtime_value(key, default, payload)))
    except Exception:
        return int(default)


def get_runtime_bool(key: str, default: bool, payload: Dict[str, Any] | None = None) -> bool:
    try:
        return bool(get_runtime_value(key, default, payload))
    except Exception:
        return bool(default)


def get_runtime_area_auto_fov_from_db(
    default: bool,
    payload: Dict[str, Any] | None = None,
) -> bool:
    return get_runtime_bool("enhanced_auto_fov_from_db", default, payload)


def get_runtime_camera_adjust_enabled(
    default: bool = False,
    payload: Dict[str, Any] | None = None,
) -> bool:
    return get_runtime_bool("camera_adjust_enabled", default, payload)


def get_runtime_camera_adjust_percent(
    default: float = 10.0,
    payload: Dict[str, Any] | None = None,
) -> float:
    try:
        value = float(get_runtime_value("camera_adjust_percent", default, payload))
    except Exception:
        value = float(default)
    return max(-90.0, min(90.0, float(value)))


def get_runtime_camera_adjust_fov_scale(
    payload: Dict[str, Any] | None = None,
) -> float:
    if get_runtime_manual_fov_sync_active(payload):
        return 1.0
    if not get_runtime_camera_adjust_enabled(False, payload):
        return 1.0
    percent = get_runtime_camera_adjust_percent(10.0, payload)
    return max(0.1, 1.0 - (float(percent) / 100.0))


def get_runtime_altitude_layers_m(
    payload: Dict[str, Any] | None = None,
    *,
    count: int = 3,
) -> tuple[float, ...]:
    base_altitude_m = get_runtime_float("altitude_m", 1000.0, payload)
    step_m = get_runtime_float("altitude_layer_step_m", 10.0, payload)
    if not math.isfinite(base_altitude_m):
        base_altitude_m = 1000.0
    if not math.isfinite(step_m) or step_m < 0.0:
        step_m = 10.0
    layer_count = max(1, int(count or 1))
    return tuple(float(base_altitude_m) + (float(step_m) * idx) for idx in range(layer_count))


def apply_runtime_camera_adjusted_fov_deg(
    fov_deg: float,
    payload: Dict[str, Any] | None = None,
    *,
    minimum_fov_deg: float = 0.1,
    context: str = "",
) -> float:
    try:
        base_fov_deg = float(fov_deg)
    except Exception:
        return float(minimum_fov_deg)
    if base_fov_deg <= 0.0:
        return float(minimum_fov_deg)
    adjusted_fov_deg = base_fov_deg * get_runtime_camera_adjust_fov_scale(payload)
    adjusted_fov_deg = max(float(adjusted_fov_deg), float(minimum_fov_deg))
    _record_runtime_camera_fov_adjustment_log(base_fov_deg, adjusted_fov_deg, payload, context=context)
    return float(adjusted_fov_deg)


def _runtime_camera_fov_adjustment_logs() -> list[str]:
    logs = getattr(_THREAD_LOCAL, "camera_fov_adjustment_logs", None)
    if not isinstance(logs, list):
        logs = []
        _THREAD_LOCAL.camera_fov_adjustment_logs = logs
    return logs


def clear_runtime_camera_fov_adjustment_logs() -> None:
    try:
        _THREAD_LOCAL.camera_fov_adjustment_logs = []
    except Exception:
        pass


def pop_runtime_camera_fov_adjustment_logs() -> list[str]:
    try:
        logs = list(_runtime_camera_fov_adjustment_logs())
        _THREAD_LOCAL.camera_fov_adjustment_logs = []
        return logs
    except Exception:
        return []


def _record_runtime_camera_fov_adjustment_log(
    base_fov_deg: float,
    adjusted_fov_deg: float,
    payload: Dict[str, Any] | None = None,
    *,
    context: str = "",
) -> None:
    log_line = format_runtime_camera_fov_adjustment_log(
        base_fov_deg,
        adjusted_fov_deg,
        payload,
        context=context,
    )
    if not log_line:
        return
    try:
        logs = _runtime_camera_fov_adjustment_logs()
        if log_line not in logs:
            logs.append(log_line)
    except Exception:
        return


def format_runtime_camera_fov_adjustment_log(
    base_fov_deg: float,
    adjusted_fov_deg: float | None = None,
    payload: Dict[str, Any] | None = None,
    *,
    context: str = "",
) -> str | None:
    try:
        base = float(base_fov_deg)
    except Exception:
        return None
    if not math.isfinite(base) or base <= 0.0:
        return None
    if not get_runtime_camera_adjust_enabled(False, payload):
        return None
    try:
        adjusted = (
            float(adjusted_fov_deg)
            if adjusted_fov_deg is not None
            else float(apply_runtime_camera_adjusted_fov_deg(base, payload))
        )
    except Exception:
        return None
    if not math.isfinite(adjusted) or adjusted <= 0.0:
        return None
    if abs(float(adjusted) - float(base)) <= 1e-6:
        return None

    scale = float(adjusted) / float(base)
    change_percent = abs(1.0 - scale) * 100.0
    direction = "축소" if adjusted < base else "확대"
    setting_percent = get_runtime_camera_adjust_percent(10.0, payload)
    context_prefix = f"{str(context).strip()}: " if str(context or "").strip() else ""
    return (
        f"[FOV][CAMERA_ADJUST] {context_prefix}"
        f"기존 {base:.2f}deg -> {change_percent:.1f}% {direction} -> 선택 {adjusted:.2f}deg "
        f"(설정={setting_percent:+.1f}%, scale={scale:.3f})"
    )


def get_runtime_manual_fov_sync_active(payload: Dict[str, Any] | None = None) -> bool:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    values = load_runtime_values(data)
    try:
        auto_fov_from_db = bool(values.get("enhanced_auto_fov_from_db", True))
    except Exception:
        auto_fov_from_db = True
    try:
        sync_enabled = bool(values.get("manual_fov_global_sync_enabled", True))
    except Exception:
        sync_enabled = True
    return (not auto_fov_from_db) and sync_enabled


def get_runtime_manual_fov_deg(
    key: str,
    default: float,
    payload: Dict[str, Any] | None = None,
    *,
    camera_adjust: bool = True,
) -> float:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    if not get_runtime_manual_fov_sync_active(data):
        return float(default)
    values = load_runtime_values(data)
    try:
        fallback = float(values.get("global_manual_fov_deg", default))
    except Exception:
        fallback = float(default)
    try:
        fov_deg = float(values.get(key, fallback))
    except Exception:
        fov_deg = fallback
    minimum_fov_deg = MIN_LINE_FOV_DEG if key in {"line_custom_fov_deg", "line_override_fov_deg"} else 0.1
    if fov_deg <= 0.0:
        fov_deg = fallback if fallback > 0.0 else float(default)
    fov_deg = max(float(fov_deg), float(minimum_fov_deg))
    if camera_adjust:
        return apply_runtime_camera_adjusted_fov_deg(fov_deg, data, minimum_fov_deg=minimum_fov_deg)
    return float(fov_deg)


def get_runtime_effective_fov_deg(
    key: str,
    default: float,
    payload: Dict[str, Any] | None = None,
    *,
    minimum_fov_deg: float = 0.1,
) -> float:
    """Resolve a mission FOV with manual-global sync taking precedence."""
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    if get_runtime_manual_fov_sync_active(data):
        return get_runtime_manual_fov_deg(
            key,
            default,
            data,
            camera_adjust=True,
        )
    return apply_runtime_camera_adjusted_fov_deg(
        default,
        data,
        minimum_fov_deg=minimum_fov_deg,
    )


def get_runtime_camera_coverage_scale(
    base_fov_deg: float,
    payload: Dict[str, Any] | None = None,
    *,
    adjusted_fov_deg: float | None = None,
) -> float:
    try:
        base_deg = float(base_fov_deg)
    except Exception:
        return 1.0
    if base_deg <= 0.0:
        return 1.0

    try:
        effective_adjusted_deg = float(
            adjusted_fov_deg
            if adjusted_fov_deg is not None
            else apply_runtime_camera_adjusted_fov_deg(base_deg, payload)
        )
    except Exception:
        effective_adjusted_deg = float(base_deg)
    if effective_adjusted_deg <= 0.0:
        return 1.0

    try:
        base_half_rad = max(math.radians(base_deg) / 2.0, 1e-6)
        adjusted_half_rad = max(math.radians(effective_adjusted_deg) / 2.0, 1e-6)
        ratio = math.tan(adjusted_half_rad) / math.tan(base_half_rad)
    except Exception:
        return 1.0
    if not math.isfinite(ratio) or ratio <= 0.0:
        return 1.0
    return float(ratio)


def apply_runtime_camera_adjusted_search_speed(
    speed_value: float,
    base_fov_deg: float,
    payload: Dict[str, Any] | None = None,
    *,
    adjusted_fov_deg: float | None = None,
) -> float:
    try:
        speed = float(speed_value)
    except Exception:
        return 0.0
    if speed <= 0.0:
        return float(speed)
    coverage_scale = get_runtime_camera_coverage_scale(
        base_fov_deg,
        payload,
        adjusted_fov_deg=adjusted_fov_deg,
    )
    return max(float(speed) * float(coverage_scale), 0.1)


def get_runtime_str(key: str, default: str, payload: Dict[str, Any] | None = None) -> str:
    try:
        value = get_runtime_value(key, default, payload)
        if value is None:
            return str(default)
        return str(value)
    except Exception:
        return str(default)


def get_runtime_prior_value(key: str, default: Any, payload: Dict[str, Any] | None = None) -> Any:
    return load_runtime_prior_values(payload).get(key, default)


def get_runtime_prior_float(key: str, default: float, payload: Dict[str, Any] | None = None) -> float:
    try:
        return float(get_runtime_prior_value(key, default, payload))
    except Exception:
        return float(default)


def get_runtime_prior_int(key: str, default: int, payload: Dict[str, Any] | None = None) -> int:
    try:
        return int(float(get_runtime_prior_value(key, default, payload)))
    except Exception:
        return int(default)


def get_runtime_attack_value(key: str, default: Any, payload: Dict[str, Any] | None = None) -> Any:
    return load_runtime_attack_values(payload).get(key, default)


def get_runtime_attack_float(key: str, default: float, payload: Dict[str, Any] | None = None) -> float:
    try:
        return float(get_runtime_attack_value(key, default, payload))
    except Exception:
        return float(default)


def get_runtime_attack_int(key: str, default: int, payload: Dict[str, Any] | None = None) -> int:
    try:
        return int(float(get_runtime_attack_value(key, default, payload)))
    except Exception:
        return int(default)


def get_runtime_attack_weapon_type(default: int = 2, payload: Dict[str, Any] | None = None) -> int:
    try:
        value = int(float(get_runtime_attack_value("weapon_type", default, payload)))
    except Exception:
        value = int(default)
    return max(0, min(3, value))


def get_runtime_attack_target_type_priority(
    default: list[int] | tuple[int, ...] | None = None,
    payload: Dict[str, Any] | None = None,
) -> list[int]:
    fallback = list(default) if default is not None else list(DEFAULT_ATTACK_TARGET_TYPE_PRIORITY)
    raw = get_runtime_attack_value("target_type_priority", fallback, payload)
    if isinstance(raw, str):
        items = raw.replace(";", ",").split(",")
    elif isinstance(raw, (list, tuple)):
        items = list(raw)
    else:
        items = list(fallback)
    result: list[int] = []
    seen: set[int] = set()
    for item in list(items) + list(fallback):
        try:
            parsed = int(float(item))
        except Exception:
            continue
        if parsed < 1 or parsed > 6 or parsed in seen:
            continue
        seen.add(parsed)
        result.append(parsed)
    return result or list(DEFAULT_ATTACK_TARGET_TYPE_PRIORITY)


def get_runtime_attack_weapon_type_for_target_type(
    target_type: int | None,
    default: int = 2,
    payload: Dict[str, Any] | None = None,
) -> int:
    fallback = get_runtime_attack_weapon_type(default, payload)
    try:
        parsed_target_type = int(target_type) if target_type is not None else None
    except Exception:
        parsed_target_type = None
    if parsed_target_type is None or parsed_target_type not in ATTACK_TARGET_TYPE_NAMES:
        return fallback
    key = f"weapon_for_target_type_{parsed_target_type}"
    default_value = DEFAULT_ATTACK_MISSION_VALUES.get(key, fallback)
    try:
        value = int(float(get_runtime_attack_value(key, default_value, payload)))
    except Exception:
        value = int(default_value)
    return max(0, min(3, value))


def get_runtime_attack_int_list(
    key: str,
    default: list[int] | tuple[int, ...],
    payload: Dict[str, Any] | None = None,
) -> list[int]:
    raw = get_runtime_attack_value(key, list(default), payload)
    if isinstance(raw, str):
        items = raw.replace(";", ",").split(",")
    elif isinstance(raw, (list, tuple)):
        items = list(raw)
    else:
        items = list(default)

    parsed: list[int] = []
    seen: set[int] = set()
    for item in items:
        try:
            value = int(float(item))
        except Exception:
            continue
        if value <= 0 or value in seen:
            continue
        seen.add(value)
        parsed.append(value)
    return parsed or [int(v) for v in default if int(v) > 0]


_FOV_DB_REQUIRED_COLUMNS = ("fov", "sep", "width", "vel")
_FOV_DB_OPTIONAL_COLUMNS = ("dps", "foot")


def _normalize_fov_db_key(value: Any) -> str:
    return str(value or "").strip().lower()


def _normalize_fov_db_raw_row(row: Dict[Any, Any]) -> Dict[str, Any]:
    return {
        _normalize_fov_db_key(key): value
        for key, value in row.items()
        if _normalize_fov_db_key(key)
    }


def _read_fov_db_csv_rows(path: Path) -> list[Dict[str, Any]]:
    for encoding in ("utf-8-sig", "cp949", "euc-kr"):
        try:
            with path.open("r", encoding=encoding, newline="") as fh:
                return [_normalize_fov_db_raw_row(dict(row)) for row in csv.DictReader(fh)]
        except UnicodeDecodeError:
            continue
    return []


def _read_fov_db_excel_rows(path: Path) -> list[Dict[str, Any]]:
    try:
        import openpyxl  # type: ignore
    except Exception:
        return []

    try:
        workbook = openpyxl.load_workbook(path, read_only=True, data_only=True)
    except Exception:
        return []

    try:
        for sheet in workbook.worksheets:
            header_map: dict[str, int] | None = None
            rows: list[Dict[str, Any]] = []
            for values in sheet.iter_rows(values_only=True):
                cells = list(values or ())
                if header_map is None:
                    normalized = [_normalize_fov_db_key(cell) for cell in cells]
                    candidate = {
                        name: idx
                        for idx, name in enumerate(normalized)
                        if name
                    }
                    if all(key in candidate for key in _FOV_DB_REQUIRED_COLUMNS):
                        header_map = candidate
                    continue

                if not any(cell not in (None, "") for cell in cells):
                    continue
                rows.append(
                    {
                        key: cells[idx] if idx < len(cells) else None
                        for key, idx in header_map.items()
                    }
                )
            if rows:
                return [_normalize_fov_db_raw_row(row) for row in rows]
    finally:
        try:
            workbook.close()
        except Exception:
            pass
    return []


def read_fov_db_raw_rows_from_path(path: str | Path) -> list[Dict[str, Any]]:
    db_path = Path(path)
    suffix = db_path.suffix.lower()
    if suffix in {".xlsx", ".xlsm"}:
        return _read_fov_db_excel_rows(db_path)
    return _read_fov_db_csv_rows(db_path)


def _to_fov_db_float(value: Any, default: float = 0.0) -> float:
    try:
        if isinstance(value, str):
            value = value.strip().replace(",", "")
        return float(value)
    except Exception:
        return float(default)


def read_fov_db_rows_from_path(path: str | Path) -> list[Dict[str, float]]:
    db_path = Path(path)
    if not db_path.exists():
        return []
    sig = _path_sig(db_path)
    if sig is not None:
        with _CACHE_LOCK:
            cached = _FOV_DB_ROWS_BY_PATH_SIG.get(sig)
            if isinstance(cached, list):
                return [dict(row) for row in cached]

    rows: list[Dict[str, float]] = []
    try:
        for row in read_fov_db_raw_rows_from_path(db_path):
            parsed = {
                "fov": _to_fov_db_float(row.get("fov"), 0.0),
                "sep": _to_fov_db_float(row.get("sep"), 0.0),
                "width": _to_fov_db_float(row.get("width"), 0.0),
                "vel": _to_fov_db_float(row.get("vel"), 0.0),
                "dps": _to_fov_db_float(row.get("dps"), 0.0),
                "foot": _to_fov_db_float(row.get("foot"), 0.0),
            }
            if (
                parsed["fov"] <= 0.0
                or parsed["sep"] <= 0.0
                or parsed["width"] <= 0.0
                or parsed["vel"] <= 0.0
            ):
                continue
            rows.append(parsed)
    except Exception:
        return []
    if sig is not None:
        with _CACHE_LOCK:
            resolved = sig[0]
            stale_keys = [key for key in _FOV_DB_ROWS_BY_PATH_SIG if key[0] == resolved and key != sig]
            for key in stale_keys:
                _FOV_DB_ROWS_BY_PATH_SIG.pop(key, None)
            _FOV_DB_ROWS_BY_PATH_SIG[sig] = [dict(row) for row in rows]
    return rows


def validate_fov_db_file(path: str | Path) -> tuple[bool, str, int]:
    db_path = Path(path)
    if not db_path.exists():
        return False, f"DB file not found: {db_path}", 0
    if db_path.suffix.lower() not in {".csv", ".xlsx", ".xlsm"}:
        return False, "FOV DB must be a .csv, .xlsx, or .xlsm file.", 0
    rows = read_fov_db_rows_from_path(db_path)
    if not rows:
        required = ", ".join(_FOV_DB_REQUIRED_COLUMNS)
        return False, f"No valid FOV DB rows found. Required columns: {required}", 0
    return True, f"Loaded {len(rows)} valid FOV DB rows.", len(rows)


def load_fov_db_rows() -> list[Dict[str, float]]:
    global _FOV_DB_ROWS_CACHE_SIG, _FOV_DB_ROWS

    path = fov_db_path()
    sig = _path_sig(path)
    if sig is None:
        return []

    with _CACHE_LOCK:
        if _FOV_DB_ROWS_CACHE_SIG == sig and isinstance(_FOV_DB_ROWS, list):
            return [dict(row) for row in _FOV_DB_ROWS]

    rows = read_fov_db_rows_from_path(path)

    with _CACHE_LOCK:
        _FOV_DB_ROWS_CACHE_SIG = sig
        _FOV_DB_ROWS = [dict(row) for row in rows]

    return [dict(row) for row in rows]


def select_fov_db_row_by_turn_radius(turn_radius_m: float) -> Dict[str, float] | None:
    rows = load_fov_db_rows()
    if not rows:
        return None

    threshold = max(float(turn_radius_m), 0.0)
    candidates = [
        row
        for row in rows
        if float(row.get("sep", 0.0) or 0.0) > threshold and float(row.get("fov", 0.0) or 0.0) > 0.0
    ]
    if candidates:
        selected = max(
            candidates,
            key=lambda item: (
                float(item.get("fov", 0.0) or 0.0),
                float(item.get("sep", 0.0) or 0.0),
                float(item.get("width", 0.0) or 0.0),
            ),
        )
        return dict(selected)

    fallback = max(
        rows,
        key=lambda item: (
            float(item.get("sep", 0.0) or 0.0),
            float(item.get("fov", 0.0) or 0.0),
            float(item.get("width", 0.0) or 0.0),
        ),
    )
    return dict(fallback)


def get_runtime_prior_mission_profile(
    default_turn_radius_m: float = 400.0,
    default_fov_deg: float = 5.0,
    payload: Dict[str, Any] | None = None,
) -> Dict[str, float]:
    turn_radius_m = max(get_runtime_float("dubins_turn_radius_m", default_turn_radius_m, payload), 0.0)
    selected_row = select_fov_db_row_by_turn_radius(turn_radius_m)
    if get_runtime_manual_fov_sync_active(payload):
        manual_base_fov_deg = get_runtime_manual_fov_deg(
            "global_manual_fov_deg",
            default_fov_deg,
            payload,
            camera_adjust=False,
        )
        manual_fov_deg = apply_runtime_camera_adjusted_fov_deg(
            manual_base_fov_deg,
            payload,
        )
        return {
            "turn_radius_m": float(turn_radius_m),
            "base_fov_deg": float(manual_base_fov_deg),
            "fov_deg": float(manual_fov_deg),
            "sep_m": float(selected_row.get("sep", 0.0) or 0.0) if isinstance(selected_row, dict) else 0.0,
            "width_m": float(selected_row.get("width", 0.0) or 0.0) if isinstance(selected_row, dict) else 0.0,
        }
    if not isinstance(selected_row, dict):
        fallback_base_fov_deg = float(default_fov_deg)
        fallback_fov_deg = apply_runtime_camera_adjusted_fov_deg(fallback_base_fov_deg, payload)
        return {
            "turn_radius_m": float(turn_radius_m),
            "base_fov_deg": float(fallback_base_fov_deg),
            "fov_deg": float(fallback_fov_deg),
            "sep_m": 0.0,
            "width_m": 0.0,
        }

    try:
        base_fov_deg = float(selected_row.get("fov", default_fov_deg) or default_fov_deg)
    except Exception:
        base_fov_deg = float(default_fov_deg)
    fov_deg = apply_runtime_camera_adjusted_fov_deg(base_fov_deg, payload)

    return {
        "turn_radius_m": float(turn_radius_m),
        "base_fov_deg": float(base_fov_deg),
        "fov_deg": float(fov_deg),
        "sep_m": float(selected_row.get("sep", 0.0) or 0.0),
        "width_m": float(selected_row.get("width", 0.0) or 0.0),
    }

def load_fov_db_max_width(default: float = 0.0) -> float:
    global _FOV_DB_CACHE_SIG, _FOV_DB_MAX_WIDTH

    path = fov_db_path()
    sig = _path_sig(path)
    if sig is None:
        return float(default)

    with _CACHE_LOCK:
        if _FOV_DB_CACHE_SIG == sig and _FOV_DB_MAX_WIDTH is not None:
            cached = float(_FOV_DB_MAX_WIDTH)
            return cached if cached > 0.0 else float(default)

    max_width = 0.0
    try:
        for row in read_fov_db_rows_from_path(path):
            try:
                width = float(row.get("width", 0.0) or 0.0)
            except Exception:
                continue
            if width > max_width:
                max_width = width
    except Exception:
        max_width = 0.0

    with _CACHE_LOCK:
        _FOV_DB_CACHE_SIG = sig
        _FOV_DB_MAX_WIDTH = float(max_width)

    return float(max_width) if max_width > 0.0 else float(default)


def get_runtime_area_review_max_segment_m(
    default: float,
    payload: Dict[str, Any] | None = None,
) -> float:
    default_value = max(float(default), 0.0)
    manual_value = max(get_runtime_float("enhanced_area_review_max_segment_m", default_value, payload), 0.0)
    configured_limit = manual_value if manual_value > 0.0 else default_value
    if get_runtime_bool("area_review_max_segment_force_enabled", False, payload):
        return float(configured_limit)
    auto_enabled = get_runtime_area_auto_fov_from_db(True, payload)
    if auto_enabled:
        db_max_width = load_fov_db_max_width(0.0)
        if db_max_width > 0.0:
            if configured_limit > 0.0:
                return float(min(float(db_max_width), float(configured_limit)))
            return float(db_max_width)
    return float(configured_limit)
