from __future__ import annotations

import copy
import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable

from modules.common.settings_paths import replan_defaults_path, replan_settings_path

try:
    from modules.common import replan_perf
except Exception:
    import sys as _sys

    _COMMON_DIR = next(
        (
            parent / "common"
            for parent in Path(__file__).resolve().parents
            if (parent / "common" / "replan_perf.py").exists()
        ),
        None,
    )
    if _COMMON_DIR is not None and str(_COMMON_DIR) not in _sys.path:
        _sys.path.insert(0, str(_COMMON_DIR))
    import replan_perf  # type: ignore


_CACHE_LOCK = threading.Lock()
_CACHE_SIG: tuple[tuple[int, int] | None, tuple[int, int] | None] | None = None
_CACHE_DATA: Dict[str, Any] | None = None

_FALLBACK_SETTINGS: Dict[str, Any] = {
    "version": 2,
    "toggles": {
        "input_refresh": True,
        "prior_mission": True,
        "dl_risk": False,
        "target_detection": True,
        "post_attack_rejoin": True,
        "forced_command": True,
        "rtb": True,
        "path_deviation": True,
        "quality_monitor": True,
        "quality_speed": False,
        "imaging_schedule": False,
        "next_collab": False,
        "fuel_threshold": False,
    },
    "input_refresh": {
        "duplicate_window_ms": 400,
        "block_when_reexecute_active": True,
    },
    "prior_mission": {
        "dl_risk_threshold": 0.50,
    },
    "dl_risk": {
        "mean_risk_threshold": 0.80,
        "cooldown_sec": 10.0,
    },
    "path_deviation": {
        "turn_rate_threshold_dps": 2.0,
        "turn_window_s": 8.0,
        "stale_timeout_s": 5.0,
        "heading_move_min_m": 2.0,
        "turn_gap_reset_s": 2.5,
        "spiral_window_s": 75.0,
        "spiral_min_points": 5,
        "center_ignore_radius_m": 40.0,
        "near_threshold_min_m": 200.0,
        "near_threshold_radius_factor": 2.0,
        "radius_error_min_m": 140.0,
        "radius_error_factor": 0.55,
        "latest_radius_min_m": 60.0,
        "latest_radius_factor": 0.16,
        "accumulation_step_max_deg": 120.0,
        "watch_angle_deg": 45.0,
        "warning_angle_deg": 70.0,
        "hold_s": 2.0,
        "release_min_distance_m": 240.0,
        "release_factor": 2.3,
        "alt_waypoint_trigger_s": 2.0,
        "alt_waypoint_lead_time_s": 10.0,
        "next_mission_entry_lead_time_s": 10.0,
        "turn_radius_scale": 1.15,
        "turn_radius_30_m": 340.0,
        "turn_radius_40_m": 450.0,
        "turn_radius_50_m": 560.0,
        "coord_heading_fallback_min_delta_deg": 25.0,
        "coord_heading_fallback_max_delta_deg": 120.0,
        "coord_heading_fallback_max_turn_dps": 15.0,
        "attitude_enabled": True,
        "attitude_turn_rate_blend": 0.35,
        "attitude_yaw_rate_blend": 0.35,
        "attitude_yaw_rate_window_s": 3.0,
        "attitude_yaw_rate_min_samples": 3,
        "attitude_min_bank_deg": 1.5,
        "attitude_max_course_yaw_error_deg": 35.0,
        "attitude_projection_enabled": True,
        "attitude_projection_max_lead_s": 20.0,
        "attitude_projection_step_s": 0.1,
        "attitude_roll_rate_window_s": 2.0,
        "attitude_roll_rate_min_samples": 3,
        "attitude_roll_target_horizon_s": 1.5,
        "attitude_roll_rate_limit_dps": 40.0,
        "attitude_roll_time_constant_s": 0.45,
        "attitude_rate_adaptive_enabled": True,
        "attitude_rate_adaptive_min_rate_dps": 1.0,
        "attitude_rate_adaptive_sample_interval_s": 0.75,
        "attitude_rate_adaptive_ema_alpha": 0.08,
        "attitude_rate_adaptive_warmup_samples": 4,
        "attitude_rate_adaptive_min_scale": 0.65,
        "attitude_rate_adaptive_max_scale": 1.35,
        # --- Speed provenance: use track-derived speed when telemetry lies ---
        "track_speed_window_s": 2.0,
        "track_speed_min_dt_s": 0.6,
        "track_speed_max_valid_mps": 120.0,
        "track_speed_min_flight_mps": 5.0,
        "track_speed_min_samples": 8,
        "track_speed_mismatch_low_ratio": 0.7,
        "track_speed_mismatch_high_ratio": 1.4,
        # --- Attitude-first line entry projection ---
        "line_projection_attitude_enabled": True,
        "line_projection_attitude_confidence_min": 0.45,
        "line_projection_roll_evidence_bank_deg": 6.0,
        "line_projection_roll_evidence_rate_dps": 1.5,
        "line_projection_bank_settled_deg": 2.5,
        # Bank at which a roll opposite the heading trend is taken as a
        # genuine reversal rather than noise.
        "line_projection_reversal_bank_deg": 10.0,
        "next_collab_entry_pipeline_latency_s": 2.5,
        # --- Per-aircraft roll-response learning (our/external/verification sim) ---
        "roll_dynamics_learn_ema_alpha": 0.05,
        "roll_dynamics_learn_warmup_samples": 6,
        "roll_slew_default_dps": 6.0,
        "roll_slew_learn_min_dps": 0.8,
        # Above our own sim's 40 deg/s so a fast airframe is learned, not clipped.
        "roll_slew_learn_max_dps": 60.0,
        "steady_bank_default_deg": 24.0,
        "steady_bank_learn_min_rate_dps": 2.5,
        "adaptive_enabled": True,
        "adaptive_sample_min_interval_s": 0.75,
        "adaptive_min_turn_rate_dps": 2.5,
        "adaptive_min_speed_mps": 25.0,
        "adaptive_max_speed_mps": 70.0,
        "adaptive_min_radius_m": 80.0,
        "adaptive_max_radius_m": 2000.0,
        "adaptive_ema_alpha": 0.08,
        "adaptive_warmup_samples": 4,
        "adaptive_min_scale": 0.85,
        "adaptive_max_scale": 1.45,
        "adaptive_threshold_gain": 0.65,
        "adaptive_threshold_down_gain": 0.35,
        "adaptive_save_interval_s": 10.0,
    },
    "quality_speed": {
        "lower_band_ratio": 0.90,
        "search_speed_up_scale": 1.10,
        "search_speed_down_scale": 0.90,
        "min_sample_count": 5,
        "max_sample_count": 30,
        "startup_grace_sec": 10.0,
        "disabled_flight_mode": 8,
    },
    "forced_command": {
        "hold_delay_seconds": 30.0,
        "signature_dedup_seconds": 0.6,
    },
    "imaging_schedule": {
        "trigger_probability": 0.20,
        "imaging_operation_modes": [1, 2, 3, 4, 5],
        "imaging_pattern_types": [3, 4, 5, 6, 7, 8, 9],
    },
    "rtb": {
        "unexpected_rtb_flight_mode": 5,
        "abnormal_health_value": 2,
        "fuel_warning_replan_level": 2,
        "signal_loss_grace_ms": 10000,
        "replan_hold_ms": 5000,
        "fault_unavailable_hold_ms": 55000,
        "command_aircraft_id": 1,
    },
    "target_detection": {
        "cooldown_ms": 10000,
        "watcher_uav_ids": [4, 5, 6],
        "attack_manned_ids": [2, 3],
        "target_type_priority": [1, 2, 3, 4, 5, 6],
        "option_presets": [
            {"option_id": 2, "option_name": "공격 특화"},
            {"option_id": 3, "option_name": "공격 배제"},
        ],
    },
    "post_attack_rejoin": {
        "closure_cooldown_ms": 30000,
        "min_remaining_eta_s": 30,
        "rejoin_margin_s": 15,
        "turn_radius_m": 180.0,
        "default_cruise_speed_mps": 35.0,
        "active_progress_skip_percent": 70,
    },
    "replan_queue": {
        # 정찰 옵션 포함 재수행 재계획은 0701/0301 산출까지 길어질 수 있어 45s로는 시간 초과가 잦다.
        "active_timeout_ms": 120000,
        "history_limit": 30,
        "target_dispatch_delay_ms": 0,
        "release_on_option_info": False,
        "suppress_active_target_options_on_new_detection": True,
    },
    "fuel_threshold": {
        "capacity_liters": 15.0,
        "yellow_ratio": 0.20,
        "red_ratio": 0.10,
    },
}


def settings_path() -> Path:
    return replan_settings_path()


def defaults_path() -> Path:
    return replan_defaults_path()


def _path_sig(path: Path) -> tuple[int, int] | None:
    try:
        stat = path.stat()
    except Exception:
        return None
    return int(stat.st_mtime_ns), int(stat.st_size)


def _read_json_dict(path: Path) -> Dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        payload = {}
    return dict(payload) if isinstance(payload, dict) else {}


def _as_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _as_int(value: Any, default: int) -> int:
    try:
        return int(float(value))
    except Exception:
        return int(default)


def _as_bool(value: Any, default: bool) -> bool:
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"1", "true", "yes", "y", "on"}:
            return True
        if text in {"0", "false", "no", "n", "off", ""}:
            return False
    try:
        return bool(value)
    except Exception:
        return bool(default)


def _coerce_int_list(value: Any, fallback: Iterable[int]) -> list[int]:
    source = list(value) if isinstance(value, (list, tuple)) else list(fallback)
    out: list[int] = []
    seen: set[int] = set()
    for item in source:
        parsed = _as_int(item, -1)
        if parsed <= 0 or parsed in seen:
            continue
        seen.add(parsed)
        out.append(parsed)
    return out or [int(v) for v in fallback if int(v) > 0]


def _normalize_option_presets(value: Any) -> list[dict[str, Any]]:
    fallback = list((_FALLBACK_SETTINGS.get("target_detection") or {}).get("option_presets") or [])
    source = list(value) if isinstance(value, list) else fallback
    normalized: list[dict[str, Any]] = []
    seen_ids: set[int] = set()
    for item in source:
        if not isinstance(item, dict):
            continue
        option_id = _as_int(item.get("option_id") or item.get("optionID"), -1)
        option_name = str(item.get("option_name") or item.get("optionName") or "").strip()
        if option_id <= 0 or not option_name or option_id in seen_ids:
            continue
        seen_ids.add(option_id)
        normalized.append(
            {
                "option_id": int(option_id),
                "option_name": option_name,
            }
        )
    return normalized or copy.deepcopy(fallback)


def _normalize_strategy(value: Any, default: str) -> str:
    text = str(value or default).strip().lower()
    if text in {"current_position", "turn_projection", "midpoint_to_next_start"}:
        return text
    return str(default)


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def default_replan_settings() -> Dict[str, Any]:
    disk_defaults = _read_json_dict(defaults_path())
    merged = _deep_merge(_FALLBACK_SETTINGS, disk_defaults)
    return normalize_replan_settings(merged, use_disk_defaults=False)


def normalize_replan_settings(
    payload: Dict[str, Any] | None,
    *,
    use_disk_defaults: bool = True,
) -> Dict[str, Any]:
    if use_disk_defaults:
        base = default_replan_settings()
    else:
        base = copy.deepcopy(_FALLBACK_SETTINGS)
    data = copy.deepcopy(payload) if isinstance(payload, dict) else {}
    merged = _deep_merge(base, data)

    merged["version"] = max(1, _as_int(merged.get("version"), int(base.get("version", 2))))

    toggles = merged.get("toggles")
    if not isinstance(toggles, dict):
        toggles = {}
        merged["toggles"] = toggles
    fallback_toggles = (base.get("toggles") if isinstance(base.get("toggles"), dict) else {}) or {}
    for key, fallback in fallback_toggles.items():
        toggles[str(key)] = _as_bool(toggles.get(key), bool(fallback))

    input_refresh_cfg = merged.get("input_refresh")
    if not isinstance(input_refresh_cfg, dict):
        input_refresh_cfg = {}
        merged["input_refresh"] = input_refresh_cfg
    input_refresh_cfg["duplicate_window_ms"] = max(
        0, _as_int(input_refresh_cfg.get("duplicate_window_ms"), 400)
    )
    input_refresh_cfg["block_when_reexecute_active"] = _as_bool(
        input_refresh_cfg.get("block_when_reexecute_active"),
        True,
    )

    prior_cfg = merged.get("prior_mission")
    if not isinstance(prior_cfg, dict):
        prior_cfg = {}
        merged["prior_mission"] = prior_cfg
    prior_cfg["dl_risk_threshold"] = min(
        1.0,
        max(0.0, _as_float(prior_cfg.get("dl_risk_threshold"), 0.50)),
    )

    dl_cfg = merged.get("dl_risk")
    if not isinstance(dl_cfg, dict):
        dl_cfg = {}
        merged["dl_risk"] = dl_cfg
    dl_cfg["mean_risk_threshold"] = min(
        1.0,
        max(0.0, _as_float(dl_cfg.get("mean_risk_threshold"), 0.80)),
    )
    dl_cfg["cooldown_sec"] = max(0.0, _as_float(dl_cfg.get("cooldown_sec"), 10.0))

    path_cfg = merged.get("path_deviation")
    if not isinstance(path_cfg, dict):
        path_cfg = {}
        merged["path_deviation"] = path_cfg
    path_cfg["turn_rate_threshold_dps"] = max(0.1, _as_float(path_cfg.get("turn_rate_threshold_dps"), 2.0))
    path_cfg["turn_window_s"] = max(2.0, _as_float(path_cfg.get("turn_window_s"), 8.0))
    path_cfg["stale_timeout_s"] = max(0.5, _as_float(path_cfg.get("stale_timeout_s"), 5.0))
    path_cfg["heading_move_min_m"] = max(0.0, _as_float(path_cfg.get("heading_move_min_m"), 2.0))
    path_cfg["turn_gap_reset_s"] = max(0.1, _as_float(path_cfg.get("turn_gap_reset_s"), 2.5))
    path_cfg["spiral_window_s"] = max(10.0, _as_float(path_cfg.get("spiral_window_s"), 75.0))
    path_cfg["spiral_min_points"] = max(3, _as_int(path_cfg.get("spiral_min_points"), 5))
    path_cfg["center_ignore_radius_m"] = max(0.0, _as_float(path_cfg.get("center_ignore_radius_m"), 40.0))
    path_cfg["near_threshold_min_m"] = max(1.0, _as_float(path_cfg.get("near_threshold_min_m"), 200.0))
    path_cfg["near_threshold_radius_factor"] = max(1.0, _as_float(path_cfg.get("near_threshold_radius_factor"), 2.0))
    path_cfg["radius_error_min_m"] = max(1.0, _as_float(path_cfg.get("radius_error_min_m"), 140.0))
    path_cfg["radius_error_factor"] = max(0.05, _as_float(path_cfg.get("radius_error_factor"), 0.55))
    path_cfg["latest_radius_min_m"] = max(1.0, _as_float(path_cfg.get("latest_radius_min_m"), 60.0))
    path_cfg["latest_radius_factor"] = max(0.01, _as_float(path_cfg.get("latest_radius_factor"), 0.16))
    path_cfg["accumulation_step_max_deg"] = max(10.0, _as_float(path_cfg.get("accumulation_step_max_deg"), 120.0))
    path_cfg["watch_angle_deg"] = max(10.0, _as_float(path_cfg.get("watch_angle_deg"), 90.0))
    path_cfg["warning_angle_deg"] = max(
        float(path_cfg["watch_angle_deg"]) + 1.0,
        _as_float(path_cfg.get("warning_angle_deg"), 150.0),
    )
    path_cfg["hold_s"] = max(0.0, _as_float(path_cfg.get("hold_s"), 2.0))
    path_cfg["release_min_distance_m"] = max(1.0, _as_float(path_cfg.get("release_min_distance_m"), 240.0))
    path_cfg["release_factor"] = max(1.0, _as_float(path_cfg.get("release_factor"), 2.3))
    path_cfg["alt_waypoint_trigger_s"] = max(0.0, _as_float(path_cfg.get("alt_waypoint_trigger_s"), 2.0))
    path_cfg["alt_waypoint_lead_time_s"] = max(0.0, _as_float(path_cfg.get("alt_waypoint_lead_time_s"), 10.0))
    path_cfg["next_mission_entry_lead_time_s"] = max(
        0.0, _as_float(path_cfg.get("next_mission_entry_lead_time_s"), 10.0)
    )
    path_cfg["turn_radius_30_m"] = max(1.0, _as_float(path_cfg.get("turn_radius_30_m"), 340.0))
    path_cfg["turn_radius_40_m"] = max(
        float(path_cfg["turn_radius_30_m"]),
        _as_float(path_cfg.get("turn_radius_40_m"), 450.0),
    )
    path_cfg["turn_radius_50_m"] = max(
        float(path_cfg["turn_radius_40_m"]),
        _as_float(path_cfg.get("turn_radius_50_m"), 560.0),
    )
    path_cfg["coord_heading_fallback_min_delta_deg"] = max(
        0.0, _as_float(path_cfg.get("coord_heading_fallback_min_delta_deg"), 25.0)
    )
    path_cfg["coord_heading_fallback_max_delta_deg"] = max(
        float(path_cfg["coord_heading_fallback_min_delta_deg"]),
        _as_float(path_cfg.get("coord_heading_fallback_max_delta_deg"), 120.0),
    )
    path_cfg["coord_heading_fallback_max_turn_dps"] = max(
        0.0, _as_float(path_cfg.get("coord_heading_fallback_max_turn_dps"), 15.0)
    )
    path_cfg["attitude_enabled"] = _as_bool(path_cfg.get("attitude_enabled"), True)
    path_cfg["attitude_turn_rate_blend"] = min(
        1.0,
        max(0.0, _as_float(path_cfg.get("attitude_turn_rate_blend"), 0.35)),
    )
    path_cfg["attitude_yaw_rate_blend"] = min(
        1.0,
        max(0.0, _as_float(path_cfg.get("attitude_yaw_rate_blend"), 0.35)),
    )
    path_cfg["attitude_yaw_rate_window_s"] = max(
        0.2,
        _as_float(path_cfg.get("attitude_yaw_rate_window_s"), 3.0),
    )
    path_cfg["attitude_yaw_rate_min_samples"] = max(
        2,
        _as_int(path_cfg.get("attitude_yaw_rate_min_samples"), 3),
    )
    path_cfg["attitude_min_bank_deg"] = min(
        85.0,
        max(0.0, _as_float(path_cfg.get("attitude_min_bank_deg"), 1.5)),
    )
    path_cfg["attitude_max_course_yaw_error_deg"] = min(
        180.0,
        max(0.0, _as_float(path_cfg.get("attitude_max_course_yaw_error_deg"), 35.0)),
    )
    path_cfg["attitude_projection_enabled"] = _as_bool(
        path_cfg.get("attitude_projection_enabled"),
        True,
    )
    path_cfg["attitude_projection_max_lead_s"] = min(
        120.0,
        max(0.0, _as_float(path_cfg.get("attitude_projection_max_lead_s"), 20.0)),
    )
    path_cfg["attitude_projection_step_s"] = min(
        1.0,
        max(0.01, _as_float(path_cfg.get("attitude_projection_step_s"), 0.1)),
    )
    path_cfg["attitude_roll_rate_window_s"] = max(
        0.2,
        _as_float(path_cfg.get("attitude_roll_rate_window_s"), 2.0),
    )
    path_cfg["attitude_roll_rate_min_samples"] = max(
        2,
        _as_int(path_cfg.get("attitude_roll_rate_min_samples"), 3),
    )
    path_cfg["attitude_roll_target_horizon_s"] = min(
        10.0,
        max(0.0, _as_float(path_cfg.get("attitude_roll_target_horizon_s"), 1.5)),
    )
    path_cfg["attitude_roll_rate_limit_dps"] = min(
        120.0,
        max(0.1, _as_float(path_cfg.get("attitude_roll_rate_limit_dps"), 40.0)),
    )
    path_cfg["attitude_roll_time_constant_s"] = min(
        10.0,
        max(0.05, _as_float(path_cfg.get("attitude_roll_time_constant_s"), 0.45)),
    )
    path_cfg["attitude_rate_adaptive_enabled"] = _as_bool(
        path_cfg.get("attitude_rate_adaptive_enabled"),
        True,
    )
    path_cfg["attitude_rate_adaptive_min_rate_dps"] = max(
        0.1,
        _as_float(path_cfg.get("attitude_rate_adaptive_min_rate_dps"), 1.0),
    )
    path_cfg["attitude_rate_adaptive_sample_interval_s"] = max(
        0.1,
        _as_float(path_cfg.get("attitude_rate_adaptive_sample_interval_s"), 0.75),
    )
    path_cfg["attitude_rate_adaptive_ema_alpha"] = min(
        1.0,
        max(0.01, _as_float(path_cfg.get("attitude_rate_adaptive_ema_alpha"), 0.08)),
    )
    path_cfg["attitude_rate_adaptive_warmup_samples"] = max(
        1,
        _as_int(path_cfg.get("attitude_rate_adaptive_warmup_samples"), 4),
    )
    path_cfg["attitude_rate_adaptive_min_scale"] = max(
        0.1,
        _as_float(path_cfg.get("attitude_rate_adaptive_min_scale"), 0.65),
    )
    path_cfg["attitude_rate_adaptive_max_scale"] = max(
        float(path_cfg["attitude_rate_adaptive_min_scale"]),
        _as_float(path_cfg.get("attitude_rate_adaptive_max_scale"), 1.35),
    )
    path_cfg["adaptive_enabled"] = _as_bool(path_cfg.get("adaptive_enabled"), True)
    path_cfg["adaptive_sample_min_interval_s"] = max(
        0.2, _as_float(path_cfg.get("adaptive_sample_min_interval_s"), 0.75)
    )
    path_cfg["adaptive_min_turn_rate_dps"] = max(
        0.1, _as_float(path_cfg.get("adaptive_min_turn_rate_dps"), 2.5)
    )
    path_cfg["adaptive_min_speed_mps"] = max(
        0.0, _as_float(path_cfg.get("adaptive_min_speed_mps"), 25.0)
    )
    path_cfg["adaptive_max_speed_mps"] = max(
        float(path_cfg["adaptive_min_speed_mps"]),
        _as_float(path_cfg.get("adaptive_max_speed_mps"), 70.0),
    )
    path_cfg["adaptive_min_radius_m"] = max(
        1.0, _as_float(path_cfg.get("adaptive_min_radius_m"), 80.0)
    )
    path_cfg["adaptive_max_radius_m"] = max(
        float(path_cfg["adaptive_min_radius_m"]),
        _as_float(path_cfg.get("adaptive_max_radius_m"), 2000.0),
    )
    path_cfg["adaptive_ema_alpha"] = min(
        1.0,
        max(0.01, _as_float(path_cfg.get("adaptive_ema_alpha"), 0.08)),
    )
    path_cfg["adaptive_warmup_samples"] = max(
        1, _as_int(path_cfg.get("adaptive_warmup_samples"), 4)
    )
    path_cfg["adaptive_min_scale"] = max(
        0.1, _as_float(path_cfg.get("adaptive_min_scale"), 0.85)
    )
    path_cfg["adaptive_max_scale"] = max(
        float(path_cfg["adaptive_min_scale"]),
        _as_float(path_cfg.get("adaptive_max_scale"), 1.45),
    )
    path_cfg["adaptive_threshold_gain"] = min(
        1.0,
        max(0.0, _as_float(path_cfg.get("adaptive_threshold_gain"), 0.65)),
    )
    path_cfg["adaptive_threshold_down_gain"] = min(
        1.0,
        max(0.0, _as_float(path_cfg.get("adaptive_threshold_down_gain"), 0.35)),
    )
    path_cfg["adaptive_save_interval_s"] = max(
        1.0, _as_float(path_cfg.get("adaptive_save_interval_s"), 10.0)
    )

    quality_cfg = merged.get("quality_speed")
    if not isinstance(quality_cfg, dict):
        quality_cfg = {}
        merged["quality_speed"] = quality_cfg
    quality_cfg["lower_band_ratio"] = min(1.0, max(0.1, _as_float(quality_cfg.get("lower_band_ratio"), 0.90)))
    quality_cfg["search_speed_up_scale"] = max(0.1, _as_float(quality_cfg.get("search_speed_up_scale"), 1.10))
    quality_cfg["search_speed_down_scale"] = max(0.1, _as_float(quality_cfg.get("search_speed_down_scale"), 0.90))
    quality_cfg["min_sample_count"] = max(1, _as_int(quality_cfg.get("min_sample_count"), 5))
    quality_cfg["max_sample_count"] = max(
        int(quality_cfg["min_sample_count"]),
        _as_int(quality_cfg.get("max_sample_count"), 30),
    )
    quality_cfg["startup_grace_sec"] = max(0.0, _as_float(quality_cfg.get("startup_grace_sec"), 10.0))
    quality_cfg["disabled_flight_mode"] = max(0, _as_int(quality_cfg.get("disabled_flight_mode"), 8))

    forced_cfg = merged.get("forced_command")
    if not isinstance(forced_cfg, dict):
        forced_cfg = {}
        merged["forced_command"] = forced_cfg
    forced_cfg["hold_delay_seconds"] = max(0.0, _as_float(forced_cfg.get("hold_delay_seconds"), 30.0))
    forced_cfg["signature_dedup_seconds"] = max(
        0.0,
        _as_float(forced_cfg.get("signature_dedup_seconds"), 0.6),
    )

    imaging_cfg = merged.get("imaging_schedule")
    if not isinstance(imaging_cfg, dict):
        imaging_cfg = {}
        merged["imaging_schedule"] = imaging_cfg
    imaging_cfg["trigger_probability"] = min(
        1.0,
        max(0.0, _as_float(imaging_cfg.get("trigger_probability"), 0.20)),
    )
    imaging_cfg["imaging_operation_modes"] = _coerce_int_list(
        imaging_cfg.get("imaging_operation_modes"),
        [1, 2, 3, 4, 5],
    )
    imaging_cfg["imaging_pattern_types"] = _coerce_int_list(
        imaging_cfg.get("imaging_pattern_types"),
        [3, 4, 5, 6, 7, 8, 9],
    )

    rtb_cfg = merged.get("rtb")
    if not isinstance(rtb_cfg, dict):
        rtb_cfg = {}
        merged["rtb"] = rtb_cfg
    rtb_cfg["unexpected_rtb_flight_mode"] = max(0, _as_int(rtb_cfg.get("unexpected_rtb_flight_mode"), 5))
    rtb_cfg["abnormal_health_value"] = max(0, _as_int(rtb_cfg.get("abnormal_health_value"), 2))
    rtb_cfg["fuel_warning_replan_level"] = max(0, _as_int(rtb_cfg.get("fuel_warning_replan_level"), 2))
    rtb_cfg["signal_loss_grace_ms"] = max(0, _as_int(rtb_cfg.get("signal_loss_grace_ms"), 10000))
    rtb_cfg["replan_hold_ms"] = max(0, _as_int(rtb_cfg.get("replan_hold_ms"), 5000))
    rtb_cfg["fault_unavailable_hold_ms"] = max(
        0,
        _as_int(rtb_cfg.get("fault_unavailable_hold_ms"), 55000),
    )
    rtb_cfg["command_aircraft_id"] = min(
        3,
        max(1, _as_int(rtb_cfg.get("command_aircraft_id"), 1)),
    )

    target_cfg = merged.get("target_detection")
    if not isinstance(target_cfg, dict):
        target_cfg = {}
        merged["target_detection"] = target_cfg
    target_cfg["cooldown_ms"] = max(0, _as_int(target_cfg.get("cooldown_ms"), 10000))
    target_cfg["watcher_uav_ids"] = _coerce_int_list(target_cfg.get("watcher_uav_ids"), [4, 5, 6])
    target_cfg["attack_manned_ids"] = _coerce_int_list(target_cfg.get("attack_manned_ids"), [2, 3])
    target_cfg["target_type_priority"] = _coerce_int_list(
        target_cfg.get("target_type_priority"),
        [1, 2, 3, 4, 5, 6],
    )
    target_cfg["option_presets"] = _normalize_option_presets(target_cfg.get("option_presets"))

    post_attack_cfg = merged.get("post_attack_rejoin")
    if not isinstance(post_attack_cfg, dict):
        post_attack_cfg = {}
        merged["post_attack_rejoin"] = post_attack_cfg
    post_attack_cfg["closure_cooldown_ms"] = max(
        0,
        _as_int(post_attack_cfg.get("closure_cooldown_ms"), 30000),
    )
    post_attack_cfg["min_remaining_eta_s"] = max(
        0,
        _as_int(post_attack_cfg.get("min_remaining_eta_s"), 30),
    )
    post_attack_cfg["rejoin_margin_s"] = max(
        0,
        _as_int(post_attack_cfg.get("rejoin_margin_s"), 15),
    )
    post_attack_cfg["turn_radius_m"] = max(
        1.0,
        _as_float(post_attack_cfg.get("turn_radius_m"), 180.0),
    )
    post_attack_cfg["default_cruise_speed_mps"] = max(
        1.0,
        _as_float(post_attack_cfg.get("default_cruise_speed_mps"), 35.0),
    )
    post_attack_cfg["active_progress_skip_percent"] = max(
        0,
        min(100, _as_int(post_attack_cfg.get("active_progress_skip_percent"), 70)),
    )

    queue_cfg = merged.get("replan_queue")
    if not isinstance(queue_cfg, dict):
        queue_cfg = {}
        merged["replan_queue"] = queue_cfg
    queue_cfg["active_timeout_ms"] = max(1000, _as_int(queue_cfg.get("active_timeout_ms"), 120000))
    queue_cfg["history_limit"] = max(5, _as_int(queue_cfg.get("history_limit"), 30))
    queue_cfg["target_dispatch_delay_ms"] = max(0, _as_int(queue_cfg.get("target_dispatch_delay_ms"), 0))
    queue_cfg["release_on_option_info"] = _as_bool(
        queue_cfg.get("release_on_option_info"),
        False,
    )
    queue_cfg["suppress_active_target_options_on_new_detection"] = _as_bool(
        queue_cfg.get("suppress_active_target_options_on_new_detection"),
        True,
    )

    fuel_cfg = merged.get("fuel_threshold")
    if not isinstance(fuel_cfg, dict):
        fuel_cfg = {}
        merged["fuel_threshold"] = fuel_cfg
    fuel_cfg["capacity_liters"] = max(0.1, _as_float(fuel_cfg.get("capacity_liters"), 15.0))
    fuel_cfg["yellow_ratio"] = min(1.0, max(0.0, _as_float(fuel_cfg.get("yellow_ratio"), 0.20)))
    fuel_cfg["red_ratio"] = min(
        float(fuel_cfg["yellow_ratio"]),
        max(0.0, _as_float(fuel_cfg.get("red_ratio"), 0.10)),
    )

    return merged


def load_recommended_defaults() -> Dict[str, Any]:
    return default_replan_settings()


def _load_replan_settings_cached() -> Dict[str, Any]:
    global _CACHE_SIG, _CACHE_DATA
    perf_start = replan_perf.start_timer()
    defaults_sig = _path_sig(defaults_path())
    current_sig = _path_sig(settings_path())
    signature = (defaults_sig, current_sig)

    if signature[0] is not None or signature[1] is not None:
        with _CACHE_LOCK:
            if _CACHE_SIG == signature and isinstance(_CACHE_DATA, dict):
                replan_perf.add_elapsed(
                    "monitoring.replan_settings.load_cached",
                    perf_start,
                    cache_hit=1,
                )
                return _CACHE_DATA

    defaults_payload = default_replan_settings()
    current_payload = _read_json_dict(settings_path())
    data = normalize_replan_settings(_deep_merge(defaults_payload, current_payload), use_disk_defaults=False)
    with _CACHE_LOCK:
        _CACHE_SIG = signature
        _CACHE_DATA = copy.deepcopy(data)
        cached = _CACHE_DATA
    replan_perf.add_elapsed(
        "monitoring.replan_settings.load_cached",
        perf_start,
        cache_miss=1,
    )
    return cached


def load_replan_settings() -> Dict[str, Any]:
    perf_start = replan_perf.start_timer()
    data = _load_replan_settings_cached()
    copy_start = time.perf_counter() if replan_perf.is_enabled() else None
    result = copy.deepcopy(data)
    copy_ms = 0.0
    if copy_start is not None:
        copy_ms = (time.perf_counter() - copy_start) * 1000.0
    replan_perf.add_elapsed(
        "monitoring.replan_settings.load",
        perf_start,
        deepcopy_ms=copy_ms,
    )
    return result


def save_replan_settings(payload: Dict[str, Any] | None) -> Dict[str, Any]:
    path = settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    data = normalize_replan_settings(payload)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    with _CACHE_LOCK:
        global _CACHE_SIG, _CACHE_DATA
        _CACHE_SIG = (_path_sig(defaults_path()), _path_sig(path))
        _CACHE_DATA = copy.deepcopy(data)
    return copy.deepcopy(data)


def save_recommended_defaults(payload: Dict[str, Any] | None) -> Dict[str, Any]:
    path = defaults_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    data = normalize_replan_settings(payload, use_disk_defaults=False)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    with _CACHE_LOCK:
        global _CACHE_SIG, _CACHE_DATA
        _CACHE_SIG = None
        _CACHE_DATA = None
    return copy.deepcopy(data)


def update_replan_toggle(key: str, enabled: bool) -> Dict[str, Any]:
    data = load_replan_settings()
    toggles = data.get("toggles")
    if not isinstance(toggles, dict):
        toggles = {}
        data["toggles"] = toggles
    toggles[str(key)] = bool(enabled)
    return save_replan_settings(data)


def get_replan_group(name: str, payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    data = payload if isinstance(payload, dict) else _load_replan_settings_cached()
    group = data.get(str(name))
    return copy.deepcopy(group) if isinstance(group, dict) else {}


def get_replan_toggles(payload: Dict[str, Any] | None = None) -> Dict[str, bool]:
    data = payload if isinstance(payload, dict) else _load_replan_settings_cached()
    toggles = data.get("toggles")
    if not isinstance(toggles, dict):
        return {}
    return {str(key): bool(value) for key, value in toggles.items()}


def get_replan_toggle(key: str, default: bool, payload: Dict[str, Any] | None = None) -> bool:
    return bool(get_replan_toggles(payload).get(str(key), bool(default)))


def get_input_refresh_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("input_refresh", payload)


def get_prior_mission_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("prior_mission", payload)


def get_dl_risk_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("dl_risk", payload)


def get_path_deviation_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("path_deviation", payload)


def get_quality_speed_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("quality_speed", payload)


def get_forced_command_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("forced_command", payload)


def get_imaging_schedule_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("imaging_schedule", payload)


def get_next_collab_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("next_collab", payload)


def get_rtb_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("rtb", payload)


def get_target_detection_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("target_detection", payload)


def get_post_attack_rejoin_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("post_attack_rejoin", payload)


def get_replan_queue_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("replan_queue", payload)


def get_fuel_threshold_settings(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    return get_replan_group("fuel_threshold", payload)
