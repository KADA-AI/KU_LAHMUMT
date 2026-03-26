from __future__ import annotations

import csv
import copy
import json
import threading
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict


_CACHE_LOCK = threading.Lock()
_CACHE_SIG: tuple[int, int] | None = None
_CACHE_DATA: Dict[str, Any] | None = None
_FOV_DB_CACHE_SIG: tuple[int, int] | None = None
_FOV_DB_MAX_WIDTH: float | None = None
_FOV_DB_ROWS_CACHE_SIG: tuple[int, int] | None = None
_FOV_DB_ROWS: list[Dict[str, float]] | None = None
_THREAD_LOCAL = threading.local()

DEFAULT_AREA_SWEEP_MODE = "vertical"
DEFAULT_AREA_SPLIT_MODE = "single_stage"
DEFAULT_UAV_PLAN_MODE = "dub_path"

DEFAULT_RUNTIME_VALUES: Dict[str, Any] = {
    "search_speed_weight": 1.0,
    "area_search_speed_weight": 1.0,
    "fov_deg": 2.4,
    "line_custom_fov_deg": 2.4,
    "area_custom_fov_deg": 2.4,
    "area_output_fov_scale": 1.0,
    "line_density_scale": 1.2,
    "area_density_scale": 1.2,
    "area_route_offset_scale": 1.0,
    "uav_wp_interval_m": 1200.0,
    "lah_wp_interval_m": 3000.0,
    "dubins_turn_radius_m": 500.0,
    "sweep_line_interp_points": 3,
    "enhanced_area_review_max_segment_m": 300.0,
    "enhanced_auto_fov_from_db": True,
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
DEFAULT_ATTACK_MISSION_VALUES: Dict[str, Any] = {
    "manned_candidate_ids": [2, 3],
    "entry_offset_m": 100.0,
    "resume_offset_m": 20.0,
    "weapon_type": 2,
    "lah_hold_seconds": 50,
    "lah_hold_near_resume_offset_m": 30.0,
    "resume_search_speed_scale": 1.3,
    "fast_num_arc_rays": 360,
    "point_cache_max": 16,
    "attack_point_altitude_offset_m": 300.0,
}
PERSISTED_RUNTIME_VALUE_KEYS = tuple(DEFAULT_RUNTIME_VALUES.keys())
PERSISTED_PRIOR_MISSION_KEYS = tuple(DEFAULT_PRIOR_MISSION_VALUES.keys())
PERSISTED_ATTACK_MISSION_KEYS = tuple(DEFAULT_ATTACK_MISSION_VALUES.keys())
PERSISTED_FLYOVER_KEYS = ("entry_offset", "dubins_prefix", "all_wps")


def settings_path() -> Path:
    return Path(__file__).resolve().parent / "uav_params.json"


def fov_db_path() -> Path:
    return Path(__file__).resolve().parents[3] / "resource" / "db" / "fov_db.csv"


def _path_sig(path: Path) -> tuple[int, int] | None:
    try:
        stat = path.stat()
    except Exception:
        return None
    return int(stat.st_mtime_ns), int(stat.st_size)


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
    data = payload if isinstance(payload, dict) else {}
    with _CACHE_LOCK:
        _CACHE_SIG = sig
        _CACHE_DATA = copy.deepcopy(data)
    return data


def get_runtime_override() -> Dict[str, Any] | None:
    override = getattr(_THREAD_LOCAL, "override_payload", None)
    return copy.deepcopy(override) if isinstance(override, dict) else None


def set_runtime_override(payload: Dict[str, Any] | None) -> None:
    if isinstance(payload, dict):
        _THREAD_LOCAL.override_payload = copy.deepcopy(payload)
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
        return base

    raw_values = payload.get("values")
    if isinstance(raw_values, dict):
        for key in PERSISTED_RUNTIME_VALUE_KEYS:
            if key in raw_values:
                base["values"][key] = raw_values[key]

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


def load_fov_db_rows() -> list[Dict[str, float]]:
    global _FOV_DB_ROWS_CACHE_SIG, _FOV_DB_ROWS

    path = fov_db_path()
    sig = _path_sig(path)
    if sig is None:
        return []

    with _CACHE_LOCK:
        if _FOV_DB_ROWS_CACHE_SIG == sig and isinstance(_FOV_DB_ROWS, list):
            return [dict(row) for row in _FOV_DB_ROWS]

    rows: list[Dict[str, float]] = []
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                try:
                    parsed = {
                        "fov": float(row.get("fov", 0.0) or 0.0),
                        "sep": float(row.get("sep", 0.0) or 0.0),
                        "width": float(row.get("width", 0.0) or 0.0),
                        "vel": float(row.get("vel", 0.0) or 0.0),
                        "dps": float(row.get("dps", 0.0) or 0.0),
                        "foot": float(row.get("foot", 0.0) or 0.0),
                    }
                except Exception:
                    continue
                if parsed["fov"] <= 0.0 or parsed["sep"] <= 0.0:
                    continue
                rows.append(parsed)
    except Exception:
        rows = []

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
    if not isinstance(selected_row, dict):
        return {
            "turn_radius_m": float(turn_radius_m),
            "fov_deg": float(default_fov_deg),
            "sep_m": 0.0,
            "width_m": 0.0,
        }

    try:
        fov_deg = float(selected_row.get("fov", default_fov_deg) or default_fov_deg)
    except Exception:
        fov_deg = float(default_fov_deg)

    return {
        "turn_radius_m": float(turn_radius_m),
        "fov_deg": float(fov_deg),
        "sep_m": float(selected_row.get("sep", 0.0) or 0.0),
        "width_m": float(selected_row.get("width", 0.0) or 0.0),
    }


def load_fov_db_max_width(default: float = 0.0) -> float:
    global _FOV_DB_CACHE_SIG, _FOV_DB_MAX_WIDTH

    path = fov_db_path()
    try:
        stat = path.stat()
        sig = (int(stat.st_mtime_ns), int(stat.st_size))
    except Exception:
        return float(default)

    with _CACHE_LOCK:
        if _FOV_DB_CACHE_SIG == sig and _FOV_DB_MAX_WIDTH is not None:
            cached = float(_FOV_DB_MAX_WIDTH)
            return cached if cached > 0.0 else float(default)

    max_width = 0.0
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
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
    manual_value = max(get_runtime_float("enhanced_area_review_max_segment_m", default, payload), 0.0)
    auto_enabled = get_runtime_bool("enhanced_auto_fov_from_db", True, payload)
    if auto_enabled:
        db_max_width = load_fov_db_max_width(0.0)
        if db_max_width > 0.0:
            return float(db_max_width)
    return float(manual_value if manual_value > 0.0 else max(float(default), 0.0))
