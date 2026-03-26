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


def get_runtime_preset_key(payload: Dict[str, Any] | None = None) -> str:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    try:
        value = data.get("preset_key", "bearing_par_sweep")
    except Exception:
        value = "bearing_par_sweep"
    return str(value or "bearing_par_sweep")


def is_runtime_custom_preset(payload: Dict[str, Any] | None = None) -> bool:
    return get_runtime_preset_key(payload).strip().lower() == "custom"


def load_runtime_values(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    values = data.get("values") if isinstance(data.get("values"), dict) else {}
    return values if isinstance(values, dict) else {}


def load_runtime_flyover(payload: Dict[str, Any] | None = None) -> Dict[str, Any]:
    data = payload if isinstance(payload, dict) else load_runtime_settings()
    flyover = data.get("flyover") if isinstance(data.get("flyover"), dict) else {}
    return flyover if isinstance(flyover, dict) else {}


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
