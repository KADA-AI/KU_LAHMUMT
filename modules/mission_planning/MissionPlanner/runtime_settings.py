from __future__ import annotations

import copy
import json
import threading
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Dict


_CACHE_LOCK = threading.Lock()
_CACHE_SIG: tuple[int, int] | None = None
_CACHE_DATA: Dict[str, Any] | None = None
_THREAD_LOCAL = threading.local()


def settings_path() -> Path:
    return Path(__file__).resolve().parent / "uav_params.json"


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
