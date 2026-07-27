from __future__ import annotations

import json
import math
from pathlib import Path
from threading import Lock
from typing import Any


DEFAULT_UAV_DYNAMICS_PROFILE_PATH = (
    Path(__file__).resolve().parents[1]
    / "sim"
    / "runtime"
    / "controllers"
    / "uav_dynamics_profile.json"
)

_PROFILE_CACHE_LOCK = Lock()
_PROFILE_CACHE: dict[str, tuple[tuple[int, int], dict[str, Any]]] = {}


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _enabled(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().casefold() not in {"", "0", "false", "no", "off"}
    return bool(value)


def load_uav_dynamics_profile(path: str | Path | None = None) -> dict[str, Any]:
    """Load the enabled SIM dynamics profile with a stat-based cache."""

    profile_path = Path(path) if path is not None else DEFAULT_UAV_DYNAMICS_PROFILE_PATH
    resolved_path = profile_path.resolve()
    cache_key = str(resolved_path)
    try:
        stat = resolved_path.stat()
        signature = (int(stat.st_mtime_ns), int(stat.st_size))
    except Exception:
        with _PROFILE_CACHE_LOCK:
            _PROFILE_CACHE.pop(cache_key, None)
        return {}

    with _PROFILE_CACHE_LOCK:
        cached = _PROFILE_CACHE.get(cache_key)
        if cached is not None and cached[0] == signature:
            return dict(cached[1])

    payload: dict[str, Any] = {}
    try:
        loaded = json.loads(resolved_path.read_text(encoding="utf-8"))
        if isinstance(loaded, dict) and _enabled(loaded.get("enabled", False)):
            payload = loaded
    except Exception:
        payload = {}

    with _PROFILE_CACHE_LOCK:
        _PROFILE_CACHE[cache_key] = (signature, payload)
    return dict(payload)


def uav_dynamics_params(
    aircraft_id: int | None = None,
    *,
    path: str | Path | None = None,
) -> dict[str, Any]:
    """Return global profile parameters merged with one aircraft override."""

    payload = load_uav_dynamics_profile(path)
    params = payload.get("params")
    merged = dict(params) if isinstance(params, dict) else {}

    overrides = payload.get("aircraft_overrides")
    if not isinstance(overrides, dict):
        overrides = payload.get("aircraftOverrides")
    if aircraft_id is None or not isinstance(overrides, dict):
        return merged

    override = overrides.get(str(int(aircraft_id)))
    if isinstance(override, dict) and isinstance(override.get("params"), dict):
        override = override["params"]
    if isinstance(override, dict):
        merged.update(override)
    return merged


def reference_turn_radius_scale_for_aircraft(
    aircraft_id: int | None = None,
    *,
    path: str | Path | None = None,
    default: float = 1.0,
) -> float:
    params = uav_dynamics_params(aircraft_id, path=path)
    parsed = _finite_float(params.get("reference_turn_radius_scale"))
    if parsed is None or parsed <= 0.0:
        parsed = _finite_float(default)
    if parsed is None or parsed <= 0.0:
        parsed = 1.0
    return max(0.05, min(float(parsed), 5.0))


__all__ = [
    "DEFAULT_UAV_DYNAMICS_PROFILE_PATH",
    "load_uav_dynamics_profile",
    "reference_turn_radius_scale_for_aircraft",
    "uav_dynamics_params",
]
