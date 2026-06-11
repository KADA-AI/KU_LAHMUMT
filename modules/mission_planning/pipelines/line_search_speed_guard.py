from __future__ import annotations

import math
from typing import Any

_DEFAULT_MIN_TRANSIT_M = 1.0
_DEFAULT_SPEED_MULTIPLIER = 16.0


def _runtime_float(key: str, default: float | None) -> float | None:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float

        value = get_runtime_float(key, default if default is not None else 0.0)
    except Exception:
        value = default
    try:
        if value is None:
            return None
        numeric = float(value)
    except Exception:
        return default
    return float(numeric) if math.isfinite(numeric) else default


def line_search_speed_min_transit_m() -> float:
    configured = _runtime_float("line_search_speed_min_transit_m", None)
    if configured is None:
        configured = _runtime_float("min_route_spacing_m", _DEFAULT_MIN_TRANSIT_M)
    try:
        value = float(configured)
    except Exception:
        value = _DEFAULT_MIN_TRANSIT_M
    if not math.isfinite(value) or value <= 0.0:
        value = _DEFAULT_MIN_TRANSIT_M
    return max(1.0, min(10000.0, float(value)))


def effective_line_search_transit_m(transit_m: Any) -> float:
    try:
        value = float(transit_m)
    except Exception:
        return 0.0
    if not math.isfinite(value) or value <= 1e-6:
        return 0.0
    return max(float(value), line_search_speed_min_transit_m())


def line_search_speed_cap_mps(
    *,
    cruise_speed_mps: Any,
    speed_scale: Any = 1.0,
) -> float | None:
    explicit_cap = _runtime_float("line_search_speed_max_mps", 0.0)
    if explicit_cap is not None and explicit_cap > 0.0:
        return max(1.0, float(explicit_cap))
    try:
        cruise = float(cruise_speed_mps)
    except Exception:
        cruise = 0.0
    if not math.isfinite(cruise) or cruise <= 0.0:
        return None
    multiplier = _runtime_float("default_search_speed_multiplier", _DEFAULT_SPEED_MULTIPLIER)
    try:
        multiplier_value = float(multiplier)
    except Exception:
        multiplier_value = _DEFAULT_SPEED_MULTIPLIER
    if not math.isfinite(multiplier_value) or multiplier_value <= 0.0:
        multiplier_value = _DEFAULT_SPEED_MULTIPLIER
    try:
        scale = max(float(speed_scale), 0.1)
    except Exception:
        scale = 1.0
    if not math.isfinite(scale) or scale <= 0.0:
        scale = 1.0
    return max(1.0, float(cruise) * float(multiplier_value) * float(scale))


def clamp_line_search_speed_mps(
    speed_mps: Any,
    *,
    cruise_speed_mps: Any,
    speed_scale: Any = 1.0,
    minimum_speed_mps: Any = 0.0,
) -> float:
    try:
        speed = float(speed_mps)
    except Exception:
        speed = 0.0
    if not math.isfinite(speed) or speed <= 0.0:
        speed = 0.0
    cap = line_search_speed_cap_mps(
        cruise_speed_mps=cruise_speed_mps,
        speed_scale=speed_scale,
    )
    if cap is not None and cap > 0.0:
        speed = min(float(speed), float(cap))
    try:
        minimum = float(minimum_speed_mps)
    except Exception:
        minimum = 0.0
    if math.isfinite(minimum) and minimum > 0.0:
        speed = max(float(speed), float(minimum))
    return max(0.0, float(speed))
