from __future__ import annotations

"""
Helpers for deriving search-speed values from sweep spacing.
"""

from typing import Optional

try:
    from ..config import SEARCH_SPEED_WEIGHT as _CFG_WEIGHT
except ImportError:
    try:
        from config import SEARCH_SPEED_WEIGHT as _CFG_WEIGHT  # type: ignore
    except ImportError:
        try:
            from modules.mission_planning.MissionPlanner.config import SEARCH_SPEED_WEIGHT as _CFG_WEIGHT  # type: ignore
        except ImportError:
            _CFG_WEIGHT = 1.0


def spacing_based_search_speed(
    *,
    sweep_len_m: Optional[float],
    spacing_m: Optional[float],
    cruise_speed_mps: Optional[float],
    weight: Optional[float] = None,
) -> Optional[float]:
    """
    Estimate search speed so that the platform clears the sweep length
    while the aircraft advances by one spacing interval.
    Returns None when inputs are invalid.
    """
    if sweep_len_m is None or spacing_m is None or cruise_speed_mps is None:
        return None

    if spacing_m <= 0.0 or cruise_speed_mps <= 0.0:
        return None

    if sweep_len_m <= 0.0:
        return 0.0

    effective_weight = _CFG_WEIGHT if weight is None else weight
    try:
        ratio = cruise_speed_mps / spacing_m
        search_speed = sweep_len_m * ratio
        return round(max(0.0, search_speed * effective_weight), 2)
    except Exception:
        return None
