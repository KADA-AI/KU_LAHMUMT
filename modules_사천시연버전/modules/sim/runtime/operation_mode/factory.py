from __future__ import annotations

from typing import Any

from .auto_tracking import ModeAutoTracking
from .aircraft_fixed import ModeAircraftFixed
from .coordinate_designate import ModeCoordinateDesignate
from .line_search import ModeLineSearch
from .base import OperationMode


def build_operation_mode(mode_id: int, **_: Any) -> OperationMode:
    """Factory: return concrete instance for supported modes 1~4."""
    if mode_id == 1:
        return ModeCoordinateDesignate()
    if mode_id == 2:
        return ModeLineSearch()
    if mode_id == 3:
        return ModeAutoTracking()
    if mode_id == 4:
        return ModeAircraftFixed()
    raise ValueError(f"Unsupported operation mode: {mode_id}")
