from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .base import OperationContext, OperationMode, OperationResult


@dataclass
class ModeCoordinateDesignate(OperationMode):
    """Mode 1: coordinate designate."""

    mode_id: int = 1

    def apply(
        self,
        *,
        uav: Any,
        filming_prop: dict,
        ctx: OperationContext,
        dt: float,
        current_wp_id: int | None,
        prev_state: Any,
    ) -> OperationResult:
        coord_orient = filming_prop.get("coordinateOrientation") or {}
        coord = coord_orient.get("coordinate") or {}
        lon = coord.get("longitude")
        lat = coord.get("latitude")
        alt = coord.get("altitude", 0.0)

        if lon is None or lat is None:
            return OperationResult(target=ctx.default_target_fn(uav), state=None)

        try:
            x, y = ctx.geo.lonlat_to_xy(lon, lat)
            if alt is None or float(alt) == 0.0:
                z = ctx.ground_height_fn(x, y)
            else:
                z = float(alt)
            return OperationResult(target=(float(x), float(y), float(z)), state=None)
        except Exception:
            return OperationResult(target=ctx.default_target_fn(uav), state=None)
