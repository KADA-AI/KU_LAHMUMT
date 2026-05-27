from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from .base import OperationContext, OperationMode, OperationResult


def _ray_intersect_ground(
    origin: tuple[float, float, float],
    direction: tuple[float, float, float],
    ground_z: float,
) -> tuple[float, float, float] | None:
    dz = direction[2]
    if abs(dz) < 1e-6:
        return None
    t = (ground_z - origin[2]) / dz
    if t <= 0:
        return None
    return (
        origin[0] + direction[0] * t,
        origin[1] + direction[1] * t,
        origin[2] + direction[2] * t,
    )


@dataclass
class ModeAircraftFixed(OperationMode):
    """Mode 4: keep gimbal fixed relative to aircraft orientation."""

    mode_id: int = 4

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
        aircraft_fixed = filming_prop.get("aircraftFixed") or {}
        pitch_deg = float(aircraft_fixed.get("gimbalPitch", 0.0))
        yaw_offset_deg = float(aircraft_fixed.get("gimbalYaw", 0.0))
        yaw_deg = float(getattr(uav.s, "yaw", 0.0)) + yaw_offset_deg
        yaw_rad = math.radians(yaw_deg)
        pitch_rad = math.radians(pitch_deg)
        cp = math.cos(pitch_rad)
        direction = (
            cp * math.cos(yaw_rad),
            -cp * math.sin(yaw_rad),
            math.sin(pitch_rad),
        )
        origin = (float(uav.s.x), float(uav.s.y), float(uav.s.z))
        ground_z = ctx.ground_height_fn(origin[0], origin[1])
        hit = _ray_intersect_ground(origin, direction, ground_z)
        if hit is not None:
            return OperationResult(target=hit, state=None)
        target = (
            origin[0] + direction[0] * 2000.0,
            origin[1] + direction[1] * 2000.0,
            origin[2] + direction[2] * 2000.0,
        )
        return OperationResult(target=target, state=None)
