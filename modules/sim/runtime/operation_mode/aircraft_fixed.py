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
        yaw_rad = math.radians(float(getattr(uav.s, "yaw", 0.0)))
        gimbal_pitch_rad = math.radians(pitch_deg)
        gimbal_yaw_rad = math.radians(yaw_offset_deg)
        if bool(getattr(ctx, "use_aircraft_attitude", False)):
            aircraft_pitch_rad = math.radians(float(getattr(uav.s, "pitch", 0.0)))
            aircraft_roll_rad = math.radians(float(getattr(uav.s, "roll", 0.0)))

            cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
            cp, sp = math.cos(aircraft_pitch_rad), math.sin(aircraft_pitch_rad)
            cr, sr = math.cos(aircraft_roll_rad), math.sin(aircraft_roll_rad)

            forward = (cp * cy, -cp * sy, sp)
            right_level = (-sy, -cy, 0.0)
            up_level = (-sp * cy, sp * sy, cp)
            right = tuple(
                right_level[i] * cr - up_level[i] * sr for i in range(3)
            )
            up = tuple(right_level[i] * sr + up_level[i] * cr for i in range(3))

            cg, sg = math.cos(gimbal_pitch_rad), math.sin(gimbal_pitch_rad)
            cgy, sgy = math.cos(gimbal_yaw_rad), math.sin(gimbal_yaw_rad)
            direction = tuple(
                cg * cgy * forward[i] + cg * sgy * right[i] + sg * up[i]
                for i in range(3)
            )
        else:
            yaw_with_offset = yaw_rad + gimbal_yaw_rad
            cp = math.cos(gimbal_pitch_rad)
            direction = (
                cp * math.cos(yaw_with_offset),
                -cp * math.sin(yaw_with_offset),
                math.sin(gimbal_pitch_rad),
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
