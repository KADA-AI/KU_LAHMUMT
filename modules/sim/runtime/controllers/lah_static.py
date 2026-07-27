from __future__ import annotations

import math
from typing import Iterable, Sequence, Tuple

from ...config import clamp
from ..lah import DEFAULT_ENVELOPE
from .waypoint_pid import PIDGains, WaypointPIDController, WaypointTarget


LAH_STATIC_VERTICAL_RATE_SCALE = 3.0


class LAHStaticWaypointController(WaypointPIDController):
    """Deterministic LAH waypoint motion that does not use PID dynamics."""

    updates_vehicle_state_directly = True

    def __init__(
        self,
        lah,
        waypoints: Sequence[WaypointTarget] | Iterable[Tuple[float, float, float]],
        *,
        gains: PIDGains | None = None,
        speed_target: float = 70.0,
        pos_tol: float = 30.0,
        name: str | None = None,
        ground_clearance: float = 50.0,
        allow_hover: bool = True,
        hold_on_complete: bool = False,
        climb_rate_mps: float = (
            DEFAULT_ENVELOPE.climb_rate_mps * LAH_STATIC_VERTICAL_RATE_SCALE
        ),
        descent_rate_mps: float = (
            DEFAULT_ENVELOPE.descent_rate_mps * LAH_STATIC_VERTICAL_RATE_SCALE
        ),
    ) -> None:
        # Inherit only the waypoint/mission state contract used by SimulationService.
        # update() below never consumes the PID gains or LAH command dynamics.
        super().__init__(
            lah,
            waypoints,
            gains=gains,
            speed_target=speed_target,
            pos_tol=pos_tol,
            name=name,
            ground_clearance=ground_clearance,
            allow_hover=allow_hover,
            hold_on_complete=hold_on_complete,
        )
        self.climb_rate_mps = max(0.0, float(climb_rate_mps))
        self.descent_rate_mps = max(0.0, float(descent_rate_mps))
        self._static_loiter_acquired = False

    def _advance_wp(self) -> None:
        super()._advance_wp()
        self._static_loiter_acquired = False

    def _arrival_tolerances(self) -> tuple[float, float]:
        # Keep mission events at the actual waypoint instead of cutting corners by
        # the broad PID capture radius (normally 30 m).
        xy_tol = max(0.05, min(1.0, float(self.pos_tol) * 0.1))
        z_tol = max(0.05, min(1.0, float(self.pos_tol) * 0.06))
        return xy_tol, z_tol

    def _resolved_target_position(
        self, target: WaypointTarget, dem=None
    ) -> tuple[float, float, float]:
        tx, ty, tz = (float(target.pos[0]), float(target.pos[1]), float(target.pos[2]))
        if dem is not None:
            try:
                tz = max(
                    tz, float(dem.get_height(tx, ty)) + float(self.ground_clearance)
                )
            except Exception:
                pass
        return tx, ty, tz

    def _target_speed(self, target: WaypointTarget | None) -> float:
        value = (
            target.speed
            if target is not None and target.speed is not None
            else self.speed_target
        )
        try:
            speed = float(value)
        except Exception:
            speed = float(self.speed_target)
        if not math.isfinite(speed):
            speed = float(self.speed_target)
        return max(0.0, speed)

    def _reset_commands(self, *, hover: bool) -> None:
        lah = self.uav
        if hasattr(lah, "hover_mode"):
            lah.hover_mode = bool(hover)
        if hasattr(lah, "cmd_vertical_speed"):
            lah.cmd_vertical_speed = 0.0
        lah.cmd_throttle = 0.0
        lah.cmd_yaw_rate = 0.0
        lah.cmd_pitch_rate = 0.0
        lah.cmd_roll_rate = 0.0
        lah.s.p = 0.0
        lah.s.q = 0.0
        lah.s.r = 0.0
        lah.s.pitch = 0.0
        lah.s.roll = 0.0

    def _hold_position(self) -> None:
        self._reset_commands(hover=True)
        self.uav.s.u = 0.0

    def _move_toward(
        self,
        target_pos: tuple[float, float, float],
        *,
        speed_mps: float,
        dt: float,
    ) -> bool:
        lah = self.uav
        s = lah.s
        tx, ty, tz = target_pos
        dx = tx - float(s.x)
        dy = ty - float(s.y)
        dz = tz - float(s.z)
        dist_xy = math.hypot(dx, dy)

        travelled_xy = 0.0
        if dist_xy > 1e-9 and speed_mps > 0.0:
            travelled_xy = min(dist_xy, speed_mps * dt)
            ratio = travelled_xy / dist_xy
            s.x = float(s.x) + dx * ratio
            s.y = float(s.y) + dy * ratio
            s.yaw = self._heading_to_target(dx, dy)

        if dz > 0.0:
            s.z = float(s.z) + min(dz, self.climb_rate_mps * dt)
        elif dz < 0.0:
            s.z = max(0.0, float(s.z) - min(-dz, self.descent_rate_mps * dt))

        self._reset_commands(hover=False)
        s.u = travelled_xy / dt if dt > 0.0 else 0.0
        xy_tol, z_tol = self._arrival_tolerances()
        return (
            math.hypot(tx - float(s.x), ty - float(s.y)) <= xy_tol
            and abs(tz - float(s.z)) <= z_tol
        )

    def _start_loiter(
        self,
        target: WaypointTarget,
        target_pos: tuple[float, float, float],
        loiter_prop: dict,
        loiter_time: float,
    ) -> None:
        self.is_loitering = True
        self.is_hovering = False
        self.loiter_timer = float(loiter_time)
        self.loiter_center = target_pos
        try:
            radius = float(loiter_prop.get("radius", 0.0) or 0.0)
        except Exception:
            radius = 0.0
        self.loiter_radius = (
            radius if radius > 1.0 else float(self.default_loiter_radius)
        )
        try:
            speed = float(loiter_prop.get("speed", 0.0) or self._target_speed(target))
        except Exception:
            speed = self._target_speed(target)
        self.loiter_speed = max(0.0, speed)
        self.loiter_dir = -1.0 if loiter_prop.get("direction", 1) == 1 else 1.0
        self.loiter_angle = math.atan2(
            float(self.uav.s.y) - target_pos[1],
            float(self.uav.s.x) - target_pos[0],
        )
        self.loiter_angle_locked = True
        self._static_loiter_acquired = False

    def _step_loiter(self, dt: float) -> bool:
        center_x, center_y, center_z = self.loiter_center
        radius = max(1.0, float(self.loiter_radius))
        speed = max(0.0, float(self.loiter_speed or self.speed_target))

        if not self._static_loiter_acquired:
            orbit_entry = (
                float(center_x) + radius * math.cos(self.loiter_angle),
                float(center_y) + radius * math.sin(self.loiter_angle),
                float(center_z),
            )
            if self._move_toward(orbit_entry, speed_mps=speed, dt=dt):
                self.uav.s.x, self.uav.s.y, self.uav.s.z = orbit_entry
                self._static_loiter_acquired = True
            return True

        previous_x = float(self.uav.s.x)
        previous_y = float(self.uav.s.y)
        angular_rate = speed / radius if radius > 0.0 else 0.0
        self.loiter_angle += self.loiter_dir * angular_rate * dt
        next_x = float(center_x) + radius * math.cos(self.loiter_angle)
        next_y = float(center_y) + radius * math.sin(self.loiter_angle)
        dx = next_x - previous_x
        dy = next_y - previous_y
        self.uav.s.x = next_x
        self.uav.s.y = next_y
        if center_z > float(self.uav.s.z):
            self.uav.s.z = float(self.uav.s.z) + min(
                float(center_z) - float(self.uav.s.z),
                self.climb_rate_mps * dt,
            )
        elif center_z < float(self.uav.s.z):
            self.uav.s.z = max(
                0.0,
                float(self.uav.s.z)
                - min(
                    float(self.uav.s.z) - float(center_z), self.descent_rate_mps * dt
                ),
            )
        if abs(dx) + abs(dy) > 1e-9:
            self.uav.s.yaw = self._heading_to_target(dx, dy)
        self._reset_commands(hover=False)
        self.uav.s.u = math.hypot(dx, dy) / dt if dt > 0.0 else 0.0
        return True

    def update(self, dt: float, dem=None, wall_dt: float | None = None) -> bool:
        del wall_dt  # Mission timers intentionally use deterministic simulation time.
        if (self.finished and not self.is_loitering) or not self.targets:
            self._hold_position()
            return False

        dt = clamp(float(dt), 0.001, 0.5)
        self._reset_commands(hover=False)
        if self.curr_idx < len(self.targets) and self._skip_done_targets():
            self.just_advanced = True
            self.advance_reason = "done"
        if self.curr_idx >= len(self.targets):
            self.finished = True
            self._hold_position()
            return False
        self.just_advanced = False

        target = self.targets[self.curr_idx]
        target_pos = self._resolved_target_position(target, dem)
        target_hover = (
            float(target.hover_time) if self.allow_hover and target.hover_time else 0.0
        )

        if self.blocked and (self.allow_hover or self.force_hover):
            target_hover = max(target_hover, 1.0)
            if not self.is_hovering:
                self.is_hovering = True
                self.hover_timer = math.inf

        if (
            (self.allow_hover or self.force_hover)
            and self.is_hovering
            and target_hover > 0.0
        ):
            self.hover_timer = max(0.0, self.hover_timer - dt)
            if self.hover_timer <= 0.0:
                self.is_hovering = False
                self.force_hover = False
                self._advance_wp()
                self._hold_position()
                return not self.finished
            self._hold_position()
            return True

        if self.is_loitering:
            self.loiter_timer = max(0.0, self.loiter_timer - dt)
            if self.loiter_timer <= 0.0:
                self.is_loitering = False
                self._advance_wp()
                self._hold_position()
                return not self.finished
            return self._step_loiter(dt)

        tx, ty, tz = target_pos
        dist_xy = math.hypot(tx - float(self.uav.s.x), ty - float(self.uav.s.y))
        dz = tz - float(self.uav.s.z)
        xy_tol, z_tol = self._arrival_tolerances()
        if dist_xy <= xy_tol and abs(dz) <= z_tol:
            self.uav.s.x = tx
            self.uav.s.y = ty
            self.uav.s.z = tz
            self._hold_position()

            if not self.blocked and self.curr_idx in self.block_indices:
                self._enter_block(tx, ty, tz, target)
                return True

            loiter_prop = target.loiter if isinstance(target.loiter, dict) else None
            if loiter_prop is None and int(target.pass_type or 0) == 2:
                loiter_prop = {
                    "radius": self.default_loiter_radius,
                    "direction": 1,
                    "time": 30.0,
                    "speed": self._target_speed(target),
                }
                target.loiter = loiter_prop
            if loiter_prop is not None:
                try:
                    loiter_time = float(loiter_prop.get("time", 0.0) or 0.0)
                except Exception:
                    loiter_time = 0.0
                if loiter_time <= 0.0 and int(target.pass_type or 0) == 2:
                    loiter_time = 30.0
                if loiter_time > 0.0:
                    self._start_loiter(target, target_pos, loiter_prop, loiter_time)
                    return True

            if self.allow_hover and target_hover > 0.0:
                self.is_hovering = True
                self.hover_timer = target_hover
                return True

            self._advance_wp()
            self._hold_position()
            return not self.finished

        self._move_toward(target_pos, speed_mps=self._target_speed(target), dt=dt)
        return True
