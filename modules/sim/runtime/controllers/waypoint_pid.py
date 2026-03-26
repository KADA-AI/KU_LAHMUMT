from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

from ...config import clamp, wrap_deg


@dataclass
class PIDGains:
    yaw: float = 0.9
    yaw_i: float = 0.0
    pitch_rate: float = 1.4
    pitch_damp: float = 1.0
    alt_pitch: float = 0.03
    alt_i: float = 0.0
    throttle: float = 0.1
    throttle_alt: float = 0.0008
    throttle_i: float = 0.0
    lookahead_m: float = 120.0
    freeze_yaw_dist: float = 50.0
    freeze_alt_ratio: float = 0.5


DEFAULT_TUNED_GAINS = PIDGains(
    yaw=0.7034928181628536,
    yaw_i=0.024126824285276113,
    pitch_rate=3.2,
    pitch_damp=0.2,
    alt_pitch=0.09,
    alt_i=0.2,
    throttle=0.02,
    throttle_alt=0.0025000000000000005,
    throttle_i=0.026169406198635482,
    lookahead_m=196.375,
    freeze_yaw_dist=62.604277298027895,
    freeze_alt_ratio=0.5,
)


@dataclass
class WaypointTarget:
    pos: Tuple[float, float, float]
    speed: float | None = None
    filming: dict | None = None
    wp_id: int | None = None
    is_done: bool = False
    hover_time: float | None = None
    loiter: dict | None = None
    attack: dict | None = None
    input_mission_id: int | None = None
    individual_mission_id: int | None = None
    path_id: int | None = None
    pass_type: int | None = None
    sep_m: float | None = None


def _merge_gains(data: dict, fallback: PIDGains) -> PIDGains:
    merged = {**fallback.__dict__}
    for key in PIDGains.__dataclass_fields__:
        if key in data:
            merged[key] = data[key]
    return PIDGains(**merged)


def load_pid_gains(path: str | Path | None = None, fallback: PIDGains | None = None) -> PIDGains:
    fallback = DEFAULT_TUNED_GAINS if fallback is None else fallback
    if path is None:
        return fallback

    p = Path(path)
    if not p.exists():
        return fallback

    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        if isinstance(data, dict) and "gains" in data and isinstance(data["gains"], dict):
            data = data["gains"]
        if not isinstance(data, dict):
            return fallback
        return _merge_gains(data, fallback)
    except Exception as exc:
        print(f"[pid] failed to load gains from {p}: {exc}")
        return fallback


def _load_gain_db(path: str | Path) -> list[dict] | None:
    p = Path(path)
    if not p.exists():
        return None
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        if isinstance(data, dict) and isinstance(data.get("records"), list):
            return list(data["records"])
    except Exception:
        return None
    return None


def load_pid_gains_for_time_scale(
    db_path: str | Path | None,
    time_scale: float,
    *,
    fallback: PIDGains | None = None,
) -> PIDGains:
    fallback = DEFAULT_TUNED_GAINS if fallback is None else fallback
    if not db_path:
        return fallback
    records = _load_gain_db(db_path)
    if not records:
        return fallback
    best = None
    best_diff = float("inf")
    for rec in records:
        try:
            ts = float(rec.get("time_scale"))
        except Exception:
            continue
        diff = abs(ts - float(time_scale))
        if diff < best_diff:
            best = rec
            best_diff = diff
    gains_dict = best.get("gains") if isinstance(best, dict) else None
    if not isinstance(gains_dict, dict):
        return fallback
    return _merge_gains(gains_dict, fallback)


class WaypointPIDController:
    def __init__(
        self,
        uav,
        waypoints: Sequence[WaypointTarget] | Iterable[Tuple[float, float, float]],
        *,
        gains: PIDGains | None = None,
        speed_target: float = 90.0,
        pos_tol: float = 30.0,
        name: str | None = None,
        ground_clearance: float = 60.0,
        allow_hover: bool = False,
        hold_on_complete: bool = False,
    ):
        self.uav = uav
        self.gains = gains or DEFAULT_TUNED_GAINS
        self.speed_target = speed_target
        self.pos_tol = pos_tol
        self.name = name or "uav"
        self.ground_clearance = ground_clearance
        self.allow_hover = allow_hover

        self.targets: List[WaypointTarget] = []
        for wp in waypoints:
            if isinstance(wp, WaypointTarget):
                self.targets.append(wp)
            elif isinstance(wp, dict) and "pos" in wp:
                pos = wp["pos"]
                filming = wp.get("filming")
                wp_id = wp.get("wp_id")
                hover_time = wp.get("hover_time")
                loiter = wp.get("loiter")
                attack = wp.get("attack")
                input_mission_id = wp.get("input_mission_id")
                individual_mission_id = wp.get("individual_mission_id")
                path_id = wp.get("path_id")
                pass_type = wp.get("pass_type")
                sep_m = wp.get("sep_m")
                if isinstance(pos, (list, tuple)) and len(pos) == 3:
                    self.targets.append(
                        WaypointTarget(
                            pos=(float(pos[0]), float(pos[1]), float(pos[2])),
                            speed=wp.get("speed"),
                            filming=filming,
                            wp_id=int(wp_id) if wp_id is not None else None,
                            is_done=bool(wp.get("is_done", False)),
                            hover_time=float(hover_time) if hover_time is not None else None,
                            loiter=loiter,
                            attack=attack if isinstance(attack, dict) else None,
                            input_mission_id=int(input_mission_id) if input_mission_id is not None else None,
                            individual_mission_id=int(individual_mission_id)
                            if individual_mission_id is not None
                            else None,
                            path_id=int(path_id) if path_id is not None else None,
                            pass_type=int(pass_type) if pass_type is not None else None,
                            sep_m=float(sep_m) if sep_m is not None else None,
                        )
                    )
            elif isinstance(wp, (list, tuple)) and len(wp) == 3:
                self.targets.append(WaypointTarget(pos=(float(wp[0]), float(wp[1]), float(wp[2]))))

        self.curr_idx = 0
        self.finished = False
        self.block_indices: dict[int, int] = {}
        self.blocked = False
        self.blocked_input_id: int | None = None
        self._blocked_idx: int | None = None
        self.input_ids: set[int] = set()
        self.default_loiter_radius = 160.0 if self.allow_hover else 300.0
        self.force_hover = False
        self.hold_on_complete = bool(hold_on_complete)

        self.yaw_int = 0.0
        self.alt_int = 0.0
        self.speed_int = 0.0
        self.yaw_int_max = 100.0
        self.alt_int_max = 10.0
        self.speed_int_max = 10.0
        self.hover_timer = 0.0
        self.is_hovering = False
        self.is_loitering = False
        self.loiter_timer = 0.0
        self.loiter_center = (0.0, 0.0, 0.0)
        self.loiter_radius = 0.0
        self.loiter_speed = 0.0
        self.loiter_dir = 1.0
        self.loiter_angle = 0.0
        self.loiter_angle_locked = False
        self._closest_wp_dist_xy = float("inf")
        self._closest_wp_idx = self.curr_idx

    def set_block_indices(self, block_indices: dict[int, int] | None = None) -> None:
        self.block_indices = dict(block_indices or {})
        self.input_ids = {int(v) for v in self.block_indices.values() if v is not None}

    def has_input_mission(self, input_id: int | None) -> bool:
        if input_id is None:
            return False
        return input_id in self.input_ids

    def release_block(self) -> bool:
        if not self.blocked:
            return False
        self.blocked = False
        self.blocked_input_id = None
        self._blocked_idx = None
        self.is_loitering = False
        self.is_hovering = False
        self.force_hover = False
        self.loiter_timer = 0.0
        self.hover_timer = 0.0
        self._advance_wp()
        return True

    def _enter_block(self, tx: float, ty: float, tz: float, target: WaypointTarget) -> None:
        self.blocked = True
        self._blocked_idx = self.curr_idx
        self.blocked_input_id = self.block_indices.get(self.curr_idx)
        self.force_hover = False
        self.is_hovering = False
        self.hover_timer = 0.0
        self.just_advanced = False
        self.advance_reason = None
        self.advance_reason: str | None = None
        self.is_loitering = True
        self.loiter_timer = math.inf
        self.loiter_center = (tx, ty, tz)
        loiter_prop = target.loiter if isinstance(target.loiter, dict) else None
        try:
            self.loiter_radius = float(loiter_prop.get("radius", 0.0) or 0.0) if loiter_prop else 0.0
        except Exception:
            self.loiter_radius = 0.0
        if self.loiter_radius <= 1.0:
            self.loiter_radius = self.default_loiter_radius
        try:
            self.loiter_speed = float(loiter_prop.get("speed", 0.0) or 0.0) if loiter_prop else 0.0
        except Exception:
            self.loiter_speed = 0.0
        if self.loiter_speed <= 0.0:
            self.loiter_speed = target.speed or self.speed_target
        direction = (loiter_prop or {}).get("direction", 1)
        self.loiter_dir = -1.0 if direction == 1 else 1.0
        self.loiter_angle = math.atan2(self.uav.s.y - ty, self.uav.s.x - tx)
        self.loiter_angle_locked = True

    def _heading_to_target(self, dx: float, dy: float) -> float:
        return wrap_deg(math.degrees(math.atan2(-dy, dx)))

    def _advance_wp(self):
        prev_loitering = bool(self.is_loitering)
        prev_hovering = bool(self.is_hovering)
        self.curr_idx += 1
        self.yaw_int = 0.0
        self.alt_int = 0.0
        self.speed_int = 0.0
        self.is_hovering = False
        self.hover_timer = 0.0
        self.is_loitering = False
        self.loiter_timer = 0.0
        self.loiter_angle_locked = False
        self._closest_wp_dist_xy = float("inf")
        self._closest_wp_idx = self.curr_idx
        self.just_advanced = True
        if prev_loitering:
            self.advance_reason = "loiter"
        elif prev_hovering:
            self.advance_reason = "hover"
        else:
            self.advance_reason = "move"
        if self.curr_idx >= len(self.targets):
            self.finished = True
            if self.hold_on_complete and self.targets:
                last_idx = max(0, len(self.targets) - 1)
                self.curr_idx = last_idx
                last = self.targets[last_idx]
                self.is_loitering = True
                self.loiter_timer = math.inf
                self.loiter_center = (float(last.pos[0]), float(last.pos[1]), float(last.pos[2]))
                self.loiter_radius = self.default_loiter_radius
                self.loiter_speed = last.speed or self.speed_target
                self.loiter_dir = 1.0
                self.loiter_angle = math.atan2(
                    self.uav.s.y - float(last.pos[1]),
                    self.uav.s.x - float(last.pos[0]),
                )
                self.loiter_angle_locked = True
                self.advance_reason = "loiter"
                return
            self._apply_hold()
            return

    def _apply_hold(self):
        self._set_hover_mode(False)
        self._set_vertical_speed_command(0.0)
        self.uav.cmd_yaw_rate = 0.0
        self.uav.cmd_pitch_rate = -self.uav.s.pitch * 1.0
        self.uav.cmd_roll_rate = -self.uav.s.roll * 1.5
        self.uav.cmd_throttle = -0.2

    def _set_hover_mode(self, enabled: bool) -> None:
        if not hasattr(self.uav, "hover_mode"):
            return
        try:
            self.uav.hover_mode = bool(enabled)
        except Exception:
            pass

    def _set_vertical_speed_command(self, value: float) -> None:
        if not hasattr(self.uav, "cmd_vertical_speed"):
            return
        try:
            self.uav.cmd_vertical_speed = float(value)
        except Exception:
            pass

    def _yaw_rate_limit(self) -> float:
        limit_fn = getattr(self.uav, "turn_rate_limit_dps", None)
        if callable(limit_fn):
            try:
                return clamp(float(limit_fn()), 0.0, float(self.uav.p.max_yaw_rate_dps))
            except Exception:
                pass
        return float(self.uav.p.max_yaw_rate_dps)

    def _apply_turn_roll_guidance(self, yaw_rate_limit: float) -> None:
        uav = self.uav
        if self.allow_hover:
            uav.cmd_roll_rate = clamp(-uav.s.roll * 1.5, -uav.p.max_roll_rate_dps, uav.p.max_roll_rate_dps)
            return

        bank_limit = min(
            abs(float(getattr(uav.p, "roll_limit_deg", 0.0))),
            abs(float(getattr(uav.p, "turn_bank_limit_deg", getattr(uav.p, "roll_limit_deg", 0.0)))),
        )
        if yaw_rate_limit <= 1e-6 or bank_limit <= 0.1:
            roll_target = 0.0
        else:
            roll_target = clamp(
                (float(uav.cmd_yaw_rate) / yaw_rate_limit) * bank_limit,
                -bank_limit,
                bank_limit,
            )
        roll_gain = max(0.5, float(getattr(uav.p, "turn_roll_gain", 2.0)))
        roll_err = roll_target - float(uav.s.roll)
        uav.cmd_roll_rate = clamp(roll_err * roll_gain, -uav.p.max_roll_rate_dps, uav.p.max_roll_rate_dps)

    def _target_pass_type(self, target: WaypointTarget | None) -> int:
        try:
            return int(target.pass_type or 0) if target is not None else 0
        except Exception:
            return 0

    def _is_flyover_target(self, target: WaypointTarget | None) -> bool:
        return (not self.allow_hover) and self._target_pass_type(target) == 3

    def _fixed_wing_guidance_target(
        self,
        tx: float,
        ty: float,
        tz: float,
        dist_xy: float,
        gains: PIDGains,
    ) -> tuple[float, float, float]:
        if self.allow_hover or self.is_loitering or self.curr_idx == 0 or self.curr_idx + 1 >= len(self.targets):
            return tx, ty, tz
        if self._is_flyover_target(self.targets[self.curr_idx]):
            return tx, ty, tz

        speed_fn = getattr(self.uav, "horizontal_speed_mps", None)
        if callable(speed_fn):
            try:
                speed_xy = max(0.0, float(speed_fn()))
            except Exception:
                speed_xy = max(0.0, float(getattr(self.uav.s, "u", 0.0)))
        else:
            speed_xy = max(0.0, float(getattr(self.uav.s, "u", 0.0)))

        yaw_rate_limit = self._yaw_rate_limit()
        if speed_xy <= 1e-3 or yaw_rate_limit <= 1e-3:
            return tx, ty, tz

        turn_radius = speed_xy / max(1e-6, math.radians(yaw_rate_limit))
        anticipation_dist = max(float(gains.lookahead_m), float(self.pos_tol) * 2.0, turn_radius * 0.45)
        if dist_xy >= anticipation_dist:
            return tx, ty, tz

        next_target = self.targets[self.curr_idx + 1]
        nx = float(next_target.pos[0]) - float(tx)
        ny = float(next_target.pos[1]) - float(ty)
        nz = float(next_target.pos[2]) - float(tz)
        next_dist = math.hypot(nx, ny)
        if next_dist <= 1e-6:
            return tx, ty, tz

        lead_dist = clamp(anticipation_dist - dist_xy, 0.0, next_dist)
        blend = clamp(lead_dist / next_dist, 0.0, 1.0)
        return (
            float(tx) + nx / next_dist * lead_dist,
            float(ty) + ny / next_dist * lead_dist,
            float(tz) + nz * blend,
        )

    def _fixed_wing_capture_radius(self, gains: PIDGains, *, flyover: bool = False) -> float:
        if self.allow_hover:
            return float(self.pos_tol)

        speed_fn = getattr(self.uav, "horizontal_speed_mps", None)
        if callable(speed_fn):
            try:
                speed_xy = max(0.0, float(speed_fn()))
            except Exception:
                speed_xy = max(0.0, float(getattr(self.uav.s, "u", 0.0)))
        else:
            speed_xy = max(0.0, float(getattr(self.uav.s, "u", 0.0)))

        yaw_rate_limit = self._yaw_rate_limit()
        if speed_xy <= 1e-3 or yaw_rate_limit <= 1e-3:
            return float(self.pos_tol)

        turn_radius = speed_xy / max(1e-6, math.radians(yaw_rate_limit))
        if flyover:
            return max(
                float(self.pos_tol),
                min(turn_radius * 0.35, max(float(self.pos_tol) * 1.5, float(gains.lookahead_m) * 0.5)),
            )
        return max(
            float(self.pos_tol),
            min(turn_radius, max(turn_radius * 0.8, float(gains.lookahead_m) * 1.5)),
        )

    def _has_passed_waypoint(self, tx: float, ty: float) -> bool:
        if not self.targets:
            return False

        rel_x = float(self.uav.s.x) - float(tx)
        rel_y = float(self.uav.s.y) - float(ty)

        if self.curr_idx + 1 < len(self.targets):
            next_target = self.targets[self.curr_idx + 1]
            out_x = float(next_target.pos[0]) - float(tx)
            out_y = float(next_target.pos[1]) - float(ty)
            out_norm = math.hypot(out_x, out_y)
            if out_norm > 1e-6 and (rel_x * out_x + rel_y * out_y) >= 0.0:
                return True

        if self.curr_idx > 0:
            prev_target = self.targets[self.curr_idx - 1]
            in_x = float(tx) - float(prev_target.pos[0])
            in_y = float(ty) - float(prev_target.pos[1])
            in_norm = math.hypot(in_x, in_y)
            if in_norm > 1e-6 and (rel_x * in_x + rel_y * in_y) >= 0.0:
                return True

        return False

    def _has_crossed_flyover_plane(self, tx: float, ty: float) -> bool:
        rel_x = float(self.uav.s.x) - float(tx)
        rel_y = float(self.uav.s.y) - float(ty)

        if self.curr_idx > 0:
            prev_target = self.targets[self.curr_idx - 1]
            axis_x = float(tx) - float(prev_target.pos[0])
            axis_y = float(ty) - float(prev_target.pos[1])
        elif self.curr_idx + 1 < len(self.targets):
            next_target = self.targets[self.curr_idx + 1]
            axis_x = float(next_target.pos[0]) - float(tx)
            axis_y = float(next_target.pos[1]) - float(ty)
        else:
            return False

        axis_norm = math.hypot(axis_x, axis_y)
        if axis_norm <= 1e-6:
            return False
        return (rel_x * axis_x + rel_y * axis_y) >= 0.0

    def _should_force_waypoint_advance(
        self,
        target: WaypointTarget,
        tx: float,
        ty: float,
        dist_xy: float,
        gains: PIDGains,
    ) -> bool:
        if self.allow_hover or self.is_loitering:
            return False

        if self._closest_wp_idx != self.curr_idx:
            self._closest_wp_idx = self.curr_idx
            self._closest_wp_dist_xy = float(dist_xy)
        else:
            self._closest_wp_dist_xy = min(float(self._closest_wp_dist_xy), float(dist_xy))

        is_flyover = self._is_flyover_target(target)
        capture_radius = self._fixed_wing_capture_radius(gains, flyover=is_flyover)
        if self._closest_wp_dist_xy > capture_radius:
            return False

        grow_margin = max(4.0, capture_radius * (0.05 if is_flyover else 0.08))
        if dist_xy <= self._closest_wp_dist_xy + grow_margin:
            return False

        if is_flyover:
            return self._has_crossed_flyover_plane(tx, ty)
        return self._has_passed_waypoint(tx, ty)

    def _apply_hover_control(self, tx: float, ty: float, tz: float, gains: PIDGains, dt: float) -> bool:
        uav = self.uav
        dx = tx - uav.s.x
        dy = ty - uav.s.y
        dz = tz - uav.s.z
        dist_xy = math.hypot(dx, dy)
        capture_radius = max(self.pos_tol * 1.25, 40.0)
        hold_xy = dist_xy <= capture_radius

        self._set_hover_mode(True)
        self.yaw_int = 0.0
        self.alt_int = 0.0
        self.speed_int = 0.0

        max_hover_yaw = uav.p.max_yaw_rate_dps * (0.25 if hold_xy else 0.35)
        if hold_xy or dist_xy <= 1e-3:
            uav.cmd_yaw_rate = clamp(-uav.s.r * 0.8, -max_hover_yaw, max_hover_yaw)
        else:
            desired_yaw = self._heading_to_target(dx, dy)
            yaw_err = ((desired_yaw - uav.s.yaw + 540.0) % 360.0) - 180.0
            yaw_err = clamp(yaw_err, -25.0, 25.0)
            yaw_gain = min(max(float(gains.yaw), 0.3), 0.9)
            uav.cmd_yaw_rate = clamp(yaw_err * yaw_gain, -max_hover_yaw, max_hover_yaw)

        uav.cmd_roll_rate = clamp(-uav.s.roll * 2.0, -uav.p.max_roll_rate_dps, uav.p.max_roll_rate_dps)

        if hasattr(uav, "cmd_vertical_speed"):
            uav.cmd_pitch_rate = clamp(
                -uav.s.pitch * max(1.2, float(gains.pitch_damp) + 0.4),
                -uav.p.max_pitch_rate_dps,
                uav.p.max_pitch_rate_dps,
            )
            max_vz = float(getattr(uav.p, "max_vertical_speed_mps", 6.0))
            vz_gain = 0.12 if hold_xy else 0.08
            vz_cmd = clamp(dz * vz_gain, -max_vz, max_vz)
            if abs(dz) < max(2.0, self.pos_tol * 0.15):
                vz_cmd = 0.0
            self._set_vertical_speed_command(vz_cmd)
        else:
            alt_err = tz - uav.s.z
            uav.cmd_pitch_rate = clamp(
                alt_err * gains.pitch_rate,
                -uav.p.max_pitch_rate_dps,
                uav.p.max_pitch_rate_dps,
            )

        desired_speed = 0.0 if hold_xy else min(max(2.0, dist_xy * 0.08), max(4.0, self.speed_target * 0.2))
        speed_err = desired_speed - uav.s.u
        throttle_cmd = speed_err * gains.throttle
        if hold_xy:
            throttle_cmd = min(throttle_cmd, 0.0)
        uav.cmd_throttle = clamp(throttle_cmd, -1.0, 0.35)
        return True

    def current_target(self) -> WaypointTarget | None:
        if (self.finished and not self.is_loitering) or not self.targets:
            return None
        if self.curr_idx >= len(self.targets):
            return None
        return self.targets[self.curr_idx]

    def update(self, dt: float, dem=None, wall_dt: float | None = None) -> bool:
        if (self.finished and not self.is_loitering) or not self.targets:
            return False

        if wall_dt is None:
            wall_dt = dt

        dt = clamp(dt, 0.001, 0.5)
        self._set_hover_mode(False)
        self._set_vertical_speed_command(0.0)
        if self.curr_idx >= len(self.targets):
            self.finished = True
            self._apply_hold()
            return False
        self.just_advanced = False

        target = self.targets[self.curr_idx]
        uav = self.uav
        gains = self.gains
        tx, ty, tz = target.pos

        if dem is not None:
            try:
                ground_at_target = dem.get_height(tx, ty)
                tz = max(tz, ground_at_target + self.ground_clearance)
            except Exception:
                pass

        dx = tx - self.uav.s.x
        dy = ty - self.uav.s.y
        dz = tz - self.uav.s.z
        dist_xy = math.hypot(dx, dy)

        if not self.blocked and not self.is_loitering and int(target.pass_type or 0) == 2:
            loiter_prop = target.loiter if isinstance(target.loiter, dict) else None
            if loiter_prop is None:
                loiter_prop = {
                    "radius": self.default_loiter_radius,
                    "direction": 1,
                    "time": 30.0,
                    "speed": target.speed or self.speed_target,
                }
                target.loiter = loiter_prop
            try:
                loiter_time = float(loiter_prop.get("time", 0.0) or 0.0)
            except Exception:
                loiter_time = 0.0
            if loiter_time <= 0.0:
                loiter_time = 30.0
            self.is_loitering = True
            self.loiter_timer = loiter_time
            self.loiter_center = (tx, ty, tz)
            try:
                self.loiter_radius = float(loiter_prop.get("radius", 0.0) or 0.0)
            except Exception:
                self.loiter_radius = 0.0
            if self.loiter_radius <= 1.0:
                self.loiter_radius = self.default_loiter_radius
            try:
                self.loiter_speed = float(loiter_prop.get("speed", 0.0) or (target.speed or self.speed_target))
            except Exception:
                self.loiter_speed = target.speed or self.speed_target
            direction = loiter_prop.get("direction", 1)
            self.loiter_dir = -1.0 if direction == 1 else 1.0
            self.loiter_angle = math.atan2(uav.s.y - ty, uav.s.x - tx)
            self.loiter_angle_locked = True

        target_hover = float(target.hover_time) if (self.allow_hover and target and target.hover_time) else 0.0
        if self.blocked and (self.allow_hover or self.force_hover):
            target_hover = max(target_hover, 1.0)
            if not self.is_hovering:
                self.is_hovering = True
                self.hover_timer = math.inf
        if (self.allow_hover or self.force_hover) and self.is_hovering and target_hover > 0.0:
            self.hover_timer = max(0.0, self.hover_timer - dt)
            if self.hover_timer <= 0.0:
                self.is_hovering = False
                self.force_hover = False
                self._advance_wp()
                return not self.finished
            return self._apply_hover_control(tx, ty, tz, gains, dt)

        if self.is_loitering:
            self.loiter_timer = max(0.0, self.loiter_timer - dt)
            if self.loiter_timer <= 0.0:
                self.is_loitering = False
                self._advance_wp()
                return not self.finished
            radius = max(1.0, self.loiter_radius)
            speed = max(0.0, self.loiter_speed if self.loiter_speed > 0 else (target.speed or self.speed_target))
            ang_rate = speed / radius
            self.loiter_angle += self.loiter_dir * ang_rate * dt
            tx = self.loiter_center[0] + radius * math.cos(self.loiter_angle)
            ty = self.loiter_center[1] + radius * math.sin(self.loiter_angle)
            tz = self.loiter_center[2]
            dx = tx - uav.s.x
            dy = ty - uav.s.y
            dz = tz - uav.s.z
            dist_xy = math.hypot(dx, dy)

        hover_capture_radius = max(self.pos_tol * 1.25, 40.0)
        if self.allow_hover and not self.is_loitering and dist_xy <= hover_capture_radius and abs(dz) > self.pos_tol * 0.6:
            return self._apply_hover_control(tx, ty, tz, gains, dt)

        entered_loiter = False
        reached_target = dist_xy < self.pos_tol or self._should_force_waypoint_advance(target, tx, ty, dist_xy, gains)
        if not self.is_loitering and reached_target and abs(dz) < self.pos_tol * 0.6:
            if not self.blocked and self.curr_idx in self.block_indices:
                self._enter_block(tx, ty, tz, target)
            loiter_prop = target.loiter if isinstance(target.loiter, dict) else None
            if loiter_prop is None and int(target.pass_type or 0) == 2:
                loiter_prop = {
                    "radius": self.default_loiter_radius,
                    "direction": 1,
                    "time": 30.0,
                    "speed": target.speed or self.speed_target,
                }
                target.loiter = loiter_prop
            loiter_time = 0.0
            if loiter_prop and not self.blocked:
                try:
                    loiter_time = float(loiter_prop.get("time", 0.0) or 0.0)
                except Exception:
                    loiter_time = 0.0
                if loiter_time <= 0.0 and int(target.pass_type or 0) == 2:
                    loiter_time = 30.0
            if loiter_time > 0.0 and loiter_prop:
                self.is_loitering = True
                self.loiter_timer = loiter_time
                self.loiter_center = (tx, ty, tz)
                try:
                    self.loiter_radius = float(loiter_prop.get("radius", 0.0) or 0.0)
                except Exception:
                    self.loiter_radius = 0.0
                try:
                    self.loiter_speed = float(loiter_prop.get("speed", 0.0) or (target.speed or self.speed_target))
                except Exception:
                    self.loiter_speed = target.speed or self.speed_target
                direction = loiter_prop.get("direction", 1)
                self.loiter_dir = -1.0 if direction == 1 else 1.0
                self.loiter_angle = math.atan2(uav.s.y - ty, uav.s.x - tx)
                self.loiter_angle_locked = True
                radius = max(1.0, self.loiter_radius)
                speed = max(0.0, self.loiter_speed if self.loiter_speed > 0 else (target.speed or self.speed_target))
                ang_rate = speed / radius
                self.loiter_angle += self.loiter_dir * ang_rate * dt
                tx = self.loiter_center[0] + radius * math.cos(self.loiter_angle)
                ty = self.loiter_center[1] + radius * math.sin(self.loiter_angle)
                tz = self.loiter_center[2]
                dx = tx - uav.s.x
                dy = ty - uav.s.y
                dz = tz - uav.s.z
                dist_xy = math.hypot(dx, dy)
                entered_loiter = True
            if self.allow_hover and target_hover > 0.0:
                if not self.is_hovering:
                    self.hover_timer = target_hover
                    self.is_hovering = True
                else:
                    self.hover_timer = max(0.0, self.hover_timer - dt)
                if self.hover_timer <= 0.0:
                    self.is_hovering = False
                    self._advance_wp()
                    return not self.finished
                return self._apply_hover_control(tx, ty, tz, gains, dt)
            if not self.blocked and not entered_loiter:
                self._advance_wp()
                return not self.finished

        guidance_tx, guidance_ty, guidance_tz = self._fixed_wing_guidance_target(tx, ty, tz, dist_xy, gains)
        guidance_dx = guidance_tx - uav.s.x
        guidance_dy = guidance_ty - uav.s.y
        guidance_dist_xy = math.hypot(guidance_dx, guidance_dy)

        if guidance_dist_xy > 1e-3 and gains.lookahead_m > 0.0:
            lx = uav.s.x + guidance_dx / guidance_dist_xy * gains.lookahead_m
            ly = uav.s.y + guidance_dy / guidance_dist_xy * gains.lookahead_m
            desired_yaw = self._heading_to_target(lx - uav.s.x, ly - uav.s.y)
        else:
            desired_yaw = self._heading_to_target(guidance_dx, guidance_dy)

        yaw_rate_limit = self._yaw_rate_limit()
        freeze_yaw = dist_xy < gains.freeze_yaw_dist and abs(dz) > self.pos_tol * gains.freeze_alt_ratio
        if freeze_yaw:
            uav.cmd_yaw_rate = clamp(-uav.s.r * 0.5, -yaw_rate_limit, yaw_rate_limit)
        else:
            yaw_err = ((desired_yaw - uav.s.yaw + 540.0) % 360.0) - 180.0
            yaw_err = clamp(yaw_err, -60.0, 60.0)
            self.yaw_int = clamp(self.yaw_int + yaw_err * dt, -self.yaw_int_max, self.yaw_int_max)
            uav.cmd_yaw_rate = clamp(
                yaw_err * gains.yaw + self.yaw_int * gains.yaw_i,
                -yaw_rate_limit,
                yaw_rate_limit,
            )

        desired_pitch = clamp(
            dz * gains.alt_pitch,
            -uav.p.pitch_limit_deg * 0.7,
            uav.p.pitch_limit_deg * 0.7,
        )
        self.alt_int = clamp(self.alt_int + dz * dt, -self.alt_int_max, self.alt_int_max)
        pitch_err = desired_pitch + self.alt_int * gains.alt_i - uav.s.pitch
        pitch_cmd = pitch_err * gains.pitch_rate - uav.s.q * gains.pitch_damp
        uav.cmd_pitch_rate = clamp(pitch_cmd, -uav.p.max_pitch_rate_dps, uav.p.max_pitch_rate_dps)

        self._apply_turn_roll_guidance(yaw_rate_limit)

        target_speed = target.speed if target.speed is not None else self.speed_target
        local_speed_target = target_speed * clamp(dist_xy / 300.0, 0.5, 1.0)
        if self.allow_hover:
            try:
                self.uav.p.max_speed = max(float(target_speed), float(self.uav.p.min_speed))
            except Exception:
                pass
        speed_err = local_speed_target - uav.s.u
        self.speed_int = clamp(self.speed_int + speed_err * dt, -self.speed_int_max, self.speed_int_max)
        alt_bias = dz * gains.throttle_alt
        if self.allow_hover:
            # Avoid stalling rotorcraft on large descents; keep forward motion.
            alt_bias = clamp(alt_bias, -0.3, 0.3)

        throttle_cmd = speed_err * gains.throttle + self.speed_int * gains.throttle_i + alt_bias
        if self.allow_hover and dist_xy > self.pos_tol:
            throttle_cmd = max(throttle_cmd, 0.05)
        uav.cmd_throttle = clamp(throttle_cmd, -1.0, 1.0)

        return True
