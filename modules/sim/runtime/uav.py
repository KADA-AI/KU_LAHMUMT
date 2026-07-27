import math
from dataclasses import dataclass
from pathlib import Path

from modules.common.turn_profile import uav_dynamics_params
from modules.common.turn_dynamics import (
    TurnPerformance,
    coordinated_turn_rate_nav_dps,
    interpolate_reference_turn_radius,
    max_achievable_turn_rate_dps,
)
from ..config import clamp, wrap_deg


@dataclass
class UAVParams:
    max_yaw_rate_dps: float = 30.0
    max_pitch_rate_dps: float = 20.0
    max_roll_rate_dps: float = 40.0
    max_speed: float = 150.0
    min_speed: float = 30.0
    accel: float = 15.0
    pitch_limit_deg: float = 25.0
    roll_limit_deg: float = 60.0
    min_turn_radius_m: float = 120.0
    turn_bank_limit_deg: float = 55.0
    turn_roll_gain: float = 2.2
    use_reference_turn_radius: bool = True
    reference_turn_radius_scale: float = 1.0
    banked_turn_enabled: bool = True
    bank_yaw_rate_blend: float = 1.0
    coordinated_turn_rate_scale: float = 1.0
    # 수직속도 envelope[m/s]. 임무계획의 uav_climb_rate_mps/uav_descent_rate_mps
    # 기본값과 일치시켜, 계획이 가정한 상승률을 sim 기체도 그대로 따르게 한다.
    # 0 이하로 두면 비활성(종전처럼 pitch 한계까지 상승 허용).
    max_climb_rate_mps: float = 5.0
    max_descent_rate_mps: float = 7.0


@dataclass
class UAVState:
    x: float = 0.0
    y: float = 0.0
    z: float = 300.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    u: float = 60.0
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0


class UAV:
    def __init__(self, params: UAVParams):
        self.p = params
        self.s = UAVState()
        self.cmd_yaw_rate = 0.0
        self.cmd_pitch_rate = 0.0
        self.cmd_roll_rate = 0.0
        self.cmd_throttle = 0.0

    def reset(self):
        self.s = UAVState()
        self.cmd_yaw_rate = self.cmd_pitch_rate = self.cmd_roll_rate = 0.0
        self.cmd_throttle = 0.0

    def horizontal_speed_mps(self) -> float:
        pitch = math.radians(float(self.s.pitch))
        return max(0.0, abs(float(self.s.u) * math.cos(pitch)))

    def reference_turn_radius_m(self) -> float:
        scale = max(0.05, float(getattr(self.p, "reference_turn_radius_scale", 1.0)))
        return float(interpolate_reference_turn_radius(self.horizontal_speed_mps()) * scale)

    def turn_rate_limit_dps(self) -> float:
        speed_xy = max(1.0, self.horizontal_speed_mps())
        performance = TurnPerformance(
            reference_radius_scale=max(
                0.05,
                float(getattr(self.p, "reference_turn_radius_scale", 1.0)),
            ),
            use_reference_radius=bool(getattr(self.p, "use_reference_turn_radius", True)),
            min_turn_radius_m=float(getattr(self.p, "min_turn_radius_m", 0.0) or 0.0),
            max_turn_rate_dps=float(getattr(self.p, "max_yaw_rate_dps", 0.0) or 0.0),
            bank_limit_deg=min(
                abs(float(getattr(self.p, "roll_limit_deg", 60.0))),
                abs(float(getattr(self.p, "turn_bank_limit_deg", 55.0))),
            ),
            roll_rate_limit_dps=abs(float(getattr(self.p, "max_roll_rate_dps", 40.0))),
            roll_time_constant_s=1.0 / max(0.1, float(getattr(self.p, "turn_roll_gain", 2.2))),
            turn_rate_scale=max(
                0.05,
                float(getattr(self.p, "coordinated_turn_rate_scale", 1.0)),
            ),
        )
        return max(0.0, float(max_achievable_turn_rate_dps(speed_xy, performance)))

    def step(self, dt: float):
        turn_rate_limit = self.turn_rate_limit_dps()
        cmd_yaw_rate = clamp(self.cmd_yaw_rate, -turn_rate_limit, turn_rate_limit)
        self.s.q = clamp(self.cmd_pitch_rate, -self.p.max_pitch_rate_dps, self.p.max_pitch_rate_dps)
        self.s.p = clamp(self.cmd_roll_rate, -self.p.max_roll_rate_dps, self.p.max_roll_rate_dps)

        self.s.pitch = clamp(self.s.pitch + self.s.q * dt, -self.p.pitch_limit_deg, self.p.pitch_limit_deg)
        self.s.roll = clamp(self.s.roll + self.s.p * dt, -self.p.roll_limit_deg, self.p.roll_limit_deg)

        if bool(getattr(self.p, "banked_turn_enabled", False)):
            speed_xy = max(1.0, self.horizontal_speed_mps())
            bank_yaw_rate = coordinated_turn_rate_nav_dps(
                self.s.roll,
                speed_xy,
                turn_rate_scale=max(
                    0.05,
                    float(getattr(self.p, "coordinated_turn_rate_scale", 1.0)),
                ),
            )
            if bank_yaw_rate is None:
                bank_yaw_rate = 0.0
            bank_yaw_rate = clamp(bank_yaw_rate, -turn_rate_limit, turn_rate_limit)
            blend = clamp(float(getattr(self.p, "bank_yaw_rate_blend", 1.0)), 0.0, 1.0)
            self.s.r = clamp(
                (cmd_yaw_rate * (1.0 - blend)) + (bank_yaw_rate * blend),
                -turn_rate_limit,
                turn_rate_limit,
            )
        else:
            self.s.r = cmd_yaw_rate

        self.s.yaw = wrap_deg(self.s.yaw + self.s.r * dt)

        a = self.p.accel * clamp(self.cmd_throttle, -1.0, 1.0)
        self.s.u = clamp(self.s.u + a * dt, self.p.min_speed, self.p.max_speed)

        # 수직속도 envelope: vz = u·sin(pitch)가 상승/강하율 한계를 넘지 않도록
        # 피치를 제한한다. 종전에는 pitch 25° 한계만 있어 순항속도에서 계획
        # 상승률(기본 5 m/s)의 3배 이상으로 상승할 수 있었고, 그 결과 달성
        # 불가능한 고도 프로파일이 sim에서는 문제없이 통과해 보였다.
        speed_u = max(0.1, abs(float(self.s.u)))
        max_climb_mps = float(getattr(self.p, "max_climb_rate_mps", 0.0) or 0.0)
        if max_climb_mps > 0.0:
            climb_pitch_cap_deg = math.degrees(math.asin(min(1.0, max_climb_mps / speed_u)))
            if self.s.pitch > climb_pitch_cap_deg:
                self.s.pitch = climb_pitch_cap_deg
        max_descent_mps = float(getattr(self.p, "max_descent_rate_mps", 0.0) or 0.0)
        if max_descent_mps > 0.0:
            descent_pitch_cap_deg = -math.degrees(math.asin(min(1.0, max_descent_mps / speed_u)))
            if self.s.pitch < descent_pitch_cap_deg:
                self.s.pitch = descent_pitch_cap_deg

        yaw = math.radians(self.s.yaw)
        pitch = math.radians(self.s.pitch)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        vx = self.s.u * cp * cy
        vy = -self.s.u * cp * sy
        vz = self.s.u * sp

        self.s.x += vx * dt
        self.s.y += vy * dt
        self.s.z = max(0.0, self.s.z + vz * dt)

    def cmd_straight(self):
        self.cmd_yaw_rate = 0.0
        self.cmd_pitch_rate = 0.0
        self.cmd_roll_rate = -self.s.roll * 2.0

    def cmd_left(self):
        self.cmd_yaw_rate = -self.p.max_yaw_rate_dps * 0.8
        self.cmd_roll_rate = -self.p.max_roll_rate_dps * 0.4

    def cmd_right(self):
        self.cmd_yaw_rate = self.p.max_yaw_rate_dps * 0.8
        self.cmd_roll_rate = self.p.max_roll_rate_dps * 0.4

    def cmd_climb(self):
        self.cmd_pitch_rate = self.p.max_pitch_rate_dps * 0.6

    def cmd_descend(self):
        self.cmd_pitch_rate = -self.p.max_pitch_rate_dps * 0.6

    def neutralize_pitch(self):
        self.cmd_pitch_rate = -self.s.pitch * 1.5

    def roll_trim(self, sign: float):
        self.cmd_roll_rate += sign * self.p.max_roll_rate_dps * 0.3


def _merge_uav_params(merged: dict[str, object], raw_params: object) -> None:
    if not isinstance(raw_params, dict):
        return
    for key in UAVParams.__dataclass_fields__:
        if key not in raw_params:
            continue
        value = raw_params.get(key)
        try:
            if isinstance(merged.get(key), bool):
                if isinstance(value, str):
                    merged[key] = value.strip().casefold() not in {"0", "false", "no", "off"}
                else:
                    merged[key] = bool(value)
            else:
                merged[key] = float(value)
        except Exception:
            continue


def load_uav_params_profile(
    path: str | Path | None = None,
    fallback: UAVParams | None = None,
    *,
    aircraft_id: int | None = None,
) -> UAVParams:
    params = fallback or UAVParams()
    if path is None:
        return params
    profile_path = Path(path)
    if not profile_path.exists():
        return params
    merged: dict[str, object] = dict(params.__dict__)
    _merge_uav_params(
        merged,
        uav_dynamics_params(aircraft_id, path=profile_path),
    )
    return UAVParams(**merged)
