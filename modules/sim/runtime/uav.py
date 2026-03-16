import math
from dataclasses import dataclass

from ...common.turn_radius import interpolate_reference_turn_radius
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
        return float(interpolate_reference_turn_radius(self.horizontal_speed_mps()))

    def turn_rate_limit_dps(self) -> float:
        speed_xy = max(1.0, self.horizontal_speed_mps())
        limits = [float(self.p.max_yaw_rate_dps)]

        if bool(getattr(self.p, "use_reference_turn_radius", True)):
            ref_radius = self.reference_turn_radius_m()
            if ref_radius > 1.0:
                limits.append(math.degrees(speed_xy / ref_radius))

        bank_limit = min(abs(float(self.p.roll_limit_deg)), abs(float(self.p.turn_bank_limit_deg)))
        if bank_limit > 0.1:
            bank_rad = math.radians(bank_limit)
            limits.append(math.degrees(9.80665 * math.tan(bank_rad) / speed_xy))

        if float(self.p.min_turn_radius_m) > 1.0:
            limits.append(math.degrees(speed_xy / float(self.p.min_turn_radius_m)))

        return max(0.0, min(limits))

    def step(self, dt: float):
        turn_rate_limit = self.turn_rate_limit_dps()
        self.s.r = clamp(self.cmd_yaw_rate, -turn_rate_limit, turn_rate_limit)
        self.s.q = clamp(self.cmd_pitch_rate, -self.p.max_pitch_rate_dps, self.p.max_pitch_rate_dps)
        self.s.p = clamp(self.cmd_roll_rate, -self.p.max_roll_rate_dps, self.p.max_roll_rate_dps)

        self.s.yaw = wrap_deg(self.s.yaw + self.s.r * dt)
        self.s.pitch = clamp(self.s.pitch + self.s.q * dt, -self.p.pitch_limit_deg, self.p.pitch_limit_deg)
        self.s.roll = clamp(self.s.roll + self.s.p * dt, -self.p.roll_limit_deg, self.p.roll_limit_deg)

        a = self.p.accel * clamp(self.cmd_throttle, -1.0, 1.0)
        self.s.u = clamp(self.s.u + a * dt, self.p.min_speed, self.p.max_speed)

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
