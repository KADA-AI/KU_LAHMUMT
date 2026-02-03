from __future__ import annotations

import math
from dataclasses import dataclass

from ..config import clamp, wrap_deg


@dataclass(frozen=True)
class LAHOperationalEnvelope:
    max_speed_kmh: float = 265.0
    cruise_speed_kmh: tuple[float, float] = (240.0, 260.0)
    min_speed_kmh: float = 0.0
    climb_rate_mps: float = 8.9
    descent_rate_mps: float = 7.0
    max_turn_rate_dps: float = 20.0
    pitch_roll_limit_deg: float = 30.0

    max_altitude_m: float = 2000.0
    min_altitude_agl_m: float = 50.0
    max_los_distance_km: float = 10.0
    min_los_elevation_deg: float = 5.0

    endurance_hours: float = 4.75
    threat_avoid_distance_m: float = 500.0
    max_threat_exposure_s: float = 5.0

    max_accel_mps2: float = 4.0
    fuel_capacity_kg: float = 920.0
    fuel_flow_hover_kgph: float = 150.0
    fuel_flow_cruise_kgph: float = 100.0

    def in_speed_limits(self, speed_kmh: float) -> bool:
        return self.min_speed_kmh <= speed_kmh <= self.max_speed_kmh

    def in_altitude_limits(self, altitude_m: float, terrain_elev_m: float = 0.0) -> bool:
        agl = altitude_m - terrain_elev_m
        return agl >= self.min_altitude_agl_m and altitude_m <= self.max_altitude_m

    def in_turn_rate_limits(self, turn_rate_dps: float) -> bool:
        return 0.0 <= turn_rate_dps <= self.max_turn_rate_dps

    def safe_from_threat(self, distance_m: float, exposure_time_s: float) -> bool:
        return distance_m >= self.threat_avoid_distance_m and exposure_time_s <= self.max_threat_exposure_s


DEFAULT_ENVELOPE: LAHOperationalEnvelope = LAHOperationalEnvelope()


@dataclass
class LAHParams:
    max_yaw_rate_dps: float = 45.0
    max_pitch_rate_dps: float = 20.0
    max_roll_rate_dps: float = 30.0
    max_speed: float = 75.0
    min_speed: float = 0.0
    accel: float = 6.0
    pitch_limit_deg: float = 30.0
    roll_limit_deg: float = 30.0


@dataclass
class LAHState:
    x: float = 0.0
    y: float = 0.0
    z: float = 100.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    u: float = 0.0
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0


class LAH:
    def __init__(self, params: LAHParams):
        self.p = params
        self.s = LAHState()
        self.cmd_yaw_rate = 0.0
        self.cmd_pitch_rate = 0.0
        self.cmd_roll_rate = 0.0
        self.cmd_throttle = 0.0

    def reset(self):
        self.s = LAHState()
        self.cmd_yaw_rate = self.cmd_pitch_rate = self.cmd_roll_rate = 0.0
        self.cmd_throttle = 0.0

    def step(self, dt: float):
        self.s.r = clamp(self.cmd_yaw_rate, -self.p.max_yaw_rate_dps, self.p.max_yaw_rate_dps)
        self.s.q = clamp(self.cmd_pitch_rate, -self.p.max_pitch_rate_dps, self.p.max_pitch_rate_dps)
        self.s.p = clamp(self.cmd_roll_rate, -self.p.max_roll_rate_dps, self.p.max_roll_rate_dps)

        self.s.yaw = wrap_deg(self.s.yaw + self.s.r * dt)
        self.s.pitch = clamp(self.s.pitch + self.s.q * dt, -self.p.pitch_limit_deg, self.p.pitch_limit_deg)
        self.s.roll = clamp(self.s.roll + self.s.p * dt, -self.p.roll_limit_deg, self.p.roll_limit_deg)

        a = self.p.accel * clamp(self.cmd_throttle, -1.0, 1.0)
        self.s.u = clamp(self.s.u + a * dt, self.p.min_speed, self.p.max_speed)
        if abs(self.cmd_throttle) < 1e-6 and self.s.u < 0.5:
            self.s.u = 0.0

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

    def cmd_hover(self):
        self.cmd_yaw_rate = 0.0
        self.cmd_pitch_rate = 0.0
        self.cmd_roll_rate = -self.s.roll * 1.5

    def cmd_straight(self):
        self.cmd_hover()

    def cmd_left(self):
        self.cmd_yaw_rate = -self.p.max_yaw_rate_dps * 0.6
        self.cmd_roll_rate = -self.p.max_roll_rate_dps * 0.4

    def cmd_right(self):
        self.cmd_yaw_rate = self.p.max_yaw_rate_dps * 0.6
        self.cmd_roll_rate = self.p.max_roll_rate_dps * 0.4

    def cmd_climb(self):
        self.cmd_pitch_rate = self.p.max_pitch_rate_dps * 0.4

    def cmd_descend(self):
        self.cmd_pitch_rate = -self.p.max_pitch_rate_dps * 0.4

    def neutralize_pitch(self):
        self.cmd_pitch_rate = -self.s.pitch * 1.2

    def roll_trim(self, sign: float):
        self.cmd_roll_rate += sign * self.p.max_roll_rate_dps * 0.3
