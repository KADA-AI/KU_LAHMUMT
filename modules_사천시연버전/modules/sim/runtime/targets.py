from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from enum import Enum, auto

from ..config import clamp, wrap_deg


class WeaponType(Enum):
    GUN = auto()
    MISSILE = auto()


@dataclass
class RadarParams:
    p_fa: float = 1e-6
    a: float = 1.0
    sigma: float = 1.0
    n: float = 1.5
    t_ref: float = 5.0


@dataclass
class WeaponParams:
    a_range: float = 1500.0
    b_slope: float = 2.0
    omega: float = 0.2
    weapon_type: WeaponType = WeaponType.GUN
    t_fire: float = 3.0
    reload: float = 6.0


@dataclass
class ThreatState:
    detected: bool = False
    t_exposed: float = 0.0
    ammo: int | None = None


@dataclass
class AirDefenseThreat:
    radar: RadarParams
    weapon: WeaponParams
    state: ThreatState = field(default_factory=ThreatState)

    def reset(self) -> None:
        self.state = ThreatState()

    def detection_prob(self, range_m: float, los: bool, dt: float) -> float:
        if not los:
            self.state.detected = False
            self.state.t_exposed = 0.0
            return 0.0
        self.state.t_exposed += max(0.0, dt)
        x = self.radar.n * (self.state.t_exposed / self.radar.t_ref - 1.0)
        x = min(x, 80.0)
        f_texposed = math.exp(x)
        range_km = max(0.001, range_m / 1000.0)
        pd = (f_texposed * self.radar.p_fa) / (self.radar.a * self.radar.sigma + range_km**4)
        pd = max(0.0, min(pd, 1.0))
        self.state.detected = pd >= 1.0 or random.random() < pd
        return pd

    def kill_prob(self, range_m: float, dt: float) -> float:
        if not self.state.detected:
            return 0.0
        if self.weapon is None or self.weapon.omega <= 0.0 or self.weapon.a_range <= 0.0:
            return 0.0
        if self.weapon.weapon_type is WeaponType.MISSILE:
            if self.state.t_exposed < self.weapon.t_fire:
                return 0.0
            if self.state.ammo == 0:
                return 0.0
            if self.state.ammo is not None:
                self.state.ammo -= 1
        pn = 1.0 / (1.0 + (range_m / self.weapon.a_range) ** (2 * self.weapon.b_slope))
        pk = pn * self.weapon.omega * max(0.0, dt) / max(0.1, self.weapon.t_fire)
        return max(0.0, min(pk, 1.0))


@dataclass
class GroundTarget:
    id: int
    type_id: int
    name: str
    x: float
    y: float
    z: float
    moving: bool
    vmin: float
    vmax: float
    roam_center: tuple[float, float] | None
    roam_radius: float | None
    threat: AirDefenseThreat | None
    alive: bool = True
    heading: float = 0.0
    heading_rate: float = 0.0
    v: float = 0.0
    v_tau: float = 4.0
    v_sigma: float = 1.0
    turn_tau: float = 1.2
    turn_sigma: float = 2.0
    spawn_x: float = 0.0
    spawn_y: float = 0.0
    spawn_z: float = 0.0

    def __post_init__(self) -> None:
        if self.spawn_x == 0.0 and self.spawn_y == 0.0 and self.spawn_z == 0.0:
            self.spawn_x = float(self.x)
            self.spawn_y = float(self.y)
            self.spawn_z = float(self.z)
        if self.moving:
            self.heading = random.uniform(0.0, 360.0)
            self.v = random.uniform(self.vmin, self.vmax) if self.vmax > 0 else 0.0
        else:
            self.heading = 0.0
            self.v = 0.0

    def reset(self) -> None:
        self.x = float(self.spawn_x)
        self.y = float(self.spawn_y)
        self.z = float(self.spawn_z)
        self.alive = True
        self.heading_rate = 0.0
        if self.moving:
            self.heading = random.uniform(0.0, 360.0)
            self.v = random.uniform(self.vmin, self.vmax) if self.vmax > 0 else 0.0
        else:
            self.heading = 0.0
            self.v = 0.0
        if self.threat is not None:
            self.threat.reset()

    def step(self, dt: float) -> None:
        if not self.alive or not self.moving:
            return
        self.heading_rate += (-self.heading_rate / self.turn_tau + self.turn_sigma * random.gauss(0, 1)) * dt
        self.heading_rate = clamp(self.heading_rate, -90.0, 90.0)
        self.heading = wrap_deg(self.heading + self.heading_rate * dt)
        dv = (-(self.v - 0.5 * (self.vmin + self.vmax)) / self.v_tau + self.v_sigma * random.gauss(0, 1)) * dt
        self.v = clamp(self.v + dv, self.vmin, self.vmax)
        if self.roam_center is not None and self.roam_radius is not None and self.roam_radius > 0:
            vec_x = self.roam_center[0] - self.x
            vec_y = self.roam_center[1] - self.y
            dist = math.hypot(vec_x, vec_y)
            if dist > max(1.0, self.roam_radius * 0.7):
                desired = math.degrees(math.atan2(vec_y, vec_x))
                err = ((desired - self.heading + 540.0) % 360.0) - 180.0
                self.heading_rate += 35.0 * clamp(err / 90.0, -1.0, 1.0) * dt
        rad = math.radians(self.heading)
        self.x += math.cos(rad) * self.v * dt
        self.y += math.sin(rad) * self.v * dt
        if self.roam_center is not None and self.roam_radius is not None and self.roam_radius > 0:
            dx = self.x - self.roam_center[0]
            dy = self.y - self.roam_center[1]
            dist = math.hypot(dx, dy)
            if dist > self.roam_radius and dist > 1e-6:
                scale = self.roam_radius / dist
                self.x = self.roam_center[0] + dx * scale
                self.y = self.roam_center[1] + dy * scale
