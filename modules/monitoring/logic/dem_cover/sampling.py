"""Sample expected enemy positions inside the target area.

Each enemy gets a random position inside the polygon, its ground elevation from
the DEM, and an altitude of ground + ``enemy_height_m`` (so it is not buried in
the terrain). Positions on nodata cells are rejected.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import CoverConfig
from .dem import DemGrid
from .geometry import Polygon2D


@dataclass(frozen=True)
class EnemyPoint:
    x: float          # native easting (m)
    y: float          # native northing (m)
    # Coarse points use integer cell centres; native-resolution LOS refinement
    # uses fractional cell-centre coordinates.  Keeping the model as float
    # supports both without changing the coarse analyser's behaviour.
    row: float        # analysis/native grid row
    col: float        # analysis/native grid column
    ground_m: float   # terrain elevation
    alt_m: float      # ground + enemy_height (the point we test LOS from)
    lat: float
    lon: float

    def as_dict(self) -> dict:
        return {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "lat": round(self.lat, 7),
            "lon": round(self.lon, 7),
            "groundM": round(self.ground_m, 2),
            "altM": round(self.alt_m, 2),
        }


def _target_enemy_count(polygon: Polygon2D, config: CoverConfig) -> int:
    """How many enemies to place: fixed if configured, else area-proportional."""
    cap = int(config.max_analysis_enemies)
    if int(config.enemy_sample_count) > 0:
        return max(1, min(int(config.enemy_sample_count), cap))
    area_km2 = polygon.area_m2() / 1_000_000.0
    want = int(round(area_km2 * float(config.enemy_density_per_km2)))
    want = max(int(config.enemy_min_count), want)
    return max(1, min(want, cap))


def sample_enemy_positions(
    dem: DemGrid,
    polygon: Polygon2D,
    config: CoverConfig,
    rng: np.random.Generator | None = None,
) -> list[EnemyPoint]:
    """Return up to ``config.enemy_sample_count`` enemy points inside ``polygon``."""
    if rng is None:
        rng = np.random.default_rng(config.random_seed)

    want = _target_enemy_count(polygon, config)
    enemies: list[EnemyPoint] = []
    attempts = 0
    max_attempts = 40
    while len(enemies) < want and attempts < max_attempts:
        attempts += 1
        candidates = polygon.sample_uniform(want - len(enemies), rng)
        if not candidates:
            break
        for x, y in candidates:
            if not dem.contains_native(x, y):
                continue
            ground = dem.sample_native(x, y)
            if not np.isfinite(ground):
                continue
            ri, ci = dem.nearest_index(x, y)
            if not dem.valid[ri, ci]:
                continue
            lat, lon = dem.native_to_latlon(x, y)
            enemies.append(
                EnemyPoint(
                    x=float(x),
                    y=float(y),
                    row=ri,
                    col=ci,
                    ground_m=float(ground),
                    alt_m=float(ground) + float(config.enemy_height_m),
                    lat=lat,
                    lon=lon,
                )
            )
            if len(enemies) >= want:
                break
    return enemies
