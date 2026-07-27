"""Shared operational terrain-LOS policy for planning, monitoring, and SIM.

The vectorised cover planner and the real-time SIM use different execution
strategies, but they must not silently use different physical assumptions.
This module owns the small set of policy constants and the scalar ray kernel
used by SIM.  The planner imports the same constants into ``CoverConfig``.

Altitude inputs are MSL metres and horizontal coordinates are projected/local
metres.  Terrain is sampled from the canonical regional DEM selected by each
subsystem.
"""

from __future__ import annotations

import math
from typing import Callable


TERRAIN_LOS_POLICY_VERSION = "regional-dem-los-v1"

# Match modules.monitoring.logic.dem_cover.visibility: the standard 4/3-earth
# radius approximation includes normal atmospheric refraction.
EFFECTIVE_EARTH_RADIUS_M = 6_371_000.0 * 4.0 / 3.0
EARTH_CURVATURE_ENABLED = True

# These values are intentionally shared with CoverConfig.  A positive
# clearance is a conservative concealment margin: terrain must rise above the
# mathematical sightline by this amount before the planner claims masking.
LOS_CLEARANCE_M = 1.0
LOS_SAMPLES_PER_CELL = 1.2
ENEMY_OBSERVER_HEIGHT_M = 5.0
MAX_ENEMY_LOS_TARGETS = 60

# LAH1 relay planning and SIM communication visualization/assessment use the
# same physical range.  Reported/manual datalink state remains a separate gate.
LAH_UAV_COMMUNICATION_RANGE_M = 10_000.0


def terrain_los_visible(
    *,
    source_x: float,
    source_y: float,
    source_altitude_m: float,
    target_x: float,
    target_y: float,
    target_altitude_m: float,
    ground_height: Callable[[float, float], float],
    terrain_resolution_m: float,
    target_height_m: float = 0.0,
    clearance_m: float = LOS_CLEARANCE_M,
    samples_per_cell: float = LOS_SAMPLES_PER_CELL,
    earth_curvature: bool = EARTH_CURVATURE_ENABLED,
) -> bool:
    """Return whether a straight 3-D ray clears the operational terrain.

    This mirrors the cover planner's LOS convention: endpoints do not block,
    terrain blocks only when it exceeds ``ray + clearance``, and the optional
    4/3-earth correction lowers intervening terrain relative to the source
    tangent.  Invalid/non-finite terrain fails closed.
    """

    values = (
        source_x,
        source_y,
        source_altitude_m,
        target_x,
        target_y,
        target_altitude_m,
        terrain_resolution_m,
        target_height_m,
        clearance_m,
        samples_per_cell,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False

    sx = float(source_x)
    sy = float(source_y)
    sz = float(source_altitude_m)
    tx = float(target_x)
    ty = float(target_y)
    try:
        source_ground_m = float(ground_height(sx, sy))
        target_ground_m = float(ground_height(tx, ty))
    except Exception:
        return False
    if not math.isfinite(source_ground_m) or not math.isfinite(target_ground_m):
        return False

    target_ray_altitude_m = max(
        float(target_altitude_m),
        target_ground_m + max(0.0, float(target_height_m)),
    )
    # Endpoints need the same vertical clearance as the intervening ray.  In
    # particular, two coordinates sharing one XY cell are not automatically
    # visible when either aircraft is still inside the safety/terrain band.
    endpoint_clearance_m = max(0.0, float(clearance_m))
    if (
        sz <= source_ground_m + endpoint_clearance_m
        or target_ray_altitude_m <= target_ground_m + endpoint_clearance_m
    ):
        return False

    dx = tx - sx
    dy = ty - sy
    horizontal_m = math.hypot(dx, dy)
    if horizontal_m <= 1.0e-9:
        return True

    cell_m = max(1.0, float(terrain_resolution_m))
    samples = max(1.0, float(samples_per_cell))
    # One or more samples per native cell.  This is intentionally independent
    # of rendering zoom and therefore stays stable across deployment PCs.
    sample_count = max(2, int(math.ceil(horizontal_m * samples / cell_m)) + 1)
    clearance = max(0.0, float(clearance_m))
    for index in range(1, sample_count - 1):
        fraction = float(index) / float(sample_count - 1)
        px = sx + (dx * fraction)
        py = sy + (dy * fraction)
        ray_altitude_m = sz + (
            (target_ray_altitude_m - sz) * fraction
        )
        try:
            terrain_m = float(ground_height(px, py))
        except Exception:
            return False
        if not math.isfinite(terrain_m):
            return False
        if bool(earth_curvature):
            along_m = horizontal_m * fraction
            terrain_m -= (along_m * along_m) / (
                2.0 * float(EFFECTIVE_EARTH_RADIUS_M)
            )
        if terrain_m > ray_altitude_m + clearance:
            return False
    return True


__all__ = [
    "EARTH_CURVATURE_ENABLED",
    "EFFECTIVE_EARTH_RADIUS_M",
    "ENEMY_OBSERVER_HEIGHT_M",
    "LAH_UAV_COMMUNICATION_RANGE_M",
    "LOS_CLEARANCE_M",
    "LOS_SAMPLES_PER_CELL",
    "MAX_ENEMY_LOS_TARGETS",
    "TERRAIN_LOS_POLICY_VERSION",
    "terrain_los_visible",
]
