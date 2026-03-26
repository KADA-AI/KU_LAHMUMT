from __future__ import annotations

import math
from typing import List, Tuple

PointLL = Tuple[float, float]


def plan_route_linear(
    input_lla: List[PointLL],
    *,
    cruise_speed: float,
) -> List[dict]:
    """Straight-line route with per-leg ETA; no DTA, no simulation."""
    if len(input_lla) < 2:
        raise ValueError("input_lla must contain at least 2 points")

    out: List[dict] = []
    for idx, (lat, lon) in enumerate(input_lla):
        if idx == 0:
            eta_ms = 0
        else:
            lat1, lon1 = input_lla[idx - 1]
            lat2, lon2 = lat, lon
            k = 111_132.0
            cos = math.cos(math.radians((lat1 + lat2) / 2.0))
            dx = (lon2 - lon1) * k * cos
            dy = (lat2 - lat1) * k
            dist = math.hypot(dx, dy)
            eta_ms = int(round((dist / max(cruise_speed, 1e-6)) * 1000.0))
        out.append({"lat": round(lat, 6), "lon": round(lon, 6), "eta_ms": eta_ms})
    return out
