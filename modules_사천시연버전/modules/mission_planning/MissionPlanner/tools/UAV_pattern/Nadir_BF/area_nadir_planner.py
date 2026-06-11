from __future__ import annotations

import math
from typing import Any, Callable

try:
    from modules.mission_planning.MissionPlanner.data_def.coord_transform import llh_to_xy, xy_to_llh
except Exception as exc:  # pragma: no cover - import guard for standalone usage
    raise ImportError("Failed to import MissionPlanner coordinate transforms") from exc


def _coerce_anchor_llh(anchor_llh: Any, fallback_llh: tuple[float, float]) -> tuple[float, float]:
    lat_fallback, lon_fallback = fallback_llh
    if isinstance(anchor_llh, dict):
        try:
            lat = float(anchor_llh.get("latitude", lat_fallback))
        except Exception:
            lat = float(lat_fallback)
        try:
            lon = float(anchor_llh.get("longitude", lon_fallback))
        except Exception:
            lon = float(lon_fallback)
        return lat, lon
    if isinstance(anchor_llh, (list, tuple)) and len(anchor_llh) >= 2:
        try:
            lat = float(anchor_llh[0])
        except Exception:
            lat = float(lat_fallback)
        try:
            lon = float(anchor_llh[1])
        except Exception:
            lon = float(lon_fallback)
        return lat, lon
    return float(lat_fallback), float(lon_fallback)


def _distance_m(a: dict, b: dict) -> float:
    lat_a = float(a.get("latitude", 0.0))
    lon_a = float(a.get("longitude", 0.0))
    lat_b = float(b.get("latitude", lat_a))
    lon_b = float(b.get("longitude", lon_a))
    dx, dy = llh_to_xy(lat_b, lon_b, lat_a, lon_a)
    return math.hypot(dx, dy)


def _sweep_spacing_m(separation_m: float, fov_deg: float, spacing_margin: float) -> float:
    base = 2.0 * max(float(separation_m), 1.0) * math.tan(max(math.radians(float(fov_deg)) / 2.0, 1e-6))
    margin = float(spacing_margin) if float(spacing_margin) > 0.0 else 1.0
    return max(base * margin, 1.0)


def _sweep_d_values(d_min: float, d_max: float, spacing_m: float) -> list[float]:
    values: list[float] = []
    current = float(d_min)
    spacing = max(float(spacing_m), 1.0)
    while current <= float(d_max) + spacing * 0.25:
        values.append(current)
        current += spacing
    if not values:
        return [float(d_min), float(d_max)]
    if len(values) == 1 and d_max > d_min:
        values.append(float(d_max))
    else:
        last = values[-1]
        if float(d_max) - last > spacing * 0.25:
            values.append(float(d_max))
    return values


class NadirAreaPlanner:
    """
    Build a nadir(overflight) path for area missions.
    - Uses strip intersections over polygon
    - Keeps alternating traversal
    - Chooses first strip direction using current(anchor) direction
    """

    def __init__(
        self,
        *,
        separation_m: float,
        fov_deg: float,
        min_segment_m: float = 3.0,
        spacing_margin: float = 1.0,
    ) -> None:
        self.separation_m = float(separation_m)
        self.fov_deg = float(fov_deg)
        self.min_segment_m = float(min_segment_m)
        self.spacing_margin = float(spacing_margin)

    def _build_strips(
        self,
        *,
        polygon_llh: list[tuple[float, float]],
        bearing_deg: float,
    ) -> list[list[dict]]:
        if not polygon_llh:
            return []
        lat0, lon0 = polygon_llh[0]
        poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in polygon_llh]

        th = math.radians(float(bearing_deg))
        tx, ty = math.sin(th), math.cos(th)
        nx, ny = math.sin(th), -math.cos(th)

        proj_norm = [nx * x + ny * y for x, y in poly_xy]
        d_min, d_max = min(proj_norm), max(proj_norm)
        spacing_m = _sweep_spacing_m(self.separation_m, self.fov_deg, self.spacing_margin)
        d_values = _sweep_d_values(d_min, d_max, spacing_m)

        edges = [(poly_xy[i], poly_xy[(i + 1) % len(poly_xy)]) for i in range(len(poly_xy))]
        lines: list[list[dict]] = []

        for d in d_values:
            hits: list[tuple[float, float]] = []
            for a, b in edges:
                f1 = nx * a[0] + ny * a[1] - d
                f2 = nx * b[0] + ny * b[1] - d
                if f1 * f2 <= 0 and f1 != f2:
                    t = f1 / (f1 - f2)
                    x = a[0] + t * (b[0] - a[0])
                    y = a[1] + t * (b[1] - a[1])
                    hits.append((x, y))
            if len(hits) < 2:
                continue

            # Pick extreme pair along strip direction to make segment stable.
            along = [tx * x + ty * y for x, y in hits]
            i_min = min(range(len(hits)), key=lambda i: along[i])
            i_max = max(range(len(hits)), key=lambda i: along[i])
            p1, p2 = hits[i_min], hits[i_max]

            lat1, lon1 = xy_to_llh(p1[0], p1[1], lat0, lon0)
            lat2, lon2 = xy_to_llh(p2[0], p2[1], lat0, lon0)
            line = [
                {"latitude": float(lat1), "longitude": float(lon1), "altitude": 0},
                {"latitude": float(lat2), "longitude": float(lon2), "altitude": 0},
            ]
            if _distance_m(line[0], line[1]) >= self.min_segment_m:
                lines.append(line)
        return lines

    def plan(
        self,
        *,
        polygon_llh: list[tuple[float, float]],
        anchor_llh: Any,
        bearing_deg: float,
        altitude_fn: Callable[[float, float], int],
    ) -> list[dict]:
        lines = self._build_strips(polygon_llh=polygon_llh, bearing_deg=bearing_deg)
        if not lines:
            return []

        anchor_lat, anchor_lon = _coerce_anchor_llh(anchor_llh, polygon_llh[0] if polygon_llh else (0.0, 0.0))
        anchor = {"latitude": float(anchor_lat), "longitude": float(anchor_lon), "altitude": 0}

        first_s, first_e = lines[0]
        first_forward = _distance_m(anchor, first_s) <= _distance_m(anchor, first_e)

        ordered: list[dict] = []
        for idx, line in enumerate(lines):
            s, e = line
            use_forward = first_forward if idx % 2 == 0 else not first_forward
            start, end = (s, e) if use_forward else (e, s)
            for point in (start, end):
                lat = float(point.get("latitude", 0.0))
                lon = float(point.get("longitude", 0.0))
                ordered.append(
                    {
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": int(altitude_fn(lat, lon)),
                    }
                )
        return ordered


def build_nadir_overflight_coords(
    *,
    polygon_llh: list[tuple[float, float]],
    anchor_llh: Any,
    bearing_deg: float,
    separation_m: float,
    fov_deg: float,
    min_segment_m: float,
    spacing_margin: float,
    altitude_fn: Callable[[float, float], int],
) -> list[dict]:
    planner = NadirAreaPlanner(
        separation_m=separation_m,
        fov_deg=fov_deg,
        min_segment_m=min_segment_m,
        spacing_margin=spacing_margin,
    )
    return planner.plan(
        polygon_llh=polygon_llh,
        anchor_llh=anchor_llh,
        bearing_deg=bearing_deg,
        altitude_fn=altitude_fn,
    )

