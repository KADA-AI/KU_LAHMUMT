"""Fast, deterministic DEM selector for an AREA-contained LAH terminal point.

The selector intentionally has no dependency on the monitoring/test cover-analysis
packages.  It keeps horizontal placement inside the supplied AREA (including
holes and an inward boundary reserve), evaluates a small deterministic candidate
set, and performs at most one batched terrain-provider call.

The returned coordinate always preserves the altitude value of
``fallback_coordinate``.  Terrain is used only to choose latitude/longitude;
the established LAH altitude/ETA pass remains authoritative afterwards.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable, Iterable, Sequence


_METRES_PER_LAT_DEG = 111_132.92
_EFFECTIVE_EARTH_RADIUS_M = 6_371_000.0 * 4.0 / 3.0
_THREAT_DEFAULT_HEIGHT_M = 5.0
_RAY_SAMPLE_SPACING_M = 250.0
_ABSOLUTE_MAX_CANDIDATES = 49
_ABSOLUTE_MAX_THREATS = 5
_ABSOLUTE_MAX_RAY_SAMPLES = 64


@dataclass(frozen=True)
class _Coordinate:
    latitude: float
    longitude: float
    altitude: float | None = None


@dataclass(frozen=True)
class _RaySpec:
    candidate_index: int
    observer_kind: str
    observer_index: int
    offset: int
    count: int
    distance_m: float


def _finite_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _first_finite(mapping: dict, keys: Iterable[str]) -> float | None:
    for key in keys:
        if key not in mapping:
            continue
        number = _finite_float(mapping.get(key))
        if number is not None:
            return number
    return None


def _coerce_coordinate(value: Any) -> _Coordinate | None:
    """Accept established coordinate dictionaries and simple lat/lon rows."""

    if isinstance(value, dict):
        nested = value.get("coordinate")
        base = nested if isinstance(nested, dict) else value
        latitude = _first_finite(base, ("latitude", "lat", "Latitude", "LAT"))
        longitude = _first_finite(base, ("longitude", "lon", "lng", "Longitude", "LON"))
        if latitude is None or longitude is None:
            return None
        altitude = _first_finite(base, ("altitude", "alt", "height", "Altitude"))
        if altitude is None and base is not value:
            altitude = _first_finite(value, ("altitude", "alt", "height", "Altitude"))
        return _Coordinate(latitude, longitude, altitude)

    if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 2:
        latitude = _finite_float(value[0])
        longitude = _finite_float(value[1])
        altitude = _finite_float(value[2]) if len(value) >= 3 else None
        if latitude is not None and longitude is not None:
            return _Coordinate(latitude, longitude, altitude)
    return None


def _fallback_altitude_value(value: Any) -> int | float:
    raw: Any = None
    if isinstance(value, dict):
        nested = value.get("coordinate")
        base = nested if isinstance(nested, dict) else value
        for key in ("altitude", "alt", "height", "Altitude"):
            if key in base:
                raw = base.get(key)
                break
        if raw is None and base is not value:
            for key in ("altitude", "alt", "height", "Altitude"):
                if key in value:
                    raw = value.get(key)
                    break
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) >= 3:
        raw = value[2]

    number = _finite_float(raw)
    if number is None:
        return 0
    # Keep the common integer representation exactly when possible.
    if isinstance(raw, int) and not isinstance(raw, bool):
        return raw
    return float(number)


def _output_coordinate(
    latitude: float,
    longitude: float,
    fallback_altitude: int | float,
) -> dict:
    return {
        "latitude": round(float(latitude), 7),
        "longitude": round(float(longitude), 7),
        "altitude": fallback_altitude,
    }


def _fallback_output(value: Any) -> dict:
    coord = _coerce_coordinate(value)
    altitude = _fallback_altitude_value(value)
    if coord is None:
        return _output_coordinate(0.0, 0.0, altitude)
    return _output_coordinate(coord.latitude, coord.longitude, altitude)


def _truthy(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return float(value) != 0.0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on", "y"}
    return False


def _diagnostics(
    *,
    reason: str,
    selected: dict,
    fallback_used: bool,
    candidate_count: int,
    dem_sample_count: int,
    cover_fraction: float | None = None,
    uav_los_clear: bool | None = None,
    uav_los_margin_m: float | None = None,
    uav_distance_m: float | None = None,
    uav_distance_feasible: bool | None = None,
    **extra: Any,
) -> dict:
    """Build the stable diagnostics contract on every return path."""

    result = {
        "reason": str(reason),
        "selected": dict(selected),
        "fallbackUsed": bool(fallback_used),
        "candidateCount": int(max(0, candidate_count)),
        "demSampleCount": int(max(0, dem_sample_count)),
        "coverFraction": None if cover_fraction is None else round(float(cover_fraction), 6),
        "uavLosClear": None if uav_los_clear is None else bool(uav_los_clear),
        "uavLosMarginM": None if uav_los_margin_m is None else round(float(uav_los_margin_m), 3),
        "uavDistanceM": None if uav_distance_m is None else round(float(uav_distance_m), 3),
        "uavDistanceFeasible": (
            None if uav_distance_feasible is None else bool(uav_distance_feasible)
        ),
    }
    result.update(extra)
    return result


def _bounded_int(value: Any, default: int, *, minimum: int, maximum: int) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        parsed = int(default)
    return max(int(minimum), min(int(maximum), parsed))


def _local_projectors(origin_latitude: float, origin_longitude: float):
    metres_per_lon = max(
        1.0,
        _METRES_PER_LAT_DEG * math.cos(math.radians(float(origin_latitude))),
    )

    def to_xy(coord: _Coordinate) -> tuple[float, float]:
        return (
            (coord.longitude - origin_longitude) * metres_per_lon,
            (coord.latitude - origin_latitude) * _METRES_PER_LAT_DEG,
        )

    def to_latlon(x: float, y: float) -> tuple[float, float]:
        return (
            origin_latitude + float(y) / _METRES_PER_LAT_DEG,
            origin_longitude + float(x) / metres_per_lon,
        )

    return to_xy, to_latlon


def _area_rows(area_list: Any) -> list[tuple[list[_Coordinate], bool]]:
    rows: list[tuple[list[_Coordinate], bool]] = []
    if not isinstance(area_list, list):
        return rows
    for row in area_list:
        if not isinstance(row, dict):
            continue
        raw_points = row.get("coordinateList")
        if not isinstance(raw_points, list):
            continue
        points = [coord for coord in (_coerce_coordinate(item) for item in raw_points) if coord]
        if len(points) >= 3:
            rows.append((points, _truthy(row.get("isHole"))))
    return rows


def _inset_for_standoff(
    allowed: Any,
    boundary_margin_ratio: float,
    boundary_margin_max_m: float,
) -> tuple[Any, float]:
    """Pull the selectable region in from the AREA edge. Returns (safe, margin)."""

    min_x, min_y, max_x, max_y = allowed.bounds
    short_span_m = max(0.0, min(float(max_x - min_x), float(max_y - min_y)))
    ratio = _finite_float(boundary_margin_ratio)
    margin_max = _finite_float(boundary_margin_max_m)
    ratio = max(0.0, ratio if ratio is not None else 0.10)
    margin_max = max(0.0, margin_max if margin_max is not None else 100.0)
    margin_m = min(margin_max, short_span_m * ratio)
    if margin_m <= 0.0:
        return allowed, 0.0
    inset = allowed.buffer(-margin_m, join_style=1).buffer(0)
    # Small/narrow AREA: containment is more important than forcing an inset
    # that erases the whole geometry.
    if not inset.is_empty and float(inset.area) > 1e-6:
        return inset, float(margin_m)
    return allowed, 0.0


def _build_safe_geometry(
    area_list: Any,
    boundary_margin_ratio: float,
    boundary_margin_max_m: float,
    *,
    search_center: "_Coordinate | None" = None,
    search_radius_m: float = 0.0,
):
    """Return (allowed, safe, to_xy, to_latlon, margin, error).

    ``search_radius_m`` caps how far from ``search_center`` the cover point may
    wander, so the aircraft can use whatever ridge is nearby instead of only
    the exact anchor.  When the AREA supplies outer geometry the disk is
    intersected with it: the AREA is the region the aircraft was tasked to
    occupy, and a hold outside it is not the mission.  Holes are always
    excluded - those are keep-out geometry rather than a search boundary.
    """

    radius_m = _finite_float(search_radius_m) or 0.0
    radius_m = max(0.0, float(radius_m))
    disk_mode = radius_m > 0.0 and search_center is not None

    rows = _area_rows(area_list)
    all_points = [point for points, _is_hole in rows for point in points]
    if not all_points and not disk_mode:
        return None, None, None, None, 0.0, "area_geometry_missing"

    try:
        from shapely.geometry import Point, Polygon
        from shapely.ops import unary_union
    except Exception:
        return None, None, None, None, 0.0, "shapely_unavailable"

    if disk_mode:
        origin_lat = float(search_center.latitude)
        origin_lon = float(search_center.longitude)
    else:
        origin_lat = sum(point.latitude for point in all_points) / len(all_points)
        origin_lon = sum(point.longitude for point in all_points) / len(all_points)
    to_xy, to_latlon = _local_projectors(origin_lat, origin_lon)

    if disk_mode:
        try:
            center_xy = to_xy(search_center)
            allowed = Point(float(center_xy[0]), float(center_xy[1])).buffer(radius_m)
            outer_parts: list[Any] = []
            hole_parts: list[Any] = []
            for points, is_hole in rows:
                polygon = Polygon([to_xy(point) for point in points]).buffer(0)
                if polygon.is_empty or float(polygon.area) <= 1e-6:
                    continue
                (hole_parts if is_hole else outer_parts).append(polygon)
            clipped_to_area = False
            if outer_parts:
                # Stay inside the tasked AREA.  Without this the scorer walks
                # the hold to the far edge of the disk - always directly away
                # from the threat - and the aircraft never occupies the
                # 공격대기지역/전투진지 it was given.
                clipped = allowed.intersection(unary_union(outer_parts)).buffer(0)
                if not clipped.is_empty and float(clipped.area) > 1e-6:
                    allowed = clipped
                    clipped_to_area = True
            if hole_parts:
                allowed = allowed.difference(unary_union(hole_parts)).buffer(0)
            if allowed.is_empty or float(allowed.area) <= 1e-6:
                return None, None, to_xy, to_latlon, 0.0, "search_disk_empty"
            if not clipped_to_area:
                # A bare search disk has no mission boundary to stand off from.
                return allowed, allowed, to_xy, to_latlon, 0.0, None
            # Once the disk is bounded by the AREA, the same boundary standoff
            # applies: a hold parked on the AREA edge is one step from being
            # outside it, and reads as the aircraft heading for the next region.
            safe, margin_m = _inset_for_standoff(
                allowed, boundary_margin_ratio, boundary_margin_max_m
            )
            return allowed, safe, to_xy, to_latlon, float(margin_m), None
        except Exception as exc:
            return None, None, to_xy, to_latlon, 0.0, f"search_disk_failed:{type(exc).__name__}"

    outer_parts: list[Any] = []
    hole_parts: list[Any] = []
    try:
        for points, is_hole in rows:
            polygon = Polygon([to_xy(point) for point in points]).buffer(0)
            if polygon.is_empty or float(polygon.area) <= 1e-6:
                continue
            (hole_parts if is_hole else outer_parts).append(polygon)
        if not outer_parts:
            return None, None, to_xy, to_latlon, 0.0, "area_outer_geometry_missing"
        allowed = unary_union(outer_parts).buffer(0)
        if hole_parts:
            allowed = allowed.difference(unary_union(hole_parts)).buffer(0)
        if allowed.is_empty or float(allowed.area) <= 1e-6:
            return None, None, to_xy, to_latlon, 0.0, "area_geometry_empty"

        safe, margin_m = _inset_for_standoff(
            allowed, boundary_margin_ratio, boundary_margin_max_m
        )
        return allowed, safe, to_xy, to_latlon, float(margin_m), None
    except Exception:
        return None, None, to_xy, to_latlon, 0.0, "area_geometry_invalid"


def _polygon_parts(geometry: Any) -> list[Any]:
    if geometry is None or geometry.is_empty:
        return []
    if getattr(geometry, "geom_type", "") == "Polygon":
        return [geometry]
    parts: list[Any] = []
    for item in getattr(geometry, "geoms", ()):
        parts.extend(_polygon_parts(item))
    return parts


def _dedupe_xy(points: Iterable[tuple[float, float]]) -> list[tuple[float, float]]:
    result: list[tuple[float, float]] = []
    seen: set[tuple[int, int]] = set()
    for x, y in points:
        key = (int(round(float(x) * 1000.0)), int(round(float(y) * 1000.0)))
        if key in seen:
            continue
        seen.add(key)
        result.append((float(x), float(y)))
    return result


def _candidate_points(
    safe_geometry: Any,
    fallback_xy: tuple[float, float] | None,
    budget: int,
) -> list[tuple[float, float]]:
    """Build a deterministic, spatially distributed set of at most 49 points."""

    try:
        from shapely.geometry import Point
    except Exception:
        return []

    budget = _bounded_int(
        budget,
        _ABSOLUTE_MAX_CANDIDATES,
        minimum=1,
        maximum=_ABSOLUTE_MAX_CANDIDATES,
    )
    seeds: list[tuple[float, float]] = []

    def add_geometry_point(point: Any) -> None:
        if point is None or point.is_empty:
            return
        if safe_geometry.covers(point):
            seeds.append((float(point.x), float(point.y)))

    add_geometry_point(safe_geometry.representative_point())
    centroid = safe_geometry.centroid
    add_geometry_point(centroid)

    parts = sorted(
        _polygon_parts(safe_geometry),
        key=lambda item: (-float(item.area), tuple(float(v) for v in item.bounds)),
    )
    for part in parts:
        add_geometry_point(part.representative_point())
        add_geometry_point(part.centroid)

    if fallback_xy is not None:
        fallback_point = Point(float(fallback_xy[0]), float(fallback_xy[1]))
        add_geometry_point(fallback_point)

    selected = _dedupe_xy(seeds)[:budget]
    if len(selected) >= budget:
        return selected

    min_x, min_y, max_x, max_y = (float(v) for v in safe_geometry.bounds)
    # Probe more cells than the final budget, then farthest-point sample them.
    # This keeps concave/multipart AREA coverage good without increasing DEM work.
    grid_side = max(7, min(25, int(math.ceil(math.sqrt(budget * 8.0)))))
    grid_points: list[tuple[float, float]] = []
    width = max_x - min_x
    height = max_y - min_y
    for row in range(grid_side):
        y = min_y + (row + 0.5) * height / grid_side if height > 0.0 else min_y
        for col in range(grid_side):
            x = min_x + (col + 0.5) * width / grid_side if width > 0.0 else min_x
            point = Point(x, y)
            if safe_geometry.covers(point):
                grid_points.append((float(x), float(y)))

    pool = [point for point in _dedupe_xy(grid_points) if point not in selected]
    pool.sort(key=lambda point: (point[1], point[0]))
    while pool and len(selected) < budget:
        if not selected:
            chosen_index = 0
        else:
            chosen_index = max(
                range(len(pool)),
                key=lambda idx: min(
                    (pool[idx][0] - current[0]) ** 2 + (pool[idx][1] - current[1]) ** 2
                    for current in selected
                ),
            )
        selected.append(pool.pop(chosen_index))
    return selected[:budget]


def _limit_coordinates(values: Any, limit: int) -> list[_Coordinate]:
    if not isinstance(values, (list, tuple)):
        return []
    coords = [coord for coord in (_coerce_coordinate(value) for value in values) if coord]
    limit = _bounded_int(limit, _ABSOLUTE_MAX_THREATS, minimum=0, maximum=_ABSOLUTE_MAX_THREATS)
    if limit <= 0 or not coords:
        return []
    if len(coords) <= limit:
        return coords
    if limit == 1:
        return [coords[len(coords) // 2]]
    indices = [int(round(index * (len(coords) - 1) / (limit - 1))) for index in range(limit)]
    return [coords[index] for index in indices]


def _ray_sample_count(distance_m: float, cap: int) -> int:
    if distance_m <= 1.0 or cap <= 0:
        return 0
    desired = max(1, int(math.ceil(distance_m / _RAY_SAMPLE_SPACING_M)) - 1)
    return min(int(cap), desired)


def _haversine_m(first: _Coordinate, second: _Coordinate) -> float:
    lat1 = math.radians(first.latitude)
    lat2 = math.radians(second.latitude)
    dlat = lat2 - lat1
    dlon = math.radians(second.longitude - first.longitude)
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * 6_371_000.0 * math.asin(min(1.0, math.sqrt(max(0.0, a))))


def _terrain_provider_or_none(
    terrain_elev_many_fn: Callable[[Iterable[Any]], Iterable[Any]] | None,
):
    if callable(terrain_elev_many_fn):
        return terrain_elev_many_fn
    try:
        from .mission_helpers import terrain_elev_many

        return terrain_elev_many
    except Exception:
        return None


def _internal_fallback_xy(safe_geometry: Any, fallback_xy: tuple[float, float] | None):
    try:
        from shapely.geometry import Point

        if fallback_xy is not None:
            fallback_point = Point(float(fallback_xy[0]), float(fallback_xy[1]))
            if safe_geometry.covers(fallback_point):
                return float(fallback_point.x), float(fallback_point.y)
        point = safe_geometry.representative_point()
        if not point.is_empty:
            return float(point.x), float(point.y)
    except Exception:
        return fallback_xy
    return fallback_xy


def _observer_altitude(
    coordinate: _Coordinate,
    ground_m: float,
    default_height_m: float,
) -> float:
    minimum = float(ground_m) + max(1.0, float(default_height_m))
    if coordinate.altitude is None:
        return minimum
    return max(float(coordinate.altitude), minimum)


def _ray_margin_m(
    spec: _RaySpec,
    terrain_values: list[float],
    candidate_ground_m: float,
    candidate_altitude_m: float,
    observer_ground_m: float,
    observer_altitude_m: float,
) -> float:
    # Endpoint clearances keep very short rays meaningful without querying more
    # DEM samples.  Interior curvature is a symmetric 4/3-earth chord bulge.
    margin = min(
        candidate_altitude_m - candidate_ground_m,
        observer_altitude_m - observer_ground_m,
    )
    if spec.count <= 0:
        return float(margin)
    for sample_index in range(spec.count):
        fraction = float(sample_index + 1) / float(spec.count + 1)
        terrain_m = float(terrain_values[spec.offset + sample_index])
        line_m = candidate_altitude_m + fraction * (observer_altitude_m - candidate_altitude_m)
        along_m = fraction * spec.distance_m
        curvature_bulge_m = (
            along_m * (spec.distance_m - along_m) / (2.0 * _EFFECTIVE_EARTH_RADIUS_M)
        )
        margin = min(margin, line_m - (terrain_m + curvature_bulge_m))
    return float(margin)


def _sigmoid_margin(margin_m: float) -> float:
    scaled = max(-12.0, min(12.0, float(margin_m) / 25.0))
    return 1.0 / (1.0 + math.exp(-scaled))


def select_lah_terminal_cover_point(
    area_list,
    fallback_coordinate,
    *,
    threat_coordinates=None,
    uav_coordinate=None,
    terrain_elev_many_fn=None,
    hold_agl_m=50.0,
    boundary_margin_ratio=0.10,
    boundary_margin_max_m=100.0,
    max_candidates=49,
    max_threats=5,
    max_ray_samples=64,
    max_uav_distance_m=20000.0,
    search_radius_m=0.0,
) -> tuple[dict, dict]:
    """Select a low, covered, UAV-visible terminal point.

    By default the point stays inside the supplied AREA and horizontal
    containment is preferred over the 20 km UAV condition.  With
    ``search_radius_m`` the point is instead sought within that radius of the
    fallback coordinate, ignoring the AREA boundary but still avoiding its
    holes; this is what lets a manned aircraft take cover on whatever terrain
    is nearby rather than only where it happens to be tasked to photograph.
    """

    fallback_output = _fallback_output(fallback_coordinate)
    fallback_coord = _coerce_coordinate(fallback_coordinate)
    fallback_altitude = _fallback_altitude_value(fallback_coordinate)
    uav = _coerce_coordinate(uav_coordinate)
    max_distance = _finite_float(max_uav_distance_m)
    max_distance = max(1.0, max_distance if max_distance is not None else 20_000.0)

    allowed, safe, to_xy, to_latlon, margin_m, geometry_error = _build_safe_geometry(
        area_list,
        boundary_margin_ratio,
        boundary_margin_max_m,
        search_center=fallback_coord,
        search_radius_m=search_radius_m,
    )
    if safe is None or to_xy is None or to_latlon is None:
        distance_m = _haversine_m(fallback_coord, uav) if fallback_coord and uav else None
        return fallback_output, _diagnostics(
            reason=geometry_error or "area_geometry_unavailable",
            selected=fallback_output,
            fallback_used=True,
            candidate_count=0,
            dem_sample_count=0,
            uav_distance_m=distance_m,
            uav_distance_feasible=(distance_m <= max_distance) if distance_m is not None else None,
            boundaryMarginM=0.0,
            threatCount=0,
            demCallCount=0,
        )

    fallback_xy = to_xy(fallback_coord) if fallback_coord is not None else None
    safe_fallback_xy = _internal_fallback_xy(safe, fallback_xy)
    if safe_fallback_xy is None:
        return fallback_output, _diagnostics(
            reason="safe_geometry_point_unavailable",
            selected=fallback_output,
            fallback_used=True,
            candidate_count=0,
            dem_sample_count=0,
            boundaryMarginM=round(float(margin_m), 3),
            threatCount=0,
            demCallCount=0,
        )

    safe_lat, safe_lon = to_latlon(*safe_fallback_xy)
    safe_output = _output_coordinate(safe_lat, safe_lon, fallback_altitude)
    candidates_xy = _candidate_points(safe, fallback_xy, max_candidates)
    if not candidates_xy:
        distance_m = (
            _haversine_m(_Coordinate(safe_lat, safe_lon), uav) if uav is not None else None
        )
        return safe_output, _diagnostics(
            reason="candidate_generation_empty",
            selected=safe_output,
            fallback_used=True,
            candidate_count=0,
            dem_sample_count=0,
            uav_distance_m=distance_m,
            uav_distance_feasible=(distance_m <= max_distance) if distance_m is not None else None,
            boundaryMarginM=round(float(margin_m), 3),
            threatCount=0,
            demCallCount=0,
        )

    candidate_coords = [
        _Coordinate(*to_latlon(x, y))
        for x, y in candidates_xy
    ]
    threats = _limit_coordinates(threat_coordinates, max_threats)
    ray_cap = _bounded_int(
        max_ray_samples,
        _ABSOLUTE_MAX_RAY_SAMPLES,
        minimum=0,
        maximum=_ABSOLUTE_MAX_RAY_SAMPLES,
    )
    provider = _terrain_provider_or_none(terrain_elev_many_fn)
    if provider is None:
        distance_m = (
            _haversine_m(_Coordinate(safe_lat, safe_lon), uav) if uav is not None else None
        )
        return safe_output, _diagnostics(
            reason="dem_provider_unavailable",
            selected=safe_output,
            fallback_used=True,
            candidate_count=len(candidate_coords),
            dem_sample_count=0,
            uav_distance_m=distance_m,
            uav_distance_feasible=(distance_m <= max_distance) if distance_m is not None else None,
            boundaryMarginM=round(float(margin_m), 3),
            threatCount=len(threats),
            demCallCount=0,
        )

    # Base samples are candidates, threats, and UAV.  Every intermediate ray
    # sample is appended to the same list so the provider is invoked only once.
    terrain_queries: list[tuple[float, float]] = [
        (coord.latitude, coord.longitude) for coord in candidate_coords
    ]
    threat_ground_offset = len(terrain_queries)
    terrain_queries.extend((coord.latitude, coord.longitude) for coord in threats)
    uav_ground_offset: int | None = None
    if uav is not None:
        uav_ground_offset = len(terrain_queries)
        terrain_queries.append((uav.latitude, uav.longitude))

    threat_xy = [to_xy(coord) for coord in threats]
    uav_xy = to_xy(uav) if uav is not None else None
    ray_specs: list[_RaySpec] = []

    def append_ray(
        candidate_index: int,
        observer_kind: str,
        observer_index: int,
        observer: _Coordinate,
        observer_xy: tuple[float, float],
    ) -> None:
        candidate = candidate_coords[candidate_index]
        cx, cy = candidates_xy[candidate_index]
        distance_m = math.hypot(observer_xy[0] - cx, observer_xy[1] - cy)
        count = _ray_sample_count(distance_m, ray_cap)
        offset = len(terrain_queries)
        for sample_index in range(count):
            fraction = float(sample_index + 1) / float(count + 1)
            terrain_queries.append(
                (
                    candidate.latitude + fraction * (observer.latitude - candidate.latitude),
                    candidate.longitude + fraction * (observer.longitude - candidate.longitude),
                )
            )
        ray_specs.append(
            _RaySpec(
                candidate_index=candidate_index,
                observer_kind=observer_kind,
                observer_index=observer_index,
                offset=offset,
                count=count,
                distance_m=float(distance_m),
            )
        )

    for candidate_index in range(len(candidate_coords)):
        for threat_index, (threat, observer_xy) in enumerate(zip(threats, threat_xy)):
            append_ray(candidate_index, "threat", threat_index, threat, observer_xy)
        if uav is not None and uav_xy is not None:
            append_ray(candidate_index, "uav", 0, uav, uav_xy)

    dem_sample_count = len(terrain_queries)
    try:
        raw_values = list(provider(terrain_queries))
        if len(raw_values) != dem_sample_count:
            raise ValueError("terrain provider returned an unexpected sample count")
        terrain_values = [float(value) for value in raw_values]
        if not all(math.isfinite(value) for value in terrain_values):
            raise ValueError("terrain provider returned a non-finite elevation")
    except Exception as exc:
        distance_m = (
            _haversine_m(_Coordinate(safe_lat, safe_lon), uav) if uav is not None else None
        )
        reason = "dem_non_finite" if "non-finite" in str(exc) else "dem_error"
        return safe_output, _diagnostics(
            reason=reason,
            selected=safe_output,
            fallback_used=True,
            candidate_count=len(candidate_coords),
            dem_sample_count=dem_sample_count,
            uav_distance_m=distance_m,
            uav_distance_feasible=(distance_m <= max_distance) if distance_m is not None else None,
            boundaryMarginM=round(float(margin_m), 3),
            threatCount=len(threats),
            demCallCount=1,
        )

    candidate_count = len(candidate_coords)
    candidate_ground = terrain_values[:candidate_count]
    threat_ground = terrain_values[
        threat_ground_offset:threat_ground_offset + len(threats)
    ]
    uav_ground = terrain_values[uav_ground_offset] if uav_ground_offset is not None else None
    hold_agl = _finite_float(hold_agl_m)
    hold_agl = max(1.0, hold_agl if hold_agl is not None else 50.0)
    candidate_altitudes = [float(ground) + hold_agl for ground in candidate_ground]
    threat_altitudes = [
        _observer_altitude(coord, ground, _THREAT_DEFAULT_HEIGHT_M)
        for coord, ground in zip(threats, threat_ground)
    ]
    uav_altitude = (
        _observer_altitude(uav, float(uav_ground), hold_agl)
        if uav is not None and uav_ground is not None
        else None
    )

    threat_margins: list[list[float | None]] = [
        [None for _ in threats] for _ in candidate_coords
    ]
    uav_margins: list[float | None] = [None for _ in candidate_coords]
    uav_distances: list[float | None] = [None for _ in candidate_coords]
    for spec in ray_specs:
        candidate_index = spec.candidate_index
        if spec.observer_kind == "threat":
            observer_index = spec.observer_index
            margin = _ray_margin_m(
                spec,
                terrain_values,
                candidate_ground[candidate_index],
                candidate_altitudes[candidate_index],
                threat_ground[observer_index],
                threat_altitudes[observer_index],
            )
            threat_margins[candidate_index][observer_index] = margin
        elif uav is not None and uav_ground is not None and uav_altitude is not None:
            margin = _ray_margin_m(
                spec,
                terrain_values,
                candidate_ground[candidate_index],
                candidate_altitudes[candidate_index],
                float(uav_ground),
                float(uav_altitude),
            )
            uav_margins[candidate_index] = margin
            uav_distances[candidate_index] = float(spec.distance_m)

    cover_fractions: list[float | None] = []
    for margins in threat_margins:
        if not margins:
            cover_fractions.append(None)
            continue
        blocked = sum(1 for value in margins if value is not None and float(value) < 0.0)
        cover_fractions.append(float(blocked) / float(len(margins)))

    eligible = list(range(candidate_count))
    distance_preference_available = False
    los_preference_available = False
    if uav is not None:
        distance_feasible = [
            index
            for index in eligible
            if uav_distances[index] is not None and float(uav_distances[index]) <= max_distance
        ]
        if distance_feasible:
            eligible = distance_feasible
            distance_preference_available = True
        clear = [
            index
            for index in eligible
            if uav_margins[index] is not None and float(uav_margins[index]) >= 0.0
        ]
        if clear:
            eligible = clear
            los_preference_available = True

    ground_low = min(candidate_ground)
    ground_high = max(candidate_ground)
    ground_span = ground_high - ground_low
    low_scores = [
        0.5 if ground_span <= 1e-6 else (ground_high - ground) / ground_span
        for ground in candidate_ground
    ]
    if fallback_xy is None:
        fallback_distances = [0.0 for _ in candidates_xy]
    else:
        fallback_distances = [
            math.hypot(x - fallback_xy[0], y - fallback_xy[1])
            for x, y in candidates_xy
        ]
    area_scale_m = max(500.0, math.sqrt(max(float(safe.area), 1.0) / math.pi))
    proximity_scores = [math.exp(-distance / area_scale_m) for distance in fallback_distances]

    scores: list[float] = []
    for index in range(candidate_count):
        weighted_terms: list[tuple[float, float]] = [
            (0.20, low_scores[index]),
            (0.10, proximity_scores[index]),
        ]
        if cover_fractions[index] is not None:
            weighted_terms.append((0.42, float(cover_fractions[index])))
        if uav_margins[index] is not None:
            weighted_terms.append((0.28, _sigmoid_margin(float(uav_margins[index]))))
        total_weight = sum(weight for weight, _value in weighted_terms)
        scores.append(
            sum(weight * value for weight, value in weighted_terms) / max(total_weight, 1e-9)
        )

    selected_index = max(
        eligible,
        key=lambda index: (
            scores[index],
            -fallback_distances[index],
            -index,
        ),
    )
    selected_coord = candidate_coords[selected_index]
    selected_output = _output_coordinate(
        selected_coord.latitude,
        selected_coord.longitude,
        fallback_altitude,
    )
    selected_margin = uav_margins[selected_index]
    selected_distance = uav_distances[selected_index]
    selected_distance_feasible = (
        selected_distance is not None and float(selected_distance) <= max_distance
        if uav is not None
        else None
    )
    selected_los_clear = (
        selected_margin is not None and float(selected_margin) >= 0.0
        if uav is not None
        else None
    )
    if uav is not None and not distance_preference_available:
        reason = "selected_inside_area_uav_distance_unavailable"
    elif uav is not None and not los_preference_available:
        reason = "selected_inside_area_best_uav_margin"
    else:
        reason = "selected_inside_area"

    return selected_output, _diagnostics(
        reason=reason,
        selected=selected_output,
        fallback_used=False,
        candidate_count=candidate_count,
        dem_sample_count=dem_sample_count,
        cover_fraction=cover_fractions[selected_index],
        uav_los_clear=selected_los_clear,
        uav_los_margin_m=selected_margin,
        uav_distance_m=selected_distance,
        uav_distance_feasible=selected_distance_feasible,
        boundaryMarginM=round(float(margin_m), 3),
        threatCount=len(threats),
        demCallCount=1,
        selectedScore=round(float(scores[selected_index]), 6),
        selectedGroundM=round(float(candidate_ground[selected_index]), 3),
        selectedLowTerrainScore=round(float(low_scores[selected_index]), 6),
        fallbackDistanceM=round(float(fallback_distances[selected_index]), 3),
        uavDistancePreferenceAvailable=bool(distance_preference_available),
        uavLosPreferenceAvailable=bool(los_preference_available),
    )


__all__ = ["select_lah_terminal_cover_point"]
