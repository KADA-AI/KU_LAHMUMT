# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Any, Iterable

from pyproj import Transformer
from shapely.geometry import GeometryCollection, LineString, MultiPoint, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import substring, unary_union

_MIN_AREA_M2 = 1e-3
_MAX_VERTEX_COUNT = 4096
_SIMPLIFY_TOLERANCE_M = 0.5
_SWEEP_BRIDGE_GAP_FACTOR = 3.0
_INTERPOLATION_MAX_AREA_RATIO = 4.0
_INTERPOLATION_STEP_SPAN_FACTOR = 0.65
_INTERPOLATION_MAX_STEP_TURN_DEG = 15.0
_INTERPOLATION_MAX_LOCAL_FILL_RATIO = 1.25
_PLANNED_PATH_MAX_LENGTH_M = 2500.0
_NO_COVERAGE_MISSION_TYPES = {5, 7}


def _coerce_float(value: object) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _normalize_geometry(geometry: BaseGeometry | None) -> BaseGeometry:
    if geometry is None:
        return GeometryCollection()
    if geometry.is_empty:
        return GeometryCollection()
    if not geometry.is_valid:
        try:
            geometry = geometry.buffer(0)
        except Exception:
            return GeometryCollection()
    if geometry.is_empty:
        return GeometryCollection()
    return geometry


def _vertex_count(geometry: BaseGeometry | None) -> int:
    if geometry is None or geometry.is_empty:
        return 0
    geom_type = geometry.geom_type
    if geom_type == "Polygon":
        count = len(getattr(geometry.exterior, "coords", []) or [])
        for ring in geometry.interiors:
            count += len(getattr(ring, "coords", []) or [])
        return int(count)
    if geom_type in ("MultiPolygon", "GeometryCollection"):
        total = 0
        for child in geometry.geoms:
            total += _vertex_count(child)
        return int(total)
    if hasattr(geometry, "coords"):
        return len(list(geometry.coords))
    return 0


@lru_cache(maxsize=64)
def _utm_transformer(zone: int, south: bool) -> Transformer:
    utm_code = f"+proj=utm +zone={zone} {'+south' if south else ''} +ellps=WGS84 +units=m +no_defs"
    return Transformer.from_crs("EPSG:4326", utm_code, always_xy=True)


def _extract_lat_lon(coord: object) -> tuple[float, float] | None:
    if not isinstance(coord, dict):
        return None
    lat = _coerce_float(coord.get("latitude"))
    lon = _coerce_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    return float(lat), float(lon)


def _coordinate_dicts(items: Iterable[object] | None) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    if items is None:
        return result
    for item in items:
        if isinstance(item, dict):
            result.append(item)
    return result


def _resolve_transformer(
    coordinate_lists: Iterable[Iterable[object] | None],
) -> Transformer | None:
    coords: list[tuple[float, float]] = []
    for items in coordinate_lists:
        for item in items or []:
            coord = _extract_lat_lon(item)
            if coord is None:
                continue
            coords.append(coord)
    if not coords:
        return None
    avg_lat = sum(lat for lat, _lon in coords) / len(coords)
    avg_lon = sum(lon for _lat, lon in coords) / len(coords)
    zone = int((avg_lon + 180.0) // 6.0) + 1
    zone = max(1, min(zone, 60))
    south = avg_lat < 0.0
    return _utm_transformer(zone, south)


def _project_points(
    items: Iterable[object] | None,
    transformer: Transformer,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    seen: set[tuple[int, int]] = set()
    for item in items or []:
        coord = _extract_lat_lon(item)
        if coord is None:
            continue
        lat, lon = coord
        x, y = transformer.transform(lon, lat)
        key = (int(round(x * 1000.0)), int(round(y * 1000.0)))
        if key in seen:
            continue
        seen.add(key)
        points.append((float(x), float(y)))
    return points


def _build_area_geometry(area_list: list[dict[str, Any]], transformer: Transformer) -> BaseGeometry:
    outers: list[BaseGeometry] = []
    holes: list[BaseGeometry] = []
    for area in area_list:
        coords = _project_points(area.get("coordinateList"), transformer)
        if len(coords) < 3:
            continue
        geometry = _normalize_geometry(Polygon(coords))
        if geometry.is_empty or geometry.area <= _MIN_AREA_M2:
            continue
        if bool(area.get("isHole")):
            holes.append(geometry)
        else:
            outers.append(geometry)
    if not outers:
        return GeometryCollection()
    geometry = _normalize_geometry(unary_union(outers))
    if holes and not geometry.is_empty:
        geometry = _normalize_geometry(geometry.difference(unary_union(holes)))
    return geometry


def _build_line_geometry(line_list: list[dict[str, Any]], transformer: Transformer) -> BaseGeometry:
    pieces: list[BaseGeometry] = []
    for line in line_list:
        coords = _project_points(line.get("coordinateList"), transformer)
        if not coords:
            continue
        width = _coerce_float(line.get("width")) or 0.0
        if len(coords) == 1:
            if width <= 0.0:
                continue
            geometry = Point(coords[0]).buffer(width / 2.0)
        else:
            geometry = LineString(coords)
            if width > 0.0:
                geometry = geometry.buffer(width / 2.0, cap_style=2, join_style=2)
        geometry = _normalize_geometry(geometry)
        if geometry.is_empty or geometry.area <= _MIN_AREA_M2:
            continue
        pieces.append(geometry)
    if not pieces:
        return GeometryCollection()
    return _normalize_geometry(unary_union(pieces))


@dataclass(frozen=True)
class MissionCoverageDefinition:
    planned_area_m2: float
    assignment_geometry: BaseGeometry
    transformer: Transformer


def build_mission_coverage_definition(mission: dict[str, Any]) -> MissionCoverageDefinition | None:
    mission_type = _coerce_int(mission.get("individual_mission_type"))
    if mission_type in _NO_COVERAGE_MISSION_TYPES:
        return None
    sweep_point_count = _coerce_int(mission.get("sweep_point_count")) or 0
    if not bool(mission.get("is_done")) and sweep_point_count <= 0:
        return None

    line_list = _coordinate_dicts(mission.get("line_list"))
    area_list = _coordinate_dicts(mission.get("area_list"))
    coordinate_lists = []
    coordinate_lists.extend(line.get("coordinateList") for line in line_list)
    coordinate_lists.extend(area.get("coordinateList") for area in area_list)
    transformer = _resolve_transformer(coordinate_lists)
    if transformer is None:
        return None

    pieces: list[BaseGeometry] = []
    area_geometry = _build_area_geometry(area_list, transformer)
    if not area_geometry.is_empty:
        pieces.append(area_geometry)
    line_geometry = _build_line_geometry(line_list, transformer)
    if not line_geometry.is_empty:
        pieces.append(line_geometry)
    if not pieces:
        return None

    assignment = _normalize_geometry(unary_union(pieces))
    planned_area_m2 = float(max(0.0, assignment.area))
    if assignment.is_empty or planned_area_m2 <= _MIN_AREA_M2:
        return None
    return MissionCoverageDefinition(
        planned_area_m2=planned_area_m2,
        assignment_geometry=assignment,
        transformer=transformer,
    )


def build_footprint_geometry(
    footprint_corners: Iterable[object] | None,
    transformer: Transformer,
) -> BaseGeometry:
    points = _project_points(footprint_corners, transformer)
    if len(points) < 3:
        return GeometryCollection()
    geometry = _normalize_geometry(MultiPoint(points).convex_hull)
    if geometry.geom_type != "Polygon":
        return GeometryCollection()
    if geometry.area <= _MIN_AREA_M2:
        return GeometryCollection()
    return geometry


def merge_coverage_geometry(
    existing_geometry: BaseGeometry | None,
    additional_geometry: BaseGeometry | None,
) -> BaseGeometry:
    existing = _normalize_geometry(existing_geometry)
    additional = _normalize_geometry(additional_geometry)
    if additional.is_empty:
        return existing
    if existing.is_empty:
        merged = additional
    else:
        merged = _normalize_geometry(unary_union([existing, additional]))
    if merged.is_empty:
        return GeometryCollection()
    if _vertex_count(merged) > _MAX_VERTEX_COUNT:
        simplified = _normalize_geometry(
            merged.simplify(_SIMPLIFY_TOLERANCE_M, preserve_topology=True)
        )
        if not simplified.is_empty and simplified.area > 0.0:
            merged = simplified
    return merged


def build_swept_footprint_geometry(
    previous_footprint: BaseGeometry | None,
    current_footprint: BaseGeometry | None,
) -> BaseGeometry:
    previous = _normalize_geometry(previous_footprint)
    current = _normalize_geometry(current_footprint)
    if previous.is_empty:
        return current
    if current.is_empty:
        return previous

    merged = _normalize_geometry(unary_union([previous, current]))
    if merged.is_empty:
        return GeometryCollection()

    try:
        prev_bounds = previous.bounds
        curr_bounds = current.bounds
        prev_diag = math.hypot(prev_bounds[2] - prev_bounds[0], prev_bounds[3] - prev_bounds[1])
        curr_diag = math.hypot(curr_bounds[2] - curr_bounds[0], curr_bounds[3] - curr_bounds[1])
        max_diag = max(prev_diag, curr_diag, 1e-6)
        centroid_gap = previous.centroid.distance(current.centroid)
    except Exception:
        return merged

    if centroid_gap > (max_diag * _SWEEP_BRIDGE_GAP_FACTOR):
        return merged

    swept = _normalize_geometry(merged.convex_hull)
    if swept.is_empty or swept.area <= merged.area:
        return merged
    return swept


def _polygon_vertices(geometry: BaseGeometry) -> list[tuple[float, float]]:
    normalized = _normalize_geometry(geometry)
    if normalized.geom_type != "Polygon":
        return []
    vertices = [(float(x), float(y)) for x, y in normalized.exterior.coords]
    if len(vertices) >= 2 and vertices[0] == vertices[-1]:
        vertices.pop()
    return vertices


def _aligned_polygon_vertices(
    previous: list[tuple[float, float]],
    current: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    if len(previous) != len(current) or not previous:
        return []
    candidates: list[list[tuple[float, float]]] = []
    for vertices in (current, list(reversed(current))):
        for offset in range(len(vertices)):
            candidates.append(vertices[offset:] + vertices[:offset])
    return min(
        candidates,
        key=lambda candidate: sum(
            ((previous[index][0] - point[0]) ** 2)
            + ((previous[index][1] - point[1]) ** 2)
            for index, point in enumerate(candidate)
        ),
    )


def _footprint_scales(
    geometry: BaseGeometry,
) -> tuple[float, float, float | None]:
    """Return minor/major spans and the stable long-axis angle (modulo 180 degrees)."""
    try:
        rectangle = _normalize_geometry(geometry.minimum_rotated_rectangle)
        vertices = _polygon_vertices(rectangle)
        if len(vertices) != 4:
            return 0.0, 0.0, None
        edges: list[tuple[float, float]] = []
        for index, point in enumerate(vertices):
            next_point = vertices[(index + 1) % len(vertices)]
            edges.append(
                (
                    math.hypot(next_point[0] - point[0], next_point[1] - point[1]),
                    math.atan2(next_point[1] - point[1], next_point[0] - point[0]),
                )
            )
        lengths = sorted(length for length, _angle in edges if length > 1e-6)
        if len(lengths) < 2:
            return 0.0, 0.0, None
        minor = float(lengths[0])
        major = float(lengths[-1])
        if major / max(minor, 1e-6) < 1.15:
            return minor, major, None
        _length, angle = max(edges, key=lambda item: item[0])
        return minor, major, float(angle % math.pi)
    except Exception:
        return 0.0, 0.0, None


def _axis_turn_radians(previous_angle: float | None, current_angle: float | None) -> float:
    if previous_angle is None or current_angle is None:
        return 0.0
    difference = abs(float(current_angle) - float(previous_angle)) % math.pi
    return float(min(difference, math.pi - difference))


def _edge_sweep_between(
    previous_vertices: list[tuple[float, float]],
    current_vertices: list[tuple[float, float]],
) -> BaseGeometry:
    """Sweep every footprint edge, including its two end corners, between samples."""
    if len(previous_vertices) < 3 or len(previous_vertices) != len(current_vertices):
        return GeometryCollection()
    pieces: list[BaseGeometry] = []
    for index in range(len(previous_vertices)):
        next_index = (index + 1) % len(previous_vertices)
        piece = _normalize_geometry(
            Polygon(
                [
                    previous_vertices[index],
                    previous_vertices[next_index],
                    current_vertices[next_index],
                    current_vertices[index],
                ]
            )
        )
        if not piece.is_empty and piece.area > _MIN_AREA_M2:
            pieces.append(piece)
    if not pieces:
        return GeometryCollection()
    return _normalize_geometry(unary_union(pieces))


def _clip_interpolated_geometry(
    geometry: BaseGeometry,
    assignment_geometry: BaseGeometry | None,
) -> BaseGeometry:
    normalized = _normalize_geometry(geometry)
    if normalized.is_empty or assignment_geometry is None:
        return normalized
    assignment = _normalize_geometry(assignment_geometry)
    if assignment.is_empty:
        return GeometryCollection()
    try:
        return _normalize_geometry(assignment.intersection(normalized))
    except Exception:
        return GeometryCollection()


def build_projected_sweep_path(
    coordinates: Iterable[object] | None,
    transformer: Transformer,
) -> LineString | None:
    """Project an ordered lineSearch chain without deleting later revisits."""
    points: list[tuple[float, float]] = []
    for item in coordinates or []:
        coordinate = _extract_lat_lon(item)
        if coordinate is None:
            continue
        latitude, longitude = coordinate
        x_value, y_value = transformer.transform(longitude, latitude)
        point = (float(x_value), float(y_value))
        if points and math.dist(points[-1], point) <= 1e-3:
            continue
        points.append(point)
    if len(points) < 2:
        return None
    try:
        path = LineString(points)
    except Exception:
        return None
    if path.is_empty or path.length <= 1e-3:
        return None
    return path


def project_coordinate_to_sweep_path(
    path: LineString | None,
    coordinate: object,
    transformer: Transformer,
    *,
    previous_chainage_m: float | None = None,
    search_window_m: float = 2000.0,
) -> tuple[float | None, float | None]:
    """Project a sensor centre onto the locally-forward part of a sweep path.

    Area rows commonly lie close to one another.  A global ``LineString.project``
    can therefore jump to an older parallel row.  Once a pass has started, the
    search is limited to a small window around its last chainage, preserving the
    ordered lineSearch contract even through U-turns.
    """
    if path is None or path.is_empty:
        return None, None
    lat_lon = _extract_lat_lon(coordinate)
    if lat_lon is None:
        return None, None
    latitude, longitude = lat_lon
    x_value, y_value = transformer.transform(longitude, latitude)
    point = Point(float(x_value), float(y_value))
    try:
        if previous_chainage_m is None:
            chainage = float(path.project(point))
            return chainage, float(path.interpolate(chainage).distance(point))

        previous = max(0.0, min(float(path.length), float(previous_chainage_m)))
        window = max(100.0, min(_PLANNED_PATH_MAX_LENGTH_M, float(search_window_m)))
        backtrack = min(50.0, window * 0.05)
        start = max(0.0, previous - backtrack)
        end = min(float(path.length), previous + window)
        if end <= start + 1e-6:
            return None, None
        local_path = substring(path, start, end)
        if local_path.is_empty or local_path.geom_type != "LineString":
            return None, None
        local_chainage = float(local_path.project(point))
        chainage = start + local_chainage
        offset = float(local_path.interpolate(local_chainage).distance(point))
        if chainage < previous and previous - chainage <= backtrack + 1e-6:
            chainage = previous
            offset = float(path.interpolate(chainage).distance(point))
        return chainage, offset
    except Exception:
        return None, None


def _union_edge_swept_samples(
    sample_vertices: list[list[tuple[float, float]]],
    samples: list[BaseGeometry],
    assignment_geometry: BaseGeometry | None,
) -> BaseGeometry:
    pieces: list[BaseGeometry] = list(samples)
    for index in range(1, len(samples)):
        edge_sweep = _edge_sweep_between(
            sample_vertices[index - 1],
            sample_vertices[index],
        )
        if edge_sweep.is_empty:
            continue
        local_fill_limit = max(
            samples[index - 1].area,
            samples[index].area,
        ) * _INTERPOLATION_MAX_LOCAL_FILL_RATIO
        if edge_sweep.area > local_fill_limit:
            sample_pair = _normalize_geometry(
                unary_union([samples[index - 1], samples[index]])
            )
            local_fill = _normalize_geometry(edge_sweep.difference(sample_pair))
            if not local_fill.is_empty and local_fill.area > local_fill_limit:
                continue
        pieces.append(edge_sweep)
    return _clip_interpolated_geometry(
        _normalize_geometry(unary_union(pieces)),
        assignment_geometry,
    )


def build_path_interpolated_footprint_geometry(
    previous_footprint: BaseGeometry | None,
    current_footprint: BaseGeometry | None,
    sweep_path: LineString | None,
    previous_chainage_m: float | None,
    current_chainage_m: float | None,
    interpolation_steps: int,
    *,
    assignment_geometry: BaseGeometry | None = None,
) -> BaseGeometry:
    """Sweep footprints along the ordered planned lineSearch sub-path.

    This is used when high search speed moves the sensor several footprint
    widths between 0401 frames.  Following the row/connector polyline closes
    endpoint and hairpin gaps without taking a diagonal convex-hull shortcut
    across the unobserved inside of a U-turn.
    """
    previous = _normalize_geometry(previous_footprint)
    current = _normalize_geometry(current_footprint)
    if previous.is_empty or current.is_empty or sweep_path is None:
        return _clip_interpolated_geometry(current, assignment_geometry)
    if previous_chainage_m is None or current_chainage_m is None:
        return _clip_interpolated_geometry(current, assignment_geometry)
    start = max(0.0, min(float(sweep_path.length), float(previous_chainage_m)))
    end = max(0.0, min(float(sweep_path.length), float(current_chainage_m)))
    path_length = end - start
    if path_length <= 1e-3 or path_length > _PLANNED_PATH_MAX_LENGTH_M:
        return _clip_interpolated_geometry(current, assignment_geometry)

    previous_vertices = _polygon_vertices(previous)
    current_vertices = _aligned_polygon_vertices(
        previous_vertices,
        _polygon_vertices(current),
    )
    if len(previous_vertices) < 3 or len(previous_vertices) != len(current_vertices):
        return _clip_interpolated_geometry(current, assignment_geometry)
    previous_area = float(max(previous.area, _MIN_AREA_M2))
    current_area = float(max(current.area, _MIN_AREA_M2))
    if max(previous_area, current_area) / min(previous_area, current_area) > _INTERPOLATION_MAX_AREA_RATIO:
        return _clip_interpolated_geometry(current, assignment_geometry)

    prev_minor, _prev_major, prev_angle = _footprint_scales(previous)
    curr_minor, _curr_major, curr_angle = _footprint_scales(current)
    valid_minor_spans = [span for span in (prev_minor, curr_minor) if span > 1e-6]
    if not valid_minor_spans:
        return _clip_interpolated_geometry(current, assignment_geometry)
    minor_span = min(valid_minor_spans)
    turn_radians = _axis_turn_radians(prev_angle, curr_angle)
    step_span_m = max(1e-6, minor_span * _INTERPOLATION_STEP_SPAN_FACTOR)
    steps = max(
        2,
        int(interpolation_steps),
        int(math.ceil(path_length / step_span_m)),
        int(math.ceil(math.degrees(turn_radians) / _INTERPOLATION_MAX_STEP_TURN_DEG)),
    )
    if steps > 300:
        return _clip_interpolated_geometry(current, assignment_geometry)

    try:
        local_path = substring(sweep_path, start, end)
    except Exception:
        return _clip_interpolated_geometry(current, assignment_geometry)
    if local_path.is_empty or local_path.geom_type != "LineString":
        return _clip_interpolated_geometry(current, assignment_geometry)

    # Regular spacing bounds every interpolation gap.  Explicit polyline
    # vertices ensure a row-end corner itself is never rounded off diagonally.
    sample_distances = {path_length * (step / steps) for step in range(steps + 1)}
    cumulative = 0.0
    local_coords = list(local_path.coords)
    for left, right in zip(local_coords, local_coords[1:]):
        cumulative += math.dist(left, right)
        sample_distances.add(min(path_length, cumulative))
    ordered_distances = sorted(sample_distances)
    if len(ordered_distances) > 300:
        return _clip_interpolated_geometry(current, assignment_geometry)

    previous_centroid = previous.centroid
    current_centroid = current.centroid
    start_anchor = local_path.interpolate(0.0)
    end_anchor = local_path.interpolate(path_length)
    start_offset = (
        float(previous_centroid.x - start_anchor.x),
        float(previous_centroid.y - start_anchor.y),
    )
    end_offset = (
        float(current_centroid.x - end_anchor.x),
        float(current_centroid.y - end_anchor.y),
    )
    previous_centered = [
        (x_value - previous_centroid.x, y_value - previous_centroid.y)
        for x_value, y_value in previous_vertices
    ]
    current_centered = [
        (x_value - current_centroid.x, y_value - current_centroid.y)
        for x_value, y_value in current_vertices
    ]

    sample_vertices: list[list[tuple[float, float]]] = []
    samples: list[BaseGeometry] = []
    for distance in ordered_distances:
        fraction = distance / path_length
        anchor = local_path.interpolate(distance)
        anchor_x = anchor.x + start_offset[0] + ((end_offset[0] - start_offset[0]) * fraction)
        anchor_y = anchor.y + start_offset[1] + ((end_offset[1] - start_offset[1]) * fraction)
        vertices = [
            (
                anchor_x + prev_x + ((curr_x - prev_x) * fraction),
                anchor_y + prev_y + ((curr_y - prev_y) * fraction),
            )
            for (prev_x, prev_y), (curr_x, curr_y) in zip(
                previous_centered,
                current_centered,
            )
        ]
        sample = _normalize_geometry(Polygon(vertices))
        if sample.is_empty or sample.area <= _MIN_AREA_M2:
            return _clip_interpolated_geometry(current, assignment_geometry)
        sample_vertices.append(vertices)
        samples.append(sample)
    return _union_edge_swept_samples(sample_vertices, samples, assignment_geometry)


def build_interpolated_footprint_geometry(
    previous_footprint: BaseGeometry | None,
    current_footprint: BaseGeometry | None,
    interpolation_steps: int,
    *,
    assignment_geometry: BaseGeometry | None = None,
) -> BaseGeometry:
    """Build a conservative continuous camera sweep between telemetry frames.

    Besides sampling the polygon itself, every corresponding edge is swept
    between adjacent samples.  That closes the narrow slits that otherwise
    appear at the left/right sides, the leading/trailing corners, and during
    row-end hairpins.  Continuity is rejected for implausible jumps, while
    distance and turn angle increase the local sample density.  The optional
    assignment clip is applied to every return path so interpolation can never
    claim ground outside the aircraft's assigned region.
    """
    previous = _normalize_geometry(previous_footprint)
    current = _normalize_geometry(current_footprint)
    if current.is_empty:
        return GeometryCollection()
    steps = max(1, min(int(interpolation_steps), 300))
    if previous.is_empty or steps <= 1:
        return _clip_interpolated_geometry(current, assignment_geometry)

    try:
        prev_bounds = previous.bounds
        curr_bounds = current.bounds
        prev_diag = math.hypot(prev_bounds[2] - prev_bounds[0], prev_bounds[3] - prev_bounds[1])
        curr_diag = math.hypot(curr_bounds[2] - curr_bounds[0], curr_bounds[3] - curr_bounds[1])
        max_diag = max(prev_diag, curr_diag, 1e-6)
        if previous.centroid.distance(current.centroid) > max_diag * _SWEEP_BRIDGE_GAP_FACTOR:
            return _clip_interpolated_geometry(current, assignment_geometry)
    except Exception:
        return _clip_interpolated_geometry(current, assignment_geometry)

    previous_vertices = _polygon_vertices(previous)
    current_vertices = _aligned_polygon_vertices(
        previous_vertices,
        _polygon_vertices(current),
    )
    if len(previous_vertices) < 3 or len(previous_vertices) != len(current_vertices):
        return _clip_interpolated_geometry(current, assignment_geometry)

    previous_area = float(max(previous.area, _MIN_AREA_M2))
    current_area = float(max(current.area, _MIN_AREA_M2))
    area_ratio = max(previous_area, current_area) / min(previous_area, current_area)
    if area_ratio > _INTERPOLATION_MAX_AREA_RATIO:
        return _clip_interpolated_geometry(current, assignment_geometry)

    prev_minor, _prev_major, prev_angle = _footprint_scales(previous)
    curr_minor, _curr_major, curr_angle = _footprint_scales(current)
    minor_span = min(span for span in (prev_minor, curr_minor) if span > 1e-6) if (
        prev_minor > 1e-6 or curr_minor > 1e-6
    ) else 0.0
    if minor_span <= 1e-6:
        return _clip_interpolated_geometry(current, assignment_geometry)

    centroid_gap = float(previous.centroid.distance(current.centroid))
    max_vertex_gap = max(
        math.hypot(curr_x - prev_x, curr_y - prev_y)
        for (prev_x, prev_y), (curr_x, curr_y) in zip(previous_vertices, current_vertices)
    )
    turn_radians = _axis_turn_radians(prev_angle, curr_angle)
    step_span_m = max(1e-6, minor_span * _INTERPOLATION_STEP_SPAN_FACTOR)
    required_steps = max(
        steps,
        int(math.ceil(centroid_gap / step_span_m)),
        int(math.ceil(max_vertex_gap / step_span_m)),
        int(
            math.ceil(
                math.degrees(turn_radians) / _INTERPOLATION_MAX_STEP_TURN_DEG
            )
        ),
    )
    if required_steps > 300:
        return _clip_interpolated_geometry(current, assignment_geometry)
    steps = max(2, required_steps)

    sample_vertices: list[list[tuple[float, float]]] = []
    samples: list[BaseGeometry] = []
    for step in range(steps + 1):
        fraction = step / steps
        vertices = [
            (
                prev_x + ((curr_x - prev_x) * fraction),
                prev_y + ((curr_y - prev_y) * fraction),
            )
            for (prev_x, prev_y), (curr_x, curr_y) in zip(
                previous_vertices,
                current_vertices,
            )
        ]
        sample = _normalize_geometry(Polygon(vertices))
        if sample.is_empty or sample.area <= _MIN_AREA_M2:
            return _clip_interpolated_geometry(current, assignment_geometry)
        sample_vertices.append(vertices)
        samples.append(sample)

    return _union_edge_swept_samples(
        sample_vertices,
        samples,
        assignment_geometry,
    )
