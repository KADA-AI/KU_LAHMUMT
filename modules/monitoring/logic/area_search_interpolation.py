# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from typing import Any, Iterable

from shapely.affinity import translate
from shapely.geometry import GeometryCollection, LineString, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import substring, unary_union

_MIN_AREA_M2 = 1e-3
_MAX_AREA_RATIO = 4.0
_MAX_CENTROID_GAP_FACTOR = 3.0


def _as_float(value: object, fallback: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(fallback)


def _as_int(value: object, fallback: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return int(fallback)


def _normalize_geometry(geometry: BaseGeometry | None) -> BaseGeometry:
    if geometry is None or geometry.is_empty:
        return GeometryCollection()
    if not geometry.is_valid:
        try:
            geometry = geometry.buffer(0)
        except Exception:
            return GeometryCollection()
    return geometry if not geometry.is_empty else GeometryCollection()


def _clip_geometry(
    geometry: BaseGeometry | None,
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


def resolve_frame_sample_fractions(
    previous_timestamp_ms: int,
    current_timestamp_ms: int,
    settings: dict[str, Any],
) -> tuple[float, ...] | None:
    """Return exact frame-time fractions including both telemetry endpoints.

    ``None`` means that continuity must be broken. Intermediate samples are
    placed at fixed frame intervals (33.333 ms at 30 Hz), matching the imported
    area-search evaluator rather than evenly redistributing irregular 0401 gaps.
    """
    start_ms = int(previous_timestamp_ms)
    end_ms = int(current_timestamp_ms)
    gap_ms = end_ms - start_ms
    max_gap_ms = max(1, _as_int(settings.get("max_interpolation_gap_ms"), 1000))
    if gap_ms <= 0 or gap_ms > max_gap_ms:
        return None

    frame_rate_hz = max(
        1.0,
        _as_float(settings.get("footprint_interpolation_hz"), 30.0),
    )
    frame_interval_ms = 1000.0 / frame_rate_hz
    max_intervals = max(1, _as_int(settings.get("max_interpolation_steps"), 120))
    required_intervals = max(1, int(math.ceil(gap_ms / frame_interval_ms)))
    if required_intervals > max_intervals:
        return None

    fractions: list[float] = [0.0]
    sample_timestamp = float(start_ms) + frame_interval_ms
    while sample_timestamp < float(end_ms) - 0.5:
        fractions.append((sample_timestamp - float(start_ms)) / float(gap_ms))
        sample_timestamp += frame_interval_ms
    fractions.append(1.0)
    return tuple(fractions)


def _polygon_vertices(geometry: BaseGeometry | None) -> list[tuple[float, float]]:
    normalized = _normalize_geometry(geometry)
    if normalized.geom_type != "Polygon":
        return []
    vertices = [(float(x), float(y)) for x, y in normalized.exterior.coords]
    if len(vertices) >= 2 and vertices[0] == vertices[-1]:
        vertices.pop()
    return vertices


def _align_footprint_corners(
    start_vertices: list[tuple[float, float]],
    end_vertices: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    if len(start_vertices) != len(end_vertices) or len(start_vertices) < 3:
        return []
    corner_orders = (
        end_vertices[index:] + end_vertices[:index]
        for index in range(len(end_vertices))
    )
    return min(
        corner_orders,
        key=lambda vertices: sum(
            math.dist(start, end) ** 2
            for start, end in zip(start_vertices, vertices)
        ),
    )


def _interpolated_polygon(
    start_vertices: list[tuple[float, float]],
    end_vertices: list[tuple[float, float]],
    fraction: float,
) -> BaseGeometry:
    polygon = Polygon(
        [
            (
                start_x + ((end_x - start_x) * fraction),
                start_y + ((end_y - start_y) * fraction),
            )
            for (start_x, start_y), (end_x, end_y) in zip(
                start_vertices,
                end_vertices,
            )
        ]
    )
    return _normalize_geometry(polygon)


def _interpolation_allowed(
    previous: BaseGeometry,
    current: BaseGeometry,
) -> bool:
    previous_area = float(max(previous.area, _MIN_AREA_M2))
    current_area = float(max(current.area, _MIN_AREA_M2))
    if max(previous_area, current_area) / min(previous_area, current_area) > _MAX_AREA_RATIO:
        return False
    try:
        previous_bounds = previous.bounds
        current_bounds = current.bounds
        previous_diagonal = math.hypot(
            previous_bounds[2] - previous_bounds[0],
            previous_bounds[3] - previous_bounds[1],
        )
        current_diagonal = math.hypot(
            current_bounds[2] - current_bounds[0],
            current_bounds[3] - current_bounds[1],
        )
        max_diagonal = max(previous_diagonal, current_diagonal, 1e-6)
        return previous.centroid.distance(current.centroid) <= (
            max_diagonal * _MAX_CENTROID_GAP_FACTOR
        )
    except Exception:
        return False


def build_frame_interpolated_footprint_geometry(
    previous_footprint: BaseGeometry | None,
    current_footprint: BaseGeometry | None,
    sample_fractions: Iterable[float] | None,
    *,
    assignment_geometry: BaseGeometry | None = None,
) -> BaseGeometry:
    """Union only footprints sampled at the configured virtual frame times."""
    previous = _normalize_geometry(previous_footprint)
    current = _normalize_geometry(current_footprint)
    if current.is_empty:
        return GeometryCollection()
    fractions = tuple(sample_fractions or ())
    if previous.is_empty or len(fractions) < 2 or not _interpolation_allowed(previous, current):
        return _clip_geometry(current, assignment_geometry)

    start_vertices = _polygon_vertices(previous)
    end_vertices = _align_footprint_corners(
        start_vertices,
        _polygon_vertices(current),
    )
    if len(start_vertices) < 3 or len(start_vertices) != len(end_vertices):
        return _clip_geometry(current, assignment_geometry)

    samples: list[BaseGeometry] = []
    for raw_fraction in fractions:
        fraction = max(0.0, min(1.0, float(raw_fraction)))
        sample = _interpolated_polygon(start_vertices, end_vertices, fraction)
        if sample.is_empty or sample.area <= _MIN_AREA_M2:
            return _clip_geometry(current, assignment_geometry)
        samples.append(sample)
    return _clip_geometry(unary_union(samples), assignment_geometry)


def _turn_distances(path: LineString) -> set[float]:
    coordinates = list(path.coords)
    if len(coordinates) < 2:
        return {0.0}
    distances = {0.0, float(path.length)}
    cumulative = 0.0
    for index in range(1, len(coordinates) - 1):
        before = coordinates[index - 1]
        current = coordinates[index]
        after = coordinates[index + 1]
        first = (current[0] - before[0], current[1] - before[1])
        second = (after[0] - current[0], after[1] - current[1])
        first_length = math.hypot(*first)
        second_length = math.hypot(*second)
        cumulative += first_length
        if first_length <= 0.1 or second_length <= 0.1:
            continue
        dot = ((first[0] * second[0]) + (first[1] * second[1])) / (
            first_length * second_length
        )
        if dot < 0.5:
            distances.add(float(cumulative))
    return distances


def build_path_frame_interpolated_footprint_geometry(
    previous_footprint: BaseGeometry | None,
    current_footprint: BaseGeometry | None,
    sweep_path: LineString | None,
    previous_chainage_m: float | None,
    current_chainage_m: float | None,
    sample_fractions: Iterable[float] | None,
    *,
    assignment_geometry: BaseGeometry | None = None,
) -> BaseGeometry:
    """Place strict frame samples along the observed part of the planned sweep."""
    previous = _normalize_geometry(previous_footprint)
    current = _normalize_geometry(current_footprint)
    fractions = set(float(value) for value in (sample_fractions or ()))
    if (
        previous.is_empty
        or current.is_empty
        or sweep_path is None
        or previous_chainage_m is None
        or current_chainage_m is None
        or len(fractions) < 2
    ):
        return build_frame_interpolated_footprint_geometry(
            previous,
            current,
            fractions,
            assignment_geometry=assignment_geometry,
        )
    start = max(0.0, min(float(sweep_path.length), float(previous_chainage_m)))
    end = max(0.0, min(float(sweep_path.length), float(current_chainage_m)))
    path_length = end - start
    if path_length <= 1e-3 or path_length > 2500.0:
        return build_frame_interpolated_footprint_geometry(
            previous,
            current,
            fractions,
            assignment_geometry=assignment_geometry,
        )
    try:
        local_path = substring(sweep_path, start, end)
    except Exception:
        local_path = None
    if local_path is None or local_path.is_empty or local_path.geom_type != "LineString":
        return build_frame_interpolated_footprint_geometry(
            previous,
            current,
            fractions,
            assignment_geometry=assignment_geometry,
        )

    start_vertices = _polygon_vertices(previous)
    end_vertices = _align_footprint_corners(start_vertices, _polygon_vertices(current))
    if len(start_vertices) < 3 or len(start_vertices) != len(end_vertices):
        return _clip_geometry(current, assignment_geometry)
    if not _interpolation_allowed(previous, current):
        # Large motion is valid here only because the ordered sweep path explains it.
        previous_area = float(max(previous.area, _MIN_AREA_M2))
        current_area = float(max(current.area, _MIN_AREA_M2))
        if max(previous_area, current_area) / min(previous_area, current_area) > _MAX_AREA_RATIO:
            return _clip_geometry(current, assignment_geometry)

    for distance in _turn_distances(local_path):
        fractions.add(distance / path_length)
    ordered_fractions = sorted(max(0.0, min(1.0, value)) for value in fractions)

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
        (x - previous_centroid.x, y - previous_centroid.y)
        for x, y in start_vertices
    ]
    current_centered = [
        (x - current_centroid.x, y - current_centroid.y)
        for x, y in end_vertices
    ]

    samples: list[BaseGeometry] = []
    for fraction in ordered_fractions:
        distance = path_length * fraction
        anchor = local_path.interpolate(distance)
        anchor_x = anchor.x + start_offset[0] + ((end_offset[0] - start_offset[0]) * fraction)
        anchor_y = anchor.y + start_offset[1] + ((end_offset[1] - start_offset[1]) * fraction)
        vertices = [
            (
                anchor_x + start_x + ((end_x - start_x) * fraction),
                anchor_y + start_y + ((end_y - start_y) * fraction),
            )
            for (start_x, start_y), (end_x, end_y) in zip(
                previous_centered,
                current_centered,
            )
        ]
        sample = _normalize_geometry(Polygon(vertices))
        if sample.is_empty or sample.area <= _MIN_AREA_M2:
            return _clip_geometry(current, assignment_geometry)
        samples.append(sample)
    return _clip_geometry(unary_union(samples), assignment_geometry)


def _footprint_minor_span(footprint: BaseGeometry) -> float:
    try:
        rectangle = _normalize_geometry(footprint.minimum_rotated_rectangle)
        vertices = _polygon_vertices(rectangle)
        if len(vertices) != 4:
            return 0.0
        lengths = [
            math.dist(vertices[index], vertices[(index + 1) % len(vertices)])
            for index in range(len(vertices))
        ]
        return float(min(length for length in lengths if length > 1e-6))
    except Exception:
        return 0.0


def build_sweep_endpoint_fill_geometry(
    source_footprint: BaseGeometry | None,
    sweep_path: LineString | None,
    source_chainage_m: float | None,
    *,
    spacing_fraction: float = 0.5,
    minimum_spacing_m: float = 5.0,
    max_samples: int = 32,
    assignment_geometry: BaseGeometry | None = None,
) -> BaseGeometry:
    """Translate the last valid footprint to a completed sweep endpoint."""
    source = _normalize_geometry(source_footprint)
    if source.is_empty or sweep_path is None or source_chainage_m is None:
        return GeometryCollection()
    start = max(0.0, min(float(sweep_path.length), float(source_chainage_m)))
    remaining = float(sweep_path.length) - start
    if remaining <= 1e-3:
        return GeometryCollection()

    minor_span = _footprint_minor_span(source)
    fraction = max(0.05, min(1.0, float(spacing_fraction)))
    spacing_m = max(1.0, float(minimum_spacing_m), minor_span * fraction)
    sample_count = max(1, int(math.ceil(remaining / spacing_m)))
    sample_count = min(sample_count, max(1, int(max_samples)))
    source_centroid = source.centroid

    samples: list[BaseGeometry] = []
    for sample_index in range(1, sample_count + 1):
        ratio = sample_index / sample_count
        target = sweep_path.interpolate(start + (remaining * ratio))
        samples.append(
            translate(
                source,
                xoff=float(target.x - source_centroid.x),
                yoff=float(target.y - source_centroid.y),
            )
        )
    return _clip_geometry(unary_union(samples), assignment_geometry)
