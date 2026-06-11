# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Any

from pyproj import Transformer
from shapely.geometry import LineString, Point

from modules.monitoring.logic.mission_coverage import (
    MissionCoverageDefinition,
    _coerce_float,
)

_DEFAULT_WIDTH_HINT_M = 30.0
_MIN_LINE_LENGTH_M = 1.0
_MIN_ALONG_TRACK_RATIO = 0.35
_MAX_BRIDGE_TIME_MS = 15000


@dataclass(frozen=True)
class LineSweepDefinition:
    lines: tuple[LineString, ...]
    line_lengths_m: tuple[float, ...]
    planned_length_m: float
    transformer: Transformer
    width_hint_m: float
    match_tolerance_m: float
    sample_radius_m: float
    max_bridge_gap_m: float


@dataclass
class LineSweepState:
    covered_intervals_by_line: dict[int, list[tuple[float, float]]] = field(default_factory=dict)
    covered_length_m: float = 0.0
    progress_direction_by_line: dict[int, int] = field(default_factory=dict)
    progress_boundary_by_line: dict[int, float] = field(default_factory=dict)
    last_line_index: int | None = None
    last_chainage_m: float | None = None
    last_xy: tuple[float, float] | None = None
    last_timestamp_ms: int | None = None


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coord_lat_lon(coord: object) -> tuple[float, float] | None:
    if not isinstance(coord, dict):
        return None
    raw_lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
    raw_lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
    lat = _coerce_float(raw_lat)
    lon = _coerce_float(raw_lon)
    if lat is None or lon is None:
        return None
    if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
        return None
    return float(lat), float(lon)


def _project_coord(coord: object, transformer: Transformer) -> tuple[float, float] | None:
    value = _coord_lat_lon(coord)
    if value is None:
        return None
    lat, lon = value
    x, y = transformer.transform(lon, lat)
    return float(x), float(y)


@lru_cache(maxsize=64)
def _utm_transformer(zone: int, south: bool) -> Transformer:
    utm_code = f"+proj=utm +zone={zone} {'+south' if south else ''} +ellps=WGS84 +units=m +no_defs"
    return Transformer.from_crs("EPSG:4326", utm_code, always_xy=True)


def _resolve_transformer_for_lists(coordinate_lists: list[list[dict[str, Any]]]) -> Transformer | None:
    coords: list[tuple[float, float]] = []
    for items in coordinate_lists:
        for item in items or []:
            value = _coord_lat_lon(item)
            if value is not None:
                coords.append(value)
    if not coords:
        return None
    avg_lat = sum(lat for lat, _lon in coords) / len(coords)
    avg_lon = sum(lon for _lat, lon in coords) / len(coords)
    zone = int((avg_lon + 180.0) // 6.0) + 1
    zone = max(1, min(zone, 60))
    return _utm_transformer(zone, avg_lat < 0.0)


def _project_points_any_case(
    items: list[dict[str, Any]],
    transformer: Transformer,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    seen: set[tuple[int, int]] = set()
    for item in items or []:
        xy = _project_coord(item, transformer)
        if xy is None:
            continue
        key = (int(round(float(xy[0]) * 1000.0)), int(round(float(xy[1]) * 1000.0)))
        if key in seen:
            continue
        seen.add(key)
        points.append((float(xy[0]), float(xy[1])))
    return points


def _valid_coordinate_lists(items: object) -> list[list[dict[str, Any]]]:
    result: list[list[dict[str, Any]]] = []
    if not isinstance(items, list):
        return result
    for raw in items:
        if not isinstance(raw, list):
            continue
        coords = [dict(item) for item in raw if isinstance(item, dict)]
        if len(coords) >= 2:
            result.append(coords)
    return result


def _line_list_coordinate_lists(items: object) -> list[list[dict[str, Any]]]:
    result: list[list[dict[str, Any]]] = []
    if not isinstance(items, list):
        return result
    for line in items:
        if not isinstance(line, dict):
            continue
        coords = [dict(item) for item in line.get("coordinateList") or [] if isinstance(item, dict)]
        if len(coords) >= 2:
            result.append(coords)
    return result


def _coordinate_list_from_key(mission: dict[str, Any], key: str) -> list[dict[str, Any]]:
    coords = [
        dict(item)
        for item in mission.get(key) or []
        if isinstance(item, dict)
    ]
    return coords if len(coords) >= 2 else []


def _has_line_source_hint(mission: dict[str, Any]) -> bool:
    if _line_list_coordinate_lists(mission.get("input_line_list")):
        return True
    if _line_list_coordinate_lists(mission.get("line_list")):
        return True
    if (_coerce_int(mission.get("sweep_point_count")) or 0) > 0:
        return True
    if _valid_coordinate_lists(mission.get("sweep_line_coordinate_lists")):
        return True
    return False


def _centerline_coordinate_lists(mission: dict[str, Any]) -> list[list[dict[str, Any]]]:
    for key in ("line_list", "input_line_list"):
        coordinate_lists = _line_list_coordinate_lists(mission.get(key))
        if coordinate_lists:
            return coordinate_lists

    if _has_line_source_hint(mission):
        for key in ("coordinate_list", "input_coordinate_list"):
            coords = _coordinate_list_from_key(mission, key)
            if len(coords) >= 2:
                return [coords]

    return _valid_coordinate_lists(mission.get("sweep_line_coordinate_lists"))


def _width_hint_m(mission: dict[str, Any]) -> float:
    candidates: list[float] = []
    for key in ("width_m", "sep_m"):
        value = _coerce_float(mission.get(key))
        if value is not None and value > 0.0:
            candidates.append(float(value))
    for rows_key in ("line_list", "input_line_list"):
        for line in mission.get(rows_key) or []:
            if not isinstance(line, dict):
                continue
            width = _coerce_float(line.get("width"))
            if width is not None and width > 0.0:
                candidates.append(float(width))
    if not candidates:
        return _DEFAULT_WIDTH_HINT_M
    return max(_DEFAULT_WIDTH_HINT_M, min(max(candidates), 2000.0))


def _build_tuning(width_hint_m: float) -> tuple[float, float, float]:
    width = max(_DEFAULT_WIDTH_HINT_M, float(width_hint_m))
    match_tolerance_m = max(25.0, min((width * 0.55) + 30.0, 900.0))
    sample_radius_m = max(5.0, min(width * 0.08, 60.0))
    max_bridge_gap_m = max(150.0, min(width * 10.0, 1500.0))
    return match_tolerance_m, sample_radius_m, max_bridge_gap_m


def build_line_sweep_definition(
    mission: dict[str, Any],
    *,
    coverage_def: MissionCoverageDefinition | None = None,
) -> LineSweepDefinition | None:
    coordinate_lists = _centerline_coordinate_lists(mission)
    if not coordinate_lists:
        return None

    transformer = coverage_def.transformer if coverage_def is not None else None
    if transformer is None:
        transformer = _resolve_transformer_for_lists(coordinate_lists)
    if transformer is None:
        return None

    lines: list[LineString] = []
    lengths: list[float] = []
    seen: set[tuple[tuple[int, int], ...]] = set()
    for coords in coordinate_lists:
        points = _project_points_any_case(coords, transformer)
        if len(points) < 2:
            continue
        line = LineString(points)
        if line.is_empty:
            continue
        length_m = float(line.length or 0.0)
        if length_m <= _MIN_LINE_LENGTH_M:
            continue
        signature = tuple((int(round(x * 10.0)), int(round(y * 10.0))) for x, y in points)
        reverse_signature = tuple(reversed(signature))
        if signature in seen or reverse_signature in seen:
            continue
        seen.add(signature)
        lines.append(line)
        lengths.append(length_m)
    planned_length_m = float(sum(lengths))
    if not lines or planned_length_m <= _MIN_LINE_LENGTH_M:
        return None

    width_hint_m = _width_hint_m(mission)
    match_tolerance_m, sample_radius_m, max_bridge_gap_m = _build_tuning(width_hint_m)
    return LineSweepDefinition(
        lines=tuple(lines),
        line_lengths_m=tuple(lengths),
        planned_length_m=planned_length_m,
        transformer=transformer,
        width_hint_m=width_hint_m,
        match_tolerance_m=match_tolerance_m,
        sample_radius_m=sample_radius_m,
        max_bridge_gap_m=max_bridge_gap_m,
    )


def reset_line_sweep_state(state: LineSweepState) -> None:
    state.covered_intervals_by_line.clear()
    state.covered_length_m = 0.0
    state.progress_direction_by_line.clear()
    state.progress_boundary_by_line.clear()
    state.last_line_index = None
    state.last_chainage_m = None
    state.last_xy = None
    state.last_timestamp_ms = None


def force_complete_line_sweep_state(definition: LineSweepDefinition, state: LineSweepState) -> None:
    state.covered_intervals_by_line = {
        int(idx): [(0.0, float(length))]
        for idx, length in enumerate(definition.line_lengths_m)
        if float(length) > 0.0
    }
    state.covered_length_m = float(definition.planned_length_m)
    state.progress_direction_by_line = {}
    state.progress_boundary_by_line = {}
    state.last_line_index = None
    state.last_chainage_m = None
    state.last_xy = None
    state.last_timestamp_ms = None


def _merge_intervals(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not intervals:
        return []
    ordered = sorted((float(a), float(b)) for a, b in intervals if b > a)
    if not ordered:
        return []
    merged: list[tuple[float, float]] = [ordered[0]]
    for start, end in ordered[1:]:
        last_start, last_end = merged[-1]
        if start <= last_end + 1e-6:
            merged[-1] = (last_start, max(last_end, end))
        else:
            merged.append((start, end))
    return merged


def _covered_length(intervals_by_line: dict[int, list[tuple[float, float]]]) -> float:
    total = 0.0
    for intervals in intervals_by_line.values():
        for start, end in intervals:
            total += max(0.0, float(end) - float(start))
    return float(total)


def _add_interval(
    definition: LineSweepDefinition,
    state: LineSweepState,
    line_index: int,
    start_m: float,
    end_m: float,
) -> bool:
    if line_index < 0 or line_index >= len(definition.line_lengths_m):
        return False
    length_m = float(definition.line_lengths_m[int(line_index)])
    start = max(0.0, min(float(start_m), length_m))
    end = max(0.0, min(float(end_m), length_m))
    if end <= start:
        return False
    before = float(state.covered_length_m)
    intervals = state.covered_intervals_by_line.setdefault(int(line_index), [])
    intervals.append((start, end))
    state.covered_intervals_by_line[int(line_index)] = _merge_intervals(intervals)
    state.covered_length_m = min(
        float(definition.planned_length_m),
        _covered_length(state.covered_intervals_by_line),
    )
    return state.covered_length_m > before + 1e-6


def _nearest_line(
    definition: LineSweepDefinition,
    point: Point,
) -> tuple[int | None, float | None, float | None]:
    best_index: int | None = None
    best_distance: float | None = None
    best_chainage: float | None = None
    for idx, line in enumerate(definition.lines):
        try:
            distance = float(line.distance(point))
            chainage = float(line.project(point))
        except Exception:
            continue
        if best_distance is None or distance < best_distance:
            best_index = int(idx)
            best_distance = distance
            best_chainage = chainage
    return best_index, best_distance, best_chainage


def update_line_sweep_state(
    definition: LineSweepDefinition,
    state: LineSweepState,
    agent_state: dict[str, Any],
    *,
    timestamp_ms: int | None,
) -> bool:
    filming = _coerce_int(agent_state.get("filming"))
    if filming == 0:
        state.last_line_index = None
        state.last_chainage_m = None
        state.last_xy = None
        state.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else None
        return False

    coord = agent_state.get("sensor_center_coordinate") or agent_state.get("coordinate")
    xy = _project_coord(coord, definition.transformer)
    if xy is None:
        return False

    point = Point(xy)
    line_index, distance_m, chainage_m = _nearest_line(definition, point)
    if line_index is None or distance_m is None or chainage_m is None:
        state.last_line_index = None
        state.last_chainage_m = None
        state.last_xy = xy
        state.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else None
        return False

    if float(distance_m) > float(definition.match_tolerance_m):
        state.last_line_index = None
        state.last_chainage_m = None
        state.last_xy = xy
        state.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else None
        return False

    radius = float(definition.sample_radius_m)
    line_length_m = float(definition.line_lengths_m[int(line_index)])
    start_m = float(chainage_m) - radius
    end_m = float(chainage_m) + radius
    direction_sign = state.progress_direction_by_line.get(int(line_index))

    can_bridge = (
        state.last_line_index == int(line_index)
        and state.last_chainage_m is not None
        and state.last_xy is not None
    )
    if can_bridge and timestamp_ms is not None and state.last_timestamp_ms is not None:
        try:
            if int(timestamp_ms) - int(state.last_timestamp_ms) > _MAX_BRIDGE_TIME_MS:
                can_bridge = False
        except Exception:
            can_bridge = False
    if can_bridge:
        prev_xy = state.last_xy
        step_m = math.hypot(float(xy[0]) - float(prev_xy[0]), float(xy[1]) - float(prev_xy[1]))
        chain_delta_m = abs(float(chainage_m) - float(state.last_chainage_m))
        if (
            step_m <= float(definition.max_bridge_gap_m)
            and chain_delta_m >= 0.5
            and (step_m <= 1e-6 or (chain_delta_m / max(step_m, 1e-6)) >= _MIN_ALONG_TRACK_RATIO)
        ):
            observed_direction = 1 if float(chainage_m) >= float(state.last_chainage_m) else -1
            if direction_sign is None:
                direction_sign = int(observed_direction)
                state.progress_direction_by_line[int(line_index)] = int(direction_sign)
            if int(direction_sign) >= 0:
                previous_boundary = float(state.progress_boundary_by_line.get(int(line_index), 0.0))
                boundary = max(
                    previous_boundary,
                    float(state.last_chainage_m) + radius,
                    float(chainage_m) + radius,
                )
                start_m = 0.0
                end_m = min(line_length_m, boundary)
                state.progress_boundary_by_line[int(line_index)] = float(end_m)
            else:
                previous_boundary = float(state.progress_boundary_by_line.get(int(line_index), line_length_m))
                boundary = min(
                    previous_boundary,
                    float(state.last_chainage_m) - radius,
                    float(chainage_m) - radius,
                )
                start_m = max(0.0, boundary)
                end_m = line_length_m
                state.progress_boundary_by_line[int(line_index)] = float(start_m)

    changed = _add_interval(definition, state, int(line_index), start_m, end_m)
    state.last_line_index = int(line_index)
    state.last_chainage_m = float(chainage_m)
    state.last_xy = xy
    state.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else None
    return changed


def line_sweep_metrics(
    definition: LineSweepDefinition | None,
    state: LineSweepState | None,
) -> tuple[float, float, int, bool]:
    if definition is None:
        return 0.0, 0.0, 0, False
    planned = float(max(0.0, definition.planned_length_m))
    covered = float(max(0.0, state.covered_length_m if state else 0.0))
    if planned <= 0.0:
        return covered, planned, 0, False
    percent = int(round((covered / planned) * 100.0))
    return covered, planned, max(0, min(100, percent)), True
