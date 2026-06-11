# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable

from shapely.geometry import LineString, Point
from shapely.geometry.base import BaseGeometry

_MIN_ROW_LENGTH_M = 2.0
_MAX_BRIDGE_TIME_MS = 15000
_MIN_ALONG_ROW_RATIO = 0.25


@dataclass(frozen=True)
class AreaRowProgressDefinition:
    lines: tuple[LineString, ...]
    line_lengths_m: tuple[float, ...]
    match_tolerance_m: float
    sample_radius_m: float
    max_bridge_gap_m: float
    completion_min_m: float
    completion_ratio: float


@dataclass
class AreaRowProgressState:
    covered_intervals_by_row: dict[int, list[tuple[float, float]]] = field(default_factory=dict)
    completed_indexes: set[int] = field(default_factory=set)
    last_row_index: int | None = None
    last_chainage_m: float | None = None
    last_xy: tuple[float, float] | None = None
    last_timestamp_ms: int | None = None
    row_direction_by_index: dict[int, int] = field(default_factory=dict)
    row_boundary_by_index: dict[int, float] = field(default_factory=dict)


@dataclass(frozen=True)
class AreaRowProgressUpdate:
    row_index: int | None
    chainage_m: float | None
    distance_m: float | None
    frontier_index: int | None
    completed_indexes: tuple[int, ...]
    changed: bool


def clone_area_row_progress_state(state: AreaRowProgressState | None) -> AreaRowProgressState:
    if state is None:
        return AreaRowProgressState()
    return AreaRowProgressState(
        covered_intervals_by_row={
            int(index): [(float(start), float(end)) for start, end in intervals]
            for index, intervals in (state.covered_intervals_by_row or {}).items()
        },
        completed_indexes={int(index) for index in (state.completed_indexes or set())},
        last_row_index=None if state.last_row_index is None else int(state.last_row_index),
        last_chainage_m=None if state.last_chainage_m is None else float(state.last_chainage_m),
        last_xy=None if state.last_xy is None else (float(state.last_xy[0]), float(state.last_xy[1])),
        last_timestamp_ms=None if state.last_timestamp_ms is None else int(state.last_timestamp_ms),
        row_direction_by_index={
            int(index): int(direction)
            for index, direction in (state.row_direction_by_index or {}).items()
        },
        row_boundary_by_index={
            int(index): float(boundary)
            for index, boundary in (state.row_boundary_by_index or {}).items()
        },
    )


def build_area_row_progress_definition(
    planned_cut_lines: Iterable[BaseGeometry],
    *,
    width_hint_m: float | None,
    cut_half_width_m: float,
) -> AreaRowProgressDefinition | None:
    lines: list[LineString] = []
    lengths: list[float] = []
    for geometry in planned_cut_lines or []:
        line = _longest_line_string(geometry)
        if line is None:
            continue
        length_m = float(line.length or 0.0)
        if length_m <= _MIN_ROW_LENGTH_M:
            continue
        lines.append(line)
        lengths.append(length_m)
    if not lines:
        return None

    half_width = max(0.8, float(cut_half_width_m or 0.0))
    width_hint = max(8.0, float(width_hint_m or 0.0), half_width * 2.0)
    match_tolerance_m = max(12.0, min((half_width * 3.8) + (width_hint * 0.18), 120.0))
    sample_radius_m = max(3.5, min(max(width_hint * 0.10, half_width * 1.4), 45.0))
    max_bridge_gap_m = max(80.0, min(width_hint * 8.0, 1200.0))
    completion_min_m = max(8.0, min(max(width_hint * 0.35, half_width * 3.2), 120.0))
    completion_ratio = 0.42
    return AreaRowProgressDefinition(
        lines=tuple(lines),
        line_lengths_m=tuple(float(value) for value in lengths),
        match_tolerance_m=float(match_tolerance_m),
        sample_radius_m=float(sample_radius_m),
        max_bridge_gap_m=float(max_bridge_gap_m),
        completion_min_m=float(completion_min_m),
        completion_ratio=float(completion_ratio),
    )


def reset_area_row_live_state(state: AreaRowProgressState, *, timestamp_ms: int | None = None) -> None:
    state.last_row_index = None
    state.last_chainage_m = None
    state.last_xy = None
    state.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else None


def update_area_row_progress_state(
    definition: AreaRowProgressDefinition,
    state: AreaRowProgressState,
    center_xy: tuple[float, float],
    *,
    timestamp_ms: int | None,
    filming: Any = None,
) -> AreaRowProgressUpdate:
    if _coerce_int(filming) == 0:
        reset_area_row_live_state(state, timestamp_ms=timestamp_ms)
        return _result(state, row_index=None, chainage_m=None, distance_m=None, changed=False)

    point = Point(float(center_xy[0]), float(center_xy[1]))
    row_index, distance_m, chainage_m = _nearest_row(definition, point)
    if row_index is None or distance_m is None or chainage_m is None:
        reset_area_row_live_state(state, timestamp_ms=timestamp_ms)
        return _result(state, row_index=None, chainage_m=None, distance_m=None, changed=False)
    if float(distance_m) > float(definition.match_tolerance_m):
        reset_area_row_live_state(state, timestamp_ms=timestamp_ms)
        return _result(
            state,
            row_index=int(row_index),
            chainage_m=float(chainage_m),
            distance_m=float(distance_m),
            changed=False,
        )

    radius = float(definition.sample_radius_m)
    length_m = float(definition.line_lengths_m[int(row_index)])
    start_m = float(chainage_m) - radius
    end_m = float(chainage_m) + radius
    direction_sign = state.row_direction_by_index.get(int(row_index))
    can_bridge = (
        state.last_row_index == int(row_index)
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
        prev_xy = state.last_xy or center_xy
        step_m = math.hypot(float(center_xy[0]) - float(prev_xy[0]), float(center_xy[1]) - float(prev_xy[1]))
        chain_delta_m = abs(float(chainage_m) - float(state.last_chainage_m or 0.0))
        if (
            step_m <= float(definition.max_bridge_gap_m)
            and chain_delta_m >= 0.5
            and (step_m <= 1e-6 or (chain_delta_m / max(step_m, 1e-6)) >= _MIN_ALONG_ROW_RATIO)
        ):
            observed_direction = 1 if float(chainage_m) >= float(state.last_chainage_m or 0.0) else -1
            if direction_sign is None:
                direction_sign = int(observed_direction)
                state.row_direction_by_index[int(row_index)] = int(direction_sign)
            if int(direction_sign) >= 0:
                previous_boundary = float(state.row_boundary_by_index.get(int(row_index), 0.0))
                boundary = max(
                    previous_boundary,
                    float(state.last_chainage_m or 0.0) + radius,
                    float(chainage_m) + radius,
                )
                start_m = 0.0
                end_m = min(length_m, boundary)
                state.row_boundary_by_index[int(row_index)] = float(end_m)
            else:
                previous_boundary = float(state.row_boundary_by_index.get(int(row_index), length_m))
                boundary = min(
                    previous_boundary,
                    float(state.last_chainage_m or 0.0) - radius,
                    float(chainage_m) - radius,
                )
                start_m = max(0.0, boundary)
                end_m = length_m
                state.row_boundary_by_index[int(row_index)] = float(start_m)

    changed = _add_interval(definition, state, int(row_index), start_m, end_m)
    before_completed = set(state.completed_indexes)
    _refresh_completed_rows(definition, state)
    if set(state.completed_indexes) != before_completed:
        changed = True

    state.last_row_index = int(row_index)
    state.last_chainage_m = float(chainage_m)
    state.last_xy = (float(center_xy[0]), float(center_xy[1]))
    state.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else None
    return _result(
        state,
        row_index=int(row_index),
        chainage_m=float(chainage_m),
        distance_m=float(distance_m),
        changed=bool(changed),
    )


def area_row_frontier_index(state: AreaRowProgressState | None) -> int | None:
    if state is None or not state.completed_indexes:
        return None
    frontier = -1
    for index in sorted(int(value) for value in state.completed_indexes if int(value) >= 0):
        if index == frontier + 1:
            frontier = int(index)
            continue
        if index <= frontier:
            continue
        break
    return int(frontier) if frontier >= 0 else None


def area_row_completed_indexes(state: AreaRowProgressState | None) -> tuple[int, ...]:
    if state is None:
        return ()
    return tuple(sorted(int(index) for index in state.completed_indexes if int(index) >= 0))


def _longest_line_string(geometry: BaseGeometry | None) -> LineString | None:
    if geometry is None or geometry.is_empty:
        return None
    if isinstance(geometry, LineString):
        return geometry
    geoms = getattr(geometry, "geoms", None)
    if geoms is None:
        return None
    lines = [item for item in geoms if isinstance(item, LineString) and not item.is_empty]
    if not lines:
        return None
    return max(lines, key=lambda item: float(item.length or 0.0))


def _coerce_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _nearest_row(
    definition: AreaRowProgressDefinition,
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


def _merge_intervals(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    ordered = sorted((float(start), float(end)) for start, end in intervals if end > start)
    if not ordered:
        return []
    merged: list[tuple[float, float]] = [ordered[0]]
    for start, end in ordered[1:]:
        prev_start, prev_end = merged[-1]
        if start <= prev_end + 1e-6:
            merged[-1] = (prev_start, max(prev_end, end))
        else:
            merged.append((start, end))
    return merged


def _add_interval(
    definition: AreaRowProgressDefinition,
    state: AreaRowProgressState,
    row_index: int,
    start_m: float,
    end_m: float,
) -> bool:
    if row_index < 0 or row_index >= len(definition.line_lengths_m):
        return False
    length_m = float(definition.line_lengths_m[int(row_index)])
    start = max(0.0, min(float(start_m), length_m))
    end = max(0.0, min(float(end_m), length_m))
    if end <= start:
        return False
    before = _covered_length(state.covered_intervals_by_row.get(int(row_index), []))
    intervals = state.covered_intervals_by_row.setdefault(int(row_index), [])
    intervals.append((start, end))
    state.covered_intervals_by_row[int(row_index)] = _merge_intervals(intervals)
    after = _covered_length(state.covered_intervals_by_row.get(int(row_index), []))
    return after > before + 1e-6


def _covered_length(intervals: list[tuple[float, float]]) -> float:
    return float(sum(max(0.0, float(end) - float(start)) for start, end in intervals))


def _row_completion_threshold_m(definition: AreaRowProgressDefinition, row_index: int) -> float:
    length_m = float(definition.line_lengths_m[int(row_index)])
    if length_m <= 0.0:
        return 0.0
    return min(
        max(float(definition.completion_min_m), length_m * float(definition.completion_ratio)),
        length_m * 0.82,
    )


def _refresh_completed_rows(
    definition: AreaRowProgressDefinition,
    state: AreaRowProgressState,
) -> None:
    for row_index, intervals in list(state.covered_intervals_by_row.items()):
        if int(row_index) < 0 or int(row_index) >= len(definition.line_lengths_m):
            continue
        covered_m = _covered_length(_merge_intervals(intervals))
        if covered_m >= _row_completion_threshold_m(definition, int(row_index)):
            state.completed_indexes.add(int(row_index))


def _result(
    state: AreaRowProgressState,
    *,
    row_index: int | None,
    chainage_m: float | None,
    distance_m: float | None,
    changed: bool,
) -> AreaRowProgressUpdate:
    return AreaRowProgressUpdate(
        row_index=row_index,
        chainage_m=chainage_m,
        distance_m=distance_m,
        frontier_index=area_row_frontier_index(state),
        completed_indexes=area_row_completed_indexes(state),
        changed=bool(changed),
    )
