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
_RECENT_ROW_WINDOW_MS = 3000
_RECENT_ROW_SAMPLE_LIMIT = 18
_RECENT_ROW_CONFIRM_SAMPLE_COUNT = 15
_RECENT_ROW_MIN_SAMPLES = 5
_RECENT_ROW_MIN_AGREEMENT_RATIO = 0.35
_RECENT_ROW_TAIL_SAMPLE_COUNT = 5
_RECENT_ROW_MIN_TAIL_AGREEMENT_RATIO = 0.45
_MAX_ROW_ADVANCE_PER_UPDATE = 1


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
    recent_row_samples: list[tuple[int, int, float, float, float, float]] = field(default_factory=list)
    accepted_frontier_index: int | None = None
    last_row_index: int | None = None
    last_chainage_m: float | None = None
    last_xy: tuple[float, float] | None = None
    last_timestamp_ms: int | None = None
    row_index_direction_sign: int | None = None
    row_direction_by_index: dict[int, int] = field(default_factory=dict)
    row_boundary_by_index: dict[int, float] = field(default_factory=dict)


@dataclass(frozen=True)
class AreaRowProgressUpdate:
    row_index: int | None
    chainage_m: float | None
    distance_m: float | None
    frontier_index: int | None
    row_index_direction_sign: int | None
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
        recent_row_samples=[
            (
                int(ts),
                int(row),
                float(chainage),
                float(distance),
                float(x_val),
                float(y_val),
            )
            for ts, row, chainage, distance, x_val, y_val in (state.recent_row_samples or [])
        ],
        accepted_frontier_index=(
            None
            if state.accepted_frontier_index is None
            else int(state.accepted_frontier_index)
        ),
        last_row_index=None if state.last_row_index is None else int(state.last_row_index),
        last_chainage_m=None if state.last_chainage_m is None else float(state.last_chainage_m),
        last_xy=None if state.last_xy is None else (float(state.last_xy[0]), float(state.last_xy[1])),
        last_timestamp_ms=None if state.last_timestamp_ms is None else int(state.last_timestamp_ms),
        row_index_direction_sign=(
            None
            if state.row_index_direction_sign is None
            else (1 if int(state.row_index_direction_sign) >= 0 else -1)
        ),
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
    state.recent_row_samples = []


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

    previous_row_index = state.last_row_index
    previous_chainage_m = state.last_chainage_m
    before_completed = set(state.completed_indexes)
    interval_changed = False
    if previous_row_index is not None and int(previous_row_index) == int(row_index) and previous_chainage_m is not None:
        interval_changed = _add_interval(
            definition,
            state,
            int(row_index),
            min(float(previous_chainage_m), float(chainage_m)) - float(definition.sample_radius_m),
            max(float(previous_chainage_m), float(chainage_m)) + float(definition.sample_radius_m),
        )
    else:
        interval_changed = _add_interval(
            definition,
            state,
            int(row_index),
            float(chainage_m) - float(definition.sample_radius_m),
            float(chainage_m) + float(definition.sample_radius_m),
        )
    _refresh_completed_rows(definition, state)

    state.last_row_index = int(row_index)
    state.last_chainage_m = float(chainage_m)
    state.last_xy = (float(center_xy[0]), float(center_xy[1]))
    state.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else None
    _append_recent_row_sample(
        state,
        row_index=int(row_index),
        chainage_m=float(chainage_m),
        distance_m=float(distance_m),
        center_xy=center_xy,
        timestamp_ms=timestamp_ms,
    )

    stable_row_index = _stable_recent_row_index(state)
    if stable_row_index is None:
        return _result(
            state,
            row_index=int(row_index),
            chainage_m=float(chainage_m),
            distance_m=float(distance_m),
            changed=bool(interval_changed or set(state.completed_indexes) != before_completed),
        )

    bounded_frontier = _bounded_frontier_index(state, int(stable_row_index))
    if bounded_frontier is None:
        return _result(
            state,
            row_index=int(row_index),
            chainage_m=float(chainage_m),
            distance_m=float(distance_m),
            changed=False,
        )

    bounded_frontier = max(0, min(int(bounded_frontier), len(definition.lines) - 1))
    state.row_index_direction_sign = 1
    state.accepted_frontier_index = int(bounded_frontier)
    # 프리픽스로 교체하면 구간 누적으로 완료된 비프리픽스 행이 지워져 완료 집합이
    # 후퇴한다 — 합집합으로 단조 유지.
    state.completed_indexes = set(state.completed_indexes or set()) | set(
        range(0, int(bounded_frontier) + 1)
    )
    changed = bool(interval_changed or set(state.completed_indexes) != before_completed)
    return _result(
        state,
        row_index=int(row_index),
        chainage_m=float(chainage_m),
        distance_m=float(distance_m),
        changed=bool(changed),
    )


def _sample_timestamp_ms(
    state: AreaRowProgressState,
    timestamp_ms: int | None,
) -> int:
    if timestamp_ms is not None:
        return int(timestamp_ms)
    if state.last_timestamp_ms is not None:
        return int(state.last_timestamp_ms) + 200
    return 0


def _append_recent_row_sample(
    state: AreaRowProgressState,
    *,
    row_index: int,
    chainage_m: float,
    distance_m: float,
    center_xy: tuple[float, float],
    timestamp_ms: int | None,
) -> None:
    sample_ts = _sample_timestamp_ms(state, timestamp_ms)
    state.recent_row_samples.append(
        (
            int(sample_ts),
            int(row_index),
            float(chainage_m),
            float(distance_m),
            float(center_xy[0]),
            float(center_xy[1]),
        )
    )
    cutoff = int(sample_ts) - int(_RECENT_ROW_WINDOW_MS)
    state.recent_row_samples = [
        sample
        for sample in state.recent_row_samples[-int(_RECENT_ROW_SAMPLE_LIMIT):]
        if int(sample[0]) >= cutoff
    ]


def _stable_recent_row_index(state: AreaRowProgressState) -> int | None:
    samples = list(state.recent_row_samples or [])
    if len(samples) < int(_RECENT_ROW_MIN_SAMPLES):
        return None
    recent = samples[-int(_RECENT_ROW_CONFIRM_SAMPLE_COUNT):]
    counts: dict[int, int] = {}
    for sample in recent:
        row_index = int(sample[1])
        counts[row_index] = int(counts.get(row_index, 0)) + 1
    if not counts:
        return None
    candidate = int(recent[-1][1])
    min_count = max(
        2,
        int(math.ceil(float(len(recent)) * float(_RECENT_ROW_MIN_AGREEMENT_RATIO))),
    )
    if int(counts.get(candidate, 0)) < int(min_count):
        return None
    tail = recent[-int(_RECENT_ROW_TAIL_SAMPLE_COUNT):]
    tail_count = sum(1 for sample in tail if int(sample[1]) == int(candidate))
    min_tail_count = max(
        2,
        int(math.ceil(float(len(tail)) * float(_RECENT_ROW_MIN_TAIL_AGREEMENT_RATIO))),
    )
    if int(tail_count) < int(min_tail_count):
        return None
    return int(candidate)


def _bounded_frontier_index(
    state: AreaRowProgressState,
    candidate_index: int,
) -> int | None:
    current_frontier = state.accepted_frontier_index
    if current_frontier is None and state.completed_indexes:
        current_frontier = max(int(index) for index in state.completed_indexes)
    if current_frontier is None:
        return min(
            max(0, int(candidate_index)),
            int(_MAX_ROW_ADVANCE_PER_UPDATE),
        )
    if int(candidate_index) <= int(current_frontier):
        return None
    max_next = int(current_frontier) + int(_MAX_ROW_ADVANCE_PER_UPDATE)
    if int(candidate_index) > max_next:
        return int(current_frontier) + 1
    return int(candidate_index)


def area_row_frontier_index(state: AreaRowProgressState | None) -> int | None:
    if state is None or not state.completed_indexes:
        return None
    completed = sorted(int(value) for value in state.completed_indexes if int(value) >= 0)
    if not completed:
        return None
    direction_sign = state.row_index_direction_sign
    if direction_sign is None:
        if state.last_row_index is not None and int(state.last_row_index) in completed:
            return int(state.last_row_index)
        return int(completed[0])
    if int(direction_sign) < 0:
        frontier = int(completed[-1])
        for index in sorted(completed, reverse=True)[1:]:
            if int(index) == int(frontier) - 1:
                frontier = int(index)
                continue
            break
        return int(frontier)
    frontier = int(completed[0])
    for index in completed[1:]:
        if int(index) == int(frontier) + 1:
            frontier = int(index)
            continue
        break
    return int(frontier)


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
        row_index_direction_sign=(
            None
            if state.row_index_direction_sign is None
            else (1 if int(state.row_index_direction_sign) >= 0 else -1)
        ),
        completed_indexes=area_row_completed_indexes(state),
        changed=bool(changed),
    )
