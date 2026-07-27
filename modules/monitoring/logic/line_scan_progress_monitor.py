# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import math
import threading
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Callable

from shapely.geometry import LineString, Point

from modules.common import db_paths
from modules.monitoring.logic.mission_line_progress import (
    LineSweepDefinition,
    LineSweepState,
    build_line_sweep_definition,
    force_complete_line_sweep_state,
    line_sweep_metrics,
    reset_line_sweep_state,
    update_line_sweep_state,
)
from modules.monitoring.logic.mission_update import build_uav_mission_view


def _to_int(value: object | None) -> int | None:
    try:
        return None if value is None else int(value)
    except Exception:
        return None


def _to_float(value: object | None) -> float | None:
    try:
        return None if value is None else float(value)
    except Exception:
        return None


def _round_m(value: object, digits: int = 3) -> float:
    try:
        return round(float(value), int(digits))
    except Exception:
        return 0.0


def _coord_list(value: object, *, min_len: int = 0) -> list[dict[str, Any]]:
    if not isinstance(value, list):
        return []
    rows = [dict(item) for item in value if isinstance(item, dict)]
    if len(rows) < int(min_len):
        return []
    return rows


def _mission_source_width_m(mission: dict[str, Any]) -> float | None:
    # The 0201 LINE width is the authoritative corridor width. SEP is an
    # execution/search spacing and must never be promoted to source width.
    for rows_key in ("input_line_list",):
        candidates: list[float] = []
        for row in mission.get(rows_key) or []:
            if not isinstance(row, dict):
                continue
            width = _to_float(row.get("width"))
            if width is not None and width > 0.0:
                candidates.append(float(width))
        if candidates:
            return float(max(candidates))

    source_width_m = _to_float(mission.get("source_line_width_m"))
    if source_width_m is not None and source_width_m > 0.0:
        return float(source_width_m)

    candidates = []
    for row in mission.get("line_list") or []:
        if not isinstance(row, dict):
            continue
        width = _to_float(row.get("width"))
        if width is not None and width > 0.0:
            candidates.append(float(width))
    if candidates:
        return float(max(candidates))

    width_m = _to_float(mission.get("width_m"))
    return float(width_m) if width_m is not None and width_m > 0.0 else None


def _mission_source_line_rows(mission: dict[str, Any]) -> list[dict[str, Any]]:
    width_m = _mission_source_width_m(mission)
    rows: list[dict[str, Any]] = []
    for rows_key in ("line_list", "input_line_list"):
        for item in mission.get(rows_key) or []:
            if not isinstance(item, dict):
                continue
            coord_list = _coord_list(item.get("coordinateList"), min_len=2)
            if len(coord_list) < 2:
                continue
            row = {"coordinateList": coord_list}
            width = _to_float(item.get("width")) or width_m
            if width is not None and width > 0.0:
                row["width"] = float(width)
            rows.append(row)
        if rows:
            return rows

    line_hint = (
        bool(rows)
        or int(_to_int(mission.get("sweep_point_count")) or 0) > 0
        or bool(mission.get("sweep_line_coordinate_lists"))
        or bool(mission.get("input_line_list"))
        or bool(mission.get("line_list"))
    )
    if line_hint:
        for coord_key in ("coordinate_list", "input_coordinate_list"):
            coord_list = _coord_list(mission.get(coord_key), min_len=2)
            if len(coord_list) >= 2:
                row = {"coordinateList": coord_list}
                if width_m is not None and width_m > 0.0:
                    row["width"] = float(width_m)
                rows.append(row)
                return rows

    for coords in mission.get("sweep_line_coordinate_lists") or []:
        coord_list = _coord_list(coords, min_len=2)
        if len(coord_list) >= 2:
            row = {"coordinateList": coord_list}
            if width_m is not None and width_m > 0.0:
                row["width"] = float(width_m)
            rows.append(row)
    if rows:
        return rows
    return rows


def _mission_source_coordinate_list(
    mission: dict[str, Any],
    rows: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    for key in ("input_coordinate_list", "coordinate_list"):
        coords = _coord_list(mission.get(key), min_len=2)
        if len(coords) >= 2:
            return coords
    for rows_key in ("input_line_list", "line_list"):
        for row in mission.get(rows_key) or []:
            if not isinstance(row, dict):
                continue
            coords = _coord_list(row.get("coordinateList"), min_len=2)
            if len(coords) >= 2:
                return coords
    for row in rows or []:
        coords = _coord_list((row or {}).get("coordinateList"), min_len=2)
        if len(coords) >= 2:
            return coords
    return []


def _waypoint_ids(mission: dict[str, Any]) -> list[int]:
    ids: list[int] = []
    for wp in mission.get("waypoints") or []:
        if not isinstance(wp, dict):
            continue
        wid = _to_int(wp.get("waypoint_id") or wp.get("waypointID"))
        if wid is not None:
            ids.append(int(wid))
    if ids:
        return ids
    for raw in mission.get("waypoint_ids") or []:
        wid = _to_int(raw)
        if wid is not None:
            ids.append(int(wid))
    return ids


def _requires_filming_completion(mission: dict[str, Any]) -> bool:
    if int(_to_int(mission.get("sweep_point_count")) or 0) > 0:
        return True
    for wp in mission.get("waypoints") or []:
        if not isinstance(wp, dict):
            continue
        operation_mode = _to_int(
            wp.get("operation_mode")
            or wp.get("operationMode")
            or wp.get("OperationMode")
        )
        if operation_mode == 2:
            return True
        if bool(wp.get("has_line_search") or wp.get("hasLineSearch")):
            return True
        if int(_to_int(wp.get("line_search_point_count")) or 0) > 0:
            return True
    return False


def _merge_intervals(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    ordered = sorted((float(a), float(b)) for a, b in intervals if float(b) > float(a))
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


def _remaining_intervals(length_m: float, covered: list[tuple[float, float]]) -> list[tuple[float, float]]:
    length = max(0.0, float(length_m))
    if length <= 0.0:
        return []
    remaining: list[tuple[float, float]] = []
    cursor = 0.0
    for start, end in _merge_intervals(covered):
        start = max(0.0, min(float(start), length))
        end = max(0.0, min(float(end), length))
        if start > cursor + 1e-6:
            remaining.append((cursor, start))
        cursor = max(cursor, end)
    if cursor < length - 1e-6:
        remaining.append((cursor, length))
    return remaining


def _intervals_payload(intervals: list[tuple[float, float]]) -> list[dict[str, float]]:
    return [
        {
            "startM": _round_m(start),
            "endM": _round_m(end),
            "lengthM": _round_m(float(end) - float(start)),
        }
        for start, end in intervals
        if float(end) > float(start)
    ]


@dataclass
class _LineMissionRuntime:
    aircraft_id: int
    mission_id: int
    input_id: int | None
    path_id: int | None
    sweep_point_count: int
    line_def: LineSweepDefinition
    requires_filming_completion: bool
    is_current: bool = False
    source_line_rows: list[dict[str, Any]] = field(default_factory=list)
    source_line_width_m: float | None = None
    source_coordinate_list: list[dict[str, Any]] = field(default_factory=list)
    state: LineSweepState = field(default_factory=LineSweepState)
    last_observed_line_index: int | None = None
    last_line_delta_sign: int | None = None
    line_transition_count: int = 0
    line_direction_change_count: int = 0
    visited_line_indexes: set[int] = field(default_factory=set)
    line_visit_sequence: list[int] = field(default_factory=list)
    last_timestamp_ms: int | None = None
    carried_covered_length_m: float = 0.0
    carry_source_aircraft_ids: tuple[int, ...] = ()


@dataclass(frozen=True)
class _CommonCoverageCarry:
    reference_endpoints: tuple[tuple[float, float], tuple[float, float]]
    normalized_intervals: tuple[tuple[float, float], ...]
    source_aircraft_ids: tuple[int, ...]
    coordinate_space: str = "runtime_normalized"


def _runtime_endpoints(runtime: _LineMissionRuntime) -> tuple[tuple[float, float], tuple[float, float]] | None:
    lines = runtime.line_def.lines
    if not lines:
        return None
    try:
        start = lines[0].coords[0]
        end = lines[-1].coords[-1]
        return (float(start[0]), float(start[1])), (float(end[0]), float(end[1]))
    except Exception:
        return None


def _orientation_reversed(
    runtime: _LineMissionRuntime,
    reference_endpoints: tuple[tuple[float, float], tuple[float, float]],
) -> bool:
    endpoints = _runtime_endpoints(runtime)
    if endpoints is None:
        return False
    return _endpoints_reversed(endpoints, reference_endpoints)


def _endpoints_reversed(
    endpoints: tuple[tuple[float, float], tuple[float, float]],
    reference_endpoints: tuple[tuple[float, float], tuple[float, float]],
) -> bool:
    (start, end), (ref_start, ref_end) = endpoints, reference_endpoints
    same_cost = math.hypot(start[0] - ref_start[0], start[1] - ref_start[1]) + math.hypot(
        end[0] - ref_end[0], end[1] - ref_end[1]
    )
    reverse_cost = math.hypot(start[0] - ref_end[0], start[1] - ref_end[1]) + math.hypot(
        end[0] - ref_start[0], end[1] - ref_start[1]
    )
    return bool(reverse_cost + 1.0 < same_cost)


def _runtime_normalized_covered_intervals(
    runtime: _LineMissionRuntime,
    *,
    reference_endpoints: tuple[tuple[float, float], tuple[float, float]],
) -> list[tuple[float, float]]:
    total_m = float(runtime.line_def.planned_length_m or 0.0)
    if total_m <= 1e-6:
        return []
    intervals: list[tuple[float, float]] = []
    offset_m = 0.0
    for line_index, line_length_m in enumerate(runtime.line_def.line_lengths_m):
        length_m = max(0.0, float(line_length_m))
        for start_m, end_m in runtime.state.covered_intervals_by_line.get(int(line_index), []):
            start = max(0.0, min(length_m, float(start_m)))
            end = max(0.0, min(length_m, float(end_m)))
            if end > start + 1e-6:
                intervals.append(((offset_m + start) / total_m, (offset_m + end) / total_m))
        offset_m += length_m
    intervals = _merge_intervals(intervals)
    if _orientation_reversed(runtime, reference_endpoints):
        intervals = _merge_intervals([(1.0 - end, 1.0 - start) for start, end in intervals])
    return intervals


def _runtime_source_line(runtime: _LineMissionRuntime) -> LineString | None:
    coords = list(runtime.source_coordinate_list or [])
    if len(coords) < 2:
        for row in runtime.source_line_rows or []:
            coords = _coord_list((row or {}).get("coordinateList"), min_len=2)
            if len(coords) >= 2:
                break
    if len(coords) < 2:
        return None

    points: list[tuple[float, float]] = []
    for coord in coords:
        lat = _to_float(coord.get("latitude") if "latitude" in coord else coord.get("Latitude"))
        lon = _to_float(coord.get("longitude") if "longitude" in coord else coord.get("Longitude"))
        if lat is None or lon is None:
            continue
        try:
            x, y = runtime.line_def.transformer.transform(float(lon), float(lat))
        except Exception:
            continue
        point = (float(x), float(y))
        if points and math.hypot(point[0] - points[-1][0], point[1] - points[-1][1]) <= 0.01:
            continue
        points.append(point)
    if len(points) < 2:
        return None
    try:
        line = LineString(points)
    except Exception:
        return None
    return line if not line.is_empty and float(line.length or 0.0) > 1e-6 else None


def _runtime_source_normalized_covered_intervals(
    runtime: _LineMissionRuntime,
) -> list[tuple[float, float]] | None:
    source_line = _runtime_source_line(runtime)
    if source_line is None:
        return None
    source_length_m = float(source_line.length or 0.0)
    if source_length_m <= 1e-6:
        return None

    intervals: list[tuple[float, float]] = []
    for line_index, line in enumerate(runtime.line_def.lines):
        line_length_m = float(runtime.line_def.line_lengths_m[line_index])
        if line_length_m <= 1e-6:
            continue
        for start_m, end_m in runtime.state.covered_intervals_by_line.get(int(line_index), []):
            start = max(0.0, min(line_length_m, float(start_m)))
            end = max(0.0, min(line_length_m, float(end_m)))
            if end <= start + 1e-6:
                continue
            try:
                start_point = line.interpolate(float(start))
                end_point = line.interpolate(float(end))
                source_start = float(source_line.project(Point(start_point))) / source_length_m
                source_end = float(source_line.project(Point(end_point))) / source_length_m
            except Exception:
                continue
            interval_start = max(0.0, min(1.0, min(source_start, source_end)))
            interval_end = max(0.0, min(1.0, max(source_start, source_end)))
            if interval_end > interval_start + 1e-9:
                intervals.append((interval_start, interval_end))
    return _merge_intervals(intervals)


def _runtime_source_endpoints(
    runtime: _LineMissionRuntime,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    source_line = _runtime_source_line(runtime)
    if source_line is None:
        return None
    try:
        start = source_line.coords[0]
        end = source_line.coords[-1]
        return (float(start[0]), float(start[1])), (float(end[0]), float(end[1]))
    except Exception:
        return None


def _intersect_interval_sets(
    left: list[tuple[float, float]],
    right: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    left_rows = _merge_intervals(left)
    right_rows = _merge_intervals(right)
    result: list[tuple[float, float]] = []
    left_index = 0
    right_index = 0
    while left_index < len(left_rows) and right_index < len(right_rows):
        left_start, left_end = left_rows[left_index]
        right_start, right_end = right_rows[right_index]
        start = max(float(left_start), float(right_start))
        end = min(float(left_end), float(right_end))
        if end > start + 1e-9:
            result.append((start, end))
        if left_end <= right_end:
            left_index += 1
        else:
            right_index += 1
    return _merge_intervals(result)


def _common_input_coverage(
    runtimes: list[_LineMissionRuntime],
) -> _CommonCoverageCarry | None:
    # One aircraft can temporarily contain more than one LINE-shaped artifact
    # for the same input.  Only its current LINE is an active owner.  A retired
    # LINE must not re-enter the intersection while that aircraft is executing
    # an attack/return mission, otherwise its stale frontier can resurrect a
    # previously photographed prefix on the next plan transition.
    by_aircraft: dict[int, list[_LineMissionRuntime]] = {}
    for runtime in runtimes:
        if runtime.input_id is None or not runtime.line_def.lines:
            continue
        by_aircraft.setdefault(int(runtime.aircraft_id), []).append(runtime)
    usable: list[_LineMissionRuntime] = []
    for candidates in by_aircraft.values():
        current = [runtime for runtime in candidates if runtime.is_current]
        if not current:
            continue
        usable.append(
            max(
                current,
                key=lambda runtime: (
                    float(runtime.state.covered_length_m)
                    / max(float(runtime.line_def.planned_length_m), 1e-6),
                    int(runtime.mission_id),
                ),
            )
        )
    if not usable:
        return None
    reference_endpoints = _runtime_endpoints(usable[0])
    if reference_endpoints is None:
        return None
    source_covered_by_runtime = [
        _runtime_source_normalized_covered_intervals(runtime)
        for runtime in usable
    ]
    source_endpoints_by_runtime = [
        _runtime_source_endpoints(runtime)
        for runtime in usable
    ]
    use_source_coordinates = all(rows is not None for rows in source_covered_by_runtime) and all(
        endpoints is not None for endpoints in source_endpoints_by_runtime
    )
    source_reference_endpoints = (
        source_endpoints_by_runtime[0]
        if use_source_coordinates
        else None
    )
    common: list[tuple[float, float]] | None = None
    for runtime_index, runtime in enumerate(usable):
        if use_source_coordinates:
            covered = list(source_covered_by_runtime[runtime_index] or [])
            runtime_source_endpoints = source_endpoints_by_runtime[runtime_index]
            if (
                source_reference_endpoints is not None
                and runtime_source_endpoints is not None
                and _endpoints_reversed(runtime_source_endpoints, source_reference_endpoints)
            ):
                covered = _merge_intervals(
                    [(1.0 - end, 1.0 - start) for start, end in covered]
                )
        else:
            covered = _runtime_normalized_covered_intervals(
                runtime,
                reference_endpoints=reference_endpoints,
            )
        common = covered if common is None else _intersect_interval_sets(common, covered)
        if not common:
            break
    return _CommonCoverageCarry(
        reference_endpoints=(
            source_reference_endpoints
            if source_reference_endpoints is not None
            else reference_endpoints
        ),
        normalized_intervals=tuple(common or []),
        source_aircraft_ids=tuple(sorted({int(runtime.aircraft_id) for runtime in usable})),
        coordinate_space=("source_normalized" if use_source_coordinates else "runtime_normalized"),
    )


def _seed_runtime_source_coverage(
    runtime: _LineMissionRuntime,
    normalized_intervals: list[tuple[float, float]],
) -> float | None:
    source_line = _runtime_source_line(runtime)
    if source_line is None:
        return None
    source_length_m = float(source_line.length or 0.0)
    if source_length_m <= 1e-6:
        return None

    covered_length_m = 0.0
    for line_index, line in enumerate(runtime.line_def.lines):
        line_length_m = float(runtime.line_def.line_lengths_m[line_index])
        if line_length_m <= 1e-6:
            continue
        try:
            source_start = float(source_line.project(Point(line.coords[0]))) / source_length_m
            source_end = float(source_line.project(Point(line.coords[-1]))) / source_length_m
        except Exception:
            continue
        source_delta = float(source_end) - float(source_start)
        if abs(source_delta) <= 1e-9:
            continue
        domain_start = min(float(source_start), float(source_end))
        domain_end = max(float(source_start), float(source_end))
        line_intervals: list[tuple[float, float]] = []
        for interval_start, interval_end in normalized_intervals:
            overlap_start = max(float(interval_start), domain_start)
            overlap_end = min(float(interval_end), domain_end)
            if overlap_end <= overlap_start + 1e-9:
                continue
            line_start = ((overlap_start - float(source_start)) / source_delta) * line_length_m
            line_end = ((overlap_end - float(source_start)) / source_delta) * line_length_m
            mapped_start = max(0.0, min(line_length_m, min(line_start, line_end)))
            mapped_end = max(0.0, min(line_length_m, max(line_start, line_end)))
            if mapped_end > mapped_start + 1e-6:
                line_intervals.append((mapped_start, mapped_end))
        merged = _merge_intervals(line_intervals)
        if merged:
            runtime.state.covered_intervals_by_line[int(line_index)] = merged
            runtime.visited_line_indexes.add(int(line_index))
            covered_length_m += sum(max(0.0, end - start) for start, end in merged)
    runtime.state.covered_length_m = min(
        float(runtime.line_def.planned_length_m or 0.0),
        float(covered_length_m),
    )
    return float(runtime.state.covered_length_m)


def _seed_runtime_common_coverage(
    runtime: _LineMissionRuntime,
    carry: _CommonCoverageCarry | None,
) -> None:
    if carry is None:
        return
    reference_endpoints = carry.reference_endpoints
    common_intervals = list(carry.normalized_intervals)
    source_aircraft_ids = carry.source_aircraft_ids
    if not common_intervals:
        return
    if carry.coordinate_space == "source_normalized":
        source_intervals = list(common_intervals)
        runtime_source_endpoints = _runtime_source_endpoints(runtime)
        if (
            runtime_source_endpoints is not None
            and _endpoints_reversed(runtime_source_endpoints, reference_endpoints)
        ):
            source_intervals = _merge_intervals(
                [(1.0 - end, 1.0 - start) for start, end in source_intervals]
            )
        carried_length_m = _seed_runtime_source_coverage(runtime, source_intervals)
        if carried_length_m is not None:
            runtime.carried_covered_length_m = float(carried_length_m)
            runtime.carry_source_aircraft_ids = tuple(source_aircraft_ids)
            return
    intervals = list(common_intervals)
    if _orientation_reversed(runtime, reference_endpoints):
        intervals = _merge_intervals([(1.0 - end, 1.0 - start) for start, end in intervals])
    total_m = float(runtime.line_def.planned_length_m or 0.0)
    if total_m <= 1e-6:
        return
    absolute_intervals = [(start * total_m, end * total_m) for start, end in intervals]
    offset_m = 0.0
    covered_length_m = 0.0
    for line_index, line_length_m in enumerate(runtime.line_def.line_lengths_m):
        length_m = max(0.0, float(line_length_m))
        line_start_m = offset_m
        line_end_m = offset_m + length_m
        line_intervals: list[tuple[float, float]] = []
        for start_m, end_m in absolute_intervals:
            start = max(float(start_m), line_start_m)
            end = min(float(end_m), line_end_m)
            if end <= start + 1e-6:
                continue
            line_intervals.append((start - line_start_m, end - line_start_m))
        merged = _merge_intervals(line_intervals)
        if merged:
            runtime.state.covered_intervals_by_line[int(line_index)] = merged
            runtime.visited_line_indexes.add(int(line_index))
            covered_length_m += sum(max(0.0, end - start) for start, end in merged)
        offset_m = line_end_m
    runtime.state.covered_length_m = min(total_m, float(covered_length_m))
    runtime.carried_covered_length_m = float(runtime.state.covered_length_m)
    runtime.carry_source_aircraft_ids = tuple(source_aircraft_ids)


class LineScanProgressMonitor:
    """Line-only scan monitor for replan progress, independent from visualization UI."""

    def __init__(
        self,
        *,
        persist_interval_ms: int = 500,
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._persist_interval_sec = max(0.0, float(persist_interval_ms) / 1000.0)
        self._logger = logger
        self._mission_plan_id: int | None = None
        self._missions: dict[int, _LineMissionRuntime] = {}
        self._aircraft_current_mission: dict[int, int | None] = {}
        self._waypoint_to_mission: dict[int, dict[int, int]] = {}
        self._last_snapshot_signature: str | None = None
        self._last_persist_monotonic = 0.0

    def apply_mission_plan(self, mission_plan_id: int | None) -> None:
        previous_by_input: dict[int, list[_LineMissionRuntime]] = {}
        for runtime in self._missions.values():
            if runtime.input_id is None:
                continue
            previous_by_input.setdefault(int(runtime.input_id), []).append(runtime)
        carry_by_input = {
            int(input_id): carry
            for input_id, runtimes in previous_by_input.items()
            if (carry := _common_input_coverage(runtimes)) is not None
        }
        plan_id = _to_int(mission_plan_id)
        self._mission_plan_id = int(plan_id) if plan_id is not None else None
        self._missions = {}
        self._aircraft_current_mission = {}
        self._waypoint_to_mission = {}
        view = build_uav_mission_view(self._mission_plan_id)
        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aid = _to_int(entry.get("aircraft_id"))
            if aid is None:
                continue
            self._aircraft_current_mission[int(aid)] = _to_int(
                entry.get("current_individual_mission_id")
            )
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                mission_id = _to_int(mission.get("individual_mission_id"))
                if mission_id is None:
                    continue
                line_def = build_line_sweep_definition(mission, coverage_def=None)
                if line_def is None:
                    continue
                source_line_rows = _mission_source_line_rows(mission)
                source_width_m = _mission_source_width_m(mission)
                runtime = _LineMissionRuntime(
                    aircraft_id=int(aid),
                    mission_id=int(mission_id),
                    input_id=_to_int(mission.get("input_id")),
                    path_id=_to_int(mission.get("path_id")),
                    sweep_point_count=max(0, int(_to_int(mission.get("sweep_point_count")) or 0)),
                    line_def=line_def,
                    requires_filming_completion=_requires_filming_completion(mission),
                    is_current=bool(
                        self._aircraft_current_mission.get(int(aid)) == int(mission_id)
                    ),
                    source_line_rows=source_line_rows,
                    source_line_width_m=source_width_m,
                    source_coordinate_list=_mission_source_coordinate_list(mission, source_line_rows),
                )
                if runtime.input_id is not None:
                    _seed_runtime_common_coverage(
                        runtime,
                        carry_by_input.get(int(runtime.input_id)),
                    )
                if bool(mission.get("is_done")):
                    force_complete_line_sweep_state(line_def, runtime.state)
                    runtime.visited_line_indexes.update(range(len(line_def.lines)))
                self._missions[int(mission_id)] = runtime
                for wid in _waypoint_ids(mission):
                    self._waypoint_to_mission.setdefault(int(aid), {})[int(wid)] = int(mission_id)
        self._persist(force=True)

    def reset_input_coverage(self, input_mission_id: int | None) -> int:
        input_id = _to_int(input_mission_id)
        if input_id is None or input_id <= 0:
            return 0
        reset_count = 0
        for runtime in self._missions.values():
            if runtime.input_id is None or int(runtime.input_id) != int(input_id):
                continue
            reset_line_sweep_state(runtime.state)
            runtime.last_observed_line_index = None
            runtime.last_line_delta_sign = None
            runtime.line_transition_count = 0
            runtime.line_direction_change_count = 0
            runtime.visited_line_indexes.clear()
            runtime.line_visit_sequence.clear()
            runtime.last_timestamp_ms = None
            runtime.carried_covered_length_m = 0.0
            runtime.carry_source_aircraft_ids = ()
            reset_count += 1
        if reset_count:
            self._persist(force=True)
        return int(reset_count)

    def update_agent_status(
        self,
        *,
        timestamp_ms: int | None,
        agent_states: list[dict[str, Any]],
    ) -> None:
        if not self._missions:
            return
        changed = False
        for row in agent_states or []:
            if not isinstance(row, dict):
                continue
            if self._update_single_agent(timestamp_ms=timestamp_ms, row=row):
                changed = True
        if changed:
            self._persist(force=False)

    def _resolve_mission_id(self, aircraft_id: int, current_wp: int | None, flying: int | None) -> int | None:
        if flying == 2:
            current = self._aircraft_current_mission.get(int(aircraft_id))
            if current in self._missions:
                return int(current)
        if current_wp is not None:
            mission_id = self._waypoint_to_mission.get(int(aircraft_id), {}).get(int(current_wp))
            if mission_id in self._missions:
                return int(mission_id)
            # An explicit waypoint that does not belong to a LINE is positive
            # evidence that this aircraft moved on to attack/return/another
            # mission.  Never fall back to its stale LINE runtime.
            return None
        current = self._aircraft_current_mission.get(int(aircraft_id))
        if current in self._missions:
            return int(current)
        # A non-LINE current mission must not update an arbitrary pending LINE
        # owned by the same aircraft.  The waypoint map will select it once
        # the aircraft actually enters that LINE path.
        return None

    def _set_aircraft_current_line(self, aircraft_id: int, mission_id: int | None) -> bool:
        aid = int(aircraft_id)
        resolved = int(mission_id) if mission_id in self._missions else None
        changed = self._aircraft_current_mission.get(aid) != resolved
        self._aircraft_current_mission[aid] = resolved
        for runtime in self._missions.values():
            if int(runtime.aircraft_id) != aid:
                continue
            next_current = bool(
                resolved is not None and int(runtime.mission_id) == int(resolved)
            )
            changed = bool(changed or runtime.is_current != next_current)
            runtime.is_current = next_current
        return bool(changed)

    def _update_single_agent(self, *, timestamp_ms: int | None, row: dict[str, Any]) -> bool:
        aid = _to_int(row.get("aircraft_id"))
        if aid is None:
            return False
        current_wp = _to_int(row.get("current_waypoint_id"))
        flying = _to_int(row.get("flying"))
        filming = _to_int(row.get("filming"))
        mission_id = self._resolve_mission_id(int(aid), current_wp, flying)
        if mission_id is None:
            if current_wp is not None:
                return self._set_aircraft_current_line(int(aid), None)
            return False
        runtime = self._missions.get(int(mission_id))
        if runtime is None:
            return False
        current_changed = self._set_aircraft_current_line(int(aid), int(mission_id))
        before = float(runtime.state.covered_length_m)
        if flying == 2 and (not runtime.requires_filming_completion or filming == 2):
            force_complete_line_sweep_state(runtime.line_def, runtime.state)
            runtime.visited_line_indexes.update(range(len(runtime.line_def.lines)))
            runtime.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else runtime.last_timestamp_ms
            return bool(current_changed or runtime.state.covered_length_m > before + 1e-6)
        update_line_sweep_state(runtime.line_def, runtime.state, row, timestamp_ms=timestamp_ms)
        self._record_line_movement(runtime, timestamp_ms=timestamp_ms)
        return bool(current_changed or runtime.state.covered_length_m > before + 1e-6)

    def _record_line_movement(self, runtime: _LineMissionRuntime, *, timestamp_ms: int | None) -> None:
        current_line = runtime.state.last_line_index
        if current_line is None:
            return
        current_line = int(current_line)
        runtime.visited_line_indexes.add(current_line)
        if not runtime.line_visit_sequence or runtime.line_visit_sequence[-1] != current_line:
            runtime.line_visit_sequence.append(current_line)
            runtime.line_visit_sequence = runtime.line_visit_sequence[-128:]
        previous = runtime.last_observed_line_index
        if previous is not None and int(previous) != current_line:
            runtime.line_transition_count += 1
            delta = current_line - int(previous)
            sign = 1 if delta > 0 else -1 if delta < 0 else 0
            if sign and runtime.last_line_delta_sign is not None and sign != runtime.last_line_delta_sign:
                runtime.line_direction_change_count += 1
            if sign:
                runtime.last_line_delta_sign = int(sign)
        runtime.last_observed_line_index = current_line
        runtime.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else runtime.last_timestamp_ms

    def snapshot(self) -> dict[str, Any]:
        entries = [self._mission_payload(runtime) for runtime in self._missions.values()]
        entries.sort(key=lambda item: (int(item.get("aircraftID") or 0), int(item.get("missionID") or 0)))
        return {
            "schemaVersion": 1,
            "source": "line_scan_progress_monitor",
            "missionPlanID": self._mission_plan_id,
            "entryCount": len(entries),
            "entries": entries,
        }

    def _mission_payload(self, runtime: _LineMissionRuntime) -> dict[str, Any]:
        covered_m, planned_m, percent, enabled = line_sweep_metrics(runtime.line_def, runtime.state)
        line_rows: list[dict[str, Any]] = []
        completed_lines = 0
        for idx, length_m in enumerate(runtime.line_def.line_lengths_m):
            covered = runtime.state.covered_intervals_by_line.get(int(idx), [])
            remaining = _remaining_intervals(float(length_m), covered)
            covered_len = sum(max(0.0, float(end) - float(start)) for start, end in _merge_intervals(covered))
            ratio = covered_len / max(float(length_m), 1e-6)
            if ratio >= 0.95:
                completed_lines += 1
            line_rows.append(
                {
                    "lineIndex": int(idx),
                    "plannedLengthM": _round_m(length_m),
                    "coveredLengthM": _round_m(covered_len),
                    "remainingLengthM": _round_m(max(0.0, float(length_m) - covered_len)),
                    "coveragePercent": max(0, min(100, int(round(ratio * 100.0)))),
                    "coveredIntervals": _intervals_payload(_merge_intervals(covered)),
                    "remainingIntervals": _intervals_payload(remaining),
                }
            )
            source_row = runtime.source_line_rows[idx] if idx < len(runtime.source_line_rows) else {}
            source_coords = _coord_list((source_row or {}).get("coordinateList"), min_len=2)
            if len(source_coords) >= 2:
                line_rows[-1]["coordinateList"] = deepcopy(source_coords)
            source_width = _to_float((source_row or {}).get("width")) or runtime.source_line_width_m
            if source_width is not None and source_width > 0.0:
                line_rows[-1]["width"] = _round_m(source_width)
        sweep_points = max(0, int(runtime.sweep_point_count or 0))
        progress_points = int(round((max(0, min(100, int(percent))) / 100.0) * sweep_points)) if sweep_points else 0
        payload = {
            "missionPlanID": self._mission_plan_id,
            "timestampMs": runtime.last_timestamp_ms,
            "aircraftID": int(runtime.aircraft_id),
            "missionID": int(runtime.mission_id),
            "isCurrent": bool(runtime.is_current),
            "inputMissionID": int(runtime.input_id) if runtime.input_id is not None else None,
            "pathID": int(runtime.path_id) if runtime.path_id is not None else None,
            "enabled": bool(enabled),
            "progressPercent": int(percent),
            "plannedLengthM": _round_m(planned_m),
            "coveredLengthM": _round_m(covered_m),
            "remainingLengthM": _round_m(max(0.0, float(planned_m) - float(covered_m))),
            "lineCount": len(runtime.line_def.lines),
            "visitedLineCount": len(runtime.visited_line_indexes),
            "completedLineCount": int(completed_lines),
            "remainingLineCount": max(0, len(runtime.line_def.lines) - int(completed_lines)),
            "currentLineIndex": runtime.state.last_line_index,
            "lineTransitionCount": int(runtime.line_transition_count),
            "lineDirectionChangeCount": int(runtime.line_direction_change_count),
            "lineVisitSequence": list(runtime.line_visit_sequence[-32:]),
            "sweepPointCount": int(sweep_points),
            "progressPoints": int(progress_points),
            "bufferPoints": int(progress_points),
            "lineList": line_rows,
        }
        if runtime.source_line_width_m is not None and runtime.source_line_width_m > 0.0:
            payload["sourceLineWidthM"] = _round_m(runtime.source_line_width_m)
        if len(runtime.source_coordinate_list) >= 2:
            payload["sourceCoordinateList"] = deepcopy(runtime.source_coordinate_list)
        if runtime.source_line_rows:
            payload["sourceLineList"] = deepcopy(runtime.source_line_rows)
        if runtime.carried_covered_length_m > 1e-6:
            payload["carriedCoveredLengthM"] = _round_m(runtime.carried_covered_length_m)
            payload["coverageCarryPolicy"] = "previous_active_owner_intersection"
            payload["coverageCarrySourceAircraftIDs"] = list(runtime.carry_source_aircraft_ids)
        return payload

    def _persist(self, *, force: bool) -> None:
        now = time.monotonic()
        if not force and self._persist_interval_sec > 0.0:
            if now - self._last_persist_monotonic < self._persist_interval_sec:
                return
        payload = self.snapshot()
        try:
            signature = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        except Exception:
            return
        if not force and signature == self._last_snapshot_signature:
            return
        try:
            base = db_paths.get_db_subpath("DSS_Internal")
            base.mkdir(parents=True, exist_ok=True)
            (base / "line_scan_progress.json").write_text(signature, encoding="utf-8")
        except Exception as exc:
            if self._logger:
                self._logger(f"[LINE] progress persist failed: {exc}")
            return
        self._last_snapshot_signature = signature
        self._last_persist_monotonic = now


class LineScanProgressWorker:
    """Latest-only background worker so high-rate 0401 never blocks the UI thread."""

    def __init__(
        self,
        *,
        logger: Callable[[str], None] | None = None,
        min_update_interval_ms: int = 200,
        persist_interval_ms: int = 500,
    ) -> None:
        self._logger = logger
        self._min_update_interval_sec = max(0.02, float(min_update_interval_ms) / 1000.0)
        self._persist_interval_ms = max(0, int(persist_interval_ms))
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stop = threading.Event()
        self._plan_jobs: list[int | None] = []
        self._reset_input_jobs: list[int] = []
        self._latest_status: tuple[int | None, list[dict[str, Any]]] | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="MSM-LineScanProgress", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

    def apply_mission_plan(self, mission_plan_id: int | None) -> None:
        plan_id = _to_int(mission_plan_id)
        with self._lock:
            self._plan_jobs.append(int(plan_id) if plan_id is not None else None)
            self._plan_jobs = self._plan_jobs[-8:]
        self._event.set()

    def reset_input_coverage(self, input_mission_id: int | None) -> None:
        input_id = _to_int(input_mission_id)
        if input_id is None or input_id <= 0:
            return
        with self._lock:
            self._reset_input_jobs.append(int(input_id))
            self._reset_input_jobs = self._reset_input_jobs[-16:]
        self._event.set()

    def submit_agent_status(
        self,
        *,
        timestamp_ms: int | None,
        agent_states: list[dict[str, Any]],
    ) -> None:
        rows: list[dict[str, Any]] = []
        keep_keys = {
            "aircraft_id",
            "current_waypoint_id",
            "flying",
            "filming",
            "sensor_operation_mode",
            "sensor_center_coordinate",
            "coordinate",
            "footprint_corners",
        }
        for item in agent_states or []:
            if not isinstance(item, dict):
                continue
            rows.append({key: item.get(key) for key in keep_keys if key in item})
        ts = _to_int(timestamp_ms)
        with self._lock:
            self._latest_status = (int(ts) if ts is not None else None, rows)
        self._event.set()

    def _run(self) -> None:
        monitor = LineScanProgressMonitor(
            persist_interval_ms=self._persist_interval_ms,
            logger=self._logger,
        )
        last_update = 0.0
        while not self._stop.is_set():
            self._event.wait(0.5)
            self._event.clear()
            while not self._stop.is_set():
                with self._lock:
                    plan_jobs = list(self._plan_jobs)
                    self._plan_jobs.clear()
                    reset_input_jobs = list(self._reset_input_jobs)
                    self._reset_input_jobs.clear()
                    latest = self._latest_status
                    self._latest_status = None
                if not plan_jobs and not reset_input_jobs and latest is None:
                    break
                for input_id in reset_input_jobs:
                    try:
                        monitor.reset_input_coverage(input_id)
                    except Exception as exc:
                        if self._logger:
                            self._logger(f"[LINE] input coverage reset failed: {exc}")
                for plan_id in plan_jobs:
                    try:
                        monitor.apply_mission_plan(plan_id)
                    except Exception as exc:
                        if self._logger:
                            self._logger(f"[LINE] plan update failed: {exc}")
                if latest is None:
                    continue
                now = time.monotonic()
                wait_sec = self._min_update_interval_sec - (now - last_update)
                if wait_sec > 0.0:
                    self._stop.wait(wait_sec)
                    with self._lock:
                        newer = self._latest_status
                        self._latest_status = None
                    if newer is not None:
                        latest = newer
                try:
                    ts, rows = latest
                    monitor.update_agent_status(timestamp_ms=ts, agent_states=rows)
                    last_update = time.monotonic()
                except Exception as exc:
                    if self._logger:
                        self._logger(f"[LINE] 0401 update failed: {exc}")
