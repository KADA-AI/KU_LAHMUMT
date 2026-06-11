# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import threading
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Callable

from modules.common import db_paths
from modules.monitoring.logic.mission_line_progress import (
    LineSweepDefinition,
    LineSweepState,
    build_line_sweep_definition,
    force_complete_line_sweep_state,
    line_sweep_metrics,
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
    candidates: list[float] = []
    for key in ("width_m", "sep_m"):
        value = _to_float(mission.get(key))
        if value is not None and value > 0.0:
            candidates.append(float(value))
    for rows_key in ("line_list", "input_line_list"):
        for row in mission.get(rows_key) or []:
            if not isinstance(row, dict):
                continue
            width = _to_float(row.get("width"))
            if width is not None and width > 0.0:
                candidates.append(float(width))
    if not candidates:
        return None
    return float(max(candidates))


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
                    source_line_rows=source_line_rows,
                    source_line_width_m=source_width_m,
                    source_coordinate_list=_mission_source_coordinate_list(mission, source_line_rows),
                )
                if bool(mission.get("is_done")):
                    force_complete_line_sweep_state(line_def, runtime.state)
                    runtime.visited_line_indexes.update(range(len(line_def.lines)))
                self._missions[int(mission_id)] = runtime
                for wid in _waypoint_ids(mission):
                    self._waypoint_to_mission.setdefault(int(aid), {})[int(wid)] = int(mission_id)
        self._persist(force=True)

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
        current = self._aircraft_current_mission.get(int(aircraft_id))
        if current in self._missions:
            return int(current)
        for mission_id, runtime in self._missions.items():
            if int(runtime.aircraft_id) == int(aircraft_id):
                return int(mission_id)
        return None

    def _update_single_agent(self, *, timestamp_ms: int | None, row: dict[str, Any]) -> bool:
        aid = _to_int(row.get("aircraft_id"))
        if aid is None:
            return False
        current_wp = _to_int(row.get("current_waypoint_id"))
        flying = _to_int(row.get("flying"))
        filming = _to_int(row.get("filming"))
        mission_id = self._resolve_mission_id(int(aid), current_wp, flying)
        if mission_id is None:
            return False
        runtime = self._missions.get(int(mission_id))
        if runtime is None:
            return False
        self._aircraft_current_mission[int(aid)] = int(mission_id)
        before = float(runtime.state.covered_length_m)
        if flying == 2 and (not runtime.requires_filming_completion or filming == 2):
            force_complete_line_sweep_state(runtime.line_def, runtime.state)
            runtime.visited_line_indexes.update(range(len(runtime.line_def.lines)))
            runtime.last_timestamp_ms = int(timestamp_ms) if timestamp_ms is not None else runtime.last_timestamp_ms
            return runtime.state.covered_length_m > before + 1e-6
        update_line_sweep_state(runtime.line_def, runtime.state, row, timestamp_ms=timestamp_ms)
        self._record_line_movement(runtime, timestamp_ms=timestamp_ms)
        return runtime.state.covered_length_m > before + 1e-6

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
            "sensor_center_coordinate",
            "coordinate",
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
                    latest = self._latest_status
                    self._latest_status = None
                if not plan_jobs and latest is None:
                    break
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
