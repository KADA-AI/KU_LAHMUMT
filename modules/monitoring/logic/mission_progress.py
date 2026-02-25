# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

_ON_MISSION_STARTUP_GUARD_MS = 10000


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_float(value: object) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _derive_cumulative_etas(raw_etas: list[float]) -> tuple[list[float], bool]:
    """Return cumulative ETAs and whether the raw values looked cumulative."""
    if not raw_etas:
        return [], False
    values = [max(0.0, float(v)) for v in raw_etas]
    eps = 1e-6
    is_non_decreasing = all(values[idx] + eps >= values[idx - 1] for idx in range(1, len(values)))
    starts_at_zero = values[0] <= eps
    if starts_at_zero and is_non_decreasing:
        return values, True
    cumulative: list[float] = []
    total = 0.0
    for v in values:
        total += v
        cumulative.append(total)
    return cumulative, False


@dataclass
class MissionMeta:
    mission_id: int
    aircraft_id: int
    input_id: int | None
    package_id: int | None
    path_id: int | None
    planned_seconds: float
    waypoint_ids: list[int]
    waypoint_eta_cumulative: dict[int, float]
    waypoint_index: dict[int, int]


@dataclass
class MissionProgressState:
    completed_seconds: float = 0.0
    current_waypoint_id: int | None = None
    segment_start_ms: int | None = None
    done: bool = False
    paused: bool = False
    awaiting_execute: bool = False
    elapsed_seconds: float = 0.0
    last_update_ms: int | None = None


class MissionProgressTracker:
    def __init__(self) -> None:
        self._system_mode_code: int | None = None
        self._on_mission_startup_guard_requested: bool = False
        self._on_mission_startup_guard_pending: set[int] = set()
        self._on_mission_startup_guard_first_wp: dict[int, int | None] = {}
        self._on_mission_startup_guard_baselined: set[int] = set()
        self._on_mission_startup_guard_start_ms: dict[int, int] = {}
        self.reset({})

    def set_system_mode(self, mode_code: int | None) -> None:
        mode = _coerce_int(mode_code)
        prev = self._system_mode_code
        self._system_mode_code = mode
        if mode == 3:
            if prev != 3:
                self._on_mission_startup_guard_requested = True
                self._arm_on_mission_startup_guard()
                if self._on_mission_startup_guard_pending:
                    self._on_mission_startup_guard_requested = False
            return
        self._on_mission_startup_guard_requested = False
        self._clear_on_mission_startup_guard()

    def reset(self, view: dict[str, Any] | None) -> None:
        self._mission_meta: dict[int, MissionMeta] = {}
        self._progress_state: dict[int, MissionProgressState] = {}
        self._aircraft_missions: dict[int, list[int]] = {}
        self._waypoint_to_mission: dict[int, dict[int, int]] = {}
        self._input_to_missions: dict[int, list[int]] = {}
        self._input_mission_ids: list[int] = []
        self._aircraft_current_mission: dict[int, int | None] = {}
        self._mission_to_package: dict[int, int | None] = {}
        self._last_completed_idx: dict[int, int] = {}
        self._completed_mission_ids: set[int] = set()
        self._completed_input_ids: set[int] = set()
        self._last_timestamp_ms: int | None = None
        self._paused_aircraft: set[int] = set()
        self._aircraft_hold_mission: dict[int, int] = {}
        self._formation_followers: dict[int, dict[str, int | None]] = {}
        self._formation_followers_map: dict[int, int | None] = {}
        self._leader_mission_by_aircraft_input: dict[tuple[int, int], int] = {}
        self._waypoint_state: dict[int, dict[int, str]] = {}

        if not view:
            return

        input_missions = view.get("input_missions") or []
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            input_id = _coerce_int(item.get("input_mission_id"))
            if input_id is None:
                continue
            self._input_mission_ids.append(input_id)
            if item.get("is_done"):
                self._completed_input_ids.add(input_id)

        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = _coerce_int(entry.get("aircraft_id"))
            if aircraft_id is None:
                continue
            package_id = _coerce_int(entry.get("individual_mission_package_id"))
            self._aircraft_missions.setdefault(aircraft_id, [])
            self._aircraft_current_mission[aircraft_id] = _coerce_int(
                entry.get("current_individual_mission_id")
            )
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                mission_id = _coerce_int(mission.get("individual_mission_id"))
                if mission_id is None:
                    continue
                input_id = _coerce_int(mission.get("input_id"))
                formation_leader_id = _coerce_int(mission.get("formation_leader_id"))
                is_formation_follower = bool(mission.get("skip_progress"))
                if not is_formation_follower:
                    if formation_leader_id is not None and int(formation_leader_id) != int(aircraft_id):
                        is_formation_follower = True
                path_id = _coerce_int(mission.get("path_id"))
                planned_seconds = _coerce_float(mission.get("eta_seconds")) or 0.0
                waypoint_ids: list[int] = []
                raw_etas: list[float] = []
                provided_cumulative: list[float | None] = []
                for wp in mission.get("waypoints") or []:
                    if not isinstance(wp, dict):
                        continue
                    wid = _coerce_int(wp.get("waypoint_id") or wp.get("waypointID"))
                    if wid is None:
                        continue
                    waypoint_ids.append(wid)
                    eta = _coerce_float(wp.get("eta"))
                    raw_etas.append(float(eta) if eta is not None else 0.0)
                    cum_eta = _coerce_float(wp.get("eta_cumulative"))
                    provided_cumulative.append(float(cum_eta) if cum_eta is not None else None)
                if not waypoint_ids:
                    for wid in mission.get("waypoint_ids") or []:
                        wid_int = _coerce_int(wid)
                        if wid_int is None:
                            continue
                        waypoint_ids.append(wid_int)
                        raw_etas.append(0.0)
                        provided_cumulative.append(None)

                cumulative_etas: list[float] = []
                if waypoint_ids:
                    if provided_cumulative and all(val is not None for val in provided_cumulative):
                        cumulative_etas = [max(0.0, float(val)) for val in provided_cumulative]  # type: ignore[arg-type]
                    else:
                        derived, _used_cumulative = _derive_cumulative_etas(raw_etas)
                        cumulative_etas = list(derived) if derived else [0.0] * len(waypoint_ids)
                        if len(cumulative_etas) < len(waypoint_ids):
                            pad = cumulative_etas[-1] if cumulative_etas else 0.0
                            cumulative_etas.extend([pad] * (len(waypoint_ids) - len(cumulative_etas)))
                        for idx, val in enumerate(provided_cumulative):
                            if val is None:
                                continue
                            if idx < len(cumulative_etas):
                                cumulative_etas[idx] = max(cumulative_etas[idx], max(0.0, float(val)))

                    for idx in range(1, len(cumulative_etas)):
                        if cumulative_etas[idx] < cumulative_etas[idx - 1]:
                            cumulative_etas[idx] = cumulative_etas[idx - 1]

                last_cumulative = cumulative_etas[-1] if cumulative_etas else 0.0
                total_seconds = max(planned_seconds, last_cumulative, 0.0)
                if waypoint_ids and total_seconds > 0:
                    if not cumulative_etas:
                        denom = max(1, len(waypoint_ids) - 1)
                        cumulative_etas = [
                            float(total_seconds) * (idx / denom) for idx in range(len(waypoint_ids))
                        ]
                    else:
                        last_cumulative = cumulative_etas[-1]
                        if last_cumulative <= 0:
                            denom = max(1, len(waypoint_ids) - 1)
                            cumulative_etas = [
                                float(total_seconds) * (idx / denom) for idx in range(len(waypoint_ids))
                            ]
                        elif abs(total_seconds - last_cumulative) / max(total_seconds, 1.0) > 0.01:
                            scale = float(total_seconds) / float(last_cumulative)
                            cumulative_etas = [float(val) * scale for val in cumulative_etas]
                    if cumulative_etas:
                        cumulative_etas[-1] = float(total_seconds)
                planned_seconds = float(total_seconds)

                waypoint_eta_cumulative: dict[int, float] = {}
                waypoint_index: dict[int, int] = {}
                for idx, wid in enumerate(waypoint_ids):
                    cum_val = cumulative_etas[idx] if idx < len(cumulative_etas) else 0.0
                    waypoint_eta_cumulative[int(wid)] = float(max(0.0, cum_val))
                    waypoint_index[int(wid)] = int(idx)
                meta = MissionMeta(
                    mission_id=mission_id,
                    aircraft_id=aircraft_id,
                    input_id=input_id,
                    package_id=package_id,
                    path_id=path_id,
                    planned_seconds=planned_seconds,
                    waypoint_ids=waypoint_ids,
                    waypoint_eta_cumulative=waypoint_eta_cumulative,
                    waypoint_index=waypoint_index,
                )
                self._mission_meta[mission_id] = meta
                self._mission_to_package[mission_id] = package_id
                self._last_completed_idx.setdefault(mission_id, -1)
                self._waypoint_state[mission_id] = {
                    int(wid): "pending" for wid in waypoint_ids
                }
                self._aircraft_missions[aircraft_id].append(mission_id)
                if input_id is not None:
                    self._input_to_missions.setdefault(input_id, []).append(mission_id)
                if is_formation_follower:
                    if formation_leader_id is not None:
                        self._formation_followers[mission_id] = {
                            "leader_aircraft_id": int(formation_leader_id),
                            "input_id": int(input_id) if input_id is not None else None,
                        }
                else:
                    if input_id is not None:
                        self._leader_mission_by_aircraft_input[(aircraft_id, int(input_id))] = mission_id
                for wid in waypoint_ids:
                    self._waypoint_to_mission.setdefault(aircraft_id, {})
                    self._waypoint_to_mission[aircraft_id].setdefault(wid, mission_id)
                if mission.get("is_done"):
                    self._progress_state[mission_id] = MissionProgressState(
                        completed_seconds=float(planned_seconds),
                        done=True,
                        elapsed_seconds=float(planned_seconds),
                    )
                    self._completed_mission_ids.add(mission_id)
                    if waypoint_ids:
                        self._last_completed_idx[mission_id] = len(waypoint_ids) - 1
                else:
                    self._progress_state.setdefault(mission_id, MissionProgressState())

        if self._formation_followers:
            for follower_id, info in self._formation_followers.items():
                leader_id = None
                leader_aircraft = info.get("leader_aircraft_id")
                input_id = info.get("input_id")
                if leader_aircraft is not None and input_id is not None:
                    leader_id = self._leader_mission_by_aircraft_input.get(
                        (int(leader_aircraft), int(input_id))
                    )
                self._formation_followers_map[follower_id] = leader_id

        if self._system_mode_code == 3:
            # Re-arm guard on every mission-view reset (e.g. 0702 ignore=2 plan switch)
            # so stale/transient onMission=2 does not immediately trigger execute-ready flow.
            self._arm_on_mission_startup_guard()
            self._on_mission_startup_guard_requested = False
        else:
            self._on_mission_startup_guard_requested = False
            self._clear_on_mission_startup_guard()

    def update(
        self,
        timestamp_ms: int | None,
        agent_states: list[dict[str, Any]] | None,
    ) -> dict[str, Any]:
        new_completed_individual: list[dict[str, int | None]] = []
        new_completed_waypoints: list[dict[str, Any]] = []
        if timestamp_ms is not None:
            self._last_timestamp_ms = int(timestamp_ms)
        if not agent_states:
            return self._build_snapshot(
                timestamp_ms,
                new_completed_individual,
                [],
                new_completed_waypoints,
            )

        for state in agent_states:
            if not isinstance(state, dict):
                continue
            aircraft_id = _coerce_int(state.get("aircraft_id"))
            if aircraft_id is None:
                continue
            if aircraft_id not in self._aircraft_missions:
                continue
            current_wp = _coerce_int(state.get("current_waypoint_id"))
            if current_wp is not None and current_wp <= 0:
                current_wp = None
            on_mission = _coerce_int(state.get("on_mission"))
            on_mission = self._filter_startup_on_mission(
                aircraft_id=aircraft_id,
                current_wp=current_wp,
                on_mission=on_mission,
                timestamp_ms=timestamp_ms,
            )
            mission_id = self._resolve_mission_for_waypoint(aircraft_id, current_wp)
            if mission_id is None:
                mission_id = self._aircraft_current_mission.get(aircraft_id)
            if mission_id is None:
                missions = self._aircraft_missions.get(aircraft_id) or []
                mission_id = missions[0] if missions else None
            if mission_id is None:
                continue

            prev_mission_id = self._aircraft_current_mission.get(aircraft_id)
            wp_map = self._waypoint_to_mission.get(aircraft_id, {})
            has_direct_wp_match = (
                current_wp is not None
                and int(wp_map.get(int(current_wp), -1)) == int(mission_id)
            )
            if on_mission != 2 and has_direct_wp_match:
                self._complete_prior_missions_on_waypoint_jump(
                    aircraft_id=aircraft_id,
                    prev_mission_id=prev_mission_id,
                    current_mission_id=mission_id,
                    timestamp_ms=timestamp_ms,
                    out_completed_individual=new_completed_individual,
                    out_waypoint_updates=new_completed_waypoints,
                )
            hold_id: int | None = None
            if on_mission == 2:
                prev_id = self._aircraft_current_mission.get(aircraft_id)
                hold_id = prev_id if prev_id is not None else mission_id
                self._aircraft_hold_mission[aircraft_id] = hold_id
                mission_id = hold_id
            else:
                self._aircraft_hold_mission.pop(aircraft_id, None)

            state_obj = self._progress_state.setdefault(mission_id, MissionProgressState())
            prev_wp_id = state_obj.current_waypoint_id
            self._mark_waypoint_reached(mission_id, current_wp)
            if timestamp_ms is not None:
                ts_int = int(timestamp_ms)
                if prev_mission_id is not None and prev_mission_id != mission_id:
                    state_obj.last_update_ms = ts_int
                else:
                    last_ms = state_obj.last_update_ms
                    if last_ms is None:
                        state_obj.last_update_ms = ts_int
                    else:
                        if (
                            not state_obj.done
                            and not state_obj.awaiting_execute
                            and not state_obj.paused
                        ):
                            delta = (ts_int - int(last_ms)) / 1000.0
                            if delta > 0:
                                state_obj.elapsed_seconds += float(delta)
                        state_obj.last_update_ms = ts_int
            self._aircraft_current_mission[aircraft_id] = mission_id
            if self._update_mission_state(mission_id, current_wp, on_mission, timestamp_ms):
                if mission_id not in self._completed_mission_ids:
                    self._completed_mission_ids.add(mission_id)
                    new_completed_individual.append(
                        {
                            "mission_id": mission_id,
                            "package_id": self._mission_to_package.get(mission_id),
                        }
                    )
            self._record_waypoint_completion(
                mission_id,
                current_wp,
                prev_wp_id,
                on_mission,
                new_completed_waypoints,
            )

        formation_map = self._sync_formation_followers(timestamp_ms, new_completed_individual)
        new_completed_input: list[int] = []
        snapshot = self._build_snapshot(
            timestamp_ms,
            new_completed_individual,
            new_completed_input,
            new_completed_waypoints,
            formation_map=formation_map,
        )
        return snapshot

    def _complete_prior_missions_on_waypoint_jump(
        self,
        *,
        aircraft_id: int,
        prev_mission_id: int | None,
        current_mission_id: int | None,
        timestamp_ms: int | None,
        out_completed_individual: list[dict[str, int | None]],
        out_waypoint_updates: list[dict[str, Any]],
    ) -> None:
        if prev_mission_id is None or current_mission_id is None:
            return
        prev_id = int(prev_mission_id)
        cur_id = int(current_mission_id)
        if prev_id == cur_id:
            return
        missions = self._aircraft_missions.get(int(aircraft_id)) or []
        if not missions:
            return
        try:
            prev_idx = missions.index(prev_id)
            cur_idx = missions.index(cur_id)
        except ValueError:
            return
        prev_meta = self._mission_meta.get(prev_id)
        cur_meta = self._mission_meta.get(cur_id)
        if prev_meta is None or cur_meta is None:
            return
        prev_state = self._progress_state.get(prev_id)
        # Guard against false positives right after replan:
        # if previous mission has never actually started, do not force-complete it.
        if prev_state is not None and not prev_state.done:
            if prev_state.current_waypoint_id is None and float(prev_state.completed_seconds or 0.0) <= 0.0:
                return
        if prev_meta.input_id is not None and cur_meta.input_id is not None:
            if int(prev_meta.input_id) != int(cur_meta.input_id):
                return
        # If waypoint jumped forward to a later individual mission, treat previous mission blocks as passed.
        if cur_idx <= prev_idx:
            return
        for mission_id in missions[prev_idx:cur_idx]:
            self._force_complete_mission(
                mission_id=int(mission_id),
                timestamp_ms=timestamp_ms,
                out_completed_individual=out_completed_individual,
                out_waypoint_updates=out_waypoint_updates,
            )

    def _force_complete_mission(
        self,
        *,
        mission_id: int,
        timestamp_ms: int | None,
        out_completed_individual: list[dict[str, int | None]],
        out_waypoint_updates: list[dict[str, Any]],
    ) -> None:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return
        state = self._progress_state.setdefault(int(mission_id), MissionProgressState())
        if state.done:
            return
        if meta.waypoint_ids:
            last_wp = int(meta.waypoint_ids[-1])
            state.current_waypoint_id = last_wp
            self._record_waypoint_completion(
                int(mission_id),
                last_wp,
                state.current_waypoint_id,
                2,
                out_waypoint_updates,
            )
            self._last_completed_idx[int(mission_id)] = len(meta.waypoint_ids) - 1
        state.done = True
        state.awaiting_execute = False
        state.paused = False
        state.completed_seconds = float(max(state.completed_seconds, max(0.0, meta.planned_seconds)))
        if timestamp_ms is not None:
            state.segment_start_ms = int(timestamp_ms)
            state.last_update_ms = int(timestamp_ms)
        self._completed_mission_ids.add(int(mission_id))
        out_completed_individual.append(
            {
                "mission_id": int(mission_id),
                "package_id": self._mission_to_package.get(int(mission_id)),
            }
        )

    def get_active_input_id(self) -> int | None:
        active_counts: dict[int, int] = {}
        done_counts: dict[int, int] = {}
        for mission_id in self._aircraft_current_mission.values():
            if mission_id is None:
                continue
            meta = self._mission_meta.get(mission_id)
            if meta is None or meta.input_id is None:
                continue
            state = self._progress_state.get(mission_id)
            if state is not None and state.done:
                done_counts[meta.input_id] = done_counts.get(meta.input_id, 0) + 1
            else:
                active_counts[meta.input_id] = active_counts.get(meta.input_id, 0) + 1
        if active_counts:
            return max(active_counts.items(), key=lambda item: item[1])[0]
        if done_counts:
            return max(done_counts.items(), key=lambda item: item[1])[0]
        return None

    def get_hold_mission_id(self, aircraft_id: int | None) -> int | None:
        if aircraft_id is None:
            return None
        try:
            return self._aircraft_hold_mission.get(int(aircraft_id))
        except Exception:
            return None

    def force_complete_input(self, input_id: int | None) -> list[dict[str, int | None]]:
        if input_id is None:
            return []
        mission_ids = self._input_to_missions.get(int(input_id), [])
        completed = self.force_complete_missions(mission_ids)
        self._completed_input_ids.add(int(input_id))
        return completed

    def force_complete_missions(self, mission_ids: list[int]) -> list[dict[str, int | None]]:
        completed: list[dict[str, int | None]] = []
        for mission_id in mission_ids:
            meta = self._mission_meta.get(mission_id)
            if meta is None:
                continue
            state = self._progress_state.setdefault(mission_id, MissionProgressState())
            if not state.done:
                state.done = True
                state.awaiting_execute = False
                state.completed_seconds = float(max(0.0, meta.planned_seconds))
                if meta.waypoint_ids:
                    state.current_waypoint_id = int(meta.waypoint_ids[-1])
                state.segment_start_ms = self._last_timestamp_ms
            self._completed_mission_ids.add(mission_id)
            completed.append({"mission_id": mission_id, "package_id": meta.package_id})
        return completed

    def reset_input_progress(self, input_id: int | None) -> list[int]:
        if input_id is None:
            return []
        mission_ids = self._input_to_missions.get(int(input_id), [])
        self.reset_missions(mission_ids)
        self._completed_input_ids.discard(int(input_id))
        return list(mission_ids)

    def reset_missions(self, mission_ids: list[int]) -> None:
        for mission_id in mission_ids:
            state = self._progress_state.setdefault(mission_id, MissionProgressState())
            state.completed_seconds = 0.0
            state.current_waypoint_id = None
            state.segment_start_ms = None
            state.done = False
            state.paused = False
            state.awaiting_execute = False
            state.elapsed_seconds = 0.0
            state.last_update_ms = None
            self._completed_mission_ids.discard(mission_id)
            meta = self._mission_meta.get(mission_id)
            if meta is not None:
                self._waypoint_state[mission_id] = {
                    int(wid): "pending" for wid in meta.waypoint_ids
                }

    def pause_aircraft(self, aircraft_id: int | None, timestamp_ms: int | None) -> None:
        if aircraft_id is None:
            return
        aid = int(aircraft_id)
        mission_ids = self._aircraft_missions.get(aid) or []
        if not mission_ids:
            return
        if timestamp_ms is not None:
            self._last_timestamp_ms = int(timestamp_ms)
        for mission_id in mission_ids:
            meta = self._mission_meta.get(mission_id)
            if meta is None:
                continue
            state = self._progress_state.setdefault(mission_id, MissionProgressState())
            if state.done:
                continue
            progress = self._progress_seconds(meta, state, timestamp_ms)
            state.completed_seconds = max(state.completed_seconds, progress)
            if timestamp_ms is not None:
                state.segment_start_ms = int(timestamp_ms)
            state.paused = True
        self._paused_aircraft.add(aid)

    def resume_aircraft(self, aircraft_id: int | None, timestamp_ms: int | None) -> None:
        if aircraft_id is None:
            return
        aid = int(aircraft_id)
        mission_ids = self._aircraft_missions.get(aid) or []
        if not mission_ids:
            return
        if timestamp_ms is not None:
            self._last_timestamp_ms = int(timestamp_ms)
        for mission_id in mission_ids:
            state = self._progress_state.setdefault(mission_id, MissionProgressState())
            if state.done:
                continue
            state.paused = False
            if timestamp_ms is not None:
                state.segment_start_ms = int(timestamp_ms)
        self._paused_aircraft.discard(aid)

    def _sync_formation_followers(
        self,
        timestamp_ms: int | None,
        new_completed_individual: list[dict[str, int | None]],
    ) -> dict[int, int]:
        if not self._formation_followers:
            return {}
        resolved: dict[int, int] = {}
        for follower_id, info in self._formation_followers.items():
            leader_id = self._formation_followers_map.get(follower_id)
            if leader_id is None:
                leader_aircraft = info.get("leader_aircraft_id")
                if leader_aircraft is not None:
                    leader_id = self._aircraft_current_mission.get(int(leader_aircraft))
                    if leader_id is None:
                        leader_list = self._aircraft_missions.get(int(leader_aircraft)) or []
                        leader_id = leader_list[0] if leader_list else None
                if leader_id is None:
                    continue
                resolved[follower_id] = int(leader_id)
            leader_state = self._progress_state.get(int(leader_id))
            if leader_state is None:
                continue
            follower_state = self._progress_state.setdefault(
                int(follower_id), MissionProgressState()
            )
            if leader_state.done and not follower_state.done:
                follower_state.done = True
                follower_state.awaiting_execute = False
                meta = self._mission_meta.get(int(follower_id))
                if meta is not None:
                    follower_state.completed_seconds = float(
                        max(follower_state.completed_seconds, meta.planned_seconds)
                    )
                if timestamp_ms is not None:
                    follower_state.segment_start_ms = int(timestamp_ms)
                self._completed_mission_ids.add(int(follower_id))
                new_completed_individual.append(
                    {
                        "mission_id": int(follower_id),
                        "package_id": self._mission_to_package.get(int(follower_id)),
                    }
                )
        return resolved

    def _arm_on_mission_startup_guard(self) -> None:
        # Some simulators can momentarily publish onMission=2 right after
        # mode-3 entry, before mission execution actually starts.
        self._on_mission_startup_guard_pending = set(self._aircraft_missions.keys())
        self._on_mission_startup_guard_first_wp = {
            int(aid): None for aid in self._on_mission_startup_guard_pending
        }
        self._on_mission_startup_guard_baselined = set()
        self._on_mission_startup_guard_start_ms = {}

    def _clear_on_mission_startup_guard(self) -> None:
        self._on_mission_startup_guard_pending = set()
        self._on_mission_startup_guard_first_wp = {}
        self._on_mission_startup_guard_baselined = set()
        self._on_mission_startup_guard_start_ms = {}

    def _release_on_mission_startup_guard(self, aircraft_id: int) -> None:
        aid = int(aircraft_id)
        self._on_mission_startup_guard_pending.discard(aid)
        self._on_mission_startup_guard_first_wp.pop(aid, None)
        self._on_mission_startup_guard_baselined.discard(aid)
        self._on_mission_startup_guard_start_ms.pop(aid, None)

    def _filter_startup_on_mission(
        self,
        *,
        aircraft_id: int,
        current_wp: int | None,
        on_mission: int | None,
        timestamp_ms: int | None,
    ) -> int | None:
        if self._system_mode_code != 3:
            return on_mission
        aid = int(aircraft_id)
        if aid not in self._on_mission_startup_guard_pending:
            return on_mission

        if timestamp_ms is not None:
            ts_int = int(timestamp_ms)
            start_ms = self._on_mission_startup_guard_start_ms.get(aid)
            if start_ms is None:
                self._on_mission_startup_guard_start_ms[aid] = ts_int
            elif ts_int >= start_ms and (ts_int - start_ms) >= _ON_MISSION_STARTUP_GUARD_MS:
                self._release_on_mission_startup_guard(aid)
                return on_mission

        if aid not in self._on_mission_startup_guard_baselined:
            self._on_mission_startup_guard_first_wp[aid] = current_wp
            self._on_mission_startup_guard_baselined.add(aid)
        else:
            first_wp = self._on_mission_startup_guard_first_wp.get(aid)
            if current_wp is not None and current_wp != first_wp:
                self._release_on_mission_startup_guard(aid)
                return on_mission

        if on_mission is None:
            return None
        if int(on_mission) != 2:
            self._release_on_mission_startup_guard(aid)
            return on_mission
        return None

    def _resolve_mission_for_waypoint(self, aircraft_id: int, waypoint_id: int | None) -> int | None:
        if waypoint_id is None:
            return None
        mapping = self._waypoint_to_mission.get(aircraft_id, {})
        if waypoint_id in mapping:
            return mapping.get(waypoint_id)
        missions = self._aircraft_missions.get(aircraft_id) or []
        for mission_id in missions:
            meta = self._mission_meta.get(mission_id)
            if meta is None or not meta.waypoint_ids:
                continue
            try:
                min_wp = min(meta.waypoint_ids)
                max_wp = max(meta.waypoint_ids)
            except ValueError:
                continue
            if min_wp <= waypoint_id <= max_wp:
                return mission_id
        return None

    def _bounds_for_waypoint(
        self,
        meta: MissionMeta | None,
        waypoint_id: int | None,
    ) -> tuple[float, float, float] | None:
        if meta is None or waypoint_id is None or not meta.waypoint_ids:
            return None
        wp_id = int(waypoint_id)
        idx = meta.waypoint_index.get(wp_id)
        if idx is None:
            return None
        upper = float(meta.waypoint_eta_cumulative.get(wp_id, 0.0))
        if idx <= 0:
            lower = 0.0
        else:
            prev_wp = int(meta.waypoint_ids[idx - 1])
            lower = float(meta.waypoint_eta_cumulative.get(prev_wp, 0.0))
        if upper < lower:
            upper = lower
        leg = max(0.0, upper - lower)
        return lower, upper, leg

    def _lower_bound_seconds(self, meta: MissionMeta | None, waypoint_id: int | None) -> float:
        bounds = self._bounds_for_waypoint(meta, waypoint_id)
        if bounds is None:
            return 0.0
        lower, _upper, _leg = bounds
        return float(lower)

    def _progress_seconds(
        self,
        meta: MissionMeta | None,
        state: MissionProgressState,
        timestamp_ms: int | None,
    ) -> float:
        base = max(0.0, float(state.completed_seconds))
        if meta is None:
            return base
        planned = max(0.0, float(meta.planned_seconds))
        if state.paused:
            return min(base, planned) if planned > 0 else base
        if state.awaiting_execute:
            return max(base, planned)
        if state.done:
            return max(base, planned)
        bounds = self._bounds_for_waypoint(meta, state.current_waypoint_id)
        if bounds is None:
            return min(base, planned) if planned > 0 else base
        lower, upper, leg = bounds
        progress = max(base, lower)
        if timestamp_ms is None or state.segment_start_ms is None:
            return min(progress, planned) if planned > 0 else progress
        elapsed = (int(timestamp_ms) - int(state.segment_start_ms)) / 1000.0
        if elapsed < 0:
            elapsed = 0.0
        if leg > 0:
            progress = max(progress, lower + min(elapsed, leg))
        else:
            progress = max(progress, upper)
        if upper > 0:
            progress = min(progress, upper)
        if planned > 0:
            progress = min(progress, planned)
        return progress

    def _update_mission_state(
        self,
        mission_id: int,
        current_wp: int | None,
        on_mission: int | None,
        timestamp_ms: int | None,
    ) -> bool:
        state = self._progress_state.setdefault(mission_id, MissionProgressState())
        meta = self._mission_meta.get(mission_id)

        if state.done:
            return True
        if meta is None:
            return False
        if state.paused and on_mission != 2:
            return False
        if on_mission == 2:
            state.awaiting_execute = True
            state.completed_seconds = float(max(state.completed_seconds, meta.planned_seconds))
            if current_wp is not None and current_wp in meta.waypoint_index:
                state.current_waypoint_id = int(current_wp)
            if timestamp_ms is not None:
                state.segment_start_ms = int(timestamp_ms)
            return False
        if on_mission is not None and state.awaiting_execute:
            state.awaiting_execute = False
        if current_wp is None:
            return False
        if current_wp not in meta.waypoint_index:
            return False

        lower_bound = self._lower_bound_seconds(meta, current_wp)
        if state.current_waypoint_id is None:
            state.completed_seconds = max(state.completed_seconds, lower_bound)
            state.current_waypoint_id = int(current_wp)
            state.segment_start_ms = int(timestamp_ms) if timestamp_ms is not None else None
            return False
        if current_wp == state.current_waypoint_id:
            state.completed_seconds = max(state.completed_seconds, lower_bound)
            return False

        current_progress = self._progress_seconds(meta, state, timestamp_ms)
        state.completed_seconds = max(current_progress, lower_bound)
        state.current_waypoint_id = int(current_wp)
        state.segment_start_ms = int(timestamp_ms) if timestamp_ms is not None else None
        return False

    def _build_snapshot(
        self,
        timestamp_ms: int | None,
        new_completed_individual: list[dict[str, int | None]],
        new_completed_input: list[int],
        new_completed_waypoints: list[dict[str, Any]],
        *,
        formation_map: dict[int, int] | None = None,
    ) -> dict[str, Any]:
        mission_progress: dict[int, dict[str, Any]] = {}
        for mission_id, meta in self._mission_meta.items():
            state = self._progress_state.get(mission_id, MissionProgressState())
            planned = max(meta.planned_seconds, 0.0)
            actual = self._progress_seconds(meta, state, timestamp_ms)
            actual_real = float(max(0.0, state.elapsed_seconds))
            last_wp = int(meta.waypoint_ids[-1]) if meta.waypoint_ids else None
            if (
                not state.done
                and not state.awaiting_execute
                and last_wp is not None
                and state.current_waypoint_id == last_wp
            ):
                lower = self._lower_bound_seconds(meta, last_wp)
                cap_seconds = lower
                if planned > 0:
                    cap_seconds = max(lower, planned * 0.95)
                if cap_seconds > 0:
                    actual = min(actual, cap_seconds)
            if planned > 0:
                actual = min(actual, planned)
            percent = self._calc_percent(actual, planned, state.done, force_full=state.awaiting_execute)
            mission_progress[mission_id] = {
                "progress_percent": percent,
                "actual_seconds": int(round(actual)),
                "actual_seconds_real": int(round(actual_real)),
                "planned_seconds": int(round(planned)),
                "done": state.done,
                "awaiting_execute": bool(state.awaiting_execute),
                "input_id": meta.input_id,
                "aircraft_id": meta.aircraft_id,
                "waypoint_status": self._serialize_waypoint_status(mission_id, meta),
            }

        if formation_map:
            for follower_id, leader_id in formation_map.items():
                leader_prog = mission_progress.get(int(leader_id))
                if not isinstance(leader_prog, dict):
                    continue
                follower_prog = mission_progress.get(int(follower_id))
                if not isinstance(follower_prog, dict):
                    follower_prog = {}
                    meta = self._mission_meta.get(int(follower_id))
                    if meta is not None:
                        follower_prog["input_id"] = meta.input_id
                        follower_prog["aircraft_id"] = meta.aircraft_id
                    mission_progress[int(follower_id)] = follower_prog
                for key in (
                    "progress_percent",
                    "actual_seconds",
                    "actual_seconds_real",
                    "planned_seconds",
                    "done",
                    "awaiting_execute",
                    "waypoint_status",
                ):
                    if key in leader_prog:
                        follower_prog[key] = leader_prog[key]

        package_progress: dict[int, dict[str, Any]] = {}
        for aircraft_id, mission_ids in self._aircraft_missions.items():
            package_progress[aircraft_id] = self._aggregate_count_progress(
                mission_ids,
                mission_progress,
            )

        input_progress: dict[int, dict[str, Any]] = {}
        for input_id in self._input_mission_ids:
            mission_ids = self._input_to_missions.get(input_id, [])
            done_override = input_id in self._completed_input_ids
            input_progress[input_id] = self._aggregate_progress(
                mission_ids,
                mission_progress,
                done_override=done_override,
            )

        for input_id, data in input_progress.items():
            if not data.get("done"):
                continue
            if input_id in self._completed_input_ids:
                continue
            self._completed_input_ids.add(input_id)
            new_completed_input.append(input_id)

        plan_progress = self._aggregate_count_progress(
            [mid for mid in self._input_mission_ids if mid in input_progress],
            input_progress,
        )

        return {
            "mission_progress": mission_progress,
            "package_progress": package_progress,
            "input_progress": input_progress,
            "plan_progress": plan_progress,
            "new_completed_individual": new_completed_individual,
            "new_completed_input": new_completed_input,
            "new_completed_waypoints": new_completed_waypoints,
        }

    def _record_waypoint_completion(
        self,
        mission_id: int,
        current_wp: int | None,
        prev_wp: int | None,
        on_mission: int | None,
        out_updates: list[dict[str, Any]],
    ) -> None:
        meta = self._mission_meta.get(mission_id)
        if meta is None or not meta.waypoint_ids:
            return
        current_idx = None
        if current_wp is not None and current_wp in meta.waypoint_index:
            current_idx = meta.waypoint_index.get(int(current_wp))
        last_completed = self._last_completed_idx.get(mission_id, -1)
        if on_mission == 2:
            target_completed = len(meta.waypoint_ids) - 1
        else:
            if current_idx is None:
                return
            target_completed = int(current_idx) - 1
        if target_completed <= last_completed:
            return
        if meta.path_id is None:
            return
        start_idx = max(0, last_completed + 1)
        end_idx = max(start_idx, target_completed + 1)
        completed_ids = meta.waypoint_ids[start_idx:end_idx]
        if not completed_ids:
            return
        for wid in completed_ids:
            self._mark_waypoint_skipped(mission_id, int(wid))
        out_updates.append(
            {
                "mission_id": mission_id,
                "path_id": meta.path_id,
                "waypoint_ids": completed_ids,
            }
        )
        self._last_completed_idx[mission_id] = target_completed

    def _mark_waypoint_reached(self, mission_id: int, waypoint_id: int | None) -> None:
        if waypoint_id is None:
            return
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return
        wid = int(waypoint_id)
        if wid not in meta.waypoint_index:
            return
        state = self._waypoint_state.setdefault(
            int(mission_id),
            {int(v): "pending" for v in meta.waypoint_ids},
        )
        state[wid] = "reached"

    def _mark_waypoint_skipped(self, mission_id: int, waypoint_id: int) -> None:
        meta = self._mission_meta.get(int(mission_id))
        if meta is None:
            return
        wid = int(waypoint_id)
        if wid not in meta.waypoint_index:
            return
        state = self._waypoint_state.setdefault(
            int(mission_id),
            {int(v): "pending" for v in meta.waypoint_ids},
        )
        if state.get(wid) == "reached":
            return
        state[wid] = "skipped"

    def _serialize_waypoint_status(
        self,
        mission_id: int,
        meta: MissionMeta,
    ) -> list[dict[str, Any]]:
        state = self._waypoint_state.setdefault(
            int(mission_id),
            {int(v): "pending" for v in meta.waypoint_ids},
        )
        items: list[dict[str, Any]] = []
        for wid in meta.waypoint_ids:
            status = str(state.get(int(wid)) or "pending")
            if status not in ("pending", "reached", "skipped"):
                status = "pending"
            items.append({"waypoint_id": int(wid), "status": status})
        return items

    def _aggregate_progress(
        self,
        ids: list[int],
        progress_map: dict[int, dict[str, Any]],
        *,
        done_override: bool | None,
    ) -> dict[str, Any]:
        actual_total = 0.0
        planned_total = 0.0
        done_flags: list[bool] = []
        ready_flags: list[bool] = []
        for mid in ids:
            data = progress_map.get(mid)
            if not data:
                continue
            actual_total += float(data.get("actual_seconds") or 0)
            planned_total += float(data.get("planned_seconds") or 0)
            done_flags.append(bool(data.get("done")))
            ready_flags.append(bool(data.get("awaiting_execute")))
        if done_override is None:
            done = all(done_flags) if done_flags else False
        else:
            done = bool(done_override) or (all(done_flags) if done_flags else False)
        force_full = False
        if not done and done_flags:
            force_full = all(
                (done_flags[idx] or ready_flags[idx]) for idx in range(len(done_flags))
            )
        percent = self._calc_percent(actual_total, planned_total, done, force_full=force_full)
        return {
            "progress_percent": percent,
            "actual_seconds": int(round(actual_total)),
            "planned_seconds": int(round(planned_total)),
            "done": done,
        }

    @staticmethod
    def _aggregate_count_progress(
        ids: list[int],
        progress_map: dict[int, dict[str, Any]],
    ) -> dict[str, Any]:
        total = 0
        done_count = 0
        for mid in ids:
            data = progress_map.get(mid)
            if not data:
                continue
            total += 1
            if data.get("done"):
                done_count += 1
        if total <= 0:
            percent = 0
            done = False
        else:
            percent = int(round((done_count / total) * 100))
            done = done_count >= total
        return {
            "progress_percent": percent,
            "actual_seconds": done_count,
            "planned_seconds": total,
            "done": done,
        }

    @staticmethod
    def _calc_percent(actual: float, planned: float, done: bool, *, force_full: bool = False) -> int:
        if planned <= 0:
            return 100 if (done or force_full) else 0
        percent = int(round((actual / planned) * 100))
        if done or force_full:
            return 100
        return max(0, min(percent, 99))
