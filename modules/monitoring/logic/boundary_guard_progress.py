from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable


def _to_int(value: object) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _to_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _set_id(value: object) -> str | None:
    text = str(value or "").strip()
    return text or None


@dataclass(frozen=True)
class BoundaryGuardSetContract:
    input_id: int
    set_id: str
    aircraft_id: int
    sequence_count: int
    first_waypoint_id: int | None
    last_waypoint_id: int | None


@dataclass
class BoundaryGuardInputRuntime:
    input_id: int
    duration_s: float
    expected_set_ids: set[str] = field(default_factory=set)
    expected_aircraft_ids: set[int] = field(default_factory=set)
    started_at_ms: int | None = None
    last_timestamp_ms: int | None = None
    cycle_count_by_set: dict[str, int] = field(default_factory=dict)
    last_waypoint_by_set: dict[str, int] = field(default_factory=dict)
    published_cycle_baseline_by_set: dict[str, int] = field(default_factory=dict)
    rearm_baseline_pending_set_ids: set[str] = field(default_factory=set)
    ready: bool = False
    tracking_blocked_aircraft_ids: set[int] = field(default_factory=set)

    def elapsed_s(self, timestamp_ms: int | None = None) -> float:
        if self.started_at_ms is None:
            return 0.0
        end_ms = (
            int(timestamp_ms)
            if timestamp_ms is not None
            else int(self.last_timestamp_ms)
            if self.last_timestamp_ms is not None
            else int(self.started_at_ms)
        )
        return max(0.0, (int(end_ms) - int(self.started_at_ms)) / 1000.0)

    @property
    def duration_satisfied(self) -> bool:
        return self.elapsed_s() + 1e-9 >= max(0.0, float(self.duration_s))

    @property
    def all_expected_sets_completed_once(self) -> bool:
        return bool(self.expected_set_ids) and all(
            int(self.cycle_count_by_set.get(set_id, 0) or 0) >= 1
            for set_id in self.expected_set_ids
        )


class BoundaryGuardProgressGate:
    """Pure Type-2 boundary guard duration/first-cycle completion gate.

    Flight-path execution owns loop control.  This ledger only observes the
    explicit loop contract and SIM-published cycle counter.  A final-to-first
    waypoint transition is retained as a compatibility fallback.
    """

    def __init__(self, *, default_duration_s: float = 600.0) -> None:
        self.default_duration_s = max(0.001, float(default_duration_s))
        self._package_id: int | None = None
        self._contracts_by_set: dict[str, BoundaryGuardSetContract] = {}
        self._set_by_waypoint_id: dict[int, str] = {}
        self._runtime_by_input: dict[int, BoundaryGuardInputRuntime] = {}

    def configure(self, view: dict[str, Any] | None) -> None:
        view = view if isinstance(view, dict) else {}
        package_id = _to_int(view.get("input_mission_package_id"))
        if (
            self._package_id is not None
            and package_id is not None
            and int(self._package_id) != int(package_id)
        ):
            self._runtime_by_input.clear()
        self._package_id = package_id

        completed_input_ids = {
            int(input_id)
            for row in view.get("input_missions") or []
            if isinstance(row, dict)
            and bool(row.get("is_done"))
            and (input_id := _to_int(row.get("input_mission_id"))) is not None
        }
        contracts: dict[str, BoundaryGuardSetContract] = {}
        set_by_waypoint: dict[int, str] = {}
        duration_by_input: dict[int, float] = {}
        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = _to_int(entry.get("aircraft_id"))
            if aircraft_id is None:
                continue
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                input_id = _to_int(mission.get("input_id"))
                if input_id is None:
                    continue
                if int(input_id) in completed_input_ids:
                    continue
                explicit_loop = bool(mission.get("boundary_guard_loop"))
                # Only the explicit portable planner contract enables special
                # duration/cycle semantics.  A generic AREA with regionType=7
                # must keep the normal one-pass completion behavior.
                if not explicit_loop:
                    continue
                set_id = _set_id(mission.get("boundary_guard_set_id"))
                if set_id is None:
                    set_id = f"input:{int(input_id)}:aircraft:{int(aircraft_id)}"
                contract = BoundaryGuardSetContract(
                    input_id=int(input_id),
                    set_id=str(set_id),
                    aircraft_id=int(aircraft_id),
                    sequence_count=max(
                        1,
                        int(_to_int(mission.get("boundary_guard_sequence_count")) or 1),
                    ),
                    first_waypoint_id=_to_int(
                        mission.get("boundary_guard_cycle_first_waypoint_id")
                    ),
                    last_waypoint_id=_to_int(
                        mission.get("boundary_guard_cycle_last_waypoint_id")
                    ),
                )
                previous = contracts.get(str(set_id))
                if previous is None:
                    contracts[str(set_id)] = contract
                elif (
                    previous.first_waypoint_id is None
                    and contract.first_waypoint_id is not None
                ):
                    contracts[str(set_id)] = contract
                duration = _to_float(mission.get("boundary_guard_duration_s"))
                if duration is not None and duration > 0.0:
                    duration_by_input[int(input_id)] = float(duration)
                for waypoint in mission.get("waypoints") or []:
                    if not isinstance(waypoint, dict):
                        continue
                    waypoint_id = _to_int(waypoint.get("waypoint_id"))
                    if waypoint_id is not None and waypoint_id > 0:
                        set_by_waypoint[int(waypoint_id)] = str(set_id)

        self._contracts_by_set = contracts
        self._set_by_waypoint_id = set_by_waypoint

        valid_inputs = {int(contract.input_id) for contract in contracts.values()}
        for stale_input in set(self._runtime_by_input) - valid_inputs:
            self._runtime_by_input.pop(int(stale_input), None)
        for input_id in valid_inputs:
            expected = [
                contract
                for contract in contracts.values()
                if int(contract.input_id) == int(input_id)
            ]
            expected_set_ids = {
                str(contract.set_id) for contract in expected
            }
            expected_aircraft_ids = {
                int(contract.aircraft_id) for contract in expected
            }
            runtime = self._runtime_by_input.get(int(input_id))
            if runtime is None:
                runtime = BoundaryGuardInputRuntime(
                    input_id=int(input_id),
                    duration_s=float(
                        duration_by_input.get(int(input_id), self.default_duration_s)
                    ),
                )
                self._runtime_by_input[int(input_id)] = runtime
            else:
                runtime.duration_s = float(
                    duration_by_input.get(int(input_id), runtime.duration_s)
                )
                previous_set_ids = set(runtime.expected_set_ids)
                stable_set_ids = previous_set_ids & expected_set_ids
                # A replan may remove or replace an owner/set.  Requirements
                # always follow the current immutable plan contract; otherwise
                # an obsolete set can keep the guard mission below 100% forever.
                runtime.cycle_count_by_set = {
                    str(set_id): int(count)
                    for set_id, count in runtime.cycle_count_by_set.items()
                    if str(set_id) in stable_set_ids
                }
                runtime.last_waypoint_by_set = {
                    str(set_id): int(waypoint_id)
                    for set_id, waypoint_id in runtime.last_waypoint_by_set.items()
                    if str(set_id) in stable_set_ids
                }
                runtime.published_cycle_baseline_by_set = {
                    str(set_id): int(count)
                    for set_id, count in runtime.published_cycle_baseline_by_set.items()
                    if str(set_id) in stable_set_ids
                }
                runtime.rearm_baseline_pending_set_ids &= stable_set_ids
                if previous_set_ids and not stable_set_ids:
                    # None of the prior loops survived this replacement.  The
                    # new contract has not started guarding yet.
                    runtime.started_at_ms = None
                    runtime.last_timestamp_ms = None
                    runtime.ready = False
                    runtime.tracking_blocked_aircraft_ids.clear()
                elif expected_set_ids - previous_set_ids:
                    # Preserve the wall clock and stable-set counters, but a
                    # newly introduced owner must still finish one cycle.
                    runtime.ready = False
            runtime.expected_set_ids = set(expected_set_ids)
            runtime.expected_aircraft_ids = set(expected_aircraft_ids)
            for set_id in expected_set_ids:
                runtime.cycle_count_by_set.setdefault(str(set_id), 0)

    def reset_input(self, input_id: int | None) -> bool:
        if input_id is None:
            return False
        runtime = self._runtime_by_input.get(int(input_id))
        if runtime is None:
            return False
        runtime.started_at_ms = None
        runtime.last_timestamp_ms = None
        runtime.ready = False
        runtime.tracking_blocked_aircraft_ids.clear()
        runtime.last_waypoint_by_set.clear()
        runtime.cycle_count_by_set = {
            str(set_id): 0 for set_id in runtime.expected_set_ids
        }
        # The controller may publish one final sample from the old execution
        # before its plan reload resets the counter.  Baseline that first
        # sample so an explicit 0803 re-execution cannot instantly reach 100%.
        runtime.published_cycle_baseline_by_set.clear()
        runtime.rearm_baseline_pending_set_ids = set(runtime.expected_set_ids)
        return True

    def is_guard_input(self, input_id: int | None) -> bool:
        return input_id is not None and int(input_id) in self._runtime_by_input

    def is_ready(self, input_id: int | None) -> bool:
        runtime = (
            self._runtime_by_input.get(int(input_id))
            if input_id is not None
            else None
        )
        return bool(runtime is not None and runtime.ready)

    def ready_input_ids(self) -> tuple[int, ...]:
        return tuple(
            sorted(
                int(input_id)
                for input_id, runtime in self._runtime_by_input.items()
                if runtime.ready
            )
        )

    def status(self, input_id: int | None) -> dict[str, Any] | None:
        runtime = (
            self._runtime_by_input.get(int(input_id))
            if input_id is not None
            else None
        )
        if runtime is None:
            return None
        if runtime.ready:
            phase = "ready"
        elif runtime.tracking_blocked_aircraft_ids:
            phase = "waiting_tracking"
        elif runtime.duration_satisfied and not runtime.all_expected_sets_completed_once:
            phase = "waiting_first_cycle"
        elif runtime.started_at_ms is None:
            phase = "waiting_activation"
        else:
            phase = "guarding"
        return {
            "input_id": int(runtime.input_id),
            "duration_s": float(runtime.duration_s),
            "started_at_ms": runtime.started_at_ms,
            "elapsed_s": float(runtime.elapsed_s()),
            "duration_satisfied": bool(runtime.duration_satisfied),
            "expected_set_ids": sorted(runtime.expected_set_ids),
            "expected_aircraft_ids": sorted(runtime.expected_aircraft_ids),
            "cycle_count_by_set": {
                str(key): int(value)
                for key, value in sorted(runtime.cycle_count_by_set.items())
            },
            "all_expected_sets_completed_once": bool(
                runtime.all_expected_sets_completed_once
            ),
            "tracking_blocked_aircraft_ids": sorted(
                runtime.tracking_blocked_aircraft_ids
            ),
            "ready": bool(runtime.ready),
            "phase": phase,
        }

    def statuses(self) -> dict[int, dict[str, Any]]:
        return {
            int(input_id): status
            for input_id in sorted(self._runtime_by_input)
            if (status := self.status(int(input_id))) is not None
        }

    def update(
        self,
        *,
        timestamp_ms: int | None,
        agent_states: Iterable[dict[str, Any]],
        active_tracking_assignments: Iterable[dict[str, Any]] = (),
    ) -> tuple[int, ...]:
        state_rows = [row for row in agent_states if isinstance(row, dict)]
        state_by_aircraft = {
            int(aircraft_id): row
            for row in state_rows
            if (aircraft_id := _to_int(row.get("aircraft_id"))) is not None
        }

        tracking_by_input: dict[int, set[int]] = {}
        unknown_tracking_aircraft: set[int] = set()
        for assignment in active_tracking_assignments:
            if not isinstance(assignment, dict) or not bool(
                assignment.get("active", True)
            ):
                continue
            aircraft_id = _to_int(assignment.get("aircraft_id"))
            if aircraft_id is None:
                continue
            assignment_input_id = _to_int(
                assignment.get("current_input_mission_id")
            )
            if assignment_input_id is None:
                unknown_tracking_aircraft.add(int(aircraft_id))
            else:
                tracking_by_input.setdefault(int(assignment_input_id), set()).add(
                    int(aircraft_id)
                )

        for row in state_rows:
            aircraft_id = _to_int(row.get("aircraft_id"))
            if aircraft_id is None:
                continue
            published_set_id = _set_id(row.get("boundary_guard_set_id"))
            current_wp = _to_int(row.get("current_waypoint_id"))
            set_id = published_set_id or (
                self._set_by_waypoint_id.get(int(current_wp))
                if current_wp is not None
                else None
            )
            if set_id is None:
                continue
            contract = self._contracts_by_set.get(str(set_id))
            if contract is None or int(contract.aircraft_id) != int(aircraft_id):
                continue
            runtime = self._runtime_by_input.get(int(contract.input_id))
            if runtime is None:
                continue
            loop_active = bool(row.get("boundary_guard_loop_active"))
            tracking_mode = _to_int(row.get("flight_mode")) == 9
            capture_active = (
                _to_int(row.get("filming")) == 1
                or bool(row.get("sensor_capture_active"))
                or bool(row.get("capture_active"))
                or bool(row.get("is_filming"))
            )
            if (
                runtime.started_at_ms is None
                and not tracking_mode
                and capture_active
                and (loop_active or current_wp in self._set_by_waypoint_id)
            ):
                if timestamp_ms is not None:
                    runtime.started_at_ms = int(timestamp_ms)
            if timestamp_ms is not None:
                runtime.last_timestamp_ms = int(timestamp_ms)

            if tracking_mode:
                # SIM publishes saved guard metadata while tracking.  It keeps
                # the timer alive, but must never synthesize a loop transition.
                continue

            published_count = _to_int(row.get("boundary_guard_cycle_count"))
            if published_count is not None and published_count >= 0:
                if str(set_id) in runtime.rearm_baseline_pending_set_ids:
                    runtime.published_cycle_baseline_by_set[str(set_id)] = int(
                        published_count
                    )
                    runtime.rearm_baseline_pending_set_ids.discard(str(set_id))
                baseline = int(
                    runtime.published_cycle_baseline_by_set.get(str(set_id), 0)
                    or 0
                )
                if int(published_count) < baseline:
                    baseline = int(published_count)
                    runtime.published_cycle_baseline_by_set[str(set_id)] = baseline
                effective_count = max(0, int(published_count) - int(baseline))
                runtime.cycle_count_by_set[str(set_id)] = max(
                    int(runtime.cycle_count_by_set.get(str(set_id), 0) or 0),
                    int(effective_count),
                )

            if current_wp is not None and current_wp > 0:
                previous_wp = runtime.last_waypoint_by_set.get(str(set_id))
                if (
                    published_count is None
                    and previous_wp is not None
                    and contract.last_waypoint_id is not None
                    and contract.first_waypoint_id is not None
                    and int(previous_wp) == int(contract.last_waypoint_id)
                    and int(current_wp) == int(contract.first_waypoint_id)
                ):
                    runtime.cycle_count_by_set[str(set_id)] = max(
                        int(runtime.cycle_count_by_set.get(str(set_id), 0) or 0),
                        int(runtime.cycle_count_by_set.get(str(set_id), 0) or 0)
                        + 1,
                    )
                runtime.last_waypoint_by_set[str(set_id)] = int(current_wp)

        newly_ready: list[int] = []
        for input_id, runtime in self._runtime_by_input.items():
            if timestamp_ms is not None and runtime.started_at_ms is not None:
                runtime.last_timestamp_ms = int(timestamp_ms)
            blocked = set(
                tracking_by_input.get(int(input_id), set())
            ) | set(unknown_tracking_aircraft)
            for aircraft_id in runtime.expected_aircraft_ids:
                row = state_by_aircraft.get(int(aircraft_id))
                if _to_int((row or {}).get("flight_mode")) == 9:
                    blocked.add(int(aircraft_id))
            runtime.tracking_blocked_aircraft_ids = (
                set(int(aid) for aid in blocked)
                & set(runtime.expected_aircraft_ids)
            )
            if runtime.ready:
                continue
            if (
                runtime.duration_satisfied
                and runtime.all_expected_sets_completed_once
                and not runtime.tracking_blocked_aircraft_ids
            ):
                runtime.ready = True
                newly_ready.append(int(input_id))
        return tuple(sorted(newly_ready))
