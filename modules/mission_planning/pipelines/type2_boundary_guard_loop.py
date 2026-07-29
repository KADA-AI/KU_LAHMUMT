from __future__ import annotations

"""Shared Type-2 boundary-guard loop contract.

The guard AREA is exported as multiple individual missions/flight paths.  One
UAV completes one observation pass only after it has flown every child path in
its owner set.  This module keeps that definition identical across initial and
replan generation.  The one waypoint which closes the complete cycle carries
the configured guard duration as its ETA contract; all physical/intermediate
waypoint ETAs remain untouched.
"""

from collections import defaultdict
from typing import Any, Dict, Iterable, List, Mapping, MutableMapping, Sequence, Tuple


BOUNDARY_GUARD_LOOP_VERSION = 1
BOUNDARY_GUARD_DEFAULT_DURATION_S = 600.0

BOUNDARY_GUARD_PRE_LINK_KEYS: Tuple[str, ...] = (
    "boundaryGuardLoop",
    "boundaryGuardLoopVersion",
    "boundaryGuardSetID",
    "boundaryGuardSequence",
    "boundaryGuardSequenceCount",
    "boundaryGuardDurationS",
)
BOUNDARY_GUARD_CYCLE_KEYS: Tuple[str, ...] = (
    "boundaryGuardCycleFirstWaypointID",
    "boundaryGuardCycleLastWaypointID",
)
BOUNDARY_GUARD_CONTRACT_KEYS: Tuple[str, ...] = (
    *BOUNDARY_GUARD_PRE_LINK_KEYS,
    *BOUNDARY_GUARD_CYCLE_KEYS,
)


def _positive_int(value: Any) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return 0
    return parsed if parsed > 0 else 0


def _positive_float(value: Any, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return float(default)
    return parsed if parsed > 0.0 else float(default)


def boundary_guard_contract(
    *,
    set_id: Any,
    sequence: int,
    sequence_count: int,
    duration_s: Any = BOUNDARY_GUARD_DEFAULT_DURATION_S,
) -> Dict[str, Any]:
    """Build the pre-waypoint-ID portion of the boundary-guard contract."""

    resolved_set_id = str(set_id or "").strip()
    resolved_sequence = _positive_int(sequence)
    resolved_count = _positive_int(sequence_count)
    if not resolved_set_id:
        raise ValueError("boundary guard set ID must not be empty")
    if resolved_sequence <= 0 or resolved_count <= 0 or resolved_sequence > resolved_count:
        raise ValueError(
            "boundary guard sequence must be within 1..boundaryGuardSequenceCount"
        )
    return {
        "boundaryGuardLoop": True,
        "boundaryGuardLoopVersion": int(BOUNDARY_GUARD_LOOP_VERSION),
        "boundaryGuardSetID": resolved_set_id,
        "boundaryGuardSequence": int(resolved_sequence),
        "boundaryGuardSequenceCount": int(resolved_count),
        "boundaryGuardDurationS": float(
            _positive_float(duration_s, BOUNDARY_GUARD_DEFAULT_DURATION_S)
        ),
    }


def extract_boundary_guard_contract(*sources: Any) -> Dict[str, Any]:
    """Merge contract fields from mappings, preferring the first source."""

    out: Dict[str, Any] = {}
    for source in sources:
        if not isinstance(source, Mapping):
            continue
        for key in BOUNDARY_GUARD_CONTRACT_KEYS:
            if key not in out and key in source:
                out[key] = source.get(key)
    return out


def is_boundary_guard_loop(source: Any) -> bool:
    return bool(
        isinstance(source, Mapping)
        and source.get("boundaryGuardLoop") is True
        and str(source.get("boundaryGuardSetID") or "").strip()
    )


def apply_boundary_guard_contract(
    target: MutableMapping[str, Any],
    contract: Mapping[str, Any],
    *,
    include_individual_mission_info: bool = False,
) -> None:
    """Copy the contract onto a payload and, for IMP rows, its info block."""

    copied: Dict[str, Any] = {}
    for key in BOUNDARY_GUARD_CONTRACT_KEYS:
        if key in contract:
            target[key] = contract.get(key)
            copied[key] = contract.get(key)

    if include_individual_mission_info:
        info = target.get("individualMissionInfo")
        if not isinstance(info, MutableMapping):
            info = {}
            target["individualMissionInfo"] = info
        for key, value in copied.items():
            info[key] = value


def clear_boundary_guard_contract(
    target: MutableMapping[str, Any],
    *,
    include_individual_mission_info: bool = False,
) -> None:
    """Remove guard-loop metadata from a synthetic or completed payload."""

    for key in BOUNDARY_GUARD_CONTRACT_KEYS:
        target.pop(key, None)
    if include_individual_mission_info:
        info = target.get("individualMissionInfo")
        if isinstance(info, MutableMapping):
            for key in BOUNDARY_GUARD_CONTRACT_KEYS:
                info.pop(key, None)


def annotate_boundary_guard_set(
    rows: Sequence[MutableMapping[str, Any]],
    *,
    set_id: Any,
    duration_s: Any = BOUNDARY_GUARD_DEFAULT_DURATION_S,
    include_individual_mission_info: bool = False,
) -> None:
    """Assign deterministic 1-based sequence metadata to one owner child set."""

    count = len(rows)
    if count <= 0:
        return
    for index, row in enumerate(rows, start=1):
        apply_boundary_guard_contract(
            row,
            boundary_guard_contract(
                set_id=set_id,
                sequence=index,
                sequence_count=count,
                duration_s=duration_s,
            ),
            include_individual_mission_info=include_individual_mission_info,
        )


def _waypoint_list(payload: Mapping[str, Any]) -> List[MutableMapping[str, Any]]:
    for key in ("waypointList", "wplist"):
        value = payload.get(key)
        if isinstance(value, list):
            return [row for row in value if isinstance(row, MutableMapping)]
    return []


def _waypoint_id(row: Mapping[str, Any]) -> int:
    for key in ("waypointID", "wpID"):
        value = _positive_int(row.get(key))
        if value > 0:
            return value
    return 0


def _set_next_waypoint_id(row: MutableMapping[str, Any], waypoint_id: int) -> None:
    # FlightPath uses nextWaypointID.  Preserve a legacy alias only when it was
    # already present instead of creating a second schema field.
    row["nextWaypointID"] = int(waypoint_id)
    if "nextWpID" in row:
        row["nextWpID"] = int(waypoint_id)


def _set_boundary_guard_cycle_eta(
    row: MutableMapping[str, Any],
    duration_s: float,
) -> None:
    """Put the configured repeat duration on the one cycle-closing waypoint."""

    numeric_duration = float(duration_s)
    value: int | float = (
        int(numeric_duration)
        if numeric_duration.is_integer()
        else numeric_duration
    )
    row["eta"] = value
    # Preserve a legacy alias only when the producer already emitted it.
    if "ETA" in row:
        row["ETA"] = value


def _group_boundary_guard_payloads(
    payloads: Iterable[MutableMapping[str, Any]],
) -> Dict[str, List[MutableMapping[str, Any]]]:
    grouped: Dict[str, List[MutableMapping[str, Any]]] = defaultdict(list)
    for payload in payloads:
        if not is_boundary_guard_loop(payload):
            continue
        grouped[str(payload.get("boundaryGuardSetID")).strip()].append(payload)
    return dict(grouped)


def link_boundary_guard_flight_path_sets(
    payloads: Iterable[MutableMapping[str, Any]],
    *,
    strict: bool = True,
) -> Dict[str, Dict[str, int]]:
    """Link every complete guard owner set into a waypoint cycle.

    Child path tails point to the next child's first waypoint and the final
    child's tail points back to the set's first waypoint.  Only that final,
    cycle-closing waypoint receives ``boundaryGuardDurationS`` as its ETA.
    """

    summary: Dict[str, Dict[str, int]] = {}
    for set_id, rows in _group_boundary_guard_payloads(payloads).items():
        def fail(message: str) -> None:
            if strict:
                raise ValueError(f"boundary guard set {set_id}: {message}")

        versions = {_positive_int(row.get("boundaryGuardLoopVersion")) for row in rows}
        counts = {_positive_int(row.get("boundaryGuardSequenceCount")) for row in rows}
        aircraft_ids = {_positive_int(row.get("aircraftID")) for row in rows}
        if versions != {BOUNDARY_GUARD_LOOP_VERSION}:
            fail("unsupported or inconsistent loop version")
            continue
        if len(aircraft_ids) != 1 or 0 in aircraft_ids:
            fail("all children must belong to the same positive aircraftID")
            continue
        if len(counts) != 1 or 0 in counts:
            fail("inconsistent sequence count")
            continue
        declared_count = next(iter(counts))
        if declared_count != len(rows):
            fail(f"declares {declared_count} children but contains {len(rows)}")
            continue

        by_sequence: Dict[int, MutableMapping[str, Any]] = {}
        invalid_sequence = False
        for row in rows:
            sequence = _positive_int(row.get("boundaryGuardSequence"))
            if sequence <= 0 or sequence > declared_count or sequence in by_sequence:
                invalid_sequence = True
                break
            by_sequence[sequence] = row
        if invalid_sequence or set(by_sequence) != set(range(1, declared_count + 1)):
            fail("sequence must be unique and contiguous from 1")
            continue

        ordered = [by_sequence[index] for index in range(1, declared_count + 1)]
        durations = {
            _positive_float(
                row.get("boundaryGuardDurationS"),
                BOUNDARY_GUARD_DEFAULT_DURATION_S,
            )
            for row in ordered
        }
        if len(durations) != 1:
            fail("inconsistent boundaryGuardDurationS")
            continue
        duration_s = next(iter(durations))
        paths: List[List[MutableMapping[str, Any]]] = []
        waypoint_ids: set[int] = set()
        invalid_waypoints = False
        for row in ordered:
            waypoints = _waypoint_list(row)
            ids = [_waypoint_id(waypoint) for waypoint in waypoints]
            if not waypoints or any(value <= 0 for value in ids):
                invalid_waypoints = True
                break
            if waypoint_ids.intersection(ids) or len(set(ids)) != len(ids):
                invalid_waypoints = True
                break
            waypoint_ids.update(ids)
            paths.append(waypoints)
        if invalid_waypoints:
            fail("children require non-empty, globally unique positive waypoint IDs")
            continue

        cycle_first = _waypoint_id(paths[0][0])
        cycle_last = _waypoint_id(paths[-1][-1])
        for index, waypoints in enumerate(paths):
            next_first = (
                _waypoint_id(paths[index + 1][0])
                if index + 1 < len(paths)
                else cycle_first
            )
            _set_next_waypoint_id(waypoints[-1], next_first)
            ordered[index]["boundaryGuardCycleFirstWaypointID"] = int(cycle_first)
            ordered[index]["boundaryGuardCycleLastWaypointID"] = int(cycle_last)
        _set_boundary_guard_cycle_eta(paths[-1][-1], duration_s)

        summary[set_id] = {
            "sequenceCount": int(declared_count),
            "cycleFirstWaypointID": int(cycle_first),
            "cycleLastWaypointID": int(cycle_last),
        }
    return summary


def resequence_boundary_guard_flight_path_sets(
    payloads: Iterable[MutableMapping[str, Any]],
) -> None:
    """Renumber an explicitly remaining subset before a resume replan links it.

    Initial and direct next-collab generation are strict and must already emit
    the full declared set.  Resume pipelines, however, may intentionally remove
    completed children before their final path transform.  In that one context
    the surviving authoritative loop rows become the new 1..N owner set.
    """

    grouped = _group_boundary_guard_payloads(payloads)
    for set_id, rows in grouped.items():
        rows.sort(
            key=lambda row: (
                _positive_int(row.get("boundaryGuardSequence")) or 2**31,
                _positive_int(row.get("pathID")) or 2**31,
            )
        )
        duration_s = next(
            (
                _positive_float(
                    row.get("boundaryGuardDurationS"),
                    BOUNDARY_GUARD_DEFAULT_DURATION_S,
                )
                for row in rows
                if row.get("boundaryGuardDurationS") is not None
            ),
            BOUNDARY_GUARD_DEFAULT_DURATION_S,
        )
        annotate_boundary_guard_set(
            rows,
            set_id=set_id,
            duration_s=duration_s,
        )


def finalize_boundary_guard_flight_path_sets_in_mission_order(
    missions: Iterable[MutableMapping[str, Any]],
    flight_paths: Iterable[MutableMapping[str, Any]],
    *,
    strict: bool = True,
) -> Dict[str, Dict[str, int]]:
    """Rebuild remaining guard subsets in executable IMP mission order.

    Resume pipelines can split the active guard child into a new path and clone
    only the later children.  The clone helper cannot know about that separately
    built resume path, so independently resequencing the clone subset produces
    mixed ``sequenceCount`` values.  This finalizer treats the supplied IMP
    order as authoritative, renumbers every supplied child as one remaining
    owner set, links the new waypoint cycle, and synchronizes it back to IMP.
    """

    mission_rows = [
        row for row in missions if isinstance(row, MutableMapping)
    ]
    path_rows = [
        row for row in flight_paths if isinstance(row, MutableMapping)
    ]
    path_by_id: Dict[int, MutableMapping[str, Any]] = {}
    for path in path_rows:
        path_id = _positive_int(path.get("pathID"))
        if path_id <= 0:
            continue
        if path_id in path_by_id:
            if strict:
                raise ValueError(f"duplicate boundary guard pathID {path_id}")
            continue
        path_by_id[path_id] = path

    ordered_by_set: Dict[str, List[MutableMapping[str, Any]]] = defaultdict(list)
    seen_path_ids: set[int] = set()
    for mission in mission_rows:
        contract = extract_boundary_guard_contract(
            mission,
            mission.get("individualMissionInfo"),
        )
        if not is_boundary_guard_loop(contract):
            continue
        set_id = str(contract.get("boundaryGuardSetID") or "").strip()
        path_id = _positive_int(mission.get("pathID"))
        path = path_by_id.get(path_id)
        if path is None:
            if strict:
                raise ValueError(
                    f"boundary guard set {set_id}: mission path {path_id} "
                    "is not present in the supplied remaining flight paths"
                )
            continue
        if path_id in seen_path_ids:
            if strict:
                raise ValueError(
                    f"boundary guard set {set_id}: duplicate mission path {path_id}"
                )
            continue
        seen_path_ids.add(path_id)
        ordered_by_set[set_id].append(path)

    supplied_guard_path_ids = {
        _positive_int(path.get("pathID"))
        for path in path_rows
        if is_boundary_guard_loop(path)
    }
    missing_missions = sorted(
        path_id
        for path_id in supplied_guard_path_ids
        if path_id > 0 and path_id not in seen_path_ids
    )
    if missing_missions and strict:
        raise ValueError(
            "boundary guard flight path(s) missing from supplied mission order: "
            + ", ".join(str(path_id) for path_id in missing_missions)
        )

    for set_id, rows in ordered_by_set.items():
        if not rows:
            continue
        duration_s = next(
            (
                _positive_float(
                    row.get("boundaryGuardDurationS"),
                    BOUNDARY_GUARD_DEFAULT_DURATION_S,
                )
                for row in rows
                if row.get("boundaryGuardDurationS") is not None
            ),
            BOUNDARY_GUARD_DEFAULT_DURATION_S,
        )
        annotate_boundary_guard_set(
            rows,
            set_id=set_id,
            duration_s=duration_s,
        )

    ordered_paths = [
        path
        for rows in ordered_by_set.values()
        for path in rows
    ]
    summary = link_boundary_guard_flight_path_sets(
        ordered_paths,
        strict=strict,
    )
    sync_boundary_guard_contract_from_flight_paths(
        mission_rows,
        ordered_paths,
    )
    return summary


def sync_boundary_guard_contract_from_flight_paths(
    missions: Iterable[MutableMapping[str, Any]],
    flight_paths: Iterable[Mapping[str, Any]],
) -> None:
    """Copy finalized (including waypoint-ID) contract back to IMP rows."""

    contract_by_path: Dict[int, Dict[str, Any]] = {}
    for payload in flight_paths:
        if not isinstance(payload, Mapping):
            continue
        path_id = _positive_int(payload.get("pathID"))
        contract = extract_boundary_guard_contract(payload)
        if path_id > 0 and is_boundary_guard_loop(contract):
            contract_by_path[path_id] = contract

    for mission in missions:
        path_id = _positive_int(mission.get("pathID"))
        contract = contract_by_path.get(path_id)
        if contract:
            apply_boundary_guard_contract(
                mission,
                contract,
                include_individual_mission_info=True,
            )


def validate_boundary_guard_flight_path_sets(
    payloads: Iterable[Mapping[str, Any]],
) -> None:
    """Validate contract shape and exact cross-path tail targets read-only."""

    copies: List[MutableMapping[str, Any]] = []
    original_tail_targets: Dict[Tuple[str, int], int] = {}
    original_tail_etas: Dict[Tuple[str, int], float | None] = {}
    original_cycle_ids: Dict[Tuple[str, int], Tuple[int, int]] = {}
    for payload in payloads:
        if not isinstance(payload, Mapping) or not is_boundary_guard_loop(payload):
            continue
        shallow = dict(payload)
        waypoints = _waypoint_list(payload)
        copied_waypoints = [dict(row) for row in waypoints]
        if "waypointList" in payload:
            shallow["waypointList"] = copied_waypoints
        else:
            shallow["wplist"] = copied_waypoints
        set_id = str(payload.get("boundaryGuardSetID") or "").strip()
        sequence = _positive_int(payload.get("boundaryGuardSequence"))
        original_cycle_ids[(set_id, sequence)] = (
            _positive_int(payload.get("boundaryGuardCycleFirstWaypointID")),
            _positive_int(payload.get("boundaryGuardCycleLastWaypointID")),
        )
        if waypoints:
            original_tail_targets[(set_id, sequence)] = _positive_int(
                waypoints[-1].get("nextWaypointID")
            )
            try:
                original_tail_etas[(set_id, sequence)] = float(
                    waypoints[-1].get("eta")
                )
            except (TypeError, ValueError):
                original_tail_etas[(set_id, sequence)] = None
        copies.append(shallow)

    if not copies:
        return
    summary = link_boundary_guard_flight_path_sets(copies, strict=True)
    for copy_payload in copies:
        set_id = str(copy_payload.get("boundaryGuardSetID") or "").strip()
        sequence = _positive_int(copy_payload.get("boundaryGuardSequence"))
        waypoints = _waypoint_list(copy_payload)
        expected_tail = _positive_int(waypoints[-1].get("nextWaypointID")) if waypoints else 0
        actual_tail = original_tail_targets.get((set_id, sequence), 0)
        if actual_tail != expected_tail:
            raise ValueError(
                f"boundary guard set {set_id}: sequence {sequence} tail points to "
                f"{actual_tail}, expected {expected_tail}"
            )
        expected_eta = None
        try:
            expected_eta = float(waypoints[-1].get("eta")) if waypoints else None
        except (TypeError, ValueError):
            expected_eta = None
        actual_eta = original_tail_etas.get((set_id, sequence))
        if actual_eta != expected_eta:
            raise ValueError(
                f"boundary guard set {set_id}: sequence {sequence} tail ETA is "
                f"{actual_eta}, expected {expected_eta}"
            )
        expected = summary.get(set_id) or {}
        first, last = original_cycle_ids.get((set_id, sequence), (0, 0))
        if first != int(expected.get("cycleFirstWaypointID") or 0):
            raise ValueError(f"boundary guard set {set_id}: cycle-first ID mismatch")
        if last != int(expected.get("cycleLastWaypointID") or 0):
            raise ValueError(f"boundary guard set {set_id}: cycle-last ID mismatch")
