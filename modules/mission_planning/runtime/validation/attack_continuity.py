from __future__ import annotations

from collections import Counter
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

from modules.common import db_paths
from modules.mission_planning.runtime.cache.source_artifacts import read_json_cached


def _to_int(value: Any) -> Optional[int]:
    try:
        if value is None or isinstance(value, bool):
            return None
        return int(value)
    except (TypeError, ValueError):
        return None


def _index_payloads(payloads: Iterable[Dict[str, Any]], key: str) -> Dict[int, Dict[str, Any]]:
    indexed: Dict[int, Dict[str, Any]] = {}
    for payload in payloads or []:
        if not isinstance(payload, dict):
            continue
        payload_id = _to_int(payload.get(key))
        if payload_id is not None and payload_id > 0:
            indexed[int(payload_id)] = payload
    return indexed


def collect_lah_attack_rows(
    mission_plan: Dict[str, Any],
    *,
    individual_mission_plans: Iterable[Dict[str, Any]] = (),
    flight_paths: Iterable[Dict[str, Any]] = (),
    manned_aircraft_ids: Sequence[int] = (2, 3),
) -> Tuple[List[Dict[str, int]], List[str]]:
    """Collect target-bearing LAH attack waypoints referenced by a plan.

    Supplied IMP/FlightPath payloads take precedence over disk.  That lets the
    attack planner validate a candidate graph before committing it, while the
    post-attack planner can validate the same graph immediately after its
    deferred artifact batch is flushed.

    Only target-bound mission entries are opened.  Ordinary LAH transit paths
    can be large, and scanning them on every detection adds cost without
    improving the continuity check: generated attack missions always retain a
    positive ``individualMissionInfo.targetID`` as provenance.
    """

    if not isinstance(mission_plan, dict):
        return [], ["mission plan payload is unavailable"]

    imp_overrides = _index_payloads(individual_mission_plans, "individualMissionPackageID")
    path_overrides = _index_payloads(flight_paths, "pathID")
    allowed_aircraft_ids = {int(value) for value in manned_aircraft_ids}
    rows: List[Dict[str, int]] = []
    errors: List[str] = []
    seen_rows: set[Tuple[int, int, int, int, int, int]] = set()

    for aircraft_entry in mission_plan.get("aircraftList") or []:
        if not isinstance(aircraft_entry, dict):
            continue
        aircraft_id = _to_int(aircraft_entry.get("aircraftID"))
        if aircraft_id is None or int(aircraft_id) not in allowed_aircraft_ids:
            continue
        imp_id = _to_int(aircraft_entry.get("individualMissionPackageID"))
        if imp_id is None or imp_id <= 0:
            errors.append(f"LAH {aircraft_id} has no individualMissionPackageID")
            continue

        imp_payload = imp_overrides.get(int(imp_id))
        if imp_payload is None:
            try:
                imp_path = db_paths.get_db_subpath(
                    "IndividualMissionPlan", f"{int(imp_id)}.json"
                )
                imp_payload = read_json_cached(
                    Path(imp_path), copy_result=False, kind="IndividualMissionPlan"
                )
            except Exception as exc:
                errors.append(f"LAH {aircraft_id} IMP {imp_id} load failed: {exc}")
                continue

        missions = (
            imp_payload.get("individualMissionList")
            if isinstance(imp_payload, dict)
            else None
        )
        if not isinstance(missions, list):
            errors.append(f"LAH {aircraft_id} IMP {imp_id} has no mission list")
            continue

        for mission_index, mission in enumerate(missions):
            if not isinstance(mission, dict):
                continue
            # Completed attack missions are commonly retained as an execution
            # prefix in a later IMP.  They are historical evidence, not an
            # in-flight commitment, and must not consume attack capacity or be
            # required to survive another replan.
            if bool(mission.get("isDone")):
                continue
            info = mission.get("individualMissionInfo")
            mission_target_id = _to_int(
                (info or {}).get("targetID") if isinstance(info, dict) else None
            )
            if mission_target_id is None or mission_target_id <= 0:
                continue
            path_id = _to_int(mission.get("pathID"))
            mission_id = _to_int(mission.get("individualMissionID"))
            if path_id is None or path_id <= 0:
                errors.append(
                    f"LAH {aircraft_id} target {mission_target_id} mission has no pathID"
                )
                continue

            path_payload = path_overrides.get(int(path_id))
            if path_payload is None:
                try:
                    path = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
                    path_payload = read_json_cached(
                        Path(path), copy_result=False, kind="FlightPath"
                    )
                except Exception as exc:
                    errors.append(
                        f"LAH {aircraft_id} target {mission_target_id} path {path_id} "
                        f"load failed: {exc}"
                    )
                    continue

            for list_name in ("lahWaypointList", "waypointList", "uavWaypointList"):
                waypoints = (
                    path_payload.get(list_name)
                    if isinstance(path_payload, dict)
                    else None
                )
                if not isinstance(waypoints, list):
                    continue
                for waypoint_index, waypoint in enumerate(waypoints):
                    if not isinstance(waypoint, dict):
                        continue
                    # A mission can remain open while its firing waypoint is
                    # already complete (for example while executing the
                    # certified descent/cover suffix).  Preserve only attack
                    # commands that have not actually executed yet.
                    if bool(waypoint.get("isDone")):
                        continue
                    attack = waypoint.get("attack")
                    target_id = _to_int(
                        (attack or {}).get("targetID") if isinstance(attack, dict) else None
                    )
                    if target_id is None or target_id <= 0:
                        continue
                    waypoint_id = _to_int(waypoint.get("waypointID")) or 0
                    identity = (
                        int(aircraft_id),
                        int(imp_id),
                        int(mission_id or 0),
                        int(path_id),
                        int(waypoint_id),
                        int(target_id),
                    )
                    if identity in seen_rows:
                        continue
                    seen_rows.add(identity)
                    rows.append(
                        {
                            "aircraftID": int(aircraft_id),
                            "individualMissionPackageID": int(imp_id),
                            "individualMissionID": int(mission_id or 0),
                            "pathID": int(path_id),
                            "waypointID": int(waypoint_id),
                            "targetID": int(target_id),
                            "weaponType": int(_to_int((attack or {}).get("weaponType")) or 0),
                            "missionIndex": int(mission_index),
                            "waypointIndex": int(waypoint_index),
                        }
                    )
    return rows, errors


def attack_identity(row: Mapping[str, Any]) -> Tuple[int, int, int, int, int, int]:
    """Return the complete committed identity of one unfinished LAH attack."""

    return (
        int(_to_int(row.get("aircraftID")) or 0),
        int(_to_int(row.get("individualMissionPackageID")) or 0),
        int(_to_int(row.get("individualMissionID")) or 0),
        int(_to_int(row.get("pathID")) or 0),
        int(_to_int(row.get("waypointID")) or 0),
        int(_to_int(row.get("targetID")) or 0),
    )


def attack_pair(row: Mapping[str, Any]) -> Tuple[int, int]:
    return (
        int(_to_int(row.get("aircraftID")) or 0),
        int(_to_int(row.get("targetID")) or 0),
    )


def missing_attack_identities(
    source_rows: Iterable[Mapping[str, Any]],
    candidate_rows: Iterable[Mapping[str, Any]],
) -> List[Tuple[int, int, int, int, int, int]]:
    """Return source attack identities not preserved verbatim by a candidate."""

    source = Counter(attack_identity(row) for row in source_rows or [])
    candidate = Counter(attack_identity(row) for row in candidate_rows or [])
    return sorted((source - candidate).elements())


def evaluate_candidate_attack_continuity(
    *,
    expected_new_pairs: Iterable[Tuple[int, int]],
    candidate_pairs: Iterable[Tuple[int, int]],
    certified_deferred_pairs: Iterable[Tuple[int, int]] = (),
    scan_errors: Iterable[str] = (),
    missing_committed_identities: Iterable[Tuple[int, ...]] = (),
    committed_order_mismatches: Iterable[Mapping[str, Any]] = (),
) -> Dict[str, Any]:
    """Decide whether an attack candidate is safe to commit.

    Existing, not-yet-fired attacks remain a strict invariant.  New contacts
    are different: terrain or the LAH altitude envelope can make one contact
    temporarily unengageable while another contact is fully certified.  In
    that case the certified subset must be committed and only the unresolved
    pairs deferred for a later monitoring cycle.
    """

    expected = {
        (int(aircraft_id), int(target_id))
        for aircraft_id, target_id in expected_new_pairs or ()
    }
    candidate = {
        (int(aircraft_id), int(target_id))
        for aircraft_id, target_id in candidate_pairs or ()
    }
    certified_deferred = {
        (int(aircraft_id), int(target_id))
        for aircraft_id, target_id in certified_deferred_pairs or ()
    }
    errors = [str(item) for item in scan_errors or () if str(item)]
    missing_committed = [tuple(item) for item in missing_committed_identities or ()]
    order_mismatches = [dict(item) for item in committed_order_mismatches or ()]
    successful_new = sorted(expected & candidate)
    deferred_new = sorted(expected - candidate)
    uncertified_missing = sorted(set(deferred_new) - certified_deferred)
    structural_ok = not (
        errors or missing_committed or order_mismatches or uncertified_missing
    )
    # A request with no new pairs is a preservation-only candidate.  When new
    # pairs were requested, at least one real attack must survive; otherwise a
    # hold-only package must not be reported as an attack plan.
    has_attack_result = not expected or bool(successful_new)

    return {
        "ok": bool(structural_ok and has_attack_result),
        "structuralOk": bool(structural_ok),
        "hasAttackResult": bool(has_attack_result),
        "successfulNewPairs": successful_new,
        "deferredNewPairs": deferred_new,
        "certifiedDeferredPairs": sorted(set(deferred_new) & certified_deferred),
        "uncertifiedMissingPairs": uncertified_missing,
        "partialSuccess": bool(successful_new and deferred_new),
        "allNewUnengageable": bool(expected and not successful_new),
    }


def compare_post_attack_pairs(
    source_rows: Iterable[Mapping[str, Any]],
    candidate_rows: Iterable[Mapping[str, Any]],
    *,
    closed_target_ids: Iterable[int] = (),
) -> Dict[str, Any]:
    """Compare stable unfinished-attack identities after one target closes.

    Closing one target must not reset a second queued attack on the same LAH.
    Its non-attack descent geometry may remain, but no executable attack command
    for the closed target may survive in the candidate package.
    """

    closed = {int(value) for value in closed_target_ids}
    expected = Counter(
        attack_identity(row)
        for row in source_rows or []
        if int(_to_int(row.get("targetID")) or 0) not in closed
    )
    actual = Counter(
        attack_identity(row)
        for row in candidate_rows or []
        if int(_to_int(row.get("targetID")) or 0) not in closed
    )
    stale_closed = sorted(
        attack_identity(row)
        for row in candidate_rows or []
        if int(_to_int(row.get("targetID")) or 0) in closed
    )
    missing = sorted((expected - actual).elements())
    unexpected = sorted((actual - expected).elements())
    return {
        "ok": not missing and not unexpected and not stale_closed,
        "expected": sorted(expected.elements()),
        "actual": sorted(actual.elements()),
        "missing": missing,
        "unexpected": unexpected,
        "staleClosed": stale_closed,
    }
