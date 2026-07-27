from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, Optional

from modules.common import db_paths


UINT32_MAX = 2**32 - 1
ICD_LINE_WIDTH_MAX = 50_000
_FLIGHT_PATH_WAYPOINT_LIST_KEYS = (
    "waypointList",
    "uavWaypointList",
    "lahWaypointList",
)

# Slowest transit a waypoint may be emitted with. Zero is not a valid
# command - stops are expressed by ``hovering``/``loiter`` instead.
_MIN_TRANSIT_SPEED_MPS = 24.0


class ReplanValidationError(RuntimeError):
    def __init__(self, errors: Iterable[str]) -> None:
        self.errors = [str(error) for error in errors]
        super().__init__("; ".join(self.errors))


def to_int(value: Any) -> Optional[int]:
    try:
        if value is None or value == "":
            return None
        return int(value)
    except Exception:
        return None


def path_band_for_aircraft(aircraft_id: int) -> tuple[int, int]:
    aid = int(aircraft_id)
    return aid * 100_000_000, aid * 100_000_000 + 99_999_999


def is_aircraft_path_id(aircraft_id: int, path_id: int) -> bool:
    start, end = path_band_for_aircraft(int(aircraft_id))
    return start <= int(path_id) <= end


def validate_uint32(value: Any, field: str, errors: list[str], *, context: str = "") -> Optional[int]:
    value_int = to_int(value)
    prefix = f"{context}: " if context else ""
    if value_int is None:
        errors.append(f"{prefix}{field} is not an int: {value!r}")
        return None
    if value_int < 0 or value_int > UINT32_MAX:
        errors.append(f"{prefix}{field} out of uint32 range: {value_int}")
    return value_int


def _payload_id(payload: dict, *keys: str) -> Optional[int]:
    for key in keys:
        value = to_int(payload.get(key))
        if value is not None:
            return int(value)
    return None


def _waypoint_list(payload: dict) -> list[dict]:
    rows = payload.get("waypointList")
    if isinstance(rows, list):
        return [row for row in rows if isinstance(row, dict)]
    rows = payload.get("uavWaypointList")
    if isinstance(rows, list):
        return [row for row in rows if isinstance(row, dict)]
    rows = payload.get("lahWaypointList")
    if isinstance(rows, list):
        return [row for row in rows if isinstance(row, dict)]
    return []


def _flight_path_waypoint_lists(payload: dict) -> Iterable[tuple[str, list]]:
    """Yield each distinct top-level FlightPath waypoint array."""

    if not isinstance(payload, dict):
        return
    seen: set[int] = set()
    for key in _FLIGHT_PATH_WAYPOINT_LIST_KEYS:
        rows = payload.get(key)
        if not isinstance(rows, list) or id(rows) in seen:
            continue
        seen.add(id(rows))
        yield key, rows


def normalize_flight_path_waypoint_altitudes_inplace(payload: dict) -> int:
    """Quantize emitted FlightPath MSL altitudes to the ICD integer type.

    Internal route solvers may keep sub-metre precision.  This function is only
    for the 0303/0304 serialization boundary.  Invalid values (for example
    booleans, strings, NaN, or infinity) are deliberately left untouched so
    validation can reject them instead of silently inventing an altitude.
    """

    changed = 0
    for _list_key, rows in _flight_path_waypoint_lists(payload):
        for waypoint in rows:
            if not isinstance(waypoint, dict):
                continue
            coordinate = waypoint.get("coordinate")
            if not isinstance(coordinate, dict) or "altitude" not in coordinate:
                continue
            altitude = coordinate.get("altitude")
            if type(altitude) is int:
                continue
            if isinstance(altitude, (bool, str, bytes)):
                continue
            try:
                numeric = float(altitude)
            except (TypeError, ValueError, OverflowError):
                continue
            if not math.isfinite(numeric):
                continue
            coordinate["altitude"] = int(round(numeric))
            changed += 1
    return int(changed)


def normalize_flight_path_waypoint_speeds_inplace(
    payload: dict,
    *,
    minimum_speed_mps: float = _MIN_TRANSIT_SPEED_MPS,
) -> int:
    """Raise zero-speed waypoints to the transit floor at the ICD boundary.

    A stop is carried by ``hovering``/``loiter``; ``speed`` is the transit
    command, and a zero there is unflyable - the aircraft holds position at the
    waypoint it was given and drifts further off plan the longer it waits. A
    replan that emits a leading zero-speed leg therefore strands the aircraft,
    so the floor is applied to every emitted waypoint rather than trusting each
    producer to have filled it in.
    """

    floor = max(0.0, float(minimum_speed_mps))
    if floor <= 0.0:
        return 0
    changed = 0
    for _list_key, rows in _flight_path_waypoint_lists(payload):
        for waypoint in rows:
            if not isinstance(waypoint, dict) or "speed" not in waypoint:
                continue
            speed = waypoint.get("speed")
            if isinstance(speed, (bool, str, bytes)):
                continue
            try:
                numeric = float(speed)
            except (TypeError, ValueError, OverflowError):
                continue
            if not math.isfinite(numeric) or numeric >= floor:
                continue
            waypoint["speed"] = round(floor, 2)
            changed += 1
    return int(changed)


def _validate_waypoint_altitude_json_types(
    payload: dict,
    errors: list[str],
    *,
    context: str,
) -> None:
    """Reject non-integer waypoint altitudes in newly supplied FlightPaths."""

    for list_key, rows in _flight_path_waypoint_lists(payload):
        for waypoint_index, waypoint in enumerate(rows):
            if not isinstance(waypoint, dict):
                continue
            coordinate = waypoint.get("coordinate")
            if not isinstance(coordinate, dict) or "altitude" not in coordinate:
                continue
            altitude = coordinate.get("altitude")
            if type(altitude) is int:
                continue
            errors.append(
                f"{context}: FlightPath.{list_key}[{waypoint_index}]."
                f"coordinate.altitude must be an integer JSON value: {altitude!r}"
            )


def _validate_line_width_json_types(
    mission: dict,
    errors: list[str],
    *,
    context: str,
) -> None:
    """Reject LINE widths that cannot satisfy the ICD ``uint width`` field."""

    info = mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        return
    line_list = info.get("lineList")
    if not isinstance(line_list, list):
        return
    for line_index, line in enumerate(line_list):
        if not isinstance(line, dict) or "width" not in line:
            continue
        width = line.get("width")
        field = f"individualMissionInfo.lineList[{line_index}].width"
        # bool is a subclass of int in Python, but it is not a JSON integer
        # accepted for the ICD uint field.
        if type(width) is not int:
            errors.append(
                f"{context}: {field} must be an integer JSON value: {width!r}"
            )
            continue
        if width < 0 or width > ICD_LINE_WIDTH_MAX:
            errors.append(
                f"{context}: {field} out of uint range 0..{ICD_LINE_WIDTH_MAX}: {width}"
            )


def _load_db_payload(directory: str, payload_id: int) -> Optional[dict]:
    try:
        path = db_paths.get_db_subpath(directory, f"{int(payload_id)}.json")
        if not path.exists():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def _load_db_payload_from_directory(
    directory_path: Path,
    directory: str,
    payload_id: int,
) -> Optional[dict]:
    """Load an existing artifact from an already-resolved DB directory."""

    try:
        path = Path(directory_path) / f"{int(payload_id)}.json"
        if not path.exists():
            return None
        data = json.loads(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def _index_payloads(rows: Iterable[dict], *keys: str) -> Dict[int, dict]:
    result: Dict[int, dict] = {}
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        row_id = _payload_id(row, *keys)
        if row_id is not None:
            result[int(row_id)] = row
    return result


def validate_unique_flightpath_ids(
    *,
    flight_plans_0303: Iterable[dict] = (),
    flight_plans_0304: Iterable[dict] = (),
    scope: str = "replan",
) -> dict[str, Any]:
    """Fail if generated 0303/0304 FlightPath payloads reuse a pathID."""

    seen: Dict[int, str] = {}
    duplicates: set[int] = set()
    for channel, rows in (("0303", flight_plans_0303 or ()), ("0304", flight_plans_0304 or ())):
        for fp in rows:
            if not isinstance(fp, dict):
                continue
            path_id = to_int(fp.get("pathID"))
            if path_id is None or int(path_id) <= 0:
                continue
            pid = int(path_id)
            if pid in seen:
                duplicates.add(pid)
            else:
                seen[pid] = str(channel)
    if duplicates:
        summary = ", ".join(str(pid) for pid in sorted(duplicates))
        raise ReplanValidationError(
            [f"{scope}: FlightPath duplicate pathID(s) before write: {summary}"]
        )
    return {
        "scope": scope,
        "flightPaths": len(seen),
        "pathIDs": sorted(seen),
    }


def validate_mission_flightpath_links(
    *,
    missions: Iterable[dict] = (),
    flight_plans_0303: Iterable[dict] = (),
    flight_plans_0304: Iterable[dict] = (),
    scope: str = "replan",
) -> dict[str, Any]:
    """Validate mission pathID references against generated FlightPath payloads."""

    mission_by_path: Dict[int, list[tuple[int, int]]] = {}
    for mission in missions or ():
        if not isinstance(mission, dict):
            continue
        aircraft_id = to_int(mission.get("aircraftID"))
        mission_id = to_int(mission.get("individualMissionID"))
        path_id = to_int(mission.get("pathID"))
        if (
            aircraft_id is None
            or mission_id is None
            or path_id is None
            or int(aircraft_id) <= 0
            or int(mission_id) <= 0
            or int(path_id) <= 0
        ):
            continue
        mission_by_path.setdefault(int(path_id), []).append((int(aircraft_id), int(mission_id)))

    fp_by_path: Dict[int, dict] = {}
    for fp in list(flight_plans_0303 or ()) + list(flight_plans_0304 or ()):
        if not isinstance(fp, dict):
            continue
        path_id = to_int(fp.get("pathID"))
        if path_id is None or int(path_id) <= 0:
            continue
        fp_by_path[int(path_id)] = fp

    errors: list[str] = []
    missing = sorted(pid for pid in mission_by_path if pid not in fp_by_path)
    if missing:
        summary = ", ".join(str(pid) for pid in missing)
        errors.append(f"{scope}: FlightPath link missing pathID(s) before write: {summary}")

    mismatches: list[str] = []
    for pid, mission_refs in sorted(mission_by_path.items()):
        if len(mission_refs) != 1:
            continue
        aircraft_id, mission_id = mission_refs[0]
        fp = fp_by_path.get(pid)
        if not isinstance(fp, dict):
            continue
        fp_aircraft_id = to_int(fp.get("aircraftID"))
        fp_mission_id = to_int(fp.get("individualMissionID"))
        if fp_aircraft_id is not None and int(fp_aircraft_id) > 0 and int(fp_aircraft_id) != int(aircraft_id):
            mismatches.append(f"pathID={pid}:aircraft {fp_aircraft_id}!={aircraft_id}")
        if fp_mission_id is not None and int(fp_mission_id) > 0 and int(fp_mission_id) != int(mission_id):
            mismatches.append(f"pathID={pid}:mission {fp_mission_id}!={mission_id}")
    if mismatches:
        summary = "; ".join(mismatches[:8])
        if len(mismatches) > 8:
            summary += f"; +{len(mismatches) - 8} more"
        errors.append(f"{scope}: FlightPath mission link mismatch before write: {summary}")
    if errors:
        raise ReplanValidationError(errors)
    return {
        "scope": scope,
        "missionsWithPath": sum(len(refs) for refs in mission_by_path.values()),
        "flightPaths": len(fp_by_path),
        "missingPathIDs": [],
        "mismatches": [],
    }


def _mission_path_ids(individual_mission_plans: Iterable[dict]) -> set[int]:
    path_ids: set[int] = set()
    for pkg in individual_mission_plans or ():
        if not isinstance(pkg, dict):
            continue
        for mission in pkg.get("individualMissionList") or ():
            if not isinstance(mission, dict):
                continue
            path_id = to_int(mission.get("pathID"))
            if path_id is not None and int(path_id) > 0:
                path_ids.add(int(path_id))
    return path_ids


def sync_flight_plan_individual_mission_ids(
    *,
    individual_mission_plans: Iterable[dict] = (),
    flight_plans_0303: Iterable[dict] = (),
    flight_plans_0304: Iterable[dict] = (),
    scope: str = "replan",
) -> dict[str, Any]:
    """Mutate generated FlightPath payloads so pathID-linked mission IDs match IMP rows."""

    mission_id_by_path: Dict[int, int] = {}
    for pkg in individual_mission_plans or ():
        if not isinstance(pkg, dict):
            continue
        for mission in pkg.get("individualMissionList") or ():
            if not isinstance(mission, dict):
                continue
            path_id = to_int(mission.get("pathID"))
            mission_id = to_int(mission.get("individualMissionID"))
            if path_id is None or int(path_id) <= 0 or mission_id is None or int(mission_id) <= 0:
                continue
            mission_id_by_path[int(path_id)] = int(mission_id)

    fixed = 0
    touched_path_ids: list[int] = []
    for fp in list(flight_plans_0303 or ()) + list(flight_plans_0304 or ()):
        if not isinstance(fp, dict):
            continue
        path_id = to_int(fp.get("pathID"))
        if path_id is None or int(path_id) not in mission_id_by_path:
            continue
        desired_id = int(mission_id_by_path[int(path_id)])
        if to_int(fp.get("individualMissionID")) != desired_id:
            fp["individualMissionID"] = desired_id
            fixed += 1
            touched_path_ids.append(int(path_id))
    return {
        "scope": scope,
        "fixed": fixed,
        "pathIDs": sorted(touched_path_ids),
    }


def collect_missing_flight_path_repairs(
    *,
    flight_path_dir: Path,
    individual_mission_plans: Iterable[dict] = (),
    flight_plans_0303: Iterable[dict] = (),
    flight_plans_0304: Iterable[dict] = (),
    scope: str = "replan",
) -> dict[str, Any]:
    """Return repair write rows for referenced FlightPath files missing from disk."""

    dir_fp = Path(flight_path_dir)
    referenced_path_ids = _mission_path_ids(individual_mission_plans)
    missing_ids = sorted(
        pid for pid in referenced_path_ids if not (dir_fp / f"{int(pid)}.json").exists()
    )
    if not missing_ids:
        return {
            "scope": scope,
            "referencedPathIDs": sorted(referenced_path_ids),
            "missingPathIDs": [],
            "repairablePathIDs": [],
            "repairRows": [],
        }

    missing_id_set = set(int(pid) for pid in missing_ids)
    fp_lookup: Dict[int, dict] = {}
    for fp in list(flight_plans_0303 or ()) + list(flight_plans_0304 or ()):
        if not isinstance(fp, dict):
            continue
        path_id = to_int(fp.get("pathID"))
        if path_id is None or int(path_id) <= 0 or int(path_id) not in missing_id_set:
            continue
        fp_lookup.setdefault(int(path_id), copy.deepcopy(fp))

    repair_rows: list[tuple[Path, dict]] = []
    for path_id in missing_ids:
        payload = fp_lookup.get(int(path_id))
        if isinstance(payload, dict):
            repair_rows.append((dir_fp / f"{int(path_id)}.json", copy.deepcopy(payload)))
    return {
        "scope": scope,
        "referencedPathIDs": sorted(referenced_path_ids),
        "missingPathIDs": missing_ids,
        "repairablePathIDs": [int(path.stem) for path, _payload in repair_rows],
        "repairRows": repair_rows,
    }


def validate_generated_artifact_payloads(
    *,
    individual_mission_plans: Iterable[dict] = (),
    flight_paths: Iterable[dict] = (),
    scope: str = "replan",
    allow_existing_db_artifacts: bool = True,
    log: Optional[Callable[[str], None]] = None,
) -> dict[str, Any]:
    """Validate generated 0302/FlightPath payloads before partial artifact writes."""

    errors: list[str] = []
    imp_rows = [row for row in individual_mission_plans or () if isinstance(row, dict)]
    fp_rows = [row for row in flight_paths or () if isinstance(row, dict)]
    fp_by_id = _index_payloads(fp_rows, "pathID")
    supplied_fp_ids = set(int(pid) for pid in fp_by_id)
    referenced_path_ids: set[int] = set()
    mission_ids: dict[int, str] = {}
    mission_by_path: dict[int, tuple[int, int, int]] = {}
    waypoint_ids: dict[int, str] = {}

    for imp_index, imp_payload in enumerate(imp_rows):
        imp_id = validate_uint32(
            _payload_id(
                imp_payload,
                "individualMissionPackageID",
                "individualMissionPlanPackageID",
            ),
            "individualMissionPackageID",
            errors,
            context=f"{scope}: impIndex={imp_index}",
        )
        imp_aircraft_id = to_int(imp_payload.get("aircraftID"))
        if imp_aircraft_id is not None and int(imp_aircraft_id) not in {1, 2, 3, 4, 5, 6}:
            errors.append(f"{scope}: imp={imp_id} aircraftID out of supported range 1..6: {imp_aircraft_id}")
        mission_list = imp_payload.get("individualMissionList")
        if not isinstance(mission_list, list):
            errors.append(f"{scope}: IMP individualMissionList is not a list imp={imp_id}")
            continue
        for mission_index, mission in enumerate(mission_list):
            if not isinstance(mission, dict):
                continue
            mission_ctx = f"{scope}: imp={imp_id} missionIndex={mission_index}"
            _validate_line_width_json_types(mission, errors, context=mission_ctx)
            mission_aircraft_raw = mission.get("aircraftID")
            mission_aircraft = to_int(mission_aircraft_raw)
            if mission_aircraft is None and mission_aircraft_raw not in (None, ""):
                errors.append(f"{mission_ctx}: aircraftID is not an int: {mission_aircraft_raw!r}")
            if mission_aircraft is None:
                mission_aircraft = imp_aircraft_id
                if mission_aircraft is None:
                    errors.append(f"{mission_ctx}: aircraftID is not an int: {mission_aircraft_raw!r}")
            elif mission_aircraft < 0 or mission_aircraft > UINT32_MAX:
                errors.append(f"{mission_ctx}: aircraftID out of uint32 range: {mission_aircraft}")
            elif imp_aircraft_id is not None and int(mission_aircraft) != int(imp_aircraft_id):
                errors.append(
                    f"{mission_ctx}: mission aircraft mismatch impAircraft={imp_aircraft_id} "
                    f"missionAircraft={mission_aircraft}"
                )
            mission_id = validate_uint32(
                mission.get("individualMissionID"),
                "individualMissionID",
                errors,
                context=mission_ctx,
            )
            path_id = validate_uint32(mission.get("pathID"), "pathID", errors, context=mission_ctx)
            if mission_id is None or path_id is None:
                continue
            if mission_id in mission_ids:
                errors.append(
                    f"{mission_ctx}: duplicate individualMissionID={mission_id} first={mission_ids[mission_id]}"
                )
            else:
                mission_ids[int(mission_id)] = mission_ctx
            if mission_aircraft is not None and not is_aircraft_path_id(int(mission_aircraft), int(path_id)):
                errors.append(f"{mission_ctx}: pathID band mismatch aircraftID={mission_aircraft} pathID={path_id}")
            if path_id in mission_by_path:
                prev_aircraft, prev_mission, prev_imp = mission_by_path[int(path_id)]
                errors.append(
                    f"{mission_ctx}: duplicate pathID={path_id} "
                    f"previousAircraft={prev_aircraft} previousMission={prev_mission} previousImp={prev_imp}"
                )
            else:
                mission_by_path[int(path_id)] = (
                    int(mission_aircraft or 0),
                    int(mission_id),
                    int(imp_id or 0),
                )
            referenced_path_ids.add(int(path_id))
            fp_payload = fp_by_id.get(int(path_id))
            if fp_payload is None and allow_existing_db_artifacts:
                fp_payload = _load_db_payload("FlightPath", int(path_id))
                if fp_payload is not None:
                    fp_by_id[int(path_id)] = fp_payload
            if fp_payload is None:
                errors.append(
                    f"{mission_ctx}: FlightPath missing pathID={path_id} "
                    f"aircraft={mission_aircraft} inputMissionID={mission.get('inputMissionID')}"
                )
                continue
            fp_path_id = validate_uint32(fp_payload.get("pathID"), "FlightPath.pathID", errors, context=mission_ctx)
            fp_aircraft_id = to_int(fp_payload.get("aircraftID"))
            fp_mission_id = to_int(fp_payload.get("individualMissionID"))
            if fp_path_id is not None and int(fp_path_id) != int(path_id):
                errors.append(f"{mission_ctx}: FlightPath payload pathID mismatch expected={path_id} actual={fp_path_id}")
            if fp_aircraft_id is not None and mission_aircraft is not None and int(fp_aircraft_id) != int(mission_aircraft):
                errors.append(
                    f"{mission_ctx}: FlightPath aircraft mismatch pathID={path_id} "
                    f"fp={fp_aircraft_id} mission={mission_aircraft}"
                )
            if fp_mission_id is not None and int(fp_mission_id) != int(mission_id):
                errors.append(
                    f"{mission_ctx}: FlightPath mission mismatch pathID={path_id} "
                    f"fp={fp_mission_id} mission={mission_id}"
                )

    orphan_paths = sorted(pid for pid in supplied_fp_ids if pid not in referenced_path_ids)
    if orphan_paths:
        errors.append(f"{scope}: orphan FlightPath payload(s): {orphan_paths[:12]}")

    for path_id, fp_payload in sorted(fp_by_id.items()):
        if int(path_id) in supplied_fp_ids:
            _validate_waypoint_altitude_json_types(
                fp_payload,
                errors,
                context=f"{scope}: pathID={path_id}",
            )
        waypoints = _waypoint_list(fp_payload)
        local_ids: set[int] = set()
        for waypoint_index, waypoint in enumerate(waypoints):
            waypoint_ctx = f"{scope}: pathID={path_id} waypointIndex={waypoint_index}"
            waypoint_id = validate_uint32(waypoint.get("waypointID"), "waypointID", errors, context=waypoint_ctx)
            if waypoint_id is None or waypoint_id <= 0:
                continue
            if waypoint_id in waypoint_ids:
                errors.append(
                    f"{waypoint_ctx}: duplicate waypointID={waypoint_id} first={waypoint_ids[waypoint_id]}"
                )
            else:
                waypoint_ids[int(waypoint_id)] = waypoint_ctx
            local_ids.add(int(waypoint_id))
        for waypoint_index, waypoint in enumerate(waypoints):
            next_id = to_int(waypoint.get("nextWaypointID"))
            if next_id is None or next_id == 0:
                continue
            if int(next_id) not in local_ids:
                errors.append(f"{scope}: pathID={path_id} waypointIndex={waypoint_index} invalid nextWaypointID={next_id}")

    if errors:
        if log:
            log(f"[VALIDATION][ERR] {scope}: {len(errors)} error(s): {'; '.join(errors[:4])}")
        raise ReplanValidationError(errors)

    summary = {
        "scope": scope,
        "individualMissionPackages": len(imp_rows),
        "individualMissions": len(mission_ids),
        "flightPaths": len(referenced_path_ids),
        "suppliedFlightPaths": len(supplied_fp_ids),
        "waypoints": len(waypoint_ids),
    }
    if log:
        log(f"[VALIDATION] {scope}: {summary}")
    return summary


def validate_replan_payloads(
    *,
    mission_plan: dict,
    individual_mission_plans: Iterable[dict] = (),
    flight_paths: Iterable[dict] = (),
    scope: str = "replan",
    allow_existing_db_artifacts: bool = True,
    validate_existing_flight_path_waypoints: bool = True,
    validate_existing_flight_path_links: bool = True,
    log: Optional[Callable[[str], None]] = None,
) -> dict[str, Any]:
    """Validate 0301/0302/0303/0304 link invariants before artifact write."""

    errors: list[str] = []
    mp = mission_plan if isinstance(mission_plan, dict) else {}
    mission_plan_id = validate_uint32(mp.get("missionPlanID"), "missionPlanID", errors, context=scope)
    imp_by_id = _index_payloads(
        [row for row in individual_mission_plans or [] if isinstance(row, dict)],
        "individualMissionPackageID",
        "individualMissionPlanPackageID",
    )
    fp_by_id = _index_payloads(
        [row for row in flight_paths or [] if isinstance(row, dict)],
        "pathID",
    )
    supplied_fp_ids = set(int(pid) for pid in fp_by_id)
    referenced_imp_ids: set[int] = set()
    referenced_path_ids: set[int] = set()
    mission_by_path: dict[int, tuple[int, int, int]] = {}
    mission_ids: dict[int, str] = {}
    waypoint_ids: dict[int, str] = {}
    resolved_db_directories: dict[str, Path] = {}

    def _resolved_db_directory(directory: str) -> Path | None:
        cached = resolved_db_directories.get(str(directory))
        if cached is not None:
            return cached
        try:
            resolved = Path(db_paths.get_db_subpath(str(directory)))
        except Exception:
            return None
        resolved_db_directories[str(directory)] = resolved
        return resolved

    def _load_existing_db_payload(directory: str, payload_id: int) -> Optional[dict]:
        resolved = _resolved_db_directory(str(directory))
        if resolved is not None:
            return _load_db_payload_from_directory(
                resolved,
                str(directory),
                int(payload_id),
            )
        # Preserve the original per-file resolution fallback if resolving the
        # directory failed transiently.
        return _load_db_payload(str(directory), int(payload_id))

    trusted_existing_fp_dir: Path | None = None
    if allow_existing_db_artifacts and not validate_existing_flight_path_links:
        # Resolve the active DB once. Calling get_db_subpath for every one of
        # the 50+ preserved paths repeatedly refreshes scenario state and
        # dominates final validation latency.
        trusted_existing_fp_dir = _resolved_db_directory("FlightPath")

    aircraft_list = mp.get("aircraftList")
    if not isinstance(aircraft_list, list):
        errors.append(f"{scope}: MissionPlan.aircraftList is not a list")
        aircraft_list = []

    for aircraft_idx, aircraft in enumerate(aircraft_list):
        if not isinstance(aircraft, dict):
            continue
        ctx = f"{scope}: plan={mission_plan_id} aircraftIndex={aircraft_idx}"
        aircraft_id = validate_uint32(aircraft.get("aircraftID"), "aircraftID", errors, context=ctx)
        if aircraft_id is not None and aircraft_id not in {1, 2, 3, 4, 5, 6}:
            errors.append(f"{ctx}: aircraftID out of supported range 1..6: {aircraft_id}")
        imp_id = _payload_id(
            aircraft,
            "individualMissionPackageID",
            "individualMissionPlanPackageID",
            "individualMissionPackageId",
        )
        if imp_id is None or imp_id <= 0:
            errors.append(f"{ctx}: missing individualMissionPackageID")
            continue
        referenced_imp_ids.add(int(imp_id))
        imp_payload = imp_by_id.get(int(imp_id))
        if imp_payload is None and allow_existing_db_artifacts:
            imp_payload = _load_existing_db_payload("IndividualMissionPlan", int(imp_id))
            if imp_payload is not None:
                imp_by_id[int(imp_id)] = imp_payload
        if imp_payload is None:
            errors.append(f"{ctx}: IMP file/payload missing imp={imp_id} aircraft={aircraft_id}")
            continue
        imp_aircraft_id = to_int(imp_payload.get("aircraftID"))
        if aircraft_id is not None and imp_aircraft_id is not None and int(imp_aircraft_id) != int(aircraft_id):
            errors.append(
                f"{ctx}: IMP aircraft mismatch imp={imp_id} planAircraft={aircraft_id} impAircraft={imp_aircraft_id}"
            )
        mission_list = imp_payload.get("individualMissionList")
        if not isinstance(mission_list, list):
            errors.append(f"{ctx}: IMP individualMissionList is not a list imp={imp_id}")
            continue
        for mission_idx, mission in enumerate(mission_list):
            if not isinstance(mission, dict):
                continue
            mission_ctx = f"{ctx} imp={imp_id} missionIndex={mission_idx}"
            _validate_line_width_json_types(mission, errors, context=mission_ctx)
            mission_aircraft_raw = mission.get("aircraftID")
            mission_aircraft = to_int(mission_aircraft_raw)
            if mission_aircraft is None and mission_aircraft_raw not in (None, ""):
                errors.append(f"{mission_ctx}: aircraftID is not an int: {mission_aircraft_raw!r}")
            if mission_aircraft is None:
                mission_aircraft = aircraft_id
                if mission_aircraft is None:
                    errors.append(f"{mission_ctx}: aircraftID is not an int: {mission_aircraft_raw!r}")
            elif mission_aircraft < 0 or mission_aircraft > UINT32_MAX:
                errors.append(f"{mission_ctx}: aircraftID out of uint32 range: {mission_aircraft}")
            elif aircraft_id is not None and int(mission_aircraft) != int(aircraft_id):
                errors.append(
                    f"{mission_ctx}: mission aircraft mismatch planAircraft={aircraft_id} missionAircraft={mission_aircraft}"
                )
            mission_id = validate_uint32(mission.get("individualMissionID"), "individualMissionID", errors, context=mission_ctx)
            path_id = validate_uint32(mission.get("pathID"), "pathID", errors, context=mission_ctx)
            if mission_id is None or path_id is None:
                continue
            if mission_id in mission_ids:
                errors.append(
                    f"{mission_ctx}: duplicate individualMissionID={mission_id} first={mission_ids[mission_id]}"
                )
            else:
                mission_ids[int(mission_id)] = mission_ctx
            if mission_aircraft is not None and not is_aircraft_path_id(int(mission_aircraft), int(path_id)):
                errors.append(
                    f"{mission_ctx}: pathID band mismatch aircraftID={mission_aircraft} pathID={path_id}"
                )
            if path_id in mission_by_path:
                prev_aircraft, prev_mission, prev_imp = mission_by_path[int(path_id)]
                errors.append(
                    f"{mission_ctx}: duplicate pathID={path_id} "
                    f"previousAircraft={prev_aircraft} previousMission={prev_mission} previousImp={prev_imp}"
                )
            else:
                mission_by_path[int(path_id)] = (
                    int(mission_aircraft or 0),
                    int(mission_id),
                    int(imp_id),
                )
            referenced_path_ids.add(int(path_id))
            fp_payload = fp_by_id.get(int(path_id))
            existing_fp_link_trusted = False
            if fp_payload is None and allow_existing_db_artifacts:
                if (
                    not validate_existing_flight_path_links
                    and int(path_id) not in supplied_fp_ids
                ):
                    try:
                        existing_fp_link_trusted = bool(
                            trusted_existing_fp_dir is not None
                            and (
                                trusted_existing_fp_dir
                                / f"{int(path_id)}.json"
                            ).is_file()
                        )
                    except Exception:
                        existing_fp_link_trusted = False
                else:
                    fp_payload = _load_existing_db_payload("FlightPath", int(path_id))
                    if fp_payload is not None:
                        fp_by_id[int(path_id)] = fp_payload
            if existing_fp_link_trusted:
                # The source IMP/path relationship is unchanged and was
                # already validated before this replan.  Existence is enough
                # here; newly supplied paths still receive the full checks.
                continue
            if fp_payload is None:
                errors.append(
                    f"{mission_ctx}: FlightPath missing pathID={path_id} aircraft={mission_aircraft} inputMissionID={mission.get('inputMissionID')}"
                )
                continue
            fp_path_id = validate_uint32(fp_payload.get("pathID"), "FlightPath.pathID", errors, context=mission_ctx)
            fp_aircraft = to_int(fp_payload.get("aircraftID"))
            fp_mission = to_int(fp_payload.get("individualMissionID"))
            if fp_path_id is not None and int(fp_path_id) != int(path_id):
                errors.append(f"{mission_ctx}: FlightPath payload pathID mismatch expected={path_id} actual={fp_path_id}")
            if fp_aircraft is not None and mission_aircraft is not None and int(fp_aircraft) != int(mission_aircraft):
                errors.append(f"{mission_ctx}: FlightPath aircraft mismatch pathID={path_id} fp={fp_aircraft} mission={mission_aircraft}")
            if fp_mission is not None and int(fp_mission) != int(mission_id):
                errors.append(f"{mission_ctx}: FlightPath mission mismatch pathID={path_id} fp={fp_mission} mission={mission_id}")

    orphan_paths = sorted(pid for pid in supplied_fp_ids if pid not in referenced_path_ids)
    if orphan_paths:
        errors.append(f"{scope}: orphan FlightPath payload(s): {orphan_paths[:12]}")

    for path_id, fp_payload in sorted(fp_by_id.items()):
        if not validate_existing_flight_path_waypoints and int(path_id) not in supplied_fp_ids:
            continue
        if int(path_id) in supplied_fp_ids:
            _validate_waypoint_altitude_json_types(
                fp_payload,
                errors,
                context=f"{scope}: pathID={path_id}",
            )
        waypoints = _waypoint_list(fp_payload)
        local_ids: set[int] = set()
        for wp_idx, waypoint in enumerate(waypoints):
            waypoint_ctx = f"{scope}: pathID={path_id} waypointIndex={wp_idx}"
            waypoint_id = validate_uint32(waypoint.get("waypointID"), "waypointID", errors, context=waypoint_ctx)
            if waypoint_id is None or waypoint_id <= 0:
                continue
            if waypoint_id in waypoint_ids:
                errors.append(
                    f"{waypoint_ctx}: duplicate waypointID={waypoint_id} first={waypoint_ids[waypoint_id]}"
                )
            else:
                waypoint_ids[int(waypoint_id)] = waypoint_ctx
            local_ids.add(int(waypoint_id))
        for wp_idx, waypoint in enumerate(waypoints):
            next_id = to_int(waypoint.get("nextWaypointID"))
            if next_id is None or next_id == 0:
                continue
            if int(next_id) not in local_ids:
                errors.append(
                    f"{scope}: pathID={path_id} waypointIndex={wp_idx} invalid nextWaypointID={next_id}"
                )

    if errors:
        if log:
            log(f"[VALIDATION][ERR] {scope}: {len(errors)} error(s): {'; '.join(errors[:4])}")
        raise ReplanValidationError(errors)

    summary = {
        "scope": scope,
        "missionPlanID": mission_plan_id,
        "aircraft": len(aircraft_list),
        "individualMissionPackages": len(referenced_imp_ids),
        "individualMissions": len(mission_ids),
        "flightPaths": len(referenced_path_ids),
        "waypoints": len(waypoint_ids),
    }
    if log:
        log(f"[VALIDATION] {scope}: {summary}")
    return summary
