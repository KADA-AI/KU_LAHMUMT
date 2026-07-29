from __future__ import annotations

from typing import Any


MAX_REPORTED_ISSUES = 80


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_bool(value: Any) -> bool:
    return isinstance(value, bool)


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _value_type(value: Any) -> str:
    if value is None:
        return "missing"
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, str):
        return "str"
    if isinstance(value, list):
        return "list"
    if isinstance(value, dict):
        return "dict"
    return type(value).__name__


def _get_ci(obj: dict[str, Any], *keys: str) -> Any:
    if not isinstance(obj, dict):
        return None
    lower = {str(k).lower(): k for k in obj.keys()}
    for key in keys:
        actual = lower.get(str(key).lower())
        if actual is not None:
            return obj.get(actual)
    return None


def _as_int(value: Any) -> int | None:
    if _is_int(value):
        return int(value)
    return None


def _waypoints(path_data: dict[str, Any]) -> list[dict[str, Any]]:
    for key in ("waypointList", "uavWaypointList", "lahWaypointList", "WaypointList"):
        value = path_data.get(key)
        if isinstance(value, list):
            return [item for item in value if isinstance(item, dict)]
    return []


def _boundary_guard_cross_path_targets(
    flight_paths: list[Any],
) -> dict[int, int]:
    """Return the one declared cross-path tail target allowed per guard path.

    A normal FlightPath must remain self-contained.  Type-2 boundary guard
    output is the sole exception: all child paths owned by one UAV form a
    versioned, ordered set and each child tail points to the next child (the
    final tail points back to the set's first waypoint).  Malformed or partial
    contracts deliberately produce no exception, so the normal validator
    warning remains visible.
    """

    groups: dict[tuple[int, str], list[dict[str, Any]]] = {}
    for raw in flight_paths:
        data = (
            raw.get("data")
            if isinstance(raw, dict) and isinstance(raw.get("data"), dict)
            else raw
        )
        if not isinstance(data, dict):
            continue
        if _get_ci(data, "boundaryGuardLoop", "boundary_guard_loop") is not True:
            continue
        set_id = str(
            _get_ci(data, "boundaryGuardSetID", "boundary_guard_set_id") or ""
        ).strip()
        path_id = _as_int(_get_ci(data, "pathID", "PathID"))
        aircraft_id = _as_int(_get_ci(data, "aircraftID", "AircraftID"))
        sequence = _as_int(
            _get_ci(data, "boundaryGuardSequence", "boundary_guard_sequence")
        )
        sequence_count = _as_int(
            _get_ci(
                data,
                "boundaryGuardSequenceCount",
                "boundary_guard_sequence_count",
            )
        )
        loop_version = _as_int(
            _get_ci(
                data,
                "boundaryGuardLoopVersion",
                "boundary_guard_loop_version",
            )
        )
        first_declared = _as_int(
            _get_ci(
                data,
                "boundaryGuardCycleFirstWaypointID",
                "boundary_guard_cycle_first_wp_id",
            )
        )
        last_declared = _as_int(
            _get_ci(
                data,
                "boundaryGuardCycleLastWaypointID",
                "boundary_guard_cycle_last_wp_id",
            )
        )
        ids = [
            int(waypoint_id)
            for waypoint in _waypoints(data)
            if (
                waypoint_id := _as_int(
                    _get_ci(waypoint, "waypointID", "WaypointID")
                )
            )
            is not None
            and int(waypoint_id) > 0
        ]
        if (
            not set_id
            or path_id is None
            or path_id <= 0
            or aircraft_id is None
            or aircraft_id <= 0
            or sequence is None
            or sequence <= 0
            or sequence_count is None
            or sequence_count <= 0
            or loop_version != 1
            or first_declared is None
            or last_declared is None
            or not ids
            or len(ids) != len(set(ids))
        ):
            continue
        groups.setdefault((int(aircraft_id), set_id), []).append(
            {
                "path_id": int(path_id),
                "sequence": int(sequence),
                "sequence_count": int(sequence_count),
                "first_declared": int(first_declared),
                "last_declared": int(last_declared),
                "waypoint_ids": ids,
            }
        )

    allowed: dict[int, int] = {}
    for rows in groups.values():
        declared_counts = {int(row["sequence_count"]) for row in rows}
        if len(declared_counts) != 1:
            continue
        declared_count = next(iter(declared_counts))
        if declared_count != len(rows):
            continue
        sequences = [int(row["sequence"]) for row in rows]
        if sorted(sequences) != list(range(1, declared_count + 1)):
            continue
        rows.sort(key=lambda row: int(row["sequence"]))
        waypoint_ids = [
            int(waypoint_id)
            for row in rows
            for waypoint_id in row["waypoint_ids"]
        ]
        if len(waypoint_ids) != len(set(waypoint_ids)):
            continue
        cycle_first = int(rows[0]["waypoint_ids"][0])
        cycle_last = int(rows[-1]["waypoint_ids"][-1])
        if any(
            int(row["first_declared"]) != cycle_first
            or int(row["last_declared"]) != cycle_last
            for row in rows
        ):
            continue
        for index, row in enumerate(rows):
            allowed[int(row["path_id"])] = (
                int(rows[index + 1]["waypoint_ids"][0])
                if index + 1 < len(rows)
                else cycle_first
            )
    return allowed


class _IssueCollector:
    def __init__(self) -> None:
        self.issues: list[dict[str, Any]] = []
        self.total_count = 0
        self.counts = {"error": 0, "warn": 0, "info": 0}

    def add(
        self,
        severity: str,
        code: str,
        path: str,
        message: str,
        *,
        expected: str | None = None,
        actual: Any = None,
    ) -> None:
        sev = severity if severity in self.counts else "warn"
        self.total_count += 1
        self.counts[sev] += 1
        if len(self.issues) >= MAX_REPORTED_ISSUES:
            return
        item: dict[str, Any] = {
            "severity": sev,
            "code": code,
            "path": path,
            "message": message,
        }
        if expected:
            item["expected"] = expected
        if actual is not None:
            item["actualType"] = _value_type(actual)
            if isinstance(actual, (str, int, float, bool)) or actual is None:
                item["actual"] = actual
        self.issues.append(item)

    def require_uint(
        self,
        obj: dict[str, Any],
        key: str,
        path: str,
        *,
        required: bool = True,
        min_value: int = 0,
        max_value: int | None = None,
    ) -> int | None:
        value = _get_ci(obj, key)
        field_path = f"{path}.{key}"
        if value is None:
            if required:
                self.add("error", "missing_uint", field_path, f"{key} is required.", expected="uint")
            return None
        if not _is_int(value):
            self.add(
                "error",
                "uint_type",
                field_path,
                f"{key} must be an integer JSON value.",
                expected="uint/int",
                actual=value,
            )
            return None
        value_int = int(value)
        if value_int < min_value or (max_value is not None and value_int > max_value):
            expected = f"{min_value}..{max_value}" if max_value is not None else f">= {min_value}"
            self.add(
                "warn",
                "uint_range",
                field_path,
                f"{key} is outside ICD range.",
                expected=expected,
                actual=value,
            )
        return value_int

    def require_bool(self, obj: dict[str, Any], key: str, path: str, *, required: bool = True) -> bool | None:
        value = _get_ci(obj, key)
        field_path = f"{path}.{key}"
        if value is None:
            if required:
                self.add("error", "missing_bool", field_path, f"{key} is required.", expected="bool")
            return None
        if not _is_bool(value):
            self.add(
                "warn",
                "bool_type",
                field_path,
                f"{key} should be a JSON bool, not 0/1/string.",
                expected="bool",
                actual=value,
            )
            return None
        return bool(value)

    def require_float(
        self,
        obj: dict[str, Any],
        key: str,
        path: str,
        *,
        required: bool = True,
        min_value: float | None = None,
        max_value: float | None = None,
    ) -> float | None:
        value = _get_ci(obj, key)
        field_path = f"{path}.{key}"
        if value is None:
            if required:
                self.add("error", "missing_float", field_path, f"{key} is required.", expected="float")
            return None
        if not _is_number(value):
            self.add(
                "error",
                "float_type",
                field_path,
                f"{key} must be numeric.",
                expected="float",
                actual=value,
            )
            return None
        if isinstance(value, int) and not isinstance(value, bool):
            self.add(
                "warn",
                "float_encoded_as_int",
                field_path,
                f"{key} is defined as float in ICD but encoded as int.",
                expected="float",
                actual=value,
            )
        value_float = float(value)
        if min_value is not None and value_float < min_value:
            self.add("warn", "float_range", field_path, f"{key} is below ICD range.", expected=f">= {min_value}", actual=value)
        if max_value is not None and value_float > max_value:
            self.add("warn", "float_range", field_path, f"{key} is above ICD range.", expected=f"<= {max_value}", actual=value)
        return value_float


def _validate_coordinate(col: _IssueCollector, coord: Any, path: str) -> None:
    if not isinstance(coord, dict):
        col.add("error", "coordinate_type", path, "coordinate must be an object.", expected="Coordinate", actual=coord)
        return
    col.require_float(coord, "latitude", path, min_value=-90.0, max_value=90.0)
    col.require_float(coord, "longitude", path, min_value=-180.0, max_value=180.0)
    altitude = _get_ci(coord, "altitude")
    if altitude is None:
        col.add("error", "missing_altitude", f"{path}.altitude", "altitude is required.", expected="int")
    elif not _is_int(altitude):
        col.add(
            "warn",
            "altitude_type",
            f"{path}.altitude",
            "altitude is defined as int in ICD.",
            expected="int",
            actual=altitude,
        )


def _validate_geometry_lists(col: _IssueCollector, info: dict[str, Any], path: str) -> None:
    coord_list = _get_ci(info, "coordinateList")
    if coord_list is not None:
        if not isinstance(coord_list, list):
            col.add("error", "coordinate_list_type", f"{path}.coordinateList", "coordinateList must be a list.", expected="List<Coordinate>", actual=coord_list)
        else:
            for idx, coord in enumerate(coord_list[:8]):
                _validate_coordinate(col, coord, f"{path}.coordinateList[{idx}]")

    line_list = _get_ci(info, "lineList")
    if line_list is not None:
        if not isinstance(line_list, list):
            col.add("error", "line_list_type", f"{path}.lineList", "lineList must be a list.", expected="List<Line>", actual=line_list)
        else:
            for idx, line in enumerate(line_list):
                if not isinstance(line, dict):
                    col.add("error", "line_type", f"{path}.lineList[{idx}]", "lineList item must be an object.", expected="Line", actual=line)
                    continue
                col.require_uint(line, "width", f"{path}.lineList[{idx}]", required=False, min_value=0, max_value=50000)
                coords = _get_ci(line, "coordinateList")
                if not isinstance(coords, list) or len(coords) < 2:
                    col.add("warn", "line_coord_count", f"{path}.lineList[{idx}].coordinateList", "line coordinateList should contain at least 2 points.")
                elif len(coords) > 0:
                    _validate_coordinate(col, coords[0], f"{path}.lineList[{idx}].coordinateList[0]")
                    _validate_coordinate(col, coords[-1], f"{path}.lineList[{idx}].coordinateList[-1]")

    area_list = _get_ci(info, "areaList")
    if area_list is not None:
        if not isinstance(area_list, list):
            col.add("error", "area_list_type", f"{path}.areaList", "areaList must be a list.", expected="List<Area>", actual=area_list)
        else:
            for idx, area in enumerate(area_list):
                if not isinstance(area, dict):
                    col.add("error", "area_type", f"{path}.areaList[{idx}]", "areaList item must be an object.", expected="Area", actual=area)
                    continue
                col.require_bool(area, "isHole", f"{path}.areaList[{idx}]", required=False)
                coords = _get_ci(area, "coordinateList")
                if not isinstance(coords, list) or len(coords) < 3:
                    col.add("warn", "area_coord_count", f"{path}.areaList[{idx}].coordinateList", "area coordinateList should contain at least 3 points.")
                elif len(coords) > 0:
                    _validate_coordinate(col, coords[0], f"{path}.areaList[{idx}].coordinateList[0]")
                    _validate_coordinate(col, coords[-1], f"{path}.areaList[{idx}].coordinateList[-1]")


def _validate_input_plans(col: _IssueCollector, input_plans: list[Any]) -> set[int]:
    input_ids: set[int] = set()
    seen_packages: set[int] = set()
    for pidx, plan in enumerate(input_plans):
        path = f"inputMissionPlans[{pidx}]"
        if not isinstance(plan, dict):
            col.add("error", "input_plan_type", path, "InputMissionPlan must be an object.", actual=plan)
            continue
        col.require_uint(plan, "timestamp", path)
        package_id = col.require_uint(plan, "inputMissionPackageID", path)
        if package_id is not None:
            if package_id in seen_packages:
                col.add("warn", "duplicate_input_package", f"{path}.inputMissionPackageID", "duplicate inputMissionPackageID.", actual=package_id)
            seen_packages.add(package_id)
        col.require_uint(plan, "inputMissionPackageType", path, required=False, max_value=7)
        col.require_uint(plan, "mainSensor", path, required=False, max_value=2)
        available = _get_ci(plan, "availableAircraftList") or []
        if isinstance(available, list):
            for idx, item in enumerate(available):
                if isinstance(item, dict):
                    col.require_uint(item, "aircraftID", f"{path}.availableAircraftList[{idx}]", max_value=6)
        missions = _get_ci(plan, "inputMissionList") or []
        if not isinstance(missions, list):
            col.add("error", "input_mission_list_type", f"{path}.inputMissionList", "inputMissionList must be a list.", actual=missions)
            continue
        seen_inputs: set[int] = set()
        for midx, mission in enumerate(missions):
            mpath = f"{path}.inputMissionList[{midx}]"
            if not isinstance(mission, dict):
                col.add("error", "input_mission_type", mpath, "InputMission must be an object.", actual=mission)
                continue
            input_id = col.require_uint(mission, "inputMissionID", mpath)
            if input_id is not None:
                if input_id in seen_inputs:
                    col.add("error", "duplicate_input_mission", f"{mpath}.inputMissionID", "duplicate inputMissionID in package.", actual=input_id)
                seen_inputs.add(input_id)
                input_ids.add(input_id)
            mission_type = col.require_uint(mission, "inputMissionType", mpath, max_value=7)
            col.require_uint(mission, "regionType", mpath, required=False, max_value=20)
            col.require_bool(mission, "isDone", mpath, required=False)
            detail = _get_ci(mission, "missionDetail")
            if isinstance(detail, dict):
                _validate_geometry_lists(col, detail, f"{mpath}.missionDetail")
                line_list = _get_ci(detail, "lineList")
                area_list = _get_ci(detail, "areaList")
                if mission_type in (1, 7) and not line_list:
                    col.add("warn", "input_line_missing", f"{mpath}.missionDetail.lineList", "line-style input mission has no lineList.")
                if mission_type in (2, 3, 4, 5, 6) and not area_list:
                    col.add("warn", "input_area_missing", f"{mpath}.missionDetail.areaList", "area-style input mission has no areaList.")
            elif detail is not None:
                col.add("error", "mission_detail_type", f"{mpath}.missionDetail", "missionDetail must be an object.", actual=detail)
    return input_ids


def _validate_flight_paths(col: _IssueCollector, flight_paths: list[Any]) -> tuple[dict[int, dict[str, Any]], dict[int, int]]:
    by_path: dict[int, dict[str, Any]] = {}
    aircraft_by_path: dict[int, int] = {}
    boundary_guard_tail_targets = _boundary_guard_cross_path_targets(flight_paths)
    for idx, raw in enumerate(flight_paths):
        path = f"flightPaths[{idx}]"
        data = raw.get("data") if isinstance(raw, dict) and isinstance(raw.get("data"), dict) else raw
        if not isinstance(data, dict):
            col.add("error", "flight_path_type", path, "FlightPath must be an object.", actual=raw)
            continue
        col.require_uint(data, "timestamp", path)
        path_id = col.require_uint(data, "pathID", path)
        aircraft_id = col.require_uint(data, "aircraftID", path, max_value=6)
        if path_id is not None:
            if path_id in by_path:
                col.add("error", "duplicate_path_id", f"{path}.pathID", "duplicate pathID.", actual=path_id)
            by_path[path_id] = data
            if aircraft_id is not None:
                aircraft_by_path[path_id] = aircraft_id
                expected_prefix = int(aircraft_id)
                actual_prefix = int(path_id) // 100000000
                if actual_prefix and actual_prefix != expected_prefix:
                    col.add(
                        "warn",
                        "path_aircraft_prefix",
                        f"{path}.pathID",
                        "pathID leading digit does not match aircraftID convention.",
                        expected=f"{expected_prefix}xxxxxxxx",
                        actual=path_id,
                    )
        is_formation = col.require_bool(data, "isFormationFlight", path, required=False)
        if is_formation:
            info = _get_ci(data, "formationInfo")
            if not isinstance(info, dict):
                col.add("error", "formation_info_missing", f"{path}.formationInfo", "formationInfo is required when isFormationFlight=true.")
            else:
                col.require_uint(info, "leaderAircraftID", f"{path}.formationInfo", max_value=6)
                formation = _get_ci(info, "formation")
                if isinstance(formation, dict):
                    for key in ("dX", "dY", "dZ"):
                        value = _get_ci(formation, key)
                        if value is not None and not _is_int(value):
                            col.add("warn", "formation_offset_type", f"{path}.formationInfo.formation.{key}", f"{key} should be int.", expected="int", actual=value)
                elif formation is not None:
                    col.add("error", "formation_type", f"{path}.formationInfo.formation", "formation must be an object.", actual=formation)
        wps = _waypoints(data)
        if not wps and not is_formation:
            col.add("error", "waypoint_list_empty", f"{path}.waypointList", "non-formation path has no waypoints.")
            continue
        seen_wp: set[int] = set()
        prev_eta: int | None = None
        for widx, wp in enumerate(wps):
            wpath = f"{path}.waypointList[{widx}]"
            waypoint_id = col.require_uint(wp, "waypointID", wpath)
            if waypoint_id is not None:
                if waypoint_id in seen_wp:
                    col.add("error", "duplicate_waypoint_id", f"{wpath}.waypointID", "duplicate waypointID in path.", actual=waypoint_id)
                seen_wp.add(waypoint_id)
            coord = _get_ci(wp, "coordinate")
            _validate_coordinate(col, coord, f"{wpath}.coordinate")
            col.require_float(wp, "speed", wpath, required=False, min_value=0.0, max_value=1000.0)
            eta = col.require_uint(wp, "eta", wpath, required=False)
            if eta is not None:
                if prev_eta is not None and eta < prev_eta:
                    col.add("warn", "eta_not_monotonic", f"{wpath}.eta", "waypoint eta should be nondecreasing.", actual=eta)
                prev_eta = eta
            col.require_float(wp, "ecf", wpath, required=False, min_value=0.0, max_value=1000.0)
            next_wp = col.require_uint(wp, "nextWaypointID", wpath, required=False)
            if next_wp not in (None, 0) and next_wp not in seen_wp:
                all_ids = {
                    _as_int(_get_ci(item, "waypointID"))
                    for item in wps
                    if isinstance(item, dict)
                }
                boundary_guard_tail_target = (
                    boundary_guard_tail_targets.get(int(path_id))
                    if path_id is not None and widx == len(wps) - 1
                    else None
                )
                if (
                    next_wp not in all_ids
                    and next_wp != boundary_guard_tail_target
                ):
                    col.add("warn", "next_waypoint_missing", f"{wpath}.nextWaypointID", "nextWaypointID does not exist in the same path.", actual=next_wp)
            col.require_uint(wp, "waypointPassType", wpath, required=False, max_value=3)
            col.require_bool(wp, "isDone", wpath, required=False)
            filming = _get_ci(wp, "filmingProperty")
            if isinstance(filming, dict):
                col.require_float(filming, "fieldOfView", f"{wpath}.filmingProperty", required=False, min_value=0.0, max_value=180.0)
                col.require_uint(filming, "sensorType", f"{wpath}.filmingProperty", required=False, max_value=2)
                op_mode = col.require_uint(filming, "operationMode", f"{wpath}.filmingProperty", required=False, max_value=5)
                line_search = _get_ci(filming, "lineSearch")
                if isinstance(line_search, dict):
                    col.require_float(line_search, "searchSpeed", f"{wpath}.filmingProperty.lineSearch", required=False, min_value=0.0)
                    coords = _get_ci(line_search, "coordinateList")
                    if isinstance(coords, list):
                        for cidx, coord in enumerate(coords[:2]):
                            _validate_coordinate(col, coord, f"{wpath}.filmingProperty.lineSearch.coordinateList[{cidx}]")
                aircraft_fixed = _get_ci(filming, "aircraftFixed")
                if isinstance(aircraft_fixed, dict):
                    col.require_float(aircraft_fixed, "gimbalPitch", f"{wpath}.filmingProperty.aircraftFixed", required=False)
                    col.require_float(aircraft_fixed, "gimbalYaw", f"{wpath}.filmingProperty.aircraftFixed", required=False)
                if op_mode == 2:
                    coords = _get_ci(line_search, "coordinateList") if isinstance(line_search, dict) else None
                    if not isinstance(coords, list) or len(coords) < 2:
                        col.add("warn", "line_search_missing", f"{wpath}.filmingProperty.lineSearch.coordinateList", "operationMode=2 requires lineSearch coordinates.")
    return by_path, aircraft_by_path


def _validate_individual_plans(
    col: _IssueCollector,
    individual_plans: list[Any],
    *,
    input_ids: set[int],
    flight_paths_by_id: dict[int, dict[str, Any]],
    aircraft_by_path: dict[int, int],
    package_aircraft_map: dict[int, int],
) -> None:
    seen_individual_ids: set[int] = set()
    seen_packages: set[int] = set()
    for pidx, plan in enumerate(individual_plans):
        path = f"individualMissionPlans[{pidx}]"
        if not isinstance(plan, dict):
            col.add("error", "individual_plan_type", path, "IndividualMissionPlan must be an object.", actual=plan)
            continue
        col.require_uint(plan, "timestamp", path)
        package_id = col.require_uint(plan, "individualMissionPackageID", path)
        aircraft_id = col.require_uint(plan, "aircraftID", path, max_value=6)
        if package_id is not None:
            if package_id in seen_packages:
                col.add("warn", "duplicate_individual_package", f"{path}.individualMissionPackageID", "duplicate individualMissionPackageID.", actual=package_id)
            seen_packages.add(package_id)
            expected_aircraft = package_aircraft_map.get(package_id)
            if expected_aircraft is not None and aircraft_id is not None and expected_aircraft != aircraft_id:
                col.add(
                    "error",
                    "package_aircraft_mismatch",
                    f"{path}.aircraftID",
                    "IndividualMissionPlan aircraftID does not match MissionPlan aircraftList.",
                    expected=str(expected_aircraft),
                    actual=aircraft_id,
                )
        missions = _get_ci(plan, "individualMissionList") or []
        if not isinstance(missions, list):
            col.add("error", "individual_mission_list_type", f"{path}.individualMissionList", "individualMissionList must be a list.", actual=missions)
            continue
        for midx, mission in enumerate(missions):
            mpath = f"{path}.individualMissionList[{midx}]"
            if not isinstance(mission, dict):
                col.add("error", "individual_mission_type", mpath, "IndividualMission must be an object.", actual=mission)
                continue
            mission_id = col.require_uint(mission, "individualMissionID", mpath)
            if mission_id is not None:
                if mission_id in seen_individual_ids:
                    col.add("error", "duplicate_individual_mission", f"{mpath}.individualMissionID", "duplicate individualMissionID.", actual=mission_id)
                seen_individual_ids.add(mission_id)
            col.require_bool(mission, "isDone", mpath)
            path_id = col.require_uint(mission, "pathID", mpath)
            if path_id is not None:
                if path_id not in flight_paths_by_id:
                    col.add("error", "path_missing", f"{mpath}.pathID", "pathID is not present in FlightPath payload.", actual=path_id)
                path_aircraft = aircraft_by_path.get(path_id)
                if aircraft_id is not None and path_aircraft is not None and int(path_aircraft) != int(aircraft_id):
                    col.add("error", "path_aircraft_mismatch", f"{mpath}.pathID", "FlightPath aircraftID does not match owning IndividualMissionPlan.", expected=str(aircraft_id), actual=path_aircraft)
            related = _get_ci(mission, "relatedMission")
            if not isinstance(related, dict):
                col.add("error", "related_mission_missing", f"{mpath}.relatedMission", "relatedMission is required.")
            else:
                rel_type = col.require_uint(related, "relatedMissionType", f"{mpath}.relatedMission", max_value=2)
                input_id = col.require_uint(related, "inputMissionID", f"{mpath}.relatedMission")
                if input_id is not None and input_ids and input_id not in input_ids:
                    col.add("error", "input_mission_missing", f"{mpath}.relatedMission.inputMissionID", "inputMissionID is not present in InputMissionPlan.", actual=input_id)
                prior_id = col.require_uint(related, "priorMissionID", f"{mpath}.relatedMission", required=False)
                if rel_type == 1 and prior_id not in (None, 0):
                    col.add("warn", "unexpected_prior_mission", f"{mpath}.relatedMission.priorMissionID", "relatedMissionType=1 should use priorMissionID=0.", actual=prior_id)
                if rel_type == 2 and (prior_id is None or prior_id <= 0):
                    col.add("warn", "prior_mission_missing", f"{mpath}.relatedMission.priorMissionID", "relatedMissionType=2 should include priorMissionID.")
            info = _get_ci(mission, "individualMissionInfo", "missionInfo")
            if isinstance(info, dict):
                col.require_uint(info, "individualMissionType", f"{mpath}.individualMissionInfo", required=False)
                col.require_uint(info, "patternType", f"{mpath}.individualMissionInfo", required=False)
                col.require_bool(info, "autoZoomIn", f"{mpath}.individualMissionInfo", required=False)
                col.require_uint(info, "targetID", f"{mpath}.individualMissionInfo", required=False)
                _validate_geometry_lists(col, info, f"{mpath}.individualMissionInfo")
            elif info is not None:
                col.add("error", "individual_mission_info_type", f"{mpath}.individualMissionInfo", "individualMissionInfo must be an object.", actual=info)


def _mission_plan_package_map(col: _IssueCollector, mission_plan: dict[str, Any] | None) -> dict[int, int]:
    if not isinstance(mission_plan, dict):
        return {}
    path = "missionPlan"
    col.require_uint(mission_plan, "timestamp", path)
    col.require_uint(mission_plan, "missionPlanID", path)
    col.require_uint(mission_plan, "missionPlanTimestamp", path, required=False)
    col.require_float(mission_plan, "planningTime", path, required=False, min_value=0.0)
    col.require_uint(mission_plan, "plannerID", path, required=False, max_value=6)
    col.require_uint(mission_plan, "inputMissionPackageID", path)
    col.require_uint(mission_plan, "missionReferencePackageID", path, required=False)
    aircraft_list = _get_ci(mission_plan, "aircraftList") or []
    package_aircraft: dict[int, int] = {}
    if not isinstance(aircraft_list, list):
        col.add("error", "aircraft_list_type", f"{path}.aircraftList", "aircraftList must be a list.", actual=aircraft_list)
        return package_aircraft
    seen_aircraft: set[int] = set()
    for idx, item in enumerate(aircraft_list):
        apath = f"{path}.aircraftList[{idx}]"
        if not isinstance(item, dict):
            col.add("error", "aircraft_entry_type", apath, "aircraftList item must be an object.", actual=item)
            continue
        aircraft_id = col.require_uint(item, "aircraftID", apath, max_value=6)
        package_id = col.require_uint(item, "individualMissionPackageID", apath)
        if aircraft_id is not None:
            if aircraft_id in seen_aircraft:
                col.add("warn", "duplicate_aircraft", f"{apath}.aircraftID", "duplicate aircraftID in MissionPlan.", actual=aircraft_id)
            seen_aircraft.add(aircraft_id)
        if aircraft_id is not None and package_id is not None:
            package_aircraft[package_id] = aircraft_id
    return package_aircraft


def validate_mission_payload(
    payload: dict[str, Any],
    *,
    mission_plan: dict[str, Any] | None = None,
) -> dict[str, Any]:
    col = _IssueCollector()
    package_aircraft_map = _mission_plan_package_map(col, mission_plan)

    input_plans = payload.get("inputMissionPlans") or []
    if input_plans and not isinstance(input_plans, list):
        col.add("error", "input_plans_type", "payload.inputMissionPlans", "inputMissionPlans must be a list.", actual=input_plans)
        input_plans = []
    input_ids = _validate_input_plans(col, input_plans)

    flight_paths = payload.get("flightPaths") or payload.get("paths") or payload.get("flightpaths") or []
    if not isinstance(flight_paths, list):
        col.add("error", "flight_paths_type", "payload.flightPaths", "flightPaths must be a list.", actual=flight_paths)
        flight_paths = []
    flight_paths_by_id, aircraft_by_path = _validate_flight_paths(col, flight_paths)

    individual_plans = payload.get("individualMissionPlans") or []
    if individual_plans and not isinstance(individual_plans, list):
        col.add("error", "individual_plans_type", "payload.individualMissionPlans", "individualMissionPlans must be a list.", actual=individual_plans)
        individual_plans = []
    _validate_individual_plans(
        col,
        individual_plans,
        input_ids=input_ids,
        flight_paths_by_id=flight_paths_by_id,
        aircraft_by_path=aircraft_by_path,
        package_aircraft_map=package_aircraft_map,
    )

    if not flight_paths:
        col.add("error", "no_flight_paths", "payload.flightPaths", "payload has no flight paths.")
    if not input_plans:
        col.add("warn", "no_input_plans", "payload.inputMissionPlans", "payload has no input mission plans.")
    if not individual_plans:
        col.add("warn", "no_individual_plans", "payload.individualMissionPlans", "payload has no individual mission plans.")

    return _build_result(col)


def _record_validation_issue(
    validation: dict[str, Any],
    severity: str,
    code: str,
    path: str,
    message: str,
    *,
    expected: str | None = None,
    actual: Any = None,
    at_front: bool = False,
) -> None:
    counts = validation.setdefault("counts", {"error": 0, "warn": 0, "info": 0})
    sev = severity if severity in counts else "warn"
    validation["issueCount"] = int(validation.get("issueCount", 0)) + 1
    counts[sev] = int(counts.get(sev, 0)) + 1
    issues = validation.setdefault("issues", [])
    if len(issues) < MAX_REPORTED_ISSUES or at_front:
        item: dict[str, Any] = {
            "severity": sev,
            "code": code,
            "path": path,
            "message": message,
        }
        if expected:
            item["expected"] = expected
        if actual is not None:
            item["actualType"] = _value_type(actual)
            item["actual"] = actual
        if at_front:
            if len(issues) >= MAX_REPORTED_ISSUES:
                issues.pop()
            issues.insert(0, item)
        else:
            issues.append(item)
    validation["reportedIssueCount"] = len(issues)
    validation["truncated"] = int(validation.get("issueCount", 0)) > len(issues)
    validation["ok"] = int(counts.get("error", 0)) == 0
    if int(counts.get("error", 0)) > 0:
        validation["status"] = "error"
    elif int(counts.get("warn", 0)) > 0:
        validation["status"] = "warn"
    else:
        validation["status"] = "ok"


def validate_mission_plan_result(plan_result: dict[str, Any]) -> dict[str, Any]:
    payload = plan_result.get("payload") if isinstance(plan_result, dict) else None
    if not isinstance(payload, dict):
        return {
            "ok": False,
            "status": "error",
            "issueCount": 1,
            "reportedIssueCount": 1,
            "truncated": False,
            "counts": {"error": 1, "warn": 0, "info": 0},
            "issues": [
                {
                    "severity": "error",
                    "code": "payload_missing",
                    "path": "payload",
                    "message": "Mission plan payload is missing.",
                }
            ],
        }
    validation = validate_mission_payload(
        payload,
        mission_plan=plan_result.get("missionPlan") if isinstance(plan_result.get("missionPlan"), dict) else None,
    )
    mission_plan = plan_result.get("missionPlan") if isinstance(plan_result.get("missionPlan"), dict) else None
    if mission_plan is not None:
        result_plan_id = _as_int(plan_result.get("missionPlanID"))
        plan_id = _as_int(_get_ci(mission_plan, "missionPlanID"))
        if result_plan_id is not None and plan_id is not None and result_plan_id != plan_id:
            _record_validation_issue(
                validation,
                "error",
                "mission_plan_id_mismatch",
                "missionPlan.missionPlanID",
                "MissionPlan file ID does not match requested missionPlanID.",
                expected=str(result_plan_id),
                actual=plan_id,
            )

    expected_input_package_id = _as_int(plan_result.get("inputMissionPackageID"))
    input_plans = plan_result.get("inputMissionPlans")
    if expected_input_package_id is not None and isinstance(input_plans, list):
        actual_input_package_ids = {
            _as_int(_get_ci(plan, "inputMissionPackageID"))
            for plan in input_plans
            if isinstance(plan, dict)
        }
        if expected_input_package_id not in actual_input_package_ids:
            _record_validation_issue(
                validation,
                "error",
                "input_package_missing_or_mismatch",
                "InputMissionPlan.inputMissionPackageID",
                "MissionPlan inputMissionPackageID does not match loaded InputMissionPlan.",
                expected=str(expected_input_package_id),
                actual=sorted(item for item in actual_input_package_ids if item is not None),
            )

    expected_individual_ids = {
        int(item)
        for item in (plan_result.get("individualMissionPackageIDs") or [])
        if _is_int(item)
    }
    individual_plans = plan_result.get("individualMissionPlans")
    if expected_individual_ids and isinstance(individual_plans, list):
        actual_individual_ids = {
            _as_int(_get_ci(plan, "individualMissionPackageID"))
            for plan in individual_plans
            if isinstance(plan, dict)
        }
        actual_ids = {item for item in actual_individual_ids if item is not None}
        missing_individual_ids = sorted(expected_individual_ids - actual_ids)
        for package_id in missing_individual_ids[:10]:
            _record_validation_issue(
                validation,
                "error",
                "individual_package_missing_or_mismatch",
                "IndividualMissionPlan.individualMissionPackageID",
                "MissionPlan aircraftList references an IndividualMissionPlan that was not loaded with the same ID.",
                expected=str(package_id),
                actual=sorted(actual_ids),
            )

    missing = plan_result.get("missingPathIds")
    if isinstance(missing, list) and missing:
        _record_validation_issue(
            validation,
            "error",
            "missing_path_files",
            "FlightPath",
            "MissionPlan references FlightPath files that were not found.",
            actual=missing[:20],
            at_front=True,
        )
    return validation


def _build_result(col: _IssueCollector) -> dict[str, Any]:
    status = "ok"
    if col.counts["error"] > 0:
        status = "error"
    elif col.counts["warn"] > 0:
        status = "warn"
    return {
        "ok": col.counts["error"] == 0,
        "status": status,
        "issueCount": int(col.total_count),
        "reportedIssueCount": len(col.issues),
        "truncated": col.total_count > len(col.issues),
        "counts": dict(col.counts),
        "issues": list(col.issues),
        "source": "sim_mission_validator",
        "icdReference": [
            "resource/nFusion 파일_0316/msg_0301/MissionPlanData.nftype",
            "resource/nFusion 파일_0316/msg_0302/IndividualMission.nftype",
            "resource/nFusion 파일_0316/msg_0303/UAVFlightPlanData.nftype",
            "resource/nFusion 파일_0316/msg_0303/Waypoint.nftype",
            "resource/nFusion 파일_0316/CommonType/Coordinate.nftype",
        ],
    }
