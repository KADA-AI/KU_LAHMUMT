from __future__ import annotations

from copy import deepcopy
from typing import Any, Iterable, Mapping


CONTROL_TRANSFER_REGION_TYPE = 2
HANDOVER_TERMINAL_MISSION_MARKER = "_appendHandoverTerminal"
CONTROL_TRANSFER_DIRECT_MISSION_MARKER = "_controlTransferDirect"
CONTROL_TRANSFER_COORDINATE_LIST_KEY = "controlTransferCoordinateList"
CONTROL_TRANSFER_FOV_DEG = 15.0


def _as_int(value: object) -> int | None:
    try:
        return int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


def is_control_transfer_input_mission(mission: object) -> bool:
    if not isinstance(mission, Mapping):
        return False
    return _as_int(mission.get("regionType")) == CONTROL_TRANSFER_REGION_TYPE


def handover_terminal_input_mission_ids(input_plan: object) -> set[int]:
    if not isinstance(input_plan, Mapping):
        return set()
    result: set[int] = set()
    for mission in input_plan.get("inputMissionList") or []:
        if not is_control_transfer_input_mission(mission):
            continue
        mission_id = _as_int(mission.get("inputMissionID"))
        if mission_id is not None and mission_id > 0:
            result.add(int(mission_id))
    return result


def individual_mission_input_id(mission: object) -> int | None:
    if not isinstance(mission, Mapping):
        return None
    related = mission.get("relatedMission")
    if isinstance(related, Mapping):
        mission_id = _as_int(related.get("inputMissionID"))
        if mission_id is not None:
            return mission_id
    return _as_int(mission.get("inputMissionID"))


def _mission_info(mission: object) -> Mapping[str, Any]:
    if not isinstance(mission, Mapping):
        return {}
    info = mission.get("individualMissionInfo")
    if isinstance(info, Mapping):
        return info
    detail = mission.get("missionDetail")
    if isinstance(detail, Mapping):
        return detail
    return mission


def control_transfer_route_coordinates(mission: object) -> list[dict[str, Any]]:
    """Return the original 0201 center line, not a width-split search strip."""

    info = _mission_info(mission)
    candidates: list[object] = [
        info.get(CONTROL_TRANSFER_COORDINATE_LIST_KEY),
        info.get("sourceCoordinateList"),
    ]
    line_list = info.get("lineList")
    if isinstance(line_list, list) and line_list:
        first_line = line_list[0]
        if isinstance(first_line, Mapping):
            candidates.append(first_line.get("coordinateList"))

    for candidate in candidates:
        if not isinstance(candidate, list):
            continue
        normalized: list[dict[str, Any]] = []
        for coordinate in candidate:
            if not isinstance(coordinate, Mapping):
                continue
            try:
                row: dict[str, Any] = {
                    "latitude": float(coordinate["latitude"]),
                    "longitude": float(coordinate["longitude"]),
                }
            except (KeyError, TypeError, ValueError):
                continue
            if coordinate.get("altitude") is not None:
                try:
                    row["altitude"] = float(coordinate.get("altitude") or 0.0)
                except (TypeError, ValueError):
                    pass
            if not normalized or (
                abs(row["latitude"] - normalized[-1]["latitude"]) > 1e-10
                or abs(row["longitude"] - normalized[-1]["longitude"]) > 1e-10
            ):
                normalized.append(row)
        if normalized:
            return normalized
    return []


def is_control_transfer_direct_mission(mission: object) -> bool:
    if not isinstance(mission, Mapping):
        return False
    info = _mission_info(mission)
    return bool(
        mission.get(CONTROL_TRANSFER_DIRECT_MISSION_MARKER)
        or info.get(CONTROL_TRANSFER_DIRECT_MISSION_MARKER)
        or _as_int(info.get("sourceRegionType")) == CONTROL_TRANSFER_REGION_TYPE
        or _as_int(mission.get("regionType")) == CONTROL_TRANSFER_REGION_TYPE
    )


def apply_control_transfer_direct_metadata(
    mission_info: dict[str, Any],
    input_mission: object,
) -> bool:
    """Attach regionType=2 transit semantics and the input center line."""

    if not is_control_transfer_input_mission(input_mission):
        return False
    mission_info["sourceRegionType"] = CONTROL_TRANSFER_REGION_TYPE
    mission_info[CONTROL_TRANSFER_DIRECT_MISSION_MARKER] = True
    mission_info["FOV"] = float(CONTROL_TRANSFER_FOV_DEG)
    coordinates = control_transfer_route_coordinates(input_mission)
    if coordinates:
        mission_info[CONTROL_TRANSFER_COORDINATE_LIST_KEY] = deepcopy(coordinates)
    return True


def mark_handover_terminal_missions(
    missions: Iterable[object],
    input_plan: object,
) -> int:
    """Mark regionType=2 rows as direct transit without adding a 0203 HO WP.

    Hand-over coordinates remain part of 0203 reference information, but they
    are not command waypoints. The final regionType=2 mission uses its own
    input center line through the final point and ignores corridor width.
    """

    input_by_id: dict[int, object] = {}
    if isinstance(input_plan, Mapping):
        for input_mission in input_plan.get("inputMissionList") or []:
            if not isinstance(input_mission, Mapping):
                continue
            mission_id = _as_int(input_mission.get("inputMissionID"))
            if mission_id is not None and mission_id > 0:
                input_by_id[int(mission_id)] = input_mission

    marked = 0
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        mission.pop(HANDOVER_TERMINAL_MISSION_MARKER, None)
        mission.pop(CONTROL_TRANSFER_DIRECT_MISSION_MARKER, None)
        if _as_int(mission.get("aircraftID")) not in (4, 5, 6):
            continue
        input_mission = input_by_id.get(individual_mission_input_id(mission) or -1)
        if not is_control_transfer_input_mission(input_mission):
            continue
        info = mission.get("individualMissionInfo")
        if not isinstance(info, dict):
            continue
        apply_control_transfer_direct_metadata(info, input_mission)
        mission[CONTROL_TRANSFER_DIRECT_MISSION_MARKER] = True
        marked += 1
    return marked


def mission_requests_handover_terminal(mission: object) -> bool:
    """HO is reference data only and never extends a generated mission path."""

    del mission
    return False


def handover_coordinate_map(reference_plan: object) -> dict[int, dict[str, float]]:
    if not isinstance(reference_plan, Mapping):
        return {}
    result: dict[int, dict[str, float]] = {}
    for row in reference_plan.get("handOverInfoList") or []:
        if not isinstance(row, Mapping):
            continue
        aircraft_id = _as_int(row.get("aircraftID"))
        coordinate = row.get("coordinate")
        if aircraft_id is None or aircraft_id <= 0 or not isinstance(coordinate, Mapping):
            continue
        try:
            latitude = float(coordinate["latitude"])
            longitude = float(coordinate["longitude"])
        except (KeyError, TypeError, ValueError):
            continue
        normalized: dict[str, float] = {
            "latitude": latitude,
            "longitude": longitude,
        }
        try:
            normalized["altitude"] = float(coordinate.get("altitude", 0.0) or 0.0)
        except (TypeError, ValueError):
            pass
        result[int(aircraft_id)] = normalized
    return result
