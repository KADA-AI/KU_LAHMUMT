from __future__ import annotations

from copy import deepcopy
from typing import Any


def _positive_int(value: Any) -> int | None:
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return None
    return parsed if parsed > 0 else None


def _valid_coordinate_count(value: Any) -> int:
    if not isinstance(value, list):
        return 0
    count = 0
    for row in value:
        if not isinstance(row, dict):
            continue
        try:
            latitude = float(row.get("latitude"))
            longitude = float(row.get("longitude"))
        except (TypeError, ValueError):
            continue
        if -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0:
            count += 1
    return count


def resolve_reexecute_lah_template_input_id(
    current_input_id: Any,
    source_template_input_id: Any,
) -> int | None:
    """Prefer the original input mission ID when resolving a reexecute role template."""
    return _positive_int(source_template_input_id) or _positive_int(current_input_id)


def has_reusable_lah_role_geometry(mission: Any) -> bool:
    """Return whether a source LAH mission contains a reusable role position/route."""
    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        return False

    # A single coordinate is a valid LAH role anchor (for example Type 9 hold).
    if _valid_coordinate_count(info.get("coordinateList")) >= 1:
        return True

    line_list = info.get("lineList")
    if isinstance(line_list, list):
        for line in line_list:
            if isinstance(line, dict) and _valid_coordinate_count(line.get("coordinateList")) >= 2:
                return True

    area_list = info.get("areaList")
    if isinstance(area_list, list):
        for area in area_list:
            if isinstance(area, dict) and _valid_coordinate_count(area.get("coordinateList")) >= 3:
                return True
    return False


def rebind_reexecute_lah_role_mission(
    source_mission: dict,
    *,
    aircraft_id: int,
    current_input_id: int,
    path_id: int,
) -> dict:
    """Clone a source role mission while binding it to the new reexecute mission."""
    mission = deepcopy(source_mission)
    related = mission.get("relatedMission")
    if not isinstance(related, dict):
        related = {}
    mission["aircraftID"] = int(aircraft_id)
    mission["individualMissionID"] = 0
    mission["isDone"] = False
    mission["relatedMission"] = {
        "relatedMissionType": related.get("relatedMissionType", 1),
        "inputMissionID": int(current_input_id),
        "priorMissionID": related.get("priorMissionID", 0),
    }
    mission["pathID"] = int(path_id)
    return mission
