"""The manned transit to the next mission may not use a later mission's corridor.

The route to a hold is a shortest path through the *union* of the input plan's
LINE corridors and AREA polygons.  With every mission's zone in that union the
corridors chain into one connected network, and a corridor belonging to a
mission far later in the plan becomes a free highway: the manned aircraft rode
the last mission's 통제권변경 line kilometres east and came back, instead of
stepping through the regions in order.

The geometry below is the 10-mission plan from the run where this was found.
"""

from __future__ import annotations

import math
from typing import Any

from modules.mission_planning.replanning.triggers.attack.pipeline import (
    _build_lah_mission_constrained_attack_route as build_route,
)
from modules.mission_planning.replanning.triggers.next_collab import pipeline as nc

# (inputMissionID, regionType, kind, width_m, coordinates) in plan order.
# regionType 2 통제권변경 / 3 ACP / 4 공격대기 / 5 전투진지 / 6 목표지역.
MISSION_SPEC: list[tuple[int, int, str, int, list[tuple[float, float]]]] = [
    (70000000, 3, "line", 1000, [(37.86438, 128.11906), (37.8838, 128.13091)]),
    (70000001, 4, "line", 1000, [(37.88375, 128.13048), (37.90227, 128.14873)]),
    (70000002, 4, "area", 0, [(37.90841, 128.13649), (37.89464, 128.16132), (37.91035, 128.17122), (37.92457, 128.1501)]),
    (70000006, 5, "line", 1000, [(37.90247, 128.16626), (37.88717, 128.19197)]),
    (70000007, 5, "area", 0, [(37.87307, 128.18769), (37.87192, 128.21848), (37.88276, 128.21837), (37.88947, 128.20975), (37.88937, 128.19607), (37.88663, 128.19097)]),
    (70000008, 6, "line", 1000, [(37.88942, 128.20312), (37.90588, 128.20226)]),
    (70000003, 6, "area", 0, [(37.95097, 128.17448), (37.90655, 128.17557), (37.90525, 128.22718), (37.95025, 128.2269)]),
    (70000009, 5, "line", 1000, [(37.90588, 128.20226), (37.88942, 128.20312)]),
    (70000004, 3, "line", 1000, [(37.87252, 128.20244), (37.86923, 128.20195)]),
    (70000005, 2, "line", 1000, [(37.86921, 128.20177), (37.86121, 128.135)]),
]

# The manned aircraft's live position at the 0803 replan, and the hold it was
# being routed to - 5.5 km apart, both in the west of the operation.
START = {"latitude": 37.83612, "longitude": 128.11238, "altitude": 771}
DEST = {"latitude": 37.88380, "longitude": 128.13091, "altitude": 957}
TARGET_INPUT_ID = 70000001


def _plan() -> dict[str, Any]:
    missions: list[dict[str, Any]] = []
    for input_id, region_type, kind, width_m, points in MISSION_SPEC:
        coordinates = [
            {"latitude": lat, "longitude": lon, "altitude": 600} for lat, lon in points
        ]
        detail: dict[str, Any] = (
            {"lineList": [{"width": width_m, "coordinateList": coordinates}]}
            if kind == "line"
            else {"areaList": [{"isHole": False, "coordinateList": coordinates}]}
        )
        missions.append(
            {
                "inputMissionID": input_id,
                "regionType": region_type,
                "missionDetail": detail,
            }
        )
    return {"inputMissionPackageID": 101, "inputMissionList": missions}


def _length_m(points: list[dict[str, Any]]) -> float:
    total = 0.0
    for first, second in zip(points, points[1:]):
        total += math.hypot(
            (second["latitude"] - first["latitude"]) * 111_132.0,
            (second["longitude"] - first["longitude"])
            * 111_320.0
            * math.cos(math.radians(second["latitude"])),
        )
    return total


def _route(limit: int | None) -> list[dict[str, Any]]:
    zones = nc._lah_operation_zones_from_input_plan(_plan(), max_mission_index=limit)
    points, _meta = build_route(
        start_coord=START,
        attack_coord=DEST,
        source_plan_id=None,
        operation_zones=zones,
    )
    return points


DIRECT_M = _length_m([START, DEST])


def test_the_target_mission_is_located_by_its_own_plan_order() -> None:
    plan = _plan()

    assert nc._input_mission_index(plan, 70000000) == 0
    assert nc._input_mission_index(plan, TARGET_INPUT_ID) == 1
    # The plan's IDs are not in ascending order - position is what matters.
    assert nc._input_mission_index(plan, 70000006) == 3
    assert nc._input_mission_index(plan, 70000005) == 9
    assert nc._input_mission_index(plan, 99999999) is None
    assert nc._input_mission_index(None, TARGET_INPUT_ID) is None


def test_zones_past_the_destination_mission_are_dropped() -> None:
    plan = _plan()

    everything = nc._lah_operation_zones_from_input_plan(plan)
    windowed = nc._lah_operation_zones_from_input_plan(plan, max_mission_index=1)

    assert len(everything) == len(MISSION_SPEC)
    assert {zone["inputMissionID"] for zone in windowed} == {70000000, 70000001}
    # No limit keeps the old behaviour for callers that do not pass one.
    assert len(windowed) < len(everything)


def test_the_transit_no_longer_detours_through_a_later_corridor() -> None:
    unbounded_m = _length_m(_route(None))
    windowed_m = _length_m(_route(1))

    # Measured on the real plan: 16.3 km for a 5.5 km leg.
    assert unbounded_m > DIRECT_M * 2.5
    assert windowed_m < DIRECT_M * 1.05


def test_the_transit_never_reaches_the_regions_this_leg_has_no_business_in() -> None:
    unbounded_max_lon = max(point["longitude"] for point in _route(None))
    windowed_max_lon = max(point["longitude"] for point in _route(1))

    # 128.19+ is the 전투진지/목표지역 band, ~5 km east of anywhere this leg belongs.
    assert unbounded_max_lon > 128.18
    assert windowed_max_lon <= DEST["longitude"] + 1e-6


def test_a_destination_that_is_not_in_the_plan_keeps_every_zone() -> None:
    """An unknown target must not silently strand the route with no geometry."""

    plan = _plan()
    limit = nc._input_mission_index(plan, 99999999)

    assert limit is None
    zones = nc._lah_operation_zones_from_input_plan(plan, max_mission_index=limit)
    assert len(zones) == len(MISSION_SPEC)


def test_earlier_corridors_stay_available_for_a_lagging_aircraft() -> None:
    """The bound is forward-only: ground already passed is still flyable."""

    zones = nc._lah_operation_zones_from_input_plan(_plan(), max_mission_index=1)

    assert 70000000 in {zone["inputMissionID"] for zone in zones}


def test_a_later_destination_opens_the_corridors_up_to_it() -> None:
    """Flying to mission 70000006 legitimately needs everything before it."""

    plan = _plan()
    limit = nc._input_mission_index(plan, 70000006)
    zones = nc._lah_operation_zones_from_input_plan(plan, max_mission_index=limit)

    assert limit == 3
    assert {zone["inputMissionID"] for zone in zones} == {
        70000000,
        70000001,
        70000002,
        70000006,
    }
    # Still nothing from the egress end of the plan.
    assert 70000005 not in {zone["inputMissionID"] for zone in zones}
