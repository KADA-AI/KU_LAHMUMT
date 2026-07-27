from __future__ import annotations

import pytest

from modules.mission_planning.pipelines import lah_operational_mode as lah_mode


def _assert_area_internal_cover_info(info: dict, cycle: dict) -> None:
    """Cover is sought near the anchor, not confined to the tasked AREA.

    Terrain that masks the aircraft does not care which polygon the mission
    happens to cover, so the selector searches a radius around the anchor.
    Holes stay excluded because those are keep-out geometry.
    """

    selected = info["coordinateList"][0]
    area_rows = cycle["battleAreaList"]
    holes = [row for row in area_rows if bool(row.get("isHole"))]
    anchor = cycle["battleAttackCoordinate"]
    assert info["_lahTerminalCoverEnabled"] is True
    assert info["_lahConstraintAreaList"] == area_rows
    assert info["_lahTerminalCoverFallbackCoordinate"] == anchor

    radius_m = lah_mode._cover_search_radius_m()
    if radius_m > 0.0:
        distance_m = _rough_distance_m(selected, anchor)
        assert distance_m <= radius_m + 1.0, (distance_m, radius_m)
    else:
        outers = [row for row in area_rows if not bool(row.get("isHole"))]
        assert any(lah_mode._point_in_polygon(selected, row["coordinateList"]) for row in outers)
    assert not any(lah_mode._point_in_polygon(selected, row["coordinateList"]) for row in holes)


def _rough_distance_m(left: dict, right: dict) -> float:
    import math

    lat_m = 111_132.92
    lon_m = lat_m * math.cos(math.radians(float(left["latitude"])))
    d_lat = (float(left["latitude"]) - float(right["latitude"])) * lat_m
    d_lon = (float(left["longitude"]) - float(right["longitude"])) * lon_m
    return math.hypot(d_lat, d_lon)


def _input_plan(pattern) -> dict:
    missions = []
    for index, (mission_type, region_type) in enumerate(pattern):
        latitude = 37.0 + index * 0.01
        longitude = 127.0 + index * 0.01
        if mission_type == 1:
            detail = {
                "lineList": [
                    {
                        "width": 1000,
                        "coordinateList": [
                            {"latitude": latitude, "longitude": longitude, "altitude": 0},
                            {"latitude": latitude + 0.005, "longitude": longitude + 0.005, "altitude": 0},
                        ],
                    }
                ]
            }
        else:
            detail = {
                "areaList": [
                    {
                        "coordinateList": [
                            {"latitude": latitude - 0.001, "longitude": longitude - 0.001},
                            {"latitude": latitude - 0.001, "longitude": longitude + 0.001},
                            {"latitude": latitude + 0.001, "longitude": longitude + 0.001},
                            {"latitude": latitude + 0.001, "longitude": longitude - 0.001},
                        ]
                    }
                ]
            }
            if region_type == lah_mode.REGION_BATTLE_POSITION:
                detail["battleAttackCoordinate"] = {
                    "latitude": latitude,
                    "longitude": longitude,
                    "altitude": 0,
                }
        missions.append(
            {
                "inputMissionID": index + 1,
                "inputMissionType": mission_type,
                "regionType": region_type,
                "missionDetail": detail,
            }
        )
    return {"inputMissionList": missions}


@pytest.mark.parametrize(
    ("pattern", "mission_index", "cycle_index"),
    [
        (lah_mode.ANTI_ARMOR_REVIEWED_PATTERN, 7, 0),
        (lah_mode.ANTI_ARMOR_REFRESHED_PATTERN, 7, 0),
        (lah_mode.ANTI_ARMOR_REFRESHED_PATTERN, 13, 1),
    ],
)
def test_target_to_battle_line_keeps_lah_at_battle_position(
    pattern,
    mission_index: int,
    cycle_index: int,
) -> None:
    plan = _input_plan(pattern)
    profile = lah_mode.detect_lah_special_operation(plan)
    assert profile is not None

    info, source_input_id, behavior = lah_mode.lah_special_info_for_index(
        plan,
        profile,
        mission_index,
    )

    cycle = profile["targetBattleCycles"][cycle_index]
    expected = cycle["battleAttackCoordinate"]
    assert behavior == "target_to_battle_battle_hold"
    assert source_input_id == cycle["battlePositionInputMissionID"]
    assert "lineList" not in info
    # The manned hold no longer pins an altitude; the terrain-following and
    # UAV-LOS passes own it.
    assert "forceAltitudeM" not in info
    assert info["_lahTerminalCoverFallbackCoordinate"] == expected
    _assert_area_internal_cover_info(info, cycle)


def test_type1_target_change_replan_finds_two_cycles_after_order_change() -> None:
    plan = _input_plan(lah_mode.ANTI_ARMOR_REFRESHED_PATTERN)
    missions = plan["inputMissionList"]
    # Simulate a refreshed Type-1 plan whose two intermediate approach lines
    # were reordered.  This intentionally no longer matches the fixed tuple.
    missions[8], missions[9] = missions[9], missions[8]

    profile = lah_mode.detect_lah_special_operation(plan)
    assert profile is not None
    assert profile["mode"] == "attack_wait_battle_target"
    assert len(profile["targetBattleCycles"]) == 2

    second_cycle = profile["targetBattleCycles"][1]
    second_target_index = second_cycle["targetIndex"]
    second_target_id = missions[second_target_index]["inputMissionID"]
    target_info = lah_mode.lah_special_info_for_input(plan, second_target_id)
    assert target_info is not None
    _assert_area_internal_cover_info(target_info, second_cycle)

    return_index = second_target_index + 1
    return_info, source_input_id, behavior = lah_mode.lah_special_info_for_index(
        plan,
        profile,
        return_index,
    )
    assert behavior == "target_to_battle_battle_hold"
    assert source_input_id == second_cycle["battlePositionInputMissionID"]
    _assert_area_internal_cover_info(return_info, second_cycle)
