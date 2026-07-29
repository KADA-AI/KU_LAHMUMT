"""Type 2 (지상작전부대 기동여건 보장) manned-aircraft hold sequence.

Do not confuse this with the Type 1 attack flow: Type 2 places the manned
aircraft by regionType across the 8-mission package, and 목표지역(6) carries two
different geometries - the 통로 is its line mission and the region itself is its
area mission.
"""

from __future__ import annotations

from typing import Any

import pytest

from modules.mission_planning.pipelines.ground_maneuver_mode import (
    build_ground_maneuver_lah_sequence,
    ground_maneuver_lah_info_for_input,
    resolve_ground_maneuver_lah_anchors,
)


def _line(*points: tuple[float, float]) -> dict[str, Any]:
    return {
        "lineList": [
            {"coordinateList": [{"latitude": lat, "longitude": lon} for lat, lon in points]}
        ]
    }


def _area(*points: tuple[float, float]) -> dict[str, Any]:
    return {
        "areaList": [
            {"coordinateList": [{"latitude": lat, "longitude": lon} for lat, lon in points]}
        ]
    }


def _missions() -> list[dict[str, Any]]:
    """The operational 8-mission Type 2 package."""

    return [
        # 1 협업기동 / ACP
        {"inputMissionID": 1, "inputMissionType": 1, "regionType": 3,
         "missionDetail": _line((37.90, 127.30), (37.92, 127.32))},
        # 2 협업기동 / 목표지역  -> 통로(line)
        {"inputMissionID": 2, "inputMissionType": 1, "regionType": 6,
         "missionDetail": _line((37.94, 127.34), (37.96, 127.36), (37.98, 127.38))},
        # 3 협업지상부대엄호 / 목표지역 -> 목표지역(area)
        {"inputMissionID": 3, "inputMissionType": 5, "regionType": 6,
         "missionDetail": _area((37.95, 127.35), (37.99, 127.35), (37.99, 127.39), (37.95, 127.39))},
        # 4 협업기동 / 경계지역 (lineList)
        {"inputMissionID": 4, "inputMissionType": 1, "regionType": 7,
         "missionDetail": _line((37.94, 127.40), (37.96, 127.42))},
        # 5 협업경계 / 경계지역 (areaList)
        {"inputMissionID": 5, "inputMissionType": 3, "regionType": 7,
         "missionDetail": _area((37.94, 127.40), (37.97, 127.40), (37.97, 127.43), (37.94, 127.43))},
        # 6 협업기동 / 목표지역
        {"inputMissionID": 6, "inputMissionType": 1, "regionType": 6,
         "missionDetail": _line((37.96, 127.36), (37.98, 127.38))},
        # 7 협업기동 / ACP
        {"inputMissionID": 7, "inputMissionType": 1, "regionType": 3,
         "missionDetail": _line((37.93, 127.33), (37.91, 127.31))},
        # 8 협업기동 / 통제권변경지역
        {"inputMissionID": 8, "inputMissionType": 1, "regionType": 2,
         "missionDetail": _line((37.90, 127.30), (37.88, 127.28))},
    ]


def _sequence() -> dict[int, dict[str, Any]]:
    rows = build_ground_maneuver_lah_sequence(
        {"inputMissionPackageType": 2, "inputMissionList": _missions()},
        package_type=2,
    )
    assert rows, "Type 2 must produce a manned sequence"
    return {int(row["inputMissionID"]): row for row in rows}


def _coord(row: dict[str, Any]) -> tuple[float, float]:
    point = row["individualMissionInfo"]["coordinateList"][0]
    return float(point["latitude"]), float(point["longitude"])


def test_corridor_and_destination_area_are_resolved_from_region_six() -> None:
    """목표지역(6) holds both the 통로 line and the region area."""

    anchors = resolve_ground_maneuver_lah_anchors(_missions())
    assert anchors is not None
    assert _round(anchors["corridorStart"]) == (37.94, 127.34)
    # Midpoint measured along the polyline, not between its endpoints.
    assert _round(anchors["corridorMid"]) == (37.96, 127.36)
    assert _round(anchors["destinationInside"]) == (37.97, 127.37)


def _round(coord: dict[str, Any]) -> tuple[float, float]:
    return round(float(coord["latitude"]), 5), round(float(coord["longitude"]), 5)


def test_manned_aircraft_trails_the_maneuver_leg_before_the_guard_phase() -> None:
    """While the UAVs work a 목표지역 the manned aircraft sits one leg back.

    Mission 2's hold is the middle of mission 1's leg and mission 3's is the
    middle of mission 2's - never the region the UAVs are working.
    """

    rows = _sequence()
    assert rows[2]["behavior"] == "previous_maneuver_mid_hold"
    assert _coord(rows[2]) == pytest.approx((37.91, 127.31))
    assert rows[3]["behavior"] == "previous_maneuver_mid_hold"
    assert _coord(rows[3]) == pytest.approx((37.96, 127.36))


def test_manned_aircraft_holds_the_destination_area_from_the_guard_phase_on() -> None:
    """경계(4,5), the 목표지역 sweep(6) and the ACP egress(7) share one hold.

    Only the UAVs move on at mission 7; the manned aircraft stays in the region
    until the 통제권변경 leg begins.
    """

    rows = _sequence()
    for mission_id in (4, 5, 6, 7):
        assert rows[mission_id]["behavior"] == "destination_area_hold"
        assert _coord(rows[mission_id]) == pytest.approx((37.97, 127.37))


def test_start_and_handover_anchors() -> None:
    rows = _sequence()
    assert rows[1]["behavior"] == "staging_hold"
    assert _coord(rows[1]) == pytest.approx((37.90, 127.30))
    # Final catch-up order: prior LINE midpoint -> ACP endpoint -> final point.
    assert rows[8]["behavior"] == "previous_mid_to_acp2_to_control_end_follow"
    route = rows[8]["individualMissionInfo"]["coordinateList"]
    assert [
        (point["latitude"], point["longitude"]) for point in route
    ] == pytest.approx([
        (37.92, 127.32),
        (37.91, 127.31),
        (37.88, 127.28),
    ])


def test_no_hold_sits_inside_the_region_the_uavs_are_working() -> None:
    """The reported defect: a hold that walked into the 목표지역 too early."""

    rows = _sequence()
    destination_area = (37.97, 127.37)
    for mission_id in (1, 2, 3):
        assert _coord(rows[mission_id]) != pytest.approx(destination_area)


def test_holds_are_points_and_final_control_transfer_is_an_ordered_route() -> None:
    rows = _sequence()
    for mission_id, row in rows.items():
        info = row["individualMissionInfo"]
        if mission_id == 8:
            assert info["individualMissionType"] == 7
            assert info["patternType"] == 10
            assert info["_lahPreserveLineEndpoints"] is True
            assert len(info["coordinateList"]) == 3
        else:
            assert info["individualMissionType"] == 9
            assert info["patternType"] == 12
            assert len(info["coordinateList"]) == 1


def test_a_package_with_no_maneuver_leg_to_trail_falls_back_to_the_corridor() -> None:
    """Missing 협업기동 line geometry must degrade, never drop the sequence."""

    missions = _missions()
    # Mission 1 keeps its line so the ACP anchor still resolves, but neither
    # pre-guard mission is a 협업기동임무 any more, so there is no leg to trail.
    missions[0]["inputMissionType"] = 7
    missions[1]["inputMissionType"] = 5
    missions[1]["missionDetail"] = _area(
        (37.95, 127.35), (37.99, 127.35), (37.99, 127.39)
    )

    rows = build_ground_maneuver_lah_sequence(
        {"inputMissionPackageType": 2, "inputMissionList": missions},
        package_type=2,
    )
    assert rows and len(rows) == len(missions)
    behaviors = {int(row["inputMissionID"]): row["behavior"] for row in rows}
    assert behaviors[3] in {"corridor_mid_hold", "corridor_start_hold", "acp1_hold"}
    assert behaviors[4] == "destination_area_hold"


def test_air_assault_packages_keep_their_own_egress() -> None:
    """Type 3 lands rather than holding a 목표지역, so it must not trail."""

    missions = _missions()
    missions[5]["regionType"] = 9  # 착륙지대
    rows = build_ground_maneuver_lah_sequence(
        {"inputMissionPackageType": 3, "inputMissionList": missions},
        package_type=3,
    )
    assert rows
    behaviors = {int(row["inputMissionID"]): row["behavior"] for row in rows}
    assert behaviors[6] == "destination_hold"
    assert behaviors[7] == "destination_to_acp2_follow"
    assert behaviors[8] == "previous_mid_to_acp2_to_control_end_follow"


def test_air_assault_acp_coded_return_is_not_the_following_real_acp() -> None:
    from modules.mission_planning.pipelines.ground_maneuver_mode import (
        detect_ground_maneuver_attack_profile,
    )

    missions = _missions()
    missions[5]["regionType"] = 3  # branch return LINE uses the ACP alias
    plan = {"inputMissionPackageType": 3, "inputMissionList": missions}

    rows = build_ground_maneuver_lah_sequence(plan, package_type=3)
    assert rows
    behaviors = {int(row["inputMissionID"]): row["behavior"] for row in rows}
    assert behaviors[6] == "destination_hold"
    assert behaviors[7] == "destination_to_acp2_follow"

    profile = detect_ground_maneuver_attack_profile(plan, package_type=3)
    assert profile is not None
    assert profile["targetHoldInputMissionID"] == 6


def test_a_replan_reuses_the_ladder_instead_of_the_uav_area_centroid() -> None:
    """다음 협업기저임무 must not park the manned aircraft in the 목표지역.

    The replan rebuilds the manned row from the UAVs' replacement geometry, and
    an area mission collapses to its centroid - the middle of the very region
    the UAVs are working. The package ladder has to win.
    """

    from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
        _area_anchor_coordinate,
        _replace_geometry_from_piece,
    )

    missions = _missions()
    plan = {"inputMissionPackageType": 2, "inputMissionList": missions}
    # Mission 3 협업지상부대엄호: the UAVs sweep the 목표지역 area.
    target_area = missions[2]["missionDetail"]["areaList"]
    centroid = _area_anchor_coordinate(target_area)

    info = ground_maneuver_lah_info_for_input(plan, 3)
    assert info is not None
    hold = info["coordinateList"][0]
    # Mission 2's polyline midpoint, one leg behind - not the region centroid.
    assert (hold["latitude"], hold["longitude"]) == pytest.approx((37.96, 127.36))
    assert (hold["latitude"], hold["longitude"]) != pytest.approx(
        (centroid["latitude"], centroid["longitude"])
    )

    # A stale template carrying the old areaList must not re-inject the centroid.
    merged = _replace_geometry_from_piece(
        {"individualMissionType": 9, "patternType": 12, "areaList": target_area},
        info,
    )
    assert merged["coordinateList"] == info["coordinateList"]
    assert not merged.get("areaList")


def test_replan_preserves_the_terminal_midpoint_acp_endpoint_route() -> None:
    from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
        _replace_geometry_from_piece,
    )

    plan = {"inputMissionPackageType": 2, "inputMissionList": _missions()}
    info = ground_maneuver_lah_info_for_input(plan, 8)
    assert info is not None

    merged = _replace_geometry_from_piece(
        {
            "individualMissionType": 9,
            "patternType": 12,
            "coordinateList": [{"latitude": 0.0, "longitude": 0.0}],
        },
        info,
    )

    assert merged["coordinateList"] == info["coordinateList"]
    assert merged["_lahPreserveLineEndpoints"] is True


def test_a_replan_of_a_non_branch_package_keeps_its_own_geometry() -> None:
    assert ground_maneuver_lah_info_for_input(
        {"inputMissionPackageType": 1, "inputMissionList": _missions()}, 3
    ) is None


def test_type_one_packages_are_untouched() -> None:
    assert (
        build_ground_maneuver_lah_sequence(
            {"inputMissionPackageType": 1, "inputMissionList": _missions()},
            package_type=1,
        )
        is None
    )
