"""The mission carries only the low-level route to the SIM popup base."""

from __future__ import annotations

from typing import Any

import pytest

from modules.mission_planning.replanning.triggers.attack import pipeline as ap

HIDE: dict[str, Any] = {"latitude": 37.8664, "longitude": 128.2099, "altitude": 700}


def _sequence(
    offset_m: float,
    *,
    regain_cover_coord: dict[str, Any] | None = HIDE,
) -> list[dict[str, Any]]:
    ids = iter(range(1, 200))
    attack = ap._offset_coordinate_m(HIDE, max(5.0, float(offset_m)), 0.0)
    attack["altitude"] = HIDE["altitude"]
    attack["attack_point_at_hide_endpoint"] = True
    attack["attack_point_popup_offset_m"] = max(5.0, float(offset_m))
    attack["attack_altitude_control"] = "sim_los_popup"
    return ap._build_lah_low_level_attack_waypoints(
        template_wp=ap._default_lah_waypoint_template(),
        start_coord=HIDE,
        attack_coord=attack,
        attack_waypoint_id=999,
        waypoint_id_provider=lambda: next(ids),
        target_id=7,
        weapon_type=1,
        speed_mps=60.0,
        regain_cover_coord=regain_cover_coord,
    )


def test_the_attack_point_is_distinct_from_the_hide_point() -> None:
    waypoints = _sequence(0.0)
    attack = waypoints[-1]["coordinate"]

    assert ap._haversine_distance_m(HIDE, attack) == pytest.approx(5.0, abs=0.5)


def test_the_shot_is_marked_on_the_terminal_waypoint_only() -> None:
    waypoints = _sequence(600.0)

    assert waypoints[-1]["waypointID"] == 999
    assert waypoints[-1]["attack"] == {"targetID": 7, "weaponType": 1}
    assert waypoints[-1]["nextWaypointID"] == 0
    assert all(
        waypoint["attack"] == {"targetID": 0, "weaponType": 0}
        for waypoint in waypoints[:-1]
    )


def test_a_lateral_popup_base_keeps_its_low_level_approach_in_the_attack_mission() -> None:
    waypoints = _sequence(600.0)

    assert len(waypoints) > 2
    assert waypoints[0]["coordinate"]["latitude"] == pytest.approx(HIDE["latitude"])
    assert waypoints[-1]["coordinate"]["altitude"] == HIDE["altitude"]


def test_no_planner_side_return_waypoint_is_emitted() -> None:
    waypoints = _sequence(600.0)
    attack_index = next(
        index
        for index, waypoint in enumerate(waypoints)
        if int((waypoint.get("attack") or {}).get("targetID") or 0) == 7
    )

    assert attack_index == len(waypoints) - 1
    assert not any(
        ap._same_lah_ground_position(
            ap._extract_lah_waypoint_coordinate(waypoint), HIDE
        )
        for waypoint in waypoints[attack_index + 1 :]
    )


def test_regain_cover_argument_does_not_change_the_serialized_plan() -> None:
    with_cover = _sequence(600.0, regain_cover_coord=HIDE)
    without_cover = _sequence(600.0, regain_cover_coord=None)

    assert [item["coordinate"] for item in with_cover] == [
        item["coordinate"] for item in without_cover
    ]
    assert [item["attack"] for item in with_cover] == [
        item["attack"] for item in without_cover
    ]


def test_the_timeline_stays_cumulative_and_ordered() -> None:
    waypoints = _sequence(600.0)
    etas = [int(item["eta"]) for item in waypoints]

    assert etas[0] == 0
    assert etas == sorted(etas)
    assert etas[-1] > etas[0]


def test_the_chain_reaches_the_single_terminal_attack_waypoint() -> None:
    waypoints = _sequence(600.0)
    terminals = [
        waypoint
        for waypoint in waypoints
        if int(waypoint.get("nextWaypointID") or 0) == 0
    ]

    assert terminals == [waypoints[-1]]
    assert all(
        int(left["nextWaypointID"]) == int(right["waypointID"])
        for left, right in zip(waypoints, waypoints[1:])
    )
