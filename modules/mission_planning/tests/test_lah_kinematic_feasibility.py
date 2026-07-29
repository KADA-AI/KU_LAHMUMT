from __future__ import annotations

import math

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0304,
)
from modules.mission_planning.replanning.triggers.post_attack import (
    pipeline as post_attack,
)


def _waypoint(
    waypoint_id: int,
    *,
    longitude: float,
    altitude: int,
    eta: int,
    speed: float = 40.0,
    target_id: int = 0,
) -> dict:
    waypoint = {
        "waypointID": waypoint_id,
        "isDone": False,
        "coordinate": {
            "latitude": 37.0,
            "longitude": longitude,
            "altitude": altitude,
        },
        "speed": speed,
        "eta": eta,
        "ecf": 0.0,
        "nextWaypointID": 0,
    }
    if target_id:
        waypoint["attack"] = {"targetID": target_id, "weaponType": 2}
    return waypoint


def _fixed_runtime_value(name: str, default):
    return {
        "lah_operational_climb_rate_mps": 5.0,
        "lah_operational_descent_rate_mps": 5.0,
        "lah_max_speed_mps": 72.0,
        "lah_vertical_transition_enabled": True,
        "lah_vertical_transition_tolerance_m": 1.0,
    }.get(name, default)


def test_infeasible_climb_gets_departure_vertical_transition(monkeypatch) -> None:
    monkeypatch.setattr(d0304, "_runtime_value", _fixed_runtime_value)
    target = _waypoint(
        2,
        longitude=127.005,
        altitude=220,
        eta=12,
        target_id=77,
    )
    packet = {
        "aircraftID": 1,
        "lahWaypointList": [
            _waypoint(1, longitude=127.0, altitude=100, eta=0),
            target,
        ],
    }

    inserted = d0304.enforce_lah_kinematic_feasibility_inplace(
        packet,
        preserve_existing_timing=False,
        allocate_waypoint_ids=False,
    )

    waypoints = packet["lahWaypointList"]
    assert inserted == 1
    assert len(waypoints) == 3
    transition = waypoints[1]
    assert transition["coordinate"]["latitude"] == waypoints[0]["coordinate"]["latitude"]
    assert transition["coordinate"]["longitude"] == waypoints[0]["coordinate"]["longitude"]
    assert 100 < transition["coordinate"]["altitude"] < 220
    assert transition["attack"]["targetID"] == 0
    assert waypoints[2]["attack"]["targetID"] == 77
    assert waypoints[2]["coordinate"] == target["coordinate"]
    assert waypoints[0]["eta"] < transition["eta"] < waypoints[2]["eta"]

    # Running the final pass again must not keep adding transition points.
    assert (
        d0304.enforce_lah_kinematic_feasibility_inplace(
            packet,
            preserve_existing_timing=False,
            allocate_waypoint_ids=False,
        )
        == 0
    )
    assert len(packet["lahWaypointList"]) == 3


def test_infeasible_descent_moves_then_descends_at_arrival(monkeypatch) -> None:
    monkeypatch.setattr(d0304, "_runtime_value", _fixed_runtime_value)
    target = _waypoint(
        2,
        longitude=127.005,
        altitude=100,
        eta=12,
        target_id=88,
    )
    packet = {
        "aircraftID": 2,
        "lahWaypointList": [
            _waypoint(1, longitude=127.0, altitude=300, eta=0),
            target,
        ],
    }

    inserted = d0304.enforce_lah_kinematic_feasibility_inplace(
        packet,
        preserve_existing_timing=False,
        allocate_waypoint_ids=False,
    )

    waypoints = packet["lahWaypointList"]
    assert inserted == 1
    transition = waypoints[1]
    assert transition["coordinate"]["latitude"] == target["coordinate"]["latitude"]
    assert transition["coordinate"]["longitude"] == target["coordinate"]["longitude"]
    assert 100 < transition["coordinate"]["altitude"] < 300
    assert transition["attack"]["targetID"] == 0
    assert waypoints[2]["attack"]["targetID"] == 88
    assert waypoints[1]["eta"] < waypoints[2]["eta"]


def test_inserted_transition_gets_reserved_id_and_repairs_next_chain(
    monkeypatch,
) -> None:
    monkeypatch.setattr(d0304, "_runtime_value", _fixed_runtime_value)
    monkeypatch.setattr(
        d0304,
        "_reserve_lah_waypoint_ids",
        lambda count: [9001] if count == 1 else [],
    )
    packet = {
        "aircraftID": 1,
        "lahWaypointList": [
            _waypoint(1, longitude=127.0, altitude=100, eta=0),
            _waypoint(2, longitude=127.005, altitude=220, eta=12),
        ],
    }

    d0304.enforce_lah_kinematic_feasibility_inplace(
        packet,
        preserve_existing_timing=False,
        allocate_waypoint_ids=True,
    )

    waypoints = packet["lahWaypointList"]
    assert [waypoint["waypointID"] for waypoint in waypoints] == [1, 9001, 2]
    assert [waypoint["nextWaypointID"] for waypoint in waypoints] == [9001, 2, 0]


def test_same_xy_vertical_leg_gets_operational_eta_without_duplicate(
    monkeypatch,
) -> None:
    monkeypatch.setattr(d0304, "_runtime_value", _fixed_runtime_value)
    packet = {
        "aircraftID": 2,
        "lahWaypointList": [
            _waypoint(10, longitude=127.0, altitude=505, eta=0, speed=73.61),
            _waypoint(11, longitude=127.0, altitude=696, eta=0, speed=73.61),
        ],
    }

    inserted = d0304.enforce_lah_kinematic_feasibility_inplace(
        packet,
        preserve_existing_timing=False,
        allocate_waypoint_ids=False,
    )

    waypoints = packet["lahWaypointList"]
    assert inserted == 0
    assert len(waypoints) == 2
    assert waypoints[0]["speed"] == 72.0
    assert waypoints[1]["speed"] == 72.0
    assert waypoints[1]["eta"] == math.ceil(191.0 / 5.0)


def test_post_attack_rebase_uses_lah_vertical_timing_not_generic_speed_guess(
    monkeypatch,
) -> None:
    monkeypatch.setattr(d0304, "_runtime_value", _fixed_runtime_value)
    payload = {
        "aircraftID": 2,
        "lahWaypointList": [
            _waypoint(20, longitude=127.0, altitude=505, eta=0, speed=73.61),
            _waypoint(21, longitude=127.0, altitude=696, eta=0, speed=73.61),
            _waypoint(22, longitude=127.01, altitude=696, eta=999, speed=73.61),
        ],
    }

    post_attack._rebase_post_attack_cover_timing(payload)

    waypoints = payload["lahWaypointList"]
    assert [waypoint["eta"] for waypoint in waypoints[:2]] == [0, 39]
    assert waypoints[-1]["eta"] > waypoints[1]["eta"]
    assert waypoints[-1]["speed"] == 72.0
