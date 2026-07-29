"""Popup climb/descent is SIM state and is absent from the flight plan."""

from __future__ import annotations

import itertools
from typing import Any

from modules.mission_planning.replanning.triggers.attack import pipeline as ap

HIDE = {"latitude": 37.955997, "longitude": 127.321100, "altitude": 212}
ATTACK = ap._offset_coordinate_m(HIDE, 120.0, 20.0)
ATTACK["altitude"] = 212
ATTACK["attack_point_at_hide_endpoint"] = True
ATTACK["attack_altitude_control"] = "sim_los_popup"
START = {"latitude": 37.950000, "longitude": 127.315000, "altitude": 300}

TEMPLATE: dict[str, Any] = {
    "waypointID": 0,
    "isDone": False,
    "coordinate": {"latitude": 0.0, "longitude": 0.0, "altitude": 0},
    "speed": 30.0,
    "eta": 0,
    "ecf": 0.0,
    "nextWaypointID": 0,
    "hovering": {"time": 0},
    "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
    "attack": {"targetID": 0, "weaponType": 0},
}


def _waypoints(
    regain_cover_coord: dict[str, Any] | None,
) -> list[dict[str, Any]]:
    ids = itertools.count(9001)
    return ap._build_lah_low_level_attack_waypoints(
        template_wp=TEMPLATE,
        start_coord=HIDE,
        attack_coord=ATTACK,
        attack_waypoint_id=9000,
        waypoint_id_provider=lambda: next(ids),
        target_id=7,
        weapon_type=3,
        speed_mps=30.0,
        route_coordinates=[HIDE, ATTACK],
        regain_cover_coord=regain_cover_coord,
    )


def test_cover_return_is_not_serialized_even_when_supplied() -> None:
    waypoints = _waypoints(HIDE)

    assert waypoints[-1]["coordinate"] == {
        "latitude": ATTACK["latitude"],
        "longitude": ATTACK["longitude"],
        "altitude": ATTACK["altitude"],
    }
    assert int((waypoints[-1]["attack"]).get("targetID") or 0) == 7


def test_regain_cover_argument_is_compatibility_only() -> None:
    with_cover = _waypoints(HIDE)
    without_cover = _waypoints(None)

    assert [waypoint["coordinate"] for waypoint in with_cover] == [
        waypoint["coordinate"] for waypoint in without_cover
    ]


def test_exactly_one_terminal_waypoint_and_the_chain_reaches_it() -> None:
    waypoints = _waypoints(HIDE)
    terminals = [w for w in waypoints if int(w.get("nextWaypointID") or 0) == 0]

    assert terminals == [waypoints[-1]]
    by_id = {int(w["waypointID"]): w for w in waypoints}
    visited: list[int] = []
    cursor = int(waypoints[0]["waypointID"])
    while cursor and cursor in by_id and cursor not in visited:
        visited.append(cursor)
        cursor = int(by_id[cursor].get("nextWaypointID") or 0)
    assert len(visited) == len(waypoints), visited


def test_eta_stays_monotonic_and_inside_uint32() -> None:
    etas = [int(w.get("eta") or 0) for w in _waypoints(HIDE)]

    assert all(later >= earlier for earlier, later in zip(etas, etas[1:])), etas
    assert all(0 <= value <= 0xFFFFFFFF for value in etas)


def test_the_attack_waypoint_is_unique_and_terminal() -> None:
    waypoints = _waypoints(HIDE)
    attacks = [
        w
        for w in waypoints
        if int((w.get("attack") or {}).get("targetID") or 0) > 0
    ]

    assert attacks == [waypoints[-1]]
    assert int(attacks[0]["attack"]["weaponType"]) == 3


def _prelude(ids: "itertools.count") -> list[dict[str, Any]]:
    plan = {
        "applied": True,
        "routeWaypoints": [
            {
                "latitude": START["latitude"],
                "longitude": START["longitude"],
                "altitude": 300.0,
                "etaS": 0.0,
                "speedMps": 30.0,
                "distanceM": 0.0,
            },
            {
                "latitude": HIDE["latitude"],
                "longitude": HIDE["longitude"],
                "altitude": float(HIDE["altitude"]),
                "etaS": 40.0,
                "speedMps": 10.0,
                "distanceM": 900.0,
            },
        ],
    }
    return ap._build_lah_tactical_route_waypoints(
        template_wp=TEMPLATE,
        plan=plan,
        waypoint_id_provider=lambda: next(ids),
        terminal_hover_seconds=ap._attack_cover_hold_seconds(),
    )


def _full_chain() -> list[dict[str, Any]]:
    ids = itertools.count(9001)
    prelude = _prelude(ids)
    attack_leg = ap._build_lah_low_level_attack_waypoints(
        template_wp=TEMPLATE,
        start_coord=HIDE,
        attack_coord=ATTACK,
        attack_waypoint_id=9500,
        waypoint_id_provider=lambda: next(ids),
        target_id=7,
        weapon_type=3,
        speed_mps=30.0,
        route_coordinates=[HIDE, ATTACK],
        regain_cover_coord=HIDE,
    )
    return ap._prepend_lah_tactical_waypoints(prelude, attack_leg)


def test_the_aircraft_waits_in_cover_only_before_the_attack() -> None:
    chain = _full_chain()
    attack_index = next(
        index
        for index, wp in enumerate(chain)
        if int((wp.get("attack") or {}).get("targetID") or 0) > 0
    )
    holds = [
        index
        for index, wp in enumerate(chain)
        if int((wp.get("hovering") or {}).get("time") or 0) > 0
    ]

    assert holds
    assert all(index < attack_index for index in holds)
    assert attack_index == len(chain) - 1


def test_only_pre_attack_points_are_published_as_concealment(monkeypatch) -> None:
    captured: dict[str, Any] = {}
    from modules.mission_planning.pipelines import lah_tactical_point_log as sidecar

    monkeypatch.setattr(
        sidecar,
        "record_tactical_points",
        lambda path_id, **kwargs: captured.update(kwargs) or True,
    )
    chain = _full_chain()
    ap._record_lah_tactical_points(
        path_id=777,
        waypoints=chain,
        plan={"applied": True, "status": "green_valid"},
        role="attacker",
        conceal_coordinate=HIDE,
    )

    conceal = [int(value) for value in captured["conceal_waypoint_ids"]]
    by_id = {int(wp["waypointID"]): wp for wp in chain}
    assert conceal
    assert any(
        int((by_id[waypoint_id].get("hovering") or {}).get("time") or 0) > 0
        for waypoint_id in conceal
    )
    assert all(
        int((by_id[waypoint_id].get("attack") or {}).get("targetID") or 0) == 0
        for waypoint_id in conceal
    )
    assert int(chain[-1]["waypointID"]) not in conceal
