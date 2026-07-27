"""A pop-up attack is climb, fire, sink.

Without the descent the aircraft holds the firing altitude, and line of sight
being symmetric, that is exactly the altitude the target can see it from.
"""

from __future__ import annotations

import itertools
from typing import Any

from modules.mission_planning.replanning.triggers.attack import pipeline as ap

HIDE = {"latitude": 37.955997, "longitude": 127.321100, "altitude": 212}
ATTACK = {"latitude": 37.955997, "longitude": 127.321100, "altitude": 231}
OFFSET_ATTACK = {"latitude": 37.957200, "longitude": 127.322300, "altitude": 350}
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
    *,
    attack_coord: dict[str, Any] = ATTACK,
) -> list[dict[str, Any]]:
    ids = itertools.count(9001)
    return ap._build_lah_low_level_attack_waypoints(
        template_wp=TEMPLATE,
        start_coord=START,
        attack_coord=attack_coord,
        attack_waypoint_id=9000,
        waypoint_id_provider=lambda: next(ids),
        target_id=7,
        weapon_type=3,
        speed_mps=30.0,
        route_coordinates=[START, ATTACK],
        regain_cover_coord=regain_cover_coord,
    )


def test_the_path_returns_to_the_exact_certified_cover_coordinate() -> None:
    waypoints = _waypoints(HIDE)
    last = waypoints[-1]

    assert last["coordinate"]["altitude"] == HIDE["altitude"]
    assert last["coordinate"]["latitude"] == HIDE["latitude"]
    assert last["coordinate"]["longitude"] == HIDE["longitude"]
    # The descent carries no attack payload of its own.
    assert int((last.get("attack") or {}).get("targetID") or 0) == 0


def test_a_lateral_popup_still_returns_to_the_certified_hide_xy() -> None:
    last = _waypoints(HIDE, attack_coord=OFFSET_ATTACK)[-1]

    assert last["coordinate"] == HIDE
    assert int(last["hovering"]["time"]) == ap._attack_cover_hold_seconds()
    assert int((last.get("attack") or {}).get("targetID") or 0) == 0


def test_exactly_one_terminal_waypoint_and_the_chain_reaches_it() -> None:
    waypoints = _waypoints(HIDE)

    terminals = [w for w in waypoints if int(w.get("nextWaypointID") or 0) == 0]
    assert len(terminals) == 1
    assert terminals[0] is waypoints[-1]

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


def test_the_attack_waypoint_keeps_its_payload_and_is_still_unique() -> None:
    waypoints = _waypoints(HIDE)
    attacks = [w for w in waypoints if int((w.get("attack") or {}).get("targetID") or 0) > 0]

    assert len(attacks) == 1
    assert attacks[0]["coordinate"]["altitude"] == ATTACK["altitude"]
    assert int(attacks[0]["attack"]["weaponType"]) == 3


def test_without_a_cover_coordinate_the_previous_shape_is_kept() -> None:
    legacy = _waypoints(None)

    assert int((legacy[-1].get("attack") or {}).get("targetID") or 0) == 7
    assert int(legacy[-1].get("nextWaypointID") or 0) == 0


def test_no_descent_is_emitted_when_there_was_no_climb() -> None:
    """Cover at or above the firing altitude means no pop-up happened."""

    level = dict(HIDE)
    level["altitude"] = ATTACK["altitude"]
    assert int((_waypoints(level)[-1].get("attack") or {}).get("targetID") or 0) == 7

    higher = dict(HIDE)
    higher["altitude"] = ATTACK["altitude"] + 50
    assert int((_waypoints(higher)[-1].get("attack") or {}).get("targetID") or 0) == 7


def test_the_knob_restores_the_previous_behaviour(monkeypatch) -> None:
    monkeypatch.setattr(
        ap,
        "get_runtime_attack_int",
        lambda key, default=0, *a, **k: (
            0 if key == "attack_regain_cover_enabled" else default
        ),
    )
    assert int((_waypoints(HIDE)[-1].get("attack") or {}).get("targetID") or 0) == 7


def _prelude(hide: dict[str, Any], ids: "itertools.count") -> list[dict[str, Any]]:
    plan = {
        "applied": True,
        "routeWaypoints": [
            {
                "latitude": START["latitude"], "longitude": START["longitude"],
                "altitude": 300.0, "etaS": 0.0, "speedMps": 30.0, "distanceM": 0.0,
            },
            {
                "latitude": hide["latitude"], "longitude": hide["longitude"],
                "altitude": float(hide["altitude"]), "etaS": 40.0,
                "speedMps": 10.0, "distanceM": 900.0,
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
    prelude = _prelude(HIDE, ids)
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


def test_the_aircraft_waits_in_cover_before_and_after_the_shot() -> None:
    chain = _full_chain()
    dwell = ap._attack_cover_hold_seconds()

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

    assert any(index < attack_index for index in holds), holds
    assert any(index > attack_index for index in holds), holds
    for index in holds:
        assert int(chain[index]["hovering"]["time"]) == dwell
        # Both dwells happen at the cover altitude, not at the firing altitude.
        assert int(chain[index]["coordinate"]["altitude"]) == HIDE["altitude"]


def test_vertical_popup_has_no_intermediate_detour_or_duplicate_altitude() -> None:
    chain = _full_chain()
    attack_index = next(
        index
        for index, wp in enumerate(chain)
        if int((wp.get("attack") or {}).get("targetID") or 0) > 0
    )
    cover_indices = [
        index
        for index, wp in enumerate(chain)
        if wp["coordinate"]["latitude"] == HIDE["latitude"]
        and wp["coordinate"]["longitude"] == HIDE["longitude"]
        and int(wp["coordinate"]["altitude"]) == HIDE["altitude"]
        and int((wp.get("hovering") or {}).get("time") or 0) > 0
    ]

    assert cover_indices == [attack_index - 1, attack_index + 1]
    assert chain[attack_index]["coordinate"]["latitude"] == HIDE["latitude"]
    assert chain[attack_index]["coordinate"]["longitude"] == HIDE["longitude"]


def test_the_dwell_is_never_folded_into_eta() -> None:
    """ICD 0304 eta is arrival time; a hover must not push it out."""

    chain = _full_chain()
    etas = [int(wp.get("eta") or 0) for wp in chain]
    assert all(later >= earlier for earlier, later in zip(etas, etas[1:])), etas

    attack_index = next(
        index
        for index, wp in enumerate(chain)
        if int((wp.get("attack") or {}).get("targetID") or 0) > 0
    )
    descent = chain[-1]
    # Only the descent itself separates the shot from regaining cover.
    assert int(descent["eta"]) - int(chain[attack_index]["eta"]) < ap._attack_cover_hold_seconds() + 5


def test_the_chain_still_has_exactly_one_terminal() -> None:
    chain = _full_chain()
    terminals = [wp for wp in chain if int(wp.get("nextWaypointID") or 0) == 0]
    assert len(terminals) == 1
    assert terminals[0] is chain[-1]


def test_both_cover_touches_are_published_for_sim_display(monkeypatch) -> None:
    """The concealment ground has no ICD field, so SIM learns it out of band."""

    captured: dict[str, Any] = {}

    from modules.mission_planning.pipelines import lah_tactical_point_log as sidecar

    def _spy(path_id, **kwargs):
        captured["pathID"] = path_id
        captured.update(kwargs)
        return True

    monkeypatch.setattr(sidecar, "record_tactical_points", _spy)

    chain = _full_chain()
    ap._record_lah_tactical_points(
        path_id=777,
        waypoints=chain,
        plan={"applied": True, "status": "green_valid"},
        role="attacker",
        conceal_coordinate=HIDE,
    )

    conceal = list(captured.get("conceal_waypoint_ids") or [])
    by_id = {int(wp["waypointID"]): wp for wp in chain}
    assert len(conceal) == 2, conceal
    for waypoint_id in conceal:
        waypoint = by_id[int(waypoint_id)]
        # Only the waypoints actually sitting in cover, never the pop-up.
        assert int(waypoint["coordinate"]["altitude"]) == HIDE["altitude"]
        assert int((waypoint.get("attack") or {}).get("targetID") or 0) == 0


def test_the_climb_and_the_shot_are_not_marked_as_cover(monkeypatch) -> None:
    captured: dict[str, Any] = {}
    from modules.mission_planning.pipelines import lah_tactical_point_log as sidecar

    monkeypatch.setattr(
        sidecar, "record_tactical_points", lambda path_id, **kw: captured.update(kw) or True
    )

    chain = _full_chain()
    ap._record_lah_tactical_points(
        path_id=777, waypoints=chain, plan=None, role="attacker", conceal_coordinate=HIDE
    )
    conceal = {int(value) for value in (captured.get("conceal_waypoint_ids") or [])}

    exposed = [
        int(wp["waypointID"])
        for wp in chain
        if int(wp["coordinate"]["altitude"]) > HIDE["altitude"]
        and ap._same_lah_ground_position(
            ap._extract_lah_waypoint_coordinate(wp), HIDE
        )
    ]
    assert exposed, "the pop-up must share the cover ground position"
    assert not (conceal & set(exposed))
