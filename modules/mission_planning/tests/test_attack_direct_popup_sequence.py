"""Pop up straight from cover: hide, climb, fire, sink. Nothing in between.

The firing point is solved inside the hide point's own neighbourhood, so the
aircraft just climbs towards it - diagonally when the search stepped aside for a
lower sightline. Routing that through the terrain follower inserted a separate
low-level approach for a couple of hundred metres, which is neither faster nor
more covered, and it replaced the certified hide altitude with the router's own
DEM floor.
"""

from __future__ import annotations

from typing import Any

import pytest

from modules.mission_planning.replanning.triggers.attack import pipeline as ap

HIDE: dict[str, Any] = {"latitude": 37.8664, "longitude": 128.2099, "altitude": 700}


def _sequence(offset_m: float, *, flagged: bool = True, altitude: int = 1150):
    ids = iter(range(1, 200))
    attack = (
        dict(HIDE)
        if offset_m <= 0.0
        else ap._offset_coordinate_m(HIDE, float(offset_m), 0.0)
    )
    attack = dict(attack)
    attack["altitude"] = altitude
    if flagged:
        attack["attack_point_at_hide_endpoint"] = True
        if offset_m > 0.0:
            attack["attack_point_popup_offset_m"] = float(offset_m)
    return ap._build_lah_low_level_attack_waypoints(
        template_wp=ap._default_lah_waypoint_template(),
        start_coord=HIDE,
        attack_coord=attack,
        attack_waypoint_id=999,
        waypoint_id_provider=lambda: next(ids),
        target_id=7,
        weapon_type=1,
        speed_mps=60.0,
        regain_cover_coord=HIDE,
    )


def test_a_diagonal_popup_is_exactly_hide_fire_cover() -> None:
    waypoints = _sequence(600.0)

    assert len(waypoints) == 3
    assert waypoints[0]["coordinate"]["altitude"] == HIDE["altitude"]
    assert waypoints[1]["waypointID"] == 999
    assert waypoints[2]["coordinate"]["altitude"] == HIDE["altitude"]


def test_the_shot_is_marked_on_the_climb_waypoint_only() -> None:
    waypoints = _sequence(600.0)

    assert waypoints[1]["attack"] == {"targetID": 7, "weaponType": 1}
    assert waypoints[0]["attack"] == {"targetID": 0, "weaponType": 0}
    assert waypoints[2]["attack"] == {"targetID": 0, "weaponType": 0}


def test_the_aircraft_dwells_in_cover_either_side_of_the_shot() -> None:
    waypoints = _sequence(600.0)
    dwell = ap._attack_cover_hold_seconds()

    assert waypoints[0]["hovering"]["time"] == dwell
    assert waypoints[2]["hovering"]["time"] == dwell
    assert waypoints[1]["hovering"]["time"] == 0


def test_a_straight_vertical_popup_keeps_the_same_three_waypoints() -> None:
    waypoints = _sequence(0.0)

    assert len(waypoints) == 3
    assert waypoints[1]["coordinate"]["latitude"] == pytest.approx(HIDE["latitude"])
    assert waypoints[1]["coordinate"]["longitude"] == pytest.approx(HIDE["longitude"])


def test_the_diagonal_leg_is_charged_for_travel_not_just_climb() -> None:
    """Charging only the climb would understate a shallow diagonal pop-up."""

    # A 20 m climb takes a few seconds; 600 m at 60 m/s takes ten.  Travel wins.
    near = _sequence(0.0, altitude=720)
    far = _sequence(600.0, altitude=720)

    assert far[1]["eta"] > near[1]["eta"]
    assert far[1]["eta"] >= 10


def test_a_steep_popup_is_charged_for_the_climb_not_the_traverse() -> None:
    """The two happen together, so the leg costs whichever is slower."""

    # A 450 m climb outlasts 600 m of ground at 60 m/s.
    steep = _sequence(600.0, altitude=1150)
    vertical = _sequence(0.0, altitude=1150)

    assert steep[1]["eta"] == vertical[1]["eta"]


def test_the_timeline_stays_cumulative_and_ordered() -> None:
    waypoints = _sequence(600.0)
    etas = [int(item["eta"]) for item in waypoints]

    assert etas[0] == 0
    assert etas == sorted(etas)
    assert etas[2] > etas[1]


def test_the_chain_links_hide_to_shot_to_cover() -> None:
    waypoints = _sequence(600.0)

    assert waypoints[0]["nextWaypointID"] == waypoints[1]["waypointID"]
    assert waypoints[1]["nextWaypointID"] == waypoints[2]["waypointID"]
    assert waypoints[2]["nextWaypointID"] == 0


def test_an_attack_point_not_from_the_hide_endpoint_still_approaches_low() -> None:
    """A separately computed attack point can be far away; keep the router."""

    waypoints = _sequence(600.0, flagged=False)

    assert len(waypoints) > 3
