"""No emitted waypoint may command zero speed.

Observed: an attack replan handed the manned aircraft a conceal route whose
leading waypoints carried ``speed: 0.0``. The aircraft never departed, sat
still, and its plan-deviation grew (369 m and 423 m were measured) while the
operator watched three manned aircraft do nothing. A stop is expressed by
``hovering``/``loiter`` - normal hold waypoints still carry their transit speed -
so a zero in ``speed`` is never a valid command, only a stranded aircraft.
"""

from __future__ import annotations

from typing import Any

import pytest

from modules.mission_planning.runtime.validation.replan_payloads import (
    _MIN_TRANSIT_SPEED_MPS,
    normalize_flight_path_waypoint_speeds_inplace,
)


def _waypoint(waypoint_id: int, speed: Any, *, hover: int = 0) -> dict[str, Any]:
    return {
        "waypointID": waypoint_id,
        "coordinate": {"latitude": 37.95, "longitude": 127.31, "altitude": 236},
        "speed": speed,
        "hovering": {"time": hover},
    }


def test_a_stranded_leading_leg_is_given_the_transit_floor() -> None:
    """The reported shape: route starts with two zero-speed waypoints."""

    payload = {
        "lahWaypointList": [
            _waypoint(15836, 0.0),
            _waypoint(15837, 0.0),
            _waypoint(15838, 30.0),
        ]
    }

    changed = normalize_flight_path_waypoint_speeds_inplace(payload)

    assert changed == 2
    speeds = [row["speed"] for row in payload["lahWaypointList"]]
    assert speeds == [_MIN_TRANSIT_SPEED_MPS, _MIN_TRANSIT_SPEED_MPS, 30.0]


def test_a_hold_waypoint_still_gets_a_transit_speed() -> None:
    """Hovering is what stops the aircraft, not a zero speed."""

    payload = {"lahWaypointList": [_waypoint(1, 0.0, hover=300)]}

    normalize_flight_path_waypoint_speeds_inplace(payload)

    assert payload["lahWaypointList"][0]["speed"] == _MIN_TRANSIT_SPEED_MPS
    assert payload["lahWaypointList"][0]["hovering"] == {"time": 300}


def test_speeds_above_the_floor_are_untouched() -> None:
    payload = {"lahWaypointList": [_waypoint(1, 40.0), _waypoint(2, 71.34)]}

    assert normalize_flight_path_waypoint_speeds_inplace(payload) == 0
    assert [row["speed"] for row in payload["lahWaypointList"]] == [40.0, 71.34]


def test_uav_and_generic_waypoint_lists_are_covered() -> None:
    payload = {
        "waypointList": [_waypoint(1, 0.0)],
        "uavWaypointList": [_waypoint(2, 0.0)],
    }

    assert normalize_flight_path_waypoint_speeds_inplace(payload) == 2


@pytest.mark.parametrize("bad", ["fast", True, None, float("nan")])
def test_unusable_values_are_left_for_validation_to_reject(bad: Any) -> None:
    """Never invent a speed for a value the schema should be rejecting."""

    payload = {"lahWaypointList": [_waypoint(1, bad)]}

    normalize_flight_path_waypoint_speeds_inplace(payload)

    emitted = payload["lahWaypointList"][0]["speed"]
    assert emitted is bad or emitted != emitted  # NaN compares unequal to itself


def test_the_attack_and_post_attack_pipelines_apply_the_floor() -> None:
    """The normalizer only helps if it runs where paths are serialized."""

    import inspect

    from modules.mission_planning.replanning.triggers.attack import pipeline as attack
    from modules.mission_planning.replanning.triggers.post_attack import (
        pipeline as post_attack,
    )

    for module in (attack, post_attack):
        source = inspect.getsource(module)
        altitude_calls = source.count(
            "normalize_flight_path_waypoint_altitudes_inplace("
        )
        speed_calls = source.count("normalize_flight_path_waypoint_speeds_inplace(")
        assert speed_calls >= altitude_calls > 0
