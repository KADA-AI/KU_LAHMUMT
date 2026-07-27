"""While one manned aircraft strikes, the rest wait in cover - not wherever.

Non-firing manned aircraft used to receive no enemy-contact context at all, so
they never ran a concealment solve and simply held position for a fixed five
minutes regardless of how long the strike took.
"""

from __future__ import annotations

from typing import Any

from modules.mission_planning.replanning.triggers.attack import pipeline as ap

CTX: dict[str, Any] = {
    "_selected_manned_aircraft": [
        {"aircraft_id": 1, "coordinate": {"latitude": 37.830, "longitude": 128.113}},
        {"aircraft_id": 2, "coordinate": {"latitude": 37.831, "longitude": 128.115}},
        {"aircraft_id": 3, "coordinate": {"latitude": 37.832, "longitude": 128.114}},
    ],
    "_attack_target_list": [
        {"coordinate": {"latitude": 37.870, "longitude": 128.127}},
    ],
}
STATE: dict[str, Any] = {"coordinate": {"latitude": 37.831, "longitude": 128.114}}


def test_the_wait_is_sized_to_the_strike_not_to_a_fixed_five_minutes() -> None:
    seconds = ap._attack_wait_hold_seconds(CTX, STATE)

    assert seconds is not None
    # Closing ~4.5 km at attack speed plus the cover dwell either side.
    assert 40 <= seconds <= 200
    assert seconds != ap.get_runtime_attack_int("lah_hold_seconds", 300)


def test_the_wait_covers_the_run_in_and_both_cover_dwells() -> None:
    seconds = ap._attack_wait_hold_seconds(CTX, STATE)
    assert seconds is not None

    origin = ap._average_coordinate(ap._manned_group_coordinates_from_ctx(CTX))
    target = ap._attack_group_target_coordinate(CTX)
    distance_m = ap._haversine_distance_m(origin, target)
    travel_s = float(distance_m) / ap._lah_max_attack_speed_mps()
    dwell_s = 2 * ap._attack_cover_hold_seconds()

    assert seconds >= int(travel_s)
    assert seconds >= dwell_s


def test_the_wait_is_clamped_into_a_sane_band() -> None:
    far = {
        "_selected_manned_aircraft": CTX["_selected_manned_aircraft"],
        "_attack_target_list": [
            {"coordinate": {"latitude": 39.5, "longitude": 130.0}},
        ],
    }
    near = {
        "_selected_manned_aircraft": CTX["_selected_manned_aircraft"],
        "_attack_target_list": [
            {"coordinate": {"latitude": 37.8310, "longitude": 128.1141}},
        ],
    }

    minimum_s = ap.get_runtime_attack_int("lah_wait_hold_min_seconds", 30)
    maximum_s = ap.get_runtime_attack_int("lah_wait_hold_max_seconds", 600)

    assert ap._attack_wait_hold_seconds(far, STATE) == maximum_s
    assert ap._attack_wait_hold_seconds(near, STATE) >= minimum_s


def test_unknown_geometry_keeps_the_configured_hold() -> None:
    assert ap._attack_wait_hold_seconds({}, STATE) is None
    assert ap._attack_wait_hold_seconds(CTX, {}) is not None  # group still known
    assert ap._attack_wait_hold_seconds({}, {}) is None


def test_every_manned_aircraft_in_the_branch_gets_the_contact_context() -> None:
    """A wingman cannot take cover from contacts it was never told about."""

    import inspect

    source = inspect.getsource(ap)
    marker = '"mode": "LAH_RELAY" if is_command_relay else "LAH_HOLD_RESUME",'
    assert marker in source
    tail = source.split(marker, 1)[1][:400]
    # The context is attached unconditionally, not only for the relay.
    assert '"enemy_contact": deepcopy(enemy_contact_context),' in tail
    assert "if is_command_relay\n                    else {}" not in tail
