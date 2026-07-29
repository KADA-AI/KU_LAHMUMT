"""Attack pop-up selection stays as close as possible to the hide point.

There is no attack-altitude ceiling.  A farther candidate may still be used
when the hide column cannot produce a firing solution, but it is never chosen
merely to save climb altitude.
"""

from __future__ import annotations

import math
from typing import Any

import pytest

from modules.mission_planning.replanning.triggers.attack import pipeline as ap

HIDE: dict[str, Any] = {"latitude": 37.8664, "longitude": 128.2099, "altitude": 620}
TARGET: dict[str, Any] = {"latitude": 37.9283, "longitude": 128.2010, "altitude": 740}
OTHER_ENEMY: dict[str, Any] = {
    "latitude": 37.8800,
    "longitude": 128.2140,
    "altitude": 720,
}


def _distance_m(left: dict[str, Any], right: dict[str, Any]) -> float:
    return math.hypot(
        (right["latitude"] - left["latitude"]) * 111_132.0,
        (right["longitude"] - left["longitude"])
        * 111_320.0
        * math.cos(math.radians(left["latitude"])),
    )


def _knobs(monkeypatch, **overrides: float) -> None:
    if (
        "attack_popup_vertical_only" not in overrides
        and any(key.startswith("attack_popup_search_") for key in overrides)
    ):
        # Lateral search is now opt-in; tests exercising that optional mode
        # must turn it on explicitly.
        overrides["attack_popup_vertical_only"] = 0.0
    real_float = ap.get_runtime_attack_float
    real_int = ap.get_runtime_attack_int

    def fake_float(key, default=0.0, *args, **kwargs):
        if key in overrides:
            return float(overrides[key])
        return real_float(key, default, *args, **kwargs)

    def fake_int(key, default=0, *args, **kwargs):
        if key in overrides:
            return int(overrides[key])
        return real_int(key, default, *args, **kwargs)

    monkeypatch.setattr(ap, "get_runtime_attack_float", fake_float)
    monkeypatch.setattr(ap, "get_runtime_attack_int", fake_int)


def _solver(monkeypatch, altitude_for):
    """Stand in for the DEM solve, returning a chosen altitude per candidate."""

    def fake(attack_coord, enemy_coord, *, lah_floor_coord=None, altitude_offset_m=None):
        altitude = altitude_for(_distance_m(HIDE, attack_coord))
        if altitude is None:
            return None, "no profile"
        return (
            {
                "latitude": float(attack_coord["latitude"]),
                "longitude": float(attack_coord["longitude"]),
                "altitude": int(altitude),
            },
            None,
        )

    monkeypatch.setattr(ap, "_compute_attack_los_altitude_batch_dem", fake)
    # These cases exercise candidate selection over synthetic sightlines. The
    # real-DEM firing certification would answer about actual Inje terrain,
    # which says nothing about the choice being made here.
    monkeypatch.setattr(
        ap,
        "_lowest_firing_altitude_m",
        lambda *, latitude, longitude, floor_m, ceiling_m, target: float(floor_m),
    )


def test_the_candidates_are_rings_around_the_hide_point(monkeypatch) -> None:
    _knobs(
        monkeypatch,
        attack_popup_search_radius_m=600.0,
        attack_popup_search_step_m=150.0,
        attack_popup_search_bearings=8,
    )

    candidates = ap._attack_popup_candidates(HIDE)

    assert len(candidates) == 1 + 4 * 8
    # The recorded offset is the true ground distance, not an approximation.
    for coordinate, offset_m in candidates:
        assert _distance_m(HIDE, coordinate) == pytest.approx(offset_m, abs=0.5)
    assert candidates[0][1] == 0.0
    assert sorted({round(offset) for _c, offset in candidates}) == [0, 150, 300, 450, 600]


def test_a_zero_radius_keeps_the_old_straight_up_behaviour(monkeypatch) -> None:
    _knobs(monkeypatch, attack_popup_search_radius_m=0.0)

    candidates = ap._attack_popup_candidates(HIDE)

    assert len(candidates) == 1
    assert candidates[0][1] == 0.0
    assert candidates[0][0]["latitude"] == pytest.approx(HIDE["latitude"])
    assert candidates[0][0]["longitude"] == pytest.approx(HIDE["longitude"])


def test_the_nearest_sightline_wins_even_when_it_needs_more_climb(monkeypatch) -> None:
    _knobs(
        monkeypatch,
        attack_popup_search_radius_m=300.0,
        attack_popup_search_step_m=150.0,
        attack_popup_lateral_penalty_m_per_100m=0.0,
    )
    # Straight up needs 1200 m; stepping out 300 m only needs 900 m.
    _solver(monkeypatch, lambda d: 1200 if d < 150 else (1050 if d < 300 else 900))

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert int(result["altitude"]) <= int(HIDE["altitude"]) + 100
    assert result["attack_altitude_control"] == "sim_los_popup"
    assert result["attack_point_popup_los_certified"] is True
    assert result["attack_point_vertical_popup"] is True
    assert _distance_m(HIDE, result) == pytest.approx(5.0, abs=0.5)


def test_a_trivial_saving_does_not_drag_the_aircraft_off_its_cover(monkeypatch) -> None:
    """5 m of altitude is not worth 600 m of exposure outside cover."""

    _knobs(
        monkeypatch,
        attack_popup_search_radius_m=600.0,
        attack_popup_search_step_m=150.0,
        attack_popup_lateral_penalty_m_per_100m=5.0,
    )
    _solver(monkeypatch, lambda d: 1200 if d < 1.0 else 1195)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert int(result["altitude"]) < 1195
    assert result["attack_point_popup_offset_m"] == pytest.approx(5.0, abs=0.5)
    assert _distance_m(HIDE, result) == pytest.approx(5.0, abs=0.5)


def test_altitude_saving_does_not_move_the_attack_away_from_cover(monkeypatch) -> None:
    _knobs(
        monkeypatch,
        attack_popup_search_radius_m=600.0,
        attack_popup_search_step_m=150.0,
        attack_popup_lateral_penalty_m_per_100m=5.0,
    )
    # Even a 200 m saving does not justify leaving the certified hide column.
    _solver(monkeypatch, lambda d: 1200 if d < 1.0 else 1000)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert int(result["altitude"]) < 1000
    assert result["attack_point_vertical_popup"] is True
    assert result["attack_point_popup_offset_m"] == pytest.approx(5.0, abs=0.5)


def test_an_equal_sightline_keeps_the_aircraft_closest_to_cover(monkeypatch) -> None:
    _knobs(
        monkeypatch,
        attack_popup_search_radius_m=450.0,
        attack_popup_search_step_m=150.0,
        attack_popup_lateral_penalty_m_per_100m=0.0,
    )
    _solver(monkeypatch, lambda _d: 1100)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert _distance_m(HIDE, result) == pytest.approx(5.0, abs=0.5)


def test_the_altitude_floor_is_still_the_hide_altitude(monkeypatch) -> None:
    """The solver is handed the hide point as the floor for every candidate."""

    _knobs(monkeypatch, attack_popup_search_radius_m=150.0, attack_popup_search_step_m=150.0)
    floors: list[Any] = []

    def fake(attack_coord, enemy_coord, *, lah_floor_coord=None, altitude_offset_m=None):
        floors.append(lah_floor_coord)
        return ({**attack_coord, "altitude": 1000}, None)

    monkeypatch.setattr(ap, "_compute_attack_los_altitude_batch_dem", fake)
    ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert floors
    for floor in floors:
        assert floor["latitude"] == pytest.approx(HIDE["latitude"])
        assert floor["longitude"] == pytest.approx(HIDE["longitude"])


def test_nothing_solvable_falls_back_instead_of_inventing_a_point(monkeypatch) -> None:
    _knobs(monkeypatch, attack_popup_search_radius_m=300.0, attack_popup_search_step_m=150.0)
    _solver(monkeypatch, lambda _d: None)
    messages: list[str] = []

    result = ap._attack_coordinate_at_hide_endpoint(
        HIDE, TARGET, emit=messages.append, aircraft_id=2
    )

    assert result is None
    assert any("Could not solve a climb" in message for message in messages)


def test_one_solvable_candidate_among_failures_is_still_used(monkeypatch) -> None:
    _knobs(
        monkeypatch,
        attack_popup_search_radius_m=300.0,
        attack_popup_search_step_m=150.0,
        attack_popup_lateral_penalty_m_per_100m=0.0,
    )
    # The hide column itself has no profile; only the outer ring solves.
    _solver(monkeypatch, lambda d: None if d < 300.0 else 980)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert int(result["altitude"]) < 980
    assert result["attack_altitude_control"] == "sim_los_popup"
    assert _distance_m(HIDE, result) == pytest.approx(300.0, abs=0.5)


def test_the_feature_switch_still_hands_back_to_the_legacy_attack_point(monkeypatch) -> None:
    _knobs(monkeypatch, attack_point_at_hide_endpoint=0)

    assert ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET) is None


def test_operational_default_keeps_neighbourhood_fallback_available() -> None:
    """Nearby candidates remain available when the hide column cannot solve."""

    assert ap.get_runtime_attack_int("attack_popup_vertical_only", 0) == 0


def test_the_default_stays_at_the_hide_column_when_it_is_solvable(monkeypatch) -> None:
    _solver(monkeypatch, lambda d: 1200 if d < 1.0 else 800)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert int(result["altitude"]) < 800
    assert result["attack_point_vertical_popup"] is True
    assert _distance_m(HIDE, result) == pytest.approx(5.0, abs=0.5)


def test_vertical_only_is_still_available_as_an_override(monkeypatch) -> None:
    _knobs(monkeypatch, attack_popup_vertical_only=1)
    _solver(monkeypatch, lambda d: 1200 if d < 1.0 else 800)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert int(result["altitude"]) < 800
    assert result["attack_point_vertical_popup"] is True
    assert _distance_m(HIDE, result) == pytest.approx(5.0, abs=0.5)


def test_a_certified_popup_above_the_envelope_is_not_serialized(monkeypatch) -> None:
    """Even an extreme LOS solution leaves only a low base in the mission."""

    _knobs(monkeypatch, attack_popup_vertical_only=1)
    # Derive from the live envelope so raising the ceiling cannot make this
    # assertion vacuous.
    over_ceiling = float(ap.DEFAULT_ENVELOPE.max_altitude_m) + 200.0
    _solver(monkeypatch, lambda _d: over_ceiling)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert result["altitude"] < over_ceiling
    assert result["attack_altitude_control"] == "sim_los_popup"


def test_an_in_envelope_popup_is_also_replaced_by_the_low_base(monkeypatch) -> None:

    _knobs(monkeypatch, attack_popup_vertical_only=1)
    under_ceiling = float(ap.DEFAULT_ENVELOPE.max_altitude_m) - 600.0
    _solver(monkeypatch, lambda _d: under_ceiling)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert int(result["altitude"]) < int(under_ceiling)
    assert result["attack_altitude_control"] == "sim_los_popup"


def test_nearest_attack_point_wins_even_when_a_farther_point_needs_less_climb(
    monkeypatch,
) -> None:
    _knobs(
        monkeypatch,
        attack_popup_search_radius_m=300.0,
        attack_popup_search_step_m=150.0,
    )
    _solver(monkeypatch, lambda distance: 5000 if distance < 1.0 else 900)

    result = ap._attack_coordinate_at_hide_endpoint(HIDE, TARGET)

    assert result is not None
    assert result["altitude"] < 900
    assert result["attack_point_vertical_popup"] is True
    assert _distance_m(HIDE, result) == pytest.approx(5.0, abs=0.5)


def test_popup_is_approved_when_every_non_target_enemy_stays_masked(monkeypatch) -> None:
    _knobs(monkeypatch, attack_popup_vertical_only=1)
    _solver(monkeypatch, lambda _d: 900)
    calls = []

    def fake_los(**kwargs):
        calls.append(kwargs)
        return {"visible": False, "reason": "TERRAIN_BLOCKED"}

    monkeypatch.setattr(ap, "evaluate_regional_los", fake_los)
    result = ap._attack_coordinate_at_hide_endpoint(
        HIDE,
        TARGET,
        threat_targets=[
            {"target_id": 7, "coordinate": dict(TARGET)},
            {"target_id": 8, "coordinate": dict(OTHER_ENEMY)},
        ],
        attack_target_id=7,
    )

    assert result is not None
    assert len(calls) == 1
    assert calls[0]["observer_latitude"] == pytest.approx(OTHER_ENEMY["latitude"])
    assert result["attack_other_enemy_los_checked"] is True
    assert result["attack_other_enemy_considered_count"] == 1
    assert result["attack_other_enemy_visible_count"] == 0
    assert result["attack_other_enemy_unknown_count"] == 0


def test_popup_uses_explicit_degraded_fallback_when_another_enemy_can_see_it(monkeypatch) -> None:
    _knobs(monkeypatch, attack_popup_vertical_only=1)
    _solver(monkeypatch, lambda _d: 900)
    monkeypatch.setattr(
        ap,
        "evaluate_regional_los",
        lambda **_kwargs: {"visible": True, "reason": "VISIBLE"},
    )

    result = ap._attack_coordinate_at_hide_endpoint(
        HIDE,
        TARGET,
        threat_targets=[
            {"target_id": 7, "coordinate": dict(TARGET)},
            {"target_id": 8, "coordinate": dict(OTHER_ENEMY)},
        ],
        attack_target_id=7,
    )

    assert result is not None
    assert result["attack_other_enemy_exposure_fallback"] is True
    assert result["attack_other_enemy_visible_count"] == 1


def test_popup_excludes_the_designated_target_from_exposure_gate(monkeypatch) -> None:
    _knobs(monkeypatch, attack_popup_vertical_only=1)
    _solver(monkeypatch, lambda _d: 900)

    def unexpected_los(**_kwargs):
        raise AssertionError("the designated target must not be treated as another enemy")

    monkeypatch.setattr(ap, "evaluate_regional_los", unexpected_los)
    result = ap._attack_coordinate_at_hide_endpoint(
        HIDE,
        TARGET,
        threat_targets=[{"target_id": 7, "coordinate": dict(TARGET)}],
        attack_target_id=7,
    )

    assert result is not None
    assert result["attack_other_enemy_considered_count"] == 0


def test_popup_uses_explicit_degraded_fallback_when_other_enemy_los_is_unknown(monkeypatch) -> None:
    _knobs(monkeypatch, attack_popup_vertical_only=1)
    _solver(monkeypatch, lambda _d: 900)
    monkeypatch.setattr(
        ap,
        "evaluate_regional_los",
        lambda **_kwargs: {"visible": None, "reason": "DEM_FILE_MISSING"},
    )

    result = ap._attack_coordinate_at_hide_endpoint(
        HIDE,
        TARGET,
        threat_targets=[
            {"target_id": 7, "coordinate": dict(TARGET)},
            {"target_id": 8, "coordinate": dict(OTHER_ENEMY)},
        ],
        attack_target_id=7,
    )

    assert result is not None
    assert result["attack_other_enemy_exposure_fallback"] is True
    assert result["attack_other_enemy_unknown_count"] == 1


def test_offsetting_by_metres_is_accurate_in_both_axes() -> None:
    east = ap._offset_coordinate_m(HIDE, 250.0, 0.0)
    north = ap._offset_coordinate_m(HIDE, 0.0, 250.0)

    assert _distance_m(HIDE, east) == pytest.approx(250.0, abs=0.5)
    assert _distance_m(HIDE, north) == pytest.approx(250.0, abs=0.5)
    assert east["longitude"] > HIDE["longitude"]
    assert east["latitude"] == pytest.approx(HIDE["latitude"])
    assert north["latitude"] > HIDE["latitude"]
    assert north["longitude"] == pytest.approx(HIDE["longitude"])
