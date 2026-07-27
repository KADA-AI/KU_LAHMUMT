"""A manned aircraft stays with its flight and never gets ahead of it.

Attack and post-attack points are solved one aircraft at a time, so without a
bound one of them wanders kilometres forward on its own.
"""

from __future__ import annotations

from typing import Any

import pytest

from modules.mission_planning.replanning.triggers.attack import pipeline as ap

GROUP = [
    {"latitude": 37.830, "longitude": 128.113},
    {"latitude": 37.831, "longitude": 128.115},
]
TARGET = {"latitude": 37.870, "longitude": 128.127}


def _centroid() -> dict[str, Any]:
    centroid = ap._average_coordinate(GROUP)
    assert centroid is not None
    return centroid


def test_a_point_far_ahead_is_pulled_back_to_the_flight_standoff() -> None:
    ahead = {"latitude": 37.866, "longitude": 128.126, "altitude": 700}

    kept = ap._keep_lah_with_group(
        ahead, group_coords=GROUP, target_coord=TARGET, aircraft_id=2
    )

    group_standoff_m = ap._haversine_distance_m(TARGET, _centroid())
    own_standoff_m = ap._haversine_distance_m(TARGET, kept)
    assert own_standoff_m >= group_standoff_m - 1.0


def test_a_point_far_to_the_flank_is_pulled_inside_the_spread() -> None:
    flank = {"latitude": 37.830, "longitude": 128.180, "altitude": 700}

    kept = ap._keep_lah_with_group(
        flank, group_coords=GROUP, target_coord=TARGET, aircraft_id=2
    )

    spread_m = ap.get_runtime_attack_float("lah_group_max_spread_m", 1500.0)
    distance_m = ap._haversine_distance_m(_centroid(), kept)
    assert distance_m <= spread_m + 50.0


def test_a_point_already_with_the_flight_is_left_alone() -> None:
    inside = {"latitude": 37.8305, "longitude": 128.1142, "altitude": 700}

    kept = ap._keep_lah_with_group(
        inside, group_coords=GROUP, target_coord=TARGET, aircraft_id=2
    )

    assert ap._haversine_distance_m(inside, kept) < 25.0


def test_altitude_is_never_touched() -> None:
    """Altitude belongs to the LOS and terrain passes, not to formation keeping."""

    ahead = {"latitude": 37.866, "longitude": 128.126, "altitude": 731}
    kept = ap._keep_lah_with_group(
        ahead, group_coords=GROUP, target_coord=TARGET, aircraft_id=2
    )
    assert kept["altitude"] == 731


def test_no_group_or_disabled_knob_changes_nothing(monkeypatch) -> None:
    lone = {"latitude": 37.866, "longitude": 128.126, "altitude": 700}

    assert ap._keep_lah_with_group(lone, group_coords=[], target_coord=TARGET) == lone

    monkeypatch.setattr(
        ap,
        "get_runtime_attack_int",
        lambda key, default=0, *a, **k: (
            0 if key == "lah_group_cohesion_enabled" else default
        ),
    )
    assert ap._keep_lah_with_group(lone, group_coords=GROUP, target_coord=TARGET) == lone


def test_group_positions_come_from_the_other_manned_aircraft_only() -> None:
    ctx = {
        "_selected_manned_aircraft": [
            {"aircraft_id": 1, "coordinate": GROUP[0]},
            {"aircraft_id": 2, "coordinate": {"latitude": 37.9, "longitude": 128.2}},
            {"aircraft_id": 3, "coordinate": GROUP[1]},
            {"aircraft_id": 5, "coordinate": {"latitude": 38.0, "longitude": 128.3}},
        ]
    }

    coords = ap._manned_group_coordinates_from_ctx(ctx, exclude_aircraft_id=2)

    assert len(coords) == 2
    assert all(row["latitude"] < 37.9 for row in coords)


def test_the_forward_direction_is_the_discovered_targets() -> None:
    ctx = {
        "_attack_target_list": [
            {"coordinate": {"latitude": 37.86, "longitude": 128.12}},
            {"coordinate": {"latitude": 37.88, "longitude": 128.14}},
        ]
    }

    centre = ap._attack_group_target_coordinate(ctx)

    assert centre is not None
    assert centre["latitude"] == pytest.approx(37.87)
    assert centre["longitude"] == pytest.approx(128.13)
