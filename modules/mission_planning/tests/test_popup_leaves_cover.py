"""The planned attack base must be distinct, low, and terrain-safe.

Observed: 30% of manned attack waypoints sat on the exact hide point - same
lat/lon, same altitude, same ETA. The aircraft was ordered to fly to where it
already was, never registered the arrival, and held at cover without shooting.

The SIM performs the vertical LOS climb, so the mission must not carry the
solved firing altitude.  It does still need a distinct horizontal attack point
so the armed waypoint cannot collapse onto the preceding hide waypoint.
"""

from __future__ import annotations

import numpy as np

from modules.monitoring.logic.dem_cover.config import CoverConfig
from modules.monitoring.logic.dem_cover.hide_com import CommunicationHideAnalyzer
from modules.mission_planning.replanning.triggers.attack import pipeline as attack


def test_the_concealment_margin_is_bigger_than_dem_noise() -> None:
    """0.25 m let a sightline grazing the ridge pass as cover."""

    assert CoverConfig().hide_safety_margin_m >= 1.0


def test_a_grazing_sightline_counts_as_exposed() -> None:
    """The reported case: cover by +0.6 m is not cover."""

    enemy_requirements_m = np.array([[500.6]], dtype=np.float64)
    uav_requirements_m = np.array([[0.0]], dtype=np.float64)
    altitudes = np.array([500.0], dtype=np.float64)
    margin = float(CoverConfig().hide_safety_margin_m)

    without_margin, _ = CommunicationHideAnalyzer._counts_at_altitude(
        altitudes, enemy_requirements_m, uav_requirements_m, 0.0
    )
    with_margin, _ = CommunicationHideAnalyzer._counts_at_altitude(
        altitudes, enemy_requirements_m, uav_requirements_m, margin
    )

    assert int(without_margin[0]) == 0  # the old reading: "hidden"
    assert int(with_margin[0]) == 1     # now correctly counted as seen


def test_a_position_well_below_the_sightline_is_still_cover() -> None:
    """The margin must not throw away genuine concealment."""

    enemy_requirements_m = np.array([[600.0]], dtype=np.float64)
    uav_requirements_m = np.array([[0.0]], dtype=np.float64)
    visible, _ = CommunicationHideAnalyzer._counts_at_altitude(
        np.array([500.0], dtype=np.float64),
        enemy_requirements_m,
        uav_requirements_m,
        float(CoverConfig().hide_safety_margin_m),
    )

    assert int(visible[0]) == 0


def test_an_attack_waypoint_must_move_a_little_from_cover() -> None:
    assert (
        attack.get_runtime_attack_float(
            "attack_point_min_horizontal_offset_m", 5.0
        )
        >= 1.0
    )


def test_the_firing_altitude_is_certification_only() -> None:

    import inspect

    source = inspect.getsource(attack._attack_coordinate_at_hide_endpoint)
    assert "attack_point_min_horizontal_offset_m" in source
    assert "_attack_point_low_level_base" in source
    # Internal LOS certification may search above the normal envelope, but the
    # resulting altitude is never serialized into the attack waypoint.
    assert "ceiling_m=None" in source
    low_base_source = inspect.getsource(attack._attack_point_low_level_base)
    assert 'attack_altitude_control"] = "sim_los_popup"' in low_base_source


def test_the_plan_ends_at_the_low_popup_base_without_a_return_leg() -> None:
    import inspect

    source = inspect.getsource(attack._build_lah_low_level_attack_waypoints)
    assert "_build_lah_low_level_waypoint_route" in source
    assert "del regain_cover_coord" in source
