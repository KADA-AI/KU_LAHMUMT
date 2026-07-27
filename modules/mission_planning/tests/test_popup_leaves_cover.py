"""Cover and a firing solution are mutually exclusive, and a pop-up must climb.

Observed: 30% of manned attack waypoints sat on the exact hide point - same
lat/lon, same altitude, same ETA. The aircraft was ordered to fly to where it
already was, never registered the arrival, and held at cover without shooting.

The cause was upstream: the sightline from the hide point cleared the ridge by
+0.6 m, which the concealment test (0.25 m numerical nudge) called blocked and
the attack solver called open. Concealment now needs a real margin, and a
pop-up that would not leave cover is lifted to a flyable minimum.
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


def test_a_popup_must_actually_leave_cover() -> None:
    assert attack._LAH_MIN_POPUP_CLIMB_M >= 5.0


def test_the_firing_point_is_lifted_off_the_hide_altitude() -> None:
    """A vertical pop-up solved level with cover gets the minimum climb."""

    import inspect

    source = inspect.getsource(attack._attack_coordinate_at_hide_endpoint)
    assert "_LAH_MIN_POPUP_CLIMB_M" in source
    # Attack LOS altitude is intentionally exempt from the ordinary envelope.
    assert "ceiling_m=None" in source
    assert "DEFAULT_ENVELOPE" not in source


def test_a_zero_length_firing_leg_is_never_emitted() -> None:
    import inspect

    source = inspect.getsource(attack._build_lah_low_level_attack_waypoints)
    assert "_same_lah_3d_coordinate" in source
