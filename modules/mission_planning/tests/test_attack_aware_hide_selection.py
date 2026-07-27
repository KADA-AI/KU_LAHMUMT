"""A hide point the aircraft cannot shoot from is poor cover.

Concealment quality and firing ability pull against each other: the deeper a
candidate sits behind the ridge beside the enemy, the higher the aircraft must
climb to see over it. Because LOS is symmetric, the altitude at which that enemy
starts to see a candidate is also the aircraft's firing altitude - a number the
refinement stage already computes for the concealment ceiling.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest

from modules.monitoring.logic.dem_cover.hide_com_refine import (
    _attack_infeasibility_rank,
)


class _Enemy:
    def __init__(self, lat: float, lon: float) -> None:
        self.lat = lat
        self.lon = lon


ENEMIES = [_Enemy(37.9283, 128.2010), _Enemy(37.9203, 128.2050)]
# rows = enemies, cols = candidates; value = altitude that enemy starts to see
# the candidate at, which is also the altitude needed to shoot that enemy.
REQUIREMENTS = np.array([[900.0, 3000.0, 1500.0], [400.0, 500.0, np.inf]])


def _rank(target: dict[str, Any] | None, ceiling: float | None):
    return _attack_infeasibility_rank(
        ENEMIES, REQUIREMENTS, attack_target_coordinate=target, attack_ceiling_m=ceiling
    )


def test_candidates_that_can_take_the_shot_rank_first() -> None:
    rank = _rank({"latitude": 37.9283, "longitude": 128.2010}, 2000.0)

    # 900 and 1500 fit under 2000; 3000 does not.
    assert list(rank) == [0, 1, 0]


def test_an_unreachable_sightline_is_ranked_last() -> None:
    """An infinite requirement means the enemy never sees that cell."""

    rank = _rank({"latitude": 37.9203, "longitude": 128.2050}, 2000.0)

    assert list(rank) == [0, 0, 1]


def test_a_lower_ceiling_shrinks_the_feasible_set() -> None:
    assert list(_rank({"latitude": 37.9283, "longitude": 128.2010}, 1000.0)) == [0, 1, 1]
    assert list(_rank({"latitude": 37.9283, "longitude": 128.2010}, 500.0)) == [1, 1, 1]


def test_without_a_target_the_ranking_is_left_alone() -> None:
    assert _rank(None, 2000.0) is None


def test_without_a_ceiling_the_ranking_is_left_alone() -> None:
    assert _rank({"latitude": 37.9283, "longitude": 128.2010}, None) is None


def test_an_unrecognised_target_never_guesses() -> None:
    """Matching the wrong enemy would rank candidates by the wrong sightline."""

    assert _rank({"latitude": 10.0, "longitude": 10.0}, 2000.0) is None


def test_a_malformed_ceiling_is_ignored() -> None:
    target = {"latitude": 37.9283, "longitude": 128.2010}

    assert _rank(target, 0.0) is None
    assert _rank(target, -50.0) is None
    assert _rank(target, float("nan")) is None


def test_a_malformed_target_is_ignored() -> None:
    assert _rank({"latitude": 37.9283}, 2000.0) is None
    assert _rank({"latitude": "x", "longitude": "y"}, 2000.0) is None


def test_the_planner_entry_points_accept_the_new_arguments() -> None:
    import inspect

    from modules.monitoring.logic.dem_cover.hide_com_refine import (
        refine_communication_hide,
    )
    from modules.mission_planning.pipelines.lah_enemy_contact import (
        _plan_enemy_contact_response_unbounded,
    )

    for fn in (refine_communication_hide, _plan_enemy_contact_response_unbounded):
        params = inspect.signature(fn).parameters
        assert "attack_target_coordinate" in params
        assert "attack_ceiling_m" in params
        # Both must default to the legacy behaviour.
        assert params["attack_target_coordinate"].default is None
        assert params["attack_ceiling_m"].default is None


def test_only_a_firing_aircraft_supplies_a_target() -> None:
    """The attacker supplies a target, but no attack-altitude ceiling."""

    import inspect

    from modules.mission_planning.replanning.triggers.attack import pipeline as ap

    source = inspect.getsource(ap._plan_lah_enemy_contact_response)
    assert 'if str(role) == "attacker":' in source
    assert "attack_target_coordinate=attack_target_coordinate" in source
    assert "attack_ceiling_m=attack_ceiling_m" in source
    assert "attack_ceiling_m: Optional[float] = None" in source
    assert "DEFAULT_ENVELOPE" not in source
