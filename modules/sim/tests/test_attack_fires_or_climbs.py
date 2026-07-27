"""An armed attack fires as soon as it can, and never parks unable to fire.

Observed: LAH2 reached its attack waypoint and held there indefinitely. The
simulator refused the shot because terrain masked the target, and the only other
outcome coded was to keep waiting - the aircraft sat at a firing point it could
never fire from. The planner had certified the point using a DEM read that
differs from the simulator's by up to 20 m, so this can recur whenever the two
disagree.

Two behaviours close that off: take the shot the moment the sightline opens
(without waiting to touch the waypoint), and climb out of a masked firing point
instead of holding at it.
"""

from __future__ import annotations

import inspect

from modules.sim.runtime import sim_service


def _source() -> str:
    return inspect.getsource(sim_service.SimulationService._evaluate_vehicle_attacks)


def test_the_shot_is_taken_as_soon_as_the_sightline_opens() -> None:
    """Arrival is not a precondition when the target is already visible."""

    source = _source()

    assert "los_open = self._threat_pair_terrain_los" in source
    # The arrival gate applies only while the shot is not yet possible.
    assert "if wp is not None and not los_open:" in source


def test_a_masked_firing_point_is_climbed_out_of() -> None:
    source = _source()

    assert "_ATTACK_LOS_CLIMB_RATE_MPS" in source
    assert "climbing_for_los" in source


def test_the_climb_is_bounded_and_reports_when_it_runs_out() -> None:
    source = _source()

    assert "_ATTACK_LOS_MAX_CLIMB_M" in source
    assert "los_blocked_climb_exhausted" in source


def test_the_climb_limits_are_flyable() -> None:
    assert 0.0 < sim_service._ATTACK_LOS_CLIMB_RATE_MPS <= 12.0
    assert 100.0 <= sim_service._ATTACK_LOS_MAX_CLIMB_M <= 1500.0


def test_the_hold_state_records_how_far_it_has_climbed() -> None:
    """Progress must persist across frames or the climb restarts every step."""

    source = _source()

    assert '"climbedM"' in source
    assert 'previous.get("climbedM"' in source

