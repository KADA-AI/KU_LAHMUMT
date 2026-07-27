"""A planned firing point must be one the shot can actually be taken from.

Observed: LAH2 reached its attack waypoint at 532 m, held, and never fired. The
attack solver read the enemy's ground cell at 674.6 m and called the sightline
clear by ~6 m; the evaluator that gates the shot read the same cell at 685.9 m
and called it blocked. An 11 m disagreement over one cell, against a 6 m margin,
is the whole bug - the aircraft sat at a firing point that could never fire.

The chosen point is now certified against the evaluator that gates the shot, and
climbs until that evaluator agrees.
"""

from __future__ import annotations

from typing import Any

import pytest

from modules.mission_planning.replanning.triggers.attack import pipeline as ap


TARGET = {"latitude": 38.02516, "longitude": 127.31109, "altitude": 675.0}


def _certify(altitude_m: float, *, ceiling_m: float = 3500.0):
    certified = ap._lowest_firing_altitude_m(
        latitude=37.98763,
        longitude=127.30227,
        floor_m=altitude_m,
        ceiling_m=ceiling_m,
        target=TARGET,
    )
    return None if certified is None else int(round(certified))


def _fake_evaluator(monkeypatch, visible_at_or_above_m: float):
    calls: list[float] = []

    def fake(**kwargs: Any) -> dict[str, Any]:
        altitude = float(kwargs["target_altitude_m"])
        calls.append(altitude)
        return {"demAvailable": True, "visible": altitude >= visible_at_or_above_m}

    import modules.monitoring.logic.dem_cover.los_api as los_api

    monkeypatch.setattr(los_api, "evaluate_regional_los", fake)
    return calls


def test_an_already_clear_point_is_left_alone(monkeypatch) -> None:
    _fake_evaluator(monkeypatch, 500.0)

    assert _certify(532.0) == 532


def test_a_masked_point_climbs_until_the_shot_clears(monkeypatch) -> None:
    """The reported case: 532 m is masked, the plan must not stop there."""

    calls = _fake_evaluator(monkeypatch, 930.0)

    certified = _certify(532.0)

    assert certified is not None and certified >= 930
    assert certified > 532
    # It climbs in steps rather than jumping to the ceiling.
    assert len(calls) > 1 and calls[0] == 532.0  # floor probed first


def test_the_initial_probe_ceiling_expands_until_los_opens(monkeypatch) -> None:
    """The compatibility ceiling is a probe, never an attack deletion gate."""

    _fake_evaluator(monkeypatch, 5000.0)

    certified = _certify(532.0, ceiling_m=1000.0)
    assert certified is not None and certified >= 5000


def test_an_unusable_evaluator_keeps_the_solver_answer(monkeypatch) -> None:
    """A missing DEM must not block planning outright."""

    import modules.monitoring.logic.dem_cover.los_api as los_api

    monkeypatch.setattr(
        los_api,
        "evaluate_regional_los",
        lambda **_kwargs: {"demAvailable": False, "visible": None},
    )

    assert _certify(532.0) == 532


def test_the_enemy_is_the_observer(monkeypatch) -> None:
    """Firing is possible exactly when the enemy's sightline is open."""

    seen: dict[str, Any] = {}

    def fake(**kwargs: Any) -> dict[str, Any]:
        seen.update(kwargs)
        return {"demAvailable": True, "visible": True}

    import modules.monitoring.logic.dem_cover.los_api as los_api

    monkeypatch.setattr(los_api, "evaluate_regional_los", fake)
    _certify(532.0)

    assert seen["observer_latitude"] == pytest.approx(TARGET["latitude"])
    assert seen["observer_longitude"] == pytest.approx(TARGET["longitude"])
    assert seen["observer_height_m"] == pytest.approx(ap._ENEMY_OBSERVER_HEIGHT_M)
    assert seen["target_latitude"] == pytest.approx(37.98763)


def test_the_real_reported_geometry_needs_a_climb() -> None:
    """End to end against the actual DEM, no stubs."""

    certified = _certify(532.0)

    assert certified is None or certified > 532
