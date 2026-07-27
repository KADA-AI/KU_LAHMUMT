"""Shared fixtures for mission_planning tests."""

from __future__ import annotations

import pytest

from modules.mission_planning.MissionPlanner.data_def import lah_terrain_path as _ltp


@pytest.fixture(autouse=True)
def _pin_low_terrain_strength(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pin the operator dial so tests do not float with uav_params.json.

    ``lah_low_terrain_strength`` is a live runtime setting; every test runs at
    the tuned baseline (1.0) unless it monkeypatches the dial itself.
    """

    monkeypatch.setattr(_ltp, "_low_terrain_strength", lambda: 1.0)


@pytest.fixture(autouse=True)
def _pin_ladder_cover_off(monkeypatch: pytest.MonkeyPatch) -> None:
    """Keep type2/3 ladder holds on their geometric anchors during tests.

    The DEM cover slide depends on live runtime settings and on whichever
    regional rasters exist on the machine; tests that exercise it opt in by
    monkeypatching ``_cover_hold_enabled`` (and stubbing the selector).
    """

    from modules.mission_planning.pipelines import ground_maneuver_mode as _gmm

    monkeypatch.setattr(_gmm, "_cover_hold_enabled", lambda: False)
