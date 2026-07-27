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
