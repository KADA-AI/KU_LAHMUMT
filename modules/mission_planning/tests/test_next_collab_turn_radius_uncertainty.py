from __future__ import annotations

from unittest.mock import patch

import pytest

from modules.mission_planning.MissionPlanner.runtime_settings import runtime_override
from modules.mission_planning.runtime.next_collab_division_runner import (
    _HeadlessDivisionPlanner,
)
from modules.mission_planning.runtime.next_collab_line_runner import (
    _HeadlessLinePlanner,
)


MARGIN_RUNTIME = {
    "values": {
        "next_collab_turn_radius_uncertainty_margin_m": 120.0,
    }
}


def test_area_turn_radius_uses_reference_floor_and_log_margin() -> None:
    planner = _HeadlessDivisionPlanner()
    planner._turn_radius_scale = 1.2
    planner._aircraft_speed_overrides = {4: 40.0}
    planner._aircraft_turn_radius_overrides = {4: 335.0}
    planner._turn_radius_for_speed_m = lambda speed_mps, *, aircraft_id=None: 540.0

    with runtime_override(MARGIN_RUNTIME):
        assert planner._default_turn_radius_m(4) == pytest.approx(660.0)


def test_area_turn_radius_keeps_wider_observed_turn() -> None:
    planner = _HeadlessDivisionPlanner()
    planner._turn_radius_scale = 1.2
    planner._aircraft_speed_overrides = {4: 40.0}
    planner._aircraft_turn_radius_overrides = {4: 600.0}
    planner._turn_radius_for_speed_m = lambda speed_mps, *, aircraft_id=None: 540.0

    with runtime_override(MARGIN_RUNTIME):
        assert planner._default_turn_radius_m(4) == pytest.approx(840.0)


def test_line_turn_radius_uses_same_reference_floor_and_margin() -> None:
    planner = _HeadlessLinePlanner()
    planner._turn_radius_scale = 1.2
    planner._aircraft_speed_overrides = {4: 40.0}
    planner._aircraft_turn_radius_overrides = {4: 335.0}

    with (
        runtime_override(MARGIN_RUNTIME),
        patch(
            "modules.mission_planning.next_area_mode.planner_window._aircraft_turn_radius_m",
            return_value=450.0,
        ),
    ):
        assert planner._default_turn_radius_m(4) == pytest.approx(660.0)


def test_margin_can_be_disabled_without_changing_reference_floor() -> None:
    planner = _HeadlessLinePlanner()
    planner._turn_radius_scale = 1.2
    planner._aircraft_speed_overrides = {4: 40.0}
    planner._aircraft_turn_radius_overrides = {4: 335.0}

    with (
        runtime_override(
            {
                "values": {
                    "next_collab_turn_radius_uncertainty_margin_m": 0.0,
                }
            }
        ),
        patch(
            "modules.mission_planning.next_area_mode.planner_window._aircraft_turn_radius_m",
            return_value=450.0,
        ),
    ):
        assert planner._default_turn_radius_m(4) == pytest.approx(540.0)
