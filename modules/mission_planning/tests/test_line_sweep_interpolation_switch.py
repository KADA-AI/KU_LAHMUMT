from __future__ import annotations

import unittest
from types import SimpleNamespace

from modules.mission_planning.MissionPlanner.runtime_settings import runtime_override
from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import (
    d0303,
)
from modules.mission_planning.pipelines import next_collab_path_builder
from modules.mission_planning.planners.donut_patrol import logic as donut_logic


def _coord(value: float) -> dict:
    return {
        "latitude": 38.0,
        "longitude": 127.0 + value,
        "altitude": 100,
    }


def _line_search_waypoint(*, points_per_strip: int = 3, strip_count: int = 2) -> dict:
    coords = [
        _coord(float(strip * 10 + point))
        for strip in range(strip_count)
        for point in range(points_per_strip)
    ]
    return {
        "filmingProperty": {
            "operationMode": d0303.OPMODE_LINE,
            "lineSearch": {
                "coordinateList": coords,
                "interpolationPoints": int(points_per_strip),
            },
        },
    }


class LineSweepInterpolationSwitchTests(unittest.TestCase):
    def test_off_collapses_existing_strips_before_dem_processing(self) -> None:
        waypoint = _line_search_waypoint(points_per_strip=3, strip_count=2)

        with runtime_override(
            {
                "values": {
                    "line_sweep_interpolation_enabled": False,
                    "sweep_line_interp_points": 5,
                }
            }
        ):
            removed = d0303._collapse_linesearch_midpoints_inplace([waypoint])

        line_search = waypoint["filmingProperty"]["lineSearch"]
        self.assertEqual(removed, 2)
        self.assertEqual(line_search["interpolationPoints"], 2)
        self.assertEqual(
            [coord["longitude"] for coord in line_search["coordinateList"]],
            [127.0, 129.0, 137.0, 139.0],
        )

    def test_off_overrides_next_collab_auto_sweep_points(self) -> None:
        with runtime_override(
            {
                "values": {
                    "line_sweep_interpolation_enabled": False,
                    "next_collab_auto_sweep_points": True,
                    "next_collab_sweep_points_per_leg": 7,
                }
            }
        ):
            self.assertFalse(next_collab_path_builder._next_collab_auto_sweep_points())
            self.assertEqual(next_collab_path_builder._next_collab_sweep_points_per_leg(), 2)
            self.assertEqual(
                len(next_collab_path_builder._line_three_point_xy([(0.0, 0.0), (1000.0, 0.0)])),
                2,
            )

    def test_off_collapses_next_collab_residual_midpoints(self) -> None:
        waypoint = _line_search_waypoint(points_per_strip=3, strip_count=2)

        with runtime_override(
            {
                "values": {
                    "line_sweep_interpolation_enabled": False,
                    "next_collab_auto_sweep_points": True,
                }
            }
        ):
            removed = next_collab_path_builder._collapse_linesearch_midpoints_inplace(
                [waypoint]
            )

        line_search = waypoint["filmingProperty"]["lineSearch"]
        self.assertEqual(removed, 2)
        self.assertEqual(line_search["interpolationPoints"], 2)
        self.assertEqual(len(line_search["coordinateList"]), 4)

    def test_on_restores_configured_next_collab_samples(self) -> None:
        with runtime_override(
            {
                "values": {
                    "line_sweep_interpolation_enabled": True,
                    "next_collab_auto_sweep_points": False,
                    "next_collab_sweep_points_per_leg": 5,
                }
            }
        ):
            self.assertEqual(next_collab_path_builder._next_collab_sweep_points_per_leg(), 5)
            self.assertEqual(
                len(next_collab_path_builder._line_three_point_xy([(0.0, 0.0), (1000.0, 0.0)])),
                5,
            )

    def test_donut_area_profile_obeys_global_switch(self) -> None:
        mission_stub = SimpleNamespace(outer_latlon=[])
        config = donut_logic.PatrolConfig(
            use_area_db=False,
            fov_deg=2.4,
            separation_m=100.0,
            search_speed_weight=1.0,
            turn_radius_m=500.0,
            turn_step_deg=15.0,
        )

        with runtime_override(
            {
                "values": {
                    "line_sweep_interpolation_enabled": False,
                    "sweep_line_interp_points": 5,
                }
            }
        ):
            off_profile = donut_logic.resolve_planning_profile(mission_stub, config)

        with runtime_override(
            {
                "values": {
                    "line_sweep_interpolation_enabled": True,
                    "sweep_line_interp_points": 5,
                }
            }
        ):
            on_profile = donut_logic.resolve_planning_profile(mission_stub, config)

        self.assertEqual(off_profile.interpolation_points, 2)
        self.assertEqual(on_profile.interpolation_points, 5)


if __name__ == "__main__":
    unittest.main()
