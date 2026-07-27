from __future__ import annotations

import math
import random
import unittest

from modules.sim.mission.auto_target_placement import (
    AutoTargetPlacementConfig,
    generate_auto_target_placements,
)
from modules.sim.runtime.geo import GeoConverter
from modules.sim.runtime.sim_service import SimulationService


def _line_mission(
    mission_id: int,
    region_type: int,
    *,
    start: tuple[float, float] = (38.0, 127.0),
    end: tuple[float, float] = (38.0, 127.03),
    width: float = 600.0,
    done: bool = False,
) -> dict:
    return {
        "inputMissionID": mission_id,
        "inputMissionType": 1,
        "regionType": region_type,
        "isDone": done,
        "missionDetail": {
            "lineList": [
                {
                    "width": width,
                    "coordinateList": [
                        {"latitude": start[0], "longitude": start[1], "altitude": 0},
                        {"latitude": end[0], "longitude": end[1], "altitude": 0},
                    ],
                }
            ],
            "areaList": None,
        },
    }


def _area_mission(
    mission_id: int,
    region_type: int,
    *,
    done: bool = False,
    with_hole: bool = False,
) -> dict:
    area_list = [
        {
            "isHole": False,
            "coordinateList": [
                {"latitude": 38.00, "longitude": 127.00, "altitude": 0},
                {"latitude": 38.00, "longitude": 127.04, "altitude": 0},
                {"latitude": 38.04, "longitude": 127.04, "altitude": 0},
                {"latitude": 38.04, "longitude": 127.00, "altitude": 0},
            ],
        }
    ]
    if with_hole:
        area_list.append(
            {
                "isHole": True,
                "coordinateList": [
                    {"latitude": 38.015, "longitude": 127.015, "altitude": 0},
                    {"latitude": 38.015, "longitude": 127.025, "altitude": 0},
                    {"latitude": 38.025, "longitude": 127.025, "altitude": 0},
                    {"latitude": 38.025, "longitude": 127.015, "altitude": 0},
                ],
            }
        )
    return {
        "inputMissionID": mission_id,
        "inputMissionType": 2,
        "regionType": region_type,
        "isDone": done,
        "missionDetail": {"lineList": None, "areaList": area_list},
    }


def _plan(*missions: dict) -> list[dict]:
    return [{"inputMissionPackageID": 100, "inputMissionList": list(missions)}]


class AutoTargetPlacementTests(unittest.TestCase):
    def test_target_line_always_gets_two_to_five_targets_inside_width(self) -> None:
        result = generate_auto_target_placements(
            _plan(_line_mission(10, 6)),
            rng=random.Random(17),
            config=AutoTargetPlacementConfig(first_mission_exclusion_radius_m=0.0),
        )

        self.assertGreaterEqual(result["targetRegionCount"], 2)
        self.assertLessEqual(result["targetRegionCount"], 5)
        self.assertEqual(result["generalCount"], 0)
        for target in result["placements"]:
            # Horizontal line at lat=38: perpendicular distance is north/south.
            distance_m = abs(float(target["lat"]) - 38.0) * 111_320.0
            self.assertLessEqual(distance_m, 300.0)
            self.assertGreaterEqual(float(target["lon"]), 127.0)
            self.assertLessEqual(float(target["lon"]), 127.03)

    def test_target_area_points_stay_out_of_hole(self) -> None:
        config = AutoTargetPlacementConfig(
            target_region_min_count=4,
            target_region_max_count=4,
            min_separation_m=120.0,
        )
        result = generate_auto_target_placements(
            _plan(_area_mission(20, 6, with_hole=True)),
            rng=random.Random(9),
            config=config,
        )

        self.assertEqual(result["count"], 4)
        for target in result["placements"]:
            lat = float(target["lat"])
            lon = float(target["lon"])
            self.assertTrue(38.00 < lat < 38.04)
            self.assertTrue(127.00 < lon < 127.04)
            self.assertFalse(38.015 < lat < 38.025 and 127.015 < lon < 127.025)

    def test_general_regions_are_sparse_but_target_region_is_guaranteed(self) -> None:
        config = AutoTargetPlacementConfig(
            general_zone_probability=0.0,
            target_region_min_count=3,
            target_region_max_count=3,
            min_separation_m=100.0,
        )
        result = generate_auto_target_placements(
            _plan(
                _line_mission(30, 3),
                _area_mission(31, 4),
                _area_mission(32, 6),
            ),
            rng=random.Random(4),
            config=config,
        )

        self.assertEqual(result["generalCount"], 0)
        self.assertEqual(result["targetRegionCount"], 3)
        self.assertTrue(all(item["inputMissionID"] == 32 for item in result["placements"]))

    def test_general_total_is_split_area_seven_to_line_three(self) -> None:
        config = AutoTargetPlacementConfig(
            general_zone_probability=1.0,
            general_zone_min_count=5,
            general_zone_max_count=5,
            min_separation_m=80.0,
            first_mission_exclusion_radius_m=0.0,
        )
        result = generate_auto_target_placements(
            _plan(_line_mission(40, 3), _area_mission(41, 4)),
            rng=random.Random(2),
            config=config,
        )

        self.assertEqual(result["zoneCount"], 2)
        self.assertEqual(result["selectedGeneralZoneCount"], 2)
        self.assertEqual(result["generalCount"], 10)
        self.assertEqual(result["areaCount"], 7)
        self.assertEqual(result["lineCount"], 3)
        self.assertEqual(
            [item["geometryType"] for item in result["placements"]].count("area"),
            7,
        )
        self.assertEqual(
            [item["geometryType"] for item in result["placements"]].count("line"),
            3,
        )

    def test_missing_geometry_receives_the_full_existing_target_count(self) -> None:
        config = AutoTargetPlacementConfig(
            target_region_min_count=10,
            target_region_max_count=10,
            min_separation_m=80.0,
            first_mission_exclusion_radius_m=0.0,
        )
        result = generate_auto_target_placements(
            _plan(_area_mission(42, 6)),
            rng=random.Random(8),
            config=config,
        )

        self.assertEqual(result["count"], 10)
        self.assertEqual(result["areaCount"], 10)
        self.assertEqual(result["lineCount"], 0)

    def test_default_excludes_targets_within_three_km_of_first_mission_start(self) -> None:
        start = (38.0, 127.0)
        config = AutoTargetPlacementConfig(
            target_region_min_count=5,
            target_region_max_count=5,
            min_separation_m=80.0,
        )
        result = generate_auto_target_placements(
            _plan(
                _line_mission(
                    41,
                    6,
                    start=start,
                    end=(38.0, 127.10),
                )
            ),
            rng=random.Random(11),
            config=config,
        )

        self.assertEqual(result["count"], 5)
        self.assertEqual(result["firstMissionExclusionRadiusM"], 3_000.0)
        self.assertGreater(result["firstMissionExclusionRejectedCandidateCount"], 0)
        geo = GeoConverter(start[1], start[0])
        start_x, start_y = geo.lonlat_to_xy(start[1], start[0])
        for target in result["placements"]:
            target_x, target_y = geo.lonlat_to_xy(target["lon"], target["lat"])
            self.assertGreaterEqual(
                math.hypot(target_x - start_x, target_y - start_y),
                3_000.0,
            )

    def test_exclusion_wins_when_the_entire_target_zone_is_inside_radius(self) -> None:
        result = generate_auto_target_placements(
            _plan(_line_mission(42, 6)),
            rng=random.Random(17),
        )

        self.assertEqual(result["count"], 0)
        self.assertGreater(result["firstMissionExclusionRejectedCandidateCount"], 0)

    def test_selected_general_geometry_may_intentionally_receive_zero_targets(self) -> None:
        config = AutoTargetPlacementConfig(
            general_zone_probability=1.0,
            general_zone_min_count=0,
            general_zone_max_count=0,
        )
        result = generate_auto_target_placements(
            _plan(_line_mission(42, 3)),
            rng=random.Random(2),
            config=config,
        )

        self.assertEqual(result["selectedGeneralZoneCount"], 1)
        self.assertEqual(result["generalCount"], 0)
        self.assertEqual(result["count"], 0)

    def test_target_count_is_per_mission_across_all_geometry_components(self) -> None:
        mission = _line_mission(43, 6)
        mission["missionDetail"]["areaList"] = _area_mission(44, 6)["missionDetail"]["areaList"]
        config = AutoTargetPlacementConfig(
            target_region_min_count=5,
            target_region_max_count=5,
            min_separation_m=80.0,
        )
        result = generate_auto_target_placements(
            _plan(mission),
            rng=random.Random(12),
            config=config,
        )

        self.assertEqual(result["zoneCount"], 2)
        self.assertEqual(result["targetRegionCount"], 5)
        self.assertEqual(result["count"], 5)

    def test_injected_rng_keeps_placement_deterministic(self) -> None:
        plan = _plan(_line_mission(45, 3), _area_mission(46, 6))
        first = generate_auto_target_placements(plan, rng=random.Random(77))
        second = generate_auto_target_placements(plan, rng=random.Random(77))

        self.assertEqual(first, second)

    def test_completed_mission_is_not_populated(self) -> None:
        result = generate_auto_target_placements(
            _plan(_area_mission(50, 6, done=True)),
            rng=random.Random(3),
        )
        self.assertEqual(result["count"], 0)

    def test_sim_toggle_applies_current_geometry_only_once(self) -> None:
        sim = SimulationService()
        sim.geo = GeoConverter(127.02, 38.02)
        sim._loaded_input_mission_plans = _plan(_area_mission(60, 6))

        first = sim.set_auto_target_placement(True, apply_current=True)
        first_count = len(sim.targets)
        second = sim.set_auto_target_placement(True, apply_current=True)

        self.assertTrue(first["ok"])
        self.assertGreaterEqual(first_count, 2)
        self.assertLessEqual(first_count, 5)
        self.assertEqual(first["areaCount"], first_count)
        self.assertEqual(first["lineCount"], 0)
        self.assertTrue(second.get("alreadyPlaced"))
        self.assertEqual(len(sim.targets), first_count)

        disabled = sim.set_auto_target_placement(False, apply_current=True)
        self.assertFalse(disabled["autoTargetPlacementEnabled"])
        self.assertEqual(len(sim.targets), first_count)

    def test_enabled_mode_populates_the_next_fresh_mission_load(self) -> None:
        sim = SimulationService()
        enabled = sim.set_auto_target_placement(True, apply_current=True)
        self.assertTrue(enabled["autoTargetPlacementEnabled"])

        result = sim.load_mission(
            {
                "missionPlanID": 7100,
                "inputMissionPlans": _plan(_area_mission(61, 6)),
                "individualMissionPlans": [
                    {
                        "aircraftID": 4,
                        "individualMissionList": [
                            {
                                "individualMissionID": 8100,
                                "pathID": 4100,
                                "isDone": False,
                                "relatedMission": {"inputMissionID": 61},
                            }
                        ],
                    }
                ],
                "flightPaths": [
                    {
                        "pathID": 4100,
                        "aircraftID": 4,
                        "waypointList": [
                            {
                                "waypointID": 1100,
                                "nextWaypointID": 1101,
                                "isDone": False,
                                "waypointPassType": 1,
                                "coordinate": {
                                    "latitude": 38.001,
                                    "longitude": 127.001,
                                    "altitude": 300.0,
                                },
                                "speed": 30.0,
                            },
                            {
                                "waypointID": 1101,
                                "nextWaypointID": 0,
                                "isDone": False,
                                "waypointPassType": 1,
                                "coordinate": {
                                    "latitude": 38.002,
                                    "longitude": 127.002,
                                    "altitude": 300.0,
                                },
                                "speed": 30.0,
                            },
                        ],
                    }
                ],
            }
        )

        self.assertTrue(result["ok"])
        self.assertGreaterEqual(len(sim.targets), 2)
        self.assertLessEqual(len(sim.targets), 5)
        self.assertEqual(result["autoTargetPlacement"]["placed"], len(sim.targets))
        start_x, start_y = sim.geo.lonlat_to_xy(127.0, 38.0)
        for target in sim.targets:
            self.assertGreaterEqual(
                math.hypot(float(target.x) - start_x, float(target.y) - start_y),
                3_000.0,
            )


if __name__ == "__main__":
    unittest.main()
