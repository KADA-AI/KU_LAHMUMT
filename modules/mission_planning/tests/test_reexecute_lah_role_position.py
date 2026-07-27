from __future__ import annotations

import ast
import unittest
from pathlib import Path
from unittest.mock import patch

from modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304 import d0304
from modules.mission_planning.replanning.reexecute_lah_role import (
    has_reusable_lah_role_geometry,
    rebind_reexecute_lah_role_mission,
    resolve_reexecute_lah_template_input_id,
)


def _source_role_mission(aircraft_id: int = 1) -> dict:
    return {
        "aircraftID": int(aircraft_id),
        "individualMissionID": 800000040 + int(aircraft_id),
        "isDone": False,
        "relatedMission": {
            "relatedMissionType": 1,
            "inputMissionID": 70000006,
            "priorMissionID": 0,
        },
        "individualMissionInfo": {
            "individualMissionType": 9,
            "patternType": 12,
            "forceAltitudeM": 1500,
            "coordinateList": [
                {"latitude": 38.0794255, "longitude": 127.22531, "altitude": 1500}
            ],
        },
        "pathID": 100000001,
    }


class ReexecuteLahRolePositionTests(unittest.TestCase):
    def test_gui_general_initial_plan_builds_forward_takeover_hold_map(self) -> None:
        gui_path = Path(__file__).resolve().parents[1] / "mission_planning_gui.py"
        tree = ast.parse(gui_path.read_text(encoding="utf-8-sig"))
        direct_build_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "build_lah_flight_plans_fixed"
        ]
        forwarded_calls = [
            node
            for node in direct_build_calls
            if any(keyword.arg == "initial_hold_by_aircraft" for keyword in node.keywords)
        ]

        # The current-role reexecute builder intentionally keeps its live-position
        # behavior.  Both general variant builders must forward the Type-1 initial
        # TO route-start map; these were the two production call paths that were missing it.
        self.assertGreaterEqual(len(direct_build_calls), 3)
        self.assertEqual(len(forwarded_calls), len(direct_build_calls) - 1)

    def test_takeover_map_pairs_each_lah_at_uav_takeover_coordinate(self) -> None:
        mrpk = {
            "takeOverInfoList": [
                {
                    "aircraftID": 4,
                    "coordinate": {"latitude": 38.0, "longitude": 127.20, "altitude": 1000},
                },
                {
                    "aircraftID": 5,
                    "coordinate": {"latitude": 38.1, "longitude": 127.21, "altitude": 1000},
                },
                {
                    "aircraftID": 6,
                    "coordinate": {"latitude": 38.2, "longitude": 127.22, "altitude": 1000},
                },
            ]
        }

        holds = d0304.lah_initial_hold_by_aircraft_from_mrpk(mrpk)

        self.assertEqual(set(holds), {1, 2, 3})
        self.assertAlmostEqual(holds[1]["longitude"], 127.20, places=7)
        self.assertAlmostEqual(holds[2]["longitude"], 127.21, places=7)
        self.assertAlmostEqual(holds[3]["longitude"], 127.22, places=7)
        self.assertAlmostEqual(holds[1]["latitude"], 38.0, places=7)
        self.assertAlmostEqual(holds[2]["latitude"], 38.1, places=7)
        self.assertAlmostEqual(holds[3]["latitude"], 38.2, places=7)

    def test_takeover_initial_hold_keeps_lah_formation_offset(self) -> None:
        mission = _source_role_mission(2)
        mission["pathID"] = 200000001
        takeover_hold = {"latitude": 38.0, "longitude": 127.20}

        with patch.object(d0304, "_terrain_elev_cached", return_value=100.0):
            [flight_plan] = d0304.build_lah_flight_plans_fixed(
                [mission],
                initial_hold_by_aircraft={2: takeover_hold},
                wp_alloc=d0304._WPAllocator(start=904001, end=904100),
            )

        first_coordinate = flight_plan["lahWaypointList"][0]["coordinate"]
        expected_latitude = 38.0 + (100.0 / 111_132.92)
        self.assertAlmostEqual(first_coordinate["latitude"], expected_latitude, places=6)
        self.assertAlmostEqual(first_coordinate["longitude"], 127.20, places=7)

    def test_0304_first_mission_transits_from_takeover_then_holds_at_mission_point(self) -> None:
        first_hold = _source_role_mission(1)
        first_hold["pathID"] = 100000001
        next_mission = _source_role_mission(1)
        next_mission["pathID"] = 100000002
        next_mission["individualMissionInfo"] = {
            "individualMissionType": 7,
            "patternType": 10,
            "coordinateList": [
                {"latitude": 38.01, "longitude": 127.20, "altitude": 1500},
                {"latitude": 38.02, "longitude": 127.20, "altitude": 1500},
            ],
        }
        takeover_hold = {"latitude": 38.0, "longitude": 127.20}

        with patch.object(d0304, "_terrain_elev_cached", return_value=100.0):
            flight_plans = d0304.build_lah_flight_plans_fixed(
                [first_hold, next_mission],
                initial_hold_by_aircraft={1: takeover_hold},
                wp_alloc=d0304._WPAllocator(start=905001, end=905050),
            )
            baseline_flight_plans = d0304.build_lah_flight_plans_fixed(
                [first_hold, next_mission],
                wp_alloc=d0304._WPAllocator(start=906001, end=906050),
            )

        self.assertEqual(len(flight_plans), 2)
        first_coordinates = [
            waypoint["coordinate"] for waypoint in flight_plans[0]["lahWaypointList"]
        ]
        second_coordinates = [
            waypoint["coordinate"] for waypoint in flight_plans[1]["lahWaypointList"]
        ]
        self.assertAlmostEqual(first_coordinates[0]["latitude"], 38.0, places=6)
        self.assertAlmostEqual(first_coordinates[-1]["latitude"], 38.0794255, places=6)
        self.assertAlmostEqual(second_coordinates[0]["latitude"], 38.0794255, places=6)
        self.assertIn("hovering", flight_plans[0]["lahWaypointList"][-1])
        # Existing LINE policy keeps the center 30% of the submitted line.
        self.assertAlmostEqual(second_coordinates[-1]["latitude"], 38.0165, places=6)
        expected_altitude = int(100 + d0304.LAH_NON_ATTACK_CLEARANCE_M)
        self.assertTrue(all(coord["altitude"] == expected_altitude for coord in first_coordinates))
        self.assertTrue(all(coord["altitude"] == expected_altitude for coord in second_coordinates))
        baseline_second_coordinates = [
            waypoint["coordinate"]
            for waypoint in baseline_flight_plans[1]["lahWaypointList"]
        ]
        self.assertEqual(second_coordinates, baseline_second_coordinates)

    def test_0304_without_takeover_override_preserves_existing_initial_hold(self) -> None:
        mission = _source_role_mission(1)

        with patch.object(d0304, "_terrain_elev_cached", return_value=100.0):
            [flight_plan] = d0304.build_lah_flight_plans_fixed(
                [mission],
                wp_alloc=d0304._WPAllocator(start=907001, end=907010),
            )

        first_coordinate = flight_plan["lahWaypointList"][0]["coordinate"]
        self.assertAlmostEqual(first_coordinate["latitude"], 38.0794255, places=6)
        self.assertAlmostEqual(first_coordinate["longitude"], 127.22531, places=6)

    def test_original_input_id_is_used_for_role_template_lookup(self) -> None:
        self.assertEqual(
            resolve_reexecute_lah_template_input_id(70000010, 70000006),
            70000006,
        )
        self.assertEqual(
            resolve_reexecute_lah_template_input_id(70000010, None),
            70000010,
        )

    def test_single_point_type9_role_is_reusable_and_rebound(self) -> None:
        source = _source_role_mission()
        self.assertTrue(has_reusable_lah_role_geometry(source))

        rebound = rebind_reexecute_lah_role_mission(
            source,
            aircraft_id=2,
            current_input_id=70000010,
            path_id=100000020,
        )

        self.assertEqual(rebound["aircraftID"], 2)
        self.assertEqual(rebound["individualMissionID"], 0)
        self.assertEqual(rebound["relatedMission"]["inputMissionID"], 70000010)
        self.assertEqual(rebound["pathID"], 100000020)
        self.assertEqual(rebound["individualMissionInfo"], source["individualMissionInfo"])
        self.assertEqual(source["relatedMission"]["inputMissionID"], 70000006)

    def test_0304_places_lah_formation_at_role_anchor(self) -> None:
        missions = [
            rebind_reexecute_lah_role_mission(
                _source_role_mission(aid),
                aircraft_id=aid,
                current_input_id=70000010,
                path_id=aid * 100000000 + 20,
            )
            for aid in (1, 2, 3)
        ]

        with patch.object(d0304, "_terrain_elev_cached", return_value=120.0):
            flight_plans = d0304.build_lah_flight_plans_fixed(
                missions,
                wp_alloc=d0304._WPAllocator(start=900001, end=900006),
            )

        self.assertEqual(len(flight_plans), 3)
        first_coordinates = {
            int(plan["aircraftID"]): plan["lahWaypointList"][0]["coordinate"]
            for plan in flight_plans
        }
        self.assertEqual(first_coordinates[1]["longitude"], 127.22531)
        self.assertGreater(first_coordinates[2]["latitude"], first_coordinates[1]["latitude"])
        self.assertLess(first_coordinates[3]["latitude"], first_coordinates[1]["latitude"])
        expected_altitude = int(120 + d0304.LAH_NON_ATTACK_CLEARANCE_M)
        self.assertTrue(
            all(coord["altitude"] == expected_altitude for coord in first_coordinates.values())
        )

    def test_0304_preserves_attack_mission_altitude_policy(self) -> None:
        mission = _source_role_mission(1)
        mission["individualMissionInfo"]["individualMissionType"] = 2
        mission["individualMissionInfo"]["forceAltitudeM"] = 730

        with patch.object(d0304, "_terrain_elev_cached", return_value=120.0):
            [flight_plan] = d0304.build_lah_flight_plans_fixed(
                [mission],
                wp_alloc=d0304._WPAllocator(start=910001, end=910002),
            )

        coordinates = [
            waypoint["coordinate"]
            for waypoint in flight_plan["lahWaypointList"]
        ]
        self.assertTrue(all(coord["altitude"] == 730 for coord in coordinates))

    def test_0304_keeps_adaptive_segments_above_dem_clearance_for_non_attack(self) -> None:
        mission = _source_role_mission(1)
        mission["individualMissionInfo"]["coordinateList"] = [
            {"latitude": 38.0, "longitude": 127.2, "altitude": 1500},
            {"latitude": 38.01, "longitude": 127.2, "altitude": 1500},
        ]

        def terrain_at_waypoint(latitude: float, _longitude: float) -> float:
            return 100.0 if float(latitude) < 38.005 else 220.0

        with patch.object(d0304, "_terrain_elev_cached", side_effect=terrain_at_waypoint):
            [flight_plan] = d0304.build_lah_flight_plans_fixed(
                [mission],
                wp_alloc=d0304._WPAllocator(start=920001, end=920020),
            )

        altitudes = [
            waypoint["coordinate"]["altitude"]
            for waypoint in flight_plan["lahWaypointList"]
        ]
        low_altitude = int(100 + d0304.LAH_NON_ATTACK_CLEARANCE_M)
        high_altitude = int(220 + d0304.LAH_NON_ATTACK_CLEARANCE_M)
        self.assertEqual(altitudes[0], low_altitude)
        self.assertEqual(altitudes[-1], high_altitude)
        for waypoint in flight_plan["lahWaypointList"]:
            coordinate = waypoint["coordinate"]
            expected = low_altitude if float(coordinate["latitude"]) < 38.005 else high_altitude
            self.assertGreaterEqual(coordinate["altitude"], expected)

    def test_0304_builds_transit_from_live_start_to_single_mission_point(self) -> None:
        mission = _source_role_mission(1)
        mission["individualMissionInfo"]["coordinateList"] = [
            {"latitude": 38.01, "longitude": 127.2, "altitude": 1500},
        ]

        with patch.object(d0304, "_terrain_elev_cached", return_value=100.0):
            [flight_plan] = d0304.build_lah_flight_plans_fixed(
                [mission],
                route_start_by_aircraft={
                    1: {"latitude": 38.0, "longitude": 127.2, "altitude": 999},
                },
                wp_alloc=d0304._WPAllocator(start=930001, end=930020),
            )

        coordinates = [
            waypoint["coordinate"]
            for waypoint in flight_plan["lahWaypointList"]
        ]
        self.assertGreaterEqual(len(coordinates), 3)
        self.assertAlmostEqual(coordinates[0]["latitude"], 38.0, places=6)
        self.assertEqual(coordinates[0]["altitude"], 999)
        self.assertAlmostEqual(coordinates[1]["latitude"], 38.0, places=6)
        expected_altitude = int(100 + d0304.LAH_NON_ATTACK_CLEARANCE_M)
        self.assertEqual(coordinates[1]["altitude"], expected_altitude)
        self.assertAlmostEqual(coordinates[-1]["latitude"], 38.01, places=6)
        self.assertEqual(coordinates[-1]["altitude"], expected_altitude)
        self.assertEqual(flight_plan["lahWaypointList"][0]["eta"], 0)
        self.assertGreater(flight_plan["lahWaypointList"][1]["eta"], 0)

    def test_0304_preserves_multi_point_replan_return_prefix(self) -> None:
        mission = _source_role_mission(1)
        mission["individualMissionInfo"]["coordinateList"] = [
            {"latitude": 38.01, "longitude": 127.2, "altitude": 1500},
        ]
        route_prefix = [
            {"latitude": 38.0, "longitude": 127.2, "altitude": 999},
            {"latitude": 38.005, "longitude": 127.195, "altitude": 999},
            {"latitude": 38.01, "longitude": 127.2, "altitude": 1500},
        ]

        with patch.object(d0304, "_terrain_elev_cached", return_value=100.0):
            [flight_plan] = d0304.build_lah_flight_plans_fixed(
                [mission],
                route_start_by_aircraft={1: route_prefix},
                wp_alloc=d0304._WPAllocator(start=940001, end=940040),
            )

        coordinates = [
            waypoint["coordinate"]
            for waypoint in flight_plan["lahWaypointList"]
        ]
        self.assertAlmostEqual(coordinates[0]["latitude"], 38.0, places=6)
        self.assertAlmostEqual(coordinates[0]["longitude"], 127.2, places=6)
        self.assertEqual(coordinates[0]["altitude"], 999)
        self.assertAlmostEqual(coordinates[1]["latitude"], 38.0, places=6)
        self.assertAlmostEqual(coordinates[1]["longitude"], 127.2, places=6)
        self.assertGreater(flight_plan["lahWaypointList"][1]["eta"], 0)
        self.assertTrue(
            any(
                abs(float(coord["latitude"]) - 38.005) < 1e-5
                and abs(float(coord["longitude"]) - 127.195) < 1e-5
                for coord in coordinates
            )
        )
        self.assertAlmostEqual(coordinates[-1]["latitude"], 38.01, places=6)
        self.assertAlmostEqual(coordinates[-1]["longitude"], 127.2, places=6)


if __name__ == "__main__":
    unittest.main()
