from __future__ import annotations

import unittest
from copy import deepcopy
from unittest.mock import patch

from modules.common import mission_area_replan_store
from modules.mission_planning.pipelines import next_collab_path_builder
from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
    _area_coverage_pass_contract_from_input_mission,
    _area_planner_components_from_detail,
    _single_area_ownership_component,
)
from modules.sim.mission.remaining_area_loader import _features_from_snapshot
from modules.sim.runtime.controllers.waypoint_pid import WaypointPIDController


def _area_detail(offset: float = 0.0) -> dict:
    coords = [
        {"latitude": 38.0 + offset, "longitude": 127.0},
        {"latitude": 38.0 + offset, "longitude": 127.01},
        {"latitude": 38.01 + offset, "longitude": 127.01},
        {"latitude": 38.01 + offset, "longitude": 127.0},
    ]
    return {
        "coordinateList": deepcopy(coords),
        "lineList": [],
        "areaList": [{"isHole": False, "coordinateList": deepcopy(coords)}],
    }


def _legacy_reciprocal_area() -> dict:
    forward_remaining = _area_detail(0.0)
    reverse_remaining = _area_detail(0.02)
    return {
        "missionType": "area",
        "inputMissionID": 70000002,
        "isDone": False,
        "remainingDetail": deepcopy(reverse_remaining),
        "areaCoveragePassContractVersion": 1,
        "coveragePassPolicy": "all_passes_required",
        "coveragePassOrder": ["forward", "reverse"],
        "remainingCoveragePasses": ["forward", "reverse"],
        "areaCoveragePhase": "outbound",
        "coveragePassDetails": [
            {
                "coveragePass": "forward",
                "plannedAreaM2": 100.0,
                "coveredAreaM2": 40.0,
                "remainingAreaM2": 60.0,
                "coveragePercent": 40,
                "isDone": False,
                "remainingDetail": deepcopy(forward_remaining),
            },
            {
                "coveragePass": "reverse",
                "plannedAreaM2": 100.0,
                "coveredAreaM2": 0.0,
                "remainingAreaM2": 100.0,
                "coveragePercent": 0,
                "isDone": False,
                "remainingDetail": deepcopy(reverse_remaining),
            },
        ],
        "coverageDepthDetails": [
            {
                "coverageDepth": 0,
                "remainingCaptureCount": 2,
                "remainingDetail": deepcopy(forward_remaining),
            },
            {
                "coverageDepth": 1,
                "remainingCaptureCount": 1,
                "remainingDetail": deepcopy(reverse_remaining),
            },
        ],
        "areaOwnershipDetails": [
            {
                "aircraftID": 4,
                "coveragePass": "forward",
                "isDone": False,
                "areaAssignmentDetail": deepcopy(forward_remaining),
                "remainingDetail": deepcopy(forward_remaining),
            },
            {
                "aircraftID": 4,
                "coveragePass": "reverse",
                "isDone": False,
                "areaAssignmentDetail": deepcopy(reverse_remaining),
                "remainingDetail": deepcopy(reverse_remaining),
            },
        ],
    }


class AreaSingleCaptureTests(unittest.TestCase):
    def test_legacy_two_pass_snapshot_becomes_one_capture_region(self) -> None:
        normalized = mission_area_replan_store.normalize_area_single_capture_entry(
            _legacy_reciprocal_area()
        )

        self.assertEqual(normalized["areaCapturePolicy"], "single_capture")
        self.assertEqual(normalized["requiredCoverageDepth"], 1)
        self.assertEqual(normalized["coveragePercent"], 40)
        self.assertEqual(normalized["remainingAreaM2"], 60.0)
        self.assertNotIn("coveragePassDetails", normalized)
        self.assertNotIn("coverageDepthDetails", normalized)
        self.assertNotIn("areaCoveragePhase", normalized)
        self.assertEqual(len(normalized["remainingDetail"]["areaList"]), 1)
        self.assertEqual(len(normalized["areaOwnershipDetails"]), 1)
        self.assertEqual(
            normalized["areaLogicalRegionPolicy"],
            "one_single_capture_region_per_aircraft",
        )
        self.assertEqual(len(normalized["areaLogicalRegionDetails"]), 1)

    def test_replan_uses_only_never_captured_depth_zero_geometry(self) -> None:
        source = _legacy_reciprocal_area()
        source.pop("coveragePassDetails")
        source.pop("coveragePassOrder")
        source.pop("remainingCoveragePasses")
        source.pop("areaCoveragePassContractVersion")

        remaining = mission_area_replan_store.coverage_replan_pending_remaining_detail(
            source
        )

        self.assertIsInstance(remaining, dict)
        self.assertEqual(len(remaining["areaList"]), 1)
        self.assertEqual(
            remaining["areaList"][0]["coordinateList"][0]["latitude"],
            38.0,
        )

    def test_next_collab_has_no_pass_contract_and_one_component(self) -> None:
        source = _legacy_reciprocal_area()
        normalized = mission_area_replan_store.normalize_area_single_capture_entry(source)
        input_mission = {"missionDetail": normalized}

        self.assertEqual(_area_coverage_pass_contract_from_input_mission(input_mission), {})
        self.assertEqual(
            len(_area_planner_components_from_detail(normalized["remainingDetail"])),
            1,
        )
        self.assertFalse(
            next_collab_path_builder._area_pass_contract(source, source)["explicit"]
        )

    def test_fragmented_remaining_geometry_is_one_logical_planning_region(self) -> None:
        fragmented = _area_detail(0.0)
        fragmented["areaList"].extend(_area_detail(0.03)["areaList"])

        component = _single_area_ownership_component(fragmented)

        self.assertIsInstance(component, dict)
        self.assertGreaterEqual(len(component.get("coordinateList") or []), 3)

    def test_force_active_cannot_restore_reciprocal_pass(self) -> None:
        with patch.object(
            next_collab_path_builder,
            "get_runtime_bool",
            return_value=False,
        ):
            plan = next_collab_path_builder._build_area_reciprocal_pass_plan(
                {},
                [],
                [],
                turn_radius_m=500.0,
                force_active=True,
            )
        self.assertFalse(plan["active"])
        self.assertEqual(plan["reason"], "single_capture_policy")

    def test_sim_renders_legacy_snapshot_as_plain_remaining_area(self) -> None:
        features = _features_from_snapshot(
            {"missionPlanID": 1, "missions": [_legacy_reciprocal_area()]}
        )

        self.assertTrue(features)
        roles = {
            str((feature.get("properties") or {}).get("visualizationRole") or "")
            for feature in features
        }
        self.assertNotIn("coverageDepth", roles)
        self.assertNotIn("coveragePass", roles)

    def test_sim_drops_legacy_return_and_reciprocal_turn_waypoints(self) -> None:
        controller = WaypointPIDController(
            object(),
            [
                {"pos": (0, 0, 100), "wp_id": 1, "area_coverage_pass": "forward"},
                {"pos": (1, 0, 100), "wp_id": 2, "area_turn_role": "reciprocal_turn"},
                {"pos": (2, 0, 100), "wp_id": 3, "area_coverage_pass": "reverse"},
            ],
        )

        self.assertEqual([target.wp_id for target in controller.targets], [1])

    def test_carried_area_ownership_is_rebound_to_replanned_target_assignments(self) -> None:
        old_owner_area = _area_detail(0.0)
        snapshot = {
            "missionPlanID": 15,
            "missions": [
                {
                    "missionPlanID": 15,
                    "missionType": "area",
                    "inputMissionID": 70000003,
                    "plannedAreaM2": 300.0,
                    "remainingAreaM2": 180.0,
                    "coveragePercent": 40,
                    "isDone": False,
                    "remainingDetail": deepcopy(old_owner_area),
                    "areaOwnershipDetails": [
                        {
                            "aircraftID": 4,
                            "individualMissionID": 41,
                            "inputMissionID": 70000003,
                            "sourceMissionPlanID": 15,
                            "pathID": 4001,
                            "remainingDetail": deepcopy(old_owner_area),
                            "areaAssignmentDetail": deepcopy(old_owner_area),
                            "takeoverPolicy": "piece_only",
                        },
                        {
                            "aircraftID": 5,
                            "individualMissionID": 51,
                            "inputMissionID": 70000003,
                            "sourceMissionPlanID": 15,
                            "pathID": 5001,
                            "remainingDetail": deepcopy(old_owner_area),
                            "areaAssignmentDetail": deepcopy(old_owner_area),
                            "takeoverPolicy": "piece_only",
                        },
                    ],
                }
            ],
        }
        assignments = {
            70000003: [
                {
                    "aircraftID": aircraft_id,
                    "individualMissionID": aircraft_id * 10,
                    "inputMissionID": 70000003,
                    "sourceMissionPlanID": 18,
                    "pathID": aircraft_id * 1000,
                    "isDone": False,
                    "areaAssignmentDetail": _area_detail(index * 0.02),
                    "remainingDetail": _area_detail(index * 0.02),
                    "takeoverPolicy": "target_plan_assignment",
                }
                for index, aircraft_id in enumerate((4, 5, 6))
            ]
        }

        rebound, input_ids = (
            mission_area_replan_store._rebind_area_snapshot_ownership_to_target_plan(
                snapshot,
                18,
                input_mission_ids={70000003},
                assignments_by_input=assignments,
            )
        )

        mission = rebound["missions"][0]
        self.assertEqual(input_ids, [70000003])
        self.assertEqual(mission["coveragePercent"], 40)
        self.assertEqual(mission["remainingAreaM2"], 180.0)
        self.assertEqual(mission["aircraftIDs"], [4, 5, 6])
        self.assertEqual(
            [row["pathID"] for row in mission["areaOwnershipDetails"]],
            [4000, 5000, 6000],
        )
        self.assertEqual(
            {row["progressSource"] for row in mission["areaProgressDetails"]},
            {"target_plan_assignment"},
        )


if __name__ == "__main__":
    unittest.main()
