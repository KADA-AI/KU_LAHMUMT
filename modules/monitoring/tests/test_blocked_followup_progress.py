from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from modules.monitoring.logic.mission_update import (
    _select_current_mission_id,
    _transition_target_input_id,
    build_uav_mission_view,
)
from modules.monitoring.logic.mission_progress import MissionProgressTracker


class BlockedFollowupProgressTests(unittest.TestCase):
    def test_blocked_only_future_package_has_no_current_progress_mission(
        self,
    ) -> None:
        blocked_missions = [
            {
                "individualMissionID": 900000001,
                "pathID": 400000001,
                "isDone": False,
                "executionBlockedUntilNextCollab": True,
                "relatedMission": {"inputMissionID": 5},
            },
            {
                "individualMissionID": 900000002,
                "pathID": 400000002,
                "isDone": False,
                "executionBlockedUntilNextCollab": True,
                "relatedMission": {"inputMissionID": 5},
            },
        ]

        self.assertIsNone(
            _select_current_mission_id(
                blocked_missions,
                "individualMissionID",
                current_input_id=4,
            )
        )
        # Once Input5 is the explicit collaborative handoff, the same retained
        # rows become selectable even if a legacy marker remains.
        self.assertEqual(
            _select_current_mission_id(
                blocked_missions,
                "individualMissionID",
                current_input_id=5,
                allow_blocked_current_input=True,
            ),
            900000001,
        )

    def test_missing_current_branch_does_not_fall_forward_to_unblocked_future(
        self,
    ) -> None:
        future_area = [
            {
                "individualMissionID": 900000613,
                "pathID": 400000120,
                "isDone": False,
                "relatedMission": {"inputMissionID": 5},
            }
        ]

        self.assertIsNone(
            _select_current_mission_id(
                future_area,
                "individualMissionID",
                current_input_id=4,
            )
        )

    def test_declared_line_input_wins_over_mixed_future_area_branch(self) -> None:
        tracker = MissionProgressTracker()
        tracker.reset(
            {
                "current_input_mission_id": 4,
                "input_missions": [
                    {"input_mission_id": 4, "is_done": False},
                    {"input_mission_id": 5, "is_done": False},
                ],
                "uav_entries": [
                    {
                        "aircraft_id": 4,
                        "individual_mission_package_id": 800000134,
                        "current_individual_mission_id": 900000613,
                        "missions": [
                            {
                                "individual_mission_id": 900000613,
                                "input_id": 5,
                                "path_id": 400000120,
                                "waypoint_ids": [13001],
                                "waypoints": [{"waypoint_id": 13001}],
                                "is_done": False,
                                "eta_seconds": 10,
                            }
                        ],
                    },
                    {
                        "aircraft_id": 5,
                        "individual_mission_package_id": 800000135,
                        "current_individual_mission_id": 900000620,
                        "missions": [
                            {
                                "individual_mission_id": 900000620,
                                "input_id": 4,
                                "path_id": 500000124,
                                "waypoint_ids": [14001],
                                "waypoints": [{"waypoint_id": 14001}],
                                "is_done": False,
                                "eta_seconds": 10,
                            }
                        ],
                    },
                ],
            }
        )

        self.assertEqual(tracker.get_active_input_id(), 4)

    def test_derived_replan_recovers_latest_explicit_next_collab_input(self) -> None:
        with (
            patch(
                "modules.monitoring.logic.mission_update."
                "next_collab_replan_store.load_detail",
                return_value=None,
            ),
            patch(
                "modules.monitoring.logic.mission_update."
                "next_collab_replan_store.load_latest_detail_at_or_before",
                return_value={
                    "missionPlanID": 700000015,
                    "inputMissionPackageID": 3,
                    "targetInputMissionID": 4,
                },
            ) as latest,
        ):
            resolved = _transition_target_input_id(
                700000024,
                {1, 2, 3, 4, 5},
                input_mission_package_id=3,
            )

        self.assertEqual(resolved, 4)
        latest.assert_called_once_with(
            700000024,
            input_mission_package_id=3,
        )

    def test_normalized_view_excludes_blocked_future_from_progress(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            for folder in (
                "MissionPlan",
                "InputMissionPlan",
                "IndividualMissionPlan",
                "FlightPath",
            ):
                (root / folder).mkdir(parents=True, exist_ok=True)

            (root / "MissionPlan" / "700000001.json").write_text(
                json.dumps(
                    {
                        "inputMissionPackageID": 710000001,
                        "aircraftList": [
                            {
                                "aircraftID": 4,
                                "individualMissionPackageID": 800000001,
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (root / "InputMissionPlan" / "710000001.json").write_text(
                json.dumps(
                    {
                        "inputMissionList": [
                            {
                                "inputMissionID": 4,
                                "inputMissionType": 2,
                                "regionType": 2,
                                "isDone": False,
                            },
                            {
                                "inputMissionID": 5,
                                "inputMissionType": 2,
                                "regionType": 3,
                                "isDone": False,
                            },
                        ]
                    }
                ),
                encoding="utf-8",
            )
            (root / "IndividualMissionPlan" / "800000001.json").write_text(
                json.dumps(
                    {
                        "individualMissionList": [
                            {
                                "individualMissionID": 900000001,
                                "pathID": 400000001,
                                "isDone": False,
                                "executionBlockedUntilNextCollab": True,
                                "relatedMission": {"inputMissionID": 5},
                                "individualMissionInfo": {
                                    "individualMissionType": 4,
                                    "patternType": 6,
                                },
                            },
                            {
                                "individualMissionID": 900000002,
                                "pathID": 400000002,
                                "isDone": False,
                                "relatedMission": {"inputMissionID": 5},
                                "individualMissionInfo": {
                                    "individualMissionType": 4,
                                    "patternType": 6,
                                },
                            },
                        ]
                    }
                ),
                encoding="utf-8",
            )
            (root / "FlightPath" / "400000002.json").write_text(
                json.dumps(
                    {
                        "pathID": 400000002,
                        "aircraftID": 4,
                        "individualMissionID": 900000002,
                        "waypointList": [
                            {
                                "waypointID": 1002,
                                "isDone": False,
                                "coordinate": {
                                    "latitude": 38.0,
                                    "longitude": 127.01,
                                    "altitude": 900,
                                },
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (root / "FlightPath" / "400000001.json").write_text(
                json.dumps(
                    {
                        "pathID": 400000001,
                        "aircraftID": 4,
                        "individualMissionID": 900000001,
                        "waypointList": [
                            {
                                "waypointID": 1001,
                                "isDone": False,
                                "coordinate": {
                                    "latitude": 38.0,
                                    "longitude": 127.0,
                                    "altitude": 900,
                                },
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            view = build_uav_mission_view(
                700000001,
                uav_ids=(4,),
                db_root=root,
            )

        entry = view["uav_entries"][0]
        self.assertIsNone(entry["current_individual_mission_id"])
        mission = entry["missions"][0]
        self.assertTrue(mission["execution_blocked_until_next_collab"])
        self.assertTrue(mission["skip_progress"])
        self.assertTrue(mission["skip_pending"])
        unmarked_future = entry["missions"][1]
        self.assertTrue(unmarked_future["execution_blocked_until_next_collab"])
        self.assertTrue(unmarked_future["skip_progress"])
        self.assertTrue(unmarked_future["skip_pending"])

        tracker = MissionProgressTracker()
        tracker.set_system_mode(3)
        tracker.reset(view)
        snapshot = tracker.update(
            1_000,
            [
                {
                    "aircraft_id": 4,
                    "current_waypoint_id": 0,
                    "flying": 2,
                    "filming": 2,
                    "flight_mode": 8,
                }
            ],
        )
        self.assertNotIn(900000001, snapshot["mission_progress"])
        self.assertNotIn(900000002, snapshot["mission_progress"])
        self.assertEqual(snapshot["new_completed_individual"], [])
        self.assertEqual(snapshot["new_completed_waypoints"], [])


if __name__ == "__main__":
    unittest.main()
