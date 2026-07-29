from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
    _normalize_reexecute_next_input_ids,
)
from modules.monitoring.gui.tabs.monitoring_visualization_tab import (
    MonitoringVisualizationTab,
)
from modules.monitoring.logic.collab_reexecute import CollabReexecuteCoordinator


class _ActiveInputStub:
    def __init__(self, input_id: int) -> None:
        self.input_id = int(input_id)

    def get_active_input_id(self) -> int:
        return int(self.input_id)


class ReexecuteNextCollaborativeTransitionTests(unittest.TestCase):
    def test_monitor_uses_clone_as_current_and_selects_following_input(self) -> None:
        tab = MonitoringVisualizationTab.__new__(MonitoringVisualizationTab)
        tab._mission_view = {
            "input_mission_package_id": 4,
            "input_missions": [
                {"input_mission_id": 1, "input_mission_type": 1, "is_done": True},
                {"input_mission_id": 9, "input_mission_type": 1, "is_done": False},
                {"input_mission_id": 2, "input_mission_type": 1, "is_done": False},
            ],
            "uav_entries": [
                {
                    "aircraft_id": 4,
                    "missions": [
                        {
                            "input_id": 9,
                            "waypoints": [{"latitude": 37.0, "longitude": 127.0}],
                        },
                        {
                            "input_id": 2,
                            "waypoints": [{"latitude": 37.1, "longitude": 127.1}],
                        },
                    ],
                }
            ],
        }
        tab._progress_tracker = _ActiveInputStub(1)
        tab._last_progress_input_id = None
        tab._last_active_input_id = None
        tab._last_effective_current_input_id = None
        tab._last_execute_next_source_input_id = None
        tab._execute_next_recommendation_source_inputs = set()
        tab._sent_0503_pending_inputs = set()
        tab._sent_0503_inputs = set()
        tab._pending_execute_inputs = []
        tab._emit_log = lambda _message: None

        context = tab.build_execute_next_replan_context(
            reexecute_source_input_id=1,
            reexecute_clone_input_id=9,
        )

        self.assertIsNotNone(context)
        self.assertEqual(context["current_input_mission_id"], 9)
        self.assertEqual(context["target_input_mission_id"], 2)
        self.assertEqual(context["reexecute_source_input_mission_id"], 1)
        self.assertEqual(context["reexecute_clone_input_mission_id"], 9)

    def test_planner_repairs_queued_source_to_clone_request(self) -> None:
        input_plan = {
            "inputMissionList": [
                {"inputMissionID": 1, "isDone": True},
                {"inputMissionID": 9, "isDone": False},
                {"inputMissionID": 2, "isDone": False},
                {"inputMissionID": 3, "isDone": False},
            ]
        }
        detail = {
            "reexecuteSourceInputMissionID": 1,
            "reexecuteCloneInputMissionID": 9,
        }

        current_id, target_id, applied = _normalize_reexecute_next_input_ids(
            input_plan,
            detail,
            1,
            9,
        )

        self.assertTrue(applied)
        self.assertEqual(current_id, 9)
        self.assertEqual(target_id, 2)

    def test_clone_mapping_expires_when_a_different_0201_arrives(self) -> None:
        coordinator = CollabReexecuteCoordinator(now_fn=lambda: 1000)
        reexecute_0201 = {
            "timestamp": 10,
            "inputMissionPackageID": 4,
            "inputMissionList": [
                {"inputMissionID": 1, "isDone": True, "missionDetail": {"x": 1}},
                {"inputMissionID": 9, "isDone": False, "missionDetail": {"x": 1}},
                {"inputMissionID": 2, "isDone": False, "missionDetail": {"x": 2}},
            ],
        }
        coordinator.on_input_plan(reexecute_0201)
        self.assertEqual(coordinator.current_clone_mapping(), (1, 9))

        coordinator.on_input_plan(
            {
                "timestamp": 11,
                "inputMissionPackageID": 5,
                "inputMissionList": [{"inputMissionID": 1, "isDone": False}],
            }
        )
        self.assertIsNone(coordinator.current_clone_mapping())


if __name__ == "__main__":
    unittest.main()
