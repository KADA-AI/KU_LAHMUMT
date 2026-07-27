from __future__ import annotations

import unittest
from unittest.mock import patch

from modules.monitoring.logic.input_refresh_replan import (
    InputRefreshReplanCoordinator,
)


class InputRefreshReplanProgressTests(unittest.TestCase):
    def test_replan_request_carries_active_input_mission_id(self) -> None:
        coordinator = InputRefreshReplanCoordinator(now_fn=lambda: 123456789)
        payload = {
            "timestamp": 123456700,
            "inputMissionPackageID": 103,
            "inputMissionList": [
                {"inputMissionID": 70000000},
                {"inputMissionID": 70000001},
                {"inputMissionID": 70000015},
            ],
        }

        with patch(
            "modules.monitoring.logic.input_refresh_replan.allocate_mission_plan_ids",
            return_value=[700000002, 700000003, 700000004],
        ):
            request, _logs = coordinator.on_input_plan(
                payload,
                system_mode=3,
                blocked=False,
                current_mission_plan_id=700000001,
                current_input_mission_id=70000000,
            )

        self.assertIsNotNone(request)
        detail = request["replanDetail"]
        self.assertEqual(detail["currentInputMissionID"], 70000000)
        self.assertTrue(detail["preserveCurrentMissionProgress"])
        self.assertEqual(detail["sourceMissionPlanID"], 700000001)


if __name__ == "__main__":
    unittest.main()
