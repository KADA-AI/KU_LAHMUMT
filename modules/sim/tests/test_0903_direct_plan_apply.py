from __future__ import annotations

import threading
import unittest
from unittest.mock import Mock, patch

from modules.sim.integration.integration_service import IntegrationService


class _FakeSim:
    def __init__(self, loaded_plan_id: int = 700000017) -> None:
        self._loaded_mission_plan_id = loaded_plan_id
        self._mission_load_lock = threading.RLock()
        self.loaded_payload: dict | None = None

    def load_mission(self, payload: dict) -> dict:
        self.loaded_payload = dict(payload)
        self._loaded_mission_plan_id = int(payload["missionPlanID"])
        return {"ok": True, "missionPlanID": self._loaded_mission_plan_id}


def _ready_plan_result() -> dict:
    individual_plan = {
        "individualMissionPackageID": 800000088,
        "aircraftID": 5,
        "individualMissionList": [
            {
                "individualMissionID": 900000177,
                "pathID": 500000039,
                "relatedMission": {"inputMissionID": 70000003},
                "isDone": False,
            }
        ],
    }
    return {
        "ok": True,
        "missionPlan": {
            "aircraftList": [
                {"aircraftID": 5, "individualMissionPackageID": 800000088}
            ]
        },
        "inputMissionPlans": [{"inputMissionPackageID": 300000001}],
        "individualMissionPlans": [individual_plan],
        "flightPaths": [{"pathID": 500000039, "uavWaypointList": []}],
        "missingPathIds": [],
        "payload": {
            "inputMissionPlans": [{"inputMissionPackageID": 300000001}],
            "individualMissionPlans": [individual_plan],
            "flightPaths": [{"pathID": 500000039, "uavWaypointList": []}],
        },
    }


class Direct0903PlanApplyTests(unittest.TestCase):
    def _service(self) -> IntegrationService:
        with patch.object(IntegrationService, "_init_bus", return_value=None):
            return IntegrationService()

    def test_0903_without_0803_latch_uses_direct_plan_apply(self) -> None:
        service = self._service()
        service._record_payload_observation = Mock()
        service._schedule_direct_plan_apply = Mock()
        service._schedule_next_collab_plan_apply = Mock()

        service._on_receive("0903", {"missionPlanID": 700000018})

        service._schedule_direct_plan_apply.assert_called_once_with(700000018)
        service._schedule_next_collab_plan_apply.assert_not_called()

    def test_0903_with_0803_latch_keeps_validated_transition_path(self) -> None:
        service = self._service()
        service._record_payload_observation = Mock()
        service._pending_next_collab_transition = {"token": 3}
        service._schedule_direct_plan_apply = Mock()
        service._schedule_next_collab_plan_apply = Mock()

        service._on_receive("0903", {"missionPlanID": 700000019})

        service._schedule_next_collab_plan_apply.assert_called_once_with(700000019)
        service._schedule_direct_plan_apply.assert_not_called()

    def test_direct_apply_loads_postattack_plan_and_preserves_state(self) -> None:
        service = self._service()
        sim = _FakeSim()
        service._sim_service = sim
        service._direct_plan_apply_seq = 1
        service._pending_direct_plan_apply = {
            "token": 1,
            "missionPlanID": 700000018,
            "applyInProgress": False,
        }

        with patch(
            "modules.sim.mission.mission_plan_loader.build_mission_plan_payload",
            return_value=_ready_plan_result(),
        ):
            service._apply_pending_direct_plan(1, 700000018, 0)

        self.assertEqual(sim._loaded_mission_plan_id, 700000018)
        self.assertIsNotNone(sim.loaded_payload)
        self.assertTrue(sim.loaded_payload["preserveState"])
        self.assertTrue(sim.loaded_payload["skipIfMissionPlanAlreadyLoaded"])
        self.assertIsNone(service._pending_direct_plan_apply)
        self.assertEqual(service._last_direct_plan_apply["status"], "applied")


if __name__ == "__main__":
    unittest.main()
