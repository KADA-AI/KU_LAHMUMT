from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

from modules.monitoring.logic.prior_mission_replan import PriorMissionReplanCoordinator
from modules.monitoring.monitoring_gui import MainWindow


def test_target_prior_mission_preserves_target_id_from_0202_to_0902() -> None:
    coordinator = PriorMissionReplanCoordinator(now_fn=lambda: 123456)

    with (
        patch.object(coordinator, "_allocate_plan_id", return_value=700000123),
        patch.object(coordinator, "_persist_detail") as persist_detail,
        patch(
            "modules.monitoring.logic.prior_mission_replan.prior_target_rediscovery_store.arm_target_rediscovery",
            return_value=None,
        ),
    ):
        payloads, _logs = coordinator.on_prior_mission(
            {
                "timestamp": 123450,
                "source": "DSC",
                "priorMissionList": [
                    {
                        "priorMissionID": 31,
                        "missionType": 2,
                        "targetOrientation": {"targetID": 157},
                    }
                ],
            },
            system_mode=3,
            current_mission_plan_id=700000010,
        )

    assert len(payloads) == 1
    payload = payloads[0]
    assert payload["priorMissionList"][0]["targetOrientation"] == {"targetID": 157}
    assert payload["replanDetail"]["targetID"] == 157
    assert payload["replanDetail"]["targetOrientation"] == {"targetID": 157}
    persist_detail.assert_called_once()
    assert persist_detail.call_args.args[1]["targetID"] == 157


def test_target_prior_mission_rejects_zero_target_id() -> None:
    coordinator = PriorMissionReplanCoordinator(now_fn=lambda: 123456)

    payloads, logs = coordinator.on_prior_mission(
        {
            "timestamp": 123450,
            "source": "DSC",
            "priorMissionList": [
                {
                    "priorMissionID": 31,
                    "missionType": 2,
                    "targetOrientation": {"targetID": 0},
                }
            ],
        },
        system_mode=3,
        current_mission_plan_id=700000010,
    )

    assert payloads == []
    assert any("requires targetOrientation.targetID" in message for message in logs)


def test_queued_prior_mission_rebases_to_latest_applied_plan_before_dispatch() -> None:
    logs: list[str] = []
    window = SimpleNamespace(
        _extract_positive_int=MainWindow._extract_positive_int,
        _current_dispatch_plan_id=lambda: 700000011,
        _append_log_line=logs.append,
    )
    payload = {
        "replanLevel": 4,
        "sourceMissionPlanID": 700000010,
        "pendingOptionList": [{"missionPlanID": 700000123}],
        "priorMissionList": [{"priorMissionID": 32, "missionType": 1}],
        "replanDetail": {
            "sourceMissionPlanID": 700000010,
            "priorMissionID": 32,
            "missionType": 1,
        },
    }

    with patch(
        "modules.monitoring.monitoring_gui.prior_replan_store.save_detail"
    ) as save_detail:
        prepared, _target_ids = MainWindow._prepare_0902_payload_for_dispatch(window, payload)

    assert prepared["sourceMissionPlanID"] == 700000011
    assert prepared["currentMissionPlanID"] == 700000011
    assert prepared["replanDetail"]["sourceMissionPlanID"] == 700000011
    assert prepared["replanDetail"]["currentMissionPlanID"] == 700000011
    save_detail.assert_called_once()
    assert save_detail.call_args.args[0] == 700000123
    assert save_detail.call_args.args[1]["sourceMissionPlanID"] == 700000011
    assert any("prior-mission replan sourcePlan rebound" in message for message in logs)
