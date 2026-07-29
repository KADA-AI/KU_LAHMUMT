from __future__ import annotations

import json
import threading

from modules.monitoring.logic import target_detection_replan as target_detection
from modules.monitoring.monitoring_gui import MainWindow


def _destroyed_target(target_id: int, watcher_id: int) -> dict:
    return {
        "targetID": int(target_id),
        "watcherID": int(watcher_id),
        "targetType": 2,
        "coordinate": {
            "latitude": 38.0,
            "longitude": 127.0,
            "altitude": 500,
        },
        "isDestroyed": True,
        "targetInFrame": False,
        "lastUpdated": 1000,
    }


def test_post_attack_close_recovers_stale_tracking_lineage_from_current_plan(
    monkeypatch,
) -> None:
    assignment = {
        "aircraft_id": 4,
        "active": True,
        "target_id": 9,
        "source_plan_id": 700000010,
        "attack_plan_id": 700000013,
        "current_input_mission_id": 3,
    }

    def _load_db_json(folder: str, file_id: int):
        if folder == "MissionPlan" and int(file_id) == 700000016:
            return {
                "aircraftList": [
                    {
                        "aircraftID": 4,
                        "individualMissionPackageID": 800000105,
                    }
                ]
            }
        if folder == "IndividualMissionPlan" and int(file_id) == 800000105:
            return {
                "individualMissionList": [
                    {
                        "individualMissionID": 900000480,
                        "isDone": False,
                        "relatedMission": {"inputMissionID": 3, "targetID": 9},
                        "individualMissionInfo": {
                            "individualMissionType": 3,
                            "targetID": 9,
                        },
                        "pathID": 400000087,
                    }
                ]
            }
        return {}

    monkeypatch.setattr(target_detection, "load_db_json", _load_db_json)
    monkeypatch.setattr(
        target_detection,
        "list_active_tracking_assignments",
        lambda: [assignment],
    )
    monkeypatch.setattr(
        target_detection,
        "resolve_plan_lineage_ids",
        lambda _plan_id: {700000016},
    )
    monkeypatch.setattr(
        target_detection,
        "allocate_mission_plan_ids",
        lambda _count: [700000018],
    )
    monkeypatch.setattr(
        target_detection,
        "_post_attack_rejoin_enabled",
        lambda: True,
    )
    monkeypatch.setattr(
        target_detection,
        "_post_attack_rejoin_config",
        lambda: {"closure_cooldown_ms": 30_000},
    )

    coordinator = target_detection.TargetDetectionCoordinator(now_fn=lambda: 2000)
    logs: list[str] = []
    payloads = coordinator._build_attack_close_payloads(
        message=None,
        target_info={"targetList": {"9-4": _destroyed_target(9, 4)}},
        current_mission_plan_id=700000016,
        mission_ids=[3, 4],
        package_id=1,
        now_ts=2000,
        logs=logs,
    )

    assert len(payloads) == 1
    detail = payloads[0]["replanDetail"]
    assert detail["targetID"] == 9
    assert detail["trackingAircraftIDList"] == [4]
    assert detail["trackingLineageRecovered"] is True
    assert detail["sourceMissionPlanID"] == 700000016
    assert any("lineage recovered" in line for line in logs)


def test_stale_tracking_lineage_without_current_plan_artifact_stays_rejected(
    monkeypatch,
) -> None:
    assignment = {
        "aircraft_id": 4,
        "active": True,
        "target_id": 9,
        "source_plan_id": 700000010,
        "attack_plan_id": 700000013,
    }
    monkeypatch.setattr(
        target_detection,
        "load_db_json",
        lambda folder, _file_id: {
            "aircraftList": [
                {"aircraftID": 4, "individualMissionPackageID": 800000105}
            ]
        }
        if folder == "MissionPlan"
        else {"individualMissionList": []},
    )
    monkeypatch.setattr(
        target_detection,
        "list_active_tracking_assignments",
        lambda: [assignment],
    )
    monkeypatch.setattr(
        target_detection,
        "resolve_plan_lineage_ids",
        lambda _plan_id: {700000016},
    )
    monkeypatch.setattr(
        target_detection,
        "_post_attack_rejoin_enabled",
        lambda: True,
    )

    coordinator = target_detection.TargetDetectionCoordinator(now_fn=lambda: 2000)
    payloads = coordinator._build_attack_close_payloads(
        message={"targetList": [_destroyed_target(9, 4)]},
        target_info=None,
        current_mission_plan_id=700000016,
        mission_ids=[3],
        package_id=1,
        now_ts=2000,
        logs=[],
    )

    assert payloads == []


def test_0402_dispatch_preserves_new_destroyed_transitions_ahead_of_latest_sample() -> None:
    processed: list[object] = []

    class _Window:
        _0402_pending_lock = threading.Lock()
        _0402_pending_payload = None
        _0402_pending_terminal_payloads: list[object] = []
        _0402_destroyed_target_ids_seen: set[int] = set()
        _0402_pending_scheduled = False
        _0402_last_signature = None

        @staticmethod
        def _payload_signature(payload):
            return json.dumps(payload, sort_keys=True).encode("utf-8")

        @staticmethod
        def _record_rx_enqueue_event(*_args, **_kwargs):
            return None

        @staticmethod
        def _invoke_on_ui_thread(_callback):
            return None

        @staticmethod
        def _schedule_0402_drain():
            return None

        @staticmethod
        def _on_rx_0402(payload):
            processed.append(payload)

    window = _Window()
    terminal_9 = {"targetList": [_destroyed_target(9, 4)]}
    ordinary_10 = {
        "targetList": [
            {
                **_destroyed_target(10, 5),
                "isDestroyed": False,
                "targetInFrame": True,
            }
        ]
    }
    terminal_11 = {"targetList": [_destroyed_target(11, 6)]}
    latest_ordinary = {
        "targetList": [
            {
                **_destroyed_target(12, 5),
                "isDestroyed": False,
                "targetInFrame": True,
            }
        ]
    }

    MainWindow._enqueue_0402_payload(window, terminal_9)
    MainWindow._enqueue_0402_payload(window, ordinary_10)
    MainWindow._enqueue_0402_payload(window, terminal_11)
    MainWindow._enqueue_0402_payload(window, latest_ordinary)

    MainWindow._drain_0402_payload(window)
    MainWindow._drain_0402_payload(window)
    MainWindow._drain_0402_payload(window)

    assert processed == [terminal_9, terminal_11, latest_ordinary]
