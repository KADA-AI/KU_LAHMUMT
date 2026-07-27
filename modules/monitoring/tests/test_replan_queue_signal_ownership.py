from __future__ import annotations

from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager


def _attack_close_payload() -> dict[str, object]:
    return {
        "replanLevel": 2,
        "replanRequest": "target 7 destroyed return replan",
        "missionPlanIDList": [{"missionPlanID": 700000008}],
        "replanDetail": {
            "trigger": "0402",
            "triggerType": "attackClosedDestroyed",
            "targetID": 7,
        },
    }


def _target_detection_payload() -> dict[str, object]:
    return {
        "replanLevel": 2,
        "replanRequest": "target 8 detected replan",
        "pendingOptionList": [
            {"missionPlanID": 700000009},
            {"missionPlanID": 700000010},
        ],
        "replanDetail": {
            "trigger": "0402",
            "targetID": 8,
            "targetType": 3,
            "watcherID": 5,
        },
    }


def _manager(clock: list[int]) -> ReplanQueueManager:
    return ReplanQueueManager(
        now_fn=lambda: clock[0],
        settings_getter=lambda: {
            "replan_queue": {"target_dispatch_delay_ms": 0},
            "target_detection": {},
        },
    )


def test_trailing_0001_does_not_complete_newly_promoted_request() -> None:
    clock = [1000]
    manager = _manager(clock)

    first = manager.enqueue(
        payload=_attack_close_payload(),
        source_tag="target_detection",
        now_ms=clock[0],
    )
    assert first["dispatch"]["queue_id"] == 1
    manager.confirm_dispatch(queue_id=1, success=True, now_ms=1001)

    queued = manager.enqueue(
        payload=_target_detection_payload(),
        source_tag="target_detection",
        now_ms=1002,
    )
    assert queued["snapshot"]["queued"][0]["queue_id"] == 2

    completed = manager.handle_signal(
        signal_name="0305",
        payload={"missionPlanningStatus": 2, "replanReason": "replan_not_needed"},
        now_ms=1003,
    )
    assert completed["dispatch"]["queue_id"] == 2
    assert completed["snapshot"]["active"]["status"] == "dispatching"
    assert completed["snapshot"]["active"]["dispatched_ms"] is None

    clock[0] = 1004
    should_dispatch, logs = manager.handle_0001(
        {"source": "MMR", "contents": "replan_not_needed"}
    )

    assert should_dispatch is False
    assert any("stale 0001 ignored before dispatch" in line for line in logs)
    snapshot = manager.snapshot(now_ms=clock[0])
    assert snapshot["active"]["queue_id"] == 2
    assert snapshot["active"]["status"] == "dispatching"
    assert [item["queue_id"] for item in snapshot["history"]] == [1]

    sent = manager.confirm_dispatch(queue_id=2, success=True, now_ms=1005)
    assert sent["snapshot"]["active"]["status"] == "active"


def test_0001_can_complete_request_after_its_0902_was_sent() -> None:
    clock = [2000]
    manager = _manager(clock)
    first = manager.enqueue(
        payload=_target_detection_payload(),
        source_tag="target_detection",
        now_ms=clock[0],
    )
    manager.confirm_dispatch(
        queue_id=first["dispatch"]["queue_id"],
        success=True,
        now_ms=2001,
    )

    clock[0] = 2002
    manager.handle_0001({"source": "MMR", "contents": "replan_not_needed"})

    snapshot = manager.snapshot(now_ms=clock[0])
    assert snapshot["active"] is None
    assert snapshot["history"][0]["queue_id"] == 1
    assert snapshot["history"][0]["completion_signal"] == "0001"


def test_target_detection_option_blocks_completion_until_keep_decision() -> None:
    clock = [3000]
    manager = _manager(clock)
    queued = manager.enqueue(
        payload=_target_detection_payload(),
        source_tag="target_detection",
        now_ms=clock[0],
    )
    manager.confirm_dispatch(
        queue_id=queued["dispatch"]["queue_id"],
        success=True,
        now_ms=3001,
    )

    blocker = manager.find_target_detection_option_decision_blocker()
    assert blocker is not None
    assert blocker["queue_id"] == 1

    manager.handle_0702({"ignore": 1})

    assert manager.find_target_detection_option_decision_blocker() is None


def test_attack_close_replan_does_not_block_completion_recommendation() -> None:
    clock = [4000]
    manager = _manager(clock)
    manager.enqueue(
        payload=_attack_close_payload(),
        source_tag="target_detection",
        now_ms=clock[0],
    )

    assert manager.find_target_detection_option_decision_blocker() is None


def test_target_detection_option_blocker_clears_after_new_plan_selection() -> None:
    clock = [5000]
    manager = _manager(clock)
    queued = manager.enqueue(
        payload=_target_detection_payload(),
        source_tag="target_detection",
        now_ms=clock[0],
    )
    manager.confirm_dispatch(
        queue_id=queued["dispatch"]["queue_id"],
        success=True,
        now_ms=5001,
    )

    assert manager.find_target_detection_option_decision_blocker() is not None

    manager.handle_0702({"ignore": 2, "missionPlanID": 700000009})

    assert manager.find_target_detection_option_decision_blocker() is None
