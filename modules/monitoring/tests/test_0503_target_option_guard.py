from __future__ import annotations

import sys
from types import MethodType, ModuleType, SimpleNamespace

from modules.monitoring.logic.replan_queue_manager import ReplanQueueManager
from modules.monitoring.monitoring_gui import MainWindow


def _target_detection_payload() -> dict[str, object]:
    return {
        "replanLevel": 2,
        "replanRequest": "target 1001 detected replan",
        "pendingOptionList": [
            {"missionPlanID": 700000007},
            {"missionPlanID": 700000008},
        ],
        "replanDetail": {
            "trigger": "0402",
            "targetID": 1001,
            "targetType": 1,
            "watcherID": 5,
        },
    }


class _Visualization:
    def __init__(self) -> None:
        self.validated: list[tuple[int, int | None]] = []
        self.sent: list[tuple[int, int | None]] = []
        self.failed: list[tuple[int, int | None]] = []

    def validate_completion_recommendation(
        self,
        recommend: int,
        input_id: int | None,
    ) -> tuple[bool, str]:
        self.validated.append((int(recommend), input_id))
        return True, "monitored completion"

    def note_completion_recommendation_sent(
        self,
        recommend: int,
        input_id: int | None,
    ) -> None:
        self.sent.append((int(recommend), input_id))

    def note_completion_recommendation_failed(
        self,
        recommend: int,
        input_id: int | None,
    ) -> None:
        self.failed.append((int(recommend), input_id))


def _host(manager: ReplanQueueManager, visualization: _Visualization) -> SimpleNamespace:
    logs: list[str] = []
    host = SimpleNamespace(
        _replan_queue_manager=manager,
        _viz_tab=visualization,
        _tab=None,
        _append_log_line=logs.append,
        _append_throttled_log_line=lambda _key, text, **_kwargs: logs.append(text),
        _find_tx_row=lambda _message_id: -1,
        _send_0502=lambda: None,
        logs=logs,
    )
    host._target_detection_option_blocker_for_0503 = MethodType(
        MainWindow._target_detection_option_blocker_for_0503,
        host,
    )
    return host


def test_0503_is_deferred_without_marking_failure_while_target_option_is_pending(
    monkeypatch,
) -> None:
    clock = [1000]
    manager = ReplanQueueManager(
        now_fn=lambda: clock[0],
        settings_getter=lambda: {
            "replan_queue": {"target_dispatch_delay_ms": 0},
            "target_detection": {},
        },
    )
    manager.enqueue(
        payload=_target_detection_payload(),
        source_tag="target_detection",
        now_ms=clock[0],
    )
    visualization = _Visualization()
    host = _host(manager, visualization)
    pushes: list[tuple[str, dict[str, object]]] = []
    push_center = ModuleType("push_center")
    push_center.push_message = lambda message_id, _messenger, *, body_dict, on_done=None: (
        pushes.append((message_id, body_dict)) or True
    )
    monkeypatch.setitem(sys.modules, "push_center", push_center)

    sent = MainWindow._send_0503(host, 1, 70000008)

    assert sent is False
    assert pushes == []
    assert visualization.validated == []
    assert visualization.sent == []
    assert visualization.failed == []
    assert any("target option decision pending" in line for line in host.logs)


def test_keep_current_decision_releases_deferred_0503_on_next_attempt(monkeypatch) -> None:
    clock = [2000]
    manager = ReplanQueueManager(
        now_fn=lambda: clock[0],
        settings_getter=lambda: {
            "replan_queue": {"target_dispatch_delay_ms": 0},
            "target_detection": {},
        },
    )
    manager.enqueue(
        payload=_target_detection_payload(),
        source_tag="target_detection",
        now_ms=clock[0],
    )
    visualization = _Visualization()
    host = _host(manager, visualization)
    pushes: list[tuple[str, dict[str, object]]] = []
    push_center = ModuleType("push_center")
    push_center.push_message = lambda message_id, _messenger, *, body_dict, on_done=None: (
        pushes.append((message_id, body_dict)) or True
    )
    monkeypatch.setitem(sys.modules, "push_center", push_center)

    assert MainWindow._send_0503(host, 1, 70000008) is False
    manager.handle_0702({"ignore": 1})
    assert MainWindow._send_0503(host, 1, 70000008) is True

    assert [message_id for message_id, _payload in pushes] == ["0503"]
    assert visualization.validated == [(1, 70000008)]
    assert visualization.sent == [(1, 70000008)]
    assert visualization.failed == []
