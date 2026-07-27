from __future__ import annotations

from types import SimpleNamespace

from modules.monitoring.monitoring_gui import MainWindow


class _ReexecuteCoordinator:
    def __init__(self) -> None:
        self.calls: list[int] = []

    def on_execute(self, execute: int) -> list[str]:
        self.calls.append(int(execute))
        return []


class _ProgressTracker:
    @staticmethod
    def get_active_input_id() -> int:
        return 1


class _Visualization:
    def __init__(self) -> None:
        self._progress_tracker = _ProgressTracker()
        self.calls: list[int] = []

    def handle_execute_command(self, *, execute: int) -> None:
        self.calls.append(int(execute))


def test_duplicate_execute_2_with_different_raw_representation_is_handled_once() -> None:
    coordinator = _ReexecuteCoordinator()
    visualization = _Visualization()
    area_resets: list[int] = []
    line_resets: list[int] = []
    logs: list[str] = []
    host = SimpleNamespace(
        _next_collab_replan_trigger_enabled=False,
        _reexecute_coord=coordinator,
        _viz_tab=visualization,
        _queue_area_snapshot_input_reset=lambda input_id: area_resets.append(int(input_id)),
        _line_scan_reset_input_coverage=lambda input_id: line_resets.append(int(input_id)),
        _append_log_line=logs.append,
    )

    first_delivery = b'{"timestamp":837919604398,"source":"DSC","execute":2}'
    same_event_from_other_ingress = {
        "execute": 2,
        "source": "DSC",
        "timestamp": 837919604398,
    }

    MainWindow._on_rx_0803(host, first_delivery)
    MainWindow._on_rx_0803(host, same_event_from_other_ingress)

    assert coordinator.calls == [2]
    assert visualization.calls == [2]
    assert area_resets == [1]
    assert line_resets == [1]
    assert "[0803] duplicate execute=2 ignored" in logs


def test_new_execute_2_timestamp_is_not_suppressed() -> None:
    coordinator = _ReexecuteCoordinator()
    visualization = _Visualization()
    host = SimpleNamespace(
        _next_collab_replan_trigger_enabled=False,
        _reexecute_coord=coordinator,
        _viz_tab=visualization,
        _queue_area_snapshot_input_reset=lambda _input_id: None,
        _line_scan_reset_input_coverage=lambda _input_id: None,
        _append_log_line=lambda _text: None,
    )

    MainWindow._on_rx_0803(
        host,
        {"timestamp": 837919604398, "source": "DSC", "execute": 2},
    )
    MainWindow._on_rx_0803(
        host,
        {"timestamp": 837919858888, "source": "DSC", "execute": 2},
    )

    assert coordinator.calls == [2, 2]
    assert visualization.calls == [2, 2]
