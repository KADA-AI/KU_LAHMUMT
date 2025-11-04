"""Facade that wires together monitoring data stores, logic, and GUI callbacks."""

from __future__ import annotations

from datetime import datetime, timezone
from typing import Any, Callable, Optional

from .data.logic_storage import LogicStorage
from .data.message_models import ModuleStatusModelModel
from .data.push_storage import PushStorage
from .data.receive_storage import ReceiveStorage
from .logic.monitoring_logic import MonitoringLogicHandler
from modules.common.push import message0102_push
from modules.common.receive_center import register_listener


class MonitoringManager:
    """Mediates between background logic and UI callbacks."""

    def __init__(self, node_messenger, receive_messages_config):
        self.node_messenger = node_messenger
        self.receive_store = ReceiveStorage()
        self.logic_store = LogicStorage()
        self.push_store = PushStorage()

        self.logic_store.set_data("SystemMode", 0)

        self.log_callback: Optional[Callable[[str, str, str, Optional[bytes]], None]] = None
        self.gui_update_callback: Optional[Callable[[str, str, Any], None]] = None

        # Optionally hook message handlers per configuration.
        for item in receive_messages_config:
            msg_id = item[0] if isinstance(item, (tuple, list)) else item
            register_listener(msg_id, self._make_handler(msg_id))

        self.logic_handler = MonitoringLogicHandler(manager=self)
        self.logic_handler.start()

        self._log("MON_MGR", "INFO", "Monitoring manager started.")
        self.send_initial_status_message()

    def shutdown(self) -> None:
        self._log("MON_MGR", "INFO", "Shutting down manager...")
        self.logic_handler.stop()

    def _log(self, tag: str, level: str, message: str, raw: Optional[bytes] = None) -> None:
        if self.log_callback:
            self.log_callback(tag, level, message, raw)
        else:
            print(f"[{tag}] [{level}] {message}")

    def _make_handler(self, msg_id: str):
        def _handler(_msg_id: str, data_object: Any):
            self.handle_message_reception(msg_id, data_object)

        return _handler

    def handle_message_reception(self, msg_id: str, data_object: Any) -> None:
        self._log("MON_MGR", "RECV", f"parsed object received: {msg_id}")
        self.receive_store.set_data(msg_id, data_object)

        try:
            self.logic_handler.monitoring_logic.handle_message(msg_id, data_object)
        except AttributeError:
            pass
        except Exception as exc:
            self._log("MON_MGR", "ERROR", f"logic message hook failed: {exc}")

        if msg_id == "0101" and hasattr(data_object, "systemMode"):
            self.set_system_mode(getattr(data_object, "systemMode"))
        elif msg_id == "0103" and hasattr(data_object, "mode"):
            self.set_system_mode(getattr(data_object, "mode"))

        if self.gui_update_callback:
            self.gui_update_callback("receive", msg_id, data_object)

    def get_received_data(self, msg_id: str) -> Any:
        return self.receive_store.get_data(msg_id)

    def get_logic_result(self, key: str) -> Any:
        return self.logic_store.get_data(key)

    def set_system_mode(self, mode: int) -> None:
        self._log("MON_MGR", "MODE_CHANGE", f"system mode set to '{mode}'")
        self.logic_store.set_data("SystemMode", mode)
        try:
            self.logic_handler.monitoring_logic.on_system_mode_changed(mode)
        except Exception:
            pass

        if self.gui_update_callback:
            self.gui_update_callback("logic", "SystemMode", None)

    def send_initial_status_message(self) -> None:
        self._log("MON_MGR", "INFO", "sending initial status message (0102)")
        epoch_ms = int((datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)).total_seconds() * 1000)
        body = ModuleStatusModelModel(timestamp=epoch_ms, source="MSM", status=1)
        try:
            message0102_push.make_and_push(body, self.node_messenger)
            self.push_store.add_data("0102", body)
        except Exception as exc:
            self._log("MON_MGR", "WARN", f"initial 0102 push failed: {exc}")

    # Backwards compatibility placeholders ----------------------------------
    def trigger_logic(self) -> None:
        pass

    def send_message(self, msg_id: str, on_done: Optional[Callable] = None) -> None:
        pass
