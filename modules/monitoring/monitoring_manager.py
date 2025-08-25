from datetime import datetime
from typing import Callable, Any, Optional

from .monitoring_data import monitoring_data_instance
from .monitoring_logic import MonitoringLogicHandler
from modules.common.receive_center import register_listener
from modules.common.push_center import push_message

class MonitoringManager:
    def __init__(self, node_messenger: Any, log_callback: Optional[Callable[[str, str, str, Optional[bytes]], None]] = None):
        self.node_messenger = node_messenger
        self.log_callback = log_callback
        self.data_store = monitoring_data_instance
        self.logic_handler = MonitoringLogicHandler()

        # Register to receive messages
        from .monitoring_config import RECEIVE_MESSAGES
        for msg_id, _ in RECEIVE_MESSAGES:
            register_listener(msg_id, self.handle_message_reception)

        self._log("MON_MGR", "INFO", "Monitoring Manager initialized.")

    def _log(self, tag: str, log_type: str, message: str, raw_data: Optional[bytes] = None):
        print(f"[{datetime.now().strftime('%H:%M:%S')}] {tag:<8} : {log_type:<4} : {message}")
        if raw_data:
            print(f"  Raw: {raw_data[:100]}{'...' if len(raw_data) > 100 else ''}")
        if self.log_callback:
            self.log_callback(tag, log_type, message, raw_data)

    def handle_message_reception(self, msg_id: str, raw: bytes):
        self._log("MON_MGR", "RECV", f"Message received: {msg_id}", raw)
        # In a real scenario, you would decode the raw bytes into a meaningful object
        # For now, we'll just store the raw bytes
        self.data_store.set_data(msg_id, {"raw": raw, "timestamp": datetime.now()})
        self._log("MON_MGR", "STORE", f"Data for {msg_id} updated in data store.")

        # Optionally, trigger logic right after receiving a message
        self.trigger_logic()

    def trigger_logic(self):
        self._log("MON_MGR", "CMD", "Triggering monitoring logic.")
        result = self.logic_handler.process_data()
        self._log("MON_MGR", "LOGIC_RESULT", f"Logic processing result: {result}")
        return result

    def send_message(self, msg_id: str, body_dict: Optional[dict] = None):
        self._log("MON_MGR", "SEND_TRY", f"Attempting to send message: {msg_id}")
        push_message(
            msg_id,
            self.node_messenger,
            on_done=self._on_send_done,
            body_dict=body_dict
        )

    def _on_send_done(self, msg_id: str, raw: Optional[bytes]):
        if raw:
            self._log("MON_MGR", "SEND_OK", f"Message sent successfully: {msg_id}", raw)
        else:
            self._log("MON_MGR", "SEND_FAIL", f"Failed to send message: {msg_id}")
