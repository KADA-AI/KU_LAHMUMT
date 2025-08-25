import sys
import os
from pathlib import Path
from typing import Callable, Any, Dict, Optional
from unittest.mock import patch

# ───────── 경로 부트스트랩 ─────────
def _bootstrap_paths():
    here = Path(__file__).resolve()
    monitoring_dir = here.parent
    modules_dir = monitoring_dir.parent
    root = modules_dir.parent
    common_dir = modules_dir / "common"
    for p in (monitoring_dir, common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try:
        os.chdir(root)
    except Exception:
        pass

_bootstrap_paths()

from modules.monitoring.monitoring_manager import MonitoringManager

# --- 가상 NodeMessenger (실제 환경에서는 nFusion의 것을 사용) ---
class MockNodeMessenger:
    def __init__(self, name="MockMessenger"):
        self.name = name
        self._receive_callback = None
        print(f"[{self.name}] Initialized.")

    def set_receive_callback(self, callback: Callable[[str, bytes], None]):
        self._receive_callback = callback

    def send(self, msg_id: str, data: Any) -> bytes:
        print(f"[{self.name}] SENDING MsgID: {msg_id}, Data: {data}")
        raw_data = f"Serialized: {msg_id} - {data}".encode("utf-8")
        return raw_data

    def simulate_receive(self, msg_id: str, raw_data: bytes):
        if self._receive_callback:
            print(f"[{self.name}] SIMULATE RECEIVE MsgID: {msg_id}")
            self._receive_callback(msg_id, raw_data)
        else:
            print(f"[{self.name}] No receive callback set for simulated receive.")

# We are patching 'push_message' in the context of where it is *used* (in monitoring_manager.py)
@patch('modules.monitoring.monitoring_manager.push_message')
def run_test(mock_push_message):
    print("--- Monitoring Module Standalone Test ---" )

    # 1. Create mock messenger
    mock_messenger = MockNodeMessenger("MonitoringTestMessenger")

    # 2. Create MonitoringManager with the mock messenger
    monitoring_manager = MonitoringManager(node_messenger=mock_messenger)

    # 3. Set the receive callback for simulation
    mock_messenger._receive_callback = monitoring_manager.handle_message_reception

    # 4. Simulate receiving a message
    print("\n--- Simulating message reception (0401) ---")
    mock_messenger.simulate_receive("0401", b'{"key": "value", "source": "test"}')

    # 5. Check data store
    from modules.monitoring.monitoring_data import monitoring_data_instance
    stored_data = monitoring_data_instance.get_data("0401")
    print("\n--- Verifying data store ---")
    print(f"Data for 0401: {stored_data}")
    assert stored_data is not None

    # 6. Trigger logic processing
    print("\n--- Triggering logic ---")
    monitoring_manager.trigger_logic()

    # 7. Check data store for processed results
    processed_result = monitoring_data_instance.get_data("last_processed_result")
    print("\n--- Verifying processed result in data store ---")
    print(f"Processed result: {processed_result}")
    assert processed_result is not None

    # 8. Test sending a message
    print("\n--- Testing message sending (0102) ---")
    test_payload = {"Status": 1, "SourceModuleName": "MonitoringTest"}
    monitoring_manager.send_message("0102", test_payload)

    # 9. Verify that our mock push_message was called correctly
    print("\n--- Verifying message sending ---")
    mock_push_message.assert_called_once()
    args, kwargs = mock_push_message.call_args
    print(f"push_message called with: msg_id={args[0]}, body_dict={kwargs.get('body_dict')}")
    assert args[0] == "0102"
    assert kwargs.get('body_dict') == test_payload

    print("\n--- Test Complete --- ")

if __name__ == "__main__":
    run_test()