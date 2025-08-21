# c:\Users\HJW\Documents\Dev\MUMT\nFusion\mission_monitoring_replan\mission_monitoring_replan_csc_manager.py
from datetime import datetime
import time  # 테스트용 time 모듈 임포트
from typing import Callable, Any, Dict, Optional

# Use try/except for imports to allow direct execution or package import
try:
    # Relative imports (when run as part of the package)
    from .monitoring_csu.monitoring_csu_logic import MonitoringCSUHandler
    from .replan_csu.replan_csu_logic import ReplanCSUHandler
    from .mission_monitoring_replan_csc_config import PUSH_MESSAGES, RECEIVE_MESSAGES
except ImportError:
    # Absolute imports (when run directly)
    from mission_monitoring_replan.monitoring_csu.monitoring_csu_logic import (
        MonitoringCSUHandler,
    )
    from mission_monitoring_replan.replan_csu.replan_csu_logic import ReplanCSUHandler
    from mission_monitoring_replan.mission_monitoring_replan_csc_config import (
        PUSH_MESSAGES,
        RECEIVE_MESSAGES,
    )
# NodeMessenger 및 push_center는 이 클래스가 직접 사용합니다.
# 실제 프로젝트에서는 nFusion 관련 import가 필요합니다.
# from dll_files.nFusionImports import NodeMessenger (가정)
# from push_center import push_message (가정)


# --- 가상 NodeMessenger 및 push_center (실제 환경에서는 nFusion의 것을 사용) ---
class MockNodeMessenger:
    def __init__(self, name="MockMessenger"):
        self.name = name
        self._receive_callback = None
        print(f"[{self.name}] Initialized.")

    def set_receive_callback(self, callback: Callable[[str, bytes], None]):
        self._receive_callback = callback

    def send(self, msg_id: str, data: Any) -> bytes:
        print(f"[{self.name}] SENDING MsgID: {msg_id}, Data: {data}")
        # 실제로는 메시지 객체를 생성하고 직렬화해야 합니다.
        raw_data = f"Serialized: {msg_id} - {data}".encode("utf-8")
        return raw_data

    def simulate_receive(self, msg_id: str, raw_data: bytes):
        """테스트 목적으로 외부에서 메시지 수신을 시뮬레이션합니다."""
        if self._receive_callback:
            print(f"[{self.name}] SIMULATE RECEIVE MsgID: {msg_id}")
            self._receive_callback(msg_id, raw_data)
        else:
            print(f"[{self.name}] No receive callback set for simulated receive.")


def mock_push_message(
    msg_id: str,
    node_messenger: MockNodeMessenger,
    *,
    on_done: Optional[Callable[[str, bytes | None, Optional[Exception]], None]] = None,
    payload: Optional[Dict] = None,
) -> bool:
    print(f"[MockPushCenter] Attempting to push {msg_id} with payload: {payload}")
    try:
        # 실제 push 모듈은 payload를 사용하여 메시지 내용을 구성합니다.
        # 여기서는 간단히 payload를 문자열로 만들어 raw 데이터로 사용합니다.
        raw_bytes = node_messenger.send(
            msg_id, payload if payload else {"default_content": "random_data"}
        )
        if on_done:
            on_done(msg_id, raw_bytes, None)
        return True
    except Exception as e:
        print(f"[MockPushCenter] Error pushing {msg_id}: {e}")
        if on_done:
            on_done(msg_id, None, e)
        return False


# --- 가상 구현 끝 ---


class MissionMonitoringReplanCSCManager:
    def __init__(
        self,
        node_messenger: Any,
        log_callback: Optional[Callable[[str, str, str, Optional[bytes]], None]] = None,
    ):
        """
        node_messenger: 실제 nFusion의 NodeMessenger 인스턴스
        log_callback: GUI 또는 다른 로거로 로그를 전달하기 위한 콜백 (tag, type, message, raw_data)
        """
        self.node_messenger = node_messenger  # 실제 NodeMessenger 주입
        # self.node_messenger.set_receive_callback(self._handle_message_reception) # 실제 nFusion 연동 시 필요

        self.monitoring_csu_handler = MonitoringCSUHandler()
        self.replan_csu_handler = ReplanCSUHandler()
        self.log_callback = log_callback

        self.csc_data_store = {
            "latest_status_0401": None,
            "latest_situation_0402": None,
            "latest_progress_0501": None,
            "replan_trigger_0902": None,
            "monitoring_analysis_results": None,
            "other_received_data": {},
        }
        self._log("CSC_MGR", "INFO", "CSC Manager 초기화됨.")

    def _log(
        self, tag: str, log_type: str, message: str, raw_data: Optional[bytes] = None
    ):
        print(
            f"[{datetime.now().strftime('%H:%M:%S')}] {tag:<8} : {log_type:<4} : {message}"
        )
        if raw_data:
            print(f"  Raw: {raw_data[:100]}{'...' if len(raw_data) > 100 else ''}")
        if self.log_callback:
            self.log_callback(tag, log_type, message, raw_data)

    def _handle_message_reception(self, msg_id: str, raw: bytes):
        """nFusion NodeMessenger로부터 메시지를 수신했을 때 호출될 콜백"""
        self._log("CSC_MGR", "RECV", f"메시지 수신: {msg_id}", raw)
        decoded_data = {
            "raw": raw,
            "timestamp": datetime.now().isoformat(),
            "content": f"decoded_{msg_id}",
        }  # 실제 디코딩 필요

        if msg_id == "0401":
            self.csc_data_store["latest_status_0401"] = decoded_data
        elif msg_id == "0402":
            self.csc_data_store["latest_situation_0402"] = decoded_data
        elif msg_id == "0501":
            self.csc_data_store["latest_progress_0501"] = decoded_data
        elif msg_id == "0902":  # 외부로부터의 재계획 요청
            self.csc_data_store["replan_trigger_0902"] = decoded_data
            self.trigger_replan_csu_logic()  # 예: 수신 시 바로 재계획 실행
        else:
            if msg_id not in self.csc_data_store["other_received_data"]:
                self.csc_data_store["other_received_data"][msg_id] = []
            self.csc_data_store["other_received_data"][msg_id].append(decoded_data)
        self._log("CSC_MGR", "STORE", f"{msg_id} 데이터 저장소 업데이트 완료.", None)

    def trigger_monitoring_csu_logic(self) -> Dict:
        self._log("CSC_MGR", "CMD", "모니터링 CSU 로직 실행 요청")
        data_for_monitoring_csu = {
            "status_info": self.csc_data_store.get("latest_status_0401"),
            "situation_info": self.csc_data_store.get("latest_situation_0402"),
            "progress_info": self.csc_data_store.get("latest_progress_0501"),
        }
        result = self.monitoring_csu_handler.run_monitoring(data_for_monitoring_csu)
        self.csc_data_store["monitoring_analysis_results"] = result
        self._log("CSC_MGR", "MON_RES", f"모니터링 CSU 결과: {result}")
        if result.get("anomalies_detected"):
            self._log("CSC_MGR", "INFO", "이상 상황 감지. 재계획 CSU 실행 고려.")
            # self.trigger_replan_csu_logic() # 필요시 자동 실행
        return result

    def trigger_replan_csu_logic(self) -> Dict:
        self._log("CSC_MGR", "CMD", "재계획 CSU 로직 실행 요청")
        data_for_replan_csu = {
            "trigger_info": self.csc_data_store.get("replan_trigger_0902"),
            "monitoring_analysis": self.csc_data_store.get(
                "monitoring_analysis_results"
            ),
        }
        result = self.replan_csu_handler.run_replan(data_for_replan_csu)
        self._log("CSC_MGR", "RPLN_RES", f"재계획 CSU 결과: {result}")

        if result.get("status") == "calculated":
            self._log(
                "CSC_MGR",
                "INFO",
                f"재계획 결과({result.get('plan_id')}) 생성됨. 메시지(0902) 발신 시도.",
            )
            # 재계획 결과를 기반으로 메시지 페이로드 구성
            payload_for_0902 = {
                "replan_id": result.get("plan_id"),
                "actions": result.get("proposed_actions"),
                # 기타 필요한 정보
            }
            self.send_message("0902", payload_for_0902)
        return result

    def send_message(self, msg_id: str, payload: Optional[Dict] = None):
        """지정된 msg_id와 payload로 메시지를 발신합니다."""
        self._log(
            "CSC_MGR", "SEND_TRY", f"메시지 발신 시도: {msg_id}, 페이로드: {payload}"
        )

        # 실제 push_center.push_message 사용
        # push_message(
        #     msg_id,
        #     self.node_messenger,
        #     on_done=self._on_send_done,
        #     payload=payload # push_message가 payload를 받을 수 있도록 수정 필요 또는 여기서 메시지 객체 생성
        # )
        # --- 가상 push_message 사용 ---
        mock_push_message(
            msg_id,
            self.node_messenger,  # MockNodeMessenger 전달
            on_done=self._on_send_done,
            payload=payload,
        )

    def _on_send_done(
        self, msg_id: str, raw: Optional[bytes], error: Optional[Exception]
    ):
        if error:
            self._log(
                "CSC_MGR",
                "SEND_FAIL",
                f"메시지 발신 실패: {msg_id}, 오류: {error}",
                raw,
            )
        else:
            self._log("CSC_MGR", "SEND_OK", f"메시지 발신 완료: {msg_id}", raw)

    def get_data_store_snapshot(self) -> Dict:
        """GUI 등에서 현재 데이터 저장소 상태를 조회하기 위한 메서드"""
        return self.csc_data_store.copy()

    def get_push_messages(self) -> tuple:
        return PUSH_MESSAGES

    def get_receive_messages(self) -> tuple:
        return RECEIVE_MESSAGES


if __name__ == "__main__":
    # Add the project root (nFusion directory) to sys.path
    # This is necessary for absolute imports (e.g., mission_monitoring_replan....)
    # to work when this file is run directly from its own directory.
    import os
    import sys

    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sys.path.insert(0, project_root)  # Insert at the beginning to prioritize

    print("MissionMonitoringReplanCSCManager 직접 실행 테스트 시작...")

    # 1. 테스트용 MockNodeMessenger 인스턴스 생성
    mock_messenger = MockNodeMessenger("DirectRunNM")

    # 2. 간단한 콘솔 로그 콜백 정의
    def direct_run_log_callback(tag: str, log_type: str, message: str, raw_data=None):
        log_line = f"[DIRECT_RUN_LOG] TAG={tag}, TYPE={log_type}, MSG={message}"
        if raw_data:
            raw_str = raw_data.decode("utf-8", errors="ignore")
            log_line += f", RAW='{raw_str[:50]}{'...' if len(raw_str) > 50 else ''}'"
        print(log_line)

    # 3. CSCManager 인스턴스 생성
    csc_manager_instance = MissionMonitoringReplanCSCManager(
        node_messenger=mock_messenger, log_callback=direct_run_log_callback
    )

    # 4. MockNodeMessenger에 수신 콜백 연결 (simulate_receive 테스트용)
    mock_messenger.set_receive_callback(csc_manager_instance._handle_message_reception)

    print("\n--- 초기 데이터 저장소 상태 ---")
    print(csc_manager_instance.get_data_store_snapshot())

    print("\n--- 모니터링 CSU 로직 실행 ---")
    mon_result = csc_manager_instance.trigger_monitoring_csu_logic()
    print(f"모니터링 결과: {mon_result}")
    print(
        f"모니터링 후 데이터 저장소: {csc_manager_instance.get_data_store_snapshot().get('monitoring_analysis_results')}"
    )

    time.sleep(0.5)

    print("\n--- '0401' 메시지 수신 시뮬레이션 ---")
    # 실제로는 외부에서 메시지가 들어와 _handle_message_reception이 호출됨
    mock_messenger.simulate_receive("0401", b"Simulated 0401 data for direct run test")
    print(
        f"'0401' 수신 후 데이터 저장소 (latest_status_0401): {csc_manager_instance.get_data_store_snapshot().get('latest_status_0401')}"
    )

    time.sleep(0.5)

    print("\n--- 재계획 CSU 로직 실행 (0401 수신 데이터 기반 가능) ---")
    # 이전 모니터링 결과나 0401 수신 데이터가 재계획에 영향을 줄 수 있음
    replan_res = csc_manager_instance.trigger_replan_csu_logic()
    print(f"재계획 결과: {replan_res}")
    # 재계획 결과에 따라 "0902" 메시지가 발신될 수 있음 (mock_push_message 로그 확인)

    time.sleep(0.5)

    print("\n--- '0902' (외부 재계획 요청) 메시지 수신 시뮬레이션 ---")
    mock_messenger.simulate_receive("0902", b"External replan trigger for direct run")
    # 이 호출로 인해 trigger_replan_csu_logic이 다시 호출되고, "0902" 메시지가 또 발신될 수 있음

    time.sleep(0.5)

    print("\n--- '0102' 메시지 직접 발신 테스트 ---")
    csc_manager_instance.send_message(
        "0102", {"mode": "standby", "reason": "direct_run_test"}
    )

    print("\nMissionMonitoringReplanCSCManager 직접 실행 테스트 종료.")
