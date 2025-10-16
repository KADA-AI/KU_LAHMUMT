# manager.py: 모듈의 모든 컴포넌트(데이터, 로직, GUI)를 총괄하고 데이터 흐름을 중재하는 중앙 관리자(Mediator) 클래스를 정의합니다.
import sys
from datetime import datetime, timezone

from datetime import timezone

from typing import Callable, Any, Optional
from functools import partial

# --- 신규 저장소 클래스 import ---
from data.receive_storage import ReceiveStorage
from data.logic_storage import LogicStorage
from data.push_storage import PushStorage
from data.message_models import ModuleStatusModelModel

from logic.monitoring_logic import MonitoringLogicHandler
from receive.receive_center import register_listener
from push import message0102_push
from udp_reporter import notify_mode


class MonitoringManager:
    """
    데이터 흐름을 중재하고 모든 컴포넌트를 관리하는 중앙 컨트롤러(Mediator).
    3개의 분리된 저장소를 소유하고 관리합니다.
    """

    def __init__(self, node_messenger, receive_messages_config: list):
        self.node_messenger = node_messenger

        # 1. 데이터 저장소 초기화
        self.receive_store = ReceiveStorage()
        self.logic_store = LogicStorage()
        self.push_store = PushStorage()

        # 2. 시스템 모드 기본값 설정
        self.logic_store.set_data("SystemMode", 0)  # 0: 초기화 모드

        # 3. 로직 핸들러 초기화 (스레드 시작은 마지막에)
        self.logic_handler = MonitoringLogicHandler(manager=self)

        # 4. GUI 콜백 초기화
        self.log_callback: Optional[Callable] = None
        self.gui_update_callback: Optional[Callable] = None

        # 5. 메시지 수신 리스너 등록 (이제 Signal/Slot으로 대체됨)
        # for msg_id, _ in receive_messages_config:
        #     handler = partial(self.handle_message_reception, msg_id)
        #     register_listener(msg_id, handler)

        self._log("MON_MGR", "INFO", "Monitoring Manager initialized.")

        # 6. 백그라운드 로직 스레드 시작
        self.logic_handler.start()

        # 7. 초기 상태 메시지(0102) 발신
        self.send_initial_status_message()

    def shutdown(self):
        """어플리케이션 종료 시 호출되어 자원을 정리합니다."""
        self._log("MON_MGR", "INFO", "Shutting down manager...")
        self.logic_handler.stop()

    def _log(
        self, tag: str, log_type: str, message: str, raw_data: Optional[bytes] = None
    ):
        if self.log_callback:
            self.log_callback(tag, log_type, message, raw_data)
        else:
            # 콜백이 설정되지 않은 경우 콘솔에 직접 출력
            print(f"[{tag}] [{log_type}] {message}")

    def handle_message_reception(self, msg_id: str, data_object: object):
        """수신된 데이터 클래스 객체를 저장소에 저장하고, GUI에 변경 사실을 알립니다."""
        self._log("MON_MGR", "RECV", f"Parsed object received: {msg_id}")
        self.receive_store.set_data(msg_id, data_object)

        # 메시지 ID에 따라 시스템 모드를 업데이트
        if msg_id == "0101" and hasattr(data_object, "systemMode"):
            self.set_system_mode(data_object.systemMode)
        elif msg_id == "0103" and hasattr(data_object, "mode"):
            self.set_system_mode(data_object.mode)

        if self.gui_update_callback:
            self.gui_update_callback("receive", msg_id, data_object)

    def get_received_data(self, msg_id: str) -> Any:
        return self.receive_store.get_data(msg_id)

    def get_logic_result(self, key: str) -> Any:
        return self.logic_store.get_data(key)

    def set_system_mode(self, mode: int):
        """시스템 실행 모드를 변경합니다. (0:초기화, 1:대기, 2:초기임무재계획, 3:임무수행)"""
        self._log("MON_MGR", "MODE_CHANGE", f"System mode set to '{mode}'.")
        self.logic_store.set_data("SystemMode", mode)

        mode_map = {
            0: "초기화 모드",
            1: "대기모드",
            2: "초기 임무 계획",
            3: "임무 수행",
        }
        mode_text = mode_map.get(mode, f"알 수 없는 모드 ({mode})")
        notify_mode(mode_text)

        if self.gui_update_callback:
            self.gui_update_callback("logic", "SystemMode")

    def send_initial_status_message(self):
        """초기화 완료 후 0102 상태 메시지를 발신합니다."""
        self._log("MON_MGR", "INFO", "Sending initial status message (0102)...")
        timestamp = int(
            (
                datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
            ).total_seconds()
            * 1000
        )

        # 0102 메시지 본문 생성
        body_obj = ModuleStatusModelModel(
            timestamp=timestamp, source="MonitoringModule", status=1  # 1: Normal
        )

        # 생성된 데이터 모델 객체를 직접 전달
        message0102_push.make_and_push(body_obj, self.node_messenger)

        # PushStorage에 저장
        self.push_store.add_data("0102", body_obj)

    # --- 기존의 다른 메소드들 (변경 없음) ---
    def trigger_logic(self):
        pass

    def send_message(self, msg_id: str, on_done: Optional[Callable] = None):
        pass
