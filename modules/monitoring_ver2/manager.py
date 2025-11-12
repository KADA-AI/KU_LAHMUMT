# manager.py: 모듈의 모든 컴포넌트(데이터, 로직, GUI)를 총괄하고 상호 연동을 중재하는 중앙 관리자(Mediator) 클래스를 정의합니다.
import sys
import json
from datetime import datetime, timezone

from typing import Callable, Any, Optional
from dataclasses import asdict
from functools import partial

from data.receive_storage import ReceiveStorage
from data.logic_storage import LogicStorage
from data.push_storage import PushStorage
from data.message_models import ModuleStatusModelModel

from logic.monitoring_logic import MonitoringLogicHandler
from receive.receive_center import register_listener
from push import message0102_push
from udp_reporter import notify_mode
from config import SYSTEM_MODE_LABELS, INITIAL_MODE
from modules.common import agent_status_snapshot

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

        # 2. 로직 핸들러 초기화 (스레드 시작은 마지막에)
        self.logic_handler = MonitoringLogicHandler(manager=self)

        # 3. GUI 콜백 초기화
        self.log_callback: Optional[Callable] = None
        self.gui_update_callback: Optional[Callable] = None

        # 4. 시스템 모드 초기화 (초기화 모드를 기본 상태로 설정)
        self.set_system_mode(INITIAL_MODE, force=True)

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
        if msg_id == "0401":
            try:
                agent_status_snapshot.save_agent_status_snapshot(data_object)
            except Exception as exc:
                self._log("MON_MGR", "WARN", f"Failed to persist 0401 snapshot: {exc}")
        try:
            self.logic_handler.monitoring_logic.handle_message(msg_id, data_object)
        except AttributeError:
            pass
        except Exception as exc:
            self._log("MON_MGR", "ERROR", f"logic message hook failed: {exc}")

        if msg_id == "0202":
            try:
                self.logic_handler.monitoring_logic.trigger_prior_mission_replan()
            except AttributeError:
                pass
            except Exception as exc:
                self._log("MON_MGR", "WARN", f"Immediate 0202 replan trigger failed: {exc}")

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

    def set_system_mode(self, mode: int, *, force: bool = False):
        """시스템 운용 모드를 변경합니다.

        기본 모드값:
          - 5     : 전원 OFF 모드
          - 0     : 초기화 모드
          - 1     : 대기 모드
          - 2     : 초기 임무 재계획 모드
          - 3     : 임무 수행 모드
          - 4     : 특일 로직 실행 모드
        """
        try:
            mode_value = int(mode)
        except (TypeError, ValueError):
            self._log("MON_MGR", "WARN", f"Invalid system mode value: {mode!r}")
            return

        current = self.logic_store.get_data("SystemMode")
        try:
            current_value = int(current) if current is not None else None
        except (TypeError, ValueError):
            current_value = None

        if not force and current_value == mode_value:
            return

        self._log("MON_MGR", "MODE_CHANGE", f"System mode set to '{mode_value}'.")
        self.logic_store.set_data("SystemMode", mode_value)
        try:
            self.logic_handler.monitoring_logic.on_system_mode_changed(mode_value)
        except Exception:
            pass

        mode_text = SYSTEM_MODE_LABELS.get(mode_value, f"알 수 없는 모드 ({mode_value})")
        notify_mode(mode_text)

        if self.gui_update_callback:
            self.gui_update_callback("logic", "SystemMode")

    def send_status_message(self, status: int = 1) -> None:
        """0102 상태 메시지를 발신합니다."""
        timestamp = int(
            (
                datetime.now(timezone.utc) - datetime(2000, 1, 1, tzinfo=timezone.utc)
            ).total_seconds()
            * 1000
        )

        body_obj = ModuleStatusModelModel(
            timestamp=timestamp,
            source="MSM",
            status=int(status),
        )

        try:
            message0102_push.make_and_push(body_obj, self.node_messenger)
            self.push_store.add_data("0102", body_obj)
            self._log("MON_MGR", "TX", f"0102 sent (status={status})")
            raw_bytes = json.dumps(asdict(body_obj), ensure_ascii=False).encode("utf-8")
            if self.gui_update_callback:
                self.gui_update_callback("send", "0102", raw_bytes)
        except Exception as exc:
            self._log("MON_MGR", "ERROR", f"0102 push failed: {exc}")

    def send_initial_status_message(self) -> None:
        """초기화 완료 후 0102 상태 메시지를 발신합니다."""
        self._log("MON_MGR", "INFO", "Sending initial status message (0102)...")
        self.send_status_message(status=1)

    def trigger_logic(self):
        pass

    def send_message(self, msg_id: str, on_done: Optional[Callable] = None):
        pass




