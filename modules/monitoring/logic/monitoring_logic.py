# logic/monitoring_logic.py: 세부 로직 클래스들을 관리하고, 백그라운드 스레드에서 주기적으로 실행하는 코디네이터를 정의합니다.

import threading
import time

from .monitoring_logic_part import MonitoringLogic
from .replan_logic_part import ReplanLogic

class MonitoringLogicHandler:
    """
    두 개의 세부 로직(모니터링, 재계획)을 소유하고,
    백그라운드 스레드에서 주기적으로 실행을 관리하는 코디네이터.
    """
    def __init__(self, manager):
        self.manager = manager
        self.monitoring_logic = MonitoringLogic(manager)
        self.replan_logic = ReplanLogic(manager)
        
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        """로직 실행 루프를 백그라운드 스레드에서 시작합니다."""
        if self._thread is not None:
            return # 이미 실행 중

        self._stop_event.clear()
        self._thread = threading.Thread(target=self._logic_loop, daemon=True)
        self._thread.start()
        self.manager._log("LOGIC_HANDLER", "INFO", "백그라운드 로직 스레드 시작됨.")

    def stop(self):
        """백그라운드 스레드를 정지시킵니다."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join() # 스레드가 완전히 종료될 때까지 대기
        self.manager._log("LOGIC_HANDLER", "INFO", "백그라운드 로직 스레드 정지됨.")

    def _logic_loop(self):
        """주기적으로 각 로직 파트의 실행을 시도하는 메인 루프."""
        while not self._stop_event.is_set():
            system_mode = self.manager.logic_store.get_data("SystemMode")

            if system_mode == 4:
                try:
                    self.manager._log("LOGIC_LOOP", "INFO", "단일 로직 수행 시작 (Mode 4)")
                    self.monitoring_logic.execute(mode_override=3)
                    self.replan_logic.execute(mode_override=3)
                    self.manager._log("LOGIC_LOOP", "INFO", "단일 로직 수행 완료. 대기 모드(1)로 전환합니다.")
                except Exception as e:
                    self.manager._log("LOGIC_LOOP", "ERROR", f"단일 로직 실행 중 예외 발생: {e}")
                finally:
                    # 로직 실행 후 시스템 모드를 1 (대기 모드)로 설정
                    self.manager.set_system_mode(1)
            else:
                try:
                    self.monitoring_logic.execute()
                    self.replan_logic.execute()
                except Exception as e:
                    self.manager._log("LOGIC_LOOP", "ERROR", f"로직 실행 중 예외 발생: {e}")
            
            # 1초 대기 (CPU 사용량 조절)
            time.sleep(1)
