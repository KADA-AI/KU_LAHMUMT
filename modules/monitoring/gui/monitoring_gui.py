# -*- coding: utf-8 -*-
# monitoring_gui.py – 메인 윈도우와 탭 관리를 담당
import sys

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QTabWidget,
)

# os.environ["KU_ROLE"] = "monitoring"

# 분리된 탭들을 임포트
from .tabs.MonitoringTab import MonitoringTab
from .tabs.ReplanTab import ReplanTab
from .tabs.DummyTab import DummyTab


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)

    def __init__(self, manager):
        super().__init__()
        self.setWindowTitle("임무 모니터링·판단 GUI (Refactored)")
        self.resize(800, 600)
        self.manager = manager

        # 탭 위젯 생성 및 설정
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # 각 탭 인스턴스 생성
        self.monitoring_tab = MonitoringTab(manager=self.manager)
        self.replan_tab = ReplanTab()
        self.dummy_tab = DummyTab()

        # 탭 위젯에 탭 추가
        self.tabs.addTab(self.monitoring_tab, "모니터링")
        self.tabs.addTab(self.replan_tab, "재계획 판단")
        self.tabs.addTab(self.dummy_tab, "더미")

        # 업데이트 유형에 따라 처리할 탭을 매핑합니다.
        # 이렇게 하면 향후 로직이 추가되어도 이 딕셔너리만 수정하면 됩니다.
        self.update_handlers = {
            "receive": self.monitoring_tab,
            "logic": self.monitoring_tab,
            # 예: "replan": self.replan_tab
        }

    def add_log_message(
        self, tag: str, log_type: str, message: str, raw_data: bytes | None
    ):
        # GUI에 복잡한 로그 위젯 대신 콘솔에 출력하도록 단순화
        log_entry = f"[{tag}] [{log_type}] {message}"
        print(f"LOG: {log_entry}")

    def update_view(self, update_type: str, key: str):
        """Manager가 데이터 변경을 알리기 위해 호출하는 콜백 메소드."""
        update_info = {"type": update_type, "key": key}  # 탭에 전달할 정보 구조

        # 매핑된 핸들러(탭)를 찾아 업데이트 메소드를 호출합니다.
        handler_tab = self.update_handlers.get(update_type)
        if handler_tab and hasattr(handler_tab, "refresh_display"):
            handler_tab.refresh_display(update_info)
        else:
            print(f"LOG (GUI): No handler found for update type '{update_type}'")


# 이 파일이 직접 실행될 경우 (테스트용)
if __name__ == "__main__":
    print("WARNING: Running GUI directly. This is for testing only.")
    app = QApplication(sys.argv)

    class MockManager:
        def __init__(self):
            self.log_callback = None
            self.gui_update_callback = None
            self.received = {"0101": {"raw": b"mock receive data"}}
            self.logic = {"res1": {"status": "mock logic ok"}}
            self.push = {"0102": [{"body": {"status": 1}}]}

        def get_received_data(self, key):
            return self.received.get(key)

        def get_logic_result(self, key):
            return self.logic.get(key)

        def get_push_history(self, key):
            return self.push.get(key, [])

        def trigger_logic(self):
            print("MockManager: trigger_logic called")

    mock_manager = MockManager()
    win = MainWindow(manager=mock_manager)
    mock_manager.log_callback = win.add_log_message
    mock_manager.gui_update_callback = win.update_view

    win.show()
    sys.exit(app.exec_())
