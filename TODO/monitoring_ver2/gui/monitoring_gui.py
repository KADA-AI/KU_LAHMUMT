# gui/monitoring_gui.py: 애플리케이션의 메인 윈도우(QMainWindow)를 생성하고, 여러 탭들을 관리합니다.

# -*- coding: utf-8 -*-
# MonitoringTab.py

from PyQt5.QtCore import pyqtSignal, Qt, pyqtSlot
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QTabWidget,
    QTextEdit,
    QDockWidget,
)

# os.environ["KU_ROLE"] = "monitoring"

# 분리된 탭들을 임포트
from .tabs.MonitoringTab import MonitoringTab
from .tabs.ReplanTab import ReplanTab
from .tabs.SystemModeControlTab import SystemModeControlTab


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)
    log_received = pyqtSignal(str)  # 스레드 안전 로깅을 위한 시그널
    update_gui_signal = pyqtSignal(str, str, object) # GUI 업데이트를 위한 새로운 시그널

    def __init__(self, manager):
        super().__init__()
        self.setWindowTitle("임무 모니터링·판단 GUI (Refactored)")
        self.resize(800, 800)  # 높이 늘림
        self.manager = manager

        # 탭 위젯 생성 및 설정
        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        # 각 탭 인스턴스 생성
        self.monitoring_tab = MonitoringTab(manager=self.manager)
        self.replan_tab = ReplanTab(manager=self.manager)

        # 탭 위젯에 탭 추가
        self.tabs.addTab(self.monitoring_tab, "모니터링")
        self.tabs.addTab(self.replan_tab, "재계획 판단")

        # --- 로그 창 추가 ---
        self.log_dock = QDockWidget("로그", self)
        self.log_widget = QTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_dock.setWidget(self.log_widget)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.log_dock)

        # 업데이트 유형에 따라 처리할 탭들을 리스트로 매핑합니다.
        self.update_handlers = {
            "receive": [self.monitoring_tab, self.replan_tab],
            "logic": [self.monitoring_tab, self.replan_tab],
        }

        # 시그널-슬롯 연결
        self.log_received.connect(self._append_log_to_widget)
        self.update_gui_signal.connect(self._perform_gui_update) # 새로운 시그널 연결

    @pyqtSlot(str)
    def _append_log_to_widget(self, message):
        """GUI 스레드에서 로그 위젯에 메시지를 추가하는 슬롯"""
        self.log_widget.append(message)

    def add_log_message(
        self, tag: str, log_type: str, message: str, raw_data: bytes | None
    ):
        """다른 스레드에서 호출 가능한 메서드. 시그널을 발생시켜 GUI 스레드에서 처리하도록 함."""
        log_entry = f"[{tag}] [{log_type}] {message}"
        self.log_received.emit(log_entry)

    @pyqtSlot(str, str, object)
    def _perform_gui_update(self, update_type: str, key: str, data_object: object = None):
        """GUI 업데이트를 메인 스레드에서 수행하는 슬롯"""
        update_info = (update_type, key)
        handlers = self.update_handlers.get(update_type, [])
        for handler_tab in handlers:
            if hasattr(handler_tab, "refresh_display"):
                handler_tab.refresh_display(update_info, data_object)

    def update_view(self, update_type: str, key: str, data_object: object = None):
        """Manager가 데이터 변경을 알리기 위해 호출하는 콜백 메소드. 시그널을 통해 GUI 스레드로 전달."""
        self.update_gui_signal.emit(update_type, key, data_object)
