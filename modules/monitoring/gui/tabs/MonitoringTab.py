# -*- coding: utf-8 -*- 
# MonitoringTab.py

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTextEdit
import json

class MonitoringTab(QWidget):
    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        self.label = QLabel("모니터링 탭: 수신된 데이터가 아래에 표시됩니다.")
        self.display = QTextEdit()
        self.display.setReadOnly(True)
        layout.addWidget(self.label)
        layout.addWidget(self.display)
        self.setLayout(layout)

    def refresh_display(self, update_info: dict):
        """Manager로부터 데이터 변경 알림을 받아 화면을 갱신합니다."""
        update_type = update_info.get("type")
        key = update_info.get("key")

        if not (update_type == "receive" and key):
            return

        # Manager에서 실제 데이터 가져오기
        data_object = self.manager.get_received_data(key)
        
        if data_object is None:
            return

        # 객체를 보기 좋은 문자열(JSON)로 변환
        try:
            # __dict__를 사용하여 객체의 속성을 딕셔너리로 변환
            data_str = json.dumps(data_object.__dict__, indent=2, ensure_ascii=False, default=str)
        except Exception:
            # JSON 변환 실패 시 일반적인 문자열로 표시
            data_str = str(data_object)

        log_message = f"--- 수신 (ID: {key}) ---\n{data_str}\n"
        
        # 최신 데이터를 위쪽에 추가
        current_text = self.display.toPlainText()
        self.display.setText(log_message + "\n" + current_text)