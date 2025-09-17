# gui/tabs/SystemModeControlTab.py: 시스템 모드 제어 탭

import time
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QComboBox, QPushButton, QHBoxLayout
from PyQt5.QtCore import pyqtSlot

# nFusion 메시지 모델 및 NodeMessenger 임포트
# 이 임포트가 작동하려면 메인 애플리케이션 시작 시 C# 어셈블리가 로드되어 있어야 합니다.
from nFusion.Model.msg_0101 import SystemOperationMode
from nFusion.Nodes.Core import NodeMessenger
from System import String, UInt64, UInt32

class SystemModeControlTab(QWidget):
    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager
        self.init_ui()

    def init_ui(self):
        main_layout = QVBoxLayout(self)

        # 제목
        title_label = QLabel("<h2>시스템 모드 제어</h2>")
        main_layout.addWidget(title_label)

        # 시스템 모드 선택
        mode_layout = QHBoxLayout()
        mode_label = QLabel("시스템 모드 선택:")
        self.mode_combo = QComboBox()
        self.mode_combo.addItem("0: 초기화 모드", 0)
        self.mode_combo.addItem("1: 대기 모드", 1)
        self.mode_combo.addItem("2: 초기 임무 재계획 모드", 2)
        self.mode_combo.addItem("3: 임무 수행 모드", 3)
        self.mode_combo.addItem("4: 단일 로직 수행 모드", 4)
        mode_layout.addWidget(mode_label)
        mode_layout.addWidget(self.mode_combo)
        main_layout.addLayout(mode_layout)

        # 메시지 전송 버튼
        send_button = QPushButton("System Mode 메시지 전송")
        send_button.clicked.connect(self._send_system_mode)
        main_layout.addWidget(send_button)

        # 로그/피드백 영역
        self.feedback_label = QLabel("")
        main_layout.addWidget(self.feedback_label)

        main_layout.addStretch(1) # 하단에 공간 추가
        self.setLayout(main_layout)

    @pyqtSlot()
    def _send_system_mode(self):
        selected_mode = self.mode_combo.currentData() # QComboBox에 저장된 실제 데이터 (int)
        
        try:
            msg_obj = SystemOperationMode()
            msg_obj.timestamp = int(time.time() * 1000)
            msg_obj.source = String("GUI_CONTROL") # 메시지 소스
            msg_obj.systemMode = UInt32(selected_mode) # 선택된 시스템 모드

            # NodeMessenger를 통해 메시지 푸시
            # manager.node_messenger는 MonitoringManager에서 NodeMessenger 인스턴스를 참조합니다.
            self.manager.node_messenger.Push[SystemOperationMode](msg_obj)
            
            feedback_msg = f"[성공] System Mode {selected_mode} 메시지 전송 완료!"
            self.feedback_label.setStyleSheet("color: green")
            self.manager._log("GUI", "INFO", feedback_msg) # Manager의 로깅 기능 사용

        except Exception as e:
            feedback_msg = f"[오류] System Mode 메시지 전송 실패: {e}"
            self.feedback_label.setStyleSheet("color: red")
            self.manager._log("GUI", "ERROR", feedback_msg) # Manager의 로깅 기능 사용
        
        self.feedback_label.setText(feedback_msg)

    def refresh_display(self, update_info, data_object=None):
        # 이 탭은 현재 Manager로부터의 업데이트를 직접 처리하지 않습니다.
        pass
