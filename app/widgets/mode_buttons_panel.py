# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QSizePolicy

from PyQt5 import QtCore

class ModeButtonsPanel(QWidget):
    """
    좌측 모드 버튼 영역.
    - 주요 운용 버튼 5개를 세로로 균등 배치
    """
    initialMissionPlanningRequested = QtCore.Signal() if hasattr(QtCore, "Signal") else QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(20)

        button_specs = [
            ("SW 실행",         "btn_sw_run"),
            ("SW 상태확인",     "btn_self_check"),
            ("대기모드",        "btn_standby"),
            ("초기임무계획모드", "btn_init_plan"),
            ("임무 수행 모드",   "btn_mission_mode"),
        ]

        for text, objname in button_specs:
            btn = QPushButton(text, self)
            btn.setObjectName(objname)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lay.addWidget(btn, 1)

    def on_click_initial_mission_planning(self):
        """
        '초기 임무 계획 모드' 버튼 클릭 시 메인 윈도우로 시그널 전달.
        - 메인 윈도우는 start_initial_mission_planning() 실행
        """
        self.initialMissionPlanningRequested.emit()
