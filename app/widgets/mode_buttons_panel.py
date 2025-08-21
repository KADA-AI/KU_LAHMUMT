# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QSizePolicy

class ModeButtonsPanel(QWidget):
    """
    모드 버튼 영역 (창 프레임 없음).
    - 총 6개 버튼을 전체 영역에 꽉 차게 배치
    - 버튼 간 상하 간격은 균등하게 늘어남
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(20)  # 개별 간격은 없애고 stretch로 균등 분배

        labels = [
            "SW 실행",
            "SW 자체점검",
            "대기모드",
            "초기임무계획모드",
            "임무 수행 모드",
            "운용자 입력 GUI",
        ]

        for text in labels:
            btn = QPushButton(text, self)
            btn.setObjectName("ModeBtn")
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            lay.addWidget(btn, 1)  # 각 버튼을 동일 비율(=전체 공간 1/6씩)로 배정
