# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt
from .cards import Card

class OperationFlowPanel(Card):
    """
    운용흐름 모니터링 자리표시자
    - 제목 없음(요청사항)
    """
    def __init__(self, parent=None):
        super().__init__("", parent)  # 제목 제거
        lbl = QLabel("운용 단계/상태 모니터링(추가 구현 예정)", self)
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setObjectName("Placeholder")
        self.body_layout.addWidget(lbl, 1)
