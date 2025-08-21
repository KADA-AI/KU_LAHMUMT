# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QTextEdit
from PyQt5.QtCore import Qt
from .cards import Card

class ModuleWithLog(Card):
    """
    모듈 컨테이너(상단: 모듈 콘텐츠 자리표시자, 하단: 각진 검정 로그 박스).
    - 카드 외곽은 둥근 모서리 + 그림자
    - 로그 박스(QTextEdit)는 #LogBox 스타일(각진 사각형, 검정 배경)
    """
    def __init__(self, title: str, parent=None):
        super().__init__(title, parent, dense=False)

        # 상단 콘텐츠(자리표시자) — 이후 실제 위젯으로 교체
        self.content = QLabel("모듈 콘텐츠 영역(추가 구현 예정)", self)
        self.content.setObjectName("Placeholder")
        self.content.setAlignment(Qt.AlignCenter)

        # 하단 로그 — 각진 검정 박스
        self.log = QTextEdit(self)
        self.log.setObjectName("LogBox")
        self.log.setReadOnly(True)

        lay = QVBoxLayout()
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)
        lay.addWidget(self.content, 2)
        lay.addWidget(self.log, 1)
        self.body_layout.addLayout(lay, 1)

    def append_log(self, text: str):
        self.log.append(text)
