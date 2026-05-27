# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QFrame, QVBoxLayout, QLabel
from PyQt5.QtWidgets import QGraphicsDropShadowEffect
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtCore import Qt

class Card(QFrame):
    """둥근 모서리 + 그림자 카드. 내부에 자유롭게 위젯 배치."""
    def __init__(self, title: str = "", parent=None, dense=False):
        super().__init__(parent)
        self.setObjectName("Card")
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setFrameShape(QFrame.NoFrame)

        lay = QVBoxLayout(self)
        if dense:
            lay.setContentsMargins(12, 10, 12, 10)
        else:
            lay.setContentsMargins(16, 14, 16, 14)
        lay.setSpacing(10)

        self.title_label = None
        if title:
            self.title_label = QLabel(title, self)
            self.title_label.setObjectName("CardTitle")
            self.title_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)

            # ✅ 제목 라벨 배경을 완전 투명 + 자동 배경 채움 끔 → 카드 배경과 동일하게 보이게
            self.title_label.setAutoFillBackground(False)
            self.title_label.setStyleSheet("background: transparent;")

            f = self.title_label.font()
            f.setPointSize(f.pointSize() + 1)
            f.setWeight(QFont.DemiBold)
            self.title_label.setFont(f)
            lay.addWidget(self.title_label)

        # 미세한 그림자
        shadow = QGraphicsDropShadowEffect(self)
        shadow.setBlurRadius(28)
        shadow.setOffset(0, 10)
        shadow.setColor(QColor(15, 23, 42, 18))
        self.setGraphicsEffect(shadow)

        self.body_layout = lay
