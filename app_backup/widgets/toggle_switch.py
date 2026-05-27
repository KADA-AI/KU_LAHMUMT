# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import QCheckBox
from PyQt5.QtCore import Qt, QSize, QPropertyAnimation, pyqtProperty
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor

class ToggleSwitch(QCheckBox):
    """
    커스텀 토글 스위치
    - 파란 캡슐(ON) / 연회색 캡슐(OFF) + 흰색 원형 손잡이
    - 마우스 클릭 시 부드럽게 슬라이드
    - 외부 QSS의 ::indicator 규칙과 무관(직접 그리기)
    """
    def __init__(self, parent=None, checked=False):  # ✅ 기본 False
        super().__init__("", parent)
        self.setCursor(Qt.PointingHandCursor)
        self.setFocusPolicy(Qt.NoFocus)
        self.setTristate(False)
        self.setChecked(bool(checked))               # ← 전달되면 그 값, 기본은 False

        # 애니메이션 오프셋 (0.0=OFF, 1.0=ON)
        self._offset = 1.0 if self.isChecked() else 0.0
        self._anim = QPropertyAnimation(self, b"offset", self)
        self._anim.setDuration(160)

        self.toggled.connect(self._on_toggled)


    # ---- property for animation ----
    def getOffset(self) -> float:
        return self._offset
    def setOffset(self, v: float):
        self._offset = max(0.0, min(1.0, float(v)))
        self.update()
    offset = pyqtProperty(float, fget=getOffset, fset=setOffset)

    # ---- mouse/keyboard handling ----
    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            # 수동 토글: 외부 스타일/중첩 위젯 영향이 있어도 확실히 동작
            self.setChecked(not self.isChecked())
        super().mouseReleaseEvent(e)

    def keyPressEvent(self, e):
        # Space/Enter로도 토글
        if e.key() in (Qt.Key_Space, Qt.Key_Return, Qt.Key_Enter):
            self.setChecked(not self.isChecked())
            e.accept()
            return
        super().keyPressEvent(e)

    # ---- animate when state changes ----
    def _on_toggled(self, on: bool):
        self._anim.stop()
        self._anim.setStartValue(self._offset)
        self._anim.setEndValue(1.0 if on else 0.0)
        self._anim.start()

    # ---- painting ----
    def sizeHint(self) -> QSize:
        return QSize(56, 28)
    def minimumSizeHint(self) -> QSize:
        return self.sizeHint()

    def paintEvent(self, e):
        w, h = self.width(), self.height()
        r = h / 2.0
        knob_margin = max(2, int(h * 0.08))
        knob_d = h - knob_margin * 2

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)

        # 색상 (ON/OFF)
        bg_on  = QColor(133, 173, 234);  bd_on  = QColor(58, 100, 167)
        bg_off = QColor(210, 215, 222);  bd_off = QColor(160, 165, 175)

        t = self._offset  # 0..1
        def lerp(c0, c1, k):
            return QColor(
                int(c0.red()   + (c1.red()   - c0.red())   * k),
                int(c0.green() + (c1.green() - c0.green()) * k),
                int(c0.blue()  + (c1.blue()  - c0.blue())  * k),
                int(c0.alpha() + (c1.alpha() - c0.alpha()) * k),
            )
        bg = lerp(bg_off, bg_on, t)
        bd = lerp(bd_off, bd_on, t)

        # 캡슐
        p.setPen(QPen(bd, 2))
        p.setBrush(QBrush(bg))
        p.drawRoundedRect(1, 1, w - 2, h - 2, r, r)

        # 손잡이
        left_x  = knob_margin + 1
        right_x = w - knob_margin - knob_d - 1
        x = int(left_x + (right_x - left_x) * t)

        p.setPen(QPen(bd, 1))
        p.setBrush(Qt.white)
        p.drawEllipse(x, knob_margin + 1, knob_d - 2, knob_d - 2)
        p.end()
