# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Tuple
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, QEasingCurve
from PyQt5.QtGui import (
    QPainter, QPen, QColor, QFont, QPainterPath, QLinearGradient, QBrush
)
from PyQt5.QtWidgets import QWidget

"""
FlowVisualizer: 외부 카드/프레임 없이 자체 페인팅하는 데이터 흐름 다이어그램
- 세 개 모듈(상단부터): 임무 모니터링 및 판단 / 임무 할당 및 계획 수립 / 의사결정 지원
- 왼쪽 수직 Spine(회색) + 각 모듈로 들어오고 나가는 화살표 2개 (→, ←)
- trigger(module, direction) 호출 시 해당 화살표에 "빛나는" 흐름 애니메이션

사용 예:
    w = FlowVisualizer()
    w.trigger("monitor", "in")     # 모니터링 모듈로 '들어옴' 방향(→) 애니메이션
    w.trigger("mission", "out")    # 임무 할당/계획 모듈에서 '나감' 방향(←) 애니메이션
    w.trigger("decision", "in")    # 의사결정 모듈 '들어옴'
"""

# 모듈 키
MODULE_KEYS = ("monitor", "mission", "decision")

@dataclass
class PulseState:
    active: bool = False
    t: float = 0.0           # 0~1 진행도
    speed: float = 0.8       # 1초 당 진행 비율
    ttl: float = 1.25        # 한 번 재생 시간(초)
    direction: str = "in"    # "in" or "out"


class FlowVisualizer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_StyledBackground, True)
        # 배경은 메인 윈도우 배경과 자연스럽게 이어지도록 투명/기본 위젯 배경 사용
        self.setAutoFillBackground(False)

        # 모듈 색상(박스)
        self.color_monitor  = QColor("#f97316")  # 주황
        self.color_mission  = QColor("#1d4ed8")  # 파랑
        self.color_decision = QColor("#3b82f6")  # 하늘/보라톤 블루

        # 텍스트(모듈명)
        self.texts = {
            "monitor":  "임무 모니터링\n및 판단 모듈",
            "mission":  "임무 할당 및\n계획 수립 모듈",
            "decision": "의사결정\n지원 모듈",
        }

        # 각 화살표에 대한 pulse 상태: (module, dir) -> PulseState
        self.pulses: Dict[Tuple[str, str], PulseState] = {}
        for m in MODULE_KEYS:
            self.pulses[(m, "in")]  = PulseState(active=False, direction="in")
            self.pulses[(m, "out")] = PulseState(active=False, direction="out")

        # 애니메이션 타이머(60fps 근사)
        self.timer = QTimer(self)
        self.timer.setInterval(16)
        self.timer.timeout.connect(self._tick)

        # 글꼴 살짝 두껍게
        self.font_title = self.font()
        self.font_title.setPointSize(self.font_title.pointSize() + 2)
        self.font_title.setBold(True)

    # ----------------------- Public API -----------------------
    def trigger(self, module: str, direction: str = "in", speed: float = 0.9, ttl: float = 1.4):
        """
        특정 모듈의 화살표에 '빛나는 흐름' 애니메이션 트리거
        module: "monitor" | "mission" | "decision"
        direction: "in"(→) | "out"(←)
        """
        if module not in MODULE_KEYS:  # 잘못된 키 방지
            return
        if direction not in ("in", "out"):
            return

        p = self.pulses[(module, direction)]
        p.active = True
        p.t = 0.0
        p.speed = max(0.3, float(speed))
        p.ttl = max(0.6, float(ttl))
        if not self.timer.isActive():
            self.timer.start()
        self.update()

    # ----------------------- Animation -----------------------
    def _tick(self):
        # 모든 pulse를 진행
        any_active = False
        dt = self.timer.interval() / 1000.0  # 초
        for key, p in self.pulses.items():
            if p.active:
                p.t += dt / p.ttl
                if p.t >= 1.0:
                    p.active = False
                    p.t = 1.0
                else:
                    any_active = True
        if not any_active:
            self.timer.stop()
        self.update()

    # ----------------------- Geometry helpers -----------------------
    def _layout_geometry(self, w: int, h: int):
        """
        주어진 위젯 폭/높이에 대해 spine, 모듈 박스, 화살표 위치를 계산
        반환: dict
        """
        margin_l = int(w * 0.06)      # 왼쪽 spine 여백
        spine_x  = margin_l
        spine_t  = int(h * 0.05)
        spine_b  = int(h * 0.95)

        # 모듈 박스 공통 크기 (더 크게, 정사각형 비율에 가깝게)
        box_size = int(min(w, h) * 0.50)     # 정사각형 크기
        box_w = box_size
        box_h = box_size

        box_x = int(spine_x + w * 0.40)      # spine에서 조금 더 오른쪽 (여백 넉넉히)

        # 세 개 상자 Y 위치 (균등)
        gap_v = int((h - (spine_t + (spine_b - spine_t)))/2)  # 거의 0 처리. 안전용
        band_h = (spine_b - spine_t)
        y1 = int(spine_t + band_h * 0.04)
        y2 = int(spine_t + band_h * 0.38)
        y3 = int(spine_t + band_h * 0.71)

        boxes = {
            "monitor":  QRectF(box_x, y1, box_w, box_h),
            "mission":  QRectF(box_x, y2, box_w, box_h),
            "decision": QRectF(box_x, y3, box_w, box_h),
        }

        # 화살표 길이 (좌우로 더 길게)
        arrow_in_len  = int(box_x - spine_x - 80)   # 기존 -18 → -40
        arrow_out_len = arrow_in_len

        # 각 박스 중앙 y에서 좌우로 그릴 화살표 y
        centers = {k: int(r.y() + r.height() / 2) for k, r in boxes.items()}

        return {
            "spine": (spine_x, spine_t, spine_b),
            "boxes": boxes,
            "centers": centers,
            "arrow_in_len": arrow_in_len,
            "arrow_out_len": arrow_out_len,
        }

    # ----------------------- Painting -----------------------
    def paintEvent(self, e):
        w = self.width()
        h = self.height()

        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), Qt.transparent)  # 카드/프레임 없이

        geom = self._layout_geometry(w, h)
        spine_x, spine_t, spine_b = geom["spine"]

        # 1) Spine
        pen_spine = QPen(QColor(160, 160, 165), 6, Qt.SolidLine, Qt.RoundCap)
        p.setPen(pen_spine)
        p.drawLine(spine_x, spine_t, spine_x, spine_b)

        # 2) 각 모듈 박스
        self._draw_module(p, geom, "monitor",  self.color_monitor)
        self._draw_module(p, geom, "mission",  self.color_mission)
        self._draw_module(p, geom, "decision", self.color_decision)

        # 3) 화살표(기본 회색 + 활성시 빛나는 오버레이)
        for key in MODULE_KEYS:
            self._draw_arrows_for_module(p, geom, key)

        p.end()

    def _draw_module(self, p: QPainter, geom: dict, key: str, color: QColor):
        rect: QRectF = geom["boxes"][key]
        rnd = 10.0

        # 박스(그라디언트로 살짝 깊이감)
        grad = QLinearGradient(rect.topLeft(), rect.bottomLeft())
        c1 = QColor(color)
        c2 = QColor(color).darker(115)
        grad.setColorAt(0.0, c1)
        grad.setColorAt(1.0, c2)
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(grad))
        path = QPainterPath()
        path.addRoundedRect(rect, rnd, rnd)
        p.drawPath(path)

        # 텍스트
        p.setPen(Qt.white)
        p.setFont(self.font_title)
        text = self.texts[key]
        p.drawText(rect.adjusted(10, 8, -10, -8), Qt.AlignCenter | Qt.TextWordWrap, text)

    def _draw_arrows_for_module(self, p: QPainter, geom: dict, key: str):
        spine_x, _, _ = geom["spine"]
        rect: QRectF = geom["boxes"][key]
        cy = geom["centers"][key]
        box_left = int(rect.left())

        # ---- 기본 화살표 (회색) ----
        base_pen = QPen(QColor(160, 160, 165), 4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        p.setPen(base_pen)
        # IN (→) spine -> box
        self._draw_arrow(p, QPointF(spine_x + 4, cy), QPointF(box_left - 8, cy), head_dir="right")
        # OUT (←) box -> spine (약간 아래로 위치해서 구분)
        self._draw_arrow(p, QPointF(box_left - 8, cy + 18), QPointF(spine_x + 4, cy + 18), head_dir="left")

        # ---- 활성 오버레이(빛나는 흐름) ----
        for direction in ("in", "out"):
            ps = self.pulses[(key, direction)]
            if not ps.active and ps.t <= 0.0:
                continue
            t = max(0.0, min(ps.t, 1.0))

            # 진행 구간(0~1)을 eOutQuad로 완만하게
            eased = QEasingCurve(QEasingCurve.OutQuad).valueForProgress(t)

            # 진행하는 하이라이트 구간(선 따라 움직이는 짧은 구간)
            # 선 길이 대비 하이라이트 비율
            hl_ratio = 0.25
            start_ratio = eased * (1.0 + hl_ratio) - hl_ratio
            end_ratio = eased
            start_ratio = max(0.0, min(1.0, start_ratio))
            end_ratio   = max(0.0, min(1.0, end_ratio))

            # 색(모듈별 + 방향별)
            base = {
                "monitor":  self.color_monitor,
                "mission":  self.color_mission,
                "decision": self.color_decision,
            }[key]
            glow = QColor(base)
            glow = glow.lighter(165) if direction == "in" else glow.lighter(135)

            # 경로/방향
            if direction == "in":
                a0 = QPointF(spine_x + 4, cy)
                a1 = QPointF(box_left - 8, cy)
                head = "right"
            else:
                a0 = QPointF(box_left - 8, cy + 18)
                a1 = QPointF(spine_x + 4, cy + 18)
                head = "left"

            # 기본 선 위에 그라디언트 브러시로 '흘러가는' 하이라이트
            self._draw_glow_segment(p, a0, a1, start_ratio, end_ratio, glow, head)

    def _draw_arrow(self, p: QPainter, a: QPointF, b: QPointF, head_dir: str):
        # 직선
        p.drawLine(a, b)
        # 화살촉
        vec = QPointF(b.x() - a.x(), b.y() - a.y())
        length = max(1.0, (vec.x()**2 + vec.y()**2) ** 0.5)
        if length < 1.0:
            return
        ux, uy = vec.x()/length, vec.y()/length
        size = 9.0
        # 두 날개 벡터
        left = QPointF(-ux, -uy)  # 반대 방향
        # 수직 벡터
        vx, vy = -uy, ux
        if head_dir == "right":
            tip = b
        else:
            tip = b
        wing1 = QPointF(tip.x() - ux*size + vx*size*0.6, tip.y() - uy*size + vy*size*0.6)
        wing2 = QPointF(tip.x() - ux*size - vx*size*0.6, tip.y() - uy*size - vy*size*0.6)
        path = QPainterPath()
        path.moveTo(tip)
        path.lineTo(wing1)
        path.lineTo(wing2)
        path.closeSubpath()
        p.fillPath(path, p.pen().color())

    def _draw_glow_segment(self, p: QPainter, a: QPointF, b: QPointF,
                           r0: float, r1: float, color: QColor, head_dir: str):
        """
        선 a->b 구간 중 [r0, r1] 만큼만 그라디언트로 밝게 칠함 (r0<=r1 in [0,1])
        """
        if r1 <= 0 or r0 >= 1 or r1 <= r0:
            return

        # 구간 점
        def lerp(P: QPointF, Q: QPointF, t: float) -> QPointF:
            return QPointF(P.x() + (Q.x() - P.x()) * t, P.y() + (Q.y() - P.y()) * t)

        s = lerp(a, b, r0)
        e = lerp(a, b, r1)

        # 그라디언트 (중앙 가장 밝게)
        grad = QLinearGradient(s, e)
        c0 = QColor(color)
        c1 = QColor(color).lighter(160)
        c2 = QColor(color)
        grad.setColorAt(0.0, c0)
        grad.setColorAt(0.5, c1)
        grad.setColorAt(1.0, c2)

        pen = QPen(QBrush(grad), 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)
        p.setPen(pen)
        p.drawLine(s, e)

        # 진행 방향 쪽 화살촉도 살짝 하이라이트
        p.setPen(QPen(QBrush(color.lighter(150)), 4))
        self._draw_arrow(p, s, e, head_dir=head_dir)
