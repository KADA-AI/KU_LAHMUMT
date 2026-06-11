"""Interactive QPainter-based visualization widgets for the algorithm
configuration tab.  Every widget here uses only PyQt5 + QPainter -- no
matplotlib or other external rendering libraries are required.

Imported by :pymod:`algo_config_tab` to provide real-time diagrams that
update whenever the user adjusts a parameter.
"""

from __future__ import annotations

import math
from typing import Optional

from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QFont,
    QFontMetrics,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPen,
    QPolygonF,
)
from PyQt5.QtWidgets import QLabel, QSizePolicy, QVBoxLayout, QWidget

# ---------------------------------------------------------------------------
# Colour palette (dark military-planning theme)
# ---------------------------------------------------------------------------
_BG = QColor("#1a1e2a")
_GRID = QColor("#2a3040")
_BLUE = QColor("#5b9cf6")
_BLUE_DIM = QColor("#3a6cc0")
_GREEN = QColor("#4ade80")
_GREEN_DIM = QColor("#22804a")
_ORANGE = QColor("#f97316")
_RED = QColor("#ef4444")
_RED_DIM = QColor("#991b1b")
_YELLOW = QColor("#facc15")
_CYAN = QColor("#22d3ee")
_WHITE = QColor("#e2e8f0")
_LABEL = QColor("#94a3b8")
_DIM_TEXT = QColor("#64748b")
_GROUND_FILL = QColor(34, 128, 74, 45)

_FONT_LABEL = QFont("Segoe UI", 8)
_FONT_LABEL.setHintingPreference(QFont.PreferNoHinting)
_FONT_VALUE = QFont("Segoe UI", 8, QFont.Bold)
_FONT_TITLE = QFont("Segoe UI", 9, QFont.Bold)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _nice_m(value: float) -> str:
    """Format a metre value for display."""
    if abs(value) >= 1000:
        return f"{value / 1000:.1f}km"
    return f"{value:.0f}m"


# ======================================================================
# Base class
# ======================================================================

class _BaseVisualizerWidget(QWidget):
    """Common base for all visualisation widgets."""

    _FIXED_HEIGHT = 220

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.setMinimumHeight(self._FIXED_HEIGHT)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setFixedHeight(self._FIXED_HEIGHT)

    # -- helpers -----------------------------------------------------------

    @staticmethod
    def _aa_painter(widget: QWidget) -> QPainter:
        p = QPainter(widget)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.setRenderHint(QPainter.TextAntialiasing, True)
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        return p

    def _fill_bg(self, p: QPainter) -> None:
        p.fillRect(self.rect(), _BG)

    def _draw_grid(self, p: QPainter, rows: int = 6, cols: int = 8) -> None:
        pen = QPen(_GRID, 1, Qt.DotLine)
        p.setPen(pen)
        w, h = self.width(), self.height()
        for r in range(1, rows):
            y = h * r / rows
            p.drawLine(QPointF(0, y), QPointF(w, y))
        for c in range(1, cols):
            x = w * c / cols
            p.drawLine(QPointF(x, 0), QPointF(x, h))

    @staticmethod
    def _draw_dim_line(
        p: QPainter,
        x1: float, y1: float,
        x2: float, y2: float,
        label: str,
        color: QColor = _ORANGE,
        offset: float = 0,
    ) -> None:
        """Draw a dimensioning line with end ticks and centred label."""
        pen = QPen(color, 1, Qt.DashLine)
        p.setPen(pen)
        dx, dy = x2 - x1, y2 - y1
        length = math.hypot(dx, dy)
        if length < 1:
            return
        nx, ny = -dy / length, dx / length
        ox, oy = nx * offset, ny * offset
        ax, ay = x1 + ox, y1 + oy
        bx, by = x2 + ox, y2 + oy
        p.drawLine(QPointF(ax, ay), QPointF(bx, by))
        # ticks
        tick = 4
        p.drawLine(QPointF(ax - nx * tick, ay - ny * tick),
                    QPointF(ax + nx * tick, ay + ny * tick))
        p.drawLine(QPointF(bx - nx * tick, by - ny * tick),
                    QPointF(bx + nx * tick, by + ny * tick))
        # label
        mx, my = (ax + bx) / 2, (ay + by) / 2
        p.setFont(_FONT_VALUE)
        fm = QFontMetrics(p.font())
        tw = fm.horizontalAdvance(label)
        p.setPen(color)
        p.drawText(QPointF(mx - tw / 2, my - 4), label)

    @staticmethod
    def _draw_label(
        p: QPainter,
        x: float, y: float,
        text: str,
        color: QColor = _LABEL,
        font: QFont = _FONT_LABEL,
        align: str = "center",
    ) -> None:
        p.setFont(font)
        p.setPen(color)
        fm = QFontMetrics(font)
        tw = fm.horizontalAdvance(text)
        if align == "center":
            p.drawText(QPointF(x - tw / 2, y), text)
        elif align == "left":
            p.drawText(QPointF(x, y), text)
        elif align == "right":
            p.drawText(QPointF(x - tw, y), text)

    @staticmethod
    def _draw_uav_icon(p: QPainter, cx: float, cy: float, size: float = 10,
                       color: QColor = _BLUE, heading_rad: float = -math.pi / 2) -> None:
        """Small triangle representing a UAV."""
        tri = QPolygonF()
        for i, angle in enumerate([0, 2.4, -2.4]):
            a = heading_rad + angle
            r = size if i == 0 else size * 0.6
            tri.append(QPointF(cx + r * math.cos(a), cy + r * math.sin(a)))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(color))
        p.drawPolygon(tri)

    @staticmethod
    def _draw_dot(p: QPainter, x: float, y: float, radius: float = 3,
                  color: QColor = _YELLOW) -> None:
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(color))
        p.drawEllipse(QPointF(x, y), radius, radius)


# ======================================================================
# 1. FovFootprintWidget -- FOV & Footprint 단면도
# ======================================================================

class FovFootprintWidget(_BaseVisualizerWidget):
    """Side-view cross-section: UAV at altitude, FOV cone, footprint on
    ground, sweep-line spacing, and sensor-to-target separation.
"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._fov_deg: float = 30.0
        self._altitude_m: float = 500.0
        self._density_scale: float = 1.0
        self._sep_scale: float = 1.0

    def update_params(self, fov_deg: float, altitude_m: float,
                      density_scale: float, sep_scale: float) -> None:
        self._fov_deg = _clamp(fov_deg, 1.0, 170.0)
        self._altitude_m = _clamp(altitude_m, 10.0, 20000.0)
        self._density_scale = _clamp(density_scale, 0.1, 10.0)
        self._sep_scale = _clamp(sep_scale, 0.0, 5.0)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        p = self._aa_painter(self)
        try:
            self._paint(p)
        finally:
            p.end()

    def _paint(self, p: QPainter) -> None:
        self._fill_bg(p)
        self._draw_grid(p)

        W, H = self.width(), self.height()
        margin_x, margin_top, margin_bot = 60, 30, 30
        draw_w = W - 2 * margin_x
        draw_h = H - margin_top - margin_bot

        ground_y = margin_top + draw_h
        uav_y = margin_top + 10
        uav_x = W / 2

        # --- ground ---
        ground_grad = QLinearGradient(0, ground_y, 0, H)
        ground_grad.setColorAt(0, _GREEN_DIM)
        ground_grad.setColorAt(1, QColor(34, 128, 74, 0))
        p.fillRect(QRectF(0, ground_y, W, H - ground_y), QBrush(ground_grad))
        p.setPen(QPen(_GREEN, 2))
        p.drawLine(QPointF(0, ground_y), QPointF(W, ground_y))

        # --- FOV geometry ---
        half_fov_rad = math.radians(self._fov_deg / 2)
        footprint_m = 2 * self._altitude_m * math.tan(half_fov_rad)
        # scale so footprint fills ~60% of draw_w
        max_foot_px = draw_w * 0.6
        if footprint_m > 0:
            scale = max_foot_px / footprint_m
        else:
            scale = 1.0
        foot_px = footprint_m * scale
        half_foot = foot_px / 2

        # UAV icon
        self._draw_uav_icon(p, uav_x, uav_y, 12, _BLUE, -math.pi / 2)

        # FOV cone
        cone_pen = QPen(_BLUE, 1.5, Qt.DashDotLine)
        p.setPen(cone_pen)
        left_ground = uav_x - half_foot
        right_ground = uav_x + half_foot
        p.drawLine(QPointF(uav_x, uav_y + 10), QPointF(left_ground, ground_y))
        p.drawLine(QPointF(uav_x, uav_y + 10), QPointF(right_ground, ground_y))

        # Footprint highlight
        p.setPen(QPen(_ORANGE, 3))
        p.drawLine(QPointF(left_ground, ground_y), QPointF(right_ground, ground_y))

        # Footprint label
        self._draw_label(p, uav_x, ground_y + 16,
                         f"Footprint {_nice_m(footprint_m)}", _ORANGE, _FONT_VALUE)

        # FOV angle label
        self._draw_label(p, uav_x, uav_y + 28,
                         f"FOV {self._fov_deg:.1f}\u00b0", _BLUE, _FONT_VALUE)

        # Altitude dimension line
        self._draw_dim_line(p, margin_x - 20, uav_y + 5,
                            margin_x - 20, ground_y,
                            f"\uace0\ub3c4 {_nice_m(self._altitude_m)}", _CYAN, 0)

        # --- sweep spacing (density) ---
        if self._density_scale > 0 and footprint_m > 0:
            sweep_spacing_m = footprint_m * self._density_scale
            sweep_spacing_px = sweep_spacing_m * scale
            if sweep_spacing_px > 4:
                n_lines = int(foot_px / sweep_spacing_px) + 3
                start_x = uav_x - (n_lines // 2) * sweep_spacing_px
                p.setPen(QPen(_YELLOW, 1, Qt.DashLine))
                for i in range(n_lines):
                    lx = start_x + i * sweep_spacing_px
                    if margin_x < lx < W - margin_x:
                        p.drawLine(QPointF(lx, ground_y - 4),
                                   QPointF(lx, ground_y + 4))
                # spacing label
                if n_lines >= 2:
                    lx0 = start_x + (n_lines // 2) * sweep_spacing_px
                    lx1 = lx0 + sweep_spacing_px
                    if lx1 < W - margin_x:
                        self._draw_dim_line(p, lx0, ground_y + 22,
                                            lx1, ground_y + 22,
                                            f"\ubc00\ub3c4 {_nice_m(sweep_spacing_m)}", _YELLOW, 0)

        # --- SEP distance ---
        if self._sep_scale > 0 and footprint_m > 0:
            sep_m = footprint_m * self._sep_scale * 0.5
            sep_px = sep_m * scale
            sep_px = _clamp(sep_px, 0, half_foot)
            sep_y = ground_y - 6
            p.setPen(QPen(_CYAN, 1.5, Qt.DotLine))
            p.drawLine(QPointF(uav_x, sep_y), QPointF(uav_x + sep_px, sep_y))
            self._draw_dot(p, uav_x + sep_px, sep_y, 3, _CYAN)
            self._draw_label(p, uav_x + sep_px / 2, sep_y - 6,
                             f"SEP {_nice_m(sep_m)}", _CYAN, _FONT_LABEL)

        # Title
        self._draw_label(p, W / 2, 16, "FOV & Footprint \ub2e8\uba74\ub3c4", _WHITE, _FONT_TITLE)


# ======================================================================
# 2. SweepPatternWidget -- Sweep 패턴 미리보기
# ======================================================================

class SweepPatternWidget(_BaseVisualizerWidget):
    """Top-down view of a sweep (zigzag) pattern inside a mission area."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._interp_points: int = 5
        self._density_scale: float = 1.0
        self._wp_interval_m: float = 500.0

    def update_params(self, interp_points: int, density_scale: float,
                      wp_interval_m: float) -> None:
        self._interp_points = int(_clamp(interp_points, 2, 30))
        self._density_scale = _clamp(density_scale, 0.1, 10.0)
        self._wp_interval_m = _clamp(wp_interval_m, 50.0, 50000.0)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        p = self._aa_painter(self)
        try:
            self._paint(p)
        finally:
            p.end()

    def _paint(self, p: QPainter) -> None:
        self._fill_bg(p)
        self._draw_grid(p)

        W, H = self.width(), self.height()
        margin = 40
        area_l = margin + 30
        area_t = margin
        area_w = W - 2 * area_l
        area_h = H - 2 * margin

        # Mission area rectangle
        p.setPen(QPen(_BLUE_DIM, 1.5, Qt.DashLine))
        p.setBrush(QBrush(QColor(91, 156, 246, 15)))
        p.drawRect(QRectF(area_l, area_t, area_w, area_h))

        # "임무 영역" label
        self._draw_label(p, area_l + area_w / 2, area_t - 6,
                         "\uc784\ubb34 \uc601\uc5ed", _BLUE_DIM, _FONT_LABEL)

        # Sweep lines
        spacing_factor = _clamp(self._density_scale, 0.2, 5.0)
        base_spacing = area_w * 0.12
        spacing = base_spacing * spacing_factor
        spacing = _clamp(spacing, 8, area_w * 0.45)
        n_sweeps = max(2, int(area_w / spacing) + 1)
        actual_spacing = area_w / max(n_sweeps - 1, 1) if n_sweeps > 1 else area_w

        sweep_path = QPainterPath()
        all_wp: list[QPointF] = []
        total_len_px = 0.0

        for i in range(n_sweeps):
            x = area_l + i * actual_spacing
            if i % 2 == 0:
                y_start, y_end = area_t + 6, area_t + area_h - 6
            else:
                y_start, y_end = area_t + area_h - 6, area_t + 6

            if i == 0:
                sweep_path.moveTo(x, y_start)
            else:
                # horizontal connector from previous sweep end
                prev_x = area_l + (i - 1) * actual_spacing
                prev_y_end = (area_t + area_h - 6) if (i - 1) % 2 == 0 else (area_t + 6)
                total_len_px += abs(x - prev_x)
                sweep_path.lineTo(x, prev_y_end)
                total_len_px += abs(y_end - prev_y_end) if prev_y_end != y_start else 0
                if prev_y_end != y_start:
                    sweep_path.lineTo(x, y_start)

            # Sweep line
            sweep_path.lineTo(x, y_end)
            seg_len_px = abs(y_end - y_start)
            total_len_px += seg_len_px

            # Interpolation dots along this leg
            for j in range(self._interp_points):
                t = j / max(self._interp_points - 1, 1)
                iy = y_start + t * (y_end - y_start)
                all_wp.append(QPointF(x, iy))

        # Draw sweep path
        p.setPen(QPen(_GREEN, 1.5))
        p.setBrush(Qt.NoBrush)
        p.drawPath(sweep_path)

        # WP dots along path at interval spacing
        # Estimate a "virtual" total sweep length in metres for WP placement
        virtual_total_m = n_sweeps * 2000.0  # representative
        scale_m_per_px = virtual_total_m / max(total_len_px, 1)
        wp_interval_px = self._wp_interval_m / max(scale_m_per_px, 0.01)
        wp_interval_px = _clamp(wp_interval_px, 6, area_h * 0.5)

        # Place WP dots along each vertical sweep leg
        for i in range(n_sweeps):
            x = area_l + i * actual_spacing
            if i % 2 == 0:
                y_s, y_e = area_t + 6, area_t + area_h - 6
            else:
                y_s, y_e = area_t + area_h - 6, area_t + 6
            leg_len = abs(y_e - y_s)
            n_wps = max(1, int(leg_len / wp_interval_px))
            for j in range(n_wps + 1):
                t = j / max(n_wps, 1)
                wy = y_s + t * (y_e - y_s)
                self._draw_dot(p, x, wy, 2.5, _YELLOW)

        # Interpolation points (larger, orange)
        for pt in all_wp:
            self._draw_dot(p, pt.x(), pt.y(), 3.5, _ORANGE)

        # Spacing dimension
        if n_sweeps >= 2:
            x0 = area_l
            x1 = area_l + actual_spacing
            self._draw_dim_line(p, x0, area_t + area_h + 16,
                                x1, area_t + area_h + 16,
                                f"\uac04\uaca9 \u00d7{self._density_scale:.2f}",
                                _ORANGE, 0)

        # Estimated total length
        est_len_m = total_len_px * scale_m_per_px
        self._draw_label(p, W - margin - 10, H - 8,
                         f"\ucd1d \uacbd\ub85c \u2248{_nice_m(est_len_m)}",
                         _DIM_TEXT, _FONT_LABEL, "right")

        # Stats
        self._draw_label(p, area_l, H - 8,
                         f"\ubcf4\uac04\uc810 {self._interp_points}  |  WP\uac04\uaca9 {_nice_m(self._wp_interval_m)}  |  Sweep {n_sweeps}\ubcf8",
                         _DIM_TEXT, _FONT_LABEL, "left")

        # Title
        self._draw_label(p, W / 2, 16, "Sweep \ud328\ud134 \ubbf8\ub9ac\ubcf4\uae30", _WHITE, _FONT_TITLE)


# ======================================================================
# 3. PriorMissionWidget -- 선행임무 접근형상 다이어그램
# ======================================================================

class PriorMissionWidget(_BaseVisualizerWidget):
    """Top-down tactical diagram: target, UAV approach path, offsets,
    loiter orbit, orientation arrow.
"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._base_offset_m: float = 500.0
        self._far_offset_m: float = 1500.0
        self._far_trigger_distance_m: float = 3000.0
        self._orientation_offset_m: float = 200.0
        self._loiter_seconds: float = 60.0
        self._approach_speed: float = 40.0

    def update_params(self, base_offset_m: float, far_offset_m: float,
                      far_trigger_distance_m: float,
                      orientation_offset_m: float, loiter_seconds: float,
                      approach_speed: float) -> None:
        self._base_offset_m = _clamp(base_offset_m, 0, 20000)
        self._far_offset_m = _clamp(far_offset_m, 0, 20000)
        self._far_trigger_distance_m = _clamp(far_trigger_distance_m, 0, 50000)
        self._orientation_offset_m = _clamp(orientation_offset_m, 0, 10000)
        self._loiter_seconds = _clamp(loiter_seconds, 0, 7200)
        self._approach_speed = _clamp(approach_speed, 0.1, 500)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        p = self._aa_painter(self)
        try:
            self._paint(p)
        finally:
            p.end()

    def _paint(self, p: QPainter) -> None:
        self._fill_bg(p)
        self._draw_grid(p)

        W, H = self.width(), self.height()
        cx = W * 0.5
        cy = H * 0.5

        # Scale: fit the largest offset circle in the widget
        max_offset = max(self._base_offset_m, self._far_offset_m,
                         self._far_trigger_distance_m * 0.5, 100)
        view_radius = min(W, H) * 0.38
        scale = view_radius / max_offset  # px per metre

        # --- target (red X) ---
        xsize = 7
        p.setPen(QPen(_RED, 2.5))
        p.drawLine(QPointF(cx - xsize, cy - xsize), QPointF(cx + xsize, cy + xsize))
        p.drawLine(QPointF(cx + xsize, cy - xsize), QPointF(cx - xsize, cy + xsize))
        self._draw_label(p, cx, cy - xsize - 4, "\ud45c\uc801", _RED, _FONT_LABEL)

        # --- base offset circle ---
        base_r = self._base_offset_m * scale
        if base_r > 2:
            p.setPen(QPen(_BLUE, 1.5, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), base_r, base_r)
            self._draw_label(p, cx + base_r + 4, cy - 4,
                             f"\uae30\ubcf8 {_nice_m(self._base_offset_m)}",
                             _BLUE, _FONT_LABEL, "left")

        # --- far offset circle ---
        far_r = self._far_offset_m * scale
        if far_r > 2 and abs(far_r - base_r) > 3:
            p.setPen(QPen(_CYAN, 1.0, Qt.DotLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(cx, cy), far_r, far_r)
            self._draw_label(p, cx - far_r - 4, cy + 12,
                             f"\uc6d0\uac70\ub9ac {_nice_m(self._far_offset_m)}",
                             _CYAN, _FONT_LABEL, "right")

        # --- far trigger distance arc (partial, top-right) ---
        trig_r = self._far_trigger_distance_m * scale
        if trig_r > 5:
            p.setPen(QPen(_DIM_TEXT, 1, Qt.DotLine))
            p.setBrush(Qt.NoBrush)
            arc_rect = QRectF(cx - trig_r, cy - trig_r, trig_r * 2, trig_r * 2)
            p.drawArc(arc_rect, 0, 90 * 16)  # top-right quadrant
            self._draw_label(p, cx + trig_r * 0.7, cy - trig_r * 0.7 - 4,
                             f"\uc804\ud658 {_nice_m(self._far_trigger_distance_m)}",
                             _DIM_TEXT, _FONT_LABEL, "left")

        # --- approach path (UAV from top-right) ---
        approach_angle = math.radians(-45)
        uav_dist_px = max(far_r, base_r) + 30
        uav_x = cx + uav_dist_px * math.cos(approach_angle)
        uav_y = cy + uav_dist_px * math.sin(approach_angle)

        # entry point on base offset circle
        entry_x = cx + base_r * math.cos(approach_angle)
        entry_y = cy + base_r * math.sin(approach_angle)

        # approach line: UAV -> entry
        p.setPen(QPen(_GREEN, 1.5))
        p.drawLine(QPointF(uav_x, uav_y), QPointF(entry_x, entry_y))
        # entry -> target
        p.setPen(QPen(_GREEN, 1.5, Qt.DashDotLine))
        p.drawLine(QPointF(entry_x, entry_y), QPointF(cx, cy))

        # entry dot
        self._draw_dot(p, entry_x, entry_y, 4, _GREEN)
        self._draw_label(p, entry_x + 8, entry_y - 8,
                         "\uc9c4\uc785\uc810", _GREEN, _FONT_LABEL, "left")

        # UAV icon
        heading = math.atan2(entry_y - uav_y, entry_x - uav_x)
        self._draw_uav_icon(p, uav_x, uav_y, 11, _BLUE, heading)
        self._draw_label(p, uav_x + 14, uav_y - 2, "UAV", _BLUE, _FONT_LABEL, "left")

        # --- loiter circle ---
        loiter_r = max(15, base_r * 0.25)
        p.setPen(QPen(_YELLOW, 1.5, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(cx, cy), loiter_r, loiter_r)
        # small arc arrow on loiter
        arrow_a = math.radians(30)
        ax = cx + loiter_r * math.cos(arrow_a)
        ay = cy - loiter_r * math.sin(arrow_a)
        self._draw_dot(p, ax, ay, 2, _YELLOW)
        self._draw_label(p, cx, cy + loiter_r + 14,
                         f"\uccb4\uacf5 {self._loiter_seconds:.0f}s",
                         _YELLOW, _FONT_VALUE)

        # --- orientation offset arrow ---
        if self._orientation_offset_m > 0:
            ori_px = self._orientation_offset_m * scale
            ori_px = _clamp(ori_px, 5, view_radius * 0.3)
            ori_angle = math.radians(0)  # east
            ox = cx + ori_px * math.cos(ori_angle)
            oy = cy + ori_px * math.sin(ori_angle)
            p.setPen(QPen(_ORANGE, 1.5))
            p.drawLine(QPointF(cx, cy), QPointF(ox, oy))
            # arrowhead
            ah_size = 5
            ah_angle = ori_angle + math.pi
            for da in [0.4, -0.4]:
                ahx = ox + ah_size * math.cos(ah_angle + da)
                ahy = oy + ah_size * math.sin(ah_angle + da)
                p.drawLine(QPointF(ox, oy), QPointF(ahx, ahy))
            self._draw_label(p, ox + 6, oy + 4,
                             f"\ubc29\ud5a5 {_nice_m(self._orientation_offset_m)}",
                             _ORANGE, _FONT_LABEL, "left")

        # Speed note
        self._draw_label(p, W - 10, H - 8,
                         f"\uc811\uadfc\uc18d\ub3c4 {self._approach_speed:.0f}m/s",
                         _DIM_TEXT, _FONT_LABEL, "right")

        # Title
        self._draw_label(p, W / 2, 16, "\uc120\ud589\uc784\ubb34 \uc811\uadfc\ud615\uc0c1",
                         _WHITE, _FONT_TITLE)


# ======================================================================
# 4. AttackProfileWidget -- 공격임무 단면도
# ======================================================================

class AttackProfileWidget(_BaseVisualizerWidget):
    """Side-view profile: attack aircraft approach, entry offset, attack
    point with altitude offset, resume, and tracking UAV orbit above.
"""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._entry_offset_m: float = 1000.0
        self._resume_offset_m: float = 800.0
        self._altitude_offset_m: float = 200.0
        self._lah_hold_seconds: float = 300.0
        self._tracking_eta_s: float = 10.0

    def update_params(self, entry_offset_m: float, resume_offset_m: float,
                      altitude_offset_m: float, lah_hold_seconds: float,
                      tracking_eta_s: float) -> None:
        self._entry_offset_m = _clamp(entry_offset_m, 0, 20000)
        self._resume_offset_m = _clamp(resume_offset_m, 0, 20000)
        self._altitude_offset_m = _clamp(altitude_offset_m, 0, 10000)
        self._lah_hold_seconds = _clamp(lah_hold_seconds, 0, 7200)
        self._tracking_eta_s = _clamp(tracking_eta_s, 0, 7200)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        p = self._aa_painter(self)
        try:
            self._paint(p)
        finally:
            p.end()

    def _paint(self, p: QPainter) -> None:
        self._fill_bg(p)
        self._draw_grid(p)

        W, H = self.width(), self.height()
        margin_x = 50
        margin_top = 35
        margin_bot = 30
        draw_w = W - 2 * margin_x
        draw_h = H - margin_top - margin_bot

        ground_y = margin_top + draw_h
        # Scale: horizontal — fit entry+resume offsets
        total_horiz_m = self._entry_offset_m + self._resume_offset_m + 200
        h_scale = draw_w / max(total_horiz_m, 1)  # px per m

        # Target on ground
        target_x = margin_x + self._entry_offset_m * h_scale
        target_x = _clamp(target_x, margin_x + 30, W - margin_x - 30)

        # --- ground ---
        ground_grad = QLinearGradient(0, ground_y, 0, H)
        ground_grad.setColorAt(0, _GREEN_DIM)
        ground_grad.setColorAt(1, QColor(34, 128, 74, 0))
        p.fillRect(QRectF(0, ground_y, W, H - ground_y), QBrush(ground_grad))
        p.setPen(QPen(_GREEN, 2))
        p.drawLine(QPointF(0, ground_y), QPointF(W, ground_y))

        # --- target triangle on ground ---
        t_size = 8
        target_tri = QPolygonF()
        target_tri.append(QPointF(target_x, ground_y - t_size))
        target_tri.append(QPointF(target_x - t_size * 0.7, ground_y))
        target_tri.append(QPointF(target_x + t_size * 0.7, ground_y))
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(_RED))
        p.drawPolygon(target_tri)
        self._draw_label(p, target_x, ground_y + 14,
                         "\ud45c\uc801", _RED, _FONT_LABEL)

        # --- LAH flight path ---
        # Positions
        entry_x = target_x - self._entry_offset_m * h_scale
        entry_x = _clamp(entry_x, margin_x, target_x - 10)
        resume_x = target_x + self._resume_offset_m * h_scale
        resume_x = _clamp(resume_x, target_x + 10, W - margin_x)
        approach_x = max(margin_x, entry_x - 40)

        cruise_y = margin_top + draw_h * 0.35
        alt_offset_px = _clamp(self._altitude_offset_m * h_scale, 0, draw_h * 0.3)
        attack_y = ground_y - alt_offset_px
        attack_y = _clamp(attack_y, margin_top + 10, ground_y - 5)

        # LAH hold position (near entry)
        hold_x = entry_x - 20
        hold_y = cruise_y

        # Flight path: approach -> hold -> entry -> attack point -> resume
        path = QPainterPath()
        path.moveTo(approach_x, cruise_y)
        path.lineTo(hold_x, hold_y)
        path.lineTo(entry_x, cruise_y)
        path.lineTo(target_x, attack_y)
        path.lineTo(resume_x, cruise_y)
        path.lineTo(min(W - margin_x + 20, resume_x + 40), cruise_y)

        p.setPen(QPen(_ORANGE, 2))
        p.setBrush(Qt.NoBrush)
        p.drawPath(path)

        # LAH icon at hold
        self._draw_uav_icon(p, hold_x, hold_y, 10, _ORANGE, 0)
        self._draw_label(p, hold_x, hold_y - 14,
                         f"LAH Hold {self._lah_hold_seconds:.0f}s",
                         _ORANGE, _FONT_VALUE)

        # Entry marker
        self._draw_dot(p, entry_x, cruise_y, 4, _CYAN)
        self._draw_label(p, entry_x, cruise_y - 10,
                         "\uc9c4\uc785", _CYAN, _FONT_LABEL)

        # Attack point marker
        self._draw_dot(p, target_x, attack_y, 5, _RED)
        self._draw_label(p, target_x + 10, attack_y - 4,
                         "\uacf5\uaca9\uc810", _RED, _FONT_LABEL, "left")

        # Resume marker
        self._draw_dot(p, resume_x, cruise_y, 4, _GREEN)
        self._draw_label(p, resume_x, cruise_y - 10,
                         "\ubcf5\uadc0", _GREEN, _FONT_LABEL)

        # --- dimension lines ---
        # Entry offset
        dim_y = ground_y + 22
        self._draw_dim_line(p, entry_x, dim_y, target_x, dim_y,
                            f"\uc9c4\uc785 {_nice_m(self._entry_offset_m)}", _CYAN, 0)
        # Resume offset
        self._draw_dim_line(p, target_x, dim_y, resume_x, dim_y,
                            f"\ubcf5\uadc0 {_nice_m(self._resume_offset_m)}", _GREEN, 0)
        # Altitude offset
        if alt_offset_px > 5:
            self._draw_dim_line(p, target_x + 20, attack_y,
                                target_x + 20, ground_y,
                                f"\uace0\ub3c4 {_nice_m(self._altitude_offset_m)}",
                                _YELLOW, 0)

        # --- UAV tracking orbit above target ---
        orbit_y = margin_top + 15
        orbit_rx = 22
        orbit_ry = 8
        p.setPen(QPen(_BLUE, 1.5, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(target_x, orbit_y), orbit_rx, orbit_ry)
        self._draw_uav_icon(p, target_x + orbit_rx, orbit_y, 8, _BLUE, 0)
        self._draw_label(p, target_x, orbit_y + orbit_ry + 12,
                         f"UAV \ucd94\uc801 ETA {self._tracking_eta_s:.0f}s",
                         _BLUE, _FONT_VALUE)

        # Dashed line from orbit to target
        p.setPen(QPen(_DIM_TEXT, 1, Qt.DotLine))
        p.drawLine(QPointF(target_x, orbit_y + orbit_ry),
                   QPointF(target_x, ground_y - t_size))

        # Title
        self._draw_label(p, W / 2, 16, "\uacf5\uaca9\uc784\ubb34 \ub2e8\uba74\ub3c4",
                         _WHITE, _FONT_TITLE)


# ======================================================================
# 5. DubinsTurnWidget -- Dubins 선회 미리보기
# ======================================================================

class DubinsTurnWidget(_BaseVisualizerWidget):
    """Top-down Dubins path (arc-straight-arc) with turn radius circles and
    waypoint dots at interval spacing."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._turn_radius_m: float = 500.0
        self._wp_interval_m: float = 300.0

    def update_params(self, turn_radius_m: float, wp_interval_m: float) -> None:
        self._turn_radius_m = _clamp(turn_radius_m, 10.0, 50000.0)
        self._wp_interval_m = _clamp(wp_interval_m, 10.0, 50000.0)
        self.update()

    def paintEvent(self, event) -> None:  # noqa: N802
        p = self._aa_painter(self)
        try:
            self._paint(p)
        finally:
            p.end()

    def _paint(self, p: QPainter) -> None:
        self._fill_bg(p)
        self._draw_grid(p)

        W, H = self.width(), self.height()
        margin = 40

        # Layout: start on left, end on right, both heading "right" with
        # an S-turn (LSL Dubins: left arc, straight, left arc)
        # We'll illustrate an RSR path for visual clarity.

        draw_w = W - 2 * margin
        draw_h = H - 2 * margin

        # Scale: radius should be ~20-30% of draw height
        desired_r_px = min(draw_h * 0.3, draw_w * 0.15)
        scale = desired_r_px / max(self._turn_radius_m, 1)  # px per m
        r_px = self._turn_radius_m * scale

        # Start and end points
        sx = margin + 30
        sy = H * 0.5
        ex = W - margin - 30
        ey = H * 0.5
        start_heading = 0  # heading east (radians)
        end_heading = 0

        # RSR Dubins: right circle at start, straight tangent, right circle at end
        # Start right circle centre (offset "right" = south for heading east)
        c1x = sx
        c1y = sy + r_px
        # End right circle centre
        c2x = ex
        c2y = ey + r_px

        # Tangent line between the two circles (external tangent for same-side)
        d = math.hypot(c2x - c1x, c2y - c1y)

        # Arc 1: from start point, turning clockwise on circle 1
        # Start angle on circle 1 (start point relative to centre)
        a1_start = math.atan2(sy - c1y, sx - c1x)  # should be -pi/2 (north)

        # Tangent points
        if d > 0:
            theta = math.atan2(c2y - c1y, c2x - c1x)
        else:
            theta = 0
        # For RSR, tangent angle = theta (same radius, external)
        tang_angle = theta - math.pi / 2  # perpendicular

        t1x = c1x + r_px * math.cos(theta - math.pi / 2)
        t1y = c1y + r_px * math.sin(theta - math.pi / 2)
        t2x = c2x + r_px * math.cos(theta - math.pi / 2)
        t2y = c2y + r_px * math.sin(theta - math.pi / 2)

        a1_end = math.atan2(t1y - c1y, t1x - c1x)
        a2_start_angle = math.atan2(t2y - c2y, t2x - c2x)
        a2_end = math.atan2(ey - c2y, ex - c2x)

        # Draw turn circles (faint)
        p.setPen(QPen(_GRID, 1, Qt.DotLine))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(QPointF(c1x, c1y), r_px, r_px)
        p.drawEllipse(QPointF(c2x, c2y), r_px, r_px)

        # radius labels
        self._draw_label(p, c1x, c1y + 4,
                         f"R={_nice_m(self._turn_radius_m)}", _DIM_TEXT, _FONT_LABEL)
        self._draw_label(p, c2x, c2y + 4,
                         f"R={_nice_m(self._turn_radius_m)}", _DIM_TEXT, _FONT_LABEL)

        # Build Dubins path as points for drawing and WP placement
        path_points: list[QPointF] = []

        # Arc 1 (clockwise = decreasing angle)
        arc1_start_a = a1_start
        arc1_end_a = a1_end
        # Normalise for clockwise
        if arc1_end_a > arc1_start_a:
            arc1_end_a -= 2 * math.pi
        n_arc1 = max(12, int(abs(arc1_start_a - arc1_end_a) / (math.pi / 30)))
        for i in range(n_arc1 + 1):
            t = i / max(n_arc1, 1)
            a = arc1_start_a + t * (arc1_end_a - arc1_start_a)
            path_points.append(QPointF(c1x + r_px * math.cos(a),
                                       c1y + r_px * math.sin(a)))

        # Straight segment
        straight_len = math.hypot(t2x - t1x, t2y - t1y)
        n_straight = max(2, int(straight_len / 3))
        for i in range(n_straight + 1):
            t = i / max(n_straight, 1)
            path_points.append(QPointF(t1x + t * (t2x - t1x),
                                       t1y + t * (t2y - t1y)))

        # Arc 2 (clockwise)
        arc2_start_a = a2_start_angle
        arc2_end_a = a2_end
        if arc2_end_a > arc2_start_a:
            arc2_end_a -= 2 * math.pi
        n_arc2 = max(12, int(abs(arc2_start_a - arc2_end_a) / (math.pi / 30)))
        for i in range(n_arc2 + 1):
            t = i / max(n_arc2, 1)
            a = arc2_start_a + t * (arc2_end_a - arc2_start_a)
            path_points.append(QPointF(c2x + r_px * math.cos(a),
                                       c2y + r_px * math.sin(a)))

        # Draw path
        if path_points:
            dubins_path = QPainterPath()
            dubins_path.moveTo(path_points[0])
            for pt in path_points[1:]:
                dubins_path.lineTo(pt)
            p.setPen(QPen(_GREEN, 2))
            p.setBrush(Qt.NoBrush)
            p.drawPath(dubins_path)

        # Compute cumulative arc length along path for WP placement
        cum_len = [0.0]
        for i in range(1, len(path_points)):
            dx = path_points[i].x() - path_points[i - 1].x()
            dy = path_points[i].y() - path_points[i - 1].y()
            cum_len.append(cum_len[-1] + math.hypot(dx, dy))
        total_path_px = cum_len[-1] if cum_len else 0

        # WP interval in px
        total_path_m = total_path_px / max(scale, 1e-9)
        wp_interval_px = self._wp_interval_m * scale
        wp_interval_px = _clamp(wp_interval_px, 5, total_path_px * 0.5) if total_path_px > 0 else 20

        # Place WP dots
        if total_path_px > 0:
            d_accum = 0.0
            seg_idx = 0
            while d_accum <= total_path_px and seg_idx < len(path_points) - 1:
                # find segment containing d_accum
                while seg_idx < len(cum_len) - 1 and cum_len[seg_idx + 1] < d_accum:
                    seg_idx += 1
                if seg_idx >= len(path_points) - 1:
                    break
                seg_start = cum_len[seg_idx]
                seg_end = cum_len[seg_idx + 1]
                seg_len = seg_end - seg_start
                if seg_len > 0:
                    frac = (d_accum - seg_start) / seg_len
                else:
                    frac = 0
                frac = _clamp(frac, 0, 1)
                wx = path_points[seg_idx].x() + frac * (path_points[seg_idx + 1].x() - path_points[seg_idx].x())
                wy = path_points[seg_idx].y() + frac * (path_points[seg_idx + 1].y() - path_points[seg_idx].y())
                self._draw_dot(p, wx, wy, 3, _YELLOW)
                d_accum += wp_interval_px

        # Start and end markers
        self._draw_uav_icon(p, sx, sy, 11, _BLUE, start_heading)
        self._draw_label(p, sx, sy - 16, "\uc2dc\uc791", _BLUE, _FONT_VALUE)

        # heading arrow from start
        arr_len = 18
        p.setPen(QPen(_BLUE, 1.5))
        p.drawLine(QPointF(sx + 12, sy),
                   QPointF(sx + 12 + arr_len, sy))
        # small arrowhead
        p.drawLine(QPointF(sx + 12 + arr_len, sy),
                   QPointF(sx + 12 + arr_len - 5, sy - 3))
        p.drawLine(QPointF(sx + 12 + arr_len, sy),
                   QPointF(sx + 12 + arr_len - 5, sy + 3))

        # End marker
        self._draw_uav_icon(p, ex, ey, 11, _CYAN, end_heading)
        self._draw_label(p, ex, ey - 16, "\ub05d", _CYAN, _FONT_VALUE)

        # heading arrow from end
        p.setPen(QPen(_CYAN, 1.5))
        p.drawLine(QPointF(ex + 12, ey),
                   QPointF(ex + 12 + arr_len, ey))
        p.drawLine(QPointF(ex + 12 + arr_len, ey),
                   QPointF(ex + 12 + arr_len - 5, ey - 3))
        p.drawLine(QPointF(ex + 12 + arr_len, ey),
                   QPointF(ex + 12 + arr_len - 5, ey + 3))

        # Scale bar (bottom left)
        bar_m = self._turn_radius_m
        bar_px = bar_m * scale
        bar_px = _clamp(bar_px, 20, draw_w * 0.3)
        bar_y = H - margin + 10
        bar_x = margin
        p.setPen(QPen(_LABEL, 1.5))
        p.drawLine(QPointF(bar_x, bar_y), QPointF(bar_x + bar_px, bar_y))
        p.drawLine(QPointF(bar_x, bar_y - 3), QPointF(bar_x, bar_y + 3))
        p.drawLine(QPointF(bar_x + bar_px, bar_y - 3),
                   QPointF(bar_x + bar_px, bar_y + 3))
        self._draw_label(p, bar_x + bar_px / 2, bar_y - 6,
                         _nice_m(bar_m), _LABEL, _FONT_LABEL)

        # Stats
        self._draw_label(p, W - 10, H - 8,
                         f"WP\uac04\uaca9 {_nice_m(self._wp_interval_m)}  |  \ucd1d \u2248{_nice_m(total_path_m)}",
                         _DIM_TEXT, _FONT_LABEL, "right")

        # Title
        self._draw_label(p, W / 2, 16, "Dubins \uc120\ud68c \ubbf8\ub9ac\ubcf4\uae30",
                         _WHITE, _FONT_TITLE)


# ======================================================================
# ConfigVisualizerPanel -- container with section titles
# ======================================================================

class ConfigVisualizerPanel(QWidget):
    """Vertical stack of titled visualiser widgets.  Provides a compact
    layout suitable for embedding in a sidebar or collapsible section."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(4)

    def add_visualizer(self, title: str, widget: QWidget) -> None:
        """Add *widget* under a compact section header *title*."""
        label = QLabel(title)
        label.setFont(_FONT_TITLE)
        label.setStyleSheet(
            "QLabel { color: #94a3b8; background: transparent;"
            " padding: 2px 4px; }"
        )
        label.setFixedHeight(20)
        self._layout.addWidget(label)
        self._layout.addWidget(widget)

    def add_stretch(self) -> None:
        self._layout.addStretch(1)
