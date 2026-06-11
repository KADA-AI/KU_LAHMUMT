from __future__ import annotations

import math

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QFontMetrics, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from modules.monitoring.logic.turn_radius_monitor import (
    AircraftTurnView,
    TurnRadiusMonitorStore,
    _math_rad_to_heading_deg,
)
from modules.monitoring.logic.replan_runtime_settings import get_path_deviation_settings


DEFAULT_HALF_RANGE_M = 500.0
MIN_HALF_RANGE_M = 120.0
MAX_HALF_RANGE_M = 6000.0
AIRCRAFT_SIZE_PX = 18.0


def _format_meters(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.0f} m"


def _format_speed(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.1f} m/s"


def _format_heading(raw_heading_deg: float | None, heading_rad: float | None) -> str:
    if raw_heading_deg is not None:
        return f"{raw_heading_deg:05.1f} deg"
    if heading_rad is not None:
        return f"{_math_rad_to_heading_deg(heading_rad):05.1f} deg"
    return "-"


def _format_turn_rate(turn_rate_dps: float | None) -> str:
    if turn_rate_dps is None:
        return "-"
    return f"{turn_rate_dps:+.2f} deg/s"


def _format_age(age_s: float | None) -> str:
    if age_s is None:
        return "-"
    return f"{age_s:.1f} s"


def _format_scale(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.3f}x"


def _format_roll(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value:.1f} deg"


def _format_adaptive_debug(view: AircraftTurnView) -> str:
    sample_count = int(getattr(view, "adaptive_sample_count", 0) or 0)
    radius_scale = _format_scale(getattr(view, "adaptive_radius_scale", None))
    threshold_scale = _format_scale(getattr(view, "adaptive_threshold_scale", None))
    roll = _format_roll(getattr(view, "adaptive_expected_roll_deg", None))
    return f"r {radius_scale} / th {threshold_scale} / roll {roll} / n {sample_count}"


def _format_waypoint_id(value: int | None) -> str:
    if value is None:
        return "-"
    return str(int(value))


def _format_waypoint_pass_type(value: int | None) -> str:
    try:
        pass_type = int(value) if value is not None else 0
    except Exception:
        pass_type = 0
    if pass_type == 1:
        return "Fly By"
    if pass_type == 2:
        return "Loiter"
    if pass_type == 3:
        return "Fly Over"
    if pass_type <= 0:
        return "-"
    return f"Type {pass_type}"


def _format_current_waypoint(value: int | None, waypoint_pass_type: int | None) -> str:
    waypoint_text = _format_waypoint_id(value)
    pass_type_text = _format_waypoint_pass_type(waypoint_pass_type)
    if waypoint_text == "-":
        return "-"
    if pass_type_text == "-":
        return waypoint_text
    return f"{waypoint_text} / {pass_type_text}"


def _waypoint_pass_type_palette(value: int | None) -> tuple[QColor, QColor, QColor]:
    try:
        pass_type = int(value) if value is not None else 0
    except Exception:
        pass_type = 0
    if pass_type == 1:
        return QColor("#0f766e"), QColor(20, 184, 166, 66), QColor("#115e59")
    if pass_type == 2:
        return QColor("#7c3aed"), QColor(167, 139, 250, 66), QColor("#5b21b6")
    if pass_type == 3:
        return QColor("#c2410c"), QColor(251, 146, 60, 72), QColor("#9a3412")
    return QColor("#2563eb"), QColor(96, 165, 250, 66), QColor("#1d4ed8")


def _format_spiral_score(view: AircraftTurnView) -> str:
    if view.spiral_state == "warning":
        return "경고"
    if view.spiral_state == "watch":
        return "주의"
    if view.current_waypoint_is_loiter:
        return "Loiter WP"
    return "정상"


def _turn_status_text(view: AircraftTurnView) -> str:
    stale_timeout_s = float(get_path_deviation_settings().get("stale_timeout_s", 5.0))
    if not view.has_data:
        return "데이터 없음"
    if view.age_s is not None and view.age_s > stale_timeout_s:
        return "수신 지연"
    if not view.is_turning or view.turn_sign == 0:
        return "직선 비행"
    return "좌선회" if view.turn_sign > 0 else "우선회"


def _nice_grid_step_m(half_range_m: float) -> float:
    candidates = (20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0)
    target = max(20.0, half_range_m / 4.0)
    for candidate in candidates:
        if candidate >= target:
            return candidate
    return candidates[-1]


def _scale_bar_m(half_range_m: float) -> float:
    candidates = (50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0)
    target = max(50.0, half_range_m / 5.0)
    chosen = candidates[0]
    for candidate in candidates:
        if candidate <= target:
            chosen = candidate
    return chosen


class MetricCard(QFrame):
    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("turnRadiusMetricCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(4)

        self._title = QLabel(title)
        self._title.setObjectName("turnRadiusMetricTitle")
        self._value = QLabel("-")
        self._value.setObjectName("turnRadiusMetricValue")
        layout.addWidget(self._title)
        layout.addWidget(self._value)

    def set_value(self, value: str) -> None:
        self._value.setText(value)


class TurnRadiusMonitorCanvas(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(680)
        self._view = AircraftTurnView(
            aircraft_id=4,
            label="UAV1",
            has_data=False,
            actual_points=[],
            ideal_points=[],
            position_m=None,
            position_coordinate=None,
            heading_rad=None,
            raw_heading_deg=None,
            speed_mps=None,
            turn_rate_dps=None,
            ideal_radius_m=None,
            actual_radius_m=None,
            turn_sign=0,
            turn_circle_center_m=None,
            turn_circle_radius_m=None,
            current_waypoint_id=None,
            current_waypoint_pass_type=None,
            flying=None,
            current_waypoint_position_m=None,
            alternate_waypoint_id=None,
            alternate_waypoint_position_m=None,
            alternate_waypoint_coordinate=None,
            alternate_waypoint_eta_s=None,
            predicted_entry_coordinate=None,
            predicted_entry_eta_s=None,
            current_waypoint_is_loiter=False,
            spiral_state="none",
            spiral_score=None,
            spiral_angle_deg=None,
            spiral_radius_m=None,
            spiral_radial_span_m=None,
            age_s=None,
            is_turning=False,
        )
        self._half_range_m = DEFAULT_HALF_RANGE_M
        self._follow_aircraft = True
        self._manual_center_m = (0.0, 0.0)
        self._dragging = False
        self._drag_last_pos: QPoint | None = None
        self._adaptive_debug_enabled = False

    def set_view(self, view: AircraftTurnView) -> None:
        self._view = view
        if self._follow_aircraft and view.position_m is not None:
            self._manual_center_m = view.position_m
        self.update()

    def set_adaptive_debug_enabled(self, enabled: bool) -> None:
        self._adaptive_debug_enabled = bool(enabled)
        self.update()

    def set_follow_aircraft(self, enabled: bool) -> None:
        self._follow_aircraft = bool(enabled)
        if self._follow_aircraft and self._view.position_m is not None:
            self._manual_center_m = self._view.position_m
        self.update()

    def reset_view(self) -> None:
        self._half_range_m = DEFAULT_HALF_RANGE_M
        if self._view.position_m is not None:
            self._manual_center_m = self._view.position_m
        self.update()

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        step = 0.88 if event.angleDelta().y() > 0 else 1.14
        self._half_range_m = max(MIN_HALF_RANGE_M, min(MAX_HALF_RANGE_M, self._half_range_m * step))
        self.update()

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._drag_last_pos = event.pos()
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
        if not self._dragging or self._drag_last_pos is None:
            super().mouseMoveEvent(event)
            return
        if self._follow_aircraft:
            super().mouseMoveEvent(event)
            return
        center = self._current_center_m()
        ppm = self._pixels_per_meter()
        if ppm <= 0.0:
            return
        dx_px = event.pos().x() - self._drag_last_pos.x()
        dy_px = event.pos().y() - self._drag_last_pos.y()
        self._manual_center_m = (
            center[0] - (dx_px / ppm),
            center[1] + (dy_px / ppm),
        )
        self._drag_last_pos = event.pos()
        self.update()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            self._dragging = False
            self._drag_last_pos = None
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event) -> None:  # type: ignore[override]
        self._follow_aircraft = True
        if self._view.position_m is not None:
            self._manual_center_m = self._view.position_m
        self.update()
        super().mouseDoubleClickEvent(event)

    def _draw_rect(self) -> QRectF:
        return QRectF(self.rect()).adjusted(24.0, 24.0, -24.0, -24.0)

    def _pixels_per_meter(self) -> float:
        draw_rect = self._draw_rect()
        usable = max(1.0, min(draw_rect.width(), draw_rect.height()))
        return usable / max(1.0, self._half_range_m * 2.0)

    def _current_center_m(self) -> tuple[float, float]:
        if self._follow_aircraft and self._view.position_m is not None:
            return self._view.position_m
        return self._manual_center_m

    def _world_to_screen(self, point_m: tuple[float, float]) -> QPointF:
        draw_rect = self._draw_rect()
        center_m = self._current_center_m()
        ppm = self._pixels_per_meter()
        return QPointF(
            draw_rect.center().x() + ((point_m[0] - center_m[0]) * ppm),
            draw_rect.center().y() - ((point_m[1] - center_m[1]) * ppm),
        )

    def _draw_grid(self, painter: QPainter) -> None:
        draw_rect = self._draw_rect()
        center_m = self._current_center_m()
        step_m = _nice_grid_step_m(self._half_range_m)
        ppm = self._pixels_per_meter()
        left_world = center_m[0] - self._half_range_m
        right_world = center_m[0] + self._half_range_m
        bottom_world = center_m[1] - self._half_range_m
        top_world = center_m[1] + self._half_range_m

        painter.save()
        painter.setPen(QPen(QColor("#dbe5ef"), 1))
        x_m = math.floor(left_world / step_m) * step_m
        while x_m <= right_world + 0.1:
            x_px = draw_rect.center().x() + ((x_m - center_m[0]) * ppm)
            painter.drawLine(QPointF(x_px, draw_rect.top()), QPointF(x_px, draw_rect.bottom()))
            x_m += step_m
        y_m = math.floor(bottom_world / step_m) * step_m
        while y_m <= top_world + 0.1:
            y_px = draw_rect.center().y() - ((y_m - center_m[1]) * ppm)
            painter.drawLine(QPointF(draw_rect.left(), y_px), QPointF(draw_rect.right(), y_px))
            y_m += step_m

        painter.setPen(QPen(QColor("#94a3b8"), 2))
        x0 = draw_rect.center().x() + ((0.0 - center_m[0]) * ppm)
        y0 = draw_rect.center().y() - ((0.0 - center_m[1]) * ppm)
        painter.drawLine(QPointF(x0, draw_rect.top()), QPointF(x0, draw_rect.bottom()))
        painter.drawLine(QPointF(draw_rect.left(), y0), QPointF(draw_rect.right(), y0))
        painter.restore()

    def _draw_path(
        self,
        painter: QPainter,
        points: list[tuple[float, float]],
        color: str,
        width: int,
        dashed: bool = False,
    ) -> None:
        if len(points) < 2:
            return
        max_points = max(96, int(self._draw_rect().width() // 2))
        draw_points = points
        if len(draw_points) > max_points:
            step = max(1, int(math.ceil(len(draw_points) / max_points)))
            draw_points = draw_points[::step]
            if draw_points[-1] != points[-1]:
                draw_points = list(draw_points) + [points[-1]]
        if len(draw_points) < 2:
            return
        poly = QPolygonF()
        for point in draw_points:
            poly.append(self._world_to_screen(point))
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        pen = QPen(QColor(color), width)
        if dashed:
            pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.drawPolyline(poly)
        painter.restore()

    def _draw_turn_circle(self, painter: QPainter) -> None:
        view = self._view
        if not view.is_turning or view.turn_circle_center_m is None or view.turn_circle_radius_m is None:
            return
        ppm = self._pixels_per_meter()
        center = self._world_to_screen(view.turn_circle_center_m)
        painter.save()
        pen = QPen(QColor("#f97316"), 2)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, view.turn_circle_radius_m * ppm, view.turn_circle_radius_m * ppm)
        painter.restore()

    def _draw_waypoint_badge(
        self,
        painter: QPainter,
        *,
        anchor: QPointF,
        text: str,
        border: QColor,
        fill: QColor,
        text_color: QColor,
    ) -> None:
        font = QFont("Malgun Gothic", 9)
        metrics = QFontMetrics(font)
        text_width = max(56, metrics.horizontalAdvance(text))
        badge_rect = QRectF(anchor.x() + 12.0, anchor.y() - 34.0, text_width + 18.0, 24.0)
        badge_fill = QColor(fill)
        badge_fill.setAlpha(max(190, fill.alpha()))

        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setFont(font)
        painter.setPen(QPen(border, 1))
        painter.setBrush(badge_fill)
        painter.drawRoundedRect(badge_rect, 8.0, 8.0)
        painter.setPen(text_color)
        painter.drawText(
            badge_rect.adjusted(9.0, 0.0, -9.0, 0.0),
            Qt.AlignVCenter | Qt.AlignLeft,
            text,
        )
        painter.restore()

    def _draw_current_waypoint(self, painter: QPainter) -> None:
        view = self._view
        if view.current_waypoint_position_m is None:
            return
        point = self._world_to_screen(view.current_waypoint_position_m)
        color, fill, text_color = _waypoint_pass_type_palette(view.current_waypoint_pass_type)
        if view.spiral_state == "watch":
            color = QColor("#f59e0b")
            fill = QColor(245, 158, 11, 85)
            text_color = QColor("#92400e")
        elif view.spiral_state == "warning":
            color = QColor("#ef4444")
            fill = QColor(239, 68, 68, 90)
            text_color = QColor("#991b1b")
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(color, 2))
        painter.setBrush(fill)
        painter.drawEllipse(point, 7.0, 7.0)
        painter.drawLine(QPointF(point.x() - 10.0, point.y()), QPointF(point.x() + 10.0, point.y()))
        painter.drawLine(QPointF(point.x(), point.y() - 10.0), QPointF(point.x(), point.y() + 10.0))
        painter.restore()
        waypoint_text = f"WP {view.current_waypoint_id if view.current_waypoint_id is not None else '?'}"
        pass_type_text = _format_waypoint_pass_type(view.current_waypoint_pass_type)
        badge_text = waypoint_text if pass_type_text == "-" else f"{waypoint_text}  {pass_type_text}"
        self._draw_waypoint_badge(
            painter,
            anchor=point,
            text=badge_text,
            border=color,
            fill=fill,
            text_color=text_color,
        )

    def _draw_alternate_waypoint(self, painter: QPainter) -> None:
        view = self._view
        if view.alternate_waypoint_position_m is None or view.alternate_waypoint_id is None:
            return
        point = self._world_to_screen(view.alternate_waypoint_position_m)
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        color = QColor("#ca8a04")
        fill = QColor(250, 204, 21, 88)
        painter.setPen(QPen(color, 2))
        painter.setBrush(fill)
        diamond = QPolygonF(
            [
                QPointF(point.x(), point.y() - 8.0),
                QPointF(point.x() + 8.0, point.y()),
                QPointF(point.x(), point.y() + 8.0),
                QPointF(point.x() - 8.0, point.y()),
            ]
        )
        painter.drawPolygon(diamond)
        if view.current_waypoint_position_m is not None:
            painter.setPen(QPen(QColor("#ca8a04"), 1, Qt.DashLine))
            painter.drawLine(point, self._world_to_screen(view.current_waypoint_position_m))
        painter.setPen(QColor("#854d0e"))
        painter.setFont(QFont("Malgun Gothic", 9))
        eta_text = ""
        if view.alternate_waypoint_eta_s is not None:
            eta_text = f" (+{view.alternate_waypoint_eta_s:.0f}s)"
        painter.drawText(
            QPointF(point.x() + 12.0, point.y() - 10.0),
            f"ALT WP {view.alternate_waypoint_id}{eta_text}",
        )
        painter.restore()

    def _draw_aircraft(self, painter: QPainter) -> None:
        view = self._view
        if view.position_m is None:
            return
        center = self._world_to_screen(view.position_m)
        heading_rad = view.heading_rad if view.heading_rad is not None else 0.0
        ux = math.cos(heading_rad)
        uy = -math.sin(heading_rad)
        vx = -uy
        vy = ux
        tip = QPointF(center.x() + (ux * AIRCRAFT_SIZE_PX), center.y() + (uy * AIRCRAFT_SIZE_PX))
        left = QPointF(
            center.x() - (ux * AIRCRAFT_SIZE_PX * 0.55) + (vx * AIRCRAFT_SIZE_PX * 0.65),
            center.y() - (uy * AIRCRAFT_SIZE_PX * 0.55) + (vy * AIRCRAFT_SIZE_PX * 0.65),
        )
        right = QPointF(
            center.x() - (ux * AIRCRAFT_SIZE_PX * 0.55) - (vx * AIRCRAFT_SIZE_PX * 0.65),
            center.y() - (uy * AIRCRAFT_SIZE_PX * 0.55) - (vy * AIRCRAFT_SIZE_PX * 0.65),
        )

        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(QColor("#eff6ff"), 2))
        painter.setBrush(QColor("#2563eb"))
        painter.drawPolygon(QPolygonF([tip, left, right]))
        painter.setPen(QPen(QColor("#38bdf8"), 2))
        painter.drawLine(center, QPointF(center.x() + (ux * 120.0), center.y() + (uy * 120.0)))
        painter.restore()

    def _draw_scale(self, painter: QPainter) -> None:
        draw_rect = self._draw_rect()
        ppm = self._pixels_per_meter()
        length_m = _scale_bar_m(self._half_range_m)
        length_px = length_m * ppm

        painter.save()
        painter.setPen(QPen(QColor("#64748b"), 2))
        x0 = draw_rect.left() + 18.0
        y0 = draw_rect.bottom() - 18.0
        painter.drawLine(QPointF(x0, y0), QPointF(x0 + length_px, y0))
        painter.drawLine(QPointF(x0, y0 - 6.0), QPointF(x0, y0 + 6.0))
        painter.drawLine(QPointF(x0 + length_px, y0 - 6.0), QPointF(x0 + length_px, y0 + 6.0))
        painter.setFont(QFont("Consolas", 10))
        painter.drawText(QPointF(x0, y0 - 10.0), f"{int(length_m)} m")
        painter.restore()

    def _draw_hud(self, painter: QPainter) -> None:
        draw_rect = self._draw_rect()
        hud_rect = QRectF(draw_rect.left() + 14.0, draw_rect.top() + 14.0, 404.0, 158.0)

        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(QColor("#d7e2ee"), 1))
        painter.setBrush(QColor(255, 255, 255, 240))
        painter.drawRoundedRect(hud_rect, 12.0, 12.0)

        label_font = QFont("Malgun Gothic", 9)
        value_font = QFont("Malgun Gothic", 9)
        row_height = 24.0
        label_x = hud_rect.left() + 16.0
        colon_x = hud_rect.left() + 104.0
        value_x = hud_rect.left() + 118.0
        rows = [
            ("Wheel", "확대 / 축소"),
            ("Drag", "화면 이동 (follow off)"),
            ("DoubleClick", "기체 중심 복귀"),
            ("View", f"+/- {int(self._half_range_m)} m"),
        ]
        for index, (label, value) in enumerate(rows):
            y = hud_rect.top() + 18.0 + (index * row_height)
            row_rect = QRectF(hud_rect.left() + 12.0, y, hud_rect.width() - 24.0, 18.0)
            painter.setFont(label_font)
            painter.setPen(QColor("#0f172a"))
            painter.drawText(
                QRectF(label_x, row_rect.top(), colon_x - label_x - 8.0, row_rect.height()),
                Qt.AlignLeft | Qt.AlignVCenter,
                label,
            )
            painter.setPen(QColor("#64748b"))
            painter.drawText(
                QRectF(colon_x, row_rect.top(), 10.0, row_rect.height()),
                Qt.AlignCenter,
                ":",
            )
            painter.setFont(value_font)
            painter.setPen(QColor("#334155"))
            painter.drawText(
                QRectF(value_x, row_rect.top(), hud_rect.right() - value_x - 14.0, row_rect.height()),
                Qt.AlignLeft | Qt.AlignVCenter,
                value,
            )

        legend_font = QFont("Malgun Gothic", 9)
        left_marker_x = hud_rect.left() + 20.0
        left_text_x = hud_rect.left() + 70.0
        right_marker_x = hud_rect.left() + 210.0
        right_text_x = hud_rect.left() + 232.0
        legend_row_1 = hud_rect.bottom() - 44.0
        legend_row_2 = hud_rect.bottom() - 21.0

        painter.setFont(legend_font)
        painter.setPen(QPen(QColor("#0f766e"), 3))
        painter.drawLine(QPointF(left_marker_x, legend_row_1), QPointF(left_marker_x + 40.0, legend_row_1))
        painter.setPen(QColor("#334155"))
        painter.drawText(
            QRectF(left_text_x, legend_row_1 - 10.0, 108.0, 18.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            "실제 경로",
        )

        painter.setPen(QPen(QColor("#f97316"), 3, Qt.DashLine))
        painter.drawLine(QPointF(left_marker_x, legend_row_2), QPointF(left_marker_x + 40.0, legend_row_2))
        painter.setPen(QColor("#334155"))
        painter.drawText(
            QRectF(left_text_x, legend_row_2 - 10.0, 118.0, 18.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            "기준 선회경로",
        )

        wp_color, _wp_fill, _wp_text = _waypoint_pass_type_palette(self._view.current_waypoint_pass_type)
        if self._view.spiral_state == "watch":
            wp_color = QColor("#f59e0b")
        elif self._view.spiral_state == "warning":
            wp_color = QColor("#ef4444")
        painter.setPen(QPen(wp_color, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(QPointF(right_marker_x, legend_row_1), 4.0, 4.0)
        painter.drawLine(QPointF(right_marker_x - 8.0, legend_row_1), QPointF(right_marker_x + 8.0, legend_row_1))
        painter.drawLine(QPointF(right_marker_x, legend_row_1 - 8.0), QPointF(right_marker_x, legend_row_1 + 8.0))
        painter.setPen(QColor("#334155"))
        current_wp_label = "현재 WP"
        current_wp_type = _format_waypoint_pass_type(self._view.current_waypoint_pass_type)
        if current_wp_type != "-":
            current_wp_label += f" ({current_wp_type})"
        painter.drawText(
            QRectF(right_text_x, legend_row_1 - 10.0, hud_rect.right() - right_text_x - 12.0, 18.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            current_wp_label,
        )

        painter.setPen(QPen(QColor("#ca8a04"), 2))
        diamond = QPolygonF(
            [
                QPointF(right_marker_x, legend_row_2 - 8.0),
                QPointF(right_marker_x + 8.0, legend_row_2),
                QPointF(right_marker_x, legend_row_2 + 8.0),
                QPointF(right_marker_x - 8.0, legend_row_2),
            ]
        )
        painter.setBrush(QColor(250, 204, 21, 88))
        painter.drawPolygon(diamond)
        painter.setPen(QColor("#334155"))
        painter.drawText(
            QRectF(right_text_x, legend_row_2 - 10.0, hud_rect.right() - right_text_x - 12.0, 18.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            "대체 WP 후보",
        )
        painter.restore()

    def _draw_adaptive_debug_overlay(self, painter: QPainter) -> None:
        if not self._adaptive_debug_enabled or not self._view.has_data:
            return
        draw_rect = self._draw_rect()
        panel_rect = QRectF(draw_rect.right() - 338.0, draw_rect.top() + 14.0, 324.0, 118.0)
        rows = [
            ("radiusScale", _format_scale(self._view.adaptive_radius_scale)),
            ("thresholdScale", _format_scale(self._view.adaptive_threshold_scale)),
            ("expectedRoll", _format_roll(self._view.adaptive_expected_roll_deg)),
            ("samples", str(int(self._view.adaptive_sample_count or 0))),
        ]

        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(QColor("#93c5fd"), 1))
        painter.setBrush(QColor(239, 246, 255, 238))
        painter.drawRoundedRect(panel_rect, 10.0, 10.0)

        painter.setFont(QFont("Consolas", 9))
        painter.setPen(QColor("#1d4ed8"))
        painter.drawText(
            QRectF(panel_rect.left() + 12.0, panel_rect.top() + 8.0, panel_rect.width() - 24.0, 18.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            "ADAPTIVE PATH DEBUG",
        )
        label_x = panel_rect.left() + 12.0
        value_x = panel_rect.left() + 146.0
        for index, (label, value) in enumerate(rows):
            y = panel_rect.top() + 32.0 + (index * 20.0)
            painter.setPen(QColor("#334155"))
            painter.drawText(QRectF(label_x, y, 126.0, 18.0), Qt.AlignLeft | Qt.AlignVCenter, label)
            painter.setPen(QColor("#0f172a"))
            painter.drawText(
                QRectF(value_x, y, panel_rect.right() - value_x - 12.0, 18.0),
                Qt.AlignLeft | Qt.AlignVCenter,
                value,
            )
        painter.restore()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#f4f7fb"))

        draw_rect = self._draw_rect()
        painter.fillRect(draw_rect, QColor("#ffffff"))
        painter.setPen(QPen(QColor("#d7e2ee"), 2))
        painter.drawRect(draw_rect)

        self._draw_grid(painter)
        self._draw_path(painter, self._view.ideal_points, "#f97316", 2, dashed=True)
        self._draw_turn_circle(painter)
        self._draw_current_waypoint(painter)
        self._draw_alternate_waypoint(painter)
        self._draw_path(painter, self._view.actual_points, "#0f766e", 3)
        self._draw_aircraft(painter)
        self._draw_scale(painter)
        self._draw_hud(painter)
        self._draw_adaptive_debug_overlay(painter)

        if not self._view.has_data:
            painter.save()
            painter.setPen(QColor("#64748b"))
            painter.setFont(QFont("Malgun Gothic", 13))
            painter.drawText(draw_rect, Qt.AlignCenter, "0401 경로추종 데이터 없음")
            painter.restore()


class TurnRadiusMonitorTab(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._store = TurnRadiusMonitorStore()
        self._selected_aircraft_id = 4
        self._ui_updates_enabled = True
        self._refresh_scheduled = False
        self._adaptive_debug_enabled = False

        self._title = QLabel("0401 경로추종 모니터링")
        self._title.setObjectName("turnRadiusPageTitle")
        self._subtitle = QLabel(
            "0401 원시 데이터를 기준으로 기체 위치, heading, 실제 경로, 기준 선회 반경을 함께 확인합니다."
        )
        self._subtitle.setObjectName("turnRadiusPageSubtitle")
        self._subtitle.setWordWrap(True)

        self._uav_buttons: dict[int, QPushButton] = {}
        self._status_card = MetricCard("상태")
        self._speed_card = MetricCard("속도")
        self._heading_card = MetricCard("헤딩")
        self._turn_rate_card = MetricCard("선회율")
        self._ideal_radius_card = MetricCard("기준 반경")
        self._actual_radius_card = MetricCard("실제 반경")
        self._error_card = MetricCard("반경 오차")
        self._current_wp_card = MetricCard("현재 WP")
        self._wp_orbit_card = MetricCard("WP 선회위험")
        self._age_card = MetricCard("마지막 수신")
        self._adaptive_card = MetricCard("Adaptive")
        self._adaptive_card.setVisible(False)
        self._warning_banner = QLabel("")
        self._warning_banner.setObjectName("turnRadiusWarningBanner")
        self._warning_banner.setVisible(False)

        self._follow_checkbox = QCheckBox("기체 중심 따라가기")
        self._follow_checkbox.setChecked(True)
        self._adaptive_debug_checkbox = QCheckBox("Adaptive debug")
        self._adaptive_debug_checkbox.setChecked(False)
        self._reset_button = QPushButton("화면 초기화")
        self._canvas = TurnRadiusMonitorCanvas()
        self._summary = QLabel("UAV1 / 0401 / 경로추종 상태")
        self._summary.setObjectName("turnRadiusSummary")

        self._build_ui()
        self._apply_style()
        self._refresh()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(12)
        root.addWidget(self._title)
        root.addWidget(self._subtitle)
        root.addWidget(self._warning_banner)

        top_row = QHBoxLayout()
        top_row.setSpacing(10)
        for aircraft_id, text in ((4, "UAV1"), (5, "UAV2"), (6, "UAV3")):
            button = QPushButton(text)
            button.setCheckable(True)
            button.clicked.connect(lambda _checked=False, aid=aircraft_id: self._select_aircraft(aid))
            self._uav_buttons[aircraft_id] = button
            top_row.addWidget(button)
        top_row.addSpacing(20)
        top_row.addWidget(self._follow_checkbox)
        top_row.addWidget(self._adaptive_debug_checkbox)
        top_row.addWidget(self._reset_button)
        top_row.addStretch(1)
        root.addLayout(top_row)
        root.addWidget(self._summary)

        cards = QHBoxLayout()
        cards.setSpacing(10)
        for card in (
            self._status_card,
            self._speed_card,
            self._heading_card,
            self._turn_rate_card,
            self._ideal_radius_card,
            self._actual_radius_card,
            self._error_card,
            self._current_wp_card,
            self._wp_orbit_card,
            self._age_card,
            self._adaptive_card,
        ):
            cards.addWidget(card)
        root.addLayout(cards)
        root.addWidget(self._canvas, 1)

        self._follow_checkbox.toggled.connect(self._canvas.set_follow_aircraft)
        self._adaptive_debug_checkbox.toggled.connect(self._set_adaptive_debug_enabled)
        self._reset_button.clicked.connect(self._canvas.reset_view)
        self._sync_aircraft_buttons()

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                background: #f4f7fb;
                color: #0f172a;
            }
            QLabel#turnRadiusPageTitle {
                font-size: 24px;
                font-weight: 700;
                color: #0f172a;
            }
            QLabel#turnRadiusPageSubtitle {
                font-size: 13px;
                color: #334155;
                padding: 2px 0 4px 2px;
            }
            QLabel#turnRadiusSummary {
                font-size: 13px;
                color: #334155;
                background: #ffffff;
                border: 1px solid #d7e2ee;
                border-radius: 10px;
                padding: 6px 10px;
            }
            QLabel#turnRadiusWarningBanner {
                border-radius: 10px;
                padding: 10px 14px;
                font-size: 13px;
                font-weight: 700;
                color: #9a3412;
                background: #ffedd5;
                border: 1px solid #fdba74;
            }
            QFrame#turnRadiusMetricCard {
                background: #ffffff;
                border: 1px solid #d7e2ee;
                border-radius: 14px;
            }
            QLabel#turnRadiusMetricTitle {
                font-size: 11px;
                font-weight: 600;
                color: #475569;
                background: #f8fafc;
                border-radius: 8px;
                padding: 2px 8px;
            }
            QLabel#turnRadiusMetricValue {
                font-size: 16px;
                font-weight: 700;
                color: #111827;
            }
            QPushButton {
                background: #e2e8f0;
                color: #0f172a;
                border: 1px solid #cbd5e1;
                border-radius: 10px;
                padding: 10px 16px;
                font-size: 13px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #cbd5e1;
            }
            QPushButton:checked {
                background: #1d4ed8;
                color: #ffffff;
                border: 1px solid #1d4ed8;
            }
            QCheckBox {
                color: #1f2937;
                spacing: 8px;
                font-size: 13px;
                font-weight: 600;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
            QCheckBox::indicator:unchecked {
                border: 1px solid #94a3b8;
                background: #ffffff;
            }
            QCheckBox::indicator:checked {
                border: 1px solid #1d4ed8;
                background: #60a5fa;
            }
            """
        )

    def _select_aircraft(self, aircraft_id: int) -> None:
        self._selected_aircraft_id = int(aircraft_id)
        self._sync_aircraft_buttons()
        if not self._ui_updates_enabled:
            return
        self._refresh()

    def _sync_aircraft_buttons(self) -> None:
        for aircraft_id, button in self._uav_buttons.items():
            button.blockSignals(True)
            button.setChecked(aircraft_id == self._selected_aircraft_id)
            button.blockSignals(False)

    def _set_adaptive_debug_enabled(self, enabled: bool) -> None:
        self._adaptive_debug_enabled = bool(enabled)
        self._adaptive_card.setVisible(self._adaptive_debug_enabled)
        self._canvas.set_adaptive_debug_enabled(self._adaptive_debug_enabled)
        self._schedule_refresh(force=True)

    def ingest_0401(self, raw_body: dict | None) -> None:
        self._store.ingest_message(raw_body)
        self._schedule_refresh()

    def build_view(self, aircraft_id: int, *, include_paths: bool = True) -> AircraftTurnView:
        return self._store.build_view(int(aircraft_id), include_paths=include_paths)

    def build_views(self, *, include_paths: bool = True) -> dict[int, AircraftTurnView]:
        return self._store.build_views(include_paths=include_paths)

    def set_ui_updates_enabled(self, enabled: bool) -> None:
        self._ui_updates_enabled = bool(enabled)
        if not self._ui_updates_enabled:
            self._refresh_scheduled = False
            return
        if enabled:
            self._schedule_refresh(force=True)

    def _schedule_refresh(self, force: bool = False) -> None:
        if not self._ui_updates_enabled and not force:
            return
        if self._refresh_scheduled:
            return
        self._refresh_scheduled = True
        QTimer.singleShot(80, self._flush_refresh)

    def _flush_refresh(self) -> None:
        self._refresh_scheduled = False
        if not self._ui_updates_enabled:
            return
        self._refresh()

    def _refresh(self) -> None:
        if not self._ui_updates_enabled:
            return
        view = self._store.build_view(self._selected_aircraft_id)
        self._canvas.set_view(view)

        self._status_card.set_value(_turn_status_text(view))
        self._speed_card.set_value(_format_speed(view.speed_mps))
        self._heading_card.set_value(_format_heading(view.raw_heading_deg, view.heading_rad))
        self._turn_rate_card.set_value(_format_turn_rate(view.turn_rate_dps))
        self._ideal_radius_card.set_value(_format_meters(view.ideal_radius_m))
        self._actual_radius_card.set_value(_format_meters(view.actual_radius_m))
        self._current_wp_card.set_value(
            _format_current_waypoint(view.current_waypoint_id, view.current_waypoint_pass_type)
        )
        self._wp_orbit_card.set_value(_format_spiral_score(view))
        self._age_card.set_value(_format_age(view.age_s))
        if self._adaptive_debug_enabled:
            self._adaptive_card.set_value(_format_adaptive_debug(view))

        radius_error = "-"
        if view.actual_radius_m is not None and view.ideal_radius_m is not None:
            radius_error = f"{(view.actual_radius_m - view.ideal_radius_m):+.0f} m"
        self._error_card.set_value(radius_error)

        summary = f"{view.label} / 0401 경로추종 / {_turn_status_text(view)}"
        if view.current_waypoint_id is not None:
            summary += f" / wp={_format_current_waypoint(view.current_waypoint_id, view.current_waypoint_pass_type)}"
        if view.spiral_state != "none":
            summary += f" / orbit={_format_spiral_score(view)}"
        if view.alternate_waypoint_id is not None:
            summary += f" / alt_wp={view.alternate_waypoint_id}"
        if view.position_m is not None:
            summary += f" / pos=({view.position_m[0]:.1f}, {view.position_m[1]:.1f}) m"
        if self._adaptive_debug_enabled:
            summary += f" / adaptive={_format_adaptive_debug(view)}"
        self._summary.setText(summary)

        self._warning_banner.setVisible(view.spiral_state in {"watch", "warning"})
        if view.spiral_state == "warning":
            angle_text = f"{view.spiral_angle_deg:.0f} deg" if view.spiral_angle_deg is not None else "-"
            self._warning_banner.setStyleSheet(
                "QLabel#turnRadiusWarningBanner { background: #fee2e2; color: #991b1b; border: 1px solid #ef4444; border-radius: 10px; padding: 10px 14px; font-size: 13px; font-weight: 700; }"
            )
            message = f"현재 WP 선회 경고 ({angle_text})"
            if view.alternate_waypoint_id is not None:
                eta_text = f"{view.alternate_waypoint_eta_s:.0f}s" if view.alternate_waypoint_eta_s is not None else "-"
                message += f" / 대체 WP {view.alternate_waypoint_id} (+{eta_text})"
            self._warning_banner.setText(message)
        elif view.spiral_state == "watch":
            angle_text = f"{view.spiral_angle_deg:.0f} deg" if view.spiral_angle_deg is not None else "-"
            self._warning_banner.setStyleSheet(
                "QLabel#turnRadiusWarningBanner { background: #fef3c7; color: #92400e; border: 1px solid #f59e0b; border-radius: 10px; padding: 10px 14px; font-size: 13px; font-weight: 700; }"
            )
            self._warning_banner.setText(
                f"현재 WP 선회 주의 ({angle_text})"
            )
