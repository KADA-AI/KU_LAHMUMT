from __future__ import annotations

import math

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
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

from modules.monitoring.logic.turn_radius_monitor import AircraftTurnView, TurnRadiusMonitorStore


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
        return f"{math.degrees(heading_rad) % 360.0:05.1f} deg"
    return "-"


def _format_turn_rate(turn_rate_dps: float | None) -> str:
    if turn_rate_dps is None:
        return "-"
    return f"{turn_rate_dps:+.2f} deg/s"


def _format_age(age_s: float | None) -> str:
    if age_s is None:
        return "-"
    return f"{age_s:.1f} s"


def _format_waypoint_id(value: int | None) -> str:
    if value is None:
        return "-"
    return str(int(value))


def _format_spiral_score(view: AircraftTurnView) -> str:
    if view.spiral_state == "warning":
        return "Warning"
    if view.spiral_state == "watch":
        return "Watch"
    if view.current_waypoint_is_loiter:
        return "Loiter WP"
    return "Normal"


def _turn_status_text(view: AircraftTurnView) -> str:
    if not view.has_data:
        return "No data"
    if view.age_s is not None and view.age_s > 5.0:
        return "Stale"
    if not view.is_turning or view.turn_sign == 0:
        return "Straight"
    return "Left turn" if view.turn_sign > 0 else "Right turn"


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
            current_waypoint_position_m=None,
            alternate_waypoint_id=None,
            alternate_waypoint_position_m=None,
            alternate_waypoint_coordinate=None,
            alternate_waypoint_eta_s=None,
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

    def set_view(self, view: AircraftTurnView) -> None:
        self._view = view
        if self._follow_aircraft and view.position_m is not None:
            self._manual_center_m = view.position_m
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
        painter.setPen(QPen(QColor("#31425b"), 1))
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

        painter.setPen(QPen(QColor("#8da0be"), 2))
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
        poly = QPolygonF()
        for point in points:
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
        pen = QPen(QColor("#67e8f9"), 2)
        pen.setStyle(Qt.DashLine)
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, view.turn_circle_radius_m * ppm, view.turn_circle_radius_m * ppm)
        painter.restore()

    def _draw_current_waypoint(self, painter: QPainter) -> None:
        view = self._view
        if view.current_waypoint_position_m is None:
            return
        point = self._world_to_screen(view.current_waypoint_position_m)
        color = QColor("#f472b6")
        fill = QColor(244, 114, 182, 70)
        if view.spiral_state == "watch":
            color = QColor("#f59e0b")
            fill = QColor(245, 158, 11, 85)
        elif view.spiral_state == "warning":
            color = QColor("#ef4444")
            fill = QColor(239, 68, 68, 90)
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(color, 2))
        painter.setBrush(fill)
        painter.drawEllipse(point, 7.0, 7.0)
        painter.drawLine(QPointF(point.x() - 10.0, point.y()), QPointF(point.x() + 10.0, point.y()))
        painter.drawLine(QPointF(point.x(), point.y() - 10.0), QPointF(point.x(), point.y() + 10.0))
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(QPointF(point.x() + 12.0, point.y() - 10.0), f"WP {view.current_waypoint_id}")
        painter.restore()

    def _draw_alternate_waypoint(self, painter: QPainter) -> None:
        view = self._view
        if view.alternate_waypoint_position_m is None or view.alternate_waypoint_id is None:
            return
        point = self._world_to_screen(view.alternate_waypoint_position_m)
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        color = QColor("#fde047")
        fill = QColor(253, 224, 71, 90)
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
            painter.setPen(QPen(QColor("#fde047"), 1, Qt.DashLine))
            painter.drawLine(point, self._world_to_screen(view.current_waypoint_position_m))
        painter.setPen(QColor("#fef3c7"))
        painter.setFont(QFont("Consolas", 9))
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
        painter.setPen(QPen(QColor("#dbeafe"), 2))
        painter.setBrush(QColor("#2563eb"))
        painter.drawPolygon(QPolygonF([tip, left, right]))
        painter.setPen(QPen(QColor("#7dd3fc"), 2))
        painter.drawLine(center, QPointF(center.x() + (ux * 120.0), center.y() + (uy * 120.0)))
        painter.restore()

    def _draw_scale(self, painter: QPainter) -> None:
        draw_rect = self._draw_rect()
        ppm = self._pixels_per_meter()
        length_m = _scale_bar_m(self._half_range_m)
        length_px = length_m * ppm

        painter.save()
        painter.setPen(QPen(QColor("#cbd5e1"), 2))
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
        hud_rect = QRectF(draw_rect.left() + 14.0, draw_rect.top() + 14.0, 320.0, 146.0)

        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 23, 42, 215))
        painter.drawRoundedRect(hud_rect, 12.0, 12.0)

        painter.setPen(QColor("#e2e8f0"))
        painter.setFont(QFont("Consolas", 10))
        lines = [
            "Wheel      : zoom in / out",
            "Drag       : pan (follow off)",
            "DoubleTap  : recenter aircraft",
            f"View       : +/- {int(self._half_range_m)} m",
        ]
        for index, line in enumerate(lines):
            painter.drawText(QPointF(hud_rect.left() + 14.0, hud_rect.top() + 24.0 + (index * 24.0)), line)

        painter.setPen(QPen(QColor("#22c55e"), 3))
        painter.drawLine(QPointF(hud_rect.left() + 18.0, hud_rect.bottom() - 46.0), QPointF(hud_rect.left() + 58.0, hud_rect.bottom() - 46.0))
        painter.setPen(QColor("#cbd5e1"))
        painter.drawText(QPointF(hud_rect.left() + 68.0, hud_rect.bottom() - 41.0), "Actual trail")

        painter.setPen(QPen(QColor("#f59e0b"), 3, Qt.DashLine))
        painter.drawLine(QPointF(hud_rect.left() + 18.0, hud_rect.bottom() - 28.0), QPointF(hud_rect.left() + 58.0, hud_rect.bottom() - 28.0))
        painter.setPen(QColor("#cbd5e1"))
        painter.drawText(QPointF(hud_rect.left() + 68.0, hud_rect.bottom() - 23.0), "Ideal turn track")

        wp_color = QColor("#f472b6")
        if self._view.spiral_state == "watch":
            wp_color = QColor("#f59e0b")
        elif self._view.spiral_state == "warning":
            wp_color = QColor("#ef4444")
        painter.setPen(QPen(wp_color, 3))
        painter.drawLine(QPointF(hud_rect.left() + 166.0, hud_rect.bottom() - 28.0), QPointF(hud_rect.left() + 206.0, hud_rect.bottom() - 28.0))
        painter.setPen(QColor("#cbd5e1"))
        painter.drawText(QPointF(hud_rect.left() + 214.0, hud_rect.bottom() - 23.0), "Current WP")

        painter.setPen(QPen(QColor("#fde047"), 2))
        diamond = QPolygonF(
            [
                QPointF(hud_rect.left() + 186.0, hud_rect.bottom() - 8.0),
                QPointF(hud_rect.left() + 194.0, hud_rect.bottom() - 16.0),
                QPointF(hud_rect.left() + 202.0, hud_rect.bottom() - 8.0),
                QPointF(hud_rect.left() + 194.0, hud_rect.bottom()),
            ]
        )
        painter.setBrush(QColor(253, 224, 71, 90))
        painter.drawPolygon(diamond)
        painter.setPen(QColor("#cbd5e1"))
        painter.drawText(QPointF(hud_rect.left() + 214.0, hud_rect.bottom() - 5.0), "Alt WP candidate")
        painter.restore()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#020617"))

        draw_rect = self._draw_rect()
        painter.fillRect(draw_rect, QColor("#0b1730"))
        painter.setPen(QPen(QColor("#475569"), 2))
        painter.drawRect(draw_rect)

        self._draw_grid(painter)
        self._draw_path(painter, self._view.ideal_points, "#f59e0b", 2, dashed=True)
        self._draw_turn_circle(painter)
        self._draw_current_waypoint(painter)
        self._draw_alternate_waypoint(painter)
        self._draw_path(painter, self._view.actual_points, "#22c55e", 3)
        self._draw_aircraft(painter)
        self._draw_scale(painter)
        self._draw_hud(painter)

        if not self._view.has_data:
            painter.save()
            painter.setPen(QColor("#cbd5e1"))
            painter.setFont(QFont("Malgun Gothic", 13))
            painter.drawText(draw_rect, Qt.AlignCenter, "0401 UAV data not available")
            painter.restore()


class TurnRadiusMonitorTab(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._store = TurnRadiusMonitorStore()
        self._selected_aircraft_id = 4
        self._ui_updates_enabled = True
        self._refresh_scheduled = False

        self._title = QLabel("0401 Turn Radius Monitor")
        self._title.setObjectName("turnRadiusPageTitle")
        self._subtitle = QLabel(
            "0401 raw data drives UAV position, heading, actual trail, and ideal speed-based turn radius overlay."
        )
        self._subtitle.setObjectName("turnRadiusPageSubtitle")
        self._subtitle.setWordWrap(True)

        self._uav_buttons: dict[int, QPushButton] = {}
        self._status_card = MetricCard("Status")
        self._speed_card = MetricCard("Speed")
        self._heading_card = MetricCard("Heading")
        self._turn_rate_card = MetricCard("Turn Trend")
        self._ideal_radius_card = MetricCard("Ideal Radius")
        self._actual_radius_card = MetricCard("Actual Radius")
        self._error_card = MetricCard("Radius Error")
        self._current_wp_card = MetricCard("Current WP")
        self._wp_orbit_card = MetricCard("WP Orbit Risk")
        self._age_card = MetricCard("Last Update")
        self._warning_banner = QLabel("")
        self._warning_banner.setObjectName("turnRadiusWarningBanner")
        self._warning_banner.setVisible(False)

        self._follow_checkbox = QCheckBox("Aircraft Center Follow")
        self._follow_checkbox.setChecked(True)
        self._reset_button = QPushButton("Reset View")
        self._canvas = TurnRadiusMonitorCanvas()
        self._summary = QLabel("UAV1 / 0401 / actual vs ideal turn tracking")
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
        ):
            cards.addWidget(card)
        root.addLayout(cards)
        root.addWidget(self._canvas, 1)

        self._follow_checkbox.toggled.connect(self._canvas.set_follow_aircraft)
        self._reset_button.clicked.connect(self._canvas.reset_view)
        self._sync_aircraft_buttons()

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget {
                background: #020617;
                color: #e2e8f0;
            }
            QLabel#turnRadiusPageTitle {
                font-size: 24px;
                font-weight: 700;
                color: #f8fafc;
            }
            QLabel#turnRadiusPageSubtitle {
                font-size: 13px;
                color: #94a3b8;
            }
            QLabel#turnRadiusSummary {
                font-size: 13px;
                color: #cbd5e1;
                padding: 4px 0 2px 2px;
            }
            QLabel#turnRadiusWarningBanner {
                border-radius: 10px;
                padding: 10px 14px;
                font-size: 13px;
                font-weight: 700;
                color: #fff7ed;
                background: #7c2d12;
            }
            QFrame#turnRadiusMetricCard {
                background: #111827;
                border: 1px solid #334155;
                border-radius: 14px;
            }
            QLabel#turnRadiusMetricTitle {
                font-size: 11px;
                color: #94a3b8;
            }
            QLabel#turnRadiusMetricValue {
                font-size: 16px;
                font-weight: 700;
                color: #f8fafc;
            }
            QPushButton {
                background: #1d4ed8;
                color: #eff6ff;
                border: none;
                border-radius: 10px;
                padding: 10px 16px;
                font-size: 13px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #2563eb;
            }
            QPushButton:checked {
                background: #0f766e;
            }
            QCheckBox {
                color: #cbd5e1;
                spacing: 8px;
                font-size: 13px;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
            QCheckBox::indicator:unchecked {
                border: 1px solid #64748b;
                background: #0f172a;
            }
            QCheckBox::indicator:checked {
                border: 1px solid #0f766e;
                background: #14b8a6;
            }
            """
        )

    def _select_aircraft(self, aircraft_id: int) -> None:
        self._selected_aircraft_id = int(aircraft_id)
        self._sync_aircraft_buttons()
        self._refresh()

    def _sync_aircraft_buttons(self) -> None:
        for aircraft_id, button in self._uav_buttons.items():
            button.blockSignals(True)
            button.setChecked(aircraft_id == self._selected_aircraft_id)
            button.blockSignals(False)

    def ingest_0401(self, raw_body: dict | None) -> None:
        self._store.ingest_message(raw_body)
        self._schedule_refresh()

    def build_view(self, aircraft_id: int) -> AircraftTurnView:
        return self._store.build_view(int(aircraft_id))

    def build_views(self) -> dict[int, AircraftTurnView]:
        return self._store.build_views()

    def set_ui_updates_enabled(self, enabled: bool) -> None:
        self._ui_updates_enabled = bool(enabled)
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
        view = self._store.build_view(self._selected_aircraft_id)
        self._canvas.set_view(view)

        self._status_card.set_value(_turn_status_text(view))
        self._speed_card.set_value(_format_speed(view.speed_mps))
        self._heading_card.set_value(_format_heading(view.raw_heading_deg, view.heading_rad))
        self._turn_rate_card.set_value(_format_turn_rate(view.turn_rate_dps))
        self._ideal_radius_card.set_value(_format_meters(view.ideal_radius_m))
        self._actual_radius_card.set_value(_format_meters(view.actual_radius_m))
        self._current_wp_card.set_value(_format_waypoint_id(view.current_waypoint_id))
        self._wp_orbit_card.set_value(_format_spiral_score(view))
        self._age_card.set_value(_format_age(view.age_s))

        radius_error = "-"
        if view.actual_radius_m is not None and view.ideal_radius_m is not None:
            radius_error = f"{(view.actual_radius_m - view.ideal_radius_m):+.0f} m"
        self._error_card.set_value(radius_error)

        summary = f"{view.label} / 0401 / {_turn_status_text(view)}"
        if view.current_waypoint_id is not None:
            summary += f" / wp={view.current_waypoint_id}"
        if view.spiral_state != "none":
            summary += f" / orbit={view.spiral_state}"
        if view.alternate_waypoint_id is not None:
            summary += f" / alt_wp={view.alternate_waypoint_id}"
        if view.position_m is not None:
            summary += f" / pos=({view.position_m[0]:.1f}, {view.position_m[1]:.1f}) m"
        self._summary.setText(summary)

        self._warning_banner.setVisible(view.spiral_state in {"watch", "warning"})
        if view.spiral_state == "warning":
            angle_text = f"{view.spiral_angle_deg:.0f} deg" if view.spiral_angle_deg is not None else "-"
            self._warning_banner.setStyleSheet(
                "QLabel#turnRadiusWarningBanner { background: #7f1d1d; color: #fee2e2; border: 1px solid #ef4444; border-radius: 10px; padding: 10px 14px; font-size: 13px; font-weight: 700; }"
            )
            message = f"Current WP orbit warning: same WP is being orbited with weak radial progress ({angle_text})."
            if view.alternate_waypoint_id is not None:
                eta_text = f"{view.alternate_waypoint_eta_s:.0f}s" if view.alternate_waypoint_eta_s is not None else "-"
                message += f" Alt WP {view.alternate_waypoint_id} projected on predicted circle (+{eta_text})."
            self._warning_banner.setText(message)
        elif view.spiral_state == "watch":
            angle_text = f"{view.spiral_angle_deg:.0f} deg" if view.spiral_angle_deg is not None else "-"
            self._warning_banner.setStyleSheet(
                "QLabel#turnRadiusWarningBanner { background: #78350f; color: #fef3c7; border: 1px solid #f59e0b; border-radius: 10px; padding: 10px 14px; font-size: 13px; font-weight: 700; }"
            )
            self._warning_banner.setText(
                f"Current WP orbit watch: possible circling around WP with accumulated bearing {angle_text}."
            )
