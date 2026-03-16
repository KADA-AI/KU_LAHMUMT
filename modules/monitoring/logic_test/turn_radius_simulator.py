from __future__ import annotations

import argparse
import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

THIS_DIR = Path(__file__).resolve().parent
if not getattr(sys, "frozen", False):
    PROJECT_ROOT = THIS_DIR.parents[2]
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))

from modules.common.qt_env import ensure_qt_platform
from modules.common.turn_radius import (
    REFERENCE_TURN_RADIUS_TABLE_MPS,
    interpolate_reference_turn_radius,
)

ensure_qt_platform()

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)


VIEW_HALF_RANGE_M = 500.0
GRID_STEP_M = 100.0
AIRCRAFT_SIZE_PX = 18.0
HISTORY_LIMIT = 2400
HISTORY_SAMPLE_STEP_M = 4.0
TICK_INTERVAL_MS = 16
TURN_BANK_LIMIT_DEG = 30.0
SPEED_STEP_MPS = 5.0
MIN_SPEED_MPS = 30.0
MAX_SPEED_MPS = 50.0
DEFAULT_SPEED_MPS = 40.0
DEFAULT_HEADING_DEG = 0.0
SPEED_RADIUS_TABLE = REFERENCE_TURN_RADIUS_TABLE_MPS


def interpolate_turn_radius(speed_mps: float) -> float:
    speed = max(MIN_SPEED_MPS, min(MAX_SPEED_MPS, float(speed_mps)))
    return float(interpolate_reference_turn_radius(speed))


def wrap_angle_rad(angle_rad: float) -> float:
    wrapped = math.fmod(angle_rad, math.tau)
    if wrapped < 0.0:
        wrapped += math.tau
    return wrapped


def heading_text(angle_rad: float) -> str:
    heading_deg = math.degrees(wrap_angle_rad(angle_rad))
    return f"{heading_deg:05.1f} deg"


@dataclass
class AircraftState:
    x_m: float = 0.0
    y_m: float = 0.0
    heading_rad: float = math.radians(DEFAULT_HEADING_DEG)
    speed_mps: float = DEFAULT_SPEED_MPS

    @property
    def turn_radius_m(self) -> float:
        return interpolate_turn_radius(self.speed_mps)


class TurnSimulatorCanvas(QWidget):
    state_updated = pyqtSignal()

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumSize(820, 820)

        self.state = AircraftState()
        self._pressed_keys: set[int] = set()
        self._paused = False
        self._history: deque[tuple[float, float]] = deque([(self.state.x_m, self.state.y_m)], maxlen=HISTORY_LIMIT)
        self._last_tick = time.perf_counter()

        self._tick = QTimer(self)
        self._tick.setInterval(TICK_INTERVAL_MS)
        self._tick.timeout.connect(self._advance_simulation)
        self._tick.start()

    @property
    def paused(self) -> bool:
        return self._paused

    @property
    def turn_input(self) -> int:
        left = Qt.Key_Left in self._pressed_keys
        right = Qt.Key_Right in self._pressed_keys
        if left == right:
            return 0
        return 1 if left else -1

    @property
    def bank_angle_deg(self) -> float:
        return float(self.turn_input) * TURN_BANK_LIMIT_DEG

    @property
    def yaw_rate_deg_s(self) -> float:
        if self.turn_input == 0:
            return 0.0
        return math.degrees(self.state.speed_mps / self.state.turn_radius_m) * float(self.turn_input)

    def reset_simulation(self) -> None:
        self.state = AircraftState()
        self._history = deque([(self.state.x_m, self.state.y_m)], maxlen=HISTORY_LIMIT)
        self._pressed_keys.clear()
        self._last_tick = time.perf_counter()
        self.state_updated.emit()
        self.update()

    def toggle_pause(self) -> None:
        self._paused = not self._paused
        self._last_tick = time.perf_counter()
        self.state_updated.emit()
        self.update()

    def set_speed(self, speed_mps: float) -> None:
        clamped = max(MIN_SPEED_MPS, min(MAX_SPEED_MPS, float(speed_mps)))
        if abs(clamped - self.state.speed_mps) < 1e-9:
            return
        self.state.speed_mps = clamped
        self.state_updated.emit()
        self.update()

    def step_speed(self, direction: int) -> None:
        if direction == 0:
            return
        self.set_speed(self.state.speed_mps + (SPEED_STEP_MPS * float(direction)))

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        key = event.key()
        if key in (Qt.Key_Left, Qt.Key_Right):
            self._pressed_keys.add(key)
            self.state_updated.emit()
            self.update()
            return
        if event.isAutoRepeat():
            return
        if key == Qt.Key_Up:
            self.step_speed(+1)
            return
        if key == Qt.Key_Down:
            self.step_speed(-1)
            return
        if key == Qt.Key_Space:
            self.toggle_pause()
            return
        if key == Qt.Key_R:
            self.reset_simulation()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.isAutoRepeat():
            return
        key = event.key()
        if key in (Qt.Key_Left, Qt.Key_Right):
            self._pressed_keys.discard(key)
            self.state_updated.emit()
            self.update()
            return
        super().keyReleaseEvent(event)

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        self.setFocus(Qt.MouseFocusReason)
        super().mousePressEvent(event)

    def _advance_simulation(self) -> None:
        now = time.perf_counter()
        dt_s = max(0.0, min(0.05, now - self._last_tick))
        self._last_tick = now
        if self._paused or dt_s <= 0.0:
            self.update()
            return

        turn_input = self.turn_input
        start_heading = self.state.heading_rad
        heading_delta = 0.0
        if turn_input != 0:
            yaw_rate_rad_s = self.state.speed_mps / self.state.turn_radius_m
            heading_delta = yaw_rate_rad_s * dt_s * float(turn_input)
        average_heading = start_heading + (heading_delta * 0.5)
        end_heading = wrap_angle_rad(start_heading + heading_delta)
        self.state.x_m += math.cos(average_heading) * self.state.speed_mps * dt_s
        self.state.y_m += math.sin(average_heading) * self.state.speed_mps * dt_s
        self.state.heading_rad = end_heading

        last_x, last_y = self._history[-1]
        if math.hypot(self.state.x_m - last_x, self.state.y_m - last_y) >= HISTORY_SAMPLE_STEP_M:
            self._history.append((self.state.x_m, self.state.y_m))

        self.state_updated.emit()
        self.update()

    def _canvas_metrics(self) -> tuple[QRectF, float]:
        full = QRectF(self.rect())
        margin = 28.0
        draw_rect = full.adjusted(margin, margin, -margin, -margin)
        pixels_per_meter = min(draw_rect.width(), draw_rect.height()) / (VIEW_HALF_RANGE_M * 2.0)
        return draw_rect, pixels_per_meter

    def _world_to_screen(self, x_m: float, y_m: float, draw_rect: QRectF, pixels_per_meter: float) -> QPointF:
        cx = draw_rect.center().x()
        cy = draw_rect.center().y()
        rel_x = x_m - self.state.x_m
        rel_y = y_m - self.state.y_m
        return QPointF(
            cx + (rel_x * pixels_per_meter),
            cy - (rel_y * pixels_per_meter),
        )

    def _draw_grid(self, painter: QPainter, draw_rect: QRectF, pixels_per_meter: float) -> None:
        painter.save()
        grid_pen = QPen(QColor("#334155"))
        grid_pen.setWidth(1)
        painter.setPen(grid_pen)

        left_world = self.state.x_m - VIEW_HALF_RANGE_M
        right_world = self.state.x_m + VIEW_HALF_RANGE_M
        first_x = math.floor(left_world / GRID_STEP_M) * GRID_STEP_M
        x_val = first_x
        while x_val <= right_world + 0.1:
            point = self._world_to_screen(x_val, self.state.y_m, draw_rect, pixels_per_meter)
            painter.drawLine(QPointF(point.x(), draw_rect.top()), QPointF(point.x(), draw_rect.bottom()))
            x_val += GRID_STEP_M

        bottom_world = self.state.y_m - VIEW_HALF_RANGE_M
        top_world = self.state.y_m + VIEW_HALF_RANGE_M
        first_y = math.floor(bottom_world / GRID_STEP_M) * GRID_STEP_M
        y_val = first_y
        while y_val <= top_world + 0.1:
            point = self._world_to_screen(self.state.x_m, y_val, draw_rect, pixels_per_meter)
            painter.drawLine(QPointF(draw_rect.left(), point.y()), QPointF(draw_rect.right(), point.y()))
            y_val += GRID_STEP_M

        axis_pen = QPen(QColor("#94a3b8"))
        axis_pen.setWidth(2)
        painter.setPen(axis_pen)
        center = draw_rect.center()
        painter.drawLine(QPointF(center.x(), draw_rect.top()), QPointF(center.x(), draw_rect.bottom()))
        painter.drawLine(QPointF(draw_rect.left(), center.y()), QPointF(draw_rect.right(), center.y()))
        painter.restore()

    def _draw_trail(self, painter: QPainter, draw_rect: QRectF, pixels_per_meter: float) -> None:
        if len(self._history) < 2:
            return
        poly = QPolygonF()
        for x_m, y_m in self._history:
            point = self._world_to_screen(x_m, y_m, draw_rect, pixels_per_meter)
            poly.append(point)
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        trail_pen = QPen(QColor("#22c55e"))
        trail_pen.setWidth(3)
        painter.setPen(trail_pen)
        painter.drawPolyline(poly)
        painter.restore()

    def _draw_turn_preview(self, painter: QPainter, draw_rect: QRectF, pixels_per_meter: float) -> None:
        if self.turn_input == 0:
            return

        heading = self.state.heading_rad
        radius = self.state.turn_radius_m
        normal_sign = 1.0 if self.turn_input > 0 else -1.0
        normal_x = -math.sin(heading) * normal_sign
        normal_y = math.cos(heading) * normal_sign
        center_world_x = self.state.x_m + (normal_x * radius)
        center_world_y = self.state.y_m + (normal_y * radius)
        center_screen = self._world_to_screen(center_world_x, center_world_y, draw_rect, pixels_per_meter)

        painter.save()
        preview_pen = QPen(QColor("#f59e0b"))
        preview_pen.setWidth(2)
        preview_pen.setStyle(Qt.DashLine)
        painter.setPen(preview_pen)
        circle_radius_px = radius * pixels_per_meter
        painter.drawEllipse(center_screen, circle_radius_px, circle_radius_px)
        painter.restore()

    def _draw_aircraft(self, painter: QPainter, draw_rect: QRectF) -> None:
        center = draw_rect.center()
        heading = self.state.heading_rad
        ux = math.cos(heading)
        uy = -math.sin(heading)
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
        aircraft = QPolygonF([tip, left, right])

        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(QColor("#e2e8f0"), 2))
        painter.setBrush(QColor("#1d4ed8"))
        painter.drawPolygon(aircraft)

        nose_pen = QPen(QColor("#7dd3fc"))
        nose_pen.setWidth(2)
        painter.setPen(nose_pen)
        painter.drawLine(center, QPointF(center.x() + (math.cos(heading) * 120.0), center.y() - (math.sin(heading) * 120.0)))
        painter.restore()

    def _draw_scale_legend(self, painter: QPainter, draw_rect: QRectF, pixels_per_meter: float) -> None:
        painter.save()
        painter.setPen(QPen(QColor("#cbd5e1"), 2))
        legend_len_m = 100.0
        legend_len_px = legend_len_m * pixels_per_meter
        x0 = draw_rect.left() + 18.0
        y0 = draw_rect.bottom() - 18.0
        painter.drawLine(QPointF(x0, y0), QPointF(x0 + legend_len_px, y0))
        painter.drawLine(QPointF(x0, y0 - 6.0), QPointF(x0, y0 + 6.0))
        painter.drawLine(QPointF(x0 + legend_len_px, y0 - 6.0), QPointF(x0 + legend_len_px, y0 + 6.0))
        painter.setFont(QFont("Consolas", 10))
        painter.drawText(QPointF(x0, y0 - 10.0), "100 m")
        painter.restore()

    def _draw_hud(self, painter: QPainter, draw_rect: QRectF) -> None:
        hud_rect = QRectF(draw_rect.left() + 14.0, draw_rect.top() + 14.0, 272.0, 146.0)
        painter.save()
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(15, 23, 42, 215))
        painter.drawRoundedRect(hud_rect, 12.0, 12.0)

        painter.setPen(QColor("#e2e8f0"))
        painter.setFont(QFont("Consolas", 10))
        lines = [
            "Arrow Left / Right : turn",
            "Arrow Up / Down   : speed +/- 5 m/s",
            "Space             : pause",
            "R                 : reset",
            f"View              : +/- {int(VIEW_HALF_RANGE_M)} m",
        ]
        for idx, text in enumerate(lines):
            painter.drawText(QPointF(hud_rect.left() + 14.0, hud_rect.top() + 24.0 + (idx * 24.0)), text)
        painter.restore()

    def paintEvent(self, event) -> None:  # type: ignore[override]
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#020617"))

        draw_rect, pixels_per_meter = self._canvas_metrics()
        painter.fillRect(draw_rect, QColor("#0f172a"))

        border_pen = QPen(QColor("#475569"))
        border_pen.setWidth(2)
        painter.setPen(border_pen)
        painter.drawRect(draw_rect)

        self._draw_grid(painter, draw_rect, pixels_per_meter)
        self._draw_trail(painter, draw_rect, pixels_per_meter)
        self._draw_turn_preview(painter, draw_rect, pixels_per_meter)
        self._draw_aircraft(painter, draw_rect)
        self._draw_scale_legend(painter, draw_rect, pixels_per_meter)
        self._draw_hud(painter, draw_rect)


class InfoCard(QFrame):
    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("infoCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(4)

        self._title = QLabel(title)
        self._title.setObjectName("infoTitle")
        self._value = QLabel("-")
        self._value.setObjectName("infoValue")
        layout.addWidget(self._title)
        layout.addWidget(self._value)

    def set_value(self, value: str) -> None:
        self._value.setText(value)


class TurnRadiusSimulatorWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Turn Radius Simulator")
        self.resize(1320, 980)

        self.canvas = TurnSimulatorCanvas()
        self.canvas.state_updated.connect(self._refresh_info)

        self.status_card = InfoCard("Mode")
        self.speed_card = InfoCard("Speed")
        self.radius_card = InfoCard("Turn Radius")
        self.bank_card = InfoCard("Bank Angle")
        self.heading_card = InfoCard("Heading")
        self.yaw_rate_card = InfoCard("Turn Rate")
        self.position_card = InfoCard("Position")

        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(self._toggle_pause)
        self.reset_button = QPushButton("Reset")
        self.reset_button.clicked.connect(self.canvas.reset_simulation)

        self._build_ui()
        self._apply_style()
        self._refresh_info()

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(14)

        title = QLabel("Monitoring Logic Test: Turn Radius Simulator")
        title.setObjectName("pageTitle")
        subtitle = QLabel(
            "Aircraft stays centered. Left/Right turns apply the requested Dubins-style radius table. "
            "Up/Down changes speed in 5 m/s steps."
        )
        subtitle.setObjectName("pageSubtitle")
        subtitle.setWordWrap(True)
        root.addWidget(title)
        root.addWidget(subtitle)

        cards_row = QHBoxLayout()
        cards_row.setSpacing(10)
        for card in (
            self.status_card,
            self.speed_card,
            self.radius_card,
            self.bank_card,
            self.heading_card,
            self.yaw_rate_card,
            self.position_card,
        ):
            cards_row.addWidget(card)
        root.addLayout(cards_row)

        button_row = QHBoxLayout()
        button_row.setSpacing(10)
        button_row.addWidget(self.pause_button)
        button_row.addWidget(self.reset_button)
        button_row.addStretch(1)
        root.addLayout(button_row)

        root.addWidget(self.canvas, 1)
        self.setCentralWidget(central)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #020617;
                color: #e2e8f0;
            }
            QLabel#pageTitle {
                font-size: 26px;
                font-weight: 700;
                color: #f8fafc;
            }
            QLabel#pageSubtitle {
                font-size: 13px;
                color: #94a3b8;
            }
            QFrame#infoCard {
                background: #111827;
                border: 1px solid #334155;
                border-radius: 14px;
            }
            QLabel#infoTitle {
                font-size: 11px;
                color: #94a3b8;
                letter-spacing: 0.5px;
            }
            QLabel#infoValue {
                font-size: 16px;
                font-weight: 700;
                color: #f8fafc;
            }
            QPushButton {
                background: #1d4ed8;
                color: #eff6ff;
                border: none;
                border-radius: 10px;
                padding: 10px 18px;
                font-size: 13px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: #2563eb;
            }
            QPushButton:pressed {
                background: #1e40af;
            }
            """
        )

    def _toggle_pause(self) -> None:
        self.canvas.toggle_pause()
        self.pause_button.setText("Resume" if self.canvas.paused else "Pause")

    def _refresh_info(self) -> None:
        state = self.canvas.state
        turn_input = self.canvas.turn_input
        if self.canvas.paused:
            mode = "Paused"
        elif turn_input > 0:
            mode = "Left turn"
        elif turn_input < 0:
            mode = "Right turn"
        else:
            mode = "Straight"

        self.status_card.set_value(mode)
        self.speed_card.set_value(f"{state.speed_mps:.0f} m/s")
        self.radius_card.set_value(f"{state.turn_radius_m:.0f} m")
        self.bank_card.set_value(f"{self.canvas.bank_angle_deg:+.0f} deg")
        self.heading_card.set_value(heading_text(state.heading_rad))
        self.yaw_rate_card.set_value(f"{self.canvas.yaw_rate_deg_s:+.2f} deg/s")
        self.position_card.set_value(f"({state.x_m:.1f}, {state.y_m:.1f}) m")
        self.pause_button.setText("Resume" if self.canvas.paused else "Pause")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Turn radius simulator for monitoring logic tests.")
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="Start the window and exit shortly after initialization.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    app = QApplication.instance() or QApplication(sys.argv)
    window = TurnRadiusSimulatorWindow()
    window.show()
    window.canvas.setFocus(Qt.OtherFocusReason)
    if args.smoke_test:
        QTimer.singleShot(200, app.quit)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
