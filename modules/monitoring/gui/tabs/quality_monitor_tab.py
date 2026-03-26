# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import time
from collections import deque
from typing import Any, Callable

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from modules.monitoring.logic.mission_update import (
    build_uav_mission_view,
    compute_filming_quality_threshold_m,
    format_timestamp_ms,
    lookup_fov_db_max_width_m,
)

_HISTORY_WINDOW_MS = 120_000
_SERIES_MAXLEN = 720

_UAV_IDS = (4, 5, 6)
_MANNED_IDS = (1, 2, 3)

_SEP_SERIES = (
    ("actual", "실측 거리", "#2563eb", Qt.SolidLine),
    ("trigger", "기준 최대거리", "#ef4444", Qt.DashLine),
)
_LINK_SERIES_META = {
    4: ("무인기 1", "#0f766e"),
    5: ("무인기 2", "#ea580c"),
    6: ("무인기 3", "#7c3aed"),
}


def _coerce_int(value: object | None) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _coerce_float(value: object | None) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _uav_label(aircraft_id: int) -> str:
    return f"무인기 {int(aircraft_id) - 3}"


def _manned_label(aircraft_id: int) -> str:
    return f"유인기 {int(aircraft_id)}"


def _format_distance(value_m: float | None) -> str:
    if value_m is None:
        return "-"
    distance = float(value_m)
    if distance >= 1000.0:
        return f"{distance / 1000.0:.2f} km"
    return f"{distance:.0f} m"


def _normalize_coordinate(value: object | None) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None
    lat = _coerce_float(value.get("latitude") or value.get("Latitude"))
    lon = _coerce_float(value.get("longitude") or value.get("Longitude"))
    alt = _coerce_float(value.get("altitude") or value.get("Altitude"))
    if lat is None or lon is None:
        return None
    out: dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        out["altitude"] = float(alt)
    return out


def _ground_distance_m(left: dict[str, float] | None, right: dict[str, float] | None) -> float | None:
    if not left or not right:
        return None
    lat1 = math.radians(float(left["latitude"]))
    lon1 = math.radians(float(left["longitude"]))
    lat2 = math.radians(float(right["latitude"]))
    lon2 = math.radians(float(right["longitude"]))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return 6_371_000.0 * c


def _slant_distance_m(left: dict[str, float] | None, right: dict[str, float] | None) -> float | None:
    ground = _ground_distance_m(left, right)
    if ground is None:
        return None
    left_alt = float((left or {}).get("altitude") or 0.0)
    right_alt = float((right or {}).get("altitude") or 0.0)
    return math.hypot(float(ground), left_alt - right_alt)


class TimeSeriesChartWidget(QWidget):
    def __init__(
        self,
        *,
        series_meta: list[tuple[str, str, str, Qt.PenStyle]],
        unit: str,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._series_meta = list(series_meta)
        self._unit = str(unit)
        self._history: dict[str, deque[tuple[int, float]]] = {
            key: deque(maxlen=_SERIES_MAXLEN) for key, *_ in self._series_meta
        }
        self._latest_ts_ms: int = 0
        self._window_ms: int = _HISTORY_WINDOW_MS
        self._ui_scale: float = 1.0
        self._monitor_enabled: bool = True
        self.setMinimumHeight(180)
        self.setProperty("_base_min_height", 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_ui_scale(self, scale: float) -> None:
        try:
            self._ui_scale = max(0.72, min(1.15, float(scale)))
        except Exception:
            self._ui_scale = 1.0
        self.update()

    def set_monitor_enabled(self, enabled: bool) -> None:
        self._monitor_enabled = bool(enabled)
        self.update()

    def append_samples(
        self,
        *,
        timestamp_ms: int | None,
        values: dict[str, float | None],
        redraw: bool = True,
    ) -> None:
        ts_ms = _coerce_int(timestamp_ms)
        if ts_ms is None:
            if self._latest_ts_ms > 0:
                ts_ms = self._latest_ts_ms + 1000
            else:
                ts_ms = int(time.time() * 1000)
        if ts_ms > self._latest_ts_ms:
            self._latest_ts_ms = int(ts_ms)
        for key, value in values.items():
            history = self._history.get(str(key))
            if history is None:
                continue
            numeric = _coerce_float(value)
            if numeric is None:
                continue
            if not history:
                bootstrap_ts = max(0, int(ts_ms) - 1000)
                history.append((bootstrap_ts, max(0.0, float(numeric))))
            history.append((int(ts_ms), max(0.0, float(numeric))))
        self._prune_old()
        if redraw:
            self.update()

    def _prune_old(self) -> None:
        if self._latest_ts_ms <= 0:
            return
        cutoff = int(self._latest_ts_ms) - int(self._window_ms)
        for history in self._history.values():
            while history and history[0][0] < cutoff:
                history.popleft()

    def _visible_values(self, now_ms: int) -> list[float]:
        values: list[float] = []
        cutoff = int(now_ms) - int(self._window_ms)
        for history in self._history.values():
            for ts_ms, value in history:
                if ts_ms >= cutoff:
                    values.append(float(value))
        return values

    def _build_polyline(
        self,
        plot_rect: QRectF,
        history: deque[tuple[int, float]],
        now_ms: int,
        *,
        y_min: float,
        y_max: float,
    ) -> list[QPointF]:
        points: list[QPointF] = []
        if not history:
            return points
        cutoff = int(now_ms) - int(self._window_ms)
        span_y = max(1e-6, float(y_max) - float(y_min))
        for ts_ms, value in history:
            if ts_ms < cutoff:
                continue
            age_ms = max(0, int(now_ms) - int(ts_ms))
            x = plot_rect.right() - (float(age_ms) / float(self._window_ms)) * plot_rect.width()
            normalized = (float(value) - float(y_min)) / span_y
            y = plot_rect.bottom() - normalized * plot_rect.height()
            points.append(QPointF(x, y))
        return points

    def paintEvent(self, event) -> None:  # type: ignore[override]
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#f8fafc"))

        outer = QRectF(0.5, 0.5, max(1.0, self.width() - 1.0), max(1.0, self.height() - 1.0))
        painter.setPen(QPen(QColor("#d6dee8"), 1))
        painter.setBrush(QColor("#ffffff"))
        painter.drawRoundedRect(outer, 10.0, 10.0)

        title_font_px = max(8, int(round(9.5 * self._ui_scale)))
        meta_font_px = max(7, int(round(8.0 * self._ui_scale)))
        header_h = max(18.0, 26.0 * self._ui_scale)
        plot_rect = QRectF(
            outer.left() + 10.0,
            outer.top() + header_h + 6.0,
            max(16.0, outer.width() - 20.0),
            max(16.0, outer.height() - header_h - 24.0),
        )
        now_ms = self._latest_ts_ms if self._latest_ts_ms > 0 else int(time.time() * 1000)
        visible_values = self._visible_values(now_ms)
        if visible_values:
            y_min = min(0.0, min(visible_values))
            y_max = max(visible_values)
            margin = max(1.0, (y_max - y_min) * 0.18, y_max * 0.08)
            y_max += margin
        else:
            y_min = 0.0
            y_max = 1.0

        legend_x = outer.left() + 10.0
        legend_y = outer.top() + 7.0
        legend_gap = max(72.0, 92.0 * self._ui_scale)
        font = painter.font()
        font.setBold(True)
        font.setPixelSize(title_font_px)
        painter.setFont(font)
        for idx, (_key, label, color_text, _style) in enumerate(self._series_meta):
            color = QColor(color_text)
            block_x = legend_x + idx * legend_gap
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawRoundedRect(QRectF(block_x, legend_y + 4.0, 12.0, 6.0), 3.0, 3.0)
            painter.setPen(QColor("#1e293b"))
            painter.drawText(
                QRectF(block_x + 18.0, legend_y, legend_gap - 16.0, 16.0),
                Qt.AlignLeft | Qt.AlignVCenter,
                str(label),
            )

        painter.setPen(QPen(QColor("#e2e8f0"), 1))
        for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
            y = plot_rect.bottom() - frac * plot_rect.height()
            painter.drawLine(QPointF(plot_rect.left(), y), QPointF(plot_rect.right(), y))

        font = painter.font()
        font.setBold(False)
        font.setPixelSize(meta_font_px)
        painter.setFont(font)
        painter.setPen(QColor("#64748b"))
        painter.drawText(
            QRectF(plot_rect.left(), plot_rect.top() - 12.0, plot_rect.width(), 12.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            f"{y_max:.0f} {self._unit}".strip(),
        )
        painter.drawText(
            QRectF(plot_rect.left(), plot_rect.bottom() + 2.0, plot_rect.width(), 12.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            "120s",
        )
        painter.drawText(
            QRectF(plot_rect.left(), plot_rect.bottom() + 2.0, plot_rect.width(), 12.0),
            Qt.AlignRight | Qt.AlignVCenter,
            "Now",
        )

        any_points = False
        for key, _label, color_text, style in self._series_meta:
            history = self._history.get(str(key), deque())
            points = self._build_polyline(plot_rect, history, now_ms, y_min=y_min, y_max=y_max)
            if len(points) >= 2:
                pen = QPen(QColor(color_text), max(1.2, 2.0 * self._ui_scale))
                pen.setStyle(style)
                painter.setPen(pen)
                for idx in range(1, len(points)):
                    painter.drawLine(points[idx - 1], points[idx])
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(color_text))
                painter.drawEllipse(points[-1], max(1.8, 2.6 * self._ui_scale), max(1.8, 2.6 * self._ui_scale))
                any_points = True
            elif len(points) == 1:
                pen = QPen(QColor(color_text), max(1.1, 1.6 * self._ui_scale))
                pen.setStyle(style)
                painter.setPen(pen)
                painter.drawLine(QPointF(plot_rect.left(), points[0].y()), points[0])
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(color_text))
                painter.drawEllipse(points[0], max(1.8, 2.6 * self._ui_scale), max(1.8, 2.6 * self._ui_scale))
                any_points = True

        if not any_points:
            font = painter.font()
            font.setBold(True)
            font.setPixelSize(max(9, int(round(10 * self._ui_scale))))
            painter.setFont(font)
            painter.setPen(QColor("#94a3b8"))
            message = "Monitoring OFF" if not self._monitor_enabled else "No data"
            painter.drawText(plot_rect, Qt.AlignCenter, message)


class SepMonitorCard(QGroupBox):
    def __init__(self, aircraft_id: int, parent: QWidget | None = None) -> None:
        super().__init__(_uav_label(aircraft_id), parent)
        self._aircraft_id = int(aircraft_id)
        self._status_label: QLabel | None = None
        self._current_wp_label: QLabel | None = None
        self._actual_sep_label: QLabel | None = None
        self._trigger_sep_label: QLabel | None = None
        self._center_label: QLabel | None = None
        self._chart = TimeSeriesChartWidget(series_meta=list(_SEP_SERIES), unit="m")
        self._build_ui()

    def _build_ui(self) -> None:
        self.setStyleSheet(
            "QGroupBox { font-weight: 700; border: 1px solid #d6dee8; border-radius: 12px; margin-top: 14px; background: #ffffff; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #0f172a; }"
        )
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 18, 12, 12)
        root.setSpacing(8)

        form = QFormLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(6)
        self._status_label = QLabel("대기")
        self._current_wp_label = QLabel("-")
        self._actual_sep_label = QLabel("-")
        self._trigger_sep_label = QLabel("-")
        self._center_label = QLabel("-")
        for label in (
            self._status_label,
            self._current_wp_label,
            self._actual_sep_label,
            self._trigger_sep_label,
            self._center_label,
        ):
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        form.addRow("상태", self._status_label)
        form.addRow("현재 WP", self._current_wp_label)
        form.addRow("실측 거리", self._actual_sep_label)
        form.addRow("기준 최대거리", self._trigger_sep_label)
        form.addRow("중심좌표", self._center_label)
        root.addLayout(form)
        root.addWidget(self._chart, 1)
        self._apply_status("대기", "#64748b")

    def set_ui_scale(self, scale: float) -> None:
        self._chart.set_ui_scale(scale)

    def set_monitor_enabled(self, enabled: bool) -> None:
        self._chart.set_monitor_enabled(enabled)

    def update_snapshot(
        self,
        *,
        timestamp_ms: int | None,
        current_waypoint_id: int | None,
        actual_distance_m: float | None,
        threshold_distance_m: float | None,
        center_coordinate: dict[str, float] | None,
        redraw: bool,
    ) -> None:
        if self._current_wp_label is not None:
            self._current_wp_label.setText(str(current_waypoint_id) if current_waypoint_id is not None else "-")
        if self._actual_sep_label is not None:
            self._actual_sep_label.setText(_format_distance(actual_distance_m))
        if self._trigger_sep_label is not None:
            self._trigger_sep_label.setText(_format_distance(threshold_distance_m))
        if self._center_label is not None:
            if center_coordinate:
                self._center_label.setText(
                    f"{center_coordinate['latitude']:.5f}, {center_coordinate['longitude']:.5f}"
                )
            else:
                self._center_label.setText("-")

        status_text = "정보없음"
        status_color = "#64748b"
        if actual_distance_m is not None and threshold_distance_m is not None:
            if float(actual_distance_m) > float(threshold_distance_m):
                status_text = "초과"
                status_color = "#dc2626"
            else:
                status_text = "정상"
                status_color = "#15803d"
        elif actual_distance_m is not None:
            status_text = "기준없음"
            status_color = "#0369a1"
        self._apply_status(status_text, status_color)
        self._chart.append_samples(
            timestamp_ms=timestamp_ms,
            values={
                "actual": actual_distance_m,
                "trigger": threshold_distance_m,
            },
            redraw=redraw,
        )

    def _apply_status(self, text: str, color_text: str) -> None:
        if self._status_label is None:
            return
        self._status_label.setText(str(text))
        self._status_label.setStyleSheet(
            f"padding: 2px 8px; border-radius: 10px; color: white; background: {color_text}; font-weight: 700;"
        )


class LinkDistanceCard(QGroupBox):
    def __init__(self, aircraft_id: int, parent: QWidget | None = None) -> None:
        super().__init__(_manned_label(aircraft_id), parent)
        self._aircraft_id = int(aircraft_id)
        self._distance_labels: dict[int, QLabel] = {}
        series_meta = [
            (f"uav_{uav_id}", _LINK_SERIES_META[uav_id][0], _LINK_SERIES_META[uav_id][1], Qt.SolidLine)
            for uav_id in _UAV_IDS
        ]
        self._chart = TimeSeriesChartWidget(series_meta=series_meta, unit="m")
        self._build_ui()

    def _build_ui(self) -> None:
        self.setStyleSheet(
            "QGroupBox { font-weight: 700; border: 1px solid #d6dee8; border-radius: 12px; margin-top: 14px; background: #ffffff; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #0f172a; }"
        )
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 18, 12, 12)
        root.setSpacing(8)

        form = QFormLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(6)
        for uav_id in _UAV_IDS:
            label = QLabel("-")
            label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            self._distance_labels[int(uav_id)] = label
            form.addRow(_uav_label(uav_id), label)
        root.addLayout(form)
        root.addWidget(self._chart, 1)

    def set_ui_scale(self, scale: float) -> None:
        self._chart.set_ui_scale(scale)

    def set_monitor_enabled(self, enabled: bool) -> None:
        self._chart.set_monitor_enabled(enabled)

    def update_distances(
        self,
        *,
        timestamp_ms: int | None,
        distances_by_uav: dict[int, float | None],
        redraw: bool,
    ) -> None:
        for uav_id, label in self._distance_labels.items():
            label.setText(_format_distance(distances_by_uav.get(int(uav_id))))
        self._chart.append_samples(
            timestamp_ms=timestamp_ms,
            values={
                f"uav_{uav_id}": distances_by_uav.get(int(uav_id))
                for uav_id in _UAV_IDS
            },
            redraw=redraw,
        )


class QualityMonitorTab(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._monitor_enabled: bool = True
        self._replan_enabled: bool = False
        self._ui_updates_enabled: bool = True
        self._dirty: bool = False
        self._mission_view: dict[str, Any] | None = None
        self._waypoint_thresholds: dict[int, dict[int, dict[str, Any]]] = {uav_id: {} for uav_id in _UAV_IDS}
        self._latest_state_map: dict[int, dict[str, Any]] = {}
        self._last_payload_timestamp_ms: int | None = None
        self._last_chart_timestamp_ms: int = 0
        self._log_callback: Callable[[str], None] | None = None
        self._monitor_toggle_callback: Callable[[bool], None] | None = None
        self._replan_toggle_callback: Callable[[bool], None] | None = None
        self._plan_summary_label: QLabel | None = None
        self._monitor_state_label: QLabel | None = None
        self._monitor_toggle_button: QPushButton | None = None
        self._replan_state_label: QLabel | None = None
        self._replan_toggle_button: QPushButton | None = None
        self._sep_cards: dict[int, SepMonitorCard] = {}
        self._link_cards: dict[int, LinkDistanceCard] = {}
        self._build_ui()

    def set_log_callback(self, callback: Callable[[str], None] | None) -> None:
        self._log_callback = callback

    def set_monitor_toggle_callback(self, callback: Callable[[bool], None] | None) -> None:
        self._monitor_toggle_callback = callback

    def set_replan_toggle_callback(self, callback: Callable[[bool], None] | None) -> None:
        self._replan_toggle_callback = callback

    def is_monitor_enabled(self) -> bool:
        return bool(self._monitor_enabled)

    def is_replan_enabled(self) -> bool:
        return bool(self._replan_enabled)

    def set_monitor_enabled(self, enabled: bool, *, emit: bool = False) -> None:
        self._monitor_enabled = bool(enabled)
        self._apply_monitor_visual_state()
        if self._monitor_enabled:
            self._refresh_cards(redraw=self._ui_updates_enabled)
        else:
            self._set_plan_summary(paused=True)
        if emit and self._monitor_toggle_callback is not None:
            try:
                self._monitor_toggle_callback(self._monitor_enabled)
            except Exception:
                pass

    def set_replan_enabled(self, enabled: bool, *, emit: bool = False) -> None:
        self._replan_enabled = bool(enabled)
        self._apply_monitor_visual_state()
        if emit and self._replan_toggle_callback is not None:
            try:
                self._replan_toggle_callback(self._replan_enabled)
            except Exception:
                pass

    def set_ui_updates_enabled(self, enabled: bool) -> None:
        self._ui_updates_enabled = bool(enabled)
        if self._ui_updates_enabled and self._dirty:
            self._refresh_cards(redraw=True)

    def update_0903(
        self,
        *,
        timestamp_ms: int | None,
        mission_plan_id: int | None,
        source: str | None = None,
    ) -> None:
        _ = timestamp_ms, source
        self._apply_mission_plan_view(mission_plan_id)

    def apply_mission_plan_decision(self, *, mission_plan_id: int | None) -> None:
        self._apply_mission_plan_view(mission_plan_id)

    def update_agent_status(
        self,
        *,
        timestamp_ms: int | None,
        agent_states: list[dict[str, Any]],
        fuel_state_map: dict[int, str] | None = None,
    ) -> None:
        _ = fuel_state_map
        self._last_payload_timestamp_ms = _coerce_int(timestamp_ms)
        chart_ts = int(time.monotonic() * 1000)
        if chart_ts <= self._last_chart_timestamp_ms:
            chart_ts = self._last_chart_timestamp_ms + 1
        self._last_chart_timestamp_ms = chart_ts
        self._latest_state_map = {}
        for state in agent_states or []:
            if not isinstance(state, dict):
                continue
            aircraft_id = _coerce_int(state.get("aircraft_id") or state.get("aircraftID"))
            if aircraft_id is None or aircraft_id <= 0:
                continue
            self._latest_state_map[int(aircraft_id)] = dict(state)
        if not self._monitor_enabled:
            self._set_plan_summary(paused=True)
            self._dirty = True
            return
        self._refresh_cards(redraw=self._ui_updates_enabled)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        title = QLabel("촬영품질 모니터")
        title.setStyleSheet("font-size: 16px; font-weight: 700; color: #0f172a;")
        subtitle = QLabel(
            "0401 기반으로 UAV 좌표와 센서 중심좌표 간 거리를 실시간 확인합니다. "
            "기준 최대거리는 현재 FOV에 해당하는 DB 최대 width와 SEP로 계산한 대각선 값입니다."
        )
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet("color: #475569;")
        root.addWidget(title)
        root.addWidget(subtitle)

        toggle_row = QHBoxLayout()
        toggle_row.setSpacing(8)
        caption = QLabel("모니터 상태")
        caption.setStyleSheet("font-weight: 700;")
        self._monitor_state_label = QLabel("")
        self._monitor_toggle_button = QPushButton("")
        self._monitor_toggle_button.clicked.connect(self._toggle_monitor)
        toggle_row.addWidget(caption)
        toggle_row.addWidget(self._monitor_state_label)
        toggle_row.addStretch(1)
        toggle_row.addWidget(self._monitor_toggle_button)
        root.addLayout(toggle_row)

        decision_row = QHBoxLayout()
        decision_row.setSpacing(8)
        decision_caption = QLabel("재계획 판단")
        decision_caption.setStyleSheet("font-weight: 700;")
        self._replan_state_label = QLabel("")
        self._replan_toggle_button = QPushButton("")
        self._replan_toggle_button.clicked.connect(self._toggle_replan)
        decision_row.addWidget(decision_caption)
        decision_row.addWidget(self._replan_state_label)
        decision_row.addStretch(1)
        decision_row.addWidget(self._replan_toggle_button)
        root.addLayout(decision_row)

        self._plan_summary_label = QLabel("MissionPlan: -")
        self._plan_summary_label.setStyleSheet(
            "padding: 8px 10px; border: 1px solid #cbd5e1; border-radius: 8px; background: #f8fafc; color: #0f172a;"
        )
        root.addWidget(self._plan_summary_label)

        sep_group = QGroupBox("촬영품질")
        sep_group.setStyleSheet(
            "QGroupBox { font-weight: 700; border: 1px solid #cbd5e1; border-radius: 12px; margin-top: 12px; background: #eff6ff; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #1d4ed8; }"
        )
        sep_layout = QHBoxLayout(sep_group)
        sep_layout.setContentsMargins(10, 18, 10, 10)
        sep_layout.setSpacing(10)
        for uav_id in _UAV_IDS:
            card = SepMonitorCard(uav_id)
            self._sep_cards[int(uav_id)] = card
            sep_layout.addWidget(card, 1)
        root.addWidget(sep_group, 1)

        link_group = QGroupBox("유인기-무인기 통신거리 확인")
        link_group.setStyleSheet(
            "QGroupBox { font-weight: 700; border: 1px solid #cbd5e1; border-radius: 12px; margin-top: 12px; background: #f8fafc; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #0f172a; }"
        )
        link_layout = QHBoxLayout(link_group)
        link_layout.setContentsMargins(10, 18, 10, 10)
        link_layout.setSpacing(10)
        for manned_id in _MANNED_IDS:
            card = LinkDistanceCard(manned_id)
            self._link_cards[int(manned_id)] = card
            link_layout.addWidget(card, 1)
        root.addWidget(link_group, 1)

        self._apply_monitor_visual_state()

    def _toggle_monitor(self) -> None:
        self.set_monitor_enabled(not self._monitor_enabled)
        if self._log_callback is not None:
            try:
                state_text = "enabled" if self._monitor_enabled else "disabled"
                self._log_callback(f"[QUALITY] 촬영품질 모니터 {state_text}")
            except Exception:
                pass
        if self._monitor_toggle_callback is not None:
            try:
                self._monitor_toggle_callback(self._monitor_enabled)
            except Exception:
                pass

    def _toggle_replan(self) -> None:
        self.set_replan_enabled(not self._replan_enabled)
        if self._log_callback is not None:
            try:
                state_text = "enabled" if self._replan_enabled else "disabled"
                self._log_callback(f"[QUALITY] 촬영품질 재계획 {state_text}")
            except Exception:
                pass
        if self._replan_toggle_callback is not None:
            try:
                self._replan_toggle_callback(self._replan_enabled)
            except Exception:
                pass

    def _apply_monitor_visual_state(self) -> None:
        enabled = bool(self._monitor_enabled)
        text = "ON" if enabled else "OFF"
        color_text = "#15803d" if enabled else "#64748b"
        button_text = "끄기" if enabled else "켜기"
        button_bg = "#dbeafe" if enabled else "#e2e8f0"
        replan_enabled = bool(self._replan_enabled)
        replan_text = "ON" if replan_enabled else "OFF"
        replan_color = "#15803d" if replan_enabled else "#64748b"
        replan_button_text = "끄기" if replan_enabled else "켜기"
        replan_button_bg = "#dbeafe" if replan_enabled else "#e2e8f0"
        if self._monitor_state_label is not None:
            self._monitor_state_label.setText(text)
            self._monitor_state_label.setStyleSheet(
                f"padding: 2px 8px; border-radius: 10px; color: white; background: {color_text}; font-weight: 700;"
            )
        if self._monitor_toggle_button is not None:
            self._monitor_toggle_button.setText(button_text)
            self._monitor_toggle_button.setStyleSheet(
                f"QPushButton {{ background: {button_bg}; color: #0f172a; padding: 6px 14px; border-radius: 8px; }}"
            )
        if self._replan_state_label is not None:
            self._replan_state_label.setText(replan_text)
            self._replan_state_label.setStyleSheet(
                f"padding: 2px 8px; border-radius: 10px; color: white; background: {replan_color}; font-weight: 700;"
            )
        if self._replan_toggle_button is not None:
            self._replan_toggle_button.setText(replan_button_text)
            self._replan_toggle_button.setStyleSheet(
                f"QPushButton {{ background: {replan_button_bg}; color: #0f172a; padding: 6px 14px; border-radius: 8px; }}"
            )
        for card in list(self._sep_cards.values()) + list(self._link_cards.values()):
            card.set_monitor_enabled(enabled)

    def _apply_mission_plan_view(self, mission_plan_id: int | None) -> None:
        self._mission_view = build_uav_mission_view(mission_plan_id, uav_ids=_UAV_IDS)
        thresholds: dict[int, dict[int, dict[str, Any]]] = {uav_id: {} for uav_id in _UAV_IDS}
        for entry in (self._mission_view or {}).get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = _coerce_int(entry.get("aircraft_id"))
            if aircraft_id is None or aircraft_id not in _UAV_IDS:
                continue
            waypoint_map = thresholds.setdefault(int(aircraft_id), {})
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                sep_m = _coerce_float(mission.get("sep_m"))
                if sep_m is None or sep_m <= 0.0:
                    continue
                meta = {
                    "sep_m": float(sep_m),
                    "width_m": _coerce_float(mission.get("width_m")),
                    "individual_mission_id": _coerce_int(mission.get("individual_mission_id")),
                    "path_id": _coerce_int(mission.get("path_id")),
                }
                for waypoint_id in mission.get("waypoint_ids") or []:
                    wid = _coerce_int(waypoint_id)
                    if wid is None or wid <= 0:
                        continue
                    waypoint_map[int(wid)] = dict(meta)
        self._waypoint_thresholds = thresholds
        self._refresh_cards(redraw=self._ui_updates_enabled)

    def _set_plan_summary(self, *, paused: bool = False) -> None:
        if self._plan_summary_label is None:
            return
        mission_plan_id = (self._mission_view or {}).get("mission_plan_id")
        updated_text = (
            format_timestamp_ms(self._last_payload_timestamp_ms)
            if self._last_payload_timestamp_ms is not None
            else "-"
        )
        suffix = " / 상태: 일시중지" if paused else ""
        self._plan_summary_label.setText(
            f"MissionPlan: {mission_plan_id or '-'} / Last Update: {updated_text}{suffix}"
        )

    def _refresh_cards(self, *, redraw: bool) -> None:
        self._dirty = True
        self._set_plan_summary(paused=not self._monitor_enabled)
        if not self._monitor_enabled:
            return
        chart_timestamp_ms = self._last_chart_timestamp_ms
        if chart_timestamp_ms <= 0:
            chart_timestamp_ms = int(time.monotonic() * 1000)
            self._last_chart_timestamp_ms = chart_timestamp_ms

        for uav_id, card in self._sep_cards.items():
            state = self._latest_state_map.get(int(uav_id), {})
            coordinate = _normalize_coordinate(state.get("coordinate"))
            center_coordinate = _normalize_coordinate(state.get("sensor_center_coordinate"))
            current_waypoint_id = _coerce_int(state.get("current_waypoint_id"))
            actual_distance_m = _ground_distance_m(coordinate, center_coordinate)
            threshold_distance_m = None
            if current_waypoint_id is not None:
                trigger_meta = self._waypoint_thresholds.get(int(uav_id), {}).get(int(current_waypoint_id), {})
                sep_m = _coerce_float(trigger_meta.get("sep_m"))
                sensor_fov_deg = _coerce_float(state.get("sensor_fov_deg"))
                fov_width_m = lookup_fov_db_max_width_m(sensor_fov_deg)
                if fov_width_m is None:
                    fov_width_m = _coerce_float(trigger_meta.get("width_m"))
                threshold_distance_m = compute_filming_quality_threshold_m(sep_m, fov_width_m)
            card.update_snapshot(
                timestamp_ms=chart_timestamp_ms,
                current_waypoint_id=current_waypoint_id,
                actual_distance_m=actual_distance_m,
                threshold_distance_m=threshold_distance_m,
                center_coordinate=center_coordinate,
                redraw=redraw,
            )

        for manned_id, card in self._link_cards.items():
            manned_state = self._latest_state_map.get(int(manned_id), {})
            manned_coordinate = _normalize_coordinate(manned_state.get("coordinate"))
            distances: dict[int, float | None] = {}
            for uav_id in _UAV_IDS:
                uav_state = self._latest_state_map.get(int(uav_id), {})
                uav_coordinate = _normalize_coordinate(uav_state.get("coordinate"))
                distances[int(uav_id)] = _slant_distance_m(manned_coordinate, uav_coordinate)
            card.update_distances(
                timestamp_ms=chart_timestamp_ms,
                distances_by_uav=distances,
                redraw=redraw,
            )

        self._dirty = not redraw
