# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import json
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
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
_SERIES_MIN_APPEND_INTERVAL_MS = 250
_QUALITY_CHART_REDRAW_INTERVAL_MS = 120

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
_GSD_SERIES = (
    ("eq_gsd", "등가 GSD", "#2563eb", Qt.SolidLine),
    ("req_gsd", "요구 GSD(등가)", "#ef4444", Qt.DashLine),
)

# Default spatial-resolution settings
_SR_DEFAULT_IMG_W = 1920
_SR_DEFAULT_IMG_H = 1080
_SR_DEFAULT_OBJ_W_M = 6.0
_SR_DEFAULT_OBJ_H_M = 3.6
_SR_DEFAULT_OBJ_MIN_PX_X = 38
_SR_DEFAULT_OBJ_MIN_PX_Y = 22
_SR_SETTINGS_VERSION = 1
_SR_SETTINGS_LOCK = threading.Lock()


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


def _sr_settings_path() -> Path:
    return Path(__file__).resolve().parents[2] / "quality_monitor_settings.json"


def _normalize_sr_settings(payload: object | None) -> dict[str, Any]:
    root = dict(payload) if isinstance(payload, dict) else {}
    sr_raw = root.get("spatial_resolution")
    if not isinstance(sr_raw, dict):
        sr_raw = {}

    img_w_px = max(1, _coerce_int(sr_raw.get("img_w_px")) or _SR_DEFAULT_IMG_W)
    img_h_px = max(1, _coerce_int(sr_raw.get("img_h_px")) or _SR_DEFAULT_IMG_H)
    obj_w_m = max(0.01, _coerce_float(sr_raw.get("obj_w_m")) or _SR_DEFAULT_OBJ_W_M)
    obj_h_m = max(0.01, _coerce_float(sr_raw.get("obj_h_m")) or _SR_DEFAULT_OBJ_H_M)
    obj_min_px_x = max(1, _coerce_int(sr_raw.get("obj_min_px_x")) or _SR_DEFAULT_OBJ_MIN_PX_X)
    obj_min_px_y = max(1, _coerce_int(sr_raw.get("obj_min_px_y")) or _SR_DEFAULT_OBJ_MIN_PX_Y)

    return {
        "version": int(_SR_SETTINGS_VERSION),
        "spatial_resolution": {
            "img_w_px": int(img_w_px),
            "img_h_px": int(img_h_px),
            "obj_w_m": float(obj_w_m),
            "obj_h_m": float(obj_h_m),
            "obj_min_px_x": int(obj_min_px_x),
            "obj_min_px_y": int(obj_min_px_y),
        },
    }


def _load_sr_settings() -> dict[str, Any]:
    path = _sr_settings_path()
    with _SR_SETTINGS_LOCK:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
    return _normalize_sr_settings(payload)


def _save_sr_settings(payload: dict[str, Any]) -> None:
    path = _sr_settings_path()
    normalized = _normalize_sr_settings(payload)
    encoded = json.dumps(normalized, ensure_ascii=False, indent=2) + "\n"
    tmp_path = path.with_name(f"{path.stem}.tmp{path.suffix}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with _SR_SETTINGS_LOCK:
        tmp_path.write_text(encoded, encoding="utf-8")
        tmp_path.replace(path)


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


def _footprint_size_m(
    corners: list[dict] | None,
) -> tuple[float | None, float | None]:
    """footprintCornerList 4점으로부터 실제 촬영 폭(W)과 높이(H)를 미터 단위로 계산.
    예상 순서: [UL, UR, LR, LL] (시뮬레이터 출력 기준).
    Returns: (width_m, height_m)
    """
    if not corners or len(corners) < 4:
        return None, None
    c = [_normalize_coordinate(pt) for pt in corners[:4]]
    if any(ci is None for ci in c):
        return None, None
    top_w = _ground_distance_m(c[0], c[1])
    bot_w = _ground_distance_m(c[3], c[2])
    left_h = _ground_distance_m(c[0], c[3])
    right_h = _ground_distance_m(c[1], c[2])
    w: float | None = None
    if top_w is not None and bot_w is not None:
        w = (top_w + bot_w) * 0.5
    elif top_w is not None:
        w = top_w
    elif bot_w is not None:
        w = bot_w
    h: float | None = None
    if left_h is not None and right_h is not None:
        h = (left_h + right_h) * 0.5
    elif left_h is not None:
        h = left_h
    elif right_h is not None:
        h = right_h
    return w, h


def _is_sweep_measurement_active(
    waypoint_meta: dict[str, Any] | None,
    *,
    sensor_operation_mode: int | None,
) -> bool:
    meta = waypoint_meta if isinstance(waypoint_meta, dict) else {}
    line_search_point_count = _coerce_int(meta.get("line_search_point_count")) or 0
    has_line_search = bool(meta.get("has_line_search")) or line_search_point_count >= 2
    waypoint_operation_mode = _coerce_int(meta.get("operation_mode"))
    if not has_line_search:
        return False
    if waypoint_operation_mode is not None and int(waypoint_operation_mode) != 2:
        return False
    if sensor_operation_mode is not None and int(sensor_operation_mode) != 2:
        return False
    return True


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
        self._sample_min_interval_ms: int = _SERIES_MIN_APPEND_INTERVAL_MS
        self._ui_scale: float = 1.0
        self._monitor_enabled: bool = True
        self.setMinimumHeight(180)
        self.setProperty("_base_min_height", 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_ui_scale(self, scale: float) -> None:
        try:
            self._ui_scale = max(0.62, min(1.15, float(scale)))
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
            numeric_value = max(0.0, float(numeric))
            if not history:
                bootstrap_ts = max(0, int(ts_ms) - 1000)
                history.append((bootstrap_ts, numeric_value))
                history.append((int(ts_ms), numeric_value))
                continue
            last_ts_ms = int(history[-1][0])
            if int(ts_ms) <= last_ts_ms:
                history[-1] = (int(ts_ms), numeric_value)
                continue
            if int(ts_ms) - last_ts_ms < int(self._sample_min_interval_ms):
                history[-1] = (int(ts_ms), numeric_value)
                continue
            history.append((int(ts_ms), numeric_value))
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

    def _visible_min_max(self, now_ms: int) -> tuple[float, float] | None:
        cutoff = int(now_ms) - int(self._window_ms)
        lo = float("inf")
        hi = float("-inf")
        found = False
        for history in self._history.values():
            for ts_ms, value in reversed(history):
                if ts_ms < cutoff:
                    break
                v = float(value)
                if v < lo:
                    lo = v
                if v > hi:
                    hi = v
                found = True
        return (lo, hi) if found else None

    def _build_polyline(
        self,
        plot_rect: QRectF,
        history: deque[tuple[int, float]],
        now_ms: int,
        *,
        y_min: float,
        y_max: float,
    ) -> QPolygonF:
        poly = QPolygonF()
        if not history:
            return poly
        cutoff = int(now_ms) - int(self._window_ms)
        span_y = max(1e-6, float(y_max) - float(y_min))
        right = plot_rect.right()
        bottom = plot_rect.bottom()
        w = plot_rect.width()
        h = plot_rect.height()
        win = float(self._window_ms)
        for ts_ms, value in history:
            if ts_ms < cutoff:
                continue
            age_ms = max(0, int(now_ms) - int(ts_ms))
            x = right - (float(age_ms) / win) * w
            normalized = (float(value) - float(y_min)) / span_y
            y = bottom - normalized * h
            poly.append(QPointF(x, y))
        return poly

    def paintEvent(self, event) -> None:  # type: ignore[override]
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#fbfcfe"))

        outer = QRectF(0.5, 0.5, max(1.0, self.width() - 1.0), max(1.0, self.height() - 1.0))
        painter.setPen(QPen(QColor("#dde4ee"), 1))
        painter.setBrush(QColor("#ffffff"))
        painter.drawRoundedRect(outer, 8.0, 8.0)

        compact = self._ui_scale < 0.75
        chart_margin_x = 8.0 if compact else 10.0
        header_gap = 4.0 if compact else 6.0
        bottom_pad = 18.0 if compact else 24.0
        title_font_px = max(7, int(round(9.5 * self._ui_scale)))
        meta_font_px = max(6, int(round(8.0 * self._ui_scale)))
        header_h = max(15.0, 24.0 * self._ui_scale)
        plot_rect = QRectF(
            outer.left() + chart_margin_x,
            outer.top() + header_h + header_gap,
            max(16.0, outer.width() - (chart_margin_x * 2.0)),
            max(16.0, outer.height() - header_h - bottom_pad),
        )
        now_ms = self._latest_ts_ms if self._latest_ts_ms > 0 else int(time.time() * 1000)
        min_max = self._visible_min_max(now_ms)
        if min_max is not None:
            y_min = min(0.0, min_max[0])
            y_max = min_max[1]
            margin = max(1.0, (y_max - y_min) * 0.18, y_max * 0.08)
            y_max += margin
        else:
            y_min = 0.0
            y_max = 1.0

        legend_x = outer.left() + chart_margin_x
        legend_y = outer.top() + (5.0 if compact else 7.0)
        series_count = max(1, len(self._series_meta))
        available_legend_w = max(48.0, outer.width() - (chart_margin_x * 2.0))
        legend_gap_min = 46.0 if compact else 54.0
        legend_gap = max(legend_gap_min, min(92.0 * self._ui_scale, available_legend_w / float(series_count)))
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
            text_rect = QRectF(block_x + 18.0, legend_y, max(18.0, legend_gap - 20.0), 16.0)
            label_text = str(label)
            try:
                label_text = painter.fontMetrics().elidedText(
                    label_text,
                    Qt.ElideRight,
                    max(12, int(text_rect.width())),
                )
            except Exception:
                pass
            painter.drawText(
                text_rect,
                Qt.AlignLeft | Qt.AlignVCenter,
                label_text,
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
            poly = self._build_polyline(plot_rect, history, now_ms, y_min=y_min, y_max=y_max)
            n = poly.count()
            if n >= 2:
                pen = QPen(QColor(color_text), max(1.2, 2.0 * self._ui_scale))
                pen.setStyle(style)
                painter.setPen(pen)
                painter.setBrush(Qt.NoBrush)
                painter.drawPolyline(poly)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(color_text))
                last_pt = poly.at(n - 1)
                painter.drawEllipse(last_pt, max(1.8, 2.6 * self._ui_scale), max(1.8, 2.6 * self._ui_scale))
                any_points = True
            elif n == 1:
                pt = poly.at(0)
                pen = QPen(QColor(color_text), max(1.1, 1.6 * self._ui_scale))
                pen.setStyle(style)
                painter.setPen(pen)
                painter.drawLine(QPointF(plot_rect.left(), pt.y()), pt)
                painter.setPen(Qt.NoPen)
                painter.setBrush(QColor(color_text))
                painter.drawEllipse(pt, max(1.8, 2.6 * self._ui_scale), max(1.8, 2.6 * self._ui_scale))
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
            f"padding: 1px 8px; border-radius: 8px; color: white; background: {color_text}; font-weight: 700;"
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


class SpatialResolutionCard(QGroupBox):
    """UAV별 공간해상도(GSD) 모니터링 카드.

    footprintCornerList 4점 → 실제 촬영 폭·높이 → GSD(m/px) 계산 후
    탐지 요구조건(객체 크기 × 최소 픽셀) 대비 만족 여부와 누적 만족률을 표시한다.
    """

    def __init__(
        self,
        aircraft_id: int,
        parent: QWidget | None = None,
        *,
        compact_horizontal: bool = False,
    ) -> None:
        super().__init__(_uav_label(aircraft_id), parent)
        self._aircraft_id = int(aircraft_id)
        self._compact_horizontal = bool(compact_horizontal)
        self._sample_count: int = 0
        self._pass_count: int = 0
        self._fp_label: QLabel | None = None
        self._eq_gsd_label: QLabel | None = None
        self._req_label: QLabel | None = None
        self._satisfy_label: QLabel | None = None
        self._status_label: QLabel | None = None
        self._chart = TimeSeriesChartWidget(series_meta=list(_GSD_SERIES), unit="m/px")
        self._build_ui()

    def _build_ui(self) -> None:
        if self._compact_horizontal:
            self.setTitle("")
        self.setStyleSheet(
            "QGroupBox { font-weight: 700; border: 1px solid #d6dee8; border-radius: 12px;"
            " margin-top: 14px; background: #ffffff; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #0f172a; }"
        )
        if self._compact_horizontal:
            root = QVBoxLayout(self)
            root.setContentsMargins(9, 8, 9, 7)
            root.setSpacing(4)
        else:
            root = QVBoxLayout(self)
            root.setContentsMargins(12, 18, 12, 12)
            root.setSpacing(8)
        form = QFormLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(8 if self._compact_horizontal else 10)
        form.setVerticalSpacing(3 if self._compact_horizontal else 6)
        self._fp_label = QLabel("-")
        self._eq_gsd_label = QLabel("-")
        self._req_label = QLabel("-")
        self._satisfy_label = QLabel("-")
        self._status_label = QLabel("대기")
        for lbl in (
            self._fp_label,
            self._eq_gsd_label,
            self._req_label,
            self._satisfy_label,
            self._status_label,
        ):
            lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        if self._compact_horizontal:
            header = QHBoxLayout()
            header.setContentsMargins(0, 0, 0, 0)
            header.setSpacing(8)
            title = QLabel(_uav_label(self._aircraft_id), self)
            title.setStyleSheet("color:#0f172a; font-size:11px; font-weight:700;")
            header.addWidget(title, 1, Qt.AlignLeft | Qt.AlignVCenter)
            self._status_label.setAlignment(Qt.AlignCenter)
            self._status_label.setMinimumWidth(62)
            self._status_label.setMaximumWidth(72)
            self._status_label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            header.addWidget(self._status_label, 0, Qt.AlignRight | Qt.AlignVCenter)
            root.addLayout(header)

            metrics = QGridLayout()
            metrics.setContentsMargins(0, 0, 0, 0)
            metrics.setHorizontalSpacing(8)
            metrics.setVerticalSpacing(1)
            label_style = "color:#475569; font-size:10px; font-weight:700;"
            value_style = "color:#0f172a; font-size:10px;"
            pairs = (
                ("풋프린트", self._fp_label, 0, 0, 1, False),
                ("등가 GSD", self._eq_gsd_label, 0, 2, 1, False),
                ("요구 GSD", self._req_label, 1, 0, 3, True),
                ("만족률", self._satisfy_label, 1, 4, 1, False),
            )
            for label_text, value_label, row, col, span, should_wrap in pairs:
                label = QLabel(label_text, self)
                label.setStyleSheet(label_style)
                value_label.setStyleSheet(value_style)
                value_label.setWordWrap(bool(should_wrap))
                metrics.addWidget(label, row, col)
                metrics.addWidget(value_label, row, col + 1, 1, span)
            metrics.setColumnStretch(1, 1)
            metrics.setColumnStretch(3, 1)
            metrics.setColumnStretch(5, 1)
            root.addLayout(metrics)
            self._chart.setMinimumHeight(68)
            self._chart.set_ui_scale(0.72)
            root.addWidget(self._chart, 1)
            self._apply_status("대기", "#64748b")
            return
        form.addRow("풋프린트", self._fp_label)
        form.addRow("등가 GSD", self._eq_gsd_label)
        form.addRow("요구 GSD(등가)", self._req_label)
        form.addRow("만족률", self._satisfy_label)
        form.addRow("상태", self._status_label)
        root.addLayout(form)
        root.addWidget(self._chart, 1)
        self._apply_status("대기", "#64748b")

    def set_ui_scale(self, scale: float) -> None:
        self._chart.set_ui_scale(scale)

    def set_monitor_enabled(self, enabled: bool) -> None:
        self._chart.set_monitor_enabled(enabled)

    def reset_history(self) -> None:
        self._sample_count = 0
        self._pass_count = 0

    def update_snapshot(
        self,
        *,
        timestamp_ms: int | None,
        measurement_enabled: bool,
        footprint_corners: list[dict] | None,
        img_w_px: int,
        img_h_px: int,
        obj_w_m: float,
        obj_h_m: float,
        obj_min_px_x: int,
        obj_min_px_y: int,
        redraw: bool,
    ) -> None:
        fp_w, fp_h = _footprint_size_m(footprint_corners)

        # 개별 축 GSD (차트 참조용)
        eq_gsd: float | None = None

        # ─── 논문(JANT 2024) 기반 면적 GSD ───────────────────────────────────
        # γ = (obj_w × obj_h) / (min_px_x × min_px_y)  [m²/px]
        # s_req = γ × (img_w × img_h)                   [m²]  → 허용 최대 풋프린트 면적
        # 만족 조건: fp_w × fp_h ≤ s_req
        # 차트 기준선: req_gsd = √γ  (등가 선형 GSD)
        total_px = max(1, img_w_px * img_h_px)
        gamma: float | None = None
        req_fp_area: float | None = None
        req_gsd: float | None = None
        if obj_min_px_x > 0 and obj_min_px_y > 0:
            gamma = (float(obj_w_m) * float(obj_h_m)) / float(obj_min_px_x * obj_min_px_y)
            req_fp_area = gamma * float(total_px)
            req_gsd = math.sqrt(gamma) if gamma > 0.0 else None

        if self._req_label is not None:
            if req_gsd is not None and req_fp_area is not None:
                self._req_label.setText(
                    f"{req_gsd:.4f} m/px  (풋프린트 ≤ {req_fp_area:.0f} m²)"
                )
            else:
                self._req_label.setText("-")

        if not measurement_enabled:
            if self._fp_label is not None:
                self._fp_label.setText("-")
            if self._eq_gsd_label is not None:
                self._eq_gsd_label.setText("-")
            if self._satisfy_label is not None:
                self._satisfy_label.setText("-")
            self._apply_status("스윕대기", "#64748b")
            self._chart.append_samples(
                timestamp_ms=timestamp_ms,
                values={"req_gsd": req_gsd},
                redraw=redraw,
            )
            return

        fp_area: float | None = fp_w * fp_h if (fp_w is not None and fp_h is not None) else None
        if fp_area is not None and fp_area >= 0.0 and total_px > 0:
            eq_gsd = math.sqrt(fp_area / float(total_px))
        have_data = fp_w is not None and fp_h is not None
        satisfied: bool | None = None
        if have_data and fp_area is not None and req_fp_area is not None:
            satisfied = bool(fp_area <= req_fp_area)
            self._sample_count += 1
            if satisfied:
                self._pass_count += 1

        # 만족률 계산
        if self._sample_count > 0:
            n_pass = min(self._pass_count, self._sample_count)
            rate = n_pass / float(self._sample_count) * 100.0
            satisfy_text = f"{rate:.1f}%  ({n_pass}/{self._sample_count})"
        else:
            satisfy_text = "-"

        # 라벨 갱신
        if self._fp_label is not None:
            if fp_w is not None and fp_h is not None and fp_area is not None:
                self._fp_label.setText(f"{fp_w:.0f} m × {fp_h:.0f} m  ({fp_area:.0f} m²)")
            else:
                self._fp_label.setText("-")
        if self._eq_gsd_label is not None:
            self._eq_gsd_label.setText(f"{eq_gsd:.4f} m/px" if eq_gsd is not None else "-")
        if self._satisfy_label is not None:
            self._satisfy_label.setText(satisfy_text)

        if satisfied is True:
            self._apply_status("만족", "#15803d")
        elif satisfied is False:
            self._apply_status("미달", "#dc2626")
        elif have_data:
            self._apply_status("기준없음", "#0369a1")
        else:
            self._apply_status("데이터없음", "#64748b")

        self._chart.append_samples(
            timestamp_ms=timestamp_ms,
            values={"eq_gsd": eq_gsd, "req_gsd": req_gsd},
            redraw=redraw,
        )

    def _apply_status(self, text: str, color_text: str) -> None:
        if self._status_label is None:
            return
        self._status_label.setText(str(text))
        self._status_label.setStyleSheet(
            f"padding: 1px 8px; border-radius: 8px; color: white;"
            f" background: {color_text}; font-weight: 700;"
        )


class SpatialResolutionMonitorPanel(QWidget):
    """Dashboard-sized spatial-resolution monitor reused from the quality tab."""

    def __init__(self, parent: QWidget | None = None, *, compact: bool = True) -> None:
        super().__init__(parent)
        self._compact = bool(compact)
        self._monitor_enabled: bool = True
        self._ui_updates_enabled: bool = True
        self._dirty: bool = False
        self._mission_view: dict[str, Any] | None = None
        self._waypoint_thresholds: dict[int, dict[int, dict[str, Any]]] = {uav_id: {} for uav_id in _UAV_IDS}
        self._latest_state_map: dict[int, dict[str, Any]] = {}
        self._last_payload_timestamp_ms: int | None = None
        self._last_chart_timestamp_ms: int = 0
        self._last_chart_redraw_monotonic_ms: int = 0
        self._log_callback: Callable[[str], None] | None = None
        self._plan_summary_label: QLabel | None = None
        self._sr_cards: dict[int, SpatialResolutionCard] = {}
        self._sr_img_w_spin: QSpinBox | None = None
        self._sr_img_h_spin: QSpinBox | None = None
        self._sr_obj_w_spin: QDoubleSpinBox | None = None
        self._sr_obj_h_spin: QDoubleSpinBox | None = None
        self._sr_obj_px_x_spin: QSpinBox | None = None
        self._sr_obj_px_y_spin: QSpinBox | None = None
        self._sr_req_hint_label: QLabel | None = None
        self._sr_settings_sync_enabled: bool = False
        self._build_ui()
        self._restore_spatial_resolution_settings()

    def set_log_callback(self, callback: Callable[[str], None] | None) -> None:
        self._log_callback = callback

    def set_monitor_enabled(self, enabled: bool) -> None:
        self._monitor_enabled = bool(enabled)
        for card in self._sr_cards.values():
            card.set_monitor_enabled(self._monitor_enabled)
        if self._monitor_enabled:
            self._refresh_cards(redraw=self._ui_updates_enabled)
        else:
            self._set_plan_summary(paused=True)

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
        chart_ts = _coerce_int(timestamp_ms)
        if chart_ts is None or chart_ts <= 0:
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
        if not self._ui_updates_enabled:
            self._dirty = True
            return
        redraw = bool(self._ui_updates_enabled)
        if redraw:
            now_ms = int(time.monotonic() * 1000)
            if (
                self._last_chart_redraw_monotonic_ms > 0
                and (now_ms - self._last_chart_redraw_monotonic_ms) < _QUALITY_CHART_REDRAW_INTERVAL_MS
            ):
                redraw = False
            else:
                self._last_chart_redraw_monotonic_ms = now_ms
        self._refresh_cards(redraw=redraw)

    def _build_ui(self) -> None:
        if self._compact:
            self._build_compact_ui_wide()
            return

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        self._plan_summary_label = QLabel("MissionPlan: -", self)
        self._plan_summary_label.setStyleSheet(
            "padding: 7px 9px; border: 1px solid #cbd5e1; border-radius: 8px;"
            " background: #f8fafc; color: #0f172a; font-size: 11px;"
        )
        root.addWidget(self._plan_summary_label)

        cfg = QWidget(self)
        cfg.setObjectName("SpatialResolutionConfig")
        cfg.setAttribute(Qt.WA_StyledBackground, True)
        cfg.setStyleSheet(
            "QWidget#SpatialResolutionConfig {"
            "background:#f8fafc; border:1px solid #dbe5ef; border-radius:8px;"
            "}"
        )
        cfg_layout = QGridLayout(cfg)
        cfg_layout.setContentsMargins(10, 8, 10, 8)
        cfg_layout.setHorizontalSpacing(8)
        cfg_layout.setVerticalSpacing(6)

        self._sr_img_w_spin = self._make_int_spin(_SR_DEFAULT_IMG_W, " px", 90)
        self._sr_img_h_spin = self._make_int_spin(_SR_DEFAULT_IMG_H, " px", 90)
        self._sr_obj_w_spin = self._make_float_spin(_SR_DEFAULT_OBJ_W_M, " m", 90)
        self._sr_obj_h_spin = self._make_float_spin(_SR_DEFAULT_OBJ_H_M, " m", 90)
        self._sr_obj_px_x_spin = self._make_int_spin(_SR_DEFAULT_OBJ_MIN_PX_X, " px", 82)
        self._sr_obj_px_y_spin = self._make_int_spin(_SR_DEFAULT_OBJ_MIN_PX_Y, " px", 82)

        rows = (
            ("이미지 W", self._sr_img_w_spin, "이미지 H", self._sr_img_h_spin),
            ("객체 W", self._sr_obj_w_spin, "객체 H", self._sr_obj_h_spin),
            ("최소 px X", self._sr_obj_px_x_spin, "최소 px Y", self._sr_obj_px_y_spin),
        )
        for row_idx, (left_label, left_widget, right_label, right_widget) in enumerate(rows):
            cfg_layout.addWidget(self._param_label(left_label), row_idx, 0)
            cfg_layout.addWidget(left_widget, row_idx, 1)
            cfg_layout.addWidget(self._param_label(right_label), row_idx, 2)
            cfg_layout.addWidget(right_widget, row_idx, 3)
        cfg_layout.setColumnStretch(4, 1)

        for spin in (
            self._sr_img_w_spin, self._sr_img_h_spin,
            self._sr_obj_w_spin, self._sr_obj_h_spin,
            self._sr_obj_px_x_spin, self._sr_obj_px_y_spin,
        ):
            spin.valueChanged.connect(self._on_sr_setting_changed)

        root.addWidget(cfg, 0)

        self._sr_req_hint_label = QLabel("", self)
        self._sr_req_hint_label.setStyleSheet("color: #64748b; font-size: 11px;")
        self._sr_req_hint_label.setWordWrap(True)
        root.addWidget(self._sr_req_hint_label)

        cards_row = QVBoxLayout()
        cards_row.setContentsMargins(0, 0, 0, 0)
        cards_row.setSpacing(6)
        for uav_id in _UAV_IDS:
            card = SpatialResolutionCard(
                uav_id,
                compact_horizontal=bool(self._compact),
            )
            if self._compact:
                self._apply_compact_card_style(card)
            self._sr_cards[int(uav_id)] = card
            cards_row.addWidget(card, 0)
        root.addLayout(cards_row, 1)

        self._update_sr_req_hint()

    def _build_compact_ui_wide(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(6)

        control_panel = QWidget(self)
        control_panel.setObjectName("SpatialResolutionControlPanel")
        control_panel.setAttribute(Qt.WA_StyledBackground, True)
        control_panel.setStyleSheet(
            "QWidget#SpatialResolutionControlPanel {"
            "background:#fbfcfe; border:1px solid #dde4ee; border-radius:10px;"
            "}"
        )
        control_grid = QGridLayout(control_panel)
        control_grid.setContentsMargins(8, 6, 8, 6)
        control_grid.setHorizontalSpacing(7)
        control_grid.setVerticalSpacing(4)

        self._plan_summary_label = QLabel("MissionPlan: -", control_panel)
        self._plan_summary_label.setStyleSheet(
            "padding: 5px 7px; border: 1px solid #d8dee8; border-radius: 8px;"
            " background: #ffffff; color: #0f172a; font-size: 10px;"
        )
        control_grid.addWidget(self._plan_summary_label, 0, 0, 1, 6)

        self._sr_img_w_spin = self._make_int_spin(_SR_DEFAULT_IMG_W, " px", 100)
        self._sr_img_h_spin = self._make_int_spin(_SR_DEFAULT_IMG_H, " px", 100)
        self._sr_obj_w_spin = self._make_float_spin(_SR_DEFAULT_OBJ_W_M, " m", 96)
        self._sr_obj_h_spin = self._make_float_spin(_SR_DEFAULT_OBJ_H_M, " m", 96)
        self._sr_obj_px_x_spin = self._make_int_spin(_SR_DEFAULT_OBJ_MIN_PX_X, " px", 96)
        self._sr_obj_px_y_spin = self._make_int_spin(_SR_DEFAULT_OBJ_MIN_PX_Y, " px", 96)

        params = (
            ("이미지 W", self._sr_img_w_spin),
            ("이미지 H", self._sr_img_h_spin),
            ("객체 W", self._sr_obj_w_spin),
            ("객체 H", self._sr_obj_h_spin),
            ("최소 px X", self._sr_obj_px_x_spin),
            ("최소 px Y", self._sr_obj_px_y_spin),
        )
        for idx, (label_text, widget) in enumerate(params):
            row = 1 + idx // 3
            col = (idx % 3) * 2
            control_grid.addWidget(self._param_label(label_text), row, col)
            control_grid.addWidget(widget, row, col + 1)
        control_grid.setColumnStretch(5, 1)

        for spin in (
            self._sr_img_w_spin, self._sr_img_h_spin,
            self._sr_obj_w_spin, self._sr_obj_h_spin,
            self._sr_obj_px_x_spin, self._sr_obj_px_y_spin,
        ):
            spin.valueChanged.connect(self._on_sr_setting_changed)

        self._sr_req_hint_label = QLabel("", control_panel)
        self._sr_req_hint_label.setStyleSheet("color: #64748b; font-size: 10px;")
        self._sr_req_hint_label.setWordWrap(True)
        control_grid.addWidget(self._sr_req_hint_label, 3, 0, 1, 6)
        root.addWidget(control_panel, 0)

        cards_col = QVBoxLayout()
        cards_col.setContentsMargins(0, 0, 0, 0)
        cards_col.setSpacing(5)
        for uav_id in _UAV_IDS:
            card = SpatialResolutionCard(
                uav_id,
                compact_horizontal=True,
            )
            self._apply_compact_card_style(card)
            self._sr_cards[int(uav_id)] = card
            cards_col.addWidget(card, 1)

        root.addLayout(cards_col, 1)
        self._update_sr_req_hint()

    def _build_compact_ui(self) -> None:
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        control_panel = QWidget(self)
        control_panel.setObjectName("SpatialResolutionControlPanel")
        control_panel.setAttribute(Qt.WA_StyledBackground, True)
        control_panel.setMinimumWidth(230)
        control_panel.setMaximumWidth(270)
        control_panel.setStyleSheet(
            "QWidget#SpatialResolutionControlPanel {"
            "background:#f8fafc; border:1px solid #dbe5ef; border-radius:8px;"
            "}"
        )
        control_col = QVBoxLayout(control_panel)
        control_col.setContentsMargins(10, 10, 10, 10)
        control_col.setSpacing(8)

        self._plan_summary_label = QLabel("MissionPlan: -", control_panel)
        self._plan_summary_label.setWordWrap(True)
        self._plan_summary_label.setStyleSheet(
            "padding: 7px 9px; border: 1px solid #cbd5e1; border-radius: 8px;"
            " background: #ffffff; color: #0f172a; font-size: 11px;"
        )
        control_col.addWidget(self._plan_summary_label)

        param_title = QLabel("공간해상도 파라미터", control_panel)
        param_title.setStyleSheet("font-weight:700; color:#0f172a; font-size:12px;")
        control_col.addWidget(param_title)

        cfg = QWidget(control_panel)
        cfg.setObjectName("SpatialResolutionConfig")
        cfg.setAttribute(Qt.WA_StyledBackground, True)
        cfg.setStyleSheet(
            "QWidget#SpatialResolutionConfig {"
            "background:#ffffff; border:1px solid #e2e8f0; border-radius:8px;"
            "}"
        )
        cfg_layout = QGridLayout(cfg)
        cfg_layout.setContentsMargins(8, 8, 8, 8)
        cfg_layout.setHorizontalSpacing(6)
        cfg_layout.setVerticalSpacing(5)

        self._sr_img_w_spin = self._make_int_spin(_SR_DEFAULT_IMG_W, " px", 92)
        self._sr_img_h_spin = self._make_int_spin(_SR_DEFAULT_IMG_H, " px", 92)
        self._sr_obj_w_spin = self._make_float_spin(_SR_DEFAULT_OBJ_W_M, " m", 92)
        self._sr_obj_h_spin = self._make_float_spin(_SR_DEFAULT_OBJ_H_M, " m", 92)
        self._sr_obj_px_x_spin = self._make_int_spin(_SR_DEFAULT_OBJ_MIN_PX_X, " px", 86)
        self._sr_obj_px_y_spin = self._make_int_spin(_SR_DEFAULT_OBJ_MIN_PX_Y, " px", 86)

        rows = (
            ("이미지 W", self._sr_img_w_spin),
            ("이미지 H", self._sr_img_h_spin),
            ("객체 W", self._sr_obj_w_spin),
            ("객체 H", self._sr_obj_h_spin),
            ("최소 px X", self._sr_obj_px_x_spin),
            ("최소 px Y", self._sr_obj_px_y_spin),
        )
        for row_idx, (label_text, widget) in enumerate(rows):
            cfg_layout.addWidget(self._param_label(label_text), row_idx, 0)
            cfg_layout.addWidget(widget, row_idx, 1)
        cfg_layout.setColumnStretch(1, 1)
        control_col.addWidget(cfg)

        for spin in (
            self._sr_img_w_spin, self._sr_img_h_spin,
            self._sr_obj_w_spin, self._sr_obj_h_spin,
            self._sr_obj_px_x_spin, self._sr_obj_px_y_spin,
        ):
            spin.valueChanged.connect(self._on_sr_setting_changed)

        self._sr_req_hint_label = QLabel("", control_panel)
        self._sr_req_hint_label.setStyleSheet("color: #64748b; font-size: 11px;")
        self._sr_req_hint_label.setWordWrap(True)
        control_col.addWidget(self._sr_req_hint_label)
        control_col.addStretch(1)

        cards_col = QVBoxLayout()
        cards_col.setContentsMargins(0, 0, 0, 0)
        cards_col.setSpacing(7)
        for uav_id in _UAV_IDS:
            card = SpatialResolutionCard(
                uav_id,
                compact_horizontal=True,
            )
            self._apply_compact_card_style(card)
            self._sr_cards[int(uav_id)] = card
            cards_col.addWidget(card, 0)
        cards_col.addStretch(1)

        root.addWidget(control_panel, 0)
        root.addLayout(cards_col, 1)
        self._update_sr_req_hint()

    def _apply_compact_card_style(self, card: SpatialResolutionCard) -> None:
        card.setMinimumHeight(134)
        card.setMaximumHeight(146)
        card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        card.setStyleSheet(
            "QGroupBox { font-weight: 700; font-size: 10px; border: 1px solid #dde4ee;"
            " border-radius: 10px; margin-top: 0; background: #ffffff; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px;"
            " color: #0f172a; }"
        )
        try:
            layout = card.layout()
            if layout is not None:
                layout.setContentsMargins(9, 8, 9, 7)
                layout.setSpacing(4)
        except Exception:
            pass
        try:
            font = card.font()
            if font.pointSize() > 0:
                font.setPointSize(max(9, font.pointSize() - 1))
            card.setFont(font)
            for label in card.findChildren(QLabel):
                child_font = label.font()
                if child_font.pointSize() > 0:
                    child_font.setPointSize(max(9, child_font.pointSize() - 1))
                label.setFont(child_font)
        except Exception:
            pass
        chart = getattr(card, "_chart", None)
        if chart is not None:
            try:
                chart.setMinimumHeight(68)
                chart.setMaximumHeight(76)
                chart.set_ui_scale(0.72)
            except Exception:
                pass

    def _make_int_spin(self, value: int, suffix: str, width: int) -> QSpinBox:
        spin = QSpinBox(self)
        spin.setRange(1, 9999)
        spin.setValue(int(value))
        spin.setSuffix(str(suffix))
        spin.setFixedWidth(int(width))
        spin.setFixedHeight(28)
        spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        return spin

    def _make_float_spin(self, value: float, suffix: str, width: int) -> QDoubleSpinBox:
        spin = QDoubleSpinBox(self)
        spin.setRange(0.01, 999.0)
        spin.setValue(float(value))
        spin.setSuffix(str(suffix))
        spin.setDecimals(2)
        spin.setSingleStep(0.1)
        spin.setFixedWidth(int(width))
        spin.setFixedHeight(28)
        spin.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        return spin

    def _param_label(self, text: str) -> QLabel:
        label = QLabel(str(text), self)
        label.setStyleSheet("color: #374151; font-size: 11px; font-weight: 700;")
        return label

    def _current_spatial_resolution_settings(self) -> dict[str, Any]:
        return _normalize_sr_settings(
            {
                "spatial_resolution": {
                    "img_w_px": self._sr_img_w_spin.value() if self._sr_img_w_spin is not None else _SR_DEFAULT_IMG_W,
                    "img_h_px": self._sr_img_h_spin.value() if self._sr_img_h_spin is not None else _SR_DEFAULT_IMG_H,
                    "obj_w_m": self._sr_obj_w_spin.value() if self._sr_obj_w_spin is not None else _SR_DEFAULT_OBJ_W_M,
                    "obj_h_m": self._sr_obj_h_spin.value() if self._sr_obj_h_spin is not None else _SR_DEFAULT_OBJ_H_M,
                    "obj_min_px_x": (
                        self._sr_obj_px_x_spin.value()
                        if self._sr_obj_px_x_spin is not None
                        else _SR_DEFAULT_OBJ_MIN_PX_X
                    ),
                    "obj_min_px_y": (
                        self._sr_obj_px_y_spin.value()
                        if self._sr_obj_px_y_spin is not None
                        else _SR_DEFAULT_OBJ_MIN_PX_Y
                    ),
                },
            }
        )

    def _restore_spatial_resolution_settings(self) -> None:
        settings = _load_sr_settings()
        sr = settings.get("spatial_resolution") or {}
        self._sr_settings_sync_enabled = False
        try:
            if self._sr_img_w_spin is not None:
                self._sr_img_w_spin.setValue(int(sr.get("img_w_px", _SR_DEFAULT_IMG_W)))
            if self._sr_img_h_spin is not None:
                self._sr_img_h_spin.setValue(int(sr.get("img_h_px", _SR_DEFAULT_IMG_H)))
            if self._sr_obj_w_spin is not None:
                self._sr_obj_w_spin.setValue(float(sr.get("obj_w_m", _SR_DEFAULT_OBJ_W_M)))
            if self._sr_obj_h_spin is not None:
                self._sr_obj_h_spin.setValue(float(sr.get("obj_h_m", _SR_DEFAULT_OBJ_H_M)))
            if self._sr_obj_px_x_spin is not None:
                self._sr_obj_px_x_spin.setValue(int(sr.get("obj_min_px_x", _SR_DEFAULT_OBJ_MIN_PX_X)))
            if self._sr_obj_px_y_spin is not None:
                self._sr_obj_px_y_spin.setValue(int(sr.get("obj_min_px_y", _SR_DEFAULT_OBJ_MIN_PX_Y)))
        finally:
            self._sr_settings_sync_enabled = True
        self._update_sr_req_hint()
        if not _sr_settings_path().exists():
            try:
                _save_sr_settings(settings)
            except Exception:
                pass

    def _persist_spatial_resolution_settings(self) -> None:
        _save_sr_settings(self._current_spatial_resolution_settings())

    def _update_sr_req_hint(self) -> None:
        if self._sr_req_hint_label is None:
            return
        px_x = self._sr_obj_px_x_spin.value() if self._sr_obj_px_x_spin else _SR_DEFAULT_OBJ_MIN_PX_X
        px_y = self._sr_obj_px_y_spin.value() if self._sr_obj_px_y_spin else _SR_DEFAULT_OBJ_MIN_PX_Y
        w = self._sr_obj_w_spin.value() if self._sr_obj_w_spin else _SR_DEFAULT_OBJ_W_M
        h = self._sr_obj_h_spin.value() if self._sr_obj_h_spin else _SR_DEFAULT_OBJ_H_M
        img_w = self._sr_img_w_spin.value() if self._sr_img_w_spin else _SR_DEFAULT_IMG_W
        img_h = self._sr_img_h_spin.value() if self._sr_img_h_spin else _SR_DEFAULT_IMG_H
        req_eq_gsd = 0.0
        req_fp_area = 0.0
        if px_x > 0 and px_y > 0 and img_w > 0 and img_h > 0:
            gamma = (float(w) * float(h)) / float(px_x * px_y)
            req_eq_gsd = math.sqrt(gamma) if gamma > 0.0 else 0.0
            req_fp_area = gamma * float(img_w * img_h)
        self._sr_req_hint_label.setText(
            f"요구 등가 GSD <= {req_eq_gsd:.4f} m/px / 허용 풋프린트 <= {req_fp_area:.0f} m2"
        )

    def _on_sr_setting_changed(self, *_args: object) -> None:
        self._update_sr_req_hint()
        if not self._sr_settings_sync_enabled:
            return
        for card in self._sr_cards.values():
            card.reset_history()
        try:
            self._persist_spatial_resolution_settings()
        except Exception as exc:
            if self._log_callback is not None:
                try:
                    self._log_callback(f"[QUALITY] spatial-resolution settings persist failed: {exc}")
                except Exception:
                    pass
        self._refresh_cards(redraw=self._ui_updates_enabled)

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
                base_meta = {
                    "sep_m": float(sep_m),
                    "width_m": _coerce_float(mission.get("width_m")),
                    "individual_mission_id": _coerce_int(mission.get("individual_mission_id")),
                    "path_id": _coerce_int(mission.get("path_id")),
                }
                waypoint_meta_by_id: dict[int, dict[str, Any]] = {}
                for waypoint in mission.get("waypoints") or []:
                    if not isinstance(waypoint, dict):
                        continue
                    wid = _coerce_int(waypoint.get("waypoint_id") or waypoint.get("waypointID"))
                    if wid is None or wid <= 0:
                        continue
                    waypoint_meta = dict(base_meta)
                    waypoint_meta.update(
                        {
                            "operation_mode": _coerce_int(waypoint.get("operation_mode")),
                            "waypoint_pass_type": _coerce_int(waypoint.get("waypoint_pass_type")),
                            "has_filming_property": bool(waypoint.get("has_filming_property")),
                            "has_line_search": bool(waypoint.get("has_line_search")),
                            "line_search_point_count": _coerce_int(waypoint.get("line_search_point_count")) or 0,
                        }
                    )
                    waypoint_meta_by_id[int(wid)] = waypoint_meta
                for waypoint_id in mission.get("waypoint_ids") or []:
                    wid = _coerce_int(waypoint_id)
                    if wid is None or wid <= 0:
                        continue
                    waypoint_map[int(wid)] = dict(waypoint_meta_by_id.get(int(wid), base_meta))
        self._waypoint_thresholds = thresholds
        if self._ui_updates_enabled:
            self._refresh_cards(redraw=True)
        else:
            self._dirty = True

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

        for uav_id, card in self._sr_cards.items():
            state = self._latest_state_map.get(int(uav_id), {})
            footprint_corners = state.get("footprint_corners") or []
            current_waypoint_id = _coerce_int(state.get("current_waypoint_id"))
            waypoint_meta = (
                self._waypoint_thresholds.get(int(uav_id), {}).get(int(current_waypoint_id), {})
                if current_waypoint_id is not None
                else {}
            )
            measurement_enabled = _is_sweep_measurement_active(
                waypoint_meta,
                sensor_operation_mode=_coerce_int(state.get("sensor_operation_mode")),
            )
            card.update_snapshot(
                timestamp_ms=chart_timestamp_ms,
                measurement_enabled=measurement_enabled,
                footprint_corners=footprint_corners,
                img_w_px=self._sr_img_w_spin.value() if self._sr_img_w_spin else _SR_DEFAULT_IMG_W,
                img_h_px=self._sr_img_h_spin.value() if self._sr_img_h_spin else _SR_DEFAULT_IMG_H,
                obj_w_m=self._sr_obj_w_spin.value() if self._sr_obj_w_spin else _SR_DEFAULT_OBJ_W_M,
                obj_h_m=self._sr_obj_h_spin.value() if self._sr_obj_h_spin else _SR_DEFAULT_OBJ_H_M,
                obj_min_px_x=self._sr_obj_px_x_spin.value() if self._sr_obj_px_x_spin else _SR_DEFAULT_OBJ_MIN_PX_X,
                obj_min_px_y=self._sr_obj_px_y_spin.value() if self._sr_obj_px_y_spin else _SR_DEFAULT_OBJ_MIN_PX_Y,
                redraw=redraw,
            )

        self._dirty = not redraw


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
        self._last_chart_redraw_monotonic_ms: int = 0
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
        self._sr_cards: dict[int, SpatialResolutionCard] = {}
        # SR 설정값 (SpinBox 위젯으로 실시간 반영)
        self._sr_img_w_spin: QSpinBox | None = None
        self._sr_img_h_spin: QSpinBox | None = None
        self._sr_obj_w_spin: QDoubleSpinBox | None = None
        self._sr_obj_h_spin: QDoubleSpinBox | None = None
        self._sr_obj_px_x_spin: QSpinBox | None = None
        self._sr_obj_px_y_spin: QSpinBox | None = None
        self._sr_req_hint_label: QLabel | None = None
        self._sr_settings_sync_enabled: bool = False
        self._build_ui()
        self._restore_spatial_resolution_settings()

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
        chart_ts = _coerce_int(timestamp_ms)
        if chart_ts is None or chart_ts <= 0:
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
        if not self._ui_updates_enabled:
            self._dirty = True
            return
        redraw = bool(self._ui_updates_enabled)
        if redraw:
            now_ms = int(time.monotonic() * 1000)
            if (
                self._last_chart_redraw_monotonic_ms > 0
                and (now_ms - self._last_chart_redraw_monotonic_ms) < _QUALITY_CHART_REDRAW_INTERVAL_MS
            ):
                redraw = False
            else:
                self._last_chart_redraw_monotonic_ms = now_ms
        self._refresh_cards(redraw=redraw)

    def _build_ui(self) -> None:
        frame = QVBoxLayout(self)
        frame.setContentsMargins(0, 0, 0, 0)
        frame.setSpacing(0)

        content = QWidget()
        root = QVBoxLayout(content)
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

        # ── 공간해상도 (GSD) 섹션 ──────────────────────────────────────
        sr_group = QGroupBox("공간해상도 (GSD) 모니터")
        sr_group.setStyleSheet(
            "QGroupBox { font-weight: 700; border: 1px solid #cbd5e1; border-radius: 12px; margin-top: 12px; background: #f0fdf4; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 6px; color: #15803d; }"
        )
        sr_outer = QVBoxLayout(sr_group)
        sr_outer.setContentsMargins(10, 18, 10, 10)
        sr_outer.setSpacing(8)

        # 설정 행
        cfg_row = QHBoxLayout()
        cfg_row.setSpacing(12)

        def _make_spin_pair(label_text: str, widget: QWidget) -> QHBoxLayout:
            row = QHBoxLayout()
            row.setSpacing(4)
            lbl = QLabel(label_text)
            lbl.setStyleSheet("color: #374151; font-size: 11px;")
            row.addWidget(lbl)
            row.addWidget(widget)
            return row

        self._sr_img_w_spin = QSpinBox()
        self._sr_img_w_spin.setRange(1, 9999)
        self._sr_img_w_spin.setValue(_SR_DEFAULT_IMG_W)
        self._sr_img_w_spin.setSuffix(" px")
        self._sr_img_w_spin.setFixedWidth(90)

        self._sr_img_h_spin = QSpinBox()
        self._sr_img_h_spin.setRange(1, 9999)
        self._sr_img_h_spin.setValue(_SR_DEFAULT_IMG_H)
        self._sr_img_h_spin.setSuffix(" px")
        self._sr_img_h_spin.setFixedWidth(90)

        self._sr_obj_w_spin = QDoubleSpinBox()
        self._sr_obj_w_spin.setRange(0.01, 999.0)
        self._sr_obj_w_spin.setValue(_SR_DEFAULT_OBJ_W_M)
        self._sr_obj_w_spin.setSuffix(" m")
        self._sr_obj_w_spin.setDecimals(2)
        self._sr_obj_w_spin.setSingleStep(0.1)
        self._sr_obj_w_spin.setFixedWidth(90)

        self._sr_obj_h_spin = QDoubleSpinBox()
        self._sr_obj_h_spin.setRange(0.01, 999.0)
        self._sr_obj_h_spin.setValue(_SR_DEFAULT_OBJ_H_M)
        self._sr_obj_h_spin.setSuffix(" m")
        self._sr_obj_h_spin.setDecimals(2)
        self._sr_obj_h_spin.setSingleStep(0.1)
        self._sr_obj_h_spin.setFixedWidth(90)

        self._sr_obj_px_x_spin = QSpinBox()
        self._sr_obj_px_x_spin.setRange(1, 9999)
        self._sr_obj_px_x_spin.setValue(_SR_DEFAULT_OBJ_MIN_PX_X)
        self._sr_obj_px_x_spin.setSuffix(" px")
        self._sr_obj_px_x_spin.setFixedWidth(80)

        self._sr_obj_px_y_spin = QSpinBox()
        self._sr_obj_px_y_spin.setRange(1, 9999)
        self._sr_obj_px_y_spin.setValue(_SR_DEFAULT_OBJ_MIN_PX_Y)
        self._sr_obj_px_y_spin.setSuffix(" px")
        self._sr_obj_px_y_spin.setFixedWidth(80)

        cfg_row.addLayout(_make_spin_pair("이미지 가로:", self._sr_img_w_spin))
        cfg_row.addLayout(_make_spin_pair("이미지 세로:", self._sr_img_h_spin))
        cfg_row.addLayout(_make_spin_pair("객체 가로:", self._sr_obj_w_spin))
        cfg_row.addLayout(_make_spin_pair("객체 세로:", self._sr_obj_h_spin))
        cfg_row.addLayout(_make_spin_pair("최소 가로 px:", self._sr_obj_px_x_spin))
        cfg_row.addLayout(_make_spin_pair("최소 세로 px:", self._sr_obj_px_y_spin))
        cfg_row.addStretch(1)

        self._sr_req_hint_label = QLabel("")
        self._sr_req_hint_label.setStyleSheet("color: #64748b; font-size: 11px;")
        self._sr_req_hint_label.setWordWrap(True)

        for spin in (
            self._sr_img_w_spin, self._sr_img_h_spin,
            self._sr_obj_w_spin, self._sr_obj_h_spin,
            self._sr_obj_px_x_spin, self._sr_obj_px_y_spin,
        ):
            spin.valueChanged.connect(self._on_sr_setting_changed)

        sr_outer.addLayout(cfg_row)
        sr_outer.addWidget(self._sr_req_hint_label)

        # UAV별 카드
        sr_cards_row = QHBoxLayout()
        sr_cards_row.setSpacing(10)
        for uav_id in _UAV_IDS:
            card = SpatialResolutionCard(uav_id)
            self._sr_cards[int(uav_id)] = card
            sr_cards_row.addWidget(card, 1)
        sr_outer.addLayout(sr_cards_row)
        root.addWidget(sr_group, 1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setWidget(content)
        frame.addWidget(scroll)

        self._update_sr_req_hint()
        self._apply_monitor_visual_state()

    def _current_spatial_resolution_settings(self) -> dict[str, Any]:
        return _normalize_sr_settings(
            {
                "spatial_resolution": {
                    "img_w_px": self._sr_img_w_spin.value() if self._sr_img_w_spin is not None else _SR_DEFAULT_IMG_W,
                    "img_h_px": self._sr_img_h_spin.value() if self._sr_img_h_spin is not None else _SR_DEFAULT_IMG_H,
                    "obj_w_m": self._sr_obj_w_spin.value() if self._sr_obj_w_spin is not None else _SR_DEFAULT_OBJ_W_M,
                    "obj_h_m": self._sr_obj_h_spin.value() if self._sr_obj_h_spin is not None else _SR_DEFAULT_OBJ_H_M,
                    "obj_min_px_x": (
                        self._sr_obj_px_x_spin.value()
                        if self._sr_obj_px_x_spin is not None
                        else _SR_DEFAULT_OBJ_MIN_PX_X
                    ),
                    "obj_min_px_y": (
                        self._sr_obj_px_y_spin.value()
                        if self._sr_obj_px_y_spin is not None
                        else _SR_DEFAULT_OBJ_MIN_PX_Y
                    ),
                },
            }
        )

    def _restore_spatial_resolution_settings(self) -> None:
        settings = _load_sr_settings()
        sr = settings.get("spatial_resolution") or {}
        self._sr_settings_sync_enabled = False
        try:
            if self._sr_img_w_spin is not None:
                self._sr_img_w_spin.setValue(int(sr.get("img_w_px", _SR_DEFAULT_IMG_W)))
            if self._sr_img_h_spin is not None:
                self._sr_img_h_spin.setValue(int(sr.get("img_h_px", _SR_DEFAULT_IMG_H)))
            if self._sr_obj_w_spin is not None:
                self._sr_obj_w_spin.setValue(float(sr.get("obj_w_m", _SR_DEFAULT_OBJ_W_M)))
            if self._sr_obj_h_spin is not None:
                self._sr_obj_h_spin.setValue(float(sr.get("obj_h_m", _SR_DEFAULT_OBJ_H_M)))
            if self._sr_obj_px_x_spin is not None:
                self._sr_obj_px_x_spin.setValue(int(sr.get("obj_min_px_x", _SR_DEFAULT_OBJ_MIN_PX_X)))
            if self._sr_obj_px_y_spin is not None:
                self._sr_obj_px_y_spin.setValue(int(sr.get("obj_min_px_y", _SR_DEFAULT_OBJ_MIN_PX_Y)))
        finally:
            self._sr_settings_sync_enabled = True
        self._update_sr_req_hint()
        if not _sr_settings_path().exists():
            try:
                _save_sr_settings(settings)
            except Exception:
                pass

    def _persist_spatial_resolution_settings(self) -> None:
        _save_sr_settings(self._current_spatial_resolution_settings())

    def _update_sr_req_hint(self) -> None:
        if self._sr_req_hint_label is None:
            return
        px_x = self._sr_obj_px_x_spin.value() if self._sr_obj_px_x_spin else _SR_DEFAULT_OBJ_MIN_PX_X
        px_y = self._sr_obj_px_y_spin.value() if self._sr_obj_px_y_spin else _SR_DEFAULT_OBJ_MIN_PX_Y
        w = self._sr_obj_w_spin.value() if self._sr_obj_w_spin else _SR_DEFAULT_OBJ_W_M
        h = self._sr_obj_h_spin.value() if self._sr_obj_h_spin else _SR_DEFAULT_OBJ_H_M
        img_w = self._sr_img_w_spin.value() if self._sr_img_w_spin else _SR_DEFAULT_IMG_W
        img_h = self._sr_img_h_spin.value() if self._sr_img_h_spin else _SR_DEFAULT_IMG_H
        req_eq_gsd = 0.0
        req_fp_area = 0.0
        if px_x > 0 and px_y > 0 and img_w > 0 and img_h > 0:
            gamma = (float(w) * float(h)) / float(px_x * px_y)
            req_eq_gsd = math.sqrt(gamma) if gamma > 0.0 else 0.0
            req_fp_area = gamma * float(img_w * img_h)
        self._sr_req_hint_label.setText(
            f"요구 등가 GSD ≤ {req_eq_gsd:.4f} m/px  / 허용 풋프린트 ≤ {req_fp_area:.0f} m²"
            f"  (객체 {w}m×{h}m 이상 {px_x}px×{px_y}px)"
        )

    def _on_sr_setting_changed(self, *_args: object) -> None:
        self._update_sr_req_hint()
        if not self._sr_settings_sync_enabled:
            return
        for card in self._sr_cards.values():
            card.reset_history()
        try:
            self._persist_spatial_resolution_settings()
        except Exception as exc:
            if self._log_callback is not None:
                try:
                    self._log_callback(f"[QUALITY] spatial-resolution settings persist failed: {exc}")
                except Exception:
                    pass
        self._refresh_cards(redraw=self._ui_updates_enabled)

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
                f"padding: 1px 8px; border-radius: 8px; color: white; background: {color_text}; font-weight: 700;"
            )
        if self._monitor_toggle_button is not None:
            self._monitor_toggle_button.setText(button_text)
            self._monitor_toggle_button.setStyleSheet(
                f"QPushButton {{ background: {button_bg}; color: #0f172a; padding: 6px 14px; border-radius: 8px; }}"
            )
        if self._replan_state_label is not None:
            self._replan_state_label.setText(replan_text)
            self._replan_state_label.setStyleSheet(
                f"padding: 1px 8px; border-radius: 8px; color: white; background: {replan_color}; font-weight: 700;"
            )
        if self._replan_toggle_button is not None:
            self._replan_toggle_button.setText(replan_button_text)
            self._replan_toggle_button.setStyleSheet(
                f"QPushButton {{ background: {replan_button_bg}; color: #0f172a; padding: 6px 14px; border-radius: 8px; }}"
            )
        for card in (
            list(self._sep_cards.values())
            + list(self._link_cards.values())
            + list(self._sr_cards.values())
        ):
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
                base_meta = {
                    "sep_m": float(sep_m),
                    "width_m": _coerce_float(mission.get("width_m")),
                    "individual_mission_id": _coerce_int(mission.get("individual_mission_id")),
                    "path_id": _coerce_int(mission.get("path_id")),
                }
                waypoint_meta_by_id: dict[int, dict[str, Any]] = {}
                for waypoint in mission.get("waypoints") or []:
                    if not isinstance(waypoint, dict):
                        continue
                    wid = _coerce_int(waypoint.get("waypoint_id") or waypoint.get("waypointID"))
                    if wid is None or wid <= 0:
                        continue
                    waypoint_meta = dict(base_meta)
                    waypoint_meta.update(
                        {
                            "operation_mode": _coerce_int(waypoint.get("operation_mode")),
                            "waypoint_pass_type": _coerce_int(waypoint.get("waypoint_pass_type")),
                            "has_filming_property": bool(waypoint.get("has_filming_property")),
                            "has_line_search": bool(waypoint.get("has_line_search")),
                            "line_search_point_count": _coerce_int(waypoint.get("line_search_point_count")) or 0,
                        }
                    )
                    waypoint_meta_by_id[int(wid)] = waypoint_meta
                for waypoint_id in mission.get("waypoint_ids") or []:
                    wid = _coerce_int(waypoint_id)
                    if wid is None or wid <= 0:
                        continue
                    waypoint_map[int(wid)] = dict(waypoint_meta_by_id.get(int(wid), base_meta))
        self._waypoint_thresholds = thresholds
        if self._ui_updates_enabled:
            self._refresh_cards(redraw=True)
        else:
            self._dirty = True

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

        for uav_id, card in self._sr_cards.items():
            state = self._latest_state_map.get(int(uav_id), {})
            footprint_corners = state.get("footprint_corners") or []
            current_waypoint_id = _coerce_int(state.get("current_waypoint_id"))
            waypoint_meta = (
                self._waypoint_thresholds.get(int(uav_id), {}).get(int(current_waypoint_id), {})
                if current_waypoint_id is not None
                else {}
            )
            measurement_enabled = _is_sweep_measurement_active(
                waypoint_meta,
                sensor_operation_mode=_coerce_int(state.get("sensor_operation_mode")),
            )
            card.update_snapshot(
                timestamp_ms=chart_timestamp_ms,
                measurement_enabled=measurement_enabled,
                footprint_corners=footprint_corners,
                img_w_px=self._sr_img_w_spin.value() if self._sr_img_w_spin else _SR_DEFAULT_IMG_W,
                img_h_px=self._sr_img_h_spin.value() if self._sr_img_h_spin else _SR_DEFAULT_IMG_H,
                obj_w_m=self._sr_obj_w_spin.value() if self._sr_obj_w_spin else _SR_DEFAULT_OBJ_W_M,
                obj_h_m=self._sr_obj_h_spin.value() if self._sr_obj_h_spin else _SR_DEFAULT_OBJ_H_M,
                obj_min_px_x=self._sr_obj_px_x_spin.value() if self._sr_obj_px_x_spin else _SR_DEFAULT_OBJ_MIN_PX_X,
                obj_min_px_y=self._sr_obj_px_y_spin.value() if self._sr_obj_px_y_spin else _SR_DEFAULT_OBJ_MIN_PX_Y,
                redraw=redraw,
            )

        self._dirty = not redraw
