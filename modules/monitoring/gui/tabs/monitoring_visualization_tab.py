# -*- coding: utf-8 -*-
from __future__ import annotations

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt5.QtGui import QColor, QBrush, QPainter, QPen, QPolygonF
import json
import time
from collections import deque
from PyQt5.QtWidgets import (
    QWidget,
    QLayout,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QFormLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QProgressBar,
    QScrollArea,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QAbstractItemView,
)

from modules.monitoring.logic.mission_update import (
    build_uav_mission_view,
    format_timestamp_ms,
    mark_individual_mission_done,
    mark_individual_mission_undone,
    mark_input_mission_done,
    mark_input_mission_undone,
    mark_waypoints_done,
    mission_plan_json_path,
)
from modules.monitoring.logic.mission_progress import MissionProgressTracker
from modules.common import db_paths
from typing import Callable


_EPOCH2000_MS = 946684800000
_SIGNAL_OK_GRACE_MS = 20000
_OPERATION_OVERRIDE_SECONDS = 5.0


class DLRiskTrendWidget(QWidget):
    """2x3 streaming risk trend panels for 6 aircraft."""

    _SERIES_COLORS = (
        "#2563eb",
        "#0ea5e9",
        "#14b8a6",
        "#22c55e",
        "#f59e0b",
        "#ef4444",
    )

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._window_ms: int = 60_000
        self._history: dict[int, deque[tuple[int, float]]] = {aid: deque() for aid in range(1, 7)}
        self._order: list[int] = [1, 2, 3, 4, 5, 6]
        self._enabled: bool = False
        self._replan_enabled: bool = False
        self._trigger_threshold: float = 0.8
        self._latest_ts_ms: int = 0
        self._ui_scale: float = 1.0
        self.setMinimumHeight(320)
        self.setProperty("_base_min_height", 320)

    def set_ui_scale(self, scale: float) -> None:
        try:
            self._ui_scale = max(0.72, min(1.0, float(scale)))
        except Exception:
            self._ui_scale = 1.0
        self.update()

    def set_panel_state(
        self,
        *,
        enabled: bool | None = None,
        replan_enabled: bool | None = None,
        trigger_threshold: float | None = None,
    ) -> None:
        if enabled is not None:
            self._enabled = bool(enabled)
        if replan_enabled is not None:
            self._replan_enabled = bool(replan_enabled)
        if trigger_threshold is not None:
            try:
                self._trigger_threshold = max(0.0, min(1.0, float(trigger_threshold)))
            except Exception:
                self._trigger_threshold = 0.8
        self.update()

    def set_display_order(self, aircraft_ids: list[int] | None) -> None:
        if not aircraft_ids:
            self._order = [1, 2, 3, 4, 5, 6]
            self.update()
            return
        normalized: list[int] = []
        seen: set[int] = set()
        for raw_id in aircraft_ids:
            try:
                aid = int(raw_id)
            except Exception:
                continue
            if aid in seen:
                continue
            seen.add(aid)
            normalized.append(aid)
            if len(normalized) >= 6:
                break
        while len(normalized) < 6:
            candidate = len(normalized) + 1
            if candidate not in seen:
                normalized.append(candidate)
                seen.add(candidate)
            else:
                break
        if normalized:
            self._order = normalized[:6]
        self.update()

    def append_sample(
        self,
        timestamp_ms: int | None,
        mean_values: list[float] | None,
        aircraft_ids: list[int] | None = None,
    ) -> None:
        if not mean_values:
            return
        ts_ms = None
        if timestamp_ms is not None:
            try:
                ts_ms = int(timestamp_ms)
            except Exception:
                ts_ms = None
        if ts_ms is None:
            if self._latest_ts_ms > 0:
                ts_ms = self._latest_ts_ms + 200
            else:
                ts_ms = int(time.time() * 1000)
        if ts_ms > self._latest_ts_ms:
            self._latest_ts_ms = ts_ms

        order = self._order
        if aircraft_ids:
            try:
                order = [int(aid) for aid in aircraft_ids[:6]]
            except Exception:
                order = self._order
        for idx, raw_val in enumerate(mean_values[:6]):
            aid = order[idx] if idx < len(order) else (idx + 1)
            try:
                aid_int = int(aid)
                risk = float(raw_val)
            except Exception:
                continue
            if risk < 0.0:
                risk = 0.0
            elif risk > 1.0:
                risk = 1.0
            history = self._history.setdefault(aid_int, deque())
            history.append((ts_ms, risk))
        self._prune_old_samples()
        self.update()

    def _prune_old_samples(self) -> None:
        if self._latest_ts_ms <= 0:
            return
        cutoff = self._latest_ts_ms - self._window_ms
        for history in self._history.values():
            while history and history[0][0] < cutoff:
                history.popleft()

    @staticmethod
    def _level(risk: float | None, threshold: float) -> str:
        if risk is None:
            return "N/A"
        if risk >= threshold:
            return "HIGH"
        if risk >= 0.5:
            return "MID"
        return "LOW"

    def _cell_rect(self, index: int) -> QRectF:
        cols = 2
        rows = 3
        outer_margin = max(4.0, 8.0 * self._ui_scale)
        gap = max(4.0, 8.0 * self._ui_scale)
        total_w = max(1.0, float(self.width()))
        total_h = max(1.0, float(self.height()))
        cell_w = max(40.0, (total_w - (outer_margin * 2.0) - gap) / cols)
        cell_h = max(36.0, (total_h - (outer_margin * 2.0) - (gap * (rows - 1))) / rows)
        row = index // cols
        col = index % cols
        x = outer_margin + col * (cell_w + gap)
        y = outer_margin + row * (cell_h + gap)
        return QRectF(x, y, cell_w, cell_h)

    def _build_polyline(self, plot_rect: QRectF, history: deque[tuple[int, float]], now_ms: int) -> QPolygonF:
        points = QPolygonF()
        if not history:
            return points
        left = plot_rect.left()
        right = plot_rect.right()
        bottom = plot_rect.bottom()
        h = max(1.0, plot_rect.height())
        w = max(1.0, plot_rect.width())
        for ts_ms, risk in history:
            age_ms = max(0, now_ms - int(ts_ms))
            if age_ms > self._window_ms:
                continue
            x = right - (float(age_ms) / float(self._window_ms)) * w
            y = bottom - float(risk) * h
            if x < left:
                continue
            points.append(QPointF(x, y))
        return points

    def paintEvent(self, event) -> None:  # type: ignore[override]
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#f8fafc"))

        now_ms = self._latest_ts_ms if self._latest_ts_ms > 0 else int(time.time() * 1000)
        title_font_px = max(8, int(round(10 * self._ui_scale)))
        meta_font_px = max(7, int(round(9 * self._ui_scale)))
        radius = max(4.0, 6.0 * self._ui_scale)

        for idx in range(6):
            cell = self._cell_rect(idx)
            aid = self._order[idx] if idx < len(self._order) else (idx + 1)
            history = self._history.get(int(aid), deque())
            latest_risk = history[-1][1] if history else None
            series_color = QColor(self._SERIES_COLORS[idx % len(self._SERIES_COLORS)])

            painter.setPen(QPen(QColor("#cbd5e1"), 1))
            painter.setBrush(QColor("#ffffff"))
            painter.drawRoundedRect(cell, radius, radius)

            head_h = max(14.0, 18.0 * self._ui_scale)
            title_rect = QRectF(cell.left() + 6, cell.top() + 3, cell.width() - 12, head_h)
            painter.setPen(QColor("#0f172a"))
            font = painter.font()
            font.setBold(True)
            font.setPixelSize(title_font_px)
            painter.setFont(font)
            painter.drawText(title_rect, Qt.AlignLeft | Qt.AlignVCenter, f"AC{int(aid)}")

            level_text = self._level(latest_risk, self._trigger_threshold)
            risk_text = "--" if latest_risk is None else f"{latest_risk * 100.0:05.1f}%"
            painter.drawText(
                title_rect,
                Qt.AlignRight | Qt.AlignVCenter,
                f"{risk_text} / {level_text}",
            )

            plot_rect = QRectF(
                cell.left() + 6,
                cell.top() + head_h + 4,
                max(10.0, cell.width() - 12),
                max(10.0, cell.height() - head_h - 14),
            )

            grid_pen = QPen(QColor("#e2e8f0"), 1)
            painter.setPen(grid_pen)
            for frac in (0.0, 0.5, 1.0):
                y = plot_rect.bottom() - frac * plot_rect.height()
                painter.drawLine(QPointF(plot_rect.left(), y), QPointF(plot_rect.right(), y))

            if self._enabled and self._replan_enabled:
                trg_y = plot_rect.bottom() - self._trigger_threshold * plot_rect.height()
                trg_pen = QPen(QColor("#ef4444"), 1)
                trg_pen.setStyle(Qt.DashLine)
                painter.setPen(trg_pen)
                painter.drawLine(QPointF(plot_rect.left(), trg_y), QPointF(plot_rect.right(), trg_y))
                font = painter.font()
                font.setBold(True)
                font.setPixelSize(meta_font_px)
                painter.setFont(font)
                painter.drawText(
                    QRectF(plot_rect.left() + 2, trg_y - 12, plot_rect.width() - 4, 12),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    "TRG",
                )

            line = self._build_polyline(plot_rect, history, now_ms)
            if len(line) >= 2:
                line_pen = QPen(series_color, max(1.2, 1.8 * self._ui_scale))
                painter.setPen(line_pen)
                painter.setBrush(Qt.NoBrush)
                painter.drawPolyline(line)
                last_point = line[-1]
                painter.setBrush(series_color)
                painter.setPen(QPen(QColor("#ffffff"), 1))
                point_r = max(1.8, 2.6 * self._ui_scale)
                painter.drawEllipse(last_point, point_r, point_r)
            elif len(line) == 1:
                painter.setBrush(series_color)
                painter.setPen(Qt.NoPen)
                point_r = max(1.8, 2.6 * self._ui_scale)
                painter.drawEllipse(line[0], point_r, point_r)

            font = painter.font()
            font.setBold(False)
            font.setPixelSize(meta_font_px)
            painter.setFont(font)
            painter.setPen(QColor("#64748b"))
            painter.drawText(
                QRectF(plot_rect.left(), plot_rect.bottom() + 1, plot_rect.width(), 12),
                Qt.AlignLeft | Qt.AlignVCenter,
                "60s",
            )
            painter.drawText(
                QRectF(plot_rect.left(), plot_rect.bottom() + 1, plot_rect.width(), 12),
                Qt.AlignRight | Qt.AlignVCenter,
                "Now",
            )


class RealtimeRiskPredictionTab(QWidget):
    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._ui_updates_enabled = True
        self._dl_panel_status: str = "DISABLED"
        self._dl_panel_enabled: bool = False
        self._dl_panel_replan_enabled: bool = False
        self._dl_panel_data_age_sec: float | None = None
        self._dl_panel_infer_age_sec: float | None = None
        self._dl_panel_last_update_ms: int | None = None
        self._dl_panel_buffer_len: int = 0
        self._dl_panel_buffer_min: int = 0
        self._dl_panel_base_ready: bool = False
        self._dl_panel_mean: list[float] = []
        self._dl_panel_std: list[float] = []
        self._dl_panel_risky_indices: set[int] = set()
        self._dl_panel_aircraft_ids: list[int] = [1, 2, 3, 4, 5, 6]
        self._dl_status_badge: QLabel | None = None
        self._dl_summary_value: QLabel | None = None
        self._dl_last_update_value: QLabel | None = None
        self._dl_buffer_value: QLabel | None = None
        self._dl_replan_value: QLabel | None = None
        self._dl_window_value: QLabel | None = None
        self._dl_risk_trend: DLRiskTrendWidget | None = None
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(10)

        panel = QGroupBox("실시간 위험도 예측")
        panel.setObjectName("DlRiskPanel")
        panel.setStyleSheet(
            """
            QGroupBox#DlRiskPanel {
                font-weight: 700;
                border: 1px solid #d6e2ef;
                border-radius: 12px;
                margin-top: 12px;
                background: #ffffff;
            }
            QGroupBox#DlRiskPanel::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 4px;
            }
            QLabel#DlRiskBadge {
                color: #ffffff;
                font-weight: 700;
                border-radius: 8px;
                padding: 4px 12px;
            }
            QLabel#DlRiskPill {
                border-radius: 10px;
                padding: 8px 10px;
                background: #f8fafc;
                border: 1px solid #d9e3ef;
            }
            """
        )

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 14, 12, 12)
        layout.setSpacing(10)

        header_row = QHBoxLayout()
        header_row.setSpacing(8)
        summary_title = QLabel("DL 추론 상태와 최근 60초 위험도 추세를 기체별로 확인합니다.")
        summary_title.setWordWrap(True)
        summary_title.setStyleSheet("color: #475569;")
        self._dl_status_badge = QLabel("DISABLED")
        self._dl_status_badge.setObjectName("DlRiskBadge")
        self._dl_status_badge.setAlignment(Qt.AlignCenter)
        header_row.addWidget(summary_title, 1)
        header_row.addWidget(self._dl_status_badge, 0)

        info_grid = QGridLayout()
        info_grid.setHorizontalSpacing(10)
        info_grid.setVerticalSpacing(8)
        self._dl_summary_value = QLabel("DL 추론 비활성")
        self._dl_last_update_value = QLabel("최근 추론: -")
        self._dl_buffer_value = QLabel("버퍼: -")
        self._dl_replan_value = QLabel("재계획 트리거: OFF")
        self._dl_window_value = QLabel("추세 창: 최근 60초 (좌측=과거, 우측=현재)")
        self._dl_window_value.setWordWrap(True)
        for label in (
            self._dl_summary_value,
            self._dl_last_update_value,
            self._dl_buffer_value,
            self._dl_replan_value,
            self._dl_window_value,
        ):
            label.setObjectName("DlRiskPill")
        info_grid.addWidget(self._dl_summary_value, 0, 0, 1, 2)
        info_grid.addWidget(self._dl_last_update_value, 1, 0)
        info_grid.addWidget(self._dl_buffer_value, 1, 1)
        info_grid.addWidget(self._dl_replan_value, 2, 0)
        info_grid.addWidget(self._dl_window_value, 2, 1)

        trend = DLRiskTrendWidget()
        trend.setMinimumHeight(380)
        trend.setProperty("_base_min_height", 380)
        self._dl_risk_trend = trend

        layout.addLayout(header_row)
        layout.addLayout(info_grid)
        layout.addWidget(trend, 1)
        root.addWidget(panel, 1)
        self._refresh_dl_panel_view()

    @staticmethod
    def _dl_status_style(status: str) -> str:
        status_text = str(status or "UNKNOWN").strip().upper()
        color_map = {
            "RUNNING": "#16a34a",
            "WARMUP": "#d97706",
            "NO_DATA": "#dc2626",
            "STALE": "#475569",
            "DISABLED": "#6b7280",
            "UNAVAILABLE": "#7c2d12",
            "ERROR": "#991b1b",
            "INIT": "#0f766e",
        }
        color = color_map.get(status_text, "#6b7280")
        return (
            "QLabel {"
            f" background: {color};"
            " color: #ffffff;"
            " font-weight: 700;"
            " border-radius: 8px;"
            " padding: 4px 12px;"
            " }"
        )

    @staticmethod
    def _dl_replan_style(enabled: bool) -> str:
        border = "#fca5a5" if enabled else "#d9e3ef"
        bg = "#fef2f2" if enabled else "#f8fafc"
        color = "#b91c1c" if enabled else "#334155"
        return (
            "QLabel {"
            f" background: {bg}; color: {color}; border: 1px solid {border};"
            " border-radius: 10px; padding: 8px 10px; font-weight: 700;"
            " }"
        )

    def _refresh_dl_panel_view(self) -> None:
        if self._dl_status_badge is not None:
            status_text = str(self._dl_panel_status or "UNKNOWN").upper()
            self._dl_status_badge.setText(status_text)
            self._dl_status_badge.setStyleSheet(self._dl_status_style(status_text))

        summary = "DL 추론 비활성"
        if self._dl_panel_enabled:
            peak = 0.0
            if self._dl_panel_mean:
                try:
                    peak = max(float(v) for v in self._dl_panel_mean)
                except Exception:
                    peak = 0.0
            risky_count = len(self._dl_panel_risky_indices)
            status_text = str(self._dl_panel_status or "").upper()
            if self._dl_panel_mean:
                summary = f"최대 위험도 {peak * 100.0:.1f}% | 고위험 {risky_count}대"
            elif status_text == "WARMUP":
                summary = "DL 워밍업 중 | 버퍼 데이터 수집 중"
            elif status_text == "NO_DATA":
                summary = "0401 데이터 대기 중"
            elif status_text == "STALE":
                summary = "최근 추론 정지 | 마지막 값을 유지 중"
            elif status_text == "UNAVAILABLE":
                summary = "DL 로더 사용 불가"
            elif status_text == "ERROR":
                summary = "DL 초기화 오류"
            else:
                summary = "DL 추론 준비 중"
        if self._dl_summary_value is not None:
            self._dl_summary_value.setText(summary)

        if self._dl_last_update_value is not None:
            last_update_text = "최근 추론: -"
            if self._dl_panel_last_update_ms is not None:
                last_update_text = f"최근 추론: {format_timestamp_ms(self._dl_panel_last_update_ms)}"
                if self._dl_panel_infer_age_sec is not None:
                    last_update_text += f" ({self._dl_panel_infer_age_sec:.1f}s 전)"
            elif self._dl_panel_infer_age_sec is not None:
                last_update_text = f"최근 추론: {self._dl_panel_infer_age_sec:.1f}s 전"
            self._dl_last_update_value.setText(last_update_text)

        if self._dl_buffer_value is not None:
            data_age_text = "-"
            if self._dl_panel_data_age_sec is not None:
                data_age_text = f"{self._dl_panel_data_age_sec:.1f}s"
            base_ready = "완료" if self._dl_panel_base_ready else "대기"
            self._dl_buffer_value.setText(
                f"버퍼: {int(self._dl_panel_buffer_len)}/{int(self._dl_panel_buffer_min)} | "
                f"기준좌표: {base_ready} | 마지막 데이터: {data_age_text}"
            )

        if self._dl_replan_value is not None:
            self._dl_replan_value.setText(
                "재계획 트리거: ON" if self._dl_panel_replan_enabled else "재계획 트리거: OFF"
            )
            self._dl_replan_value.setStyleSheet(
                self._dl_replan_style(self._dl_panel_replan_enabled)
            )

        if self._dl_window_value is not None:
            threshold_text = "재계획 기준선 80%"
            if not self._dl_panel_enabled:
                threshold_text = "DL 비활성"
            self._dl_window_value.setText(f"추세 창: 최근 60초 (좌측=과거, 우측=현재) | {threshold_text}")

        self._refresh_dl_risk_table()

    def _refresh_dl_risk_table(self) -> None:
        trend = self._dl_risk_trend
        if trend is None:
            return
        trend.set_display_order(self._dl_panel_aircraft_ids)
        trend.set_panel_state(
            enabled=self._dl_panel_enabled,
            replan_enabled=self._dl_panel_replan_enabled,
            trigger_threshold=0.8,
        )

    def update_dl_panel(
        self,
        *,
        status: str | None = None,
        enabled: bool | None = None,
        replan_enabled: bool | None = None,
        mean: list[float] | None = None,
        std: list[float] | None = None,
        risky_indices: list[int] | None = None,
        aircraft_ids: list[int] | None = None,
        timestamp_ms: int | None = None,
        data_age_sec: float | None = None,
        infer_age_sec: float | None = None,
        buffer_len: int | None = None,
        min_buffer: int | None = None,
        base_ready: bool | None = None,
    ) -> None:
        appended = False
        if status is not None:
            self._dl_panel_status = str(status)
        if enabled is not None:
            self._dl_panel_enabled = bool(enabled)
        if replan_enabled is not None:
            self._dl_panel_replan_enabled = bool(replan_enabled)
        if mean is not None:
            self._dl_panel_mean = [float(v) for v in mean[:6]]
            appended = True
        if std is not None:
            self._dl_panel_std = [float(v) for v in std[:6]]
        if risky_indices is not None:
            normalized: set[int] = set()
            for raw_idx in risky_indices:
                try:
                    idx = int(raw_idx)
                except Exception:
                    continue
                if 0 <= idx < 6:
                    normalized.add(idx)
            self._dl_panel_risky_indices = normalized
        if aircraft_ids is not None:
            normalized_ids: list[int] = []
            for raw_id in aircraft_ids[:6]:
                try:
                    normalized_ids.append(int(raw_id))
                except Exception:
                    continue
            if normalized_ids:
                self._dl_panel_aircraft_ids = normalized_ids
        if timestamp_ms is not None:
            try:
                self._dl_panel_last_update_ms = int(timestamp_ms)
            except Exception:
                pass
        if data_age_sec is not None:
            self._dl_panel_data_age_sec = float(data_age_sec)
        if infer_age_sec is not None:
            self._dl_panel_infer_age_sec = float(infer_age_sec)
        if buffer_len is not None:
            self._dl_panel_buffer_len = int(buffer_len)
        if min_buffer is not None:
            self._dl_panel_buffer_min = int(min_buffer)
        if base_ready is not None:
            self._dl_panel_base_ready = bool(base_ready)
        if appended and self._dl_risk_trend is not None:
            self._dl_risk_trend.append_sample(
                self._dl_panel_last_update_ms,
                self._dl_panel_mean,
                self._dl_panel_aircraft_ids,
            )
        if self._ui_updates_enabled:
            self._refresh_dl_panel_view()

    def set_ui_updates_enabled(self, enabled: bool) -> None:
        self._ui_updates_enabled = bool(enabled)
        if self._ui_updates_enabled:
            self._refresh_dl_panel_view()


class MonitoringVisualizationTab(QWidget):
    """Monitoring visualization UI."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._ui_scale: float = 1.0
        self._last_applied_scale: float = -1.0
        self._layout_base_metrics: dict[int, tuple[tuple[int, int, int, int], int]] = {}
        self._root_layout: QVBoxLayout | None = None
        self._id_fields: list[QLineEdit] = []
        self._summary_progress_bars: list[QProgressBar] = []
        self._lowlevel_scrolls: list[QScrollArea] = []
        base_font_size = self.font().pointSizeF()
        if base_font_size <= 0:
            base_font_size = 9.0
        self._base_font_size = float(base_font_size)
        self._individual_package_fields: list[QLineEdit] = []
        self._individual_package_bars: list[QProgressBar] = []
        self._individual_low_layouts: list[QHBoxLayout] = []
        self._individual_wp_layouts: list[QHBoxLayout] = []
        self._mission_tables: list[QTableWidget] = []
        self._aircraft_labels: dict[int, QLabel] = {}
        self._signal_labels: dict[int, QLabel] = {}
        self._equipment_labels: dict[int, QLabel] = {}
        self._operation_labels: dict[int, QLabel] = {}
        self._mission_header_labels: dict[int, QLabel] = {}
        self._available_ids: set[int] = set()
        self._fuel_state_by_aircraft: dict[int, str] = {}
        self._availability_stage: str | None = None
        self._latest_state_map: dict[int, dict[str, object]] = {}
        self._last_signal_received_ms_by_aircraft: dict[int, int] = {}
        self._forced_wait_aircraft: set[int] = set()
        self._operation_override_by_aircraft: dict[int, dict[str, object]] = {}
        self._input_mission_low_layout: QHBoxLayout | None = None
        self._mission_plan_bar: QProgressBar | None = None
        self._mission_view: dict | None = None
        self._progress_tracker = MissionProgressTracker()
        self._last_forced_input_id: int | None = None
        self._last_forced_mission_ids: list[int] = []
        self._last_active_input_id: int | None = None
        self._last_progress_input_id: int | None = None
        self._last_progress_snapshot: dict | None = None
        self._last_status_timestamp_ms: int | None = None
        self._sent_0503_inputs: set[int] = set()
        self._forced_completion_inputs: set[int] = set()
        self._pending_completion_inputs: list[int] = []
        self._pending_execute_inputs: list[int] = []
        self._sent_0503_pending_inputs: set[int] = set()
        self._sent_final_completion: bool = False
        self._recommend_callback = None
        self._notice_callback = None
        self._reexecute_callback = None
        self._log_callback: Callable[[str], None] | None = None
        self._sweep_log_state: dict[tuple[int, int], dict[str, int]] = {}
        self._sweep_log_buffer_sec: int = 0
        self._sweep_progress_cache: dict[tuple[int, int], dict[str, int | float | str | None]] = {}
        self._ui_updates_enabled = True
        self._dl_panel_status: str = "DISABLED"
        self._dl_panel_enabled: bool = False
        self._dl_panel_replan_enabled: bool = False
        self._dl_panel_data_age_sec: float | None = None
        self._dl_panel_infer_age_sec: float | None = None
        self._dl_panel_last_update_ms: int | None = None
        self._dl_panel_buffer_len: int = 0
        self._dl_panel_buffer_min: int = 0
        self._dl_panel_base_ready: bool = False
        self._dl_panel_mean: list[float] = []
        self._dl_panel_std: list[float] = []
        self._dl_panel_risky_indices: set[int] = set()
        self._dl_panel_aircraft_ids: list[int] = [1, 2, 3, 4, 5, 6]
        self._dl_status_badge: QLabel | None = None
        self._dl_summary_value: QLabel | None = None
        self._dl_last_update_value: QLabel | None = None
        self._dl_buffer_value: QLabel | None = None
        self._dl_replan_value: QLabel | None = None
        self._dl_window_value: QLabel | None = None
        self._dl_risk_trend: DLRiskTrendWidget | None = None
        self._build_ui()
        self._status_refresh_timer = QTimer(self)
        self._status_refresh_timer.setInterval(1000)
        self._status_refresh_timer.timeout.connect(self._refresh_live_status_panels)
        self._status_refresh_timer.start()
        QTimer.singleShot(0, self._apply_responsive_scale)

    def _build_ui(self) -> None:
        frame = QVBoxLayout(self)
        frame.setContentsMargins(12, 10, 12, 10)
        frame.setSpacing(0)

        content = QWidget()
        root = QVBoxLayout(content)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)
        self._root_layout = root

        root.addWidget(self._build_update_group())
        status_grid = QGridLayout()
        status_grid.setHorizontalSpacing(10)
        status_grid.setVerticalSpacing(10)
        status_grid.addWidget(self._build_availability_group(), 0, 0)
        status_grid.addWidget(self._build_signal_status_group(), 0, 1)
        status_grid.addWidget(self._build_equipment_status_group(), 1, 0)
        status_grid.addWidget(self._build_operation_status_group(), 1, 1)
        root.addLayout(status_grid)
        root.addWidget(self._build_mission_plan_group())
        root.addWidget(self._build_individual_plan_group())

        tables_row = QHBoxLayout()
        tables_row.setSpacing(10)
        for uav_id in (4, 5, 6):
            tables_row.addWidget(self._build_mission_table_group(uav_id), 1)
        root.addLayout(tables_row, 1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setWidget(content)
        frame.addWidget(scroll, 1)

    def _build_right_panel(self) -> QWidget:
        panel = QGroupBox("DL Risk Monitor")
        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        header_row = QHBoxLayout()
        header_row.setContentsMargins(0, 0, 0, 0)
        header_row.setSpacing(8)
        title = QLabel("실시간 위험도 예측")
        title.setStyleSheet("font-weight: 700;")
        self._dl_status_badge = QLabel("DISABLED")
        self._dl_status_badge.setAlignment(Qt.AlignCenter)
        header_row.addWidget(title, 1)
        header_row.addWidget(self._dl_status_badge, 0)

        self._dl_summary_value = QLabel("DL 추론 비활성")
        self._dl_summary_value.setWordWrap(True)
        self._dl_last_update_value = QLabel("최근 추론: -")
        self._dl_buffer_value = QLabel("버퍼: -")
        self._dl_replan_value = QLabel("재계획 트리거: OFF")
        self._dl_replan_value.setAlignment(Qt.AlignCenter)
        self._dl_window_value = QLabel("추세 창: 최근 60초 (좌측=과거, 우측=현재)")
        self._dl_window_value.setWordWrap(True)

        info_form = QFormLayout()
        info_form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        info_form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        info_form.setHorizontalSpacing(8)
        info_form.setVerticalSpacing(6)
        info_form.addRow("요약", self._dl_summary_value)
        info_form.addRow("최근 추론", self._dl_last_update_value)
        info_form.addRow("워밍업", self._dl_buffer_value)
        info_form.addRow("시간축", self._dl_window_value)
        info_form.addRow("", self._dl_replan_value)

        trend = DLRiskTrendWidget()
        trend.setMinimumHeight(260)
        trend.setProperty("_base_min_height", 260)
        self._dl_risk_trend = trend

        layout.addLayout(header_row)
        layout.addLayout(info_form)
        layout.addWidget(trend, 1)
        panel.setLayout(layout)
        self._refresh_dl_panel_view()
        return panel

    @staticmethod
    def _dl_status_style(status: str) -> str:
        status_text = str(status or "UNKNOWN").strip().upper()
        color_map = {
            "RUNNING": "#16a34a",
            "WARMUP": "#d97706",
            "NO_DATA": "#dc2626",
            "STALE": "#475569",
            "DISABLED": "#6b7280",
            "UNAVAILABLE": "#7c2d12",
            "ERROR": "#991b1b",
            "INIT": "#0f766e",
        }
        color = color_map.get(status_text, "#6b7280")
        return (
            "QLabel {"
            f" background: {color}; color: #ffffff;"
            " font-weight: 700;"
            " border-radius: 6px;"
            " padding: 4px 10px;"
            " }"
        )

    def _dl_replan_style(self, enabled: bool) -> str:
        color = "#ef4444" if enabled else "#475569"
        radius = self._scaled(5, min_value=3)
        pad_v = self._scaled(3, min_value=2)
        pad_h = self._scaled(8, min_value=6)
        font_px = self._scaled(11, min_value=8)
        return (
            "QLabel {"
            f" background: {color}; color: #ffffff;"
            f" border-radius: {radius}px;"
            f" padding: {pad_v}px {pad_h}px;"
            f" font-weight: 700; font-size: {font_px}px;"
            " }"
        )

    @staticmethod
    def _dl_level_from_mean(mean_value: float | None, risky: bool) -> tuple[str, str]:
        if mean_value is None:
            return "N/A", "#9ca3af"
        if risky or mean_value >= 0.8:
            return "HIGH", "#ef4444"
        if mean_value >= 0.5:
            return "MID", "#f59e0b"
        return "LOW", "#22c55e"

    def _refresh_dl_panel_view(self) -> None:
        if self._dl_status_badge is not None:
            status_text = str(self._dl_panel_status or "UNKNOWN").upper()
            self._dl_status_badge.setText(status_text)
            self._dl_status_badge.setStyleSheet(self._dl_status_style(status_text))

        if self._dl_replan_value is not None:
            self._dl_replan_value.setText(
                "재계획 트리거: ON" if self._dl_panel_replan_enabled else "재계획 트리거: OFF"
            )
            self._dl_replan_value.setStyleSheet(
                self._dl_replan_style(self._dl_panel_replan_enabled)
            )

        summary = "DL 추론 비활성"
        if self._dl_panel_enabled:
            peak = 0.0
            if self._dl_panel_mean:
                try:
                    peak = max(float(v) for v in self._dl_panel_mean)
                except Exception:
                    peak = 0.0
            risky_count = len(self._dl_panel_risky_indices)
            status_text = str(self._dl_panel_status or "").upper()
            if self._dl_panel_mean:
                summary = f"최대 위험도 {peak * 100.0:.1f}% | 고위험 {risky_count}대"
            elif status_text == "WARMUP":
                summary = "DL 워밍업 중 | 버퍼 데이터 수집 중"
            elif status_text == "NO_DATA":
                summary = "0401 데이터 대기 중"
            elif status_text == "STALE":
                summary = "최근 추론 정지 | 마지막 값을 유지 중"
            elif status_text == "UNAVAILABLE":
                summary = "DL 로더 사용 불가"
            elif status_text == "ERROR":
                summary = "DL 초기화 오류"
            else:
                summary = "DL 추론 준비 중"
        if self._dl_summary_value is not None:
            self._dl_summary_value.setText(summary)

        last_update_text = "최근 추론: -"
        if self._dl_panel_last_update_ms is not None:
            last_update_text = f"최근 추론: {format_timestamp_ms(self._dl_panel_last_update_ms)}"
            if self._dl_panel_infer_age_sec is not None:
                last_update_text += f" ({self._dl_panel_infer_age_sec:.1f}s 전)"
        elif self._dl_panel_infer_age_sec is not None:
            last_update_text = f"최근 추론: {self._dl_panel_infer_age_sec:.1f}s 전"
        if self._dl_last_update_value is not None:
            self._dl_last_update_value.setText(last_update_text)

        data_age_text = "-"
        if self._dl_panel_data_age_sec is not None:
            data_age_text = f"{self._dl_panel_data_age_sec:.1f}s"
        base_ready = "완료" if self._dl_panel_base_ready else "대기"
        if self._dl_buffer_value is not None:
            self._dl_buffer_value.setText(
                f"버퍼: {int(self._dl_panel_buffer_len)}/{int(self._dl_panel_buffer_min)} | "
                f"기준좌표: {base_ready} | 마지막 데이터: {data_age_text}"
            )

        self._refresh_dl_risk_table()

    def _refresh_dl_risk_table(self) -> None:
        trend = self._dl_risk_trend
        if trend is None:
            return
        trend.set_display_order(self._dl_panel_aircraft_ids)
        trend.set_panel_state(
            enabled=self._dl_panel_enabled,
            replan_enabled=self._dl_panel_replan_enabled,
            trigger_threshold=0.8,
        )

    def update_dl_panel(
        self,
        *,
        status: str | None = None,
        enabled: bool | None = None,
        replan_enabled: bool | None = None,
        mean: list[float] | None = None,
        std: list[float] | None = None,
        risky_indices: list[int] | None = None,
        aircraft_ids: list[int] | None = None,
        timestamp_ms: int | None = None,
        data_age_sec: float | None = None,
        infer_age_sec: float | None = None,
        buffer_len: int | None = None,
        min_buffer: int | None = None,
        base_ready: bool | None = None,
    ) -> None:
        appended = False
        if status is not None:
            self._dl_panel_status = str(status)
        if enabled is not None:
            self._dl_panel_enabled = bool(enabled)
        if replan_enabled is not None:
            self._dl_panel_replan_enabled = bool(replan_enabled)
        if mean is not None:
            self._dl_panel_mean = [float(v) for v in mean[:6]]
            appended = True
        if std is not None:
            self._dl_panel_std = [float(v) for v in std[:6]]
        if risky_indices is not None:
            normalized: set[int] = set()
            for raw_idx in risky_indices:
                try:
                    idx = int(raw_idx)
                except Exception:
                    continue
                if 0 <= idx < 6:
                    normalized.add(idx)
            self._dl_panel_risky_indices = normalized
        if aircraft_ids is not None:
            normalized_ids: list[int] = []
            for raw_id in aircraft_ids[:6]:
                try:
                    normalized_ids.append(int(raw_id))
                except Exception:
                    continue
            if normalized_ids:
                self._dl_panel_aircraft_ids = normalized_ids
        if timestamp_ms is not None:
            try:
                self._dl_panel_last_update_ms = int(timestamp_ms)
            except Exception:
                pass
        if data_age_sec is not None:
            self._dl_panel_data_age_sec = float(data_age_sec)
        if infer_age_sec is not None:
            self._dl_panel_infer_age_sec = float(infer_age_sec)
        if buffer_len is not None:
            self._dl_panel_buffer_len = int(buffer_len)
        if min_buffer is not None:
            self._dl_panel_buffer_min = int(min_buffer)
        if base_ready is not None:
            self._dl_panel_base_ready = bool(base_ready)
        if appended and self._dl_risk_trend is not None:
            self._dl_risk_trend.append_sample(
                self._dl_panel_last_update_ms,
                self._dl_panel_mean,
                self._dl_panel_aircraft_ids,
            )
        self._refresh_dl_panel_view()

    def _build_update_group(self) -> QGroupBox:
        group = QGroupBox("임무 갱신 요청 상태")
        layout = QHBoxLayout()
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(12)

        self._update_status_value = QLabel("요청 없음")
        self._update_detail_value = QLabel("최근 0903 요청이 없습니다.")
        self._update_detail_value.setWordWrap(True)
        self._decision_status_value = QLabel("결정 없음")
        self._decision_detail_value = QLabel("최근 0702 결정이 없습니다.")
        self._decision_detail_value.setWordWrap(True)

        left_box = QGroupBox("0903")
        left_form = QFormLayout()
        left_form.setLabelAlignment(Qt.AlignLeft | Qt.AlignTop)
        left_form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        left_form.setHorizontalSpacing(12)
        left_form.setVerticalSpacing(6)
        left_form.addRow("상태", self._update_status_value)
        left_form.addRow("상세", self._update_detail_value)
        left_box.setLayout(left_form)

        right_box = QGroupBox("0702")
        right_form = QFormLayout()
        right_form.setLabelAlignment(Qt.AlignLeft | Qt.AlignTop)
        right_form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        right_form.setHorizontalSpacing(12)
        right_form.setVerticalSpacing(6)
        right_form.addRow("상태", self._decision_status_value)
        right_form.addRow("상세", self._decision_detail_value)
        right_box.setLayout(right_form)

        layout.addWidget(left_box, 1)
        layout.addWidget(right_box, 1)
        group.setLayout(layout)
        return group

    def _build_mission_plan_group(self) -> QGroupBox:
        group = QGroupBox("Mission Plan")
        layout = QVBoxLayout()
        layout.setSpacing(6)

        top_row = QWidget()
        top_layout = QHBoxLayout(top_row)
        top_layout.setContentsMargins(0, 0, 0, 0)
        top_layout.setSpacing(10)
        self._mission_plan_id_field = self._make_id_field("-")
        bar = self._make_progress_bar()
        self._summary_progress_bars.append(bar)
        self._mission_plan_bar = bar
        top_layout.addWidget(self._mission_plan_id_field)
        top_layout.addWidget(bar, 1)
        layout.addWidget(top_row)

        low_scroll, low_layout = self._build_lowlevel_scroll(
            base_height=40,
            min_height=24,
        )
        self._input_mission_low_layout = low_layout
        layout.addWidget(low_scroll)
        group.setLayout(layout)
        return group

    def _build_individual_plan_group(self) -> QGroupBox:
        group = QGroupBox("Individual Mission Plan")
        layout = QVBoxLayout()
        layout.setSpacing(6)
        for _ in range(3):
            row = QWidget()
            row_layout = QVBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(6)

            top_row = QWidget()
            top_layout = QHBoxLayout(top_row)
            top_layout.setContentsMargins(0, 0, 0, 0)
            top_layout.setSpacing(10)
            package_id_field = self._make_id_field("-")
            bar = self._make_progress_bar()
            self._summary_progress_bars.append(bar)
            top_layout.addWidget(package_id_field)
            top_layout.addWidget(bar, 1)

            low_scroll, low_layout = self._build_lowlevel_scroll(
                base_height=40,
                min_height=24,
            )
            wp_scroll, wp_layout = self._build_lowlevel_scroll(
                base_height=20,
                min_height=16,
            )

            row_layout.addWidget(top_row)
            row_layout.addWidget(low_scroll)
            row_layout.addWidget(wp_scroll)
            layout.addWidget(row)
            self._individual_package_fields.append(package_id_field)
            self._individual_package_bars.append(bar)
            self._individual_low_layouts.append(low_layout)
            self._individual_wp_layouts.append(wp_layout)
        group.setLayout(layout)
        return group

    def _build_mission_table_group(self, uav_id: int) -> QGroupBox:
        group = QGroupBox()
        group.setTitle("")
        table = QTableWidget(0, 5)
        table.setHorizontalHeaderLabels(
            ["개별 임무 ID", "Input ID", "Path ID", "Waypoint IDs", "촬영률"]
        )
        table.verticalHeader().setVisible(False)
        header = table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        header.setStretchLastSection(False)
        header.setMinimumHeight(22)
        table.setAlternatingRowColors(True)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setSelectionMode(QAbstractItemView.SingleSelection)
        table.setMinimumHeight(140)
        table.setWordWrap(False)
        table.setTextElideMode(Qt.ElideNone)
        table.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        table.setProperty("_base_min_height", 140)
        table.setProperty("_base_header_min_height", 22)
        table.verticalHeader().setDefaultSectionSize(22)

        header_row = QWidget()
        header_layout = QHBoxLayout(header_row)
        header_layout.setContentsMargins(0, 0, 0, 0)
        header_layout.setSpacing(6)
        header_label = QLabel()
        header_label.setTextFormat(Qt.RichText)
        header_label.setStyleSheet("font-weight: 600;")
        header_layout.addWidget(header_label)
        header_layout.addStretch(1)

        layout = QVBoxLayout()
        layout.setSpacing(6)
        layout.addWidget(header_row)
        layout.addWidget(table)
        group.setLayout(layout)
        self._mission_header_labels[int(uav_id)] = header_label
        self._update_mission_header(int(uav_id))
        self._mission_tables.append(table)
        return group

    def _build_availability_group(self) -> QGroupBox:
        group = QGroupBox("기체 상태")
        layout = QFormLayout()
        layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)

        manned = self._make_pill_row_with_ids(
            [(1, "유인 1 (미확인)"), (2, "유인 2 (미확인)"), (3, "유인 3 (미확인)")],
            registry=self._aircraft_labels,
        )
        uav = self._make_pill_row_with_ids(
            [(4, "무인 4 (미확인)"), (5, "무인 5 (미확인)"), (6, "무인 6 (미확인)")],
            registry=self._aircraft_labels,
        )
        layout.addRow("유인기", manned)
        layout.addRow("무인기", uav)
        group.setLayout(layout)
        return group

    def _build_signal_status_group(self) -> QGroupBox:
        group = QGroupBox("0401 신호 상태")
        layout = QFormLayout()
        layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)

        manned = self._make_pill_row_with_ids(
            [(1, "유인 1 (신호 미확인)"), (2, "유인 2 (신호 미확인)"), (3, "유인 3 (신호 미확인)")],
            registry=self._signal_labels,
        )
        uav = self._make_pill_row_with_ids(
            [(4, "무인 4 (신호 미확인)"), (5, "무인 5 (신호 미확인)"), (6, "무인 6 (신호 미확인)")],
            registry=self._signal_labels,
        )
        layout.addRow("유인기", manned)
        layout.addRow("무인기", uav)
        group.setLayout(layout)
        return group

    def _build_equipment_status_group(self) -> QGroupBox:
        group = QGroupBox("임무장비 상태")
        layout = QFormLayout()
        layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)

        manned = self._make_pill_row_with_ids(
            [(1, "유인 1 (-)"), (2, "유인 2 (-)"), (3, "유인 3 (-)")],
            registry=self._equipment_labels,
        )
        uav = self._make_pill_row_with_ids(
            [(4, "무인 4 (미확인)"), (5, "무인 5 (미확인)"), (6, "무인 6 (미확인)")],
            registry=self._equipment_labels,
        )
        layout.addRow("유인기", manned)
        layout.addRow("무인기", uav)
        group.setLayout(layout)
        return group

    def _build_operation_status_group(self) -> QGroupBox:
        group = QGroupBox("운용 상태")
        layout = QFormLayout()
        layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        layout.setHorizontalSpacing(10)
        layout.setVerticalSpacing(6)

        manned = self._make_pill_row_with_ids(
            [(1, "유인 1 (미확인)"), (2, "유인 2 (미확인)"), (3, "유인 3 (미확인)")],
            registry=self._operation_labels,
        )
        uav = self._make_pill_row_with_ids(
            [(4, "무인 4 (미확인)"), (5, "무인 5 (미확인)"), (6, "무인 6 (미확인)")],
            registry=self._operation_labels,
        )
        layout.addRow("유인기", manned)
        layout.addRow("무인기", uav)
        group.setLayout(layout)
        return group

    def update_availability(self, available_ids: list[int], *, stage: str = "0201") -> None:
        if self._availability_stage_priority(stage) < self._availability_stage_priority(self._availability_stage):
            return
        self._availability_stage = stage
        available = {int(aid) for aid in available_ids or []}
        self._available_ids = set(available)
        for aid, label in self._aircraft_labels.items():
            is_available = aid in available
            role = "유인" if aid <= 3 else "무인"
            status = "가용" if is_available else "불가용"
            label.setText(f"{role} {aid} ({status})")
            pill_status = "good" if is_available else "bad"
            label.setProperty("_pill_status", pill_status)
            label.setStyleSheet(self._pill_style(pill_status))
        for aid in list(self._mission_header_labels.keys()):
            self._update_mission_header(aid)

    def update_agent_status(
        self,
        *,
        timestamp_ms: int | None,
        agent_states: list[dict],
        fuel_state_map: dict[int, str] | None = None,
    ) -> None:
        if timestamp_ms is not None:
            try:
                self._last_status_timestamp_ms = int(timestamp_ms)
            except Exception:
                pass
        if fuel_state_map is not None:
            normalized_fuel: dict[int, str] = {}
            for key, value in dict(fuel_state_map).items():
                try:
                    aid = int(key)
                except Exception:
                    continue
                normalized_fuel[aid] = str(value)
            self._fuel_state_by_aircraft = normalized_fuel
        state_map = {
            int(state.get("aircraft_id")): state
            for state in agent_states
            if state.get("aircraft_id") is not None
        }
        self._latest_state_map = {int(aid): dict(data) for aid, data in state_map.items()}
        received_ms = self._current_ms_since_2000()
        for aid in state_map.keys():
            self._last_signal_received_ms_by_aircraft[int(aid)] = int(received_ms)
        self._refresh_status_panels(reference_timestamp_ms=timestamp_ms)

        update_ui = bool(self._ui_updates_enabled)
        self._update_mission_progress(timestamp_ms, agent_states, update_ui=update_ui)
        if update_ui:
            for aid in list(self._mission_header_labels.keys()):
                self._update_mission_header(aid)

    def _refresh_live_status_panels(self) -> None:
        if not self._latest_state_map and self._last_status_timestamp_ms is None:
            return
        self._refresh_status_panels(reference_timestamp_ms=self._current_ms_since_2000())

    def _refresh_status_panels(self, *, reference_timestamp_ms: int | None) -> None:
        state_map = self._latest_state_map if isinstance(self._latest_state_map, dict) else {}
        mission_flags = self._mission_runtime_flags_by_aircraft()

        for aid, label in self._signal_labels.items():
            role = "유인" if aid <= 3 else "무인"
            last_received_ms = self._last_signal_received_ms_by_aircraft.get(int(aid))
            ok = self._signal_ok(self._current_ms_since_2000(), last_received_ms)
            status = "신호 정상" if ok else "신호 이상"
            style = "good" if ok else "bad"
            self._update_status_label(label, f"{role} {aid} ({status})", style)

        for aid, label in self._equipment_labels.items():
            role = "유인" if aid <= 3 else "무인"
            if aid <= 3:
                self._update_status_label(label, f"{role} {aid} (-)", "unknown")
                continue
            payload_health = state_map.get(aid, {}).get("payload_health")
            if payload_health == 1:
                status = "정상"
                style = "good"
            elif payload_health == 0:
                status = "미정"
                style = "warn"
            elif payload_health is None:
                status = "미확인"
                style = "bad"
            else:
                status = "비정상"
                style = "bad"
            self._update_status_label(label, f"{role} {aid} ({status})", style)

        for aid, label in self._operation_labels.items():
            role = "유인" if aid <= 3 else "무인"
            status, style = self._operation_status_for_aircraft(
                int(aid),
                state_map.get(aid) or {},
                mission_flags.get(int(aid)) or {},
            )
            self._update_status_label(label, f"{role} {aid} ({status})", style)

    def _mission_runtime_flags_by_aircraft(self) -> dict[int, dict[str, bool]]:
        result: dict[int, dict[str, bool]] = {}
        view = self._mission_view if isinstance(self._mission_view, dict) else {}
        snapshot = self._last_progress_snapshot if isinstance(self._last_progress_snapshot, dict) else {}
        mission_progress = snapshot.get("mission_progress") if isinstance(snapshot, dict) else {}
        if not isinstance(mission_progress, dict):
            mission_progress = {}
        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = entry.get("aircraft_id")
            try:
                aid = int(aircraft_id)
            except Exception:
                continue
            missions = [mission for mission in entry.get("missions") or [] if isinstance(mission, dict)]
            current_mission_id = self._resolve_current_mission_id(
                missions,
                aid,
                snapshot,
            )
            progress_entry = mission_progress.get(current_mission_id) if current_mission_id is not None else None
            result[int(aid)] = {
                "awaiting_execute": bool((progress_entry or {}).get("awaiting_execute")),
            }
        return result

    def _operation_status_for_aircraft(
        self,
        aircraft_id: int,
        state: dict[str, object],
        mission_flags: dict[str, bool],
    ) -> tuple[str, str]:
        override = self._pop_active_operation_override(int(aircraft_id))
        if override is not None:
            return str(override.get("text") or "임무복귀"), str(override.get("style") or "good")

        if aircraft_id in self._forced_wait_aircraft:
            return "강제대기", "warn"

        flight_mode = state.get("flight_mode")
        on_mission = state.get("on_mission")
        try:
            flight_mode_int = int(flight_mode) if flight_mode is not None else None
        except Exception:
            flight_mode_int = None
        try:
            on_mission_int = int(on_mission) if on_mission is not None else None
        except Exception:
            on_mission_int = None

        if aircraft_id <= 3:
            if flight_mode_int == 5:
                return "RTB", "bad"
            if state:
                return "감시중", "good"
            return "미확인", "bad"

        if flight_mode_int == 5:
            return "RTB", "bad"
        if bool(mission_flags.get("awaiting_execute")):
            return "실행대기", "warn"
        if flight_mode_int == 9:
            return "표적추적", "good"
        if flight_mode_int == 8:
            return "Loiter", "warn"
        if on_mission_int == 2:
            return "대기", "warn"
        if on_mission_int == 1:
            return "임무수행", "good"
        if flight_mode_int == 7:
            return "비행", "good"
        if state:
            return "상태확인", "warn"
        return "미확인", "bad"

    def _pop_active_operation_override(self, aircraft_id: int) -> dict[str, object] | None:
        override = self._operation_override_by_aircraft.get(int(aircraft_id))
        if not isinstance(override, dict):
            return None
        expires_monotonic = override.get("expires_monotonic")
        try:
            expires = float(expires_monotonic) if expires_monotonic is not None else None
        except Exception:
            expires = None
        if expires is not None and time.monotonic() > expires:
            self._operation_override_by_aircraft.pop(int(aircraft_id), None)
            return None
        return override

    def _update_status_label(self, label: QLabel | None, text: str, style: str) -> None:
        if label is None:
            return
        label.setText(str(text))
        label.setProperty("_pill_status", str(style))
        label.setStyleSheet(self._pill_style(str(style)))

    @staticmethod
    def _current_ms_since_2000() -> int:
        return int(time.time() * 1000) - _EPOCH2000_MS

    def set_log_callback(self, callback: Callable[[str], None] | None) -> None:
        self._log_callback = callback

    def set_system_mode(self, mode_code: int | None) -> None:
        try:
            self._progress_tracker.set_system_mode(mode_code)
        except Exception:
            pass

    def _emit_log(self, text: str) -> None:
        cb = self._log_callback
        if cb is None:
            return
        try:
            cb(text)
        except Exception:
            pass

    def set_forced_wait(
        self,
        *,
        aircraft_id: int | None,
        paused: bool,
        timestamp_ms: int | None,
    ) -> None:
        if aircraft_id is None:
            return
        aid = None
        try:
            aid = int(aircraft_id)
            if paused:
                self._progress_tracker.pause_aircraft(int(aircraft_id), timestamp_ms)
            else:
                self._progress_tracker.resume_aircraft(int(aircraft_id), timestamp_ms)
        except Exception:
            return
        if aid is not None:
            if paused:
                self._forced_wait_aircraft.add(int(aid))
                self._operation_override_by_aircraft.pop(int(aid), None)
            else:
                self._forced_wait_aircraft.discard(int(aid))
                self._operation_override_by_aircraft[int(aid)] = {
                    "text": "임무복귀",
                    "style": "good",
                    "expires_monotonic": float(time.monotonic()) + float(_OPERATION_OVERRIDE_SECONDS),
                }
        snapshot = self._progress_tracker.update(timestamp_ms, [])
        self._apply_progress_snapshot(snapshot)
        self._refresh_status_panels(reference_timestamp_ms=timestamp_ms)
        for aid in list(self._mission_header_labels.keys()):
            self._update_mission_header(aid)

    def update_0903(
        self,
        *,
        timestamp_ms: int | None,
        mission_plan_id: int | None,
        source: str | None = None,
    ) -> None:
        if mission_plan_id is None:
            if hasattr(self, "_update_status_value"):
                self._update_status_value.setText("요청 없음")
            if hasattr(self, "_update_detail_value"):
                self._update_detail_value.setText("최근 0903 요청이 없습니다.")
            if hasattr(self, "_mission_plan_id_field"):
                self._mission_plan_id_field.setText("-")
            self._mission_plan_path = None
            self._apply_mission_plan_view(None)
            return

        ts_text = format_timestamp_ms(timestamp_ms)
        detail_lines = [
            f"시간: {ts_text}",
            f"missionPlanID: {mission_plan_id}",
        ]
        if source:
            detail_lines.insert(1, f"source: {source}")
        detail_lines.append(f"DB: MissionPlan\\{mission_plan_id}.json")

        if hasattr(self, "_update_status_value"):
            self._update_status_value.setText("요청 수신")
        if hasattr(self, "_update_detail_value"):
            self._update_detail_value.setText("\n".join(detail_lines))
        if hasattr(self, "_mission_plan_id_field"):
            self._mission_plan_id_field.setText(str(mission_plan_id))
        self._mission_plan_path = mission_plan_json_path(mission_plan_id)
        self._apply_mission_plan_view(mission_plan_id)

    def apply_mission_plan_decision(
        self,
        *,
        mission_plan_id: int | None,
    ) -> None:
        """Apply a decided mission plan (e.g., 0702 ignore=2) without touching 0903 UI."""
        if mission_plan_id is None:
            return
        if hasattr(self, "_mission_plan_id_field"):
            self._mission_plan_id_field.setText(str(mission_plan_id))
        self._mission_plan_path = mission_plan_json_path(mission_plan_id)
        self._apply_mission_plan_view(mission_plan_id)

    def update_0702_status(
        self,
        *,
        status: str,
        detail: str | None = None,
    ) -> None:
        if hasattr(self, "_decision_status_value"):
            self._decision_status_value.setText(status)
        if detail is not None and hasattr(self, "_decision_detail_value"):
            self._decision_detail_value.setText(detail)

    def handle_execute_command(self, *, execute: int | None) -> None:
        if execute == 1:
            self._handle_execute_next()
            return
        if execute == 2:
            self._handle_execute_repeat()
            if callable(self._reexecute_callback):
                try:
                    self._reexecute_callback(execute)
                except Exception:
                    pass
            return
        return

    def _handle_execute_next(self) -> None:
        view = self._mission_view
        if not view:
            return
        input_missions = view.get("input_missions") or []
        current_input_id = self._pick_current_input_id(input_missions)
        if current_input_id is not None and not self._is_input_done(input_missions, int(current_input_id)):
            self._force_complete_input_mission(view, int(current_input_id))
        target_input_id = self._pick_next_input_id(input_missions, current_input_id)
        if target_input_id is None:
            if callable(self._notice_callback):
                try:
                    self._notice_callback("모니터링 모듈: 다음 협업기저임무가 없습니다.")
                except Exception:
                    pass
            snapshot = self._progress_tracker.update(None, None)
            self._apply_progress_snapshot(snapshot)
            return

        if (
            target_input_id != current_input_id
            and self._is_input_done(input_missions, target_input_id)
        ):
            self._repeat_input_mission(view, int(target_input_id))
        self._progress_tracker.activate_input(int(target_input_id))
        self._last_active_input_id = int(target_input_id)
        snapshot = self._progress_tracker.update(None, None)
        self._apply_progress_snapshot(snapshot)

    def _handle_execute_repeat(self) -> None:
        view = self._mission_view
        if not view:
            return

        input_missions = view.get("input_missions") or []
        target_input_id = self._progress_tracker.get_active_input_id()
        if target_input_id is None:
            target_input_id = self._last_progress_input_id
        if target_input_id is None:
            target_input_id = self._resolve_current_input_id(input_missions)
        if target_input_id is None:
            return

        # Keep following the currently displayed mission until a new plan is
        # actually applied by 0702/0903. Resetting the old plan here makes the
        # UI jump back to an earlier mission during the replan wait window.
        self._last_active_input_id = int(target_input_id)

        snapshot = self._progress_tracker.update(None, None)
        self._apply_progress_snapshot(snapshot)

    def build_execute_next_replan_context(self) -> dict[str, object] | None:
        view = self._mission_view
        if not isinstance(view, dict):
            return None

        input_missions = view.get("input_missions") or []
        current_input_id = self._pick_current_input_id(input_missions)
        target_input_id = self._pick_next_input_id(input_missions, current_input_id)
        if current_input_id is None or target_input_id is None:
            return None

        current_input_progress = self._input_progress_percent(input_missions, int(current_input_id))
        current_input_done = self._is_input_done(input_missions, int(current_input_id))
        recommendation_active = False
        try:
            recommendation_active = (
                int(current_input_id) in self._sent_0503_inputs
                or int(current_input_id) in self._sent_0503_pending_inputs
                or int(current_input_id) in self._pending_execute_inputs
            )
        except Exception:
            recommendation_active = False

        target_aircraft_ids: list[int] = []
        target_entry_aircraft_list: list[dict[str, object]] = []
        representative_target_coords: list[dict[str, float]] = []
        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = entry.get("aircraft_id")
            try:
                aircraft_id_int = int(aircraft_id) if aircraft_id is not None else None
            except Exception:
                aircraft_id_int = None
            if aircraft_id_int is None or aircraft_id_int <= 0:
                continue
            has_target = False
            target_entry_coord = None
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                mission_input_id = mission.get("input_id")
                try:
                    mission_input_id_int = int(mission_input_id) if mission_input_id is not None else None
                except Exception:
                    mission_input_id_int = None
                if mission_input_id_int is None or mission_input_id_int != int(target_input_id):
                    continue
                has_target = True
                target_entry_coord = self._mission_start_coordinate(mission)
                break
            if has_target and aircraft_id_int not in target_aircraft_ids:
                target_aircraft_ids.append(int(aircraft_id_int))
            if target_entry_coord is not None:
                target_entry_aircraft_list.append(
                    {
                        "aircraftID": int(aircraft_id_int),
                        "coordinate": dict(target_entry_coord),
                    }
                )
                representative_target_coords.append(dict(target_entry_coord))

        if not target_aircraft_ids:
            return None

        return {
            "input_mission_package_id": view.get("input_mission_package_id"),
            "current_input_mission_id": int(current_input_id),
            "target_input_mission_id": int(target_input_id),
            "target_aircraft_ids": list(target_aircraft_ids),
            "current_input_progress_percent": int(current_input_progress),
            "current_input_is_done": bool(current_input_done),
            "current_input_recommendation_active": bool(recommendation_active),
            "entry_strategy": (
                "turn_projection"
                if bool(current_input_done or recommendation_active or current_input_progress >= 100)
                else "midpoint_to_next_start"
            ),
            "target_entry_aircraft_list": target_entry_aircraft_list,
            "representative_target_entry_coordinate": self._centroid_coordinate(representative_target_coords),
        }

    @staticmethod
    def _coord_from_waypoint_summary(payload: object | None) -> dict[str, float] | None:
        if not isinstance(payload, dict):
            return None
        try:
            lat = float(payload.get("latitude"))
            lon = float(payload.get("longitude"))
        except Exception:
            return None
        coord: dict[str, float] = {
            "latitude": float(lat),
            "longitude": float(lon),
        }
        try:
            alt = float(payload.get("altitude"))
        except Exception:
            alt = None
        if alt is not None:
            coord["altitude"] = float(alt)
        return coord

    @classmethod
    def _coord_from_points(cls, payload: object | None) -> dict[str, float] | None:
        if not isinstance(payload, dict):
            return None
        try:
            lat = float(payload.get("latitude"))
            lon = float(payload.get("longitude"))
        except Exception:
            return None
        coord: dict[str, float] = {
            "latitude": float(lat),
            "longitude": float(lon),
        }
        try:
            alt = float(payload.get("altitude"))
        except Exception:
            alt = None
        if alt is not None:
            coord["altitude"] = float(alt)
        return coord

    @classmethod
    def _centroid_coordinate(cls, coords: list[dict[str, float]]) -> dict[str, float] | None:
        if not coords:
            return None
        lat_vals = [float(item["latitude"]) for item in coords if "latitude" in item]
        lon_vals = [float(item["longitude"]) for item in coords if "longitude" in item]
        if not lat_vals or not lon_vals:
            return None
        result: dict[str, float] = {
            "latitude": sum(lat_vals) / float(len(lat_vals)),
            "longitude": sum(lon_vals) / float(len(lon_vals)),
        }
        alt_vals = [float(item["altitude"]) for item in coords if "altitude" in item]
        if alt_vals:
            result["altitude"] = sum(alt_vals) / float(len(alt_vals))
        return result

    @classmethod
    def _mission_start_coordinate(cls, mission: dict) -> dict[str, float] | None:
        waypoints = mission.get("waypoints") if isinstance(mission.get("waypoints"), list) else []
        preferred: dict[str, float] | None = None
        for waypoint in waypoints:
            coord = cls._coord_from_waypoint_summary(waypoint)
            if coord is None:
                continue
            operation_mode = waypoint.get("operation_mode")
            pass_type = waypoint.get("waypoint_pass_type")
            try:
                operation_mode_int = int(operation_mode) if operation_mode is not None else None
            except Exception:
                operation_mode_int = None
            try:
                pass_type_int = int(pass_type) if pass_type is not None else None
            except Exception:
                pass_type_int = None
            if operation_mode_int == 4 or pass_type_int == 2:
                if preferred is None:
                    preferred = dict(coord)
                continue
            return dict(coord)
        if preferred is not None:
            return preferred

        line_list = mission.get("line_list") if isinstance(mission.get("line_list"), list) else []
        for line in line_list:
            if not isinstance(line, dict):
                continue
            coords = line.get("coordinateList") if isinstance(line.get("coordinateList"), list) else []
            for coord_payload in coords:
                coord = cls._coord_from_points(coord_payload)
                if coord is not None:
                    return coord

        area_list = mission.get("area_list") if isinstance(mission.get("area_list"), list) else []
        area_centers: list[dict[str, float]] = []
        for area in area_list:
            if not isinstance(area, dict):
                continue
            coords = area.get("coordinateList") if isinstance(area.get("coordinateList"), list) else []
            normalized: list[dict[str, float]] = []
            for coord_payload in coords:
                coord = cls._coord_from_points(coord_payload)
                if coord is not None:
                    normalized.append(coord)
            if normalized:
                center = cls._centroid_coordinate(normalized)
                if center is not None:
                    area_centers.append(center)
        return cls._centroid_coordinate(area_centers)

    @staticmethod
    def _input_progress_percent(input_missions: list[dict], input_id: int) -> int:
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            try:
                value_int = int(item.get("input_mission_id"))
            except Exception:
                value_int = None
            if value_int is None or int(value_int) != int(input_id):
                continue
            try:
                return int(item.get("progress_percent") or 0)
            except Exception:
                return 0
        return 0

    def _pick_current_input_id(self, input_missions: list[dict]) -> int | None:
        current_input_id = self._progress_tracker.get_active_input_id()
        if current_input_id is None:
            current_input_id = self._last_progress_input_id
        if current_input_id is None:
            current_input_id = self._last_active_input_id
        if current_input_id is None:
            current_input_id = self._resolve_current_input_id(input_missions)
        try:
            return int(current_input_id) if current_input_id is not None else None
        except Exception:
            return None

    def _pick_next_input_id(self, input_missions: list[dict], current_input_id: int | None) -> int | None:
        ordered_ids: list[int] = []
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            value = item.get("input_mission_id")
            try:
                value_int = int(value) if value is not None else None
            except Exception:
                value_int = None
            if value_int is None:
                continue
            if value_int not in ordered_ids:
                ordered_ids.append(value_int)
        if not ordered_ids:
            return None
        if current_input_id is None:
            next_pending = self._first_pending_id(input_missions, "input_mission_id")
            if next_pending is not None:
                return int(next_pending)
            return int(ordered_ids[0])
        try:
            idx = ordered_ids.index(int(current_input_id))
        except Exception:
            next_pending = self._first_pending_id(input_missions, "input_mission_id")
            if next_pending is not None:
                return int(next_pending)
            return int(ordered_ids[0])
        next_idx = idx + 1
        if 0 <= next_idx < len(ordered_ids):
            return int(ordered_ids[next_idx])
        return None

    @staticmethod
    def _is_input_done(input_missions: list[dict], input_id: int) -> bool:
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            value = item.get("input_mission_id")
            try:
                value_int = int(value) if value is not None else None
            except Exception:
                value_int = None
            if value_int is None or int(value_int) != int(input_id):
                continue
            return bool(item.get("is_done"))
        return False

    def _force_complete_input_mission(self, view: dict, input_id: int) -> None:
        completed = self._progress_tracker.force_complete_input(input_id)
        input_package_id = view.get("input_mission_package_id")
        mark_input_mission_done(input_package_id, input_id)

        completed_mission_ids: set[int] = set()
        for item in completed:
            if not isinstance(item, dict):
                continue
            mission_id = item.get("mission_id")
            package_id = item.get("package_id")
            try:
                mission_id_int = int(mission_id) if mission_id is not None else None
            except Exception:
                mission_id_int = None
            if mission_id_int is None:
                continue
            completed_mission_ids.add(mission_id_int)
            mark_individual_mission_done(package_id, mission_id_int)

        for item in view.get("input_missions") or []:
            if not isinstance(item, dict):
                continue
            value = item.get("input_mission_id")
            try:
                value_int = int(value) if value is not None else None
            except Exception:
                value_int = None
            if value_int is None or int(value_int) != int(input_id):
                continue
            item["is_done"] = True
            item["progress_percent"] = 100

        for entry in view.get("uav_entries") or []:
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                input_value = mission.get("input_id")
                try:
                    input_value_int = int(input_value) if input_value is not None else None
                except Exception:
                    input_value_int = None
                if input_value_int is None or int(input_value_int) != int(input_id):
                    continue
                mission_id = mission.get("individual_mission_id")
                try:
                    mission_id_int = int(mission_id) if mission_id is not None else None
                except Exception:
                    mission_id_int = None
                if mission_id_int is None:
                    continue
                completed_mission_ids.add(mission_id_int)
                mission["is_done"] = True
                mission["progress_percent"] = 100
                waypoint_ids = mission.get("waypoint_ids") or []
                path_id = mission.get("path_id")
                mark_waypoints_done(path_id, waypoint_ids)
                status_list = mission.get("waypoint_status")
                if isinstance(status_list, list):
                    for wp in status_list:
                        if not isinstance(wp, dict):
                            continue
                        status = str(wp.get("status") or "pending")
                        if status != "reached":
                            wp["status"] = "skipped"

        self._forced_completion_inputs.add(int(input_id))
        self._last_forced_input_id = int(input_id)
        self._last_forced_mission_ids = sorted(completed_mission_ids)
        try:
            self._sent_0503_pending_inputs.discard(int(input_id))
        except Exception:
            pass
        try:
            new_pending: list[int] = []
            for pid in self._pending_completion_inputs:
                try:
                    if int(pid) == int(input_id):
                        continue
                except Exception:
                    pass
                new_pending.append(pid)
            self._pending_completion_inputs = new_pending
        except Exception:
            self._pending_completion_inputs = []
        try:
            new_pending_execute: list[int] = []
            for pid in self._pending_execute_inputs:
                try:
                    if int(pid) == int(input_id):
                        continue
                except Exception:
                    pass
                new_pending_execute.append(pid)
            self._pending_execute_inputs = new_pending_execute
        except Exception:
            self._pending_execute_inputs = []
        self._sent_final_completion = False

    def _repeat_input_mission(self, view: dict, input_id: int) -> None:
        input_package_id = view.get("input_mission_package_id")
        mark_input_mission_undone(input_package_id, input_id)
        try:
            self._sent_0503_inputs.discard(int(input_id))
        except Exception:
            pass
        try:
            self._sent_0503_pending_inputs.discard(int(input_id))
        except Exception:
            pass
        try:
            self._forced_completion_inputs.discard(int(input_id))
        except Exception:
            pass
        try:
            new_pending: list[int] = []
            for pid in self._pending_completion_inputs:
                try:
                    if int(pid) == int(input_id):
                        continue
                except Exception:
                    pass
                new_pending.append(pid)
            self._pending_completion_inputs = new_pending
        except Exception:
            self._pending_completion_inputs = []
        try:
            new_pending_execute: list[int] = []
            for pid in self._pending_execute_inputs:
                try:
                    if int(pid) == int(input_id):
                        continue
                except Exception:
                    pass
                new_pending_execute.append(pid)
            self._pending_execute_inputs = new_pending_execute
        except Exception:
            self._pending_execute_inputs = []
        self._sent_final_completion = False

        missions_by_id: dict[int, int | None] = {}
        for entry in view.get("uav_entries") or []:
            package_id = entry.get("individual_mission_package_id")
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                mission_id = mission.get("individual_mission_id")
                if mission_id is None:
                    continue
                try:
                    missions_by_id[int(mission_id)] = package_id
                except Exception:
                    continue

        for mission_id, package_id in missions_by_id.items():
            if not any(
                isinstance(m, dict) and m.get("individual_mission_id") == mission_id and m.get("input_id") == input_id
                for entry in view.get("uav_entries") or []
                for m in entry.get("missions") or []
            ):
                continue
            mark_individual_mission_undone(package_id, mission_id)

        for item in view.get("input_missions") or []:
            if isinstance(item, dict) and item.get("input_mission_id") == input_id:
                item["is_done"] = False

        for entry in view.get("uav_entries") or []:
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                if mission.get("input_id") != input_id:
                    continue
                mission["is_done"] = False
        try:
            self._sent_0503_inputs.discard(int(input_id))
        except Exception:
            pass
        try:
            self._forced_completion_inputs.discard(int(input_id))
        except Exception:
            pass

        self._progress_tracker.reset_input_progress(input_id)

    def _undo_forced_completion(
        self,
        view: dict,
        input_id: int,
        mission_ids: list[int],
    ) -> None:
        input_package_id = view.get("input_mission_package_id")
        mark_input_mission_undone(input_package_id, input_id)

        missions_by_id: dict[int, int | None] = {}
        for entry in view.get("uav_entries") or []:
            package_id = entry.get("individual_mission_package_id")
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                mission_id = mission.get("individual_mission_id")
                if mission_id is None:
                    continue
                try:
                    missions_by_id[int(mission_id)] = package_id
                except Exception:
                    continue

        for mission_id in mission_ids:
            package_id = missions_by_id.get(int(mission_id))
            mark_individual_mission_undone(package_id, mission_id)

        for item in view.get("input_missions") or []:
            if isinstance(item, dict) and item.get("input_mission_id") == input_id:
                item["is_done"] = False

        for entry in view.get("uav_entries") or []:
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                if mission.get("input_id") != input_id:
                    continue
                mission["is_done"] = False
        try:
            self._forced_completion_inputs.discard(int(input_id))
        except Exception:
            pass
        try:
            new_pending: list[int] = []
            for pid in self._pending_completion_inputs:
                try:
                    if int(pid) == int(input_id):
                        continue
                except Exception:
                    pass
                new_pending.append(pid)
            self._pending_completion_inputs = new_pending
        except Exception:
            self._pending_completion_inputs = []
        try:
            new_pending_execute: list[int] = []
            for pid in self._pending_execute_inputs:
                try:
                    if int(pid) == int(input_id):
                        continue
                except Exception:
                    pass
                new_pending_execute.append(pid)
            self._pending_execute_inputs = new_pending_execute
        except Exception:
            self._pending_execute_inputs = []
        try:
            self._sent_0503_pending_inputs.discard(int(input_id))
        except Exception:
            pass
        self._sent_final_completion = False

        self._progress_tracker.reset_input_progress(input_id)

    def _reset_progress_only(self, view: dict, input_id: int) -> None:
        for item in view.get("input_missions") or []:
            if isinstance(item, dict) and item.get("input_mission_id") == input_id:
                item["is_done"] = False

        for entry in view.get("uav_entries") or []:
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                if mission.get("input_id") != input_id:
                    continue
                mission["is_done"] = False
        try:
            self._forced_completion_inputs.discard(int(input_id))
        except Exception:
            pass
        try:
            new_pending: list[int] = []
            for pid in self._pending_completion_inputs:
                try:
                    if int(pid) == int(input_id):
                        continue
                except Exception:
                    pass
                new_pending.append(pid)
            self._pending_completion_inputs = new_pending
        except Exception:
            self._pending_completion_inputs = []
        try:
            new_pending_execute: list[int] = []
            for pid in self._pending_execute_inputs:
                try:
                    if int(pid) == int(input_id):
                        continue
                except Exception:
                    pass
                new_pending_execute.append(pid)
            self._pending_execute_inputs = new_pending_execute
        except Exception:
            self._pending_execute_inputs = []
        self._sent_final_completion = False

        self._progress_tracker.reset_input_progress(input_id)
        try:
            self._sent_0503_inputs.discard(int(input_id))
        except Exception:
            pass
        try:
            self._sent_0503_pending_inputs.discard(int(input_id))
        except Exception:
            pass

    def _apply_mission_plan_view(self, mission_plan_id: int | None) -> None:
        view = build_uav_mission_view(mission_plan_id, uav_ids=(4, 5, 6))
        self._mission_view = view
        self._progress_tracker.reset(view)
        self._last_forced_input_id = None
        self._last_forced_mission_ids = []
        self._last_active_input_id = None
        self._last_progress_input_id = None
        self._sent_0503_inputs = {
            int(item.get("input_mission_id"))
            for item in view.get("input_missions") or []
            if isinstance(item, dict) and item.get("input_mission_id") is not None and item.get("is_done")
        }
        self._forced_completion_inputs = set()
        self._pending_completion_inputs = []
        self._pending_execute_inputs = []
        self._sent_0503_pending_inputs = set()
        self._sent_final_completion = False
        entries = view.get("uav_entries") or []

        for idx, field in enumerate(self._individual_package_fields):
            package_id = None
            if idx < len(entries):
                package_id = entries[idx].get("individual_mission_package_id")
            field.setText(self._fmt_value(package_id))

        for idx, table in enumerate(self._mission_tables):
            table.setRowCount(0)
            if idx >= len(entries):
                continue
            entry = entries[idx]
            missions = entry.get("missions") or []
            for mission in missions:
                row = table.rowCount()
                table.insertRow(row)
                table.setItem(row, 0, QTableWidgetItem(self._fmt_value(mission.get("individual_mission_id"))))
                table.setItem(row, 1, QTableWidgetItem(self._fmt_value(mission.get("input_id"))))
                table.setItem(row, 2, QTableWidgetItem(self._fmt_value(mission.get("path_id"))))
                table.setItem(row, 3, QTableWidgetItem(self._fmt_waypoints(mission.get("waypoint_ids"))))
                coverage_text = "-"
                if mission.get("coverage_enabled"):
                    try:
                        coverage_text = f"{int(mission.get('coverage_percent') or 0)}%"
                    except Exception:
                        coverage_text = "-%"
                table.setItem(row, 4, QTableWidgetItem(coverage_text))
        snapshot = self._progress_tracker.update(None, None)
        self._apply_progress_snapshot(snapshot)

    def set_recommend_callback(self, callback) -> None:
        self._recommend_callback = callback

    def set_notice_callback(self, callback) -> None:
        self._notice_callback = callback

    def set_reexecute_callback(self, callback) -> None:
        self._reexecute_callback = callback

    @staticmethod
    def _fmt_value(value: object) -> str:
        if value is None:
            return "-"
        return str(value)

    @staticmethod
    def _fmt_waypoints(value: object) -> str:
        if not value:
            return "-"
        if isinstance(value, (list, tuple)):
            return ", ".join(str(v) for v in value)
        return str(value)

    def _make_id_field(self, text: str) -> QLineEdit:
        field = QLineEdit(text)
        field.setReadOnly(True)
        field.setProperty("_base_fixed_width", 180)
        field.setFixedWidth(180)
        field.setAlignment(Qt.AlignCenter)
        self._id_fields.append(field)
        return field

    def _build_lowlevel_scroll(
        self,
        *,
        base_height: int = 44,
        min_height: int = 28,
    ) -> tuple[QScrollArea, QHBoxLayout]:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setProperty("_base_fixed_height", int(base_height))
        scroll.setProperty("_min_fixed_height", int(min_height))
        scroll.setFixedHeight(int(base_height))

        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        scroll.setWidget(container)
        self._lowlevel_scrolls.append(scroll)
        return scroll, layout

    def _populate_lowlevel_bars(
        self,
        layout: QHBoxLayout,
        items: list[dict],
        current_id: int | None,
        *,
        id_key: str,
        progress_key: str = "progress_percent",
        actual_key: str = "actual_seconds",
        planned_key: str = "planned_seconds",
        eta_key: str | None = None,
    ) -> None:
        self._clear_layout(layout)
        for item in items:
            if not isinstance(item, dict):
                continue
            item_id = item.get(id_key)
            if item_id is None:
                continue
            label = item.get("label") or str(item_id)
            is_done = bool(item.get("is_done"))
            value = item.get(progress_key)
            if value is None:
                value = 100 if is_done else 0
            try:
                progress_percent = int(value)
            except Exception:
                progress_percent = 0
            highlight = current_id is not None and int(item_id) == int(current_id)
            actual_seconds = item.get(actual_key)
            planned_seconds = item.get(planned_key)
            if planned_seconds is None and eta_key:
                planned_seconds = item.get(eta_key)
            bar = self._make_small_progress_bar(
                str(label),
                int(progress_percent),
                highlight=highlight,
                actual_seconds=actual_seconds,
                planned_seconds=planned_seconds,
                coverage_percent=item.get("coverage_percent"),
                coverage_enabled=bool(item.get("coverage_enabled")),
            )
            layout.addWidget(bar)
        layout.addStretch(1)

    @staticmethod
    def _clear_layout(layout: QHBoxLayout) -> None:
        while layout.count():
            item = layout.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.deleteLater()

    def _make_small_progress_bar(
        self,
        label: str,
        value: int,
        *,
        highlight: bool = False,
        actual_seconds: object | None = None,
        planned_seconds: object | None = None,
        coverage_percent: object | None = None,
        coverage_enabled: bool = False,
    ) -> QProgressBar:
        bar = QProgressBar()
        bar.setRange(0, 100)
        bar.setValue(int(value))
        suffix = ""
        if planned_seconds is not None:
            try:
                planned_int = int(planned_seconds)
                actual_int = int(actual_seconds) if actual_seconds is not None else 0
                suffix = f" ({actual_int:04d}s/{planned_int:04d}s)"
            except Exception:
                suffix = " (----s/----s)"
        format_text = f"{label} - %p%{suffix}"
        if coverage_enabled:
            try:
                coverage_text = f"{int(coverage_percent or 0)}%"
            except Exception:
                coverage_text = "-%"
            format_text = f"{format_text} | 촬영 {coverage_text}"
        bar.setFormat(format_text)
        bar.setTextVisible(True)
        bar.setAlignment(Qt.AlignCenter)
        bar.setProperty("_small_progress", True)
        bar.setProperty("_small_progress_highlight", bool(highlight))
        self._style_small_progress_bar(bar, highlight=highlight)
        return bar

    def _style_small_progress_bar(self, bar: QProgressBar, *, highlight: bool) -> None:
        bar.setFixedHeight(self._scaled(20, min_value=16))
        bar.setMinimumWidth(self._scaled(260, min_value=160))
        border_width = self._scaled(2 if highlight else 1, min_value=1)
        radius = self._scaled(3, min_value=2)
        font_px = self._scaled(11, min_value=8)
        border_color = "#2563eb" if highlight else "#9aa3a8"
        bar.setStyleSheet(
            "QProgressBar { "
            f"border: {border_width}px solid {border_color}; "
            f"border-radius: {radius}px; "
            "background: #f5f5f5; text-align: center; "
            f"font-size: {font_px}px; "
            "}"
            "QProgressBar::chunk { background-color: #7ee38b; }"
        )

    def _wp_chip_style(self, status: str) -> str:
        if status == "reached":
            bg = "#2563eb"
            fg = "#ffffff"
        elif status == "skipped":
            bg = "#f59e0b"
            fg = "#111827"
        else:
            bg = "#111827"
            fg = "#f9fafb"
        radius = self._scaled(3, min_value=2)
        pad_v = self._scaled(1, min_value=1)
        pad_h = self._scaled(6, min_value=4)
        font_px = self._scaled(11, min_value=8)
        return (
            "QLabel {"
            f" background: {bg}; color: {fg};"
            f" border-radius: {radius}px; padding: {pad_v}px {pad_h}px;"
            f" font-size: {font_px}px; font-weight: 600;"
            " }"
        )

    def _populate_waypoint_status_row(
        self,
        layout: QHBoxLayout,
        missions: list[dict],
        current_mission_id: int | None,
    ) -> None:
        self._clear_layout(layout)
        mission_obj: dict | None = None
        if current_mission_id is not None:
            for mission in missions:
                if not isinstance(mission, dict):
                    continue
                try:
                    if int(mission.get("individual_mission_id")) == int(current_mission_id):
                        mission_obj = mission
                        break
                except Exception:
                    continue
        if mission_obj is None:
            for mission in reversed(missions):
                if not isinstance(mission, dict):
                    continue
                mission_obj = mission
                break
        if not isinstance(mission_obj, dict):
            label = QLabel("WP -")
            label.setProperty("_wp_status", "pending")
            label.setStyleSheet(self._wp_chip_style("pending"))
            layout.addWidget(label)
            layout.addStretch(1)
            return
        status_by_wp: dict[int, str] = {}
        for item in mission_obj.get("waypoint_status") or []:
            if not isinstance(item, dict):
                continue
            wid = item.get("waypoint_id")
            status = str(item.get("status") or "pending")
            try:
                wid_int = int(wid)
            except Exception:
                continue
            status_by_wp[wid_int] = status
        wp_ids = mission_obj.get("waypoint_ids") or []
        for raw_wid in wp_ids:
            try:
                wid = int(raw_wid)
            except Exception:
                continue
            status = status_by_wp.get(wid, "pending")
            chip = QLabel(f"WP{wid}")
            chip.setAlignment(Qt.AlignCenter)
            chip.setProperty("_wp_status", status)
            chip.setStyleSheet(self._wp_chip_style(status))
            layout.addWidget(chip)
        layout.addStretch(1)

    def _make_pill_row_with_ids(
        self,
        items: list[tuple[int, str]],
        *,
        registry: dict[int, QLabel],
    ) -> QWidget:
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        for aircraft_id, text in items:
            label = QLabel(text)
            label.setAlignment(Qt.AlignCenter)
            label.setProperty("_pill_status", "unknown")
            label.setStyleSheet(self._pill_style("unknown"))
            registry[int(aircraft_id)] = label
            layout.addWidget(label)
        layout.addStretch(1)
        return container

    def _make_progress_bar(self) -> QProgressBar:
        bar = QProgressBar()
        bar.setRange(0, 100)
        bar.setValue(0)
        bar.setFormat("진행 %p%")
        bar.setTextVisible(True)
        bar.setProperty("_main_progress", True)
        self._style_main_progress_bar(bar)
        return bar

    def _style_main_progress_bar(self, bar: QProgressBar) -> None:
        border_width = self._scaled(1, min_value=1)
        radius = self._scaled(3, min_value=2)
        min_h = self._scaled(22, min_value=18)
        font_px = self._scaled(11, min_value=8)
        bar.setMinimumHeight(min_h)
        bar.setStyleSheet(
            "QProgressBar { "
            f"border: {border_width}px solid #9aa3a8; "
            f"border-radius: {radius}px; "
            "background: #f5f5f5; text-align: center; "
            f"font-size: {font_px}px; "
            "}"
            "QProgressBar::chunk { background-color: #7ee38b; }"
        )

    def _pill_style(self, status: str | bool) -> str:
        if isinstance(status, bool):
            status = "good" if status else "bad"
        color_map = {
            "good": "#22c55e",
            "warn": "#f59e0b",
            "bad": "#ef4444",
            "unknown": "#9aa3a8",
        }
        color = color_map.get(status, "#9aa3a8")
        pad_v = self._scaled(4, min_value=2)
        pad_h = self._scaled(10, min_value=6)
        radius = self._scaled(6, min_value=4)
        font_px = self._scaled(11, min_value=8)
        return (
            f"padding: {pad_v}px {pad_h}px; border-radius: {radius}px; "
            f"background-color: {color}; color: #ffffff; font-weight: 600; font-size: {font_px}px;"
        )

    def _dot_color_for_aircraft(self, aircraft_id: int) -> str:
        base_color = "#9ca3af"
        if int(aircraft_id) in self._available_ids:
            base_color = "#22c55e"
        fuel_state = self._fuel_state_by_aircraft.get(int(aircraft_id))
        if fuel_state == "yellow":
            return "#f59e0b"
        if fuel_state == "red":
            return "#ef4444"
        return base_color

    @staticmethod
    def _coverage_label_text(data: dict[str, object] | None) -> str:
        if not isinstance(data, dict) or not bool(data.get("coverage_enabled")):
            return "촬영 -"
        try:
            return f"촬영 {int(data.get('coverage_percent') or 0)}%"
        except Exception:
            return "촬영 -"

    def _update_mission_header(self, aircraft_id: int) -> None:
        label = self._mission_header_labels.get(int(aircraft_id))
        if label is None:
            return
        dot_color = self._dot_color_for_aircraft(int(aircraft_id))
        coverage_text = "촬영 -"
        snapshot = self._last_progress_snapshot if isinstance(self._last_progress_snapshot, dict) else {}
        package_coverage = snapshot.get("package_coverage") if isinstance(snapshot, dict) else {}
        if isinstance(package_coverage, dict):
            coverage_text = self._coverage_label_text(package_coverage.get(int(aircraft_id)))
        label.setText(
            f"UAV {int(aircraft_id)} <span style=\"color: {dot_color};\">●</span> | {coverage_text}"
        )

    @staticmethod
    def _signal_ok(timestamp_ms: int | None, last_signal_time: int | None) -> bool:
        if timestamp_ms is None or last_signal_time is None:
            return False
        try:
            ts = int(timestamp_ms)
            ls = int(last_signal_time)
        except Exception:
            return False
        if ls <= 0:
            return False
        return abs(ts - ls) <= _SIGNAL_OK_GRACE_MS

    @staticmethod
    def _availability_stage_priority(stage: str | None) -> int:
        if stage == "0802":
            return 3
        if stage == "0401":
            return 2
        if stage == "0201":
            return 1
        return 0

    def _make_pill_row(self, items: list[str]) -> QWidget:
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        for text in items:
            label = QLabel(text)
            label.setAlignment(Qt.AlignCenter)
            label.setProperty("_pill_status", "unknown")
            label.setStyleSheet(self._pill_style("unknown"))
            layout.addWidget(label)
        layout.addStretch(1)
        return container

    def _scaled(self, value: int, *, min_value: int = 0) -> int:
        scaled = int(round(float(value) * float(self._ui_scale)))
        if scaled < min_value:
            return int(min_value)
        return scaled

    def _target_ui_scale(self) -> float:
        w = max(1, self.width())
        h = max(1, self.height())
        width_scale = w / 1220.0
        height_scale = h / 860.0
        return max(0.64, min(1.0, min(width_scale, height_scale)))

    def resizeEvent(self, event) -> None:  # type: ignore[override]
        super().resizeEvent(event)
        self._apply_responsive_scale()

    def _apply_responsive_scale(self) -> None:
        scale = self._target_ui_scale()
        if self._last_applied_scale > 0 and abs(scale - self._last_applied_scale) < 0.02:
            return
        self._ui_scale = scale
        self._last_applied_scale = scale

        font = self.font()
        font.setPointSizeF(max(7.0, self._base_font_size * scale))
        self.setFont(font)

        for layout in self.findChildren(QLayout):
            key = id(layout)
            if key not in self._layout_base_metrics:
                margins = layout.contentsMargins()
                self._layout_base_metrics[key] = (
                    (margins.left(), margins.top(), margins.right(), margins.bottom()),
                    int(layout.spacing()),
                )
            base_margins, base_spacing = self._layout_base_metrics[key]
            layout.setContentsMargins(
                self._scaled(base_margins[0]),
                self._scaled(base_margins[1]),
                self._scaled(base_margins[2]),
                self._scaled(base_margins[3]),
            )
            if base_spacing >= 0:
                layout.setSpacing(self._scaled(base_spacing))

        for field in self._id_fields:
            try:
                base_w = int(field.property("_base_fixed_width") or 180)
            except Exception:
                base_w = 180
            field.setFixedWidth(self._scaled(base_w, min_value=120))

        for scroll in self._lowlevel_scrolls:
            try:
                base_h = int(scroll.property("_base_fixed_height") or 44)
            except Exception:
                base_h = 44
            try:
                min_h = int(scroll.property("_min_fixed_height") or 28)
            except Exception:
                min_h = 28
            scroll.setFixedHeight(self._scaled(base_h, min_value=min_h))

        for bar in self._summary_progress_bars:
            self._style_main_progress_bar(bar)
        for bar in self.findChildren(QProgressBar):
            if bool(bar.property("_small_progress")):
                self._style_small_progress_bar(
                    bar,
                    highlight=bool(bar.property("_small_progress_highlight")),
                )

        for table in self._mission_tables:
            try:
                base_h = int(table.property("_base_min_height") or 160)
            except Exception:
                base_h = 160
            try:
                base_header_h = int(table.property("_base_header_min_height") or 22)
            except Exception:
                base_header_h = 22
            table.setMinimumHeight(self._scaled(base_h, min_value=96))
            table.horizontalHeader().setMinimumHeight(self._scaled(base_header_h, min_value=18))
            table.verticalHeader().setDefaultSectionSize(self._scaled(22, min_value=18))

        if self._dl_risk_trend is not None:
            trend = self._dl_risk_trend
            try:
                base_h = int(trend.property("_base_min_height") or 320)
            except Exception:
                base_h = 320
            trend.setMinimumHeight(self._scaled(base_h, min_value=160))
            try:
                trend.set_ui_scale(scale)
            except Exception:
                pass

        for label in list(self._aircraft_labels.values()) + list(self._signal_labels.values()):
            status = str(label.property("_pill_status") or "unknown")
            label.setStyleSheet(self._pill_style(status))
        for label in self.findChildren(QLabel):
            wp_status = label.property("_wp_status")
            if wp_status is not None:
                label.setStyleSheet(self._wp_chip_style(str(wp_status)))
        self._refresh_dl_panel_view()

    def _update_mission_progress(
        self,
        timestamp_ms: int | None,
        agent_states: list[dict],
        *,
        update_ui: bool,
    ) -> None:
        if not self._mission_view:
            return
        snapshot = self._progress_tracker.update(timestamp_ms, agent_states)
        active_input_id = self._progress_tracker.get_active_input_id()
        if active_input_id is not None:
            snapshot["active_input_id"] = active_input_id
        self._apply_progress_snapshot(snapshot, update_ui=update_ui)
        self._apply_completion_updates(snapshot)
        self._log_sweep_progress(snapshot, timestamp_ms)

    def _log_sweep_progress(self, snapshot: dict, timestamp_ms: int | None) -> None:
        view = self._mission_view
        if not view:
            return
        mission_progress = snapshot.get("mission_progress") or {}
        if not mission_progress:
            return
        buffer_sec = int(self._sweep_log_buffer_sec)
        updated = False
        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = entry.get("aircraft_id")
            try:
                aircraft_id_int = int(aircraft_id)
            except Exception:
                continue
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                mission_id = mission.get("individual_mission_id")
                try:
                    mission_id_int = int(mission_id) if mission_id is not None else None
                except Exception:
                    mission_id_int = None
                if mission_id_int is None:
                    continue
                progress = mission_progress.get(mission_id_int)
                if not isinstance(progress, dict):
                    continue
                sweep_count = int(mission.get("sweep_point_count") or 0)
                if sweep_count <= 0:
                    continue
                planned_seconds = int(progress.get("planned_seconds") or 0)
                if planned_seconds <= 0:
                    continue
                actual_seconds = int(progress.get("actual_seconds") or 0)
                remaining_seconds = max(0, planned_seconds - actual_seconds)
                progress_ratio = max(0.0, min(1.0, actual_seconds / planned_seconds))
                buffer_ratio = max(0.0, min(1.0, (actual_seconds + buffer_sec) / planned_seconds))
                progress_pct = int(round(progress_ratio * 100))
                buffer_pct = int(round(buffer_ratio * 100))
                completed_points = min(sweep_count, int(progress_ratio * sweep_count))
                buffer_points = min(sweep_count, int(buffer_ratio * sweep_count))
                sec_per_point = planned_seconds / max(1, sweep_count)

                key = (aircraft_id_int, mission_id_int)
                last = self._sweep_log_state.get(key) or {}
                last_ms = last.get("ms")
                last_pct = last.get("pct")
                last_buf = last.get("buf")
                if (
                    timestamp_ms is not None
                    and last_ms is not None
                    and abs(int(timestamp_ms) - int(last_ms)) < 1000
                    and last_pct == progress_pct
                    and last_buf == buffer_pct
                ):
                    continue

                self._sweep_log_state[key] = {
                    "ms": int(timestamp_ms) if timestamp_ms is not None else 0,
                    "pct": progress_pct,
                    "buf": buffer_pct,
                }
                path_id = mission.get("path_id")
                self._sweep_progress_cache[key] = {
                    "timestamp_ms": int(timestamp_ms) if timestamp_ms is not None else None,
                    "aircraft_id": aircraft_id_int,
                    "mission_id": mission_id_int,
                    "path_id": path_id,
                    "elapsed_seconds": actual_seconds,
                    "remaining_seconds": remaining_seconds,
                    "planned_seconds": planned_seconds,
                    "sweep_point_count": sweep_count,
                    "seconds_per_point": round(sec_per_point, 3),
                    "progress_percent": progress_pct,
                    "progress_points": completed_points,
                    "buffer_seconds": buffer_sec,
                    "buffer_percent": buffer_pct,
                    "buffer_points": buffer_points,
                }
                updated = True
        if updated:
            self._persist_sweep_progress()

    def _persist_sweep_progress(self) -> None:
        try:
            base = db_paths.get_db_subpath("DSS_Internal")
        except Exception:
            return
        payload = {
            "timestamp_ms": None,
            "entries": list(self._sweep_progress_cache.values()),
        }
        for entry in payload["entries"]:
            ts = entry.get("timestamp_ms")
            if ts is None:
                continue
            payload["timestamp_ms"] = ts
            break
        try:
            base.mkdir(parents=True, exist_ok=True)
            path = base / "sweep_progress.json"
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            pass

    def _persist_coverage_progress(self, snapshot: dict) -> None:
        view = self._mission_view
        if not view:
            return
        try:
            base = db_paths.get_db_subpath("DSS_Internal")
        except Exception:
            return
        mission_entries: list[dict[str, object]] = []
        for entry in view.get("uav_entries") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = entry.get("aircraft_id")
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                mission_entries.append(
                    {
                        "aircraft_id": aircraft_id,
                        "input_id": mission.get("input_id"),
                        "mission_id": mission.get("individual_mission_id"),
                        "mission_type": mission.get("individual_mission_type"),
                        "coverage_enabled": bool(mission.get("coverage_enabled")),
                        "coverage_percent": int(mission.get("coverage_percent") or 0),
                        "covered_area_m2": float(mission.get("covered_area_m2") or 0.0),
                        "planned_area_m2": float(mission.get("planned_area_m2") or 0.0),
                        "done": bool(mission.get("is_done")),
                    }
                )
        payload = {
            "timestamp_ms": snapshot.get("timestamp_ms") or self._last_status_timestamp_ms,
            "mission_plan_id": view.get("mission_plan_id"),
            "plan_coverage": dict(snapshot.get("plan_coverage") or {}),
            "input_coverage": {
                str(key): dict(value)
                for key, value in (snapshot.get("input_coverage") or {}).items()
                if isinstance(value, dict)
            },
            "package_coverage": {
                str(key): dict(value)
                for key, value in (snapshot.get("package_coverage") or {}).items()
                if isinstance(value, dict)
            },
            "missions": mission_entries,
        }
        try:
            base.mkdir(parents=True, exist_ok=True)
            path = base / "coverage_progress.json"
            path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            pass

    def _apply_progress_snapshot(self, snapshot: dict, *, update_ui: bool = True) -> None:
        view = self._mission_view
        if not view:
            return
        prev_snapshot = self._last_progress_snapshot
        self._last_progress_snapshot = snapshot
        self._update_last_progress_input(snapshot, prev_snapshot)
        input_progress = snapshot.get("input_progress") or {}
        mission_progress = snapshot.get("mission_progress") or {}
        package_progress = snapshot.get("package_progress") or {}
        plan_progress = snapshot.get("plan_progress") or {}
        package_coverage = snapshot.get("package_coverage") or {}
        input_coverage = snapshot.get("input_coverage") or {}
        plan_coverage = snapshot.get("plan_coverage") or {}

        input_missions = view.get("input_missions") or []
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            input_id = item.get("input_mission_id")
            if input_id in input_progress:
                prog = input_progress[input_id]
                item["progress_percent"] = prog.get("progress_percent", 0)
                item["actual_seconds"] = prog.get("actual_seconds", 0)
                item["planned_seconds"] = prog.get("planned_seconds", 0)
                item["is_done"] = bool(prog.get("done"))
            if input_id in input_coverage:
                cov = input_coverage[input_id]
                item["coverage_percent"] = cov.get("coverage_percent", 0)
                item["covered_area_m2"] = cov.get("covered_area_m2", 0.0)
                item["planned_area_m2"] = cov.get("planned_area_m2", 0.0)
                item["coverage_enabled"] = bool(cov.get("coverage_enabled"))

        current_input_id = self._resolve_current_input_id(input_missions)
        if current_input_id is not None:
            self._last_active_input_id = current_input_id
        self._queue_pending_execute_inputs(input_missions)
        if update_ui:
            if self._input_mission_low_layout is not None:
                self._populate_lowlevel_bars(
                    self._input_mission_low_layout,
                    input_missions,
                    current_input_id,
                    id_key="input_mission_id",
                )
            if self._mission_plan_bar is not None:
                plan_progress_percent = int(plan_progress.get("progress_percent", 0))
                self._mission_plan_bar.setValue(plan_progress_percent)
                if bool(plan_coverage.get("coverage_enabled")):
                    try:
                        coverage_text = f"{int(plan_coverage.get('coverage_percent') or 0)}%"
                    except Exception:
                        coverage_text = "-%"
                    self._mission_plan_bar.setFormat(f"진행 %p% | 촬영 {coverage_text}")
                else:
                    self._mission_plan_bar.setFormat("진행 %p%")

        entries = view.get("uav_entries") or []
        for idx, entry in enumerate(entries):
            missions = entry.get("missions") or []
            aircraft_id = entry.get("aircraft_id")
            for mission in missions:
                if not isinstance(mission, dict):
                    continue
                mission_id = mission.get("individual_mission_id")
                if mission_id in mission_progress:
                    prog = mission_progress[mission_id]
                    mission["progress_percent"] = prog.get("progress_percent", 0)
                    mission["actual_seconds"] = prog.get("actual_seconds", 0)
                    mission["actual_seconds_real"] = prog.get("actual_seconds_real", 0)
                    mission["planned_seconds"] = prog.get("planned_seconds", 0)
                    mission["is_done"] = bool(prog.get("done"))
                    mission["waypoint_status"] = prog.get("waypoint_status") or []
                    mission["coverage_percent"] = prog.get("coverage_percent", 0)
                    mission["covered_area_m2"] = prog.get("covered_area_m2", 0.0)
                    mission["planned_area_m2"] = prog.get("planned_area_m2", 0.0)
                    mission["coverage_enabled"] = bool(prog.get("coverage_enabled"))

            current_mission_id = self._resolve_current_mission_id(
                missions,
                aircraft_id,
                snapshot,
            )
            if current_mission_id is not None:
                entry["current_individual_mission_id"] = int(current_mission_id)
            if update_ui and idx < len(self._individual_low_layouts):
                self._populate_lowlevel_bars(
                    self._individual_low_layouts[idx],
                    missions,
                    current_mission_id,
                    id_key="individual_mission_id",
                    actual_key="actual_seconds_real",
                    eta_key="eta_seconds",
                )
            if update_ui and idx < len(self._individual_wp_layouts):
                self._populate_waypoint_status_row(
                    self._individual_wp_layouts[idx],
                    missions,
                    current_mission_id,
                )

            aircraft_id = entry.get("aircraft_id")
            if (
                update_ui
                and idx < len(self._individual_package_bars)
                and aircraft_id in package_progress
            ):
                pkg_progress_percent = int(package_progress[aircraft_id].get("progress_percent", 0))
                pkg_cov = package_coverage.get(aircraft_id) or {}
                self._individual_package_bars[idx].setValue(pkg_progress_percent)
                if bool(pkg_cov.get("coverage_enabled")):
                    try:
                        coverage_text = f"{int(pkg_cov.get('coverage_percent') or 0)}%"
                    except Exception:
                        coverage_text = "-%"
                    self._individual_package_bars[idx].setFormat(
                        f"진행 %p% | 촬영 {coverage_text}"
                    )
                else:
                    self._individual_package_bars[idx].setFormat("진행 %p%")

            if update_ui and idx < len(self._mission_tables):
                table = self._mission_tables[idx]
                table_current_id = current_mission_id
                for row in range(min(table.rowCount(), len(missions))):
                    mission = missions[row]
                    if not isinstance(mission, dict):
                        continue
                    item = table.item(row, 4)
                    if item is None:
                        item = QTableWidgetItem()
                        table.setItem(row, 4, item)
                    if mission.get("coverage_enabled"):
                        try:
                            item.setText(f"{int(mission.get('coverage_percent') or 0)}%")
                        except Exception:
                            item.setText("-%")
                    else:
                        item.setText("-")
                self._update_table_row_status(table, missions, table_current_id)

        if update_ui:
            for aid in list(self._mission_header_labels.keys()):
                self._update_mission_header(aid)

        # completion recommendations are collected by the monitoring GUI (logic layer)
        self._persist_coverage_progress(snapshot)

    def build_0501_payload(self, *, timestamp_ms: int | None, source: str = "MSM") -> dict | None:
        view = self._mission_view
        if not view:
            return None

        snapshot = self._last_progress_snapshot
        if snapshot is None:
            snapshot = self._progress_tracker.update(None, None)
            self._last_progress_snapshot = snapshot
        mission_progress = snapshot.get("mission_progress") or {}

        input_missions = view.get("input_missions") or []
        current_input_id = self._resolve_current_input_id(input_missions)
        if current_input_id is not None:
            self._last_active_input_id = current_input_id

        progress_list: list[dict[str, object]] = []
        for entry in view.get("uav_entries") or []:
            aircraft_id = entry.get("aircraft_id")
            if aircraft_id is None:
                continue
            missions = entry.get("missions") or []
            current_mission_id = self._resolve_current_mission_id(
                missions,
                aircraft_id,
                snapshot,
            )

            progress = 0
            if current_mission_id is not None:
                prog_entry = mission_progress.get(current_mission_id)
                if prog_entry:
                    try:
                        progress = int(prog_entry.get("progress_percent", 0))
                    except Exception:
                        progress = 0
                else:
                    for mission in missions:
                        if mission.get("individual_mission_id") == current_mission_id:
                            try:
                                progress = int(mission.get("progress_percent") or 0)
                            except Exception:
                                progress = 0
                            break

            progress_list.append(
                {
                    "aircraftID": int(aircraft_id),
                    "currentIndividualMission": {
                        "individualMissionID": int(current_mission_id or 0),
                    },
                    "currentIndividualMissionProgress": int(progress),
                }
            )

        try:
            plan_id = int(view.get("mission_plan_id") or 0)
        except Exception:
            plan_id = 0
        try:
            input_id = int(current_input_id) if current_input_id is not None else 0
        except Exception:
            input_id = 0

        ts_source = timestamp_ms
        if ts_source is None:
            ts_source = self._last_status_timestamp_ms
        ts = int(ts_source) if ts_source is not None else 0
        return {
            "timestamp": ts,
            "source": source,
            "currentMissionPlanID": plan_id,
            "currentInputMissionID": input_id,
            "individualMissionProgressStatusList": progress_list,
        }

    def get_latest_status_timestamp_ms(self) -> int | None:
        return self._last_status_timestamp_ms

    def _update_last_progress_input(self, snapshot: dict, prev_snapshot: dict | None) -> None:
        active_input_id = snapshot.get("active_input_id")
        if active_input_id is not None:
            try:
                self._last_progress_input_id = int(active_input_id)
            except Exception:
                pass
            return
        mission_progress = snapshot.get("mission_progress") or {}
        prev_progress = (prev_snapshot or {}).get("mission_progress") or {}
        best_input_id = None
        best_delta = 0

        for mission_id, cur in mission_progress.items():
            if not isinstance(cur, dict):
                continue
            input_id = cur.get("input_id")
            try:
                input_id_int = int(input_id) if input_id is not None else None
            except Exception:
                input_id_int = None
            if input_id_int is None:
                continue
            cur_actual = int(cur.get("actual_seconds") or 0)
            prev_entry = prev_progress.get(mission_id) if isinstance(prev_progress, dict) else None
            prev_actual = int(prev_entry.get("actual_seconds") or 0) if isinstance(prev_entry, dict) else 0
            cur_done = bool(cur.get("done"))
            prev_done = bool(prev_entry.get("done")) if isinstance(prev_entry, dict) else False
            delta = cur_actual - prev_actual
            if delta > best_delta:
                best_delta = delta
                best_input_id = input_id_int
            elif best_input_id is None and cur_done and not prev_done:
                best_input_id = input_id_int

        if best_input_id is None:
            in_progress = []
            for cur in mission_progress.values():
                if not isinstance(cur, dict):
                    continue
                try:
                    progress = int(cur.get("progress_percent") or 0)
                except Exception:
                    progress = 0
                if not (0 < progress < 100):
                    continue
                input_id = cur.get("input_id")
                try:
                    input_id_int = int(input_id) if input_id is not None else None
                except Exception:
                    input_id_int = None
                if input_id_int is None:
                    continue
                actual = int(cur.get("actual_seconds") or 0)
                in_progress.append((actual, input_id_int))
            if in_progress:
                in_progress.sort()
                best_input_id = in_progress[-1][1]

        if best_input_id is not None:
            self._last_progress_input_id = best_input_id

    def _apply_completion_updates(self, snapshot: dict) -> None:
        view = self._mission_view
        if not view:
            return
        input_package_id = view.get("input_mission_package_id")
        for item in snapshot.get("new_completed_waypoints") or []:
            if not isinstance(item, dict):
                continue
            path_id = item.get("path_id")
            waypoint_ids = item.get("waypoint_ids") or []
            mark_waypoints_done(path_id, waypoint_ids)
        for item in snapshot.get("new_completed_individual") or []:
            if not isinstance(item, dict):
                continue
            mission_id = item.get("mission_id")
            package_id = item.get("package_id")
            mark_individual_mission_done(package_id, mission_id)
            try:
                mission_id_int = int(mission_id) if mission_id is not None else None
            except Exception:
                mission_id_int = None
            if mission_id_int is not None:
                for entry in view.get("uav_entries") or []:
                    for mission in entry.get("missions") or []:
                        if not isinstance(mission, dict):
                            continue
                        if mission.get("individual_mission_id") == mission_id_int:
                            mission["is_done"] = True
        for input_id in snapshot.get("new_completed_input") or []:
            mark_input_mission_done(input_package_id, input_id)
            try:
                input_id_int = int(input_id) if input_id is not None else None
            except Exception:
                input_id_int = None
            if input_id_int is None:
                continue
            for item in view.get("input_missions") or []:
                if isinstance(item, dict) and item.get("input_mission_id") == input_id_int:
                    item["is_done"] = True
            for entry in view.get("uav_entries") or []:
                for mission in entry.get("missions") or []:
                    if not isinstance(mission, dict):
                        continue
                    if mission.get("input_id") == input_id_int:
                        mission["is_done"] = True
            if input_id_int not in self._pending_completion_inputs:
                self._pending_completion_inputs.append(input_id_int)

    @staticmethod
    def _dedupe_input_ids(values: list[object]) -> list[int]:
        ordered: list[int] = []
        seen: set[int] = set()
        for raw_id in values:
            try:
                input_id_int = int(raw_id)
            except Exception:
                continue
            if input_id_int in seen:
                continue
            seen.add(input_id_int)
            ordered.append(input_id_int)
        return ordered

    def _remove_pending_input(
        self,
        input_id: int,
        *,
        from_completion: bool = True,
        from_execute: bool = True,
    ) -> None:
        def _keep_value(value: object) -> bool:
            try:
                return int(value) != int(input_id)
            except Exception:
                return True

        if from_completion:
            self._pending_completion_inputs = [
                value
                for value in self._pending_completion_inputs
                if _keep_value(value)
            ]
        if from_execute:
            self._pending_execute_inputs = [
                value
                for value in self._pending_execute_inputs
                if _keep_value(value)
            ]

    def _resolve_final_completion_input_id(self, all_input_ids: list[int]) -> int | None:
        final_input_id = None
        if self._last_progress_input_id is not None:
            final_input_id = self._last_progress_input_id
        elif self._last_active_input_id is not None:
            final_input_id = self._last_active_input_id
        elif all_input_ids:
            final_input_id = all_input_ids[-1]
        if final_input_id is None:
            return None
        try:
            return int(final_input_id)
        except Exception:
            return None

    def note_completion_recommendation_sent(self, recommend: int, input_id: int | None) -> None:
        try:
            recommend_int = int(recommend)
        except Exception:
            return
        try:
            input_id_int = int(input_id) if input_id is not None else None
        except Exception:
            input_id_int = None

        if recommend_int == 3:
            self._sent_final_completion = True
            if input_id_int is not None:
                if input_id_int in self._pending_completion_inputs:
                    self._sent_0503_inputs.add(input_id_int)
                self._sent_0503_pending_inputs.discard(input_id_int)
                self._remove_pending_input(input_id_int, from_completion=True, from_execute=True)
            return

        if recommend_int != 1 or input_id_int is None:
            return

        if input_id_int in self._pending_completion_inputs:
            self._sent_0503_inputs.add(input_id_int)
            self._remove_pending_input(input_id_int, from_completion=True, from_execute=True)
        else:
            self._sent_0503_pending_inputs.add(input_id_int)
            self._remove_pending_input(input_id_int, from_completion=False, from_execute=True)

    def note_completion_recommendation_failed(self, recommend: int, input_id: int | None) -> None:
        try:
            recommend_int = int(recommend)
        except Exception:
            return
        if recommend_int == 3:
            self._sent_final_completion = False
        try:
            input_id_int = int(input_id) if input_id is not None else None
        except Exception:
            input_id_int = None
        if input_id_int is None:
            return
        self._sent_0503_pending_inputs.discard(input_id_int)

    def pop_completion_recommendations(self) -> list[tuple[int, int]]:
        view = self._mission_view
        if not view:
            return []
        input_missions = view.get("input_missions") or []
        if not input_missions:
            return []

        remaining_ids: list[int] = []
        all_input_ids: list[int] = []
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            input_id = item.get("input_mission_id")
            try:
                input_id_int = int(input_id) if input_id is not None else None
            except Exception:
                input_id_int = None
            if input_id_int is None:
                continue
            all_input_ids.append(input_id_int)
            try:
                progress = int(item.get("progress_percent") or 0)
            except Exception:
                progress = 0
            if (not item.get("is_done")) and progress < 100:
                remaining_ids.append(input_id_int)

        remaining = bool(remaining_ids)
        if remaining:
            self._sent_final_completion = False

        recommendations: list[tuple[int, int]] = []
        emitted: set[tuple[int, int]] = set()
        final_completion_emitted = bool(self._sent_final_completion)

        if self._pending_execute_inputs:
            queued = self._dedupe_input_ids(self._pending_execute_inputs)

            if (not remaining) and not final_completion_emitted:
                final_input_id = self._resolve_final_completion_input_id(all_input_ids)
                if final_input_id is not None:
                    recommendations.append((3, final_input_id))
                    emitted.add((3, final_input_id))
                    final_completion_emitted = True
            else:
                for input_id_int in queued:
                    if input_id_int in self._sent_0503_pending_inputs:
                        continue
                    if input_id_int in self._forced_completion_inputs:
                        continue
                    input_done = False
                    for item in input_missions:
                        if not isinstance(item, dict):
                            continue
                        if item.get("input_mission_id") == input_id_int:
                            input_done = bool(item.get("is_done"))
                            break
                    if input_done:
                        continue
                    if (1, input_id_int) in emitted:
                        continue
                    recommendations.append((1, input_id_int))
                    emitted.add((1, input_id_int))

        if self._pending_completion_inputs:
            queued = self._dedupe_input_ids(self._pending_completion_inputs)

            for input_id_int in queued:
                if input_id_int in self._sent_0503_inputs:
                    continue
                if input_id_int in self._forced_completion_inputs:
                    continue
                recommend = 3 if not remaining else 1
                if recommend == 3 and final_completion_emitted:
                    continue
                if recommend == 1 and input_id_int in self._sent_0503_pending_inputs:
                    continue
                if (recommend, input_id_int) in emitted:
                    continue
                recommendations.append((recommend, input_id_int))
                emitted.add((recommend, input_id_int))
                if recommend == 3:
                    final_completion_emitted = True

        if not remaining and not final_completion_emitted:
            final_input_id = self._resolve_final_completion_input_id(all_input_ids)
            if final_input_id is not None:
                recommendations.append((3, final_input_id))
                final_completion_emitted = True

        return recommendations

    def _queue_pending_execute_inputs(self, input_missions: list[dict]) -> None:
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            input_id = item.get("input_mission_id")
            try:
                input_id_int = int(input_id) if input_id is not None else None
            except Exception:
                input_id_int = None
            if input_id_int is None:
                continue
            if item.get("is_done"):
                continue
            try:
                progress = int(item.get("progress_percent") or 0)
            except Exception:
                progress = 0
            if progress < 100:
                continue
            if input_id_int in self._sent_0503_pending_inputs:
                continue
            if input_id_int in self._pending_execute_inputs:
                continue
            self._pending_execute_inputs.append(input_id_int)

    def set_ui_updates_enabled(self, enabled: bool) -> None:
        self._ui_updates_enabled = bool(enabled)
        if self._ui_updates_enabled and self._last_progress_snapshot is not None:
            self._apply_progress_snapshot(self._last_progress_snapshot, update_ui=True)

    def _update_table_row_status(
        self,
        table: QTableWidget,
        missions: list[dict],
        current_id: int | None,
    ) -> None:
        red_bg = QColor("#fca5a5")
        green_bg = QColor("#86efac")
        default_brush = QBrush()
        default_fg = QBrush()
        for row in range(table.rowCount()):
            mission = missions[row] if row < len(missions) else None
            mission_id = mission.get("individual_mission_id") if isinstance(mission, dict) else None
            is_done = bool(mission.get("is_done")) if isinstance(mission, dict) else False
            is_current = (
                current_id is not None
                and mission_id is not None
                and int(mission_id) == int(current_id)
            )
            for col in range(table.columnCount()):
                item = table.item(row, col)
                if item is None:
                    continue
                if is_done:
                    item.setBackground(red_bg)
                    item.setForeground(QColor("#111827"))
                elif is_current:
                    item.setBackground(green_bg)
                    item.setForeground(QColor("#111827"))
                else:
                    item.setBackground(default_brush)
                    item.setForeground(default_fg)

    @staticmethod
    def _next_pending_id(items: list[dict], id_key: str) -> int | None:
        for item in items:
            if not isinstance(item, dict):
                continue
            if item.get("skip_pending"):
                continue
            if item.get("is_done"):
                continue
            value = item.get(id_key)
            if value is not None:
                try:
                    return int(value)
                except Exception:
                    return None
        for item in reversed(items):
            if not isinstance(item, dict):
                continue
            if item.get("skip_pending"):
                continue
            value = item.get(id_key)
            if value is not None:
                try:
                    return int(value)
                except Exception:
                    return None
        return None

    @staticmethod
    def _snapshot_current_mission_map(snapshot: dict | None) -> dict[int, int]:
        raw = (snapshot or {}).get("aircraft_current_mission") if isinstance(snapshot, dict) else {}
        if not isinstance(raw, dict):
            return {}
        result: dict[int, int] = {}
        for raw_aircraft_id, raw_mission_id in raw.items():
            try:
                aircraft_id = int(raw_aircraft_id)
                mission_id = int(raw_mission_id)
            except Exception:
                continue
            result[aircraft_id] = mission_id
        return result

    def _resolve_current_mission_id(
        self,
        missions: list[dict],
        aircraft_id: int | None,
        snapshot: dict | None = None,
    ) -> int | None:
        current_map = self._snapshot_current_mission_map(snapshot)
        try:
            aircraft_id_int = int(aircraft_id) if aircraft_id is not None else None
        except Exception:
            aircraft_id_int = None
        if aircraft_id_int is not None:
            mission_id = current_map.get(aircraft_id_int)
            if mission_id is not None:
                return int(mission_id)
        current_id = self._next_pending_id(missions, "individual_mission_id")
        if current_id is not None:
            return current_id
        if missions:
            try:
                return int(missions[-1].get("individual_mission_id"))
            except Exception:
                return None
        return None

    def _resolve_current_input_id(self, input_missions: list[dict]) -> int | None:
        if self._last_progress_input_id is not None:
            return self._last_progress_input_id
        current_id = self._next_pending_id(input_missions, "input_mission_id")
        if current_id is not None:
            return current_id
        if self._last_active_input_id is not None:
            return self._last_active_input_id
        if not input_missions:
            return None
        last_item = input_missions[-1]
        if isinstance(last_item, dict) and last_item.get("input_mission_id") is not None:
            try:
                return int(last_item.get("input_mission_id"))
            except Exception:
                return None
        return None

    @staticmethod
    def _first_pending_id(items: list[dict], id_key: str) -> int | None:
        for item in items:
            if not isinstance(item, dict):
                continue
            if item.get("skip_pending"):
                continue
            if item.get("is_done"):
                continue
            value = item.get(id_key)
            if value is not None:
                try:
                    return int(value)
                except Exception:
                    return None
        return None
