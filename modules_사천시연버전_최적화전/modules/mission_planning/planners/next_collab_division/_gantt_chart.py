"""GanttChartWidget -- ETA/Gantt chart display."""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QSizePolicy, QWidget

from ._geo_utils import _uav_color, _qcolor


class GanttChartWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._paths: List[Dict[str, Any]] = []
        self._visible_uav_ids: List[int] = []
        self.setObjectName("GanttChart")
        self.setMinimumHeight(170)
        self.setMaximumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_paths(self, rows: Sequence[Dict[str, Any]], visible_uav_ids: Sequence[int]) -> None:
        self._paths = [dict(row) for row in rows if isinstance(row, dict)]
        self._visible_uav_ids = [int(aid) for aid in visible_uav_ids]
        self.update()

    def _timeline_rows(self, row: Dict[str, Any]) -> List[Dict[str, Any]]:
        timeline_rows = row.get("timelineRows")
        out: List[Dict[str, Any]] = []
        if isinstance(timeline_rows, list):
            for item in timeline_rows:
                if not isinstance(item, dict):
                    continue
                try:
                    eta_val = float(item.get("etaSec", 0.0) or 0.0)
                except Exception:
                    continue
                out.append(
                    {
                        "label": str(item.get("label", "") or ""),
                        "kind": str(item.get("kind", "") or ""),
                        "etaSec": eta_val,
                    }
                )
        out.sort(key=lambda item: float(item.get("etaSec", 0.0) or 0.0))
        return out

    def _estimated_total_sec(self, row: Dict[str, Any]) -> float:
        try:
            total_sec = float(row.get("estimatedTotalSec", 0.0) or 0.0)
        except Exception:
            total_sec = 0.0
        if total_sec > 0.0:
            return total_sec
        timeline_rows = self._timeline_rows(row)
        if timeline_rows:
            return float(max(item.get("etaSec", 0.0) or 0.0 for item in timeline_rows))
        return 0.0

    def _phase_rows(self, row: Dict[str, Any]) -> List[Dict[str, Any]]:
        phase_rows = row.get("phaseRows")
        out: List[Dict[str, Any]] = []
        if isinstance(phase_rows, list):
            for item in phase_rows:
                if not isinstance(item, dict):
                    continue
                try:
                    start_sec = float(item.get("startSec", 0.0) or 0.0)
                    end_sec = float(item.get("endSec", 0.0) or 0.0)
                except Exception:
                    continue
                if end_sec <= start_sec + 1e-6:
                    continue
                out.append(
                    {
                        "label": str(item.get("label", "") or ""),
                        "kind": str(item.get("kind", "") or ""),
                        "startSec": start_sec,
                        "endSec": end_sec,
                    }
                )
        if out:
            return out
        total_sec = self._estimated_total_sec(row)
        if total_sec > 0.0:
            return [{"label": "Route", "kind": "route", "startSec": 0.0, "endSec": total_sec}]
        return []

    def _aggregated_rows(self) -> List[Dict[str, Any]]:
        visible_order = {int(aid): idx for idx, aid in enumerate(self._visible_uav_ids)}
        grouped: Dict[int, Dict[str, Any]] = {}

        for row in self._paths:
            if not isinstance(row, dict):
                continue
            aid = int(row.get("aircraftID", 0) or 0)
            if aid not in visible_order:
                continue

            bucket = grouped.get(aid)
            if bucket is None:
                bucket = {
                    "aircraftID": int(aid),
                    "targetLabel": "",
                    "targetLabels": [],
                    "timelineRows": [],
                    "phaseRows": [],
                    "estimatedTotalSec": 0.0,
                }
                grouped[aid] = bucket

            offset_sec = float(bucket.get("estimatedTotalSec", 0.0) or 0.0)
            row_total_sec = self._estimated_total_sec(row)
            target_label = str(row.get("targetLabel", "") or "").strip()
            if target_label:
                labels = bucket.get("targetLabels")
                if isinstance(labels, list) and target_label not in labels:
                    labels.append(target_label)

            for phase in self._phase_rows(row):
                bucket["phaseRows"].append(
                    {
                        "label": str(phase.get("label", "") or ""),
                        "kind": str(phase.get("kind", "") or ""),
                        "startSec": float(phase.get("startSec", 0.0) or 0.0) + offset_sec,
                        "endSec": float(phase.get("endSec", 0.0) or 0.0) + offset_sec,
                    }
                )

            for marker in self._timeline_rows(row):
                bucket["timelineRows"].append(
                    {
                        "label": str(marker.get("label", "") or ""),
                        "kind": str(marker.get("kind", "") or ""),
                        "etaSec": float(marker.get("etaSec", 0.0) or 0.0) + offset_sec,
                    }
                )

            bucket["estimatedTotalSec"] = offset_sec + row_total_sec

        out: List[Dict[str, Any]] = []
        for aid, bucket in grouped.items():
            labels = bucket.get("targetLabels")
            if isinstance(labels, list):
                if len(labels) == 1:
                    bucket["targetLabel"] = str(labels[0])
                elif len(labels) > 1:
                    bucket["targetLabel"] = f"{len(labels)} missions"
            timeline_rows = bucket.get("timelineRows")
            if isinstance(timeline_rows, list):
                timeline_rows.sort(key=lambda item: float(item.get("etaSec", 0.0) or 0.0))
            phase_rows = bucket.get("phaseRows")
            if isinstance(phase_rows, list):
                phase_rows.sort(key=lambda item: float(item.get("startSec", 0.0) or 0.0))
            out.append(bucket)

        out.sort(
            key=lambda item: (
                visible_order.get(int(item.get("aircraftID", 0) or 0), 999),
                int(item.get("aircraftID", 0) or 0),
            )
        )
        return out

    def _grid_step_sec(self, max_sec: float) -> float:
        if max_sec <= 30.0:
            return 5.0
        if max_sec <= 60.0:
            return 10.0
        if max_sec <= 120.0:
            return 20.0
        if max_sec <= 240.0:
            return 30.0
        return 60.0

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#eef3f9"))

        panel = QRectF(8.0, 8.0, float(self.width()) - 16.0, float(self.height()) - 16.0)
        painter.setPen(QPen(QColor("#d3dde8"), 1.0))
        painter.setBrush(QColor("#ffffff"))
        painter.drawRoundedRect(panel, 12.0, 12.0)

        title_rect = QRectF(panel.left() + 14.0, panel.top() + 8.0, panel.width() - 28.0, 18.0)
        painter.setPen(QColor("#0f172a"))
        painter.drawText(title_rect, Qt.AlignLeft | Qt.AlignVCenter, "WP ETA / Gantt")

        rows = self._aggregated_rows()
        max_total_sec = max((self._estimated_total_sec(row) for row in rows), default=0.0)

        if not rows or max_total_sec <= 0.0:
            empty_rect = QRectF(panel.left() + 14.0, panel.top() + 30.0, panel.width() - 28.0, panel.height() - 40.0)
            painter.setPen(QColor("#64748b"))
            painter.drawText(empty_rect, Qt.AlignCenter, "경로 ETA 데이터가 아직 없습니다.")
            return

        left_label_w = 90.0
        right_pad = 34.0
        top_axis_h = 22.0
        chart_rect = QRectF(
            panel.left() + 14.0 + left_label_w,
            panel.top() + 30.0 + top_axis_h,
            panel.width() - 28.0 - left_label_w - right_pad,
            panel.height() - 46.0 - top_axis_h,
        )
        label_rect = QRectF(panel.left() + 14.0, chart_rect.top(), left_label_w - 8.0, chart_rect.height())

        grid_step_sec = self._grid_step_sec(max_total_sec)
        tick_sec = 0.0
        while tick_sec <= max_total_sec + 1e-6:
            x = chart_rect.left() + (chart_rect.width() * (tick_sec / max_total_sec))
            painter.setPen(QPen(QColor("#dbe4ef"), 1.0))
            painter.drawLine(QPointF(x, chart_rect.top()), QPointF(x, chart_rect.bottom()))
            tick_rect = QRectF(x - 18.0, panel.top() + 30.0, 36.0, 14.0)
            painter.setPen(QColor("#475569"))
            painter.drawText(tick_rect, Qt.AlignCenter, f"{int(tick_sec)}s")
            tick_sec += grid_step_sec

        row_h = chart_rect.height() / float(max(1, len(rows)))
        for idx, row in enumerate(rows):
            aid = int(row.get("aircraftID", 0) or 0)
            target_label = str(row.get("targetLabel", "") or "")
            color = _uav_color(aid, 230)
            y0 = chart_rect.top() + (float(idx) * row_h)
            y1 = y0 + row_h
            bar_rect = QRectF(chart_rect.left(), y0 + 7.0, chart_rect.width(), max(12.0, row_h - 14.0))

            painter.setPen(QColor("#0f172a"))
            painter.drawText(
                QRectF(label_rect.left(), y0, label_rect.width(), row_h),
                Qt.AlignLeft | Qt.AlignVCenter,
                f"UAV{aid}" + (f" / {target_label}" if target_label else ""),
            )

            painter.setPen(QPen(QColor("#cbd5e1"), 1.0))
            painter.setBrush(QColor("#f8fafc"))
            painter.drawRoundedRect(bar_rect, 6.0, 6.0)

            for phase in self._phase_rows(row):
                start_sec = float(phase.get("startSec", 0.0) or 0.0)
                end_sec = float(phase.get("endSec", 0.0) or 0.0)
                x0 = chart_rect.left() + (chart_rect.width() * (start_sec / max_total_sec))
                x1 = chart_rect.left() + (chart_rect.width() * (end_sec / max_total_sec))
                phase_rect = QRectF(x0, bar_rect.top(), max(2.0, x1 - x0), bar_rect.height())
                phase_kind = str(phase.get("kind", "") or "")
                if phase_kind == "turn":
                    fill = _uav_color(aid, 170)
                elif phase_kind == "setup":
                    fill = QColor("#f59e0b")
                elif phase_kind == "waypoint":
                    fill = _uav_color(aid, 145)
                elif phase_kind in {"ingress", "straight"}:
                    fill = _uav_color(aid, 105)
                else:
                    fill = _uav_color(aid, 130)
                painter.setPen(Qt.NoPen)
                painter.setBrush(fill)
                painter.drawRoundedRect(phase_rect, 6.0, 6.0)

            for marker in self._timeline_rows(row):
                eta_sec = float(marker.get("etaSec", 0.0) or 0.0)
                x = chart_rect.left() + (chart_rect.width() * (eta_sec / max_total_sec))
                painter.setPen(QPen(color, 1.2))
                painter.drawLine(QPointF(x, bar_rect.top() - 4.0), QPointF(x, bar_rect.bottom() + 4.0))
                label = str(marker.get("label", "") or "")
                if label:
                    marker_rect = QRectF(x - 18.0, y0 - 1.0, 36.0, 14.0)
                    painter.setPen(QColor("#334155"))
                    painter.drawText(marker_rect, Qt.AlignCenter, label)

            total_sec = self._estimated_total_sec(row)
            total_rect = QRectF(chart_rect.right() + 6.0, y0, right_pad, row_h)
            painter.setPen(QColor("#334155"))
            painter.drawText(total_rect, Qt.AlignLeft | Qt.AlignVCenter, f"{total_sec:.1f}s")

