from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QWidget


_TASK_COLORS = [
    "#e53935",
    "#1e88e5",
    "#43a047",
    "#fb8c00",
    "#8e24aa",
    "#00897b",
    "#6d4c41",
    "#3949ab",
    "#c2185b",
    "#7cb342",
]


def _pick_task_color(parent_order: int) -> QColor:
    idx = max(0, int(parent_order) - 1) % len(_TASK_COLORS)
    return QColor(_TASK_COLORS[idx])


class ScheduleGanttWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._schedule: Optional[Dict[str, Any]] = None
        self.setMinimumHeight(220)

    def set_schedule(self, schedule: Optional[Dict[str, Any]]) -> None:
        self._schedule = schedule if isinstance(schedule, dict) else None
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#f8fafc"))

        pad = 14.0
        frame = QRectF(
            pad,
            pad,
            max(1.0, self.width() - (pad * 2.0)),
            max(1.0, self.height() - (pad * 2.0)),
        )
        painter.setPen(QPen(QColor("#cbd5e1"), 1))
        painter.setBrush(QColor(255, 255, 255, 240))
        painter.drawRoundedRect(frame, 8, 8)

        title_font = QFont()
        title_font.setPointSize(9)
        title_font.setBold(True)
        painter.setFont(title_font)
        painter.setPen(QPen(QColor("#0f172a"), 1))
        painter.drawText(QRectF(frame.left() + 10.0, frame.top() + 6.0, 260.0, 18.0), Qt.AlignLeft | Qt.AlignVCenter, "Scheduling Gantt")

        if not isinstance(self._schedule, dict):
            self._draw_empty(painter, frame, "Run 6) Scheduling to render timeline.")
            return

        timelines = self._schedule.get("timelines")
        if not isinstance(timelines, list) or not timelines:
            self._draw_empty(painter, frame, "No schedule result.")
            return

        max_t = max(float(t.get("totalSec", 0.0) or 0.0) for t in timelines)
        max_t = max(1.0, max_t)
        slot_rows = self._schedule.get("slotAssignments")
        if not isinstance(slot_rows, list):
            slot_rows = []

        left_w = 92.0
        right_w = 70.0
        axis_h = 24.0
        row_h = 34.0
        row_gap = 10.0

        chart_left = frame.left() + left_w
        chart_right = frame.right() - right_w
        chart_top = frame.top() + 28.0
        chart_w = max(1.0, chart_right - chart_left)

        self._draw_axis(painter, chart_left, chart_top, chart_w, max_t)

        body_top = chart_top + axis_h
        for ridx, row in enumerate(timelines):
            y = body_top + ridx * (row_h + row_gap)
            self._draw_row(painter, frame, chart_left, chart_w, y, row_h, max_t, row)
        self._draw_slot_separators(
            painter,
            chart_left=chart_left,
            body_top=body_top,
            chart_w=chart_w,
            max_t=max_t,
            row_h=row_h,
            row_gap=row_gap,
            row_count=len(timelines),
            timelines=timelines,
            slot_rows=slot_rows,
        )

    def _draw_empty(self, painter: QPainter, frame: QRectF, text: str) -> None:
        font = QFont()
        font.setPointSize(9)
        painter.setFont(font)
        painter.setPen(QPen(QColor("#475569"), 1))
        painter.drawText(frame, Qt.AlignCenter, text)

    def _draw_axis(self, painter: QPainter, x0: float, y0: float, w: float, max_t: float) -> None:
        painter.setPen(QPen(QColor("#64748b"), 1))
        painter.drawLine(QPointF(x0, y0 + 16.0), QPointF(x0 + w, y0 + 16.0))

        tick_n = 6
        font = QFont()
        font.setPointSize(7)
        painter.setFont(font)
        for i in range(tick_n + 1):
            t = i / tick_n
            x = x0 + w * t
            sec = max_t * t
            painter.setPen(QPen(QColor("#94a3b8"), 1))
            painter.drawLine(QPointF(x, y0 + 13.0), QPointF(x, y0 + 19.0))
            painter.setPen(QPen(QColor("#475569"), 1))
            painter.drawText(QRectF(x - 24.0, y0, 48.0, 12.0), Qt.AlignCenter, f"{sec:.0f}s")

    def _draw_row(
        self,
        painter: QPainter,
        frame: QRectF,
        chart_left: float,
        chart_w: float,
        y: float,
        row_h: float,
        max_t: float,
        row: Dict[str, Any],
    ) -> None:
        uav_id = int(row.get("uavID", 0) or 0)
        total_sec = float(row.get("totalSec", 0.0) or 0.0)
        task_sec = float(row.get("taskSec", 0.0) or 0.0)
        move_sec = float(row.get("moveSec", 0.0) or 0.0)
        segs = row.get("segments")
        if not isinstance(segs, list):
            segs = []

        painter.setPen(QPen(QColor("#cbd5e1"), 1))
        painter.setBrush(QColor("#f8fafc"))
        painter.drawRoundedRect(QRectF(chart_left, y, chart_w, row_h), 5, 5)

        font = QFont()
        font.setPointSize(8)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QPen(QColor("#0f172a"), 1))
        painter.drawText(
            QRectF(frame.left() + 6.0, y + 2.0, 82.0, row_h - 4.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            f"UAV{uav_id}",
        )

        for seg in segs:
            if not isinstance(seg, dict):
                continue
            s = float(seg.get("startSec", 0.0) or 0.0)
            e = float(seg.get("endSec", s) or s)
            if e < s:
                s, e = e, s
            x0 = chart_left + (s / max_t) * chart_w
            x1 = chart_left + (e / max_t) * chart_w
            bw = max(1.0, x1 - x0)
            bar = QRectF(x0, y + 6.0, bw, row_h - 12.0)

            kind = str(seg.get("kind", "task"))
            if kind == "move":
                fill = QColor("#94a3b8")
                fill.setAlpha(120)
                painter.setPen(QPen(QColor("#64748b"), 1))
                painter.setBrush(fill)
                painter.drawRect(bar)
                pen = QPen(QColor("#64748b"), 1)
                pen.setStyle(Qt.DashLine)
                painter.setPen(pen)
                painter.drawLine(
                    QPointF(bar.left(), bar.center().y()),
                    QPointF(bar.right(), bar.center().y()),
                )
            elif kind in {"prestart_hold", "sync_hold", "sync_wait"}:
                fill = QColor("#f59e0b")
                fill.setAlpha(170)
                painter.setPen(QPen(QColor("#78350f"), 1))
                painter.setBrush(fill)
                painter.drawRoundedRect(bar, 4, 4)
            else:
                c = _pick_task_color(int(seg.get("parentOrder", 0) or 0))
                fill = QColor(c)
                fill.setAlpha(185)
                painter.setPen(QPen(QColor("#111827"), 1))
                painter.setBrush(fill)
                painter.drawRoundedRect(bar, 4, 4)

            label = self._segment_label(seg)
            min_w = 18.0 if kind == "move" else 42.0
            if bw >= min_w and label:
                lf = QFont()
                lf.setPointSize(7)
                painter.setFont(lf)
                painter.setPen(QPen(QColor("#0f172a"), 1))
                painter.drawText(bar, Qt.AlignCenter, label)
            elif kind == "move" and label:
                lf = QFont()
                lf.setPointSize(7)
                painter.setFont(lf)
                painter.setPen(QPen(QColor("#0f172a"), 1))
                painter.drawText(
                    QRectF(bar.right() + 2.0, bar.top() - 1.0, 28.0, bar.height() + 2.0),
                    Qt.AlignLeft | Qt.AlignVCenter,
                    label,
                )

        info = f"T{total_sec:.0f}s  task {task_sec:.0f}s  move {move_sec:.0f}s"
        inf_font = QFont()
        inf_font.setPointSize(7)
        painter.setFont(inf_font)
        painter.setPen(QPen(QColor("#334155"), 1))
        painter.drawText(
            QRectF(frame.right() - 66.0, y + 2.0, 62.0, row_h - 4.0),
            Qt.AlignRight | Qt.AlignVCenter,
            info,
        )

    def _segment_label(self, seg: Dict[str, Any]) -> str:
        kind = str(seg.get("kind", "task"))
        if kind == "move":
            mps = seg.get("moveSpeedMps")
            try:
                mv = float(mps)
                if math.isfinite(mv) and mv > 0.0:
                    mv_q = 5.0 * round(mv / 5.0)
                    mv_q = max(30.0, min(60.0, mv_q))
                    return f"M{mv_q:.0f}"
            except Exception:
                pass
            return "M"
        if kind in {"prestart_hold", "sync_hold", "sync_wait"}:
            return "T5/P1 HOLD"
        label = str(seg.get("label", ""))
        v = seg.get("speedKmh")
        if v is None:
            return label
        try:
            vv = float(v)
        except Exception:
            return label
        if not math.isfinite(vv):
            return label
        if label:
            return f"{label} V{vv:.0f}"
        return f"V{vv:.0f}"

    def _draw_slot_separators(
        self,
        painter: QPainter,
        *,
        chart_left: float,
        body_top: float,
        chart_w: float,
        max_t: float,
        row_h: float,
        row_gap: float,
        row_count: int,
        timelines: List[Dict[str, Any]],
        slot_rows: List[Dict[str, Any]],
    ) -> None:
        if row_count <= 0 or max_t <= 0.0:
            return
        y0 = body_top - 2.0
        y1 = body_top + row_count * (row_h + row_gap) - row_gap + 2.0
        font = QFont()
        font.setPointSize(7)
        painter.setFont(font)
        # Prefer dynamic separators from actual task end times (robust after insertion/shift).
        slot_finish: Dict[int, float] = {}
        for row in timelines:
            if not isinstance(row, dict):
                continue
            segs = row.get("segments")
            if not isinstance(segs, list):
                continue
            for seg in segs:
                if not isinstance(seg, dict):
                    continue
                if str(seg.get("kind", "")) != "task":
                    continue
                idx = int(seg.get("slotIndex", 0) or 0)
                if idx <= 0:
                    continue
                end_sec = float(seg.get("endSec", 0.0) or 0.0)
                if end_sec > float(slot_finish.get(idx, 0.0)):
                    slot_finish[idx] = float(end_sec)
        if not slot_finish:
            for row in slot_rows:
                if not isinstance(row, dict):
                    continue
                idx = int(row.get("slotIndex", 0) or 0)
                if idx <= 0:
                    continue
                sec = float(row.get("slotFinishSec", 0.0) or 0.0)
                if sec > float(slot_finish.get(idx, 0.0)):
                    slot_finish[idx] = float(sec)

        for slot_idx, sec in sorted(slot_finish.items()):
            if sec <= 1e-6:
                continue
            x = chart_left + (sec / max_t) * chart_w
            pen = QPen(QColor(51, 65, 85, 160), 1)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(QPointF(x, y0), QPointF(x, y1))
            painter.setPen(QPen(QColor("#1e293b"), 1))
            painter.drawText(QRectF(x + 2.0, y0 - 11.0, 36.0, 10.0), Qt.AlignLeft | Qt.AlignVCenter, f"S{int(slot_idx)}")
