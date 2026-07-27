from __future__ import annotations

import copy
import sys
import time
from pathlib import Path
from typing import Any

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

try:
    from modules.common import replan_perf
except Exception:
    _COMMON_DIR = next(
        (
            parent / "common"
            for parent in Path(__file__).resolve().parents
            if (parent / "common" / "replan_perf.py").exists()
        ),
        None,
    )
    if _COMMON_DIR is not None and str(_COMMON_DIR) not in sys.path:
        sys.path.insert(0, str(_COMMON_DIR))
    import replan_perf  # type: ignore


def _fmt_ms(value: object | None) -> str:
    try:
        ms = int(value) if value is not None else 0
    except Exception:
        return "-"
    if ms < 1000:
        return f"{ms} ms"
    seconds = ms / 1000.0
    if seconds < 60.0:
        return f"{seconds:.1f} s"
    minutes = int(seconds // 60)
    remain = int(seconds % 60)
    return f"{minutes}m {remain:02d}s"


def _fmt_plan_ids(values: list[int] | None) -> str:
    if not values:
        return "-"
    return ", ".join(str(int(v)) for v in values[:4]) + (" ..." if len(values) > 4 else "")


class _Badge(QLabel):
    def __init__(self, text: str = "", tone: str = "muted", parent: QWidget | None = None) -> None:
        super().__init__(text, parent)
        self.setObjectName("replanQueueBadge")
        self.setAlignment(Qt.AlignCenter)
        self.setProperty("tone", tone)
        self.setMinimumHeight(24)

    def set_badge(self, text: str, *, tone: str) -> None:
        self.setText(str(text))
        self.setProperty("tone", str(tone))
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()


class _MetricCard(QFrame):
    def __init__(self, title: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("replanQueueMetric")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(4)
        self._title = QLabel(title)
        self._title.setObjectName("replanQueueMetricTitle")
        self._value = QLabel("-")
        self._value.setObjectName("replanQueueMetricValue")
        layout.addWidget(self._title)
        layout.addWidget(self._value)

    def set_value(self, value: str) -> None:
        self._value.setText(str(value))


class _ReplanCard(QFrame):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("replanQueueItem")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(8)

        top = QHBoxLayout()
        top.setSpacing(8)
        self._source = _Badge("-", tone="info")
        self._stage = _Badge("-", tone="muted")
        self._order = QLabel("#-")
        self._order.setObjectName("replanQueueOrder")
        top.addWidget(self._source, 0)
        top.addWidget(self._stage, 0)
        top.addStretch(1)
        top.addWidget(self._order, 0)

        self._reason = QLabel("-")
        self._reason.setObjectName("replanQueueReason")
        self._reason.setWordWrap(True)

        self._meta = QLabel("-")
        self._meta.setObjectName("replanQueueMeta")
        self._meta.setWordWrap(True)

        self._time = QLabel("-")
        self._time.setObjectName("replanQueueTime")

        self._busy = QProgressBar()
        self._busy.setTextVisible(False)
        self._busy.setRange(0, 0)
        self._busy.setFixedHeight(8)

        layout.addLayout(top)
        layout.addWidget(self._reason)
        layout.addWidget(self._meta)
        layout.addWidget(self._time)
        layout.addWidget(self._busy)

    def set_item(self, item: dict[str, Any], *, position: int, active: bool) -> None:
        status = str(item.get("status") or "")
        tone = "info"
        if status == "completed":
            tone = "success"
        elif status in {"timed_out", "dispatch_failed"}:
            tone = "warn"
        elif status == "queued":
            tone = "muted"
        self._source.set_badge(str(item.get("source_label") or "-"), tone="info")
        self._stage.set_badge(str(item.get("stage_label") or "-"), tone=tone)
        self._order.setText(f"#{position}")
        self._reason.setText(str(item.get("reason") or "-"))

        meta_bits: list[str] = []
        target_type_label = str(item.get("target_type_label") or "-")
        target_id = item.get("target_id")
        if target_id:
            meta_bits.append(f"표적 {target_type_label} / ID {target_id}")
        aircraft_id = item.get("aircraft_id")
        if aircraft_id:
            meta_bits.append(f"항공기 {aircraft_id}")
        watcher_id = item.get("watcher_id")
        if watcher_id:
            meta_bits.append(f"watcher {watcher_id}")
        prior_mission_id = item.get("prior_mission_id")
        if prior_mission_id:
            meta_bits.append(f"prior {prior_mission_id}")
        plan_ids = item.get("plan_ids") or []
        if plan_ids:
            meta_bits.append(f"plan { _fmt_plan_ids(plan_ids) }")
        duplicates = int(item.get("duplicates") or 0)
        if duplicates > 0:
            meta_bits.append(f"중복 억제 {duplicates}회")
        self._meta.setText("  |  ".join(meta_bits) if meta_bits else "추가 메타데이터 없음")

        time_bits: list[str] = [f"생성 {_fmt_ms(item.get('age_ms'))}"]
        active_ms = item.get("active_ms")
        if active_ms is not None:
            time_bits.append(f"진행 {_fmt_ms(active_ms)}")
        completion_signal = item.get("completion_signal")
        if completion_signal:
            time_bits.append(f"완료신호 {completion_signal}")
        elif item.get("last_signal"):
            time_bits.append(f"최근신호 {item.get('last_signal')}")
        self._time.setText("  ·  ".join(time_bits))

        self._busy.setVisible(bool(active))
        if not active:
            self._busy.setRange(0, 1)
            self._busy.setValue(1)
        else:
            self._busy.setRange(0, 0)


class ReplanQueueTab(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._snapshot: dict[str, Any] = {}
        self._ui_updates_enabled = True
        self._dirty = False
        self._refresh_scheduled = False
        self._last_refresh_monotonic = 0.0
        self._min_refresh_interval_ms = 250
        self._perf: dict[str, float | int] = {
            "snapshot_copy_count": 0,
            "snapshot_copy_total_ms": 0.0,
            "snapshot_copy_max_ms": 0.0,
            "refresh_count": 0,
            "refresh_total_ms": 0.0,
            "refresh_max_ms": 0.0,
        }
        self._active_card: _ReplanCard | None = None
        self._queue_container: QVBoxLayout | None = None
        self._history_container: QVBoxLayout | None = None
        self._priority_label: QLabel | None = None
        self._empty_queue_label: QLabel | None = None
        self._empty_history_label: QLabel | None = None
        self._metric_active: _MetricCard | None = None
        self._metric_queue: _MetricCard | None = None
        self._metric_history: _MetricCard | None = None
        self._metric_timeout: _MetricCard | None = None
        self._last_sync_label: QLabel | None = None
        self._build_ui()
        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(350)
        self._refresh_timer.timeout.connect(self._refresh_view)
        self._refresh_timer.start()

    def set_log_callback(self, callback) -> None:
        _ = callback

    def set_ui_updates_enabled(self, enabled: bool) -> None:
        self._ui_updates_enabled = bool(enabled)
        if hasattr(self, "_refresh_timer"):
            if self._ui_updates_enabled:
                if not self._refresh_timer.isActive():
                    self._refresh_timer.start()
            elif self._refresh_timer.isActive():
                self._refresh_timer.stop()
        if self._ui_updates_enabled and self._dirty:
            self._schedule_refresh_view()

    def set_snapshot(self, snapshot: dict[str, Any] | None) -> None:
        if self._ui_updates_enabled:
            started_at = time.perf_counter()
            self._snapshot = copy.deepcopy(snapshot or {})
            self._record_perf("snapshot_copy", started_at)
        else:
            self._snapshot = dict(snapshot or {})
        self._dirty = True
        if self._ui_updates_enabled:
            self._schedule_refresh_view()

    def performance_metrics(self) -> dict[str, float | int]:
        metrics = dict(self._perf)
        copy_count = int(metrics.get("snapshot_copy_count") or 0)
        refresh_count = int(metrics.get("refresh_count") or 0)
        copy_total = float(metrics.get("snapshot_copy_total_ms") or 0.0)
        refresh_total = float(metrics.get("refresh_total_ms") or 0.0)
        metrics["snapshot_copy_avg_ms"] = round(copy_total / copy_count, 3) if copy_count else 0.0
        metrics["refresh_avg_ms"] = round(refresh_total / refresh_count, 3) if refresh_count else 0.0
        return metrics

    def _record_perf(self, prefix: str, started_at: float) -> float:
        elapsed_ms = max(0.0, (time.perf_counter() - float(started_at)) * 1000.0)
        count_key = f"{prefix}_count"
        total_key = f"{prefix}_total_ms"
        max_key = f"{prefix}_max_ms"
        self._perf[count_key] = int(self._perf.get(count_key, 0) or 0) + 1
        self._perf[total_key] = float(self._perf.get(total_key, 0.0) or 0.0) + elapsed_ms
        self._perf[max_key] = max(float(self._perf.get(max_key, 0.0) or 0.0), elapsed_ms)
        replan_perf.add(f"monitoring.replan_queue_tab.{prefix}", elapsed_ms=elapsed_ms)
        return elapsed_ms

    def _schedule_refresh_view(self) -> None:
        if not self._ui_updates_enabled:
            return
        if self._refresh_scheduled:
            return
        now = time.perf_counter()
        elapsed_ms = (now - float(self._last_refresh_monotonic or 0.0)) * 1000.0
        delay_ms = max(0, int(self._min_refresh_interval_ms - elapsed_ms))
        if delay_ms <= 0:
            self._refresh_view()
            return
        self._refresh_scheduled = True
        QTimer.singleShot(delay_ms, self._run_scheduled_refresh_view)

    def _run_scheduled_refresh_view(self) -> None:
        self._refresh_scheduled = False
        self._refresh_view()

    def _build_ui(self) -> None:
        self.setObjectName("replanQueueTab")
        self.setStyleSheet(
            """
            QWidget#replanQueueTab { background: #eef4fb; color: #10203b; }
            QFrame#replanQueueHero { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #f8fbff, stop:1 #ebf3ff); border: 1px solid #d3deed; border-radius: 20px; }
            QFrame#replanQueueSection { background: #ffffff; border: 1px solid #d8e3ef; border-radius: 18px; }
            QFrame#replanQueueMetric { background: #f8fbff; border: 1px solid #d7e3f2; border-radius: 16px; }
            QFrame#replanQueueItem { background: #fbfdff; border: 1px solid #dbe5f1; border-left: 4px solid #6aa4ff; border-radius: 16px; }
            QLabel#replanQueueTitle { font-size: 24px; font-weight: 700; color: #10203b; }
            QLabel#replanQueueSubtitle { color: #5f7087; font-size: 12px; }
            QLabel#replanQueueMetricTitle { color: #607086; font-size: 11px; }
            QLabel#replanQueueMetricValue { color: #10203b; font-size: 22px; font-weight: 700; }
            QLabel#replanQueueSectionTitle { color: #10203b; font-size: 16px; font-weight: 700; }
            QLabel#replanQueueSectionNote { color: #62748b; font-size: 11px; }
            QLabel#replanQueueReason { color: #10203b; font-size: 15px; font-weight: 700; }
            QLabel#replanQueueMeta { color: #516277; font-size: 11px; }
            QLabel#replanQueueTime { color: #6b7c92; font-size: 11px; }
            QLabel#replanQueueOrder { color: #63748a; font-size: 12px; font-weight: 700; }
            QLabel#replanQueueEmpty { color: #6f829a; font-size: 12px; padding: 16px 8px; }
            QLabel#replanQueueHint { background: #edf5ff; color: #2a538e; border: 1px solid #d2e2fa; border-radius: 12px; padding: 8px 10px; }
            QLabel#replanQueueBadge { border-radius: 12px; padding: 4px 10px; font-size: 11px; font-weight: 700; }
            QLabel#replanQueueBadge[tone="muted"] { background: #ecf1f7; color: #53657b; }
            QLabel#replanQueueBadge[tone="info"] { background: #dfeeff; color: #245da8; }
            QLabel#replanQueueBadge[tone="success"] { background: #e3f5e7; color: #1c7b43; }
            QLabel#replanQueueBadge[tone="warn"] { background: #fff0df; color: #b15b04; }
            QProgressBar { background: #e4edf8; border: none; border-radius: 4px; }
            QProgressBar::chunk { background: #2f6df1; border-radius: 4px; }
            QScrollArea { background: transparent; border: none; }
            """
        )

        frame = QVBoxLayout(self)
        frame.setContentsMargins(0, 0, 0, 0)
        frame.setSpacing(0)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)

        content = QWidget()
        root = QVBoxLayout(content)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(14)

        root.addWidget(self._build_header())
        root.addWidget(self._build_metrics())
        root.addWidget(self._build_active_section())
        root.addWidget(self._build_queue_section())
        root.addWidget(self._build_history_section())
        root.addStretch(1)

        scroll.setWidget(content)
        frame.addWidget(scroll)

    def _build_header(self) -> QFrame:
        card = QFrame()
        card.setObjectName("replanQueueHero")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(8)
        title = QLabel("재계획 Queue")
        title.setObjectName("replanQueueTitle")
        subtitle = QLabel(
            "한 번에 하나의 재계획만 진행하고, 나머지 트리거는 queue에서 순차 대기시킵니다. "
            "0702 또는 0903, 또는 계획 실패 공지가 오면 다음 항목이 자동으로 이어집니다."
        )
        subtitle.setObjectName("replanQueueSubtitle")
        subtitle.setWordWrap(True)
        self._priority_label = QLabel("표적 우선순위: -")
        self._priority_label.setObjectName("replanQueueHint")
        self._last_sync_label = QLabel("마지막 갱신: -")
        self._last_sync_label.setObjectName("replanQueueSubtitle")
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(self._priority_label)
        layout.addWidget(self._last_sync_label)
        return card

    def _build_metrics(self) -> QWidget:
        row = QWidget()
        layout = QGridLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)
        self._metric_active = _MetricCard("현재 Active")
        self._metric_queue = _MetricCard("대기 Queue")
        self._metric_history = _MetricCard("완료 이력")
        self._metric_timeout = _MetricCard("Active Timeout")
        layout.addWidget(self._metric_active, 0, 0)
        layout.addWidget(self._metric_queue, 0, 1)
        layout.addWidget(self._metric_history, 0, 2)
        layout.addWidget(self._metric_timeout, 0, 3)
        return row

    def _build_active_section(self) -> QFrame:
        card = QFrame()
        card.setObjectName("replanQueueSection")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)
        title = QLabel("현재 진행중")
        title.setObjectName("replanQueueSectionTitle")
        note = QLabel("현재 active 항목은 의사결정 결과나 계획 반영 신호가 올 때까지 유지됩니다.")
        note.setObjectName("replanQueueSectionNote")
        note.setWordWrap(True)
        self._active_card = _ReplanCard()
        layout.addWidget(title)
        layout.addWidget(note)
        layout.addWidget(self._active_card)
        return card

    def _build_queue_section(self) -> QFrame:
        card = QFrame()
        card.setObjectName("replanQueueSection")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)
        title = QLabel("대기 Queue")
        title.setObjectName("replanQueueSectionTitle")
        note = QLabel("먼저 들어온 요청부터 처리하되, 0402 한 건 안에서는 정렬된 우선순서가 그대로 유지됩니다.")
        note.setObjectName("replanQueueSectionNote")
        note.setWordWrap(True)
        container = QWidget()
        self._queue_container = QVBoxLayout(container)
        self._queue_container.setContentsMargins(0, 0, 0, 0)
        self._queue_container.setSpacing(10)
        self._empty_queue_label = QLabel("대기중인 재계획이 없습니다.")
        self._empty_queue_label.setObjectName("replanQueueEmpty")
        self._queue_container.addWidget(self._empty_queue_label)
        layout.addWidget(title)
        layout.addWidget(note)
        layout.addWidget(container)
        return card

    def _build_history_section(self) -> QFrame:
        card = QFrame()
        card.setObjectName("replanQueueSection")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(10)
        title = QLabel("최근 완료")
        title.setObjectName("replanQueueSectionTitle")
        note = QLabel("완료, 전송실패, timeout 모두 이력으로 남겨 현재 흐름을 추적합니다.")
        note.setObjectName("replanQueueSectionNote")
        note.setWordWrap(True)
        container = QWidget()
        self._history_container = QVBoxLayout(container)
        self._history_container.setContentsMargins(0, 0, 0, 0)
        self._history_container.setSpacing(10)
        self._empty_history_label = QLabel("아직 완료된 이력이 없습니다.")
        self._empty_history_label.setObjectName("replanQueueEmpty")
        self._history_container.addWidget(self._empty_history_label)
        layout.addWidget(title)
        layout.addWidget(note)
        layout.addWidget(container)
        return card

    def _refresh_view(self) -> None:
        self._refresh_scheduled = False
        if not self._ui_updates_enabled:
            return
        if not self._dirty:
            return
        started_at = time.perf_counter()
        snapshot = dict(self._snapshot or {})
        stats = snapshot.get("stats") if isinstance(snapshot.get("stats"), dict) else {}
        active = snapshot.get("active") if isinstance(snapshot.get("active"), dict) else None
        queued = [item for item in snapshot.get("queued") or [] if isinstance(item, dict)]
        history = [item for item in snapshot.get("history") or [] if isinstance(item, dict)]

        if self._metric_active is not None:
            self._metric_active.set_value("ON" if active else "OFF")
        if self._metric_queue is not None:
            self._metric_queue.set_value(str(int(stats.get("queued_count") or len(queued))))
        if self._metric_history is not None:
            self._metric_history.set_value(str(int(stats.get("history_count") or len(history))))
        if self._metric_timeout is not None:
            self._metric_timeout.set_value(_fmt_ms(stats.get("active_timeout_ms")))

        if self._priority_label is not None:
            order = [str(int(v)) for v in snapshot.get("target_priority_order") or [] if str(v).strip()]
            self._priority_label.setText(f"표적 우선순위: {' > '.join(order) if order else '-'}")
        if self._last_sync_label is not None:
            metrics = self.performance_metrics()
            self._last_sync_label.setToolTip(
                "uiCopyAvg={:.3f}ms, uiRefreshAvg={:.3f}ms, uiCopyMax={:.3f}ms, uiRefreshMax={:.3f}ms".format(
                    float(metrics.get("snapshot_copy_avg_ms") or 0.0),
                    float(metrics.get("refresh_avg_ms") or 0.0),
                    float(metrics.get("snapshot_copy_max_ms") or 0.0),
                    float(metrics.get("refresh_max_ms") or 0.0),
                )
            )
            self._last_sync_label.setText(
                f"마지막 갱신: active {('있음' if active else '없음')} / queue {len(queued)}건 / history {len(history)}건"
            )

        if self._active_card is not None:
            active_item = active or {
                "source_label": "대기중",
                "stage_label": "유휴",
                "status": "queued",
                "reason": "현재 진행중인 재계획이 없습니다.",
                "age_ms": 0,
                "plan_ids": [],
            }
            self._active_card.set_item(active_item, position=1, active=bool(active))

        self._rebuild_list(self._queue_container, self._empty_queue_label, queued, active=False)
        self._rebuild_list(self._history_container, self._empty_history_label, history, active=False)
        self._dirty = False
        self._last_refresh_monotonic = time.perf_counter()
        self._record_perf("refresh", started_at)

    def _rebuild_list(
        self,
        layout: QVBoxLayout | None,
        empty_label: QLabel | None,
        items: list[dict[str, Any]],
        *,
        active: bool,
    ) -> None:
        if layout is None:
            return
        perf_start = replan_perf.start_timer()
        removed_widgets = 0
        added_cards = 0
        while layout.count():
            child = layout.takeAt(0)
            widget = child.widget()
            if widget is not None:
                if empty_label is not None and widget is empty_label:
                    widget.setParent(None)
                    continue
                widget.deleteLater()
                removed_widgets += 1
        if not items:
            if empty_label is not None:
                layout.addWidget(empty_label)
            replan_perf.add_elapsed(
                "monitoring.replan_queue_tab.rebuild_list",
                perf_start,
                items=0,
                removed_widgets=removed_widgets,
                added_cards=0,
                empty=1,
            )
            return
        for index, item in enumerate(items, start=1):
            card = _ReplanCard()
            card.set_item(item, position=index, active=active)
            layout.addWidget(card)
            added_cards += 1
        layout.addStretch(1)
        replan_perf.add_elapsed(
            "monitoring.replan_queue_tab.rebuild_list",
            perf_start,
            items=len(items),
            removed_widgets=removed_widgets,
            added_cards=added_cards,
            empty=0,
        )


__all__ = ["ReplanQueueTab"]
