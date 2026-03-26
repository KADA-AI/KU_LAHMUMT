# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Callable

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from modules.monitoring.logic.mission_progress import MissionProgressTracker
from modules.monitoring.logic.mission_schedule import (
    build_aircraft_schedule_view,
    format_duration,
)
from modules.monitoring.logic.mission_update import build_uav_mission_view, format_timestamp_ms


class MissionScheduleTab(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._mission_view: dict[str, Any] | None = None
        self._last_progress_snapshot: dict[str, Any] | None = None
        self._progress_tracker = MissionProgressTracker()
        self._selected_aircraft_id = 4
        self._log_callback: Callable[[str], None] | None = None
        self._path_trigger_toggle_callback: Callable[[bool], None] | None = None
        self._schedule_trigger_toggle_callback: Callable[[bool], None] | None = None
        self._next_collab_trigger_toggle_callback: Callable[[bool], None] | None = None
        self._fuel_threshold_toggle_callback: Callable[[bool], None] | None = None
        self._path_trigger_enabled = True
        self._schedule_trigger_enabled = False
        self._next_collab_trigger_enabled = False
        self._fuel_threshold_enabled = False
        self._aircraft_buttons: dict[int, QPushButton] = {}
        self._path_summary_labels: dict[str, QLabel] = {}
        self._imaging_summary_labels: dict[str, QLabel] = {}
        self._plan_summary_label: QLabel | None = None
        self._path_trigger_state_label: QLabel | None = None
        self._path_trigger_toggle_button: QPushButton | None = None
        self._schedule_trigger_state_label: QLabel | None = None
        self._schedule_trigger_toggle_button: QPushButton | None = None
        self._next_collab_trigger_state_label: QLabel | None = None
        self._next_collab_trigger_toggle_button: QPushButton | None = None
        self._fuel_threshold_state_label: QLabel | None = None
        self._fuel_threshold_toggle_button: QPushButton | None = None
        self._path_table: QTableWidget | None = None
        self._imaging_table: QTableWidget | None = None
        self._build_ui()

    def set_log_callback(self, callback: Callable[[str], None] | None) -> None:
        self._log_callback = callback

    def set_path_trigger_toggle_callback(self, callback: Callable[[bool], None] | None) -> None:
        self._path_trigger_toggle_callback = callback

    def set_path_trigger_enabled(self, enabled: bool, *, emit: bool = False) -> None:
        self._path_trigger_enabled = bool(enabled)
        self._refresh_trigger_controls()
        if emit and self._path_trigger_toggle_callback is not None:
            try:
                self._path_trigger_toggle_callback(self._path_trigger_enabled)
            except Exception:
                pass

    def set_schedule_trigger_toggle_callback(self, callback: Callable[[bool], None] | None) -> None:
        self._schedule_trigger_toggle_callback = callback

    def set_schedule_trigger_enabled(self, enabled: bool, *, emit: bool = False) -> None:
        self._schedule_trigger_enabled = bool(enabled)
        self._refresh_trigger_controls()
        if emit and self._schedule_trigger_toggle_callback is not None:
            try:
                self._schedule_trigger_toggle_callback(self._schedule_trigger_enabled)
            except Exception:
                pass

    def set_next_collab_trigger_toggle_callback(self, callback: Callable[[bool], None] | None) -> None:
        self._next_collab_trigger_toggle_callback = callback

    def set_next_collab_trigger_enabled(self, enabled: bool, *, emit: bool = False) -> None:
        self._next_collab_trigger_enabled = bool(enabled)
        self._refresh_trigger_controls()
        if emit and self._next_collab_trigger_toggle_callback is not None:
            try:
                self._next_collab_trigger_toggle_callback(self._next_collab_trigger_enabled)
            except Exception:
                pass

    def set_fuel_threshold_toggle_callback(self, callback: Callable[[bool], None] | None) -> None:
        self._fuel_threshold_toggle_callback = callback

    def set_fuel_threshold_enabled(self, enabled: bool, *, emit: bool = False) -> None:
        self._fuel_threshold_enabled = bool(enabled)
        self._refresh_trigger_controls()
        if emit and self._fuel_threshold_toggle_callback is not None:
            try:
                self._fuel_threshold_toggle_callback(self._fuel_threshold_enabled)
            except Exception:
                pass

    def set_imaging_trigger_toggle_callback(self, callback: Callable[[bool], None] | None) -> None:
        self.set_schedule_trigger_toggle_callback(callback)

    def set_imaging_trigger_enabled(self, enabled: bool, *, emit: bool = False) -> None:
        self.set_schedule_trigger_enabled(enabled, emit=emit)

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
        if not self._mission_view:
            return
        snapshot = self._progress_tracker.update(timestamp_ms, agent_states)
        active_input_id = self._progress_tracker.get_active_input_id()
        if active_input_id is not None:
            snapshot["active_input_id"] = active_input_id
        self._last_progress_snapshot = snapshot
        self._refresh_view()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        title = QLabel("ETA / WP / 촬영 스케줄 모니터")
        title.setStyleSheet("font-size: 16px; font-weight: 700;")
        subtitle = QLabel("경로 ETA와 촬영 ETA를 분리해서 보고, 각 waypoint의 계획 대비 실제 도착 시간차를 확인합니다.")
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet("color: #475569;")
        hint = QLabel("재계획 ON/OFF와 기준값 조정은 `임무 재계획 관리` 탭에서 통합 관리합니다.")
        hint.setWordWrap(True)
        hint.setStyleSheet(
            "padding: 8px 10px; border: 1px solid #dbe5f0; border-radius: 8px; background: #f8fbff; color: #334155;"
        )
        root.addWidget(title)
        root.addWidget(subtitle)
        root.addWidget(hint)

        selector_row = QHBoxLayout()
        selector_row.setSpacing(8)
        for aircraft_id, label in ((4, "UAV1"), (5, "UAV2"), (6, "UAV3")):
            button = QPushButton(label)
            button.setCheckable(True)
            button.clicked.connect(lambda checked, aid=aircraft_id: self._select_aircraft(aid))
            button.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
            selector_row.addWidget(button)
            self._aircraft_buttons[int(aircraft_id)] = button
        selector_row.addStretch(1)
        root.addLayout(selector_row)

        self._plan_summary_label = QLabel("MissionPlan: -")
        self._plan_summary_label.setStyleSheet(
            "padding: 8px 10px; border: 1px solid #cbd5e1; border-radius: 8px; background: #f8fafc;"
        )
        root.addWidget(self._plan_summary_label)

        summary_row = QHBoxLayout()
        summary_row.setSpacing(10)
        summary_row.addWidget(self._build_summary_group("경로 ETA", self._path_summary_labels), 1)
        summary_row.addWidget(self._build_summary_group("촬영 ETA", self._imaging_summary_labels), 1)
        root.addLayout(summary_row)

        root.addWidget(self._build_table_group("WP 경로 스케줄", "path"))
        root.addWidget(self._build_table_group("촬영 스케줄", "imaging"))
        self._select_aircraft(self._selected_aircraft_id)

    def _build_summary_group(self, title: str, target: dict[str, QLabel]) -> QGroupBox:
        group = QGroupBox(title)
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(6)
        for key, label in (
            ("mission_id", "Mission"),
            ("waypoint_id", "Current WP"),
            ("planned", "Planned"),
            ("actual", "Actual"),
            ("delta", "Delta"),
            ("state", "State"),
        ):
            value = QLabel("-")
            value.setTextInteractionFlags(Qt.TextSelectableByMouse)
            target[key] = value
            form.addRow(label, value)
        group.setLayout(form)
        return group

    def _build_table_group(self, title: str, kind: str) -> QGroupBox:
        group = QGroupBox(title)
        layout = QVBoxLayout(group)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        table = QTableWidget(0, 9)
        table.setHorizontalHeaderLabels(
            ["Mission", "Path", "WP", "Type", "Planned ETA", "Actual", "Delta", "Result", "Arrival"]
        )
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QAbstractItemView.NoSelection)
        table.setAlternatingRowColors(True)
        table.setWordWrap(False)
        table.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)
        header = table.horizontalHeader()
        for col in range(7):
            header.setSectionResizeMode(col, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(7, QHeaderView.Stretch)
        header.setSectionResizeMode(8, QHeaderView.Stretch)
        header.setStretchLastSection(False)
        header.setMinimumSectionSize(68)

        if kind == "path":
            self._path_table = table
        else:
            self._imaging_table = table
        layout.addWidget(table)
        return group

    def _select_aircraft(self, aircraft_id: int) -> None:
        self._selected_aircraft_id = int(aircraft_id)
        for aid, button in self._aircraft_buttons.items():
            selected = int(aid) == int(self._selected_aircraft_id)
            button.setChecked(selected)
            if selected:
                button.setStyleSheet(
                    "QPushButton { background: #1d4ed8; color: white; font-weight: 700; padding: 6px 14px; border-radius: 8px; }"
                )
            else:
                button.setStyleSheet(
                    "QPushButton { background: #e2e8f0; color: #0f172a; padding: 6px 14px; border-radius: 8px; }"
                )
        self._refresh_view()

    def _apply_mission_plan_view(self, mission_plan_id: int | None) -> None:
        self._mission_view = build_uav_mission_view(mission_plan_id, uav_ids=(4, 5, 6))
        self._progress_tracker.reset(self._mission_view)
        self._last_progress_snapshot = self._progress_tracker.update(None, None)
        self._refresh_view()

    def _refresh_view(self) -> None:
        mission_view = self._mission_view
        snapshot = self._last_progress_snapshot
        if not isinstance(mission_view, dict):
            self._set_plan_summary("MissionPlan: -")
            self._fill_summary(self._path_summary_labels, {})
            self._fill_summary(self._imaging_summary_labels, {})
            self._populate_table(self._path_table, [])
            self._populate_table(self._imaging_table, [])
            return

        schedule_view = build_aircraft_schedule_view(
            mission_view,
            snapshot or {},
            self._selected_aircraft_id,
        )
        timestamp_ms = schedule_view.get("timestamp_ms")
        mission_plan_id = mission_view.get("mission_plan_id")
        current_mission_id = schedule_view.get("current_mission_id")
        updated_text = format_timestamp_ms(timestamp_ms) if timestamp_ms is not None else "-"
        self._set_plan_summary(
            f"MissionPlan: {mission_plan_id or '-'} / Aircraft: {self._selected_aircraft_id} / "
            f"Current Mission: {current_mission_id or '-'} / Last Update: {updated_text}"
        )
        self._fill_summary(self._path_summary_labels, schedule_view.get("path_summary") or {}, imaging=False)
        self._fill_summary(self._imaging_summary_labels, schedule_view.get("imaging_summary") or {}, imaging=True)
        self._populate_table(self._path_table, schedule_view.get("path_rows") or [])
        self._populate_table(self._imaging_table, schedule_view.get("imaging_rows") or [])

    def _set_plan_summary(self, text: str) -> None:
        if self._plan_summary_label is not None:
            self._plan_summary_label.setText(str(text))

    def _refresh_trigger_controls(self) -> None:
        pass

    def _fill_summary(self, labels: dict[str, QLabel], data: dict[str, Any], *, imaging: bool = False) -> None:
        if not labels:
            return
        labels["mission_id"].setText(str(data.get("mission_id") or "-"))
        waypoint_value = data.get("current_waypoint_id") if not imaging else data.get("reached_imaging_waypoint_count")
        if imaging:
            total = data.get("imaging_waypoint_count")
            labels["waypoint_id"].setText("-" if waypoint_value is None or total is None else f"{waypoint_value}/{total}")
        else:
            labels["waypoint_id"].setText(str(waypoint_value or "-"))
        planned = data.get("planned_latest_seconds") if imaging else data.get("planned_total_seconds")
        actual = data.get("actual_latest_seconds") if imaging else data.get("actual_total_seconds")
        delta = data.get("delta_latest_seconds") if imaging else data.get("current_waypoint_delta_seconds")
        labels["planned"].setText(format_duration(planned))
        labels["actual"].setText(format_duration(actual))
        labels["delta"].setText(format_duration(delta))
        state_text = data.get("latest_state") if imaging else data.get("current_waypoint_state")
        labels["state"].setText(str(state_text or "-"))

    def _populate_table(self, table: QTableWidget | None, rows: list[dict[str, Any]]) -> None:
        if table is None:
            return
        table.setRowCount(0)
        for row_data in rows:
            row = table.rowCount()
            table.insertRow(row)
            values = [
                row_data.get("mission_id"),
                row_data.get("path_id"),
                row_data.get("waypoint_id"),
                "촬영" if row_data.get("is_imaging") else "경로",
                format_duration(row_data.get("planned_seconds")),
                format_duration(row_data.get("actual_seconds")),
                format_duration(row_data.get("delta_seconds")),
                row_data.get("schedule_state") or "-",
                format_timestamp_ms(row_data.get("completion_timestamp_ms")),
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem("-" if value is None else str(value))
                self._style_table_item(item, row_data)
                table.setItem(row, col, item)

    def _style_table_item(self, item: QTableWidgetItem, row_data: dict[str, Any]) -> None:
        state = str(row_data.get("schedule_state") or "")
        if state == "Late":
            item.setBackground(QColor("#fee2e2"))
            item.setForeground(QColor("#991b1b"))
            return
        if state == "Early":
            item.setBackground(QColor("#dbeafe"))
            item.setForeground(QColor("#1d4ed8"))
            return
        if state == "On time":
            item.setBackground(QColor("#dcfce7"))
            item.setForeground(QColor("#166534"))
            return
        if state == "Skipped":
            item.setBackground(QColor("#ffedd5"))
            item.setForeground(QColor("#9a3412"))
            return
        if state == "Pending":
            item.setForeground(QColor("#64748b"))
