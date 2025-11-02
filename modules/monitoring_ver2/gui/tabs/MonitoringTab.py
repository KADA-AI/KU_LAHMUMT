# -*- coding: utf-8 -*-
# MonitoringTab.py – 모니터링 모듈의 요약 탭 UI

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Optional, Tuple

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QComboBox,
    QGroupBox,
    QFormLayout,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
)

from ..widgets.CircularProgressBar import CircularProgressBar
from modules.monitoring_ver2.config import SYSTEM_MODE_OPTIONS


class MonitoringTab(QWidget):
    """임무 모니터링 요약 탭."""

    def __init__(self, manager, parent: QWidget | None = None):
        super().__init__(parent)
        self.manager = manager
        self.uav_aircraft_ids = [4, 5, 6]

        # UI 구성 요소
        self.system_mode_combo: QComboBox | None = None
        self.progress_bars: List[CircularProgressBar] = []
        self.plan_id_value: QLabel | None = None
        self.plan_package_value: QLabel | None = None
        self.plan_inputs_value: QLabel | None = None
        self.plan_active_value: QLabel | None = None
        self.plan_completion_value: QLabel | None = None
        self.input_table: QTableWidget | None = None
        self.mission_table: QTableWidget | None = None

        self._init_ui()
        self.refresh_display(("logic", "SystemMode"))
        self._refresh_mission_overview()

    # ------------------------------------------------------------------ UI 구성
    def _init_ui(self) -> None:
        layout = QVBoxLayout(self)

        # 시스템 모드 표시/선택
        mode_groupbox = QGroupBox("시스템 운용 모드")
        mode_layout = QFormLayout()
        self.system_mode_combo = QComboBox()
        for value, label in SYSTEM_MODE_OPTIONS:
            self.system_mode_combo.addItem(label, value)
        self.system_mode_combo.currentIndexChanged.connect(self.on_system_mode_changed)
        mode_layout.addRow(QLabel("현재 모드:"), self.system_mode_combo)
        mode_groupbox.setLayout(mode_layout)
        layout.addWidget(mode_groupbox)

        # UAV 진행률
        progress_groupbox = QGroupBox("임무 진행률")
        progress_layout = QHBoxLayout()
        progress_groupbox.setLayout(progress_layout)
        for idx, aircraft_id in enumerate(self.uav_aircraft_ids):
            progress_bar = CircularProgressBar()
            progress_bar.setText(f"UAV {idx + 1} (ID {aircraft_id})")
            self.progress_bars.append(progress_bar)
            progress_layout.addWidget(progress_bar)
        layout.addWidget(progress_groupbox)

        # 임무 구조 오버뷰
        overview_layout = QHBoxLayout()

        # 1) 임무 계획 요약
        plan_group = QGroupBox("임무계획 요약")
        plan_form = QFormLayout()
        self.plan_id_value = QLabel("-")
        self.plan_package_value = QLabel("-")
        self.plan_inputs_value = QLabel("-")
        self.plan_active_value = QLabel("-")
        self.plan_completion_value = QLabel("-")
        plan_form.addRow("MissionPlan ID:", self.plan_id_value)
        plan_form.addRow("Input Package ID:", self.plan_package_value)
        plan_form.addRow("Input Mission IDs:", self.plan_inputs_value)
        plan_form.addRow("Active Input ID:", self.plan_active_value)
        plan_form.addRow("완료된 입력 임무:", self.plan_completion_value)
        plan_group.setLayout(plan_form)
        overview_layout.addWidget(plan_group, stretch=1)

        # 2) 입력 임무 테이블
        input_group = QGroupBox("입력 임무 상태")
        self.input_table = QTableWidget(0, 4)
        self.input_table.setHorizontalHeaderLabels(
            ["Input ID", "상태", "관련 항공기 수", "개별 임무 수"]
        )
        self._prepare_table(self.input_table)
        input_group_layout = QVBoxLayout()
        input_group_layout.addWidget(self.input_table)
        input_group.setLayout(input_group_layout)
        overview_layout.addWidget(input_group, stretch=2)

        # 3) 개별 임무/경로 테이블
        mission_group = QGroupBox("개별 임무 및 경로")
        self.mission_table = QTableWidget(0, 5)
        self.mission_table.setHorizontalHeaderLabels(
            ["Aircraft", "개별 임무 ID", "Input ID", "Path ID", "Waypoint IDs"]
        )
        self._prepare_table(self.mission_table, stretch_last=True)
        mission_group_layout = QVBoxLayout()
        mission_group_layout.addWidget(self.mission_table)
        mission_group.setLayout(mission_group_layout)
        overview_layout.addWidget(mission_group, stretch=3)

        layout.addLayout(overview_layout)

    @staticmethod
    def _prepare_table(table: QTableWidget, *, stretch_last: bool = False) -> None:
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QTableWidget.NoEditTriggers)
        table.setSelectionBehavior(QTableWidget.SelectRows)
        table.setSelectionMode(QTableWidget.SingleSelection)
        header = table.horizontalHeader()
        for idx in range(table.columnCount()):
            mode = QHeaderView.Stretch if (stretch_last and idx == table.columnCount() - 1) else QHeaderView.ResizeToContents
            header.setSectionResizeMode(idx, mode)

    # ------------------------------------------------------------------ 이벤트 핸들러
    def on_system_mode_changed(self, index: int) -> None:
        if self.system_mode_combo is None:
            return
        mode_value = self.system_mode_combo.itemData(index)
        if mode_value is None:
            return
        self.manager.set_system_mode(int(mode_value))

    def refresh_display(self, update_info: tuple, data_object: object = None) -> None:
        source, key = update_info
        if source == "send":
            return

        # 시스템 모드 반영
        if (source == "logic" and key == "SystemMode") or (
            source == "receive" and key == "0101"
        ):
            current_mode = (
                getattr(data_object, "systemMode", None)
                if (source == "receive" and key == "0101" and data_object is not None)
                else self.manager.get_logic_result("SystemMode")
            )
            if self.system_mode_combo is not None and current_mode is not None:
                try:
                    mode_int = int(current_mode)
                except (TypeError, ValueError):
                    mode_int = None
                if mode_int is not None:
                    idx = self.system_mode_combo.findData(mode_int)
                    if idx >= 0:
                        self.system_mode_combo.blockSignals(True)
                        self.system_mode_combo.setCurrentIndex(idx)
                        self.system_mode_combo.blockSignals(False)

        # 0501 진행률 반영
        if key == "0501":
            mission_progress_data = self.manager.get_logic_result("0501_data")
            if (
                mission_progress_data
                and "individualMissionProgressStatusList" in mission_progress_data
            ):
                progress_by_aircraft: Dict[Any, Dict[str, Any]] = {}
                for entry in mission_progress_data["individualMissionProgressStatusList"]:
                    if isinstance(entry, dict):
                        progress_by_aircraft[entry.get("aircraftID")] = entry

                for idx, aircraft_id in enumerate(self.uav_aircraft_ids):
                    if idx >= len(self.progress_bars):
                        break
                    bar = self.progress_bars[idx]
                    entry = progress_by_aircraft.get(aircraft_id, {})
                    progress_value = entry.get("currentIndividualMissionProgress") or 0
                    try:
                        bar.setValue(int(progress_value))
                    except Exception:
                        bar.setValue(0)
                    bar.setText(f"UAV {idx + 1} (ID {aircraft_id}): {progress_value}%")

        # 연료 경고 상태 반영
        if source == "logic" and key == "fuel_data":
            fuel_data = self.manager.logic_store.get_data("fuel_data")
            if fuel_data and isinstance(fuel_data, list):
                fuel_by_aircraft: Dict[Any, Dict[str, Any]] = {}
                for fuel_item in fuel_data:
                    if isinstance(fuel_item, dict):
                        fuel_by_aircraft[fuel_item.get("id")] = fuel_item

                for idx, aircraft_id in enumerate(self.uav_aircraft_ids):
                    if idx >= len(self.progress_bars):
                        break
                    bar = self.progress_bars[idx]
                    fuel_item = fuel_by_aircraft.get(aircraft_id)
                    if not fuel_item:
                        bar.setText(f"UAV {idx + 1} (ID {aircraft_id}) Fuel: N/A")
                        bar.setColor(QColor(0, 255, 0))
                        continue

                    warning_text = fuel_item.get("warning", "green") or "green"
                    display_text = warning_text
                    if warning_text == "red":
                        color = QColor(255, 0, 0)
                    elif warning_text == "yellow":
                        color = QColor(255, 255, 0)
                    elif warning_text == "unknown":
                        color = QColor(128, 128, 128)
                    else:
                        color = QColor(0, 255, 0)
                        if not warning_text:
                            display_text = "green"

                    bar.setText(f"UAV {idx + 1} (ID {aircraft_id}) Fuel: {display_text}")
                    bar.setColor(color)

        # 임무 오버뷰 갱신
        self._refresh_mission_overview()

    # ------------------------------------------------------------------ 내부 헬퍼
    def _refresh_mission_overview(self) -> None:
        """Update mission summary labels and tables based on current tracker state."""
        context = self.manager.logic_store.get_data("current_mission_plan")
        if not isinstance(context, dict):
            context = {}

        def _to_int(value: Any) -> Optional[int]:
            try:
                return int(value)
            except (TypeError, ValueError):
                return None

        plan_id = _to_int(context.get("missionPlanID"))
        input_package_id = _to_int(context.get("inputMissionPackageID"))
        active_input_id = context.get("activeInputMissionID")
        if active_input_id is None:
            active_input_id = self.manager.logic_store.get_data("active_input_mission_id")
        active_input_id = _to_int(active_input_id)

        completed_inputs_store = self.manager.logic_store.get_data("completed_input_ids") or []
        completed_set: set[int] = set()
        for value in completed_inputs_store:
            try:
                completed_set.add(int(value))
            except (TypeError, ValueError):
                continue

        raw_input_ids = context.get("inputMissionIDs") or []
        input_ids = [
            _to_int(value) for value in raw_input_ids if _to_int(value) is not None
        ]

        if self.plan_id_value:
            self.plan_id_value.setText(str(plan_id) if plan_id is not None else "-")
        if self.plan_package_value:
            self.plan_package_value.setText(
                str(input_package_id) if input_package_id is not None else "-"
            )
        if self.plan_inputs_value:
            self.plan_inputs_value.setText(
                ", ".join(str(i) for i in input_ids) if input_ids else "-"
            )
        if self.plan_active_value:
            self.plan_active_value.setText(
                str(active_input_id) if active_input_id is not None else "-"
            )

        input_summary: Dict[int, Dict[str, Any]] = {}
        mission_rows: List[Tuple[Any, Dict[str, Any], int]] = []

        aircraft_data = context.get("aircraft") or {}
        for aircraft_id, payload in aircraft_data.items():
            missions = payload.get("missions") or []
            try:
                aircraft_int = int(aircraft_id)
            except (TypeError, ValueError):
                aircraft_int = aircraft_id
            for idx, mission in enumerate(missions):
                mission_rows.append((aircraft_int, mission, idx))
                input_id = _to_int(mission.get("inputMissionID"))
                if input_id is None:
                    continue
                summary_entry = input_summary.setdefault(
                    input_id,
                    {"aircraft": set(), "count": 0, "completed": True},
                )
                summary_entry["aircraft"].add(aircraft_int)
                summary_entry["count"] += 1
                if not mission.get("isDone"):
                    summary_entry["completed"] = False

        # 입력 임무 테이블 업데이트
        if self.input_table is not None:
            self.input_table.setRowCount(0)
            for row, input_id in enumerate(sorted(input_summary.keys())):
                entry = input_summary[input_id]
                self.input_table.insertRow(row)
                is_completed = entry["completed"] or (input_id in completed_set)
                status = "완료" if is_completed else "진행 중"
                if active_input_id is not None and input_id == active_input_id:
                    status += " (활성)"
                values = [
                    str(input_id),
                    status,
                    str(len(entry["aircraft"])),
                    str(entry["count"]),
                ]
                for col, value in enumerate(values):
                    item = QTableWidgetItem(value)
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                    item.setTextAlignment(Qt.AlignCenter)
                    self.input_table.setItem(row, col, item)
                background: Optional[QColor] = None
                if is_completed:
                    background = QColor(255, 220, 220)
                if active_input_id is not None and input_id == active_input_id:
                    background = QColor(220, 235, 255)
                    if is_completed:
                        background = QColor(255, 200, 200)
                if background:
                    for col in range(self.input_table.columnCount()):
                        self.input_table.item(row, col).setBackground(background)

            if self.plan_completion_value:
                total_inputs = len(input_summary)
                completed_ids_display = [
                    input_id
                    for input_id, info in input_summary.items()
                    if info["completed"] or (input_id in completed_set)
                ]
                completed_count = len(completed_ids_display)
                if total_inputs == 0:
                    total_inputs = len(input_ids) or len(completed_set)
                    completed_ids_display = sorted(completed_set)
                    completed_count = len(completed_ids_display)
                summary_text = f"{completed_count} / {total_inputs}" if total_inputs else "0 / 0"
                if completed_ids_display:
                    summary_text += f" (IDs: {', '.join(str(i) for i in sorted(completed_ids_display))})"
                self.plan_completion_value.setText(summary_text)
        else:
            if self.plan_completion_value:
                summary_text = "0 / 0"
                if completed_set:
                    summary_text = f"{len(completed_set)} / 0 (IDs: {', '.join(str(i) for i in sorted(completed_set))})"
                self.plan_completion_value.setText(summary_text)

        # 개별 임무 테이블 업데이트
        if self.mission_table is not None:
            self.mission_table.setRowCount(0)
            mission_rows.sort(
                key=lambda item: (
                    item[0],
                    _to_int(item[1].get("individualMissionID")) or 0,
                )
            )
            for row, (aircraft_id, mission, mission_index) in enumerate(mission_rows):
                self.mission_table.insertRow(row)
                individual_id = _to_int(mission.get("individualMissionID"))
                input_id = _to_int(mission.get("inputMissionID"))
                path_id = _to_int(mission.get("pathID"))
                waypoint_ids = mission.get("waypoints") or []
                waypoint_text = ", ".join(
                    str(_to_int(wp) if _to_int(wp) is not None else wp)
                    for wp in waypoint_ids
                ) or "-"
                is_done = bool(mission.get("isDone"))

                values = [
                    str(aircraft_id),
                    str(individual_id) if individual_id is not None else "-",
                    str(input_id) if input_id is not None else "-",
                    str(path_id) if path_id is not None else "-",
                    waypoint_text,
                ]
                for col, value in enumerate(values):
                    item = QTableWidgetItem(value)
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                    alignment = Qt.AlignCenter if col < 4 else Qt.AlignLeft
                    item.setTextAlignment(alignment)
                    self.mission_table.setItem(row, col, item)
                background = None
                if is_done:
                    background = QColor(255, 220, 220)
                elif input_id is not None and input_id == active_input_id:
                    background = QColor(220, 235, 255)
                if background:
                    for col in range(self.mission_table.columnCount()):
                        self.mission_table.item(row, col).setBackground(background)
