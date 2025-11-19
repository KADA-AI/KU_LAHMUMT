# -*- coding: utf-8 -*-
# MonitoringTab.py – 모니터링 모듈의 요약 탭 UI

from __future__ import annotations

from datetime import datetime, timezone, timedelta
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple

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
        self.manned_aircraft_ids = [1, 2, 3]

        # UI 구성 요소
        self.system_mode_combo: QComboBox | None = None
        self.progress_bars: List[CircularProgressBar] = []
        self.progress_detail_labels: List[QLabel] = []
        self.plan_id_value: QLabel | None = None
        self.plan_package_value: QLabel | None = None
        self.plan_inputs_value: QLabel | None = None
        self.plan_active_value: QLabel | None = None
        self.plan_completion_value: QLabel | None = None
        self.plan_update_status_value: QLabel | None = None
        self.plan_update_detail_value: QLabel | None = None
        self.input_table: QTableWidget | None = None
        self.mission_table: QTableWidget | None = None
        self.aircraft_availability_labels: Dict[int, QLabel] = {}
        self._availability_label_text: Dict[int, str] = {}

        self._init_ui()
        self.refresh_display(("logic", "SystemMode"))
        self._refresh_mission_overview()
        self._update_plan_update_panel()

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
        self._build_plan_update_section(layout)

        # UAV 진행률

        progress_groupbox = QGroupBox("임무 진행률")
        progress_layout = QHBoxLayout()
        progress_groupbox.setLayout(progress_layout)
        for idx, aircraft_id in enumerate(self.uav_aircraft_ids):
            wrapper = QWidget()
            wrapper_layout = QVBoxLayout(wrapper)
            wrapper_layout.setSpacing(6)
            wrapper_layout.setContentsMargins(0, 0, 0, 0)
            wrapper_layout.setAlignment(Qt.AlignTop)

            progress_bar = CircularProgressBar()
            progress_bar.setText(f"UAV {idx + 1} (ID {aircraft_id})")
            self.progress_bars.append(progress_bar)

            detail_label = QLabel("")
            detail_label.setAlignment(Qt.AlignCenter)
            detail_label.setStyleSheet("color: #666666; font-size: 11px;")
            detail_label.setWordWrap(True)
            self.progress_detail_labels.append(detail_label)

            wrapper_layout.addWidget(progress_bar)
            wrapper_layout.addWidget(detail_label)
            progress_layout.addWidget(wrapper)
        layout.addWidget(progress_groupbox)


        # 항공기 가용 상태
        self._build_availability_section(layout)

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

    def _build_availability_section(self, parent_layout: QVBoxLayout) -> None:
        availability_group = QGroupBox("항공기 가용 상태")
        form_layout = QFormLayout()
        form_layout.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        form_layout.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        form_layout.setHorizontalSpacing(12)
        form_layout.setVerticalSpacing(6)

        def _make_row(label_text: str, ids: List[int], prefix: str) -> QWidget:
            container = QWidget()
            row_layout = QHBoxLayout(container)
            row_layout.setContentsMargins(0, 0, 0, 0)
            row_layout.setSpacing(8)
            for aircraft_id in ids:
                availability_label = self._create_availability_label(
                    f"{prefix} {aircraft_id}", aircraft_id
                )
                row_layout.addWidget(availability_label)
            row_layout.addStretch()
            form_layout.addRow(label_text, container)
            return container

        _make_row("유인기:", self.manned_aircraft_ids, "유인")
        _make_row("무인기:", self.uav_aircraft_ids, "무인")

        availability_group.setLayout(form_layout)
        parent_layout.addWidget(availability_group)

    def _build_plan_update_section(self, parent_layout: QVBoxLayout) -> None:
        update_group = QGroupBox("임무 갱신 요청 상태 (0903)")
        form_layout = QFormLayout()
        self.plan_update_status_value = QLabel("요청 없음")
        self.plan_update_detail_value = QLabel("최근 0903 요청이 없습니다.")
        self.plan_update_detail_value.setWordWrap(True)
        form_layout.addRow("상태", self.plan_update_status_value)
        form_layout.addRow("상세", self.plan_update_detail_value)
        update_group.setLayout(form_layout)
        parent_layout.addWidget(update_group)

    def _create_availability_label(self, base_text: str, aircraft_id: int) -> QLabel:
        label = QLabel(f"{base_text} (미확인)")
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(86)
        label.setStyleSheet(self._availability_style("unknown"))
        self.aircraft_availability_labels[aircraft_id] = label
        self._availability_label_text[aircraft_id] = base_text
        return label

    @staticmethod
    def _availability_style(status: str) -> str:
        palette = {
            "available": ("#2ecc71", "#ffffff"),
            "unavailable": ("#e74c3c", "#ffffff"),
            "unknown": ("#95a5a6", "#ffffff"),
        }
        bg, fg = palette.get(status, palette["unknown"])
        return (
            f"padding: 4px 10px; border-radius: 6px; background-color: {bg};"
            f" color: {fg}; font-weight: 600;"
        )

    def _update_aircraft_availability(self) -> None:
        available_ids, has_reference = self._collect_available_aircraft_ids()
        for aircraft_id, label in self.aircraft_availability_labels.items():
            base_text = self._availability_label_text.get(
                aircraft_id, f"ID {aircraft_id}"
            )
            if not has_reference:
                suffix = "미확인"
                status = "unknown"
            elif aircraft_id in available_ids:
                suffix = "가용"
                status = "available"
            else:
                suffix = "비가용"
                status = "unavailable"
            label.setText(f"{base_text} ({suffix})")
            label.setStyleSheet(self._availability_style(status))

    def _collect_available_aircraft_ids(self) -> Tuple[Set[int], bool]:
        available: Set[int] = set()
        # 1) Use mission plan file snapshot stored in logic_store
        try:
            stored_available = self.manager.logic_store.get_data(
                "input_plan_available_aircraft"
            )
        except Exception:
            stored_available = None
        if stored_available is not None:
            if isinstance(stored_available, Iterable) and not isinstance(
                stored_available, (str, bytes)
            ):
                for value in stored_available:
                    try:
                        available.add(int(value))
                    except (TypeError, ValueError):
                        continue
            return available, True

        raw_list = None
        try:
            input_plan = self.manager.receive_store.get_data("0201")
        except Exception:
            input_plan = None

        if input_plan is not None:
            if hasattr(input_plan, "availableAircraftList"):
                raw_list = getattr(input_plan, "availableAircraftList")
            elif isinstance(input_plan, dict):
                raw_list = input_plan.get("availableAircraftList")

        if raw_list is not None:
            for item in raw_list or []:
                aircraft_value = None
                if isinstance(item, dict):
                    aircraft_value = item.get("aircraftID")
                else:
                    aircraft_value = getattr(item, "aircraftID", None)
                try:
                    if aircraft_value is None:
                        continue
                    available.add(int(aircraft_value))
                except (TypeError, ValueError):
                    continue
            return available, True

        context = self.manager.logic_store.get_data("current_mission_plan")
        if isinstance(context, dict):
            aircraft_map = context.get("aircraft")
            if isinstance(aircraft_map, dict):
                for key in aircraft_map.keys():
                    try:
                        available.add(int(key))
                    except (TypeError, ValueError):
                        continue
                if available:
                    return available, True

        return set(), False

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

    def _update_plan_update_panel(self, payload: Optional[Dict[str, Any]] = None) -> None:
        if self.plan_update_status_value is None or self.plan_update_detail_value is None:
            return
        if payload is None or not isinstance(payload, dict):
            payload = self.manager.logic_store.get_data("mission_update_status") or {}
        if not payload:
            self.plan_update_status_value.setText("요청 없음")
            self.plan_update_detail_value.setText("최근 0903 요청이 없습니다.")
            return
        status_map = {
            "requested": "요청 수신",
            "applied": "적용 완료",
            "failed": "적용 실패",
        }
        status_label = status_map.get(payload.get("status"), str(payload.get("status")))
        plan_id = payload.get("planID") or payload.get("requestedPlanID")
        if plan_id is not None:
            status_text = f"{status_label} (Plan {plan_id})"
        else:
            status_text = status_label
        self.plan_update_status_value.setText(status_text)

        detail_lines: List[str] = []
        requested_plan = payload.get("requestedPlanID")
        if requested_plan and plan_id != requested_plan:
            detail_lines.append(f"요청 Plan ID: {requested_plan}")
        request_ts = payload.get("requestTimestamp")
        if request_ts is not None:
            detail_lines.append(f"요청 시각: {self._format_epoch2000_ms(request_ts)}")
        received_ts = payload.get("receivedAt")
        if received_ts is not None and received_ts != request_ts:
            detail_lines.append(f"수신 시각: {self._format_epoch2000_ms(received_ts)}")
        state_ts = payload.get("stateTimestamp") or payload.get("updatedAt")
        if state_ts is not None:
            detail_lines.append(f"최종 업데이트: {self._format_epoch2000_ms(state_ts)}")
        applied_plan = payload.get("appliedPlanID")
        if applied_plan is not None:
            detail_lines.append(f"적용 Plan ID: {applied_plan}")
        fallback_plan = payload.get("fallbackPlanID")
        if fallback_plan is not None:
            detail_lines.append(f"대체 Plan ID: {fallback_plan}")
        reason = payload.get("reason") or payload.get("detail")
        if reason:
            detail_lines.append(f"비고: {reason}")
        raw_info = payload.get("rawMissionPlanID")
        if raw_info is not None and plan_id is None:
            detail_lines.append(f"원본 식별자: {raw_info}")

        detail_text = "\n".join(detail_lines) if detail_lines else "세부 정보 없음"
        self.plan_update_detail_value.setText(detail_text)

    @staticmethod
    def _format_epoch2000_ms(value: Any) -> str:
        try:
            timestamp_ms = int(value)
        except (TypeError, ValueError):
            return "-"
        base = datetime(2000, 1, 1, tzinfo=timezone.utc)
        dt = base + timedelta(milliseconds=timestamp_ms)
        try:
            localized = dt.astimezone()
        except Exception:
            localized = dt
        return localized.strftime("%Y-%m-%d %H:%M:%S")

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

        if source == "logic" and key == "mission_update_status":
            payload = data_object if isinstance(data_object, dict) else None
            self._update_plan_update_panel(payload)

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

                highlight_ids: Set[int] = set()
                highlight_raw = (
                    self.manager.logic_store.get_data("transit_aircraft_ids") or []
                )
                for raw_id in highlight_raw:
                    try:
                        highlight_ids.add(int(raw_id))
                    except (TypeError, ValueError):
                        continue

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
                    bar.setHighlighted(aircraft_id in highlight_ids)
                    bar.setText(f"UAV {idx + 1} (ID {aircraft_id}): {progress_value}%")
            else:
                for bar in self.progress_bars:
                    bar.setHighlighted(False)

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
        mission_rows: List[Tuple[int, Any, Dict[str, Any], int]] = []

        aircraft_data = context.get("aircraft") or {}
        for aircraft_order, (aircraft_id, payload) in enumerate(aircraft_data.items()):
            missions = payload.get("missions") or []
            try:
                aircraft_int = int(aircraft_id)
            except (TypeError, ValueError):
                aircraft_int = aircraft_id
            for idx, mission in enumerate(missions):
                mission_rows.append((aircraft_order, aircraft_int, mission, idx))
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
            mission_rows.sort(key=lambda item: (item[0], item[3]))
            for row, (_, aircraft_id, mission, mission_index) in enumerate(mission_rows):
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

        self._update_aircraft_availability()
        self._update_multi_mission_labels()

    def _get_multi_mission_summary(self) -> Dict[int, Tuple[int, int]]:
        try:
            summary_data = self.manager.logic_store.get_data("aircraft_multi_mission_summary")
        except Exception:
            return {}
        if not isinstance(summary_data, dict):
            return {}
        items = summary_data.get("items")
        if not isinstance(items, dict):
            return {}

        def _to_int(value: Any) -> Optional[int]:
            try:
                return int(value)
            except (TypeError, ValueError):
                return None

        result: Dict[int, Tuple[int, int]] = {}
        for key, payload in items.items():
            if isinstance(key, str):
                aircraft_id = _to_int(key)
            else:
                aircraft_id = _to_int(key)
            if aircraft_id is None or not isinstance(payload, dict):
                continue
            completed = _to_int(payload.get("completed")) or 0
            total = _to_int(payload.get("total")) or 0
            result[aircraft_id] = (completed, total)
        return result

    def _update_multi_mission_labels(self) -> None:
        summary = self._get_multi_mission_summary()
        for idx, aircraft_id in enumerate(self.uav_aircraft_ids):
            if idx >= len(self.progress_detail_labels):
                break
            label = self.progress_detail_labels[idx]
            counts = summary.get(aircraft_id)
            if not counts or counts[1] <= 1:
                label.setText("")
                continue
            label.setText(f"{counts[0]} / {counts[1]} 개별임무")
