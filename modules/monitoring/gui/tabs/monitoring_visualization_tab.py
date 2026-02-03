# -*- coding: utf-8 -*-
from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QBrush
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QFormLayout,
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
    mission_plan_json_path,
)
from modules.monitoring.logic.mission_progress import MissionProgressTracker

class MonitoringVisualizationTab(QWidget):
    """Monitoring visualization UI (placeholder only)."""

    def __init__(self, parent: QWidget | None = None):
        super().__init__(parent)
        self._individual_package_fields: list[QLineEdit] = []
        self._individual_package_bars: list[QProgressBar] = []
        self._individual_low_layouts: list[QHBoxLayout] = []
        self._mission_tables: list[QTableWidget] = []
        self._aircraft_labels: dict[int, QLabel] = {}
        self._signal_labels: dict[int, QLabel] = {}
        self._mission_header_labels: dict[int, QLabel] = {}
        self._available_ids: set[int] = set()
        self._fuel_state_by_aircraft: dict[int, str] = {}
        self._availability_stage: str | None = None
        self._input_mission_low_layout: QHBoxLayout | None = None
        self._mission_plan_bar: QProgressBar | None = None
        self._mission_view: dict | None = None
        self._progress_tracker = MissionProgressTracker()
        self._last_forced_input_id: int | None = None
        self._last_forced_mission_ids: list[int] = []
        self._last_active_input_id: int | None = None
        self._last_progress_input_id: int | None = None
        self._last_progress_snapshot: dict | None = None
        self._sent_0503_inputs: set[int] = set()
        self._forced_completion_inputs: set[int] = set()
        self._pending_completion_inputs: list[int] = []
        self._pending_execute_inputs: list[int] = []
        self._sent_0503_pending_inputs: set[int] = set()
        self._sent_final_completion: bool = False
        self._recommend_callback = None
        self._notice_callback = None
        self._reexecute_callback = None
        self._ui_updates_enabled = True
        self._build_ui()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(10)

        root.addWidget(self._build_update_group())
        root.addWidget(self._build_mission_plan_group())
        root.addWidget(self._build_individual_plan_group())

        tables_row = QHBoxLayout()
        tables_row.setSpacing(10)
        for uav_id in (4, 5, 6):
            tables_row.addWidget(self._build_mission_table_group(uav_id), 1)
        root.addLayout(tables_row, 1)

        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(10)
        bottom_row.addWidget(self._build_availability_group(), 1)
        bottom_row.addWidget(self._build_signal_status_group(), 1)
        root.addLayout(bottom_row)

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
        self._mission_plan_bar = bar
        top_layout.addWidget(self._mission_plan_id_field)
        top_layout.addWidget(bar, 1)
        layout.addWidget(top_row)

        low_scroll, low_layout = self._build_lowlevel_scroll()
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
            top_layout.addWidget(package_id_field)
            top_layout.addWidget(bar, 1)

            low_scroll, low_layout = self._build_lowlevel_scroll()

            row_layout.addWidget(top_row)
            row_layout.addWidget(low_scroll)
            layout.addWidget(row)
            self._individual_package_fields.append(package_id_field)
            self._individual_package_bars.append(bar)
            self._individual_low_layouts.append(low_layout)
        group.setLayout(layout)
        return group

    def _build_mission_table_group(self, uav_id: int) -> QGroupBox:
        group = QGroupBox()
        group.setTitle("")
        table = QTableWidget(0, 4)
        table.setHorizontalHeaderLabels(
            ["개별 임무 ID", "Input ID", "Path ID", "Waypoint IDs"]
        )
        table.verticalHeader().setVisible(False)
        header = table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeToContents)
        header.setStretchLastSection(False)
        header.setMinimumHeight(22)
        table.setAlternatingRowColors(True)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setSelectionMode(QAbstractItemView.SingleSelection)
        table.setMinimumHeight(160)
        table.setWordWrap(False)
        table.setTextElideMode(Qt.ElideNone)
        table.setHorizontalScrollMode(QAbstractItemView.ScrollPerPixel)

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
        group = QGroupBox("항공기 가용 상태")
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
        group = QGroupBox("최근 신호 상태")
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
            label.setStyleSheet(self._pill_style("good" if is_available else "bad"))
        for aid in list(self._mission_header_labels.keys()):
            self._update_mission_header(aid)

    def update_agent_status(
        self,
        *,
        timestamp_ms: int | None,
        agent_states: list[dict],
        fuel_state_map: dict[int, str] | None = None,
    ) -> None:
        self._availability_stage = "0401"
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
        for aid, label in self._aircraft_labels.items():
            role = "유인" if aid <= 3 else "무인"
            health = state_map.get(aid, {}).get("health")
            if health == 1:
                status = "정상"
                style = "good"
            elif health == 0:
                status = "미정"
                style = "warn"
            else:
                status = "비정상"
                style = "bad"
            if health is None:
                status = "미확인"
                style = "bad"
            label.setText(f"{role} {aid} ({status})")
            label.setStyleSheet(self._pill_style(style))

        for aid, label in self._signal_labels.items():
            role = "유인" if aid <= 3 else "무인"
            last_signal = state_map.get(aid, {}).get("last_signal_time")
            ok = self._signal_ok(timestamp_ms, last_signal)
            status = "신호 정상" if ok else "신호 이상"
            label.setText(f"{role} {aid} ({status})")
            label.setStyleSheet(self._pill_style("good" if ok else "bad"))

        update_ui = bool(self._ui_updates_enabled)
        self._update_mission_progress(timestamp_ms, agent_states, update_ui=update_ui)
        if update_ui:
            for aid in list(self._mission_header_labels.keys()):
                self._update_mission_header(aid)

    def set_forced_wait(
        self,
        *,
        aircraft_id: int | None,
        paused: bool,
        timestamp_ms: int | None,
    ) -> None:
        if aircraft_id is None:
            return
        try:
            if paused:
                self._progress_tracker.pause_aircraft(int(aircraft_id), timestamp_ms)
            else:
                self._progress_tracker.resume_aircraft(int(aircraft_id), timestamp_ms)
        except Exception:
            return
        snapshot = self._progress_tracker.update(timestamp_ms, [])
        self._apply_progress_snapshot(snapshot)
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
        next_pending_id = self._first_pending_id(input_missions, "input_mission_id")
        if next_pending_id is None:
            target_input_id = self._resolve_current_input_id(input_missions)
            if callable(self._recommend_callback):
                try:
                    self._recommend_callback(3, target_input_id)
                except Exception:
                    pass
            if callable(self._notice_callback):
                try:
                    self._notice_callback("다음 임무가 없습니다")
                except Exception:
                    pass
            snapshot = self._progress_tracker.update(None, None)
            self._apply_progress_snapshot(snapshot)
            return
        current_input_id = self._last_progress_input_id
        if current_input_id is None:
            current_input_id = self._progress_tracker.get_active_input_id()
        if current_input_id is None:
            current_input_id = self._last_active_input_id
        if current_input_id is None:
            current_input_id = self._resolve_current_input_id(input_missions)
        if current_input_id is None:
            return
        current_done = False
        for item in input_missions:
            if not isinstance(item, dict):
                continue
            if item.get("input_mission_id") == current_input_id:
                current_done = bool(item.get("is_done"))
                break
        if current_done:
            try:
                if int(current_input_id) != int(next_pending_id):
                    current_input_id = int(next_pending_id)
                    current_done = False
            except Exception:
                current_input_id = next_pending_id
                current_done = False
            if current_done:
                snapshot = self._progress_tracker.update(None, None)
                self._apply_progress_snapshot(snapshot)
                return

        for item in input_missions:
            if not isinstance(item, dict):
                continue
            if item.get("input_mission_id") == current_input_id:
                item["is_done"] = True

        fallback_completed: list[dict[str, int | None]] = []
        mission_ids: list[int] = []
        for entry in view.get("uav_entries") or []:
            package_id = entry.get("individual_mission_package_id")
            for mission in entry.get("missions") or []:
                if not isinstance(mission, dict):
                    continue
                if mission.get("input_id") != current_input_id:
                    continue
                mission_id = mission.get("individual_mission_id")
                if mission_id is None:
                    continue
                mission["is_done"] = True
                try:
                    mission_ids.append(int(mission_id))
                except Exception:
                    continue
                fallback_completed.append(
                    {
                        "mission_id": int(mission_id),
                        "package_id": package_id,
                    }
                )

        completed = self._progress_tracker.force_complete_input(current_input_id)
        if not completed and mission_ids:
            completed = self._progress_tracker.force_complete_missions(mission_ids)
        completed_map: dict[int, int | None] = {}
        for item in completed + fallback_completed:
            mission_id = item.get("mission_id") if isinstance(item, dict) else None
            if mission_id is None:
                continue
            completed_map[int(mission_id)] = item.get("package_id")

        input_package_id = view.get("input_mission_package_id")
        mark_input_mission_done(input_package_id, current_input_id)
        for mission_id, package_id in completed_map.items():
            mark_individual_mission_done(package_id, mission_id)

        self._last_forced_input_id = current_input_id
        self._last_forced_mission_ids = sorted(completed_map.keys())
        try:
            self._forced_completion_inputs.add(int(current_input_id))
        except Exception:
            pass

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

        self._repeat_input_mission(view, target_input_id)

        snapshot = self._progress_tracker.update(None, None)
        self._apply_progress_snapshot(snapshot)

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
            table.resizeColumnsToContents()

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

    @staticmethod
    def _make_id_field(text: str) -> QLineEdit:
        field = QLineEdit(text)
        field.setReadOnly(True)
        field.setFixedWidth(180)
        field.setAlignment(Qt.AlignCenter)
        return field

    def _build_lowlevel_scroll(self) -> tuple[QScrollArea, QHBoxLayout]:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedHeight(44)

        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        scroll.setWidget(container)
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
            highlight = current_id is not None and int(item_id) == int(current_id)
            actual_seconds = item.get(actual_key)
            planned_seconds = item.get(planned_key)
            if planned_seconds is None and eta_key:
                planned_seconds = item.get(eta_key)
            bar = self._make_small_progress_bar(
                str(label),
                int(value),
                highlight=highlight,
                actual_seconds=actual_seconds,
                planned_seconds=planned_seconds,
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
        bar.setFormat(f"{label} - %p%{suffix}")
        bar.setTextVisible(True)
        bar.setAlignment(Qt.AlignCenter)
        bar.setFixedHeight(20)
        bar.setMinimumWidth(260)
        border = "2px solid #2563eb" if highlight else "1px solid #9aa3a8"
        bar.setStyleSheet(
            "QProgressBar { border: "
            + border
            + "; border-radius: 3px; background: #f5f5f5; text-align: center; }"
            "QProgressBar::chunk { background-color: #7ee38b; }"
        )
        return bar

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
            label.setStyleSheet(self._pill_style("unknown"))
            registry[int(aircraft_id)] = label
            layout.addWidget(label)
        layout.addStretch(1)
        return container

    @staticmethod
    def _make_progress_bar() -> QProgressBar:
        bar = QProgressBar()
        bar.setRange(0, 100)
        bar.setValue(0)
        bar.setFormat("%p%")
        bar.setTextVisible(True)
        bar.setMinimumHeight(22)
        bar.setStyleSheet(
            "QProgressBar { border: 1px solid #9aa3a8; border-radius: 3px; "
            "background: #f5f5f5; text-align: center; }"
            "QProgressBar::chunk { background-color: #7ee38b; }"
        )
        return bar

    @staticmethod
    def _pill_style(status: str | bool) -> str:
        if isinstance(status, bool):
            status = "good" if status else "bad"
        color_map = {
            "good": "#22c55e",
            "warn": "#f59e0b",
            "bad": "#ef4444",
            "unknown": "#9aa3a8",
        }
        color = color_map.get(status, "#9aa3a8")
        return (
            "padding: 4px 10px; border-radius: 6px; "
            f"background-color: {color}; color: #ffffff; font-weight: 600;"
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

    def _update_mission_header(self, aircraft_id: int) -> None:
        label = self._mission_header_labels.get(int(aircraft_id))
        if label is None:
            return
        dot_color = self._dot_color_for_aircraft(int(aircraft_id))
        label.setText(
            f"UAV {int(aircraft_id)} <span style=\"color: {dot_color};\">●</span>"
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
        return abs(ts - ls) <= 7000

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
            label.setStyleSheet(self._pill_style("unknown"))
            layout.addWidget(label)
        layout.addStretch(1)
        return container

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
                self._mission_plan_bar.setValue(int(plan_progress.get("progress_percent", 0)))

        entries = view.get("uav_entries") or []
        for idx, entry in enumerate(entries):
            missions = entry.get("missions") or []
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

            current_mission_id = self._next_pending_id(missions, "individual_mission_id")
            if update_ui and idx < len(self._individual_low_layouts):
                self._populate_lowlevel_bars(
                    self._individual_low_layouts[idx],
                    missions,
                    current_mission_id,
                    id_key="individual_mission_id",
                    actual_key="actual_seconds_real",
                    eta_key="eta_seconds",
                )

            aircraft_id = entry.get("aircraft_id")
            if (
                update_ui
                and idx < len(self._individual_package_bars)
                and aircraft_id in package_progress
            ):
                self._individual_package_bars[idx].setValue(
                    int(package_progress[aircraft_id].get("progress_percent", 0))
                )

            if update_ui and idx < len(self._mission_tables):
                table = self._mission_tables[idx]
                table_current_id = self._first_pending_id(missions, "individual_mission_id")
                self._update_table_row_status(table, missions, table_current_id)

        # completion recommendations are collected by the monitoring GUI (logic layer)

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
            current_mission_id = self._next_pending_id(missions, "individual_mission_id")
            if current_mission_id is None and missions:
                try:
                    current_mission_id = int(missions[-1].get("individual_mission_id"))
                except Exception:
                    current_mission_id = None

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

        ts = int(timestamp_ms) if timestamp_ms is not None else 0
        return {
            "timestamp": ts,
            "source": source,
            "currentMissionPlanID": plan_id,
            "currentInputMissionID": input_id,
            "individualMissionProgressStatusList": progress_list,
        }

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

        if self._pending_execute_inputs:
            queued: list[int] = []
            seen: set[int] = set()
            for raw_id in self._pending_execute_inputs:
                try:
                    input_id_int = int(raw_id)
                except Exception:
                    continue
                if input_id_int in seen:
                    continue
                seen.add(input_id_int)
                queued.append(input_id_int)
            self._pending_execute_inputs = []

            if (not remaining) and not self._sent_final_completion:
                final_input_id = None
                if self._last_progress_input_id is not None:
                    final_input_id = self._last_progress_input_id
                elif self._last_active_input_id is not None:
                    final_input_id = self._last_active_input_id
                elif all_input_ids:
                    final_input_id = all_input_ids[-1]
                if final_input_id is not None:
                    try:
                        final_input_id = int(final_input_id)
                    except Exception:
                        final_input_id = None
                if final_input_id is not None:
                    self._sent_0503_pending_inputs.add(final_input_id)
                    recommendations.append((3, final_input_id))
                    self._sent_final_completion = True
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
                    self._sent_0503_pending_inputs.add(input_id_int)
                    recommendations.append((1, input_id_int))

        if self._pending_completion_inputs:
            queued: list[int] = []
            seen: set[int] = set()
            for raw_id in self._pending_completion_inputs:
                try:
                    input_id_int = int(raw_id)
                except Exception:
                    continue
                if input_id_int in seen:
                    continue
                seen.add(input_id_int)
                queued.append(input_id_int)
            self._pending_completion_inputs = []

            for input_id_int in queued:
                if input_id_int in self._sent_0503_inputs:
                    continue
                if input_id_int in self._forced_completion_inputs:
                    continue
                recommend = 3 if not remaining else 1
                if recommend == 3 and self._sent_final_completion:
                    continue
                if recommend == 1 and input_id_int in self._sent_0503_pending_inputs:
                    continue
                self._sent_0503_inputs.add(input_id_int)
                if recommend == 3:
                    try:
                        self._sent_0503_pending_inputs.discard(input_id_int)
                    except Exception:
                        pass
                    self._sent_final_completion = True
                recommendations.append((recommend, input_id_int))
                if recommend == 3 and not remaining:
                    self._sent_final_completion = True

        if not remaining and not self._sent_final_completion:
            final_input_id = None
            if self._last_progress_input_id is not None:
                final_input_id = self._last_progress_input_id
            elif self._last_active_input_id is not None:
                final_input_id = self._last_active_input_id
            elif all_input_ids:
                final_input_id = all_input_ids[-1]
            if final_input_id is not None:
                try:
                    final_input_id = int(final_input_id)
                except Exception:
                    final_input_id = None
            if final_input_id is not None:
                recommendations.append((3, final_input_id))
                self._sent_final_completion = True

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

    def _resolve_current_input_id(self, input_missions: list[dict]) -> int | None:
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
