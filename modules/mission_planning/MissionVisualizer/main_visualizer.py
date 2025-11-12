"""
Interactive viewer for InputMissionPlan packages and all related mission artifacts.

Usage:
    python -m modules.mission_planning.MissionVisualizer.main_visualizer
"""
from __future__ import annotations

import json
import sys
import tempfile
from collections import defaultdict
from contextlib import contextmanager
from dataclasses import dataclass, field
from itertools import cycle
from pathlib import Path
from typing import Any, Iterable, Optional

import folium
from branca.element import MacroElement, Template
from PyQt5.QtCore import Qt, QUrl
from PyQt5.QtWidgets import (
    QApplication,
    QFileDialog,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)
from PyQt5.QtWebEngineWidgets import QWebEngineView

HERE = Path(__file__).resolve()
PKG_DIR = HERE.parent
PROJECT_ROOT = HERE.parents[3]
for path in (PKG_DIR, PROJECT_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from modules.mission_planning.id_relationship_tab import RelationshipCache


@dataclass
class ConnectedEntry:
    """Individual mission entry linked back to an InputMission."""

    individual_package_id: Optional[int]
    aircraft_id: Optional[int]
    individual_mission_id: Optional[int]
    input_mission_id: Optional[int]
    plan_ids: list[int] = field(default_factory=list)
    path_id: Optional[int] = None
    info: dict[str, Any] = field(default_factory=dict)
    is_done: bool = False
    file_path: Optional[Path] = None
    raw_entry: Optional[dict[str, Any]] = None


@dataclass
class FlightPathEntry:
    """Lightweight holder for a flight path (0303/0304)."""

    path_id: int
    aircraft_id: Optional[int]
    coordinates: list[tuple[float, float, Optional[float]]] = field(default_factory=list)
    waypoints: list[dict[str, Any]] = field(default_factory=list)
    file_path: Optional[Path] = None


@contextmanager
def block_signals(widget):
    widget.blockSignals(True)
    try:
        yield
    finally:
        widget.blockSignals(False)


class MissionPlanVisualizer(QWidget):
    """Folium + PyQt viewer for InputMissionPlan and linked missions."""

    MISSION_COLORS = [
        "#ff6b6b",
        "#ff9f43",
        "#feca57",
        "#1dd1a1",
        "#54a0ff",
        "#5f27cd",
        "#c56cf0",
        "#ffb8b8",
        "#00d2d3",
        "#576574",
        "#01a3a4",
        "#f368e0",
        "#ff9ff3",
    ]

    AIRCRAFT_COLORS = [
        "#1f77b4",
        "#ff7f0e",
        "#2ca02c",
        "#d62728",
        "#9467bd",
        "#8c564b",
        "#e377c2",
        "#7f7f7f",
        "#bcbd22",
        "#17becf",
    ]
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Input Mission Plan Visualizer")
        self.resize(1700, 950)

        self.default_dir = self._resolve_default_dir()
        self.default_dir.mkdir(parents=True, exist_ok=True)

        self._map_dir = Path(tempfile.mkdtemp(prefix="mission_visualizer_"))
        self._map_html = self._map_dir / "mission_map.html"

        self.package_data: Optional[dict[str, Any]] = None
        self.package_id: Optional[int] = None
        self.db_root: Optional[Path] = None

        self.connected_entries: list[ConnectedEntry] = []
        self.entries_by_input: dict[int, list[ConnectedEntry]] = defaultdict(list)
        self.paths_data: dict[int, FlightPathEntry] = {}
        self.connected_plans: dict[int, dict[str, Any]] = {}

        self._mission_row_lookup: dict[int, int] = {}
        self._current_focus_id: Optional[int] = None
        self._current_individual_index: Optional[int] = None

        self._mission_color_cycle = cycle(self.MISSION_COLORS)
        self._mission_color_map: dict[int, str] = {}
        self._aircraft_color_cycle = cycle(self.AIRCRAFT_COLORS)
        self._aircraft_colors: dict[int, str] = {}

        self._build_ui()
        self._render_blank_map()

    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        root = QHBoxLayout(self)

        map_frame = QFrame()
        map_layout = QVBoxLayout(map_frame)
        map_layout.setContentsMargins(0, 0, 0, 0)
        self.map_view = QWebEngineView()
        map_layout.addWidget(self.map_view)
        root.addWidget(map_frame, 2)

        side_frame = QFrame()
        side_layout = QVBoxLayout(side_frame)
        root.addWidget(side_frame, 1)

        file_row = QHBoxLayout()
        self.file_edit = QLineEdit()
        self.file_edit.setReadOnly(True)
        self.file_edit.setPlaceholderText("InputMissionPlan *.json 파일을 선택하세요.")
        file_row.addWidget(self.file_edit, 1)
        file_btn = QPushButton("파일 열기")
        file_btn.clicked.connect(self._choose_file)
        file_row.addWidget(file_btn)
        side_layout.addLayout(file_row)

        self.summary_label = QLabel("패키지를 불러오면 요약 정보가 여기에 표시됩니다.")
        self.summary_label.setWordWrap(True)
        side_layout.addWidget(self.summary_label)

        self.connection_label = QLabel("연결된 MissionPlan/Individual 정보가 여기에 표시됩니다.")
        self.connection_label.setWordWrap(True)
        side_layout.addWidget(self.connection_label)

        side_layout.addWidget(QLabel("Input 임무 목록"))
        self.mission_list = QListWidget()
        self.mission_list.itemSelectionChanged.connect(self._handle_mission_selection)
        side_layout.addWidget(self.mission_list, 1)

        side_layout.addWidget(QLabel("연결된 개별 임무"))
        self.individual_list = QListWidget()
        self.individual_list.itemSelectionChanged.connect(self._handle_individual_selection)
        side_layout.addWidget(self.individual_list, 1)

        self.detail_box = QPlainTextEdit()
        self.detail_box.setReadOnly(True)
        self.detail_box.setPlaceholderText("임무를 선택하면 상세 정보가 표시됩니다.")
        side_layout.addWidget(self.detail_box, 1)

        btn_row = QHBoxLayout()
        show_all_btn = QPushButton("선택 해제")
        show_all_btn.clicked.connect(self._clear_focus)
        btn_row.addWidget(show_all_btn)

        refresh_btn = QPushButton("지도 새로고침")
        refresh_btn.clicked.connect(self._render_map)
        btn_row.addWidget(refresh_btn)
        side_layout.addLayout(btn_row)
        side_layout.addStretch()
    # ---------------------------------------------------------- File actions
    def _choose_file(self) -> None:
        start_dir = str(self.default_dir if self.default_dir.exists() else PROJECT_ROOT)
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "InputMissionPlan 파일 선택",
            start_dir,
            "JSON Files (*.json);;All Files (*)",
        )
        if file_path:
            self._load_package(Path(file_path))

    def _load_package(self, file_path: Path) -> None:
        try:
            with file_path.open("r", encoding="utf-8") as fp:
                data = json.load(fp)
        except Exception as exc:
            QMessageBox.critical(self, "파일 오류", f"파일을 읽을 수 없습니다.\n{exc}")
            return

        if not isinstance(data, dict):
            QMessageBox.warning(self, "형식 오류", "JSON 루트가 객체가 아닙니다.")
            return

        self.package_data = data
        self.package_id = data.get("inputMissionPackageID")
        self.db_root = self._detect_db_root(file_path)

        self.file_edit.setText(str(file_path))
        self._mission_color_cycle = cycle(self.MISSION_COLORS)
        self._mission_color_map.clear()
        self._aircraft_color_cycle = cycle(self.AIRCRAFT_COLORS)
        self._aircraft_colors.clear()
        self._current_focus_id = None
        self._current_individual_index = None

        self._load_connected_scope()
        self._update_summary()
        self._update_connection_summary()
        self._populate_mission_list()
        self._populate_individual_list()
        self.detail_box.setPlainText("임무를 선택하면 상세 정보가 표시됩니다.")
        self._render_map()

    # ------------------------------------------------------ Connected data
    def _load_connected_scope(self) -> None:
        self.connected_entries.clear()
        self.entries_by_input = defaultdict(list)
        self.paths_data.clear()
        self.connected_plans.clear()

        if self.package_id is None or self.db_root is None or not self.db_root.exists():
            self.connection_label.setText("연결된 데이터를 찾을 수 없습니다.")
            return

        cache = RelationshipCache(self.db_root)
        try:
            cache.refresh()
            cache.filter_scope({"packages": {int(self.package_id)}})
        except Exception as exc:
            self.connection_label.setText(f"ID 연관 정보를 불러오지 못했습니다: {exc}")
            return

        plan_ids = cache.get_plan_ids_for_package(int(self.package_id))
        for pid in plan_ids:
            if pid in cache.mission_plans:
                self.connected_plans[pid] = cache.mission_plans[pid]

        entries: list[ConnectedEntry] = []
        for imp_id, imp in cache.individual_packages.items():
            data = imp.get("data") or {}
            aircraft_id = imp.get("aircraft_id")
            plan_list = sorted(cache.individual_package_to_plans.get(imp_id, []))
            file_path = imp.get("path")
            for mission in data.get("individualMissionList") or []:
                related = mission.get("relatedMission") or {}
                entry = ConnectedEntry(
                    individual_package_id=imp_id,
                    aircraft_id=aircraft_id,
                    individual_mission_id=mission.get("individualMissionID"),
                    input_mission_id=related.get("inputMissionID"),
                    plan_ids=plan_list,
                    path_id=mission.get("pathID"),
                    info=mission.get("individualMissionInfo") or {},
                    is_done=mission.get("isDone", False),
                    file_path=file_path,
                    raw_entry=mission,
                )
                entries.append(entry)
                if entry.input_mission_id is not None:
                    self.entries_by_input[int(entry.input_mission_id)].append(entry)

        self.connected_entries = entries
        aircraft_ids = sorted(
            {entry.aircraft_id for entry in self.connected_entries if entry.aircraft_id is not None}
        )
        for aircraft_id in aircraft_ids:
            self._aircraft_colors[aircraft_id] = next(self._aircraft_color_cycle)

        path_ids = {entry.path_id for entry in self.connected_entries if entry.path_id}
        self.paths_data = self._load_flight_paths(path_ids)

    def _load_flight_paths(self, path_ids: set[int]) -> dict[int, FlightPathEntry]:
        data: dict[int, FlightPathEntry] = {}
        if not path_ids or not self.db_root:
            return data

        path_dir = self.db_root / "FlightPath"
        for pid in sorted(path_ids):
            path_file = path_dir / f"{pid}.json"
            if not path_file.exists():
                continue
            try:
                payload = json.loads(path_file.read_text(encoding="utf-8"))
            except Exception:
                continue

            waypoints = payload.get("lahWaypointList") or payload.get("waypointList") or []
            coords: list[tuple[float, float, Optional[float]]] = []
            for wp in waypoints:
                coord = wp.get("coordinate") or {}
                lat = coord.get("latitude")
                lon = coord.get("longitude")
                if lat is None or lon is None:
                    continue
                coords.append((lat, lon, coord.get("altitude")))

            data[int(pid)] = FlightPathEntry(
                path_id=int(pid),
                aircraft_id=payload.get("aircraftID"),
                coordinates=coords,
                waypoints=waypoints,
                file_path=path_file,
            )
        return data
    # ------------------------------------------------------------- UI helpers
    def _update_summary(self) -> None:
        if not self.package_data:
            self.summary_label.setText("패키지를 불러오면 요약 정보가 여기에 표시됩니다.")
            return

        pkg_id = self.package_data.get("inputMissionPackageID", "-")
        pkg_type = self.package_data.get("inputMissionPackageType", "-")
        missions = self.package_data.get("inputMissionList") or []

        type_counts: dict[Any, int] = {}
        point_count = line_count = area_count = 0
        for mission in missions:
            m_type = mission.get("inputMissionType", "N/A")
            type_counts[m_type] = type_counts.get(m_type, 0) + 1
            detail = mission.get("missionDetail") or {}
            point_count += len(detail.get("coordinateList") or [])
            line_count += sum(len(line.get("coordinateList") or []) for line in detail.get("lineList") or [])
            area_count += sum(len(area.get("coordinateList") or []) for area in detail.get("areaList") or [])

        type_text = ", ".join(f"type {t}: {c}" for t, c in sorted(type_counts.items())) or "없음"
        root_text = f"DB Root: {self._display_path(self.db_root)}" if self.db_root else "DB Root: 미확인"
        summary = (
            f"패키지 ID: {pkg_id}\n"
            f"패키지 타입: {pkg_type}\n"
            f"임무 수: {len(missions)} ({type_text})\n"
            f"좌표 수: 점 {point_count} / 선 노드 {line_count} / 면 노드 {area_count}\n"
            f"{root_text}"
        )
        self.summary_label.setText(summary)

    def _update_connection_summary(self) -> None:
        if not self.connected_entries:
            self.connection_label.setText("연결된 MissionPlan/Individual 데이터를 찾을 수 없습니다.")
            return

        plan_ids = ", ".join(str(pid) for pid in sorted(self.connected_plans.keys())) or "없음"
        aircrafts = ", ".join(
            f"AC{aid}" for aid in sorted(self._aircraft_colors.keys())
        ) or "없음"
        text = (
            f"MissionPlan ID: {plan_ids}\n"
            f"개별 임무 수: {len(self.connected_entries)} (항공기 {aircrafts})\n"
            f"FlightPath: {len(self.paths_data)}개"
        )
        self.connection_label.setText(text)

    def _populate_mission_list(self) -> None:
        self.mission_list.clear()
        self._mission_row_lookup.clear()
        if not self.package_data:
            return

        missions = self.package_data.get("inputMissionList") or []
        for idx, mission in enumerate(missions):
            mission_id = mission.get("inputMissionID")
            detail = mission.get("missionDetail") or {}
            points = len(detail.get("coordinateList") or [])
            lines = len(detail.get("lineList") or [])
            areas = len(detail.get("areaList") or [])
            linked = len(self.entries_by_input.get(int(mission_id or -1), []))
            text = (
                f"ID {mission_id} | type {mission.get('inputMissionType', '-')}"
                f" | 점 {points} · 선 {lines} · 면 {areas}"
                f" | 연결 {linked}"
            )
            item = QListWidgetItem(text)
            item.setData(Qt.UserRole, mission_id)
            if mission.get("isDone"):
                item.setForeground(Qt.gray)
            self.mission_list.addItem(item)
            if mission_id is not None:
                self._mission_row_lookup[int(mission_id)] = idx

    def _populate_individual_list(self) -> None:
        self.individual_list.clear()
        for idx, entry in enumerate(self.connected_entries):
            label = (
                f"AC {entry.aircraft_id or '?'} | IMP {entry.individual_package_id} | "
                f"IM {entry.individual_mission_id} | Input {entry.input_mission_id or '-'}"
            )
            if entry.path_id:
                label += f" | Path {entry.path_id}"
            item = QListWidgetItem(label)
            item.setData(Qt.UserRole, idx)
            if entry.is_done:
                item.setForeground(Qt.darkGreen)
            self.individual_list.addItem(item)
    def _handle_mission_selection(self) -> None:
        current_item = self.mission_list.currentItem()
        if not current_item:
            self._current_focus_id = None
            if self._current_individual_entry() is None:
                self.detail_box.setPlainText("임무를 선택하면 상세 정보가 표시됩니다.")
            self._render_map()
            return

        mission_id = current_item.data(Qt.UserRole)
        self._current_focus_id = mission_id
        mission = self._find_mission(mission_id)
        self._update_input_detail(mission)
        self._render_map()

    def _handle_individual_selection(self) -> None:
        current_item = self.individual_list.currentItem()
        if not current_item:
            self._current_individual_index = None
            if self._current_focus_id is None:
                self.detail_box.setPlainText("임무를 선택하면 상세 정보가 표시됩니다.")
            self._render_map()
            return

        idx = current_item.data(Qt.UserRole)
        if idx is None or idx < 0 or idx >= len(self.connected_entries):
            return
        self._current_individual_index = idx
        entry = self.connected_entries[idx]
        self._update_individual_detail(entry)

        if entry.input_mission_id is not None and entry.input_mission_id in self._mission_row_lookup:
            row = self._mission_row_lookup[int(entry.input_mission_id)]
            with block_signals(self.mission_list):
                self.mission_list.setCurrentRow(row)
            self._current_focus_id = entry.input_mission_id
        self._render_map()

    def _clear_focus(self) -> None:
        with block_signals(self.mission_list):
            self.mission_list.clearSelection()
        with block_signals(self.individual_list):
            self.individual_list.clearSelection()
        self._current_focus_id = None
        self._current_individual_index = None
        self.detail_box.setPlainText("임무를 선택하면 상세 정보가 표시됩니다.")
        self._render_map()
    # ----------------------------------------------------------- Detail panes
    def _update_input_detail(self, mission: Optional[dict[str, Any]]) -> None:
        if not mission:
            self.detail_box.setPlainText("임무가 선택되지 않았습니다.")
            return

        detail = mission.get("missionDetail") or {}
        linked_entries = self.entries_by_input.get(int(mission.get("inputMissionID") or -1), [])

        def format_coords(items: Iterable[dict[str, Any]]) -> str:
            lines = []
            for coord in items or []:
                lat, lon = coord.get("latitude"), coord.get("longitude")
                if lat is None or lon is None:
                    continue
                alt = coord.get("altitude", "N/A")
                lines.append(f"  - ({lat:.6f}, {lon:.6f}, alt={alt})")
            return "\n".join(lines) or "  - 없음"

        text = (
            f"InputMissionID: {mission.get('inputMissionID')}\n"
            f"타입: {mission.get('inputMissionType')}\n"
            f"완료 여부: {mission.get('isDone')}\n"
            f"연결된 개별 임무: {len(linked_entries)}\n"
        )
        if linked_entries:
            text += "\n".join(
                f"  - AC{entry.aircraft_id or '?'} / IMP {entry.individual_package_id} "
                f"/ IM {entry.individual_mission_id} / Path {entry.path_id or '-'}"
                for entry in linked_entries
            )
        else:
            text += "  - 없음"

        lines_text = "\n\n선 좌표:\n" + "\n".join(
            f"선 {idx + 1} (폭 {line.get('width', '-')})\n{format_coords(line.get('coordinateList') or [])}"
            for idx, line in enumerate(detail.get("lineList") or [])
        ) if detail.get("lineList") else "\n\n선 좌표:\n  - 없음"

        areas_text = "\n\n면 좌표:\n" + "\n".join(
            f"면 {idx + 1} ({'Hole' if area.get('isHole') else 'Outer'})\n{format_coords(area.get('coordinateList') or [])}"
            for idx, area in enumerate(detail.get("areaList") or [])
        ) if detail.get("areaList") else "\n\n면 좌표:\n  - 없음"

        points_text = "\n\n포인트 좌표:\n" + format_coords(detail.get("coordinateList"))

        self.detail_box.setPlainText(text + points_text + lines_text + areas_text)

    def _update_individual_detail(self, entry: ConnectedEntry) -> None:
        path_info = self.paths_data.get(entry.path_id) if entry.path_id else None
        lines = [
            f"IndividualMissionID: {entry.individual_mission_id}",
            f"InputMissionID: {entry.input_mission_id}",
            f"A/C ID: {entry.aircraft_id or '-'} (IMP {entry.individual_package_id})",
            f"MissionPlan IDs: {', '.join(str(pid) for pid in entry.plan_ids) or '-'}",
            f"PathID: {entry.path_id or '-'} ({'경로 있음' if path_info else '경로 없음'})",
            f"완료 여부: {entry.is_done}",
        ]

        detail = entry.info or {}
        lines.append(f"IndividualMissionType: {detail.get('individualMissionType', '-')}")
        lines.append(f"PatternType: {detail.get('patternType', '-')}")
        if detail.get("targetID"):
            lines.append(f"TargetID: {detail.get('targetID')}")

        lines.append("\n포인트 좌표:\n" + self._format_coords(detail.get("coordinateList")))

        if detail.get("lineList"):
            lines.append("\n선 좌표:")
            for idx, line in enumerate(detail.get("lineList") or []):
                lines.append(
                    f"  선 {idx + 1} (폭 {line.get('width', '-')})\n{self._format_coords(line.get('coordinateList'))}"
                )
        else:
            lines.append("\n선 좌표:\n  - 없음")

        if detail.get("areaList"):
            lines.append("\n면 좌표:")
            for idx, area in enumerate(detail.get("areaList") or []):
                label = "Hole" if area.get("isHole") else "Outer"
                lines.append(f"  면 {idx + 1} ({label})\n{self._format_coords(area.get('coordinateList'))}")
        else:
            lines.append("\n면 좌표:\n  - 없음")

        if path_info and path_info.coordinates:
            lines.append(f"\nFlightPath ({len(path_info.coordinates)} waypoint):")
            for idx, (lat, lon, alt) in enumerate(path_info.coordinates[:20], start=1):
                lines.append(f"  WP{idx}: ({lat:.6f}, {lon:.6f}, alt={alt})")
            if len(path_info.coordinates) > 20:
                lines.append("  ...")

        self.detail_box.setPlainText("\n".join(lines))
    # ----------------------------------------------------------- Map helpers
    def _render_blank_map(self) -> None:
        blank_map = folium.Map(location=[37.5665, 126.978], zoom_start=7, tiles="OpenStreetMap")
        blank_map.get_root().html.add_child(
            folium.Element("<div style='padding:8px;font-weight:bold;'>InputMissionPlan 파일을 불러오세요.</div>")
        )
        self._write_map(blank_map)

    def _render_map(self) -> None:
        if not self.package_data:
            self._render_blank_map()
            return

        missions = self.package_data.get("inputMissionList") or []
        coords = self._gather_all_coordinates(missions)
        if coords:
            avg_lat = sum(lat for lat, _ in coords) / len(coords)
            avg_lon = sum(lon for _, lon in coords) / len(coords)
        else:
            avg_lat, avg_lon = 37.5665, 126.978

        fmap = folium.Map(location=[avg_lat, avg_lon], tiles="OpenStreetMap", zoom_start=8)
        if coords:
            sw = [min(lat for lat, _ in coords), min(lon for _, lon in coords)]
            ne = [max(lat for lat, _ in coords), max(lon for _, lon in coords)]
            fmap.fit_bounds([sw, ne])

        focus_entry = self._current_individual_entry()
        focus_input_id = self._current_focus_id or (focus_entry.input_mission_id if focus_entry else None)

        self._draw_input_missions(fmap, missions, focus_input_id)
        self._draw_individual_missions(fmap, focus_entry, focus_input_id)
        self._draw_flight_paths(fmap, focus_entry)
        self._attach_legend(fmap, missions, focus_input_id, focus_entry)
        self._write_map(fmap)

    def _draw_input_missions(
        self,
        fmap: folium.Map,
        missions: list[dict[str, Any]],
        focus_input_id: Optional[int],
    ) -> None:
        for mission in missions:
            mission_id = mission.get("inputMissionID")
            detail = mission.get("missionDetail") or {}
            color = self._color_for_mission(mission_id)
            highlight = focus_input_id is None or mission_id == focus_input_id
            tone = color if highlight else "#B0B4B8"
            opacity = 0.9 if highlight else 0.32
            linked = len(self.entries_by_input.get(int(mission_id or -1), []))
            tooltip = (
                f"Input Mission {mission_id} (type {mission.get('inputMissionType')})"
                f"<br>linked individual missions: {linked}"
            )
            self._draw_geometry(detail, fmap, tone, opacity, tooltip)
    def _draw_individual_missions(
        self,
        fmap: folium.Map,
        focus_entry: Optional[ConnectedEntry],
        focus_input_id: Optional[int],
    ) -> None:
        if not self.connected_entries:
            return

        for entry in self.connected_entries:
            color = self._color_for_aircraft(entry.aircraft_id)
            if focus_entry:
                highlight = entry is focus_entry
            elif focus_input_id is not None and entry.input_mission_id is not None:
                highlight = int(entry.input_mission_id) == int(focus_input_id)
            else:
                highlight = True

            tone = color if highlight else "#8b939c"
            opacity = 0.9 if highlight else 0.25
            tooltip = (
                f"AC {entry.aircraft_id or '-'} | IMP {entry.individual_package_id}"
                f"<br>IM {entry.individual_mission_id} / Input {entry.input_mission_id}"
                f"<br>Plan IDs: {', '.join(str(pid) for pid in entry.plan_ids) or '-'}"
                f"<br>Path {entry.path_id or '-'}"
            )
            self._draw_geometry(entry.info or {}, fmap, tone, opacity, tooltip)

    def _draw_flight_paths(self, fmap: folium.Map, focus_entry: Optional[ConnectedEntry]) -> None:
        if not self.paths_data:
            return

        focus_path_id = focus_entry.path_id if focus_entry else None

        for path_id, fp_entry in self.paths_data.items():
            coords = [(lat, lon) for lat, lon, _ in fp_entry.coordinates]
            if len(coords) < 2:
                continue
            color = self._color_for_aircraft(fp_entry.aircraft_id)
            if focus_path_id:
                highlight = path_id == focus_path_id
            elif focus_entry:
                highlight = path_id == focus_entry.path_id
            else:
                highlight = True

            tone = color if highlight else "#9da3aa"
            opacity = 0.8 if highlight else 0.25
            tooltip = (
                f"FlightPath {path_id} | AC {fp_entry.aircraft_id or '-'}"
                f"<br>Waypoints: {len(fp_entry.coordinates)}"
            )
            folium.PolyLine(
                locations=coords,
                color=tone,
                weight=4 if highlight else 2,
                opacity=opacity,
                dash_array="6,6",
                tooltip=tooltip,
            ).add_to(fmap)

    def _draw_geometry(
        self,
        detail: dict[str, Any],
        fmap: folium.Map,
        color: str,
        opacity: float,
        tooltip: str,
    ) -> None:
        emphasize = opacity >= 0.8
        radius = 7 if emphasize else 5
        weight = 6 if emphasize else 3

        for point in detail.get("coordinateList") or []:
            lat = point.get("latitude")
            lon = point.get("longitude")
            if lat is None or lon is None:
                continue
            folium.CircleMarker(
                location=(lat, lon),
                radius=radius,
                color=color,
                fill=True,
                fill_color=color,
                fill_opacity=opacity,
                tooltip=tooltip,
            ).add_to(fmap)

        for line in detail.get("lineList") or []:
            coords = [
                (coord.get("latitude"), coord.get("longitude"))
                for coord in line.get("coordinateList") or []
                if coord.get("latitude") is not None and coord.get("longitude") is not None
            ]
            if len(coords) < 2:
                continue
            folium.PolyLine(
                locations=coords,
                color=color,
                weight=weight,
                opacity=opacity,
                tooltip=f"{tooltip}<br>폭 {line.get('width', '-')}",
            ).add_to(fmap)

        for area in detail.get("areaList") or []:
            coords = [
                (coord.get("latitude"), coord.get("longitude"))
                for coord in area.get("coordinateList") or []
                if coord.get("latitude") is not None and coord.get("longitude") is not None
            ]
            if len(coords) < 3:
                continue
            folium.Polygon(
                locations=coords,
                color=color,
                weight=2 if emphasize else 1,
                fill=True,
                fill_color=color,
                fill_opacity=0.25 if emphasize else 0.1,
                tooltip=tooltip,
            ).add_to(fmap)

    def _attach_legend(
        self,
        fmap: folium.Map,
        missions: list[dict[str, Any]],
        focus_input_id: Optional[int],
        focus_entry: Optional[ConnectedEntry],
    ) -> None:
        if not missions and not self._aircraft_colors:
            return

        mission_rows = []
        for mission in missions:
            mission_id = mission.get("inputMissionID")
            color = self._color_for_mission(mission_id)
            label = f"ID {mission_id}"
            if focus_input_id is not None and mission_id == focus_input_id:
                label += " ★"
            mission_rows.append(f"<li><span style='background:{color};'></span>{label}</li>")

        aircraft_rows = []
        for aircraft_id, color in sorted(self._aircraft_colors.items()):
            label = f"AC {aircraft_id}"
            if focus_entry and aircraft_id == focus_entry.aircraft_id:
                label += " ★"
            aircraft_rows.append(f"<li><span style='background:{color};'></span>{label}</li>")

        mission_block = (
            "<strong>Input Missions</strong><ul>"
            + "".join(mission_rows)
            + "</ul>"
            if mission_rows
            else ""
        )
        aircraft_block = (
            "<strong>Aircraft Layers</strong><ul>"
            + "".join(aircraft_rows)
            + "</ul>"
            if aircraft_rows
            else ""
        )

        legend_content = mission_block + aircraft_block
        if not legend_content:
            return

        legend_html = f"""
        {{% macro html(this, kwargs) %}}
        <style>
        .mission-legend {{
            position: fixed;
            bottom: 30px;
            left: 30px;
            z-index: 9999;
            background: rgba(255,255,255,0.95);
            padding: 10px 14px;
            border-radius: 8px;
            box-shadow: 0 2px 6px rgba(0,0,0,0.25);
            font-size: 12px;
            max-height: 280px;
            overflow-y: auto;
        }}
        .mission-legend ul {{
            list-style: none;
            margin: 4px 0 10px 0;
            padding: 0;
        }}
        .mission-legend li {{
            display: flex;
            align-items: center;
            margin-bottom: 4px;
        }}
        .mission-legend li span {{
            display: inline-block;
            width: 12px;
            height: 12px;
            border-radius: 50%;
            margin-right: 6px;
        }}
        </style>
        <div class="mission-legend">
            {legend_content}
        </div>
        {{% endmacro %}}
        """
        macro = MacroElement()
        macro._template = Template(legend_html)
        fmap.get_root().add_child(macro)

    def _write_map(self, fmap: folium.Map) -> None:
        fmap.save(self._map_html)
        html = self._map_html.read_text(encoding="utf-8")
        base_url = QUrl.fromLocalFile(str(self._map_dir))
        self.map_view.setHtml(html, base_url)
    # --------------------------------------------------------------- Utilities
    def _color_for_mission(self, mission_id: Optional[int]) -> str:
        if mission_id is None:
            return "#333333"
        if mission_id not in self._mission_color_map:
            self._mission_color_map[mission_id] = next(self._mission_color_cycle)
        return self._mission_color_map[mission_id]

    def _color_for_aircraft(self, aircraft_id: Optional[int]) -> str:
        if aircraft_id is None:
            return "#4a5562"
        if aircraft_id not in self._aircraft_colors:
            self._aircraft_colors[aircraft_id] = next(self._aircraft_color_cycle)
        return self._aircraft_colors[aircraft_id]

    def _gather_all_coordinates(self, missions: list[dict[str, Any]]) -> list[tuple[float, float]]:
        coords: list[tuple[float, float]] = []
        for mission in missions:
            detail = mission.get("missionDetail") or {}
            coords.extend(self._collect_geometry_coords(detail))
        for entry in self.connected_entries:
            coords.extend(self._collect_geometry_coords(entry.info or {}))
        for fp_entry in self.paths_data.values():
            coords.extend((lat, lon) for lat, lon, _ in fp_entry.coordinates)
        return coords

    def _collect_geometry_coords(self, detail: dict[str, Any]) -> list[tuple[float, float]]:
        coords: list[tuple[float, float]] = []
        for coord in detail.get("coordinateList") or []:
            lat, lon = coord.get("latitude"), coord.get("longitude")
            if lat is not None and lon is not None:
                coords.append((lat, lon))
        for line in detail.get("lineList") or []:
            for coord in line.get("coordinateList") or []:
                lat, lon = coord.get("latitude"), coord.get("longitude")
                if lat is not None and lon is not None:
                    coords.append((lat, lon))
        for area in detail.get("areaList") or []:
            for coord in area.get("coordinateList") or []:
                lat, lon = coord.get("latitude"), coord.get("longitude")
                if lat is not None and lon is not None:
                    coords.append((lat, lon))
        return coords

    def _find_mission(self, mission_id: Optional[int]) -> Optional[dict[str, Any]]:
        if mission_id is None or not self.package_data:
            return None
        for mission in self.package_data.get("inputMissionList") or []:
            if mission.get("inputMissionID") == mission_id:
                return mission
        return None

    def _current_individual_entry(self) -> Optional[ConnectedEntry]:
        if self._current_individual_index is None:
            return None
        if 0 <= self._current_individual_index < len(self.connected_entries):
            return self.connected_entries[self._current_individual_index]
        return None

    def _format_coords(self, coords: Optional[Iterable[dict[str, Any]]]) -> str:
        lines = []
        for coord in coords or []:
            lat, lon = coord.get("latitude"), coord.get("longitude")
            if lat is None or lon is None:
                continue
            alt = coord.get("altitude", "N/A")
            lines.append(f"  - ({lat:.6f}, {lon:.6f}, alt={alt})")
        return "\n".join(lines) or "  - 없음"

    def _detect_db_root(self, file_path: Path) -> Path:
        """Try to locate the scenario root that contains mission artifacts."""
        candidates: list[Path] = []
        parent = file_path.parent
        candidates.append(parent)
        candidates.extend(parent.parents[:4])  # walk up a few levels

        for candidate in candidates:
            if not candidate:
                continue
            base = candidate
            if candidate.name.lower() in {
                "inputmissionplan",
                "missionplan",
                "individualmissionplan",
                "flightpath",
            }:
                base = candidate.parent
            if self._looks_like_db_root(base):
                return base
        return file_path.parent

    def _resolve_default_dir(self) -> Path:
        """Prefer the Logs base_root from current_scenario.json, fallback to database path."""
        fallback = (PROJECT_ROOT / "database" / "InputMissionPlan").resolve()
        scenario_file = PROJECT_ROOT / "current_scenario.json"
        try:
            with scenario_file.open("r", encoding="utf-8") as fp:
                data = json.load(fp)
            base_root = data.get("base_root")
            if base_root:
                logs_dir = Path(base_root).expanduser()
                if logs_dir.exists():
                    return logs_dir.resolve()
        except Exception:
            pass
        return fallback

    def _looks_like_db_root(self, directory: Path) -> bool:
        try:
            directory = directory.resolve()
        except Exception:
            return False
        return any((directory / name).exists() for name in [
            "InputMissionPlan",
            "MissionPlan",
            "IndividualMissionPlan",
            "FlightPath",
        ])

    def _display_path(self, path: Optional[Path]) -> str:
        if not path:
            return "-"
        try:
            return path.resolve().as_posix()
        except Exception:
            return str(path).replace("\\", "/")

    # ----------------------------------------------------------- Entry points
    def _current_individual_entry_id(self) -> Optional[int]:
        entry = self._current_individual_entry()
        return entry.individual_mission_id if entry else None


def main() -> None:
    app = QApplication.instance() or QApplication(sys.argv)
    viewer = MissionPlanVisualizer()
    viewer.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
