"""DivisionPlannerWindow -- main application window with all pipeline logic."""
from __future__ import annotations

import copy
import itertools
import json
import math
import os
import sys
import threading
import traceback
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import split as geom_split, unary_union
from modules.mission_planning.MissionPlanner.runtime_settings import (
    apply_runtime_camera_adjusted_fov_deg,
    apply_runtime_camera_adjusted_search_speed,
    fov_db_path,
    format_runtime_camera_fov_adjustment_log,
    get_runtime_camera_coverage_scale,
    read_fov_db_rows_from_path,
)

from ._constants import (
    MODE_IDLE, MODE_DRAW_AREA, MODE_DRAW_LINE, MODE_LINE_WIDTH_PENDING,
    MODE_MISSION_READY, MODE_PLACE_UAV, MODE_SET_UAV_HEADING, MODE_RESULT_READY,
    MISSION_AREA, MISSION_LINE,
    _R, _ORIGIN_LAT, _ORIGIN_LON, _DEFAULT_ALT,
    _UAV_IDS, UAV_COLORS,
    _INITIAL_HALF_SPAN_M,
    _MISSION_PLANNER_DIR, _PROJECT_ROOT,
    TURN_RADIUS_BY_SPEED_MPS, TURN_PREVIEW_SPEED_MPS, TURN_PREVIEW_RADIUS_M,
    TURN_PREVIEW_BANK_DEG, TURN_PREVIEW_HORIZON_S,
    ASSIGNMENT_PATH_ARC_STEP_S, ASSIGNMENT_PATH1_ANGLE_WEIGHT, ASSIGNMENT_PATH1_TURN_TIME_WEIGHT,
    STAGE2_GRID_SIZE_SMALL_M, STAGE2_GRID_SIZE_MEDIUM_M, STAGE2_GRID_SIZE_LARGE_M,
    STAGE2_GRID_BOUND_SMALL_M, STAGE2_GRID_BOUND_MEDIUM_M,
    STAGE2_MIN_CELL_AREA_RATIO, STAGE2_DEFAULT_AREA_RATE_M2PS,
    STAGE2_ANCHOR_BLEND, STAGE2_MAX_SWATH_WIDTH_M,
    STAGE2_SMOOTH_BUFFER_RATIO, STAGE2_SIMPLIFY_RATIO, STAGE2_SIMPLIFY_MIN_M,
    STAGE2_PAIR_RELAX_BUFFER_RATIO, STAGE2_PAIR_SIMPLIFY_RATIO, STAGE2_OVERLAP_BUFFER_RATIO,
    NEXT_COLLAB_SWEEP_STEP_RATIO, NEXT_COLLAB_ENTRY_TPRIME_TARGET_SEP_RATIO,
    NEXT_COLLAB_ENTRY_TPRIME_RATIO_SCALE, NEXT_COLLAB_AREA_PATH0_TRIGGER_SEP_M,
    NEXT_COLLAB_AREA_PATH0_TARGET_SEP_RATIO, NEXT_COLLAB_TAKEOVER_FIRST_STEP_RATIO,
    _turn_radius_for_speed,
    _cached_fov_db_rows, _largest_sep_covering_db_row_for_width,
    _prepare_legacy_missionplanner_path,
    run_split_pipeline, review_overflow_areas,
    review_assigned_areas_local, assign_split_result_by_takeover_distance,
    build_0302_packages_from_split_with_lah, save_0302_packages,
    build_0303_0304_from_0302_packages, save_0303_plans, save_0304_plans,
    calculate_expected_velocity, generate_expected_paths,
    run_milp_scheduling,
    PROFILE_DEFAULT, apply_logic_type_decider,
    get_runtime_area_auto_fov_from_db, get_runtime_area_review_max_segment_m,
    get_runtime_float, get_runtime_int, get_runtime_str,
    SplitPiece, SplitRunResult,
)
from ._geo_utils import (
    local_xy_to_llh, llh_to_local_xy, meters_to_coord, coord_to_xy, coords_to_xy,
    _distance, _dedupe_points, normalize_area_points, corridor_polygon_xy, centroid_xy,
    _qcolor, _uav_color, _format_coord,
    _undirected_angle_deg, _bearing_deg_from_xy,
)
from ._canvas_state import CanvasState
from ._planning_canvas import PlanningCanvas
from ._gantt_chart import GanttChartWidget


_FovCacheRows = Tuple[Tuple[Tuple[str, float], ...], ...]
_FovCachePayload = Tuple[_FovCacheRows, Tuple[float, ...]]
_FOV_DB_WINDOW_CACHE_LOCK = threading.Lock()
_FOV_DB_WINDOW_CACHE: Dict[Tuple[str, int, int], _FovCachePayload] = {}


def _clone_fov_db_cache_rows(
    cached: _FovCachePayload,
) -> Tuple[List[Dict[str, float]], List[float]]:
    row_items, widths = cached
    return [dict(row) for row in row_items], [float(width) for width in widths]


def _pack_fov_db_cache_rows(
    rows: Sequence[Dict[str, float]],
    widths: Sequence[float],
) -> _FovCachePayload:
    packed_rows = tuple(tuple(sorted((str(key), float(value)) for key, value in row.items())) for row in rows)
    packed_widths = tuple(float(width) for width in widths)
    return packed_rows, packed_widths


class DivisionPlannerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Division Test Mission Planner")
        self.resize(1460, 980)

        self.state = CanvasState(line_width_m=300.0)
        self._selected_uav_count = 1
        self._cmpk_payload: Optional[Dict[str, Any]] = None
        self._mrpk_payload: Optional[Dict[str, Any]] = None
        self._fov_db_rows_cache: Optional[List[Dict[str, float]]] = None
        self._fov_db_widths_cache: Optional[List[float]] = None
        self._mid_line_no_split_active = False

        self._build_ui()
        self._apply_style()
        self._sync_canvas()
        self._refresh_ui()

    def _turn_radius_scale_value(self) -> float:
        try:
            scale = float(getattr(self, "_turn_radius_scale", 0.0) or 0.0)
        except Exception:
            scale = 0.0
        if scale <= 0.0:
            scale = float(get_runtime_float("next_collab_turn_radius_scale", 1.20))
        return max(0.1, float(scale))

    def _default_turn_radius_m(self) -> float:
        return float(TURN_PREVIEW_RADIUS_M) * float(self._turn_radius_scale_value())

    def _turn_radius_for_speed_m(self, speed_mps: float) -> float:
        return float(_turn_radius_for_speed(speed_mps)) * float(self._turn_radius_scale_value())

    def _bearing_from_points_xy(
        self,
        start_xy: Tuple[float, float],
        end_xy: Tuple[float, float],
    ) -> Optional[float]:
        dx = float(end_xy[0]) - float(start_xy[0])
        dy = float(end_xy[1]) - float(start_xy[1])
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None
        return float((math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0)

    def _piece_bearing_deg(self, piece: SplitPiece) -> Optional[float]:
        data = piece.data if isinstance(piece.data, dict) else {}
        for key in ("phaseMoveBearing_deg", "bearing_deg", "bearingIn_deg", "bearingFromPrev"):
            value = data.get(key)
            try:
                if value is None:
                    continue
                return float(value) % 360.0
            except Exception:
                continue

        for key in ("Centerline", "coordinateList", "rawCoordinateList"):
            coords = coords_to_xy(data.get(key, []))
            if len(coords) < 2:
                continue
            bearing = self._bearing_from_points_xy(coords[0], coords[-1])
            if bearing is not None:
                return bearing
        return None

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(14)

        left_panel = QFrame()
        left_panel.setObjectName("LeftPanel")
        left_panel.setMaximumWidth(390)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(18, 18, 18, 18)
        left_layout.setSpacing(12)
        layout.addWidget(left_panel, 0)

        title = QLabel("단일 임무 분할 / 경로 계획")
        title.setObjectName("PanelTitle")
        left_layout.addWidget(title)

        subtitle = QLabel(
            f"기준 원점 LLA: {_ORIGIN_LAT:.6f}, {_ORIGIN_LON:.6f}\n"
            "초기 가시 범위: 동/서/남/북 각 2km"
        )
        subtitle.setObjectName("MutedLabel")
        subtitle.setWordWrap(True)
        left_layout.addWidget(subtitle)

        step1 = QGroupBox("1. 임무 형상 입력")
        step1_layout = QVBoxLayout(step1)
        step1_layout.setSpacing(10)
        self.btn_start_area = QPushButton("영역 입력 시작")
        self.btn_start_area.clicked.connect(self._start_area_input)
        self.btn_start_line = QPushButton("Line 입력 시작")
        self.btn_start_line.clicked.connect(self._start_line_input)
        self.btn_undo = QPushButton("마지막 점 취소")
        self.btn_undo.clicked.connect(self._undo_last_point)
        self.btn_reset_view = QPushButton("뷰 초기화")
        self.btn_reset_view.clicked.connect(self._reset_view)
        self.btn_reset_all = QPushButton("전체 초기화")
        self.btn_reset_all.clicked.connect(self._reset_all)

        row1 = QHBoxLayout()
        row1.addWidget(self.btn_start_area)
        row1.addWidget(self.btn_start_line)
        step1_layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(self.btn_undo)
        row2.addWidget(self.btn_reset_view)
        step1_layout.addLayout(row2)
        step1_layout.addWidget(self.btn_reset_all)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        self.spin_line_width = QDoubleSpinBox()
        self.spin_line_width.setRange(10.0, 5_000.0)
        self.spin_line_width.setDecimals(1)
        self.spin_line_width.setSingleStep(10.0)
        self.spin_line_width.setSuffix(" m")
        self.spin_line_width.setValue(self.state.line_width_m)
        self.spin_line_width.valueChanged.connect(self._on_line_width_changed)
        form.addRow("Line 폭", self.spin_line_width)
        step1_layout.addLayout(form)

        self.lbl_mission_state = QLabel("-")
        self.lbl_mission_state.setWordWrap(True)
        self.lbl_mission_state.setObjectName("StatusLabel")
        step1_layout.addWidget(self.lbl_mission_state)
        left_layout.addWidget(step1)

        step2 = QGroupBox("2. UAV 대수 선택")
        step2_layout = QVBoxLayout(step2)
        step2_layout.setSpacing(10)
        self.cmb_uav_count = QComboBox()
        self.cmb_uav_count.addItems(["1", "2", "3"])
        self.cmb_uav_count.currentIndexChanged.connect(self._on_uav_count_changed)
        self.btn_confirm_uav_count = QPushButton("다음")
        self.btn_confirm_uav_count.clicked.connect(self._confirm_uav_count)
        step2_layout.addWidget(self.cmb_uav_count)
        step2_layout.addWidget(self.btn_confirm_uav_count)
        self.lbl_uav_count = QLabel("-")
        self.lbl_uav_count.setWordWrap(True)
        self.lbl_uav_count.setObjectName("StatusLabel")
        step2_layout.addWidget(self.lbl_uav_count)
        left_layout.addWidget(step2)

        step3 = QGroupBox("3. UAV 위치 입력 / 계획 실행")
        step3_layout = QVBoxLayout(step3)
        step3_layout.setSpacing(10)
        self.btn_start_uav_input = QPushButton("UAV 위치 입력 시작")
        self.btn_start_uav_input.clicked.connect(self._start_uav_input)
        self.btn_run_area_division = QPushButton("Area Division Run")
        self.btn_run_area_division.clicked.connect(self._run_area_division)
        self.btn_mid_line_generation = QPushButton("Mid Line Generation")
        self.btn_mid_line_generation.clicked.connect(self._generate_mid_lines)
        self.btn_make_path_0 = QPushButton("Make Path - 0")
        self.btn_make_path_0.clicked.connect(self._make_path_0)
        self.btn_make_sweep = QPushButton("Make Sweep")
        self.btn_make_sweep.clicked.connect(self._make_sweep)
        self.btn_make_new_area = QPushButton("Make New Area")
        self.btn_make_new_area.clicked.connect(self._make_new_area)
        self.btn_mid_line_generation_2 = QPushButton("Mid Line Generation 2")
        self.btn_mid_line_generation_2.clicked.connect(self._generate_mid_lines_2)
        self.btn_assignment_path_1 = QPushButton("Assignment - Path 1")
        self.btn_assignment_path_1.clicked.connect(self._assignment_path_1)
        self.btn_make_waypoint = QPushButton("Make Waypoint")
        self.btn_make_waypoint.clicked.connect(self._make_waypoint)
        self.btn_assignment_next_mission = QPushButton("Assignment - Next Mission")
        self.btn_assignment_next_mission.clicked.connect(self._assignment_next_mission)
        self.btn_make_path_2 = QPushButton("Make Path - 2")
        self.btn_make_path_2.clicked.connect(self._make_path_2)
        self.btn_make_sweep_2 = QPushButton("Make Sweep 2")
        self.btn_make_sweep_2.clicked.connect(self._make_sweep_2)
        self.btn_check_visibility = QPushButton("Check Visibility")
        self.btn_check_visibility.clicked.connect(self._check_visibility)
        self.btn_stage2_area_division = QPushButton("Stage 2 Area Division")
        self.btn_stage2_area_division.clicked.connect(self._run_stage2_area_division)
        self.btn_run_plan = QPushButton("실제 파이프라인 실행")
        self.btn_run_plan.clicked.connect(self._run_planning)
        ratio_form = QFormLayout()
        ratio_form.setLabelAlignment(Qt.AlignLeft)
        self.stage2_ratio_spins: Dict[int, QDoubleSpinBox] = {}
        for aid in _UAV_IDS:
            spin = QDoubleSpinBox()
            spin.setRange(0.0, 100.0)
            spin.setDecimals(2)
            spin.setSingleStep(0.1)
            spin.setValue(1.0)
            self.stage2_ratio_spins[int(aid)] = spin
            ratio_form.addRow(f"UAV{aid} ratio", spin)
        step3_layout.addWidget(self.btn_start_uav_input)
        step3_layout.addWidget(self.btn_run_area_division)
        step3_layout.addWidget(self.btn_mid_line_generation)
        step3_layout.addWidget(self.btn_make_path_0)
        step3_layout.addWidget(self.btn_make_sweep)
        step3_layout.addWidget(self.btn_make_new_area)
        step3_layout.addWidget(self.btn_mid_line_generation_2)
        step3_layout.addWidget(self.btn_assignment_path_1)
        step3_layout.addWidget(self.btn_make_waypoint)
        step3_layout.addWidget(self.btn_assignment_next_mission)
        step3_layout.addWidget(self.btn_make_path_2)
        step3_layout.addWidget(self.btn_make_sweep_2)
        self.lbl_uav_positions = QLabel("-")
        self.lbl_uav_positions.setWordWrap(True)
        self.lbl_uav_positions.setObjectName("StatusLabel")
        step3_layout.addWidget(self.lbl_uav_positions)
        left_layout.addWidget(step3)

        view_box = QGroupBox("4. UAV 시각화")
        view_layout = QVBoxLayout(view_box)
        view_layout.setSpacing(8)
        self.chk_view_uav4 = QCheckBox("UAV4")
        self.chk_view_uav5 = QCheckBox("UAV5")
        self.chk_view_uav6 = QCheckBox("UAV6")
        self.chk_view_uav4.setStyleSheet(f"color: {UAV_COLORS[4]}; font-weight: 600;")
        self.chk_view_uav5.setStyleSheet(f"color: {UAV_COLORS[5]}; font-weight: 600;")
        self.chk_view_uav6.setStyleSheet(f"color: {UAV_COLORS[6]}; font-weight: 600;")
        for chk in (self.chk_view_uav4, self.chk_view_uav5, self.chk_view_uav6):
            chk.setChecked(True)
            chk.toggled.connect(lambda _checked: self._refresh_ui())
            view_layout.addWidget(chk)
        left_layout.addWidget(view_box)

        result_box = QGroupBox("결과")
        result_layout = QVBoxLayout(result_box)
        result_layout.setSpacing(8)
        self.txt_result = QPlainTextEdit()
        self.txt_result.setReadOnly(True)
        result_layout.addWidget(self.txt_result, 1)
        left_layout.addWidget(result_box, 1)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)
        self.canvas = PlanningCanvas()
        self.canvas.worldLeftClicked.connect(self._on_canvas_left_click)
        self.canvas.worldRightClicked.connect(self._on_canvas_right_click)
        self.canvas.hoverTextChanged.connect(self.statusBar().showMessage)
        self.gantt_chart = GanttChartWidget()
        right_layout.addWidget(self.canvas, 1)
        right_layout.addWidget(self.gantt_chart, 0)
        layout.addWidget(right_panel, 1)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow { background: #eef3f9; }
            #LeftPanel {
                background: #f8fbff;
                border: 1px solid #d6dee9;
                border-radius: 12px;
            }
            #PanelTitle { font-size: 20px; font-weight: 700; color: #0f172a; }
            #MutedLabel { color: #475569; }
            #StatusLabel {
                color: #1e293b;
                background: #edf2f7;
                border-radius: 8px;
                padding: 8px 10px;
            }
            QGroupBox {
                font-weight: 700;
                color: #0f172a;
                border: 1px solid #d3dde8;
                border-radius: 10px;
                margin-top: 10px;
                background: #ffffff;
            }
            QGroupBox::title { left: 12px; padding: 0 4px; }
            QPushButton {
                min-height: 22px;
                border: 1px solid #c7d2df;
                border-radius: 7px;
                background: #f8fafc;
                color: #0f172a;
                padding: 1px 6px;
            }
            QPushButton:hover { background: #eff6ff; }
            QPushButton:disabled { color: #8a99ab; background: #e7edf4; }
            QComboBox, QDoubleSpinBox, QPlainTextEdit {
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                background: #ffffff;
                padding: 6px;
            }
            #GanttChart {
                background: #eef3f9;
                border: none;
            }
            """
        )

    def _sync_canvas(self) -> None:
        self.canvas.set_state(
            CanvasState(
                mode=self.state.mode,
                mission_kind=self.state.mission_kind,
                draft_points_xy=list(self.state.draft_points_xy),
                mission_points_xy=list(self.state.mission_points_xy),
                line_width_m=float(self.state.line_width_m),
                line_width_pending=bool(self.state.line_width_pending),
                uav_positions_xy=list(self.state.uav_positions_xy),
                uav_heading_deg=list(self.state.uav_heading_deg),
                uav_ids=list(self.state.uav_ids),
                split_result=self.state.split_result,
                expected_paths=list(self.state.expected_paths),
                assignment_path_rows=list(self.state.assignment_path_rows),
                flight_plans_0303=list(self.state.flight_plans_0303),
                flight_plans_0304=list(self.state.flight_plans_0304),
                visibility_segments=list(self.state.visibility_segments),
                mid_line_segments=list(self.state.mid_line_segments),
                tangent_checks=list(self.state.tangent_checks),
                next_mission_rows=list(self.state.next_mission_rows),
                show_next_mission_circles=bool(self.state.show_next_mission_circles),
                show_turn_overlays=bool(self.state.show_turn_overlays),
                visible_uav_ids=self._visible_uav_ids(),
            )
        )
        self.gantt_chart.set_paths(self.state.expected_paths, self._visible_uav_ids())

    def _uav_inputs_complete(self) -> bool:
        if not self.state.uav_ids:
            return False
        if len(self.state.uav_positions_xy) != len(self.state.uav_ids):
            return False
        if len(self.state.uav_heading_deg) != len(self.state.uav_positions_xy):
            return False
        return all(heading is not None for heading in self.state.uav_heading_deg)

    def _pending_uav_heading_index(self) -> Optional[int]:
        if self.state.mode != MODE_SET_UAV_HEADING:
            return None
        if not self.state.uav_positions_xy:
            return None
        idx = len(self.state.uav_positions_xy) - 1
        if idx < 0:
            return None
        if idx >= len(self.state.uav_heading_deg):
            return idx
        return idx if self.state.uav_heading_deg[idx] is None else None

    def _pop_last_uav_input(self) -> bool:
        if not self.state.uav_positions_xy:
            return False
        self.state.uav_positions_xy.pop()
        if self.state.uav_heading_deg:
            self.state.uav_heading_deg.pop()
        self.state.mode = MODE_PLACE_UAV if self.state.uav_ids else MODE_MISSION_READY
        return True

    def _refresh_ui(self) -> None:
        mission_ready = bool(self.state.mission_points_xy)
        line_mode = self.state.mission_kind == MISSION_LINE
        can_confirm_uav = mission_ready
        uav_count_locked = bool(self.state.uav_ids)
        expected_uav_count = len(self.state.uav_ids) if self.state.uav_ids else self._selected_uav_count
        uav_done = self._uav_inputs_complete() if expected_uav_count > 0 else False
        mid_line_ready = bool(self.state.split_result is not None and bool(self.state.split_result.pieces))
        no_split_mode_ready = self._mid_line_no_split_mode()
        make_new_area_ready = bool(self.state.mid_line_segments)
        mid_line2_ready = bool(
            self._has_make_new_area_result()
            and self.state.mid_line_segments
            and not no_split_mode_ready
            and any(int(piece.assigned_uav or 0) > 0 for piece in (self.state.split_result.pieces if self.state.split_result else []))
        )
        assignment_path_1_ready = bool(
            mid_line2_ready
            and any(
                isinstance(row, dict) and isinstance(row.get("stage2Centers"), list) and row.get("stage2Centers")
                for row in self.state.mid_line_segments
            )
        )
        make_waypoint_ready = bool(self.state.assignment_path_rows) and not no_split_mode_ready
        make_path_2_ready = bool(
            self.state.assignment_path_rows
            and not no_split_mode_ready
            and any(
                isinstance(row, dict) and str(row.get("source", "") or "") == "next_mission"
                for row in self.state.assignment_path_rows
            )
        )
        visibility_ready = bool(
            self.state.split_result is not None
            and any(int(piece.assigned_uav or 0) > 0 for piece in self.state.split_result.pieces)
        )
        stage2_ready = bool(
            mission_ready
            and (self.state.mission_kind == MISSION_AREA)
            and self.state.split_result is not None
            and any(int(piece.assigned_uav or 0) > 0 for piece in self.state.split_result.pieces)
        )

        self.spin_line_width.setEnabled(line_mode)
        self.cmb_uav_count.setEnabled(can_confirm_uav)
        self.btn_confirm_uav_count.setEnabled(can_confirm_uav)
        self.btn_start_uav_input.setEnabled(uav_count_locked)
        self.btn_run_area_division.setEnabled(
            mission_ready and (self.state.mission_kind == MISSION_AREA) and uav_count_locked and uav_done
        )
        self.btn_mid_line_generation.setEnabled(mid_line_ready)
        self.btn_mid_line_generation.setVisible(mid_line_ready)
        self.btn_make_path_0.setEnabled(no_split_mode_ready)
        self.btn_make_path_0.setVisible(no_split_mode_ready)
        make_sweep_ready = bool(
            no_split_mode_ready
            and self.state.expected_paths
            and any(
                isinstance(r, dict) and str(r.get("source", "") or "") == "make_path_0"
                for r in self.state.expected_paths
            )
        )
        self.btn_make_sweep.setEnabled(make_sweep_ready)
        self.btn_make_sweep.setVisible(make_sweep_ready)
        self.btn_make_new_area.setEnabled(make_new_area_ready and not no_split_mode_ready)
        self.btn_make_new_area.setVisible(make_new_area_ready and not no_split_mode_ready)
        self.btn_mid_line_generation_2.setEnabled(mid_line2_ready)
        self.btn_mid_line_generation_2.setVisible(mid_line2_ready)
        self.btn_assignment_path_1.setEnabled(assignment_path_1_ready and not no_split_mode_ready)
        self.btn_assignment_path_1.setVisible(assignment_path_1_ready and not no_split_mode_ready)
        self.btn_make_waypoint.setEnabled(make_waypoint_ready)
        self.btn_make_waypoint.setVisible(make_waypoint_ready)
        self.btn_make_path_2.setEnabled(make_path_2_ready)
        self.btn_make_path_2.setVisible(make_path_2_ready)
        make_sweep_2_ready = bool(
            self.state.expected_paths
            and not no_split_mode_ready
            and any(
                isinstance(r, dict) and str(r.get("source", "") or "") == "make_path_2"
                for r in self.state.expected_paths
            )
        )
        self.btn_make_sweep_2.setEnabled(make_sweep_2_ready)
        self.btn_make_sweep_2.setVisible(make_sweep_2_ready)
        next_mission_ready = bool(
            self.state.expected_paths
            and not no_split_mode_ready
            and any(
                isinstance(r, dict) and r.get("waypointEndXY") is not None
                for r in self.state.expected_paths
            )
        )
        self.btn_assignment_next_mission.setEnabled(next_mission_ready)
        self.btn_assignment_next_mission.setVisible(next_mission_ready)
        self.btn_stage2_area_division.setEnabled(False)
        self.btn_stage2_area_division.setVisible(False)
        self.btn_check_visibility.setEnabled(False)
        self.btn_check_visibility.setVisible(False)
        self.btn_run_plan.setEnabled(False)
        self.btn_run_plan.setVisible(False)
        self.btn_undo.setEnabled(bool(self.state.draft_points_xy) or bool(self.state.uav_positions_xy))
        for aid, spin in self.stage2_ratio_spins.items():
            spin.setEnabled(int(aid) in self.state.uav_ids)
            spin.setVisible(False)

        if not mission_ready:
            self.lbl_mission_state.setText(
                "영역: 좌클릭으로 점 입력 후 우클릭으로 닫기\n"
                "Line: 좌클릭으로 점 입력 후 우클릭 -> 폭 입력 상태 -> 다시 우클릭으로 확정"
            )
        elif self.state.mission_kind == MISSION_AREA:
            self.lbl_mission_state.setText(f"영역 입력 완료\n점 개수: {len(self.state.mission_points_xy)}개")
        else:
            self.lbl_mission_state.setText(
                f"Line 입력 완료\n점 개수: {len(self.state.mission_points_xy)}개 / 폭: {self.state.line_width_m:.1f}m"
            )

        if not mission_ready:
            self.lbl_uav_count.setText("임무 형상이 확정되면 UAV 대수를 1~3대 중 선택할 수 있습니다.")
        elif not uav_count_locked:
            self.lbl_uav_count.setText(
                f"현재 선택값: {self._selected_uav_count}대\n다음을 누르면 UAV 입력 단계로 넘어갑니다."
            )
        else:
            self.lbl_uav_count.setText(f"확정 UAV: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}")

        if not uav_count_locked:
            self.lbl_uav_positions.setText("UAV 대수를 먼저 확정하세요.")
        elif not self.state.uav_positions_xy:
            self.lbl_uav_positions.setText(
                f"입력 대기: {len(self.state.uav_ids)}대의 위치와 heading이 필요합니다.\n"
                "지도에서 한 번 클릭해 위치를 두고, 가이드 선을 확인한 뒤 한 번 더 클릭해 heading을 확정하세요."
            )
        else:
            lines = []
            for idx, point_xy in enumerate(self.state.uav_positions_xy):
                aid = self.state.uav_ids[idx] if idx < len(self.state.uav_ids) else idx + 1
                heading = self.state.uav_heading_deg[idx] if idx < len(self.state.uav_heading_deg) else None
                heading_text = f" | HDG {float(heading):.1f} deg" if heading is not None else " | heading 대기"
                lines.append(f"UAV{aid}: {_format_coord(point_xy)}{heading_text}")
            if len(self.state.uav_positions_xy) < len(self.state.uav_ids):
                lines.append(f"남은 UAV 위치 입력: {len(self.state.uav_ids) - len(self.state.uav_positions_xy)}대")
            pending_idx = self._pending_uav_heading_index()
            if pending_idx is not None and pending_idx < len(self.state.uav_ids):
                lines.append(f"UAV{self.state.uav_ids[pending_idx]}: 지도에서 한 번 더 클릭해 heading을 확정하세요.")
            self.lbl_uav_positions.setText("\n".join(lines))

        self._sync_canvas()

    def _append_result(self, text: str) -> None:
        existing = self.txt_result.toPlainText().strip()
        self.txt_result.setPlainText(f"{existing}\n{text}".strip())
        self.txt_result.verticalScrollBar().setValue(self.txt_result.verticalScrollBar().maximum())

    def _set_result(self, text: str) -> None:
        self.txt_result.setPlainText(text)

    def _clear_plan_result(self) -> None:
        self.state.split_result = None
        self.state.expected_paths = []
        self.state.assignment_path_rows = []
        self.state.mission_check_rows = []
        self.state.flight_plans_0303 = []
        self.state.flight_plans_0304 = []
        self.state.visibility_segments = []
        self.state.mid_line_segments = []
        self.state.tangent_checks = []
        self.state.show_turn_overlays = True
        self._mid_line_no_split_active = False
        self._cmpk_payload = None
        self._mrpk_payload = None

    def _reset_all(self) -> None:
        self.state = CanvasState(line_width_m=float(self.spin_line_width.value()))
        self._selected_uav_count = int(self.cmb_uav_count.currentText())
        self._clear_plan_result()
        self._set_result("")
        self._refresh_ui()

    def _reset_view(self) -> None:
        self.canvas.reset_view()

    def _output_root(self) -> Path:
        root = Path(__file__).resolve().parent / "output"
        root.mkdir(parents=True, exist_ok=True)
        return root

    def _area_mode(self) -> str:
        try:
            return str(get_runtime_str("area_sweep_mode", "parallel") or "parallel").strip().lower()
        except Exception:
            return "parallel"

    def _review_max_segment_m(self) -> float:
        try:
            return float(get_runtime_area_review_max_segment_m(500.0))
        except Exception:
            return 500.0

    def _uav_plan_mode(self) -> str:
        try:
            raw = str(get_runtime_str("uav_plan_mode", "normal") or "normal").strip().lower()
        except Exception:
            raw = "normal"
        return "dub_path" if raw == "dub_path" else "normal"

    def _flow_mode(self) -> str:
        raw = str(os.environ.get("DIVISION_TEST_FLOW_MODE", "initial") or "initial").strip().lower()
        return "replan" if raw == "replan" else "initial"

    def _is_replan_flow(self) -> bool:
        return self._flow_mode() == "replan"

    def _assignment_summary_text(self, split_result: SplitRunResult) -> str:
        counts: Dict[int, int] = {}
        for piece in split_result.pieces:
            aid = int(piece.assigned_uav or 0)
            if aid <= 0:
                continue
            counts[aid] = int(counts.get(aid, 0)) + 1
        if not counts:
            return "-"
        return ", ".join(f"UAV{aid}={count}" for aid, count in sorted(counts.items()))

    def _target_group_name(self, target_label: str) -> str:
        normalized = str(target_label or "").strip().upper()
        if len(normalized) >= 2 and normalized[0] in {"F", "N"}:
            return normalized[1:]
        return normalized

    def _target_group_key(self, piece_index: int, target_label: str) -> str:
        normalized_group = self._target_group_name(target_label)
        if normalized_group:
            return f"P{int(piece_index)}:{normalized_group}"
        return f"P{int(piece_index)}"

    def _clone_path_row_with_waypoint_end_label(
        self,
        row: Dict[str, Any],
        waypoint_end_label: str,
    ) -> Dict[str, Any]:
        cloned = dict(row)
        marker_rows = cloned.get("markerRows")
        if isinstance(marker_rows, list):
            new_marker_rows: List[Dict[str, Any]] = []
            for marker in marker_rows:
                if not isinstance(marker, dict):
                    continue
                marker_copy = dict(marker)
                if str(marker_copy.get("kind", "") or "") == "waypoint_end":
                    marker_copy["label"] = str(waypoint_end_label)
                new_marker_rows.append(marker_copy)
            cloned["markerRows"] = new_marker_rows

        timeline_rows = cloned.get("timelineRows")
        if isinstance(timeline_rows, list):
            new_timeline_rows: List[Dict[str, Any]] = []
            for item in timeline_rows:
                if not isinstance(item, dict):
                    continue
                item_copy = dict(item)
                if str(item_copy.get("kind", "") or "") == "waypoint_end":
                    item_copy["label"] = str(waypoint_end_label)
                new_timeline_rows.append(item_copy)
            cloned["timelineRows"] = new_timeline_rows

        cloned["waypointEndLabel"] = str(waypoint_end_label)
        return cloned

    def _wp_only_display_row(self, row: Dict[str, Any]) -> Dict[str, Any]:
        cleaned = dict(row)
        for key in (
            "entryLineEndpointsXY",
            "entryTPrimeXY",
            "centerLineXY",
            "sweepLineListXY",
        ):
            cleaned.pop(key, None)
        return cleaned

    def _has_make_new_area_result(self) -> bool:
        if self.state.split_result is None:
            return False
        for piece in self.state.split_result.pieces:
            data = piece.data if isinstance(piece.data, dict) else {}
            review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
            if bool(review.get("makeNewAreaApplied")):
                return True
        return False

    def _mid_line_split_counts(
        self,
        rows: Optional[Sequence[Dict[str, Any]]] = None,
    ) -> Tuple[int, int]:
        total_count = 0
        no_split_count = 0
        source_rows = self.state.mid_line_segments if rows is None else rows
        for row in source_rows:
            if not isinstance(row, dict):
                continue
            total_count += 1
            width_start_m = float(
                row.get("widthStartM", 0.0)
                or row.get("maxWidthM", 0.0)
                or row.get("widthM", 0.0)
                or 0.0
            )
            db_max_width_m = float(row.get("dbMaxWidthM", 0.0) or 0.0)
            if width_start_m > 0.0 and db_max_width_m > 0.0 and width_start_m <= db_max_width_m + 1e-6:
                no_split_count += 1
        return total_count, no_split_count

    def _mid_line_no_split_mode(
        self,
        rows: Optional[Sequence[Dict[str, Any]]] = None,
    ) -> bool:
        total_count, no_split_count = self._mid_line_split_counts(rows)
        return total_count > 0 and total_count == no_split_count

    def _ensure_split_branch_allowed(self, action_title: str) -> bool:
        if self._mid_line_no_split_mode():
            QMessageBox.information(
                self,
                action_title,
                "현재 Mid Line Generation 결과는 DB width 범위 안입니다. split 흐름 대신 Make Path - 0을 사용하세요.",
            )
            return False
        return True

    def _dominant_edge_bearing_deg(
        self,
        points_xy: Sequence[Tuple[float, float]],
    ) -> Optional[float]:
        if len(points_xy) < 2:
            return None
        best_bearing: Optional[float] = None
        best_len_m = 0.0
        for idx in range(len(points_xy)):
            start_xy = points_xy[idx]
            end_xy = points_xy[(idx + 1) % len(points_xy)]
            seg_len_m = _distance(start_xy, end_xy)
            if seg_len_m <= best_len_m:
                continue
            bearing_deg = self._bearing_from_points_xy(start_xy, end_xy)
            if bearing_deg is None:
                continue
            best_len_m = float(seg_len_m)
            best_bearing = float(bearing_deg)
        return best_bearing

    def _mid_line_reference_bearing_deg(
        self,
        pieces: Sequence[SplitPiece],
    ) -> Optional[float]:
        for piece in sorted(pieces, key=lambda row: int(row.piece_index or 0)):
            bearing_deg = self._piece_bearing_deg(piece)
            if bearing_deg is not None:
                return float(bearing_deg)
            coords = coords_to_xy((piece.data or {}).get("coordinateList", []))
            bearing_deg = self._dominant_edge_bearing_deg(coords)
            if bearing_deg is not None:
                return float(bearing_deg)
        return self._dominant_edge_bearing_deg(self.state.mission_points_xy)

    def _fov_db_widths_m(self) -> List[float]:
        rows = self._fov_db_rows()
        self._fov_db_widths_cache = sorted(
            {float(row.get("width", 0.0) or 0.0) for row in rows if float(row.get("width", 0.0) or 0.0) > 0.0}
        )
        return list(self._fov_db_widths_cache)

    def _fov_db_rows(self) -> List[Dict[str, float]]:
        db_path = fov_db_path()
        try:
            resolved = db_path.resolve()
            stat = resolved.stat()
            db_sig = (str(resolved), int(stat.st_mtime_ns), int(stat.st_size))
        except Exception:
            db_sig = None

        if (
            isinstance(self._fov_db_rows_cache, list)
            and getattr(self, "_fov_db_cache_sig", None) == db_sig
        ):
            return [dict(row) for row in self._fov_db_rows_cache]
        if db_sig is not None:
            with _FOV_DB_WINDOW_CACHE_LOCK:
                cached = _FOV_DB_WINDOW_CACHE.get(db_sig)
            if cached is not None:
                rows_cached, widths_cached = _clone_fov_db_cache_rows(cached)
                self._fov_db_rows_cache = rows_cached
                self._fov_db_widths_cache = widths_cached
                self._fov_db_cache_sig = db_sig
                return [dict(row) for row in self._fov_db_rows_cache]

        widths: List[float] = []
        rows: List[Dict[str, float]] = []
        try:
            for row in read_fov_db_rows_from_path(db_path):
                try:
                    width_m = float(row.get("width", 0.0) or 0.0)
                    sep_m = float(row.get("sep", 0.0) or 0.0)
                    vel = float(row.get("vel", 0.0) or 0.0)
                    fov = float(row.get("fov", 0.0) or 0.0)
                    foot_m = float(row.get("foot", 0.0) or 0.0)
                except Exception:
                    continue
                if width_m > 0.0:
                    widths.append(float(width_m))
                    rows.append(
                        {
                            "width": float(width_m),
                            "sep": float(sep_m),
                            "vel": float(vel),
                            "fov": float(fov),
                            "foot": float(foot_m),
                        }
                    )
        except Exception as _db_err:
            print(f"[FOV_DB] LOAD FAILED: {_db_err} | path={db_path}")
            widths = []
            rows = []

        if not rows:
            fallback_rows = _cached_fov_db_rows()
            rows = [
                {
                    "width": float(row[0]),
                    "sep": float(row[1]),
                    "vel": float(row[2]),
                    "fov": float(row[3]),
                    "foot": float(row[4]) if len(row) > 4 else 0.0,
                }
                for row in fallback_rows
                if float(row[0]) > 0.0
            ]
            widths = [float(row["width"]) for row in rows if float(row.get("width", 0.0) or 0.0) > 0.0]

        print(f"[FOV_DB] loaded {len(rows)} rows, {len(set(widths))} unique widths, path={db_path}")
        self._fov_db_rows_cache = sorted(rows, key=lambda item: (float(item.get("width", 0.0) or 0.0), float(item.get("sep", 0.0) or 0.0)))
        self._fov_db_widths_cache = sorted(set(widths))
        self._fov_db_cache_sig = db_sig
        if db_sig is not None:
            with _FOV_DB_WINDOW_CACHE_LOCK:
                stale_keys = [key for key in _FOV_DB_WINDOW_CACHE if key[0] == db_sig[0] and key != db_sig]
                for key in stale_keys:
                    _FOV_DB_WINDOW_CACHE.pop(key, None)
                _FOV_DB_WINDOW_CACHE[db_sig] = _pack_fov_db_cache_rows(
                    self._fov_db_rows_cache,
                    self._fov_db_widths_cache,
                )
        return [dict(row) for row in self._fov_db_rows_cache]

    def _covering_db_width_m(self, width_m: float) -> Optional[float]:
        target_m = float(width_m)
        if target_m <= 0.0:
            return None
        for db_width_m in self._fov_db_widths_m():
            if float(db_width_m) + 1e-6 >= target_m:
                return float(db_width_m)
        return None

    def _covering_db_row(self, width_m: float) -> Optional[Dict[str, float]]:
        rows = self._fov_db_rows()
        target_m = max(0.0, float(width_m))
        if target_m <= 0.0 or not rows:
            return None
        for row in rows:
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m:
                return dict(row)
        return dict(rows[-1])

    def _largest_sep_covering_db_row(self, width_m: float) -> Optional[Dict[str, float]]:
        rows = self._fov_db_rows()
        target_m = max(0.0, float(width_m))
        if target_m <= 0.0 or not rows:
            return None

        candidates = [
            dict(row)
            for row in rows
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
        ]
        if candidates:
            return min(
                candidates,
                key=lambda row: (
                    -float(row.get("sep", 0.0) or 0.0),
                    float(row.get("width", 0.0) or 0.0),
                    -float(row.get("fov", 0.0) or 0.0),
                ),
            )

        return max(
            (dict(row) for row in rows),
            key=lambda row: (
                float(row.get("width", 0.0) or 0.0),
                float(row.get("sep", 0.0) or 0.0),
            ),
        )

    def _next_collab_sweep_step_ratio(self) -> float:
        value = float(get_runtime_float("next_collab_sweep_step_ratio", NEXT_COLLAB_SWEEP_STEP_RATIO))
        return max(0.05, min(1.0, value))

    def _next_collab_entry_tprime_ratio_scale(self) -> float:
        value = float(
            get_runtime_float(
                "next_collab_entry_tprime_ratio_scale",
                NEXT_COLLAB_ENTRY_TPRIME_RATIO_SCALE,
            )
        )
        return max(0.10, min(5.0, value))

    def _next_collab_entry_tprime_target_sep_ratio(self) -> float:
        value = float(
            get_runtime_float(
                "next_collab_entry_tprime_target_sep_ratio",
                NEXT_COLLAB_ENTRY_TPRIME_TARGET_SEP_RATIO,
            )
        )
        return max(0.05, min(1.0, value * self._next_collab_entry_tprime_ratio_scale()))

    def _next_collab_area_path0_target_sep_ratio(self) -> float:
        value = float(
            get_runtime_float(
                "next_collab_area_path0_target_sep_ratio",
                NEXT_COLLAB_AREA_PATH0_TARGET_SEP_RATIO,
            )
        )
        return max(0.05, min(1.0, value))

    def _next_collab_area_path0_trigger_sep_m(self, db_max_sep_m: float) -> float:
        value = float(
            get_runtime_float(
                "next_collab_area_path0_trigger_sep_m",
                NEXT_COLLAB_AREA_PATH0_TRIGGER_SEP_M,
            )
        )
        if value <= 0.0:
            return max(0.0, float(db_max_sep_m))
        return max(1.0, min(10000.0, value))

    def _next_collab_takeover_first_step_ratio(self) -> float:
        value = float(
            get_runtime_float(
                "next_collab_takeover_first_step_ratio",
                NEXT_COLLAB_TAKEOVER_FIRST_STEP_RATIO,
            )
        )
        return max(0.10, min(1.0, value))

    def _next_collab_area_fov_scale(self) -> float:
        value = float(get_runtime_float("next_collab_area_fov_scale", 1.0))
        return max(0.10, min(5.0, value))

    def _next_collab_area_imaging_sep_target_m(self) -> float:
        value = float(get_runtime_float("next_collab_area_imaging_sep_target_m", 300.0))
        return max(1.0, min(5000.0, value))

    def _next_collab_area_imaging_sep_ref_m(self, sep_m: float | None) -> float:
        target_m = self._next_collab_area_imaging_sep_target_m()
        try:
            sep_val = float(sep_m or 0.0)
        except Exception:
            sep_val = 0.0
        if sep_val <= 0.0:
            return float(target_m)
        return min(float(sep_val), float(target_m))

    def _fov_db_smaller_fov_steps(self) -> int:
        try:
            steps = int(get_runtime_int("fov_db_smaller_fov_steps", 3))
        except Exception:
            steps = 3
        return max(0, min(20, int(steps)))

    def _area_fov_db_smaller_fov_steps(self) -> int:
        try:
            steps = int(get_runtime_int("area_fov_db_smaller_fov_steps", 1))
        except Exception:
            steps = 1
        return max(0, min(20, int(steps)))

    def _runtime_fov_db_sep_safety_factor(self) -> float:
        try:
            factor = float(get_runtime_float("fov_db_sep_safety_factor", 1.7))
        except Exception:
            factor = 1.7
        if factor <= 0.0:
            factor = 1.0
        return float(factor)

    def _next_collab_db_sep_requirement_m(self, sep_m: float | None) -> float:
        try:
            sep_val = float(sep_m or 0.0)
        except Exception:
            sep_val = 0.0
        return max(0.0, float(sep_val)) * float(self._runtime_fov_db_sep_safety_factor())

    def _next_collab_area_db_weights(self) -> Tuple[float, float, float]:
        width_weight = max(0.0, float(get_runtime_float("next_collab_line_db_width_weight", 0.30)))
        sep_weight = max(0.0, float(get_runtime_float("next_collab_line_db_sep_weight", 0.25)))
        fov_weight = max(0.0, float(get_runtime_float("next_collab_line_db_fov_weight", 0.45)))
        total = float(width_weight + sep_weight + fov_weight)
        if total <= 1e-9:
            return 0.30, 0.25, 0.45
        return (
            float(width_weight) / total,
            float(sep_weight) / total,
            float(fov_weight) / total,
        )

    @staticmethod
    def _normalized_unit_score(value: float, minimum: float, maximum: float, *, prefer_low: bool) -> float:
        if maximum - minimum <= 1e-9:
            return 1.0
        ratio = (float(value) - float(minimum)) / float(maximum - minimum)
        ratio = max(0.0, min(1.0, float(ratio)))
        return 1.0 - ratio if prefer_low else ratio

    def _next_collab_area_search_speed_scale(self) -> float:
        value = float(get_runtime_float("next_collab_area_search_speed_scale", 1.3))
        return max(0.10, min(5.0, value))

    def _next_collab_area_density_speed_scale(self) -> float:
        try:
            base_density = float(get_runtime_float("area_density_scale", 1.2))
        except Exception:
            base_density = 1.2
        try:
            next_density = float(get_runtime_float("next_collab_area_density_scale", max(base_density, 2.4)))
        except Exception:
            next_density = max(base_density, 2.4)
        if base_density <= 1e-6 or next_density <= 1e-6:
            return 1.0
        return max(1.0, min(5.0, float(next_density) / float(base_density)))

    def _next_collab_area_gsd_margin_ratio(self) -> float:
        try:
            value = float(get_runtime_float("next_collab_area_gsd_margin_ratio", 0.90))
        except Exception:
            value = 0.90
        return max(0.10, min(1.0, float(value)))

    def _next_collab_area_required_footprint_area_m2(self) -> Optional[float]:
        cached = getattr(self, "_next_collab_area_required_footprint_area_cache", None)
        if cached is not None:
            return cached
        path = _PROJECT_ROOT / "modules" / "monitoring" / "quality_monitor_settings.json"
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
        sr = payload.get("spatial_resolution") if isinstance(payload, dict) else {}
        if not isinstance(sr, dict):
            sr = {}

        def _positive_float(key: str, default: float) -> float:
            try:
                value = float(sr.get(key, default))
            except Exception:
                value = float(default)
            return max(0.01, float(value))

        def _positive_int(key: str, default: int) -> int:
            try:
                value = int(float(sr.get(key, default)))
            except Exception:
                value = int(default)
            return max(1, int(value))

        img_w_px = _positive_int("img_w_px", 1920)
        img_h_px = _positive_int("img_h_px", 1080)
        obj_w_m = _positive_float("obj_w_m", 6.0)
        obj_h_m = _positive_float("obj_h_m", 3.6)
        obj_min_px_x = _positive_int("obj_min_px_x", 38)
        obj_min_px_y = _positive_int("obj_min_px_y", 22)
        total_px = int(img_w_px) * int(img_h_px)
        if total_px <= 0 or obj_min_px_x <= 0 or obj_min_px_y <= 0:
            return None
        gamma = (float(obj_w_m) * float(obj_h_m)) / float(obj_min_px_x * obj_min_px_y)
        required_area = float(gamma) * float(total_px)
        if required_area <= 0.0:
            self._next_collab_area_required_footprint_area_cache = None
            return None
        result = float(required_area) * float(self._next_collab_area_gsd_margin_ratio())
        self._next_collab_area_required_footprint_area_cache = result
        return result

    def _next_collab_area_estimated_footprint_area_m2(
        self,
        *,
        sep_m: float,
        fov_deg: float,
    ) -> Optional[float]:
        foot_m = self._next_collab_area_footprint_m(float(sep_m), float(fov_deg))
        if foot_m is None or foot_m <= 0.0:
            return None
        # Use a square footprint as a conservative planner-side proxy. The
        # simulator computes the exact rectangle later from camera geometry.
        return float(foot_m) * float(foot_m)

    def _next_collab_area_manual_base_fov_deg(self) -> Optional[float]:
        auto_from_db = bool(get_runtime_area_auto_fov_from_db(True))
        if auto_from_db:
            return None
        area_mode = str(self._area_mode() or "").strip().lower()
        fov_key = "area_nadir_fov_deg" if area_mode in {"nadir", "directdown", "bf_nadir"} else "area_custom_fov_deg"
        manual_fov_deg = float(
            get_runtime_float(
                fov_key,
                get_runtime_float(
                    "area_custom_fov_deg",
                    0.0,
                ),
            )
        )
        return float(manual_fov_deg) if manual_fov_deg > 0.0 else None

    def _scale_next_collab_area_fov(self, value: Optional[float]) -> Optional[float]:
        if value is None:
            return None
        scaled = float(value) * float(self._next_collab_area_fov_scale())
        if scaled <= 0.0:
            return None
        adjusted = apply_runtime_camera_adjusted_fov_deg(
            scaled,
            context="NEXTCOLLAB AREA DB",
        )
        return float(adjusted) if adjusted > 0.0 else None

    def _scale_next_collab_area_foot(
        self,
        value: Optional[float],
        *,
        base_fov_deg: Optional[float] = None,
    ) -> Optional[float]:
        if value is None:
            return None
        scaled = float(value) * float(self._next_collab_area_fov_scale())
        if scaled <= 0.0:
            return None
        if base_fov_deg is not None and float(base_fov_deg) > 0.0:
            scaled *= float(
                get_runtime_camera_coverage_scale(
                    float(base_fov_deg) * float(self._next_collab_area_fov_scale())
                )
            )
        return float(scaled) if scaled > 0.0 else None

    def _next_collab_area_manual_fov_deg(self) -> Optional[float]:
        manual_fov_deg = self._next_collab_area_manual_base_fov_deg()
        if manual_fov_deg is None:
            return None
        adjusted = apply_runtime_camera_adjusted_fov_deg(
            manual_fov_deg,
            context="NEXTCOLLAB AREA MANUAL",
        )
        return float(adjusted) if adjusted > 0.0 else None

    def _next_collab_area_camera_base_fov_deg(self, value: Optional[float]) -> Optional[float]:
        manual_fov_deg = self._next_collab_area_manual_base_fov_deg()
        if manual_fov_deg is not None:
            return float(manual_fov_deg)
        if value is None:
            return None
        scaled = float(value) * float(self._next_collab_area_fov_scale())
        return float(scaled) if scaled > 0.0 else None

    def _append_next_collab_fov_adjust_log(
        self,
        lines: List[str],
        *,
        context: str,
        base_fov_deg: Optional[float],
        adjusted_fov_deg: Optional[float],
    ) -> None:
        if base_fov_deg is None or adjusted_fov_deg is None:
            return
        log_line = format_runtime_camera_fov_adjustment_log(
            float(base_fov_deg),
            float(adjusted_fov_deg),
            context=context,
        )
        if log_line:
            lines.append(log_line)

    def _next_collab_area_footprint_m(
        self,
        sep_m: Optional[float],
        fov_deg: Optional[float],
    ) -> Optional[float]:
        try:
            sep_val = float(sep_m)
            fov_val = float(fov_deg)
        except Exception:
            return None
        if sep_val <= 0.0 or fov_val <= 0.0:
            return None
        base = 2.0 * max(sep_val, 1.0) * math.tan(max(math.radians(fov_val) / 2.0, 1e-6))
        return float(base) if base > 0.0 else None

    def _next_collab_area_min_db_footprint_for_fov_m(
        self,
        db_fov_deg: Optional[float],
    ) -> Optional[float]:
        try:
            target_fov = float(db_fov_deg)
        except Exception:
            return None
        if target_fov <= 0.0:
            return None

        rows = [
            dict(row)
            for row in self._fov_db_rows()
            if float(row.get("fov", 0.0) or 0.0) > 0.0
            and float(row.get("foot", 0.0) or 0.0) > 0.0
        ]
        if not rows:
            return None

        # FOV values in the DB are discrete camera modes. Keep the match tight so
        # a 2.4 deg row never borrows the footprint from an adjacent FOV mode.
        tolerance = max(0.01, abs(target_fov) * 1e-4)
        matching = [
            row
            for row in rows
            if abs(float(row.get("fov", 0.0) or 0.0) - target_fov) <= tolerance
        ]
        if not matching:
            return None
        return float(
            min(
                float(row.get("foot", 0.0) or 0.0)
                for row in matching
            )
        )

    def _next_collab_area_spacing_footprint_m(
        self,
        sep_m: Optional[float],
        fov_deg: Optional[float],
        *,
        db_fov_deg: Optional[float] = None,
    ) -> Optional[float]:
        computed_foot = self._next_collab_area_footprint_m(sep_m, fov_deg)
        if self._next_collab_area_manual_base_fov_deg() is None:
            min_db_foot = self._next_collab_area_min_db_footprint_for_fov_m(db_fov_deg)
            if min_db_foot is not None and min_db_foot > 0.0:
                scaled_min_foot = self._scale_next_collab_area_foot(
                    min_db_foot,
                    base_fov_deg=db_fov_deg,
                )
                if scaled_min_foot is not None and scaled_min_foot > 0.0:
                    if computed_foot is not None and computed_foot > 0.0:
                        return min(float(computed_foot), float(scaled_min_foot))
                    return float(scaled_min_foot)
        return computed_foot

    def _next_collab_area_sweep_spacing_m(
        self,
        sep_m: Optional[float],
        fov_deg: Optional[float],
        *,
        db_fov_deg: Optional[float] = None,
    ) -> Optional[float]:
        footprint_m = self._next_collab_area_spacing_footprint_m(
            sep_m,
            fov_deg,
            db_fov_deg=db_fov_deg,
        )
        if footprint_m is None or footprint_m <= 0.0:
            return None
        try:
            base_density = float(get_runtime_float("area_density_scale", 1.2))
        except Exception:
            base_density = 1.2
        try:
            density = float(get_runtime_float("next_collab_area_density_scale", max(base_density, 2.4)))
        except Exception:
            density = max(base_density, 2.4)
        density = max(float(density), 1e-6)
        return max(float(footprint_m) / float(density), 1.0)

    def _resolve_next_collab_area_runtime_fov_foot(
        self,
        resolved_fov: Optional[float],
        resolved_foot: Optional[float],
        *,
        sep_m: Optional[float],
    ) -> Tuple[Optional[float], Optional[float]]:
        manual_fov_deg = self._next_collab_area_manual_fov_deg()
        if manual_fov_deg is not None:
            return float(manual_fov_deg), self._next_collab_area_footprint_m(sep_m, manual_fov_deg)
        return (
            self._scale_next_collab_area_fov(resolved_fov),
            self._scale_next_collab_area_foot(resolved_foot, base_fov_deg=resolved_fov),
        )

    def _scale_next_collab_area_search_speed(
        self,
        value: Optional[float],
        *,
        base_fov_deg: Optional[float] = None,
    ) -> Optional[float]:
        if value is None:
            return None
        scaled = float(value) * float(self._next_collab_area_search_speed_scale())
        scaled *= float(self._next_collab_area_density_speed_scale())
        manual_base_fov_deg = self._next_collab_area_manual_base_fov_deg()
        if manual_base_fov_deg is not None and manual_base_fov_deg > 0.0:
            scaled = apply_runtime_camera_adjusted_search_speed(
                scaled,
                manual_base_fov_deg,
                adjusted_fov_deg=self._next_collab_area_manual_fov_deg(),
            )
        elif base_fov_deg is not None and float(base_fov_deg) > 0.0:
            local_base_fov_deg = float(base_fov_deg) * float(self._next_collab_area_fov_scale())
            scaled = apply_runtime_camera_adjusted_search_speed(
                scaled,
                local_base_fov_deg,
                adjusted_fov_deg=self._scale_next_collab_area_fov(base_fov_deg),
            )
        return float(scaled) if scaled > 0.0 else None

    def _next_collab_entry_tprime_db_row(self, width_m: float) -> Optional[Dict[str, float]]:
        target_m = max(0.0, float(width_m))
        if target_m <= 0.0:
            return None

        candidates = [
            dict(row)
            for row in self._fov_db_rows()
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
        ]
        if not candidates:
            fallback_rows = _cached_fov_db_rows()
            candidates = [
                {
                    "width": float(row[0]),
                    "sep": float(row[1]),
                    "vel": float(row[2]),
                    "fov": float(row[3]),
                    "foot": float(row[4]) if len(row) > 4 else 0.0,
                }
                for row in fallback_rows
                if float(row[0]) + 1e-6 >= target_m
            ]
        if not candidates:
            fallback_row = _largest_sep_covering_db_row_for_width(width_m)
            if isinstance(fallback_row, dict):
                return dict(fallback_row)
            return None

        max_sep_row = min(
            candidates,
            key=lambda row: (
                -float(row.get("sep", 0.0) or 0.0),
                float(row.get("width", 0.0) or 0.0),
                -float(row.get("fov", 0.0) or 0.0),
            ),
        )
        max_sep = float(max_sep_row.get("sep", 0.0) or 0.0)
        if max_sep <= 0.0:
            return dict(max_sep_row)

        target_sep = max_sep * float(self._next_collab_entry_tprime_target_sep_ratio())
        lower_or_equal = [
            dict(row)
            for row in candidates
            if float(row.get("sep", 0.0) or 0.0) <= target_sep + 1e-6
        ]
        if lower_or_equal:
            return max(
                lower_or_equal,
                key=lambda row: (
                    float(row.get("sep", 0.0) or 0.0),
                    -float(row.get("fov", 0.0) or 0.0),
                    -float(row.get("width", 0.0) or 0.0),
                ),
            )

        return min(
            candidates,
            key=lambda row: (
                abs(float(row.get("sep", 0.0) or 0.0) - target_sep),
                float(row.get("width", 0.0) or 0.0),
                -float(row.get("fov", 0.0) or 0.0),
            ),
        )

    def _next_collab_entry_tprime_target_sep_m(self, width_m: float) -> float:
        db_row = self._next_collab_entry_tprime_db_row(width_m)
        if not isinstance(db_row, dict):
            return 0.0
        sep_m = float(db_row.get("sep", 0.0) or 0.0)
        if sep_m <= 0.0:
            return 0.0
        return float(sep_m)

    def _next_collab_resolved_db_row(
        self,
        width_m: float,
        sep_m: float,
    ) -> Optional[Dict[str, float]]:
        target_m = max(0.0, float(width_m))
        limit_sep_m = self._next_collab_db_sep_requirement_m(float(sep_m))
        if target_m <= 0.0:
            return None

        candidates = [
            dict(row)
            for row in self._fov_db_rows()
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
        ]
        if not candidates:
            fallback_rows = _cached_fov_db_rows()
            candidates = [
                {
                    "width": float(row[0]),
                    "sep": float(row[1]),
                    "vel": float(row[2]),
                    "fov": float(row[3]),
                    "foot": float(row[4]) if len(row) > 4 else 0.0,
                }
                for row in fallback_rows
                if float(row[0]) + 1e-6 >= target_m
            ]
        if not candidates:
            return None

        matching: List[Dict[str, float]] = []
        if limit_sep_m > 0.0:
            matching = [
                dict(row)
                for row in candidates
                if float(row.get("sep", 0.0) or 0.0) >= limit_sep_m
            ]
        pool = matching or candidates
        width_weight, sep_weight, fov_weight = self._next_collab_area_db_weights()

        width_errors = [
            max(float(row.get("width", 0.0) or 0.0) - target_m, 0.0)
            for row in pool
        ]
        if limit_sep_m > 0.0 and matching:
            sep_errors = [
                max(float(row.get("sep", 0.0) or 0.0) - limit_sep_m, 0.0)
                for row in pool
            ]
        elif limit_sep_m > 0.0:
            sep_errors = [
                abs(float(row.get("sep", 0.0) or 0.0) - limit_sep_m)
                for row in pool
            ]
        else:
            sep_errors = [0.0 for _row in pool]
        fov_values = [float(row.get("fov", 0.0) or 0.0) for row in pool]

        min_width_error = min(width_errors) if width_errors else 0.0
        max_width_error = max(width_errors) if width_errors else 0.0
        min_sep_error = min(sep_errors) if sep_errors else 0.0
        max_sep_error = max(sep_errors) if sep_errors else 0.0
        min_fov = min(fov_values) if fov_values else 0.0
        max_fov = max(fov_values) if fov_values else 0.0

        scored_rows: List[Tuple[float, Dict[str, float], float, float]] = []
        for row, width_error, sep_error, fov_value in zip(pool, width_errors, sep_errors, fov_values):
            width_score = self._normalized_unit_score(
                float(width_error),
                float(min_width_error),
                float(max_width_error),
                prefer_low=True,
            )
            sep_score = self._normalized_unit_score(
                float(sep_error),
                float(min_sep_error),
                float(max_sep_error),
                prefer_low=True,
            )
            fov_score = self._normalized_unit_score(
                float(fov_value),
                float(min_fov),
                float(max_fov),
                prefer_low=False,
            )
            total_score = (
                (float(width_weight) * float(width_score))
                + (float(sep_weight) * float(sep_score))
                + (float(fov_weight) * float(fov_score))
            )
            scored_rows.append((float(total_score), dict(row), float(width_error), float(sep_error)))

        _best_score, best_row, best_width_error, best_sep_error = max(
            scored_rows,
            key=lambda item: (
                float(item[0]),
                -float(item[2]),
                -float(item[3]),
                float(item[1].get("fov", 0.0) or 0.0),
                float(item[1].get("sep", 0.0) or 0.0),
                float(item[1].get("vel", 0.0) or 0.0),
            ),
        )
        _ = best_width_error, best_sep_error
        return dict(best_row)

    def _next_collab_area_apply_smaller_fov_steps(
        self,
        base_row: Optional[Dict[str, float]],
        *,
        width_m: float,
        sep_m: float,
    ) -> Optional[Dict[str, float]]:
        if not isinstance(base_row, dict):
            return base_row
        steps = self._area_fov_db_smaller_fov_steps()
        if steps <= 0:
            return dict(base_row)
        try:
            base_fov = float(base_row.get("fov", 0.0) or 0.0)
        except Exception:
            base_fov = 0.0
        if base_fov <= 0.0:
            return dict(base_row)

        target_m = max(0.0, float(width_m))
        limit_sep_m = self._next_collab_db_sep_requirement_m(float(sep_m))
        if target_m <= 0.0:
            return dict(base_row)

        candidates = [
            dict(row)
            for row in self._fov_db_rows()
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
            and float(row.get("fov", 0.0) or 0.0) > 0.0
        ]
        if not candidates:
            return dict(base_row)

        pool = candidates
        if limit_sep_m > 0.0:
            matching = [
                dict(row)
                for row in candidates
                if float(row.get("sep", 0.0) or 0.0) >= limit_sep_m
            ]
            if matching:
                pool = matching

        lower_fovs = sorted(
            {
                round(float(row.get("fov", 0.0) or 0.0), 6)
                for row in pool
                if float(row.get("fov", 0.0) or 0.0) < base_fov - 1e-6
            },
            reverse=True,
        )
        if not lower_fovs:
            return dict(base_row)

        target_fov = float(lower_fovs[min(steps - 1, len(lower_fovs) - 1)])
        fov_rows = [
            dict(row)
            for row in pool
            if abs(float(row.get("fov", 0.0) or 0.0) - target_fov) <= 1e-6
        ]
        if not fov_rows:
            return dict(base_row)

        if limit_sep_m > 0.0 and any(float(row.get("sep", 0.0) or 0.0) >= limit_sep_m for row in fov_rows):
            eligible = [
                row
                for row in fov_rows
                if float(row.get("sep", 0.0) or 0.0) >= limit_sep_m
            ]
            return dict(
                min(
                    eligible,
                    key=lambda row: (
                        float(row.get("sep", 0.0) or 0.0) - limit_sep_m,
                        -float(row.get("vel", 0.0) or 0.0),
                        -float(row.get("width", 0.0) or 0.0),
                    ),
                )
            )

        return dict(
            min(
                fov_rows,
                key=lambda row: (
                    abs(float(row.get("sep", 0.0) or 0.0) - max(limit_sep_m, 0.0)),
                    -float(row.get("vel", 0.0) or 0.0),
                    -float(row.get("width", 0.0) or 0.0),
                ),
            )
        )

    def _next_collab_area_promote_fov_by_gsd_margin(
        self,
        selected_row: Optional[Dict[str, float]],
        *,
        width_m: float,
        sep_m: float,
    ) -> Optional[Dict[str, float]]:
        if not isinstance(selected_row, dict):
            return selected_row
        if self._next_collab_area_manual_base_fov_deg() is not None:
            return dict(selected_row)
        required_area_m2 = self._next_collab_area_required_footprint_area_m2()
        if required_area_m2 is None or required_area_m2 <= 0.0:
            return dict(selected_row)
        try:
            selected_fov = float(selected_row.get("fov", 0.0) or 0.0)
        except Exception:
            selected_fov = 0.0
        if selected_fov <= 0.0:
            return dict(selected_row)

        try:
            sep_ref_m = float(sep_m or 0.0)
        except Exception:
            sep_ref_m = 0.0
        if sep_ref_m <= 0.0:
            sep_ref_m = float(selected_row.get("sep", 0.0) or 0.0)
        if sep_ref_m <= 0.0:
            return dict(selected_row)

        current_area_m2 = self._next_collab_area_estimated_footprint_area_m2(
            sep_m=float(sep_ref_m),
            fov_deg=float(self._scale_next_collab_area_fov(selected_fov) or selected_fov),
        )
        if current_area_m2 is None or current_area_m2 > float(required_area_m2):
            return dict(selected_row)

        limit_sep_m = self._next_collab_db_sep_requirement_m(float(sep_m))
        db_rows = self._fov_db_rows()
        higher_fovs = sorted(
            {
                round(float(row.get("fov", 0.0) or 0.0), 6)
                for row in db_rows
                if float(row.get("fov", 0.0) or 0.0) > selected_fov + 1e-6
            }
        )
        if not higher_fovs:
            return dict(selected_row)

        safe_fovs: List[Tuple[float, float]] = []
        for fov in higher_fovs:
            effective_fov = float(self._scale_next_collab_area_fov(fov) or fov)
            area_m2 = self._next_collab_area_estimated_footprint_area_m2(
                sep_m=float(sep_ref_m),
                fov_deg=float(effective_fov),
            )
            if area_m2 is None or area_m2 > float(required_area_m2):
                continue
            safe_fovs.append((float(fov), float(area_m2)))
        if not safe_fovs:
            return dict(selected_row)

        max_safe_fov, best_area_m2 = max(safe_fovs, key=lambda item: float(item[0]))
        same_fov_rows = [
            dict(row)
            for row in db_rows
            if abs(float(row.get("fov", 0.0) or 0.0) - float(max_safe_fov)) <= 1e-6
        ]
        if not same_fov_rows:
            return dict(selected_row)
        if limit_sep_m > 0.0:
            sep_matching = [
                dict(row)
                for row in same_fov_rows
                if float(row.get("sep", 0.0) or 0.0) + 1e-6 >= limit_sep_m
            ]
            if sep_matching:
                same_fov_rows = sep_matching

        best_row = min(
            same_fov_rows,
            key=lambda row: (
                abs(float(row.get("sep", 0.0) or 0.0) - max(limit_sep_m, 0.0)),
                -float(row.get("vel", 0.0) or 0.0),
                -float(row.get("width", 0.0) or 0.0),
            ),
        )
        promoted = dict(best_row)
        promoted["gsdPromotedFromFov"] = float(selected_fov)
        promoted["gsdSepM"] = float(sep_ref_m)
        promoted["gsdEstimatedFootprintAreaM2"] = float(best_area_m2)
        promoted["gsdRequiredFootprintAreaM2"] = float(required_area_m2)
        return promoted

    def _next_collab_area_resolved_db_values(
        self,
        width_m: float,
        sep_m: float,
    ) -> Dict[str, Any]:
        def _positive_float(value: Any) -> Optional[float]:
            try:
                parsed = float(value or 0.0)
            except Exception:
                return None
            return float(parsed) if parsed > 0.0 else None

        try:
            imaging_sep_ref_m = float(sep_m or 0.0)
        except Exception:
            imaging_sep_ref_m = 0.0
        if imaging_sep_ref_m < 0.0:
            imaging_sep_ref_m = 0.0
        resolved_db_row = self._next_collab_resolved_db_row(width_m, imaging_sep_ref_m)
        resolved_db_row = self._next_collab_area_apply_smaller_fov_steps(
            resolved_db_row,
            width_m=float(width_m),
            sep_m=float(imaging_sep_ref_m),
        )
        resolved_db_row = self._next_collab_area_promote_fov_by_gsd_margin(
            resolved_db_row,
            width_m=float(width_m),
            sep_m=float(imaging_sep_ref_m),
        )
        resolved_fov: Optional[float] = None
        resolved_vel: Optional[float] = None
        resolved_foot: Optional[float] = None
        resolved_db_sep = 0.0
        resolved_db_width = 0.0

        if isinstance(resolved_db_row, dict):
            resolved_fov = _positive_float(resolved_db_row.get("fov"))
            resolved_vel = _positive_float(resolved_db_row.get("vel"))
            resolved_foot = _positive_float(resolved_db_row.get("foot"))
            try:
                resolved_db_sep = float(resolved_db_row.get("sep", 0.0) or 0.0)
            except Exception:
                resolved_db_sep = 0.0
            try:
                resolved_db_width = float(resolved_db_row.get("width", 0.0) or 0.0)
            except Exception:
                resolved_db_width = 0.0

        try:
            sep_ref_m = float(imaging_sep_ref_m)
        except Exception:
            sep_ref_m = 0.0
        if sep_ref_m <= 0.0:
            sep_ref_m = float(resolved_db_sep)

        resolved_fov_base = resolved_fov
        resolved_fov, resolved_foot = self._resolve_next_collab_area_runtime_fov_foot(
            resolved_fov,
            resolved_foot,
            sep_m=sep_ref_m,
        )
        if isinstance(resolved_db_row, dict) and float(resolved_db_row.get("gsdSepM", 0.0) or 0.0) > 0.0:
            resolved_foot = self._next_collab_area_spacing_footprint_m(
                float(resolved_db_row.get("gsdSepM", 0.0) or 0.0),
                resolved_fov,
                db_fov_deg=resolved_fov_base,
            )
        resolved_vel = self._scale_next_collab_area_search_speed(
            resolved_vel,
            base_fov_deg=resolved_fov_base,
        )
        resolved_camera_base_fov = self._next_collab_area_camera_base_fov_deg(resolved_fov_base)

        return {
            "dbRow": dict(resolved_db_row) if isinstance(resolved_db_row, dict) else None,
            "dbSepM": float(resolved_db_sep),
            "dbWidthM": float(resolved_db_width),
            "resolvedDbFovDeg": resolved_fov_base,
            "resolvedBaseFovDeg": resolved_camera_base_fov,
            "resolvedFovDeg": resolved_fov,
            "resolvedFootM": resolved_foot,
            "resolvedVelMps": resolved_vel,
            "gsdSepM": float(resolved_db_row.get("gsdSepM", 0.0) or 0.0) if isinstance(resolved_db_row, dict) else 0.0,
            "gsdPromotedFromFov": (
                float(resolved_db_row.get("gsdPromotedFromFov", 0.0) or 0.0)
                if isinstance(resolved_db_row, dict) and resolved_db_row.get("gsdPromotedFromFov") is not None
                else None
            ),
        }

    def _next_collab_resolved_foot_m(
        self,
        width_m: float,
        sep_m: float,
    ) -> float:
        resolved_row = self._next_collab_resolved_db_row(width_m, sep_m)
        if isinstance(resolved_row, dict):
            foot_m = float(resolved_row.get("foot", 0.0) or 0.0)
            if foot_m > 0.0:
                return float(foot_m)

        entry_row = self._next_collab_entry_tprime_db_row(width_m)
        if isinstance(entry_row, dict):
            foot_m = float(entry_row.get("foot", 0.0) or 0.0)
            if foot_m > 0.0:
                return float(foot_m)

        target_m = max(0.0, float(width_m))
        limit_sep_m = max(0.0, float(sep_m))
        rows = [dict(row) for row in self._fov_db_rows() if float(row.get("foot", 0.0) or 0.0) > 0.0]
        if not rows:
            return 0.0

        matching = [
            dict(row)
            for row in rows
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
            and float(row.get("sep", 0.0) or 0.0) + 1e-6 >= limit_sep_m
        ]
        if matching:
            best = min(
                matching,
                key=lambda row: (
                    float(row.get("width", 0.0) or 0.0) - target_m,
                    float(row.get("sep", 0.0) or 0.0) - limit_sep_m,
                    -float(row.get("fov", 0.0) or 0.0),
                ),
            )
            return float(best.get("foot", 0.0) or 0.0)

        width_matching = [
            dict(row)
            for row in rows
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
        ]
        if width_matching:
            best = min(
                width_matching,
                key=lambda row: (
                    float(row.get("width", 0.0) or 0.0) - target_m,
                    abs(float(row.get("sep", 0.0) or 0.0) - limit_sep_m),
                    -float(row.get("fov", 0.0) or 0.0),
                ),
            )
            return float(best.get("foot", 0.0) or 0.0)

        best = min(
            rows,
            key=lambda row: (
                abs(float(row.get("width", 0.0) or 0.0) - target_m),
                abs(float(row.get("sep", 0.0) or 0.0) - limit_sep_m),
                -float(row.get("fov", 0.0) or 0.0),
            ),
        )
        return float(best.get("foot", 0.0) or 0.0)

    def _mid_line_overlay_bundle(
        self,
        split_result: Optional[SplitRunResult],
    ) -> Tuple[float, List[Dict[str, Any]], List[str]]:
        if split_result is None or not split_result.pieces:
            raise ValueError("Mid line generation requires split polygons.")

        pieces = [
            piece
            for piece in sorted(split_result.pieces, key=lambda row: int(row.piece_index or 0))
            if len(coords_to_xy((piece.data or {}).get("coordinateList", []))) >= 3
        ]
        if not pieces:
            raise ValueError("Mid line generation requires split polygons.")

        reference_bearing_deg = self._mid_line_reference_bearing_deg(pieces)
        if reference_bearing_deg is None:
            raise ValueError("Unable to determine a reference bearing for the split pieces.")

        overlays: List[Dict[str, Any]] = []
        lines = [f"[MID] generation refBearing={float(reference_bearing_deg):.1f}deg"]
        for piece in pieces:
            coords_xy = coords_to_xy((piece.data or {}).get("coordinateList", []))
            geometry = self._mid_line_overlay_geometry(coords_xy, float(reference_bearing_deg))
            if geometry is None:
                continue
            aid = int(piece.assigned_uav or 0)
            overlay_row = {
                "pieceIndex": int(piece.piece_index or 0),
                "aircraftID": aid,
                "bearingDeg": float(reference_bearing_deg),
                **geometry,
            }
            split_boundary_points_xy = self._mid_line_split_boundary_points(coords_xy, overlay_row)
            if split_boundary_points_xy:
                overlay_row["splitBoundaryPointsXY"] = split_boundary_points_xy
            t0_preview = None
            uav_state = self._uav_state_for_aircraft(aid) if aid > 0 else None
            if uav_state is not None:
                origin_xy, heading_deg = uav_state
                t0_preview = self._mid_line_t0_preview(overlay_row, origin_xy, heading_deg)
                if isinstance(t0_preview, dict):
                    overlay_row.update(t0_preview)
                    self._apply_first_line_width_reference(overlay_row, t0_preview)
            overlay_row["widthStartM"] = float(
                overlay_row.get("widthStartM", 0.0)
                or overlay_row.get("maxWidthM", 0.0)
                or overlay_row.get("widthM", 0.0)
                or 0.0
            )
            overlay_row["sepStartM"] = float(
                overlay_row.get("t0ShapePointDistM", 0.0)
                or 0.0
            )
            overlays.append(overlay_row)
            max_width_m = float(overlay_row.get("maxWidthM", 0.0) or 0.0)
            left_width_m = float(overlay_row.get("maxWidthLeftM", 0.0) or 0.0)
            right_width_m = float(overlay_row.get("maxWidthRightM", 0.0) or 0.0)
            db_cover_width_m = float(overlay_row.get("dbCoverWidthM", 0.0) or 0.0)
            db_max_width_m = float(overlay_row.get("dbMaxWidthM", 0.0) or 0.0)
            sep_start_m = float(overlay_row.get("sepStartM", 0.0) or 0.0)
            width_start_m = float(overlay_row.get("widthStartM", 0.0) or 0.0)
            width_db_row = self._covering_db_row(width_start_m)
            width_db_width_m = float(width_db_row.get("width", 0.0) or 0.0) if isinstance(width_db_row, dict) else 0.0
            width_db_sep_m = float(width_db_row.get("sep", 0.0) or 0.0) if isinstance(width_db_row, dict) else 0.0
            split_required = bool(overlay_row.get("midLineRequired", True))
            overlay_row["startDbWidthM"] = float(width_db_width_m)
            overlay_row["startDbSepM"] = float(width_db_sep_m)
            overlay_row["midLineRequired"] = bool(split_required)
            line_text = (
                f"  P{int(piece.piece_index or 0)}"
                f"{f' / UAV{aid}' if aid > 0 else ''}: "
                f"boxWidth {float(geometry.get('widthM', 0.0)):.0f}m"
            )
            first_width_m = float(overlay_row.get("firstLineWidthM", 0.0) or 0.0)
            if first_width_m > 0.0:
                line_text += f" | firstWidth {first_width_m:.0f}m"
            elif max_width_m > 0.0:
                line_text += (
                    f" | maxWidth {max_width_m:.0f}m"
                    f" (L {left_width_m:.0f}m / R {right_width_m:.0f}m)"
                )
            if isinstance(t0_preview, dict):
                line_text += (
                    f" | T0 {str(t0_preview.get('t0Branch', '') or '?')} "
                    f"{float(t0_preview.get('t0EtaSec', 0.0) or 0.0):.1f}s"
                )
            if sep_start_m > 0.0:
                line_text += f" | Sep_Start {sep_start_m:.0f}m"
            if width_start_m > 0.0:
                line_text += f" | Width_Start {width_start_m:.0f}m"
            if not split_required and width_db_width_m > 0.0:
                line_text += f" | DB OK (W {width_db_width_m:.0f}m / SEP {width_db_sep_m:.0f}m)"
            elif split_required and db_max_width_m > 0.0:
                line_text += f" | split needed (DB max W {db_max_width_m:.0f}m)"
            lines.append(line_text)

        if not overlays:
            raise ValueError("Unable to build mid-line geometry from the split result.")
        return float(reference_bearing_deg), overlays, lines

    def _apply_first_line_width_reference(
        self,
        overlay: Dict[str, Any],
        t0_preview: Optional[Dict[str, Any]],
    ) -> None:
        if not isinstance(overlay, dict) or not isinstance(t0_preview, dict):
            return
        mid_line_xy = overlay.get("midLineXY")
        target_xy_raw = t0_preview.get("t0TargetXY")
        if not (
            isinstance(mid_line_xy, list) and len(mid_line_xy) >= 2
            and isinstance(target_xy_raw, (tuple, list)) and len(target_xy_raw) >= 2
        ):
            return
        try:
            ml0 = (float(mid_line_xy[0][0]), float(mid_line_xy[0][1]))
            ml1 = (float(mid_line_xy[1][0]), float(mid_line_xy[1][1]))
            target_xy = (float(target_xy_raw[0]), float(target_xy_raw[1]))
        except Exception:
            return

        first_key = "max" if _distance(target_xy, ml0) < _distance(target_xy, ml1) else "min"
        prefix = "edgeMax" if first_key == "max" else "edgeMin"
        try:
            first_width_m = float(overlay.get(f"{prefix}WidthM", 0.0) or 0.0)
        except Exception:
            first_width_m = 0.0
        if first_width_m <= 0.0:
            return

        overlay["firstLineWidthSource"] = str(first_key)
        overlay["firstLineWidthM"] = float(first_width_m)
        overlay["widthStartM"] = float(first_width_m)
        overlay["maxWidthM"] = float(first_width_m)
        for suffix in ("WidthLineXY", "WidthCenterXY", "WidthLeftM", "WidthRightM"):
            src_key = f"{prefix}{suffix}"
            if src_key in overlay:
                dst_suffix = suffix.replace("Width", "maxWidth", 1)
                overlay[dst_suffix] = overlay[src_key]

        db_widths = self._fov_db_widths_m()
        db_cover_width_m = self._covering_db_width_m(first_width_m) if db_widths else None
        overlay["dbMaxWidthM"] = float(db_widths[-1]) if db_widths else 0.0
        overlay["dbCoverWidthM"] = float(db_cover_width_m) if db_cover_width_m is not None else 0.0
        overlay["midLineRequired"] = False if db_cover_width_m is not None else True
        if not bool(overlay.get("midLineRequired", True)):
            overlay.pop("splitParts", None)
            overlay.pop("stage2Centers", None)

    def _mid_line_overlay_geometry(
        self,
        coords_xy: Sequence[Tuple[float, float]],
        bearing_deg: float,
    ) -> Optional[Dict[str, Any]]:
        if len(coords_xy) < 3:
            return None

        poly = Polygon(coords_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            return None
        if isinstance(poly, MultiPolygon):
            geoms = [geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty]
            if not geoms:
                return None
            poly = max(geoms, key=lambda geom: float(geom.area))

        theta = math.radians(float(bearing_deg) % 360.0)
        ux = math.sin(theta)
        uy = math.cos(theta)
        vx = uy
        vy = -ux

        s_vals = [(float(x) * ux) + (float(y) * uy) for (x, y) in coords_xy]
        t_vals = [(float(x) * vx) + (float(y) * vy) for (x, y) in coords_xy]
        if not s_vals or not t_vals:
            return None

        min_s = float(min(s_vals))
        max_s = float(max(s_vals))
        min_t = float(min(t_vals))
        max_t = float(max(t_vals))

        def _from_st(s_val: float, t_val: float) -> Tuple[float, float]:
            return (
                (float(s_val) * ux) + (float(t_val) * vx),
                (float(s_val) * uy) + (float(t_val) * vy),
            )

        mid_t = 0.5 * (min_t + max_t)
        mid_s = 0.5 * (min_s + max_s)
        overlay = {
            "boxXY": [
                _from_st(min_s, min_t),
                _from_st(max_s, min_t),
                _from_st(max_s, max_t),
                _from_st(min_s, max_t),
            ],
            "midLineXY": [
                _from_st(min_s, mid_t),
                _from_st(max_s, mid_t),
            ],
            "centerXY": _from_st(mid_s, mid_t),
            "lengthM": max(0.0, max_s - min_s),
            "widthM": max(0.0, max_t - min_t),
        }
        length_m = max(0.0, max_s - min_s)
        probe_pad_m = max(60.0, float(max_t - min_t) * 0.75)

        def _width_geometry_at_s(s_probe: float) -> Optional[Dict[str, Any]]:
            probe_line = LineString(
                [
                    _from_st(s_probe, min_t - probe_pad_m),
                    _from_st(s_probe, max_t + probe_pad_m),
                ]
            )
            width_line = self._longest_linestring_xy(poly.intersection(probe_line))
            if width_line is None or len(width_line.coords) < 2:
                return None

            start_xy = (float(width_line.coords[0][0]), float(width_line.coords[0][1]))
            end_xy = (float(width_line.coords[-1][0]), float(width_line.coords[-1][1]))
            start_t = (float(start_xy[0]) * vx) + (float(start_xy[1]) * vy)
            end_t = (float(end_xy[0]) * vx) + (float(end_xy[1]) * vy)
            seg_min_t = min(start_t, end_t)
            seg_max_t = max(start_t, end_t)

            total_m = max(0.0, float(width_line.length))
            if (seg_min_t - 1.0) <= mid_t <= (seg_max_t + 1.0):
                left_m = max(0.0, mid_t - seg_min_t)
                right_m = max(0.0, seg_max_t - mid_t)
                total_m = max(float(total_m), float(left_m + right_m))
            else:
                left_m = total_m * 0.5
                right_m = total_m * 0.5
            return {
                "maxWidthLineXY": [
                    start_xy,
                    end_xy,
                ],
                "maxWidthCenterXY": (
                    (start_xy[0] + end_xy[0]) * 0.5,
                    (start_xy[1] + end_xy[1]) * 0.5,
                ),
                "maxWidthLeftM": float(left_m),
                "maxWidthRightM": float(right_m),
                "maxWidthM": float(total_m),
            }

        edge_offset_m = 0.0
        if length_m > 1e-6:
            edge_offset_m = min(max(10.0, length_m * 0.005), 75.0, length_m * 0.25)
        edge_min_geom = _width_geometry_at_s(min_s + edge_offset_m)
        edge_max_geom = _width_geometry_at_s(max_s - edge_offset_m)
        if edge_min_geom is not None:
            overlay["edgeMinWidthLineXY"] = edge_min_geom.get("maxWidthLineXY")
            overlay["edgeMinWidthCenterXY"] = edge_min_geom.get("maxWidthCenterXY")
            overlay["edgeMinWidthLeftM"] = float(edge_min_geom.get("maxWidthLeftM", 0.0) or 0.0)
            overlay["edgeMinWidthRightM"] = float(edge_min_geom.get("maxWidthRightM", 0.0) or 0.0)
            overlay["edgeMinWidthM"] = float(edge_min_geom.get("maxWidthM", 0.0) or 0.0)
        if edge_max_geom is not None:
            overlay["edgeMaxWidthLineXY"] = edge_max_geom.get("maxWidthLineXY")
            overlay["edgeMaxWidthCenterXY"] = edge_max_geom.get("maxWidthCenterXY")
            overlay["edgeMaxWidthLeftM"] = float(edge_max_geom.get("maxWidthLeftM", 0.0) or 0.0)
            overlay["edgeMaxWidthRightM"] = float(edge_max_geom.get("maxWidthRightM", 0.0) or 0.0)
            overlay["edgeMaxWidthM"] = float(edge_max_geom.get("maxWidthM", 0.0) or 0.0)

        edge_geometries = [row for row in (edge_min_geom, edge_max_geom) if isinstance(row, dict)]
        best_geometry = (
            max(edge_geometries, key=lambda row: float(row.get("maxWidthM", 0.0) or 0.0))
            if edge_geometries
            else None
        )
        if best_geometry is not None:
            overlay.update(best_geometry)

        max_width_m = float(overlay.get("maxWidthM", 0.0) or 0.0)
        db_widths = self._fov_db_widths_m()
        db_cover_width_m = self._covering_db_width_m(max_width_m) if db_widths and max_width_m > 0.0 else None
        overlay["dbMaxWidthM"] = float(db_widths[-1]) if db_widths else 0.0
        overlay["dbCoverWidthM"] = float(db_cover_width_m) if db_cover_width_m is not None else 0.0
        overlay["midLineRequired"] = (
            False if (max_width_m > 0.0 and db_cover_width_m is not None) else True
        )

        if not bool(overlay.get("midLineRequired", True)):
            return overlay

        split_parts: List[Dict[str, Any]] = []
        try:
            # Extend midline beyond polygon bounds so geom_split always crosses the boundary cleanly.
            # Without this, endpoints sitting exactly on the polygon edge cause Shapely to skip the split.
            _split_pad_m = max(80.0, (max_s - min_s) * 0.5 + 80.0)
            _extended_splitter = LineString([
                _from_st(min_s - _split_pad_m, mid_t),
                _from_st(max_s + _split_pad_m, mid_t),
            ])
            split_geom = geom_split(poly, _extended_splitter)
            part_polys = [geom for geom in getattr(split_geom, "geoms", []) if isinstance(geom, Polygon) and not geom.is_empty]
        except Exception:
            part_polys = []
        if len(part_polys) >= 2:
            part_rows: List[Tuple[float, Dict[str, Any]]] = []
            for part_poly in part_polys:
                coords_part = _dedupe_points(
                    [(float(x), float(y)) for (x, y) in list(part_poly.exterior.coords)[:-1]],
                    min_dist_m=1.0,
                )
                if len(coords_part) < 3:
                    continue
                center = part_poly.centroid
                try:
                    if center.is_empty or not part_poly.buffer(1e-6).contains(center):
                        center = part_poly.representative_point()
                except Exception:
                    center = part_poly.representative_point()
                center_t = (float(center.x) * vx) + (float(center.y) * vy)
                part_rows.append(
                    (
                        float(center_t),
                        {
                            "polygonXY": coords_part,
                            "centroidXY": (float(center.x), float(center.y)),
                        },
                    )
                )
            part_rows.sort(key=lambda item: float(item[0]))
            for part_idx, (_center_t, part_row) in enumerate(part_rows):
                prefix = "A" if part_idx == 0 else "B"
                labeled_points = [
                    {"label": f"{prefix}{idx + 1}", "xy": point_xy}
                    for idx, point_xy in enumerate(part_row["polygonXY"])
                ]
                split_parts.append(
                    {
                        "name": prefix,
                        "polygonXY": part_row["polygonXY"],
                        "centroidXY": part_row["centroidXY"],
                        "pointLabels": labeled_points,
                    }
                )
        if split_parts:
            overlay["splitParts"] = split_parts
        return overlay

    def _normalize_split_part_rows(
        self,
        part_polys: Sequence[Polygon],
        bearing_deg: float,
    ) -> List[Dict[str, Any]]:
        ux, uy, vx, vy = self._mid_line_axis_vectors(bearing_deg)
        part_rows: List[Tuple[float, Dict[str, Any]]] = []
        for part_poly in part_polys:
            if not isinstance(part_poly, Polygon) or part_poly.is_empty:
                continue
            coords_part = _dedupe_points(
                [(float(x), float(y)) for (x, y) in list(part_poly.exterior.coords)[:-1]],
                min_dist_m=1.0,
            )
            if len(coords_part) < 3:
                continue
            center = part_poly.centroid
            try:
                if center.is_empty or not part_poly.buffer(1e-6).contains(center):
                    center = part_poly.representative_point()
            except Exception:
                center = part_poly.representative_point()
            center_t = (float(center.x) * vx) + (float(center.y) * vy)
            part_rows.append(
                (
                    float(center_t),
                    {
                        "polygonXY": coords_part,
                        "centroidXY": (float(center.x), float(center.y)),
                    },
                )
            )

        part_rows.sort(key=lambda item: float(item[0]))
        out: List[Dict[str, Any]] = []
        for part_idx, (_center_t, part_row) in enumerate(part_rows):
            name = chr(ord("A") + min(part_idx, 25))
            labeled_points = [
                {"label": f"{name}{idx + 1}", "xy": point_xy}
                for idx, point_xy in enumerate(part_row["polygonXY"])
            ]
            out.append(
                {
                    "name": name,
                    "polygonXY": part_row["polygonXY"],
                    "centroidXY": part_row["centroidXY"],
                    "pointLabels": labeled_points,
                }
            )
        return out

    def _resolved_split_parts_for_overlay(
        self,
        overlay: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)
        raw_parts = overlay.get("splitParts")
        raw_polys: List[Polygon] = []
        if isinstance(raw_parts, list):
            for part in raw_parts:
                if not isinstance(part, dict):
                    continue
                polygon_xy = part.get("polygonXY")
                if not isinstance(polygon_xy, list) or len(polygon_xy) < 3:
                    continue
                try:
                    part_poly = self._largest_polygon_xy(
                        Polygon(
                            [
                                (float(point_xy[0]), float(point_xy[1]))
                                for point_xy in polygon_xy
                                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                            ]
                        ).buffer(0)
                    )
                except Exception:
                    part_poly = None
                if part_poly is not None and not part_poly.is_empty:
                    raw_polys.append(part_poly)
        piece_index = int(overlay.get("pieceIndex", 0) or 0)
        piece = next(
            (
                piece_row
                for piece_row in (self.state.split_result.pieces if self.state.split_result is not None else [])
                if int(piece_row.piece_index or 0) == piece_index
            ),
            None,
        )
        piece_poly = self._piece_polygon_xy(piece) if piece is not None else None
        mid_line_xy = overlay.get("midLineXY")
        if (
            piece_poly is None
            or piece_poly.is_empty
            or not isinstance(mid_line_xy, list)
            or len(mid_line_xy) < 2
        ):
            return self._normalize_split_part_rows(raw_polys, bearing_deg) if raw_polys else []

        line_points = [
            (float(point_xy[0]), float(point_xy[1]))
            for point_xy in mid_line_xy
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
        ]
        if len(line_points) < 2:
            return self._normalize_split_part_rows(raw_polys, bearing_deg) if raw_polys else []
        part_polys = self._split_piece_polys_by_midline(piece_poly, line_points)
        if len(part_polys) >= 2:
            return self._normalize_split_part_rows(part_polys, bearing_deg)
        return self._normalize_split_part_rows(raw_polys, bearing_deg) if raw_polys else []

    def _split_piece_polys_by_midline(
        self,
        piece_poly: Polygon,
        line_points: Sequence[Tuple[float, float]],
    ) -> List[Polygon]:
        if piece_poly is None or piece_poly.is_empty or len(line_points) < 2:
            return []

        start_xy = (float(line_points[0][0]), float(line_points[0][1]))
        end_xy = (float(line_points[-1][0]), float(line_points[-1][1]))
        dx = float(end_xy[0]) - float(start_xy[0])
        dy = float(end_xy[1]) - float(start_xy[1])
        line_len_m = math.hypot(dx, dy)
        if line_len_m <= 1e-6:
            return []

        minx, miny, maxx, maxy = piece_poly.bounds
        pad_m = max(float(maxx - minx), float(maxy - miny), line_len_m) * 2.0
        ux = dx / line_len_m
        uy = dy / line_len_m
        cutter = LineString(
            [
                (float(start_xy[0]) - (ux * pad_m), float(start_xy[1]) - (uy * pad_m)),
                (float(end_xy[0]) + (ux * pad_m), float(end_xy[1]) + (uy * pad_m)),
            ]
        )

        try:
            split_geom = geom_split(piece_poly, cutter)
            part_polys = [
                geom.buffer(0)
                for geom in getattr(split_geom, "geoms", [])
                if isinstance(geom, Polygon) and not geom.is_empty
            ]
        except Exception:
            part_polys = []

        if len(part_polys) < 2:
            return []
        if len(part_polys) == 2:
            return [
                geom
                for geom in part_polys
                if isinstance(geom, Polygon) and not geom.is_empty
            ]

        pos_group: List[Polygon] = []
        neg_group: List[Polygon] = []
        for geom in part_polys:
            if not isinstance(geom, Polygon) or geom.is_empty:
                continue
            try:
                ref_point = geom.centroid
                if ref_point.is_empty or not geom.buffer(1e-6).contains(ref_point):
                    ref_point = geom.representative_point()
            except Exception:
                ref_point = geom.representative_point()
            cross_val = (
                (float(dx) * (float(ref_point.y) - float(start_xy[1])))
                - (float(dy) * (float(ref_point.x) - float(start_xy[0])))
            )
            if cross_val >= 0.0:
                pos_group.append(geom)
            else:
                neg_group.append(geom)

        merged_parts: List[Polygon] = []
        for group in (neg_group, pos_group):
            if not group:
                continue
            merged = unary_union(group).buffer(0)
            merged_poly = self._largest_polygon_xy(merged)
            if merged_poly is not None and not merged_poly.is_empty:
                merged_parts.append(merged_poly)

        if len(merged_parts) >= 2:
            return merged_parts
        return [
            geom
            for geom in part_polys
            if isinstance(geom, Polygon) and not geom.is_empty
        ]

    def _mid_line_axis_vectors(
        self,
        bearing_deg: float,
    ) -> Tuple[float, float, float, float]:
        theta = math.radians(float(bearing_deg) % 360.0)
        ux = math.sin(theta)
        uy = math.cos(theta)
        vx = uy
        vy = -ux
        return ux, uy, vx, vy

    def _replace_piece_polygon(
        self,
        piece: SplitPiece,
        poly: Polygon,
        *,
        review_patch: Optional[Dict[str, Any]] = None,
    ) -> None:
        coords_xy = [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]]
        if len(coords_xy) < 3:
            return
        data = copy.deepcopy(piece.data if isinstance(piece.data, dict) else {})
        old_coords = data.get("coordinateList") if isinstance(data.get("coordinateList"), list) else []
        alt_m = 0.0
        if old_coords and isinstance(old_coords[0], dict):
            alt_m = float(old_coords[0].get("altitude", 0.0) or 0.0)
        coord_list = [meters_to_coord(x, y, alt_m=alt_m) for (x, y) in coords_xy]
        data["coordinateList"] = coord_list
        data["rawCoordinateList"] = copy.deepcopy(coord_list)
        review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
        review["makeNewAreaApplied"] = True
        if isinstance(review_patch, dict):
            for key, value in review_patch.items():
                review[str(key)] = value
        data["reviewArea"] = review
        piece.data = data

    def _prefer_raw_split_preview_polygons(
        self,
        split_result: Optional[SplitRunResult],
    ) -> None:
        if split_result is None:
            return
        for piece in split_result.pieces:
            if int(piece.mission_type) not in {2, 3, 6}:
                continue
            data = piece.data if isinstance(piece.data, dict) else None
            if not isinstance(data, dict):
                continue
            raw_coords = data.get("rawCoordinateList")
            if not (isinstance(raw_coords, list) and len(coords_to_xy(raw_coords)) >= 3):
                continue
            patched = copy.deepcopy(data)
            patched["coordinateList"] = copy.deepcopy(raw_coords)
            review = patched.get("reviewArea") if isinstance(patched.get("reviewArea"), dict) else {}
            review["previewUsesRawPolygon"] = True
            patched["reviewArea"] = review
            piece.data = patched

    def _make_new_area_polygon(
        self,
        piece: SplitPiece,
        overlay: Dict[str, Any],
        mission_poly: Optional[Polygon],
    ) -> Tuple[Optional[Polygon], Dict[str, Any]]:
        poly = self._piece_polygon_xy(piece)
        if poly is None or poly.is_empty:
            return None, {"reason": "invalidPiece"}
        clip_poly: Optional[Polygon] = None
        if mission_poly is not None and not mission_poly.is_empty:
            clip_poly = self._largest_polygon_xy(mission_poly.buffer(0))

        bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)
        split_parts = self._resolved_split_parts_for_overlay(overlay)
        if split_parts:
            overlay["splitParts"] = split_parts
        part_sources: List[Dict[str, Any]] = []
        if isinstance(split_parts, list) and split_parts:
            for part_idx, part in enumerate(split_parts):
                if not isinstance(part, dict):
                    continue
                polygon_xy = part.get("polygonXY")
                if not isinstance(polygon_xy, list) or len(polygon_xy) < 3:
                    continue
                part_sources.append(
                    {
                        "name": str(part.get("name", "") or ("A" if part_idx == 0 else "B")),
                        "polygonXY": [(float(x), float(y)) for (x, y) in polygon_xy],
                    }
                )
        if not part_sources:
            coords_xy = _dedupe_points(
                [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]],
                min_dist_m=1.0,
            )
            if len(coords_xy) < 3:
                return None, {"reason": "invalidPieceCoords", "oldAreaM2": float(poly.area)}
            part_sources = [{"name": "P", "polygonXY": coords_xy}]

        part_results: List[Dict[str, Any]] = []
        boxed_part_polys: List[Polygon] = []
        boxed_part_polygons_xy: List[Dict[str, Any]] = []
        changed_any = False
        for part in part_sources:
            part_name = str(part.get("name", "") or "?")
            part_coords_xy = part.get("polygonXY")
            if not isinstance(part_coords_xy, list) or len(part_coords_xy) < 3:
                continue

            part_poly = self._largest_polygon_xy(Polygon(part_coords_xy).buffer(0))
            if part_poly is None or part_poly.is_empty:
                part_results.append({"name": part_name, "reason": "invalidPart"})
                continue

            part_overlay = self._mid_line_overlay_geometry(part_coords_xy, bearing_deg)
            if part_overlay is None:
                boxed_part_polys.append(part_poly)
                part_results.append(
                    {
                        "name": part_name,
                        "reason": "noMidLine",
                        "oldAreaM2": float(part_poly.area),
                        "newAreaM2": float(part_poly.area),
                    }
                )
                continue

            box_xy = self._preferred_box_xy_from_part_overlay(
                part_coords_xy,
                part_overlay,
                bearing_deg,
            )
            if not isinstance(box_xy, list) or len(box_xy) < 4:
                boxed_part_polys.append(part_poly)
                part_results.append(
                    {
                        "name": part_name,
                        "reason": "missingBox",
                        "oldAreaM2": float(part_poly.area),
                        "newAreaM2": float(part_poly.area),
                    }
                )
                continue

            box_poly = self._largest_polygon_xy(
                Polygon([(float(x), float(y)) for (x, y) in box_xy]).buffer(0)
            )
            if box_poly is None or box_poly.is_empty:
                boxed_part_polys.append(part_poly)
                part_results.append(
                    {
                        "name": part_name,
                        "reason": "invalidBox",
                        "oldAreaM2": float(part_poly.area),
                        "newAreaM2": float(part_poly.area),
                    }
                )
                continue

            if clip_poly is not None:
                clipped_box_poly = self._largest_polygon_xy(clip_poly.intersection(box_poly).buffer(0))
                if clipped_box_poly is None or clipped_box_poly.is_empty:
                    boxed_part_polys.append(part_poly)
                    part_results.append(
                        {
                            "name": part_name,
                            "reason": "boxOutsideMission",
                            "oldAreaM2": float(part_poly.area),
                            "newAreaM2": float(part_poly.area),
                        }
                    )
                    continue
                box_poly = clipped_box_poly

            old_part_area_m2 = float(part_poly.area)
            new_part_area_m2 = float(box_poly.area)
            diff_geom = part_poly.symmetric_difference(box_poly)
            diff_area_m2 = float(diff_geom.area) if diff_geom is not None and not diff_geom.is_empty else 0.0
            boxed_part_polys.append(box_poly)
            boxed_part_polygons_xy.append(
                {
                    "name": part_name,
                    "polygonXY": _dedupe_points(
                        [(float(x), float(y)) for (x, y) in list(box_poly.exterior.coords)[:-1]],
                        min_dist_m=1.0,
                    ),
                }
            )
            part_results.append(
                {
                    "name": part_name,
                    "reason": "boxedPart" if diff_area_m2 > 1e-6 else "alreadyBoxed",
                    "oldAreaM2": old_part_area_m2,
                    "newAreaM2": new_part_area_m2,
                    "boxLengthM": float(part_overlay.get("lengthM", 0.0) or 0.0),
                    "boxWidthM": float(part_overlay.get("maxWidthM", 0.0) or part_overlay.get("widthM", 0.0) or 0.0),
                }
            )
            if diff_area_m2 > 1e-6:
                changed_any = True

        if not boxed_part_polys:
            return None, {"reason": "noParts", "oldAreaM2": float(poly.area), "partResults": part_results}

        merged = unary_union(boxed_part_polys).buffer(0)
        reconnect_reason = ""
        disconnected_count = 0
        if isinstance(merged, MultiPolygon):
            disconnected_count = len([geom for geom in merged.geoms if isinstance(geom, Polygon) and not geom.is_empty])
            merged_box_coords_xy: List[Tuple[float, float]] = []
            for boxed_poly in boxed_part_polys:
                merged_box_coords_xy.extend(
                    _dedupe_points(
                        [(float(x), float(y)) for (x, y) in list(boxed_poly.exterior.coords)[:-1]],
                        min_dist_m=1.0,
                    )
                )
            reconnect_overlay = (
                self._mid_line_overlay_geometry(merged_box_coords_xy, bearing_deg)
                if len(merged_box_coords_xy) >= 3
                else None
            )
            reconnect_box_xy = reconnect_overlay.get("boxXY") if isinstance(reconnect_overlay, dict) else None
            if isinstance(reconnect_box_xy, list) and len(reconnect_box_xy) >= 4:
                new_poly = self._largest_polygon_xy(
                    Polygon([(float(x), float(y)) for (x, y) in reconnect_box_xy]).buffer(0)
                )
                reconnect_reason = "reconnectedBox"
            else:
                new_poly = self._largest_polygon_xy(merged.convex_hull.buffer(0))
                reconnect_reason = "reconnectedHull"
        else:
            new_poly = self._largest_polygon_xy(merged)
        old_area_m2 = float(poly.area)
        if new_poly is None or new_poly.is_empty:
            return None, {"reason": "invalidMergedBox", "oldAreaM2": old_area_m2, "partResults": part_results}

        if clip_poly is not None:
            clipped_new_poly = self._largest_polygon_xy(clip_poly.intersection(new_poly).buffer(0))
            if clipped_new_poly is not None and not clipped_new_poly.is_empty:
                new_poly = clipped_new_poly

        new_area_m2 = float(new_poly.area)
        info: Dict[str, Any] = {
            "reason": "splitBoxedArea",
            "oldAreaM2": old_area_m2,
            "newAreaM2": new_area_m2,
            "bearingDeg": bearing_deg,
            "partResults": part_results,
            "boxedPartPolygonsXY": boxed_part_polygons_xy,
        }
        if reconnect_reason:
            info["reconnectReason"] = reconnect_reason
            info["disconnectedPartCount"] = int(disconnected_count)
        diff_geom = poly.symmetric_difference(new_poly)
        diff_area_m2 = float(diff_geom.area) if diff_geom is not None and not diff_geom.is_empty else 0.0
        info["deltaAreaM2"] = float(new_area_m2 - old_area_m2)
        info["shapeDeltaM2"] = diff_area_m2
        if (not changed_any) or diff_area_m2 <= 1e-6:
            info["reason"] = "alreadySplitBoxed"
            return None, info
        return new_poly, info

    def _turn_prediction_points_xy(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        speed_mps: float = TURN_PREVIEW_SPEED_MPS,
        horizon_s: float = TURN_PREVIEW_HORIZON_S,
        radius_m: float | None = None,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        if radius_m is None:
            radius_m = self._default_turn_radius_m()
        if radius_m <= 0.0:
            return origin_xy, origin_xy

        theta = math.radians(float(bearing_deg) % 360.0)
        left_center_xy = (
            float(origin_xy[0]) - (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) + (math.sin(theta) * float(radius_m)),
        )
        right_center_xy = (
            float(origin_xy[0]) + (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) - (math.sin(theta) * float(radius_m)),
        )

        def rotate_xy(vec_xy: Tuple[float, float], angle_rad: float) -> Tuple[float, float]:
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            return (
                (float(vec_xy[0]) * cos_a) - (float(vec_xy[1]) * sin_a),
                (float(vec_xy[0]) * sin_a) + (float(vec_xy[1]) * cos_a),
            )

        arc_angle = (float(speed_mps) * float(horizon_s)) / float(radius_m)
        left_radius_xy = (
            float(origin_xy[0]) - float(left_center_xy[0]),
            float(origin_xy[1]) - float(left_center_xy[1]),
        )
        right_radius_xy = (
            float(origin_xy[0]) - float(right_center_xy[0]),
            float(origin_xy[1]) - float(right_center_xy[1]),
        )
        left_rotated_xy = rotate_xy(left_radius_xy, arc_angle)
        right_rotated_xy = rotate_xy(right_radius_xy, -arc_angle)
        left_point_xy = (
            float(left_center_xy[0]) + float(left_rotated_xy[0]),
            float(left_center_xy[1]) + float(left_rotated_xy[1]),
        )
        right_point_xy = (
            float(right_center_xy[0]) + float(right_rotated_xy[0]),
            float(right_center_xy[1]) + float(right_rotated_xy[1]),
        )
        return left_point_xy, right_point_xy

    def _turn_circle_centers_xy(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        radius_m: float | None = None,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        if radius_m is None:
            radius_m = self._default_turn_radius_m()
        theta = math.radians(float(bearing_deg) % 360.0)
        left_center_xy = (
            float(origin_xy[0]) - (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) + (math.sin(theta) * float(radius_m)),
        )
        right_center_xy = (
            float(origin_xy[0]) + (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) - (math.sin(theta) * float(radius_m)),
        )
        return left_center_xy, right_center_xy

    def _distance_point_to_segment(
        self,
        point_xy: Tuple[float, float],
        seg_start_xy: Tuple[float, float],
        seg_end_xy: Tuple[float, float],
    ) -> float:
        sx = float(seg_start_xy[0])
        sy = float(seg_start_xy[1])
        ex = float(seg_end_xy[0])
        ey = float(seg_end_xy[1])
        px = float(point_xy[0])
        py = float(point_xy[1])
        dx = ex - sx
        dy = ey - sy
        denom = (dx * dx) + (dy * dy)
        if denom <= 1e-9:
            return math.hypot(px - sx, py - sy)
        t = ((px - sx) * dx + (py - sy) * dy) / denom
        t = max(0.0, min(1.0, t))
        proj_x = sx + (t * dx)
        proj_y = sy + (t * dy)
        return math.hypot(px - proj_x, py - proj_y)

    def _orient_sweep_lines_toward_anchor(
        self,
        sweep_lines_xy: List[List[Tuple[float, float]]],
        anchor_xy: Tuple[float, float] | None,
    ) -> List[List[Tuple[float, float]]]:
        normalized = [
            [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in line_xy
                if isinstance(point_xy, tuple) and len(point_xy) >= 2
            ]
            for line_xy in (sweep_lines_xy or [])
            if isinstance(line_xy, list)
        ]
        normalized = [line_xy for line_xy in normalized if len(line_xy) >= 2]
        if len(normalized) < 2 or not (isinstance(anchor_xy, tuple) and len(anchor_xy) >= 2):
            return normalized
        original_first_xy = normalized[0][0]
        reversed_lines_xy = [list(line_xy) for line_xy in reversed(normalized)]
        reversed_first_xy = reversed_lines_xy[0][0]
        if _distance(anchor_xy, reversed_first_xy) + 1e-6 < _distance(anchor_xy, original_first_xy):
            return reversed_lines_xy
        return normalized

    def _orient_sweep_line_directions_pathwise(
        self,
        sweep_lines_xy: List[List[Tuple[float, float]]],
        anchor_xy: Tuple[float, float] | None,
    ) -> List[List[Tuple[float, float]]]:
        normalized = [
            [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in line_xy
                if isinstance(point_xy, tuple) and len(point_xy) >= 2
            ]
            for line_xy in (sweep_lines_xy or [])
            if isinstance(line_xy, list)
        ]
        normalized = [line_xy for line_xy in normalized if len(line_xy) >= 2]
        if not normalized:
            return normalized
        if not (isinstance(anchor_xy, tuple) and len(anchor_xy) >= 2):
            return normalized

        prev_xy = (float(anchor_xy[0]), float(anchor_xy[1]))
        out: List[List[Tuple[float, float]]] = []
        for line_xy in normalized:
            first_xy = line_xy[0]
            last_xy = line_xy[-1]
            if _distance(prev_xy, last_xy) + 1e-6 < _distance(prev_xy, first_xy):
                line_xy = list(reversed(line_xy))
                first_xy = line_xy[0]
                last_xy = line_xy[-1]
            out.append(line_xy)
            prev_xy = (float(last_xy[0]), float(last_xy[1]))
        return out

    def _closest_point_on_segment_xy(
        self,
        point_xy: Tuple[float, float],
        seg_start_xy: Tuple[float, float],
        seg_end_xy: Tuple[float, float],
    ) -> Tuple[Tuple[float, float], float]:
        sx = float(seg_start_xy[0])
        sy = float(seg_start_xy[1])
        ex = float(seg_end_xy[0])
        ey = float(seg_end_xy[1])
        px = float(point_xy[0])
        py = float(point_xy[1])
        dx = ex - sx
        dy = ey - sy
        denom = (dx * dx) + (dy * dy)
        if denom <= 1e-9:
            return (sx, sy), math.hypot(px - sx, py - sy)
        t = ((px - sx) * dx + (py - sy) * dy) / denom
        t = max(0.0, min(1.0, t))
        proj_x = sx + (t * dx)
        proj_y = sy + (t * dy)
        return (proj_x, proj_y), math.hypot(px - proj_x, py - proj_y)

    def _line_avoids_turn_circles(
        self,
        start_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        radius_m: float | None = None,
    ) -> bool:
        if radius_m is None:
            radius_m = self._default_turn_radius_m()
        if _distance(start_xy, target_xy) < 1e-9:
            return True
        for center_xy in self._turn_circle_centers_xy(origin_xy, bearing_deg, radius_m=radius_m):
            if self._distance_point_to_segment(center_xy, start_xy, target_xy) < (float(radius_m) - 1e-6):
                return False
        return True

    def _turn_branch_toward_target(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        target_xy: Tuple[float, float],
    ) -> Optional[str]:
        target_bearing_deg = _bearing_deg_from_xy(origin_xy, target_xy)
        if target_bearing_deg is None:
            return None
        delta_deg = ((float(target_bearing_deg) - float(bearing_deg) + 540.0) % 360.0) - 180.0
        if abs(delta_deg) <= 1e-6:
            return None
        return "R" if delta_deg > 0.0 else "L"

    def _arc_horizon_to_point_on_branch(
        self,
        origin_xy: Tuple[float, float],
        circle_center_xy: Tuple[float, float],
        point_xy: Tuple[float, float],
        *,
        branch: str,
        radius_m: float | None = None,
        speed_mps: float = TURN_PREVIEW_SPEED_MPS,
    ) -> float:
        if radius_m is None:
            radius_m = self._default_turn_radius_m()
        start_angle = math.atan2(
            float(origin_xy[1]) - float(circle_center_xy[1]),
            float(origin_xy[0]) - float(circle_center_xy[0]),
        )
        point_angle = math.atan2(
            float(point_xy[1]) - float(circle_center_xy[1]),
            float(point_xy[0]) - float(circle_center_xy[0]),
        )
        if str(branch).upper() == "L":
            delta_rad = (point_angle - start_angle) % (2.0 * math.pi)
        else:
            delta_rad = (start_angle - point_angle) % (2.0 * math.pi)
        return (float(delta_rad) * float(radius_m)) / float(speed_mps)

    def _refine_visibility_start_xy(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        target_xy: Tuple[float, float],
        *,
        branch: str,
        min_horizon_s: float,
        max_horizon_s: float,
        radius_m: float | None = None,
    ) -> Optional[Dict[str, Any]]:
        if radius_m is None:
            radius_m = self._default_turn_radius_m()
        branch_key = str(branch).upper()
        left_center_xy, right_center_xy = self._turn_circle_centers_xy(origin_xy, bearing_deg, radius_m=radius_m)
        circle_center_xy = left_center_xy if branch_key == "L" else right_center_xy

        dx = float(target_xy[0]) - float(circle_center_xy[0])
        dy = float(target_xy[1]) - float(circle_center_xy[1])
        dist_to_target = math.hypot(dx, dy)
        if dist_to_target <= float(radius_m) + 1e-6:
            return None

        alpha = math.atan2(dy, dx)
        beta = math.acos(float(radius_m) / float(dist_to_target))
        tangent_angles = [alpha + beta, alpha - beta]

        best: Optional[Dict[str, Any]] = None
        for tangent_angle in tangent_angles:
            tangent_xy = (
                float(circle_center_xy[0]) + (float(radius_m) * math.cos(tangent_angle)),
                float(circle_center_xy[1]) + (float(radius_m) * math.sin(tangent_angle)),
            )
            horizon_s = self._arc_horizon_to_point_on_branch(
                origin_xy,
                circle_center_xy,
                tangent_xy,
                branch=branch_key,
                radius_m=radius_m,
            )
            if horizon_s < float(min_horizon_s) - 1e-6:
                continue
            if horizon_s > float(max_horizon_s) + 1e-6:
                continue
            if not self._line_avoids_turn_circles(tangent_xy, target_xy, origin_xy, bearing_deg, radius_m=radius_m):
                continue
            candidate = {
                "startXY": tangent_xy,
                "horizonSec": float(horizon_s),
                "branch": branch_key,
            }
            if best is None or float(candidate["horizonSec"]) < float(best["horizonSec"]):
                best = candidate
        return best

    def _uav_prediction_points_by_id(self) -> Dict[int, Tuple[Tuple[float, float], Tuple[float, float]]]:
        out: Dict[int, Tuple[Tuple[float, float], Tuple[float, float]]] = {}
        for idx, aid in enumerate(self.state.uav_ids):
            if idx >= len(self.state.uav_positions_xy):
                continue
            if idx >= len(self.state.uav_heading_deg):
                continue
            heading = self.state.uav_heading_deg[idx]
            if heading is None:
                continue
            out[int(aid)] = self._turn_prediction_points_xy(
                self.state.uav_positions_xy[idx],
                float(heading),
            )
        return out

    def _piece_assignment_target_xy(self, piece: SplitPiece) -> Optional[Tuple[float, float]]:
        data = piece.data if isinstance(piece.data, dict) else {}
        for key in ("coordinateList", "Centerline", "rawCoordinateList"):
            coords = coords_to_xy(data.get(key, []))
            if len(coords) >= 3:
                poly = Polygon(coords)
                if not poly.is_empty:
                    center = poly.centroid
                    return float(center.x), float(center.y)
            center_xy = centroid_xy(coords)
            if center_xy is not None:
                return center_xy
        return None

    def _piece_assignment_coords_xy(self, piece: SplitPiece) -> List[Tuple[float, float]]:
        data = piece.data if isinstance(piece.data, dict) else {}
        for key in ("coordinateList", "rawCoordinateList", "Centerline"):
            coords = coords_to_xy(data.get(key, []))
            if len(coords) >= 3:
                return [(float(x), float(y)) for x, y in coords]
        return []

    def _assignment_t0_preview_for_piece(
        self,
        piece: SplitPiece,
        aircraft_id: int,
        reference_bearing_deg: float,
    ) -> Optional[Dict[str, Any]]:
        coords_xy = self._piece_assignment_coords_xy(piece)
        if len(coords_xy) < 3:
            return None
        geometry = self._mid_line_overlay_geometry(coords_xy, float(reference_bearing_deg))
        if not isinstance(geometry, dict):
            return None
        uav_state = self._uav_state_for_aircraft(int(aircraft_id))
        if uav_state is None:
            return None

        overlay: Dict[str, Any] = {
            "pieceIndex": int(piece.piece_index or 0),
            "aircraftID": int(aircraft_id),
            "bearingDeg": float(reference_bearing_deg),
            **geometry,
        }
        split_boundary_points_xy = self._mid_line_split_boundary_points(coords_xy, overlay)
        if split_boundary_points_xy:
            overlay["splitBoundaryPointsXY"] = split_boundary_points_xy
        origin_xy, heading_deg = uav_state
        return self._mid_line_t0_preview(overlay, origin_xy, float(heading_deg))

    def _overlay_st_bounds(
        self,
        overlay: Dict[str, Any],
    ) -> Optional[Tuple[float, float, float, float, float, float, float, float, float]]:
        box_xy = overlay.get("boxXY")
        if not isinstance(box_xy, list) or len(box_xy) < 4:
            return None
        bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)
        ux, uy, vx, vy = self._mid_line_axis_vectors(bearing_deg)
        pts: List[Tuple[float, float]] = []
        for point_xy in box_xy:
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                pts.append((float(point_xy[0]), float(point_xy[1])))
        if len(pts) < 4:
            return None
        s_vals = [(float(x) * ux) + (float(y) * uy) for (x, y) in pts]
        t_vals = [(float(x) * vx) + (float(y) * vy) for (x, y) in pts]
        if not s_vals or not t_vals:
            return None
        return (
            float(bearing_deg),
            float(ux),
            float(uy),
            float(vx),
            float(vy),
            float(min(s_vals)),
            float(max(s_vals)),
            float(min(t_vals)),
            float(max(t_vals)),
        )

    def _from_st_xy(
        self,
        s_val: float,
        t_val: float,
        ux: float,
        uy: float,
        vx: float,
        vy: float,
    ) -> Tuple[float, float]:
        return (
            (float(s_val) * float(ux)) + (float(t_val) * float(vx)),
            (float(s_val) * float(uy)) + (float(t_val) * float(vy)),
        )

    def _preferred_box_xy_from_part_overlay(
        self,
        coords_xy: Sequence[Tuple[float, float]],
        overlay: Dict[str, Any],
        bearing_deg: float,
    ) -> Optional[List[Tuple[float, float]]]:
        pts = [(float(x), float(y)) for (x, y) in coords_xy]
        if len(pts) < 3:
            return None

        ux, uy, vx, vy = self._mid_line_axis_vectors(bearing_deg)
        s_vals = [(float(x) * ux) + (float(y) * uy) for (x, y) in pts]
        t_vals = [(float(x) * vx) + (float(y) * vy) for (x, y) in pts]
        if not s_vals or not t_vals:
            return None
        min_s = float(min(s_vals))
        max_s = float(max(s_vals))
        min_t = float(min(t_vals))
        max_t = float(max(t_vals))
        return [
            self._from_st_xy(min_s, min_t, ux, uy, vx, vy),
            self._from_st_xy(max_s, min_t, ux, uy, vx, vy),
            self._from_st_xy(max_s, max_t, ux, uy, vx, vy),
            self._from_st_xy(min_s, max_t, ux, uy, vx, vy),
        ]

    def _make_path_face_points(
        self,
        overlay: Dict[str, Any],
        origin_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Tuple[float, float]]]:
        bounds = self._overlay_st_bounds(overlay)
        if bounds is None:
            return None
        _bearing_deg, ux, uy, vx, vy, min_s, max_s, min_t, max_t = bounds
        mid_t = 0.5 * (float(min_t) + float(max_t))
        min_face_xy = self._from_st_xy(min_s, mid_t, ux, uy, vx, vy)
        max_face_xy = self._from_st_xy(max_s, mid_t, ux, uy, vx, vy)
        min_face_line_xy = [
            self._from_st_xy(min_s, min_t, ux, uy, vx, vy),
            self._from_st_xy(min_s, max_t, ux, uy, vx, vy),
        ]
        max_face_line_xy = [
            self._from_st_xy(max_s, min_t, ux, uy, vx, vy),
            self._from_st_xy(max_s, max_t, ux, uy, vx, vy),
        ]
        far_is_min = _distance(origin_xy, min_face_xy) >= _distance(origin_xy, max_face_xy)
        target_face_xy = min_face_xy if far_is_min else max_face_xy
        target_face_line_xy = min_face_line_xy if far_is_min else max_face_line_xy
        return {
            "targetFaceXY": target_face_xy,
            "targetFaceLineXY": target_face_line_xy,
            "nearFaceXY": max_face_xy if far_is_min else min_face_xy,
            "nearFaceLineXY": max_face_line_xy if far_is_min else min_face_line_xy,
        }

    def _bearing_centerline_target_points(
        self,
        coords_xy: Sequence[Tuple[float, float]],
        bearing_deg: float,
        origin_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Any]]:
        if len(coords_xy) < 3:
            return None
        poly = Polygon(coords_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            return None
        if isinstance(poly, MultiPolygon):
            geoms = [geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty]
            if not geoms:
                return None
            poly = max(geoms, key=lambda geom: float(geom.area))

        ux, uy, vx, vy = self._mid_line_axis_vectors(bearing_deg)
        pts = [(float(x), float(y)) for (x, y) in coords_xy]
        s_vals = [(float(x) * ux) + (float(y) * uy) for (x, y) in pts]
        t_vals = [(float(x) * vx) + (float(y) * vy) for (x, y) in pts]
        if not s_vals or not t_vals:
            return None

        min_s = float(min(s_vals))
        max_s = float(max(s_vals))
        min_t = float(min(t_vals))
        max_t = float(max(t_vals))
        mid_s = 0.5 * (min_s + max_s)
        mid_t = 0.5 * (min_t + max_t)

        center_point = poly.centroid
        try:
            if center_point.is_empty or not poly.buffer(1e-6).contains(center_point):
                center_point = poly.representative_point()
        except Exception:
            center_point = poly.representative_point()

        pad_m = max(80.0, float(max_s - min_s) * 0.25)
        guide = LineString(
            [
                self._from_st_xy(min_s - pad_m, mid_t, ux, uy, vx, vy),
                self._from_st_xy(max_s + pad_m, mid_t, ux, uy, vx, vy),
            ]
        )
        guide_center_xy = self._from_st_xy(mid_s, mid_t, ux, uy, vx, vy)
        guide_center_point = Point(float(guide_center_xy[0]), float(guide_center_xy[1]))
        center_segments = self._linestring_segments_xy(poly.intersection(guide))
        if not center_segments:
            return None

        def _segment_rank(seg: LineString) -> Tuple[float, float, float]:
            try:
                dist_to_center = float(seg.distance(guide_center_point))
            except Exception:
                dist_to_center = float("inf")
            try:
                mid_point = seg.interpolate(0.5, normalized=True)
                mid_dist = float(mid_point.distance(guide_center_point))
            except Exception:
                mid_dist = dist_to_center
            return (dist_to_center, mid_dist, -float(seg.length))

        centerline = min(center_segments, key=_segment_rank)
        if centerline is None or len(centerline.coords) < 2:
            return None

        start_xy = (float(centerline.coords[0][0]), float(centerline.coords[0][1]))
        end_xy = (float(centerline.coords[-1][0]), float(centerline.coords[-1][1]))
        far_is_start = _distance(origin_xy, start_xy) >= _distance(origin_xy, end_xy)
        target_xy = start_xy if far_is_start else end_xy
        near_xy = end_xy if far_is_start else start_xy
        return {
            "targetXY": target_xy,
            "nearXY": near_xy,
            "centerLineXY": [start_xy, end_xy],
            "shapeCenterXY": (float(center_point.x), float(center_point.y)),
        }

    def _mid_line_far_endpoint_xy(
        self,
        overlay: Dict[str, Any],
        origin_xy: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        mid_line_xy = overlay.get("midLineXY")
        if not isinstance(mid_line_xy, list) or len(mid_line_xy) < 2:
            return None
        endpoints_xy: List[Tuple[float, float]] = []
        for point_xy in mid_line_xy:
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                endpoints_xy.append((float(point_xy[0]), float(point_xy[1])))
        if len(endpoints_xy) < 2:
            return None
        return max(
            endpoints_xy,
            key=lambda point_xy: (
                _distance(origin_xy, point_xy),
                float(point_xy[0]),
                float(point_xy[1]),
            ),
        )

    def _closest_split_part_point_label(
        self,
        overlay: Dict[str, Any],
        ref_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Any]]:
        split_parts = overlay.get("splitParts")
        if not isinstance(split_parts, list):
            return None

        best: Optional[Dict[str, Any]] = None
        best_key: Optional[Tuple[float, str, float, float]] = None
        for part in split_parts:
            if not isinstance(part, dict):
                continue
            point_labels = part.get("pointLabels")
            if not isinstance(point_labels, list):
                continue
            for point_row in point_labels:
                if not isinstance(point_row, dict):
                    continue
                point_xy = point_row.get("xy")
                if not (isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2):
                    continue
                point_xy_val = (float(point_xy[0]), float(point_xy[1]))
                label = str(point_row.get("label", "") or "")
                dist_m = _distance(ref_xy, point_xy_val)
                candidate = {
                    "xy": point_xy_val,
                    "label": label,
                    "distM": float(dist_m),
                }
                candidate_key = (
                    float(dist_m),
                    label,
                    float(point_xy_val[0]),
                    float(point_xy_val[1]),
                )
                if best is None or best_key is None or candidate_key < best_key:
                    best = candidate
                    best_key = candidate_key
        return best

    def _mid_line_split_boundary_points(
        self,
        coords_xy: Sequence[Tuple[float, float]],
        overlay: Dict[str, Any],
    ) -> List[Tuple[float, float]]:
        if len(coords_xy) < 3:
            return []
        mid_line_xy = overlay.get("midLineXY")
        if not isinstance(mid_line_xy, list) or len(mid_line_xy) < 2:
            return []

        line_points = [
            (float(point_xy[0]), float(point_xy[1]))
            for point_xy in mid_line_xy
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
        ]
        if len(line_points) < 2:
            return []

        poly = Polygon(coords_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            return []
        if isinstance(poly, MultiPolygon):
            geoms = [geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty]
            if not geoms:
                return []
            poly = max(geoms, key=lambda geom: float(geom.area))

        start_xy = line_points[0]
        end_xy = line_points[-1]
        dx = float(end_xy[0]) - float(start_xy[0])
        dy = float(end_xy[1]) - float(start_xy[1])
        line_len_m = math.hypot(dx, dy)
        if line_len_m <= 1e-6:
            return []
        ux = dx / line_len_m
        uy = dy / line_len_m
        minx, miny, maxx, maxy = poly.bounds
        pad_m = max(float(maxx - minx), float(maxy - miny), line_len_m) * 2.0
        probe_line = LineString(
            [
                (float(start_xy[0]) - (ux * pad_m), float(start_xy[1]) - (uy * pad_m)),
                (float(end_xy[0]) + (ux * pad_m), float(end_xy[1]) + (uy * pad_m)),
            ]
        )
        candidates = self._geometry_point_candidates_xy(poly.boundary.intersection(probe_line))
        if not candidates:
            return []

        ranked: List[Tuple[float, Tuple[float, float]]] = []
        for point_xy in candidates:
            px = float(point_xy[0]) - float(start_xy[0])
            py = float(point_xy[1]) - float(start_xy[1])
            cross_m = abs((px * uy) - (py * ux))
            if cross_m > 2.0:
                continue
            proj_m = (px * ux) + (py * uy)
            ranked.append((float(proj_m), (float(point_xy[0]), float(point_xy[1]))))
        if not ranked:
            return []

        ranked.sort(key=lambda item: float(item[0]))
        picked = [ranked[0][1]]
        if len(ranked) > 1:
            picked.append(ranked[-1][1])
        return _dedupe_points(picked, min_dist_m=1.0)

    def _mid_line_t0_preview(
        self,
        overlay: Dict[str, Any],
        origin_xy: Tuple[float, float],
        heading_deg: float,
    ) -> Optional[Dict[str, Any]]:
        target_xy = self._mid_line_far_endpoint_xy(overlay, origin_xy)
        if target_xy is None:
            return None

        preferred_branch = self._turn_branch_toward_target(origin_xy, heading_deg, target_xy)
        left_center_xy, right_center_xy = self._turn_circle_centers_xy(origin_xy, heading_deg)
        best: Optional[Dict[str, Any]] = None
        best_key: Optional[Tuple[float, float, float, float]] = None

        for branch, circle_center_xy in (("L", left_center_xy), ("R", right_center_xy)):
            tangent_points = self._circle_tangent_points_xy(circle_center_xy, target_xy)
            for tangent_xy in tangent_points:
                tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
                if not self._branch_forward_tangent(
                    circle_center_xy,
                    tangent_point_xy,
                    target_xy,
                    branch=branch,
                ):
                    continue
                if not self._line_avoids_turn_circles(
                    tangent_point_xy,
                    target_xy,
                    origin_xy,
                    heading_deg,
                ):
                    continue
                horizon_s = self._arc_horizon_to_point_on_branch(
                    origin_xy,
                    circle_center_xy,
                    tangent_point_xy,
                    branch=branch,
                )
                ingress_dist_m = _distance(tangent_point_xy, target_xy)
                eta_sec = float(horizon_s) + (float(ingress_dist_m) / float(TURN_PREVIEW_SPEED_MPS))
                candidate = {
                    "t0TargetXY": target_xy,
                    "t0TangentXY": tangent_point_xy,
                    "t0CircleCenterXY": (float(circle_center_xy[0]), float(circle_center_xy[1])),
                    "t0Branch": branch,
                    "t0HorizonSec": float(horizon_s),
                    "t0EtaSec": float(eta_sec),
                }
                candidate_key = (
                    0.0 if preferred_branch == branch else 1.0,
                    float(eta_sec),
                    float(horizon_s),
                    float(ingress_dist_m),
                )
                if best is None or best_key is None or candidate_key < best_key:
                    best = candidate
                    best_key = candidate_key

        if best is None:
            return None

        route_xy, marker_rows, timeline_rows = self._build_turn_prefix_rows(
            origin_xy,
            heading_deg,
            branch=str(best.get("t0Branch", "") or ""),
            horizon_s=float(best.get("t0HorizonSec", 0.0) or 0.0),
            tangent_point_xy=best["t0TangentXY"],
            tangent_label="T0",
            suppress_last_turn_label=True,
        )
        target_point_xy = best["t0TargetXY"]
        if _distance(route_xy[-1], target_point_xy) > 1e-6:
            route_xy.append(target_point_xy)
        marker_rows.append(
            {
                "xy": target_point_xy,
                "label": "",
                "kind": "mid_far",
                "etaSec": float(best.get("t0EtaSec", 0.0) or 0.0),
            }
        )
        timeline_rows.append(
            {
                "label": "MID",
                "kind": "mid_far",
                "etaSec": float(best.get("t0EtaSec", 0.0) or 0.0),
            }
        )
        split_boundary_points_xy = overlay.get("splitBoundaryPointsXY")
        boundary_candidates: List[Tuple[float, float]] = []
        if isinstance(split_boundary_points_xy, list):
            for point_xy in split_boundary_points_xy:
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                    boundary_candidates.append((float(point_xy[0]), float(point_xy[1])))
        if boundary_candidates:
            shape_point_xy = min(
                boundary_candidates,
                key=lambda point_xy: (
                    _distance(best["t0TangentXY"], point_xy),
                    float(point_xy[0]),
                    float(point_xy[1]),
                ),
            )
            best["t0ShapePointXY"] = shape_point_xy
            best["t0ShapePointDistM"] = _distance(best["t0TangentXY"], shape_point_xy)
        else:
            shape_point = self._closest_split_part_point_label(overlay, best["t0TangentXY"])
            if isinstance(shape_point, dict):
                best["t0ShapePointXY"] = shape_point["xy"]
                best["t0ShapePointLabel"] = str(shape_point.get("label", "") or "")
                best["t0ShapePointDistM"] = float(shape_point.get("distM", 0.0) or 0.0)
        best["t0RouteXY"] = route_xy
        best["t0MarkerRows"] = marker_rows
        best["t0TimelineRows"] = timeline_rows
        return best

    def _circle_tangent_points_xy(
        self,
        circle_center_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        *,
        radius_m: float | None = None,
    ) -> List[Tuple[float, float]]:
        if radius_m is None:
            radius_m = self._default_turn_radius_m()
        dx = float(target_xy[0]) - float(circle_center_xy[0])
        dy = float(target_xy[1]) - float(circle_center_xy[1])
        dist_m = math.hypot(dx, dy)
        if dist_m <= float(radius_m) + 1e-6:
            return []

        alpha = math.atan2(dy, dx)
        beta = math.acos(float(radius_m) / float(dist_m))
        out: List[Tuple[float, float]] = []
        for tangent_angle in (alpha + beta, alpha - beta):
            out.append(
                (
                    float(circle_center_xy[0]) + (float(radius_m) * math.cos(tangent_angle)),
                    float(circle_center_xy[1]) + (float(radius_m) * math.sin(tangent_angle)),
                )
            )
        return out

    def _branch_forward_tangent(
        self,
        circle_center_xy: Tuple[float, float],
        tangent_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        *,
        branch: str,
    ) -> bool:
        rx = float(tangent_xy[0]) - float(circle_center_xy[0])
        ry = float(tangent_xy[1]) - float(circle_center_xy[1])
        if str(branch).upper() == "L":
            tangent_dir_xy = (-ry, rx)
        else:
            tangent_dir_xy = (ry, -rx)
        target_dir_xy = (
            float(target_xy[0]) - float(tangent_xy[0]),
            float(target_xy[1]) - float(tangent_xy[1]),
        )
        return ((float(tangent_dir_xy[0]) * float(target_dir_xy[0])) + (float(tangent_dir_xy[1]) * float(target_dir_xy[1]))) > 1e-6

    def _sweep_lines_for_polygon_xy(
        self,
        polygon_xy: Sequence[Tuple[float, float]],
        bearing_deg: float,
        *,
        sep_m: float,
    ) -> List[List[Tuple[float, float]]]:
        if sep_m <= 0.0 or len(polygon_xy) < 3:
            return []
        piece_poly = self._largest_polygon_xy(Polygon(polygon_xy).buffer(0))
        if piece_poly is None or piece_poly.is_empty:
            return []
        overlay = self._mid_line_overlay_geometry(polygon_xy, bearing_deg)
        bounds = self._overlay_st_bounds(
            {
                "bearingDeg": float(bearing_deg),
                **(overlay or {}),
            }
        )
        if bounds is None:
            return []
        _bearing_deg, ux, uy, vx, vy, min_s, max_s, min_t, max_t = bounds
        mid_t = 0.5 * (float(min_t) + float(max_t))
        pad_m = max(80.0, float(max_s - min_s) * 0.25)

        t_values: List[float] = [mid_t]
        step_idx = 1
        while True:
            added = False
            left_t = mid_t - (float(step_idx) * float(sep_m))
            right_t = mid_t + (float(step_idx) * float(sep_m))
            if left_t >= float(min_t) - 1e-6:
                t_values.append(float(left_t))
                added = True
            if right_t <= float(max_t) + 1e-6:
                t_values.append(float(right_t))
                added = True
            if not added:
                break
            step_idx += 1

        lines_xy: List[List[Tuple[float, float]]] = []
        for t_val in t_values:
            guide = LineString(
                [
                    self._from_st_xy(min_s - pad_m, t_val, ux, uy, vx, vy),
                    self._from_st_xy(max_s + pad_m, t_val, ux, uy, vx, vy),
                ]
            )
            seg = self._longest_linestring_xy(piece_poly.intersection(guide))
            if seg is None or len(seg.coords) < 2:
                continue
            lines_xy.append(
                [
                    (float(seg.coords[0][0]), float(seg.coords[0][1])),
                    (float(seg.coords[-1][0]), float(seg.coords[-1][1])),
                ]
            )
        return lines_xy

    def _build_turn_prefix_rows(
        self,
        origin_xy: Tuple[float, float],
        heading_deg: float,
        *,
        branch: str,
        horizon_s: float,
        tangent_point_xy: Tuple[float, float],
        tangent_label: str = "T",
        suppress_last_turn_label: bool = False,
    ) -> Tuple[List[Tuple[float, float]], List[Dict[str, Any]], List[Dict[str, Any]]]:
        route_xy: List[Tuple[float, float]] = [origin_xy]
        marker_rows: List[Dict[str, Any]] = []
        timeline_rows: List[Dict[str, Any]] = []

        if horizon_s > 1e-6 and branch in {"L", "R"}:
            sample_step_s = min(float(TURN_PREVIEW_HORIZON_S), float(ASSIGNMENT_PATH_ARC_STEP_S))
            sample_count = int(math.floor(horizon_s / float(sample_step_s)))
            for idx in range(1, sample_count + 1):
                sample_horizon_s = float(idx) * float(sample_step_s)
                if sample_horizon_s >= horizon_s - 1e-6:
                    break
                left_xy, right_xy = self._turn_prediction_points_xy(
                    origin_xy,
                    heading_deg,
                    horizon_s=sample_horizon_s,
                )
                point_xy = left_xy if branch == "L" else right_xy
                if _distance(route_xy[-1], point_xy) > 1e-6:
                    route_xy.append(point_xy)
                marker_multiple = sample_horizon_s / float(TURN_PREVIEW_HORIZON_S)
                if abs(marker_multiple - round(marker_multiple)) <= 1e-6:
                    marker_rows.append(
                        {
                            "xy": point_xy,
                            "label": f"{int(sample_horizon_s)}s",
                            "kind": "turn",
                            "etaSec": sample_horizon_s,
                        }
                    )
                    timeline_rows.append({"label": f"{int(sample_horizon_s)}s", "kind": "turn", "etaSec": sample_horizon_s})

        if suppress_last_turn_label and marker_rows and timeline_rows:
            last_marker = marker_rows[-1]
            last_timeline = timeline_rows[-1]
            if (
                str(last_marker.get("kind", "") or "") == "turn"
                and str(last_timeline.get("kind", "") or "") == "turn"
            ):
                marker_rows.pop()
                timeline_rows.pop()

        if _distance(route_xy[-1], tangent_point_xy) > 1e-6:
            route_xy.append(tangent_point_xy)
        marker_rows.append({"xy": tangent_point_xy, "label": tangent_label, "kind": "tangent", "etaSec": horizon_s})
        timeline_rows.append({"label": tangent_label, "kind": "tangent", "etaSec": horizon_s})
        return route_xy, marker_rows, timeline_rows

    def _build_assignment_path_1_row(
        self,
        candidate: Dict[str, Any],
        origin_override: Optional[Tuple[Tuple[float, float], float]] = None,
        *,
        tangent_label: str = "T",
        suppress_last_turn_label: bool = False,
    ) -> Optional[Dict[str, Any]]:
        aid = int(candidate.get("aircraftID", 0) or 0)
        if aid <= 0:
            return None
        if origin_override is not None:
            origin_xy, heading_deg = origin_override
        else:
            uav_state = self._uav_state_for_aircraft(aid)
            if uav_state is None:
                return None
            origin_xy, heading_deg = uav_state
        tangent_xy = candidate.get("tangentXY")
        target_xy = candidate.get("targetXY")
        if not (
            isinstance(tangent_xy, (tuple, list))
            and len(tangent_xy) >= 2
            and isinstance(target_xy, (tuple, list))
            and len(target_xy) >= 2
        ):
            return None

        tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
        target_point_xy = (float(target_xy[0]), float(target_xy[1]))
        branch = str(candidate.get("branch", "") or "")
        horizon_s = float(candidate.get("horizonSec", 0.0) or 0.0)

        route_xy, marker_rows, timeline_rows = self._build_turn_prefix_rows(
            origin_xy,
            heading_deg,
            branch=branch,
            horizon_s=horizon_s,
            tangent_point_xy=tangent_point_xy,
            tangent_label=tangent_label,
            suppress_last_turn_label=suppress_last_turn_label,
        )

        if _distance(route_xy[-1], target_point_xy) > 1e-6:
            route_xy.append(target_point_xy)
        target_eta_sec = horizon_s + (_distance(tangent_point_xy, target_point_xy) / float(TURN_PREVIEW_SPEED_MPS))
        marker_rows.append(
            {
                "xy": target_point_xy,
                "label": str(candidate.get("targetLabel", "") or "END1"),
                "kind": "last_face",
                "etaSec": target_eta_sec,
            }
        )
        timeline_rows.append(
            {
                "label": str(candidate.get("targetLabel", "") or "END1"),
                "kind": "last_face",
                "etaSec": target_eta_sec,
            }
        )
        phase_rows: List[Dict[str, Any]] = []
        if horizon_s > 1e-6:
            phase_rows.append({"label": "Turn", "kind": "turn", "startSec": 0.0, "endSec": horizon_s})
        if target_eta_sec > horizon_s + 1e-6:
            phase_rows.append({"label": "Ingress", "kind": "ingress", "startSec": horizon_s, "endSec": target_eta_sec})

        center_line_xy = candidate.get("centerLineXY")
        part_polygon_xy = candidate.get("partPolygonXY")
        part_width_m = float(candidate.get("partWidthM", 0.0) or 0.0)
        db_row = self._next_collab_entry_tprime_db_row(part_width_m)
        if not isinstance(db_row, dict):
            db_row = _largest_sep_covering_db_row_for_width(part_width_m)
        return {
            "source": "assignment_path_1",
            "aircraftID": int(aid),
            "originXY": (float(origin_xy[0]), float(origin_xy[1])),
            "originHeadingDeg": float(heading_deg),
            "pieceIndex": int(candidate.get("pieceIndex", 0) or 0),
            "targetLabel": str(candidate.get("targetLabel", "") or ""),
            "routeXY": route_xy,
            "markerRows": marker_rows,
            "timelineRows": timeline_rows,
            "phaseRows": phase_rows,
            "estimatedTotalSec": float(target_eta_sec),
            "targetFaceXY": target_point_xy,
            "targetXY": target_point_xy,
            "tangentXY": tangent_point_xy,
            "tangentLabel": str(tangent_label or "T"),
            "horizonSec": float(horizon_s),
            "branch": branch,
            "entryAngleDeg": float(candidate.get("entryAngleDeg", 0.0) or 0.0),
            "segmentLenM": float(candidate.get("segmentLenM", 0.0) or 0.0),
            "selectionScore": float(candidate.get("selectionScore", 0.0) or 0.0),
            "centerLineXY": [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in center_line_xy
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
            ] if isinstance(center_line_xy, list) else [],
            "partPolygonXY": [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in part_polygon_xy
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
            ] if isinstance(part_polygon_xy, list) else [],
            "partWidthM": float(part_width_m),
            "dbSepM": float(db_row.get("sep", 0.0) or 0.0) if isinstance(db_row, dict) else 0.0,
            "dbWidthM": float(db_row.get("width", 0.0) or 0.0) if isinstance(db_row, dict) else 0.0,
            "midLineLengthM": float(candidate.get("midLineLengthM", 0.0) or 0.0),
            "bearingDeg": float(candidate.get("bearingDeg", 0.0) or 0.0),
        }

    def _build_make_waypoint_row(
        self,
        base_row: Dict[str, Any],
        *,
        source: str = "make_waypoint",
        waypoint_end_label: str = "MP_E_1",
    ) -> Optional[Dict[str, Any]]:
        aid = int(base_row.get("aircraftID", 0) or 0)
        if aid <= 0:
            return None
        origin_xy_raw = base_row.get("originXY")
        origin_heading_raw = base_row.get("originHeadingDeg")
        if (
            isinstance(origin_xy_raw, (tuple, list))
            and len(origin_xy_raw) >= 2
            and origin_heading_raw is not None
        ):
            origin_xy = (float(origin_xy_raw[0]), float(origin_xy_raw[1]))
            heading_deg = float(origin_heading_raw)
        else:
            fallback_origin = None
            if str(base_row.get("source", "") or "") == "next_mission":
                for nm_row in self.state.next_mission_rows:
                    if not isinstance(nm_row, dict):
                        continue
                    if int(nm_row.get("aircraftID", 0) or 0) != aid:
                        continue
                    mp_end_raw = nm_row.get("mpEndXY")
                    bearing_raw = nm_row.get("bearingDeg")
                    if (
                        isinstance(mp_end_raw, (tuple, list))
                        and len(mp_end_raw) >= 2
                        and bearing_raw is not None
                    ):
                        fallback_origin = (
                            (float(mp_end_raw[0]), float(mp_end_raw[1])),
                            float(bearing_raw),
                        )
                        break
            if fallback_origin is not None:
                origin_xy, heading_deg = fallback_origin
            else:
                uav_state = self._uav_state_for_aircraft(aid)
                if uav_state is None:
                    return None
                origin_xy, heading_deg = uav_state

        tangent_xy = base_row.get("tangentXY")
        target_xy = base_row.get("targetXY")
        if not (
            isinstance(tangent_xy, (tuple, list)) and len(tangent_xy) >= 2
            and isinstance(target_xy, (tuple, list)) and len(target_xy) >= 2
        ):
            return None

        tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
        target_point_xy = (float(target_xy[0]), float(target_xy[1]))

        branch = str(base_row.get("branch", "") or "")
        horizon_s = float(base_row.get("horizonSec", 0.0) or 0.0)
        tangent_label = str(base_row.get("tangentLabel", "") or "T")
        route_xy, marker_rows, timeline_rows = self._build_turn_prefix_rows(
            origin_xy,
            heading_deg,
            branch=branch,
            horizon_s=horizon_s,
            tangent_point_xy=tangent_point_xy,
            tangent_label=tangent_label,
            suppress_last_turn_label=(tangent_label != "T"),
        )

        # Determine start point: T' if exists, else T
        entry_t_prime_raw = base_row.get("entryTPrimeXY")
        if isinstance(entry_t_prime_raw, (tuple, list)) and len(entry_t_prime_raw) >= 2:
            start_xy = (float(entry_t_prime_raw[0]), float(entry_t_prime_raw[1]))
            start_label = "T'"
        else:
            start_xy = tangent_point_xy
            start_label = tangent_label

        # Ingress direction: T → FA/FB (target)
        ingress_dx = target_point_xy[0] - tangent_point_xy[0]
        ingress_dy = target_point_xy[1] - tangent_point_xy[1]
        ingress_len = math.hypot(ingress_dx, ingress_dy)
        if ingress_len <= 1e-6:
            return None
        ux = ingress_dx / ingress_len
        uy = ingress_dy / ingress_len

        # Shape length: depth of part along ingress (NA→FA), NOT width (NA→NB)
        shape_length_m = 0.0
        entry_endpoints_raw = base_row.get("entryLineEndpointsXY")
        if isinstance(entry_endpoints_raw, list) and len(entry_endpoints_raw) >= 2:
            ep0_raw = entry_endpoints_raw[0]
            ep1_raw = entry_endpoints_raw[1]
            if (
                isinstance(ep0_raw, (tuple, list)) and len(ep0_raw) >= 2
                and isinstance(ep1_raw, (tuple, list)) and len(ep1_raw) >= 2
            ):
                ep0 = (float(ep0_raw[0]), float(ep0_raw[1]))
                ep1 = (float(ep1_raw[0]), float(ep1_raw[1]))
                # Project near face endpoints and target onto ingress axis
                near_proj = min(
                    ep0[0] * ux + ep0[1] * uy,
                    ep1[0] * ux + ep1[1] * uy,
                )
                far_proj = target_point_xy[0] * ux + target_point_xy[1] * uy
                shape_length_m = abs(far_proj - near_proj)
        if shape_length_m <= 1e-6:
            shape_length_m = float(base_row.get("midLineLengthM", 0.0) or 0.0)
        # midLineLengthM = actual NB→FB euclidean distance.
        # When ingress is diagonal, the projection-based shape_length can be
        # shorter than NB→FB.  Ensure shape_length is never below that.
        _mid_line_len_m = float(base_row.get("midLineLengthM", 0.0) or 0.0)
        if _mid_line_len_m > shape_length_m:
            shape_length_m = float(_mid_line_len_m)

        # MP_End: start_xy + shape_length along ingress direction
        mp_end_xy = (
            start_xy[0] + ux * shape_length_m,
            start_xy[1] + uy * shape_length_m,
        )

        # Build route
        if _distance(route_xy[-1], start_xy) > 1e-6:
            route_xy.append(start_xy)
        if _distance(route_xy[-1], mp_end_xy) > 1e-6:
            route_xy.append(mp_end_xy)

        # ETA calculations — use DB vel (km/h → m/s) for ingress+mission, fallback to turn speed
        resolved_vel_raw = base_row.get("resolvedVelMps")
        mission_speed_mps = float(resolved_vel_raw) / 3.6 if resolved_vel_raw is not None and float(resolved_vel_raw) > 0.0 else float(TURN_PREVIEW_SPEED_MPS)
        start_eta_sec = horizon_s + (_distance(tangent_point_xy, start_xy) / mission_speed_mps)
        end_eta_sec = start_eta_sec + (_distance(start_xy, mp_end_xy) / mission_speed_mps)

        # Markers
        if start_label == "T'" and _distance(tangent_point_xy, start_xy) > 1e-6:
            marker_rows.append({"xy": start_xy, "label": start_label, "kind": "waypoint_start", "etaSec": start_eta_sec})
            timeline_rows.append({"label": start_label, "kind": "waypoint_start", "etaSec": start_eta_sec})
        marker_rows.append({"xy": mp_end_xy, "label": str(waypoint_end_label), "kind": "waypoint_end", "etaSec": end_eta_sec})
        timeline_rows.append({"label": str(waypoint_end_label), "kind": "waypoint_end", "etaSec": end_eta_sec})

        phase_rows: List[Dict[str, Any]] = []
        if horizon_s > 1e-6:
            phase_rows.append({"label": "Turn", "kind": "turn", "startSec": 0.0, "endSec": horizon_s})
        if start_eta_sec > horizon_s + 1e-6:
            phase_rows.append({"label": "Ingress", "kind": "ingress", "startSec": horizon_s, "endSec": start_eta_sec})
        if end_eta_sec > start_eta_sec + 1e-6:
            phase_rows.append({"label": "Mission", "kind": "waypoint", "startSec": start_eta_sec, "endSec": end_eta_sec})

        part_width_m = float(base_row.get("partWidthM", 0.0) or 0.0)
        sep_cand_m = float(base_row.get("sepCandM", 0.0) or 0.0)
        resolved_fov = base_row.get("resolvedFovDeg")
        resolved_vel = base_row.get("resolvedVelMps")
        bearing_deg = float(base_row.get("bearingDeg", 0.0) or 0.0)

        part_polygon_xy_raw = base_row.get("partPolygonXY")
        part_polygon_xy = [
            (float(p[0]), float(p[1]))
            for p in part_polygon_xy_raw
            if isinstance(p, (tuple, list)) and len(p) >= 2
        ] if isinstance(part_polygon_xy_raw, list) else []

        return {
            "source": str(source),
            "aircraftID": int(aid),
            "originXY": (float(origin_xy[0]), float(origin_xy[1])),
            "originHeadingDeg": float(heading_deg),
            "pieceIndex": int(base_row.get("pieceIndex", 0) or 0),
            "targetLabel": str(base_row.get("targetLabel", "") or ""),
            "routeXY": route_xy,
            "markerRows": marker_rows,
            "timelineRows": timeline_rows,
            "phaseRows": phase_rows,
            "estimatedTotalSec": float(end_eta_sec),
            "tangentXY": tangent_point_xy,
            "targetXY": target_point_xy,
            "waypointStartXY": start_xy,
            "waypointEndXY": mp_end_xy,
            "waypointEndLabel": str(waypoint_end_label),
            "targetFaceXY": mp_end_xy,
            "horizonSec": float(horizon_s),
            "branch": branch,
            "startLabel": start_label,
            "tangentLabel": tangent_label,
            "shapeLengthM": float(shape_length_m),
            "partWidthM": float(part_width_m),
            "sepCandM": float(sep_cand_m),
            "resolvedDbFovDeg": base_row.get("resolvedDbFovDeg"),
            "resolvedBaseFovDeg": base_row.get("resolvedBaseFovDeg"),
            "resolvedFovDeg": resolved_fov,
            "resolvedFootM": base_row.get("resolvedFootM"),
            "resolvedVelMps": resolved_vel,
            "gsdSepM": base_row.get("gsdSepM"),
            "gsdPromotedFromFov": base_row.get("gsdPromotedFromFov"),
            "entryTPrimeXY": base_row.get("entryTPrimeXY"),
            "entryLineEndpointsXY": base_row.get("entryLineEndpointsXY"),
            "partPolygonXY": part_polygon_xy,
            "bearingDeg": float(bearing_deg),
        }

    def _build_check_mission_row(
        self,
        base_row: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        aid = int(base_row.get("aircraftID", 0) or 0)
        if aid <= 0:
            return None
        uav_state = self._uav_state_for_aircraft(aid)
        if uav_state is None:
            return None
        origin_xy, heading_deg = uav_state

        tangent_xy = base_row.get("tangentXY")
        target_xy = base_row.get("targetXY")
        part_polygon_xy_raw = base_row.get("partPolygonXY")
        if not (
            isinstance(tangent_xy, (tuple, list))
            and len(tangent_xy) >= 2
            and isinstance(target_xy, (tuple, list))
            and len(target_xy) >= 2
            and isinstance(part_polygon_xy_raw, list)
            and len(part_polygon_xy_raw) >= 3
        ):
            return None

        tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
        target_point_xy = (float(target_xy[0]), float(target_xy[1]))
        part_polygon_xy = [
            (float(point_xy[0]), float(point_xy[1]))
            for point_xy in part_polygon_xy_raw
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
        ]
        if len(part_polygon_xy) < 3:
            return None

        bearing_deg = float(base_row.get("bearingDeg", 0.0) or 0.0)
        part_overlay = self._mid_line_overlay_geometry(part_polygon_xy, bearing_deg)
        if not isinstance(part_overlay, dict):
            return None

        # Use the actual aircraft origin (not the tangent point) so the near/far
        # face selection is consistent with the other replan paths. When the
        # aircraft is far from the area the tangent point is much closer to the
        # polygon than the aircraft itself, which would flip the near/far face
        # decision and place WP_E_0 on the wrong side.
        face_points = self._make_path_face_points(part_overlay, origin_xy)
        if not isinstance(face_points, dict):
            return None
        near_line_xy_raw = face_points.get("nearFaceLineXY")
        near_face_xy_raw = face_points.get("nearFaceXY")
        target_face_xy_raw = face_points.get("targetFaceXY")
        if not (isinstance(near_line_xy_raw, list) and len(near_line_xy_raw) >= 2):
            return None
        if not (
            isinstance(near_face_xy_raw, (tuple, list))
            and len(near_face_xy_raw) >= 2
            and isinstance(target_face_xy_raw, (tuple, list))
            and len(target_face_xy_raw) >= 2
        ):
            return None
        near_line_xy = [
            (float(point_xy[0]), float(point_xy[1]))
            for point_xy in near_line_xy_raw
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
        ]
        if len(near_line_xy) < 2:
            return None
        near_face_xy = (float(near_face_xy_raw[0]), float(near_face_xy_raw[1]))
        target_face_xy = (float(target_face_xy_raw[0]), float(target_face_xy_raw[1]))

        branch = str(base_row.get("branch", "") or "")
        horizon_s = float(base_row.get("horizonSec", 0.0) or 0.0)
        tangent_label = str(base_row.get("tangentLabel", "") or "T")
        route_xy, marker_rows, timeline_rows = self._build_turn_prefix_rows(
            origin_xy,
            heading_deg,
            branch=branch,
            horizon_s=horizon_s,
            tangent_point_xy=tangent_point_xy,
            tangent_label=tangent_label,
            suppress_last_turn_label=(tangent_label != "T"),
        )

        near_width_m = _distance(near_line_xy[0], near_line_xy[-1])
        db_row = self._next_collab_entry_tprime_db_row(near_width_m)
        sep_m = float(db_row.get("sep", 0.0) or 0.0) if isinstance(db_row, dict) else 0.0
        mission_distance_m = self._distance_point_to_segment(
            tangent_point_xy,
            near_line_xy[0],
            near_line_xy[-1],
        )

        mission_start_xy = near_face_xy
        start_label = "M"
        if sep_m > 1e-6:
            mission_start_xy = self._project_along_xy(
                near_face_xy,
                target_face_xy,
                sep_m,
            )

        _cm_vel = base_row.get("resolvedVelMps") if isinstance(base_row, dict) else None
        _cm_mps = float(_cm_vel) / 3.6 if _cm_vel is not None and float(_cm_vel) > 0.0 else float(TURN_PREVIEW_SPEED_MPS)
        start_eta_sec = horizon_s + (_distance(tangent_point_xy, mission_start_xy) / _cm_mps)
        if _distance(route_xy[-1], mission_start_xy) > 1e-6:
            route_xy.append(mission_start_xy)
        marker_rows.append({"xy": mission_start_xy, "label": start_label, "kind": "waypoint_start", "etaSec": start_eta_sec})
        timeline_rows.append({"label": start_label, "kind": "waypoint_start", "etaSec": start_eta_sec})

        phase_rows: List[Dict[str, Any]] = []
        if horizon_s > 1e-6:
            phase_rows.append({"label": "Turn", "kind": "turn", "startSec": 0.0, "endSec": horizon_s})
        if start_eta_sec > horizon_s + 1e-6:
            phase_rows.append({"label": "MissionStart", "kind": "ingress", "startSec": horizon_s, "endSec": start_eta_sec})

        return {
            **dict(base_row),
            "source": "check_mission",
            "routeXY": route_xy,
            "markerRows": marker_rows,
            "timelineRows": timeline_rows,
            "phaseRows": phase_rows,
            "estimatedTotalSec": float(start_eta_sec),
            "missionDistanceM": float(mission_distance_m),
            "nearWidthM": float(near_width_m),
            "dbSepM": float(sep_m),
            "dbWidthM": float(db_row.get("width", 0.0) or 0.0) if isinstance(db_row, dict) else 0.0,
            "missionStartXY": mission_start_xy,
            "missionStartLabel": start_label,
            "nearLineXY": near_line_xy,
            "nearFaceXY": near_face_xy,
            "targetFaceXY": target_face_xy,
            "sweepLineListXY": [near_line_xy],
            "tangentLabel": tangent_label,
        }

    def _assign_split_result_by_prediction_distance(self, split_result: SplitRunResult) -> Dict[str, Any]:
        prediction_map = self._uav_prediction_points_by_id()
        assigned_pieces = 0
        uav_summary: Dict[int, int] = {}
        piece_lines: List[str] = []

        for piece in split_result.pieces:
            piece.assigned_uav = None

        if not prediction_map:
            return {
                "pieceCount": int(len(split_result.pieces)),
                "assignedPieces": 0,
                "uavSummary": {},
                "pieceLines": piece_lines,
            }

        target_items: List[Tuple[SplitPiece, Tuple[float, float]]] = []
        for piece in split_result.pieces:
            target_xy = self._piece_assignment_target_xy(piece)
            if target_xy is not None:
                target_items.append((piece, target_xy))

        usable_uavs = sorted(int(aid) for aid in prediction_map.keys())
        if not target_items or not usable_uavs:
            return {
                "pieceCount": int(len(split_result.pieces)),
                "assignedPieces": 0,
                "uavSummary": {},
                "pieceLines": piece_lines,
            }

        reference_bearing_deg = self._mid_line_reference_bearing_deg(
            [piece for piece, _target_xy in target_items]
        )
        turn_guard_enabled = reference_bearing_deg is not None
        turn_radius_m = max(0.0, float(self._default_turn_radius_m()))
        near_turn_threshold_m = max(0.0, turn_radius_m * 1.2)
        turn_penalty_m = max(3000.0, turn_radius_m * 8.0)
        turn_info_by_candidate: Dict[Tuple[int, int], Dict[str, Any]] = {}
        feasible_aircraft_by_piece: Dict[int, set[int]] = {}
        if turn_guard_enabled:
            for piece, _target_xy in target_items:
                piece_idx = int(piece.piece_index or 0)
                for aid in usable_uavs:
                    preview = self._assignment_t0_preview_for_piece(
                        piece,
                        int(aid),
                        float(reference_bearing_deg),
                    )
                    feasible = isinstance(preview, dict)
                    turn_info_by_candidate[(piece_idx, int(aid))] = {
                        "feasible": bool(feasible),
                        "branch": str(preview.get("t0Branch", "") or "") if isinstance(preview, dict) else "",
                    }
                    if feasible:
                        feasible_aircraft_by_piece.setdefault(piece_idx, set()).add(int(aid))

        def _best_candidate_for_target(
            piece: SplitPiece,
            target_xy: Tuple[float, float],
            aid: int,
        ) -> Tuple[float, float, str, Dict[str, Any]]:
            left_xy, right_xy = prediction_map[int(aid)]
            uav_state = self._uav_state_for_aircraft(int(aid))
            preferred_branch: Optional[str] = None
            if uav_state is not None:
                preferred_branch = self._turn_branch_toward_target(
                    uav_state[0],
                    float(uav_state[1]),
                    target_xy,
                )
            left_dist = _distance(target_xy, left_xy)
            right_dist = _distance(target_xy, right_xy)
            if preferred_branch == "L":
                dist_m, branch = left_dist, "L"
            elif preferred_branch == "R":
                dist_m, branch = right_dist, "R"
            elif left_dist <= right_dist:
                dist_m, branch = left_dist, "L"
            else:
                dist_m, branch = right_dist, "R"

            piece_idx = int(piece.piece_index or 0)
            turn_info = dict(turn_info_by_candidate.get((piece_idx, int(aid))) or {})
            penalty_m = 0.0
            feasible_alternatives = feasible_aircraft_by_piece.get(piece_idx, set())
            if (
                turn_guard_enabled
                and feasible_alternatives
                and not bool(turn_info.get("feasible", False))
                and float(dist_m) <= near_turn_threshold_m + 1e-6
            ):
                penalty_m = float(turn_penalty_m)
                turn_info["penaltyM"] = float(penalty_m)
                turn_info["nearThresholdM"] = float(near_turn_threshold_m)
            return float(dist_m), float(dist_m) + float(penalty_m), branch, turn_info

        assigned_records: List[Tuple[SplitPiece, int, str, float, float, Dict[str, Any]]] = []

        if len(target_items) <= len(usable_uavs):
            best_total = float("inf")
            best_records: List[Tuple[SplitPiece, int, str, float, float, Dict[str, Any]]] = []
            for aid_perm in itertools.permutations(usable_uavs, len(target_items)):
                total = 0.0
                candidate_records: List[Tuple[SplitPiece, int, str, float, float, Dict[str, Any]]] = []
                for (piece, target_xy), aid in zip(target_items, aid_perm):
                    dist_m, score_m, branch, turn_info = _best_candidate_for_target(piece, target_xy, aid)
                    total += float(score_m)
                    candidate_records.append((piece, int(aid), branch, float(dist_m), float(score_m), dict(turn_info)))
                if total < best_total:
                    best_total = total
                    best_records = candidate_records
            assigned_records = best_records
        else:
            used_uavs: set[int] = set()
            for piece, target_xy in target_items:
                candidate_uavs = [aid for aid in usable_uavs if aid not in used_uavs]
                if not candidate_uavs:
                    candidate_uavs = list(usable_uavs)

                best_aid = 0
                best_branch = ""
                best_dist = float("inf")
                best_score = float("inf")
                best_turn_info: Dict[str, Any] = {}
                for aid in candidate_uavs:
                    dist_m, score_m, branch, turn_info = _best_candidate_for_target(piece, target_xy, aid)
                    if score_m < best_score:
                        best_dist = float(dist_m)
                        best_score = float(score_m)
                        best_aid = int(aid)
                        best_branch = branch
                        best_turn_info = dict(turn_info)
                if best_aid <= 0:
                    continue
                assigned_records.append((piece, best_aid, best_branch, best_dist, best_score, best_turn_info))
                used_uavs.add(best_aid)

        turn_penalty_count = sum(
            1
            for _piece, _aid, _branch, dist_m, score_m, _turn_info in assigned_records
            if float(score_m) > float(dist_m) + 1e-6
        )
        for piece, best_aid, best_branch, best_dist, best_score, turn_info in assigned_records:
            piece.assigned_uav = int(best_aid)
            assigned_pieces += 1
            uav_summary[best_aid] = int(uav_summary.get(best_aid, 0)) + 1
            turn_suffix = ""
            if turn_guard_enabled:
                if bool(turn_info.get("feasible", False)):
                    turn_branch = str(turn_info.get("branch", "") or "")
                    turn_suffix = f", T0 {turn_branch}" if turn_branch else ", T0 ok"
                elif float(best_score) > float(best_dist) + 1e-6:
                    turn_suffix = ", T0 penalized"
                elif feasible_aircraft_by_piece.get(int(piece.piece_index or 0), set()):
                    turn_suffix = ", T0 no-penalty"
            piece_lines.append(
                f"  P{piece.piece_index}: UAV{best_aid} "
                f"({best_branch}, {best_dist:.1f}m{turn_suffix})"
            )

        return {
            "pieceCount": int(len(split_result.pieces)),
            "assignedPieces": int(assigned_pieces),
            "uavSummary": {int(k): int(v) for k, v in sorted(uav_summary.items())},
            "pieceLines": piece_lines,
            "turnGuardEnabled": bool(turn_guard_enabled),
            "turnPenaltyCount": int(turn_penalty_count),
        }

    def _largest_polygon_xy(self, geom: Any) -> Optional[Polygon]:
        if geom is None:
            return None
        candidate = geom
        if getattr(candidate, "is_empty", True):
            return None
        if isinstance(candidate, Polygon):
            poly = candidate
        else:
            gt = str(getattr(candidate, "geom_type", ""))
            if gt not in {"MultiPolygon", "GeometryCollection"}:
                return None
            polys = [g for g in getattr(candidate, "geoms", []) if isinstance(g, Polygon) and not g.is_empty]
            if not polys:
                return None
            poly = max(polys, key=lambda g: g.area)
        if not poly.is_valid:
            fixed = poly.buffer(0)
            if isinstance(fixed, Polygon) and not fixed.is_empty:
                poly = fixed
            else:
                polys = [g for g in getattr(fixed, "geoms", []) if isinstance(g, Polygon) and not g.is_empty]
                if not polys:
                    return None
                poly = max(polys, key=lambda g: g.area)
        return poly if not poly.is_empty and poly.area > 1e-6 else None

    def _mission_polygon_xy(self) -> Optional[Polygon]:
        if self.state.mission_kind != MISSION_AREA or len(self.state.mission_points_xy) < 3:
            return None
        return self._largest_polygon_xy(Polygon(self.state.mission_points_xy))

    def _stage2_grid_size_m(self, mission_poly: Polygon) -> float:
        minx, miny, maxx, maxy = mission_poly.bounds
        width_m = max(0.0, float(maxx - minx))
        height_m = max(0.0, float(maxy - miny))
        if width_m <= float(STAGE2_GRID_BOUND_SMALL_M) and height_m <= float(STAGE2_GRID_BOUND_SMALL_M):
            return float(STAGE2_GRID_SIZE_SMALL_M)
        if width_m <= float(STAGE2_GRID_BOUND_MEDIUM_M) and height_m <= float(STAGE2_GRID_BOUND_MEDIUM_M):
            return float(STAGE2_GRID_SIZE_MEDIUM_M)
        return float(STAGE2_GRID_SIZE_LARGE_M)

    def _piece_polygon_xy(self, piece: SplitPiece) -> Optional[Polygon]:
        data = piece.data if isinstance(piece.data, dict) else {}
        coords = coords_to_xy(data.get("coordinateList", []))
        if len(coords) < 3:
            return None
        return self._largest_polygon_xy(Polygon(coords))

    def _path_row_piece_polygon_xy(self, path_row: Dict[str, Any]) -> Optional[Polygon]:
        part_polygon_xy_raw = path_row.get("partPolygonXY")
        if isinstance(part_polygon_xy_raw, list):
            part_polygon_xy = [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in part_polygon_xy_raw
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
            ]
            if len(part_polygon_xy) >= 3:
                polygon = self._largest_polygon_xy(Polygon(part_polygon_xy).buffer(0))
                if polygon is not None and not polygon.is_empty:
                    return polygon

        aid = int(path_row.get("aircraftID", 0) or 0)
        piece_idx = int(path_row.get("pieceIndex", 0) or 0)
        if self.state.split_result is not None:
            for piece in self.state.split_result.pieces:
                if (
                    int(piece.piece_index or 0) == piece_idx
                    and int(piece.assigned_uav or 0) == aid
                ):
                    return self._piece_polygon_xy(piece)
        return None

    def _visibility_segment_by_aircraft_id(self, aircraft_id: int) -> Optional[Dict[str, Any]]:
        aid = int(aircraft_id)
        for row in self.state.visibility_segments:
            if not isinstance(row, dict):
                continue
            if int(row.get("aircraftID", 0) or 0) == aid:
                return row
        return None

    def _stage2_ratio_map(self, active_uav_ids: Sequence[int]) -> Dict[int, float]:
        out: Dict[int, float] = {}
        for aid in active_uav_ids:
            spin = self.stage2_ratio_spins.get(int(aid))
            value = float(spin.value()) if spin is not None else 0.0
            out[int(aid)] = max(0.0, value)
        return out

    def _stage2_piece_segment(self, aircraft_id: int, piece: SplitPiece) -> Optional[Dict[str, Any]]:
        cached = self._visibility_segment_by_aircraft_id(int(aircraft_id))
        if cached is not None:
            return cached
        uav_state = self._uav_state_for_aircraft(int(aircraft_id))
        target_xy = self._piece_assignment_target_xy(piece)
        if uav_state is None or target_xy is None:
            return None
        return self._find_visibility_segment(
            int(aircraft_id),
            uav_state[0],
            float(uav_state[1]),
            target_xy,
        )

    def _stage2_seed_xy(self, aircraft_id: int, piece: SplitPiece) -> Tuple[float, float]:
        piece_poly = self._piece_polygon_xy(piece)
        centroid_xy_val = self._piece_assignment_target_xy(piece)
        if centroid_xy_val is None and piece_poly is not None:
            centroid_xy_val = (float(piece_poly.centroid.x), float(piece_poly.centroid.y))
        if centroid_xy_val is None:
            uav_state = self._uav_state_for_aircraft(int(aircraft_id))
            if uav_state is not None:
                return uav_state[0]
            return 0.0, 0.0

        anchor_xy = centroid_xy_val
        seg = self._stage2_piece_segment(int(aircraft_id), piece)
        if isinstance(seg, dict):
            start_xy = seg.get("startXY")
            if isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2:
                anchor_xy = (float(start_xy[0]), float(start_xy[1]))
        else:
            uav_state = self._uav_state_for_aircraft(int(aircraft_id))
            if uav_state is not None:
                anchor_xy = uav_state[0]

        blend = float(STAGE2_ANCHOR_BLEND)
        return (
            ((1.0 - blend) * float(centroid_xy_val[0])) + (blend * float(anchor_xy[0])),
            ((1.0 - blend) * float(centroid_xy_val[1])) + (blend * float(anchor_xy[1])),
        )

    def _stage2_entry_time_sec(self, aircraft_id: int, piece: SplitPiece) -> float:
        target_xy = self._piece_assignment_target_xy(piece)
        if target_xy is None:
            return 0.0

        seg = self._stage2_piece_segment(int(aircraft_id), piece)
        if isinstance(seg, dict):
            start_xy = seg.get("startXY")
            if isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2:
                start = (float(start_xy[0]), float(start_xy[1]))
                horizon_s = float(seg.get("horizonSec", 0.0) or 0.0)
                return horizon_s + (_distance(start, target_xy) / float(TURN_PREVIEW_SPEED_MPS))

        uav_state = self._uav_state_for_aircraft(int(aircraft_id))
        if uav_state is None:
            return 0.0
        return _distance(uav_state[0], target_xy) / float(TURN_PREVIEW_SPEED_MPS)

    def _longest_linestring_xy(self, geom: Any) -> Optional[LineString]:
        if geom is None or getattr(geom, "is_empty", True):
            return None
        if isinstance(geom, LineString):
            return geom if float(geom.length) > 1e-6 else None
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiLineString", "GeometryCollection"}:
            return None
        lines = [g for g in getattr(geom, "geoms", []) if isinstance(g, LineString) and float(g.length) > 1e-6]
        if not lines:
            return None
        return max(lines, key=lambda g: float(g.length))

    def _linestring_segments_xy(self, geom: Any) -> List[LineString]:
        if geom is None or getattr(geom, "is_empty", True):
            return []
        if isinstance(geom, LineString):
            return [geom] if float(geom.length) > 1e-6 else []
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiLineString", "GeometryCollection"}:
            return []
        return [g for g in getattr(geom, "geoms", []) if isinstance(g, LineString) and float(g.length) > 1e-6]

    def _geometry_point_candidates_xy(self, geom: Any) -> List[Tuple[float, float]]:
        if geom is None or getattr(geom, "is_empty", True):
            return []
        if isinstance(geom, Point):
            return [(float(geom.x), float(geom.y))]
        if isinstance(geom, LineString):
            coords = list(geom.coords)
            if not coords:
                return []
            if len(coords) == 1:
                return [(float(coords[0][0]), float(coords[0][1]))]
            return [
                (float(coords[0][0]), float(coords[0][1])),
                (float(coords[-1][0]), float(coords[-1][1])),
            ]
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiPoint", "MultiLineString", "GeometryCollection"}:
            return []
        out: List[Tuple[float, float]] = []
        for item in getattr(geom, "geoms", []):
            out.extend(self._geometry_point_candidates_xy(item))
        return _dedupe_points(out, min_dist_m=1.0)

    def _first_polygon_entry_point_xy(
        self,
        polygon_xy: Sequence[Tuple[float, float]],
        start_xy: Tuple[float, float],
        toward_xy: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        if len(polygon_xy) < 3 or _distance(start_xy, toward_xy) <= 1e-6:
            return None

        poly = Polygon(polygon_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            return None
        if isinstance(poly, MultiPolygon):
            geoms = [geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty]
            if not geoms:
                return None
            poly = max(geoms, key=lambda geom: float(geom.area))

        dx = float(toward_xy[0]) - float(start_xy[0])
        dy = float(toward_xy[1]) - float(start_xy[1])
        total_len_m = math.hypot(dx, dy)
        if total_len_m <= 1e-6:
            return None
        ux = dx / total_len_m
        uy = dy / total_len_m
        pad_m = max(80.0, total_len_m * 0.25)
        probe_end_xy = (
            float(toward_xy[0]) + (ux * pad_m),
            float(toward_xy[1]) + (uy * pad_m),
        )
        probe_line = LineString([start_xy, probe_end_xy])
        candidates = self._geometry_point_candidates_xy(poly.boundary.intersection(probe_line))
        if not candidates:
            return None

        best_xy: Optional[Tuple[float, float]] = None
        best_proj_m: Optional[float] = None
        for point_xy in candidates:
            px = float(point_xy[0]) - float(start_xy[0])
            py = float(point_xy[1]) - float(start_xy[1])
            proj_m = (px * ux) + (py * uy)
            cross_m = abs((px * uy) - (py * ux))
            if proj_m <= 1e-6 or cross_m > 2.0:
                continue
            if best_proj_m is None or proj_m < best_proj_m:
                best_proj_m = float(proj_m)
                best_xy = (float(point_xy[0]), float(point_xy[1]))
        return best_xy

    def _first_sweep_segment_info(
        self,
        sweep_lines_xy: Sequence[Sequence[Tuple[float, float]]],
        start_xy: Tuple[float, float],
        toward_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Any]]:
        if not sweep_lines_xy or _distance(start_xy, toward_xy) <= 1e-6:
            return None

        dx = float(toward_xy[0]) - float(start_xy[0])
        dy = float(toward_xy[1]) - float(start_xy[1])
        path_len_m = math.hypot(dx, dy)
        if path_len_m <= 1e-6:
            return None
        ux = dx / path_len_m
        uy = dy / path_len_m
        pad_m = max(80.0, path_len_m * 0.5)
        probe_line = LineString(
            [
                start_xy,
                (float(toward_xy[0]) + (ux * pad_m), float(toward_xy[1]) + (uy * pad_m)),
            ]
        )

        best: Optional[Dict[str, Any]] = None
        for line_xy in sweep_lines_xy:
            if not isinstance(line_xy, (list, tuple)) or len(line_xy) < 2:
                continue
            seg_points = [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in line_xy
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
            ]
            if len(seg_points) < 2:
                continue
            closest_xy, dist_m = self._closest_point_on_segment_xy(
                start_xy,
                seg_points[0],
                seg_points[-1],
            )
            px = float(closest_xy[0]) - float(start_xy[0])
            py = float(closest_xy[1]) - float(start_xy[1])
            proj_m = (px * ux) + (py * uy)
            if proj_m <= -1e-6:
                continue

            intersection = LineString(seg_points).intersection(probe_line)
            hit_candidates = self._geometry_point_candidates_xy(intersection)
            hit_xy: Optional[Tuple[float, float]] = None
            hit_proj_m: Optional[float] = None
            for point_xy in hit_candidates:
                hx = float(point_xy[0]) - float(start_xy[0])
                hy = float(point_xy[1]) - float(start_xy[1])
                cand_proj_m = (hx * ux) + (hy * uy)
                if cand_proj_m <= 1e-6:
                    continue
                if hit_proj_m is None or cand_proj_m < hit_proj_m:
                    hit_proj_m = float(cand_proj_m)
                    hit_xy = (float(point_xy[0]), float(point_xy[1]))

            candidate = {
                "segmentXY": [seg_points[0], seg_points[-1]],
                "closestPointXY": closest_xy,
                "distanceM": float(dist_m),
                "projectionM": float(proj_m),
                "hitXY": hit_xy,
                "hitProjectionM": float(hit_proj_m) if hit_proj_m is not None else None,
            }
            score = (
                float(candidate["distanceM"]),
                float(candidate["projectionM"]),
            )
            if best is None:
                best = {"score": score, **candidate}
            else:
                best_score = best.get("score", (float("inf"), float("inf")))
                if score < best_score:
                    best = {"score": score, **candidate}
        if best is None:
            return None
        best.pop("score", None)
        return best

    def _ray_point_with_segment_distance_xy(
        self,
        start_xy: Tuple[float, float],
        toward_xy: Tuple[float, float],
        seg_start_xy: Tuple[float, float],
        seg_end_xy: Tuple[float, float],
        target_distance_m: float,
    ) -> Optional[Tuple[float, float]]:
        dx = float(toward_xy[0]) - float(start_xy[0])
        dy = float(toward_xy[1]) - float(start_xy[1])
        path_len_m = math.hypot(dx, dy)
        if path_len_m <= 1e-6:
            return None
        ux = dx / path_len_m
        uy = dy / path_len_m

        start_distance_m = self._distance_point_to_segment(start_xy, seg_start_xy, seg_end_xy)
        if start_distance_m <= float(target_distance_m) + 1e-6:
            return start_xy

        max_s = max(
            float(path_len_m) * 1.5,
            _distance(start_xy, seg_start_xy),
            _distance(start_xy, seg_end_xy),
            float(target_distance_m) * 2.0,
            100.0,
        )
        scan_n = 120
        prev_s = 0.0
        for idx in range(1, scan_n + 1):
            cur_s = (float(idx) / float(scan_n)) * float(max_s)
            point_xy = (
                float(start_xy[0]) + (ux * cur_s),
                float(start_xy[1]) + (uy * cur_s),
            )
            cur_dist = self._distance_point_to_segment(point_xy, seg_start_xy, seg_end_xy)
            if cur_dist <= float(target_distance_m) + 1e-6:
                lo_s = float(prev_s)
                hi_s = float(cur_s)
                for _ in range(32):
                    mid_s = 0.5 * (lo_s + hi_s)
                    mid_xy = (
                        float(start_xy[0]) + (ux * mid_s),
                        float(start_xy[1]) + (uy * mid_s),
                    )
                    mid_dist = self._distance_point_to_segment(mid_xy, seg_start_xy, seg_end_xy)
                    if mid_dist <= float(target_distance_m):
                        hi_s = mid_s
                    else:
                        lo_s = mid_s
                    return (
                        float(start_xy[0]) + (ux * hi_s),
                        float(start_xy[1]) + (uy * hi_s),
                    )
            prev_s = float(cur_s)
        return None

    def _project_along_xy(
        self,
        start_xy: Tuple[float, float],
        toward_xy: Tuple[float, float],
        distance_m: float,
    ) -> Tuple[float, float]:
        dx = float(toward_xy[0]) - float(start_xy[0])
        dy = float(toward_xy[1]) - float(start_xy[1])
        length_m = math.hypot(dx, dy)
        if length_m <= 1e-6:
            return (float(start_xy[0]), float(start_xy[1]))
        scale = float(distance_m) / float(length_m)
        return (
            float(start_xy[0]) + (dx * scale),
            float(start_xy[1]) + (dy * scale),
        )

    def _stage2_axis_guide_line(
        self,
        mission_poly: Polygon,
        center_xy: Tuple[float, float],
        bearing_deg: float,
    ) -> Optional[LineString]:
        minx, miny, maxx, maxy = mission_poly.bounds
        extent_m = max(float(maxx - minx), float(maxy - miny), 1.0) * 3.0
        theta = math.radians(float(bearing_deg) % 360.0)
        ux = math.sin(theta)
        uy = math.cos(theta)
        axis_line = LineString(
            [
                (float(center_xy[0]) - (ux * extent_m), float(center_xy[1]) - (uy * extent_m)),
                (float(center_xy[0]) + (ux * extent_m), float(center_xy[1]) + (uy * extent_m)),
            ]
        )
        clipped = mission_poly.intersection(axis_line)
        return self._longest_linestring_xy(clipped)

    def _stage2_guide_geom(
        self,
        mission_poly: Polygon,
        aircraft_id: int,
        piece: SplitPiece,
    ) -> Optional[Any]:
        target_xy = self._piece_assignment_target_xy(piece)
        geoms: List[Any] = []
        if target_xy is None:
            return None

        seg = self._stage2_piece_segment(int(aircraft_id), piece)
        path_points: List[Tuple[float, float]] = []
        uav_state = self._uav_state_for_aircraft(int(aircraft_id))
        if uav_state is not None:
            path_points.append((float(uav_state[0][0]), float(uav_state[0][1])))
        if isinstance(seg, dict):
            turn_points = seg.get("turnPoints")
            if isinstance(turn_points, list):
                for point_xy in turn_points:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        path_points.append((float(point_xy[0]), float(point_xy[1])))
            start_xy = seg.get("startXY")
            if isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2:
                path_points.append((float(start_xy[0]), float(start_xy[1])))
        path_points.append((float(target_xy[0]), float(target_xy[1])))
        path_points = _dedupe_points(path_points, min_dist_m=5.0)
        if len(path_points) >= 2:
            geoms.append(LineString(path_points))

        bearing_deg = self._piece_bearing_deg(piece)
        if bearing_deg is None and len(path_points) >= 2:
            bearing_deg = _bearing_deg_from_xy(path_points[-2], path_points[-1])
        if bearing_deg is not None:
            axis_line = self._stage2_axis_guide_line(mission_poly, target_xy, float(bearing_deg))
            if axis_line is not None:
                geoms.append(axis_line)

        if not geoms:
            return None
        if len(geoms) == 1:
            return geoms[0]
        return unary_union(geoms)

    def _stage2_guide_distance_m(
        self,
        guide_geom: Optional[Any],
        point_xy: Tuple[float, float],
    ) -> float:
        if guide_geom is None or getattr(guide_geom, "is_empty", True):
            return float("inf")
        return float(guide_geom.distance(Point(float(point_xy[0]), float(point_xy[1]))))

    def _stage2_smooth_polygon_xy(
        self,
        poly: Polygon,
        mission_poly: Polygon,
        *,
        grid_size_m: float,
    ) -> Polygon:
        if poly.is_empty:
            return poly
        smooth_buffer_m = max(8.0, float(grid_size_m) * float(STAGE2_SMOOTH_BUFFER_RATIO))
        simplify_m = max(float(STAGE2_SIMPLIFY_MIN_M), float(grid_size_m) * float(STAGE2_SIMPLIFY_RATIO))

        # Close small staircase teeth from the grid partition first.
        smoothed = poly.buffer(
            float(smooth_buffer_m),
            join_style=2,
            quad_segs=1,
        ).buffer(
            -float(smooth_buffer_m),
            join_style=2,
            quad_segs=1,
        )
        candidate = self._largest_polygon_xy(smoothed)
        if candidate is None:
            candidate = poly

        # Then aggressively collapse tiny jagged edges into a cleaner polygonal boundary.
        simplified = candidate.simplify(float(simplify_m), preserve_topology=False)
        candidate = self._largest_polygon_xy(simplified) or candidate

        clipped = mission_poly.intersection(candidate).buffer(0)
        return self._largest_polygon_xy(clipped) or poly

    def _stage2_relax_polygon_xy(
        self,
        poly: Polygon,
        clip_poly: Polygon,
        *,
        grid_size_m: float,
    ) -> Polygon:
        if poly.is_empty:
            return poly
        relax_buffer_m = max(4.0, float(grid_size_m) * float(STAGE2_PAIR_RELAX_BUFFER_RATIO))
        simplify_m = max(8.0, float(grid_size_m) * float(STAGE2_PAIR_SIMPLIFY_RATIO))
        relaxed = poly.buffer(
            float(relax_buffer_m),
            join_style=2,
            quad_segs=1,
        )
        candidate = self._largest_polygon_xy(relaxed) or poly
        simplified = candidate.simplify(float(simplify_m), preserve_topology=True)
        candidate = self._largest_polygon_xy(simplified) or candidate
        clipped = clip_poly.intersection(candidate).buffer(0)
        return self._largest_polygon_xy(clipped) or poly

    def _stage2_expand_overlap_polygon_xy(
        self,
        poly: Polygon,
        mission_poly: Polygon,
        *,
        grid_size_m: float,
    ) -> Polygon:
        if poly.is_empty:
            return poly
        overlap_buffer_m = max(3.0, float(grid_size_m) * float(STAGE2_OVERLAP_BUFFER_RATIO))
        expanded = poly.buffer(
            float(overlap_buffer_m),
            join_style=2,
            quad_segs=1,
        )
        candidate = self._largest_polygon_xy(expanded) or poly
        clipped = mission_poly.intersection(candidate).buffer(0)
        return self._largest_polygon_xy(clipped) or poly

    def _stage2_line_parts(self, geom: Any) -> List[LineString]:
        if geom is None or getattr(geom, "is_empty", True):
            return []
        if isinstance(geom, LineString):
            return [geom] if float(geom.length) > 1e-6 else []
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiLineString", "GeometryCollection"}:
            return []
        out: List[LineString] = []
        for item in getattr(geom, "geoms", []):
            out.extend(self._stage2_line_parts(item))
        return out

    def _stage2_fit_line_direction_xy(
        self,
        points_xy: Sequence[Tuple[float, float]],
    ) -> Optional[Tuple[float, float]]:
        if len(points_xy) < 2:
            return None
        mx = sum(float(p[0]) for p in points_xy) / float(len(points_xy))
        my = sum(float(p[1]) for p in points_xy) / float(len(points_xy))
        sxx = sum((float(p[0]) - mx) ** 2 for p in points_xy)
        syy = sum((float(p[1]) - my) ** 2 for p in points_xy)
        sxy = sum((float(p[0]) - mx) * (float(p[1]) - my) for p in points_xy)
        if abs(sxx) < 1e-9 and abs(syy) < 1e-9:
            return None
        angle_rad = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        vx = math.cos(angle_rad)
        vy = math.sin(angle_rad)
        norm = math.hypot(vx, vy)
        if norm <= 1e-9:
            return None
        return float(vx / norm), float(vy / norm)

    def _stage2_regularize_pair_boundary(
        self,
        poly_a: Polygon,
        poly_b: Polygon,
        *,
        seed_a_xy: Tuple[float, float],
        seed_b_xy: Tuple[float, float],
        grid_size_m: float,
    ) -> Tuple[Polygon, Polygon]:
        shared_geom = poly_a.boundary.intersection(poly_b.boundary)
        shared_lines = [
            line
            for line in self._stage2_line_parts(shared_geom)
            if float(line.length) >= max(6.0, float(grid_size_m) * 0.35)
        ]
        if not shared_lines:
            return poly_a, poly_b

        shared_len = sum(float(line.length) for line in shared_lines)
        if shared_len < max(20.0, float(grid_size_m) * 1.2):
            return poly_a, poly_b

        sample_points_xy: List[Tuple[float, float]] = []
        cx = 0.0
        cy = 0.0
        total_w = 0.0
        for line in shared_lines:
            coords = [(float(x), float(y)) for (x, y) in list(line.coords)]
            sample_points_xy.extend(coords)
            mid = line.interpolate(0.5, normalized=True)
            cx += float(mid.x) * float(line.length)
            cy += float(mid.y) * float(line.length)
            total_w += float(line.length)
        if total_w <= 1e-9:
            return poly_a, poly_b

        dir_xy = self._stage2_fit_line_direction_xy(sample_points_xy)
        if dir_xy is None:
            longest = max(shared_lines, key=lambda line: float(line.length))
            coords = list(longest.coords)
            if len(coords) < 2:
                return poly_a, poly_b
            dx = float(coords[-1][0]) - float(coords[0][0])
            dy = float(coords[-1][1]) - float(coords[0][1])
            norm = math.hypot(dx, dy)
            if norm <= 1e-9:
                return poly_a, poly_b
            dir_xy = (float(dx / norm), float(dy / norm))

        union_poly = self._largest_polygon_xy(poly_a.union(poly_b))
        if union_poly is None:
            return poly_a, poly_b

        center_xy = (float(cx / total_w), float(cy / total_w))
        minx, miny, maxx, maxy = union_poly.bounds
        extent_m = max(float(maxx - minx), float(maxy - miny), 1.0) * 4.0
        cut_line = LineString(
            [
                (
                    float(center_xy[0]) - (float(dir_xy[0]) * extent_m),
                    float(center_xy[1]) - (float(dir_xy[1]) * extent_m),
                ),
                (
                    float(center_xy[0]) + (float(dir_xy[0]) * extent_m),
                    float(center_xy[1]) + (float(dir_xy[1]) * extent_m),
                ),
            ]
        )

        try:
            split_out = geom_split(union_poly, cut_line)
        except Exception:
            return poly_a, poly_b

        parts = [
            geom
            for geom in getattr(split_out, "geoms", [])
            if isinstance(geom, Polygon) and not geom.is_empty and float(geom.area) > 1e-6
        ]
        if len(parts) < 2:
            return poly_a, poly_b

        point_a = Point(float(seed_a_xy[0]), float(seed_a_xy[1]))
        point_b = Point(float(seed_b_xy[0]), float(seed_b_xy[1]))
        parts_a: List[Polygon] = []
        parts_b: List[Polygon] = []
        for part in parts:
            overlap_a = float(part.intersection(poly_a).area)
            overlap_b = float(part.intersection(poly_b).area)
            if abs(overlap_a - overlap_b) <= 1e-6:
                rep = part.representative_point()
                if float(rep.distance(point_a)) <= float(rep.distance(point_b)):
                    parts_a.append(part)
                else:
                    parts_b.append(part)
            elif overlap_a >= overlap_b:
                parts_a.append(part)
            else:
                parts_b.append(part)

        if not parts_a or not parts_b:
            return poly_a, poly_b

        new_a = self._largest_polygon_xy(unary_union(parts_a))
        new_b = self._largest_polygon_xy(unary_union(parts_b))
        if new_a is None or new_b is None:
            return poly_a, poly_b
        new_a = self._stage2_relax_polygon_xy(
            new_a,
            union_poly,
            grid_size_m=float(grid_size_m),
        )
        new_b = self._stage2_relax_polygon_xy(
            new_b,
            union_poly,
            grid_size_m=float(grid_size_m),
        )
        return new_a, new_b

    def _stage2_regularize_shared_boundaries(
        self,
        polygon_by_aid: Dict[int, Polygon],
        *,
        seed_xy_by_aid: Dict[int, Tuple[float, float]],
        grid_size_m: float,
    ) -> Dict[int, Polygon]:
        aids = sorted(int(aid) for aid in polygon_by_aid.keys())
        out: Dict[int, Polygon] = {int(aid): poly for aid, poly in polygon_by_aid.items()}
        for _ in range(1):
            changed = False
            for left_idx in range(len(aids)):
                for right_idx in range(left_idx + 1, len(aids)):
                    aid_a = int(aids[left_idx])
                    aid_b = int(aids[right_idx])
                    poly_a = out.get(int(aid_a))
                    poly_b = out.get(int(aid_b))
                    if poly_a is None or poly_b is None:
                        continue
                    new_a, new_b = self._stage2_regularize_pair_boundary(
                        poly_a,
                        poly_b,
                        seed_a_xy=seed_xy_by_aid[int(aid_a)],
                        seed_b_xy=seed_xy_by_aid[int(aid_b)],
                        grid_size_m=float(grid_size_m),
                    )
                    if (
                        float(new_a.symmetric_difference(poly_a).area) > 1e-6
                        or float(new_b.symmetric_difference(poly_b).area) > 1e-6
                    ):
                        out[int(aid_a)] = new_a
                        out[int(aid_b)] = new_b
                        changed = True
            if not changed:
                break
        return out

    def _stage2_polygon_parts(self, geom: Any) -> List[Polygon]:
        if geom is None or getattr(geom, "is_empty", True):
            return []
        if isinstance(geom, Polygon):
            return [geom] if float(geom.area) > 1e-6 else []
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiPolygon", "GeometryCollection"}:
            return []
        out: List[Polygon] = []
        for item in getattr(geom, "geoms", []):
            out.extend(self._stage2_polygon_parts(item))
        return out

    def _stage2_fill_uncovered_gaps(
        self,
        mission_poly: Polygon,
        polygon_by_aid: Dict[int, Polygon],
        *,
        seed_xy_by_aid: Dict[int, Tuple[float, float]],
        grid_size_m: float,
    ) -> Dict[int, Polygon]:
        out: Dict[int, Polygon] = {int(aid): poly for aid, poly in polygon_by_aid.items()}
        contact_tol_m = max(1.0, float(grid_size_m) * 0.35)

        for _ in range(3):
            coverage = unary_union([poly for poly in out.values() if poly is not None and not poly.is_empty])
            uncovered = mission_poly.difference(coverage)
            gap_parts = [
                gap
                for gap in self._stage2_polygon_parts(uncovered)
                if float(gap.area) > 1e-3
            ]
            if not gap_parts:
                break

            changed = False
            for gap in sorted(gap_parts, key=lambda poly: float(poly.area), reverse=True):
                rep = gap.representative_point()
                rep_xy = (float(rep.x), float(rep.y))
                candidate_aids = [
                    int(aid)
                    for aid, poly in out.items()
                    if poly is not None and not poly.is_empty and float(poly.distance(gap)) <= float(contact_tol_m)
                ]
                if not candidate_aids:
                    candidate_aids = [int(aid) for aid in out.keys()]
                if not candidate_aids:
                    continue

                best_aid = min(
                    candidate_aids,
                    key=lambda aid: (
                        -float(out[int(aid)].boundary.intersection(gap.boundary).length),
                        _distance(rep_xy, seed_xy_by_aid.get(int(aid), rep_xy)),
                        float(out[int(aid)].distance(rep)),
                        int(aid),
                    ),
                )
                merged = out[int(best_aid)].union(gap).buffer(0)
                new_poly = self._largest_polygon_xy(merged)
                if new_poly is None:
                    continue
                out[int(best_aid)] = new_poly
                changed = True

            if not changed:
                break

        return out

    def _stage2_target_area_map(
        self,
        split_result: SplitRunResult,
        piece_by_aid: Dict[int, SplitPiece],
        ratio_map: Dict[int, float],
        *,
        grid_size_m: float,
    ) -> Tuple[Dict[int, float], Dict[int, Dict[str, float]]]:
        calculate_expected_velocity(
            split_result,
            expected_paths=self.state.expected_paths,
        )

        stats_by_aid: Dict[int, Dict[str, float]] = {}
        total_area_m2 = 0.0
        total_time_sec = 0.0
        total_valid_area_m2 = 0.0
        total_valid_scan_sec = 0.0

        for aid, piece in piece_by_aid.items():
            poly = self._piece_polygon_xy(piece)
            area_m2 = float(poly.area) if poly is not None else 0.0
            total_area_m2 += area_m2
            data = piece.data if isinstance(piece.data, dict) else {}
            exp_vel = data.get("expVel") if isinstance(data.get("expVel"), dict) else {}
            scan_sec_raw = exp_vel.get("timeSelectedSec")
            scan_sec = float(scan_sec_raw) if isinstance(scan_sec_raw, (int, float)) and float(scan_sec_raw) > 1e-6 else None
            if scan_sec is not None and area_m2 > 1e-6:
                total_valid_area_m2 += float(area_m2)
                total_valid_scan_sec += float(scan_sec)
            entry_sec = float(self._stage2_entry_time_sec(int(aid), piece))
            stats_by_aid[int(aid)] = {
                "currentAreaM2": float(area_m2),
                "entrySec": float(entry_sec),
                "scanSec": float(scan_sec) if scan_sec is not None else -1.0,
                "areaRateM2ps": -1.0,
            }

        common_area_rate_m2ps = (
            float(total_valid_area_m2) / float(total_valid_scan_sec)
            if total_valid_area_m2 > 1e-6 and total_valid_scan_sec > 1e-6
            else float(STAGE2_DEFAULT_AREA_RATE_M2PS)
        )
        ratio_sum = sum(max(0.0, float(v)) for v in ratio_map.values())
        if ratio_sum <= 1e-9:
            raise ValueError("Stage 2 ratio sum must be greater than 0.")

        raw_target_area_map: Dict[int, float] = {}
        min_target_area_m2 = float(grid_size_m * grid_size_m * STAGE2_MIN_CELL_AREA_RATIO)
        for aid, meta in stats_by_aid.items():
            scan_sec = float(meta["scanSec"]) if float(meta["scanSec"]) > 0.0 else (
                float(meta["currentAreaM2"]) / max(float(common_area_rate_m2ps), 1e-6)
            )
            current_total_sec = float(meta["entrySec"]) + float(scan_sec)
            total_time_sec += float(current_total_sec)
            meta["scanSec"] = float(scan_sec)
            meta["areaRateM2ps"] = float(common_area_rate_m2ps)
            meta["areaRateMode"] = "shared"
            meta["currentTotalSec"] = float(current_total_sec)

        for aid, meta in stats_by_aid.items():
            desired_total_sec = float(total_time_sec) * (float(ratio_map.get(int(aid), 0.0)) / float(ratio_sum))
            desired_scan_sec = max(1.0, float(desired_total_sec) - float(meta["entrySec"]))
            raw_target_area_m2 = max(min_target_area_m2, desired_scan_sec * float(common_area_rate_m2ps))
            raw_target_area_map[int(aid)] = float(raw_target_area_m2)
            meta["desiredTotalSec"] = float(desired_total_sec)
            meta["desiredScanSec"] = float(desired_scan_sec)
            meta["sharedAreaRateM2ps"] = float(common_area_rate_m2ps)

        raw_sum_m2 = sum(float(v) for v in raw_target_area_map.values())
        scale = (float(total_area_m2) / float(raw_sum_m2)) if raw_sum_m2 > 1e-6 else 1.0
        target_area_map: Dict[int, float] = {}
        for aid, raw_area_m2 in raw_target_area_map.items():
            target_area_map[int(aid)] = float(raw_area_m2) * float(scale)
            stats_by_aid[int(aid)]["targetAreaM2"] = float(target_area_map[int(aid)])

        return target_area_map, stats_by_aid

    def _stage2_build_cells(
        self,
        mission_poly: Polygon,
        *,
        cell_size_m: float,
    ) -> Tuple[List[Dict[str, Any]], List[List[int]]]:
        minx, miny, maxx, maxy = mission_poly.bounds
        cell_size = max(5.0, float(cell_size_m))
        start_x = math.floor(minx / cell_size) * cell_size
        start_y = math.floor(miny / cell_size) * cell_size
        nx = max(1, int(math.ceil((maxx - start_x) / cell_size)))
        ny = max(1, int(math.ceil((maxy - start_y) / cell_size)))
        min_area_m2 = float(cell_size * cell_size * STAGE2_MIN_CELL_AREA_RATIO)

        cells: List[Dict[str, Any]] = []
        rc_to_idx: Dict[Tuple[int, int], int] = {}
        for row in range(ny):
            for col in range(nx):
                x0 = start_x + (float(col) * cell_size)
                y0 = start_y + (float(row) * cell_size)
                clipped = mission_poly.intersection(box(x0, y0, x0 + cell_size, y0 + cell_size))
                poly = self._largest_polygon_xy(clipped)
                if poly is None or float(poly.area) < float(min_area_m2):
                    continue
                point = poly.representative_point()
                idx = len(cells)
                cells.append(
                    {
                        "geom": poly,
                        "centerXY": (float(point.x), float(point.y)),
                        "areaM2": float(poly.area),
                        "rc": (int(col), int(row)),
                    }
                )
                rc_to_idx[(int(col), int(row))] = int(idx)

        neighbors: List[List[int]] = [[] for _ in cells]
        for idx, cell in enumerate(cells):
            col, row = cell["rc"]
            for dc, dr in (
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ):
                near_idx = rc_to_idx.get((int(col) + int(dc), int(row) + int(dr)))
                if near_idx is not None:
                    neighbors[idx].append(int(near_idx))

        return cells, neighbors

    def _stage2_pick_seed_cell_idx(
        self,
        cells: Sequence[Dict[str, Any]],
        seed_xy: Tuple[float, float],
        *,
        piece_poly: Optional[Polygon] = None,
        used_indices: Optional[set[int]] = None,
    ) -> Optional[int]:
        used = used_indices if used_indices is not None else set()
        best_inside: Optional[Tuple[float, int]] = None
        best_any: Optional[Tuple[float, int]] = None
        for idx, cell in enumerate(cells):
            if idx in used:
                continue
            center_xy = cell["centerXY"]
            dist_m = _distance(center_xy, seed_xy)
            point = Point(float(center_xy[0]), float(center_xy[1]))
            if piece_poly is not None and float(piece_poly.distance(point)) <= 1e-6:
                if best_inside is None or float(dist_m) < float(best_inside[0]):
                    best_inside = (float(dist_m), int(idx))
            if best_any is None or float(dist_m) < float(best_any[0]):
                best_any = (float(dist_m), int(idx))
        if best_inside is not None:
            return int(best_inside[1])
        if best_any is not None:
            return int(best_any[1])
        return None

    def _stage2_growth_score(
        self,
        cell_center_xy: Tuple[float, float],
        *,
        seed_xy: Tuple[float, float],
        anchor_xy: Tuple[float, float],
        guide_geom: Optional[Any],
        current_area_m2: float,
        target_area_m2: float,
    ) -> float:
        guide_dist_m = self._stage2_guide_distance_m(guide_geom, cell_center_xy)
        if not math.isfinite(guide_dist_m):
            guide_dist_m = (
                ((1.0 - float(STAGE2_ANCHOR_BLEND)) * _distance(cell_center_xy, seed_xy))
                + (float(STAGE2_ANCHOR_BLEND) * _distance(cell_center_xy, anchor_xy))
            )
        seed_dist_m = _distance(cell_center_xy, seed_xy)
        anchor_dist_m = _distance(cell_center_xy, anchor_xy)
        half_swath_m = float(STAGE2_MAX_SWATH_WIDTH_M) * 0.5
        over_width_m = max(0.0, float(guide_dist_m) - float(half_swath_m))
        width_penalty = (float(over_width_m) * 10.0) + ((float(over_width_m) ** 2) * 0.06)
        base_score = (
            (float(guide_dist_m) * 1.9)
            + (float(seed_dist_m) * 0.08)
            + (float(anchor_dist_m) * 0.04)
            + float(width_penalty)
        )
        overload_ratio = max(0.0, float(current_area_m2) - float(target_area_m2)) / max(float(target_area_m2), 1.0)
        return float(base_score) + (800.0 * float(overload_ratio))

    def _stage2_rebalance_area_polygons(
        self,
        mission_poly: Polygon,
        piece_by_aid: Dict[int, SplitPiece],
        target_area_map: Dict[int, float],
        *,
        grid_size_m: float,
    ) -> Tuple[Dict[int, Polygon], Dict[int, float]]:
        cells, neighbors = self._stage2_build_cells(mission_poly, cell_size_m=float(grid_size_m))
        if len(cells) < len(piece_by_aid):
            raise ValueError("Stage 2 review needs at least one grid cell per assigned UAV.")

        aids = sorted(int(aid) for aid in piece_by_aid.keys())
        piece_poly_by_aid = {int(aid): self._piece_polygon_xy(piece) for aid, piece in piece_by_aid.items()}
        seed_xy_by_aid = {int(aid): self._stage2_seed_xy(int(aid), piece) for aid, piece in piece_by_aid.items()}
        guide_geom_by_aid = {
            int(aid): self._stage2_guide_geom(mission_poly, int(aid), piece)
            for aid, piece in piece_by_aid.items()
        }
        anchor_xy_by_aid: Dict[int, Tuple[float, float]] = {}
        for aid, piece in piece_by_aid.items():
            anchor_xy = seed_xy_by_aid[int(aid)]
            seg = self._stage2_piece_segment(int(aid), piece)
            if isinstance(seg, dict):
                start_xy = seg.get("startXY")
                if isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2:
                    anchor_xy = (float(start_xy[0]), float(start_xy[1]))
            else:
                uav_state = self._uav_state_for_aircraft(int(aid))
                if uav_state is not None:
                    anchor_xy = uav_state[0]
            anchor_xy_by_aid[int(aid)] = anchor_xy

        owner_by_idx: List[int] = [0 for _ in cells]
        frontier_by_aid: Dict[int, set[int]] = {int(aid): set() for aid in aids}
        current_area_by_aid: Dict[int, float] = {int(aid): 0.0 for aid in aids}
        used_seed_idx: set[int] = set()

        for aid in aids:
            seed_idx = self._stage2_pick_seed_cell_idx(
                cells,
                seed_xy_by_aid[int(aid)],
                piece_poly=piece_poly_by_aid.get(int(aid)),
                used_indices=used_seed_idx,
            )
            if seed_idx is None:
                raise ValueError(f"Stage 2 seed cell selection failed for UAV{aid}.")
            owner_by_idx[int(seed_idx)] = int(aid)
            current_area_by_aid[int(aid)] += float(cells[int(seed_idx)]["areaM2"])
            used_seed_idx.add(int(seed_idx))

        for aid in aids:
            for idx, owner in enumerate(owner_by_idx):
                if int(owner) != int(aid):
                    continue
                for near_idx in neighbors[idx]:
                    if owner_by_idx[int(near_idx)] <= 0:
                        frontier_by_aid[int(aid)].add(int(near_idx))

        unassigned_n = sum(1 for owner in owner_by_idx if int(owner) <= 0)
        guard = 0
        while unassigned_n > 0 and guard < (len(cells) * 12):
            guard += 1
            candidate_aid: Optional[int] = None
            candidate_idx: Optional[int] = None

            underfilled = [
                int(aid)
                for aid in aids
                if frontier_by_aid[int(aid)] and (float(current_area_by_aid[int(aid)]) + 1e-6) < float(target_area_map[int(aid)])
            ]
            if underfilled:
                candidate_aid = max(
                    underfilled,
                    key=lambda aid: (
                        (float(target_area_map[int(aid)]) - float(current_area_by_aid[int(aid)]))
                        / max(float(target_area_map[int(aid)]), 1.0)
                    ),
                )
                candidate_idx = min(
                    frontier_by_aid[int(candidate_aid)],
                    key=lambda idx: self._stage2_growth_score(
                        cells[int(idx)]["centerXY"],
                        seed_xy=seed_xy_by_aid[int(candidate_aid)],
                        anchor_xy=anchor_xy_by_aid[int(candidate_aid)],
                        guide_geom=guide_geom_by_aid.get(int(candidate_aid)),
                        current_area_m2=float(current_area_by_aid[int(candidate_aid)]),
                        target_area_m2=float(target_area_map[int(candidate_aid)]),
                    ),
                )
            else:
                best_row: Optional[Tuple[float, int, int]] = None
                for aid in aids:
                    for idx in frontier_by_aid[int(aid)]:
                        score = self._stage2_growth_score(
                            cells[int(idx)]["centerXY"],
                            seed_xy=seed_xy_by_aid[int(aid)],
                            anchor_xy=anchor_xy_by_aid[int(aid)],
                            guide_geom=guide_geom_by_aid.get(int(aid)),
                            current_area_m2=float(current_area_by_aid[int(aid)]),
                            target_area_m2=float(target_area_map[int(aid)]),
                        )
                        row = (float(score), int(aid), int(idx))
                        if best_row is None or row < best_row:
                            best_row = row
                if best_row is not None:
                    candidate_aid = int(best_row[1])
                    candidate_idx = int(best_row[2])

            if candidate_aid is None or candidate_idx is None:
                unassigned_indices = [idx for idx, owner in enumerate(owner_by_idx) if int(owner) <= 0]
                if not unassigned_indices:
                    break
                best_row = min(
                    (
                        (
                            self._stage2_growth_score(
                                cells[int(idx)]["centerXY"],
                                seed_xy=seed_xy_by_aid[int(aid)],
                                anchor_xy=anchor_xy_by_aid[int(aid)],
                                guide_geom=guide_geom_by_aid.get(int(aid)),
                                current_area_m2=float(current_area_by_aid[int(aid)]),
                                target_area_m2=float(target_area_map[int(aid)]),
                            ),
                            int(aid),
                            int(idx),
                        )
                        for aid in aids
                        for idx in unassigned_indices
                    ),
                    key=lambda row: (float(row[0]), int(row[1]), int(row[2])),
                )
                candidate_aid = int(best_row[1])
                candidate_idx = int(best_row[2])

            owner_by_idx[int(candidate_idx)] = int(candidate_aid)
            current_area_by_aid[int(candidate_aid)] += float(cells[int(candidate_idx)]["areaM2"])
            for aid in aids:
                frontier_by_aid[int(aid)].discard(int(candidate_idx))
            for near_idx in neighbors[int(candidate_idx)]:
                if owner_by_idx[int(near_idx)] <= 0:
                    frontier_by_aid[int(candidate_aid)].add(int(near_idx))
            unassigned_n -= 1

        polygon_by_aid: Dict[int, Polygon] = {}
        for aid in aids:
            geoms = [cells[idx]["geom"] for idx, owner in enumerate(owner_by_idx) if int(owner) == int(aid)]
            if not geoms:
                raise ValueError(f"Stage 2 produced an empty region for UAV{aid}.")
            merged = unary_union(geoms)
            clipped = mission_poly.intersection(merged).buffer(0)
            poly = self._largest_polygon_xy(clipped)
            if poly is None:
                raise ValueError(f"Stage 2 polygon generation failed for UAV{aid}.")
            poly = self._stage2_smooth_polygon_xy(
                poly,
                mission_poly,
                grid_size_m=float(grid_size_m),
            )
            polygon_by_aid[int(aid)] = poly

        polygon_by_aid = self._stage2_regularize_shared_boundaries(
            polygon_by_aid,
            seed_xy_by_aid=seed_xy_by_aid,
            grid_size_m=float(grid_size_m),
        )
        polygon_by_aid = self._stage2_fill_uncovered_gaps(
            mission_poly,
            polygon_by_aid,
            seed_xy_by_aid=seed_xy_by_aid,
            grid_size_m=float(grid_size_m),
        )
        polygon_by_aid = self._stage2_regularize_shared_boundaries(
            polygon_by_aid,
            seed_xy_by_aid=seed_xy_by_aid,
            grid_size_m=float(grid_size_m),
        )
        for aid in aids:
            polygon_by_aid[int(aid)] = self._stage2_expand_overlap_polygon_xy(
                polygon_by_aid[int(aid)],
                mission_poly,
                grid_size_m=float(grid_size_m),
            )

        achieved_area_map: Dict[int, float] = {}
        for aid in aids:
            achieved_area_map[int(aid)] = float(polygon_by_aid[int(aid)].area)

        return polygon_by_aid, achieved_area_map

    def _stage2_apply_review_polygons(
        self,
        piece_by_aid: Dict[int, SplitPiece],
        polygon_by_aid: Dict[int, Polygon],
        ratio_map: Dict[int, float],
        target_area_map: Dict[int, float],
        *,
        grid_size_m: float,
    ) -> None:
        for aid, piece in piece_by_aid.items():
            poly = polygon_by_aid.get(int(aid))
            if poly is None:
                continue
            coords_xy = [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]]
            if len(coords_xy) < 3:
                continue
            data = copy.deepcopy(piece.data if isinstance(piece.data, dict) else {})
            alt_m = 0.0
            old_coords = data.get("coordinateList") if isinstance(data.get("coordinateList"), list) else []
            if old_coords and isinstance(old_coords[0], dict):
                alt_m = float(old_coords[0].get("altitude", 0.0) or 0.0)
            coord_list = [meters_to_coord(x, y, alt_m=alt_m) for (x, y) in coords_xy]
            data["coordinateList"] = coord_list
            data["rawCoordinateList"] = copy.deepcopy(coord_list)
            review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
            review.update(
                {
                    "stage2Rebalanced": True,
                    "targetRatio": float(ratio_map.get(int(aid), 0.0)),
                    "targetAreaM2": float(target_area_map.get(int(aid), 0.0)),
                    "reviewedAreaM2": float(poly.area),
                    "gridSizeM": float(grid_size_m),
                    "maxSwathWidthM": float(STAGE2_MAX_SWATH_WIDTH_M),
                }
            )
            data["reviewArea"] = review
            piece.data = data
            piece.assigned_uav = int(aid)

    def _has_stage2_review(self, split_result: Optional[SplitRunResult]) -> bool:
        if split_result is None:
            return False
        for piece in split_result.pieces:
            data = piece.data if isinstance(piece.data, dict) else {}
            review = data.get("reviewArea")
            if isinstance(review, dict) and bool(review.get("stage2Rebalanced", False)):
                return True
        return False

    def _uav_state_for_aircraft(self, aircraft_id: int) -> Optional[Tuple[Tuple[float, float], float]]:
        aid = int(aircraft_id)
        for idx, current_aid in enumerate(self.state.uav_ids):
            if int(current_aid) != aid:
                continue
            if idx >= len(self.state.uav_positions_xy) or idx >= len(self.state.uav_heading_deg):
                return None
            heading = self.state.uav_heading_deg[idx]
            if heading is None:
                return None
            return self.state.uav_positions_xy[idx], float(heading)
        return None

    def _find_visibility_segment(
        self,
        aircraft_id: int,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        target_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Any]]:
        radius_m = self._default_turn_radius_m()
        if self._line_avoids_turn_circles(origin_xy, target_xy, origin_xy, bearing_deg, radius_m=radius_m):
            return {
                "aircraftID": int(aircraft_id),
                "startXY": origin_xy,
                "endXY": target_xy,
                "horizonSec": 0.0,
                "branch": "direct",
                "turnPoints": [],
            }

        max_steps = max(
            1,
            int(
                math.ceil(
                    (2.0 * math.pi * radius_m)
                    / (TURN_PREVIEW_SPEED_MPS * TURN_PREVIEW_HORIZON_S)
                )
            ),
        )
        preferred_branch = self._turn_branch_toward_target(origin_xy, bearing_deg, target_xy)
        for step_idx in range(1, max_steps + 1):
            horizon_s = float(step_idx) * float(TURN_PREVIEW_HORIZON_S)
            left_xy, right_xy = self._turn_prediction_points_xy(
                origin_xy,
                bearing_deg,
                horizon_s=horizon_s,
                radius_m=radius_m,
            )
            candidates: List[Dict[str, Any]] = []
            branch_points = (("L", left_xy), ("R", right_xy))
            if preferred_branch == "L":
                branch_points = (("L", left_xy),)
            elif preferred_branch == "R":
                branch_points = (("R", right_xy),)
            for branch, candidate_xy in branch_points:
                if not self._line_avoids_turn_circles(
                    candidate_xy,
                    target_xy,
                    origin_xy,
                    bearing_deg,
                    radius_m=radius_m,
                ):
                    continue
                turn_points: List[Tuple[float, float]] = []
                turn_point_horizons: List[float] = []
                max_marker_idx = step_idx
                for prior_idx in range(1, max_marker_idx + 1):
                    prior_horizon_s = float(prior_idx) * float(TURN_PREVIEW_HORIZON_S)
                    prior_left_xy, prior_right_xy = self._turn_prediction_points_xy(
                        origin_xy,
                        bearing_deg,
                        horizon_s=prior_horizon_s,
                        radius_m=radius_m,
                    )
                    turn_points.append(prior_left_xy if branch == "L" else prior_right_xy)
                    turn_point_horizons.append(float(prior_horizon_s))
                start_xy = candidate_xy
                start_horizon_s = float(horizon_s)
                if step_idx >= 2:
                    refined = self._refine_visibility_start_xy(
                        origin_xy,
                        bearing_deg,
                        target_xy,
                        branch=branch,
                        min_horizon_s=float(step_idx - 1) * float(TURN_PREVIEW_HORIZON_S),
                        max_horizon_s=horizon_s,
                        radius_m=radius_m,
                    )
                    if refined is not None:
                        start_xy = refined["startXY"]
                        start_horizon_s = float(refined["horizonSec"])
                        if turn_points:
                            turn_points[-1] = start_xy
                            turn_point_horizons[-1] = float(start_horizon_s)
                        if (
                            len(turn_points) >= 3
                            and (turn_point_horizons[-1] - turn_point_horizons[-2])
                            < (float(TURN_PREVIEW_HORIZON_S) - 1e-6)
                        ):
                            del turn_points[-2]
                            del turn_point_horizons[-2]
                candidates.append(
                    {
                        "aircraftID": int(aircraft_id),
                        "startXY": start_xy,
                        "endXY": target_xy,
                        "horizonSec": float(start_horizon_s),
                        "branch": branch,
                        "turnPoints": turn_points,
                    }
                )
            if candidates:
                best = min(
                    candidates,
                    key=lambda row: (
                        float(row.get("horizonSec", 0.0) or 0.0),
                        _distance(row["startXY"], target_xy),
                    ),
                )
                return best
        return None

    def _ensure_ready_for_division(self, *, area_only: bool = False) -> bool:
        if not self.state.mission_points_xy or self.state.mission_kind is None:
            QMessageBox.warning(self, "임무 없음", "임무 형상을 먼저 만들어 주세요.")
            return False
        if area_only and self.state.mission_kind != MISSION_AREA:
            QMessageBox.warning(self, "영역 분할", "Area Division Run은 영역 임무에서만 사용할 수 있습니다.")
            return False
        if not self.state.uav_ids:
            QMessageBox.warning(self, "UAV 없음", "UAV 대수를 먼저 확정해 주세요.")
            return False
        if not self._uav_inputs_complete():
            QMessageBox.warning(self, "UAV 입력 부족", "모든 UAV의 위치와 heading을 입력해 주세요.")
            return False
        return True

    def _direction_debug_lines(self, split_result: SplitRunResult) -> List[str]:
        lines: List[str] = []
        for direction in split_result.directions:
            if direction.bearing_move_deg is None and direction.bearing_split_deg is None:
                continue
            area_tag = ""
            if direction.source_area_index is not None:
                area_tag = f" A{int(direction.source_area_index)}"
            text = (
                f"[DIR] M{direction.parent_order}{area_tag} ID={direction.mission_id} "
                f"move={float(direction.bearing_move_deg or 0.0):.2f} "
                f"split={float(direction.bearing_split_deg or 0.0):.2f}"
            )
            if direction.bearing_in_deg is not None:
                text += f" in={float(direction.bearing_in_deg):.2f}"
            if direction.bearing_out_deg is not None:
                text += f" out={float(direction.bearing_out_deg):.2f}"
            lines.append(text)
        return lines

    def _run_split_stage(self) -> Tuple[SplitRunResult, List[str]]:
        self._cmpk_payload = self._build_cmpk_payload()
        self._mrpk_payload = self._build_mrpk_payload()
        use_replan_flow = self._is_replan_flow()
        split_result = run_split_pipeline(
            self._cmpk_payload,
            self._mrpk_payload,
            list(self.state.uav_ids),
            apply_assignment=use_replan_flow,
            apply_scheduling=False,
        )
        stage_lines = [f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}"]
        if use_replan_flow:
            stage_lines.append(f"[1a] preassign {self._assignment_summary_text(split_result)}")
        stage_lines.extend(self._direction_debug_lines(split_result))
        return split_result, stage_lines

    def _visible_uav_ids(self) -> List[int]:
        out: List[int] = []
        mapping = (
            (4, getattr(self, "chk_view_uav4", None)),
            (5, getattr(self, "chk_view_uav5", None)),
            (6, getattr(self, "chk_view_uav6", None)),
        )
        for aid, widget in mapping:
            if widget is None or widget.isChecked():
                out.append(int(aid))
        return out

    def _start_area_input(self) -> None:
        self.state.mode = MODE_DRAW_AREA
        self.state.mission_kind = MISSION_AREA
        self.state.draft_points_xy = []
        self.state.mission_points_xy = []
        self.state.line_width_pending = False
        self.state.uav_ids = []
        self.state.uav_positions_xy = []
        self.state.uav_heading_deg = []
        self._clear_plan_result()
        self._set_result("영역 입력을 시작했습니다.")
        self._refresh_ui()

    def _start_line_input(self) -> None:
        self.state.mode = MODE_DRAW_LINE
        self.state.mission_kind = MISSION_LINE
        self.state.draft_points_xy = []
        self.state.mission_points_xy = []
        self.state.line_width_m = float(self.spin_line_width.value())
        self.state.line_width_pending = False
        self.state.uav_ids = []
        self.state.uav_positions_xy = []
        self.state.uav_heading_deg = []
        self._clear_plan_result()
        self._set_result("Line 입력을 시작했습니다.")
        self._refresh_ui()

    def _undo_last_point(self) -> None:
        if self.state.mode in (MODE_DRAW_AREA, MODE_DRAW_LINE, MODE_LINE_WIDTH_PENDING):
            if self.state.draft_points_xy:
                self.state.draft_points_xy.pop()
                if self.state.mode == MODE_LINE_WIDTH_PENDING and len(self.state.draft_points_xy) >= 2:
                    self.state.mode = MODE_DRAW_LINE
                    self.state.line_width_pending = False
                self._clear_plan_result()
                self._refresh_ui()
                return

        if self.state.mode in (MODE_PLACE_UAV, MODE_SET_UAV_HEADING, MODE_RESULT_READY, MODE_MISSION_READY):
            if self._pop_last_uav_input():
                self._clear_plan_result()
                self._refresh_ui()

    def _on_line_width_changed(self, value: float) -> None:
        self.state.line_width_m = float(value)
        if self.state.mission_kind == MISSION_LINE and self.state.mission_points_xy:
            self._clear_plan_result()
        self._refresh_ui()

    def _on_uav_count_changed(self, _index: int) -> None:
        self._selected_uav_count = int(self.cmb_uav_count.currentText())
        if self.state.uav_ids and len(self.state.uav_ids) != self._selected_uav_count:
            self.state.uav_ids = []
            self.state.uav_positions_xy = []
            self.state.uav_heading_deg = []
            self.state.mode = MODE_MISSION_READY if self.state.mission_points_xy else self.state.mode
            self._clear_plan_result()
        self._refresh_ui()

    def _confirm_uav_count(self) -> None:
        if not self.state.mission_points_xy:
            QMessageBox.warning(self, "임무 없음", "임무 형상을 먼저 입력해 주세요.")
            return

        self.state.uav_ids = list(_UAV_IDS[: self._selected_uav_count])
        self.state.uav_positions_xy = []
        self.state.uav_heading_deg = []
        self.state.mode = MODE_MISSION_READY
        self._clear_plan_result()
        self._append_result(f"UAV 대수 확정: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}")
        self._refresh_ui()

    def _start_uav_input(self) -> None:
        if not self.state.uav_ids:
            QMessageBox.warning(self, "UAV 대수", "UAV 대수를 먼저 확정해 주세요.")
            return
        self.state.uav_positions_xy = []
        self.state.uav_heading_deg = []
        self.state.mode = MODE_PLACE_UAV
        self._clear_plan_result()
        self._append_result("UAV 입력 시작: 한 번 클릭해 위치를 두고, 다시 클릭해 heading을 확정합니다.")
        self._refresh_ui()

    def _on_canvas_left_click(self, east_m: float, north_m: float) -> None:
        point_xy = (float(east_m), float(north_m))
        if self.state.mode in (MODE_DRAW_AREA, MODE_DRAW_LINE):
            if self.state.draft_points_xy and _distance(self.state.draft_points_xy[-1], point_xy) < 1.0:
                return
            self.state.draft_points_xy.append(point_xy)
            self._clear_plan_result()
            self._refresh_ui()
            return

        if self.state.mode == MODE_PLACE_UAV:
            if len(self.state.uav_positions_xy) >= len(self.state.uav_ids):
                return
            if self.state.uav_positions_xy and _distance(self.state.uav_positions_xy[-1], point_xy) < 1.0:
                return
            self.state.uav_positions_xy.append(point_xy)
            self.state.uav_heading_deg.append(None)
            aid = self.state.uav_ids[len(self.state.uav_positions_xy) - 1]
            self.state.mode = MODE_SET_UAV_HEADING
            self._clear_plan_result()
            self._append_result(f"UAV{aid} 위치 설정 완료. 다시 클릭해 heading을 정하세요.")
            self._refresh_ui()
            return

        if self.state.mode == MODE_SET_UAV_HEADING:
            idx = self._pending_uav_heading_index()
            if idx is None or idx >= len(self.state.uav_positions_xy):
                return
            anchor_xy = self.state.uav_positions_xy[idx]
            if _distance(anchor_xy, point_xy) < 5.0:
                return
            heading = _bearing_deg_from_xy(anchor_xy, point_xy)
            if heading is None:
                return
            while len(self.state.uav_heading_deg) <= idx:
                self.state.uav_heading_deg.append(None)
            self.state.uav_heading_deg[idx] = float(heading)
            aid = self.state.uav_ids[idx] if idx < len(self.state.uav_ids) else idx + 1
            self._clear_plan_result()
            self._append_result(f"UAV{aid} heading 설정: {float(heading):.1f} deg")
            if self._uav_inputs_complete():
                self.state.mode = MODE_MISSION_READY
                self._append_result("모든 UAV 위치와 heading 입력이 완료되었습니다. 계획을 실행할 수 있습니다.")
            else:
                self.state.mode = MODE_PLACE_UAV
            self._refresh_ui()
            return

    def _on_canvas_right_click(self, _east_m: float, _north_m: float) -> None:
        if self.state.mode == MODE_DRAW_AREA:
            self._finalize_area()
            return
        if self.state.mode == MODE_DRAW_LINE:
            self._enter_line_width_pending()
            return
        if self.state.mode == MODE_LINE_WIDTH_PENDING:
            self._finalize_line()
            return
        if self.state.mode == MODE_SET_UAV_HEADING:
            self._undo_last_point()

    def _finalize_area(self) -> None:
        if len(self.state.draft_points_xy) < 3:
            QMessageBox.warning(self, "영역 점 부족", "영역은 최소 3개 점이 필요합니다.")
            return
        try:
            points_xy, corrected = normalize_area_points(self.state.draft_points_xy)
        except Exception as exc:
            QMessageBox.critical(self, "영역 생성 실패", str(exc))
            return

        self.state.mission_points_xy = points_xy
        self.state.draft_points_xy = []
        self.state.mode = MODE_MISSION_READY
        self.state.line_width_pending = False
        self._clear_plan_result()

        msg = f"영역 입력 완료: {len(points_xy)}개 점"
        if corrected:
            msg += " (자가교차/중복점 자동 보정)"
        self._append_result(msg)
        self._refresh_ui()

    def _enter_line_width_pending(self) -> None:
        if len(self.state.draft_points_xy) < 2:
            QMessageBox.warning(self, "Line 점 부족", "Line은 최소 2개 점이 필요합니다.")
            return
        self.state.mode = MODE_LINE_WIDTH_PENDING
        self.state.line_width_pending = True
        self.state.line_width_m = float(self.spin_line_width.value())
        self._refresh_ui()

    def _finalize_line(self) -> None:
        points_xy = _dedupe_points(self.state.draft_points_xy)
        if len(points_xy) < 2:
            QMessageBox.warning(self, "Line 점 부족", "Line은 최소 2개 점이 필요합니다.")
            return
        if self.state.line_width_m <= 0.0:
            QMessageBox.warning(self, "폭 오류", "폭은 0보다 커야 합니다.")
            return

        self.state.mission_points_xy = points_xy
        self.state.draft_points_xy = []
        self.state.mode = MODE_MISSION_READY
        self.state.line_width_pending = False
        self._clear_plan_result()
        self._append_result(f"Line 입력 완료: {len(points_xy)}개 점 / 폭 {self.state.line_width_m:.1f}m")
        self._refresh_ui()

    def _run_area_division(self) -> None:
        if not self._ensure_ready_for_division(area_only=True):
            return

        try:
            self._cmpk_payload = self._build_cmpk_payload()
            self._mrpk_payload = self._build_mrpk_payload()
            split_result = run_split_pipeline(
                self._cmpk_payload,
                self._mrpk_payload,
                list(self.state.uav_ids),
                apply_assignment=False,
                apply_scheduling=False,
            )
            self._prefer_raw_split_preview_polygons(split_result)
            stage_lines = [f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}"]
            stage_lines.extend(self._direction_debug_lines(split_result))
            assign_report = self._assign_split_result_by_prediction_distance(split_result)
            stage_lines.append(
                "[1a] prediction-assign "
                f"speed={TURN_PREVIEW_SPEED_MPS:.0f}m/s "
                f"horizon={TURN_PREVIEW_HORIZON_S:.0f}s "
                f"radius={self._default_turn_radius_m():.0f}m "
                f"assigned={int(assign_report.get('assignedPieces', 0))}/"
                f"{int(assign_report.get('pieceCount', 0))} "
                f"{self._assignment_summary_text(split_result)}"
                + (
                    " turnGuard=on"
                    + (
                        f"/penalty={int(assign_report.get('turnPenaltyCount', 0))}"
                        if int(assign_report.get("turnPenaltyCount", 0) or 0) > 0
                        else ""
                    )
                    if bool(assign_report.get("turnGuardEnabled", False))
                    else ""
                )
            )
            self.state.split_result = split_result
            self.state.expected_paths = []
            self.state.assignment_path_rows = []
            self.state.mission_check_rows = []
            self.state.flight_plans_0303 = []
            self.state.flight_plans_0304 = []
            self.state.show_turn_overlays = True
            self.state.mid_line_segments = []
            self._mid_line_no_split_active = False
            self.state.mode = MODE_RESULT_READY

            lines = [
                f"Mission: {self.state.mission_kind}",
                f"UAV: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}",
                *stage_lines,
                "",
                f"Area division pieces: {len(split_result.pieces)}",
            ]
            piece_lines = assign_report.get("pieceLines")
            if isinstance(piece_lines, list) and piece_lines:
                lines.extend(str(text) for text in piece_lines)
            else:
                for piece in split_result.pieces:
                    lines.append(
                        f"  P{piece.piece_index}: type {piece.mission_type} / "
                        f"vertices {len((piece.data or {}).get('coordinateList', []))}"
                    )
            self._set_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._clear_plan_result()
            self.state.mode = MODE_MISSION_READY
            self._set_result(traceback.format_exc())
            QMessageBox.critical(self, "영역 분할 실패", str(exc))
            self._refresh_ui()

    def _generate_mid_lines(self) -> None:
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Mid Line Generation", "癒쇱? Area Division Run???ㅽ뻾??二쇱꽭??")
            return

        try:
            self._mid_line_no_split_active = False
            _reference_bearing_deg, overlays, lines = self._mid_line_overlay_bundle(self.state.split_result)
            total_count, no_split_count = self._mid_line_split_counts(overlays)
            for _dbg_row in overlays:
                if isinstance(_dbg_row, dict):
                    _dbg_ws = float(_dbg_row.get("widthStartM", 0.0) or _dbg_row.get("maxWidthM", 0.0) or _dbg_row.get("widthM", 0.0) or 0.0)
                    _dbg_dm = float(_dbg_row.get("dbMaxWidthM", 0.0) or 0.0)
                    lines.append(
                        f"[DBG] P{_dbg_row.get('pieceIndex','?')} widthStartM={_dbg_ws:.1f} dbMaxWidthM={_dbg_dm:.1f}"
                        f" → {'NO_SPLIT' if _dbg_ws > 0 and _dbg_dm > 0 and _dbg_ws <= _dbg_dm + 1e-6 else 'SPLIT'}"
                    )
            lines.append(f"[DBG] total={total_count} no_split={no_split_count}")
            self._mid_line_no_split_active = bool(total_count > 0 and no_split_count == total_count)
            if self._mid_line_no_split_active:
                normalized_overlays: List[Dict[str, Any]] = []
                for row in overlays:
                    if not isinstance(row, dict):
                        continue
                    row_copy = dict(row)
                    row_copy["midLineRequired"] = False
                    row_copy.pop("splitParts", None)
                    row_copy.pop("stage2Centers", None)
                    normalized_overlays.append(row_copy)
                overlays = normalized_overlays
                self.state.expected_paths = []
                self.state.assignment_path_rows = []
                self.state.next_mission_rows = []
                self.state.mission_check_rows = []
                self.state.visibility_segments = []
                self.state.tangent_checks = []
            if total_count > 0 and no_split_count == total_count:
                lines.append("[MID] DB covers all current widths. Split workflow disabled; use Make Path - 0.")
            elif 0 < no_split_count < total_count:
                lines.append(
                    f"[MID] mixed DB coverage: {no_split_count}/{total_count} piece(s) are coverable, "
                    "so the current split workflow stays enabled."
                )
            elif total_count > 0:
                lines.append("[MID] no DB-covered width found. Continue the current split workflow.")
            self.state.mid_line_segments = overlays
            self.state.tangent_checks = []
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Mid Line Generation", str(exc))
            self._refresh_ui()

    def _generate_mid_lines_2(self) -> None:
        if not self._ensure_split_branch_allowed("Mid Line Generation 2"):
            return
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Mid Line Generation 2", "먼저 Area Division Run을 실행해 주세요.")
            return
        if not self.state.mid_line_segments:
            QMessageBox.warning(self, "Mid Line Generation 2", "먼저 Mid Line Generation을 실행해 주세요.")
            return
        if not self._has_make_new_area_result():
            QMessageBox.warning(self, "Mid Line Generation 2", "먼저 Make New Area를 실행해 주세요.")
            return

        try:
            overlay_rows: List[Dict[str, Any]] = []
            lines = ["[MID2] split-part far-face generation"]
            for row in self.state.mid_line_segments:
                if not isinstance(row, dict):
                    continue
                overlay = dict(row)
                aid = int(overlay.get("aircraftID", 0) or 0)
                piece_index = int(overlay.get("pieceIndex", 0) or 0)
                uav_state = self._uav_state_for_aircraft(aid)
                origin_xy = uav_state[0] if uav_state is not None else None
                bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)
                split_parts = self._resolved_split_parts_for_overlay(overlay)
                if split_parts:
                    overlay["splitParts"] = split_parts
                centers: List[Dict[str, Any]] = []
                if isinstance(split_parts, list) and split_parts:
                    for idx, part in enumerate(split_parts, start=1):
                        if not isinstance(part, dict):
                            continue
                        name = str(part.get("name", "") or chr(ord("A") + idx - 1))
                        polygon_xy = part.get("polygonXY")
                        center_xy = None
                        near_xy = None
                        line_xy = None
                        shape_center_xy = None
                        part_width_m = 0.0
                        mid_line_length_m = 0.0
                        if isinstance(polygon_xy, list) and len(polygon_xy) >= 3:
                            try:
                                part_coords_xy = [
                                    (float(point_xy[0]), float(point_xy[1]))
                                    for point_xy in polygon_xy
                                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                ]
                                if len(part_coords_xy) >= 3:
                                    part_overlay = self._mid_line_overlay_geometry(part_coords_xy, bearing_deg)
                                    if isinstance(part_overlay, dict):
                                        part_width_m = float(
                                            part_overlay.get("maxWidthM", 0.0)
                                            or part_overlay.get("widthM", 0.0)
                                            or 0.0
                                        )
                                    if origin_xy is not None:
                                        centerline_points = self._bearing_centerline_target_points(
                                            part_coords_xy,
                                            bearing_deg,
                                            origin_xy,
                                        )
                                        if isinstance(centerline_points, dict):
                                            raw_shape_center_xy = centerline_points.get("shapeCenterXY")
                                            if isinstance(raw_shape_center_xy, (tuple, list)) and len(raw_shape_center_xy) >= 2:
                                                shape_center_xy = (
                                                    float(raw_shape_center_xy[0]),
                                                    float(raw_shape_center_xy[1]),
                                                )
                                            target_xy = centerline_points.get("targetXY")
                                            if isinstance(target_xy, (tuple, list)) and len(target_xy) >= 2:
                                                center_xy = (
                                                    float(target_xy[0]),
                                                    float(target_xy[1]),
                                                )
                                            raw_near_xy = centerline_points.get("nearXY")
                                            if isinstance(raw_near_xy, (tuple, list)) and len(raw_near_xy) >= 2:
                                                near_xy = (
                                                    float(raw_near_xy[0]),
                                                    float(raw_near_xy[1]),
                                                )
                                            raw_line_xy = centerline_points.get("centerLineXY")
                                            if isinstance(raw_line_xy, list) and len(raw_line_xy) >= 2:
                                                line_xy = [
                                                    (float(line_point[0]), float(line_point[1]))
                                                    for line_point in raw_line_xy
                                                    if isinstance(line_point, (tuple, list)) and len(line_point) >= 2
                                                ]
                                                if len(line_xy) >= 2:
                                                    mid_line_length_m = _distance(line_xy[0], line_xy[-1])
                                    if center_xy is None:
                                        if part_overlay is not None and origin_xy is not None:
                                            face_points = self._make_path_face_points(part_overlay, origin_xy)
                                            if isinstance(face_points, dict):
                                                target_face_xy = face_points.get("targetFaceXY")
                                                if isinstance(target_face_xy, (tuple, list)) and len(target_face_xy) >= 2:
                                                    center_xy = (
                                                        float(target_face_xy[0]),
                                                        float(target_face_xy[1]),
                                                    )
                                                raw_near_face_xy = face_points.get("nearFaceXY")
                                                if isinstance(raw_near_face_xy, (tuple, list)) and len(raw_near_face_xy) >= 2:
                                                    near_xy = (
                                                        float(raw_near_face_xy[0]),
                                                        float(raw_near_face_xy[1]),
                                                    )
                                                target_face_line_xy = face_points.get("targetFaceLineXY")
                                                if isinstance(target_face_line_xy, list) and len(target_face_line_xy) >= 2:
                                                    raw_face_pts = [
                                                        (float(line_point[0]), float(line_point[1]))
                                                        for line_point in target_face_line_xy
                                                        if isinstance(line_point, (tuple, list)) and len(line_point) >= 2
                                                    ]
                                                    # Clip face line to actual part polygon to avoid extending outside
                                                    if len(raw_face_pts) >= 2 and len(part_coords_xy) >= 3:
                                                        try:
                                                            clip_poly = Polygon(part_coords_xy).buffer(0)
                                                            clipped = clip_poly.intersection(LineString(raw_face_pts))
                                                            if not clipped.is_empty:
                                                                clipped_seg = self._longest_linestring_xy(clipped)
                                                                if clipped_seg is not None and len(clipped_seg.coords) >= 2:
                                                                    raw_face_pts = [
                                                                        (float(c[0]), float(c[1]))
                                                                        for c in clipped_seg.coords
                                                                    ]
                                                                    # Update near_xy to midpoint of clipped line
                                                                    near_xy = (
                                                                        (raw_face_pts[0][0] + raw_face_pts[-1][0]) * 0.5,
                                                                        (raw_face_pts[0][1] + raw_face_pts[-1][1]) * 0.5,
                                                                    )
                                                        except Exception:
                                                            pass
                                                    line_xy = raw_face_pts
                                                    if len(line_xy) >= 2:
                                                        mid_line_length_m = _distance(line_xy[0], line_xy[-1])
                            except Exception:
                                center_xy = None
                                line_xy = None
                        if not (isinstance(center_xy, tuple) and len(center_xy) >= 2):
                            raw_center_xy = part.get("centroidXY")
                            if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                                shape_center_xy = (
                                    float(raw_center_xy[0]),
                                    float(raw_center_xy[1]),
                                )
                                center_xy = (float(raw_center_xy[0]), float(raw_center_xy[1]))
                        if not (isinstance(center_xy, tuple) and len(center_xy) >= 2):
                            if isinstance(polygon_xy, list) and len(polygon_xy) >= 3:
                                try:
                                    part_poly = Polygon(
                                        [
                                            (float(point_xy[0]), float(point_xy[1]))
                                            for point_xy in polygon_xy
                                            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                        ]
                                    ).buffer(0)
                                    if isinstance(part_poly, MultiPolygon):
                                        geoms = [
                                            geom
                                            for geom in part_poly.geoms
                                            if isinstance(geom, Polygon) and not geom.is_empty
                                        ]
                                        part_poly = max(geoms, key=lambda geom: float(geom.area)) if geoms else part_poly
                                    if not part_poly.is_empty:
                                        point = part_poly.representative_point()
                                        shape_center_xy = (float(point.x), float(point.y))
                                        center_xy = (float(point.x), float(point.y))
                                except Exception:
                                    center_xy = None
                        if shape_center_xy is None:
                            raw_center_xy = part.get("centroidXY")
                            if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                                shape_center_xy = (
                                    float(raw_center_xy[0]),
                                    float(raw_center_xy[1]),
                                )
                        if shape_center_xy is None and isinstance(center_xy, tuple) and len(center_xy) >= 2:
                            shape_center_xy = center_xy
                        if not (isinstance(center_xy, tuple) and len(center_xy) >= 2):
                            continue
                        # Compute near face line: the two polygon vertices closest to near_xy in the s-direction
                        near_face_line_xy: Optional[List[Tuple[float, float]]] = None
                        if near_xy is not None and isinstance(polygon_xy, list) and len(polygon_xy) >= 3:
                            try:
                                nf_coords = [
                                    (float(p[0]), float(p[1]))
                                    for p in polygon_xy
                                    if isinstance(p, (tuple, list)) and len(p) >= 2
                                ]
                                if len(nf_coords) >= 3:
                                    nf_ux, nf_uy, nf_vx, nf_vy = self._mid_line_axis_vectors(bearing_deg)
                                    near_s = float(near_xy[0]) * nf_ux + float(near_xy[1]) * nf_uy
                                    # Sort vertices by their s-value and pick the two nearest to near_s
                                    verts_s = sorted(nf_coords, key=lambda p: abs(float(p[0]) * nf_ux + float(p[1]) * nf_uy - near_s))
                                    near_face_line_xy = [verts_s[0], verts_s[1]]
                            except Exception:
                                near_face_line_xy = None
                        centers.append(
                            {
                                "name": name,
                                "centerXY": (float(center_xy[0]), float(center_xy[1])),
                                "label": f"F{name}",
                                "nearXY": near_xy,
                                "nearLabel": f"N{name}",
                                "lineXY": line_xy,
                                "nearFaceLineXY": near_face_line_xy,
                                "shapeCenterXY": shape_center_xy,
                                "polygonXY": [
                                    (float(point_xy[0]), float(point_xy[1]))
                                    for point_xy in polygon_xy
                                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                ] if isinstance(polygon_xy, list) else [],
                                "widthM": float(part_width_m),
                                "midLineLengthM": float(mid_line_length_m),
                                "bearingDeg": float(bearing_deg),
                            }
                        )
                    existing_names = {
                        str(center.get("name", "") or "")
                        for center in centers
                        if isinstance(center, dict)
                    }
                    for fallback_part in split_parts:
                        if not isinstance(fallback_part, dict):
                            continue
                        fallback_name = str(fallback_part.get("name", "") or "")
                        if fallback_name in existing_names:
                            continue
                        polygon_xy = fallback_part.get("polygonXY")
                        if not isinstance(polygon_xy, list) or len(polygon_xy) < 3:
                            continue
                        fallback_center_xy = None
                        raw_center_xy = fallback_part.get("centroidXY")
                        if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                            fallback_center_xy = (float(raw_center_xy[0]), float(raw_center_xy[1]))
                        if fallback_center_xy is None:
                            try:
                                fallback_poly = self._largest_polygon_xy(
                                    Polygon(
                                        [
                                            (float(point_xy[0]), float(point_xy[1]))
                                            for point_xy in polygon_xy
                                            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                        ]
                                    ).buffer(0)
                                )
                                if fallback_poly is not None and not fallback_poly.is_empty:
                                    point = fallback_poly.representative_point()
                                    fallback_center_xy = (float(point.x), float(point.y))
                            except Exception:
                                fallback_center_xy = None
                        if fallback_center_xy is None:
                            continue
                        centers.append(
                            {
                                "name": fallback_name,
                                "centerXY": fallback_center_xy,
                                "label": f"F{fallback_name}",
                                "lineXY": None,
                                "shapeCenterXY": fallback_center_xy,
                                "polygonXY": [
                                    (float(point_xy[0]), float(point_xy[1]))
                                    for point_xy in polygon_xy
                                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                ],
                                "widthM": 0.0,
                                "midLineLengthM": 0.0,
                                "bearingDeg": float(bearing_deg),
                            }
                        )
                else:
                    center_xy = None
                    near_xy = None
                    line_xy = None
                    shape_center_xy = None
                    piece_width_m = 0.0
                    mid_line_length_m = 0.0
                    piece_coords_xy: List[Tuple[float, float]] = []
                    if origin_xy is not None:
                        piece = next(
                            (
                                piece_row
                                for piece_row in (self.state.split_result.pieces if self.state.split_result is not None else [])
                                if int(piece_row.piece_index or 0) == piece_index
                            ),
                            None,
                        )
                        piece_poly = self._piece_polygon_xy(piece) if piece is not None else None
                        if piece_poly is not None and not piece_poly.is_empty:
                            point = piece_poly.representative_point()
                            shape_center_xy = (float(point.x), float(point.y))
                            piece_coords_xy = [
                                (float(x), float(y))
                                for (x, y) in list(piece_poly.exterior.coords)[:-1]
                            ]
                            piece_overlay = self._mid_line_overlay_geometry(piece_coords_xy, bearing_deg)
                            if isinstance(piece_overlay, dict):
                                piece_width_m = float(
                                    piece_overlay.get("maxWidthM", 0.0)
                                    or piece_overlay.get("widthM", 0.0)
                                    or 0.0
                                )
                            centerline_points = self._bearing_centerline_target_points(
                                piece_coords_xy,
                                bearing_deg,
                                origin_xy,
                            )
                            if isinstance(centerline_points, dict):
                                raw_shape_center_xy = centerline_points.get("shapeCenterXY")
                                if isinstance(raw_shape_center_xy, (tuple, list)) and len(raw_shape_center_xy) >= 2:
                                    shape_center_xy = (
                                        float(raw_shape_center_xy[0]),
                                        float(raw_shape_center_xy[1]),
                                    )
                                target_xy = centerline_points.get("targetXY")
                                if isinstance(target_xy, (tuple, list)) and len(target_xy) >= 2:
                                    center_xy = (
                                        float(target_xy[0]),
                                        float(target_xy[1]),
                                    )
                                raw_near_xy = centerline_points.get("nearXY")
                                if isinstance(raw_near_xy, (tuple, list)) and len(raw_near_xy) >= 2:
                                    near_xy = (
                                        float(raw_near_xy[0]),
                                        float(raw_near_xy[1]),
                                    )
                                raw_line_xy = centerline_points.get("centerLineXY")
                                if isinstance(raw_line_xy, list) and len(raw_line_xy) >= 2:
                                    line_xy = [
                                        (float(line_point[0]), float(line_point[1]))
                                        for line_point in raw_line_xy
                                        if isinstance(line_point, (tuple, list)) and len(line_point) >= 2
                                    ]
                                    if len(line_xy) >= 2:
                                        mid_line_length_m = _distance(line_xy[0], line_xy[-1])
                        if center_xy is None:
                            face_points = self._make_path_face_points(overlay, origin_xy)
                            if isinstance(face_points, dict):
                                target_face_xy = face_points.get("targetFaceXY")
                                if isinstance(target_face_xy, (tuple, list)) and len(target_face_xy) >= 2:
                                    center_xy = (
                                        float(target_face_xy[0]),
                                        float(target_face_xy[1]),
                                    )
                                raw_near_face_xy = face_points.get("nearFaceXY")
                                if isinstance(raw_near_face_xy, (tuple, list)) and len(raw_near_face_xy) >= 2:
                                    near_xy = (
                                        float(raw_near_face_xy[0]),
                                        float(raw_near_face_xy[1]),
                                    )
                                target_face_line_xy = face_points.get("targetFaceLineXY")
                                if isinstance(target_face_line_xy, list) and len(target_face_line_xy) >= 2:
                                    line_xy = [
                                        (float(point_xy[0]), float(point_xy[1]))
                                    for point_xy in target_face_line_xy
                                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                ]
                                    if len(line_xy) >= 2:
                                        mid_line_length_m = _distance(line_xy[0], line_xy[-1])
                    if center_xy is None:
                        raw_center_xy = overlay.get("centerXY")
                        if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                            center_xy = (float(raw_center_xy[0]), float(raw_center_xy[1]))
                    if shape_center_xy is None:
                        raw_center_xy = overlay.get("centerXY")
                        if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                            shape_center_xy = (float(raw_center_xy[0]), float(raw_center_xy[1]))
                    if isinstance(center_xy, tuple) and len(center_xy) >= 2:
                        centers.append(
                            {
                                "name": "P",
                                "centerXY": (float(center_xy[0]), float(center_xy[1])),
                                "label": "FP",
                                "nearXY": near_xy,
                                "nearLabel": "NP",
                                "lineXY": line_xy,
                                "shapeCenterXY": shape_center_xy,
                                "polygonXY": piece_coords_xy,
                                "widthM": float(piece_width_m),
                                "midLineLengthM": float(mid_line_length_m),
                                "bearingDeg": float(bearing_deg),
                            }
                        )

                centers.sort(key=lambda center: str(center.get("name", "") or str(center.get("label", "") or "")))
                overlay["stage2Centers"] = centers
                overlay["midLineStage2Applied"] = True
                if centers:
                    label_text = ", ".join(
                        f"{str(center.get('label', '?'))}=({float(center['centerXY'][0]):.1f}, {float(center['centerXY'][1]):.1f})"
                        for center in centers
                        if isinstance(center, dict) and isinstance(center.get("centerXY"), (tuple, list)) and len(center.get("centerXY")) >= 2
                    )
                    lines.append(
                        f"  P{piece_index} / UAV{aid}: {label_text}"
                    )
                else:
                    lines.append(f"  P{piece_index} / UAV{aid}: 작은 도형 반대면 중심 없음")
                overlay_rows.append(overlay)

            self.state.mid_line_segments = overlay_rows
            self.state.expected_paths = []
            self.state.assignment_path_rows = []
            self.state.mission_check_rows = []
            self.state.tangent_checks = []
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Mid Line Generation 2", str(exc))
            self._refresh_ui()

    def _build_tangent_check_rows(
        self,
        uav_overrides: Optional[Dict[int, Tuple[Tuple[float, float], float, float]]] = None,
        exclude_target_groups: Optional[set[str]] = None,
    ) -> Tuple[List[Dict[str, Any]], List[str]]:
        tangent_rows: List[Dict[str, Any]] = []
        lines = [
            "[TAN] check tangent points",
            "rule: Mid Line Generation 2 target x all UAV turn circles (L/R) x two tangent points",
        ]

        for row in self.state.mid_line_segments:
            if not isinstance(row, dict):
                continue
            piece_index = int(row.get("pieceIndex", 0) or 0)
            stage2_centers = row.get("stage2Centers")
            if not isinstance(stage2_centers, list) or not stage2_centers:
                continue

            for center_row in stage2_centers:
                if not isinstance(center_row, dict):
                    continue
                # FA(centerXY) + NA(nearXY) 양쪽을 target 후보로
                target_candidates: List[Tuple[Tuple[float, float], str]] = []
                far_xy_raw = center_row.get("centerXY")
                if isinstance(far_xy_raw, (tuple, list)) and len(far_xy_raw) >= 2:
                    far_label = str(center_row.get("label", "") or f"P{piece_index}")
                    target_candidates.append(((float(far_xy_raw[0]), float(far_xy_raw[1])), far_label))
                near_xy_raw = center_row.get("nearXY")
                if isinstance(near_xy_raw, (tuple, list)) and len(near_xy_raw) >= 2:
                    near_label = str(center_row.get("nearLabel", "") or f"N{piece_index}")
                    target_candidates.append(((float(near_xy_raw[0]), float(near_xy_raw[1])), near_label))

                shape_center_xy = center_row.get("shapeCenterXY")
                shape_center_val = None
                if isinstance(shape_center_xy, (tuple, list)) and len(shape_center_xy) >= 2:
                    shape_center_val = (float(shape_center_xy[0]), float(shape_center_xy[1]))
                center_line_xy_raw = center_row.get("lineXY")
                center_line_xy: List[Tuple[float, float]] = []
                if isinstance(center_line_xy_raw, list):
                    for point_xy in center_line_xy_raw:
                        if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                            center_line_xy.append((float(point_xy[0]), float(point_xy[1])))
                polygon_xy_raw = center_row.get("polygonXY")
                part_polygon_xy: List[Tuple[float, float]] = []
                if isinstance(polygon_xy_raw, list):
                    for point_xy in polygon_xy_raw:
                        if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                            part_polygon_xy.append((float(point_xy[0]), float(point_xy[1])))
                try:
                    part_width_m = float(center_row.get("widthM", 0.0) or 0.0)
                except Exception:
                    part_width_m = 0.0
                try:
                    mid_line_length_m = float(center_row.get("midLineLengthM", 0.0) or 0.0)
                except Exception:
                    mid_line_length_m = 0.0
                try:
                    center_bearing_deg = float(center_row.get("bearingDeg", row.get("bearingDeg", 0.0) or 0.0) or 0.0)
                except Exception:
                    center_bearing_deg = 0.0

                for target_point_xy, target_label in target_candidates:
                    if (
                        exclude_target_groups
                        and self._target_group_key(piece_index, target_label) in exclude_target_groups
                    ):
                        continue
                    target_tokens: List[str] = []

                    for aid in self.state.uav_ids:
                        if uav_overrides and int(aid) in uav_overrides:
                            origin_xy, heading_deg, override_radius = uav_overrides[int(aid)]
                        else:
                            uav_state = self._uav_state_for_aircraft(int(aid))
                            if uav_state is None:
                                target_tokens.append(f"UAV{int(aid)}:missing")
                                continue
                            origin_xy, heading_deg = uav_state
                            override_radius = self._default_turn_radius_m()
                        left_center_xy, right_center_xy = self._turn_circle_centers_xy(
                            origin_xy, heading_deg, radius_m=override_radius,
                        )
                        for branch, circle_center_xy in (("L", left_center_xy), ("R", right_center_xy)):
                            tangent_points = self._circle_tangent_points_xy(
                                circle_center_xy,
                                target_point_xy,
                                radius_m=override_radius,
                            )
                            valid_tangent_rows: List[Tuple[Tuple[float, float], float]] = []
                            for tangent_xy in tangent_points:
                                tangent_point_xy_val = (float(tangent_xy[0]), float(tangent_xy[1]))
                                if not self._branch_forward_tangent(
                                    circle_center_xy,
                                    tangent_point_xy_val,
                                    target_point_xy,
                                    branch=branch,
                                ):
                                    continue
                                horizon_s = self._arc_horizon_to_point_on_branch(
                                    origin_xy,
                                    circle_center_xy,
                                    tangent_point_xy_val,
                                    branch=branch,
                                    radius_m=override_radius,
                                )
                                valid_tangent_rows.append((tangent_point_xy_val, float(horizon_s)))
                            if not valid_tangent_rows:
                                target_tokens.append(f"UAV{int(aid)}{branch}:0")
                                continue
                            target_tokens.append(f"UAV{int(aid)}{branch}:{len(valid_tangent_rows)}")
                            for tangent_idx, (tangent_xy, horizon_s) in enumerate(valid_tangent_rows, start=1):
                                tangent_rows.append(
                                    {
                                        "pieceIndex": int(piece_index),
                                        "targetLabel": target_label,
                                        "targetXY": target_point_xy,
                                        "shapeCenterXY": shape_center_val,
                                        "centerLineXY": center_line_xy,
                                        "partPolygonXY": part_polygon_xy,
                                        "partWidthM": float(part_width_m),
                                        "midLineLengthM": float(mid_line_length_m),
                                        "bearingDeg": float(center_bearing_deg),
                                        "aircraftID": int(aid),
                                        "branch": branch,
                                        "circleCenterXY": (
                                            float(circle_center_xy[0]),
                                            float(circle_center_xy[1]),
                                        ),
                                        "tangentIndex": int(tangent_idx),
                                        "tangentXY": (
                                            float(tangent_xy[0]),
                                            float(tangent_xy[1]),
                                        ),
                                        "horizonSec": float(horizon_s),
                                        "label": f"{int(aid)}{branch}{int(tangent_idx)}",
                                    }
                                )

                    if target_tokens:
                        lines.append(f"  P{piece_index} / {target_label}: " + ", ".join(target_tokens))

        if not tangent_rows:
            lines.append("  tangent point 없음")
        return tangent_rows, lines

    def _assignment_path_1(self) -> None:
        if not self._ensure_split_branch_allowed("Assignment - Path 1"):
            return
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Assignment - Path 1", "먼저 Area Division Run을 실행해 주세요.")
            return
        if not self.state.mid_line_segments:
            QMessageBox.warning(self, "Assignment - Path 1", "먼저 Mid Line Generation을 실행해 주세요.")
            return

        try:
            tangent_rows, tangent_lines = self._build_tangent_check_rows()
            self.state.tangent_checks = tangent_rows
            candidate_by_uav_target: Dict[Tuple[int, str], Dict[str, Any]] = {}
            active_uavs: List[int] = []
            target_keys: List[str] = []
            lines = [
                "[AP1] assign first target by tangent-entry angle",
                f"rule: weighted score = angle*{float(ASSIGNMENT_PATH1_ANGLE_WEIGHT):.0f} + turnTime*{float(ASSIGNMENT_PATH1_TURN_TIME_WEIGHT):.0f}",
            ]
            if tangent_lines:
                lines.extend(tangent_lines)

            for aid in self.state.uav_ids:
                if self._uav_state_for_aircraft(int(aid)) is not None:
                    active_uavs.append(int(aid))

            seen_target_keys: set[str] = set()
            for row in tangent_rows:
                if not isinstance(row, dict):
                    continue
                aid = int(row.get("aircraftID", 0) or 0)
                target_xy = row.get("targetXY")
                tangent_xy = row.get("tangentXY")
                shape_center_xy = row.get("shapeCenterXY")
                if not (
                    isinstance(target_xy, (tuple, list))
                    and len(target_xy) >= 2
                    and isinstance(tangent_xy, (tuple, list))
                    and len(tangent_xy) >= 2
                    and isinstance(shape_center_xy, (tuple, list))
                    and len(shape_center_xy) >= 2
                ):
                    continue

                target_point_xy = (float(target_xy[0]), float(target_xy[1]))
                tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
                shape_center_point_xy = (float(shape_center_xy[0]), float(shape_center_xy[1]))
                angle_deg = _undirected_angle_deg(
                    target_point_xy,
                    shape_center_point_xy,
                    tangent_point_xy,
                )
                if angle_deg is None:
                    continue

                piece_index = int(row.get("pieceIndex", 0) or 0)
                target_label = str(row.get("targetLabel", "") or f"P{piece_index}")
                target_key = self._target_group_key(piece_index, target_label)
                candidate = dict(row)
                candidate["targetKey"] = target_key
                candidate["targetGroupKey"] = target_key
                candidate["entryAngleDeg"] = float(angle_deg)
                candidate["segmentLenM"] = _distance(tangent_point_xy, target_point_xy)
                candidate["selectionScore"] = (
                    (float(candidate["entryAngleDeg"]) * float(ASSIGNMENT_PATH1_ANGLE_WEIGHT))
                    + (float(candidate.get("horizonSec", 0.0) or 0.0) * float(ASSIGNMENT_PATH1_TURN_TIME_WEIGHT))
                )

                score_key = (
                    float(candidate["selectionScore"]),
                    float(candidate.get("horizonSec", 0.0) or 0.0),
                    float(candidate["segmentLenM"]),
                )
                existing = candidate_by_uav_target.get((aid, target_key))
                if existing is None:
                    candidate_by_uav_target[(aid, target_key)] = candidate
                else:
                    existing_key = (
                        float(existing.get("selectionScore", 0.0) or 0.0),
                        float(existing.get("horizonSec", 0.0) or 0.0),
                        float(existing.get("segmentLenM", 0.0) or 0.0),
                    )
                    if score_key < existing_key:
                        candidate_by_uav_target[(aid, target_key)] = candidate
                if target_key not in seen_target_keys:
                    seen_target_keys.add(target_key)
                    target_keys.append(target_key)

            if not candidate_by_uav_target or not active_uavs or not target_keys:
                lines.append("  usable tangent candidate 없음")
                self.state.assignment_path_rows = []
                self.state.mission_check_rows = []
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            assign_count = min(len(active_uavs), len(target_keys))
            best_records: List[Dict[str, Any]] = []
            best_score: Optional[Tuple[float, float, float]] = None

            for uav_subset in itertools.combinations(active_uavs, assign_count):
                for target_perm in itertools.permutations(target_keys, assign_count):
                    candidate_rows: List[Dict[str, Any]] = []
                    weighted_sum = 0.0
                    horizon_sum = 0.0
                    segment_sum = 0.0
                    valid = True
                    for aid, target_key in zip(uav_subset, target_perm):
                        candidate = candidate_by_uav_target.get((int(aid), str(target_key)))
                        if candidate is None:
                            valid = False
                            break
                        candidate_rows.append(candidate)
                        weighted_sum += float(candidate.get("selectionScore", 0.0) or 0.0)
                        horizon_sum += float(candidate.get("horizonSec", 0.0) or 0.0)
                        segment_sum += float(candidate.get("segmentLenM", 0.0) or 0.0)
                    if not valid:
                        continue
                    score = (float(weighted_sum), float(horizon_sum), float(segment_sum))
                    if best_score is None or score < best_score:
                        best_score = score
                        best_records = candidate_rows

            if not best_records:
                lines.append("  중복 없는 배정 조합을 만들지 못함")
                self.state.assignment_path_rows = []
                self.state.mission_check_rows = []
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            # Build lookup: (pieceIndex, targetLabel) -> lineXY endpoints for straight-line display
            near_point_lookup: Dict[Tuple[int, str], List[Tuple[float, float]]] = {}
            for seg_row in self.state.mid_line_segments:
                if not isinstance(seg_row, dict):
                    continue
                seg_piece_idx = int(seg_row.get("pieceIndex", 0) or 0)
                seg_centers = seg_row.get("stage2Centers")
                if not isinstance(seg_centers, list):
                    continue
                for seg_center in seg_centers:
                    if not isinstance(seg_center, dict):
                        continue
                    seg_label = str(seg_center.get("label", "") or "").strip().upper()
                    seg_line_raw = seg_center.get("nearFaceLineXY")
                    if isinstance(seg_line_raw, list) and len(seg_line_raw) >= 2:
                        endpoints = [
                            (float(p[0]), float(p[1]))
                            for p in seg_line_raw
                            if isinstance(p, (tuple, list)) and len(p) >= 2
                        ]
                        if len(endpoints) >= 2:
                            near_point_lookup[(seg_piece_idx, seg_label)] = [endpoints[0], endpoints[-1]]

            path_rows: List[Dict[str, Any]] = []
            assigned_uavs = {int(row.get("aircraftID", 0) or 0) for row in best_records}
            for candidate in sorted(best_records, key=lambda row: int(row.get("aircraftID", 0) or 0)):
                path_row = self._build_assignment_path_1_row(candidate, tangent_label="T1")
                if path_row is not None:
                    part_width_m = float(path_row.get("partWidthM", 0.0) or 0.0)
                    if float(path_row.get("dbSepM", 0.0) or 0.0) <= 0.0 and part_width_m > 0.0:
                        db_row = self._next_collab_entry_tprime_db_row(part_width_m)
                        if isinstance(db_row, dict):
                            path_row["dbSepM"] = float(db_row.get("sep", 0.0) or 0.0)
                            path_row["dbWidthM"] = float(db_row.get("width", 0.0) or 0.0)
                    # Attach T -> near entry point info for straight-line display
                    cand_aid = int(candidate.get("aircraftID", 0) or 0)
                    cand_piece = int(candidate.get("pieceIndex", 0) or 0)
                    cand_label = str(candidate.get("targetLabel", "") or "").strip().upper()
                    near_pt = near_point_lookup.get((cand_piece, cand_label))
                    if near_pt is not None:
                        path_row["entryLineEndpointsXY"] = near_pt

                    # --- Sep_cand / Entry T' / FOV determination ---
                    tangent_xy_raw = path_row.get("tangentXY")
                    target_xy_raw = path_row.get("targetFaceXY") or path_row.get("targetXY")
                    if (
                        near_pt is not None
                        and isinstance(tangent_xy_raw, (tuple, list)) and len(tangent_xy_raw) >= 2
                        and isinstance(target_xy_raw, (tuple, list)) and len(target_xy_raw) >= 2
                    ):
                        t_xy = (float(tangent_xy_raw[0]), float(tangent_xy_raw[1]))
                        fa_xy = (float(target_xy_raw[0]), float(target_xy_raw[1]))
                        ep0 = near_pt[0]
                        ep1 = near_pt[1]
                        d0 = _distance(t_xy, ep0)
                        d1 = _distance(t_xy, ep1)
                        sep_cand = (d0 + d1) / 2.0

                        db_sep_row = self._next_collab_entry_tprime_db_row(part_width_m)
                        target_sep = self._next_collab_entry_tprime_target_sep_m(part_width_m)

                        entry_t_prime_xy = None
                        if sep_cand > target_sep and target_sep > 0.0:
                            # Need Entry T': advance along ingress (T→FA) until Sep_cand ≈ target_sep
                            ingress_dx = fa_xy[0] - t_xy[0]
                            ingress_dy = fa_xy[1] - t_xy[1]
                            ingress_len = math.hypot(ingress_dx, ingress_dy)
                            if ingress_len > 1e-6:
                                ux_i = ingress_dx / ingress_len
                                uy_i = ingress_dy / ingress_len
                                lo, hi = 0.0, ingress_len
                                for _ in range(40):
                                    mid = (lo + hi) * 0.5
                                    test_xy = (t_xy[0] + ux_i * mid, t_xy[1] + uy_i * mid)
                                    test_sep = (_distance(test_xy, ep0) + _distance(test_xy, ep1)) * 0.5
                                    if test_sep > target_sep:
                                        lo = mid
                                    else:
                                        hi = mid
                                entry_t_prime_xy = (
                                    t_xy[0] + ux_i * lo,
                                    t_xy[1] + uy_i * lo,
                                )
                                # Recalculate Sep_cand from T'
                                d0 = _distance(entry_t_prime_xy, ep0)
                                d1 = _distance(entry_t_prime_xy, ep1)
                                sep_cand = (d0 + d1) / 2.0

                                # Degenerate case: when the aircraft is far from
                                # the polygon the geometric minimum sep along the
                                # ingress line can still be larger than target_sep,
                                # and the binary search converges to fa_xy (the far
                                # face) so the entry waypoint ends up past the area.
                                # Snap to the closest approach to the near-face
                                # midpoint instead so WP_E_0 stays on the near side.
                                if sep_cand > target_sep + 1e-6:
                                    ep_mid_x = (ep0[0] + ep1[0]) * 0.5
                                    ep_mid_y = (ep0[1] + ep1[1]) * 0.5
                                    proj_len = (
                                        (ep_mid_x - t_xy[0]) * ux_i
                                        + (ep_mid_y - t_xy[1]) * uy_i
                                    )
                                    proj_len = max(0.0, min(ingress_len, proj_len))
                                    entry_t_prime_xy = (
                                        t_xy[0] + ux_i * proj_len,
                                        t_xy[1] + uy_i * proj_len,
                                    )
                                    sep_cand = (
                                        _distance(entry_t_prime_xy, ep0)
                                        + _distance(entry_t_prime_xy, ep1)
                                    ) * 0.5

                        path_row["sepCandM"] = float(sep_cand)
                        path_row["entryTPrimeXY"] = entry_t_prime_xy

                        area_db_values = self._next_collab_area_resolved_db_values(part_width_m, sep_cand)
                        resolved_camera_base_fov = area_db_values.get("resolvedBaseFovDeg")
                        resolved_db_fov = area_db_values.get("resolvedDbFovDeg")
                        resolved_fov = area_db_values.get("resolvedFovDeg")
                        resolved_foot = area_db_values.get("resolvedFootM")
                        resolved_vel = area_db_values.get("resolvedVelMps")
                        path_row["dbSepM"] = float(area_db_values.get("dbSepM", 0.0) or 0.0)
                        path_row["dbWidthM"] = float(area_db_values.get("dbWidthM", 0.0) or 0.0)
                        path_row["resolvedDbFovDeg"] = resolved_db_fov
                        path_row["resolvedBaseFovDeg"] = resolved_camera_base_fov
                        path_row["resolvedFovDeg"] = resolved_fov
                        path_row["resolvedFootM"] = resolved_foot
                        path_row["resolvedVelMps"] = resolved_vel
                        path_row["gsdSepM"] = float(area_db_values.get("gsdSepM", 0.0) or 0.0)
                        path_row["gsdPromotedFromFov"] = area_db_values.get("gsdPromotedFromFov")
                        self._append_next_collab_fov_adjust_log(
                            lines,
                            context=(
                                f"NEXTCOLLAB AREA PATH1 UAV{int(candidate.get('aircraftID', 0) or 0)}"
                                f"/P{int(candidate.get('pieceIndex', 0) or 0)}"
                            ),
                            base_fov_deg=resolved_camera_base_fov,
                            adjusted_fov_deg=resolved_fov,
                        )

                    # Recalculate ETA using resolved vel (km/h → m/s), turn phase stays at fixed speed
                    _rv = path_row.get("resolvedVelMps") if isinstance(path_row, dict) else None
                    _mission_mps = float(_rv) / 3.6 if _rv is not None and float(_rv) > 0.0 else float(TURN_PREVIEW_SPEED_MPS)
                    _tangent_xy = path_row.get("tangentXY") if isinstance(path_row, dict) else None
                    _target_xy = path_row.get("targetXY") if isinstance(path_row, dict) else None
                    _horizon_s = float(path_row.get("horizonSec", 0.0) or 0.0) if isinstance(path_row, dict) else 0.0
                    if (
                        isinstance(_tangent_xy, (tuple, list)) and len(_tangent_xy) >= 2
                        and isinstance(_target_xy, (tuple, list)) and len(_target_xy) >= 2
                    ):
                        _t_xy = (float(_tangent_xy[0]), float(_tangent_xy[1]))
                        _f_xy = (float(_target_xy[0]), float(_target_xy[1]))
                        _new_eta = _horizon_s + (_distance(_t_xy, _f_xy) / _mission_mps)
                        path_row["estimatedTotalSec"] = float(_new_eta)
                        # Update marker/timeline ETA for non-turn items
                        for _mk in path_row.get("markerRows", []):
                            if isinstance(_mk, dict) and _mk.get("kind") not in ("turn", "tangent"):
                                _mk["etaSec"] = float(_new_eta)
                        for _tl in path_row.get("timelineRows", []):
                            if isinstance(_tl, dict) and _tl.get("kind") not in ("turn", "tangent"):
                                _tl["etaSec"] = float(_new_eta)
                        # Update phase rows
                        for _ph in path_row.get("phaseRows", []):
                            if isinstance(_ph, dict) and _ph.get("kind") == "ingress":
                                _ph["endSec"] = float(_new_eta)

                    path_rows.append(path_row)
                eta_total_sec = float(path_row.get("estimatedTotalSec", 0.0) or 0.0) if isinstance(path_row, dict) else 0.0
                eta_tangent_sec = float(candidate.get("horizonSec", 0.0) or 0.0)
                sep_cand_val = float(path_row.get("sepCandM", 0.0) or 0.0) if isinstance(path_row, dict) else 0.0
                resolved_fov_val = path_row.get("resolvedFovDeg") if isinstance(path_row, dict) else None
                resolved_vel_val = path_row.get("resolvedVelMps") if isinstance(path_row, dict) else None
                entry_needed = path_row.get("entryTPrimeXY") is not None if isinstance(path_row, dict) else False
                entry_tag = " | T' entry" if entry_needed else ""
                fov_tag = f" | FOV {resolved_fov_val:.1f}" if resolved_fov_val is not None else " | FOV -"
                vel_tag = f" | VEL {resolved_vel_val:.0f}" if resolved_vel_val is not None else ""
                lines.append(
                    f"  UAV{int(candidate.get('aircraftID', 0) or 0)}"
                    f" -> P{int(candidate.get('pieceIndex', 0) or 0)} / {str(candidate.get('targetLabel', '') or '?')}"
                    f" | {str(candidate.get('branch', '') or '')}{int(candidate.get('tangentIndex', 0) or 0)}"
                    f" | W {float(path_row.get('partWidthM', 0.0) or 0.0):.0f}m"
                    f" | Sep {sep_cand_val:.0f}m{entry_tag}{fov_tag}{vel_tag}"
                    f" | angle {float(candidate.get('entryAngleDeg', 0.0) or 0.0):.1f}deg"
                    f" | score {float(candidate.get('selectionScore', 0.0) or 0.0):.1f}"
                )

            for aid in active_uavs:
                if int(aid) not in assigned_uavs:
                    lines.append(f"  UAV{int(aid)} -> 미배정")

            self.state.expected_paths = path_rows
            self.state.assignment_path_rows = [dict(row) for row in path_rows]
            self.state.next_mission_rows = []
            self.state.mission_check_rows = []
            self.state.visibility_segments = []
            self.state.show_turn_overlays = True
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Assignment - Path 1", str(exc))
            self._refresh_ui()

    def _assignment_next_mission(self) -> None:
        prev_paths = [
            self._clone_path_row_with_waypoint_end_label(dict(row), "MP_E_1")
            if isinstance(row, dict) and row.get("waypointEndXY") is not None
            else dict(row)
            for row in self.state.expected_paths
            if isinstance(row, dict)
        ]
        if not prev_paths:
            QMessageBox.warning(self, "Next Mission", "먼저 Make Waypoint을 실행해 주세요.")
            return

        try:
            # --- 1) MP_E에서 UAV 오버라이드 + 이미 할당된 target 수집 ---
            uav_overrides: Dict[int, Tuple[Tuple[float, float], float, float]] = {}
            exclude_target_groups: set[str] = set()
            for row in prev_paths:
                if not isinstance(row, dict):
                    continue
                aid = int(row.get("aircraftID", 0) or 0)
                piece_idx = int(row.get("pieceIndex", 0) or 0)
                target_label = str(row.get("targetLabel", "") or "")
                if target_label:
                    exclude_target_groups.add(self._target_group_key(piece_idx, target_label))
                mp_end_raw = row.get("waypointEndXY")
                mp_start_raw = row.get("waypointStartXY")
                if mp_end_raw is None or mp_start_raw is None:
                    continue
                mp_end_xy = (float(mp_end_raw[0]), float(mp_end_raw[1]))
                mp_start_xy = (float(mp_start_raw[0]), float(mp_start_raw[1]))
                bearing_deg = _bearing_deg_from_xy(mp_start_xy, mp_end_xy)
                if bearing_deg is None:
                    continue
                vel_kmh = float(row.get("resolvedVelMps", 0.0) or 0.0)
                vel_mps = vel_kmh / 3.6 if vel_kmh > 0.0 else TURN_PREVIEW_SPEED_MPS
                turn_radius_m = self._turn_radius_for_speed_m(vel_mps)
                uav_overrides[aid] = (mp_end_xy, bearing_deg, turn_radius_m)

            if not uav_overrides:
                QMessageBox.warning(self, "Next Mission", "MP_E 데이터가 없습니다.")
                return

            lines = [
                "[AP2] Assignment - Next Mission",
                f"  excluded groups: {sorted(exclude_target_groups)}",
            ]
            for aid, (pos, hdg, rad) in sorted(uav_overrides.items()):
                lines.append(f"  UAV{aid}: MP_E=({pos[0]:.1f}, {pos[1]:.1f}) hdg={hdg:.1f}° R={rad:.0f}m")

            # --- 2) 남은 영역의 FA/FB, NA/NB와 접선점(T2) 찾기 ---
            tangent_rows, tangent_lines = self._build_tangent_check_rows(
                uav_overrides=uav_overrides,
                exclude_target_groups=exclude_target_groups,
            )
            if tangent_lines:
                lines.extend(tangent_lines)

            # --- 3) 선회 길이 + 각도 기반 최적 배정 (assignment path 1과 동일) ---
            candidate_by_uav_target: Dict[Tuple[int, str], Dict[str, Any]] = {}
            active_uavs = sorted(uav_overrides.keys())
            target_keys: List[str] = []
            seen_target_keys: set = set()

            for row in tangent_rows:
                if not isinstance(row, dict):
                    continue
                aid = int(row.get("aircraftID", 0) or 0)
                target_xy = row.get("targetXY")
                tangent_xy = row.get("tangentXY")
                shape_center_xy = row.get("shapeCenterXY")
                if not (
                    isinstance(target_xy, (tuple, list)) and len(target_xy) >= 2
                    and isinstance(tangent_xy, (tuple, list)) and len(tangent_xy) >= 2
                    and isinstance(shape_center_xy, (tuple, list)) and len(shape_center_xy) >= 2
                ):
                    continue
                target_point_xy = (float(target_xy[0]), float(target_xy[1]))
                tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
                shape_center_point_xy = (float(shape_center_xy[0]), float(shape_center_xy[1]))
                angle_deg = _undirected_angle_deg(
                    target_point_xy, shape_center_point_xy, tangent_point_xy,
                )
                if angle_deg is None:
                    continue
                piece_index = int(row.get("pieceIndex", 0) or 0)
                target_label = str(row.get("targetLabel", "") or f"P{piece_index}")
                target_key = self._target_group_key(piece_index, target_label)
                candidate = dict(row)
                candidate["targetKey"] = target_key
                candidate["targetGroupKey"] = target_key
                candidate["entryAngleDeg"] = float(angle_deg)
                candidate["segmentLenM"] = _distance(tangent_point_xy, target_point_xy)
                candidate["selectionScore"] = (
                    float(candidate["entryAngleDeg"]) * float(ASSIGNMENT_PATH1_ANGLE_WEIGHT)
                    + float(candidate.get("horizonSec", 0.0) or 0.0) * float(ASSIGNMENT_PATH1_TURN_TIME_WEIGHT)
                )
                score_key = (
                    float(candidate["selectionScore"]),
                    float(candidate.get("horizonSec", 0.0) or 0.0),
                    float(candidate["segmentLenM"]),
                )
                existing = candidate_by_uav_target.get((aid, target_key))
                if existing is None:
                    candidate_by_uav_target[(aid, target_key)] = candidate
                else:
                    existing_key = (
                        float(existing.get("selectionScore", 0.0) or 0.0),
                        float(existing.get("horizonSec", 0.0) or 0.0),
                        float(existing.get("segmentLenM", 0.0) or 0.0),
                    )
                    if score_key < existing_key:
                        candidate_by_uav_target[(aid, target_key)] = candidate
                if target_key not in seen_target_keys:
                    seen_target_keys.add(target_key)
                    target_keys.append(target_key)

            if not candidate_by_uav_target or not active_uavs or not target_keys:
                lines.append("  남은 영역에 대한 tangent candidate 없음")
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            assign_count = min(len(active_uavs), len(target_keys))
            best_records: List[Dict[str, Any]] = []
            best_score: Optional[Tuple[float, float, float]] = None

            for uav_subset in itertools.combinations(active_uavs, assign_count):
                for target_perm in itertools.permutations(target_keys, assign_count):
                    candidate_rows: List[Dict[str, Any]] = []
                    weighted_sum = 0.0
                    horizon_sum = 0.0
                    segment_sum = 0.0
                    valid = True
                    for aid, target_key in zip(uav_subset, target_perm):
                        candidate = candidate_by_uav_target.get((int(aid), str(target_key)))
                        if candidate is None:
                            valid = False
                            break
                        candidate_rows.append(candidate)
                        weighted_sum += float(candidate.get("selectionScore", 0.0) or 0.0)
                        horizon_sum += float(candidate.get("horizonSec", 0.0) or 0.0)
                        segment_sum += float(candidate.get("segmentLenM", 0.0) or 0.0)
                    if not valid:
                        continue
                    score = (float(weighted_sum), float(horizon_sum), float(segment_sum))
                    if best_score is None or score < best_score:
                        best_score = score
                        best_records = candidate_rows

            if not best_records:
                lines.append("  중복 없는 배정 조합을 만들지 못함")
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            # --- path row 빌드 (assignment path 1과 동일) ---
            near_point_lookup: Dict[Tuple[int, str], List[Tuple[float, float]]] = {}
            for seg_row in self.state.mid_line_segments:
                if not isinstance(seg_row, dict):
                    continue
                seg_piece_idx = int(seg_row.get("pieceIndex", 0) or 0)
                seg_centers = seg_row.get("stage2Centers")
                if not isinstance(seg_centers, list):
                    continue
                for seg_center in seg_centers:
                    if not isinstance(seg_center, dict):
                        continue
                    # FA label → nearFaceLineXY (FA에서 진입 시 near face)
                    seg_label = str(seg_center.get("label", "") or "").strip().upper()
                    seg_line_raw = seg_center.get("nearFaceLineXY")
                    if isinstance(seg_line_raw, list) and len(seg_line_raw) >= 2:
                        endpoints = [
                            (float(p[0]), float(p[1]))
                            for p in seg_line_raw
                            if isinstance(p, (tuple, list)) and len(p) >= 2
                        ]
                        if len(endpoints) >= 2:
                            near_point_lookup[(seg_piece_idx, seg_label)] = [endpoints[0], endpoints[-1]]
                    # NP label → lineXY (NP에서 진입 시 far face가 entry line)
                    near_label = str(seg_center.get("nearLabel", "") or "").strip().upper()
                    if near_label:
                        far_line_raw = seg_center.get("lineXY")
                        if isinstance(far_line_raw, list) and len(far_line_raw) >= 2:
                            far_endpoints = [
                                (float(p[0]), float(p[1]))
                                for p in far_line_raw
                                if isinstance(p, (tuple, list)) and len(p) >= 2
                            ]
                            if len(far_endpoints) >= 2:
                                near_point_lookup[(seg_piece_idx, near_label)] = [far_endpoints[0], far_endpoints[-1]]

            path2_rows: List[Dict[str, Any]] = []
            for candidate in sorted(best_records, key=lambda r: int(r.get("aircraftID", 0) or 0)):
                cand_aid = int(candidate.get("aircraftID", 0) or 0)
                ovr = uav_overrides.get(cand_aid)
                origin_ovr = (ovr[0], ovr[1]) if ovr else None  # (position_xy, heading_deg)
                path_row = self._build_assignment_path_1_row(
                    candidate,
                    origin_override=origin_ovr,
                    tangent_label="T2",
                    suppress_last_turn_label=True,
                )
                if path_row is None:
                    continue
                part_width_m = float(path_row.get("partWidthM", 0.0) or 0.0)
                if float(path_row.get("dbSepM", 0.0) or 0.0) <= 0.0 and part_width_m > 0.0:
                    db_row = self._next_collab_entry_tprime_db_row(part_width_m)
                    if isinstance(db_row, dict):
                        path_row["dbSepM"] = float(db_row.get("sep", 0.0) or 0.0)
                        path_row["dbWidthM"] = float(db_row.get("width", 0.0) or 0.0)
                cand_piece = int(candidate.get("pieceIndex", 0) or 0)
                cand_label = str(candidate.get("targetLabel", "") or "").strip().upper()
                near_pt = near_point_lookup.get((cand_piece, cand_label))
                if near_pt is not None:
                    path_row["entryLineEndpointsXY"] = near_pt

                # Sep_cand / Entry T' / FOV
                tangent_xy_raw = path_row.get("tangentXY")
                target_xy_raw = path_row.get("targetFaceXY") or path_row.get("targetXY")
                if (
                    near_pt is not None
                    and isinstance(tangent_xy_raw, (tuple, list)) and len(tangent_xy_raw) >= 2
                    and isinstance(target_xy_raw, (tuple, list)) and len(target_xy_raw) >= 2
                ):
                    t_xy = (float(tangent_xy_raw[0]), float(tangent_xy_raw[1]))
                    fa_xy = (float(target_xy_raw[0]), float(target_xy_raw[1]))
                    ep0, ep1 = near_pt[0], near_pt[1]
                    sep_cand = (_distance(t_xy, ep0) + _distance(t_xy, ep1)) / 2.0

                    # Path 2 correction: near_pt is defined relative to the original
                    # UAV position.  When T2 approaches from the opposite direction
                    # (from MP_E_1), near_pt may point to the far face instead of the
                    # actual entry face, inflating sep_cand.  Cross-check against the
                    # real polygon boundary distance and use the smaller value.
                    _part_poly_raw = path_row.get("partPolygonXY")
                    if isinstance(_part_poly_raw, list) and len(_part_poly_raw) >= 3:
                        try:
                            _pp = Polygon([
                                (float(p[0]), float(p[1]))
                                for p in _part_poly_raw
                                if isinstance(p, (tuple, list)) and len(p) >= 2
                            ])
                            if not _pp.is_valid:
                                _pp = _pp.buffer(0)
                            if not _pp.is_empty:
                                _real_dist = _pp.exterior.distance(Point(t_xy))
                                if _real_dist < sep_cand:
                                    sep_cand = float(_real_dist)
                        except Exception:
                            pass

                    db_sep_row = self._next_collab_entry_tprime_db_row(part_width_m)
                    target_sep = self._next_collab_entry_tprime_target_sep_m(part_width_m)

                    entry_t_prime_xy = None
                    if sep_cand > target_sep and target_sep > 0.0:
                        ingress_dx = fa_xy[0] - t_xy[0]
                        ingress_dy = fa_xy[1] - t_xy[1]
                        ingress_len = math.hypot(ingress_dx, ingress_dy)
                        if ingress_len > 1e-6:
                            ux_i = ingress_dx / ingress_len
                            uy_i = ingress_dy / ingress_len
                            lo, hi = 0.0, ingress_len
                            for _ in range(40):
                                mid_v = (lo + hi) * 0.5
                                test_xy = (t_xy[0] + ux_i * mid_v, t_xy[1] + uy_i * mid_v)
                                test_sep = (_distance(test_xy, ep0) + _distance(test_xy, ep1)) * 0.5
                                if test_sep > target_sep:
                                    lo = mid_v
                                else:
                                    hi = mid_v
                            entry_t_prime_xy = (t_xy[0] + ux_i * lo, t_xy[1] + uy_i * lo)
                            sep_cand = (_distance(entry_t_prime_xy, ep0) + _distance(entry_t_prime_xy, ep1)) * 0.5

                            # Degenerate case: when the aircraft is far from the
                            # polygon the geometric minimum sep along the ingress
                            # line can still be larger than target_sep, and the
                            # binary search converges to fa_xy so the entry
                            # waypoint ends up past the area.  Snap to the closest
                            # approach to the near-face midpoint so WP_E_0 stays
                            # on the near side.
                            if sep_cand > target_sep + 1e-6:
                                ep_mid_x = (ep0[0] + ep1[0]) * 0.5
                                ep_mid_y = (ep0[1] + ep1[1]) * 0.5
                                proj_len = (
                                    (ep_mid_x - t_xy[0]) * ux_i
                                    + (ep_mid_y - t_xy[1]) * uy_i
                                )
                                proj_len = max(0.0, min(ingress_len, proj_len))
                                entry_t_prime_xy = (
                                    t_xy[0] + ux_i * proj_len,
                                    t_xy[1] + uy_i * proj_len,
                                )
                                sep_cand = (
                                    _distance(entry_t_prime_xy, ep0)
                                    + _distance(entry_t_prime_xy, ep1)
                                ) * 0.5

                    path_row["sepCandM"] = float(sep_cand)
                    path_row["entryTPrimeXY"] = entry_t_prime_xy

                    area_db_values = self._next_collab_area_resolved_db_values(part_width_m, sep_cand)
                    resolved_camera_base_fov = area_db_values.get("resolvedBaseFovDeg")
                    resolved_db_fov = area_db_values.get("resolvedDbFovDeg")
                    resolved_fov = area_db_values.get("resolvedFovDeg")
                    resolved_foot = area_db_values.get("resolvedFootM")
                    resolved_vel = area_db_values.get("resolvedVelMps")
                    path_row["dbSepM"] = float(area_db_values.get("dbSepM", 0.0) or 0.0)
                    path_row["dbWidthM"] = float(area_db_values.get("dbWidthM", 0.0) or 0.0)
                    path_row["resolvedDbFovDeg"] = resolved_db_fov
                    path_row["resolvedBaseFovDeg"] = resolved_camera_base_fov
                    path_row["resolvedFovDeg"] = resolved_fov
                    path_row["resolvedFootM"] = resolved_foot
                    path_row["resolvedVelMps"] = resolved_vel
                    path_row["gsdSepM"] = float(area_db_values.get("gsdSepM", 0.0) or 0.0)
                    path_row["gsdPromotedFromFov"] = area_db_values.get("gsdPromotedFromFov")
                    self._append_next_collab_fov_adjust_log(
                        lines,
                        context=(
                            f"NEXTCOLLAB AREA NEXT_MISSION UAV{int(candidate.get('aircraftID', 0) or 0)}"
                            f"/P{int(candidate.get('pieceIndex', 0) or 0)}"
                        ),
                        base_fov_deg=resolved_camera_base_fov,
                        adjusted_fov_deg=resolved_fov,
                    )

                # ETA recalculation
                _rv = path_row.get("resolvedVelMps")
                _mission_mps = float(_rv) / 3.6 if _rv is not None and float(_rv) > 0.0 else float(TURN_PREVIEW_SPEED_MPS)
                _tangent_xy = path_row.get("tangentXY")
                _target_xy = path_row.get("targetFaceXY") or path_row.get("targetXY")
                _horizon_s = float(path_row.get("horizonSec", 0.0) or 0.0)
                if (
                    isinstance(_tangent_xy, (tuple, list)) and len(_tangent_xy) >= 2
                    and isinstance(_target_xy, (tuple, list)) and len(_target_xy) >= 2
                ):
                    _t_xy = (float(_tangent_xy[0]), float(_tangent_xy[1]))
                    _f_xy = (float(_target_xy[0]), float(_target_xy[1]))
                    _new_eta = _horizon_s + (_distance(_t_xy, _f_xy) / _mission_mps)
                    path_row["estimatedTotalSec"] = float(_new_eta)

                path_row["source"] = "next_mission"
                path2_rows.append(path_row)

                sep_val = float(path_row.get("sepCandM", 0.0) or 0.0)
                fov_val = path_row.get("resolvedFovDeg")
                vel_val = path_row.get("resolvedVelMps")
                entry_tag = " | T'" if path_row.get("entryTPrimeXY") is not None else ""
                fov_tag = f" | FOV {fov_val:.1f}" if fov_val is not None else " | FOV -"
                vel_tag = f" | VEL {vel_val:.0f}" if vel_val is not None else ""
                lines.append(
                    f"  UAV{int(candidate.get('aircraftID', 0) or 0)}"
                    f" -> P{int(candidate.get('pieceIndex', 0) or 0)} / {str(candidate.get('targetLabel', '') or '?')}"
                    f" | {str(candidate.get('branch', '') or '')}{int(candidate.get('tangentIndex', 0) or 0)}"
                    f" | Sep {sep_val:.0f}m{entry_tag}{fov_tag}{vel_tag}"
                    f" | angle {float(candidate.get('entryAngleDeg', 0.0) or 0.0):.1f}°"
                    f" | score {float(candidate.get('selectionScore', 0.0) or 0.0):.1f}"
                )

            # Path 1 + Path 2 병합
            all_paths = list(prev_paths) + path2_rows
            self.state.expected_paths = all_paths
            self.state.assignment_path_rows = [dict(r) for r in all_paths]
            # MP_E 턴 원 시각화 유지
            nm_rows: list = []
            for aid, (pos, hdg, rad) in uav_overrides.items():
                vel_mps_val = rad  # approximate — use actual from prev_paths
                nm_rows.append({
                    "aircraftID": aid,
                    "mpEndXY": pos,
                    "bearingDeg": hdg,
                    "velMps": 0.0,
                    "turnRadiusM": rad,
                })
            self.state.next_mission_rows = nm_rows
            self.state.show_next_mission_circles = True
            self.state.show_turn_overlays = False
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Next Mission", str(exc))
            self._refresh_ui()

    def _make_new_area(self) -> None:
        if not self._ensure_split_branch_allowed("Make New Area"):
            return
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Make New Area", "먼저 Area Division Run을 실행해 주세요.")
            return
        if not self.state.mid_line_segments:
            QMessageBox.warning(self, "Make New Area", "먼저 Mid Line Generation을 실행해 주세요.")
            return

        try:
            overlay_by_piece = {
                int(row.get("pieceIndex", 0) or 0): row
                for row in self.state.mid_line_segments
                if isinstance(row, dict)
            }
            changed_count = 0
            lines = ["[NEW AREA] split parts -> oriented boxes"]

            for piece in sorted(self.state.split_result.pieces, key=lambda row: int(row.piece_index or 0)):
                overlay = overlay_by_piece.get(int(piece.piece_index or 0))
                if overlay is None:
                    aid = int(piece.assigned_uav or 0)
                    lines.append(
                        f"  P{int(piece.piece_index or 0)}"
                        f"{f' / UAV{aid}' if aid > 0 else ''}: no mid-line box"
                    )
                    continue
                aid = int(piece.assigned_uav or 0)
                new_poly, info = self._make_new_area_polygon(piece, overlay, None)
                part_results = info.get("partResults")
                part_summary = ""
                if isinstance(part_results, list) and part_results:
                    tokens: List[str] = []
                    for row in part_results:
                        if not isinstance(row, dict):
                            continue
                        name = str(row.get("name", "") or "?")
                        reason = str(row.get("reason", "") or "")
                        old_part_area = float(row.get("oldAreaM2", 0.0) or 0.0)
                        new_part_area = float(row.get("newAreaM2", old_part_area) or old_part_area)
                        if new_part_area > 0.0 or old_part_area > 0.0:
                            tokens.append(f"{name}:{reason}({old_part_area:.0f}->{new_part_area:.0f})")
                        else:
                            tokens.append(f"{name}:{reason}")
                    if tokens:
                        part_summary = " [" + ", ".join(tokens) + "]"
                area_text = (
                    f"(area {float(info.get('oldAreaM2', 0.0)):.0f}"
                    f"->{float(info.get('newAreaM2', info.get('oldAreaM2', 0.0))):.0f}m2)"
                )
                if new_poly is None:
                    lines.append(
                        f"  P{int(piece.piece_index or 0)}"
                        f"{f' / UAV{aid}' if aid > 0 else ''}: keep "
                        f"{str(info.get('reason', 'unchanged'))} {area_text}{part_summary}"
                    )
                    continue
                self._replace_piece_polygon(piece, new_poly, review_patch=info)
                changed_count += 1
                lines.append(
                    f"  P{int(piece.piece_index or 0)}"
                    f"{f' / UAV{aid}' if aid > 0 else ''}: boxed {area_text}{part_summary}"
                )

            if changed_count <= 0:
                lines.append("  no area changed")

            _reference_bearing_deg, overlays, _mid_lines = self._mid_line_overlay_bundle(self.state.split_result)
            self.state.mid_line_segments = overlays
            self.state.expected_paths = []
            self.state.assignment_path_rows = []
            self.state.mission_check_rows = []
            self.state.flight_plans_0303 = []
            self.state.flight_plans_0304 = []
            self.state.visibility_segments = []
            self.state.tangent_checks = []
            self.state.show_turn_overlays = True
            self.state.mode = MODE_RESULT_READY
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make New Area", str(exc))
            self._refresh_ui()

    def _check_mission(self) -> None:
        if not self.state.assignment_path_rows:
            QMessageBox.warning(self, "Check Mission", "먼저 Assignment - Path 1을 수행해 주세요.")
            return

        try:
            self.state.mission_check_rows = []
            checked_rows: List[Dict[str, Any]] = []
            lines = ["[MISSION] check mission start by T / Near line / SEP"]
            for base_row in sorted(
                self.state.assignment_path_rows,
                key=lambda row: (
                    int(row.get("aircraftID", 0) or 0),
                    int(row.get("pieceIndex", 0) or 0),
                ),
            ):
                if not isinstance(base_row, dict):
                    continue
                checked_row = self._build_check_mission_row(base_row)
                if checked_row is None:
                    lines.append(
                        f"  UAV{int(base_row.get('aircraftID', 0) or 0)}"
                        f" -> P{int(base_row.get('pieceIndex', 0) or 0)} / {str(base_row.get('targetLabel', '') or '?')}"
                        " : mission check failed"
                    )
                    continue
                checked_rows.append(checked_row)
                lines.append(
                    f"  UAV{int(checked_row.get('aircraftID', 0) or 0)}"
                    f" -> P{int(checked_row.get('pieceIndex', 0) or 0)} / {str(checked_row.get('targetLabel', '') or '?')}"
                    f" | A {float(checked_row.get('missionDistanceM', 0.0) or 0.0):.0f}m"
                    f" | width {float(checked_row.get('nearWidthM', 0.0) or 0.0):.0f}m"
                    f" | sep {float(checked_row.get('dbSepM', 0.0) or 0.0):.0f}m"
                    f" | start {str(checked_row.get('missionStartLabel', 'T') or 'T')}"
                )

            if not checked_rows:
                self.state.mission_check_rows = []
                lines.append("  usable mission check row 없음")
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            self.state.mission_check_rows = checked_rows
            self.state.expected_paths = list(checked_rows)
            self.state.visibility_segments = []
            self.state.tangent_checks = []
            self.state.show_turn_overlays = True
            self.state.mode = MODE_RESULT_READY
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Check Mission", str(exc))
            self._refresh_ui()

    def _make_waypoint(self) -> None:
        if not self._ensure_split_branch_allowed("Make Waypoint"):
            return
        source_rows = self.state.assignment_path_rows
        if not source_rows:
            QMessageBox.warning(self, "Make Waypoint", "먼저 Assignment - Path 1을 실행해 주세요.")
            return

        try:
            expected_rows: List[Dict[str, Any]] = []
            lines = ["[WP] make waypoint from tangent + sep + mid-line-2 length"]
            for base_row in sorted(
                source_rows,
                key=lambda row: (
                    int(row.get("aircraftID", 0) or 0),
                    int(row.get("pieceIndex", 0) or 0),
                ),
            ):
                if not isinstance(base_row, dict):
                    continue
                waypoint_row = self._build_make_waypoint_row(base_row)
                if waypoint_row is None:
                    lines.append(
                        f"  UAV{int(base_row.get('aircraftID', 0) or 0)}"
                        f" -> P{int(base_row.get('pieceIndex', 0) or 0)}: waypoint build failed"
                    )
                    continue

                expected_rows.append(waypoint_row)
                lines.append(
                    f"  UAV{int(waypoint_row.get('aircraftID', 0) or 0)}"
                    f" -> P{int(waypoint_row.get('pieceIndex', 0) or 0)} / {str(waypoint_row.get('targetLabel', '') or '?')}"
                    f" | start {str(waypoint_row.get('startLabel', 'T') or 'T')}"
                    f" | shapeLen {float(waypoint_row.get('shapeLengthM', 0.0) or 0.0):.0f}m"
                    f" | sepCand {float(waypoint_row.get('sepCandM', 0.0) or 0.0):.0f}m"
                    f" | FOV {float(waypoint_row.get('resolvedFovDeg', 0.0) or 0.0):.1f}"
                    f" | VEL {float(waypoint_row.get('resolvedVelMps', 0.0) or 0.0):.0f}"
                )

            if not expected_rows:
                lines.append("  usable waypoint row 없음")
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            self.state.expected_paths = expected_rows
            self.state.visibility_segments = []
            self.state.tangent_checks = []
            self.state.show_turn_overlays = True
            self.state.mode = MODE_RESULT_READY
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make Waypoint", str(exc))
            self._refresh_ui()

    def _make_path_0(self) -> None:
        if not self._mid_line_no_split_mode():
            QMessageBox.warning(self, "Make Path - 0", "먼저 Mid Line Generation에서 DB cover 상태를 확인해 주세요.")
            return
        try:
            lines = ["[PATH0] Make Path - 0"]
            expected_rows: List[Dict[str, Any]] = []
            db_rows = _cached_fov_db_rows()
            db_max_sep = max((float(r[1]) for r in db_rows), default=0.0) if db_rows else 0.0
            area_path0_trigger_sep = self._next_collab_area_path0_trigger_sep_m(db_max_sep)
            lines.append(f"  DB max sep = {db_max_sep:.0f}m")
            lines.append(f"  Area Path0 trigger sep = {area_path0_trigger_sep:.0f}m")

            for overlay in self.state.mid_line_segments:
                if not isinstance(overlay, dict):
                    continue
                aid = int(overlay.get("aircraftID", 0) or 0)
                piece_index = int(overlay.get("pieceIndex", 0) or 0)
                if aid <= 0:
                    continue

                # --- T0 data from mid line preview ---
                t0_tangent_raw = overlay.get("t0TangentXY")
                t0_target_raw = overlay.get("t0TargetXY")
                t0_branch = str(overlay.get("t0Branch", "") or "")
                t0_horizon_sec = float(overlay.get("t0HorizonSec", 0.0) or 0.0)
                if (
                    not isinstance(t0_tangent_raw, (tuple, list)) or len(t0_tangent_raw) < 2
                    or not isinstance(t0_target_raw, (tuple, list)) or len(t0_target_raw) < 2
                ):
                    lines.append(f"  P{piece_index}/UAV{aid}: T0 preview 없음, skip")
                    continue

                t0_xy = (float(t0_tangent_raw[0]), float(t0_tangent_raw[1]))
                fa_xy = (float(t0_target_raw[0]), float(t0_target_raw[1]))

                uav_state = self._uav_state_for_aircraft(aid)
                if uav_state is None:
                    continue
                origin_xy, heading_deg = uav_state

                width_m = float(
                    overlay.get("widthStartM", 0.0)
                    or overlay.get("maxWidthM", 0.0)
                    or overlay.get("widthM", 0.0)
                    or 0.0
                )
                length_m = float(overlay.get("lengthM", 0.0) or 0.0)
                bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)

                # --- Near face endpoints from boxXY ---
                # boxXY corners: [0]=(min_s,min_t) [1]=(max_s,min_t) [2]=(max_s,max_t) [3]=(min_s,max_t)
                # Near face = the short edge of the box closest to the UAV (opposite side of FA)
                box_xy_raw = overlay.get("boxXY")
                mid_line_xy_raw = overlay.get("midLineXY")
                ep0: Optional[Tuple[float, float]] = None
                ep1: Optional[Tuple[float, float]] = None
                if (
                    isinstance(box_xy_raw, list) and len(box_xy_raw) >= 4
                    and isinstance(mid_line_xy_raw, list) and len(mid_line_xy_raw) >= 2
                ):
                    corners = [
                        (float(c[0]), float(c[1]))
                        for c in box_xy_raw
                        if isinstance(c, (tuple, list)) and len(c) >= 2
                    ]
                    if len(corners) >= 4:
                        ml0 = (float(mid_line_xy_raw[0][0]), float(mid_line_xy_raw[0][1]))
                        ml1 = (float(mid_line_xy_raw[1][0]), float(mid_line_xy_raw[1][1]))
                        if _distance(fa_xy, ml1) < _distance(fa_xy, ml0):
                            ep0, ep1 = corners[0], corners[3]
                        else:
                            ep0, ep1 = corners[1], corners[2]
                if ep0 is None or ep1 is None:
                    face_points = self._make_path_face_points(overlay, t0_xy)
                    if isinstance(face_points, dict):
                        near_face_line_raw = face_points.get("nearFaceLineXY")
                        target_face_raw = face_points.get("targetFaceXY")
                        if isinstance(near_face_line_raw, list) and len(near_face_line_raw) >= 2:
                            near_face_line_xy = [
                                (float(point_xy[0]), float(point_xy[1]))
                                for point_xy in near_face_line_raw
                                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                            ]
                            if len(near_face_line_xy) >= 2:
                                ep0, ep1 = near_face_line_xy[0], near_face_line_xy[1]
                        if isinstance(target_face_raw, (tuple, list)) and len(target_face_raw) >= 2:
                            fa_xy = (float(target_face_raw[0]), float(target_face_raw[1]))

                # --- Sep calculation ---
                if ep0 is not None and ep1 is not None:
                    d0 = _distance(t0_xy, ep0)
                    d1 = _distance(t0_xy, ep1)
                    sep_cand = (d0 + d1) / 2.0
                else:
                    sep_cand = float(overlay.get("sepStartM", 0.0) or overlay.get("t0ShapePointDistM", 0.0) or 0.0)

                # --- Entry_0 check ---
                entry_t_prime_xy: Optional[Tuple[float, float]] = None
                ingress_dx = fa_xy[0] - t0_xy[0]
                ingress_dy = fa_xy[1] - t0_xy[1]
                ingress_len = math.hypot(ingress_dx, ingress_dy)
                if ingress_len <= 1e-6:
                    lines.append(f"  P{piece_index}/UAV{aid}: ingress 거리 0, skip")
                    continue
                ux = ingress_dx / ingress_len
                uy = ingress_dy / ingress_len

                if sep_cand > area_path0_trigger_sep and area_path0_trigger_sep > 0.0:
                    # Entry_0 필요: binary search for T0'
                    target_sep = area_path0_trigger_sep * float(self._next_collab_area_path0_target_sep_ratio())
                    lo, hi = 0.0, ingress_len
                    if ep0 is not None and ep1 is not None:
                        for _ in range(40):
                            mid_val = (lo + hi) * 0.5
                            test_xy = (t0_xy[0] + ux * mid_val, t0_xy[1] + uy * mid_val)
                            test_sep = (_distance(test_xy, ep0) + _distance(test_xy, ep1)) * 0.5
                            if test_sep > target_sep:
                                lo = mid_val
                            else:
                                hi = mid_val
                        entry_t_prime_xy = (t0_xy[0] + ux * lo, t0_xy[1] + uy * lo)
                        d0 = _distance(entry_t_prime_xy, ep0)
                        d1 = _distance(entry_t_prime_xy, ep1)
                        sep_cand = (d0 + d1) / 2.0

                        # Degenerate case: when the aircraft is far from the
                        # polygon the geometric minimum sep along the ingress
                        # line can still be larger than target_sep, and the
                        # binary search converges to fa_xy so WP_E_0 ends up
                        # past the far face instead of on the near side.  Snap
                        # to the closest approach to the near-face midpoint so
                        # the entry waypoint stays on the near side.
                        if sep_cand > target_sep + 1e-6:
                            ep_mid_x = (ep0[0] + ep1[0]) * 0.5
                            ep_mid_y = (ep0[1] + ep1[1]) * 0.5
                            proj_len = (
                                (ep_mid_x - t0_xy[0]) * ux
                                + (ep_mid_y - t0_xy[1]) * uy
                            )
                            proj_len = max(0.0, min(ingress_len, proj_len))
                            entry_t_prime_xy = (
                                t0_xy[0] + ux * proj_len,
                                t0_xy[1] + uy * proj_len,
                            )
                            sep_cand = (
                                _distance(entry_t_prime_xy, ep0)
                                + _distance(entry_t_prime_xy, ep1)
                            ) * 0.5
                    else:
                        # Fallback: single boundary point
                        shape_pt_raw = overlay.get("t0ShapePointXY")
                        if isinstance(shape_pt_raw, (tuple, list)) and len(shape_pt_raw) >= 2:
                            shape_pt = (float(shape_pt_raw[0]), float(shape_pt_raw[1]))
                            for _ in range(40):
                                mid_val = (lo + hi) * 0.5
                                test_xy = (t0_xy[0] + ux * mid_val, t0_xy[1] + uy * mid_val)
                                if _distance(test_xy, shape_pt) > target_sep:
                                    lo = mid_val
                                else:
                                    hi = mid_val
                            entry_t_prime_xy = (t0_xy[0] + ux * lo, t0_xy[1] + uy * lo)
                            sep_cand = _distance(entry_t_prime_xy, shape_pt)

                # --- Waypoint start ---
                start_xy = entry_t_prime_xy if entry_t_prime_xy is not None else t0_xy
                start_label = "T0'" if entry_t_prime_xy is not None else "T0"

                # --- Shape length: NB→FB 거리 (ingress 방향 기준) ---
                # midLineLengthM = midline 기반 NB→FB euclidean distance
                mid_line_len_m = float(overlay.get("midLineLengthM", 0.0) or 0.0)
                shape_length_m = mid_line_len_m if mid_line_len_m > 0.0 else length_m

                boundary_pts_raw = overlay.get("splitBoundaryPointsXY")
                if isinstance(boundary_pts_raw, list) and len(boundary_pts_raw) >= 2:
                    bp_list = [
                        (float(p[0]), float(p[1]))
                        for p in boundary_pts_raw
                        if isinstance(p, (tuple, list)) and len(p) >= 2
                    ]
                    if len(bp_list) >= 2:
                        proj0 = bp_list[0][0] * ux + bp_list[0][1] * uy
                        proj1 = bp_list[1][0] * ux + bp_list[1][1] * uy
                        proj_len = abs(proj1 - proj0)
                        if proj_len > shape_length_m:
                            shape_length_m = proj_len
                if shape_length_m <= 1e-6:
                    shape_length_m = length_m

                # --- WP_End_0 ---
                mp_end_xy = (
                    start_xy[0] + ux * shape_length_m,
                    start_xy[1] + uy * shape_length_m,
                )

                area_db_values = self._next_collab_area_resolved_db_values(width_m, sep_cand)
                resolved_db_sep = float(area_db_values.get("dbSepM", 0.0) or 0.0)
                resolved_db_width = float(area_db_values.get("dbWidthM", 0.0) or 0.0)
                resolved_camera_base_fov = area_db_values.get("resolvedBaseFovDeg")
                resolved_db_fov = area_db_values.get("resolvedDbFovDeg")
                resolved_fov = area_db_values.get("resolvedFovDeg")
                resolved_foot = area_db_values.get("resolvedFootM")
                resolved_vel = area_db_values.get("resolvedVelMps")

                # --- ETA ---
                mission_speed_mps = (
                    float(resolved_vel) / 3.6
                    if resolved_vel is not None and float(resolved_vel) > 0.0
                    else float(TURN_PREVIEW_SPEED_MPS)
                )
                ingress_eta_sec = _distance(t0_xy, start_xy) / mission_speed_mps if entry_t_prime_xy is not None else 0.0
                start_eta_sec = t0_horizon_sec + ingress_eta_sec
                mission_eta_sec = _distance(start_xy, mp_end_xy) / mission_speed_mps
                end_eta_sec = start_eta_sec + mission_eta_sec

                # --- Build route ---
                route_xy, marker_rows, timeline_rows = self._build_turn_prefix_rows(
                    origin_xy, heading_deg,
                    branch=t0_branch,
                    horizon_s=t0_horizon_sec,
                    tangent_point_xy=t0_xy,
                    tangent_label="T0",
                    suppress_last_turn_label=True,
                )

                if entry_t_prime_xy is not None and _distance(t0_xy, start_xy) > 1e-6:
                    if _distance(route_xy[-1], start_xy) > 1e-6:
                        route_xy.append(start_xy)
                    marker_rows.append({"xy": start_xy, "label": "T0'", "kind": "waypoint_start", "etaSec": start_eta_sec})
                    timeline_rows.append({"label": "T0'", "kind": "waypoint_start", "etaSec": start_eta_sec})
                if _distance(route_xy[-1], mp_end_xy) > 1e-6:
                    route_xy.append(mp_end_xy)
                marker_rows.append({"xy": mp_end_xy, "label": "MP_E_0", "kind": "waypoint_end", "etaSec": end_eta_sec})
                timeline_rows.append({"label": "MP_E_0", "kind": "waypoint_end", "etaSec": end_eta_sec})

                # --- Phase rows (Gantt) ---
                phase_rows: List[Dict[str, Any]] = []
                if t0_horizon_sec > 1e-6:
                    phase_rows.append({"label": "Turn", "kind": "turn", "startSec": 0.0, "endSec": t0_horizon_sec})
                if start_eta_sec > t0_horizon_sec + 1e-6:
                    phase_rows.append({"label": "Ingress", "kind": "ingress", "startSec": t0_horizon_sec, "endSec": start_eta_sec})
                if end_eta_sec > start_eta_sec + 1e-6:
                    phase_rows.append({"label": "Mission", "kind": "waypoint", "startSec": start_eta_sec, "endSec": end_eta_sec})

                expected_row = {
                    "source": "make_path_0",
                    "aircraftID": int(aid),
                    "pieceIndex": int(piece_index),
                    "originXY": origin_xy,
                    "originHeadingDeg": float(heading_deg),
                    "routeXY": route_xy,
                    "markerRows": marker_rows,
                    "timelineRows": timeline_rows,
                    "phaseRows": phase_rows,
                    "estimatedTotalSec": float(end_eta_sec),
                    "tangentXY": t0_xy,
                    "targetXY": fa_xy,
                    "waypointStartXY": start_xy,
                    "waypointEndXY": mp_end_xy,
                    "waypointEndLabel": "MP_E_0",
                    "targetFaceXY": fa_xy,
                    "horizonSec": float(t0_horizon_sec),
                    "branch": t0_branch,
                    "startLabel": start_label,
                    "tangentLabel": "T0",
                    "shapeLengthM": float(shape_length_m),
                    "partWidthM": float(width_m),
                    "sepCandM": float(sep_cand),
                    "dbSepM": float(resolved_db_sep),
                    "dbWidthM": float(resolved_db_width),
                    "resolvedDbFovDeg": resolved_db_fov,
                    "resolvedBaseFovDeg": resolved_camera_base_fov,
                    "resolvedFovDeg": resolved_fov,
                    "resolvedFootM": resolved_foot,
                    "resolvedVelMps": resolved_vel,
                    "gsdSepM": float(area_db_values.get("gsdSepM", 0.0) or 0.0),
                    "gsdPromotedFromFov": area_db_values.get("gsdPromotedFromFov"),
                    "entryTPrimeXY": entry_t_prime_xy,
                    "bearingDeg": float(bearing_deg),
                }
                expected_rows.append(expected_row)

                # --- Log ---
                entry_tag = f"T0'(sep={sep_cand:.0f}m)" if entry_t_prime_xy is not None else f"Direct(sep={sep_cand:.0f}m)"
                fov_tag = f"FOV={resolved_fov:.1f}°" if resolved_fov is not None else "FOV=-"
                vel_tag = f"VEL={resolved_vel:.0f}km/h" if resolved_vel is not None else ""
                t_to_tprime = _distance(t0_xy, start_xy) if entry_t_prime_xy is not None else 0.0
                start_to_end = _distance(start_xy, mp_end_xy)
                lines.append(
                    f"  P{piece_index}/UAV{aid}: {entry_tag}"
                    f" | W={width_m:.0f}m | shapeL={shape_length_m:.0f}m (midLine={mid_line_len_m:.0f}m)"
                    f" | T→T'={t_to_tprime:.0f}m | {start_label}→WP_E={start_to_end:.0f}m"
                    f" | {fov_tag} {vel_tag}"
                    f" | ETA={end_eta_sec:.1f}s"
                )
                self._append_next_collab_fov_adjust_log(
                    lines,
                    context=f"NEXTCOLLAB AREA PATH0 UAV{aid}/P{piece_index}",
                    base_fov_deg=resolved_camera_base_fov,
                    adjusted_fov_deg=resolved_fov,
                )

            if not expected_rows:
                lines.append("  경로 생성 실패: T0 preview가 있는 overlay가 없습니다.")
            self.state.expected_paths = expected_rows
            self.state.assignment_path_rows = []
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make Path - 0", str(exc))
            self._refresh_ui()

    def _make_sweep(self) -> None:
        path_0_rows = [
            row for row in self.state.expected_paths
            if isinstance(row, dict) and str(row.get("source", "") or "") == "make_path_0"
        ]
        if not path_0_rows:
            QMessageBox.warning(self, "Make Sweep", "먼저 Make Path - 0을 실행해 주세요.")
            return
        try:
            lines = ["[SWEEP] Make Sweep"]
            db_rows = self._fov_db_rows()

            for path_row in path_0_rows:
                aid = int(path_row.get("aircraftID", 0) or 0)
                piece_idx = int(path_row.get("pieceIndex", 0) or 0)

                # ---- find matching overlay ----
                overlay: Optional[Dict[str, Any]] = None
                for ov in self.state.mid_line_segments:
                    if (
                        isinstance(ov, dict)
                        and int(ov.get("aircraftID", 0) or 0) == aid
                        and int(ov.get("pieceIndex", 0) or 0) == piece_idx
                    ):
                        overlay = ov
                        break
                if overlay is None:
                    lines.append(f"  UAV{aid}/P{piece_idx}: overlay 없음, skip")
                    continue

                sep_cand_m = float(path_row.get("sepCandM", 0.0) or 0.0)
                sep_ref_m = float(path_row.get("gsdSepM", 0.0) or 0.0)
                if sep_ref_m <= 0.0:
                    sep_ref_m = float(path_row.get("dbSepM", 0.0) or 0.0)
                if sep_ref_m <= 0.0:
                    sep_ref_m = self._next_collab_db_sep_requirement_m(sep_cand_m)
                resolved_fov_deg = path_row.get("resolvedFovDeg")
                resolved_db_fov_deg = path_row.get("resolvedDbFovDeg")
                foot_m = float(
                    self._next_collab_area_spacing_footprint_m(
                        sep_ref_m,
                        resolved_fov_deg,
                        db_fov_deg=resolved_db_fov_deg,
                    ) or 0.0
                )
                if foot_m <= 0.0:
                    lines.append(f"  UAV{aid}/P{piece_idx}: foot 결정 불가, skip")
                    continue

                # Prefer the polygon already localized onto the path row.
                piece_poly = self._path_row_piece_polygon_xy(path_row)

                # ---- s-t frame: ingress direction (T0 → WP_E) ----
                wp_start_raw = path_row.get("waypointStartXY") or path_row.get("tangentXY")
                wp_end_raw = path_row.get("waypointEndXY")
                if (
                    isinstance(wp_start_raw, (tuple, list)) and len(wp_start_raw) >= 2
                    and isinstance(wp_end_raw, (tuple, list)) and len(wp_end_raw) >= 2
                ):
                    dx = float(wp_end_raw[0]) - float(wp_start_raw[0])
                    dy = float(wp_end_raw[1]) - float(wp_start_raw[1])
                    ingress_len = math.hypot(dx, dy)
                    if ingress_len > 1e-6:
                        ux = dx / ingress_len   # s-axis (ingress direction)
                        uy = dy / ingress_len
                    else:
                        theta = math.radians(float(overlay.get("bearingDeg", 0.0) or 0.0) % 360.0)
                        ux = math.sin(theta)
                        uy = math.cos(theta)
                else:
                    theta = math.radians(float(overlay.get("bearingDeg", 0.0) or 0.0) % 360.0)
                    ux = math.sin(theta)
                    uy = math.cos(theta)
                vx = uy                # t-axis (perpendicular to ingress)
                vy = -ux

                # determine s-t extents from polygon or boxXY
                if piece_poly is not None and not piece_poly.is_empty:
                    ext_coords = list(piece_poly.exterior.coords)
                else:
                    box_xy_raw = overlay.get("boxXY")
                    if not isinstance(box_xy_raw, list) or len(box_xy_raw) < 4:
                        lines.append(f"  UAV{aid}/P{piece_idx}: polygon/boxXY 없음, skip")
                        continue
                    ext_coords = [
                        (float(c[0]), float(c[1]))
                        for c in box_xy_raw
                        if isinstance(c, (tuple, list)) and len(c) >= 2
                    ]
                if len(ext_coords) < 3:
                    continue

                s_vals = [float(x) * ux + float(y) * uy for x, y in ext_coords]
                t_vals = [float(x) * vx + float(y) * vy for x, y in ext_coords]
                min_s, max_s = min(s_vals), max(s_vals)
                min_t, max_t = min(t_vals), max(t_vals)
                total_length = max_s - min_s
                total_width = max_t - min_t
                pad_t = total_width * 0.1

                # ---- sweep lines: line next-collab와 동일한 sep+FOV spacing ----
                sweep_step_m = float(
                    self._next_collab_area_sweep_spacing_m(
                        sep_ref_m,
                        resolved_fov_deg,
                        db_fov_deg=resolved_db_fov_deg,
                    ) or 0.0
                )
                if sweep_step_m <= 0.0:
                    lines.append(f"  UAV{aid}/P{piece_idx}: spacing 결정 불가, skip")
                    continue
                path_row["lineSweepSpacingM"] = float(sweep_step_m)
                path_row["areaDensitySpeedScale"] = float(self._next_collab_area_density_speed_scale())

                # take over list 시작인 경우: 첫번째 WP까지 2배 촘촘
                is_takeover = bool(path_row.get("entryTPrimeXY") is not None)
                dense_first_step = (
                    sweep_step_m * float(self._next_collab_takeover_first_step_ratio())
                    if is_takeover
                    else sweep_step_m
                )

                s_positions: List[float] = []
                if total_length <= sweep_step_m:
                    s_positions = [0.5 * (min_s + max_s)]
                else:
                    # 첫 구간: dense_first_step 간격으로 촘촘하게
                    cursor = min_s + dense_first_step * 0.5
                    first_boundary = min_s + sweep_step_m
                    while cursor < first_boundary and cursor < max_s:
                        s_positions.append(cursor)
                        cursor += dense_first_step
                    # 나머지 구간: 일반 간격
                    cursor = first_boundary + sweep_step_m * 0.5
                    while cursor < max_s:
                        s_positions.append(cursor)
                        cursor += sweep_step_m
                n_lines = len(s_positions)

                sweep_lines_xy: List[List[Tuple[float, float]]] = []
                for line_idx, s_val in enumerate(s_positions):
                    # extended line across the width (t-axis direction)
                    p0 = (
                        s_val * ux + (min_t - pad_t) * vx,
                        s_val * uy + (min_t - pad_t) * vy,
                    )
                    p1 = (
                        s_val * ux + (max_t + pad_t) * vx,
                        s_val * uy + (max_t + pad_t) * vy,
                    )

                    if piece_poly is not None and not piece_poly.is_empty:
                        # clip to actual polygon
                        probe = LineString([p0, p1])
                        clipped = piece_poly.intersection(probe)
                        segs: List[LineString] = []
                        if isinstance(clipped, LineString) and not clipped.is_empty:
                            segs.append(clipped)
                        elif hasattr(clipped, "geoms"):
                            segs.extend(
                                g for g in clipped.geoms
                                if isinstance(g, LineString) and not g.is_empty
                            )
                        for seg in segs:
                            seg_coords = list(seg.coords)
                            if len(seg_coords) < 2:
                                continue
                            pts = [(float(c[0]), float(c[1])) for c in seg_coords]
                            if line_idx % 2 != 0:
                                pts = list(reversed(pts))
                            sweep_lines_xy.append(pts)
                    else:
                        # fallback: full extent
                        if line_idx % 2 == 0:
                            sweep_lines_xy.append([p0, p1])
                        else:
                            sweep_lines_xy.append([p1, p0])

                path_row["sweepLineListXY"] = sweep_lines_xy
                path_row["sweepFootM"] = float(foot_m)
                path_row["sweepLineCount"] = int(n_lines)

                lines.append(
                    f"  UAV{aid}/P{piece_idx}: foot={foot_m:.1f}m (step={sweep_step_m:.1f}m, line-rule)"
                    f" | L={total_length:.0f}m → {n_lines} sweep lines"
                    f" | W={total_width:.0f}m (clipped={'Y' if piece_poly else 'N'})"
                    f" | spacing=sep+FOV"
                )

            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make Sweep", str(exc))
            self._refresh_ui()

    def _make_path_2(self) -> None:
        source_rows = [
            dict(row)
            for row in self.state.assignment_path_rows
            if isinstance(row, dict) and str(row.get("source", "") or "") == "next_mission"
        ]
        if not source_rows:
            QMessageBox.warning(self, "Make Path - 2", "癒쇱? Assignment - Next Mission???ㅽ뻾??二쇱꽭??")
            return

        try:
            keep_rows = [
                self._wp_only_display_row(
                    self._clone_path_row_with_waypoint_end_label(dict(row), "MP_E_1")
                )
                for row in self.state.expected_paths
                if isinstance(row, dict) and str(row.get("source", "") or "") != "next_mission"
            ]

            path2_rows: List[Dict[str, Any]] = []
            lines = ["[WP2] make path 2 from T2 + second ingress length"]
            for base_row in sorted(
                source_rows,
                key=lambda row: (
                    int(row.get("aircraftID", 0) or 0),
                    int(row.get("pieceIndex", 0) or 0),
                ),
            ):
                path2_row = self._build_make_waypoint_row(
                    base_row,
                    source="make_path_2",
                    waypoint_end_label="MP_E_2",
                )
                if path2_row is None:
                    lines.append(
                        f"  UAV{int(base_row.get('aircraftID', 0) or 0)}"
                        f" -> P{int(base_row.get('pieceIndex', 0) or 0)}: path2 build failed"
                    )
                    continue

                path2_rows.append(self._wp_only_display_row(path2_row))
                lines.append(
                    f"  UAV{int(path2_row.get('aircraftID', 0) or 0)}"
                    f" -> P{int(path2_row.get('pieceIndex', 0) or 0)} / {str(path2_row.get('targetLabel', '') or '?')}"
                    f" | start {str(path2_row.get('startLabel', 'T2') or 'T2')}"
                    f" | end {str(path2_row.get('waypointEndLabel', 'MP_E_2') or 'MP_E_2')}"
                    f" | shapeLen {float(path2_row.get('shapeLengthM', 0.0) or 0.0):.0f}m"
                    f" | FOV {float(path2_row.get('resolvedFovDeg', 0.0) or 0.0):.1f}"
                    f" | VEL {float(path2_row.get('resolvedVelMps', 0.0) or 0.0):.0f}"
                )

            if not path2_rows:
                lines.append("  usable path2 row ?놁쓬")
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            self.state.expected_paths = keep_rows + path2_rows
            self.state.mid_line_segments = []
            self.state.next_mission_rows = []
            self.state.show_next_mission_circles = False
            self.state.visibility_segments = []
            self.state.tangent_checks = []
            self.state.show_turn_overlays = False
            self.state.mode = MODE_RESULT_READY
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make Path - 2", str(exc))
            self._refresh_ui()

    def _make_sweep_2(self) -> None:
        """Make Path - 2 이후: path1(T1→WP_E_1) + path2(T2→WP_E_2) 각각 sweep 생성."""
        sweep_targets = [
            row for row in self.state.expected_paths
            if isinstance(row, dict)
            and row.get("waypointStartXY") is not None
            and row.get("waypointEndXY") is not None
        ]
        if not sweep_targets:
            QMessageBox.warning(self, "Make Sweep 2", "먼저 Make Path - 2를 실행해 주세요.")
            return
        try:
            lines = ["[SWEEP2] Make Sweep 2"]
            db_rows = self._fov_db_rows()

            for path_row in sweep_targets:
                aid = int(path_row.get("aircraftID", 0) or 0)
                piece_idx = int(path_row.get("pieceIndex", 0) or 0)
                src = str(path_row.get("source", "") or "")
                wp_label = str(path_row.get("waypointEndLabel", "") or "")

                # ---- ingress direction: T/T' → WP_E ----
                wp_start_raw = path_row.get("waypointStartXY")
                wp_end_raw = path_row.get("waypointEndXY")
                if not (
                    isinstance(wp_start_raw, (tuple, list)) and len(wp_start_raw) >= 2
                    and isinstance(wp_end_raw, (tuple, list)) and len(wp_end_raw) >= 2
                ):
                    continue
                dx = float(wp_end_raw[0]) - float(wp_start_raw[0])
                dy = float(wp_end_raw[1]) - float(wp_start_raw[1])
                ingress_len = math.hypot(dx, dy)
                if ingress_len <= 1e-6:
                    continue
                ux = dx / ingress_len
                uy = dy / ingress_len
                vx = uy
                vy = -ux

                sep_cand_m = float(path_row.get("sepCandM", 0.0) or 0.0)
                sep_ref_m = float(path_row.get("gsdSepM", 0.0) or 0.0)
                if sep_ref_m <= 0.0:
                    sep_ref_m = float(path_row.get("dbSepM", 0.0) or 0.0)
                if sep_ref_m <= 0.0:
                    sep_ref_m = self._next_collab_db_sep_requirement_m(sep_cand_m)
                resolved_fov_deg = path_row.get("resolvedFovDeg")
                resolved_db_fov_deg = path_row.get("resolvedDbFovDeg")
                foot_m = float(
                    self._next_collab_area_spacing_footprint_m(
                        sep_ref_m,
                        resolved_fov_deg,
                        db_fov_deg=resolved_db_fov_deg,
                    ) or 0.0
                )
                if foot_m <= 0.0:
                    lines.append(f"  UAV{aid}/P{piece_idx}/{wp_label}: foot 결정 불가, skip")
                    continue

                # ---- build polygon for clipping from partPolygonXY ----
                piece_poly: Optional[Polygon] = None
                part_poly_raw = path_row.get("partPolygonXY")
                if isinstance(part_poly_raw, list) and len(part_poly_raw) >= 3:
                    poly_pts = [
                        (float(p[0]), float(p[1]))
                        for p in part_poly_raw
                        if isinstance(p, (tuple, list)) and len(p) >= 2
                    ]
                    if len(poly_pts) >= 3:
                        piece_poly = Polygon(poly_pts)
                        if not piece_poly.is_valid:
                            piece_poly = piece_poly.buffer(0)
                        if piece_poly.is_empty:
                            piece_poly = None
                        elif piece_poly.geom_type == "MultiPolygon":
                            polys = [
                                g for g in piece_poly.geoms
                                if isinstance(g, Polygon) and not g.is_empty
                            ]
                            piece_poly = max(polys, key=lambda g: g.area) if polys else None

                # ---- s-t extents ----
                if piece_poly is not None and not piece_poly.is_empty:
                    ext_coords = list(piece_poly.exterior.coords)
                else:
                    ext_coords = [
                        (float(wp_start_raw[0]), float(wp_start_raw[1])),
                        (float(wp_end_raw[0]), float(wp_end_raw[1])),
                    ]
                if len(ext_coords) < 2:
                    continue

                s_vals = [float(x) * ux + float(y) * uy for x, y in ext_coords]
                t_vals = [float(x) * vx + float(y) * vy for x, y in ext_coords]
                min_s, max_s = min(s_vals), max(s_vals)
                min_t, max_t = min(t_vals), max(t_vals)
                total_length = max_s - min_s
                total_width = max_t - min_t
                pad_t = total_width * 0.1

                # ---- sweep lines: line next-collab와 동일한 sep+FOV spacing ----
                sweep_step_m = float(
                    self._next_collab_area_sweep_spacing_m(
                        sep_ref_m,
                        resolved_fov_deg,
                        db_fov_deg=resolved_db_fov_deg,
                    ) or 0.0
                )
                if sweep_step_m <= 0.0:
                    lines.append(f"  UAV{aid}/P{piece_idx}/{wp_label}: spacing 결정 불가, skip")
                    continue
                path_row["lineSweepSpacingM"] = float(sweep_step_m)
                path_row["areaDensitySpeedScale"] = float(self._next_collab_area_density_speed_scale())

                # take over list 시작인 경우: 첫번째 WP까지 2배 촘촘
                is_takeover = bool(path_row.get("entryTPrimeXY") is not None)
                dense_first_step = (
                    sweep_step_m * float(self._next_collab_takeover_first_step_ratio())
                    if is_takeover
                    else sweep_step_m
                )

                s_positions: List[float] = []
                if total_length <= sweep_step_m:
                    s_positions = [0.5 * (min_s + max_s)]
                else:
                    cursor = min_s + dense_first_step * 0.5
                    first_boundary = min_s + sweep_step_m
                    while cursor < first_boundary and cursor < max_s:
                        s_positions.append(cursor)
                        cursor += dense_first_step
                    cursor = first_boundary + sweep_step_m * 0.5
                    while cursor < max_s:
                        s_positions.append(cursor)
                        cursor += sweep_step_m
                n_lines = len(s_positions)

                sweep_lines_xy: List[List[Tuple[float, float]]] = []
                for line_idx, s_val in enumerate(s_positions):
                    p0 = (
                        s_val * ux + (min_t - pad_t) * vx,
                        s_val * uy + (min_t - pad_t) * vy,
                    )
                    p1 = (
                        s_val * ux + (max_t + pad_t) * vx,
                        s_val * uy + (max_t + pad_t) * vy,
                    )
                    if piece_poly is not None and not piece_poly.is_empty:
                        probe = LineString([p0, p1])
                        clipped = piece_poly.intersection(probe)
                        segs: List[LineString] = []
                        if isinstance(clipped, LineString) and not clipped.is_empty:
                            segs.append(clipped)
                        elif hasattr(clipped, "geoms"):
                            segs.extend(
                                g for g in clipped.geoms
                                if isinstance(g, LineString) and not g.is_empty
                            )
                        for seg in segs:
                            seg_coords = list(seg.coords)
                            if len(seg_coords) < 2:
                                continue
                            pts = [(float(c[0]), float(c[1])) for c in seg_coords]
                            if line_idx % 2 != 0:
                                pts = list(reversed(pts))
                            sweep_lines_xy.append(pts)
                    else:
                        if line_idx % 2 == 0:
                            sweep_lines_xy.append([p0, p1])
                        else:
                            sweep_lines_xy.append([p1, p0])

                path_row["sweepLineListXY"] = sweep_lines_xy
                path_row["sweepFootM"] = float(foot_m)
                path_row["sweepLineCount"] = int(n_lines)

                start_label = str(path_row.get("startLabel", "T") or "T")
                lines.append(
                    f"  UAV{aid}/P{piece_idx} {start_label}→{wp_label}:"
                    f" foot={foot_m:.1f}m (step={sweep_step_m:.1f}m, line-rule)"
                    f" | L={total_length:.0f}m → {n_lines} sweep lines"
                    f" | clipped={'Y' if piece_poly else 'N'} | spacing=sep+FOV"
                )

            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make Sweep 2", str(exc))
            self._refresh_ui()

    def _run_stage2_area_division(self) -> None:
        if not self._ensure_ready_for_division(area_only=True):
            return
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Stage 2 Area Division", "먼저 Area Division Run을 실행해 주세요.")
            return

        try:
            mission_poly = self._mission_polygon_xy()
            if mission_poly is None:
                raise ValueError("Stage 2 review requires a valid area mission polygon.")
            grid_size_m = float(self._stage2_grid_size_m(mission_poly))

            area_piece_lists: Dict[int, List[SplitPiece]] = {}
            for piece in self.state.split_result.pieces:
                if int(piece.mission_type) not in {2, 3, 6}:
                    continue
                aid = int(piece.assigned_uav or 0)
                if aid <= 0:
                    continue
                area_piece_lists.setdefault(int(aid), []).append(piece)

            if not area_piece_lists:
                raise ValueError("Stage 2 review requires assigned area pieces.")

            multi_piece_uavs = [aid for aid, rows in sorted(area_piece_lists.items()) if len(rows) != 1]
            if multi_piece_uavs:
                raise ValueError(
                    "Stage 2 review currently supports one assigned area per UAV. "
                    f"Multiple pieces found for: {', '.join(f'UAV{aid}' for aid in multi_piece_uavs)}"
                )

            active_uav_ids = sorted(int(aid) for aid in area_piece_lists.keys())
            piece_by_aid = {int(aid): rows[0] for aid, rows in area_piece_lists.items()}
            ratio_map = self._stage2_ratio_map(active_uav_ids)
            if sum(float(v) for v in ratio_map.values()) <= 1e-9:
                raise ValueError("At least one Stage 2 ratio must be greater than 0.")

            target_area_map, stats_by_aid = self._stage2_target_area_map(
                self.state.split_result,
                piece_by_aid,
                ratio_map,
                grid_size_m=grid_size_m,
            )
            polygon_by_aid, achieved_area_map = self._stage2_rebalance_area_polygons(
                mission_poly,
                piece_by_aid,
                target_area_map,
                grid_size_m=grid_size_m,
            )
            self._stage2_apply_review_polygons(
                piece_by_aid,
                polygon_by_aid,
                ratio_map,
                target_area_map,
                grid_size_m=grid_size_m,
            )

            self.state.visibility_segments = []
            self.state.expected_paths = []
            self.state.assignment_path_rows = []
            self.state.mission_check_rows = []
            self.state.flight_plans_0303 = []
            self.state.flight_plans_0304 = []
            self.state.show_turn_overlays = True
            self.state.mid_line_segments = []
            self.state.tangent_checks = []
            self.state.mode = MODE_RESULT_READY

            total_area_m2 = sum(float(v) for v in achieved_area_map.values())
            ratio_text = ", ".join(f"UAV{aid}={float(ratio_map.get(aid, 0.0)):.2f}" for aid in active_uav_ids)
            lines = [
                "[1b] stage2-area-review "
                f"grid={grid_size_m:.0f}m "
                f"ratios=({ratio_text}) "
                f"assignment={self._assignment_summary_text(self.state.split_result)}"
            ]
            for aid in active_uav_ids:
                meta = stats_by_aid[int(aid)]
                achieved_area_m2 = float(achieved_area_map.get(int(aid), 0.0))
                area_rate_m2ps = max(float(meta.get("areaRateM2ps", 0.0) or 0.0), 1e-6)
                est_total_sec = float(meta.get("entrySec", 0.0) or 0.0) + (achieved_area_m2 / area_rate_m2ps)
                share_pct = (100.0 * achieved_area_m2 / total_area_m2) if total_area_m2 > 1e-6 else 0.0
                lines.append(
                    f"  UAV{aid}: "
                    f"time {float(meta.get('currentTotalSec', 0.0)):.1f}s -> "
                    f"target {float(meta.get('desiredTotalSec', 0.0)):.1f}s -> "
                    f"est {est_total_sec:.1f}s | "
                    f"area {float(meta.get('currentAreaM2', 0.0)):.0f} -> {achieved_area_m2:.0f} m2 "
                    f"(target {float(target_area_map.get(int(aid), 0.0)):.0f}, share {share_pct:.1f}%)"
                )
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Stage 2 Area Division", str(exc))
            self._refresh_ui()

    def _check_visibility(self) -> None:
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Visibility", "먼저 Area Division Run을 실행해 주세요.")
            return

        max_steps = max(
            1,
            int(
                math.ceil(
                    (2.0 * math.pi * self._default_turn_radius_m())
                    / (TURN_PREVIEW_SPEED_MPS * TURN_PREVIEW_HORIZON_S)
                )
            ),
        )
        blocked_limit_s = int(max_steps * TURN_PREVIEW_HORIZON_S)

        visibility_segments: List[Dict[str, Any]] = []
        lines = [
            "[VIS] check visibility",
            f"rule: direct -> +{int(TURN_PREVIEW_HORIZON_S)}s -> +{int(TURN_PREVIEW_HORIZON_S * 2)}s ...",
        ]

        for piece in self.state.split_result.pieces:
            aid = int(piece.assigned_uav or 0)
            if aid <= 0:
                lines.append(f"  P{piece.piece_index}: assigned UAV 없음")
                continue

            uav_state = self._uav_state_for_aircraft(aid)
            if uav_state is None:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: UAV 위치 또는 heading 없음")
                continue

            target_xy = self._piece_assignment_target_xy(piece)
            if target_xy is None:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: centroid 계산 실패")
                continue

            origin_xy, heading_deg = uav_state
            segment = self._find_visibility_segment(aid, origin_xy, heading_deg, target_xy)
            if segment is None:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: +{blocked_limit_s}s까지 직선 연결 불가")
                continue

            visibility_segments.append(segment)
            horizon_s = float(segment.get("horizonSec", 0.0) or 0.0)
            branch = str(segment.get("branch", "") or "")
            if horizon_s <= 0.0:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: direct")
            else:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: +{horizon_s:.1f}s {branch}")

        self.state.visibility_segments = visibility_segments
        self.state.show_turn_overlays = False
        self._append_result("\n".join(lines))
        self._refresh_ui()

    def _run_planning(self) -> None:
        if self.state.uav_ids and not self._uav_inputs_complete():
            QMessageBox.warning(self, "UAV 입력 부족", "모든 UAV의 위치와 heading을 입력해 주세요.")
            return
        if not self.state.mission_points_xy or self.state.mission_kind is None:
            QMessageBox.warning(self, "임무 없음", "임무 형상을 먼저 입력해 주세요.")
            return
        if not self.state.uav_ids:
            QMessageBox.warning(self, "UAV 없음", "UAV 대수를 먼저 확정해 주세요.")
            return
        if len(self.state.uav_positions_xy) != len(self.state.uav_ids):
            QMessageBox.warning(self, "UAV 위치 부족", "모든 UAV 위치를 입력해 주세요.")
            return

        try:
            use_replan_flow = self._is_replan_flow()
            reuse_stage2_review = self.state.mission_kind == MISSION_AREA and self._has_stage2_review(self.state.split_result)
            if reuse_stage2_review:
                self._cmpk_payload = self._build_cmpk_payload()
                self._mrpk_payload = self._build_mrpk_payload()
                split_result = copy.deepcopy(self.state.split_result)
                stage_lines = [
                    f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}",
                    f"[1b] stage2-reviewed split reused pieces={len(split_result.pieces)}",
                ]
            else:
                split_result, stage_lines = self._run_split_stage()

            type_report = apply_logic_type_decider(split_result, self._cmpk_payload, profile_code=PROFILE_DEFAULT)
            stage_lines.append(
                "[2] type-decider "
                f"changed={int(type_report.get('changedPieces', 0))}/{int(type_report.get('pieceCount', 0))}"
            )

            expected_paths = generate_expected_paths(split_result, self._mrpk_payload)
            split_result.expected_paths = list(expected_paths)
            stage_lines.append(f"[3] expected-path count={len(expected_paths)}")

            review_report: Optional[Dict[str, Any]] = None
            if reuse_stage2_review:
                split_result.expected_paths = [
                    row
                    for row in expected_paths
                    if isinstance(row, dict) and str(row.get("source", "")).startswith("line_center_offset_dir")
                ]
                stage_lines.append("[4] area-review skipped (stage2 reviewed)")
            elif self._area_mode() not in {"nadir", "directdown", "bf_nadir"}:
                if use_replan_flow:
                    assign_report = assign_split_result_by_takeover_distance(
                        split_result,
                        self._mrpk_payload,
                        list(self.state.uav_ids),
                    )
                    stage_lines.append(
                        "[3a] replan-preassign "
                        f"assigned={int(assign_report.get('assignedPieces', 0))}/"
                        f"{int(assign_report.get('pieceCount', 0))} "
                        f"{self._assignment_summary_text(split_result)}"
                    )
                    review_report = review_assigned_areas_local(
                        split_result,
                        self._mrpk_payload,
                        max_segment_m=self._review_max_segment_m(),
                    )
                else:
                    review_report = review_overflow_areas(
                        split_result,
                        expected_paths,
                        max_segment_m=self._review_max_segment_m(),
                    )
                line_paths = [
                    row
                    for row in expected_paths
                    if isinstance(row, dict) and str(row.get("source", "")).startswith("line_center_offset_dir")
                ]
                split_result.expected_paths = line_paths
                if use_replan_flow:
                    stage_lines.append(
                        "[4] replan-review "
                        f"localized={int(review_report.get('localized', 0))} "
                        f"targets={int(review_report.get('targets', 0))} "
                        f"pieces={int(review_report.get('oldPieceCount', len(split_result.pieces)))}->"
                        f"{int(review_report.get('newPieceCount', len(split_result.pieces)))}"
                    )
                else:
                    stage_lines.append(
                        "[4] area-review "
                        f"overflow={int(review_report.get('overflowRows', 0))} "
                        f"targets={int(review_report.get('targets', 0))} "
                        f"pieces={int(review_report.get('oldPieceCount', len(split_result.pieces)))}->"
                        f"{int(review_report.get('newPieceCount', len(split_result.pieces)))}"
                    )
            else:
                split_result.expected_paths = [
                    row
                    for row in expected_paths
                    if isinstance(row, dict) and str(row.get("source", "")).startswith("line_center_offset_dir")
                ]
                stage_lines.append("[4] area-review skipped (nadir mode)")

            vel_report = calculate_expected_velocity(
                split_result,
                expected_paths=split_result.expected_paths,
            )
            stage_lines.append(
                "[5] expected-velocity "
                f"pieces={int(vel_report.get('pieceCount', 0))} dbRows={int(vel_report.get('dbRowCount', 0))}"
            )

            sched_report = run_milp_scheduling(
                split_result,
                mrpk=self._mrpk_payload,
                uav_ids_override=list(self.state.uav_ids),
                respect_piece_assignment=use_replan_flow,
            )
            split_result.schedule_result = sched_report
            stage_lines.append(
                "[6] scheduling "
                f"status={str(sched_report.get('status', ''))} "
                f"gap={float(sched_report.get('balanceGapSec', 0.0) or 0.0):.1f}s "
                f"inserted={int(sched_report.get('insertedMissionCount', 0))}"
            )

            packages_0302 = build_0302_packages_from_split_with_lah(split_result, cmpk=self._cmpk_payload)
            _prepare_legacy_missionplanner_path()
            uav_plan_mode = self._uav_plan_mode()
            fp_0303, fp_0304 = build_0303_0304_from_0302_packages(
                packages_0302,
                mrpk=self._mrpk_payload,
                uav_plan_mode=uav_plan_mode,
            )

            out_root = self._output_root()
            out_0302 = out_root / "auto_0302"
            out_0303 = out_root / "auto_0303"
            out_0304 = out_root / "auto_0304"
            paths_0302 = save_0302_packages(packages_0302, out_0302)
            paths_0303 = save_0303_plans(fp_0303, out_0303)
            paths_0304 = save_0304_plans(fp_0304, out_0304)
            stage_lines.append(
                f"[7] 0302 files={len(paths_0302)} | 0303 files={len(paths_0303)} | 0304 files={len(paths_0304)}"
            )

            self.state.split_result = split_result
            self.state.expected_paths = list(split_result.expected_paths)
            self.state.flight_plans_0303 = list(fp_0303)
            self.state.flight_plans_0304 = list(fp_0304)
            self.state.mode = MODE_RESULT_READY

            lines = [f"Mission: {self.state.mission_kind}", f"UAV: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}"]
            lines.extend(stage_lines)
            lines.extend(
                [
                    "",
                    f"Final split pieces: {len(split_result.pieces)}",
                    f"Cached expected paths: {len(self.state.expected_paths)}",
                    f"Actual 0303 paths: {len(fp_0303)}",
                    f"UAV plan mode: {uav_plan_mode}",
                ]
            )
            for piece in split_result.pieces:
                lines.append(
                    f"  P{piece.piece_index}: UAV{int(piece.assigned_uav or 0)} / "
                    f"type {piece.mission_type} / vertices {len((piece.data or {}).get('coordinateList', []))}"
                )
            lines.extend(
                [
                    "",
                    f"0302 output: {out_0302}",
                    f"0303 output: {out_0303}",
                    f"0304 output: {out_0304}",
                ]
            )
            self._set_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._clear_plan_result()
            self.state.mode = MODE_MISSION_READY
            self._set_result(traceback.format_exc())
            QMessageBox.critical(self, "계획 실행 실패", str(exc))
            self._refresh_ui()

    def _build_cmpk_payload(self) -> Dict[str, Any]:
        if self.state.mission_kind == MISSION_AREA:
            mission = {
                "inputMissionID": 1,
                "inputMissionType": 2,
                "missionDetail": {
                    "areaList": [
                        {
                            "isHole": False,
                            "coordinateList": [meters_to_coord(x, y) for (x, y) in self.state.mission_points_xy],
                        }
                    ]
                },
            }
        else:
            line_coords = [meters_to_coord(x, y) for (x, y) in self.state.mission_points_xy]
            mission = {
                "inputMissionID": 1,
                "inputMissionType": 1,
                "missionDetail": {
                    "coordinateList": list(line_coords),
                    "lineList": [
                        {
                            "width": float(self.state.line_width_m),
                            "coordinateList": list(line_coords),
                        }
                    ],
                },
            }

        return {
            "availableAircraftList": [{"aircraftID": int(aid)} for aid in self.state.uav_ids],
            "inputMissionList": [mission],
        }

    def _build_mrpk_payload(self) -> Dict[str, Any]:
        take_over_info_list: List[Dict[str, Any]] = []
        for idx, (aid, point_xy) in enumerate(zip(self.state.uav_ids, self.state.uav_positions_xy)):
            row = {
                "aircraftID": int(aid),
                "coordinate": meters_to_coord(point_xy[0], point_xy[1]),
            }
            heading = self.state.uav_heading_deg[idx] if idx < len(self.state.uav_heading_deg) else None
            if heading is not None:
                row["headingDeg"] = float(heading)
            take_over_info_list.append(row)
        return {
            "takeOverInfoList": take_over_info_list
        }
