from __future__ import annotations

import traceback
from copy import deepcopy
from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QRadioButton,
    QSplitter,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..algo.split_runner import run_split_pipeline
from ..algo.area_review import review_overflow_areas
from ..assignment.allocator import resolve_uav_ids
from ..io.export_0301 import build_0301_from_0302_packages, save_0301_plan
from ..io.export_0302 import build_0302_packages_from_split_with_lah, save_0302_packages
from ..io.export_0303_0304 import (
    build_0303_0304_from_0302_packages,
    save_0303_plans,
    save_0304_plans,
)
from ..io.export_internal_icd import build_internal_icd_payload, save_internal_icd_payload
from ..io.mission_loader import load_0201, load_0203
from .widgets import ScheduleGanttWidget
from ..map.map_canvas import MissionMapCanvas
from ..models import SplitRunResult
from ..pathing import calculate_expected_velocity, generate_expected_paths
from ..scheduling import (
    run_milp_scheduling,
)
from ..type_decider import apply_logic_type_decider
try:
    from ...runtime_settings import get_runtime_float, get_runtime_str
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float, get_runtime_str  # type: ignore
from modules.mission_planning.ui import MissionAlgoConfigTab


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        if QApplication.instance() is None:
            raise RuntimeError("QApplication must be created before MainWindow.")
        super().__init__()
        self.setWindowTitle("0201/0203 Mission Split Tester")
        self.resize(1600, 920)

        self.cmpk_path: Optional[Path] = None
        self.mrpk_path: Optional[Path] = None
        self.cmpk_data: Optional[dict] = None
        self.mrpk_data: Optional[dict] = None
        self.split_result: Optional[SplitRunResult] = None
        self.map_view: Optional[MissionMapCanvas] = None
        self.gantt_view: Optional[ScheduleGanttWidget] = None
        self._base_schedule_for_insert: Optional[dict] = None
        self.flight_plans_0303: list[dict] = []
        self.flight_plans_0304: list[dict] = []

        self._build_ui()
        self._render_map()

    def _review_max_segment_m(self) -> float:
        return float(get_runtime_float("enhanced_area_review_max_segment_m", 550.0))

    def _build_ui(self) -> None:
        root = QWidget()
        layout = QHBoxLayout(root)
        self.setCentralWidget(root)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        layout.addWidget(left, 1)

        self.btn_load_0201 = QPushButton("Load 0201 (CMPK)")
        self.btn_load_0201.clicked.connect(self._load_0201)
        left_layout.addWidget(self.btn_load_0201)
        self.lbl_0201 = QLabel("not loaded")
        left_layout.addWidget(self.lbl_0201)

        self.btn_load_0203 = QPushButton("Load 0203 (MRPK)")
        self.btn_load_0203.clicked.connect(self._load_0203)
        left_layout.addWidget(self.btn_load_0203)
        self.lbl_0203 = QLabel("not loaded")
        left_layout.addWidget(self.lbl_0203)

        self.btn_algo_config = QPushButton("Settings")
        self.btn_algo_config.clicked.connect(self._open_algo_config_dialog)
        left_layout.addWidget(self.btn_algo_config)

        self.chk_auto_uav = QCheckBox("UAV count from 0201 (auto)")
        self.chk_auto_uav.setChecked(True)
        self.chk_auto_uav.toggled.connect(self._on_auto_uav_toggled)
        left_layout.addWidget(self.chk_auto_uav)

        self.spin_uav = QSpinBox()
        self.spin_uav.setRange(1, 20)
        self.spin_uav.setValue(3)
        self.spin_uav.setEnabled(False)
        left_layout.addWidget(self.spin_uav)

        self.btn_run_assignment = QPushButton("Run AnS")
        self.btn_run_assignment.clicked.connect(self._run_assignment_pipeline)
        left_layout.addWidget(self.btn_run_assignment)

        self.btn_divider = QPushButton("1) Run div")
        self.btn_divider.clicked.connect(self._run_divider)
        left_layout.addWidget(self.btn_divider)

        self.btn_gen_exp_path = QPushButton("2) Gen exp path")
        self.btn_gen_exp_path.clicked.connect(self._run_gen_exp_path)
        left_layout.addWidget(self.btn_gen_exp_path)

        type_row = QWidget()
        type_row_layout = QHBoxLayout(type_row)
        type_row_layout.setContentsMargins(0, 0, 0, 0)
        type_row_layout.setSpacing(6)

        self.btn_run_type_decider = QPushButton("3) Run Type Decider")
        self.btn_run_type_decider.clicked.connect(self._run_type_decider)
        type_row_layout.addWidget(self.btn_run_type_decider, 1)

        self.rd_type_logic = QRadioButton("Logic")
        self.rd_type_rl = QRadioButton("RL")
        self.rd_type_logic.setChecked(True)
        self.type_mode_group = QButtonGroup(self)
        self.type_mode_group.addButton(self.rd_type_logic)
        self.type_mode_group.addButton(self.rd_type_rl)
        type_row_layout.addWidget(self.rd_type_logic)
        type_row_layout.addWidget(self.rd_type_rl)
        left_layout.addWidget(type_row)

        profile_row = QWidget()
        profile_row_layout = QHBoxLayout(profile_row)
        profile_row_layout.setContentsMargins(0, 0, 0, 0)
        profile_row_layout.setSpacing(6)
        profile_row_layout.addWidget(QLabel("Profile:"))
        self.rd_profile_default = QRadioButton("Default(6)")
        self.rd_profile_recon = QRadioButton("Recon(4)")
        self.rd_profile_min_time = QRadioButton("MinTime(5)")
        self.rd_profile_default.setChecked(True)
        self.profile_group = QButtonGroup(self)
        self.profile_group.addButton(self.rd_profile_default)
        self.profile_group.addButton(self.rd_profile_recon)
        self.profile_group.addButton(self.rd_profile_min_time)
        profile_row_layout.addWidget(self.rd_profile_default)
        profile_row_layout.addWidget(self.rd_profile_recon)
        profile_row_layout.addWidget(self.rd_profile_min_time)
        profile_row_layout.addStretch(1)
        left_layout.addWidget(profile_row)

        self.btn_review_area = QPushButton("4) Review Area")
        self.btn_review_area.clicked.connect(self._run_review_area)
        left_layout.addWidget(self.btn_review_area)

        self.btn_cal_exp_vel = QPushButton("5) Cal Exp Vel/Time")
        self.btn_cal_exp_vel.clicked.connect(self._run_cal_exp_vel)
        left_layout.addWidget(self.btn_cal_exp_vel)

        self.btn_scheduling = QPushButton("6) Scheduling")
        self.btn_scheduling.clicked.connect(self._run_scheduling)
        left_layout.addWidget(self.btn_scheduling)

        self.btn_build_0303_0304 = QPushButton("8) 0303,0304 planning")
        self.btn_build_0303_0304.clicked.connect(self._run_build_0303_0304)
        left_layout.addWidget(self.btn_build_0303_0304)

        self.btn_build_0301 = QPushButton("9) Build 0301")
        self.btn_build_0301.clicked.connect(self._run_build_0301)
        left_layout.addWidget(self.btn_build_0301)

        self.btn_clear = QPushButton("Clear Split Overlay")
        self.btn_clear.clicked.connect(self._clear_split)
        left_layout.addWidget(self.btn_clear)

        self.btn_export_0302 = QPushButton("Export 0302 JSON")
        self.btn_export_0302.clicked.connect(self._export_0302)
        left_layout.addWidget(self.btn_export_0302)

        self.btn_export_internal_icd = QPushButton("Export Internal ICD JSON")
        self.btn_export_internal_icd.clicked.connect(self._export_internal_icd)
        left_layout.addWidget(self.btn_export_internal_icd)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        left_layout.addWidget(self.log, 1)

        right_split = QSplitter(Qt.Vertical)
        map_panel = QWidget()
        map_panel_layout = QVBoxLayout(map_panel)
        map_panel_layout.setContentsMargins(0, 0, 0, 0)
        map_panel_layout.setSpacing(4)

        self.map_view = MissionMapCanvas()
        map_panel_layout.addWidget(self.map_view, 1)

        layer_row = QWidget()
        layer_layout = QHBoxLayout(layer_row)
        layer_layout.setContentsMargins(2, 0, 2, 2)
        layer_layout.setSpacing(8)
        layer_layout.addWidget(QLabel("Layer:"))
        self.chk_layer_0201 = QCheckBox("0201")
        self.chk_layer_0203 = QCheckBox("0203")
        self.chk_layer_split = QCheckBox("Split")
        self.chk_layer_dir = QCheckBox("Direction")
        self.chk_layer_exp = QCheckBox("ExpPath")
        self.chk_layer_0303_route = QCheckBox("0303 Route")
        self.chk_layer_0303_sweep = QCheckBox("0303 Sweep")
        self.chk_layer_0304 = QCheckBox("0304")
        self.chk_layer_path_unmanned = QCheckBox("UAV Path")
        self.chk_layer_path_manned = QCheckBox("Manned Path")
        for chk in (
            self.chk_layer_0201,
            self.chk_layer_0203,
            self.chk_layer_split,
            self.chk_layer_dir,
            self.chk_layer_exp,
            self.chk_layer_0303_route,
            self.chk_layer_0303_sweep,
            self.chk_layer_0304,
            self.chk_layer_path_unmanned,
            self.chk_layer_path_manned,
        ):
            chk.setChecked(True)
            chk.toggled.connect(lambda _checked: self._render_map())
            layer_layout.addWidget(chk)
        layer_layout.addStretch(1)
        map_panel_layout.addWidget(layer_row)

        self.gantt_view = ScheduleGanttWidget()
        right_split.addWidget(map_panel)
        right_split.addWidget(self.gantt_view)
        right_split.setStretchFactor(0, 4)
        right_split.setStretchFactor(1, 2)
        layout.addWidget(right_split, 2)

    def _on_auto_uav_toggled(self, checked: bool) -> None:
        self.spin_uav.setEnabled(not checked)

    def _current_layer_visibility(self) -> dict:
        def _ck(name: str, default: bool = True) -> bool:
            w = getattr(self, name, None)
            if isinstance(w, QCheckBox):
                return bool(w.isChecked())
            return bool(default)

        return {
            "show_0201": _ck("chk_layer_0201", True),
            "show_0203": _ck("chk_layer_0203", True),
            "show_split": _ck("chk_layer_split", True),
            "show_direction": _ck("chk_layer_dir", True),
            "show_expected_paths": _ck("chk_layer_exp", True),
            "show_0303_route": _ck("chk_layer_0303_route", True),
            "show_0303_sweep": _ck("chk_layer_0303_sweep", True),
            "show_0304": _ck("chk_layer_0304", True),
            "show_path_unmanned": _ck("chk_layer_path_unmanned", True),
            "show_path_manned": _ck("chk_layer_path_manned", True),
        }

    def _load_0201(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select 0201 JSON",
            str(Path.cwd()),
            "JSON files (*.json)",
        )
        if not path:
            return
        try:
            payload = load_0201(path)
        except Exception as exc:
            QMessageBox.critical(self, "Load 0201 Failed", str(exc))
            return

        self.cmpk_path = Path(path)
        self.cmpk_data = payload
        self.split_result = None
        self._base_schedule_for_insert = None
        self.flight_plans_0303 = []
        self.flight_plans_0304 = []
        self.lbl_0201.setText(str(self.cmpk_path))
        self._log(f"[0201] loaded: {self.cmpk_path}")
        self._render_map()

    def _load_0203(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select 0203 JSON",
            str(Path.cwd()),
            "JSON files (*.json)",
        )
        if not path:
            return
        try:
            payload = load_0203(path)
        except Exception as exc:
            QMessageBox.critical(self, "Load 0203 Failed", str(exc))
            return

        self.mrpk_path = Path(path)
        self.mrpk_data = payload
        self.split_result = None
        self._base_schedule_for_insert = None
        self.flight_plans_0303 = []
        self.flight_plans_0304 = []
        self.lbl_0203.setText(str(self.mrpk_path))
        self._log(f"[0203] loaded: {self.mrpk_path}")
        self._render_map()

    def _resolve_uav_ids_for_ui(self) -> list[int]:
        if self.cmpk_data is None:
            return [4]
        if self.chk_auto_uav.isChecked():
            uav_ids = resolve_uav_ids(self.cmpk_data, override_count=None)
        else:
            uav_ids = resolve_uav_ids(self.cmpk_data, override_count=int(self.spin_uav.value()))
        return uav_ids or [4]

    def _selected_profile_code(self) -> int:
        if getattr(self, "rd_profile_recon", None) is not None and self.rd_profile_recon.isChecked():
            return 4
        if getattr(self, "rd_profile_min_time", None) is not None and self.rd_profile_min_time.isChecked():
            return 5
        return 6

    def _run_divider(self) -> None:
        if self.cmpk_data is None:
            QMessageBox.warning(self, "Missing 0201", "Load 0201 first.")
            return
        if self.mrpk_data is None:
            QMessageBox.warning(self, "Missing 0203", "Load 0203 first.")
            return

        try:
            uav_ids = self._resolve_uav_ids_for_ui()
            result = run_split_pipeline(
                self.cmpk_data,
                self.mrpk_data,
                uav_ids,
                apply_assignment=False,
                apply_scheduling=False,
            )
            self.split_result = result
            self._base_schedule_for_insert = None
            self.flight_plans_0303 = []
            self.flight_plans_0304 = []

            self._log(f"[DIVIDER] divide complete. uav_ids={result.uav_ids}, pieces={len(result.pieces)}")
            for d in result.directions:
                if d.bearing_move_deg is not None and d.bearing_split_deg is not None:
                    area_tag = ""
                    if d.source_area_index is not None:
                        area_tag = f" A{int(d.source_area_index)}"
                    msg = (
                        f"[DIR] M{d.parent_order}{area_tag} ID={d.mission_id} "
                        f"move={d.bearing_move_deg:.2f} split={d.bearing_split_deg:.2f}"
                    )
                    if d.bearing_in_deg is not None:
                        msg += f" in={d.bearing_in_deg:.2f}"
                    if d.bearing_out_deg is not None:
                        msg += f" out={d.bearing_out_deg:.2f}"
                    self._log(msg)
            self._render_map()
        except Exception as exc:
            self._log("[ERROR] divide failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Divider Failed", str(exc))

    def _run_assignment(self) -> None:
        if self.split_result is None:
            QMessageBox.warning(self, "No Divide Result", "Run Divider first.")
            return
        assigned = [p for p in self.split_result.pieces if int(p.assigned_uav or 0) > 0]
        if not assigned:
            QMessageBox.warning(self, "No Assignment", "Run 6) Scheduling first.")
            return
        try:
            packages = build_0302_packages_from_split_with_lah(self.split_result, cmpk=self.cmpk_data)
            auto_dir = Path(__file__).resolve().parents[2] / "temp" / "auto_0302"
            paths = save_0302_packages(packages, auto_dir)
            self._log(f"[0302][AUTO] saved {len(paths)} package file(s): {auto_dir}")
            for p in paths:
                self._log(f"[0302][AUTO] {p}")
            QMessageBox.information(
                self,
                "0302 Auto Saved",
                f"Saved {len(paths)} file(s) to\n{auto_dir}",
            )
        except Exception as exc:
            self._log("[ERROR] 0302 auto save failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "0302 Auto Save Failed", str(exc))

    def _run_assignment_pipeline(self) -> None:
        if self.cmpk_data is None:
            QMessageBox.warning(self, "Missing 0201", "Load 0201 first.")
            return
        if self.mrpk_data is None:
            QMessageBox.warning(self, "Missing 0203", "Load 0203 first.")
            return

        try:
            self._log("[PIPE] start AssignmentPipeline (1~7)")

            # 1) Run div
            uav_ids = self._resolve_uav_ids_for_ui()
            result = run_split_pipeline(
                self.cmpk_data,
                self.mrpk_data,
                uav_ids,
                apply_assignment=False,
                apply_scheduling=False,
            )
            self.split_result = result
            self._base_schedule_for_insert = None
            self.flight_plans_0303 = []
            self.flight_plans_0304 = []
            self._log(f"[PIPE][1] divide complete. uav_ids={result.uav_ids}, pieces={len(result.pieces)}")
            QApplication.processEvents()

            # 2) Gen exp path
            expected_paths = generate_expected_paths(self.split_result, self.mrpk_data)
            self.split_result.expected_paths = expected_paths
            self._log_expected_paths(expected_paths, prefix="[PIPE][2][EXP]")
            QApplication.processEvents()

            # 3) Run Type Decider (Logic only in current implementation)
            mode = "Logic" if self.rd_type_logic.isChecked() else "RL"
            if mode != "Logic":
                self._log("[PIPE][3] RL not implemented. fallback to Logic.")
            profile_code = self._selected_profile_code()
            tr = apply_logic_type_decider(self.split_result, self.cmpk_data, profile_code=profile_code)
            self._log(
                "[PIPE][3] "
                f"type-decider done. packageType={int(tr.get('packageType', 0))} "
                f"profileCode={int(tr.get('profileCode', 0))} "
                f"changed={int(tr.get('changedPieces', 0))}/{int(tr.get('pieceCount', 0))}"
            )
            self.split_result.schedule_result = {}
            self._base_schedule_for_insert = None
            self.flight_plans_0303 = []
            self.flight_plans_0304 = []
            QApplication.processEvents()

            # 4) Review Area
            expected_paths = generate_expected_paths(self.split_result, self.mrpk_data)
            self.split_result.expected_paths = expected_paths
            rr = review_overflow_areas(
                self.split_result,
                expected_paths,
                max_segment_m=self._review_max_segment_m(),
            )
            line_paths = [r for r in expected_paths if str(r.get("source", "")).startswith("line_center_offset_dir")]
            self.split_result.expected_paths = line_paths
            self._log(
                "[PIPE][4] "
                f"maxSegment={self._review_max_segment_m():.1f}m "
                f"review-area done. overflow={int(rr.get('overflowRows', 0))} "
                f"targets={int(rr.get('targets', 0))} "
                f"pieces={int(rr.get('oldPieceCount', 0))}->{int(rr.get('newPieceCount', 0))}"
            )
            QApplication.processEvents()

            # 5) Cal Exp Vel/Time
            vr = calculate_expected_velocity(
                self.split_result,
                expected_paths=self.split_result.expected_paths,
            )
            rows = vr.get("rows", [])
            no_cand = 0
            if isinstance(rows, list):
                no_cand = sum(
                    1
                    for row in rows
                    if isinstance(row, dict) and int(row.get("candidateCount", 0) or 0) <= 0
                )
            self._log(
                "[PIPE][5] "
                f"vel/time done. pieces={int(vr.get('pieceCount', 0))} "
                f"dbRows={int(vr.get('dbRowCount', 0))} "
                f"noCandidate={int(no_cand)}"
            )
            self.split_result.schedule_result = {}
            self._base_schedule_for_insert = None
            self.flight_plans_0303 = []
            self.flight_plans_0304 = []
            QApplication.processEvents()

            # 6) Scheduling
            _ = calculate_expected_velocity(
                self.split_result,
                expected_paths=self.split_result.expected_paths,
            )
            sched = run_milp_scheduling(
                self.split_result,
                mrpk=self.mrpk_data,
                uav_ids_override=uav_ids,
            )
            self._base_schedule_for_insert = deepcopy(sched) if isinstance(sched, dict) else None
            self._log(
                "[PIPE][6] "
                f"sync={str(sched.get('syncMode', ''))} "
                f"scheduling done. solver={str(sched.get('solver', ''))} "
                f"status={str(sched.get('status', ''))} "
                f"gap={float(sched.get('balanceGapSec', 0.0) or 0.0):.1f}s"
            )
            ins_n = int(sched.get("insertedMissionCount", 0) or 0)
            if ins_n > 0:
                self._log(f"[PIPE][6] sync-hold inserted={ins_n}")
            QApplication.processEvents()

            # 7) Run Assignment (Auto 0302)
            packages = build_0302_packages_from_split_with_lah(self.split_result, cmpk=self.cmpk_data)
            auto_dir = Path(__file__).resolve().parents[2] / "temp" / "auto_0302"
            paths = save_0302_packages(packages, auto_dir)
            self._log(f"[PIPE][7] auto 0302 saved {len(paths)} file(s): {auto_dir}")
            for p in paths:
                self._log(f"[PIPE][7] {p}")

            self._render_map()
            self._log("[PIPE] done.")
            QMessageBox.information(
                self,
                "AssignmentPipeline Complete",
                f"Completed 1~7 and saved {len(paths)} file(s) to\n{auto_dir}",
            )
        except Exception as exc:
            self._log("[ERROR] assignment pipeline failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "AssignmentPipeline Failed", str(exc))

    def _run_build_0303_0304(self) -> None:
        if self.cmpk_data is None:
            QMessageBox.warning(self, "Missing 0201", "Load 0201 first.")
            return
        if self.mrpk_data is None:
            QMessageBox.warning(self, "Missing 0203", "Load 0203 first.")
            return
        if self.split_result is None:
            QMessageBox.warning(self, "No Split Result", "Run Divider first.")
            return

        assigned = [p for p in self.split_result.pieces if int(p.assigned_uav or 0) > 0]
        if not assigned:
            QMessageBox.warning(self, "No Assignment", "Run 6) Scheduling first.")
            return

        try:
            packages = build_0302_packages_from_split_with_lah(self.split_result, cmpk=self.cmpk_data)
            fp_0303, fp_0304 = build_0303_0304_from_0302_packages(
                packages,
                mrpk=self.mrpk_data,
            )

            out_0303 = Path(__file__).resolve().parents[2] / "temp" / "auto_0303"
            out_0304 = Path(__file__).resolve().parents[2] / "temp" / "auto_0304"
            paths_0303 = save_0303_plans(fp_0303, out_0303)
            paths_0304 = save_0304_plans(fp_0304, out_0304)

            self.flight_plans_0303 = list(fp_0303)
            self.flight_plans_0304 = list(fp_0304)

            self._log(
                "[0303/0304] "
                f"generated 0303={len(fp_0303)} 0304={len(fp_0304)}"
            )
            self._log(f"[0303] saved {len(paths_0303)} file(s): {out_0303}")
            for p in paths_0303[:10]:
                self._log(f"[0303] {p}")
            if len(paths_0303) > 10:
                self._log(f"[0303] ... and {len(paths_0303) - 10} more")

            self._log(f"[0304] saved {len(paths_0304)} file(s): {out_0304}")
            for p in paths_0304[:10]:
                self._log(f"[0304] {p}")
            if len(paths_0304) > 10:
                self._log(f"[0304] ... and {len(paths_0304) - 10} more")

            self._render_map()
            QMessageBox.information(
                self,
                "0303/0304 Saved",
                (
                    f"0303: {len(paths_0303)} file(s)\n{out_0303}\n\n"
                    f"0304: {len(paths_0304)} file(s)\n{out_0304}"
                ),
            )
        except Exception as exc:
            self._log("[ERROR] build 0303/0304 failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Build 0303/0304 Failed", str(exc))

    def _open_algo_config_dialog(self) -> None:
        dlg = QDialog(self)
        dlg.setWindowTitle("Mission Planner Settings")
        dlg.resize(560, 720)
        lay = QVBoxLayout(dlg)
        settings_path = Path(__file__).resolve().parents[2] / "uav_params.json"

        def _on_apply(payload):
            values = payload.get("values") if isinstance(payload, dict) else {}
            self._log(
                "[SETTINGS] updated "
                f"(preset={payload.get('preset_key')}, "
                f"algo={payload.get('algo_key')}, "
                f"sep={values.get('default_sweep_separation_m')}, "
                f"fov={values.get('fov_deg')}, "
                f"autoFovDb={values.get('enhanced_auto_fov_from_db')})"
            )

        lay.addWidget(MissionAlgoConfigTab(settings_path, on_apply=_on_apply, parent=dlg))
        dlg.exec_()

    def _run_build_0301(self) -> None:
        if self.cmpk_data is None:
            QMessageBox.warning(self, "Missing 0201", "Load 0201 first.")
            return
        if self.mrpk_data is None:
            QMessageBox.warning(self, "Missing 0203", "Load 0203 first.")
            return
        if self.split_result is None:
            QMessageBox.warning(self, "No Split Result", "Run Divider first.")
            return

        assigned = [p for p in self.split_result.pieces if int(p.assigned_uav or 0) > 0]
        if not assigned:
            QMessageBox.warning(self, "No Assignment", "Run 6) Scheduling first.")
            return

        try:
            packages = build_0302_packages_from_split_with_lah(self.split_result, cmpk=self.cmpk_data)
            plan = build_0301_from_0302_packages(
                packages,
                cmpk=self.cmpk_data,
                mrpk=self.mrpk_data,
                planning_time_ms=0.0,
            )
            out_dir = Path(__file__).resolve().parents[2] / "temp" / "auto_0301"
            path = save_0301_plan(plan, out_dir)
            self._log(f"[0301] saved: {path}")
            self._log(
                "[0301] "
                f"missionPlanID={int(plan.get('missionPlanID', 0))} "
                f"aircraft={len(plan.get('aircraftList', []))}"
            )
            QMessageBox.information(self, "0301 Saved", f"Saved 0301:\n{path}")
        except Exception as exc:
            self._log("[ERROR] build 0301 failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Build 0301 Failed", str(exc))

    def _log_expected_paths(self, paths: list[dict], prefix: str = "[EXP]") -> None:
        self._log(f"{prefix} generated expected path(s): {len(paths)}")
        for row in paths:
            parent = int(row.get("parentOrder", 0) or 0)
            idx = int(row.get("index", 0) or 0)
            src = str(row.get("source", ""))
            sep_m = float(row.get("sepM", 0.0) or 0.0)
            width_ref = float(row.get("widthRefM", 0.0) or 0.0)
            set_name = str(row.get("setName", f"E{parent}-{idx}"))
            point_count = int(row.get("pointCount", 0) or 0)
            self._log(
                f"{prefix} M{parent}-{idx} {src} width={width_ref:.2f}m sep={sep_m:.2f}m "
                f"set={set_name} points={point_count}"
            )

    def _run_gen_exp_path(self) -> None:
        if self.split_result is None:
            QMessageBox.warning(self, "No Divide Result", "Run Divider first.")
            return
        if self.mrpk_data is None:
            QMessageBox.warning(self, "Missing 0203", "Load 0203 first.")
            return
        try:
            expected_paths = generate_expected_paths(self.split_result, self.mrpk_data)
            self.split_result.expected_paths = expected_paths
            self._log_expected_paths(expected_paths, prefix="[EXP]")
            self._render_map()
        except Exception as exc:
            self._log("[ERROR] expected path generation failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Gen exp path Failed", str(exc))

    def _run_review_area(self) -> None:
        try:
            area_mode = str(get_runtime_str("area_sweep_mode", "parallel") or "parallel").strip().lower()
        except Exception:
            area_mode = "parallel"
        if area_mode in {"nadir", "directdown", "bf_nadir"}:
            self._log("[REVIEW] skipped: Nadir Mode does not use Review Area")
            return
        if self.split_result is None:
            QMessageBox.warning(self, "No Divide Result", "Run Divider first.")
            return
        if self.mrpk_data is None:
            QMessageBox.warning(self, "Missing 0203", "Load 0203 first.")
            return
        area_pieces = [p for p in self.split_result.pieces if int(p.mission_type) in (2, 3, 6)]
        if area_pieces:
            has_type_decider = all(
                isinstance(p.data, dict) and int((p.data or {}).get("patternType", 0) or 0) > 0
                for p in area_pieces
            )
            if not has_type_decider:
                QMessageBox.warning(
                    self,
                    "Type Decider Required",
                    "Run Type Decider first. Review Area uses patternType (patternType==6 only).",
                )
                return
        try:
            expected_paths = generate_expected_paths(self.split_result, self.mrpk_data)
            self.split_result.expected_paths = expected_paths
            self._log_expected_paths(expected_paths, prefix="[EXP]")
            overflow_rows = [
                r
                for r in expected_paths
                if str(r.get("source", "")).startswith("area_")
                and str(r.get("pathRole", "base")) == "base"
                and float(r.get("sepM", 0.0) or 0.0) <= 1e-6
            ]
            self._log(f"[REVIEW] overflow_rows={len(overflow_rows)}")

            report = review_overflow_areas(
                self.split_result,
                expected_paths,
                max_segment_m=self._review_max_segment_m(),
            )
            self._log(
                "[REVIEW] "
                f"maxSegment={self._review_max_segment_m():.1f}m, "
                f"overflow_rows={int(report.get('overflowRows', 0))}, "
                f"targets={int(report.get('targets', 0))}, "
                f"pieces={int(report.get('oldPieceCount', 0))}->{int(report.get('newPieceCount', 0))}"
            )
            for d in report.get("details", []):
                if not isinstance(d, dict) or not d.get("changed"):
                    continue
                self._log(
                    "[REVIEW] "
                    f"M{int(d.get('parentOrder', 0))}-{int(d.get('oldPieceIndex', d.get('pieceIndex', 0)))} "
                    f"S{int(d.get('splitStage', 0))} "
                    f"splitCount={int(d.get('splitCount', 1))} "
                    f"seg={float(d.get('segmentLenM', 0.0)):.2f}m "
                    f"axis={float(d.get('axisBearingDeg', 0.0)):.2f}"
                )
            skipped = [
                d
                for d in report.get("details", [])
                if isinstance(d, dict) and str(d.get("reason", "")) == "skip_patternType_not_6"
            ]
            if skipped:
                self._log(f"[REVIEW] skipped(non-pattern6)={len(skipped)}")

            # Do not regenerate area expected paths after review.
            # Keep only line expected paths from the original set.
            line_paths = [r for r in expected_paths if str(r.get("source", "")).startswith("line_center_offset_dir")]
            self.split_result.expected_paths = line_paths
            self.split_result.schedule_result = {}
            self._base_schedule_for_insert = None
            self.flight_plans_0303 = []
            self.flight_plans_0304 = []
            self._log(
                "[REVIEW] "
                f"area expected paths cleared, line expected paths kept: {len(line_paths)}"
            )
            self._render_map()
        except Exception as exc:
            self._log("[ERROR] review area failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Review Area Failed", str(exc))

    def _run_cal_exp_vel(self) -> None:
        if self.split_result is None:
            QMessageBox.warning(self, "No Divide Result", "Run Divider first.")
            return
        try:
            report = calculate_expected_velocity(
                self.split_result,
                # Use current path cache if available, but do not generate or mutate paths here.
                expected_paths=self.split_result.expected_paths,
            )
            self._log(
                "[VEL] "
                f"calculated. dbRows={int(report.get('dbRowCount', 0))} "
                f"dbMaxWidth={float(report.get('dbMaxWidthM', 0.0)):.2f}m "
                f"pieces={int(report.get('pieceCount', 0))}"
            )
            seen_area_groups: set[str] = set()
            for row in report.get("rows", []):
                if not isinstance(row, dict):
                    continue
                parent = int(row.get("parentOrder", 0) or 0)
                piece_idx = int(row.get("pieceIndex", 0) or 0)
                width_ref = float(row.get("widthRefM", 0.0) or 0.0)
                width_src = str(row.get("widthSource", ""))
                cand_n = int(row.get("candidateCount", 0) or 0)
                vel_approx = bool(row.get("velApprox", False))
                vel_source = str(row.get("velSource", ""))
                gkey = str(row.get("areaTimeGroupKey", ""))
                gcount = int(row.get("areaTimeGroupCount", 0) or 0)
                gleader = bool(row.get("areaTimeGroupLeader", False))
                if gkey and gcount > 1:
                    if not gleader:
                        continue
                    if gkey in seen_area_groups:
                        continue
                    seen_area_groups.add(gkey)
                if cand_n <= 0:
                    self._log(
                        "[VEL] "
                        f"M{parent}-{piece_idx} width={width_ref:.2f}m src={width_src} -> no candidate"
                    )
                    continue
                vmin = float(row.get("velMinKmh", 0.0) or 0.0)
                vmax = float(row.get("velMaxKmh", 0.0) or 0.0)
                tmin_s = row.get("timeMinSec")
                tmax_s = row.get("timeMaxSec")
                if gkey and gcount > 1:
                    tmin_s = row.get("groupTimeMinSec")
                    tmax_s = row.get("groupTimeMaxSec")
                if tmin_s is None or tmax_s is None:
                    tmin_m = row.get("timeMinMin")
                    tmax_m = row.get("timeMaxMin")
                    if tmin_m is not None and tmax_m is not None:
                        tmin_s = float(tmin_m) * 60.0
                        tmax_s = float(tmax_m) * 60.0
                msg = (
                    "[VEL] "
                    f"M{parent}-{piece_idx} width={width_ref:.2f}m src={width_src} "
                    f"vel={vmin:.0f}~{vmax:.0f}km/h"
                )
                if gkey and gcount > 1:
                    msg += f" group={gkey}"
                if vel_approx:
                    msg += f" approx({vel_source})"
                if tmin_s is not None and tmax_s is not None:
                    msg += f" time={float(tmin_s):.1f}~{float(tmax_s):.1f}s"
                self._log(msg)

            self.split_result.schedule_result = {}
            self._base_schedule_for_insert = None
            self.flight_plans_0303 = []
            self.flight_plans_0304 = []
            self._render_map()
        except Exception as exc:
            self._log("[ERROR] cal exp vel failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Cal Exp Vel Failed", str(exc))

    def _run_scheduling(self) -> None:
        if self.split_result is None:
            QMessageBox.warning(self, "No Divide Result", "Run Divider first.")
            return
        if self.mrpk_data is None:
            QMessageBox.warning(self, "Missing 0203", "Load 0203 first.")
            return
        try:
            # Ensure velocity/time candidates exist for all current pieces.
            report = calculate_expected_velocity(
                self.split_result,
                expected_paths=self.split_result.expected_paths,
            )
            self._log(
                "[SCH] "
                f"velocity cache ready. pieces={int(report.get('pieceCount', 0))} "
                f"dbRows={int(report.get('dbRowCount', 0))}"
            )
            uav_ids = self._resolve_uav_ids_for_ui()
            sched = run_milp_scheduling(
                self.split_result,
                mrpk=self.mrpk_data,
                uav_ids_override=uav_ids,
            )
            self._log(
                "[SCH] "
                f"sync={str(sched.get('syncMode', ''))} "
                f"done. solver={str(sched.get('solver', ''))} "
                f"status={str(sched.get('status', ''))} "
                f"slots={int(sched.get('slotCount', 0))} "
                f"solveMs={float(sched.get('solveMs', 0.0) or 0.0):.1f}"
            )
            gap_sec = float(sched.get("balanceGapSec", 0.0) or 0.0)
            self._log(f"[SCH] balance gap={gap_sec:.1f}s")

            timelines = sched.get("timelines", [])
            if isinstance(timelines, list):
                for row in timelines:
                    if not isinstance(row, dict):
                        continue
                    self._log(
                        "[SCH] "
                        f"UAV{int(row.get('uavID', 0))} "
                        f"total={float(row.get('totalSec', 0.0) or 0.0):.1f}s "
                        f"task={float(row.get('taskSec', 0.0) or 0.0):.1f}s "
                        f"move={float(row.get('moveSec', 0.0) or 0.0):.1f}s"
                    )
            ins_n = int(sched.get("insertedMissionCount", 0) or 0)
            if ins_n > 0:
                self._log(f"[SCH] sync-hold inserted={ins_n}")

            self._base_schedule_for_insert = deepcopy(sched) if isinstance(sched, dict) else None
            self.flight_plans_0303 = []
            self.flight_plans_0304 = []
            self._render_map()
        except Exception as exc:
            self._log("[ERROR] scheduling failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Scheduling Failed", str(exc))

    def _run_type_decider(self) -> None:
        if self.split_result is None:
            QMessageBox.warning(self, "No Divide Result", "Run Divider first.")
            return
        if self.cmpk_data is None:
            QMessageBox.warning(self, "Missing 0201", "Load 0201 first.")
            return
        mode = "Logic" if self.rd_type_logic.isChecked() else "RL"
        if mode != "Logic":
            self._log(f"[TYPE] type decider placeholder executed (mode={mode}, not implemented).")
            return
        try:
            profile_code = self._selected_profile_code()
            report = apply_logic_type_decider(
                self.split_result,
                self.cmpk_data,
                profile_code=profile_code,
            )
            self._log(
                "[TYPE] "
                f"logic applied. packageType={int(report.get('packageType', 0))} "
                f"profileCode={int(report.get('profileCode', 0))} "
                f"changed={int(report.get('changedPieces', 0))}/{int(report.get('pieceCount', 0))}"
            )
            pri = report.get("priorityAircraftIDs", [])
            if isinstance(pri, list) and pri:
                self._log("[TYPE] aircraft priority: " + ", ".join(str(int(x)) for x in pri))
            leader = int(report.get("leaderAircraftID", 0) or 0)
            if leader > 0:
                self._log(f"[TYPE] formation leader aircraftID={leader}")
            im_counts = report.get("imTypeCounts", {})
            if isinstance(im_counts, dict) and im_counts:
                self._log("[TYPE] individualMissionType counts: " + ", ".join(f"{k}:{v}" for k, v in im_counts.items()))
            pt_counts = report.get("patternTypeCounts", {})
            if isinstance(pt_counts, dict) and pt_counts:
                self._log("[TYPE] patternType counts: " + ", ".join(f"{k}:{v}" for k, v in pt_counts.items()))
            assignments = report.get("assignments", [])
            if isinstance(assignments, list):
                for row in assignments:
                    if not isinstance(row, dict):
                        continue
                    self._log(
                        "[TYPE] "
                        f"M{int(row.get('parentOrder', 0))}-{int(row.get('pieceIndex', 0))} "
                        f"type={int(row.get('individualMissionType', 0))} "
                        f"pattern={int(row.get('patternType', 0))} "
                        f"rule={str(row.get('rule', ''))}"
                    )
            warnings = report.get("warnings", [])
            if isinstance(warnings, list):
                for w in warnings[:10]:
                    self._log(f"[TYPE][WARN] {w}")
                if len(warnings) > 10:
                    self._log(f"[TYPE][WARN] ... and {len(warnings) - 10} more")
            self.split_result.schedule_result = {}
            self._base_schedule_for_insert = None
            self.flight_plans_0303 = []
            self.flight_plans_0304 = []
            self._render_map()
        except Exception as exc:
            self._log("[ERROR] type decider failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Type Decider Failed", str(exc))

    def _clear_split(self) -> None:
        self.split_result = None
        self._base_schedule_for_insert = None
        self.flight_plans_0303 = []
        self.flight_plans_0304 = []
        self._log("[DIVIDER] split overlay cleared.")
        self._render_map()

    def _export_0302(self) -> None:
        if self.split_result is None:
            QMessageBox.warning(self, "No Split Result", "Run Divider first, then 6) Scheduling.")
            return
        if any((p.assigned_uav or 0) <= 0 for p in self.split_result.pieces):
            QMessageBox.warning(self, "No Assignment", "Run 6) Scheduling first, then export 0302.")
            return

        out_dir = QFileDialog.getExistingDirectory(self, "Select output folder for 0302 JSON", str(Path.cwd()))
        if not out_dir:
            return

        try:
            packages = build_0302_packages_from_split_with_lah(self.split_result, cmpk=self.cmpk_data)
            paths = save_0302_packages(packages, out_dir)
            self._log(f"[0302] exported {len(paths)} package files:")
            for p in paths:
                self._log(f"  - {p}")
            QMessageBox.information(self, "Export Complete", f"Saved {len(paths)} file(s) to\n{out_dir}")
        except Exception as exc:
            self._log("[ERROR] export 0302 failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Export Failed", str(exc))

    def _export_internal_icd(self) -> None:
        if self.split_result is None:
            QMessageBox.warning(self, "No Split Result", "Run Divider first.")
            return

        default_name = "Assignment_InternalICD.json"
        if self.cmpk_path is not None:
            default_name = f"{self.cmpk_path.stem}_InternalICD.json"
        default_path = str(Path.cwd() / default_name)
        out_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Internal ICD JSON",
            default_path,
            "JSON files (*.json)",
        )
        if not out_path:
            return

        try:
            payload = build_internal_icd_payload(
                self.split_result,
                cmpk=self.cmpk_data,
                mrpk=self.mrpk_data,
                cmpk_path=str(self.cmpk_path) if self.cmpk_path is not None else None,
                mrpk_path=str(self.mrpk_path) if self.mrpk_path is not None else None,
            )
            saved = save_internal_icd_payload(payload, out_path)
            self._log(f"[ICD] internal ICD saved: {saved}")
            self._log(
                "[ICD] "
                f"pieces={int(payload.get('summary', {}).get('pieceCount', 0))}, "
                f"areaPieces={int(payload.get('summary', {}).get('areaPieceCount', 0))}, "
                f"linePieces={int(payload.get('summary', {}).get('linePieceCount', 0))}, "
                f"expectedPaths={int(payload.get('summary', {}).get('expectedPathCount', 0))}"
            )
            QMessageBox.information(self, "Export Complete", f"Saved Internal ICD JSON:\n{saved}")
        except Exception as exc:
            self._log("[ERROR] export internal ICD failed")
            self._log(str(exc))
            self._log(traceback.format_exc())
            QMessageBox.critical(self, "Internal ICD Export Failed", str(exc))

    def _render_map(self) -> None:
        if self.map_view is not None:
            self.map_view.set_data(
                self.cmpk_data,
                self.mrpk_data,
                self.split_result,
                flight_plans_0303=self.flight_plans_0303,
                flight_plans_0304=self.flight_plans_0304,
                layer_visibility=self._current_layer_visibility(),
            )
        if self.gantt_view is not None:
            sched = self.split_result.schedule_result if self.split_result is not None else None
            self.gantt_view.set_schedule(sched if isinstance(sched, dict) and sched else None)

    def _log(self, msg: str) -> None:
        self.log.appendPlainText(msg)

