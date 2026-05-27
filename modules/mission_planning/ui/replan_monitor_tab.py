"""
다음협업기저임무 재계획 모니터링 탭.

실시간 0401 UAV 데이터 (receive_center) + 현재 활성 InputMissionPlan 기반으로
다음 협업기저 area 임무에 대해 division planner 알고리즘을 테스트/시각화.
결과는 resource/db/test_alg/ 에 저장 (실제 임무계획 미적용).
"""
from __future__ import annotations

import json
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from modules.common import db_paths
from modules.mission_planning.runtime.next_collab_heading import (
    monitor_heading_to_planner_bearing_deg,
)

# 0401 실시간 수신 ─────────────────────────────────────────────────────
try:
    from modules.common import receive_center as _rc
    _RECEIVE_CENTER_AVAILABLE = True
except Exception:
    _RECEIVE_CENTER_AVAILABLE = False

# Division planner imports ─────────────────────────────────────────────
_DIVISION_PLANNER_AVAILABLE = False
_IMPORT_ERROR = ""
try:
    from modules.mission_planning.planners.next_collab_division._planning_canvas import (
        PlanningCanvas,
    )
    from modules.mission_planning.planners.next_collab_division._constants import (
        MODE_MISSION_READY,
        MODE_RESULT_READY,
        MISSION_AREA,
    )
    from modules.mission_planning.planners.next_collab_division._geo_utils import (
        llh_to_local_xy,
    )
    _DIVISION_PLANNER_AVAILABLE = True
except Exception as _err:
    _IMPORT_ERROR = str(_err)
    print(f"[ReplanMonitorTab] division planner import failed: {_err}")

# ── constants ─────────────────────────────────────────────────────────
_PROJECT_ROOT = Path(__file__).resolve().parents[3]
_TEST_ALG_DIR = _PROJECT_ROOT / "resource" / "db" / "test_alg"


def _monitor_heading_to_planner_bearing_deg(raw_heading_deg: object | None) -> float:
    return float(monitor_heading_to_planner_bearing_deg(raw_heading_deg))


class ReplanMonitorTab(QWidget):
    """다음협업기저임무 재계획 테스트 탭."""

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)

        # ── state ─────────────────────────────────────────────────────
        self._uav_states: List[Dict[str, Any]] = []
        self._next_area_mission: Optional[Dict[str, Any]] = None
        self._next_mission_id: Optional[int] = None
        self._input_package_id: Optional[int] = None
        self._target_uav_ids: List[int] = []
        self._mission_polygon_llh: List[Dict[str, float]] = []
        self._planner = None
        self._mission_loaded = False

        # ── UI ────────────────────────────────────────────────────────
        root_layout = QHBoxLayout(self)
        root_layout.setContentsMargins(6, 6, 6, 6)

        # LEFT panel ──────────────────────────────────────────────────
        left_panel = QWidget()
        left_panel.setFixedWidth(340)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setSpacing(8)

        # -- 1. 실시간 상태 ---
        grp_status = QGroupBox("1. 실시간 상태")
        grp_status_layout = QVBoxLayout(grp_status)
        self.lbl_0401 = QLabel("0401: 수신 대기중...")
        self.lbl_0401.setWordWrap(True)
        self.lbl_0401.setStyleSheet("font-size: 10px; color: #666;")
        self.lbl_mission = QLabel("임무: 미로드")
        self.lbl_mission.setWordWrap(True)
        self.lbl_mission.setStyleSheet("font-size: 10px; color: #666;")
        self.btn_refresh_mission = QPushButton("활성 임무 새로고침")
        self.btn_refresh_mission.clicked.connect(self._load_active_mission)
        grp_status_layout.addWidget(self.lbl_0401)
        grp_status_layout.addWidget(self.lbl_mission)
        grp_status_layout.addWidget(self.btn_refresh_mission)
        left_layout.addWidget(grp_status)

        # -- 2. 재계획 실행 ---
        grp_run = QGroupBox("2. 다음 협업기저임무 재계획")
        grp_run_layout = QVBoxLayout(grp_run)
        self.btn_run_all = QPushButton("재계획 실행")
        self.btn_run_all.clicked.connect(self._run_full_pipeline)
        self.btn_run_all.setEnabled(False)
        self.btn_run_all.setStyleSheet("font-weight: bold; font-size: 13px; padding: 8px;")
        grp_run_layout.addWidget(self.btn_run_all)

        self.btn_step_division = QPushButton("Step 1: Area Division")
        self.btn_step_division.clicked.connect(self._step_area_division)
        self.btn_step_division.setEnabled(False)
        self.btn_step_midline = QPushButton("Step 2: Mid Line Generation")
        self.btn_step_midline.clicked.connect(self._step_mid_line)
        self.btn_step_midline.setEnabled(False)
        self.btn_step_path0 = QPushButton("Step 3: Make Path - 0")
        self.btn_step_path0.clicked.connect(self._step_make_path_0)
        self.btn_step_path0.setEnabled(False)
        self.btn_step_sweep = QPushButton("Step 4: Make Sweep")
        self.btn_step_sweep.clicked.connect(self._step_make_sweep)
        self.btn_step_sweep.setEnabled(False)
        for btn in (self.btn_step_division, self.btn_step_midline,
                     self.btn_step_path0, self.btn_step_sweep):
            grp_run_layout.addWidget(btn)
        left_layout.addWidget(grp_run)

        # -- 3. 결과 저장 ---
        grp_save = QGroupBox("3. 결과")
        grp_save_layout = QVBoxLayout(grp_save)
        self.btn_save = QPushButton("test_alg 에 결과 저장")
        self.btn_save.clicked.connect(self._save_results)
        self.btn_save.setEnabled(False)
        grp_save_layout.addWidget(self.btn_save)
        left_layout.addWidget(grp_save)

        # -- 로그 ---
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(500)
        self.log_text.setStyleSheet("font-size: 10px; font-family: Consolas, monospace;")
        left_layout.addWidget(self.log_text, stretch=1)

        root_layout.addWidget(left_panel)

        # RIGHT panel: PlanningCanvas ──────────────────────────────────
        if _DIVISION_PLANNER_AVAILABLE:
            self._canvas = PlanningCanvas(self)
            self._canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            root_layout.addWidget(self._canvas, stretch=1)
            self._log("[INIT] PlanningCanvas 로드 완료")
        else:
            lbl = QLabel(f"Division planner 로드 실패:\n{_IMPORT_ERROR}")
            lbl.setAlignment(Qt.AlignCenter)
            lbl.setWordWrap(True)
            root_layout.addWidget(lbl, stretch=1)
            self._canvas = None
            self._log(f"[INIT] Canvas 실패: {_IMPORT_ERROR}")

        # ── 0401 실시간 수신 등록 ─────────────────────────────────────
        if _RECEIVE_CENTER_AVAILABLE:
            _rc.register_listener("0401", self._on_0401_received)
            self._log("[INIT] 0401 receive_center 등록 완료")
        else:
            self._log("[INIT] receive_center 사용 불가")

        # ── 초기 임무 로드 ────────────────────────────────────────────
        QTimer.singleShot(500, self._load_active_mission)

    # ──────────────────────────────────────────────────────────────────
    #  로그
    # ──────────────────────────────────────────────────────────────────
    def _log(self, text: str) -> None:
        self.log_text.appendPlainText(text)

    # ──────────────────────────────────────────────────────────────────
    #  0401 실시간 수신 (receive_center 콜백)
    # ──────────────────────────────────────────────────────────────────
    def _on_0401_received(self, _msg_id: str, payload: Any) -> None:
        """receive_center에서 0401 메시지가 들어올 때 자동 호출."""
        try:
            body: Dict[str, Any] = {}
            if isinstance(payload, dict):
                body = payload
            elif isinstance(payload, (bytes, bytearray)):
                body = json.loads(payload.decode("utf-8"))
            elif isinstance(payload, str):
                body = json.loads(payload)
            elif payload is not None:
                body = json.loads(json.dumps(payload, default=str))

            agent_states = body.get("agentStateList") or body.get("agent_states") or []
            uav_states: List[Dict[str, Any]] = []
            for state in agent_states:
                if not isinstance(state, dict):
                    continue
                aid = state.get("aircraftID")
                if aid is None:
                    continue
                coord = state.get("coordinate") or {}
                lat = coord.get("latitude")
                lon = coord.get("longitude")
                vel = state.get("velocity") or {}
                if lat is None or lon is None:
                    continue
                uav_states.append({
                    "aircraftID": int(aid),
                    "isUnmanned": bool(state.get("isUnmanned", False)),
                    "latitude": float(lat),
                    "longitude": float(lon),
                    "altitude": float(coord.get("altitude", 0) or 0),
                    "heading": float(vel.get("heading", 0) or 0),
                    "speed": float(vel.get("speed", 0) or 0),
                })

            self._uav_states = uav_states
            unmanned = [s for s in uav_states if s["isUnmanned"]]
            self._target_uav_ids = sorted(s["aircraftID"] for s in unmanned)

            # 라벨 업데이트
            uav_text = " | ".join(
                f"UAV{s['aircraftID']}({s['latitude']:.4f},{s['longitude']:.4f} h={s['heading']:.0f}°)"
                for s in unmanned
            )
            self.lbl_0401.setText(f"0401: UAV {len(unmanned)}대\n{uav_text}")

            # 캔버스에 UAV 위치 업데이트 (CanvasState 올바른 필드)
            if self._canvas is not None and _DIVISION_PLANNER_AVAILABLE and unmanned:
                cs = self._canvas._state
                cs.uav_ids = list(self._target_uav_ids)
                # uav_positions_xy: List[Tuple] — uav_ids 순서대로
                cs.uav_positions_xy = []
                cs.uav_heading_deg = []
                for aid in cs.uav_ids:
                    s = next((x for x in unmanned if x["aircraftID"] == aid), None)
                    if s:
                        xy = llh_to_local_xy(s["latitude"], s["longitude"])
                        cs.uav_positions_xy.append(xy)
                        cs.uav_heading_deg.append(
                            _monitor_heading_to_planner_bearing_deg(s["heading"])
                        )
                self._canvas.update()

            self._update_buttons()
        except Exception:
            pass  # 실시간 수신 실패는 무시

    # ──────────────────────────────────────────────────────────────────
    #  활성 임무 로드 (현재 소티 기준)
    # ──────────────────────────────────────────────────────────────────
    def _load_active_mission(self) -> None:
        try:
            db_root = db_paths.get_active_db_root()
            input_dir = db_root / "InputMissionPlan"
            if not input_dir.exists():
                self.lbl_mission.setText("임무: InputMissionPlan 없음")
                return

            json_files = sorted(input_dir.glob("*.json"),
                                key=lambda p: p.stat().st_mtime, reverse=True)
            if not json_files:
                self.lbl_mission.setText("임무: 파일 없음")
                return

            input_data = None
            for json_path in json_files:
                try:
                    data = json.loads(json_path.read_text(encoding="utf-8"))
                    if isinstance(data.get("inputMissionList"), list):
                        input_data = data
                        self._input_package_id = int(
                            data.get("inputMissionPackageID", 0) or 0
                        )
                        break
                except Exception:
                    continue

            if input_data is None:
                self.lbl_mission.setText("임무: 유효한 데이터 없음")
                return

            # 다음 미완료 Area 임무 (areaList 보유 기준)
            mission_list = input_data.get("inputMissionList") or []
            next_area = None
            for mission in mission_list:
                if not isinstance(mission, dict):
                    continue
                if mission.get("isDone"):
                    continue
                detail = mission.get("missionDetail") or {}
                area_list = detail.get("areaList") or []
                if (area_list
                    and isinstance(area_list[0], dict)
                    and area_list[0].get("coordinateList")):
                    next_area = mission
                    break

            if next_area is None:
                summary = ", ".join(
                    f"ID={m.get('inputMissionID')}(t={m.get('inputMissionType')})"
                    for m in mission_list if isinstance(m, dict)
                )
                self.lbl_mission.setText(f"임무: Area 없음 [{summary}]")
                if not self._mission_loaded:
                    self._log(f"[MISSION] pkgID={self._input_package_id} Area 없음: {summary}")
                return

            self._next_area_mission = next_area
            self._next_mission_id = int(next_area.get("inputMissionID", 0) or 0)

            # Area 폴리곤 좌표
            detail = next_area.get("missionDetail") or {}
            area_list = detail.get("areaList") or []
            polygon_llh: List[Dict[str, float]] = []
            if area_list:
                for c in area_list[0].get("coordinateList") or []:
                    if isinstance(c, dict) and c.get("latitude") is not None:
                        polygon_llh.append({
                            "latitude": float(c["latitude"]),
                            "longitude": float(c["longitude"]),
                            "altitude": float(c.get("altitude", 0) or 0),
                        })
            self._mission_polygon_llh = polygon_llh

            self.lbl_mission.setText(
                f"임무: ID={self._next_mission_id}"
                f" (type={next_area.get('inputMissionType')})"
                f" {len(polygon_llh)}pts"
            )

            if not self._mission_loaded:
                self._log(
                    f"[MISSION] 다음 Area 임무: ID={self._next_mission_id}"
                    f" type={next_area.get('inputMissionType')}"
                    f" vertices={len(polygon_llh)}"
                )
                for i, pt in enumerate(polygon_llh):
                    self._log(f"  V{i}: ({pt['latitude']:.6f}, {pt['longitude']:.6f})")
                self._mission_loaded = True

            # 캔버스에 폴리곤 표시
            if self._canvas is not None and _DIVISION_PLANNER_AVAILABLE and polygon_llh:
                cs = self._canvas._state
                cs.mission_points_xy = [
                    llh_to_local_xy(pt["latitude"], pt["longitude"])
                    for pt in polygon_llh
                ]
                cs.mission_kind = MISSION_AREA
                cs.mode = MODE_MISSION_READY
                self._canvas.update()

            self._update_buttons()
        except Exception:
            self._log(f"[MISSION] ERROR:\n{traceback.format_exc()}")

    # ──────────────────────────────────────────────────────────────────
    #  Planner 초기화
    # ──────────────────────────────────────────────────────────────────
    def _ensure_planner(self) -> bool:
        if self._planner is not None:
            return True
        if not _DIVISION_PLANNER_AVAILABLE:
            self._log("[ERROR] Division planner not available")
            return False
        try:
            from modules.mission_planning.planners.next_collab_division._planner_window import (
                DivisionPlannerWindow,
            )
            self._planner = DivisionPlannerWindow()
            self._planner.hide()
            # canvas의 state를 planner와 공유
            if self._canvas is not None:
                self._planner.state = self._canvas._state
                self._planner.canvas = self._canvas
            self._log("[PLANNER] 초기화 완료")
            return True
        except Exception:
            self._log(f"[ERROR] Planner 초기화 실패:\n{traceback.format_exc()}")
            return False

    # ──────────────────────────────────────────────────────────────────
    #  Pipeline Steps
    # ──────────────────────────────────────────────────────────────────
    def _step_area_division(self) -> None:
        if not self._ensure_planner():
            return
        try:
            self._log("[STEP1] Area Division...")

            # planner state에 최신 데이터 반영
            cs = self._planner.state
            cs.mode = MODE_MISSION_READY
            self._log(
                f"  mission_points={len(cs.mission_points_xy)}"
                f" uav_ids={cs.uav_ids}"
                f" positions={len(cs.uav_positions_xy)}"
                f" headings={len(cs.uav_heading_deg)}"
            )

            self._planner._run_area_division()

            sr = self._planner.state.split_result
            if sr is not None:
                self._log(f"[STEP1] 완료: {len(sr.pieces)} pieces")
                if self._canvas:
                    self._canvas._state.split_result = sr
                    self._canvas._state.mode = MODE_RESULT_READY
                    self._canvas.update()
            else:
                self._log("[STEP1] 결과 없음")
            self._update_buttons()
        except Exception:
            self._log(f"[STEP1] ERROR:\n{traceback.format_exc()}")

    def _step_mid_line(self) -> None:
        if not self._ensure_planner():
            return
        try:
            self._log("[STEP2] Mid Line Generation...")
            self._planner._generate_mid_lines()
            overlays = self._planner.state.mid_line_segments
            if self._canvas:
                self._canvas._state.mid_line_segments = overlays
                self._canvas.update()
            no_split = self._planner._mid_line_no_split_mode()
            self._log(f"[STEP2] 완료: {len(overlays or [])} overlays, no_split={no_split}")
            self._update_buttons()
        except Exception:
            self._log(f"[STEP2] ERROR:\n{traceback.format_exc()}")

    def _step_make_path_0(self) -> None:
        if not self._ensure_planner():
            return
        try:
            self._log("[STEP3] Make Path - 0...")
            self._planner._make_path_0()
            paths = self._planner.state.expected_paths
            if self._canvas:
                self._canvas._state.expected_paths = paths
                self._canvas.update()
            self._log(f"[STEP3] 완료: {len(paths or [])} paths")
            self._update_buttons()
        except Exception:
            self._log(f"[STEP3] ERROR:\n{traceback.format_exc()}")

    def _step_make_sweep(self) -> None:
        if not self._ensure_planner():
            return
        try:
            self._log("[STEP4] Make Sweep...")
            self._planner._make_sweep()
            if self._canvas:
                self._canvas._state.expected_paths = self._planner.state.expected_paths
                self._canvas.update()
            for p in (self._planner.state.expected_paths or []):
                n = p.get("sweepLineCount", 0)
                if n > 0:
                    self._log(f"  UAV{p.get('aircraftID','?')}: {n} sweep lines")
            self._log("[STEP4] 완료")
            self._update_buttons()
        except Exception:
            self._log(f"[STEP4] ERROR:\n{traceback.format_exc()}")

    # ──────────────────────────────────────────────────────────────────
    #  전체 자동 실행
    # ──────────────────────────────────────────────────────────────────
    def _run_full_pipeline(self) -> None:
        self._log("=" * 50)
        self._log(f"[AUTO] 재계획: missionID={self._next_mission_id} UAV={self._target_uav_ids}")
        self._log("=" * 50)
        try:
            self._step_area_division()
            if self._planner is None or self._planner.state.split_result is None:
                self._log("[AUTO] Area Division 실패")
                return

            self._step_mid_line()
            if not self._planner._mid_line_no_split_mode():
                self._log("[AUTO] no-split 모드 아님 — 현재는 no-split만 지원")
                return

            self._step_make_path_0()
            has_path0 = any(
                isinstance(r, dict) and str(r.get("source", "") or "") == "make_path_0"
                for r in (self._planner.state.expected_paths or [])
            )
            if not has_path0:
                self._log("[AUTO] Make Path - 0 결과 없음")
                return

            self._step_make_sweep()
            self._log("[AUTO] 재계획 완료!")
            self.btn_save.setEnabled(True)
        except Exception:
            self._log(f"[AUTO] ERROR:\n{traceback.format_exc()}")

    # ──────────────────────────────────────────────────────────────────
    #  결과 저장
    # ──────────────────────────────────────────────────────────────────
    def _save_results(self) -> None:
        try:
            _TEST_ALG_DIR.mkdir(parents=True, exist_ok=True)
            ts = datetime.now().strftime("%Y%m%dT%H%M%S")
            out_dir = _TEST_ALG_DIR / f"replan_{self._next_mission_id or 0}_{ts}"
            out_dir.mkdir(parents=True, exist_ok=True)

            info = {
                "missionID": self._next_mission_id,
                "inputPackageID": self._input_package_id,
                "polygonLLH": self._mission_polygon_llh,
                "uavStates": [s for s in self._uav_states if s.get("isUnmanned")],
                "targetUavIDs": self._target_uav_ids,
                "timestamp": ts,
            }
            (out_dir / "input_info.json").write_text(
                json.dumps(info, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            if self._planner and self._planner.state.expected_paths:
                (out_dir / "expected_paths.json").write_text(
                    json.dumps(_json_safe(self._planner.state.expected_paths),
                               ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
            if self._planner and self._planner.state.split_result is not None:
                sr = self._planner.state.split_result
                (out_dir / "split_result.json").write_text(
                    json.dumps(_json_safe({
                        "uav_count": sr.uav_count, "uav_ids": sr.uav_ids,
                        "pieces": [{"piece_index": p.piece_index,
                                     "assigned_uav": p.assigned_uav,
                                     "data": p.data} for p in sr.pieces],
                    }), ensure_ascii=False, indent=2), encoding="utf-8",
                )
            self._log(f"[SAVE] 저장: {out_dir}")
            QMessageBox.information(self, "저장", f"결과 저장:\n{out_dir}")
        except Exception:
            self._log(f"[SAVE] ERROR:\n{traceback.format_exc()}")

    # ──────────────────────────────────────────────────────────────────
    #  버튼 상태
    # ──────────────────────────────────────────────────────────────────
    def _update_buttons(self) -> None:
        ready = (
            bool(self._target_uav_ids)
            and self._next_area_mission is not None
            and bool(self._mission_polygon_llh)
            and len(self._uav_states) > 0
        )
        self.btn_run_all.setEnabled(ready)
        self.btn_step_division.setEnabled(ready)

        has_split = (self._planner is not None
                     and self._planner.state.split_result is not None)
        self.btn_step_midline.setEnabled(has_split)

        has_midline = (has_split
                       and self._planner is not None
                       and bool(self._planner.state.mid_line_segments)
                       and self._planner._mid_line_no_split_mode())
        self.btn_step_path0.setEnabled(has_midline)

        has_path0 = (self._planner is not None and any(
            isinstance(r, dict) and str(r.get("source", "") or "") == "make_path_0"
            for r in (self._planner.state.expected_paths or [])
        ))
        self.btn_step_sweep.setEnabled(has_path0)
        self.btn_save.setEnabled(has_path0)


def _json_safe(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {k: _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(item) for item in obj]
    if isinstance(obj, (int, float, str, bool, type(None))):
        return obj
    return str(obj)
