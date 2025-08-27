# 파일: /mnt/data/mission_planning_gui.py
# -*- coding: utf-8 -*-
# mission_planning_gui.py – 업무 할당·계획수립(AssignmentPlanning) 전용 GUI
from __future__ import annotations

import sys, os, threading, json, re, time, shutil
os.environ["KU_ROLE"] = "mission"
from pathlib import Path

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QShortcut, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSlider
from PyQt5.QtGui import QKeySequence

# ───────── Qt 경고 필터 ─────────
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message: return
    sys.stderr.write(message + "\n")
qInstallMessageHandler(_qt_silent_handler)

_EPOCH2000_MS = 946684800000
def _now_ms_since_2000():
    import time
    return int(time.time() * 1000) - _EPOCH2000_MS

# ───────── 경로 부트스트랩 ─────────
def _bootstrap_paths():
    here = Path(__file__).resolve()
    modules_dir = here.parents[1]                # .../modules
    root = modules_dir.parent                    # .../<project root>
    common_dir = modules_dir / "common"
    for p in (modules_dir / "mission_planning", common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try: os.chdir(root)
    except Exception: pass
    return root, common_dir
PROJECT_ROOT, COMMON_DIR = _bootstrap_paths()

# ───────── 설정/라이선스 정규화 ─────────
def _ensure_fusion_configs():
    cands = [
        PROJECT_ROOT / "nFusionSettings.json",
        COMMON_DIR   / "nFusionSettings.json",
        PROJECT_ROOT / "FusionSettings.json",
        COMMON_DIR   / "FusionSettings.json",
        PROJECT_ROOT / "nFusion" / "FusionSettings.json",
    ]
    src = next((p for p in cands if p.exists()), None)
    if src is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json 이 없습니다.")
    dst = PROJECT_ROOT / "nFusionSettings.json"
    if src != dst:
        dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")

    lcands = [
        PROJECT_ROOT / "nFusionLicense.lic",
        COMMON_DIR   / "nFusionLicense.lic",
        PROJECT_ROOT / "nFusion" / "nFusionLicense.lic",
    ]
    lsrc = next((p for p in lcands if p.exists()), None)
    if lsrc:
        ldst = PROJECT_ROOT / "nFusionLicense.lic"
        if lsrc != ldst:
            ldst.write_text(lsrc.read_text(encoding="utf-8"), encoding="utf-8")
    return str(dst)

# ───────── nFusion DLL/어셈블리 로드 ─────────
from dll_files.nFusionImports import *  # FusionNodeIoc, NodeMessenger, clr 등
def _load_msglib_and_deps():
    _clr = globals().get("clr", None)
    if _clr is None:
        try: from dll_files.nFusionImports import clr as _clr  # type: ignore
        except Exception: import clr as _clr  # type: ignore
    msg_dir = COMMON_DIR / "msg_files"
    stem = msg_dir / "MessageLibrary"
    try: _clr.AddReference(str(stem))
    except Exception: _clr.AddReference(str(stem.with_suffix(".dll")))
    for s in ("K4586Model", "K4586Model.Assist", "MiscUtil"):
        dll = msg_dir / (s + ".dll")
        if dll.exists():
            try: _clr.AddReference(str(dll.with_suffix("")))
            except Exception:
                try: _clr.AddReference(str(dll))
                except Exception: pass
_settings_path = _ensure_fusion_configs()
_ = _load_msglib_and_deps()

# Receiver 타입 로드(메시지 컨슈머 등록용)
from receive import *  # noqa

# 탭
from Tabs.assignment_planning_tab import AssignmentPlanningTab


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)   # UDP 제어 → UI 스레드
    log_sig      = pyqtSignal(str)    # 백스레드 로그 → UI 스레드
    start_push_seq = pyqtSignal()     # ★ 푸시 시퀀스 시작(메인스레드)  ← 추가

    def __init__(self):
        super().__init__()
        self.setWindowTitle("업무 할당·계획수립 GUI")
        self.resize(1100, 700)

        self._self_check_sent = False
        self._last_ctrl_ts = {}     # 디듀프
        self._initplan_running = False
        self._last_mission_plan_id = None

        tabs = QTabWidget()
        self._tab = AssignmentPlanningTab(messenger=NodeMessenger)
        tabs.addTab(self._tab, "업무 할당·계획수립 CSC")

        # ───── 상단 슬라이더 바 ─────
        top = QWidget(); top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(8,4,8,4); top_layout.addStretch(1)
        self.mode_slider = QSlider(Qt.Horizontal); self.mode_slider.setRange(0,4)
        self.mode_slider.setSingleStep(1); self.mode_slider.setTickInterval(1)
        self.mode_slider.setTickPosition(QSlider.TicksBelow); self.mode_slider.setFixedWidth(420)
        self.mode_slider.valueChanged.connect(self._on_mode_slider_changed)
        self.mode_now = QLabel("대기모드"); self.mode_now.setStyleSheet("font-weight:600; padding-left:8px;")
        lbl = QLabel("모드:"); lbl.setStyleSheet("color:#789; padding-right:6px;")
        top_layout.addWidget(lbl); top_layout.addWidget(self.mode_slider); top_layout.addWidget(self.mode_now)

        center = QWidget(); v = QVBoxLayout(center); v.setContentsMargins(0,0,0,0)
        v.addWidget(top); v.addWidget(tabs)
        self.setCentralWidget(center)
        self._set_mode_slider_by_text("전원 OFF")

        self.ctrl_payload.connect(self._handle_ctrl_payload)
        self.log_sig.connect(self._append_log_line)
        self.start_push_seq.connect(self._start_push_sequence)

        threading.Thread(target=self._rx_setup, daemon=True).start()
        self._start_control_udp()
        self._install_test_shortcuts()

    def _start_push_sequence(self):
        QTimer.singleShot(0,    lambda: self._click_tx_button_for("0304"))
        QTimer.singleShot(200,  lambda: self._click_tx_button_for("0303"))
        QTimer.singleShot(400,  lambda: self._click_tx_button_for("0302"))
        QTimer.singleShot(1400, lambda: self._click_tx_button_for("0301"))

        QTimer.singleShot(1600, lambda: self._push_0305(status=1, reason="초기임무재계획"))
        QTimer.singleShot(1700, lambda: (
            self._last_mission_plan_id is not None and
            self._push_0901_options(int(self._last_mission_plan_id))
        ))

    # ───────── TX 버튼 실제 클릭 (추가) ─────────
    def _click_tx_button_for(self, code: str):
        """
        TX 테이블에서 메시지 코드 행을 찾아 버튼 click()을 우선 시도.
        버튼 위젯이 없으면 내부 핸들러 호출로 폴백.
        """
        try:
            tab = getattr(self, "_tab", None)
            if tab is None or not hasattr(tab, "tbl_tx"):
                self._append_log_line(f"[WARN] TX 테이블을 찾을 수 없음 → code={code}")
                return

            tbl = tab.tbl_tx
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == str(code):
                    target_row = r
                    break

            if target_row < 0:
                self._append_log_line(f"[WARN] TX 테이블에 {code} 행이 없음")
                return

            # (A) 셀 위젯 버튼 클릭
            try:
                btn = tbl.cellWidget(target_row, 3)
                if btn is not None and hasattr(btn, "click"):
                    btn.click()
                    self._append_log_line(f"[PUSH] {code} 버튼 click()")
                    return
            except Exception:
                pass

            # (B) 내부 핸들러 직접 호출
            try:
                if hasattr(tab, "_on_tx_button_clicked"):
                    tab._on_tx_button_clicked(target_row)
                    self._append_log_line(f"[PUSH] {code} 내부 핸들러 호출")
                    return
            except Exception:
                pass

            self._append_log_line(f"[ERR] {code} 푸시 실행 실패: 버튼/핸들러 접근 불가")
        except Exception as e:
            self._append_log_line(f"[ERR] {code} 푸시 실행 실패: {e}")

    # ───────── 로깅 유틸 ─────────
    def _append_log_line(self, text: str):
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text); return
        except Exception:
            pass
        try: print(text)
        except Exception: pass

    # ───────── 모드/슬라이더 ─────────
    def _on_mode_slider_changed(self, val: int):
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        try: self.mode_now.setText(labels[int(val)])
        except Exception: pass
        self._append_log_line(f"[MODE] 슬라이더 변경 → {labels[int(val)] if 0 <= val < len(labels) else val}")

    def _set_mode_slider_by_text(self, text: str):
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        norm = re.sub(r"\s+", "", str(text)).lower()
        mapping = {"전원off":0,"off":0,"poweroff":0,"0":0,"전원on":1,"on":1,"poweron":1,"1":1,"대기모드":2,"대기":2,"standby":2,"2":2,"초기임무계획":3,"초기임무계획모드":3,"initplan":3,"initial":3,"3":3,"임무수행":4,"execution":4,"4":4}
        val = mapping.get(norm, 2)
        try:
            if getattr(self, "mode_slider", None):
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True); self.mode_slider.setValue(val); self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None): self.mode_now.setText(labels[val])
        except Exception: pass

    # ───────── 버스 초기화 ─────────
    def _rx_setup(self):
        FusionNodeIoc.Configure()
        NodeMessenger.Initialize("MultiTopicReceiveNode")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()

    def _push_0305(self, status: int, reason: str = "초기임무재계획"):
        try:
            from push_center import push_message
            body = {
                "timestamp": _now_ms_since_2000(),
                "source": "IDM",                    # ★ 요구사항 반영
                "missionPlanningStatus": int(status),
                "replanReason": reason,
            }
            push_message("0305", NodeMessenger, body_dict=body)
            self.log_sig.emit(f"[0305] status={status}, reason={reason} 전송")
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0305 전송 실패: {e}")

    def _push_0901_options(self, mission_plan_id: int):
        try:
            from push_center import push_message
            ts = _now_ms_since_2000()
            body = {
                "timestamp": ts,
                "source": "DSC",                  # ★ 요구사항 반영
                "requestTime": ts,                # ★ timestamp와 동일
                "pendingOptionList": [
                    {
                        "optionID": 1,
                        "optionName": "시스템추천",
                        "missionPlanID": int(mission_plan_id),
                    }
                ],
            }
            push_message("0901", NodeMessenger, body_dict=body)
            self.log_sig.emit(f"[0901] 옵션요청 1건(MPID={mission_plan_id}) 전송")
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0901 전송 실패: {e}")

    # ───────── 0102 폴백 송신 ─────────
    def _send_self_check_0102(self, status: int = 1, _retry: int = 0):
        try:
            from push_center import push_message
        except Exception as e:
            self._append_log_line(f"0102 push import 실패: {e}"); return
        try:
            body = self._tab._build_overridden_body("0102") or {}
        except Exception:
            body = {}
        if not body:
            body = {"timestamp": _now_ms_since_2000(), "source": "MMR", "status": 1}
        try:
            push_message("0102", NodeMessenger, body_dict=body)
            self._append_log_line("자체점검(0102) 발신")
            self._self_check_sent = True
        except Exception as e:
            if _retry < 5: QTimer.singleShot(500, lambda: self._send_self_check_0102(status=status, _retry=_retry+1))
            else: self._append_log_line(f"자체점검(0102) 발신 실패: {e}")

    # ───────── UDP 컨트롤 수신 ─────────
    def _start_control_udp(self):
        import socket, json, threading, os
        if getattr(self, "_ctrl_udp_started", False): return
        self._ctrl_udp_started = True
        port = int(os.getenv("KU_CTRL_PORT", "45981"))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", port))
            self._append_log_line(f"CTRL UDP 수신 대기 시작 (127.0.0.1:{port})")
        except Exception as e:
            self._append_log_line(f"CTRL UDP 바인드 실패: {e}"); return
        def loop():
            while True:
                try:
                    data, _ = sock.recvfrom(8192)
                    payload = json.loads(data.decode("utf-8", "ignore"))
                    self.ctrl_payload.emit(payload)
                except Exception: pass
        threading.Thread(target=loop, daemon=True).start()

    # ───────── 테스트 단축키 ─────────
    def _install_test_shortcuts(self):
        QShortcut(QKeySequence("1"), self, activated=lambda: self._ensure_0102(True))
        QShortcut(QKeySequence("0"), self, activated=lambda: self._ensure_0102(False))

    def _ensure_0102(self, on: bool) -> bool:
        try:
            tab = self._tab; tbl = tab.tbl_tx
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0102": target_row = r; break
            if target_row < 0: self._append_log_line("[CTRL] TX 테이블에 0102 행이 없음"); return False
            running = "0102" in getattr(tab, "periodic_timers", {})
            if (on and not running) or ((not on) and running):
                try:
                    btn = tbl.cellWidget(target_row, 3)
                    if btn is not None and hasattr(btn, "click"): btn.click(); return True
                except Exception: pass
                try:
                    if hasattr(tab, "_on_tx_button_clicked"): tab._on_tx_button_clicked(target_row); return True
                except Exception: pass
                self._send_self_check_0102(status=1 if on else 0); return True
            return True
        except Exception as e:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {e}"); return False

    # ───────── 모드 명령 처리 ─────────
    def _handle_ctrl_payload(self, payload: dict):
        import time
        try: cmd = str(payload.get("cmd") or "")
        except Exception: return
        key = f"{cmd}:{payload.get('text') or payload.get('status')}"
        now = time.monotonic(); last = self._last_ctrl_ts.get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        if cmd == "self_check":
            try: status = int(payload.get("status", 1))
            except Exception: status = 1
            ok = self._ensure_0102(on=(status == 1))
            if not ok: self._send_self_check_0102(status=status)

        elif cmd == "mode":
            text = str(payload.get("text") or "").strip()
            self._append_log_line(f"[CTRL] 모드 변경 요청 수신: {text}")
            self._set_mode_slider_by_text(text)
            # 초기임무계획이면 파이프라인 실행
            norm = re.sub(r"\s+", "", text).lower()
            if norm in ("초기임무계획", "초기임무계획모드", "initplan", "initial"):
                self._run_initial_plan_async()

    # ───────── 초기임무계획 파이프라인 ─────────
    def _run_initial_plan_async(self):
        if self._initplan_running:
            self._append_log_line("[INFO] 초기임무계획 이미 실행 중")
            return
        self._initplan_running = True
        threading.Thread(target=self._run_initial_plan_do, name="InitPlan-GUI", daemon=True).start()

    def _run_initial_plan_do(self):
        try:
            # 0) 경로·모듈 import
            from pathlib import Path
            import os, sys, json, time, shutil
            mp_pkg_dir  = Path(PROJECT_ROOT) / "modules" / "mission_planning" / "MissionPlanner"
            for p in (mp_pkg_dir, mp_pkg_dir.parent, Path(PROJECT_ROOT) / "modules"):
                p_str = str(p)
                if p.exists() and p_str not in sys.path:
                    sys.path.insert(0, p_str)

            from AnS import run_divide_and_pattern, build_mission_plan_0301
            from data_def import d0301, d0302, d0303, d0304
            from data_def.id_allocator import next_path_id

            # IMP에서 pathID 우선 추출(없으면 None)
            def _imp_path_id(im: dict) -> int | None:
                for k in ("pathID","pathId","individualMissionPathID","missionPathID"):
                    v = im.get(k)
                    try:
                        if v is not None:
                            return int(v)
                    except Exception:
                        pass
                mi = im.get("missionInfo")
                if isinstance(mi, dict):
                    for k in ("pathID","pathId"):
                        v = mi.get(k)
                        try:
                            if v is not None:
                                return int(v)
                        except Exception:
                            pass
                return None

            # FP 생성 후 pathID 강제 일치(안전망)
            def _enforce_fp_path_ids(fps: list[dict], pid_map: dict[tuple[int,int], int]) -> int:
                fixed = 0
                for fp in fps or []:
                    try:
                        aid = int(fp.get("aircraftID", 0))
                        mid = int(fp.get("individualMissionID", 0))
                        key = (aid, mid)
                        if key in pid_map:
                            desired = int(pid_map[key])
                            if str(fp.get("pathID")) != str(desired):
                                fp["pathID"] = desired
                                fixed += 1
                    except Exception:
                        pass
                return fixed

            # ★ 0305: 진행 중
            self.log_sig.emit("[STEP 0] 재계획 수행 상태: 진행 중")
            self._push_0305(status=2, reason="초기임무재계획")

            # 1) 입력 경로
            db_root = Path(os.environ.get("KU_MISSION_DB_ROOT") or (Path(PROJECT_ROOT) / "database"))
            dir_0201 = db_root / "InputMissionPlan"
            dir_0203 = db_root / "MissionReferenceInfo"
            out_root = db_root / "mission_output"
            out_root.mkdir(parents=True, exist_ok=True)
            def _pick_json(d: Path) -> Path | None:
                cands = sorted([p for p in d.glob("*.json") if p.is_file()])
                return cands[0] if cands else None
            cmpk_path = _pick_json(dir_0201); mrpk_path = _pick_json(dir_0203)
            if not cmpk_path or not mrpk_path:
                self.log_sig.emit("[ERR] 0201/0203 입력 JSON을 찾을 수 없습니다."); 
                return

            # 2) 0201+0203 → 0302(IMP)
            self.log_sig.emit("[STEP 1] Divide & Pattern 시작")
            imp_paths = run_divide_and_pattern(str(cmpk_path), str(mrpk_path), str(out_root), log=lambda msg: self.log_sig.emit(str(msg)))
            if not imp_paths:
                self.log_sig.emit("[ERR] IMP 생성 결과가 없습니다."); 
                return
            self.log_sig.emit(f"[OK] IMP {len(imp_paths)}개 생성")

            # 3) 0301 생성
            mp_path = out_root / f"MissionPlan_{int(time.time()*1000)}.json"
            build_mission_plan_0301(str(cmpk_path), str(mrpk_path), imp_paths, str(mp_path))
            with mp_path.open(encoding="utf-8") as f:
                mp_json = json.load(f)
            imp_id_map = {a["aircraftID"]: a["individualMissionPackageID"] for a in mp_json.get("aircraftList", [])}
            self.log_sig.emit(f"[OK] 0301 생성 → {mp_path.name}")

            # 4) missions 집계(0302 원소들 메모리로)
            missions = []
            for imp in imp_paths:
                with open(imp, encoding="utf-8") as f:
                    pkg = json.load(f)
                aid = int(pkg["aircraftID"])
                for im in pkg.get("individualMissionList", []):
                    im2 = dict(im); im2["aircraftID"] = aid
                    if "individualMissionPlanPackageID" not in im2 and imp_id_map:
                        im2["individualMissionPlanPackageID"] = imp_id_map.get(aid)
                    missions.append(im2)

            # 5) pathID 매핑: LAH(1~3)는 합법 ID 재발급, UAV(4~6)는 IMP 값 유지(있으면)
            pid_map: dict[tuple[int,int], int] = {}
            for im in missions:
                aid = int(im.get("aircraftID", 0))
                mid = int(im.get("individualMissionID", 0))
                if aid in (1,2,3):  # LAH
                    pid = int(next_path_id(aid))     # ← d0304의 허용구간 보장
                    im["pathID"] = pid
                    pid_map[(aid, mid)] = pid
                else:               # UAV
                    imp_pid = _imp_path_id(im)
                    if imp_pid is not None:
                        im["pathID"] = int(imp_pid)
                        pid_map[(aid, mid)] = int(imp_pid)

            self.log_sig.emit("[INFO] pathID 매핑 완료(0302·0303·0304 일치 보장)")

            # 6) 0303/0304 FP 생성
            self.log_sig.emit("[STEP 2] FlightPath 0303/0304 생성 시작")
            wp_alloc = d0303._WPAllocator()
            manned_missions = [im for im in missions if int(im.get("aircraftID", 0)) in (1,2,3)]
            uav_missions    = [im for im in missions if int(im.get("aircraftID", 0)) in (4,5,6)]

            flight_plans_0303 = d0303.build_flight_plans(uav_missions, wp_alloc, 40.0, turn_step_deg=15.0) if uav_missions else []
            flight_plans_0304 = d0304.build_lah_flight_plans_fixed(manned_missions, cruise_speed=40.0, wp_alloc=wp_alloc) if manned_missions else []

            # 안전망: pathID 강제 동기화
            fixed3 = _enforce_fp_path_ids(flight_plans_0303, pid_map)
            fixed4 = _enforce_fp_path_ids(flight_plans_0304, pid_map)
            if fixed3 or fixed4:
                self.log_sig.emit(f"[INFO] FP pathID 강제 적용: 0303 fixed={fixed3}, 0304 fixed={fixed4}")

            if not flight_plans_0303 and not flight_plans_0304:
                self.log_sig.emit("[ERR] 0303/0304 모두 실패"); 
                return
            self.log_sig.emit(f"[OK] FP 요약 → 0303={len(flight_plans_0303)} / 0304={len(flight_plans_0304)}")

            # 7) 디스크 저장 (0301/0302/0303/0304)
            self.log_sig.emit("[STEP 3] 결과 저장")
            dir_mp  = db_root / "MissionPlan"
            dir_imp = db_root / "IndividualMissionPlan"
            dir_fp  = db_root / "FlightPath"
            for d in (dir_mp, dir_imp, dir_fp):
                d.mkdir(parents=True, exist_ok=True)
                for p in d.glob("*.json"):
                    try: p.unlink()
                    except Exception: pass

            try:
                mp_id = str(mp_json.get("missionPlanID") or mp_json.get("MissionPlanID"))
            except Exception:
                mp_id = str(int(time.time()))
            (dir_mp / f"{mp_id}.json").write_text(json.dumps(mp_json, indent=2, ensure_ascii=False), encoding="utf-8")

            imp_pkgs = d0302.build_mission_packages(missions, cmpk_id=int(cmpk_path.stem), plan_pkg_map=imp_id_map)
            for pkg in imp_pkgs:
                imp_id = str(pkg.get("individualMissionPackageID") or pkg.get("individualMissionPlanPackageID"))
                (dir_imp / f"{imp_id}.json").write_text(json.dumps(pkg, indent=2, ensure_ascii=False), encoding="utf-8")

            def _dump_fps(lst):
                cnt = 0
                for fp in lst:
                    pid = str(fp["pathID"])
                    (dir_fp / f"{pid}.json").write_text(json.dumps(fp, indent=2, ensure_ascii=False), encoding="utf-8"); cnt += 1
                return cnt
            c3 = _dump_fps(flight_plans_0303); c4 = _dump_fps(flight_plans_0304)

            # 임시 out 폴더 정리
            try:
                if out_root.exists(): shutil.rmtree(out_root)
            except Exception:
                pass

            self.log_sig.emit(f"✔ 저장 완료  →  MissionPlan 1, IndividualMission {len(imp_pkgs)}, FlightPath {c3 + c4}")

            # 방금 만든 MP ID (0901용)
            try:
                self._last_mission_plan_id = int(mp_id)
            except Exception:
                self._last_mission_plan_id = None

            # 8) 순차 푸시(0304→0303→0302→0301) 이후 0305완료/0901
            self.log_sig.emit("[STEP 4] 0304→0303→0302→0301 순차 푸시 (완료 후 0305→0901)")
            self.start_push_seq.emit()

        except Exception as e:
            self.log_sig.emit(f"[ERR] 초기임무계획 파이프라인 실패: {e}")
        finally:
            self._initplan_running = False

# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
