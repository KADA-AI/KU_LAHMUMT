# -*- coding: utf-8 -*- 
# monitoring_gui.py – 임무 모니터링·판단 전용 GUI
from __future__ import annotations

import sys, os, threading, re, time, json, socket
os.environ["KU_ROLE"] = "monitoring"
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]  # .../KU_LAHMUMT
for _p in (_ROOT, _ROOT / "modules", _ROOT / "modules" / "monitoring_ver2", _ROOT / "modules" / "common"):
    _ps = str(_p)
    if _p.exists() and _ps not in sys.path:
        sys.path.insert(0, _ps)

from modules.common.qt_env import ensure_qt_platform
ensure_qt_platform()

from PyQt5.QtCore import (
    qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer, Qt, QEvent, QObject
)
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QShortcut,
    QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSlider
)
from PyQt5.QtGui import QKeySequence

from modules.common.status_reporter import send_status_ok
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port
from modules.common import db_paths
from receive_center import register_listener

from modules.monitoring.config import RECEIVE_MESSAGES as VER2_RECEIVE_MESSAGES


from modules.monitoring.tabs.ver2_monitoring_tab import Ver2MonitoringTab

from modules.monitoring.tabs.ver2_replan_tab import Ver2ReplanTab   # ★ 0101 리스너 등록용

# ───────── Qt 경고 필터 ─────────
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
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
    for p in (modules_dir / "monitoring", modules_dir / "monitoring_ver2", common_dir, root):
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
from dll_files.nFusionImports import *  # FusionNodeIoc, NodeMessenger, clr

def _load_msglib_and_deps():
    _clr = globals().get("clr", None)
    if _clr is None:
        try:
            from dll_files.nFusionImports import clr as _clr  # type: ignore
        except Exception:
            import clr as _clr  # type: ignore
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

from receive import *  # modules/common/receive
from Tabs.mission_monitoring_tab import MissionMonitoringTab

# ───────── 모듈별 모니터링 포트(모니터링/MSM) ─────────
def _mon_port() -> int:
    """모니터링 GUI → 대시보드(run.py) 모니터링 전송 포트"""
    try:
        return int(os.getenv("KU_MON_MONITORING_PORT", "46982"))
    except Exception:
        return 46982

def _z4(s: str) -> str:
    s = str(s).strip()
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s

# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle('모니터링(MSM)')
        self.resize(1100, 700)

        # Power/State
        self._power_on = False
        self._self_check_sent = False
        self._last_ctrl_ts = {}   # 디듀프용
        self._staged_replan_context = None
        self._auto_initplan_triggered = False

        self._manager = None

        self._manager_init_attempts = 0

        self._ver2_monitor_tab = None

        self._ver2_replan_tab = None

        self._ver2_tabs_added = False

        tabs = QTabWidget()

        self._tabs = tabs
        self._tab = MissionMonitoringTab(messenger=NodeMessenger)
        self._install_mode_parsefail_log_filter()

        def _override_body(mid: str):
            code = str(mid).strip()
            if code == "0102":
                return {"Timestamp": _now_ms_since_2000(), "Status": 1, "Source": "MSM"}
            if code == "0902":
                try:
                    return self._build_0902_body()   # ← 아래 2)에서 추가
                except Exception as e:
                    self._append_log_line(f"[0902] 바디 생성 실패: {e}")
                    return None
            return None

        self._tab._build_overridden_body = _override_body

        self._install_power_gate_hooks()  # TX 차단 가드
        self._install_mon_wires()         # ★ 모니터링 전송 훅 (TX/MODE)
        tabs.addTab(self._tab, "임무 모니터링·판단 CSC")

        # ───── 상단 슬라이더 바 ─────
        top = QWidget()
        top_layout = QHBoxLayout(top); top_layout.setContentsMargins(8,4,8,4); top_layout.addStretch(1)
        self.mode_slider = QSlider(Qt.Horizontal)
        self.mode_slider.setRange(0,4); self.mode_slider.setSingleStep(1)
        self.mode_slider.setTickInterval(1); self.mode_slider.setTickPosition(QSlider.TicksBelow)
        self.mode_slider.setFixedWidth(420)
        self.mode_slider.valueChanged.connect(self._on_mode_slider_changed)
        self.mode_now = QLabel("대기모드"); self.mode_now.setStyleSheet("font-weight:600; padding-left:8px;")
        lbl = QLabel("모드:"); lbl.setStyleSheet("color:#789; padding-right:6px;")
        top_layout.addWidget(lbl); top_layout.addWidget(self.mode_slider); top_layout.addWidget(self.mode_now)

        center = QWidget(); v = QVBoxLayout(center); v.setContentsMargins(0,0,0,0)
        v.addWidget(top); v.addWidget(tabs)
        self.setCentralWidget(center)

        # 초기 전원 OFF
        self._set_mode_slider_by_text("전원 OFF")
        self._apply_power_state()

        self.ctrl_payload.connect(self._handle_ctrl_payload)
        threading.Thread(target=self._rx_setup, daemon=True).start()
        self._start_control_udp()
        self._install_test_shortcuts()

        QTimer.singleShot(800, lambda: send_status_ok("MSM"))

        def _on_ctrl(payload: dict):
            try:
                if (payload or {}).get("cmd") == "self_check" and int((payload or {}).get("status", 0)) == 1:
                    send_status_ok("MSM")
            except Exception:
                pass
        start_ctrl_listener(env_ctrl_port(45982), _on_ctrl)

        # ★★★ 0101 수신 → 모드 반영 리스너 (기존 유지)
        self._install_0101_mode_listener()
        # ★★★ RX 테이블 폴링으로도 0101을 잡아 모드 반영(리시버 경로 불안정 대비, 기존 동작 불변)
        self._start_0101_rx_poller()

        QTimer.singleShot(0, self._ensure_manager_initialized)

    def _ensure_manager_initialized(self):

        if self._manager is not None:

            return

        self._manager_init_attempts += 1

        try:
            from modules.monitoring.manager import MonitoringManager

            receive_ids = [mid for mid, _ in VER2_RECEIVE_MESSAGES]

            self._manager = MonitoringManager(NodeMessenger, receive_ids)

        except Exception as exc:

            if self._manager_init_attempts <= 3:

                self._append_log_line(f"[VER2] Manager init 실패: {exc}")

            QTimer.singleShot(1500, self._ensure_manager_initialized)

            return

        self._manager.log_callback = self._handle_manager_log

        self._manager.gui_update_callback = self._handle_manager_update

        self._attach_ver2_tabs()

        self._handle_manager_update("logic", "SystemMode", None)



    def _attach_ver2_tabs(self):

        if self._ver2_tabs_added or self._manager is None:

            return

        try:

            self._ver2_monitor_tab = Ver2MonitoringTab(manager=self._manager)

            self._ver2_replan_tab = Ver2ReplanTab(manager=self._manager)

        except Exception as exc:

            self._append_log_line(f"[VER2] 탭 생성 실패: {exc}")

            self._ver2_monitor_tab = None

            self._ver2_replan_tab = None

            QTimer.singleShot(1500, self._ensure_manager_initialized)

            return

        if hasattr(self, "_tabs") and self._tabs is not None:

            self._tabs.addTab(self._ver2_monitor_tab, "모니터링 요약 (ver2)")

            self._tabs.addTab(self._ver2_replan_tab, "재계획 판단 (ver2)")

        self._ver2_tabs_added = True



    def _handle_manager_log(self, tag: str, level: str, message: str, raw: bytes | None):

        def _log():

            self._append_log_line(f"[{tag}] [{level}] {message}")

        QTimer.singleShot(0, _log)



    def _handle_manager_update(self, update_type: str, key: str, data_object=None):

        def _dispatch():

            targets = [self._ver2_monitor_tab, self._ver2_replan_tab]

            for tab in targets:

                if tab and hasattr(tab, "refresh_display"):

                    try:

                        tab.refresh_display((update_type, key), data_object)

                    except Exception:

                        pass

        QTimer.singleShot(0, _dispatch)



    def _collect_input_mission_ids(self) -> list:
        """
        database/InputMissionPlan/ 아래 모든 *.json에서 inputMissionList[].inputMissionID 수집.
        없으면 (예시값) [107,108] 폴백.
        """
        ids = []
        try:
            base = db_paths.get_db_subpath("InputMissionPlan")
            cand_files = []
            if base.exists() and base.is_dir():
                cand_files.extend([p for p in base.glob("*.json") if p.is_file()])
            single = db_paths.get_db_subpath("InputMissionPlan.json")
            if single.exists():
                cand_files.append(single)

            import json
            for fp in cand_files:
                try:
                    obj = json.loads(fp.read_text(encoding="utf-8"))
                    for it in (obj.get("inputMissionList") or []):
                        try:
                            ids.append(int(it.get("inputMissionID")))
                        except Exception:
                            continue
                except Exception:
                    continue
            ids = sorted(set(ids))
        except Exception:
            ids = []
        if not ids:
            # 폴백(요청 예시)
            ids = [107, 108]
        return ids

    # ───────── 0902: missionPlanID 시퀀스 ─────────
    def _next_mission_plan_ids(self, count: int) -> list:
        """
        database/mission_plan_seq.txt 에서 연속 missionPlanID 지급.
        - 최초: 700000001 시작
        - 호출마다 오름차순, 중복 방지
        """
        seq_file = db_paths.get_db_subpath("mission_plan_seq.txt")
        start = 700000001
        try:
            if seq_file.exists():
                txt = seq_file.read_text(encoding="utf-8").strip()
                if txt:
                    start = max(start, int(txt))
        except Exception:
            start = 700000001
        out = list(range(start, start + int(count)))
        try:
            seq_file.parent.mkdir(parents=True, exist_ok=True)
            seq_file.write_text(str(start + int(count)), encoding="utf-8")
        except Exception:
            pass
        return out

    # ───────── 0902: 하드코딩 바디 생성 ─────────
    def _build_0902_body(self) -> dict:
        """
        요청 사양에 맞춰 0902 재계획요청 생성 (다른 필드는 고정, ID 규칙만 준수).
        - source      : 'MMR' (요청대로 고정)
        - replanLevel : 1
        - replanReason: '초기임무재계획'
        - inputMissionIDList: database/InputMissionPlan 의 모든 inputMissionID
        - pendingOptionList : option 1~3(초기임무재계획 시 1개), missionPlanID 700000001~ 오름차순, 중복X
        """
        now = _now_ms_since_2000()
        input_ids = self._collect_input_mission_ids()

        reason = "초기임무재계획"
        option_specs = [
            {"optionID": 1, "optionName": "시스템추천"},
            {"optionID": 2, "optionName": "임무시간최소화"},
            {"optionID": 3, "optionName": "촬영효과최대"},
        ]
        if reason == "초기임무재계획":
            option_specs = option_specs[:1]

        mpids = self._next_mission_plan_ids(len(option_specs))

        body = {
            "timestamp": now,
            "source": "MMR",  # 미션플래너에서 고정
            "replanRequestTime": {
                "replanRequestTimestamp": now
            },
            "replanLevel": 1,
            "inputMissionIDList": [{"inputMissionID": i} for i in input_ids],
            "replanReason": reason,
            "pendingOptionList": [
                {
                    "optionID": spec["optionID"],
                    "optionName": spec["optionName"],
                    "missionPlanID": mpid,
                }
                for spec, mpid in zip(option_specs, mpids)
            ],
        }
        # 로그(선택)
        if mpids:
            self._append_log_line(
                f"[0902] 재계획 요청 생성 완료 (inputMissionIDs={len(input_ids)}, optionCount={len(option_specs)}, mpid@{mpids[0]})"
            )
        else:
            self._append_log_line(
                f"[0902] 재계획 요청 생성 완료 (inputMissionIDs={len(input_ids)}, optionCount=0)"
            )
        return body

    def _install_mode_parsefail_log_filter(self):
        """탭의 append_log를 래핑해 '[MODE] … 모드 파싱 실패' 로그를 숨긴다."""
        tab = getattr(self, "_tab", None)
        if not tab or not hasattr(tab, "append_log"):
            return
        self._orig_append_log = tab.append_log

        def _filtered_append_log(text: str):
            t = str(text)
            if "[MODE]" in t and "수신은 했으나 모드 파싱 실패" in t:
                return  # ← 해당 라인만 무시
            return self._orig_append_log(text)

        tab.append_log = _filtered_append_log
        
    # ───────── 0101 모드 수신 리스너 ─────────
    def _install_0101_mode_listener(self):
        """
        receive_center.notify("0101", raw)를 직접 수신해
        systemMode 숫자코드를 슬라이더로 바로 반영.
        """
        class _Rx0101:
            def __init__(self, host): self.host = host
            def mark_received(self, msg_id: str, raw: bytes | None = None):
                try:
                    self.host._on_rx_0101(raw)
                except Exception:
                    pass

        try:
            self._rx0101 = _Rx0101(self)
            register_listener("0101", self._rx0101)
            self._append_log_line("[0101] 모드 수신 리스너 등록 완료")
        except Exception as e:
            self._append_log_line(f"[0101] 리스너 등록 실패: {e}")


    def _unwrap_payload(self, payload) -> bytes:
        tab = getattr(self, "_tab", None)
        if tab and hasattr(tab, "_latest_payload_bytes"):
            latest = tab._latest_payload_bytes(payload)
            if isinstance(latest, bytes):
                return latest
        if isinstance(payload, list):
            payload = payload[-1] if payload else b""
        if isinstance(payload, dict):
            payload = payload.get("raw")
        if payload is None:
            return b""
        if isinstance(payload, bytes):
            return payload
        if isinstance(payload, str):
            return payload.encode("utf-8", "ignore")
        try:
            return bytes(payload)
        except Exception:
            return b""

    def _on_rx_0101(self, raw: bytes | None):
        # 1) RAW → 텍스트
        raw_latest = self._unwrap_payload(raw)
        txt = raw_latest.decode("utf-8", "ignore")
        # 2) JSON 추출 시도(원문이 JSON만 올 때와, 프리텍스트가 붙을 때 모두 대응)
        m = re.search(r"\{.*\}", txt, flags=re.S)
        jtxt = m.group(0) if m else txt.strip()
        # 3) 딕셔너리 파싱(안되면 빈 dict)
        try:
            body = json.loads(jtxt) if jtxt.startswith("{") else {}
        except Exception:
            body = {}

        # 4) 코드 추출(대/소문자·자료형·문자열·불리언 모두 흡수)
        code = self._extract_mode_code(body)
        if code is None:
            # 마지막 폴백: RAW에서 정규식으로 직접 찾기
            mm = re.search(r'"systemMode"\s*:\s*([0-9]+)', txt)
            if mm:
                try: code = int(mm.group(1))
                except Exception: code = None

        # ▼▼▼ 여기만 변경: 더 이상 "파싱 실패" 로그를 찍지 않고 조용히 반환
        if code is None:
            return
        # ▲▲▲

        ok = self._apply_system_mode_code(code)
        if not ok:
            self._append_log_line(f"[MODE] 미지원 코드({code})")
        else:
            self._append_log_line(f"[0101] 시스템 운용 모드 수신 → code={code}")

    def _extract_mode_code(self, body: dict) -> int | None:
        """
        dict의 다양한 키에서 모드코드를 견고하게 추출.
        - 키 대/소문자 무시
        - 값이 str/bool/float 모두 허용
        """
        if not isinstance(body, dict):
            return None
        low = {str(k).lower(): body[k] for k in body.keys() if k is not None}
        for key in ("systemmode", "mode", "modecode", "state"):
            if key in low:
                v = low[key]
                # bool → 0/1
                if isinstance(v, bool):
                    return 1 if v else 0
                # 숫자/문자 모두 int로
                try:
                    return int(v)
                except Exception:
                    try:
                        return int(float(str(v).strip()))
                    except Exception:
                        return None
        return None

    def _apply_system_mode_code(self, code: int) -> bool:
        """
        외부 0101 systemMode 매핑 (요청 정의)
          0 : 초기화 모드
          1 : 대기 모드
          2 : 초기임무계획 모드
          3 : 임무수행 모드
        내부 슬라이더(0~4): [0=전원 OFF, 1=초기화 모드, 2=대기모드, 3=초기 임무 계획, 4=임무 수행]
        → 교차 매핑: 0→1, 1→2, 2→3, 3→4
        """
        code_to_slider = {0: 1, 1: 2, 2: 3, 3: 4}
        if code not in code_to_slider:
            return False
        val = code_to_slider[code]
        try:
            # 슬라이더 값 세팅 + 부수효과 실행
            self.mode_slider.blockSignals(True)
            self.mode_slider.setValue(val)
            self.mode_slider.blockSignals(False)
            self._on_mode_slider_changed(val)
        except Exception:
            return False
        return True

    # ───────── RX 테이블 폴링 기반 0101 모드 반영(비침투, 안전) ─────────
    def _start_0101_rx_poller(self):
        """탭의 RX 테이블에 저장된 0101 RAW(UserRole)를 주기적으로 확인해 모드 반영."""
        self._last_0101_raw = None
        self._poll_0101_timer = QTimer(self)
        self._poll_0101_timer.setInterval(250)  # 4Hz
        self._poll_0101_timer.timeout.connect(self._poll_0101_in_rx_table)
        self._poll_0101_timer.start()

    def _poll_0101_in_rx_table(self):
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None) if tab else None
            if tbl is None:
                return
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0101":
                    target_row = r
                    break
            if target_row < 0:
                return
            item = tbl.item(target_row, 0)
            raw_payload = item.data(Qt.UserRole) if item else None
            raw_latest = self._unwrap_payload(raw_payload)
            if not raw_latest or (self._last_0101_raw is not None and raw_latest == self._last_0101_raw):
                return

            txt = raw_latest.decode("utf-8", "ignore")
            m = re.search(r"\{.*\}", txt, flags=re.S)
            jtxt = m.group(0) if m else txt.strip()
            body = {}
            try:
                if jtxt.startswith("{"):
                    body = json.loads(jtxt)
            except Exception:
                body = {}

            code = self._extract_mode_code(body)
            if code is None:
                mm = re.search(r'"systemMode"\s*:\s*([0-9]+)', txt)
                if mm:
                    try: code = int(mm.group(1))
                    except Exception: code = None

            if code is not None:
                if self._apply_system_mode_code(code):
                    # 새 RAW에 대해서만 로그
                    self._append_log_line(f"[0101/POLL] 모드 반영 → code={code}")
                self._last_0101_raw = raw_latest
        except Exception:
            pass

    # ───────── 모드/슬라이더 유틸 ─────────
    def _sw_code(self) -> str:
        role = (os.environ.get("KU_ROLE") or "").lower()
        return {"mission":"MMR","monitoring":"MSM","decision":"MOB"}.get(role, "MMR")

    def _on_mode_slider_changed(self, val: int):
        labels = ["전원 OFF", "초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]
        try: self.mode_now.setText(labels[int(val)])
        except Exception: pass
        self._power_on = (int(val) != 0)
        self._append_log_line(f"[MODE] 슬라이더 변경 → {labels[int(val)] if 0 <= val < len(labels) else val}")
        # ★ 모드 변경도 대시보드로 통지
        try: self._send_mon("mode", text=labels[int(val)], role="MSM")
        except Exception: pass
        self._apply_power_state()
        self._handle_mode_transition(int(val))
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    def _set_mode_slider_by_text(self, text: str):
        # 텍스트 별칭 확장 (공백/‘모드’ 접미 허용)
        labels = ["전원 OFF", "초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]
        norm = re.sub(r"\s+", "", str(text)).lower()

        mapping = {
            "전원off": 0, "off": 0, "poweroff": 0, "0": 0,
            "전원on":  1, "on": 1,  "poweron": 1,  "1": 1,
            "대기모드": 2, "대기": 2, "standby": 2, "2": 2,
            "초기임무계획": 3, "초기임무계획모드": 3, "initplan": 3, "initial": 3, "3": 3,
            "임무수행": 4, "임무수행모드": 4, "execution": 4, "4": 4,

            # 요청 정의와 표현 일치
            "초기화모드": 1,  # 초기화 모드 → 초기화 모드 단계로 표시
            "초기임무계획모드": 3,
        }

        val = mapping.get(norm, 2)
        try:
            if getattr(self, "mode_slider", None) is not None:
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True)
                    self.mode_slider.setValue(val)
                    self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None) is not None:
                self.mode_now.setText(labels[val])
            self._send_mon("mode", text=labels[val], role="MSM")
        except Exception:
            pass

        self._power_on = (int(val) != 0)
        self._apply_power_state()
        self._handle_mode_transition(int(val))
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    def _handle_mode_transition(self, mode_idx: int):
        try:
            idx = int(mode_idx)
        except Exception:
            return
        if idx == 3:
            if not getattr(self, '_auto_initplan_triggered', False):
                self._auto_initplan_triggered = True
                QTimer.singleShot(0, self._auto_prepare_replan)
        else:
            self._auto_initplan_triggered = False

    def _auto_prepare_replan(self):
        try:
            payload = self._build_0902_body()
        except Exception as exc:
            self._append_log_line(f'[AUTO] 0902 컨텍스트 생성 실패: {exc}')
            return
        if not isinstance(payload, dict):
            return
        mission_ids = [item.get('inputMissionID') for item in payload.get('inputMissionIDList', []) if isinstance(item, dict) and item.get('inputMissionID') is not None]
        options = [opt for opt in payload.get('pendingOptionList', []) if isinstance(opt, dict)]
        plan_ids = [opt.get('missionPlanID') for opt in options if opt.get('missionPlanID') is not None]
        option_names = [opt.get('optionName') for opt in options if opt.get('optionName')]
        if not option_names and options:
            option_names = [f'옵션{i+1}' for i in range(len(options))]
        context = {
            'plan_ids': plan_ids,
            'mission_ids': mission_ids,
            'option_names': option_names,
            'replan_level': payload.get('replanLevel', 1),
            'reason': payload.get('replanReason') or '초기 임무계획',
        }
        self._stage_replan_context(context, trigger='auto')
        if self._auto_press_0902_button():
            self._append_log_line('[AUTO] 0902 재계획 요청 자동 송신 실행')
        else:
            self._append_log_line('[AUTO] 0902 자동 송신 실패: 버튼을 찾지 못함')

    # ───────── 모니터링(대시보드) 전송 훅 ─────────
    def _auto_press_0902_button(self) -> bool:
        tab = getattr(self, '_tab', None)
        tbl = getattr(tab, 'tbl_tx', None) if tab else None
        if tbl is None:
            return False
        row = -1
        if hasattr(tab, '_find_tx_row'):
            try:
                row = tab._find_tx_row('0902')
            except Exception:
                row = -1
        if row is None or row < 0:
            for r in range(tbl.rowCount()):
                item = tbl.item(r, 0)
                if item and item.text().strip() == '0902':
                    row = r
                    break
        if row is None or row < 0:
            return False
        try:
            if hasattr(tab, '_on_tx_button_clicked'):
                tab._on_tx_button_clicked(row)
            else:
                tab._on_tx_double_clicked(row, 0)
            return True
        except Exception:
            try:
                tab._on_tx_double_clicked(row, 0)
                return True
            except Exception:
                return False

    def _install_mon_wires(self):
        """
        - TX 완료(mark_sent) 시 → {"kind":"tx","msg_id":"XXXX"} UDP 전송
        - 주기 TX(_log_only)도 동일 처리
        - 버튼 클릭 경로에서도 선제 통지(실패해도 무해)
        """
        tab = self._tab

        # (1) mark_sent 래핑 (TX 전송 알림)
        if hasattr(tab, "mark_sent"):
            self._orig_mark_sent = tab.mark_sent
            def _wrapped_mark_sent(msg_id: str, raw: bytes = None):
                try:
                    self._send_mon("tx", msg_id=_z4(str(msg_id)))
                except Exception:
                    pass
                return self._orig_mark_sent(msg_id, raw)
            tab.mark_sent = _wrapped_mark_sent  # type: ignore

        # (2) _log_only 래핑(주기 TX 로그 경로)
        if hasattr(tab, "_log_only"):
            self._orig_log_only = tab._log_only
            def _wrapped_log_only(row: int, msg_id: str, raw: bytes = None):
                try:
                    self._send_mon("tx", msg_id=_z4(str(msg_id)))
                except Exception:
                    pass
                return self._orig_log_only(row, msg_id, raw)
            tab._log_only = _wrapped_log_only  # type: ignore

        # (3) 버튼 클릭 경로에서도 선제 통지
        if hasattr(tab, "tbl_tx") and hasattr(tab, "_on_tx_button_clicked"):
            self._orig_tx_click_for_mon = tab._on_tx_button_clicked
            def _wrapped_click_for_mon(row: int):
                try:
                    it = getattr(tab, "tbl_tx").item(row, 0)
                    if it:
                        self._send_mon("tx", msg_id=_z4(it.text()))
                except Exception:
                    pass
                return self._orig_tx_click_for_mon(row)
            tab._on_tx_button_clicked = _wrapped_click_for_mon  # type: ignore

    def _send_mon(self, kind: str, **payload):
        """
        대시보드(run.py)가 수신하는 모듈별 모니터링 UDP(JSON).
        포트: KU_MON_MONITORING_PORT(기본 46982)
        kind: "tx" | "mode"
        """
        data = {"kind": str(kind), **payload}
        try:
            buf = json.dumps(data, ensure_ascii=False).encode("utf-8", "ignore")
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(buf, ("127.0.0.1", _mon_port()))
            s.close()
        except Exception:
            pass  # 모니터링용이므로 실패해도 동작에는 영향 없음

    # ───────── Power ON 시 0.5s 뒤 0102 5Hz 자동 시작 ─────────
    def _start_0102_stream(self):
        if not self._power_on:
            return
        try:
            self._tab.periodic_config['0102'] = 5  # 5Hz 강제
        except Exception:
            pass
        self._ensure_selfcheck_0102(True)

    # ───────── Power OFF 가드 설치(발신/우회 클릭 차단) ─────────
    def _install_power_gate_hooks(self):
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)

            # TX만 차단
            if tbl is not None:
                class _PG(QObject):
                    def __init__(self, host): super().__init__(host); self.host = host
                    def eventFilter(self, obj, ev):
                        if not self.host._power_on and ev.type() in (
                            QEvent.MouseButtonPress, QEvent.MouseButtonRelease,
                            QEvent.MouseButtonDblClick, QEvent.KeyPress, QEvent.KeyRelease
                        ):
                            return True
                        return False
                self._pg_filter = _PG(self)
                tbl.installEventFilter(self._pg_filter)

            # TX 버튼 우회만 차단
            if hasattr(tab, "_on_tx_button_clicked"):
                self._orig_tx_click = tab._on_tx_button_clicked
                def _wrapped_tx_click(row):
                    if not self._power_on:
                        self._append_log_line("[BLOCK] Power OFF → TX 버튼 무시")
                        return
                    # (선제 통지 유지)
                    try:
                        it = getattr(tab, "tbl_tx", None).item(row, 0) if getattr(tab, "tbl_tx", None) else None
                        if it: self._send_mon("tx", msg_id=_z4(it.text()))
                    except Exception: pass
                    return self._orig_tx_click(row)
                tab._on_tx_button_clicked = _wrapped_tx_click

        except Exception:
            pass

    def _apply_power_state(self):
        on = bool(self._power_on)
        try:
            self._update_tx_table_enabled(on)
            self._update_rx_table_enabled(True)  # ★ RX 테이블은 항상 활성 (UDP 미사용)
            if not on:
                self._stop_all_periodic()
        except Exception:
            pass

    def _update_tx_table_enabled(self, enabled: bool):
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)
            if tbl is None:
                return
            tbl.setEnabled(enabled)
            for r in range(tbl.rowCount()):
                w = tbl.cellWidget(r, 3)  # 전송 버튼 컬럼
                if w is not None and hasattr(w, "setEnabled"):
                    w.setEnabled(enabled)
        except Exception:
            pass

    def _update_rx_table_enabled(self, enabled: bool):
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_rx", None)
            if tbl is None:
                return
            tbl.setEnabled(enabled)
            cols = getattr(tbl, "columnCount", lambda: 4)()
            for r in range(tbl.rowCount()):
                for c in range(cols):
                    w = tbl.cellWidget(r, c)
                    if w is not None and hasattr(w, "setEnabled"):
                        w.setEnabled(enabled)
        except Exception:
            pass

    def _stop_all_periodic(self):
        try:
            tab = self._tab
            timers = getattr(tab, "periodic_timers", {})
            for code, t in list(timers.items()):
                try: t.stop()
                except Exception: pass
            try: timers.clear()
            except Exception: pass
            self._append_log_line("[POWER] periodic TX 정지")
        except Exception:
            pass

    def _append_log_line(self, text: str):
        try:
            # ⚠️ 불필요한 0101 파싱 실패 로그는 무시
            t = str(text)
            if "[MODE]" in t and "수신은 했으나 모드 파싱 실패" in t:
                return

            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text)
                return
        except Exception:
            pass
        try:
            print(text)
        except Exception:
            pass

    # ───────── 버스 초기화 ─────────
    def _rx_setup(self):
        FusionNodeIoc.Configure()
        NodeMessenger.Initialize("MSM_ReceiveNode")  # 🔧 기존 "MultiTopicReceiveNode" → 고유화
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()

    # ───────── 단발 0102 송신(폴백) ─────────
    def _send_self_check_0102(self, status: int = 1, _retry: int = 0):
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0102 폴백 차단")
            return
        try:
            from push_center import push_message
        except Exception as e:
            self._append_log_line(f"0102 push import 실패: {e}")
            return

        # 탭 오버라이드 우선(대소문자 키/고정값)
        try:
            body = self._tab._build_overridden_body("0102") or {}
        except Exception:
            body = {}
        # 실패 시 폴백(소문자 키)
        if not body:
            src = "MSM"
            try:
                st = self._tab._self_diag_status()
            except Exception:
                st = 0
            body = {"timestamp": _now_ms_since_2000(), "source": src, "status": st}

        try:
            push_message("0102", NodeMessenger, body_dict=body)
            self._append_log_line("자체점검(0102) 발신")
            self._self_check_sent = True
        except Exception as e:
            if _retry < 5:
                QTimer.singleShot(500, lambda: self._send_self_check_0102(status=status, _retry=_retry+1))
            else:
                self._append_log_line(f"자체점검(0102) 발신 실패: {e}")

    # ───────── UDP 컨트롤 수신 ─────────
    def _start_control_udp(self):
        """
        대시보드 제어 명령 수신 (기본 포트 45982)
        - 백그라운드 스레드 → ctrl_payload 시그널 emit
        """
        import socket, json, threading, os
        if getattr(self, "_ctrl_udp_started", False): return
        self._ctrl_udp_started = True

        port = int(os.getenv("KU_CTRL_PORT", "45982"))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", port))
            self._append_log_line(f"CTRL UDP 수신 대기 시작 (127.0.0.1:{port})")
        except Exception as e:
            self._append_log_line(f"CTRL UDP 바인드 실패: {e}")
            return

        def loop():
            while True:
                try:
                    data, _ = sock.recvfrom(8192)
                    payload = json.loads(data.decode("utf-8", "ignore"))
                    self.ctrl_payload.emit(payload)
                except Exception:
                    pass
        threading.Thread(target=loop, daemon=True).start()

    def _ensure_selfcheck_0102(self, on: bool) -> bool:
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0102 차단")
            return False
        try:
            tab = getattr(self, "_tab", None)
            if tab is None or not hasattr(tab, "tbl_tx"):
                self._append_log_line("[CTRL] 0102 대상 탭/테이블을 찾지 못함"); return False
            tbl = tab.tbl_tx
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0102":
                    target_row = r; break
            if target_row < 0:
                self._append_log_line("[CTRL] TX 테이블에 0102 행이 없음"); return False
            running = "0102" in getattr(tab, "periodic_timers", {})
            if (on and not running) or ((not on) and running):
                try:
                    btn = tbl.cellWidget(target_row, 3)
                    if btn is not None and hasattr(btn, "click"):
                        btn.click(); self._append_log_line(f"[CTRL] 0102 버튼 click() → {'ON' if on else 'OFF'} 요청"); return True
                except Exception: pass
                try:
                    if hasattr(tab, "_on_tx_button_clicked"):
                        tab._on_tx_button_clicked(target_row); self._append_log_line(f"[CTRL] 0102 토글 메서드 호출 → {'ON' if on else 'OFF'} 요청"); return True
                except Exception: pass
                self._send_self_check_0102(status=1 if on else 0); return True
            self._append_log_line(f"[CTRL] 0102 상태 유지: {'ON' if running else 'OFF'}"); return True
        except Exception as e:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {e}"); return False

    def _install_test_shortcuts(self):
        QShortcut(QKeySequence("1"), self, activated=lambda: self._ensure_selfcheck_0102(True))
        QShortcut(QKeySequence("0"), self, activated=lambda: self._ensure_selfcheck_0102(False))

    def _stage_replan_context(self, raw_context, trigger: str | None = None):
        if not isinstance(raw_context, dict):
            self._append_log_line('[CTRL] 0902 재계획 컨텍스트 준비 실패: 형식 오류')
            return

        plan_ids = []
        for val in raw_context.get('plan_ids', []):
            try: plan_ids.append(int(val))
            except Exception: continue

        mission_ids = []
        for val in raw_context.get('mission_ids', []):
            try: mission_ids.append(int(val))
            except Exception: continue

        option_names = []
        for name in raw_context.get('option_names', []):
            if name is None: continue
            option_names.append(str(name))
        while len(option_names) < len(plan_ids):
            option_names.append(f'옵션{len(option_names) + 1}')

        reason = str(raw_context.get('reason') or '초기임무계획')

        try: replan_level = int(raw_context.get('replan_level', 1))
        except Exception: replan_level = 1

        fallback_plan_id = raw_context.get('fallback_plan_id')
        try:
            fallback_plan_id = int(fallback_plan_id) if fallback_plan_id is not None else None
        except Exception:
            fallback_plan_id = None

        ctx = {
            'plan_ids': plan_ids,
            'mission_ids': mission_ids,
            'option_names': option_names[: len(plan_ids)],
            'reason': reason,
            'replan_level': replan_level,
        }
        if fallback_plan_id is not None:
            ctx['fallback_plan_id'] = fallback_plan_id

        self._staged_replan_context = ctx

        tab = getattr(self, '_tab', None)
        if tab and hasattr(tab, 'set_replan_context'):
            try:
                tab.set_replan_context(ctx)
            except Exception as exc:
                self._append_log_line(f'[CTRL] 0902 컨텍스트 적용 실패: {exc}')
            else:
                try:
                    row = tab._find_tx_row('0902')
                except Exception:
                    row = -1
                if row >= 0:
                    try:
                        tab.tbl_tx.selectRow(row)
                        state_item = tab.tbl_tx.item(row, 2)
                        if state_item is not None:
                            state_item.setText('준비')
                    except Exception:
                        pass

        summary = ', '.join(str(pid) for pid in plan_ids) or '-'
        prefix = '[AUTO]' if trigger == 'auto' else '[CTRL]'
        self._append_log_line(f'{prefix} 0902 재계획 요청 준비 완료 (planIds: {summary})')
        if trigger != 'auto':
            self._append_log_line('[GUIDE] 모니터링 탭에서 0902 버튼을 눌러 재계획 요청을 송신하세요.')

    # ───────── CTRL 핸들러 ─────────
    def _handle_ctrl_payload(self, payload: dict):
        import time
        try: cmd = str(payload.get("cmd") or "")
        except Exception: return
        key = f"{cmd}:{payload.get('text') or payload.get('status')}"
        now = time.monotonic(); last = self._last_ctrl_ts.get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        # OFF 시 mode 외 차단
        if not self._power_on and cmd not in ("mode",):
            self._append_log_line(f"[BLOCK] Power OFF → CTRL '{cmd}' 무시")
            return

        if cmd == "self_check":
            try: status = int(payload.get("status", 1))
            except Exception: status = 1
            ok = self._ensure_selfcheck_0102(on=(status == 1))
            if not ok: self._send_self_check_0102(status=status)

        elif cmd == "stage_replan":
            self._stage_replan_context(payload.get('context') or {}, trigger=payload.get('trigger'))
            return

        elif cmd == "replan":
            plan_ids = []
            for val in payload.get("planIds", []):
                try: plan_ids.append(int(val))
                except Exception: continue
            mission_ids = []
            for val in payload.get("missionIds", []):
                try: mission_ids.append(int(val))
                except Exception: continue
            option_names = list(payload.get("optionNames") or [])
            while len(option_names) < len(plan_ids):
                option_names.append(f"옵션{len(option_names) + 1}")
            context = {
                "plan_ids": plan_ids,
                "mission_ids": mission_ids,
                "option_names": option_names,
                "replan_level": int(payload.get("replanLevel", 1)),
            }
            if payload.get("fallbackPlanId") is not None:
                try: context["fallback_plan_id"] = int(payload.get("fallbackPlanId"))
                except Exception: pass
            reason = str(payload.get("reason") or "초기임무재계획")
            tab = getattr(self, "_tab", None)
            ok = False
            if tab and hasattr(tab, "send_replan_request"):
                try: ok = tab.send_replan_request(context, reason)
                except Exception as exc:
                    self._append_log_line(f"[CTRL] 0902 재계획 요청 실행 실패: {exc}")
                    ok = False
            if ok: self._append_log_line("[CTRL] 0902 재계획 요청 실행")
            else:  self._append_log_line("[CTRL] 0902 재계획 요청 실행 실패")

        elif cmd == "mode":
            text = str(payload.get("text") or "").strip() or "모드"
            self._append_log_line(f"[CTRL] 모드 변경 요청 수신: {text}")
            self._set_mode_slider_by_text(text)

    # ───────── Qt 이벤트 ─────────
    def closeEvent(self, event):

        try:

            manager = getattr(self, '_manager', None)

            if manager is not None:

                manager.shutdown()

        except Exception:

            pass

        try:

            super().closeEvent(event)

        except Exception:

            pass



    def showEvent(self, event):
        try: super().showEvent(event)
        except Exception: pass
        if getattr(self, "_shown_once", False): return
        self._shown_once = True
        self._append_log_line("SW 켜짐")

# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())


