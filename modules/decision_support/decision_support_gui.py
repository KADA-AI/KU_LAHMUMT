# -*- coding: utf-8 -*-
# decision_support_gui.py – 의사결정 지원 전용 GUI
from __future__ import annotations

import sys, os, threading, re, json, time, socket
os.environ["KU_ROLE"] = "decision"  # MOB
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]  # .../KU_LAHMUMT
for _p in (_ROOT, _ROOT / "modules", _ROOT / "modules" / "common"):
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

# ───────── Qt 경고 필터 ─────────
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    sys.stderr.write(message + "\n")
qInstallMessageHandler(_qt_silent_handler)

# ───────── 경로/모듈 부트스트랩 ─────────
def _bootstrap_paths():
    here = Path(__file__).resolve()
    modules_dir = here.parents[1]                # .../modules
    root = modules_dir.parent                    # .../<project root>
    common_dir = modules_dir / "common"
    for p in (modules_dir / "decision_support", common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try: os.chdir(root)
    except Exception: pass
    return root, common_dir
PROJECT_ROOT, COMMON_DIR = _bootstrap_paths()

from modules.common.status_reporter import send_status_ok
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port
from receive_center import register_listener

_EPOCH2000_MS = 946_684_800_000
def _now_ms_since_2000() -> int:
    return int(time.time() * 1000) - _EPOCH2000_MS

# ───────── nFusion 설정/라이선스 + MessageLibrary 로드 ─────────
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
from Tabs.decision_support_tab import DecisionSupportTab

# ───────── 모듈별 모니터링 포트(의사결정) ─────────
def _mon_port() -> int:
    """의사결정 GUI → 대시보드(run.py) 모니터링 전송 포트"""
    try:
        return int(os.getenv("KU_MON_DECISION_PORT", "46983"))
    except Exception:
        return 46983

def _z4(s: str) -> str:
    s = str(s).strip()
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s

# ───────── 고정 0102 PUSH (단발/폴백 용) ─────────
def _push_0102_fixed(status: int = 1):
    """버스 준비 이후 MOB/Status 고정 바디 단발 0102."""
    try:
        from push_center import push_message
    except Exception as e:
        try: sys.stderr.write(f"[0102] push import 실패: {e}\n")
        except Exception: pass
        return False
    body = {
        "Timestamp": _now_ms_since_2000(),
        "Status": int(status),           # 0/1/2
        "Source": "MOB",                 # 의사결정 모듈명
    }
    try:
        return bool(push_message("0102", NodeMessenger, body_dict=body))
    except Exception as e:
        try: sys.stderr.write(f"[0102] push 실패: {e}\n")
        except Exception: pass
        return False


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle("의사결정지원 GUI")
        self.resize(1100, 700)

        # 전원/버스/디듀프 상태
        self._power_on = False
        self._bus_ready = False
        self._last_ctrl_ts: dict[str, float] = {}
        self._last_0102_sent_ms = 0
        self._rx_counts = {}

        # 탭
        tabs = QTabWidget()
        self._tab = DecisionSupportTab(messenger=NodeMessenger, owner=self)

        # 0102 바디 오버라이드: 항상 MOB 고정형 생성
        self._tab._build_overridden_body = lambda mid: (
            {"Timestamp": _now_ms_since_2000(), "Status": 1, "Source": "MOB"}
            if str(mid).strip() == "0102" else None
        )

        self._install_power_gate_hooks()
        self._install_mon_wires()   # ★ 모니터링 전송 훅 설치
        tabs.addTab(self._tab, "의사결정지원 CSC")

        # 상단 모드 슬라이더
        top = QWidget(); top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(8,4,8,4); top_layout.addStretch(1)
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

        # 초기 전원 OFF로 시작
        self._set_mode_slider_by_text("전원 OFF")
        self._apply_power_state()

        # 시그널
        self.ctrl_payload.connect(self._handle_ctrl_payload)

        # BUS 초기화
        threading.Thread(target=self._rx_setup, daemon=True).start()

        # CTRL 수신(기본 45983)
        self._start_control_udp()
        self._install_test_shortcuts()

        # GUI 표시 후 상태 OK(=1) 1회 송신
        QTimer.singleShot(800, lambda: send_status_ok("MOB"))

        # 외부 self_check=1 수신 시 상태 OK 재송신
        start_ctrl_listener(env_ctrl_port(45983),
            lambda p: (send_status_ok("MOB")
                       if (p or {}).get("cmd")=="self_check" and int((p or {}).get("status",0))==1 else None)
        )

        # BUS 준비되면(또는 약간의 지연 뒤) 0102 단발 보장 시도
        QTimer.singleShot(120, self._send_0102_when_ready)

        # ★★★ 0101 수신 → 모드 반영 리스너 & 폴링 보강
        self._install_0101_mode_listener()
        self._start_0101_rx_poller()

    # ───────── 0101 모드 수신 리스너 ─────────
    def _install_0101_mode_listener(self):
        """
        receive_center.notify('0101', raw)를 직접 수신해
        systemMode 숫자코드를 슬라이더로 반영.
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

    # ───────── 0101 RAW 처리: systemMode 숫자 → 슬라이더 반영 ─────────
    def _on_rx_0101(self, raw: bytes | None):
        # 1) RAW → 텍스트
        txt = (raw or b"").decode("utf-8", "ignore")
        # 2) JSON 블록 추출
        m = re.search(r"\{.*\}", txt, flags=re.S)
        jtxt = m.group(0) if m else txt.strip()
        # 3) 파싱
        try:
            body = json.loads(jtxt) if jtxt.startswith("{") else {}
        except Exception:
            body = {}

        # 4) 코드 추출(여러 키/형식 허용)
        code = self._extract_mode_code(body)
        if code is None:
            mm = re.search(r'"systemMode"\s*:\s*([0-9]+)', txt)
            if mm:
                try: code = int(mm.group(1))
                except Exception: code = None
        if code is None:
            return

        # 5) 적용
        self._apply_system_mode_code(code)

    def _extract_mode_code(self, body: dict) -> int | None:
        """
        다양한 키에서 모드코드 추출(대/소문자, str/bool/float 안전).
        """
        if not isinstance(body, dict):
            return None
        low = {str(k).lower(): body[k] for k in body.keys() if k is not None}
        for key in ("systemmode", "mode", "modecode", "state"):
            if key in low:
                v = low[key]
                if isinstance(v, bool):
                    return 1 if v else 0
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
        외부 0101 systemMode 정의:
          0 : 초기화 모드
          1 : 대기 모드
          2 : 초기임무계획 모드
          3 : 임무수행 모드
        내부 슬라이더(0~4): [0=전원 OFF, 1=전원 ON, 2=대기모드, 3=초기 임무 계획, 4=임무 수행]
        매핑: 0→1, 1→2, 2→3, 3→4
        """
        code_to_slider = {0: 1, 1: 2, 2: 3, 3: 4}
        if code not in code_to_slider:
            return False
        val = code_to_slider[code]
        try:
            self.mode_slider.blockSignals(True)
            self.mode_slider.setValue(val)
            self.mode_slider.blockSignals(False)
            self._on_mode_slider_changed(val)
            return True
        except Exception:
            return False

    # ───────── RX 테이블 폴링 기반 0101 모드 반영(보강용) ─────────
    def _start_0101_rx_poller(self):
        """탭의 RX 테이블 UserRole에 저장된 0101 RAW를 주기적으로 확인해 모드 반영."""
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
            payload = item.data(Qt.UserRole) if item else None
            if tab and hasattr(tab, "_latest_payload_bytes"):
                raw_latest = tab._latest_payload_bytes(payload)
            else:
                raw_latest = payload if isinstance(payload, (bytes, bytearray)) else b""
            if not raw_latest:
                return
            if self._last_0101_raw is not None and raw_latest == self._last_0101_raw:
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
                    try:
                        code = int(mm.group(1))
                    except Exception:
                        code = None

            if code is not None:
                self._apply_system_mode_code(code)
                self._last_0101_raw = raw_latest
        except Exception:
            pass
