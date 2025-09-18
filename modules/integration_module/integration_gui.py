# -*- coding: utf-8 -*-
# modules/integration_module/integration_gui.py
# Integration GUI – 연동 모듈 전용 GUI
from __future__ import annotations

import sys, os, threading, json, re, time
os.environ["KU_ROLE"] = "integration"
from pathlib import Path

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer, Qt, QEvent, QObject
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSlider

# ───────── Qt 경고 필터 ─────────
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    sys.stderr.write(message + "\n")
qInstallMessageHandler(_qt_silent_handler)

# ───────── 경로 부트스트랩 ─────────
_ROOT = Path(__file__).resolve().parents[2]  # .../KU_LAHMUMT
for _p in (_ROOT, _ROOT / "modules", _ROOT / "modules" / "common"):
    _ps = str(_p)
    if _p.exists() and _ps not in sys.path:
        sys.path.insert(0, _ps)

def _bootstrap_paths():
    here = Path(__file__).resolve()
    modules_dir = here.parents[1]                # .../modules
    root = modules_dir.parent                    # .../<project root>
    common_dir = modules_dir / "common"
    for p in (modules_dir / "integration_module", common_dir, root):
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

# 공용 수신 등록 (버스 핸들러들 로드)
from receive import *  # noqa

# 탭
from Tabs.integration_tab import IntegrationTab

# 상태 OK(=0102) 헬퍼
from modules.common.status_reporter import send_status_ok
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port


class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)  # 백그라운드 → UI 스레드

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle("연동 모듈 GUI")
        self.resize(1100, 700)

        self._power_on = False
        self._last_ctrl_ts = {}

        # ── 중앙 탭
        tabs = QTabWidget()
        self._tab = IntegrationTab(messenger=NodeMessenger)
        tabs.addTab(self._tab, "Integration CSC")

        # ── 상단 모드 슬라이더
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

        # 초기 OFF
        self._set_mode_slider_by_text("전원 OFF")
        self._apply_power_state()

        # 버스 초기화 + CTRL 리스너 + UDP 제어 수신
        self.ctrl_payload.connect(self._handle_ctrl_payload)
        threading.Thread(target=self._rx_setup, daemon=True).start()
        self._start_control_udp()

        # GUI 표시 후 상태 OK 송신(모듈 코드: INT)
        QTimer.singleShot(800, lambda: send_status_ok("INT"))

        # 외부 self_check=1 수신 시에도 상태 OK 송신
        start_ctrl_listener(env_ctrl_port(45985), lambda payload: (
            send_status_ok("INT") if (payload or {}).get("cmd") == "self_check" and int((payload or {}).get("status", 0)) == 1 else None
        ))

        # Power OFF 시 UI 입력 차단 가드 설치
        self._install_power_gate_hooks()

    # ───────── Power Gate ─────────
    def _install_power_gate_hooks(self):
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)
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
        except Exception:
            pass

    def _apply_power_state(self):
        on = bool(self._power_on)
        try:
            self._update_tx_enabled(on)
            if not on:
                self._stop_all_periodic()
        except Exception:
            pass

    def _update_tx_enabled(self, enabled: bool):
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)
            if tbl is None:
                return
            tbl.setEnabled(enabled)
            for r in range(tbl.rowCount()):
                w = tbl.cellWidget(r, 3)   # '발신' 버튼 컬럼
                if w is not None and hasattr(w, "setEnabled"):
                    w.setEnabled(enabled)
        except Exception:
            pass

    def _stop_all_periodic(self):
        try:
            timers = getattr(self._tab, "periodic_timers", {})
            for _, t in list(timers.items()):
                try: t.stop()
                except Exception: pass
            try: timers.clear()
            except Exception: pass
        except Exception:
            pass

    # ───────── 모드/슬라이더 ─────────
    def _on_mode_slider_changed(self, val: int):
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        try: self.mode_now.setText(labels[int(val)])
        except Exception: pass
        self._power_on = (int(val) != 0)
        self._apply_power_state()

    def _set_mode_slider_by_text(self, text: str):
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        norm = re.sub(r"\s+", "", str(text)).lower()
        mapping = {
            "전원off":0,"off":0,"poweroff":0,"0":0,
            "전원on":1,"on":1,"poweron":1,"1":1,
            "대기모드":2,"대기":2,"standby":2,"2":2,
            "초기임무계획":3,"초기임무계획모드":3,"initplan":3,"initial":3,"3":3,
            "임무수행":4,"execution":4,"4":4,
        }
        val = mapping.get(norm, 2)
        try:
            if getattr(self, "mode_slider", None):
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True); self.mode_slider.setValue(val); self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None): self.mode_now.setText(labels[val])
        except Exception: pass
        self._power_on = (int(val) != 0)

    # ───────── 버스 초기화 ─────────
    def _rx_setup(self):
        try:
            FusionNodeIoc.Configure()
            NodeMessenger.Initialize("INT_ReceiveNode")
            NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
            NodeMessenger.InitAllSubscriberFromAssembly()
            NodeMessenger.RegistAllProviderFromFusionNodeIoc()
        except Exception as e:
            self._append_log_line(f"[BUS] init 실패: {e}")

    # ───────── UDP 제어 수신 ─────────
    def _start_control_udp(self):
        import socket
        if getattr(self, "_ctrl_udp_started", False): return
        self._ctrl_udp_started = True

        port = int(os.getenv("KU_CTRL_PORT", "45985"))
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

    # ───────── CTRL 처리 ─────────
    def _handle_ctrl_payload(self, payload: dict):
        try: cmd = str(payload.get("cmd") or "")
        except Exception: return

        # 간단 디듀프
        key = f"{cmd}:{payload.get('text') or payload.get('status')}"
        now = time.monotonic(); last = getattr(self, "_last_ctrl_ts", {}).get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        if not self._power_on and cmd not in ("mode",):
            self._append_log_line(f"[BLOCK] Power OFF → CTRL '{cmd}' 무시")
            return

        if cmd == "self_check":
            try:
                status = int(payload.get("status", 1))
            except Exception:
                status = 1
            if status == 1:
                send_status_ok("INT")
        elif cmd == "mode":
            text = str(payload.get("text") or "").strip()
            self._append_log_line(f"[CTRL] MODE change request: {text}")
            self._set_mode_slider_by_text(text)

    # ───────── 로깅 ─────────
    def _append_log_line(self, text: str):
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text); return
        except Exception:
            pass
        try: print(text)
        except Exception: pass


# ───────── 엔트리 ─────────
if __name__ == "__main__":
    from PyQt5.QtWidgets import QLabel, QHBoxLayout, QVBoxLayout  # noqa: F401  (위에서 사용)
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())
