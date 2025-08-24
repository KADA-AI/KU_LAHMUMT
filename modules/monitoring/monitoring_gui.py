# -*- coding: utf-8 -*-
# monitoring_gui.py – 임무 모니터링·판단 전용 GUI
from __future__ import annotations

import sys, os, threading
os.environ["KU_ROLE"] = "monitoring"
from pathlib import Path

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QShortcut, QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSlider
from PyQt5.QtGui import QKeySequence

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
    for p in (modules_dir / "monitoring", common_dir, root):
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


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    ctrl_payload = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("임무 모니터링·판단 GUI")
        self.resize(1100, 700)
        self._self_check_sent = False
        self._last_ctrl_ts = {}   # 디듀프용

        tabs = QTabWidget()
        self._tab = MissionMonitoringTab(messenger=NodeMessenger)
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
        self._set_mode_slider_by_text("전원 OFF")

        self.ctrl_payload.connect(self._handle_ctrl_payload)
        threading.Thread(target=self._rx_setup, daemon=True).start()
        # UDP 컨트롤 수신 시작(포커스/최소화 무관)
        self._start_control_udp()
        self._install_test_shortcuts()

    def _append_log_line(self, text: str):
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text); return
        except Exception:
            pass
        try:
            print(text)
        except Exception:
            pass



    # ───────── 모드/슬라이더 유틸 ─────────
    def _sw_code(self) -> str:
        role = (os.environ.get("KU_ROLE") or "").lower()
        return {"mission":"MMR","monitoring":"MSM","decision":"MOB"}.get(role, "MMR")

    def _on_mode_slider_changed(self, val: int):
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        try: self.mode_now.setText(labels[int(val)])
        except Exception: pass
        self._append_log_line(f"[MODE] 슬라이더 변경 → {labels[int(val)] if 0 <= val < len(labels) else val}")

    def _set_mode_slider_by_text(self, text: str):
        import re
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        norm = re.sub(r"\s+", "", str(text)).lower()  # "전원 OFF" → "전원off"
        mapping = {
            "전원off": 0, "off": 0, "poweroff": 0, "0": 0,
            "전원on":  1, "on": 1,  "poweron": 1,  "1": 1,
            "대기모드": 2, "대기": 2, "standby": 2, "2": 2,
            "초기임무계획": 3, "초기임무계획모드": 3, "initplan": 3, "initial": 3, "3": 3,
            "임무수행": 4, "execution": 4, "4": 4,
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
        except Exception:
            pass

    # ───────── 버스 초기화 ─────────
    def _rx_setup(self):
        FusionNodeIoc.Configure()
        NodeMessenger.Initialize("MultiTopicReceiveNode")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()

    # ───────── 단발 0102 송신(폴백) ─────────
    def _send_self_check_0102(self, status: int = 1, _retry: int = 0):
        try:
            from push_center import push_message
        except Exception as e:
            self._append_log_line(f"0102 push import 실패: {e}")
            return

        # 탭에서 표준 바디 생성 시도
        try:
            body = self._tab._build_overridden_body("0102") or {}
        except Exception:
            body = {}
        # 실패 시 폴백: 2000 epoch + 내부 점검 결과 사용
        if not body:
            import os
            role = (os.environ.get("KU_ROLE") or "").lower()
            src = {"mission": "MMR", "monitoring": "MSM", "decision": "MOB"}.get(role, "MMR")
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

    def showEvent(self, event):
        try: super().showEvent(event)
        except Exception: pass
        if getattr(self, "_shown_once", False): return
        self._shown_once = True
        self._append_log_line("SW 켜짐")

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
        # (동일: 0102 행 찾아 버튼 click / 내부 토글 / 폴백 push)
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
            ok = self._ensure_selfcheck_0102(on=(status == 1))
            if not ok: self._send_self_check_0102(status=status)
        elif cmd == "mode":
            text = str(payload.get("text") or "").strip() or "모드"
            self._append_log_line(f"[CTRL] 모드 변경 요청 수신: {text}")
            self._set_mode_slider_by_text(text)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())