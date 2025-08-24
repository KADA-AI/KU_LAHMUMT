# -*- coding: utf-8 -*-
# monitoring_gui.py – 임무 모니터링·판단 전용 GUI
from __future__ import annotations

import sys, os, threading
os.environ["KU_ROLE"] = "info_management"
from pathlib import Path

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QShortcut
from PyQt5.QtGui import QKeySequence

# ───────── Qt 경고 필터 ─────────
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    sys.stderr.write(message + "\n")

qInstallMessageHandler(_qt_silent_handler)

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
from Tabs.manage_info_tab import ManageInfo


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    # 백그라운드 UDP → UI 스레드 안전 디스패치
    ctrl_payload = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("정보관리 GUI")
        self.resize(1100, 700)

        self._self_check_sent = False
        self._last_ctrl_ts = {}   # 디듀프용

        tabs = QTabWidget()
        self._tab = ManageInfo(messenger=NodeMessenger)
        tabs.addTab(self._tab, "정보관리 CSC")
        self.setCentralWidget(tabs)

        # UDP payload는 반드시 UI 스레드에서 처리
        self.ctrl_payload.connect(self._handle_ctrl_payload)

        # 버스 초기화는 백그라운드에서
        threading.Thread(target=self._rx_setup, daemon=True).start()

        # UDP 컨트롤 수신 시작(포커스/최소화 무관)
        self._start_control_udp()

        # 테스트 단축키(1: ON, 0: OFF)
        self._install_test_shortcuts()

    # ───────── 로깅 유틸 ─────────
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

    # ───────── 버스 초기화 ─────────
    def _rx_setup(self):
        FusionNodeIoc.Configure()
        NodeMessenger.Initialize("MultiTopicReceiveNode")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()

    # ───────── 단발 0102 송신(폴백용) ─────────
    def _send_self_check_0102(self, status: int = 1, _retry: int = 0):
        try:
            from push_center import push_message
        except Exception as e:
            self._append_log_line(f"0102 push import 실패: {e}")
            return

        body = {"Status": int(status), "SourceModuleName": "Mission State Monitor"}
        try:
            push_message("0102", NodeMessenger, body_dict=body)
            if not self._self_check_sent:
                self._append_log_line("자체점검(0102) 발신")
                self._self_check_sent = True
        except Exception as e:
            if _retry < 5:
                QTimer.singleShot(500, lambda: self._send_self_check_0102(status=status, _retry=_retry+1))
            else:
                self._append_log_line(f"자체점검(0102) 발신 실패: {e}")

    # ───────── 최초 표시 ─────────
    def showEvent(self, event):
        try:
            super().showEvent(event)
        except Exception:
            pass
        if getattr(self, "_shown_once", False):
            return
        self._shown_once = True
        self._append_log_line("SW 켜짐")
        # 자동 자체점검 발신 없음

    # ───────── UDP 컨트롤 수신 ─────────
    def _start_control_udp(self):
        """
        대시보드 제어 명령 수신 (기본 포트 45982)
        - 백그라운드 스레드 → ctrl_payload 시그널 emit으로 UI 스레드 처리 보장
        """
        import socket, json, threading, os

        if getattr(self, "_ctrl_udp_started", False):
            return
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
                    self.ctrl_payload.emit(payload)  # ← UI 스레드로 큐잉
                except Exception:
                    pass

        threading.Thread(target=loop, daemon=True).start()

    # ───────── 0102 토글 보장(실제 버튼 클릭) ─────────
    def _ensure_selfcheck_0102(self, on: bool) -> bool:
        """
        TX 테이블에서 msg_id == '0102' 행을 찾아 실제 '발신' 버튼을 클릭해 ON/OFF 보장.
        우선순위: (A) 셀 위젯 버튼 click() → (B) 내부 토글 메서드 호출 → (C) push_center 폴백
        """
        try:
            tab = getattr(self, "_tab", None)
            if tab is None or not hasattr(tab, "tbl_tx"):
                self._append_log_line("[CTRL] 0102 대상 탭/테이블을 찾지 못함")
                return False

            tbl = tab.tbl_tx
            # 1) 0102 행 찾기
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0102":
                    target_row = r
                    break
            if target_row < 0:
                self._append_log_line("[CTRL] TX 테이블에 0102 행이 없음")
                return False

            # 2) 현재 주기 전송 상태 확인
            running = "0102" in getattr(tab, "periodic_timers", {})

            # 필요 상태와 다르면 실제 '발신' 버튼을 클릭해 토글
            if (on and not running) or ((not on) and running):
                # (A) 셀 위젯 버튼 click() 시도
                try:
                    btn = tbl.cellWidget(target_row, 3)   # 3번 컬럼이 '발신' 버튼
                    if btn is not None and hasattr(btn, "click"):
                        btn.click()
                        self._append_log_line(f"[CTRL] 0102 버튼 click() → {'ON' if on else 'OFF'} 요청")
                        return True
                except Exception:
                    pass
                # (B) 내부 토글 메서드 직접 호출(버튼과 동일 경로)
                try:
                    if hasattr(tab, "_on_tx_button_clicked"):
                        tab._on_tx_button_clicked(target_row)
                        self._append_log_line(f"[CTRL] 0102 토글 메서드 호출 → {'ON' if on else 'OFF'} 요청")
                        return True
                except Exception:
                    pass
                # (C) 최후 폴백: 직접 1회 push (상태 토글은 못하지만 발신 보장)
                self._send_self_check_0102(status=1 if on else 0)
                return True

            # 이미 원하는 상태면 유지
            self._append_log_line(f"[CTRL] 0102 상태 유지: {'ON' if running else 'OFF'}")
            return True

        except Exception as e:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {e}")
            return False

    # ───────── 테스트 단축키 ─────────
    def _install_test_shortcuts(self):
        """
        테스트용 단축키:
          - '1'  → TX 테이블의 0102 주기발신 ON
          - '0'  → TX 테이블의 0102 주기발신 OFF
        """
        QShortcut(QKeySequence("1"), self, activated=lambda: self._ensure_selfcheck_0102(True))
        QShortcut(QKeySequence("0"), self, activated=lambda: self._ensure_selfcheck_0102(False))

    # ───────── 대시보드 명령 처리(UI 스레드) ─────────
    def _handle_ctrl_payload(self, payload: dict):
        """
        - cmd == 'self_check': status(1/0)에 따라 0102 발신을 ON/OFF 보장
        - cmd == 'mode'      : 텍스트 로그
        - NOTE: 1.0s 디듀프(동일 cmd/status 연속 수신 방지)
        """
        import time
        try:
            cmd = str(payload.get("cmd") or "").lower()
        except Exception:
            cmd = ""

        key = f"{cmd}|{payload.get('status')}"
        now = time.monotonic()
        last = self._last_ctrl_ts.get(key, 0.0)
        if (now - last) < 1.0:
            return
        self._last_ctrl_ts[key] = now

        if cmd == "self_check":
            try:
                status = int(payload.get("status", 1))
            except Exception:
                status = 1
            ok = self._ensure_selfcheck_0102(on=(status == 1))
            if not ok:
                self._send_self_check_0102(status=status)
        elif cmd == "mode":
            text = str(payload.get("text") or "").strip() or "모드"
            self._append_log_line(f"[CTRL] 모드 변경 요청 수신: {text}")


# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())
