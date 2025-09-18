# -*- coding: utf-8 -*-
# info.py – 정보관리(INF) 전용 GUI
from __future__ import annotations

import sys, os, threading, json, re, time, socket
os.environ["KU_ROLE"] = "info"  # INF
from pathlib import Path

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

# ───────── 경로 부트스트랩 ─────────
_ROOT = Path(__file__).resolve().parents[2]  # .../KU_LAHMUMT
for _p in (_ROOT, _ROOT / "modules", _ROOT / "modules" / "common"):
    _ps = str(_p)
    if _p.exists() and _ps not in sys.path:
        sys.path.insert(0, _ps)

from modules.common.status_reporter import send_status_ok
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port

_EPOCH2000_MS = 946_684_800_000
def _now_ms_since_2000() -> int:
    return int(time.time() * 1000) - _EPOCH2000_MS

# ───────── 로컬 부트스트랩(정보관리 경로로 교정) ─────────
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

from receive import *  # noqa
from Tabs.manage_info_tab import ManageInfo

# ───────── 모듈별 모니터링 포트(정보관리) ─────────
def _mon_port() -> int:
    """정보관리 GUI → 대시보드(run.py) 모니터링 전송 포트"""
    try:
        return int(os.getenv("KU_MON_INFO_PORT", "46984"))
    except Exception:
        return 46984

def _z4(s: str) -> str:
    s = str(s).strip()
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    # 백그라운드 → UI 스레드용
    ctrl_payload = pyqtSignal(dict)
    log_sig      = pyqtSignal(str)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle("정보관리 GUI")
        self.resize(1100, 700)

        # 파워/상태
        self._power_on = False
        self._self_check_sent = False
        self._last_ctrl_ts = {}   # 디듀프

        tabs = QTabWidget()
        self._tab = ManageInfo(messenger=NodeMessenger)

        # 0102 바디 고정 오버라이드: Timestamp/Status/Source(INF)
        self._tab._build_overridden_body = lambda mid: (
            {"Timestamp": _now_ms_since_2000(), "Status": 1, "Source": "INF"}
            if str(mid).strip() == "0102" else None
        )

        self._install_power_gate_hooks()  # Power OFF 가드
        self._install_mon_wires()         # ★ 모니터링 전송 훅
        tabs.addTab(self._tab, "정보관리 CSC")

        # ───── 상단 슬라이더 바 ─────
        top = QWidget()
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(8,4,8,4); top_layout.addStretch(1)
        self.mode_slider = QSlider(Qt.Horizontal)
        self.mode_slider.setRange(0,4)
        self.mode_slider.setSingleStep(1)
        self.mode_slider.setTickInterval(1)
        self.mode_slider.setTickPosition(QSlider.TicksBelow)
        self.mode_slider.setFixedWidth(420)
        self.mode_slider.valueChanged.connect(self._on_mode_slider_changed)
        self.mode_now = QLabel("대기모드"); self.mode_now.setStyleSheet("font-weight:600; padding-left:8px;")
        lbl = QLabel("모드:"); lbl.setStyleSheet("color:#789; padding-right:6px;")
        top_layout.addWidget(lbl); top_layout.addWidget(self.mode_slider); top_layout.addWidget(self.mode_now)

        center = QWidget(); v = QVBoxLayout(center); v.setContentsMargins(0,0,0,0)
        v.addWidget(top); v.addWidget(tabs)
        self.setCentralWidget(center)

        # 초기 전원 OFF 적용
        self._set_mode_slider_by_text("전원 OFF")
        self._apply_power_state()

        # 신호 연결
        self.ctrl_payload.connect(self._handle_ctrl_payload)
        self.log_sig.connect(self._append_log_line)

        # BUS 초기화 + UDP 컨트롤 리스너
        threading.Thread(target=self._rx_setup, daemon=True).start()
        self._start_control_udp()
        self._install_test_shortcuts()

        # GUI 표시 후 status=1 한 번 송신
        QTimer.singleShot(800, lambda: send_status_ok("INF"))

        # CTRL 리스너: self_check ON → status=1 송신
        def _on_ctrl(payload: dict):
            try:
                if (payload or {}).get("cmd") == "self_check" and int((payload or {}).get("status", 0)) == 1:
                    send_status_ok("INF")
            except Exception:
                pass
        start_ctrl_listener(env_ctrl_port(45984), _on_ctrl)

    # ───────── 모니터링(대시보드) 전송 훅 ─────────
    def _install_mon_wires(self):
        """
        - TX 완료(mark_sent) 시 → {"kind":"tx","msg_id":"XXXX"} UDP 전송
        - 주기 TX(_log_only)도 동일 처리
        - 버튼 클릭 경로에서도 선제 통지(실패해도 무해)
        - 모드 변경 시점에서 {"kind":"mode","text":..., "role":"INF"} 전송(아래 슬라이더 핸들러에서 수행)
        """
        tab = self._tab

        # (1) mark_sent 래핑
        if hasattr(tab, "mark_sent"):
            self._orig_mark_sent = tab.mark_sent
            def _wrapped_mark_sent(msg_id: str, raw: bytes | None = None):
                try: self._send_mon("tx", msg_id=_z4(str(msg_id)))
                except Exception: pass
                return self._orig_mark_sent(msg_id, raw)
            tab.mark_sent = _wrapped_mark_sent  # type: ignore

        # (2) _log_only 래핑(주기 전송 로그 경로)
        if hasattr(tab, "_log_only"):
            self._orig_log_only = tab._log_only
            def _wrapped_log_only(row: int, msg_id: str, raw: bytes | None):
                try: self._send_mon("tx", msg_id=_z4(str(msg_id)))
                except Exception: pass
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
        포트: KU_MON_INFO_PORT(기본 46984)
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

    # ───────── 전원 ON 시 0.5s 뒤 0102 5Hz 자동 시작 ─────────
    def _start_0102_stream(self):
        if not self._power_on:
            return
        try:
            # 5Hz 보장 (CSCTabBase의 periodic_config 사용)
            self._tab.periodic_config['0102'] = 5
        except Exception:
            pass
        self._ensure_0102(True)

    # ───────── Power OFF 가드(발신/수신/카운트/우회 클릭 차단) ─────────
    def _install_power_gate_hooks(self):
        """
        Power OFF 시 TX만 막음. RX는 항상 통과.
        """
        try:
            tab = self._tab
            tbl_tx = getattr(tab, "tbl_tx", None)

            # TX 입력만 차단
            if tbl_tx is not None:
                class _PG(QObject):
                    def __init__(self, host): super().__init__(host); self.host = host
                    def eventFilter(self, obj, ev):
                        if not self.host._power_on and ev.type() in (
                            QEvent.MouseButtonPress, QEvent.MouseButtonRelease,
                            QEvent.MouseButtonDblClick, QEvent.KeyPress, QEvent.KeyRelease
                        ):
                            return True
                        return False
                self._pg_filter_tx = _PG(self)
                tbl_tx.installEventFilter(self._pg_filter_tx)

            # TX 버튼 우회만 차단
            if hasattr(tab, "_on_tx_button_clicked"):
                self._orig_tx_click = tab._on_tx_button_clicked
                def _wrapped_tx_click(row):
                    if not self._power_on:
                        self._append_log_line("[BLOCK] Power OFF → TX 버튼 무시")
                        return
                    try:
                        it = getattr(tab, "tbl_tx", None).item(row, 0) if getattr(tab, "tbl_tx", None) else None
                        if it: self._send_mon("tx", msg_id=_z4(it.text()))
                    except Exception: pass
                    return self._orig_tx_click(row)
                tab._on_tx_button_clicked = _wrapped_tx_click

            # ✘ 제거: tbl_rx 이벤트 필터, mark_received/bump_rx 차단 래퍼
        except Exception:
            pass

    def _apply_power_state(self):
        on = bool(self._power_on)
        try:
            self._update_tx_table_enabled(on)
            self._update_rx_table_enabled(True)  # ★ RX는 항상 활성
            if not on:
                self._stop_all_periodic()
        except Exception:
            pass

    def _update_tx_table_enabled(self, enabled: bool):
        """TX 테이블 및 전송 버튼 활성/비활성."""
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
        """RX 테이블 및 셀 위젯(버튼 등) 활성/비활성."""
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
        """주기 전송(0102 등) 일괄 중지."""
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

    # ───────── 유틸 ─────────
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
        self._power_on = (int(val) != 0)
        self._append_log_line(f"[MODE] 슬라이더 변경 → {labels[int(val)] if 0 <= val < len(labels) else val}")
        # ★ 대시보드에도 모드 통지
        try: self._send_mon("mode", text=labels[int(val)], role="INF")
        except Exception: pass
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    def _set_mode_slider_by_text(self, text: str):
        labels = ["전원 OFF", "전원 ON", "대기모드", "초기 임무 계획", "임무 수행"]
        norm = re.sub(r"\s+", "", str(text)).lower()
        mapping = {
            "전원off":0,"off":0,"poweroff":0,"0":0,
            "전원on":1,"on":1,"poweron":1,"1":1,
            "대기모드":2,"대기":2,"standby":2,"2":2,
            "초기임무계획":3,"초기임무계획모드":3,"initplan":3,"initial":3,"3":3,
            "임무수행":4,"execution":4,"4":4
        }
        val = mapping.get(norm, 2)
        try:
            if getattr(self, "mode_slider", None):
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True)
                    self.mode_slider.setValue(val)
                    self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None):
                self.mode_now.setText(labels[val])
            # ★ 텍스트로 모드 세팅될 때도 모니터링 통지
            self._send_mon("mode", text=labels[val], role="INF")
        except Exception:
            pass
        self._power_on = (int(val) != 0)
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    # ───────── BUS 초기화 ─────────
    def _rx_setup(self):
        FusionNodeIoc.Configure()
        NodeMessenger.Initialize("INF_ReceiveNode")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()

    # ───────── 0102 폴백(직접 push) ─────────
    def _send_self_check_0102(self, status: int = 1, _retry: int = 0):
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0102 폴백 차단")
            return
        try:
            from push_center import push_message
        except Exception as e:
            self._append_log_line(f"0102 push import 실패: {e}")
            return
        # 오버라이드가 실패한 경우 대비 폴백(대소문자 고정)
        body = {"Timestamp": _now_ms_since_2000(), "Status": int(status), "Source": "INF"}
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
        대시보드 제어 명령 수신 (기본 포트 45984)
        - 백그라운드 스레드 → ctrl_payload 시그널 emit
        """
        import socket, json, os
        if getattr(self, "_ctrl_udp_started", False): return
        self._ctrl_udp_started = True

        port = int(os.getenv("KU_CTRL_PORT", "45984"))
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

    # ───────── 테스트 단축키 ─────────
    def _install_test_shortcuts(self):
        QShortcut(QKeySequence("1"), self, activated=lambda: self._ensure_0102(True))
        QShortcut(QKeySequence("0"), self, activated=lambda: self._ensure_0102(False))

    # ───────── 0102 ON/OFF 보장 ─────────
    def _ensure_0102(self, on: bool) -> bool:
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0102 차단")
            return False
        try:
            tab = self._tab; tbl = tab.tbl_tx
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0102":
                    target_row = r; break
            if target_row < 0:
                self._append_log_line("[CTRL] TX 테이블에 0102 행이 없음"); return False
            running = "0102" in getattr(tab, "periodic_timers", {})
            if (on and not running) or ((not on) and running):
                # (A) 버튼 click()
                try:
                    btn = tbl.cellWidget(target_row, 3)
                    if btn is not None and hasattr(btn, "click"):
                        btn.click(); self._append_log_line(f"[CTRL] 0102 버튼 click() → {'ON' if on else 'OFF'} 요청"); return True
                except Exception: pass
                # (B) 내부 핸들러
                try:
                    if hasattr(tab, "_on_tx_button_clicked"):
                        tab._on_tx_button_clicked(target_row); self._append_log_line(f"[CTRL] 0102 토글 메서드 호출 → {'ON' if on else 'OFF'} 요청"); return True
                except Exception: pass
                # (C) 폴백 1회 push
                self._send_self_check_0102(status=1 if on else 0); return True
            self._append_log_line(f"[CTRL] 0102 상태 유지: {'ON' if running else 'OFF'}"); return True
        except Exception as e:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {e}"); return False

    # ───────── CTRL 처리(UI 스레드) ─────────
    def _handle_ctrl_payload(self, payload: dict):
        import time
        try: cmd = str(payload.get("cmd") or "").lower()
        except Exception: return
        key = f"{cmd}:{payload.get('text') or payload.get('status')}"
        now = time.monotonic(); last = self._last_ctrl_ts.get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        if not self._power_on and cmd not in ("mode",):
            self._append_log_line(f"[BLOCK] Power OFF → CTRL '{cmd}' 무시")
            return

        if cmd == "self_check":
            try: status = int(payload.get("status", 1))
            except Exception: status = 1
            ok = self._ensure_0102(on=(status == 1))
            if not ok:
                self._send_self_check_0102(status=status)

        elif cmd == "system_mode":
            try:
                mode = int(payload.get("mode", 2))
            except Exception:
                mode = 2
            self._append_log_line(f"[CTRL] 시스템모드(0101) 전송 요청 수신: {mode}")

            # 0101 실제 발신
            self._send_system_mode_0101(mode)

            # UI도 함께 맞춰주기(대시보드 모드 표시/펄스를 위해)
            label_map = {0:"전원 OFF", 1:"전원 ON", 2:"대기모드", 3:"초기 임무 계획", 4:"임무 수행"}
            self._set_mode_slider_by_text(label_map.get(mode, "대기모드"))
            return
        
        elif cmd == "mode":
            text = str(payload.get("text") or "").strip() or "모드"
            self._append_log_line(f"[CTRL] 모드 변경 요청 수신: {text}")
            self._set_mode_slider_by_text(text)

    def _send_system_mode_0101(self, mode: int = 2):
        """
        SystemMode(0101) 메시지 발신:
        - Timestamp: ms since 2000
        - SystemMode: int (0~4)
        - Source: 'INF'
        """
        try:
            from push_center import push_message
            body = {
                "Timestamp": _now_ms_since_2000(),
                "SystemMode": int(mode),
                "Source": "INF",
            }
            push_message("0101", NodeMessenger, body_dict=body)
            self._append_log_line(f"시스템운용모드(0101) 발신: {mode}")
        except Exception as e:
            self._append_log_line(f"[ERR] 0101 push 실패: {e}")

# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())
