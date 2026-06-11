# -*- coding: utf-8 -*-
# info.py – 정보관리(INF) 전용 GUI
from __future__ import annotations

import sys, os, threading, json, re, time
os.environ["KU_ROLE"] = "info"  # INF
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]  # .../KU_LAHMUMT
for _p in (_ROOT, _ROOT / "modules", _ROOT / "modules" / "common"):
    _ps = str(_p)
    if _p.exists() and _ps not in sys.path:
        sys.path.insert(0, _ps)
from modules.common.qt_env import ensure_qt_platform
ensure_qt_platform()
from modules.common.gui_style import load_shared_stylesheet, polish_tabs, position_window_from_env
from modules.common.process_console import emit_process_log, ensure_console, install_process_file_logging

ensure_console(os.getenv("KU_CONSOLE_TITLE", "KU Info Manage Console"))
install_process_file_logging("info_manage")

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
from modules.common.status_reporter import send_status_ok
from modules.common import db_paths
from modules.common.settings_paths import (
    ensure_fusion_license_file,
    ensure_fusion_settings_file,
    fusion_runtime_working_dir,
)
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port
from modules.common.gui_process_control import (
    apply_initial_visibility,
    env_flag,
    handle_window_control,
    hide_instead_of_close,
)

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
    dst = ensure_fusion_settings_file(project_root=PROJECT_ROOT, common_dir=COMMON_DIR)
    if dst is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json 이 없습니다.")
    ensure_fusion_license_file(project_root=PROJECT_ROOT, common_dir=COMMON_DIR)
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
    def _already_loaded(exc: Exception) -> bool:
        return "already loaded" in str(exc).lower()

    try:
        _clr.AddReference(str(stem))
    except Exception as exc:
        if not _already_loaded(exc):
            try:
                _clr.AddReference(str(stem.with_suffix(".dll")))
            except Exception as exc2:
                if not _already_loaded(exc2):
                    raise
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
        self.setWindowTitle('정보관리 모듈')
        # default footprint sized so multiple GUIs can coexist comfortably
        self.resize(1100, 700)

        # 파워/상태
        self._power_on = True
        self._self_check_sent = False
        self._last_ctrl_ts = {}   # 디듀프
        self._viewer_only = env_flag("KU_VIEWER_ONLY", False)

        tabs = QTabWidget()
        polish_tabs(tabs)
        self._tab = ManageInfo(messenger=NodeMessenger)

        # 0102 바디 고정 오버라이드: Timestamp/Status/Source(INF)
        self._tab._build_overridden_body = lambda mid: (
            {"Timestamp": _now_ms_since_2000(), "Status": 1, "Source": "INF"}
            if str(mid).strip() == "0102" else None
        )

        self._install_power_gate_hooks()  # Power OFF 가드
        tabs.addTab(self._tab, "정보관리 모듈")

        self.mode_slider = None
        self.mode_now = None
        self._current_mode_code = 0
        center = QWidget(self)
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(12, 12, 12, 12)
        center_layout.setSpacing(10)
        center_layout.addWidget(tabs)
        self.setCentralWidget(center)

        # 초기 실행 시 곧바로 초기화 모드로 설정
        self._set_mode_slider_by_text("초기화 모드")
        self._apply_power_state()

        # 신호 연결
        self.ctrl_payload.connect(self._handle_ctrl_payload)
        self.log_sig.connect(self._append_log_line)

        # CTRL(UDP) listener: dashboard broadcast -> UI thread
        self._ctrl_thread = None
        try:
            port = env_ctrl_port(45984)
            self._ctrl_thread = start_ctrl_listener(port, lambda payload: self.ctrl_payload.emit(payload))
            self._append_log_line(f"[CTRL] listener started @ 127.0.0.1:{port}")
        except Exception as e:
            self._append_log_line(f"[CTRL] listener start failed: {e}")

        if not self._viewer_only:
            # BUS 초기화 + 테스트 단축키
            threading.Thread(target=self._rx_setup, daemon=True).start()
        self._install_test_shortcuts()
        self._init_db_root_sync()

        if self._viewer_only:
            self._append_log_line("[VIEWER] passive info GUI; service owns 0101/0102")
        else:
            # GUI 표시 후 status=1 한 번 송신
            QTimer.singleShot(2000, lambda: send_status_ok("INF"))

        # CTRL 리스너: self_check ON → status=1 송신
    # ───────── 모니터링(대시보드) 전송 훅 ─────────
    def _start_0102_stream(self):
        if not self._power_on:
            return
        self._append_log_line("[INFO] 0102 periodic transmission disabled for INF mock mode")

    def _init_db_root_sync(self) -> None:
        self._db_root = None
        self._refresh_db_root(log_first=True)
        self._db_root_timer = QTimer(self)
        self._db_root_timer.setInterval(1000)
        self._db_root_timer.timeout.connect(self._refresh_db_root)
        self._db_root_timer.start()

    def _refresh_db_root(self, log_first: bool = False) -> None:
        try:
            root = db_paths.peek_active_db_root()
        except Exception as exc:
            self._append_log_line(f"[PATH] DB root check failed: {exc}")
            return
        if root is None:
            return
        root_str = str(root)
        if not root.exists():
            self._db_root = root_str
            return
        if log_first or root_str != getattr(self, "_db_root", None):
            self._db_root = root_str
            self._append_log_line(f"[PATH] DB root -> {root_str}")

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
            emit_process_log("info_manage", str(text))
        except Exception:
            pass
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text); return
        except Exception:
            pass
        try: print(text)
        except Exception: pass

    # ───────── 모드/슬라이더 ─────────
    def _on_mode_slider_changed(self, val: int):
        labels = ["초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]
        try: self.mode_now.setText(labels[int(val)])
        except Exception: pass
        try:
            self._current_mode_code = int(val)
        except Exception:
            self._current_mode_code = 0
        self._power_on = True
        self._append_log_line(f"[MODE] 슬라이더 변경 → {labels[int(val)] if 0 <= val < len(labels) else val}")
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    def _resolve_mode_code_from_text(self, text: str) -> int:
        norm = re.sub(r"\s+", "", str(text)).lower()
        mapping = {
            "전원off":0,"off":0,"poweroff":0,
            "전원on":0,"on":0,"poweron":0,
            "0":0,
            "초기화":0,"초기화모드":0,"초기화mode":0,
            "1":1,"대기모드":1,"대기":1,"standby":1,
            "2":2,"초기임무계획":2,"초기임무계획모드":2,"initplan":2,"initial":2,
            "3":3,"임무수행":3,"execution":3,
        }
        return int(mapping.get(norm, 1))

    def _should_ignore_ctrl_mode_change(self, requested_code: int) -> bool:
        if int(getattr(self, "_current_mode_code", 0)) == 3 and int(requested_code) != 3:
            self._append_log_line(
                f"[MODE] CTRL mode change ignored during execution: requested={requested_code}"
            )
            return True
        return False

    def _set_mode_slider_by_text(self, text: str):
        labels = ["초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]
        val = self._resolve_mode_code_from_text(text)
        try:
            if getattr(self, "mode_slider", None):
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True)
                    self.mode_slider.setValue(val)
                    self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None):
                self.mode_now.setText(labels[val])
            # ★ 텍스트로 모드 세팅될 때도 모니터링 통지
        except Exception:
            pass
        self._current_mode_code = int(val)
        self._power_on = True
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    # ───────── BUS 초기화 ─────────
    def _rx_setup(self):
        with fusion_runtime_working_dir(project_root=PROJECT_ROOT):
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
        status = 1  # 0102 Status는 항상 1
        body = {"Timestamp": _now_ms_since_2000(), "Status": status, "Source": "INF"}
        try:
            push_message("0102", NodeMessenger, body_dict=body)
            self._append_log_line("자체점검(0102) 발신")
            self._self_check_sent = True
        except Exception as e:
            if _retry < 5:
                QTimer.singleShot(500, lambda: self._send_self_check_0102(status=status, _retry=_retry+1))
            else:
                self._append_log_line(f"자체점검(0102) 발신 실패: {e}")

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
                try:
                    if hasattr(tab, "send_tx_row"):
                        ok = bool(tab.send_tx_row(target_row, interactive=False))
                        self._append_log_line(f"[CTRL] 0102 direct handler → {'ON' if on else 'OFF'} 요청")
                        return ok
                except Exception:
                    pass
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
                self._send_self_check_0102(); return True
            self._append_log_line(f"[CTRL] 0102 상태 유지: {'ON' if running else 'OFF'}"); return True
        except Exception as e:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {e}"); return False

    # ───────── CTRL 처리(UI 스레드) ─────────
    def _handle_ctrl_payload(self, payload: dict):
        if handle_window_control(self, payload, role="info", log=self._append_log_line):
            return
        import time
        try: cmd = str(payload.get("cmd") or "").lower()
        except Exception: return
        if cmd == "system_mode":
            token = payload.get("mode")
        elif cmd == "mode":
            token = payload.get("text")
        else:
            token = payload.get("status")
        if getattr(self, "_viewer_only", False) and cmd not in (
            "show_window",
            "show_gui",
            "open_gui",
            "raise_window",
            "hide_window",
            "hide_gui",
            "db_root",
            "debug_db_root",
            "log_db_root",
            "mode",
        ):
            self._append_log_line(f"[VIEWER] CTRL ignored: {payload}")
            return
        key = f"{cmd}:{token}"
        now = time.monotonic(); last = self._last_ctrl_ts.get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        if not self._power_on and cmd not in ("mode", "db_root", "debug_db_root", "log_db_root"):
            self._append_log_line(f"[BLOCK] Power OFF → CTRL '{cmd}' 무시")
            return

        if cmd == "self_check":
            try: status = int(payload.get("status", 1))
            except Exception: status = 1
            ok = self._ensure_0102(on=(status == 1))
            if not ok:
                self._send_self_check_0102()

        elif cmd == "system_mode":
            try:
                mode = int(payload.get("mode", 2))
            except Exception:
                mode = 2
            self._append_log_line(f"[CTRL] 시스템모드(0101) 전송 요청 수신: {mode}")

            # 0101 실제 발신
            self._send_system_mode_0101(mode)

            # UI도 함께 맞춰주기(대시보드 모드 표시/펄스를 위해)
            label_map = {0:"초기화 모드", 1:"대기모드", 2:"초기 임무 계획", 3:"임무 수행"}
            self._set_mode_slider_by_text(label_map.get(mode, "대기모드"))
            return
        
        elif cmd == "mode":
            text = str(payload.get("text") or "").strip() or "모드"
            if self._should_ignore_ctrl_mode_change(self._resolve_mode_code_from_text(text)):
                return
            self._append_log_line(f"[CTRL] 모드 변경 요청 수신: {text}")
            self._set_mode_slider_by_text(text)

        elif cmd in ("db_root", "debug_db_root", "log_db_root"):
            self._refresh_db_root(log_first=True)

    def _send_system_mode_0101(self, mode: int = 2):
        """
        SystemMode(0101) 메시지 발신:
        - Timestamp: ms since 2000
        - SystemMode: int (0~3)
        - Source: 'INF'
        """
        # Primary path: send exactly like GUI operation on 0101 row.
        try:
            mode = int(mode)
        except Exception:
            mode = 2
        try:
            tab = getattr(self, "_tab", None)
            row = int(tab._find_tx_row("0101")) if tab and hasattr(tab, "_find_tx_row") else -1
            if row >= 0 and hasattr(tab, "_send_system_mode"):
                combo = getattr(tab, "_mode_combo", None)
                if combo is not None:
                    for i in range(combo.count()):
                        try:
                            if int(combo.itemData(i)) == mode:
                                if combo.currentIndex() != i:
                                    combo.setCurrentIndex(i)
                                break
                        except Exception:
                            continue
                tab._send_system_mode(row, mode)
                self._append_log_line(f"시스템운용모드(0101) GUI 전송: {mode}")
                return
        except Exception as e:
            self._append_log_line(f"[WARN] 0101 GUI 경로 실패 -> direct push fallback: {e}")

        # Fallback path: direct push
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

    def closeEvent(self, event):
        if hide_instead_of_close(self, event, log=self._append_log_line):
            return
        super().closeEvent(event)

# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    load_shared_stylesheet(app, PROJECT_ROOT)
    win = MainWindow()
    apply_initial_visibility(app, win, position_window_from_env)
    sys.exit(app.exec_())


