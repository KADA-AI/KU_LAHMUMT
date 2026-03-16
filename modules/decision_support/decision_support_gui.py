# -*- coding: utf-8 -*-
# decision_support_gui.py – 의사결정 지원 전용 GUI
from __future__ import annotations

import sys, os, threading, re, json
from typing import Any
os.environ["KU_ROLE"] = "decision"  # MOB
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

ensure_console(os.getenv("KU_CONSOLE_TITLE", "KU Decision Support Console"))
install_process_file_logging("decision_support")

from PyQt5.QtCore import (
    qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer, Qt, QEvent, QObject, QRect
)
from PyQt5.QtGui import QKeySequence, QPainter, QColor, QFontMetrics, QFont
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QShortcut,
    QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSlider,
    QStyle, QStyleOptionSlider,
)

class ModeTickLabels(QWidget):
    def __init__(self, slider, labels, parent=None):
        super().__init__(parent)
        self._slider = slider
        self._labels = list(labels)
        self._pad = 0
        self._font = QFont("Malgun Gothic")
        self._font.setPointSize(8)
        metrics = QFontMetrics(self._font)
        self.setFixedHeight(max(30, metrics.height() * 2 + 2))
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._slider.installEventFilter(self)
        self._sync_width()

    def _calc_pad(self):
        metrics = QFontMetrics(self._font)
        max_width = 0
        for label in self._labels:
            lines = str(label).splitlines() or [""]
            width = max(metrics.horizontalAdvance(line) for line in lines)
            if width > max_width:
                max_width = width
        rect_width = max(max_width + 6, 24)
        return int(rect_width / 2) + 2

    def _sync_width(self):
        self._pad = self._calc_pad()
        self.setFixedWidth(self._slider.width() + self._pad * 2)
        self.update()

    def eventFilter(self, obj, event):
        if obj is self._slider and event.type() == QEvent.Resize:
            self._sync_width()
        return super().eventFilter(obj, event)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.TextAntialiasing)
        painter.setPen(QColor("#6b7280"))
        painter.setFont(self._font)
        metrics = QFontMetrics(self._font)
        option = QStyleOptionSlider()
        self._slider.initStyleOption(option)
        style = self._slider.style()
        groove = style.subControlRect(QStyle.CC_Slider, option, QStyle.SC_SliderGroove, self._slider)
        handle = style.subControlRect(QStyle.CC_Slider, option, QStyle.SC_SliderHandle, self._slider)
        min_val = self._slider.minimum()
        max_val = self._slider.maximum()
        count = len(self._labels)
        if count < 2 or max_val == min_val:
            return
        step = (max_val - min_val) / (count - 1)
        available = groove.width() - handle.width()
        if available < 0:
            available = 0
        for idx, label in enumerate(self._labels):
            val = min_val + int(round(step * idx))
            pos = style.sliderPositionFromValue(min_val, max_val, val, available, option.upsideDown)
            x = self._pad + groove.x() + (handle.width() // 2) + pos
            lines = str(label).splitlines() or [""]
            text_width = max(metrics.horizontalAdvance(line) for line in lines)
            rect_width = max(text_width + 6, 24)
            x0 = int(x - rect_width / 2)
            rect = QRect(int(x0), 0, int(rect_width), self.height())
            painter.drawText(rect, Qt.AlignHCenter | Qt.AlignTop, label)
        painter.end()



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
from modules.common import db_paths
from modules.common.fusion_files import copy_file_with_retry
from modules.common.option_codes import (
    DEFAULT_OPTION_CODE_SEQUENCE,
    normalize_option_code,
    option_code_to_label,
)
from receive_center import register_listener
from modules.decision_support.core import (
    OptionInfoMessenger,
    OptionPayloadBuilder,
    OptionRequestDecoder,
    SelfCheckMessenger,
    now_ms_since_2000,
)

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
        copy_file_with_retry(src, dst)

    lcands = [
        PROJECT_ROOT / "nFusionLicense.lic",
        COMMON_DIR   / "nFusionLicense.lic",
        PROJECT_ROOT / "nFusion" / "nFusionLicense.lic",
    ]
    lsrc = next((p for p in lcands if p.exists()), None)
    if lsrc:
        ldst = PROJECT_ROOT / "nFusionLicense.lic"
        if lsrc != ldst:
            copy_file_with_retry(lsrc, ldst)
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

try:
    from modules.common.generator import message0701_generator as _message0701
except Exception:
    _message0701 = None

def _z4(s: str) -> str:
    s = str(s).strip()
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s

_MODE_LABELS = [
    "초기화 모드",
    "대기모드",
    "초기 임무 계획",
    "임무 수행",
]

_MODE_TEXT_ALIASES = {
    "전원off": 0,
    "off": 0,
    "poweroff": 0,
    "전원on": 0,
    "on": 0,
    "poweron": 0,
    "0": 0,
    "초기화": 0,
    "초기화모드": 0,
    "초기화mode": 0,
    "1": 1,
    "대기모드": 1,
    "대기": 1,
    "standby": 1,
    "wait": 1,
    "2": 2,
    "초기임무계획": 2,
    "초기임무계획모드": 2,
    "initplan": 2,
    "initial": 2,
    "3": 3,
    "임무수행": 3,
    "임무수행모드": 3,
    "execution": 3,
    "run": 3,
}

_SLIDER_TO_SYSTEM_MODE = {0: 0, 1: 1, 2: 2, 3: 3}
_SYSTEM_MODE_TO_SLIDER = {code: slider for slider, code in _SLIDER_TO_SYSTEM_MODE.items()}
_MISSION_EXECUTE_CODE = 3

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
        self.setWindowTitle('의사결정(MOB)')
        # default footprint sized so multiple GUIs can coexist comfortably
        self.resize(1100, 700)

        # 전원/버스/디듀프 상태
        self._power_on = True
        self._bus_ready = False
        self._pending_option_entries: list[dict] = []
        self._last_ctrl_ts: dict[str, float] = {}
        self._last_0102_sent_ms = 0
        self._rx_counts = {}
        self._self_check_sent = False
        self._system_mode_code: int | None = None
        self._selfcheck_messenger = SelfCheckMessenger(NodeMessenger)
        self._option_messenger = OptionInfoMessenger(NodeMessenger)
        self._option_decoder = OptionRequestDecoder()
        self._option_builder = OptionPayloadBuilder(db_paths)

        # 탭
        tabs = QTabWidget()
        polish_tabs(tabs)
        self._tab = DecisionSupportTab(messenger=NodeMessenger, owner=self)

        # 0102 바디 오버라이드: 항상 MOB 고정형 생성
        self._tab._build_overridden_body = lambda mid: (
            {"Timestamp": now_ms_since_2000(), "Status": 1, "Source": "MOB"}
            if str(mid).strip() == "0102" else None
        )

        self._install_power_gate_hooks()
        tabs.addTab(self._tab, "의사결정지원 CSC")

        # 상단 모드 슬라이더
        top = QWidget()
        top.setObjectName("TopBar")
        top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(4, 2, 4, 2)
        top_layout.setSpacing(12)
        top_layout.addStretch(1)
        self.mode_slider = QSlider(Qt.Horizontal)
        self.mode_slider.setRange(0, 3); self.mode_slider.setSingleStep(1)
        self.mode_slider.setTickInterval(1); self.mode_slider.setTickPosition(QSlider.TicksBelow)
        self.mode_slider.setFixedWidth(420)
        self.mode_slider.valueChanged.connect(self._on_mode_slider_changed)
        slider_wrap = QWidget()
        slider_wrap.setObjectName("ModePanel")
        slider_layout = QVBoxLayout(slider_wrap)
        slider_layout.setContentsMargins(0, 0, 0, 0)
        slider_layout.setSpacing(2)
        slider_layout.addWidget(self.mode_slider, 0, Qt.AlignHCenter)
        self.mode_hint = ModeTickLabels(
            self.mode_slider,
            ["0\n초기화", "1\n대기", "2\n초기임무계획", "3\n임무수행"],
            slider_wrap,
        )
        slider_layout.addWidget(self.mode_hint, 0, Qt.AlignHCenter)
        self.mode_now = QLabel("초기화 모드")
        self.mode_now.setObjectName("ModeStatusLabel")
        self.mode_now.setFixedWidth(140)
        self.mode_now.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        lbl = QLabel("모드:")
        lbl.setObjectName("ModeCaptionLabel")
        top_layout.addWidget(lbl); top_layout.addWidget(slider_wrap); top_layout.addWidget(self.mode_now)

        center = QWidget()
        v = QVBoxLayout(center)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(10)
        v.addWidget(top); v.addWidget(tabs)
        self.setCentralWidget(center)

        # 초기 기동과 동시에 초기화 모드로 진입
        self._set_mode_slider_by_text("초기화 모드")
        self._apply_power_state()

        # 시그널
        self.ctrl_payload.connect(self._handle_ctrl_payload)

        # BUS 초기화
        threading.Thread(target=self._rx_setup, daemon=True).start()

        self._install_test_shortcuts()

        # GUI 표시 후 상태 OK(=1) 1회 송신
        QTimer.singleShot(2000, lambda: send_status_ok("MOB"))

        # BUS 준비되면(또는 약간의 지연 뒤) 0102 단발 보장 시도
        QTimer.singleShot(120, self._send_0102_when_ready)

        # ★★★ 0101 수신 → 모드 반영 리스너 & 폴링 보강
        self._install_option_request_listener()
        self._install_0101_mode_listener()
        self._start_0101_rx_poller()

    def _append_log_line(self, text: str):
        try:
            emit_process_log("decision_support", str(text))
        except Exception:
            pass
        try:
            if getattr(self, "_tab", None) and hasattr(self._tab, "append_log"):
                self._tab.append_log(text)
                return
        except Exception:
            pass
        try:
            print(text)
        except Exception:
            pass

    def _install_power_gate_hooks(self):
        """Power OFF 시 TX 동작을 차단."""
        try:
            tab = getattr(self, "_tab", None)
            tbl_tx = getattr(tab, "tbl_tx", None) if tab else None

            if tbl_tx is not None:
                class _PG(QObject):
                    def __init__(self, host):
                        super().__init__(host)
                        self.host = host
                    def eventFilter(self, obj, ev):
                        if not self.host._power_on and ev.type() in (
                            QEvent.MouseButtonPress, QEvent.MouseButtonRelease,
                            QEvent.MouseButtonDblClick, QEvent.KeyPress, QEvent.KeyRelease
                        ):
                            return True
                        return False
                self._pg_filter_tx = _PG(self)
                tbl_tx.installEventFilter(self._pg_filter_tx)

            if tab and hasattr(tab, "_on_tx_button_clicked"):
                self._orig_tx_click_for_gate = tab._on_tx_button_clicked
                def _wrapped_tx_click(row: int):
                    if not self._power_on:
                        self._append_log_line("[BLOCK] Power OFF → TX 버튼 무시")
                        return
                    return self._orig_tx_click_for_gate(row)
                tab._on_tx_button_clicked = _wrapped_tx_click
        except Exception:
            pass

    def _is_mission_execute_mode(self) -> bool:
        return self._system_mode_code == _MISSION_EXECUTE_CODE

    def _update_system_mode_state_from_slider(self, slider_value: int) -> None:
        try:
            slider_idx = int(slider_value)
        except Exception:
            slider_idx = -1
        code = _SLIDER_TO_SYSTEM_MODE.get(slider_idx)
        if code == self._system_mode_code:
            return
        self._system_mode_code = code
        if code == _MISSION_EXECUTE_CODE:
            QTimer.singleShot(0, self._flush_pending_option_entries)

    def _set_mode_slider_by_text(self, text: str):
        labels = _MODE_LABELS
        norm = re.sub(r"\s+", "", str(text)).lower()
        val = _MODE_TEXT_ALIASES.get(norm, 1)
        try:
            if hasattr(self, "mode_slider") and self.mode_slider.value() != val:
                self.mode_slider.blockSignals(True)
                self.mode_slider.setValue(val)
                self.mode_slider.blockSignals(False)
            if hasattr(self, "mode_now"):
                self.mode_now.setText(labels[val])
        except Exception:
            pass
        self._update_system_mode_state_from_slider(val)
        self._power_on = True
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)
            QTimer.singleShot(650, lambda: self._send_self_check_0102(status=1))
        else:
            self._self_check_sent = False
            self._ensure_selfcheck_0102(False)

    def _on_mode_slider_changed(self, val: int):
        labels = _MODE_LABELS
        try:
            self.mode_now.setText(labels[int(val)])
        except Exception:
            pass
        self._update_system_mode_state_from_slider(val)
        self._power_on = True
        label = labels[int(val)] if 0 <= val < len(labels) else str(val)
        self._append_log_line(f"[MODE] 모드 변경 → {label}")
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)
            QTimer.singleShot(650, lambda: self._send_self_check_0102(status=1))
        else:
            self._self_check_sent = False
            self._ensure_selfcheck_0102(False)

    def _apply_power_state(self):
        on = bool(self._power_on)
        try:
            self._update_tx_table_enabled(on)
            self._update_rx_table_enabled(True)
            if not on:
                self._stop_all_periodic()
                self._ensure_selfcheck_0102(False)
        except Exception:
            pass

    def _update_tx_table_enabled(self, enabled: bool):
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_tx", None)
            if tbl is None:
                return
            tbl.setEnabled(enabled)
            for r in range(tbl.rowCount()):
                w = tbl.cellWidget(r, 3)
                if w is not None and hasattr(w, "setEnabled"):
                    w.setEnabled(enabled)
        except Exception:
            pass

    def _update_rx_table_enabled(self, enabled: bool):
        try:
            tab = getattr(self, "_tab", None)
            tbl = getattr(tab, "tbl_rx", None)
            if tbl is None:
                return
            tbl.setEnabled(enabled)
        except Exception:
            pass

    def _stop_all_periodic(self):
        try:
            tab = getattr(self, "_tab", None)
            if tab and hasattr(tab, "stop_all_periodic"):
                tab.stop_all_periodic()
        except Exception:
            pass

    def _start_0102_stream(self, _retry: int = 0):
        if not self._power_on:
            return
        if not getattr(self, "_bus_ready", False):
            if _retry == 0:
                self._append_log_line("[0102] NodeMessenger 초기화 대기 중 – 자동 송신 보류")
            if _retry < 30:
                QTimer.singleShot(300, lambda r=_retry + 1: self._start_0102_stream(r))
            else:
                self._append_log_line("[WARN] NodeMessenger가 준비되지 않아 0102 자동 송신을 건너뜁니다.")
            return
        try:
            tab = getattr(self, "_tab", None)
            if tab is not None and hasattr(tab, "periodic_config"):
                tab.periodic_config["0102"] = 5
        except Exception:
            pass
        self._ensure_selfcheck_0102(True)

    def _ensure_selfcheck_0102(self, on: bool) -> bool:
        if on and not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0102 제어 차단")
            return False
        if on and not getattr(self, "_bus_ready", False):
            self._append_log_line("[WAIT] NodeMessenger 초기화 전 – 0102 ON 요청을 지연합니다.")
            QTimer.singleShot(300, self._start_0102_stream)
            return False
        try:
            tab = getattr(self, "_tab", None)
            if tab is None or not hasattr(tab, "tbl_tx"):
                if on:
                    self._append_log_line("[CTRL] 0102 대상 테이블을 찾지 못했습니다.")
                return False
            tbl = tab.tbl_tx
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == "0102":
                    target_row = r
                    break
            if target_row < 0:
                if on:
                    self._append_log_line("[CTRL] TX 테이블에 0102 행이 없습니다.")
                return False
            running = "0102" in getattr(tab, "periodic_timers", {})
            if (on and not running) or ((not on) and running):
                try:
                    btn = tbl.cellWidget(target_row, 3)
                    if btn is not None and hasattr(btn, "click"):
                        btn.click()
                        self._append_log_line(f"[CTRL] 0102 버튼 click() → {'ON' if on else 'OFF'} 요청")
                        return True
                except Exception:
                    pass
                try:
                    if hasattr(tab, "_on_tx_button_clicked"):
                        tab._on_tx_button_clicked(target_row)
                        self._append_log_line(f"[CTRL] 0102 토글 메서드 호출 → {'ON' if on else 'OFF'} 요청")
                        return True
                except Exception:
                    pass
                self._send_self_check_0102(status=1 if on else 0)
                return True
            self._append_log_line(f"[CTRL] 0102 상태 유지: {'ON' if running else 'OFF'}")
            return True
        except Exception as exc:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {exc}")
            return False

    def _handle_ctrl_payload(self, payload: dict):
        if not isinstance(payload, dict):
            return
        cmd = str(payload.get("cmd") or "").strip().lower()
        if cmd == "mode":
            text = str(payload.get("text") or "").strip()
            self._append_log_line(f"[CTRL] MODE change request: {text}")
            self._set_mode_slider_by_text(text)
            return
        if cmd == "self_check":
            try:
                status = int(payload.get("status", payload.get("value", 1)))
            except Exception:
                status = 1
            self._append_log_line(f"[CTRL] self_check status={status}")
            self._send_self_check_0102(status=status)
            return
        if cmd in ("power", "power_on", "poweroff"):
            status = str(payload.get("status") or payload.get("text") or "").lower()
            if payload.get("on") is not None:
                on = bool(payload.get("on"))
            else:
                on = status in ("1", "on", "true", "yes")
            self._power_on = on
            self._apply_power_state()
            self._append_log_line(f"[CTRL] POWER {'ON' if on else 'OFF'}")
            return
        self._append_log_line(f"[CTRL] {payload}")

    def _rx_setup(self):
        try:
            FusionNodeIoc.Configure()
            NodeMessenger.Initialize("MOB_ReceiveNode")
            NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
            NodeMessenger.InitAllSubscriberFromAssembly()
            NodeMessenger.RegistAllProviderFromFusionNodeIoc()
            self._bus_ready = True
            self._append_log_line("[BUS] MOB NodeMessenger 초기화 완료")
            self._flush_pending_option_entries()
        except Exception as exc:
            self._append_log_line(f"[WARN] RX setup 실패: {exc}")
            self._bus_ready = False

    def _install_test_shortcuts(self):
        try:
            QShortcut(QKeySequence("Ctrl+1"), self, activated=lambda: self._set_mode_slider_by_text("대기"))
            QShortcut(QKeySequence("Ctrl+2"), self, activated=lambda: self._set_mode_slider_by_text("임무 수행"))
        except Exception:
            pass

    def _send_0102_when_ready(self, _retry: int = 0):
        if not getattr(self, "_bus_ready", False):
            if _retry < 20:
                if _retry == 0:
                    self._append_log_line("[0102] 버스 초기화 전이라 0102 송신을 보류합니다.")
                QTimer.singleShot(250, lambda: self._send_0102_when_ready(_retry + 1))
                return
            self._append_log_line("[0102] 버스 준비 확인이 지연되어 강제 송신을 시도합니다.")
        self._send_self_check_0102(status=1)

    def _send_self_check_0102(self, status: int = 1, _retry: int = 0):
        if status == 1 and self._self_check_sent and _retry == 0:
            self._append_log_line("[0102] 상태 보고 이미 송신됨")
            return
        if status == 1 and not getattr(self, "_power_on", False):
            self._append_log_line("[BLOCK] Power OFF → 0102 송신 차단")
            return
        if not getattr(self, "_bus_ready", False) and _retry == 0:
            self._append_log_line("[0102] 버스 준비 전 상태에서 송신을 시도합니다.")
        ok = self._selfcheck_messenger.send(status=status)
        if not ok:
            err = getattr(self._selfcheck_messenger, "last_error", None)
            detail = f"{err}" if err else "unknown reason"
            self._append_log_line(f"[WARN] 0102 송신 실패: {detail}")
        if ok:
            self._self_check_sent = (status == 1)
            self._last_0102_sent_ms = now_ms_since_2000()
            self._append_log_line(f"[0102] 상태 보고 송신 (status={status})")
        else:
            if _retry < 5:
                QTimer.singleShot(500, lambda: self._send_self_check_0102(status=status, _retry=_retry + 1))
            else:
                self._append_log_line("[WARN] 0102 송신 재시도 한계 도달")

    # ───────── 0101 모드 수신 리스너 ─────────
    def _install_option_request_listener(self):
        """0901 옵션 요청 수신 시 0701을 생성하도록 리스너를 등록합니다."""
        class _Rx0901:
            def __init__(self, host):
                self.host = host
            def mark_received(self, msg_id: str, raw: bytes | None = None):
                try:
                    self.host._on_rx_0901(raw)
                except Exception:
                    pass
        try:
            self._rx0901 = _Rx0901(self)
            register_listener("0901", self._rx0901)
            self._append_log_line("[0901] 옵션 요청 리스너 등록 완료")
        except Exception as e:
            self._append_log_line(f"[0901] 리스너 등록 실패: {e}")

    def _on_rx_0901(self, raw: bytes | None):
        option_entries = self._option_decoder.decode(raw)
        if not option_entries:
            self._append_log_line("[0901] payload decode 실패")
            return
        self._latest_option_entries = option_entries
        if not self._bus_ready:
            self._append_log_line("[0701] NodeMessenger 초기화 대기 중 – 옵션 정보를 큐에 보관")
            self._pending_option_entries = list(option_entries)
            QTimer.singleShot(300, self._flush_pending_option_entries)
            return
        self._push_0701_from_entries(option_entries)

    def _flush_pending_option_entries(self):
        if not self._pending_option_entries:
            return
        if not self._bus_ready:
            QTimer.singleShot(300, self._flush_pending_option_entries)
            return
        entries = list(self._pending_option_entries)
        self._pending_option_entries = []
        self._push_0701_from_entries(entries)


    def _push_0701_from_entries(self, option_entries: list[dict]) -> None:
        entries: list[dict[str, Any]] = []
        defaults = list(DEFAULT_OPTION_CODE_SEQUENCE) or [1]
        for idx, entry in enumerate(option_entries):
            if not entry or "optionID" not in entry or "missionPlanID" not in entry:
                continue
            try:
                option_id = int(entry["optionID"])
                mission_plan_id = int(entry["missionPlanID"])
            except Exception:
                continue
            code = normalize_option_code(
                entry.get("optionName"),
                fallback=defaults[idx] if idx < len(defaults) else defaults[-1],
            )
            if code is None:
                code = defaults[-1]
            entries.append(
                {
                    "optionID": option_id,
                    "missionPlanID": mission_plan_id,
                    "optionName": code,
                    "optionLabel": option_code_to_label(code),
                }
            )
        if not entries:
            self._append_log_line("[0701] option 목록이 비어 전송 생략")
            return
        body = self._option_builder.build_body(entries, source="MOB")
        saved_path = self._option_builder.persist_body(body)
        if saved_path is not None:
            self._append_log_line(f"[0701] option info saved: {saved_path.name}")
        else:
            err = getattr(self._option_builder, "last_error", None)
            detail = f"{err}" if err else "unknown reason"
            self._append_log_line(f"[0701] option save skipped: {detail}")
        ok = self._option_messenger.send(body)
        if ok:
            self._append_log_line(f"[0701] option info sent (count={len(body.get('optionList') or [])})")
            try:
                raw_bytes = json.dumps(body, ensure_ascii=False).encode("utf-8")
                if getattr(self, "_tab", None) is not None:
                    self._tab.mark_sent(_z4("0701"), raw_bytes)
            except Exception:
                pass
        else:
            err = getattr(self._option_messenger, "last_error", None)
            detail = f"{err}" if err else "unknown reason"
            self._append_log_line(f"[0701] push failed: {detail}")

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
        내부 슬라이더(0~3): [0=초기화 모드, 1=대기모드, 2=초기 임무 계획, 3=임무 수행]
        매핑: 동일(0→0, 1→1, 2→2, 3→3)
        """
        slider_val = _SYSTEM_MODE_TO_SLIDER.get(code)
        if slider_val is None:
            return False
        try:
            self.mode_slider.blockSignals(True)
            self.mode_slider.setValue(slider_val)
            self.mode_slider.blockSignals(False)
            self._on_mode_slider_changed(slider_val)
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


# ───────── 엔트리 포인트 ─────────
def _main():
    app = QApplication(sys.argv)
    load_shared_stylesheet(app, PROJECT_ROOT)
    win = MainWindow()
    win.show()
    position_window_from_env(app, win)
    return app.exec_()


if __name__ == "__main__":
    sys.exit(_main())



