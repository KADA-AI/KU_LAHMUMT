# -*- coding: utf-8 -*-
# decision_support_gui.py – 의사결정 지원 전용 GUI
from __future__ import annotations

import sys, os, threading, re, json, time
os.environ["KU_ROLE"] = "decision"  # MOB
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

# ───────── 경로/모듈 부트스트랩 ─────────
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
from receive_center import register_listener

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
        "Source": "MOB",       # 의사결정 모듈명
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

        # 탭
        tabs = QTabWidget()
        self._tab = DecisionSupportTab(messenger=NodeMessenger)

        # 0102 바디 오버라이드: 항상 MOB 고정형 생성
        self._tab._build_overridden_body = lambda mid: (
            {"Timestamp": _now_ms_since_2000(), "Status": 1, "Source": "MOB"}
            if str(mid).strip() == "0102" else None
        )

        self._install_power_gate_hooks()
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

    # ───────── Power OFF 가드 ─────────
    def _install_power_gate_hooks(self):
        """
        Power OFF일 때:
         - TX 테이블 입력 차단(마우스/키)
         - 탭 TX 클릭 슬롯(_on_tx_button_clicked) 차단
         - 탭 수신 콜백(mark_received) 차단
         - 탭 RX 카운트(bump_rx) 차단
        """
        try:
            tab = self._tab
            tbl = getattr(tab, "tbl_tx", None)

            # (A) TX 테이블 입력 하드 차단
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

            # (B) TX 버튼 우회 호출 차단
            if hasattr(tab, "_on_tx_button_clicked"):
                self._orig_tx_click = tab._on_tx_button_clicked
                def _wrapped_tx_click(row):
                    if not self._power_on:
                        self._append_log_line("[BLOCK] Power OFF → TX 버튼 무시")
                        return
                    return self._orig_tx_click(row)
                tab._on_tx_button_clicked = _wrapped_tx_click

            # (C) RX 수신 콜백 차단
            if hasattr(tab, "mark_received"):
                self._orig_tab_mark_received = tab.mark_received
                def _wrapped_mark_received(msg_id, raw=None):
                    if not self._power_on:
                        return
                    return self._orig_tab_mark_received(msg_id, raw)
                tab.mark_received = _wrapped_mark_received

            # (D) RX 카운트 차단
            if hasattr(tab, "bump_rx"):
                self._orig_bump_rx = tab.bump_rx
                def _wrapped_bump_rx(mid):
                    if not self._power_on:
                        return
                    return self._orig_bump_rx(mid)
                tab.bump_rx = _wrapped_bump_rx

        except Exception:
            pass

    def _apply_power_state(self):
        on = bool(self._power_on)
        try:
            self._update_tx_table_enabled(on)
            self._update_rx_table_enabled(on)
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
        """주기 전송(0102 등) 정지."""
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

    # ───────── 0102 자동 스트림 (ON+0.5s, 5Hz) ─────────
    def _start_0102_stream(self):
        if not (self._power_on and self._bus_ready):
            return
        try:
            # 5Hz 보장
            self._tab.periodic_config['0102'] = 5
        except Exception:
            pass
        self._ensure_0102(True)

    def _send_0102_when_ready(self):
        """버스 준비 & 전원 ON일 때 0102 단발 보장(디듀프)."""
        if self._bus_ready and self._power_on:
            now_ms = int(time.time()*1000)
            if now_ms - self._last_0102_sent_ms > 300:
                ok = _push_0102_fixed(1)
                self._append_log_line("[AUTO] MOB 0102(Status=1) 발신" + ("" if ok else " (실패)"))
                self._last_0102_sent_ms = now_ms
        else:
            QTimer.singleShot(200, self._send_0102_when_ready)

    # ───────── BUS init ─────────
    def _rx_setup(self):
        try:
            FusionNodeIoc.Configure()
            NodeMessenger.Initialize("MultiTopicReceiveNode")
            NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
            NodeMessenger.InitAllSubscriberFromAssembly()
            NodeMessenger.RegistAllProviderFromFusionNodeIoc()
            self._bus_ready = True
        except Exception as e:
            self._append_log_line(f"[BUS] init 실패: {e}")
            self._bus_ready = False

    # ───────── 로깅 ─────────
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
        self._apply_power_state()
        if self._power_on:
            # ON 후 0.5s → 0102 5Hz 자동 시작
            QTimer.singleShot(500, self._start_0102_stream)
            # 단발 보장
            QTimer.singleShot(520, self._send_0102_when_ready)

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
            if getattr(self, "mode_slider", None) is not None:
                if self.mode_slider.value() != val:
                    self.mode_slider.blockSignals(True)
                    self.mode_slider.setValue(val)
                    self.mode_slider.blockSignals(False)
            if getattr(self, "mode_now", None) is not None:
                self.mode_now.setText(labels[val])
        except Exception:
            pass
        self._power_on = (int(val) != 0)
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)
            QTimer.singleShot(520, self._send_0102_when_ready)

    # ───────── UDP CTRL 수신 ─────────
    def _start_control_udp(self):
        """
        대시보드 제어 명령 수신 (기본 포트 45983)
        - 백그라운드 스레드 → ctrl_payload 시그널 emit
        """
        import socket, json
        if getattr(self, "_ctrl_udp_started", False): return
        self._ctrl_udp_started = True

        port = int(os.getenv("KU_CTRL_PORT", "45983"))
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

        key = f"{cmd}:{payload.get('text') or payload.get('status')}"
        now = time.monotonic(); last = self._last_ctrl_ts.get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        # Power OFF이면 모드 외 무시
        if not self._power_on and cmd not in ("mode",):
            self._append_log_line(f"[BLOCK] Power OFF → CTRL '{cmd}' 무시")
            return

        if cmd == "self_check":
            try: status = int(payload.get("status", 1))
            except Exception: status = 1
            if status == 1:
                self._send_0102_when_ready()
                self._start_0102_stream()
            else:
                if self._bus_ready:
                    _push_0102_fixed(0)
            return

        elif cmd == "mode":
            text = str(payload.get("text") or "").strip() or "모드"
            self._append_log_line(f"[CTRL] 모드 변경 요청 수신: {text}")
            self._set_mode_slider_by_text(text)

    # ───────── 0901 수신 → 0701 클릭 전송 ─────────
    def mark_received(self, msg_id: str, raw: bytes | None = None):
        if not self._power_on:
            return
        if str(msg_id).zfill(4) == "0901":
            mpid = None
            try:
                if raw:
                    txt = raw.decode("utf-8", "ignore")
                    m = re.search(r"\{.*\}", txt, flags=re.S)
                    if m:
                        obj = json.loads(m.group(0))
                        lst = obj.get("pendingOptionList") or []
                        if lst:
                            mpid = int(lst[0].get("missionPlanID") or 0)
            except Exception:
                pass
            if mpid:
                self._last_mission_plan_id = mpid
                setattr(self._tab, "_last_mission_plan_id", mpid)
            self._append_log_line(f"[AUTO] 0901 옵션요청 수신 → 0701 클릭 전송 (MPID={getattr(self,'_last_mission_plan_id', None)})")
            QTimer.singleShot(200, lambda: self._click_tx_button_for("0701"))

    # ───────── TX 클릭 유틸 ─────────
    def _click_tx_button_for(self, code: str):
        if not self._power_on:
            self._append_log_line(f"[BLOCK] Power OFF → TX '{code}' 차단")
            return
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
                    target_row = r; break
            if target_row < 0:
                self._append_log_line(f"[WARN] TX 테이블에 {code} 행이 없음"); return
            # (A) 버튼 click()
            try:
                btn = tbl.cellWidget(target_row, 3)
                if btn is not None and hasattr(btn, "click"):
                    btn.click(); self._append_log_line(f"[PUSH] {code} 버튼 click()"); return
            except Exception: pass
            # (B) 내부 핸들러 호출
            try:
                if hasattr(tab, "_on_tx_button_clicked"):
                    tab._on_tx_button_clicked(target_row); self._append_log_line(f"[PUSH] {code} 내부 핸들러 호출"); return
            except Exception: pass
            self._append_log_line(f"[ERR] {code} 푸시 실행 실패: 버튼/핸들러 접근 불가")
        except Exception as e:
            self._append_log_line(f"[ERR] {code} 푸시 실행 실패: {e}")

    # ───────── 단축키(테스트) ─────────
    def _install_test_shortcuts(self):
        QShortcut(QKeySequence("1"), self, activated=lambda: self._ensure_0102(True))
        QShortcut(QKeySequence("0"), self, activated=lambda: self._ensure_0102(False))

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
                self._append_log_line("[CTRL] TX 테이블에 0102 행이 없음")
                # 주기 토글 실패 시 단발로라도 발신
                if on and self._bus_ready:
                    _push_0102_fixed(1)
                    return True
                return False
            running = "0102" in getattr(tab, "periodic_timers", {})
            if (on and not running) or ((not on) and running):
                try:
                    btn = tbl.cellWidget(target_row, 3)
                    if btn is not None and hasattr(btn, "click"):
                        btn.click(); return True
                except Exception: pass
                try:
                    if hasattr(tab, "_on_tx_button_clicked"):
                        tab._on_tx_button_clicked(target_row); return True
                except Exception: pass
                # 마지막 폴백: 단발
                _push_0102_fixed(1 if on else 0); return True
            return True
        except Exception as e:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {e}")
            return False


# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())
