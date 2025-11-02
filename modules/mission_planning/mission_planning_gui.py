# -*- coding: utf-8 -*-
# mission_planning_gui.py – 임무 할당·계획수립 전용 GUI (S110 플로우 대응)
from __future__ import annotations

import sys, os, threading, json, re, time, shutil, socket
os.environ["KU_ROLE"] = "mission"  # MMR
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
    QWidget, QLabel, QHBoxLayout, QVBoxLayout, QSlider, QPushButton
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
from modules.common.ctrl_listener import start_ctrl_listener, env_ctrl_port
from modules.common import db_paths
from receive_center import register_listener   # ★ 0101 모드 수신 리스너

_EPOCH2000_MS = 946_684_800_000
def _now_ms_since_2000() -> int:
    return int(time.time() * 1000) - _EPOCH2000_MS

# ───────── nFusion 설정/라이선스 정규화 + MessageLibrary 로드 ─────────
def _bootstrap_paths():
    here = Path(__file__).resolve()
    modules_dir = here.parents[1]
    root = modules_dir.parent
    common_dir = modules_dir / "common"
    for p in (modules_dir / "mission_planning", common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try: os.chdir(root)
    except Exception: pass
    return root, common_dir
PROJECT_ROOT, COMMON_DIR = _bootstrap_paths()

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

from dll_files.nFusionImports import *  # FusionNodeIoc, NodeMessenger, clr 등

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

# 수신 등록 모듈(내부에서 각 탭의 RECEIVE 등록을 수행)
from receive import *  # noqa

# 탭
from Tabs.assignment_planning_tab import AssignmentPlanningTab
from id_relationship_tab import IdRelationshipTab

# ───────── 모듈별 모니터링 포트(임무계획/MMR) ─────────
def _mon_port() -> int:
    """임무계획 GUI → 대시보드(run.py) 모니터링 전송 포트"""
    try:
        return int(os.getenv("KU_MON_ASSIGNMENT_PORT", "46981"))
    except Exception:
        return 46981

def _z4(s: str) -> str:
    s = str(s).strip()
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s


# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    # 백그라운드 → UI 스레드용 신호
    ctrl_payload   = pyqtSignal(dict)   # UDP 제어
    log_sig        = pyqtSignal(str)    # 로그
    start_push_seq = pyqtSignal()       # 0301/0305/0901 순차 푸시 트리거

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle('임무계획(MMR)')
        self.resize(1100, 700)

        # 파워/상태
        self._power_on = False
        self._self_check_sent = False
        self._last_ctrl_ts = {}     # 디듀프
        self._rx_counts = {}

        # 파이프라인 컨텍스트
        self._initplan_running = False
        self._last_mission_plan_id = None
        self._last_mission_plan_ids = []
        self._staged_plan_context: dict = {}
        self._active_plan_context: dict = {}
        self._pending_plan_push: dict | None = None
        self._scheduled_0301_plan_ids: list[int] = []
        self._replan_delay_timer: QTimer | None = None
        self._option_id_counter = 0

        # ── 중앙 탭(AssignmentPlanningTab)
        tabs = QTabWidget()
        self._tab = AssignmentPlanningTab(messenger=NodeMessenger)
        self._tab.set_replan_callback(self._handle_replan_received)

        self._tab._build_overridden_body = lambda mid: (
            {"Timestamp": _now_ms_since_2000(), "Status": 1, "Source": "MMR"}
            if str(mid).strip() == "0102" else None
        )

        self._install_power_gate_hooks()       # Power OFF 가드
        self._install_mon_wires()              # ★ 모니터링 전송 훅
        self._install_0301_override()          # 0301 전송 커스텀
        tabs.addTab(self._tab, "임무 할당·계획수립 CSC")
        self._id_tab = IdRelationshipTab()
        tabs.addTab(self._id_tab, "ID Explorer")

        # ── 상단 모드 슬라이더
        top = QWidget(); top_layout = QHBoxLayout(top)
        top_layout.setContentsMargins(8, 4, 8, 4); top_layout.addStretch(1)
        self.mode_slider = QSlider(Qt.Horizontal)
        self.mode_slider.setRange(0, 4)
        self.mode_slider.setSingleStep(1)
        self.mode_slider.setTickInterval(1)
        self.mode_slider.setTickPosition(QSlider.TicksBelow)
        self.mode_slider.setFixedWidth(420)
        self.mode_slider.valueChanged.connect(self._on_mode_slider_changed)
        self.mode_now = QLabel("대기모드"); self.mode_now.setStyleSheet("font-weight:600; padding-left:8px;")
        lbl = QLabel("모드:"); lbl.setStyleSheet("color:#789; padding-right:6px;")
        top_layout.addWidget(lbl); top_layout.addWidget(self.mode_slider); top_layout.addWidget(self.mode_now)

        center = QWidget(); v = QVBoxLayout(center); v.setContentsMargins(0, 0, 0, 0)
        v.addWidget(top); v.addWidget(tabs)
        self.setCentralWidget(center)

        # 초기 전원 OFF 적용
        self._set_mode_slider_by_text("전원 OFF")
        self._apply_power_state()

        # 신호 연결
        self.ctrl_payload.connect(self._handle_ctrl_payload)
        self.log_sig.connect(self._append_log_line)
        self.start_push_seq.connect(self._start_push_sequence)

        # nFusion RX 초기화 + UDP 컨트롤 리스너
        threading.Thread(target=self._rx_setup, daemon=True).start()
        self._start_control_udp()
        self._install_test_shortcuts()

        # GUI 표시 후 상태 OK(=1) 한 번 송신
        QTimer.singleShot(800, lambda: send_status_ok("MMR"))

        # run.py 등의 self_check ON 신호 수신 시에도 내부에서만 0102=1 송신
        def _on_ctrl(payload: dict):
            try:
                if (payload or {}).get("cmd") == "self_check" and int((payload or {}).get("status", 0)) == 1:
                    send_status_ok("MMR")
            except Exception:
                pass
        start_ctrl_listener(env_ctrl_port(45981), _on_ctrl)

        # ★★★ 0101 수신 → 모드 반영 리스너 + 폴백 폴링 설치
        self._install_0101_mode_listener()
        self._start_0101_rx_poller()

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

    def _on_rx_0101(self, raw: bytes | None):
        # 1) RAW → 텍스트
        txt = (raw or b"").decode("utf-8", "ignore")
        # 2) JSON 추출(프리텍스트 대응)
        m = re.search(r"\{.*\}", txt, flags=re.S)
        jtxt = m.group(0) if m else txt.strip()
        # 3) 딕셔너리 파싱
        try:
            body = json.loads(jtxt) if jtxt.startswith("{") else {}
        except Exception:
            body = {}

        # 4) 코드 추출(여러 키/형식 대응)
        code = self._extract_mode_code(body)
        if code is None:
            # 마지막 폴백: RAW에서 직접 캡쳐
            mm = re.search(r'"systemMode"\s*:\s*([0-9]+)', txt)
            if mm:
                try: code = int(mm.group(1))
                except Exception: code = None

        if code is None:
            # 조용히 무시(불필요한 실패 로그 없음)
            return

        if self._apply_system_mode_code(code):
            self._append_log_line(f"[0101] 시스템 운용 모드 수신 → code={code}")
        else:
            self._append_log_line(f"[MODE] 미지원 코드({code})")

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
        외부 0101 systemMode 매핑
          0 : 초기화 모드
          1 : 대기 모드
          2 : 초기임무계획 모드
          3 : 임무수행 모드
        내부 슬라이더(0~4): [0=전원 OFF, 1=전원 ON, 2=대기모드, 3=초기 임무 계획, 4=임무 수행]
        → 매핑: 0→1, 1→2, 2→3, 3→4
        """
        code_to_slider = {0: 1, 1: 2, 2: 3, 3: 4}
        if code not in code_to_slider:
            return False
        val = code_to_slider[code]
        try:
            self.mode_slider.blockSignals(True)
            self.mode_slider.setValue(val)
            self.mode_slider.blockSignals(False)
            # 기존 부수효과(전원/주기TX/모니터링 통지) 실행
            self._on_mode_slider_changed(val)
        except Exception:
            return False
        return True

    # ───────── RX 테이블 폴링 기반 0101 모드 반영(리시버 경로 폴백) ─────────
    def _start_0101_rx_poller(self):
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
                if self._apply_system_mode_code(code):
                    self._append_log_line(f"[0101/POLL] 모드 동기화 code={code}")
                self._last_0101_raw = raw_latest
        except Exception:
            pass


    # ───────── 모니터링(대시보드) 전송 훅 ─────────
    def _install_mon_wires(self):
        """
        - TX 완료(mark_sent) 시 → {"kind":"tx","msg_id":"XXXX"} UDP 전송
        - 주기 TX(_log_only) 경로도 동일 처리
        - 버튼 클릭 경로에서도 선제 통지(실패해도 무해)
        - 모드 변경은 슬라이더 핸들러에서 {"kind":"mode","text":..., "role":"MMR"} 송신
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

        # (4) RX 카운트 발생 지점 훅: bump_rx 호출 ‘후’에 비동기 UDP 알림
        if hasattr(tab, "bump_rx") and not hasattr(self, "_rx_bump_wrapped"):
            _orig_bump_rx = tab.bump_rx
            def _wrapped_bump_rx(msg_id: str):
                _orig_bump_rx(msg_id)
                mid = _z4(str(msg_id))
                def _pulse():
                    cnt = self._rx_counts.get(mid, 0) + 1
                    self._rx_counts[mid] = cnt
                    self._send_mon("rx", msg_id=mid, count=cnt)
                QTimer.singleShot(0, _pulse)
            tab.bump_rx = _wrapped_bump_rx  # type: ignore
            self._rx_bump_wrapped = True

    def _install_0301_override(self):
        tab = getattr(self, "_tab", None)
        if not tab or not hasattr(tab, "_on_tx_button_clicked") or hasattr(self, "_tx_override_installed"):
            return

        original_handler = tab._on_tx_button_clicked

        def _wrapped(row: int):
            code = ""
            try:
                item = tab.tbl_tx.item(row, 0)
                if item is not None:
                    code = item.text().strip()
            except Exception:
                code = ""

            if code == "0301":
                plan_ids: list[int] = []
                for pid in self._scheduled_0301_plan_ids or []:
                    try:
                        plan_ids.append(int(pid))
                    except Exception:
                        continue
                plan_ids = list(dict.fromkeys(plan_ids))
                if not plan_ids:
                    return original_handler(row)
                try:
                    self._send_mon("tx", msg_id=_z4("0301"))
                except Exception:
                    pass
                self._send_0301_batch(plan_ids)
                return

            return original_handler(row)

        tab._on_tx_button_clicked = _wrapped  # type: ignore
        self._tx_override_installed = True

    def _push_single_0301(self, mission_plan_id: int):
        try:
            from push_center import push_message
        except Exception as exc:
            self._append_log_line(f"[ERR] 0301 push unavailable: {exc}")
            return

        try:
            mpid = int(mission_plan_id)
        except Exception:
            self._append_log_line(f"[WARN] 0301 skipped: invalid missionPlanID={mission_plan_id}")
            return

        body = {
            "timestamp": _now_ms_since_2000(),
            "source": "MMR",
            "missionPlanID": mpid,
        }

        try:
            push_message("0301", NodeMessenger, body_dict=body)
            raw = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
        except Exception as exc:
            self._append_log_line(f"[ERR] 0301 push failed: {exc}")
            return

        try:
            self.log_sig.emit(f"[0301] missionPlanID={mpid} 전송")
        except Exception:
            pass

        try:
            self._tab.mark_sent(_z4("0301"), raw)
        except Exception:
            pass

    def _send_0301_batch(self, plan_ids: list[int]):
        for pid in plan_ids:
            self._push_single_0301(pid)
        self._scheduled_0301_plan_ids = []

    def _send_mon(self, kind: str, **payload):
        """
        대시보드(run.py)가 수신하는 모듈별 모니터링 UDP(JSON).
        포트: KU_MON_ASSIGNMENT_PORT(기본 46981)
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

    def _start_0102_stream(self):
        """전원 ON 직후 0.5s 뒤 0102를 5Hz로 자동 시작."""
        if not self._power_on:
            return
        try:
            self._tab.periodic_config['0102'] = 5
        except Exception:
            pass
        self._ensure_0102(True)

    # ───────── Power OFF 가드(발신/수신/카운트/우회 클릭 차단) ─────────
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
            self._update_rx_table_enabled(True)  # ✅ RX는 항상 보이게
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

    # ───────── 순차 푸시(0301 송신 → 0305 완료 → 0903 요청) ─────────
    def _start_push_sequence(self):
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF -> push sequence blocked")
            return
        payload = self._pending_plan_push or {}
        plan_ids = list(payload.get("plan_ids") or [])
        option_names = list(payload.get("option_names") or [])
        reason = payload.get("reason") or "init-plan"

        is_execution_mode = False
        try:
            mode_slider = getattr(self, "mode_slider", None)
            if mode_slider is not None:
                is_execution_mode = int(mode_slider.value()) == 4
        except Exception:
            is_execution_mode = False

        if not plan_ids:
            self._append_log_line("[WARN] No missionPlanID to push (0301)")
            return

        # send 0301
        QTimer.singleShot(0, lambda: self._click_tx_button_for("0301"))
        # send 0305 completion
        QTimer.singleShot(600, lambda: self._push_0305(status=2, reason=reason))

        if is_execution_mode:
            self._append_log_line("[INFO] Execution mode -> sending 0901 instead of 0903")
            QTimer.singleShot(900, lambda: self._push_0901_options(plan_ids, option_names))
        else:
            base_delay = 900
            scheduled = False
            for idx, plan_id in enumerate(plan_ids):
                try:
                    mpid = int(plan_id)
                except Exception:
                    self._append_log_line(f"[WARN] 0903 skip: invalid missionPlanID={plan_id}")
                    continue
                delay = base_delay + idx * 200
                QTimer.singleShot(delay, lambda pid=mpid: self._push_0903(pid))
                scheduled = True
            if not scheduled:
                self._append_log_line("[WARN] No valid missionPlanID for 0903 push")

        self._pending_plan_push = None

    def _click_tx_button_for(self, code: str):
        if not self._power_on:
            self._append_log_line(f"[BLOCK] Power OFF -> TX '{code}' blocked")
            return
        try:
            tab = getattr(self, "_tab", None)
            if tab is None or not hasattr(tab, "tbl_tx"):
                self._append_log_line(f"[WARN] TX table missing for code={code}")
                return

            tbl = tab.tbl_tx
            target_row = -1
            for r in range(tbl.rowCount()):
                it = tbl.item(r, 0)
                if it and it.text().strip() == str(code):
                    target_row = r
                    break

            if target_row < 0:
                self._append_log_line(f"[WARN] TX table has no entry for {code}")
                return

            try:
                btn = tbl.cellWidget(target_row, 3)
                if btn is not None and hasattr(btn, "click"):
                    try:
                        self._send_mon("tx", msg_id=_z4(code))
                    except Exception:
                        pass
                    btn.click()
                    self._append_log_line(f"[PUSH] {code} button click()")
                    return
            except Exception:
                pass

            try:
                if hasattr(tab, "_on_tx_button_clicked"):
                    try:
                        self._send_mon("tx", msg_id=_z4(code))
                    except Exception:
                        pass
                    tab._on_tx_button_clicked(target_row)
                    self._append_log_line(f"[PUSH] {code} handler invoked")
                    return
            except Exception:
                pass

            self._append_log_line(f"[ERR] {code} push failed: no button/handler")
        except Exception as e:
            self._append_log_line(f"[ERR] {code} push failed: {e}")

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
        # ★ 모드 변경도 대시보드로 통지
        try: self._send_mon("mode", text=labels[int(val)], role="MMR")
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
            # ★ 텍스트 기반 모드 변경도 통지
            self._send_mon("mode", text=labels[val], role="MMR")
        except Exception:
            pass
        self._power_on = (int(val) != 0)
        self._apply_power_state()
        if self._power_on:
            QTimer.singleShot(500, self._start_0102_stream)

    # ───────── nFusion RX 초기화 ─────────
    def _rx_setup(self):
        FusionNodeIoc.Configure()
        NodeMessenger.Initialize("MMR_ReceiveNode")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()

    # ───────── 0305 / 0903 요청 ─────────
    def _push_0305(self, status: int, reason: str = "초기임무재계획"):
        try:
            from push_center import push_message
            body = {
                "timestamp": _now_ms_since_2000(),
                "source": "MMR",
                "missionPlanningStatus": int(status),  # 1: 재계획 수행 중, 2: 재계획 완료
                "replanReason": reason,
            }
            orig_mark_fn = getattr(self, "_orig_mark_sent", None)
            mon_sent_via_wrapper = {"done": False}

            def _after_push(mid, raw):
                mid_norm = _z4(str(mid))
                if callable(orig_mark_fn):
                    try:
                        orig_mark_fn(mid_norm, raw)
                        return
                    except Exception:
                        pass
                tab = getattr(self, "_tab", None)
                mark_method = getattr(tab, "mark_sent", None) if tab else None
                if callable(mark_method):
                    try:
                        mark_method(mid_norm, raw)
                        if callable(orig_mark_fn):
                            mon_sent_via_wrapper["done"] = True
                    except Exception:
                        pass

            push_message(
                "0305",
                NodeMessenger,
                on_done=_after_push,
                body_dict=body,
            )
            self.log_sig.emit(f"[0305] status={status}, reason={reason} 전송")
            try:
                if not mon_sent_via_wrapper["done"]:
                    self._send_mon("tx", msg_id=_z4("0305"), missionPlanningStatus=int(status))
            except Exception:
                pass
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0305 전송 실패: {e}")

    def _push_0903(self, mission_plan_id):
        try:
            from push_center import push_message
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0903 push unavailable: {e}")
            return

        if mission_plan_id is None:
            self.log_sig.emit("[WARN] 0903 skipped: missionPlanID missing")
            return

        try:
            mpid = int(mission_plan_id)
        except Exception:
            self.log_sig.emit(f"[WARN] 0903 skipped: invalid missionPlanID={mission_plan_id}")
            return

        body = {
            "timestamp": _now_ms_since_2000(),
            "source": "MMR",
            "missionPlanID": mpid,
        }
        try:
            push_message("0903", NodeMessenger, body_dict=body)
            self.log_sig.emit(f"[0903] request sent (missionPlanID={mpid})")
            try:
                raw = json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore")
            except Exception:
                raw = None
            try:
                self._tab.mark_sent(_z4("0903"), raw)
            except Exception:
                try:
                    self._send_mon("tx", msg_id=_z4("0903"), missionPlanID=mpid)
                except Exception:
                    pass
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0903 push failed: {e}")


    def _allocate_option_ids(self, count: int) -> list[int]:
        ids: list[int] = []
        try:
            total = int(count)
        except Exception:
            total = 0
        for _ in range(max(total, 0)):
            self._option_id_counter += 1
            ids.append(self._option_id_counter)
        return ids

    def _push_0901_options(self, plan_ids, option_names):
        """Push option info request using supplied plan IDs."""
        try:
            from push_center import push_message
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0901 push unavailable: {e}")
            return
        try:
            ts = _now_ms_since_2000()
            plan_list = list(plan_ids or [])
            name_list = list(option_names or [])
            valid_entries: list[tuple[int, str]] = []
            for idx, plan_id in enumerate(plan_list, 1):
                try:
                    pid = int(plan_id)
                except Exception:
                    continue
                name = name_list[idx - 1] if idx - 1 < len(name_list) else f"option{idx}"
                valid_entries.append((pid, str(name)))
            if not valid_entries:
                self.log_sig.emit("[WARN] 0901 skipped: no entries")
                return
            option_ids = self._allocate_option_ids(len(valid_entries))
            entries = [
                {"optionID": oid, "optionName": name, "missionPlanID": pid}
                for oid, (pid, name) in zip(option_ids, valid_entries)
            ]
            body = {
                "timestamp": ts,
                "source": "MMR",
                "requestTime": ts,
                "pendingOptionList": entries,
            }
            push_message("0901", NodeMessenger, body_dict=body)
            self.log_sig.emit(f"[0901] option request sent (count={len(entries)})")
            try:
                self._send_mon("tx", msg_id=_z4("0901"), optionCount=len(entries))
            except Exception:
                pass
        except Exception as e:
            self.log_sig.emit(f"[ERR] 0901 push failed: {e}")

    # ───────── 0102 폴백(일반적으론 send_status_ok 사용) ─────────
    def _send_self_check_0102(self, status: int = 1, _retry: int = 0):
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0102 폴백 차단")
            return
        try:
            from push_center import push_message
        except Exception as e:
            self._append_log_line(f"0102 push import 실패: {e}")
            return

        # 탭이 바디를 오버라이드해 줄 수 있으면 사용, 아니면 폴백
        try:
            body = self._tab._build_overridden_body("0102") or {}
        except Exception:
            body = {}
        if not body:
            body = {"timestamp": _now_ms_since_2000(), "source": "MMR", "status": int(status)}

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

    def _install_test_shortcuts(self):
        # 테스트: 1 → 0102 ON 토글, 0 → 0102 OFF 토글
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
                self._append_log_line("[CTRL] TX 테이블에 0102 행이 없음"); return False
            running = "0102" in getattr(tab, "periodic_timers", {})
            if (on and not running) or ((not on) and running):
                try:
                    btn = tbl.cellWidget(target_row, 3)
                    if btn is not None and hasattr(btn, "click"):
                        btn.click(); return True
                except Exception:
                    pass
                try:
                    if hasattr(tab, "_on_tx_button_clicked"):
                        tab._on_tx_button_clicked(target_row); return True
                except Exception:
                    pass
                self._send_self_check_0102(status=1 if on else 0); return True
            return True
        except Exception as e:
            self._append_log_line(f"[CTRL] 0102 토글 처리 실패: {e}"); return False

    # ───────── CTRL/수신 핸들러 ─────────
    def _handle_ctrl_payload(self, payload: dict):
        import time
        try: cmd = str(payload.get("cmd") or "")
        except Exception: return

        key = f"{cmd}:{payload.get('text') or payload.get('status')}"
        now = time.monotonic(); last = self._last_ctrl_ts.get(key, 0.0)
        if (now - last) < 1.0: return
        self._last_ctrl_ts[key] = now

        # Power OFF 상태에서는 mode 외에는 무시
        if not self._power_on and cmd not in ("mode",):
            self._append_log_line(f"[BLOCK] Power OFF → CTRL '{cmd}' 무시")
            return

        if cmd == "self_check":
            try: status = int(payload.get("status", 1))
            except Exception: status = 1
            ok = self._ensure_0102(on=(status == 1))
            if not ok:
                self._send_self_check_0102(status=status)

        elif cmd == "mode":
            text = str(payload.get("text") or "").strip()
            self._append_log_line(f"[CTRL] MODE change request: {text}")
            self._set_mode_slider_by_text(text)

        elif cmd == "init_plan_context":
            # 외부에서 초기 컨텍스트를 제공하는 경우(파일 경로/ID 등)
            self._stage_plan_context(payload.get("context") or {}, payload.get("trigger") or "")
            return

    # ───────── 0902(재계획 요청) 처리 ─────────
    def _parse_replan_payload(self, raw: bytes | None):
        if not raw:
            return None
        try:
            text = raw.decode("utf-8", "ignore")
            m = re.search(r"{.*}", text, flags=re.S)
            if not m: return None
            return json.loads(m.group(0))
        except Exception:
            return None

    def _stage_plan_context(self, raw_context: dict, trigger: str = ""):
        """외부에서 사전 컨텍스트를 주입(파일 경로/ID/옵션명 등)."""
        if not isinstance(raw_context, dict):
            self._append_log_line("[CTRL] init_plan_context ignored: invalid payload")
            return

        ctx: dict = {}
        # plan_ids
        plan_ids: list[int] = []
        for v in raw_context.get("plan_ids", []):
            try: plan_ids.append(int(v))
            except Exception: pass

        # mission_ids(필요시)
        mission_ids: list[int] = []
        for v in raw_context.get("mission_ids", []):
            try: mission_ids.append(int(v))
            except Exception: pass

        # option_names
        option_names: list[str] = []
        for name in raw_context.get("option_names", []):
            if name is not None:
                option_names.append(str(name))
        while len(option_names) < len(plan_ids):
            option_names.append(f"option{len(option_names) + 1}")

        if plan_ids:     ctx["plan_ids"] = plan_ids
        if mission_ids:  ctx["mission_ids"] = mission_ids
        if option_names: ctx["option_names"] = option_names

        # 입력 파일 경로(선택)
        for key in ("cmpk_path", "mrpk_path"):
            value = raw_context.get(key)
            if isinstance(value, str) and value.strip():
                ctx[key] = value.strip()

        ctx["reason"] = str(raw_context.get("reason") or "init-plan")
        try: ctx["replan_level"] = int(raw_context.get("replan_level", 1))
        except Exception: ctx["replan_level"] = 1

        if raw_context.get("fallback_plan_id") is not None:
            try: ctx["fallback_plan_id"] = int(raw_context.get("fallback_plan_id"))
            except Exception: pass

        self._staged_plan_context = ctx
        summary = ", ".join(str(pid) for pid in ctx.get("plan_ids", [])) or "-"
        note = f"[CTRL] init_plan_context received (planIds={summary})"
        if trigger: note += f" trigger={trigger}"
        self._append_log_line(note)

    def _handle_replan_received(self, msg_id, raw):
        """탭에서 0902 수신 시 호출해주는 콜백."""
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → 0902 수신 무시")
            return

        payload = self._parse_replan_payload(raw)
        if not payload:
            self._append_log_line("[ERR] 0902 payload parse failed")
            return

        staged = self._staged_plan_context if isinstance(getattr(self, '_staged_plan_context', {}), dict) else {}

        # 0902에서 옵션/계획 ID 추출
        plan_ids: list[int] = []
        option_names: list[str] = []
        for item in payload.get("pendingOptionList") or []:
            try:
                plan_ids.append(int(item.get("missionPlanID")))
            except Exception:
                continue
            name = item.get("optionName")
            if name:
                option_names.append(str(name))

        # (필요시) 입력 미션 ID
        mission_ids: list[int] = []
        for item in payload.get("inputMissionIDList") or []:
            try:
                mission_ids.append(int(item.get("inputMissionID")))
            except Exception:
                continue

        reason = str(payload.get("replanReason") or staged.get("reason") or "init-plan")

        ctx = dict(staged)
        if plan_ids:     ctx["plan_ids"] = plan_ids
        if option_names: ctx["option_names"] = option_names
        if mission_ids:  ctx["mission_ids"] = mission_ids
        ctx["reason"] = reason
        try:
            ctx["replan_level"] = int(payload.get("replanLevel", ctx.get("replan_level", 1)))
        except Exception:
            ctx["replan_level"] = ctx.get("replan_level", 1)
        if payload.get("fallbackPlanId") is not None:
            try: ctx["fallback_plan_id"] = int(payload.get("fallbackPlanId"))
            except Exception: pass

        self._active_plan_context = ctx
        summary = ", ".join(str(pid) for pid in ctx.get("plan_ids", [])) or "-"
        self._append_log_line(f"[AUTO] 0902 received (planIds={summary})")
        self._schedule_replan_pipeline(delay_ms=1500)

    # ───────── 재계획 파이프라인(파일 생성/저장 후 0301만 송신) ─────────
    def _schedule_replan_pipeline(self, delay_ms: int = 1000) -> None:
        """Delay the replan pipeline start to avoid race conditions with rapid 0902 dispatch."""
        timer = getattr(self, "_replan_delay_timer", None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()
        if delay_ms <= 0:
            self._replan_delay_timer = None
            self._run_replan_pipeline_async()
            return
        timer = QTimer(self)
        timer.setSingleShot(True)
        timer.setInterval(int(delay_ms))
        timer.timeout.connect(self._run_replan_pipeline_async)
        self._replan_delay_timer = timer
        timer.start()
        self._append_log_line(f"[AUTO] replan pipeline scheduled after {delay_ms/1000:.1f}s delay")

    def _run_replan_pipeline_async(self):
        timer = getattr(self, "_replan_delay_timer", None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()
            self._replan_delay_timer = None
        if not self._power_on:
            self._append_log_line("[BLOCK] Power OFF → replan pipeline 차단")
            return
        if self._initplan_running:
            self._append_log_line("[INFO] replan pipeline already running")
            return
        self._initplan_running = True
        self._pending_plan_push = None
        ctx = getattr(self, "_active_plan_context", {}) or {}
        reason = str(ctx.get("reason") or "초기임무재계획")
        self._push_0305(status=1, reason=reason)
        threading.Thread(target=self._run_replan_pipeline_do, name="Replan-GUI", daemon=True).start()

    def _run_replan_pipeline_do(self):
        try:
            import os, json
            from pathlib import Path

            ctx = getattr(self, '_active_plan_context', {}) or {}
            staged = self._staged_plan_context if isinstance(getattr(self, '_staged_plan_context', {}), dict) else {}
            reason = str(ctx.get('reason') or staged.get('reason') or 'init-plan')

            self.log_sig.emit(f"[STEP 0] Replan pipeline start (reason={reason})")

            mp_pkg_dir = Path(PROJECT_ROOT) / 'modules' / 'mission_planning' / 'MissionPlanner'
            for p in (mp_pkg_dir, mp_pkg_dir.parent, Path(PROJECT_ROOT) / 'modules'):
                p_str = str(p)
                if p.exists() and p_str not in sys.path:
                    sys.path.insert(0, p_str)

            from AnS import run_divide_and_pattern, build_mission_plan_0301
            from data_def import d0302, d0303, d0304
            from data_def.id_allocator import next_path_id

            def _imp_path_id(im):
                for key in ('pathID', 'pathId', 'individualMissionPathID', 'missionPathID'):
                    value = im.get(key)
                    try:
                        if value is not None:
                            return int(value)
                    except Exception:
                        continue
                mission_info = im.get('missionInfo')
                if isinstance(mission_info, dict):
                    for key in ('pathID', 'pathId'):
                        value = mission_info.get(key)
                        try:
                            if value is not None:
                                return int(value)
                        except Exception:
                            continue
                return None

            def _enforce_fp_path_ids(fps, pid_map):
                fixed = 0
                for fp in fps or []:
                    try:
                        aid = int(fp.get('aircraftID', 0))
                        mid = int(fp.get('individualMissionID', 0))
                        desired = pid_map.get((aid, mid))
                        if desired is not None and fp.get('pathID') != desired:
                            fp['pathID'] = desired
                            fixed += 1
                    except Exception:
                        continue
                return fixed

            db_root = db_paths.get_active_db_root()
            dir_0201 = db_root / 'InputMissionPlan'
            dir_0203 = db_root / 'MissionReferenceInfo'
            out_root_base = db_root / 'mission_output'
            out_root_base.mkdir(parents=True, exist_ok=True)

            def _pick_json(directory: Path):
                candidates = sorted(p for p in directory.glob('*.json') if p.is_file())
                return candidates[0] if candidates else None

            def _resolve_path(value, directory: Path):
                if value:
                    try:
                        candidate = Path(value)
                        if candidate.exists():
                            return candidate
                    except Exception:
                        pass
                return _pick_json(directory)

            cmpk_path = _resolve_path(ctx.get('cmpk_path') or staged.get('cmpk_path'), dir_0201)
            mrpk_path = _resolve_path(ctx.get('mrpk_path') or staged.get('mrpk_path'), dir_0203)
            if not cmpk_path or not mrpk_path:
                self.log_sig.emit('[ERR] Replan pipeline aborted: missing 0201/0203 input')
                return

            plan_ids_source = ctx.get('plan_ids') or staged.get('plan_ids') or []
            plan_ids: list[int | None] = []
            for val in plan_ids_source:
                try:
                    plan_ids.append(int(val))
                except Exception:
                    plan_ids.append(None)

            option_names = list(ctx.get('option_names') or staged.get('option_names') or [])
            plan_count = max(len(plan_ids), len(option_names), 1)
            while len(plan_ids) < plan_count:
                plan_ids.append(None)
            while len(option_names) < plan_count:
                option_names.append(f'option{len(option_names) + 1}')

            try:
                cmpk_id = int(Path(cmpk_path).stem)
            except Exception:
                cmpk_id = 0

            dir_mp = db_root / 'MissionPlan'
            dir_imp = db_root / 'IndividualMissionPlan'
            dir_fp = db_root / 'FlightPath'
            for directory in (dir_mp, dir_imp, dir_fp):
                directory.mkdir(parents=True, exist_ok=True)

            def _scan_existing_ids(target_dir: Path) -> set[int]:
                results: set[int] = set()
                try:
                    for item in target_dir.glob("*.json"):
                        stem = item.stem
                        if stem.isdigit():
                            try:
                                results.add(int(stem))
                            except Exception:
                                continue
                except Exception:
                    pass
                return results

            used_plan_ids: set[int] = _scan_existing_ids(dir_mp)
            next_plan_id_seed = max(used_plan_ids) + 1 if used_plan_ids else 700000000

            def _allocate_plan_id(preferred: int | None) -> int:
                nonlocal next_plan_id_seed
                if preferred is not None:
                    try:
                        candidate = int(preferred)
                        if candidate not in used_plan_ids:
                            used_plan_ids.add(candidate)
                            return candidate
                        self.log_sig.emit(f"[WARN] missionPlanID {candidate} already exists; allocating new ID")
                    except Exception:
                        pass
                while next_plan_id_seed in used_plan_ids:
                    next_plan_id_seed += 1
                assigned = next_plan_id_seed
                used_plan_ids.add(assigned)
                next_plan_id_seed += 1
                return assigned

            generated_plan_ids: list[int] = []
            option_names_out: list[str] = []
            total_imp_files = 0
            total_fp_files = 0

            for idx in range(plan_count):
                variant_no = idx + 1
                requested_plan_id = plan_ids[idx]
                option_name = option_names[idx]

                iter_out_root = out_root_base / f'variant_{variant_no:02d}'
                if iter_out_root.exists():
                    shutil.rmtree(iter_out_root)
                iter_out_root.mkdir(parents=True, exist_ok=True)

                self.log_sig.emit(f"[STEP 1.{variant_no}] Divide & Pattern start")
                imp_paths = run_divide_and_pattern(
                    str(cmpk_path),
                    str(mrpk_path),
                    str(iter_out_root),
                    log=lambda msg, n=variant_no: self.log_sig.emit(f"[variant {n}] {msg}")
                )
                if not imp_paths:
                    self.log_sig.emit(f"[ERR] IMP generation failed (variant={variant_no})")
                    return
                self.log_sig.emit(f"[OK] IMP generated: {len(imp_paths)} file(s) (variant={variant_no})")

                mp_tmp = iter_out_root / f"MissionPlan_{int(time.time()*1000)}.json"
                build_mission_plan_0301(str(cmpk_path), str(mrpk_path), imp_paths, str(mp_tmp))
                with mp_tmp.open(encoding='utf-8') as f:
                    mp_json = json.load(f)
                imp_id_map = {a.get('aircraftID'): a.get('individualMissionPackageID') for a in mp_json.get('aircraftList', [])}
                self.log_sig.emit(f"[OK] MissionPlan built: {mp_tmp.name} (variant={variant_no})")

                missions = []
                for imp_path in imp_paths:
                    with open(imp_path, encoding='utf-8') as f:
                        pkg = json.load(f)
                    aid = int(pkg.get('aircraftID', 0))
                    for im in pkg.get('individualMissionList', []):
                        im_copy = dict(im)
                        im_copy['aircraftID'] = aid
                        if 'individualMissionPlanPackageID' not in im_copy and imp_id_map:
                            im_copy['individualMissionPlanPackageID'] = imp_id_map.get(aid)
                        missions.append(im_copy)

                pid_map = {}
                for im in missions:
                    aid = int(im.get('aircraftID', 0))
                    mid = int(im.get('individualMissionID', 0))
                    if aid in (1, 2, 3):
                        pid = int(next_path_id(aid))
                        im['pathID'] = pid
                        pid_map[(aid, mid)] = pid
                    else:
                        imp_pid = _imp_path_id(im)
                        if imp_pid is not None:
                            im['pathID'] = int(imp_pid)
                            pid_map[(aid, mid)] = int(imp_pid)
                self.log_sig.emit(f"[INFO] pathID mapping done for 0302/0303/0304 (variant={variant_no})")

                manned = [im for im in missions if int(im.get('aircraftID', 0)) in (1, 2, 3)]
                unmanned = [im for im in missions if int(im.get('aircraftID', 0)) in (4, 5, 6)]
                wp_alloc = d0303._WPAllocator()
                flight_plans_0303 = d0303.build_flight_plans(unmanned, wp_alloc, 40.0, turn_step_deg=15.0) if unmanned else []
                flight_plans_0304 = d0304.build_lah_flight_plans_fixed(manned, cruise_speed=40.0, wp_alloc=wp_alloc) if manned else []

                fixed3 = _enforce_fp_path_ids(flight_plans_0303, pid_map)
                fixed4 = _enforce_fp_path_ids(flight_plans_0304, pid_map)
                if fixed3 or fixed4:
                    self.log_sig.emit(f"[INFO] FlightPath pathID enforced (variant={variant_no}): 0303={fixed3}, 0304={fixed4}")
                if not flight_plans_0303 and not flight_plans_0304:
                    self.log_sig.emit(f"[ERR] FlightPath generation failed (variant={variant_no})")
                    return
                self.log_sig.emit(f"[OK] FlightPath counts (variant={variant_no}): 0303={len(flight_plans_0303)} / 0304={len(flight_plans_0304)}")

                plan_id_value = requested_plan_id if requested_plan_id is not None else mp_json.get('missionPlanID')
                try:
                    preferred_plan_id = int(plan_id_value)
                except Exception:
                    preferred_plan_id = None
                plan_id = _allocate_plan_id(preferred_plan_id)
                mp_json['missionPlanID'] = plan_id

                (dir_mp / f"{plan_id}.json").write_text(json.dumps(mp_json, indent=2, ensure_ascii=False), encoding='utf-8')

                imp_pkgs = d0302.build_mission_packages(missions, cmpk_id=cmpk_id, plan_pkg_map=imp_id_map)
                for pkg in imp_pkgs:
                    imp_id = pkg.get('individualMissionPackageID') or pkg.get('individualMissionPlanPackageID')
                    if imp_id is None:
                        continue
                    (dir_imp / f"{int(imp_id)}.json").write_text(json.dumps(pkg, indent=2, ensure_ascii=False), encoding='utf-8')
                total_imp_files += len(imp_pkgs)

                def _dump_fp(target_dir, fps):
                    count = 0
                    for fp in fps:
                        pid = fp.get('pathID')
                        if pid is None:
                            continue
                        (target_dir / f"{int(pid)}.json").write_text(json.dumps(fp, indent=2, ensure_ascii=False), encoding='utf-8')
                        count += 1
                    return count

                fp_count_0303 = _dump_fp(dir_fp, flight_plans_0303)
                fp_count_0304 = _dump_fp(dir_fp, flight_plans_0304)
                total_fp_files += fp_count_0303 + fp_count_0304

                self.log_sig.emit(f"[OK] Stored variant {variant_no}: MissionPlanID={plan_id}, IMP={len(imp_pkgs)}, FlightPath={fp_count_0303 + fp_count_0304}")

                generated_plan_ids.append(plan_id)
                option_names_out.append(option_name)

                try:
                    shutil.rmtree(iter_out_root)
                except Exception:
                    pass

            try:
                if out_root_base.exists():
                    shutil.rmtree(out_root_base)
            except Exception:
                pass

            self.log_sig.emit(f"[OK] Stored mission data: MissionPlan={len(generated_plan_ids)}, IndividualMission={total_imp_files}, FlightPath={total_fp_files}")

            self._last_mission_plan_ids = generated_plan_ids
            self._last_mission_plan_id = generated_plan_ids[0] if generated_plan_ids else None
            ctx['plan_ids'] = generated_plan_ids
            ctx['option_names'] = option_names_out
            self._active_plan_context = ctx

            self._schedule_plan_delivery(generated_plan_ids, option_names_out, reason)

        except Exception as exc:
            self.log_sig.emit(f"[ERR] Replan pipeline failed: {exc}")
        finally:
            self._initplan_running = False
    def _schedule_plan_delivery(self, plan_ids, option_names, reason):
        self._pending_plan_push = {
            "plan_ids":     list(plan_ids or []),
            "option_names": list(option_names or []),
            "reason":       reason,
        }
        try:
            self._scheduled_0301_plan_ids = [int(pid) for pid in (plan_ids or []) if pid is not None]
        except Exception:
            self._scheduled_0301_plan_ids = list(plan_ids or [])
        summary = ", ".join(str(pid) for pid in plan_ids or []) or "-"
        self.log_sig.emit(f"[STEP 4] 0301 push queued (planIds={summary})")
        self.start_push_seq.emit()


# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())


