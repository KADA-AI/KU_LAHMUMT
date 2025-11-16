# /run.py
# -*- coding: utf-8 -*-
# KU_LAHMUMT 대시보드 실행 & 모듈 모니터링/관리 전용
# ─────────────────────────────────────────────────────────────
# [모듈별 모니터링(수신) 포트 기본값 / 환경변수]
# - mission_planning (modules/mission_planning/mission_planning_gui.py) → 46981  (KU_MON_ASSIGNMENT_PORT)
# - monitoring       (modules/monitoring/monitoring_gui.py)             → 46982  (KU_MON_MONITORING_PORT)
# - decision         (modules/decision_support/decision_support_gui.py) → 46983  (KU_MON_DECISION_PORT)
# - info             (modules/info_manage/info_manage.py)               → 46984  (KU_MON_INFO_PORT)
#
# ※ CTRL 브로드캐스트(송신)는 45981/45982/45983/45984 유지.
#   본 파일의 46981~46984는 ‘모듈→대시보드(본 파일)’ 모니터링 수 용입니다.

from __future__ import annotations
import os, sys, subprocess, threading
os.environ["KU_ROLE"] = "decision"
from pathlib import Path
from typing import Dict, List, Tuple

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, QObject, pyqtSignal, QTimer, QMetaObject, Qt, Q_ARG, pyqtSlot
from PyQt5.QtGui import QCursor, QTextCursor
from PyQt5.QtWidgets import QApplication, QTextEdit, QPlainTextEdit

from modules.common.states.manager import StateManager
from modules.common import db_paths
from modules.common.button_wiring import wire_dashboard_buttons
from modules.monitoring_ver2.utils.vehicle_status import write_vehicle_status
import collections


def _debug_log(message: str) -> None:
    # Debug logging disabled to avoid creating log files
    return

# ─────────────────────────────────────────────────────────────
# Qt 경고 필터 (선택)
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    if message.startswith("QMainWindowLayout::"):
        return
    sys.stderr.write(message + "\n")
qInstallMessageHandler(_qt_silent_handler)

# ─────────────────────────────────────────────────────────────
# 경로 부트스트랩
def _bootstrap_paths():
    root = Path(__file__).resolve().parent
    modules_dir = root / "modules"
    common_dir = modules_dir / "common"
    ds_dir = modules_dir / "decision_support"
    for p in (root, modules_dir, common_dir, ds_dir):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try:
        os.chdir(root)
    except Exception:
        pass
    return root, common_dir, ds_dir

PROJECT_ROOT, COMMON_DIR, DS_DIR = _bootstrap_paths()


db_paths.bootstrap_db_root()

# ─────────────────────────────────────────────────────────────
# 스타일(QSS) 로드
def _load_qss(app: QApplication):
    candidates = [
        PROJECT_ROOT / "app" / "resources" / "style.qss",
        PROJECT_ROOT / "resources" / "style.qss",
    ]
    for p in candidates:
        if p.exists():
            try:
                app.setStyleSheet(p.read_text(encoding="utf-8"))
                break
            except Exception:
                pass

# ─────────────────────────────────────────────────────────────
# nFusion 설정/라이선스 정규화 + MessageLibrary 로드
def _ensure_fusion_configs():
    settings_candidates = [
        PROJECT_ROOT / "nFusionSettings.json",
        DS_DIR / "nFusionSettings.json",
        COMMON_DIR / "nFusionSettings.json",
        PROJECT_ROOT / "FusionSettings.json",
        DS_DIR / "FusionSettings.json",
        COMMON_DIR / "FusionSettings.json",
        PROJECT_ROOT / "nFusion" / "FusionSettings.json",
    ]
    src = next((p for p in settings_candidates if p.exists()), None)
    if src is None:
        sys.stderr.write("WARN: nFusionSettings.json/FusionSettings.json not found; running without bus.\n")
        return None
    dst = PROJECT_ROOT / "nFusionSettings.json"
    if src != dst:
        dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    # license
    lic_candidates = [
        PROJECT_ROOT / "nFusionLicense.lic",
        DS_DIR / "nFusionLicense.lic",
        COMMON_DIR / "nFusionLicense.lic",
        PROJECT_ROOT / "nFusion" / "nFusionLicense.lic",
    ]
    lic_src = next((p for p in lic_candidates if p.exists()), None)
    if lic_src is not None:
        lic_dst = PROJECT_ROOT / "nFusionLicense.lic"
        if lic_src != lic_dst:
            lic_dst.write_text(lic_src.read_text(encoding="utf-8"), encoding="utf-8")
    return str(dst)

def _load_msglib_and_deps():
    try:
        from dll_files.nFusionImports import clr
    except Exception:
        import clr  # type: ignore
    msg_dir = COMMON_DIR / "msg_files"
    stem = msg_dir / "MessageLibrary"
    try:
        clr.AddReference(str(stem))
    except Exception:
        clr.AddReference(str(stem.with_suffix(".dll")))
    for s in ("K4586Model", "K4586Model.Assist", "MiscUtil"):
        dll = msg_dir / (s + ".dll")
        if dll.exists():
            try:
                clr.AddReference(str(dll.with_suffix("")))
            except Exception:
                try:
                    clr.AddReference(str(dll))
                except Exception:
                    pass

# ─────────────────────────────────────────────────────────────
# 메인 윈도우 로드(경로 가변 지원)
try:
    from app.ui.main_window import MainWindow  # type: ignore
except Exception:
    from main_window import MainWindow  # type: ignore

try:
    from app.ui.widgets.module_with_log import ModuleWithLog
except Exception:
    try:
        from module_with_log import ModuleWithLog
    except Exception:
        ModuleWithLog = object

try:
    from app.ui.widgets.flow_visualizer import FlowVisualizer
except Exception:
    try:
        from flow_visualizer import FlowVisualizer
    except Exception:
        FlowVisualizer = object

# receive_center API 확보(공용/DS 통합 어느쪽이든)
_register_listener = None
try:
    from receive_center import register_listener as _register_listener  # root
except Exception:
    try:
        from modules.common.receive_center import register_listener as _register_listener
    except Exception:
        pass

# NodeMessenger 가져오기 (버스 초기화용)
NFusion_OK = False
try:
    from dll_files.nFusionImports import FusionNodeIoc, NodeMessenger  # type: ignore
    NFusion_OK = True
except Exception:
    NFusion_OK = False

# ─────────────────────────────────────────────────────────────
# 메시지 정의 읽기 유틸
def _normalize_defs(defs) -> List[Tuple[str, str]]:
    out: List[Tuple[str, str]] = []
    if not defs:
        return out
    for item in defs:
        if isinstance(item, dict):
            mid = item.get("id") or item.get("msg_id") or item.get("message_id") or item.get("code")
            name = item.get("name") or item.get("label") or item.get("title") or (f"MSG {mid}" if mid is not None else "MSG")
            if mid is not None:
                out.append((str(mid), str(name)))
        elif isinstance(item, (tuple, list)):
            if len(item) == 2:
                mid, name = item
                out.append((str(mid), str(name)))
            elif len(item) == 1:
                mid = item[0]
                out.append((str(mid), f"MSG {mid}"))
        elif isinstance(item, (int, str)):
            mid = str(item)
            out.append((mid, f"MSG {mid}"))
    return out

def _load_tab_defs() -> Dict[str, Dict[str, List[Tuple[str, str]]]]:
    import importlib
    result = {}
    pairs = [
        ("assignment", "Tabs.assignment_planning_tab", "AssignmentPlanningTab"),
        ("monitoring", "Tabs.mission_monitoring_tab", "MissionMonitoringTab"),
        ("decision",   "Tabs.decision_support_tab",   "DecisionSupportTab"),
    ]
    for key, mod_name, cls_name in pairs:
        tx_defs, rx_defs = [], []
        try:
            mod = importlib.import_module(mod_name)
            cls = getattr(mod, cls_name, None)
            cand_tx = []
            cand_rx = []
            for holder in (mod, cls):
                if holder is None:
                    continue
                if hasattr(holder, "PUSH_MESSAGES"):
                    cand_tx += list(getattr(holder, "PUSH_MESSAGES"))
                if hasattr(holder, "RECEIVE_MESSAGES"):
                    cand_rx += list(getattr(holder, "RECEIVE_MESSAGES"))
            tx_defs = _normalize_defs(cand_tx)
            rx_defs = _normalize_defs(cand_rx)
        except Exception as e:
            sys.stderr.write(f"[WARN] message defs load failed for {mod_name}: {e}\n")
        result[key] = {"tx": tx_defs, "rx": rx_defs}
    return result


_env_safe = os.getenv("KU_SAFE_STATES_WHEN_APPS_RUNNING", "")
SAFE_STATES_WHEN_APPS_RUNNING = set([s.strip() for s in _env_safe.split(",") if s.strip()]) or {"S110"}

# ─────────────────────────────────────────────────────────────
# 대시보드 오케스트레이터 (모니터링/관리 전용)
class DashboardOrchestrator(QObject):
    dashEvent = pyqtSignal(str, str)        # (kind, msg_id)
    dashPulse = pyqtSignal(str, str, str)   # (role, kind, msg_id) — 이미 있으면 유지
    uiLog     = pyqtSignal(str)             # 글로벌/폴백용 단일 문자열
    uiLog2    = pyqtSignal(str, str)        # 역할 지정 (role, text)

    def __init__(self, window: MainWindow):
        super().__init__(window)
        self.win = window
        self.widgets = self._resolve_widgets(window)
        self._log_line_limit = self._resolve_log_limit()
        self._apply_log_limits()
        self._apply_dashboard_button_styles()
        self.msg_map = _load_tab_defs()
        self._wire_operation_panel()

        self._state_mgr = StateManager()

        self._manage_info_proc = None
        self._latest_db_payloads = {}
        self._push_suppression = {}
        self._standby_pending = False
        self._scenario_activated_once = False

        # 모드 루프 방지용 상태
        self._mode_text = "전원 OFF"
        self._last_mode_broadcast_ms = 0.0

        # 안전 로거 alias
        self._log_everywhere = getattr(self, "_log_everywhere", None) or (lambda text: self._append_log_global(str(text)))
        self._safe_log = lambda text: self._log_everywhere(text) if hasattr(self, "_log_everywhere") else self._append_log_global(text)

        self._fallback_log_role = os.getenv("KU_MON_FALLBACK_LOG", "decision")
        self.dashEvent.connect(self._handle_dash_event)
        self.dashPulse.connect(self._handle_dash_pulse)   # 이미 연결돼 있으면 유지
        self.uiLog.connect(self._append_log_global)       # 단일 문자열 → 글로벌/폴백
        self.uiLog2.connect(self._log_to_role)            # (role, text) → 해당 모듈

        self._init_rows()
        wire_dashboard_buttons(self)   # 버튼 → 상태 디스패치 연결

        reset_btn = getattr(self.win, "btn_decision_reset", None)
        if reset_btn is not None and hasattr(reset_btn, "clicked"):
            try:
                reset_btn.clicked.connect(self._handle_decision_reset_button)
            except Exception:
                pass

        self._start_bus_monitor()
        self._start_module_sockets()   # 4개 모듈 모니터링 포트 바인드

        # 시작 시 전원 OFF
        self._set_mode_text_all("전원 OFF")

        self._module_mode = {"assignment": "전원 OFF", "monitoring": "전원 OFF", "decision": "전원 OFF", "info": "전원 OFF"}

        self._hz_threshold = float(os.getenv("KU_MON_HZ_THRESH", "1.0"))  # 1Hz 이하는 비주기로 간주, 로그 생략
        self._hz_window    = float(os.getenv("KU_MON_HZ_WIN", "5.0"))     # 이동 평균 창(초)
        self._hz_stats: dict[str, dict] = {}
        self._hz_lock = threading.Lock()

        self._last_system_mode_code: int | None = None
        info = db_paths.get_info()
        self._scenario_timestamp = info.get("timestamp_ms")
        try:
            db_root_init = info.get("db_root") or db_paths.get_active_db_root_str()
            self.win.update_db_root(db_root_init)
            self.win.update_scenario_root(info.get("base_root"))
            tooltip = f"{info.get('iso') or '??'} @ {db_root_init}"
            self.win.update_scenario_status_indicator(False, tooltip)
        except Exception:
            pass

    def _log_to_role(self, role: str, text: str):
        """해당 role 모듈 로그에만 기록. 실패 시 글로벌 폴백."""
        r = self._normalize_role(role)
        target = self.widgets.get(r)
        if target and hasattr(target, "append_log"):
            try:
                target.append_log(text)
                return
            except Exception:
                pass
        # 폴백
        self._append_log_global(text)

    def _hz_touch(self, role: str, kind: str, mid: str):
        """
        role-kind-mid 스트림의 이벤트 1건을 반영하여
        - 최근 Hz(이동창 평균)
        - 누적 Hz(시작~현재 평균)
        를 갱신한다. 최근 Hz > threshold 일 때만 로그 출력.
        """
        import time
        now = time.monotonic()
        key = f"{self._normalize_role(role)}:{kind}:{self._norm_code(mid)}"

        with self._hz_lock:
            s = self._hz_stats.get(key)
            if s is None:
                s = {"count": 0, "first": now, "last": now, "dq": collections.deque(), "last_log": 0.0}
                self._hz_stats[key] = s

            s["count"] += 1
            s["last"] = now
            dq = s["dq"]
            dq.append(now)

            # 이동창 유지
            cutoff = now - self._hz_window
            while dq and dq[0] < cutoff:
                dq.popleft()

            # 최근 Hz: 창 내 간격 평균 기반 (샘플 2개 이상일 때)
            if len(dq) >= 2:
                recent_hz = (len(dq) - 1) / max(1e-6, dq[-1] - dq[0])
            else:
                recent_hz = 0.0

            # 누적 Hz: 전체 구간 평균
            span = max(1e-6, s["last"] - s["first"])
            cum_hz = s["count"] / span

            s["recent_hz"] = recent_hz
            s["cum_hz"] = cum_hz

            # 1Hz 이하는 비주기로 간주 → 로그 생략
            if recent_hz <= self._hz_threshold:
                return

            # 과도한 로그 방지(최소 1초 간격)
            if now - s["last_log"] >= 1.0:
                self.uiLog2.emit(role, f"[HZ] {self._normalize_role(role)}:{kind}:{self._norm_code(mid)}  recent≈{recent_hz:.2f} Hz   cum≈{cum_hz:.2f} Hz")
                s["last_log"] = now

    def _set_mode_text_single(self, role: str, text: str):
        role = self._normalize_role(role)
        mod = self.widgets.get(role)
        if not mod:
            return
        # 우선 set_mode_text가 있으면 사용
        if hasattr(mod, "set_mode_text") and callable(mod.set_mode_text):
            try:
                mod.set_mode_text(text)
                return
            except Exception:
                pass
        # 폴백: mode_line QLabel 직접 세팅
        try:
            ml = getattr(mod, "mode_line", None)
            if ml is not None and hasattr(ml, "setText"):
                ml.setText(text)
        except Exception:
            pass

    def _normalize_role(self, role: str) -> str:
        r = (role or "").strip().lower()
        if r in ("assignment", "mission", "mission_planning", "mmr"):
            return "assignment"
        if r in ("monitoring", "monitor", "msm"):
            return "monitoring"
        if r in ("decision", "mob", "decision_support", "decision-support"):
            return "decision"
        if r in ("info", "info_manage", "imr", "information"):
            return "info"
        return r or "unknown"


    def _owner_modules_for(self, mid: str):
        """msg_map을 뒤져 mid를 RECEIVE로 가진 탭 키 목록을 반환 (코드 4자리 정규화)"""
        midn = self._norm_code(mid)
        owners = []
        for mk, defs in self.msg_map.items():
            rx_ids = {self._norm_code(m) for m, _ in defs.get("rx", [])}  # ★ 정규화
            if midn in rx_ids:
                owners.append(mk)
        return owners

    def _handle_dash_pulse(self, role: str, kind: str, msg_id: str):
        role = self._normalize_role(role)
        mid  = self._norm_code(msg_id)

        w = {
            "assignment": self.widgets.get("assignment"),
            "monitoring": self.widgets.get("monitoring"),
            "decision":   self.widgets.get("decision"),
        }.get(role)
        if not w:
            return

        defs = self.msg_map.get(role, {})

        if kind == "tx" and hasattr(w, "bump_tx"):
            w.bump_tx(mid)
            self._animate(role, "out")
            if hasattr(w, "append_log"): w.append_log(f"[{mid}] PUSH 완료")

        elif kind == "rx":
            # ★ 여기서도 RX 정의를 4자리로 맞춰서 비교
            rx_ids = {self._norm_code(m) for m, _ in defs.get("rx", [])}
            did = False
            if mid in rx_ids and hasattr(w, "bump_rx"):
                w.bump_rx(mid); did = True
                self._animate(role, "in")
                if hasattr(w, "append_log"): w.append_log(f"[{mid}] RX 수신")
            if not did:
                owners = self._owner_modules_for(mid)
                for mk in owners:
                    ww = {
                        "assignment": self.widgets.get("assignment"),
                        "monitoring": self.widgets.get("monitoring"),
                        "decision":   self.widgets.get("decision"),
                    }.get(mk)
                    if ww and hasattr(ww, "bump_rx"):
                        ww.bump_rx(mid)
                        self._animate(mk, "in")
                        if hasattr(ww, "append_log"): ww.append_log(f"[{mid}] RX 수신")

    def _init_rows(self):
        # ★ 행을 채울 때도 4자리로 정규화해서, 나중에 bump_rx("0301")이 정확히 매칭되게
        for key, defs in self.msg_map.items():
            mod = self.widgets.get(key)
            if not mod:
                continue
            tx_pairs = [(self._norm_code(mid), 0) for (mid, _name) in defs.get("tx", [])]  # ★
            rx_pairs = [(self._norm_code(mid), 0) for (mid, _name) in defs.get("rx", [])]  # ★
            set_tx = getattr(mod, "set_tx_rows", None)
            if callable(set_tx):
                set_tx(tx_pairs)
            set_rx = getattr(mod, "set_rx_rows", None)
            if callable(set_rx):
                set_rx(rx_pairs)


    # ── 모듈별 모니터링 소켓(UDP) 시작: 4개 포트 바인드
    def _get_monitor_port_map(self) -> Dict[str, int]:
        return {
            "assignment": int(os.getenv("KU_MON_ASSIGNMENT_PORT", "46981")),
            "monitoring": int(os.getenv("KU_MON_MONITORING_PORT", "46982")),
            "decision":   int(os.getenv("KU_MON_DECISION_PORT", "46983")),
            "info":       int(os.getenv("KU_MON_INFO_PORT", "46984")),
        }

    def _start_module_sockets(self):
        import socket, json, time

        self._seen_evt_ts = {}
        self._monitor_socks = {}

        port_map = self._get_monitor_port_map()
        try:
            lines = [f"[MON PORTS] {role}: {port}" for role, port in port_map.items()]
            self._safe_log("\n".join(lines))
        except Exception:
            pass

        def _dedup(key: str, window: float = 0.15) -> bool:
            now = time.monotonic()
            last = self._seen_evt_ts.get(key, 0.0)
            if (now - last) < window:
                return True
            self._seen_evt_ts[key] = now
            return False

        def _reader(role: str, sock: "socket.socket"):
            while True:
                try:
                    data, _addr = sock.recvfrom(65535)
                except Exception:
                    break

                # ── 원본 payload/파싱 ──
                kind, mid, text, src_role = "", "", "", ""
                payload = {}
                try:
                    payload = json.loads(data.decode("utf-8", "ignore")) if data else {}
                except Exception:
                    payload = {}

                if isinstance(payload, dict):
                    kind = str(payload.get("kind") or payload.get("direction") or "").strip().lower()
                    mid  = str(payload.get("msg_id") or payload.get("id") or payload.get("code") or "").strip()
                    text = str(payload.get("text") or "").strip()
                    src_role = str(payload.get("role") or role).strip()
                    cmd = str(payload.get("cmd") or "").strip().lower()
                    _debug_log(f"_reader payload role={role} kind={kind} mid={mid} cmd={cmd} payload={payload!r}")

                    if cmd == "self_check" and not mid:
                        mid = "0102"
                        kind = kind or "tx"
                        src_role = src_role or role


                # ── ★ 디버그 프린트: UDP 수신 패킷 원본/핵심 필드 ──
                try:
                    self.uiLog2.emit(role, f"[UDP RECV] {role}/{src_role}  kind={kind}  mid={mid}")
                except Exception:
                    pass

                # 1) tx/rx → 해당 role만 카운트 + HZ 집계
                if kind in ("tx", "rx") and mid:
                    norm_role = self._normalize_role(src_role or role)
                    if not _dedup(f"mon:{norm_role}:{kind}:{mid}"):
                        try:
                            self._hz_touch(norm_role, kind, mid)   # HZ 집계
                        except Exception:
                            pass
                        self.dashPulse.emit(norm_role, kind, mid)  # 카운트/애니메

                # 2) mode → 동기화
                if kind == "mode" and text:
                    self._handle_mode_event(src_role or role, text)

                # 3) 부가 로그
                log_txt = None
                if isinstance(payload, dict):
                    log_txt = payload.get("log") or payload.get("message") or None
                if log_txt:
                    self.uiLog2.emit(src_role or role, str(log_txt))

        # 소켓 4개 바인드 + 스레드 시작 (그대로)
        import socket
        for role, port in port_map.items():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(("127.0.0.1", port))
                self._monitor_socks[role] = s
                t = threading.Thread(target=_reader, args=(role, s), daemon=True)
                t.start()
            except Exception as e:
                try:
                    self._safe_log(f"[WARN] monitor UDP bind failed: {role}:{port} → {e}")
                except Exception:
                    pass

    # --------- mode 이벤트 처리(수신→대시보드/모듈 전체 동기화) ---------
    def _normalize_mode_text(self, text: str) -> str:
        t = "".join(str(text).split()).lower()
        if t in ("전원off", "off", "poweroff", "0"): return "전원 OFF"
        if t in ("전원on",  "on",  "poweron",  "1"): return "초기화 모드"
        if t in ("대기모드", "대기", "standby", "2"): return "대기모드"
        if t in ("초기임무계획", "초기임무계획모드", "initplan", "initial", "3", "초기임무계획모드진입"): return "초기임무계획"
        if t in ("임무수행", "execution", "4"): return "임무 수행"
        return text or "대기모드"

    def _handle_mode_event(self, src_role: str, text: str):
        import time
        role = self._normalize_role(src_role or "unknown")
        norm = self._normalize_mode_text(text)

        prev = self._module_mode.get(role)
        if prev == norm:
            return
        self._module_mode[role] = norm

        QMetaObject.invokeMethod(
            self,
            "_apply_mode_event",
            Qt.QueuedConnection,
            Q_ARG(str, role),
            Q_ARG(str, norm),
        )

    @pyqtSlot(str, str)
    def _apply_mode_event(self, role: str, norm: str):
        self._set_mode_text_single(role, norm)
        self._safe_log(f"[MODE] {role} → {norm}")
        self._visualize_mode_change(role, norm)


    def _visualize_mode_change(self, role: str, text: str):
        flow = self.widgets.get("flow")
        if not flow and callable(getattr(self.win, "_pulse", None)):
            # 대시보드가 제공하는 간단 펄스만 있는 경우
            vis_key = {"assignment": "mission", "monitoring": "monitor", "decision": "decision"}.get(self._normalize_role(role), "system")
            try:
                self.win._pulse(vis_key, "mode")
            except Exception:
                pass
            return

        if not flow:
            return

        vis_key = {"assignment": "mission", "monitoring": "monitor", "decision": "decision"}.get(self._normalize_role(role), "system")
        # 가능한 API들을 안전하게 시도
        for name in ("set_mode_text", "setModeText", "set_status", "setStatus", "setMode"):
            fn = getattr(flow, name, None)
            if callable(fn):
                try:
                    fn(str(text))
                    break
                except Exception:
                    pass
        try:
            trig = getattr(flow, "trigger", None)
            if callable(trig):
                trig(vis_key, "mode")
        except Exception:
            pass

    # --------- UI 위젯 해결 ---------

    def _apply_dashboard_button_styles(self) -> None:
        """Set the auto boot and module shutdown buttons to green."""
        green_style = "\n".join((
            "QPushButton {",
            "  background: #16a34a;",
            "  color: white;",
            "  border: none;",
            "  border-radius: 8px;",
            "  padding: 6px 12px;",
            "}",
            "QPushButton:hover { background: #15803d; }",
            "QPushButton:pressed { background: #166534; }",
        ))
        targets = (
            getattr(self.win, "btn_auto_boot", None),
            getattr(self.win, "btn_module_shutdown", None),
        )
        for btn in targets:
            if btn is None:
                continue
            try:
                btn.setStyleSheet(green_style)
            except Exception:
                pass

    def _resolve_widgets(self, win):
        flow = getattr(win, "flow", None)
        if flow is None:
            try:
                from app.ui.widgets.flow_visualizer import FlowVisualizer  # type: ignore
            except Exception:
                FlowVisualizer = None
            if FlowVisualizer:
                try:
                    candidates = win.findChildren(FlowVisualizer)
                    if candidates:
                        flow = candidates[0]
                except Exception:
                    flow = None

        log_edit = None
        try:
            from PyQt5.QtWidgets import QPlainTextEdit, QTextEdit
            # Only treat widgets explicitly tagged as logs to avoid dumping debug text in memo fields.
            for widget in win.findChildren((QPlainTextEdit, QTextEdit)):
                name = (widget.objectName() or "").lower()
                if "log" in name:
                    log_edit = widget
                    break
        except Exception:
            pass

        modules = {
            "assignment": getattr(win, "module_mission", None),
            "monitoring": getattr(win, "module_monitor", None),
            "decision":   getattr(win, "module_decision", None),
        }

        ops_panel = getattr(win, "operation_panel", None)
        if ops_panel is None:
            try:
                from app.ui.widgets.operation_flow_panel import OperationFlowPanel  # type: ignore
            except Exception:
                OperationFlowPanel = None
            if OperationFlowPanel:
                try:
                    panels = win.findChildren(OperationFlowPanel)
                    if panels:
                        ops_panel = panels[0]
                except Exception:
                    ops_panel = None

        return {"flow": flow, "log_edit": log_edit, "operation_panel": ops_panel, **modules}

    def _resolve_log_limit(self) -> int:
        raw = (os.getenv("KU_MON_LOG_MAX_LINES") or "10").strip()
        try:
            value = int(raw)
        except ValueError:
            value = 10
        return max(0, value)

    def _apply_log_limits(self) -> None:
        limit = getattr(self, "_log_line_limit", 10)
        self._configure_text_widget(self.widgets.get("log_edit"), limit)
        for key in ("assignment", "monitoring", "decision", "info"):
            widget = self.widgets.get(key)
            if widget is None:
                continue
            if hasattr(widget, "set_log_max_lines"):
                try:
                    widget.set_log_max_lines(limit)
                    continue
                except Exception:
                    pass
            log_widget = getattr(widget, "log", None)
            self._configure_text_widget(log_widget, limit)

    def _configure_text_widget(self, widget, limit: int) -> None:
        if widget is None:
            return
        try:
            doc = widget.document()
        except Exception:
            return
        try:
            doc.setMaximumBlockCount(limit)
        except Exception:
            pass
        self._truncate_text_widget(widget, limit)

    def _truncate_text_widget(self, widget, limit: int) -> None:
        if widget is None:
            return
        if limit <= 0:
            return
        try:
            doc = widget.document()
            while doc.blockCount() > limit:
                cursor = QTextCursor(doc)
                cursor.movePosition(QTextCursor.Start)
                cursor.select(QTextCursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()
        except Exception:
            pass

    def _recently_seen(self, tag: str, mid: str, window: float = 0.3) -> bool:
        import time
        t = time.monotonic()
        d = getattr(self, "_seen_any", {})
        key = f"{tag}:{mid}"
        last = d.get(key, 0.0)
        if (t - last) < window:
            return True
        d[key] = t
        self._seen_any = d
        return False

    # --------- 초기 목록 채우기 ---------
    def _init_rows(self):
        for key, defs in self.msg_map.items():
            mod = self.widgets.get(key)
            if not mod:
                continue
            tx_pairs = [(mid, 0) for (mid, _name) in defs.get("tx", [])]
            rx_pairs = [(mid, 0) for (mid, _name) in defs.get("rx", [])]
            set_tx = getattr(mod, "set_tx_rows", None)
            if callable(set_tx):
                set_tx(tx_pairs)
            set_rx = getattr(mod, "set_rx_rows", None)
            if callable(set_rx):
                set_rx(rx_pairs)

    # --------- 상태 디스패치 (패널/버튼 공용 진입점) ---------
    def _handle_operation_state(self, code: str):
        try:
            ok = self._state_mgr.dispatch(code, self)
        except Exception as e:
            ok = False
            self._safe_log(f"[ERR] 상태 디스패치 오류: {e}")
        if not ok:
            self._safe_log(f"[OPS] {code} 상태는 아직 연결되지 않았습니다.")

    # --------- 버스 모니터링 시작 ---------
    def _start_bus_monitor(self):
        try:
            _ensure_fusion_configs()
        except Exception:
            pass
        try:
            _load_msglib_and_deps()
        except Exception as e:
            sys.stderr.write(f"[WARN] _load_msglib_and_deps failed: {e}\n")

        try:
            import receive  # noqa: F401
        except Exception as e:
            sys.stderr.write(f"[WARN] failed to import receive: {e}\n")
            return

        try:
            from receive_center import register_listener as register_listener  # 루트
        except Exception:
            try:
                from modules.common.receive_center import register_listener as register_listener  # 공용
            except Exception as e:
                sys.stderr.write(f"[WARN] receive_center.register_listener not available: {e}\n")
                return

        all_ids = set()
        for defs in self.msg_map.values():
            for mid, _ in (defs.get("tx", []) + defs.get("rx", [])):
                all_ids.add(str(mid))
        for mid in sorted(all_ids):
            try:
                register_listener(mid, self)
            except Exception as e:
                sys.stderr.write(f"[WARN] register_listener({mid}) failed: {e}\n")

        def _rx_setup():
            try:
                from dll_files.nFusionImports import FusionNodeIoc, NodeMessenger  # noqa
                FusionNodeIoc.Configure()
                NodeMessenger.Initialize("CommonChannel")
                NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
                NodeMessenger.InitAllSubscriberFromAssembly()
                NodeMessenger.RegistAllProviderFromFusionNodeIoc()
            except Exception as e:
                sys.stderr.write(f"[WARN] NodeMessenger init failed: {e}\n")
        threading.Thread(target=_rx_setup, daemon=True).start()

    def _norm_code(self, mid) -> str:
        """메시지 코드를 '0102'처럼 4자리 0패딩으로 통일"""
        s = str(mid).strip()
        return s.zfill(4) if s.isdigit() and len(s) < 4 else s
    
    # --------- 버스 수신 시각화 + 특수 메시지 처리 ---------
    def mark_received(self, msg_id: str, raw: bytes | None = None):
        mid = self._norm_code(msg_id)
        payload_obj = self._extract_message_json(raw)

        # 소스 모듈 추정 → role 정규화
        src_key = None
        if isinstance(payload_obj, dict):
            src = (payload_obj.get("source")
                   or payload_obj.get("source")
                   or payload_obj.get("requestModuleName"))
            if src:
                src_key = self._normalize_role(str(src))

        # 디듀프(역할 구분)
        tag = f"bus:{src_key or 'unknown'}"
        if self._recently_seen(tag, mid, window=0.02):
            return

        key_to_widget = {
            "assignment": self.widgets.get("assignment"),
            "monitoring": self.widgets.get("monitoring"),
            "decision":   self.widgets.get("decision"),
        }

        # 대상 모듈 결정: 소스가 명확하면 그 하나만,
        # 아니면 rx 정의의 '유일 소유자'인 경우에만 1곳 카운트
        target_modules = []
        if src_key in key_to_widget and key_to_widget[src_key]:
            target_modules = [src_key]
        else:
            owners = [mk for mk, defs in self.msg_map.items()
                      if any(m == mid for m, _ in defs.get("rx", []))]
            if len(owners) == 1:
                target_modules = owners  # 유일 소유자일 때만

        for module_key in target_modules:
            w = key_to_widget.get(module_key)
            if not w:
                continue
            defs = self.msg_map.get(module_key, {})
            rx_ids = {m for m, _ in defs.get("rx", [])}
            if mid in rx_ids and hasattr(w, "bump_rx"):
                w.bump_rx(mid); self._animate(module_key, "in")
                if hasattr(w, "append_log"): w.append_log(f"[{mid}] RX 수신")

        # 특수 메시지 처리(모드 전환 등)
        self._handle_special_bus_message(mid, payload_obj, raw)

    def _handle_special_bus_message(self, mid: str, payload: dict | None, raw: bytes | None) -> None:
        # 0101 SystemMode==2 → 초기임무계획 모드 진입
        if mid == "0101":
            self._handle_system_mode_message(payload, raw)
            return

        # 0305 재계획 진행/완료 로깅
        if mid == "0305":
            status = None
            if isinstance(payload, dict):
                status = payload.get("missionPlanningStatus") or payload.get("MissionPlanningStatus")
            try:
                status = int(status) if status is not None else None
            except Exception:
                status = None
            if status == 0:
                self._safe_log("[OPS] 0305 재계획 진행 중")
            elif status == 1:
                self._safe_log("[OPS] 0305 재계획 완료 (대기모드 전환은 0702 수신 후)")
            return

        # 0702 수신 → 대기모드 전환
        if mid == "0702":
            self._safe_log("[OPS] 0702 수신 → 대기모드 전환")
            self._enter_standby()
            return

    def _handle_system_mode_message(self, payload: dict | None, raw: bytes | None) -> None:
        obj = payload if isinstance(payload, dict) else self._extract_message_json(raw)
        if not isinstance(obj, dict):
            return

        timestamp = None
        for key in ("timestamp", "Timestamp"):
            value = obj.get(key)
            if value is None:
                continue
            try:
                timestamp = int(value)
                break
            except Exception:
                try:
                    timestamp = int(float(str(value).strip()))
                    break
                except Exception:
                    continue

        code = None
        for key in ("systemMode", "SystemMode"):
            value = obj.get(key)
            if value is None:
                continue
            try:
                code = int(value)
                break
            except Exception:
                try:
                    code = int(float(str(value).strip()))
                    break
                except Exception:
                    continue

        if code is None:
            return

        if code == 1:
            self._maybe_activate_scenario(timestamp)

        if code == 2:
            self._safe_log("[OPS] SystemMode=2 수신 → 초기임무계획 모드 진입")
            self._enter_initial_plan()

        self._last_system_mode_code = code



    def _maybe_activate_scenario(self, timestamp: int | None) -> None:
        if self._last_system_mode_code == 1:
            return
        if self._scenario_activated_once:
            reuse_root = db_paths.get_active_db_root_str()
            self._safe_log(f"[OPS] Standby re-entry - reuse existing DB @ {reuse_root}")
            try:
                self.win.update_scenario_status_indicator(False, f"재사용 경로: {reuse_root}")
            except Exception:
                pass
            return
        if timestamp is None:
            self._safe_log("[OPS] Standby entry missing timestamp -> skip DB activation")
            try:
                self.win.update_scenario_status_indicator(False, "타임스탬프 없음 → 기존 경로 유지")
            except Exception:
                pass
            return
        prev_db_root = db_paths.get_active_db_root_str()
        try:
            info = db_paths.activate_scenario(timestamp)
        except Exception as exc:
            self._safe_log(f"[ERR] Standby activation prep failed: {exc}")
            try:
                self.win.update_scenario_status_indicator(False, f"Standby 준비 실패: {exc}")
            except Exception:
                pass
            return

        self._scenario_timestamp = info.get("timestamp_ms")
        db_root = info.get("db_root")
        if db_root:
            try:
                self.win.update_db_root(db_root)
                self.win.update_scenario_root(info.get("base_root"))
            except Exception:
                pass
        db_root_str = db_root or db_paths.get_active_db_root_str()
        iso = info.get("iso") or timestamp
        change_state = "신규 경로 적용"
        if prev_db_root and db_root_str:
            try:
                prev_norm = os.path.normcase(os.path.normpath(str(prev_db_root)))
                curr_norm = os.path.normcase(os.path.normpath(str(db_root_str)))
            except Exception:
                prev_norm = str(prev_db_root)
                curr_norm = str(db_root_str)
            if prev_norm == curr_norm:
                change_state = "경로 유지"
        elif not db_root_str:
            change_state = "경로 확인 불가"
        self._safe_log(f"[OPS] Standby 활성화({change_state}) {iso} @ {db_root_str}")
        tooltip = f"{iso} @ {db_root_str or '경로 없음'}"
        changed = (change_state == "신규 경로 적용")
        try:
            self.win.update_scenario_status_indicator(changed, tooltip)
        except Exception:
            pass
        if db_root_str:
            try:
                if not Path(db_root_str).exists():
                    self._safe_log(f"[WARN] Standby DB path missing: {db_root_str}")
            except Exception:
                self._safe_log(f"[WARN] Standby DB path inspection failed: {db_root_str}")
        self._scenario_activated_once = True
        try:
            write_vehicle_status(None)
        except Exception:
            pass

    def _handle_decision_reset_button(self):
        """User-triggered reset to rerun decision module flow on next standby."""
        try:
            prev_root = db_paths.get_active_db_root_str()
        except Exception:
            prev_root = None

        try:
            handler = getattr(self.win, "_handle_module_shutdown", None)
            if callable(handler):
                handler()
        except Exception as exc:
            try:
                self._safe_log(f"[WARN] Decision reset module shutdown failed: {exc}")
            except Exception:
                pass

        self._scenario_activated_once = False
        self._scenario_timestamp = None
        self._last_system_mode_code = None
        self._standby_pending = False

        log_msg = "[OPS] Decision module reset ready for next standby"
        if prev_root:
            log_msg += f" (current DB: {prev_root})"
        try:
            self._safe_log(log_msg)
        except Exception:
            pass

        try:
            tooltip = f"Reset 대기: {prev_root}" if prev_root else "Reset 대기: DB 미선택"
            self.win.update_scenario_status_indicator(False, tooltip)
        except Exception:
            pass

        try:
            self._set_mode_text_all("모듈 초기화 대기")
        except Exception:
            pass

    # --------- 모드/CTRL/런처/로깅 유틸 ---------
    def _log_assignment(self, text: str):
        mod = self.widgets.get("assignment")
        if mod and hasattr(mod, "append_log"):
            try: mod.append_log(text)
            except Exception: pass
        self._append_log_global(text)

    def _enter_standby(self):
        self._self_check_all(False)
        self._set_mode_text_all("대기모드")
        self._safe_log("모든 SW 대기모드 진입")

    def _enter_initial_plan(self):
        self._self_check_all(False)
        self._set_mode_text_all("초기임무계획")
        self._safe_log("모든 SW 초기임무계획 모드")

    def _self_check_all(self, on: bool = True):
        payload = {"cmd": "self_check", "status": 1 if on else 0}
        self._broadcast_ctrl(payload)
        self._safe_log(f"자체점검 0102 {'ON' if on else 'OFF'} 전송")

    def _set_mode_text_all(self, text: str):
        for key in ("assignment", "monitoring", "decision"):
            mod = self.widgets.get(key)
            if not mod:
                continue
            if hasattr(mod, "set_mode_text") and callable(mod.set_mode_text):
                try: mod.set_mode_text(text); continue
                except Exception: pass
            try:
                ml = getattr(mod, "mode_line", None)
                if ml is not None and hasattr(ml, "setText"):
                    ml.setText(text)
            except Exception:
                pass

    def _broadcast_ctrl(self, payload: dict):
        import socket, json
        targets = [("assignment", 45981), ("monitoring", 45982), ("decision", 45983), ("info", 45984)]
        data = json.dumps(payload).encode("utf-8")
        for role, port in targets:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(data, ("127.0.0.1", port))
                s.close()
            except Exception as e:
                try: sys.stderr.write(f"[WARN] CTRL send to {role}:{port} failed: {e}\n")
                except Exception: pass

    def _send_ctrl_single(self, target: str, payload: dict) -> bool:
        import socket, json
        port_map = {"assignment": 45981, "monitoring": 45982, "decision": 45983, "info": 45984}
        port = port_map.get(target)
        if port is None:
            return False
        try:
            data = json.dumps(payload).encode("utf-8")
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.sendto(data, ("127.0.0.1", port))
            s.close()
            return True
        except Exception as e:
            try: sys.stderr.write(f"[WARN] CTRL send to {target}:{port} failed: {e}\n")
            except Exception: pass
            return False

    def trigger_initial_plan_pipeline(self, reason: str = "초기임무재계획") -> None:
        """Send control commands that drive the initial mission-planning pipeline."""
        normalized_reason = str(reason or "초기임무재계획")
        context = {"reason": normalized_reason, "replan_level": 1}

        self._safe_log(f"[OPS] S110: preparing initial mission planning (reason={normalized_reason})")

        try:
            ok_assign = self._send_ctrl_single("assignment", {"cmd": "init_plan_context", "context": context, "trigger": "S110"})
            if not ok_assign:
                self._safe_log('[WARN] init_plan_context dispatch to assignment failed')
        except Exception as exc:
            self._safe_log(f'[WARN] init_plan_context dispatch error: {exc}')

        try:
            ok_stage = self._send_ctrl_single("monitoring", {"cmd": "stage_replan", "context": context})
            if not ok_stage:
                self._safe_log('[WARN] stage_replan dispatch to monitoring failed')
        except Exception as exc:
            self._safe_log(f'[WARN] stage_replan dispatch error: {exc}')

        def _fire_replan():
            payload = {"cmd": "replan", "reason": normalized_reason, "replanLevel": context.get("replan_level", 1)}
            try:
                ok = self._send_ctrl_single("monitoring", payload)
                if ok:
                    self._safe_log('[OPS] 0902 replan request sent to monitoring')
                else:
                    self._safe_log('[WARN] 0902 replan request dispatch failed')
            except Exception as exc:
                self._safe_log(f'[WARN] replan dispatch error: {exc}')

        QTimer.singleShot(300, _fire_replan)


    def _launch_all_guis(self):
        for sn in ("mission_planning_gui.py", "monitoring_gui.py", "decision_support_gui.py", "info_manage.py"):
            self._launch_gui(sn)
        QTimer.singleShot(1000, lambda: self._set_mode_text_all("초기화 모드"))
        QTimer.singleShot(1000, lambda: self._broadcast_ctrl({"cmd": "mode", "text": "초기화 모드"}))
        QTimer.singleShot(1000, lambda: self._safe_log("모든 SW 초기화 모드 진입"))

    def _launch_gui(self, script_name: str):
        import sys, os, subprocess
        from pathlib import Path

        root = Path(__file__).resolve().parent
        modules_dir = root / "modules"

        candidates = [
            root / script_name,
            modules_dir / script_name,
            modules_dir / "mission_planning" / script_name,
            modules_dir / "monitoring" / script_name,
            modules_dir / "decision_support" / script_name,
            modules_dir / "info_manage" / script_name,
        ]
        script = next((p for p in candidates if p.exists()), None)
        if script is None:
            msg = f"[RUN ERR] not found: {script_name}\n - searched:\n   " + "\n   ".join(str(c) for c in candidates)
            try: self._safe_log(msg)
            except Exception: pass
            try: sys.stderr.write(msg + "\n")
            except Exception: pass
            return

        ui_line = getattr(self.win, "_db_path_line", None)
        ui_val = ui_line.text().strip() if ui_line and hasattr(ui_line, "text") else ""
        db_root = ui_val or db_paths.get_active_db_root_str()
        try: Path(db_root).mkdir(parents=True, exist_ok=True)
        except Exception: pass

        env = os.environ.copy()
        env.setdefault("KU_LAUNCHED_BY_DASHBOARD", "1")
        env["KU_MISSION_DB_ROOT"] = db_root

        port_map = {
            "mission_planning_gui.py": "45981",
            "monitoring_gui.py":       "45982",
            "decision_support_gui.py": "45983",
            "info_manage.py":          "45984",
        }
        try:
            script_basename = script.name
        except Exception:
            script_basename = script_name
        env["KU_CTRL_PORT"] = port_map.get(script_basename, port_map.get(script_name, ""))
        env.pop("KU_WINDOW_OFFSET", None)

        try: self._safe_log(f"[RUN] {script_basename} @ {script}")
        except Exception: pass

        creationflags = 0
        if os.name == "nt":
            creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)

        try:
            subprocess.Popen(
                [sys.executable, str(script)],
                cwd=str(root),
                env=env,
                start_new_session=True,
                creationflags=creationflags,
            )
        except Exception as e:
            err = f"[RUN ERR] {script_basename}: {e}"
            try: self._safe_log(err)
            except Exception: pass
            try: sys.stderr.write(err + "\n")
            except Exception: pass
            return

    def _extract_message_json(self, raw: bytes | None):
        if not raw:
            return None
        try:
            import json, re
            text_data = raw.decode("utf-8", "ignore")
            match = re.search(r"\{.*\}", text_data, flags=re.S)
            if not match:
                return None
            return json.loads(match.group(0))
        except Exception:
            return None

    def _animate(self, module_key: str, direction: str):
        vis_key = {"assignment": "mission", "monitoring": "monitor", "decision": "decision"}.get(module_key, module_key)
        fnp = getattr(self.win, "_pulse", None)
        if callable(fnp):
            try: fnp(vis_key, direction); return
            except Exception: pass
        flow = self.widgets.get("flow")
        if not flow:
            return
        fn = getattr(flow, "trigger", None)
        if callable(fn):
            try: fn(vis_key, direction)
            except Exception: pass

    def _wire_operation_panel(self):
        """운용 패널(체크리스트/상태 버튼) → 상태 디스패치 연결"""
        panel = self.widgets.get("operation_panel") if hasattr(self, "widgets") else None
        if not panel:
            return
        try:
            panel.stateTriggered.connect(self._handle_operation_state)
        except Exception:
            pass

    def _handle_dash_event(self, kind: str, msg_id: str):
        mid = str(msg_id)
        if self._recently_seen(f"udp:{kind}", mid):
            return

        key_to_widget = {
            "assignment": self.widgets.get("assignment"),
            "monitoring": self.widgets.get("monitoring"),
            "decision":   self.widgets.get("decision"),
        }
        for module_key, defs in self.msg_map.items():
            w = key_to_widget.get(module_key)
            if not w:
                continue
            if kind == "tx" and any(m == mid for m, _ in defs.get("tx", [])):
                if hasattr(w, "bump_tx"): w.bump_tx(mid)
                self._animate(module_key, "out")
                if hasattr(w, "append_log"): w.append_log(f"[{mid}] PUSH 완료")
            if kind == "rx" and any(m == mid for m, _ in defs.get("rx", [])):
                if hasattr(w, "bump_rx"): w.bump_rx(mid)
                self._animate(module_key, "in")
                if hasattr(w, "append_log"): w.append_log(f"[{mid}] RX 수신")

    def _append_log_global(self, text: str):
        ed = self.widgets.get("log_edit")
        if ed is not None:
            try:
                # 모듈 내부 로그 위젯과의 중복 방지 로직은 유지
                p = ed.parent()
                while p is not None:
                    from PyQt5.QtWidgets import QWidget  # 안전
                    if isinstance(p, ModuleWithLog):
                        break
                    p = p.parent()
            except Exception:
                pass
            try:
                if isinstance(ed, QPlainTextEdit): ed.appendPlainText(text)
                elif isinstance(ed, QTextEdit): ed.append(text)
                else:  # 예외적으로 타입이 다르면 폴백700
                    raise RuntimeError("no global text edit")
                return
            except Exception:
                pass

        # 전역 로그 위젯이 없거나 실패 → 폴백 모듈로
        fb = self.widgets.get(self._fallback_log_role)
        if fb and hasattr(fb, "append_log"):
            try:
                fb.append_log(text)
                return
            except Exception:
                pass

        # 최종 폴백: 콘솔(옵션) — 기본적으로는 터미널에 출력하지 않는다.
        allow_console = os.getenv("KU_MON_CONSOLE_FALLBACK", "").strip().lower()
        if allow_console in ("1", "true", "yes", "on"):
            try:
                print(text)
            except Exception:
                pass


def _position_window_at_cursor(app: QApplication, win):
    try:
        cursor_pos = QCursor.pos()
    except Exception:
        return
    screen = app.screenAt(cursor_pos) if hasattr(app, "screenAt") else None
    if screen is not None:
        screen_geo = screen.geometry()
    else:
        desktop = app.desktop() if hasattr(app, "desktop") else None
        screen_geo = desktop.screenGeometry(cursor_pos) if desktop and hasattr(desktop, "screenGeometry") else None

    frame_geo = win.frameGeometry()
    frame_w = frame_geo.width() or win.width()
    frame_h = frame_geo.height() or win.height()
    target_x = cursor_pos.x() - frame_w // 2
    target_y = cursor_pos.y() - frame_h // 2

    if screen_geo is not None and frame_w > 0 and frame_h > 0:
        left = screen_geo.x()
        top = screen_geo.y()
        right = left + screen_geo.width()
        bottom = top + screen_geo.height()
        target_x = max(left, min(target_x, right - frame_w))
        target_y = max(top, min(target_y, bottom - frame_h))

    win.move(target_x, target_y)


def _arm_auto_boot(win, delay_ms: int = 1000) -> None:
    """Trigger the auto boot button (or handler) after the given delay."""
    btn = getattr(win, "btn_auto_boot", None)
    if btn is not None and hasattr(btn, "click"):
        try:
            QTimer.singleShot(delay_ms, btn.click)
            return
        except Exception:
            pass

    handler = getattr(win, "_handle_auto_boot", None)
    if callable(handler):
        try:
            QTimer.singleShot(delay_ms, handler)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    _load_qss(app)
    win = MainWindow()
    win.show()
    _position_window_at_cursor(app, win)
    orch = DashboardOrchestrator(win)
    _arm_auto_boot(win, 1000)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
