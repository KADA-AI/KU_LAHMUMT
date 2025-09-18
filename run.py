# /run.py
# -*- coding: utf-8 -*-
# KU_LAHMUMT 대시보드 실행 & 모듈 모니터링/관리 전용

from __future__ import annotations
import os, sys, subprocess, threading
os.environ["KU_ROLE"] = "decision"
from pathlib import Path
from typing import Dict, List, Tuple, Optional

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, QObject, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QTextEdit, QPlainTextEdit

from modules.common.states.manager import StateManager
from modules.common.button_wiring import wire_dashboard_buttons

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

# ─────────────────────────────────────────────────────────────
# 대시보드 오케스트레이터 (모니터링/관리 전용)
class DashboardOrchestrator(QObject):
    dashEvent = pyqtSignal(str, str)   # (kind, msg_id) → UI 펄스
    uiLog     = pyqtSignal(str)        # 텍스트 로그 UI 스레드 기록

    def __init__(self, window: MainWindow):
        super().__init__(window)
        self.win = window
        self.widgets = self._resolve_widgets(window)
        self.msg_map = _load_tab_defs()
        self._wire_operation_panel()

        self._state_mgr = StateManager()

        self._manage_info_proc = None
        self._latest_db_payloads = {}
        self._push_suppression = {}
        self._last_mode_trigger = 0.0
        self._standby_pending = False

        # 안전 로거 alias
        self._log_everywhere = getattr(self, "_log_everywhere", None) or (lambda text: self._append_log_global(str(text)))
        self._safe_log = lambda text: self._log_everywhere(text) if hasattr(self, "_log_everywhere") else self._append_log_global(text)

        self.dashEvent.connect(self._handle_dash_event)
        self.uiLog.connect(self._log_assignment)

        self._init_rows()
        wire_dashboard_buttons(self)   # 버튼 → 상태 디스패치 연결

        self._start_bus_monitor()
        self._start_dashboard_socket()

        # 시작 시 전원 OFF
        self._set_mode_text_all("전원 OFF")
        self._broadcast_ctrl({"cmd": "mode", "text": "전원 OFF"})

    # ── UDP 대시보드 소켓
    def _start_dashboard_socket(self):
        import socket, json, time, threading
        self._seen_evt_ts = {}
        port = int(os.getenv("KU_DASHBOARD_PORT", "45991"))
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("127.0.0.1", port))
        except Exception as e:
            sys.stderr.write(f"[WARN] dashboard UDP bind failed: {e}\n")
            return

        def _dedup(key: str) -> bool:
            now = time.monotonic()
            last = self._seen_evt_ts.get(key, 0.0)
            if (now - last) < 0.15:
                return True
            self._seen_evt_ts[key] = now
            return False

        def loop():
            while True:
                try:
                    data, _ = sock.recvfrom(8192)
                    payload = json.loads(data.decode("utf-8", "ignore"))
                    kind = str(payload.get("kind") or "")
                    mid  = str(payload.get("msg_id") or "")
                    if kind not in ("tx", "rx") or not mid:
                        continue
                    if _dedup(f"udp:{kind}:{mid}"):
                        continue
                    self.dashEvent.emit(kind, mid)
                except Exception:
                    pass
        threading.Thread(target=loop, daemon=True).start()

    def _wire_operation_panel(self):
        panel = self.widgets.get("operation_panel") if hasattr(self, "widgets") else None
        if panel is None:
            return
        try:
            panel.stateTriggered.connect(self._handle_operation_state)
        except Exception:
            pass

    def _norm_code(self, mid) -> str:
        s = str(mid)
        return s.zfill(4) if s.isdigit() and len(s) < 4 else s

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

    # --------- UI 위젯 해결 ---------
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
            for widget in win.findChildren((QPlainTextEdit, QTextEdit)):
                name = (widget.objectName() or "").lower()
                if "log" in name:
                    log_edit = widget
                    break
            if log_edit is None:
                widgets = win.findChildren((QPlainTextEdit, QTextEdit))
                if widgets:
                    log_edit = widgets[0]
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

    # --------- 버스 수신 시각화 + 특수 메시지 처리 ---------
    def mark_received(self, msg_id: str, raw: bytes | None = None):
        mid = self._norm_code(msg_id)
        payload_obj = self._extract_message_json(raw)

        # 소스 추정(로깅/디듀프용)
        src_key = None
        if isinstance(payload_obj, dict):
            src = payload_obj.get("SourceModuleName") or payload_obj.get("source") or payload_obj.get("requestModuleName")
            if src:
                s = str(src).upper()
                if "MMR" in s or "MULTI-AGENT MISSION PLANNER" in s:
                    src_key = "assignment"
                elif "MSM" in s or "MISSION STATE MONITOR" in s:
                    src_key = "monitoring"
                elif "MOB" in s or "MISSION OPTION BUILDER" in s:
                    src_key = "decision"

        tag = f"bus:{src_key or 'all'}"
        if self._recently_seen(tag, mid, window=0.02):
            return

        key_to_widget = {
            "assignment": self.widgets.get("assignment"),
            "monitoring": self.widgets.get("monitoring"),
            "decision":   self.widgets.get("decision"),
        }

        # UI 카운트/애니메이션
        for module_key in ("assignment", "monitoring", "decision"):
            w = key_to_widget.get(module_key)
            if not w:
                continue
            defs = self.msg_map.get(module_key, {})
            tx_ids = {m for m, _ in defs.get("tx", [])}
            rx_ids = {m for m, _ in defs.get("rx", [])}
            if mid in tx_ids and hasattr(w, "bump_tx"):
                w.bump_tx(mid); self._animate(module_key, "out")
                if hasattr(w, "append_log"): w.append_log(f"[{mid}] PUSH 완료")
            if mid in rx_ids and hasattr(w, "bump_rx"):
                w.bump_rx(mid); self._animate(module_key, "in")
                if hasattr(w, "append_log"): w.append_log(f"[{mid}] RX 수신")

        # 특수 메시지 처리(새 플로우 반영)
        self._handle_special_bus_message(mid, payload_obj, raw)

    def _handle_special_bus_message(self, mid: str, payload: dict | None, raw: bytes | None) -> None:
        """
        새 S110 플로우에 맞게 '모니터링/관리'만 수행:
          - 0101 SystemMode==2 → 초기임무계획 모드 진입(대시보드/모듈 CTRL 모드 전파)
          - 0305: in-progress/완료 로깅만 (대기모드 전환은 0702에서)
          - 0702: 사용자가 Info에서 승인 버튼(0702) 눌러 전송 시 → 대기모드 전환
          - 0201/0203 자동 PUSH/파이프라인 수행 없음(각 모듈 버튼이 전담)
        """
        # 0101 SystemMode 처리
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

        # (주의) 예전 자동 0201/0203 PUSH, 파이프라인 실행 등은 삭제됨

    def _handle_system_mode_message(self, payload: dict | None, raw: bytes | None) -> None:
        obj = payload if isinstance(payload, dict) else self._extract_message_json(raw)
        if not isinstance(obj, dict):
            return
        # systemMode==2 → 초기임무계획
        for key in ("systemMode", "SystemMode"):
            val = obj.get(key)
            try:
                if val is not None and int(val) == 2:
                    self._safe_log("[OPS] SystemMode=2 수신 → 초기임무계획 모드 진입")
                    self._enter_initial_plan()  # 모드 전파(CTRL), 대시보드 표시
                    return
            except Exception:
                continue

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
        self._broadcast_ctrl({"cmd": "mode", "text": "대기모드"})
        self._safe_log("모든 SW 대기모드 진입")

    def _enter_initial_plan(self):
        self._self_check_all(False)
        self._set_mode_text_all("초기임무계획")
        self._broadcast_ctrl({"cmd": "mode", "text": "초기임무계획"})
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

    def _launch_all_guis(self):
        for sn in ("mission_planning_gui.py", "monitoring_gui.py", "decision_support_gui.py"):
            self._launch_gui(sn)
        QTimer.singleShot(1000, lambda: self._set_mode_text_all("전원 ON"))
        QTimer.singleShot(1000, lambda: self._broadcast_ctrl({"cmd": "mode", "text": "전원 ON"}))
        QTimer.singleShot(1000, lambda: self._safe_log("모든 SW 전원 ON"))

    def _launch_gui(self, script_name: str):
        import sys, os, subprocess
        from pathlib import Path

        root = Path(__file__).resolve().parent
        modules_dir = root / "modules"

        # ✅ 후보 경로 확장 + 가시 로그
        candidates = [
            root / script_name,
            modules_dir / script_name,                           # modules 바로 아래
            modules_dir / "mission_planning" / script_name,
            modules_dir / "monitoring" / script_name,
            modules_dir / "decision_support" / script_name,
        ]
        script = next((p for p in candidates if p.exists()), None)
        if script is None:
            msg = f"[RUN ERR] not found: {script_name}\n - searched:\n   " + "\n   ".join(str(c) for c in candidates)
            try: self._safe_log(msg)
            except Exception: pass
            try: sys.stderr.write(msg + "\n")
            except Exception: pass
            return

        # DB root
        ui_line = getattr(self.win, "_db_path_line", None)
        ui_val = ui_line.text().strip() if ui_line and hasattr(ui_line, "text") else ""
        db_root = ui_val or os.environ.get("KU_MISSION_DB_ROOT") or str(root / "database")
        try: Path(db_root).mkdir(parents=True, exist_ok=True)
        except Exception: pass

        # 환경변수
        env = os.environ.copy()
        env.setdefault("KU_LAUNCHED_BY_DASHBOARD", "1")
        env["KU_MISSION_DB_ROOT"] = db_root

        # ✅ 포트 매핑(대시보드 브로드캐스트와 일치)
        port_map = {
            "mission_planning_gui.py": "45981",  # assignment
            "monitoring_gui.py":       "45982",  # monitoring
            "decision_support_gui.py": "45983",  # decision
        }
        try:
            script_basename = script.name
        except Exception:
            script_basename = script_name
        env["KU_CTRL_PORT"] = port_map.get(script_basename, port_map.get(script_name, ""))

        # ✅ 실행 전 가시 로그
        try: self._safe_log(f"[RUN] {script_basename} @ {script}")
        except Exception: pass

        # Windows에서 콘솔 확인이 필요하면 CREATE_NEW_CONSOLE 사용(선택)
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

    def _append_log_global(self, text: str):
        ed = self.widgets.get("log_edit")
        if ed is None:
            return
        try:
            p = ed.parent()
            while p is not None:
                if isinstance(p, ModuleWithLog):
                    return
                p = p.parent()
        except Exception:
            pass
        try:
            if isinstance(ed, QPlainTextEdit): ed.appendPlainText(text)
            elif isinstance(ed, QTextEdit): ed.append(text)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    _load_qss(app)
    win = MainWindow()
    win.show()
    orch = DashboardOrchestrator(win)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
