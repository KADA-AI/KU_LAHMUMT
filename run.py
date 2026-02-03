# /run.py
# -*- coding: utf-8 -*-
# KU_LAHMUMT 대시보드 실행 & 모듈 모니터링/관리 전용
# -------------------------------------------------------------

from __future__ import annotations
import os, sys, subprocess, threading, json
from datetime import datetime, timezone
os.environ["KU_ROLE"] = "decision"
from pathlib import Path
from typing import Dict, List, Tuple

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, QObject, pyqtSignal, QTimer, QMetaObject, Qt, Q_ARG, pyqtSlot
from PyQt5.QtGui import QCursor, QTextCursor
from PyQt5.QtWidgets import QApplication, QTextEdit, QPlainTextEdit

from modules.common import db_paths
try:
    from modules.monitoring.utils.vehicle_status import write_vehicle_status
except ModuleNotFoundError:
    try:
        from modules.monitoring.utils.vehicle_status import write_vehicle_status
    except ModuleNotFoundError:
        def write_vehicle_status(_available_ids=None):
            return


def _debug_log(message: str) -> None:
    # Debug logging disabled to avoid creating log files
    return

# -------------------------------------------------------------
# Qt 경고 필터 (선택)
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    if message.startswith("QMainWindowLayout::"):
        return
    sys.stderr.write(message + "\n")
qInstallMessageHandler(_qt_silent_handler)

# -------------------------------------------------------------
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


hardened_base_ids = {
    "missionPlanID": 700_000_000,
    "individualMissionPackage": 800_000_000,
    "individualMission": 900_000_000,
    "pathID": {
        1: 100_000_000,
        2: 200_000_000,
        3: 300_000_000,
        4: 400_000_000,
        5: 500_000_000,
        6: 600_000_000,
    },
}


def _force_reset_id_files() -> None:
    """
    Override mission/IMP/path counters to a fixed baseline on each launch.
    This writes directly to data_def id tracker files regardless of runtime state.
    """
    try:
        data_def_dir = PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner" / "data_def"
        data_def_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        return

    tracker_path = data_def_dir / "id_tracker.json"
    tracker_0202_path = data_def_dir / "id_tracker_0202.json"
    counters_path = data_def_dir / "_id_counters.json"

    tracker_payload = {
        "missionPlanID": hardened_base_ids["missionPlanID"],
        "individualMissionPackage": hardened_base_ids["individualMissionPackage"],
        "individualMission": hardened_base_ids["individualMission"],
        "pathID": hardened_base_ids["pathID"],
    }
    tracker_0202_payload = {"individualMissionID": 950_000_000}
    counters_payload = {
        "missionPlanID": hardened_base_ids["missionPlanID"],
        "impPackageID": hardened_base_ids["individualMissionPackage"],
    }

    try:
        tracker_path.write_text(
            json.dumps(tracker_payload, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
    except Exception:
        pass
    try:
        tracker_0202_path.write_text(
            json.dumps(tracker_0202_payload, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
    except Exception:
        pass
    try:
        counters_path.write_text(
            json.dumps(counters_payload, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
    except Exception:
        pass

    # Clear path/waypoint usage so id_allocator seeding does not pull old maxima.
    try:
        root = db_paths.bootstrap_db_root()
        dss_dir = root / "DSS_Internal"
        for fname in ("path_usage.json", "waypoint_usage.json"):
            target = dss_dir / fname
            if target.exists():
                try:
                    target.unlink()
                except Exception:
                    pass
    except Exception:
        pass


_force_reset_id_files()
db_paths.bootstrap_db_root()

# -------------------------------------------------------------
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

# -------------------------------------------------------------
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
            # License files can contain non-text bytes; copy as binary to avoid decode errors.
            lic_dst.write_bytes(lic_src.read_bytes())
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

# -------------------------------------------------------------

# ---------------------------------------------------------------------------
# ID counter reset on cold start (missionPlan/IMP/path/0202 IDs)
def _reset_id_counters() -> None:
    """
    Reset mission/IMP/path ID trackers so each app launch starts from base seeds.
    This is intentionally aggressive: existing counter files are overwritten.
    """
    try:
        from modules.mission_planning.MissionPlanner.data_def import id_allocator
        from modules.mission_planning.MissionPlanner.data_def import id_allocator_0202
    except Exception:
        return

    base = id_allocator.BASE
    target = {
        # Reset so the next allocation starts at the *_001 baseline.
        "missionPlanID": max(0, int(base.get("missionPlanID", 0) - 1)),
        "individualMissionPackage": max(0, int(base.get("individualMissionPackage", 0) - 1)),
        "individualMission": max(0, int(base.get("individualMission", 0) - 1)),
        "pathID": {int(k): max(0, int(v) - 1) for k, v in (base.get("pathID") or {}).items()},
    }
    try:
        store = Path(id_allocator.__file__).resolve().parent / "id_tracker.json"
        store.parent.mkdir(parents=True, exist_ok=True)
        # reset in-memory state as well as file
        new_state = dict(target)
        try:
            id_allocator._state.clear()  # type: ignore[attr-defined]
            id_allocator._state.update(new_state)  # type: ignore[attr-defined]
            id_allocator._volatile_counters = {  # type: ignore[attr-defined]
                key: base.get(key, 1) - 1 for key in id_allocator.VOLATILE_KEYS  # type: ignore[attr-defined]
            }
        except Exception:
            pass
        store.write_text(
            json.dumps(new_state, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
    except Exception:
        pass

    try:
        base_0202 = getattr(id_allocator_0202, "_BASE_STATE", {"individualMissionID": 0})
        state_0202 = {"individualMissionID": base_0202.get("individualMissionID", 0)}
        try:
            if hasattr(id_allocator_0202, "_STATE"):
                id_allocator_0202._STATE.clear()  # type: ignore[attr-defined]
                id_allocator_0202._STATE.update(state_0202)  # type: ignore[attr-defined]
        except Exception:
            pass
        store_0202 = Path(id_allocator_0202.__file__).resolve().parent / "id_tracker_0202.json"
        store_0202.write_text(
            json.dumps(state_0202, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
    except Exception:
        pass

    try:
        counter_file = Path(id_allocator.__file__).resolve().parent / "_id_counters.json"
        counter_payload = {
            "missionPlanID": base.get("missionPlanID", 0),
            "impPackageID": base.get("individualMissionPackage", 0),
        }
        counter_file.write_text(
            json.dumps(counter_payload, ensure_ascii=False, separators=(",", ":")),
            encoding="utf-8",
        )
    except Exception:
        pass
# 메인 윈도우 로드(경로 가변 지원)
try:
    from app.ui.main_window import MainWindow  # type: ignore
except Exception:
    from main_window import MainWindow  # type: ignore

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

# -------------------------------------------------------------
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

# -------------------------------------------------------------
# 대시보드 오케스트레이터 (모니터링/관리 전용)
class DashboardOrchestrator(QObject):
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

        self._manage_info_proc = None
        self._latest_db_payloads = {}
        self._push_suppression = {}
        self._standby_pending = False
        self._scenario_activated_once = False

        # 모드 루프 방지용 상태
        self._mode_text = "초기화 모드"
        self._last_mode_broadcast_ms = 0.0

        # 안전 로거 alias
        self._log_everywhere = getattr(self, "_log_everywhere", None) or (lambda text: self._append_log_global(str(text)))
        self._safe_log = lambda text: self._log_everywhere(text) if hasattr(self, "_log_everywhere") else self._append_log_global(text)

        self._fallback_log_role = os.getenv("KU_MON_FALLBACK_LOG", "decision")
        self.uiLog.connect(self._append_log_global)       # 단일 문자열 -> 글로벌/폴백
        self.uiLog2.connect(self._log_to_role)            # (role, text) -> 해당 모듈

        self._init_rows()
        reset_btn = getattr(self.win, "btn_decision_reset", None)
        if reset_btn is not None and hasattr(reset_btn, "clicked"):
            try:
                reset_btn.clicked.connect(self._handle_decision_reset_button)
            except Exception:
                pass

        self._start_bus_monitor()
        # UDP 기반 모니터링 채널 제거됨

        # 시작 시 초기화 모드
        self._set_mode_text_all("초기화 모드")

        self._module_mode = {"assignment": "초기화 모드", "monitoring": "초기화 모드", "decision": "초기화 모드", "info": "초기화 모드"}

        self._last_system_mode_code: int | None = None
        info = db_paths.get_info()
        self._scenario_timestamp = info.get("timestamp_ms")
        try:
            db_root_init = info.get("db_root") or db_paths.get_active_db_root_str()
            self.win.update_db_root(db_root_init)
            self.win.update_scenario_root(info.get("base_root"))
            tooltip = f"{info.get('iso') or '->->'} @ {db_root_init}"
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


    def _init_rows(self):
        # * 행을 채울 때도 4자리로 정규화해서, 나중에 bump_rx("0301")이 정확히 매칭되게
        for key, defs in self.msg_map.items():
            mod = self.widgets.get(key)
            if not mod:
                continue
            tx_pairs = [(self._norm_code(mid), 0) for (mid, _name) in defs.get("tx", [])]  # *
            rx_pairs = [(self._norm_code(mid), 0) for (mid, _name) in defs.get("rx", [])]  # *
            set_tx = getattr(mod, "set_tx_rows", None)
            if callable(set_tx):
                set_tx(tx_pairs)
            set_rx = getattr(mod, "set_rx_rows", None)
            if callable(set_rx):
                set_rx(rx_pairs)


    # --------- mode 이벤트 처리(수신->대시보드/모듈 전체 동기화) ---------
    def _normalize_mode_text(self, text: str) -> str:
        t = "".join(str(text).split()).lower()
        if t in ("전원off", "off", "poweroff", "전원on", "on", "poweron"): return "초기화 모드"
        if t in ("초기화모드", "초기화", "init", "initialize", "initialization", "0"): return "초기화 모드"
        if t in ("대기모드", "대기", "standby", "1"): return "대기모드"
        if t in ("초기임무계획", "초기임무계획모드", "initplan", "initialplan", "initial", "2", "초기임무계획모드진입"): return "초기임무계획"
        if t in ("임무수행", "임무수행모드", "execution", "missionmode", "3"): return "임무 수행"
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
        self._safe_log(f"[MODE] {role} -> {norm}")

    # --------- UI 위젯 해결 ---------

    def _apply_dashboard_button_styles(self) -> None:
        """Set the module shutdown button to green."""
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
        targets = (getattr(self.win, "btn_module_shutdown", None),)
        for btn in targets:
            if btn is None:
                continue
            try:
                btn.setStyleSheet(green_style)
            except Exception:
                pass

    def _resolve_widgets(self, win):
        log_edit = None
        try:
            from PyQt5.QtWidgets import QPlainTextEdit, QTextEdit
            # Only treat widgets explicitly tagged as logs to avoid dumping debug text in memo fields.
            for widget in win.findChildren((QPlainTextEdit, QTextEdit)):
                name = (widget.objectName() or "").lower()
                if "log" not in name:
                    continue
                log_edit = widget
                break
        except Exception:
            pass

        modules = {
            "assignment": getattr(win, "module_mission", None),
            "monitoring": getattr(win, "module_monitor", None),
            "decision":   getattr(win, "module_decision", None),
        }

        return {"log_edit": log_edit, **modules}

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

        # 소스 모듈 추정 -> role 정규화
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
        # 0101 SystemMode==2 -> 초기임무계획 모드 진입
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

        # 0702 수신 -> 대기모드 전환
        if mid == "0702":
            self._safe_log("[OPS] 0702 수신 -> 대기모드 전환")
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
            self._safe_log("[OPS] SystemMode=2 수신 -> 초기임무계획 모드 진입")
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
                self.win.update_scenario_status_indicator(False, "타임스탬프 없음 -> 기존 경로 유지")
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
        """User-triggered reset: shutdown modules, reset DB state, then relaunch."""
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

        try:
            _force_reset_id_files()
        except Exception:
            pass
        try:
            _reset_id_counters()
        except Exception:
            pass

        self._scenario_activated_once = False
        self._scenario_timestamp = None
        try:
            tooltip = f"Reset 대기: {prev_root}" if prev_root else "Reset 대기: DB 미선택"
            self.win.update_scenario_status_indicator(False, tooltip)
        except Exception:
            pass

        self._last_system_mode_code = None
        self._standby_pending = False

        log_msg = "[OPS] SW reset requested - relaunching modules (new DB on next standby)"
        if prev_root:
            log_msg += f" (prev DB: {prev_root})"
        try:
            self._safe_log(log_msg)
        except Exception:
            pass

        try:
            self._set_mode_text_all("모듈 초기화 중")
        except Exception:
            pass

        QTimer.singleShot(800, self._launch_all_guis)

    # --------- 모드/CTRL/런처/로깅 유틸 ---------
    def _log_assignment(self, text: str):
        mod = self.widgets.get("assignment")
        if mod and hasattr(mod, "append_log"):
            try: mod.append_log(text)
            except Exception: pass
        self._append_log_global(text)

    def _enter_standby(self):
        self._set_mode_text_all("대기모드")
        self._safe_log("모든 SW 대기모드 진입")

    def _enter_initial_plan(self):
        self._set_mode_text_all("초기임무계획")
        self._safe_log("모든 SW 초기임무계획 모드")

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

    def trigger_initial_plan_pipeline(self, reason: str = "초기임무재계획") -> None:
        normalized_reason = str(reason or "초기임무재계획")
        self._safe_log(
            f"[OPS] S110: initial mission planning requested (reason={normalized_reason})"
        )


    def _launch_all_guis(self):
        for sn in ("mission_planning_gui.py", "monitoring_gui.py", "decision_support_gui.py", "info_manage.py"):
            self._launch_gui(sn)
        QTimer.singleShot(1000, lambda: self._set_mode_text_all("초기화 모드"))
        QTimer.singleShot(1000, lambda: self._safe_log("모든 SW 초기화 모드 진입"))

    def _self_check_all(self, on: bool = True) -> None:
        """Best-effort self-check trigger for any embedded modules."""
        status = 1 if on else 0
        try:
            self._safe_log(f"[OPS] self_check broadcast requested status={status}")
        except Exception:
            pass
        for key in ("assignment", "monitoring", "decision", "info"):
            mod = self.widgets.get(key)
            sender = getattr(mod, "_send_self_check_0102", None)
            if callable(sender):
                try:
                    sender(status=status)
                except Exception:
                    pass

    def _launch_gui(self, script_name: str):
        import sys, os, subprocess
        from pathlib import Path

        root = Path(__file__).resolve().parent
        modules_dir = root / "modules"
        script_alias = script_name
        extra_args: list[str] = []

        candidates = []
        if script_name == "monitoring_gui.py":
            candidates.append(modules_dir / "monitoring" / "test_monitoring.py")
        candidates += [
            root / script_alias,
            modules_dir / script_alias,
            modules_dir / "mission_planning" / script_alias,
            modules_dir / "monitoring" / script_alias,
            modules_dir / "decision_support" / script_alias,
            modules_dir / "info_manage" / script_alias,
        ]
        script = next((p for p in candidates if p.exists()), None)
        if script is None:
            msg = f"[RUN ERR] not found: {script_name}\n - searched:\n   " + "\n   ".join(str(c) for c in candidates)
            try: self._safe_log(msg)
            except Exception: pass
            try: sys.stderr.write(msg + "\n")
            except Exception: pass
            return
        if script.name == "test_monitoring.py":
            extra_args = ["--gui"]

        ui_line = getattr(self.win, "_db_path_line", None)
        ui_val = ui_line.text().strip() if ui_line and hasattr(ui_line, "text") else ""
        db_root = ui_val or db_paths.get_active_db_root_str()
        try: Path(db_root).mkdir(parents=True, exist_ok=True)
        except Exception: pass

        env = os.environ.copy()
        env.setdefault("KU_LAUNCHED_BY_DASHBOARD", "1")
        env["KU_MISSION_DB_ROOT"] = db_root

        try:
            script_basename = script.name
        except Exception:
            script_basename = script_alias
        env.pop("KU_WINDOW_OFFSET", None)

        try: self._safe_log(f"[RUN] {script_basename} @ {script}")
        except Exception: pass

        creationflags = 0
        if os.name == "nt":
            creationflags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)

        role_map = {
            "mission_planning_gui.py": "mission",
            "monitoring_gui.py": "monitor",
            "test_monitoring.py": "monitor",
            "decision_support_gui.py": "decision",
            "info_manage.py": "info",
        }
        role_key = role_map.get(script_basename)
        role_processes = getattr(self.win, "_role_processes", None)
        if role_key and isinstance(role_processes, dict):
            existing = role_processes.get(role_key)
            if existing and existing.poll() is None:
                try:
                    self._safe_log(f"[RUN] already running: {script_basename}")
                except Exception:
                    pass
                return

        try:
            proc = subprocess.Popen(
                [sys.executable, str(script), *extra_args],
                cwd=str(root),
                env=env,
                start_new_session=True,
                creationflags=creationflags,
            )
            if role_key and isinstance(role_processes, dict):
                role_processes[role_key] = proc
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
        return

    def _append_log_global(self, text: str):
        ed = self.widgets.get("log_edit")
        if ed is not None:
            try:
                if isinstance(ed, QPlainTextEdit): ed.appendPlainText(text)
                elif isinstance(ed, QTextEdit): ed.append(text)
                else:  # 예외적으로 타입이 다르면 폴백700
                    raise RuntimeError("no global text edit")
                return
            except Exception:
                pass

        # 전역 로그 위젯이 없거나 실패 -> 폴백 모듈로
        fb = self.widgets.get(self._fallback_log_role)
        if fb and hasattr(fb, "append_log"):
            try:
                fb.append_log(text)
                return
            except Exception:
                pass

        # 최종 폴백: 콘솔(옵션) -> 기본적으로는 터미널에 출력하지 않는다.
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


# -------------------------------------------------------------
def main():
    try:
        _reset_id_counters()
    except Exception:
        pass
    app = QApplication(sys.argv)
    _load_qss(app)
    win = MainWindow()
    win.show()
    _position_window_at_cursor(app, win)
    orch = DashboardOrchestrator(win)
    QTimer.singleShot(800, orch._launch_all_guis)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
