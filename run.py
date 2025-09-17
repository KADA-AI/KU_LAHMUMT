# -*- coding: utf-8 -*-
# run.py – KU_LAHMUMT 대시보드 실행 & 모듈 연동
from __future__ import annotations

import os, sys, subprocess, threading
os.environ["KU_ROLE"] = "decision"
from pathlib import Path
from typing import Dict, List, Tuple, Optional


from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, QObject, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QTextEdit, QPlainTextEdit


# ─────────────────────────────────────────────────────────────
# Qt 경고 필터 (선택)
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    if message.startswith("QMainWindowLayout::"):
        return  # 레이아웃 카운트/추가 경고 숨김
    sys.stderr.write(message + "\n")

qInstallMessageHandler(_qt_silent_handler)

def _ku_fp_worker(kind: str, missions: list, speed: float, sys_paths_snapshot: list):
    """
    kind: '0303' | '0304'
    missions: 0302 기반 임무 목록(직렬화 가능해야 함)
    speed: 크루즈 속도
    sys_paths_snapshot: 부모 프로세스의 sys.path 스냅샷(자식에서 import 경로 맞추기용)
    """
    import sys
    for p in sys_paths_snapshot:
        if p not in sys.path:
            sys.path.insert(0, p)

    # 여기서 새로 import (자식 프로세스 환경)
    from data_def import d0303, d0304

    wp_alloc = d0303._WPAllocator()
    if kind == "0303":
        # UAV 4~6 경로 생성
        return d0303.build_flight_plans(missions, wp_alloc, speed, turn_step_deg=15.0)
    else:
        # LAH 1~3 경로 생성 (고정 100m 분할 버전)
        return d0304.build_lah_flight_plans_fixed(missions, cruise_speed=speed, wp_alloc=wp_alloc)
    
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
        os.chdir(root)  # CWD 고정(설정/라이선스/어셈블리 검색 안정)
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
    # settings
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
    # license (있으면 루트로 정규화)
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
    # 의존 DLL(있으면)
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

# NodeMessenger 가져오기
NFusion_OK = False
try:
    from dll_files.nFusionImports import FusionNodeIoc, NodeMessenger  # type: ignore
    NFusion_OK = True
except Exception:
    NFusion_OK = False


# ─────────────────────────────────────────────────────────────
# 메시지 정의 읽기 유틸
def _normalize_defs(defs) -> List[Tuple[str, str]]:
    """다양한 형식의 메시지 정의를 (msg_id, label) 리스트로 통일."""
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
    """
    각 탭의 PUSH/RECEIVE 메시지 정의를 불러와서
    {module_key: {'tx': [(id,label)...], 'rx': [...]} } 형태로 반환
    module_key ∈ {'assignment','monitoring','decision'}
    """
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
# 대시보드 오케스트레이터 (버튼/목록/카운트/애니메이션/로그)
class DashboardOrchestrator(QObject):
    dashEvent = pyqtSignal(str, str)   # (kind, msg_id) → 메인 스레드에서 _handle_dash_event 호출
    uiLog     = pyqtSignal(str)        # 텍스트 로그를 메인 스레드에서 기록
    def __init__(self, window: MainWindow):
        super().__init__(window)
        self.win = window
        self.widgets = self._resolve_widgets(window)
        self.msg_map = _load_tab_defs()
        self._wire_operation_panel()

        self.dashEvent.connect(self._handle_dash_event)
        self.uiLog.connect(self._log_assignment)

        self._init_rows()
        self._connect_launch_buttons()
        self._start_bus_monitor()
        self._start_dashboard_socket()

        # [추가] 앱 시작 시, 초기 모드를 전원 OFF로 강제
        self._set_mode_text_all("전원 OFF")
        self._broadcast_ctrl({"cmd": "mode", "text": "전원 OFF"})

    def _start_dashboard_socket(self):
        """
        모듈 GUI들이 보내는 로컬 UDP 이벤트(kind: 'tx'|'rx', msg_id)를 수신해
        짝 여부와 무관하게 해당 모듈의 OUT/IN을 시각화한다.
        (QTimer.singleShot 대신, cross-thread signal emit 사용 → QThread 경고 제거)
        """
        import socket, json, time, threading

        self._seen_evt_ts = {}  # key -> last_monotonic

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
                    # ⬇️ 변경 포인트: 타이머 대신 시그널 emit ⇒ 메인 스레드에서 처리
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


    def _call_fp_with_timeout(self, kind: str, missions: list, speed: float, timeout_s: int = 60):
        """
        0303/0304 FP 생성을 서브프로세스에서 실행하고 timeout으로 감시.
        kind: '0303' 또는 '0304'
        """
        import sys, time, concurrent.futures
        sys_paths_snapshot = sys.path[:]  # 자식에서 동일 경로로 import 되도록 전달

        t0 = time.perf_counter()
        try:
            with concurrent.futures.ProcessPoolExecutor(max_workers=1) as ex:
                fut = ex.submit(_ku_fp_worker, kind, missions, speed, sys_paths_snapshot)
                res = fut.result(timeout=timeout_s)
            dt = time.perf_counter() - t0
            self._post_log_assignment(f"[OK] {kind} 생성 완료 ({len(res)}개, {dt:.1f}s)")
            return res
        except concurrent.futures.TimeoutError:
            dt = time.perf_counter() - t0
            raise TimeoutError(f"{kind} 생성이 {timeout_s}s를 초과하여 중단됨 ({dt:.1f}s 경과)")
        
    def _norm_code(self, mid) -> str:
        s = str(mid)
        return s.zfill(4) if s.isdigit() and len(s) < 4 else s

    def _handle_dash_event(self, kind: str, msg_id: str):
        """
        UDP로 들어온 이벤트를 모듈 정의에 따라 반영한다.
        kind: 'tx' → OUT, 'rx' → IN
        """
        mid = str(msg_id)
        # UDP와 BUS가 같은 메시지를 거의 동시 전달할 때 중복 반영 방지
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
            "decision": getattr(win, "module_decision", None),
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
        """서로 다른 소스(udp/bus) 간 중복 처리 방지용 타임윈도우 디듀프"""
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
        """
        각 모듈 카드의 송신/수신 목록을 '코드,카운트(초기 0)' 형태로 세팅한다.
        ModuleWithLog.set_tx_rows/set_rx_rows는 [(code, count), ...] 포맷을 기대한다.
        """
        for key, defs in self.msg_map.items():
            mod = self.widgets.get(key)
            if not mod:
                continue

            # (msg_id, msg_name) -> (msg_id, 0) 로 변환
            tx_pairs = [(mid, 0) for (mid, _name) in defs.get("tx", [])]
            rx_pairs = [(mid, 0) for (mid, _name) in defs.get("rx", [])]

            # API 호출
            set_tx = getattr(mod, "set_tx_rows", None)
            if callable(set_tx):
                set_tx(tx_pairs)

            set_rx = getattr(mod, "set_rx_rows", None)
            if callable(set_rx):
                set_rx(rx_pairs)

    def _log_everywhere(self, text: str):
        """
        대시보드 전역 로그 + 3개 모듈 카드에 모두 기록하되,
        동일 카드에 같은 문구가 0.8s 안에 중복 기록되지 않도록 막는다.
        """
        self._append_log_global(text)

        import time
        t = time.monotonic()
        last_map = getattr(self, "_last_card_log", {})  # wid -> (last_text, last_ts)

        # 세 카드에만 브로드캐스트
        for key in ("assignment", "monitoring", "decision"):
            mod = self.widgets.get(key)
            if not mod or not hasattr(mod, "append_log"):
                continue

            wid = id(mod)
            last = last_map.get(wid)
            if last and last[0] == text and (t - last[1]) < 0.2:
                continue

            try:
                mod.append_log(text)
            except Exception:
                pass

            last_map[wid] = (text, t)

        self._last_card_log = last_map

    # --------- 버튼 → 서브프로세스 실행 ---------
    # --------- operation state shortcuts ---------
    def _handle_operation_state(self, code: str):
        handlers = {
            "S100": self._handle_state_s100,
        }
        handler = handlers.get(code)
        if handler:
            handler()
        else:
            self._log_everywhere(f"[OPS] {code} 상태는 아직 연결되지 않았습니다.")

    def _handle_state_s100(self):
        self._log_everywhere("[OPS] S100 초기화 모드 실행")
        self._set_mode_text_all("초기화")
        self._launch_all_guis()
        QTimer.singleShot(2000, lambda: self._self_check_all(True))


    def _connect_launch_buttons(self):
        """
        - 'SW 실행' 버튼 → 세 모듈 GUI를 한 번에 실행
        - 'SW 자체점검' → 각 모듈 GUI에 0102(status=1) 명령 브로드캐스트
        - '대기모드'   → 0102(status=0) + 대시보드 모드 텍스트 '대기모드'
        - '초기임무계획모드' → 전체 자동 플로우 실행(0201/0203 로드→0301~0304 생성/저장→대시보드 코드펄스)
        - 개별 GUI 실행은 각 ModuleWithLog 카드의 'GUI 실행' 버튼으로만 연결
        """
        try:
            from PyQt5.QtWidgets import QPushButton
        except Exception:
            return

        # ① SW 실행
        btn_all = self._find_button(("sw 실행", "전체 실행", "all run", "launch all", "run all", "전체 구동"))
        if btn_all is not None:
            btn_all.clicked.connect(lambda _=False: self._launch_all_guis())

        # ② SW 자체점검
        btn_self = self._find_button(("sw 자체점검", "자체점검", "self check", "self-check", "selfcheck"))
        if btn_self is not None:
            btn_self.clicked.connect(lambda _=False: self._self_check_all(True))

        # ③ 대기모드
        btn_standby = self._find_button(("대기모드", "대기", "standby"))
        if btn_standby is not None:
            btn_standby.clicked.connect(lambda _=False: self._enter_standby())

        # ④ 초기임무계획모드  👉 전체 자동 플로우 연결 (기존 _enter_initial_plan 에서 변경)
        btn_init = self._find_button(("초기임무계획모드", "초기임무계획", "initial", "init plan"))
        if btn_init is not None:
            try:
                btn_init.clicked.disconnect()
            except Exception:
                pass
            btn_init.clicked.connect(lambda _=False: self._enter_initial_plan())

        # ⑤ 개별 GUI 실행: 각 카드의 'GUI 실행' 버튼에만 연결
        mapping = {
            "assignment": "mission_planning_gui.py",
            "monitoring": "monitoring_gui.py",
            "decision":   "decision_support_gui.py",
        }
        for key, script in mapping.items():
            card = self.widgets.get(key)
            if not card:
                continue
            btn = getattr(card, "btn_run", None)
            if btn and hasattr(btn, "clicked"):
                try:
                    btn.clicked.disconnect()
                except Exception:
                    pass
                btn.clicked.connect(lambda _=False, sn=script: self._launch_gui(sn))


    def _enter_initial_plan_and_run(self):
        """
        초기 임무 계획 모드(완전 자동):
        1) 임무할당모드에 '임무 계획 수행 중' 로그
        2) main_MP 파이프라인을 GUI 없이(headless) 자동 실행
        3) load 0201/0203에서 파일 자동 선택 후 입력 처리
        4) 0304 탭의 save missions에 해당하는 디스크 저장 수행
        5) 저장 완료되면 0304→0303→0302(0.2s 간격), 1초 후 0301 코드 펄스
        6) 완료 로그
        """
        # 기존 모드 전환 브로드캐스트(+카드 모드 텍스트)는 유지
        self._enter_initial_plan()

        # 1) 카드(할당) 로그 + 전역 로그
        self._log_assignment("임무 계획 수행 중...")

        # 2~5) 헤드리스 파이프라인 비동기 실행
        self._run_headless_pipeline_async()


    def _run_headless_pipeline_async(self):
        import threading
        th = threading.Thread(target=self._run_headless_pipeline_do, name="InitPlan-Headless", daemon=True)
        th.start()

    def _run_headless_pipeline_do(self):
        """
        main_MP GUI 없이 동일한 파이프라인을 수행:
        - 입력: database/InputMissionPlan, database/MissionReferenceInfo 의 첫 번째 *.json
        - 처리: 0201+0203→IMP(0302), 0301 MissionPlan, 0303/0304 FlightPath
        - 저장: database/MissionPlan, IndividualMissionPlan, FlightPath
        """
        import os, sys, json, time, shutil
        from pathlib import Path

        # 0) 경로 세팅
        root = Path(__file__).resolve().parent
        modules_dir = root / "modules"
        mp_pkg_dir  = modules_dir / "mission_planning" / "MissionPlanner"
        for p in (mp_pkg_dir, mp_pkg_dir.parent, modules_dir):
            p_str = str(p)
            if p.exists() and p_str not in sys.path:
                sys.path.insert(0, p_str)

        # 1) data_def / AnS import
        try:
            from AnS import run_divide_and_pattern, build_mission_plan_0301
            from data_def import d0301, d0302, d0303, d0304
            from data_def.id_allocator import next_path_id
        except Exception as e:
            self._post_log_assignment(f"[ERR] pipeline import 실패: {e}")
            return

        # 유틸
        def _imp_path_id(im: dict) -> int | None:
            for k in ("pathID","pathId","individualMissionPathID","missionPathID"):
                v = im.get(k)
                try:
                    if v is not None:
                        return int(v)
                except Exception:
                    pass
            mi = im.get("missionInfo")
            if isinstance(mi, dict):
                for k in ("pathID","pathId"):
                    v = mi.get(k)
                    try:
                        if v is not None:
                            return int(v)
                    except Exception:
                        pass
            return None

        def _enforce_fp_path_ids(fps: list[dict], pid_map: dict[tuple[int,int], int]) -> int:
            fixed = 0
            for fp in fps or []:
                try:
                    aid = int(fp.get("aircraftID", 0))
                    mid = int(fp.get("individualMissionID", 0))
                    key = (aid, mid)
                    if key in pid_map:
                        desired = int(pid_map[key])
                        if str(fp.get("pathID")) != str(desired):
                            fp["pathID"] = desired
                            fixed += 1
                except Exception:
                    pass
            return fixed

        # 2) 입력 파일 자동 선택
        db_root = Path(os.environ.get("KU_MISSION_DB_ROOT") or (root / "database"))
        dir_0201 = db_root / "InputMissionPlan"
        dir_0203 = db_root / "MissionReferenceInfo"
        dir_0201.mkdir(parents=True, exist_ok=True)
        dir_0203.mkdir(parents=True, exist_ok=True)

        def _pick_json(d: Path) -> Path | None:
            cands = sorted([p for p in d.glob("*.json") if p.is_file()])
            return cands[0] if cands else None

        cmpk_path = _pick_json(dir_0201)
        mrpk_path = _pick_json(dir_0203)
        if not cmpk_path or not mrpk_path:
            self._post_log_assignment("[ERR] 0201/0203 입력 JSON을 찾을 수 없습니다.")
            return

        # 3) 0201+0203 → IMP(0302)
        out_root = db_root / "mission_output"
        out_root.mkdir(parents=True, exist_ok=True)
        try:
            imp_paths = run_divide_and_pattern(
                cmpk_path=str(cmpk_path),
                ref_path=str(mrpk_path),
                out_dir=str(out_root),
                log=lambda msg: self._post_log_assignment(str(msg))
            )
            if not imp_paths:
                raise RuntimeError("IMP 생성 결과가 없습니다.")
            self._post_log_assignment(f"[OK] IMP {len(imp_paths)}개 생성")
        except Exception as e:
            self._post_log_assignment(f"[ERR] divide/pattern 실패: {e}")
            return

        # 4) 0301 MissionPlan 생성 & 적재
        mp_path = out_root / f"MissionPlan_{int(time.time()*1000)}.json"
        try:
            build_mission_plan_0301(str(cmpk_path), str(mrpk_path), imp_paths, str(mp_path))
            with mp_path.open(encoding="utf-8") as f:
                mp_json = json.load(f)
            imp_id_map = {a["aircraftID"]: a["individualMissionPackageID"] for a in mp_json.get("aircraftList", [])}
            self._post_log_assignment(f"[OK] 0301 생성 → {mp_path.name}")
        except Exception as e:
            self._post_log_assignment(f"[ERR] 0301 생성 실패: {e}")
            return

        # 5) missions 집계 + pathID 매핑(0302/0303/0304 일치 보장)
        missions = []; pid_map = {}
        try:
            for imp in imp_paths:
                with open(imp, encoding="utf-8") as f:
                    pkg = json.load(f)
                aid = int(pkg["aircraftID"])
                for im in pkg.get("individualMissionList", []):
                    im2 = dict(im); im2["aircraftID"] = aid
                    if "individualMissionPlanPackageID" not in im2 and imp_id_map:
                        im2["individualMissionPlanPackageID"] = imp_id_map.get(aid)

                    mid = int(im2.get("individualMissionID", 0))
                    if aid in (1,2,3):
                        pid = int(next_path_id(aid))  # LAH: 합법 ID 재발급
                        im2["pathID"] = pid
                        pid_map[(aid, mid)] = pid
                    else:
                        imp_pid = _imp_path_id(im2)
                        if imp_pid is not None:
                            im2["pathID"] = int(imp_pid)
                            pid_map[(aid, mid)] = int(imp_pid)

                    missions.append(im2)
        except Exception as e:
            self._post_log_assignment(f"[ERR] IMP 집계 실패: {e}")
            return

        # 6) 0303 / 0304 생성
        self._post_log_assignment("[STEP 5] FlightPath 0303/0304 생성 시작")
        to3 = int(os.getenv("KU_FP3_TIMEOUT_S", "60"))
        to4 = int(os.getenv("KU_FP4_TIMEOUT_S", "60"))

        manned_missions = [im for im in missions if int(im.get("aircraftID", 0)) in (1,2,3)]
        uav_missions    = [im for im in missions if int(im.get("aircraftID", 0)) in (4,5,6)]

        flight_plans_0303, flight_plans_0304 = [], []
        try:
            if hasattr(self, "_call_fp_with_timeout"):
                flight_plans_0303 = self._call_fp_with_timeout("0303", uav_missions, 40.0, timeout_s=to3)
            else:
                wp_alloc_3 = d0303._WPAllocator()
                flight_plans_0303 = d0303.build_flight_plans(uav_missions, wp_alloc_3, 40.0, turn_step_deg=15.0)
            fixed3 = _enforce_fp_path_ids(flight_plans_0303, pid_map)
            if fixed3:
                self._post_log_assignment(f"[INFO] 0303 FP pathID 강제 적용: fixed={fixed3}")
            self._post_log_assignment(f"[OK] 0303 생성: {len(flight_plans_0303)}개")
        except Exception as e:
            self._post_log_assignment(f"[ERR] 0303 생성 실패/타임아웃: {e}")

        try:
            if not manned_missions:
                self._post_log_assignment("[ERR] 0304 생성 불가: 유인기 임무 없음")
                flight_plans_0304 = []
            else:
                if hasattr(self, "_call_fp_with_timeout"):
                    flight_plans_0304 = self._call_fp_with_timeout("0304", manned_missions, 40.0, timeout_s=to4)
                else:
                    wp_alloc_4 = d0303._WPAllocator()
                    flight_plans_0304 = d0304.build_lah_flight_plans_fixed(manned_missions, cruise_speed=40.0, wp_alloc=wp_alloc_4)
                fixed4 = _enforce_fp_path_ids(flight_plans_0304, pid_map)
                if fixed4:
                    self._post_log_assignment(f"[INFO] 0304 FP pathID 강제 적용: fixed={fixed4}")
                if not flight_plans_0304:
                    self._post_log_assignment("[ERR] 0304 생성 결과가 비어있습니다.")
                else:
                    self._post_log_assignment(f"[OK] 0304 생성: {len(flight_plans_0304)}개")
        except Exception as e:
            self._post_log_assignment(f"[ERR] 0304 생성 실패/타임아웃: {e}")

        if not flight_plans_0303 and not flight_plans_0304:
            self._post_log_assignment("[ERR] 0303/0304 모두 실패 → 파이프라인 중단")
            return

        self._post_log_assignment(f"[OK] FP 생성 요약 → 0303={len(flight_plans_0303)} / 0304={len(flight_plans_0304)}")

        # 7) 디스크 저장 (0301/0302/0303/0304)
        try:
            dir_mp  = db_root / "MissionPlan"
            dir_imp = db_root / "IndividualMissionPlan"
            dir_fp  = db_root / "FlightPath"
            for d in (dir_mp, dir_imp, dir_fp):
                d.mkdir(parents=True, exist_ok=True)
                removed = 0
                for p in d.glob("*.json"):
                    try:
                        p.unlink(); removed += 1
                    except Exception:
                        pass
                if removed:
                    self._post_log_assignment(f"[INFO] 기존 임무 정리: {d.name} ({removed}개 삭제)")

            try:
                mp_id = str(mp_json.get("missionPlanID") or mp_json.get("MissionPlanID"))
            except Exception:
                mp_id = str(int(time.time()))
            (dir_mp / f"{mp_id}.json").write_text(json.dumps(mp_json, indent=2, ensure_ascii=False), encoding="utf-8")

            # 0302: pathID가 이미 주입된 missions 사용
            imp_pkgs = d0302.build_mission_packages(missions, cmpk_id=int(cmpk_path.stem), plan_pkg_map=imp_id_map)
            for pkg in imp_pkgs:
                imp_id = str(pkg.get("individualMissionPackageID") or pkg.get("individualMissionPlanPackageID"))
                (dir_imp / f"{imp_id}.json").write_text(json.dumps(pkg, indent=2, ensure_ascii=False), encoding="utf-8")

            def _dump_fps(lst):
                cnt = 0
                for fp in lst:
                    pid = str(fp["pathID"])
                    (dir_fp / f"{pid}.json").write_text(json.dumps(fp, indent=2, ensure_ascii=False), encoding="utf-8"); cnt += 1
                return cnt
            c3 = _dump_fps(flight_plans_0303)
            c4 = _dump_fps(flight_plans_0304)

            try:
                if out_root.exists():
                    shutil.rmtree(out_root)
                    self._post_log_assignment(f"[INFO] 임시폴더 삭제: {out_root.name}")
            except Exception:
                pass

            self._post_log_assignment(f"✔ 저장 완료  →  MissionPlan 1, IndividualMission {len(imp_pkgs)}, FlightPath {c3 + c4}")
        except Exception as e:
            self._post_log_assignment(f"[ERR] 저장 실패: {e}")
            return

        # 8) 코드 펄스
        self._after_save_sequence()



    def _log_assignment(self, text: str):
        """
        메인 스레드에서 호출되어야 하는 안전한 로그 기록기.
        - 할당 카드(있으면) + 전역 로그에 동시에 기록
        """
        mod = self.widgets.get("assignment")
        if mod and hasattr(mod, "append_log"):
            try:
                mod.append_log(text)
            except Exception:
                pass
        self._append_log_global(text)


    def _after_save_sequence(self):
        """
        저장 후:
        - 0.2초 간격으로 0304 → 0303 → 0302
        - 추가 1초 뒤 0301
        - 그 뒤 완료 로그
        (Qt 타이머 대신 threading.Timer 사용 → QThread 경고 제거)
        """
        import threading

        def pulse(code):
            self._emit_dash_udp("tx", code)

        threading.Timer(0.0,  lambda: pulse("0304")).start()
        threading.Timer(0.2,  lambda: pulse("0303")).start()
        threading.Timer(0.4,  lambda: pulse("0302")).start()
        threading.Timer(1.4,  lambda: pulse("0301")).start()
        threading.Timer(1.45, lambda: self._post_log_assignment("초기 임무 계획 모드 완료")).start()


    def _emit_dash_udp(self, kind: str, msg_id: str):
        """
        대시보드 수신 포트(KU_DASHBOARD_PORT, 기본 45991)로 UDP 이벤트 전송
        kind: 'tx' | 'rx'
        """
        import os, socket, json
        port = int(os.getenv("KU_DASHBOARD_PORT", "45991"))
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            payload = json.dumps({"kind": kind, "msg_id": str(msg_id)}).encode("utf-8")
            s.sendto(payload, ("127.0.0.1", port))
        finally:
            try:
                s.close()
            except Exception:
                pass

    def _log_assignment(self, text: str):
        """즉시(동기) 호출 시: 할당 카드 + 전역 로그에 남김"""
        mod = self.widgets.get("assignment")
        if mod and hasattr(mod, "append_log"):
            try:
                mod.append_log(text)
            except Exception:
                pass
        self._append_log_global(text)

    def _post_log_assignment(self, text: str):
        """
        백그라운드 스레드에서 안전하게 호출하는 로그 기록기.
        메인 스레드로 시그널만 보낸다.
        """
        try:
            self.uiLog.emit(str(text))
        except Exception:
            try:
                print(text)
            except Exception:
                pass
            

    def _self_check_all(self, on: bool = True):
        """모든 모듈 GUI에 자체점검 0102 On/Off 명령 전송"""
        payload = {"cmd": "self_check", "status": 1 if on else 0}
        self._broadcast_ctrl(payload)
        self._log_everywhere(f"자체점검 0102 {'ON' if on else 'OFF'} 전송")

    def _enter_standby(self):
        """대기모드: 0102 Off + 대시보드 모드텍스트 '대기모드'"""
        self._self_check_all(False)
        self._set_mode_text_all("대기모드")
        self._broadcast_ctrl({"cmd": "mode", "text": "대기모드"})
        self._log_everywhere("모든 SW 대기모드 진입")

    def _enter_initial_plan(self):
        """초기임무계획: 0102 Off + 대시보드 모드텍스트 '초기임무계획'"""
        self._self_check_all(False)
        self._set_mode_text_all("초기임무계획")
        self._broadcast_ctrl({"cmd": "mode", "text": "초기임무계획"})
        self._log_everywhere("모든 SW 초기임무계획 모드")

    def _set_mode_text_all(self, text: str):
        """대시보드의 세 카드(할당/모니터/의사) 'Mode 모니터링 txt' 표시 갱신"""
        for key in ("assignment", "monitoring", "decision"):
            mod = self.widgets.get(key)
            if not mod:
                continue
            # ModuleWithLog 전용 API 우선
            if hasattr(mod, "set_mode_text") and callable(mod.set_mode_text):
                try:
                    mod.set_mode_text(text)
                    continue
                except Exception:
                    pass
            # 백업: 속성 직접 접근
            try:
                ml = getattr(mod, "mode_line", None)
                if ml is not None and hasattr(ml, "setText"):
                    ml.setText(text)
            except Exception:
                pass

    def _broadcast_ctrl(self, payload: dict):
        """
        로컬 UDP로 각 모듈 GUI에 제어 명령 전달.
        포트 규약:
        - mission_planning_gui.py : 45981 (assignment)
        - monitoring_gui.py       : 45982 (monitoring)
        - decision_support_gui.py : 45983 (decision)
        """
        import socket, json
        targets = [("assignment", 45981), ("monitoring", 45982), ("decision", 45983)]
        data = json.dumps(payload).encode("utf-8")
        for role, port in targets:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.sendto(data, ("127.0.0.1", port))
                s.close()
            except Exception as e:
                try:
                    import sys
                    sys.stderr.write(f"[WARN] CTRL send to {role}:{port} failed: {e}\n")
                except Exception:
                    pass


    def _launch_all_guis(self):
        """
        'SW 실행' 버튼 액션: 세 모듈 GUI를 순차 실행
        """
        for sn in ("mission_planning_gui.py", "monitoring_gui.py", "decision_support_gui.py"):
            self._launch_gui(sn)

        # ★ GUI들이 CTRL UDP 바인드 시간을 가진 뒤, 메인스레드에서 전원 ON 브로드캐스트
        QTimer.singleShot(1000, lambda: self._set_mode_text_all("전원 ON"))
        QTimer.singleShot(1000, lambda: self._broadcast_ctrl({"cmd": "mode", "text": "전원 ON"}))
        QTimer.singleShot(1000, lambda: self._log_everywhere("모든 SW 전원 ON"))

    def _find_button(self, keywords) -> Optional[object]:
        """
        주어진 키워드가 텍스트/오브젝트명이에 포함된 첫 QPushButton을 찾는다.
        (한글 '실행' 도 매칭되도록 단순화)
        """
        try:
            from PyQt5.QtWidgets import QPushButton
            for btn in self.win.findChildren(QPushButton):
                text = (btn.text() or "").lower()
                objn = (btn.objectName() or "").lower()
                if any(k in text for k in keywords) or any(k in objn for k in keywords):
                    return btn
        except Exception:
            pass
        return None
    

    def _launch_gui(self, script_name: str):
        """
        GUI 스크립트 탐색 우선순위를 확장 + 자식 프로세스에 KU_MISSION_DB_ROOT 주입
        """
        import sys, os, subprocess
        from pathlib import Path

        # 프로젝트 루트
        root = Path(__file__).resolve().parent
        modules_dir = root / "modules"

        candidates = [
            root / script_name,
            modules_dir / "mission_planning" / script_name,
            modules_dir / "monitoring" / script_name,
            modules_dir / "decision_support" / script_name,
        ]
        script = next((p for p in candidates if p.exists()), None)
        if script is None:
            try:
                # 카드 로그가 있다면 거기에 남김 (없으면 조용히 무시)
                mod = getattr(self.widgets, "get", lambda *_: None)("decision")
                if mod and hasattr(mod, "append_log"):
                    mod.append_log(f"[RUN ERR] not found: {script_name}")
            except Exception:
                pass
            return

        # ── ★ DB 루트 경로 결정 ─────────────────────────────────────
        # 우선순위: (1) UI에서 고른 경로 → (2) 기존 ENV → (3) <프로젝트루트>\database
        ui_line = getattr(self.win, "_db_path_line", None)
        ui_val = ui_line.text().strip() if ui_line and hasattr(ui_line, "text") else ""

        db_root = ui_val or os.environ.get("KU_MISSION_DB_ROOT") or str(root / "database")
        # 존재하지 않으면 만들어 둠(하위 폴더는 각 GUI에서 생성)
        try:
            Path(db_root).mkdir(parents=True, exist_ok=True)
        except Exception:
            pass

        # ── 자식 환경 구성 ──────────────────────────────────────────
        env = os.environ.copy()
        env.setdefault("KU_LAUNCHED_BY_DASHBOARD", "1")
        env["KU_MISSION_DB_ROOT"] = db_root   # ★ 여기!
        
        port_map = {
            "monitoring_gui.py" : "45981",
            "mission_planning_gui.py" : "45982",
            "decision_support_gui.py" : "45983"
        }
        try:
            script_basename = script.name
        except Exception :
            script_basename = script_name
        env["KU_CTRL_PORT"] = port_map.get(script_basename, port_map.get(script_name, ""))

        try:
            subprocess.Popen(
                [sys.executable, str(script)],
                cwd=str(root),
                env=env,
                start_new_session=True
            )
        except Exception as e:
            try:
                mod = getattr(self.widgets, "get", lambda *_: None)("decision")
                if mod and hasattr(mod, "append_log"):
                    mod.append_log(f"[RUN ERR] {e}")
            except Exception:
                pass

    # --------- 버스 모니터링 시작 ---------
    def _start_bus_monitor(self):
        """
        대시보드(run.py 프로세스)를 nFusion 버스에 가입시키고,
        모든 메시지 ID에 대해 self(mark_received)를 리스너로 등록한다.
        !! 중요: receive 패키지를 import하기 전에 MessageLibrary를 AddReference 해야 한다.
        """
        # 0) 설정/어셈블리 선로딩 (← 순서 중요)
        try:
            _ensure_fusion_configs()
        except Exception:
            pass
        try:
            _load_msglib_and_deps()  # MessageLibrary + 의존 DLL 로드
        except Exception as e:
            sys.stderr.write(f"[WARN] _load_msglib_and_deps failed: {e}\n")

        # 1) receive 패키지 로드(Consumer 타입 등록을 위해 필수)
        try:
            import receive  # noqa: F401  # 이제 .NET 네임스페이스가 열려 있어야 정상 import됨
        except Exception as e:
            sys.stderr.write(f"[WARN] failed to import receive: {e}\n")
            return

        # 2) 리스너 등록 API 확보
        try:
            from receive_center import register_listener as register_listener  # 루트
        except Exception:
            try:
                from modules.common.receive_center import register_listener as register_listener  # 공용
            except Exception as e:
                sys.stderr.write(f"[WARN] receive_center.register_listener not available: {e}\n")
                return

        # 3) 우리 프로세스가 받을 메시지 ID 전체를 수집 후 리스너로 self 등록
        all_ids = set()
        for defs in self.msg_map.values():
            for mid, _ in (defs.get("tx", []) + defs.get("rx", [])):
                all_ids.add(str(mid))
        for mid in sorted(all_ids):
            try:
                register_listener(mid, self)  # 수신 시 self.mark_received(...) 호출됨
            except Exception as e:
                sys.stderr.write(f"[WARN] register_listener({mid}) failed: {e}\n")

        # 4) 버스 초기화(서브스크라이버/컨슈머 등록 포함)
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

    def mark_received(self, msg_id: str, raw: bytes | None = None):
        mid = self._norm_code(msg_id)

        # --- raw에서 source 추출(MMR/MSM/MOB 또는 한글 풀네임) ---
        src_key = None
        if raw:
            try:
                import re, json
                txt = raw.decode("utf-8", "ignore")
                m = re.search(r"\{.*\}", txt, flags=re.S)
                if m:
                    obj = json.loads(m.group(0))
                    src = obj.get("SourceModuleName") or obj.get("source") or obj.get("requestModuleName")
                    if src:
                        s = str(src).upper()
                        if "MMR" in s or "MULTI-AGENT MISSION PLANNER" in s:
                            src_key = "assignment"
                        elif "MSM" in s or "MISSION STATE MONITOR" in s:
                            src_key = "monitoring"
                        elif "MOB" in s or "MISSION OPTION BUILDER" in s:
                            src_key = "decision"
            except Exception:
                pass

        # --- 디듀프: 모듈별로 분리 + 5Hz 고려해 0.02s ---
        tag = f"bus:{src_key or 'all'}"
        if self._recently_seen(tag, mid, window=0.02):
            return

        key_to_widget = {
            "assignment": self.widgets.get("assignment"),
            "monitoring": self.widgets.get("monitoring"),
            "decision":   self.widgets.get("decision"),
        }

        # 업데이트 대상 모듈 결정
        if src_key:
            targets = [src_key]
        else:
            # 소스가 불명확하면 메시지 정의에 따라 모두 반영(기존 맵)
            targets = []
            for key, defs in self.msg_map.items():
                tx = {m for m, _ in defs.get("tx", [])}
                rx = {m for m, _ in defs.get("rx", [])}
                if mid in tx or mid in rx:
                    targets.append(key)

        # 반영
        for module_key in targets:
            w = key_to_widget.get(module_key)
            if not w:
                continue
            defs = self.msg_map.get(module_key, {})
            tx_ids = {m for m, _ in defs.get("tx", [])}
            rx_ids = {m for m, _ in defs.get("rx", [])}

            touched = False
            if mid in tx_ids and hasattr(w, "bump_tx"):
                w.bump_tx(mid)
                self._animate(module_key, "out")
                if hasattr(w, "append_log"): w.append_log(f"[{mid}] PUSH 완료")
                touched = True

            if mid in rx_ids and hasattr(w, "bump_rx"):
                w.bump_rx(mid)
                self._animate(module_key, "in")
                if hasattr(w, "append_log"): w.append_log(f"[{mid}] RX 수신")
                touched = True


    def _bump(self, mod, kind: str, msg_id: str):
        fn = getattr(mod, f"bump_{kind}", None)
        if callable(fn):
            try:
                fn(msg_id)
            except Exception:
                pass

    def _animate(self, module_key: str, direction: str):
        # module_key: 'assignment' | 'monitoring' | 'decision'
        vis_key = {"assignment": "mission", "monitoring": "monitor", "decision": "decision"}.get(module_key, module_key)

        # 1) MainWindow에 내장된 헬퍼가 있으면 그것부터 사용(동일 로직 보장)
        fnp = getattr(self.win, "_pulse", None)
        if callable(fnp):
            try:
                fnp(vis_key, direction)
                return
            except Exception:
                pass

        # 2) 직접 flow.trigger 호출
        flow = self.widgets.get("flow")
        if not flow:
            return
        fn = getattr(flow, "trigger", None)
        if callable(fn):
            try:
                fn(vis_key, direction)
            except Exception:
                pass

    def _append_log_module(self, mod, text: str):
        fn = getattr(mod, "append_log", None)
        if callable(fn):
            try:
                fn(text)
            except Exception:
                pass

    def _append_log_global(self, text: str):
        ed = self.widgets.get("log_edit")
        if ed is None:
            return
        # 안전장치: 전역 로그로 잡힌 위젯이 ModuleWithLog 내부라면 쓰지 않음
        try:
            p = ed.parent()
            while p is not None:
                if isinstance(p, ModuleWithLog):
                    return  # 카드 안의 LogBox면 전역 로그로 쓰지 않음
                p = p.parent()
        except Exception:
            pass

        try:
            if isinstance(ed, QPlainTextEdit):
                ed.appendPlainText(text)
            elif isinstance(ed, QTextEdit):
                ed.append(text)
        except Exception:
            pass



# ─────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    _load_qss(app)
    win = MainWindow()
    win.show()
    # 오케스트레이터 구동(버튼 연결, 목록/카운트/애니메이션/로그)
    orch = DashboardOrchestrator(win)
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
