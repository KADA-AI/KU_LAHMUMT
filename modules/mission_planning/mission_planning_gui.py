# -*- coding: utf-8 -*-
# mission_planning_gui.py – 임무 할당·계획수립 전용 GUI (S110 플로우 대응)
from __future__ import annotations

import sys, os, threading, json, re, time, shutil, socket, subprocess, math, copy
os.environ["KU_ROLE"] = "mission"  # MMR
from pathlib import Path
from typing import Any, Dict, Optional, Set, List

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
from modules.common.option_codes import (
    DEFAULT_OPTION_CODE_SEQUENCE,
    ensure_option_code_sequence,
    normalize_option_code,
    option_code_to_label,
)
from receive_center import register_listener, unregister_listener   # ★ 0101 모드 수신 리스너
from latest_input_cache import (
    reset_latest_inputs,
    update_from_payload as cache_update_from_payload,
    get_latest_package_id,
    get_latest_snapshot,
    describe_latest_ids,
    resolve_path_from_cache,
)

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
        self._session_scope = self._create_empty_scope()
        self._plan_status = "임무계획 전"
        self._option_id_counter = 0

        reset_latest_inputs()
        self._last_logged_input_ids = {"0201": None, "0203": None}
        self._input_listener_refs: list[tuple[str, callable]] = []
        self._install_input_listeners()

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

        # 초기 기동 시 즉시 초기화 모드로 전환
        self._set_mode_slider_by_text("초기화 모드")
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
        내부 슬라이더(0~4): [0=전원 OFF, 1=초기화 모드, 2=대기모드, 3=초기 임무 계획, 4=임무 수행]
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
        """초기화 모드 직후 0.5s 뒤 0102를 5Hz로 자동 시작."""
        if not self._power_on:
            return
        try:
            self._tab.periodic_config['0102'] = 5
        except Exception:
            pass
        self._ensure_0102(True)

    # ───────── 최신 0201/0203 ID 트래킹 ─────────
    def _install_input_listeners(self):
        """0201/0203 수신 리스너를 등록하고 캐시를 유지한다."""
        if getattr(self, "_input_listener_refs", None):
            for msg_id, handler in self._input_listener_refs:
                try:
                    unregister_listener(msg_id, handler)
                except Exception:
                    pass
            self._input_listener_refs.clear()
        for msg_id, handler in (("0201", self._on_input_payload_0201), ("0203", self._on_input_payload_0203)):
            try:
                register_listener(msg_id, handler)
                self._input_listener_refs.append((msg_id, handler))
            except Exception:
                self._append_log_line(f"[WARN] Listener registration failed for {msg_id}")

    def _load_attack_context(self, cmpk_path: Path) -> Optional[Dict[str, Any]]:
        try:
            with cmpk_path.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
        except Exception as exc:
            try:
                self.log_sig.emit(f"[WARN] 공격용 0201 메타데이터 읽기 실패({cmpk_path.name}): {exc}")
            except Exception:
                pass
            return None
        context = data.get("_attackContext") or data.get("attackContext")
        if isinstance(context, dict):
            return context
        return None

    def _build_attack_context_from_replan_detail(self, detail: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(detail, dict):
            return None
        coordinate = detail.get("coordinate") or detail.get("targetCoordinate") or {}
        lat = coordinate.get("latitude")
        lon = coordinate.get("longitude")
        if lat is None or lon is None:
            return None
        altitude = coordinate.get("altitude") or 0.0
        try:
            lat = float(lat)
            lon = float(lon)
            altitude = float(altitude)
        except Exception:
            return None
        target_context = {
            "target": {
                "latitude": lat,
                "longitude": lon,
                "altitude": altitude,
            },
            "targetID": detail.get("targetID"),
            "detail": detail,
        }
        watcher_id = detail.get("watcherID")
        if watcher_id is not None:
            target_context["watcherID"] = watcher_id
        return target_context

    def _compute_attack_waypoint(self, friendly: Dict[str, Any], target: Dict[str, Any], variant_no: int) -> Dict[str, float]:
        fallback = {
            "latitude": float(target.get("latitude") or friendly.get("latitude") or 0.0),
            "longitude": float(target.get("longitude") or friendly.get("longitude") or 0.0),
            "altitude": float(target.get("altitude") or friendly.get("altitude") or 0.0),
        }
        script_path = PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner" / "data_def" / "lah_attack_assistance.py"
        friendly_lat = friendly.get("latitude")
        friendly_lon = friendly.get("longitude")
        target_lat = target.get("latitude")
        target_lon = target.get("longitude")
        if script_path.exists() and friendly_lat is not None and friendly_lon is not None and target_lat is not None and target_lon is not None:
            cmd = [
                sys.executable or "python",
                str(script_path),
                "--friendly-lat",
                str(friendly_lat),
                "--friendly-lon",
                str(friendly_lon),
                "--enemy-lat",
                str(target_lat),
                "--enemy-lon",
                str(target_lon),
                "--output-json",
            ]
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, check=False)
                if result.returncode == 0:
                    data = json.loads(result.stdout or "{}")
                    attack_point = data.get("attack_point") or {}
                    lat_val = attack_point.get("lat") or attack_point.get("latitude") or target_lat
                    lon_val = attack_point.get("lon") or attack_point.get("longitude") or target_lon
                    alt_val = attack_point.get("alt_m") or attack_point.get("altitude") or target.get("altitude") or friendly.get("altitude") or 0.0
                    return {
                        "latitude": float(lat_val),
                        "longitude": float(lon_val),
                        "altitude": float(alt_val),
                    }
                else:
                    stderr_msg = (result.stderr or "").strip()
                    self.log_sig.emit(
                        f"[WARN] 공격 추천 좌표 계산 실패(variant={variant_no}, code={result.returncode}): {stderr_msg}"
                    )
            except Exception as exc:
                try:
                    self.log_sig.emit(f"[WARN] 공격 추천 좌표 계산 중 예외 발생(variant={variant_no}): {exc}")
                except Exception:
                    pass
        return fallback

    def _apply_attack_customizations(
        self,
        missions: List[Dict[str, Any]],
        flight_plans_0304: List[Dict[str, Any]],
        attack_context: Dict[str, Any],
        variant_no: int,
        *,
        replan_detail: Optional[Dict[str, Any]] = None,
    ) -> None:
        def _normalize_coord(raw: Optional[Dict[str, Any]]) -> Optional[Dict[str, float]]:
            if not isinstance(raw, dict):
                return None
            lat = raw.get("latitude")
            lon = raw.get("longitude")
            if lat is None or lon is None:
                return None
            alt = raw.get("altitude", 0.0)
            try:
                return {
                    "latitude": float(lat),
                    "longitude": float(lon),
                    "altitude": float(alt),
                }
            except Exception:
                return None

        def _estimate_eta_ms(p0: Dict[str, float], p1: Dict[str, float], speed_mps: float = 40.0) -> int:
            lat1, lon1 = p0["latitude"], p0["longitude"]
            lat2, lon2 = p1["latitude"], p1["longitude"]
            k = 111_132.92
            cos = math.cos(math.radians((lat1 + lat2) / 2))
            dx = (lon2 - lon1) * k * cos
            dy = (lat2 - lat1) * k
            dist_m = math.hypot(dx, dy)
            if dist_m <= 0 or speed_mps <= 0:
                return 0
            return int(round(1000 * dist_m / speed_mps))

        def _build_wp_entry(
            coord: Dict[str, float],
            waypoint_id: int,
            next_waypoint_id: int,
            eta_ms: int,
            *,
            target_id_value: int,
            weapon_type_value: int,
            ecf_value: float,
            speed_value: float = 40.0,
        ) -> Dict[str, Any]:
            return {
                "waypointID": waypoint_id,
                "coordinate": {
                    "latitude": round(coord["latitude"], 6),
                    "longitude": round(coord["longitude"], 6),
                    "altitude": int(round(coord.get("altitude", 0.0))),
                },
                "speed": speed_value,
                "eta": int(eta_ms),
                "ecf": float(ecf_value),
                "nextWaypointID": next_waypoint_id,
                "hovering": {"time": 0},
                "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
                "attack": {
                    "targetID": max(0, int(target_id_value)),
                    "weaponType": max(0, int(weapon_type_value)),
                },
            }

        target = attack_context.get("target") or {}
        target_coord = _normalize_coord(target)
        if target_coord is None:
            self.log_sig.emit(f"[WARN] 공격 옵션(variant={variant_no})에 target 좌표 정보가 없어 기본 임무를 유지합니다.")
            return
        manned_missions = [im for im in missions if int(im.get("aircraftID", 0)) in (1, 2)]
        if not manned_missions:
            self.log_sig.emit(f"[WARN] 공격 옵션(variant={variant_no}) 대상 유인기 임무를 찾지 못했습니다.")
            return
        manned_missions.sort(key=lambda im: int(im.get("individualMissionID") or 0))
        primary_mission = manned_missions[0]
        mission_info = primary_mission.get("individualMissionInfo") or {}
        coord_list = mission_info.get("coordinateList") or []
        friendly_coord = None
        if coord_list and isinstance(coord_list[0], dict):
            friendly_coord = _normalize_coord(coord_list[0])
        if friendly_coord is None:
            friendly_coord = dict(target_coord)
        attack_waypoint = self._compute_attack_waypoint(friendly_coord, target_coord, variant_no)
        target_id = attack_context.get("targetID")
        try:
            target_id_int = int(target_id) if target_id is not None else 0
        except Exception:
            target_id_int = 0

        coordinate_entries: List[Dict[str, float]] = []
        if friendly_coord:
            coordinate_entries.append(
                {
                    "latitude": friendly_coord["latitude"],
                    "longitude": friendly_coord["longitude"],
                    "altitude": friendly_coord.get("altitude", 0.0),
                }
            )
        coordinate_entries.append(
            {
                "latitude": target_coord["latitude"],
                "longitude": target_coord["longitude"],
                "altitude": target_coord.get("altitude", 0.0),
            }
        )

        mission_info_override = {
            "individualMissionType": 2,
            "patternType": 2,
            "autoZoomIn": 0,
            "coordinateList": coordinate_entries,
        }
        if target_id_int:
            mission_info_override["targetID"] = target_id_int
        if replan_detail:
            mission_info_override["_attackDetail"] = replan_detail
        primary_mission["individualMissionInfo"] = mission_info_override
        primary_mission["isDone"] = False

        attack_path_id = int(primary_mission.get("pathID") or 0)
        attack_aircraft_id = int(primary_mission.get("aircraftID") or 0)
        base_wp_id = 10_000 + variant_no * 10
        approach_coord = friendly_coord or dict(attack_waypoint)
        travel_eta_ms = _estimate_eta_ms(approach_coord, attack_waypoint)

        start_wp = _build_wp_entry(
            approach_coord,
            waypoint_id=base_wp_id,
            next_waypoint_id=base_wp_id + 1,
            eta_ms=0,
            target_id_value=0,
            weapon_type_value=0,
            ecf_value=0.0,
        )
        attack_wp = _build_wp_entry(
            attack_waypoint,
            waypoint_id=base_wp_id + 1,
            next_waypoint_id=0,
            eta_ms=travel_eta_ms,
            target_id_value=target_id_int,
            weapon_type_value=1,
            ecf_value=1.0,
        )

        replaced_fp = False
        for fp in flight_plans_0304 or []:
            try:
                fp_path_id = int(fp.get("pathID"))
            except Exception:
                fp_path_id = None
            if fp_path_id == attack_path_id and int(fp.get("aircraftID", 0)) == attack_aircraft_id:
                fp["lahWaypointList"] = [start_wp, attack_wp]
                replaced_fp = True
                break
        if not replaced_fp:
            self.log_sig.emit(f"[WARN] 공격 비행경로를 덮어쓸 pathID {attack_path_id}를 찾지 못했습니다.")
        else:
            self.log_sig.emit(
                f"[variant {variant_no}] 공격 임무 설정 완료 (aircraft={attack_aircraft_id}, targetID={target_id_int})"
            )

    def _on_input_payload_0201(self, msg_id, payload):
        self._handle_latest_input_payload(msg_id, payload)

    def _on_input_payload_0203(self, msg_id, payload):
        self._handle_latest_input_payload(msg_id, payload)

    def _create_empty_scope(self) -> Dict[str, Set[int]]:
        return {
            "packages": set(),
            "plans": set(),
            "individual_packages": set(),
            "paths": set(),
        }

    def _reset_session_scope(self) -> None:
        self._session_scope = self._create_empty_scope()

    def _submit_id_tab_update(
        self,
        *,
        scope: Optional[Dict[str, Set[int]]] = None,
        cmpk_id: Optional[int] = None,
        mrpk_id: Optional[int] = None,
        plan_state: Optional[str] = None,
    ) -> None:
        tab = getattr(self, "_id_tab", None)
        if tab is None:
            return
        scope_payload = None
        if scope is not None:
            scope_payload = {
                "packages": set(scope.get("packages", set())),
                "plans": set(scope.get("plans", set())),
                "individual_packages": set(scope.get("individual_packages", set())),
                "paths": set(scope.get("paths", set())),
            }
        def _apply() -> None:
            if scope_payload is not None:
                tab.update_session_scope(scope_payload)
            tab.update_input_status(
                cmpk_id=cmpk_id,
                mrpk_id=mrpk_id,
                plan_state=plan_state,
            )
        QTimer.singleShot(0, _apply)

    def _set_plan_status(self, status: str) -> None:
        self._plan_status = status
        self._submit_id_tab_update(plan_state=status)

    def _handle_latest_input_payload(self, msg_id: str, payload):
        try:
            prev = self._last_logged_input_ids.get(msg_id)
        except Exception:
            prev = None
        cache_update_from_payload(msg_id, payload)
        current = get_latest_package_id(msg_id)
        if current is None or current == prev:
            self._submit_id_tab_update(plan_state=self._plan_status)
            return
        self._last_logged_input_ids[msg_id] = current
        src = None
        if isinstance(payload, dict):
            src = payload.get("Source") or payload.get("source")
        note = f"[INFO] Latest {msg_id} ID updated → {current}"
        if src:
            note += f" (source={src})"
        self.log_sig.emit(note)

        cmpk_update = current if msg_id == "0201" else None
        mrpk_update = current if msg_id == "0203" else None
        scope_update: Optional[Dict[str, Set[int]]] = None
        if msg_id == "0201":
            if current is not None:
                self._reset_session_scope()
                self._session_scope["packages"].add(int(current))
            self._plan_status = "임무계획 전"
            scope_update = self._session_scope
        self._submit_id_tab_update(
            scope=scope_update,
            cmpk_id=cmpk_update,
            mrpk_id=mrpk_update,
            plan_state=self._plan_status,
        )

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
        force_direct = bool(payload.get("force_direct_update"))
        plan_ids = list(payload.get("plan_ids") or [])
        option_names = list(payload.get("option_names") or [])
        reason = payload.get("reason") or "init-plan"

        is_execution_mode = False
        if not force_direct:
            try:
                mode_slider = getattr(self, "mode_slider", None)
                if mode_slider is not None:
                    is_execution_mode = int(mode_slider.value()) == 4
            except Exception:
                is_execution_mode = False
        else:
            self._append_log_line("[INFO] replanLevel=4 → skip 0901/0701, direct 0903 delivery")

        if not plan_ids:
            self._append_log_line("[WARN] No missionPlanID to push (0301)")
            return

        # send 0301
        QTimer.singleShot(0, lambda: self._click_tx_button_for("0301"))
        # send 0305 completion
        QTimer.singleShot(600, lambda: self._push_0305(status=2, reason=reason))

        plan_meta = payload.get("option_meta") or {}

        if is_execution_mode and not force_direct:
            self._append_log_line("[INFO] Execution mode -> sending 0901 instead of 0903")
            QTimer.singleShot(900, lambda meta=plan_meta: self._push_0901_options(plan_ids, option_names, meta))
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
        labels = ["전원 OFF", "초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]
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
        labels = ["전원 OFF", "초기화 모드", "대기모드", "초기 임무 계획", "임무 수행"]
        norm = re.sub(r"\s+", "", str(text)).lower()
        mapping = {
            "전원off":0,"off":0,"poweroff":0,"0":0,
            "전원on":1,"on":1,"poweron":1,"1":1,
            "초기화":1,"초기화모드":1,"초기화mode":1,
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

    def _push_0901_options(self, plan_ids, option_names, plan_meta=None):
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
            meta_map = dict(plan_meta or {})
            valid_entries: list[tuple[int, int]] = []
            defaults = list(DEFAULT_OPTION_CODE_SEQUENCE) or [1]
            for idx, plan_id in enumerate(plan_list, 1):
                try:
                    pid = int(plan_id)
                except Exception:
                    continue
                raw_name = name_list[idx - 1] if idx - 1 < len(name_list) else None
                code = normalize_option_code(
                    raw_name,
                    fallback=defaults[idx - 1] if idx - 1 < len(defaults) else defaults[-1],
                )
                if code is None:
                    code = defaults[-1]
                valid_entries.append((pid, code))
            if not valid_entries:
                self.log_sig.emit("[WARN] 0901 skipped: no entries")
                return
            option_ids = self._allocate_option_ids(len(valid_entries))
            entries = []
            for oid, (pid, code) in zip(option_ids, valid_entries):
                entry = {"optionID": oid, "optionName": code, "missionPlanID": pid}
                meta = meta_map.get(pid)
                if meta:
                    entry["optionMeta"] = meta
                entries.append(entry)
            body = {
                "timestamp": ts,
                "source": "MMR",
                "requestTime": ts,
                "pendingOptionList": entries,
            }
            push_message("0901", NodeMessenger, body_dict=body)
            labels = ", ".join(
                f"{entry['optionName']}({option_code_to_label(entry['optionName'])})"
                for entry in entries
            )
            self.log_sig.emit(f"[0901] option request sent (count={len(entries)}, codes={labels})")
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
        detail_payload = payload.get("replanDetail")
        if detail_payload is not None:
            ctx["replan_detail"] = detail_payload
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
        self._schedule_replan_pipeline(delay_ms=100)

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

            generated_imp_ids: Set[int] = set()
            generated_path_ids: Set[int] = set()

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

            def _locate_prior_mission_plan():
                dss_dir = db_root / 'DSS_Internal'
                if not dss_dir.exists():
                    return None
                candidates = sorted(
                    (p for p in dss_dir.glob("0201_*.json") if p.is_file()),
                    key=lambda p: p.stat().st_mtime,
                    reverse=True,
                )
                for candidate in candidates:
                    try:
                        data = json.loads(candidate.read_text(encoding='utf-8'))
                    except Exception:
                        continue
                    ctx = data.get("_priorMissionContext")
                    if isinstance(ctx, dict):
                        return candidate, ctx
                return None

            def _apply_prior_mission_customizations(
                missions,
                flight_plans_0303,
                context,
                variant_no,
                pid_map,
                generated_path_ids,
            ):
                if not isinstance(context, dict):
                    return

                def _to_int(value, default=None):
                    try:
                        iv = int(value)
                        return iv
                    except Exception:
                        return default

                def _to_float(value, default=None):
                    try:
                        fv = float(value)
                        return fv
                    except Exception:
                        return default

                input_mission_id = _to_int(context.get("inputMissionID")) or _to_int(context.get("mission_id"))
                if input_mission_id is None:
                    self.log_sig.emit(f"[WARN] Prior mission customization skipped (variant={variant_no}): missing inputMissionID")
                    return

                coord = context.get("coordinate") or {}
                lat = _to_float(coord.get("latitude"))
                lon = _to_float(coord.get("longitude"))
                alt = _to_float(coord.get("altitude"), 800.0)
                if lat is None or lon is None:
                    self.log_sig.emit(f"[WARN] Prior mission customization skipped (variant={variant_no}): coordinate missing")
                    return

                prior_mission_id = _to_int(context.get("priorMissionID"), 0) or 0
                mission_type = _to_int(context.get("missionType"), 1) or 1
                target_id = _to_int(context.get("targetID"))
                preferred_aircraft_ids = (4, 5, 6)

                mission_entry = None
                fallback_entry = None
                for im in missions:
                    rel = im.get("relatedMission") or {}
                    if _to_int(rel.get("inputMissionID")) != input_mission_id:
                        continue
                    fallback_entry = im if fallback_entry is None else fallback_entry
                    if _to_int(im.get("aircraftID")) in preferred_aircraft_ids:
                        mission_entry = im
                        break
                if mission_entry is None:
                    mission_entry = fallback_entry
                if mission_entry is None:
                    self.log_sig.emit(f"[WARN] Prior mission customization skipped (variant={variant_no}): matching mission not found")
                    return

                aircraft_id = _to_int(mission_entry.get("aircraftID"))
                if aircraft_id not in preferred_aircraft_ids:
                    aircraft_id = preferred_aircraft_ids[0]
                    mission_entry["aircraftID"] = aircraft_id

                path_id = _to_int(mission_entry.get("pathID"))
                if not path_id or path_id <= 0:
                    path_id = next_path_id(aircraft_id)
                    mission_entry["pathID"] = path_id
                generated_path_ids.add(path_id)
                pid_map[(aircraft_id, _to_int(mission_entry.get("individualMissionID")))] = path_id

                rel_block = dict(mission_entry.get("relatedMission") or {})
                rel_block["relatedMissionType"] = 2
                rel_block["inputMissionID"] = input_mission_id
                rel_block["priorMissionID"] = prior_mission_id
                mission_entry["relatedMission"] = rel_block
                mission_entry["isDone"] = False

                mission_info = dict(mission_entry.get("individualMissionInfo") or {})
                mission_info["individualMissionType"] = 1 if mission_type == 2 else 5
                mission_info["patternType"] = 1
                mission_info["autoZoomIn"] = True
                mission_info["coordinateList"] = [
                    {
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": int(round(alt)),
                    }
                ]
                mission_info["lineList"] = []
                mission_info["areaList"] = []
                mission_info["targetID"] = target_id if (mission_type == 2 and target_id is not None) else 0
                mission_entry["individualMissionInfo"] = mission_info

                flight_entry = None
                for fp in flight_plans_0303 or []:
                    if _to_int(fp.get("aircraftID")) == aircraft_id:
                        flight_entry = fp
                        break
                if flight_entry is None:
                    flight_entry = {
                        "timestamp": _now_ms_since_2000(),
                        "Source": "MMR",
                        "pathID": path_id,
                        "aircraftID": aircraft_id,
                        "isFormationFlight": False,
                        "waypointList": [],
                    }
                    flight_plans_0303.append(flight_entry)
                else:
                    flight_entry["pathID"] = path_id
                    flight_entry["aircraftID"] = aircraft_id

                filming_property = {
                    "fieldOfView": 15,
                    "sensorType": 1,
                    "operationMode": 3 if mission_type == 2 else 1,
                }
                if mission_type == 2 and target_id is not None:
                    filming_property["autoTracking"] = {"targetID": target_id}
                else:
                    filming_property["coordinateOrientation"] = {
                        "coordinate": {
                            "latitude": lat,
                            "longitude": lon,
                            "altitude": 0,
                        }
                    }

                waypoint_id = 1
                try:
                    if flight_entry.get("waypointList"):
                        waypoint_id = _to_int(flight_entry["waypointList"][0].get("waypointID")) or 1
                except Exception:
                    waypoint_id = 1

                waypoint = {
                    "waypointID": waypoint_id,
                    "coordinate": {
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": int(round(alt)),
                    },
                    "speed": 35.0,
                    "eta": 300,
                    "ecf": 0.0,
                    "nextWaypointID": 0,
                    "waypointPassType": 2,
                    "filmingProperty": filming_property,
                    "loiterProperty": {
                        "radius": 400,
                        "direction": 1,
                        "time": 300,
                        "speed": 3,
                    },
                }

                flight_entry["timestamp"] = _now_ms_since_2000()
                flight_entry["Source"] = flight_entry.get("Source") or "MMR"
                flight_entry["isFormationFlight"] = False
                flight_entry["waypointList"] = [waypoint]

                self.log_sig.emit(
                    f"[variant {variant_no}] Prior mission customization applied "
                    f"(aircraft={aircraft_id}, inputMissionID={input_mission_id})"
                )
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

            self.log_sig.emit(f"[INFO] Latest input snapshot → {describe_latest_ids()}")

            latest_cmpk_id = get_latest_package_id("0201")
            latest_mrpk_id = get_latest_package_id("0203")

            cmpk_path = None
            cmpk_missing = False
            if latest_cmpk_id is not None:
                candidate = dir_0201 / f"{latest_cmpk_id}.json"
                if candidate.exists():
                    cmpk_path = candidate
                    ctx['inputMissionPackageID'] = latest_cmpk_id
                    self.log_sig.emit(f"[STEP 0] Using latest 0201 ID {latest_cmpk_id} ({candidate.name})")
                else:
                    snap_0201 = get_latest_snapshot("0201")
                    payload_0201 = getattr(snap_0201, "payload", None)
                    if isinstance(payload_0201, dict) and (
                        payload_0201.get("inputMissionList") or payload_0201.get("availableAircraftList")
                    ):
                        payload_copy = dict(payload_0201)
                        payload_copy.setdefault("inputMissionPackageID", latest_cmpk_id)
                        try:
                            candidate.write_text(json.dumps(payload_copy, ensure_ascii=False, indent=2), encoding="utf-8")
                            cmpk_path = candidate
                            ctx['inputMissionPackageID'] = latest_cmpk_id
                            self.log_sig.emit(f"[STEP 0] Materialized latest 0201 ID {latest_cmpk_id} from cache payload ({candidate.name})")
                        except Exception as exc:
                            self.log_sig.emit(f"[ERR] Failed to materialize latest 0201 ID {latest_cmpk_id}: {exc}")
                            cmpk_missing = True
                    else:
                        self.log_sig.emit(f"[ERR] Latest 0201 ID {latest_cmpk_id} missing and cache payload unavailable")
                        cmpk_missing = True
            if cmpk_missing:
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return
            if cmpk_path is None:
                fallback_cmpk = ctx.get('cmpk_path') or staged.get('cmpk_path')
                cmpk_path = _resolve_path(fallback_cmpk, dir_0201)
                if cmpk_path:
                    try:
                        ctx.setdefault('inputMissionPackageID', int(Path(cmpk_path).stem))
                    except Exception:
                        pass
                    self.log_sig.emit(f"[INFO] Fallback 0201 file selected: {cmpk_path.name}")

            mrpk_path = None
            mrpk_missing = False
            if latest_mrpk_id is not None:
                candidate = dir_0203 / f"{latest_mrpk_id}.json"
                if candidate.exists():
                    mrpk_path = candidate
                    ctx['missionReferencePackageID'] = latest_mrpk_id
                    self.log_sig.emit(f"[STEP 0] Using latest 0203 ID {latest_mrpk_id} ({candidate.name})")
                else:
                    snap_0203 = get_latest_snapshot("0203")
                    payload_0203 = getattr(snap_0203, "payload", None)
                    if isinstance(payload_0203, dict) and (
                        payload_0203.get("takeOverInfoList") or payload_0203.get("flightAreaList") or payload_0203.get("handOverInfoList")
                    ):
                        payload_copy = dict(payload_0203)
                        payload_copy.setdefault("missionReferencePackageID", latest_mrpk_id)
                        try:
                            candidate.write_text(json.dumps(payload_copy, ensure_ascii=False, indent=2), encoding="utf-8")
                            mrpk_path = candidate
                            ctx['missionReferencePackageID'] = latest_mrpk_id
                            self.log_sig.emit(f"[STEP 0] Materialized latest 0203 ID {latest_mrpk_id} from cache payload ({candidate.name})")
                        except Exception as exc:
                            self.log_sig.emit(f"[ERR] Failed to materialize latest 0203 ID {latest_mrpk_id}: {exc}")
                            mrpk_missing = True
                    else:
                        self.log_sig.emit(f"[ERR] Latest 0203 ID {latest_mrpk_id} missing and cache payload unavailable")
                        mrpk_missing = True
            if mrpk_missing:
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return
            if mrpk_path is None:
                fallback_mrpk = ctx.get('mrpk_path') or staged.get('mrpk_path')
                mrpk_path = _resolve_path(fallback_mrpk, dir_0203)
                if mrpk_path:
                    try:
                        ctx.setdefault('missionReferencePackageID', int(Path(mrpk_path).stem))
                    except Exception:
                        pass
                    self.log_sig.emit(f"[INFO] Fallback 0203 file selected: {mrpk_path.name}")

            if not cmpk_path or not mrpk_path:
                self.log_sig.emit('[ERR] Replan pipeline aborted: missing 0201/0203 input')
                self._plan_status = "임무계획 실패"
                self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                return

            # Exclude completed input missions (isDone=True) before generating new plans.
            mission_whitelist: Set[int] = set()
            for source in (ctx.get("mission_ids"), staged.get("mission_ids")):
                for value in source or []:
                    try:
                        mission_whitelist.add(int(value))
                    except Exception:
                        continue

            filtered_cmpk_path = cmpk_path
            try:
                with cmpk_path.open("r", encoding="utf-8") as fh:
                    cmpk_data = json.load(fh)
            except Exception as exc:
                self.log_sig.emit(f"[WARN] Failed to load 0201 for mission filtering: {exc}")
            else:
                mission_list = cmpk_data.get("inputMissionList")
                if isinstance(mission_list, list):
                    filtered_list = []
                    removed_ids: list[str] = []
                    converted_ids: list[str] = []
                    width_adjusted_ids: list[str] = []
                    active_ids: list[int] = []
                    for mission in mission_list:
                        mid_raw = mission.get("inputMissionID")
                        try:
                            mid_int = int(mid_raw)
                        except Exception:
                            mid_int = None
                        if mission_whitelist and (mid_int is None or mid_int not in mission_whitelist):
                            removed_ids.append(str(mid_raw))
                            continue
                        if bool(mission.get("isDone")):
                            removed_ids.append(str(mid_raw))
                            continue
                        mtype = mission.get("inputMissionType")
                        if not isinstance(mtype, int) or mtype == 0:
                            detail = mission.get("missionDetail") or {}
                            if detail.get("lineList"):
                                mission["inputMissionType"] = 1
                                mtype = 1
                                converted_ids.append(f"{mid_raw}->1")
                            elif detail.get("areaList"):
                                mission["inputMissionType"] = 2
                                mtype = 2
                                converted_ids.append(f"{mid_raw}->2")
                            else:
                                removed_ids.append(str(mid_raw))
                                continue
                        if mtype == 1:
                            detail = mission.get("missionDetail") or {}
                            for entry in detail.get("lineList") or []:
                                try:
                                    width_val = float(entry.get("width", 0))
                                except Exception:
                                    width_val = 0.0
                                if width_val <= 0:
                                    entry["width"] = 1000
                                    width_adjusted_ids.append(str(mid_raw))
                        filtered_list.append(mission)
                        if mid_int is not None:
                            active_ids.append(mid_int)
                    if not filtered_list:
                        self.log_sig.emit("[WARN] No pending missions remain after filtering; skipping replan pipeline.")
                        self._plan_status = "replan_skipped"
                        self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                        return
                    if active_ids:
                        ctx["mission_ids"] = active_ids

                    if len(filtered_list) != len(mission_list) or (mission_whitelist and set(active_ids) != mission_whitelist):
                        cmpk_data["inputMissionList"] = filtered_list
                        filtered_dir = out_root_base / "_filtered"
                        filtered_dir.mkdir(parents=True, exist_ok=True)
                        filtered_cmpk_path = filtered_dir / cmpk_path.name
                        try:
                            filtered_cmpk_path.write_text(
                                json.dumps(cmpk_data, ensure_ascii=False, indent=2),
                                encoding="utf-8",
                            )
                            cmpk_path = filtered_cmpk_path
                        except Exception as exc:
                            self.log_sig.emit(f"[WARN] Failed to persist filtered 0201 snapshot: {exc}")
                        else:
                            removed_summary = ", ".join(removed_ids) if removed_ids else "-"
                            converted_summary = ", ".join(converted_ids) if converted_ids else "-"
                            width_summary = ", ".join(width_adjusted_ids) if width_adjusted_ids else "-"
                            self.log_sig.emit(
                                "[INFO] Filtered completed input missions "
                                f"(removed={removed_summary or '-'}, converted={converted_summary or '-'}, "
                                f"widthAdjusted={width_summary or '-'})"
                            )
                else:
                    self.log_sig.emit("[WARN] 0201 payload missing valid inputMissionList; continuing without filtering")

            ctx['cmpk_path'] = str(cmpk_path)
            ctx['mrpk_path'] = str(mrpk_path)

            plan_ids_source = ctx.get('plan_ids') or staged.get('plan_ids') or []
            plan_ids: list[int | None] = []
            for val in plan_ids_source:
                try:
                    plan_ids.append(int(val))
                except Exception:
                    plan_ids.append(None)

            raw_option_values = list(ctx.get('option_names') or staged.get('option_names') or [])
            plan_count = max(len(plan_ids), len(raw_option_values), 1)
            while len(plan_ids) < plan_count:
                plan_ids.append(None)
            option_codes = ensure_option_code_sequence(raw_option_values, plan_count)
            option_labels: List[str] = []
            for idx in range(plan_count):
                label = ""
                if idx < len(raw_option_values):
                    value = raw_option_values[idx]
                    label = str(value).strip() if value is not None else ""
                option_labels.append(label)
            attack_option_labels = {"공격추천", "공격 특화"}
            attack_option_indices: Set[int] = {
                idx for idx, label in enumerate(option_labels) if label in attack_option_labels
            }
            for attack_idx in attack_option_indices:
                if 0 <= attack_idx < len(option_codes):
                    option_codes[attack_idx] = 2
            shared_attack_detail = ctx.get("replan_detail") if isinstance(ctx.get("replan_detail"), dict) else None
            shared_attack_context = (
                self._build_attack_context_from_replan_detail(shared_attack_detail) if shared_attack_detail else None
            )
            attack_cmpk_path: Optional[Path] = None
            if attack_option_indices:
                dss_dir = db_root / 'DSS_Internal'
                attack_candidates: List[Path] = []
                if dss_dir.exists():
                    attack_candidates.extend(sorted(dss_dir.glob("0201_*.json")))
                    attack_candidates.extend(sorted(dss_dir.glob("0201_attack*.json")))
                if attack_candidates:
                    attack_cmpk_path = max(attack_candidates, key=lambda p: p.stat().st_mtime)
                else:
                    legacy_attack = dss_dir / '0201_attack.json'
                    if legacy_attack.exists():
                        attack_cmpk_path = legacy_attack
                if attack_cmpk_path:
                    self.log_sig.emit(
                        f"[INFO] 공격 옵션에 {attack_cmpk_path.name} 적용: {sorted(idx + 1 for idx in attack_option_indices)}"
                    )
                elif shared_attack_context is None:
                    self.log_sig.emit("[WARN] 공격 옵션이 있으나 활용 가능한 대상 정보가 없어 기본 임무를 유지합니다.")
                    attack_option_indices.clear()

            prior_option_indices: Set[int] = {idx for idx, label in enumerate(option_labels) if label == "선행임무 재계획"}
            prior_variant_contexts: Dict[int, Dict[str, Any]] = {}
            if prior_option_indices:
                prior_plan = _locate_prior_mission_plan()
                if prior_plan is None:
                    self.log_sig.emit("[WARN] 선행임무 재계획 옵션이 있으나 DSS_Internal/0201_prior 데이터를 찾지 못했습니다.")
                    prior_option_indices.clear()
                else:
                    prior_path, prior_ctx = prior_plan
                    for idx in prior_option_indices:
                        prior_variant_contexts[idx] = {"path": prior_path, "context": prior_ctx}
                    self.log_sig.emit(
                        f"[INFO] 선행임무 재계획 옵션에 {prior_path.name} 적용: "
                        f"{sorted(i + 1 for i in prior_option_indices)}"
                    )

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
            option_codes_out: list[int] = []
            plan_meta_map: Dict[int, Dict[str, Any]] = {}
            total_imp_files = 0
            total_fp_files = 0

            for idx in range(plan_count):
                variant_no = idx + 1
                requested_plan_id = plan_ids[idx]
                option_code = option_codes[idx]
                cmpk_source_path = cmpk_path
                variant_attack_context: Optional[Dict[str, Any]] = None
                variant_prior_context: Optional[Dict[str, Any]] = None
                attack_option_selected = idx in attack_option_indices
                if attack_option_selected and attack_cmpk_path is not None:
                    cmpk_source_path = attack_cmpk_path
                    self.log_sig.emit(
                        f"[variant {variant_no}] 공격 전용 0201 적용: {cmpk_source_path.name}"
                    )
                if attack_option_selected:
                    if shared_attack_context:
                        variant_attack_context = copy.deepcopy(shared_attack_context)
                    elif attack_cmpk_path is not None:
                        variant_attack_context = self._load_attack_context(cmpk_source_path)
                if idx in prior_variant_contexts:
                    prior_info = prior_variant_contexts[idx]
                    cmpk_source_path = prior_info["path"]
                    variant_prior_context = prior_info.get("context")
                    self.log_sig.emit(
                        f"[variant {variant_no}] 선행임무 0201 적용: {cmpk_source_path.name}"
                    )

                iter_out_root = out_root_base / f'variant_{variant_no:02d}'
                if iter_out_root.exists():
                    shutil.rmtree(iter_out_root)
                iter_out_root.mkdir(parents=True, exist_ok=True)

                self.log_sig.emit(f"[STEP 1.{variant_no}] Divide & Pattern start")
                imp_paths = run_divide_and_pattern(
                    str(cmpk_source_path),
                    str(mrpk_path),
                    str(iter_out_root),
                    log=lambda msg, n=variant_no: self.log_sig.emit(f"[variant {n}] {msg}")
                )
                if not imp_paths:
                    self.log_sig.emit(f"[ERR] IMP generation failed (variant={variant_no})")
                    self._plan_status = "임무계획 실패"
                    self._submit_id_tab_update(scope=self._session_scope, plan_state=self._plan_status)
                    return
                self.log_sig.emit(f"[OK] IMP generated: {len(imp_paths)} file(s) (variant={variant_no})")

                mp_tmp = iter_out_root / f"MissionPlan_{int(time.time()*1000)}.json"
                build_mission_plan_0301(str(cmpk_source_path), str(mrpk_path), imp_paths, str(mp_tmp))
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
                        generated_path_ids.add(pid)
                    else:
                        imp_pid = _imp_path_id(im)
                        if imp_pid is not None:
                            pid_val = int(imp_pid)
                            im['pathID'] = pid_val
                            pid_map[(aid, mid)] = pid_val
                            generated_path_ids.add(pid_val)
                self.log_sig.emit(f"[INFO] pathID mapping done for 0302/0303/0304 (variant={variant_no})")

                manned = [im for im in missions if int(im.get('aircraftID', 0)) in (1, 2, 3)]
                unmanned = [im for im in missions if int(im.get('aircraftID', 0)) in (4, 5, 6)]
                wp_alloc = d0303._WPAllocator()
                flight_plans_0303 = d0303.build_flight_plans(unmanned, wp_alloc, 40.0, turn_step_deg=15.0) if unmanned else []
                flight_plans_0304 = d0304.build_lah_flight_plans_fixed(manned, cruise_speed=40.0, wp_alloc=wp_alloc) if manned else []

                if variant_attack_context:
                    self._apply_attack_customizations(
                        missions,
                        flight_plans_0304 or [],
                        variant_attack_context,
                        variant_no,
                        replan_detail=shared_attack_detail,
                    )
                if variant_prior_context:
                    _apply_prior_mission_customizations(
                        missions,
                        flight_plans_0303,
                        variant_prior_context,
                        variant_no,
                        pid_map,
                        generated_path_ids,
                    )

                for fp in (flight_plans_0303 or []) + (flight_plans_0304 or []):
                    pid_val = fp.get('pathID')
                    if pid_val is not None:
                        try:
                            generated_path_ids.add(int(pid_val))
                        except Exception:
                            pass

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
                plan_meta_entry = plan_meta_map.setdefault(plan_id, {})
                if variant_attack_context:
                    attack_meta = {
                        "attack": True,
                        "targetCount": int(variant_attack_context.get("targetCount") or 1),
                        "targetID": variant_attack_context.get("targetID"),
                    }
                    if shared_attack_detail:
                        attack_meta["replanDetail"] = shared_attack_detail
                    plan_meta_entry.update(attack_meta)
                if variant_prior_context:
                    try:
                        prior_mid = int(variant_prior_context.get("priorMissionID") or 0)
                    except Exception:
                        prior_mid = 0
                    try:
                        prior_input_id = int(variant_prior_context.get("inputMissionID") or 0)
                    except Exception:
                        prior_input_id = 0
                    plan_meta_entry.update(
                        {
                            "priorMission": True,
                            "priorMissionID": prior_mid,
                            "inputMissionID": prior_input_id,
                        }
                    )

                (dir_mp / f"{plan_id}.json").write_text(json.dumps(mp_json, indent=2, ensure_ascii=False), encoding='utf-8')

                imp_pkgs = d0302.build_mission_packages(missions, cmpk_id=cmpk_id, plan_pkg_map=imp_id_map)
                for pkg in imp_pkgs:
                    imp_id = pkg.get('individualMissionPackageID') or pkg.get('individualMissionPlanPackageID')
                    if imp_id is None:
                        continue
                    try:
                        generated_imp_ids.add(int(imp_id))
                    except Exception:
                        pass
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
                option_codes_out.append(int(option_code))
                self.log_sig.emit(
                    f"[INFO] Option mapping #{variant_no}: "
                    f"planID={plan_id}, optionCode={option_code}({option_code_to_label(option_code)})"
                )

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
            ctx['option_names'] = option_codes_out
            ctx["_option_meta"] = dict(plan_meta_map)
            self._active_plan_context = ctx

            try:
                input_pkg_id_int = int(ctx.get('inputMissionPackageID'))
            except Exception:
                input_pkg_id_int = None
            try:
                ref_pkg_id_int = int(ctx.get('missionReferencePackageID'))
            except Exception:
                ref_pkg_id_int = None

            plan_id_set = {int(pid) for pid in generated_plan_ids if pid is not None}
            imp_id_set = {int(val) for val in generated_imp_ids if val is not None}
            path_id_set = {int(val) for val in generated_path_ids if val is not None}

            if input_pkg_id_int is not None:
                self._session_scope['packages'].add(input_pkg_id_int)
            self._session_scope['plans'].update(plan_id_set)
            self._session_scope['individual_packages'].update(imp_id_set)
            self._session_scope['paths'].update(path_id_set)
            self._plan_status = "임무계획 완료"
            self._submit_id_tab_update(
                scope=self._session_scope,
                cmpk_id=input_pkg_id_int,
                mrpk_id=ref_pkg_id_int,
                plan_state=self._plan_status,
            )

            force_direct_update = False
            try:
                force_direct_update = int(ctx.get("replan_level", 0)) == 4
            except Exception:
                force_direct_update = False

            self._schedule_plan_delivery(
                generated_plan_ids,
                option_codes_out,
                reason,
                plan_meta_map,
                force_direct_update=force_direct_update,
            )

        except Exception as exc:
            self.log_sig.emit(f"[ERR] Replan pipeline failed: {exc}")
        finally:
            self._initplan_running = False

    def closeEvent(self, event):
        try:
            for msg_id, handler in getattr(self, "_input_listener_refs", []):
                unregister_listener(msg_id, handler)
        except Exception:
            pass
        super().closeEvent(event)

    def _schedule_plan_delivery(
        self,
        plan_ids,
        option_names,
        reason,
        option_meta=None,
        *,
        force_direct_update: bool = False,
    ):
        self._pending_plan_push = {
            "plan_ids":     list(plan_ids or []),
            "option_names": list(option_names or []),
            "reason":       reason,
            "option_meta":  dict(option_meta or {}),
            "force_direct_update": bool(force_direct_update),
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
