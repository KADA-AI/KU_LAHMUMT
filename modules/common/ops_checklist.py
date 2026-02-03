# 파일: modules/common/ops_checklist.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, Optional, Sequence, List
import re, json

from PyQt5.QtCore import Qt, QObject, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QDialog, QLabel, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QFrame,           # ← 기존
    QScrollArea, QWidget                         # ← 추가
)

# ─────────────────────────────────────────────────────────────
# 간단한 UI 위젯들
class _Row(QFrame):
    """
    3개 모듈(assignment/monitoring/decision) 단위로 '완료점'을 찍는 줄.
    expected_keys로 카운트 기준(분모)을 조정하고,
    style_map으로 각 키의 성격을 지정한다: {'assignment':'tx'|'rx', ...}
      - tx  → 초록(#2aa745)
      - rx  → 파랑(#2d7cf6)
    """
    ALL_KEYS: Sequence[str] = ("assignment", "monitoring", "decision")

    def __init__(
        self,
        title: str,
        parent=None,
        expected_keys: Optional[Sequence[str]] = None,
        style_map: Optional[Dict[str, str]] = None
    ):
        super().__init__(parent)
        self._title  = QLabel(title)
        self._count  = QLabel("0/3")
        self._status = QLabel("미완료")
        self._detail = QLabel("-")
        self._detail.setWordWrap(True)

        self._dot_a = QLabel("●"); self._dot_b = QLabel("●"); self._dot_c = QLabel("●")
        for d in (self._dot_a, self._dot_b, self._dot_c):
            d.setStyleSheet("color:#999; font-size:14px;")

        lay = QGridLayout(self)
        lay.addWidget(self._title, 0, 0, 1, 3)
        lay.addWidget(self._count, 0, 3)
        lay.addWidget(self._status, 0, 4)

        lay.addWidget(QLabel("할당"), 1, 0)
        lay.addWidget(QLabel("모니터링"), 1, 1)
        lay.addWidget(QLabel("의사결정"), 1, 2)
        lay.addWidget(self._dot_a, 2, 0, alignment=Qt.AlignHCenter)
        lay.addWidget(self._dot_b, 2, 1, alignment=Qt.AlignHCenter)
        lay.addWidget(self._dot_c, 2, 2, alignment=Qt.AlignHCenter)
        lay.addWidget(self._detail, 3, 0, 1, 5)

        # 완료/분모/스타일
        self._expected_keys: Sequence[str] = tuple(expected_keys) if expected_keys else self.ALL_KEYS
        self._done_flags: Dict[str, bool] = {k: False for k in self.ALL_KEYS}
        self._style_map: Dict[str, str] = {k: (style_map or {}).get(k, "") for k in self.ALL_KEYS}

        self._recalc()
        self._update_dots()

        self.setStyleSheet("""
            QFrame { border: 1px solid #ccd; border-radius: 6px; padding: 6px; }
            QLabel { font-size: 12px; }
        """)
        self._title.setStyleSheet("font-weight:600;")
        self._status.setStyleSheet("font-weight:600;")

    def reset(self):
        for k in list(self._done_flags.keys()):
            self._done_flags[k] = False
        self._recalc()
        self._update_dots()
        self._detail.setText("-")

    def set_ok(self, key: str, on: bool = True):
        key = (key or "").strip().lower()
        if key in self._done_flags:
            self._done_flags[key] = bool(on)
            self._recalc()
            self._update_dots()

    def set_all(self, on: bool):
        for k in list(self._done_flags.keys()):
            self._done_flags[k] = bool(on)
        self._recalc()
        self._update_dots()

    def set_detail(self, text: str):
        self._detail.setText(str(text or "-"))

    def _color_for(self, key: str, ok: bool) -> str:
        # expected이 아닌 키는 흐리게 고정
        if key not in self._expected_keys:
            return "color:#bbb;"
        if not ok:
            return "color:#999;"
        style = (self._style_map.get(key) or "").lower()
        if style == "rx":
            return "color:#2d7cf6; font-weight:700;"   # 파랑
        # 기본=tx
        return "color:#2aa745; font-weight:700;"        # 초록

    def _update_dots(self):
        keys = list(self.ALL_KEYS)
        dots = [self._dot_a, self._dot_b, self._dot_c]
        for k, d in zip(keys, dots):
            ok = self._done_flags.get(k, False)
            d.setStyleSheet(self._color_for(k, ok))

    def _recalc(self):
        n = sum(1 for k, v in self._done_flags.items() if k in self._expected_keys and v)
        den = max(1, len(self._expected_keys))
        self._count.setText(f"{n}/{den}")
        if n >= den:
            self._status.setText("완료")
            self._status.setStyleSheet("color:#2aa745; font-weight:700;")
        elif n == 0:
            self._status.setText("미완료")
            self._status.setStyleSheet("color:#999; font-weight:600;")
        else:
            self._status.setText("진행중")
            self._status.setStyleSheet("color:#2d7cf6; font-weight:600;")

# ─────────────────────────────────────────────────────────────
# S100 체크리스트 (기존)
class S100ChecklistDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("S100 운용 체크리스트")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.setMinimumWidth(400)

        legend = QLabel("● 초록=생성(TX)   ● 파랑=수신(RX)")
        legend.setStyleSheet("color:#666; font-size:11px;")

        # 1) 실행 여부: 색상 분류 대상 아님(중립)
        self.r1 = _Row("1) SW 실행 여부 확인 - 3개중 3개완료")

        # 2) 0102 '송신' → TX=초록
        self.r2 = _Row(
            "2) 모듈 상태정보 송신(0102) - 3개중 3개 완료",
            expected_keys=("assignment","monitoring","decision"),
            style_map={"assignment":"tx","monitoring":"tx","decision":"tx"}
        )

        # 3) 대기모드 '수신' → RX=파랑
        self.r3 = _Row(
            "3) 대기모드 전환(Mode=대기모드) - 3개중 3개 완료",
            expected_keys=("assignment","monitoring","decision"),
            style_map={"assignment":"rx","monitoring":"rx","decision":"rx"}
        )

        self.r1.set_detail("mission / monitoring / decision 실행 감시")
        self.r2.set_detail("각 모듈의 0102 최초 송신 감지")
        self.r3.set_detail("각 모듈의 mode=대기모드 감지")

        for r in (self.r1, self.r2, self.r3):
            r.set_all(False)

        self.btn_reset = QPushButton("초기화")
        self.btn_close = QPushButton("닫기")

        btns = QHBoxLayout()
        btns.addStretch(1)
        btns.addWidget(self.btn_reset)
        btns.addWidget(self.btn_close)

        lay = QVBoxLayout(self)
        lay.addWidget(legend)
        lay.addWidget(self.r1)
        lay.addWidget(self.r2)
        lay.addWidget(self.r3)
        lay.addLayout(btns)

        self.btn_reset.clicked.connect(self._on_reset)
        self.btn_close.clicked.connect(self.close)

    def _on_reset(self):
        self.r1.reset(); self.r2.reset(); self.r3.reset()

class S100ChecklistController(QObject):
    # UI 스레드 업데이트 신호
    sig_set_launch   = pyqtSignal(str, bool)   # (module_key, on)
    sig_set_0102     = pyqtSignal(str, bool)   # (module_key, on)
    sig_set_standby  = pyqtSignal(str, bool)   # (module_key, on)

    SCRIPT_TO_KEY = {
        "mission_planning_gui.py": "assignment",
        "mission_planning":        "assignment",
        "assignment":              "assignment",
        "mmr":                     "assignment",
        "monitoring_gui.py":       "monitoring",
        "monitoring":              "monitoring",
        "msm":                     "monitoring",
        "decision_support_gui.py": "decision",
        "decision_support":        "decision",
        "decision":                "decision",
        "mob":                     "decision",
        "info_manage":             "info",
        "info":                    "info",
        "inf":                     "info",
    }

    def __init__(self, orch: Any):
        super().__init__()
        self.orch = orch
        self.ui = S100ChecklistDialog()
        self.ui.show()

        self._launched: Dict[str, bool] = {}
        self._sent0102: Dict[str, bool] = {}
        self._standby:  Dict[str, bool] = {}

        # 신호 연결
        self.sig_set_launch.connect(lambda key, on: self.ui.r1.set_ok(key, on))
        self.sig_set_0102.connect(lambda key, on: self.ui.r2.set_ok(key, on))
        self.sig_set_standby.connect(lambda key, on: self.ui.r3.set_ok(key, on))

        # 훅 장착
        self._hook_global_logger()
        self._hook_mark_received()
        self._hook_dash_pulse()
        self._hook_mode_event()

    # ---------- 공용: role→key 정규화 ----------
    @classmethod
    def _norm_role_to_key(cls, role: str) -> Optional[str]:
        s = (role or "").strip().lower()
        if s in ("assignment", "mission", "mmr", "mission_planning"): return "assignment"
        if s in ("monitoring", "msm"): return "monitoring"
        if s in ("decision", "mob", "decision_support"): return "decision"
        return cls.SCRIPT_TO_KEY.get(s)

    # ---------- 훅 1: 전역 로그([RUN] ….py) → 실행 여부 ----------
    def _hook_global_logger(self):
        orig = getattr(self.orch, "_append_global_log", None)
        if not callable(orig):
            return
        def wrapper(text: str):
            try:
                self._on_global_log(text)
            except Exception:
                pass
            return orig(text)
        self.orch._append_global_log = wrapper  # type: ignore

    def _on_global_log(self, text: str):
        m = re.search(r"\[RUN\]\s+([A-Za-z0-9_]+)\.py\b", str(text))
        if not m:
            return
        key = self._norm_role_to_key(m.group(1))
        if key and not self._launched.get(key, False):
            self._launched[key] = True
            self.sig_set_launch.emit(key, True)

    # ---------- 훅 2: 버스 수신(mark_received) → 0102 ----------
    def _hook_mark_received(self):
        orig = getattr(self.orch, "mark_received", None)
        if not callable(orig):
            return
        def wrapper(msg_id: str, raw: Optional[bytes] = None):
            try:
                self._on_mark_received(msg_id, raw)
            except Exception:
                pass
            return orig(msg_id, raw)
        self.orch.mark_received = wrapper  # type: ignore

    def _extract_json(self, raw: Optional[bytes]):
        if not raw:
            return None
        try:
            s = raw.decode("utf-8", "ignore")
            m = re.search(r"\{.*\}", s, flags=re.S)
            if not m:
                return None
            return json.loads(m.group(0))
        except Exception:
            return None

    @staticmethod
    def _src_to_key(src: str) -> Optional[str]:
        s = (src or "").upper()
        if "MMR" in s or "MISSION PLANNING" in s: return "assignment"
        if "MSM" in s or "MONITOR" in s:         return "monitoring"
        if "MOB" in s or "DECISION" in s:        return "decision"
        return None

    def _on_mark_received(self, msg_id: str, raw: Optional[bytes]):
        if str(msg_id).zfill(4) != "0102":
            return
        obj = self._extract_json(raw) or {}
        src = (obj.get("Source") or obj.get("source") or "")
        key = self._src_to_key(src)
        if key and not self._sent0102.get(key, False):
            self._sent0102[key] = True
            self.sig_set_0102.emit(key, True)

    # ---------- 훅 3: dashPulse(role, kind, msg_id) ----------
    def _hook_dash_pulse(self):
        try:
            self.orch.dashPulse.connect(self._on_dash_pulse)
        except Exception:
            pass

    def _on_dash_pulse(self, role: str, kind: str, msg_id: str):
        key = self._norm_role_to_key(role)
        if not key:
            return
        # 어떤 펄스든 오면 "해당 모듈 실행"으로 간주
        if not self._launched.get(key, False):
            self._launched[key] = True
            self.sig_set_launch.emit(key, True)
        # 0102 tx/rx → 2번 완료
        if kind.lower() in ("tx", "rx") and str(msg_id).zfill(4) == "0102":
            if not self._sent0102.get(key, False):
                self._sent0102[key] = True
                self.sig_set_0102.emit(key, True)

    # ---------- ★ 훅 4: _handle_mode_event(src_role, text) — 대기모드 전환 ----------
    def _hook_mode_event(self):
        orig = getattr(self.orch, "_handle_mode_event", None)
        if not callable(orig):
            return
        def wrapper(src_role: str, text: str):
            try:
                key = self._norm_role_to_key(src_role)
                norm = None
                # 오케스트레이터의 정규화 함수가 있으면 그대로 사용
                if hasattr(self.orch, "_normalize_mode_text"):
                    try:
                        norm = str(self.orch._normalize_mode_text(text))
                    except Exception:
                        norm = str(text or "")
                else:
                    norm = str(text or "")
                t = "".join(norm.split()).lower()
                is_standby = (t in ("대기모드", "대기", "standby", "1"))
                if key and is_standby and not self._standby.get(key, False):
                    self._standby[key] = True
                    self.sig_set_standby.emit(key, True)
            except Exception:
                pass
            return orig(src_role, text)
        self.orch._handle_mode_event = wrapper  # type: ignore


# ─────────────────────────────────────────────────────────────
# 외부 진입점: S100에서 호출
def ensure_s100_checklist(orch: Any) -> S100ChecklistController:
    """
    오케스트레이터에 1개만 붙도록 보장하고 창을 보여준다.
    """
    ctrl = getattr(orch, "_s100_checklist_ctrl", None)
    if isinstance(ctrl, S100ChecklistController):
        try:
            ctrl.ui.show()
            ctrl.ui.raise_(); ctrl.ui.activateWindow()
        except Exception:
            pass
        return ctrl
    ctrl = S100ChecklistController(orch)
    setattr(orch, "_s100_checklist_ctrl", ctrl)
    return ctrl



# ─────────────────────────────────────────────────────────────
# (위쪽 클래스/함수들 그대로 두고) 여기부터 교체
class S110ChecklistDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("S110 초기임무계획 체크리스트")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.setMinimumWidth(400)
        self.resize(720, 520)

        legend = QLabel("● 초록=송신(TX)   ● 파랑=수신(RX)")
        legend.setStyleSheet("color:#666; font-size:11px;")

        self.r1 = _Row(
            "1) 비행 참조정보 수신 (0203)",
            expected_keys=("assignment", "monitoring", "decision"),
            style_map={"assignment": "rx", "monitoring": "rx", "decision": "rx"},
        )
        self.r2 = _Row(
            "2) 협업기저임무계획 수신 (0201)",
            expected_keys=("assignment", "monitoring", "decision"),
            style_map={"assignment": "rx", "monitoring": "rx", "decision": "rx"},
        )
        self.r3 = _Row(
            "3) 초기 임무계획 모드 전환",
            expected_keys=("assignment", "monitoring", "decision"),
            style_map={"assignment": "rx", "monitoring": "rx", "decision": "rx"},
        )
        self.r4 = _Row(
            "4) [모니터링] 0902 데이터 송신",
            expected_keys=("monitoring",),
            style_map={"monitoring": "tx"},
        )
        self.r5 = _Row(
            "5) [할당] 0305 송신 (상태=1)",
            expected_keys=("assignment",),
            style_map={"assignment": "tx"},
        )
        self.r6 = _Row(
            "6) [할당] 0301 데이터 송신",
            expected_keys=("assignment",),
            style_map={"assignment": "tx"},
        )
        self.r7 = _Row(
            "7) 0301 데이터 수신",
            expected_keys=("monitoring", "decision"),
            style_map={"monitoring": "rx", "decision": "rx"},
        )
        self.r8 = _Row(
            "8) [할당] 0305 송신 (상태=2)",
            expected_keys=("assignment",),
            style_map={"assignment": "tx"},
        )
        self.r9 = _Row(
            "9) [할당] 0903 데이터 송신",
            expected_keys=("assignment",),
            style_map={"assignment": "tx"},
        )
        self.r10 = _Row(
            "10) [모니터링] 0903 데이터 수신",
            expected_keys=("monitoring",),
            style_map={"monitoring": "rx"},
        )

        self.r1.set_detail("할당/모니터링/의사결정 모듈이 0203을 수신하는지 확인")
        self.r2.set_detail("할당/모니터링/의사결정 모듈이 0201을 수신하는지 확인")
        self.r3.set_detail("각 모듈이 초기 임무계획 모드로 전환했는지")
        self.r4.set_detail("모니터링 모듈이 0902 메시지를 송신했는지")
        self.r5.set_detail("할당 모듈이 0305를 상태=1로 송신했는지")
        self.r6.set_detail("할당 모듈이 0301 메시지를 송신했는지")
        self.r7.set_detail("모니터링·의사결정 모듈이 0301을 수신했는지")
        self.r8.set_detail("할당 모듈이 0305를 상태=2로 송신했는지")
        self.r9.set_detail("0301에서 확보한 MissionPlanID로 0903 송신 여부 확인")
        self.r10.set_detail("모니터링 모듈이 0903을 수신했는지 확인")

        rows = (
            self.r1, self.r2, self.r3, self.r4, self.r5, self.r6,
            self.r7, self.r8, self.r9, self.r10,
        )
        for r in rows:
            r.set_all(False)

        self.btn_reset = QPushButton("초기화")
        self.btn_close = QPushButton("닫기")

        btns = QHBoxLayout()
        btns.addStretch(1)
        btns.addWidget(self.btn_reset)
        btns.addWidget(self.btn_close)

        content = QWidget()
        content_v = QVBoxLayout(content)
        content_v.addWidget(legend)
        for r in rows:
            content_v.addWidget(r)
        content_v.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(content)

        lay = QVBoxLayout(self)
        lay.addWidget(scroll)
        lay.addLayout(btns)

        self.btn_reset.clicked.connect(self._on_reset)
        self.btn_close.clicked.connect(self.close)

        self._rows = rows

    def _on_reset(self):
        for r in self._rows:
            r.reset()


class S110ChecklistController(QObject):
    sig_set = pyqtSignal(int, str, bool, str)

    def __init__(self, orch: Any):
        super().__init__()
        self.orch = orch
        self.ui = S110ChecklistDialog()
        self.ui.show()

        self._rx0203: Dict[str, bool] = {}
        self._rx0201: Dict[str, bool] = {}
        self._mode_init: Dict[str, bool] = {}
        self._monitoring_tx_0902 = False
        self._assignment_tx_0305_status1 = False
        self._assignment_tx_0305_status2 = False
        self._assignment_tx_0301 = False
        self._rx0301: Dict[str, bool] = {}
        self._pending_0903_tx = False
        self._assignment_tx_0903 = False
        self._monitoring_rx_0903 = False
        self._mission_plan_id: Optional[str] = None
        self._last_0903_mission_plan_id: Optional[str] = None

        self.sig_set.connect(self._on_sig_set)

        self._hook_dash_pulse()
        self._hook_mode_event()
        self._hook_mark_received()

    def _on_sig_set(self, idx: int, key: str, on: bool, detail: str):
        row_map = {
            1: self.ui.r1, 2: self.ui.r2, 3: self.ui.r3, 4: self.ui.r4,
            5: self.ui.r5, 6: self.ui.r6, 7: self.ui.r7, 8: self.ui.r8,
            9: self.ui.r9, 10: self.ui.r10,
        }
        row = row_map.get(int(idx))
        if not row:
            return
        if detail:
            row.set_detail(detail)
        row.set_ok(key, on)

    @staticmethod
    def _norm_role_to_key(role: str) -> Optional[str]:
        s = (role or "").strip().lower()
        if s in ("assignment", "mission", "mmr", "mission_planning", "assignment_planning"):
            return "assignment"
        if s in ("monitoring", "msm"):
            return "monitoring"
        if s in ("decision", "mob", "decision_support"):
            return "decision"
        return None

    @staticmethod
    def _norm_mid(mid: str) -> str:
        m = str(mid).strip()
        return m.zfill(4) if m.isdigit() and len(m) < 4 else m

    def _hook_dash_pulse(self):
        try:
            self.orch.dashPulse.connect(self._on_dash_pulse)
        except Exception:
            pass

    def _on_dash_pulse(self, role: str, kind: str, msg_id: str):
        key = self._norm_role_to_key(role)
        if not key:
            return
        k = (kind or "").lower()
        mid = self._norm_mid(msg_id)

        if k == "rx" and mid == "0203":
            if not self._rx0203.get(key, False):
                self._rx0203[key] = True
                self.sig_set.emit(1, key, True, "")

        if k == "rx" and mid == "0201":
            if not self._rx0201.get(key, False):
                self._rx0201[key] = True
                self.sig_set.emit(2, key, True, "")

        if k == "tx" and key == "monitoring" and mid == "0902":
            if not self._monitoring_tx_0902:
                self._monitoring_tx_0902 = True
                self.sig_set.emit(4, "monitoring", True, "")

        if k == "tx" and key == "assignment" and mid == "0305":
            if not self._assignment_tx_0305_status1 and not self._assignment_tx_0301:
                self._assignment_tx_0305_status1 = True
                self.sig_set.emit(5, "assignment", True, "MissionPlanningStatus=1(재계획 수행 중)")
            elif self._assignment_tx_0301 and not self._assignment_tx_0305_status2:
                self._assignment_tx_0305_status2 = True
                self.sig_set.emit(8, "assignment", True, "MissionPlanningStatus=2(재계획 완료)")

        if k == "tx" and key == "assignment" and mid == "0301":
            if not self._assignment_tx_0301:
                self._assignment_tx_0301 = True
                self.sig_set.emit(6, "assignment", True, "")

        if k == "rx" and mid == "0301" and key in ("monitoring", "decision"):
            if not self._rx0301.get(key, False):
                self._rx0301[key] = True
                self.sig_set.emit(7, key, True, "")

        if k == "tx" and key == "assignment" and mid == "0903":
            self._pending_0903_tx = True
            self._assignment_tx_0903 = True
            detail = "송신 감지" if not self._mission_plan_id else f"송신 감지 (MissionPlanID={self._mission_plan_id})"
            if self._mission_plan_id is None:
                detail = "송신 감지 (MissionPlanID 미확보)"
            self.sig_set.emit(9, "assignment", True, detail)

        if k == "rx" and key == "monitoring" and mid == "0903":
            if not self._monitoring_rx_0903:
                self._monitoring_rx_0903 = True
                detail = ""
                if self._last_0903_mission_plan_id:
                    detail = f"MissionPlanID={self._last_0903_mission_plan_id}"
                self.sig_set.emit(10, "monitoring", True, detail)

    def _hook_mode_event(self):
        orig = getattr(self.orch, "_handle_mode_event", None)
        if not callable(orig):
            return

        def wrapper(src_role: str, text: str):
            try:
                key = self._norm_role_to_key(src_role)
                norm = self._normalize_mode(text)
                if key and self._is_initial_plan(norm) and not self._mode_init.get(key, False):
                    self._mode_init[key] = True
                    self.sig_set.emit(3, key, True, norm)
            except Exception:
                pass
            return orig(src_role, text)

        self.orch._handle_mode_event = wrapper  # type: ignore

    def _normalize_mode(self, text: str) -> str:
        if hasattr(self.orch, "_normalize_mode_text"):
            try:
                text = str(self.orch._normalize_mode_text(text))
            except Exception:
                pass
        return "".join(str(text or "").split()).lower()

    @staticmethod
    def _is_initial_plan(norm_text: str) -> bool:
        return norm_text in ("초기임무계획", "초기임무", "initplan", "initialplan", "2")

    def _hook_mark_received(self):
        orig = getattr(self.orch, "mark_received", None)
        if not callable(orig):
            return

        def wrapper(msg_id: str, raw: Optional[bytes] = None):
            try:
                self._on_mark_received(msg_id, raw)
            except Exception:
                pass
            return orig(msg_id, raw)

        self.orch.mark_received = wrapper  # type: ignore

    def _on_mark_received(self, msg_id: str, raw: Optional[bytes]):
        mid = self._norm_mid(msg_id)
        payload = self._extract_json(raw) or {}

        if mid == "0301":
            mpid = self._extract_mission_plan_id(payload)
            if mpid:
                self._mission_plan_id = mpid

        if mid == "0903":
            mpid = self._extract_mission_plan_id(payload)
            if mpid:
                self._last_0903_mission_plan_id = mpid
            src = payload.get("Source") or payload.get("source") or payload.get("RequestModuleName") or ""
            key = self._src_to_key(src)
            expected = self._mission_plan_id or "-"
            if mpid:
                match = bool(self._mission_plan_id) and mpid == self._mission_plan_id
                detail = f"MissionPlanID={mpid}" if match else f"MissionPlanID 불일치 (0903={mpid} / 0301={expected})"
            else:
                match = False
                detail = "MissionPlanID 없음"

            if self._pending_0903_tx or key == "assignment":
                self.sig_set.emit(9, "assignment", match, detail)
                if match:
                    self._assignment_tx_0903 = True
                elif self._mission_plan_id:
                    try:
                        self.orch._safe_log(f"[OPS][WARN] 0903 MissionPlanID mismatch: 0903={mpid or '없음'}, 0301={expected}")
                    except Exception:
                        pass
                self._pending_0903_tx = False

    @staticmethod
    def _extract_json(raw: Optional[bytes]):
        if not raw:
            return None
        try:
            s = raw.decode("utf-8", "ignore")
            m = re.search(r"\{.*\}", s, flags=re.S)
            if not m:
                return None
            return json.loads(m.group(0))
        except Exception:
            return None

    @staticmethod
    def _extract_mission_plan_id(payload: Dict[str, Any]):
        for key in ("missionPlanId", "MissionPlanID", "MISSIONPLANID"):
            val = payload.get(key)
            if val is not None:
                text = str(val).strip()
                if text:
                    return text
        return None

    @staticmethod
    def _src_to_key(src: str) -> Optional[str]:
        s = (src or "").upper()
        if "MMR" in s or "MISSION PLANNING" in s or "ASSIGNMENT" in s:
            return "assignment"
        if "MSM" in s or "MONITOR" in s:
            return "monitoring"
        if "MOB" in s or "DECISION" in s:
            return "decision"
        return None



def ensure_s110_checklist(orch: Any) -> S110ChecklistController:
    ctrl = getattr(orch, "_s110_checklist_ctrl", None)
    if isinstance(ctrl, S110ChecklistController):
        try:
            ctrl.ui.show()
            ctrl.ui.raise_(); ctrl.ui.activateWindow()
        except Exception:
            pass
        return ctrl
    ctrl = S110ChecklistController(orch)
    setattr(orch, "_s110_checklist_ctrl", ctrl)
    return ctrl




# ─────────────────────────────────────────────────────────────
# S120 체크리스트 (임무 수행 진입)
class S120ChecklistDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("S120 임무 수행 체크리스트")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.setMinimumWidth(400)

        legend = QLabel("● 초록=송신(TX)   ● 파랑=수신(RX)")
        legend.setStyleSheet("color:#666; font-size:11px;")

        self.r1 = _Row(
            "1) 임무 수행 모드 수신",
            expected_keys=("assignment", "monitoring", "decision"),
            style_map={"assignment": "rx", "monitoring": "rx", "decision": "rx"},
        )
        self.r2 = _Row(
            "2) 0402 상태정보(정적) 수신",
            expected_keys=("assignment", "monitoring", "decision"),
            style_map={"assignment": "rx", "monitoring": "rx", "decision": "rx"},
        )
        self.r3 = _Row(
            "3) 0401 상태정보(동적) 수신",
            expected_keys=("assignment", "monitoring", "decision"),
            style_map={"assignment": "rx", "monitoring": "rx", "decision": "rx"},
        )
        self.r4 = _Row(
            "4) [모니터링] 0501 임무수행 상태 TX",
            expected_keys=("monitoring",),
            style_map={"monitoring": "tx"},
        )
        self.r5 = _Row(
            "5) [할당/의결] 0501 수신",
            expected_keys=("assignment", "decision"),
            style_map={"assignment": "rx", "decision": "rx"},
        )

        self.r1.set_detail("MMR/MOB 모듈 포함 모든 모듈이 임무 수행 모드로 전환")
        self.r2.set_detail("모듈별 0402(StateInfo-Static) 수신 여부")
        self.r3.set_detail("모듈별 0401(StateInfo-Dynamic) 수신 여부")
        self.r4.set_detail("모니터링 모듈이 0501을 송신했는지 확인")
        self.r5.set_detail("할당·의결 모듈이 0501을 수신했는지 확인")

        rows = (self.r1, self.r2, self.r3, self.r4, self.r5)
        for r in rows:
            r.set_all(False)

        self.btn_reset = QPushButton("초기화")
        self.btn_close = QPushButton("닫기")

        btns = QHBoxLayout()
        btns.addStretch(1)
        btns.addWidget(self.btn_reset)
        btns.addWidget(self.btn_close)

        content = QWidget()
        content_v = QVBoxLayout(content)
        content_v.addWidget(legend)
        for r in rows:
            content_v.addWidget(r)
        content_v.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(content)

        lay = QVBoxLayout(self)
        lay.addWidget(scroll)
        lay.addLayout(btns)

        self.btn_reset.clicked.connect(self._on_reset)
        self.btn_close.clicked.connect(self.close)

        self._rows = rows

    def _on_reset(self):
        for r in self._rows:
            r.reset()


class S120ChecklistController(QObject):
    sig_set = pyqtSignal(int, str, bool, str)

    def __init__(self, orch: Any):
        super().__init__()
        self.orch = orch
        self.ui = S120ChecklistDialog()
        self.ui.show()

        self._mode_exec: Dict[str, bool] = {}
        self._rx0402: Dict[str, bool] = {}
        self._rx0401: Dict[str, bool] = {}
        self._monitoring_tx_0501 = False
        self._rx0501: Dict[str, bool] = {}

        self.sig_set.connect(self._on_sig_set)

        self._hook_dash_pulse()
        self._hook_mode_event()

    def _on_sig_set(self, idx: int, key: str, on: bool, detail: str):
        row_map = {
            1: self.ui.r1,
            2: self.ui.r2,
            3: self.ui.r3,
            4: self.ui.r4,
            5: self.ui.r5,
        }
        row = row_map.get(int(idx))
        if not row:
            return
        if detail:
            row.set_detail(detail)
        row.set_ok(key, on)

    @staticmethod
    def _norm_role_to_key(role: str) -> Optional[str]:
        s = (role or "").strip().lower()
        if s in ("assignment", "mission", "mmr", "mission_planning", "assignment_planning"):
            return "assignment"
        if s in ("monitoring", "msm"):
            return "monitoring"
        if s in ("decision", "mob", "decision_support"):
            return "decision"
        return None

    @staticmethod
    def _norm_mid(mid: str) -> str:
        m = str(mid).strip()
        return m.zfill(4) if m.isdigit() and len(m) < 4 else m

    def _hook_dash_pulse(self):
        try:
            self.orch.dashPulse.connect(self._on_dash_pulse)
        except Exception:
            pass

    def _on_dash_pulse(self, role: str, kind: str, msg_id: str):
        key = self._norm_role_to_key(role)
        if not key:
            return
        k = (kind or "").lower()
        mid = self._norm_mid(msg_id)

        if k == "rx" and mid == "0402":
            if not self._rx0402.get(key, False):
                self._rx0402[key] = True
                self.sig_set.emit(2, key, True, "")

        if k == "rx" and mid == "0401":
            if not self._rx0401.get(key, False):
                self._rx0401[key] = True
                self.sig_set.emit(3, key, True, "")

        if k == "tx" and key == "monitoring" and mid == "0501":
            if not self._monitoring_tx_0501:
                self._monitoring_tx_0501 = True
                self.sig_set.emit(4, "monitoring", True, "")

        if k == "rx" and mid == "0501" and key in ("assignment", "decision"):
            if not self._rx0501.get(key, False):
                self._rx0501[key] = True
                self.sig_set.emit(5, key, True, "")

    def _hook_mode_event(self):
        orig = getattr(self.orch, "_handle_mode_event", None)
        if not callable(orig):
            return

        def wrapper(src_role: str, text: str):
            try:
                key = self._norm_role_to_key(src_role)
                norm = self._normalize_mode(text)
                if key and self._is_execution_mode(norm) and not self._mode_exec.get(key, False):
                    self._mode_exec[key] = True
                    self.sig_set.emit(1, key, True, norm)
            except Exception:
                pass
            return orig(src_role, text)

        self.orch._handle_mode_event = wrapper  # type: ignore

    def _normalize_mode(self, text: str) -> str:
        if hasattr(self.orch, "_normalize_mode_text"):
            try:
                text = str(self.orch._normalize_mode_text(text))
            except Exception:
                pass
        return "".join(str(text or "").split()).lower()

    @staticmethod
    def _is_execution_mode(norm_text: str) -> bool:
        return norm_text in ("임무수행", "임무수행모드", "execution", "missionmode", "3")


def ensure_s120_checklist(orch: Any) -> S120ChecklistController:
    ctrl = getattr(orch, "_s120_checklist_ctrl", None)
    if isinstance(ctrl, S120ChecklistController):
        try:
            ctrl.ui.show()
            ctrl.ui.raise_(); ctrl.ui.activateWindow()
        except Exception:
            pass
        return ctrl
    ctrl = S120ChecklistController(orch)
    setattr(orch, "_s120_checklist_ctrl", ctrl)
    return ctrl



# ─────────────────────────────────────────────────────────────
# S210 체크리스트 (정상 수행)
class S210ChecklistDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("S210 정상 수행 체크리스트")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.setMinimumWidth(400)

        legend = QLabel("● 초록=송신(TX)   ● 파랑=수신(RX)")
        legend.setStyleSheet("color:#666; font-size:11px;")

        self.r1 = _Row(
            "1) 0402 정상상황 정보 수신",
            expected_keys=("assignment", "monitoring", "decision"),
            style_map={"assignment": "rx", "monitoring": "rx", "decision": "rx"},
        )
        self.r2 = _Row(
            "2) 0401 유무인기 상태정보 수신",
            expected_keys=("assignment", "monitoring", "decision"),
            style_map={"assignment": "rx", "monitoring": "rx", "decision": "rx"},
        )
        self.r3 = _Row(
            "3) [모니터링] 0501 상태정보 송신",
            expected_keys=("monitoring",),
            style_map={"monitoring": "tx"},
        )
        self.r4 = _Row(
            "4) [할당/의결] 0501 상태정보 수신",
            expected_keys=("assignment", "decision"),
            style_map={"assignment": "rx", "decision": "rx"},
        )

        self.r1.set_detail("모든 모듈이 0402(정상 상황 정보)를 수신")
        self.r2.set_detail("모든 모듈이 0401(유/무인기 상태정보)을 수신")
        self.r3.set_detail("모니터링 모듈이 0501 상태정보를 송신(5Hz)")
        self.r4.set_detail("할당·의결 모듈이 0501 상태정보를 수신")

        rows = (self.r1, self.r2, self.r3, self.r4)
        for r in rows:
            r.set_all(False)

        self.btn_reset = QPushButton("초기화")
        self.btn_close = QPushButton("닫기")

        btns = QHBoxLayout()
        btns.addStretch(1)
        btns.addWidget(self.btn_reset)
        btns.addWidget(self.btn_close)

        content = QWidget()
        content_v = QVBoxLayout(content)
        content_v.addWidget(legend)
        for r in rows:
            content_v.addWidget(r)
        content_v.addStretch(1)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setWidget(content)

        lay = QVBoxLayout(self)
        lay.addWidget(scroll)
        lay.addLayout(btns)

        self.btn_reset.clicked.connect(self._on_reset)
        self.btn_close.clicked.connect(self.close)

        self._rows = rows

    def _on_reset(self):
        for r in self._rows:
            r.reset()


class S210ChecklistController(QObject):
    sig_set = pyqtSignal(int, str, bool, str)

    def __init__(self, orch: Any):
        super().__init__()
        self.orch = orch
        self.ui = S210ChecklistDialog()
        self.ui.show()

        self._rx0402: Dict[str, bool] = {}
        self._rx0401: Dict[str, bool] = {}
        self._monitoring_tx_0501 = False
        self._rx0501: Dict[str, bool] = {}

        self.sig_set.connect(self._on_sig_set)

        self._hook_dash_pulse()

    def _on_sig_set(self, idx: int, key: str, on: bool, detail: str):
        row_map = {
            1: self.ui.r1,
            2: self.ui.r2,
            3: self.ui.r3,
            4: self.ui.r4,
        }
        row = row_map.get(int(idx))
        if not row:
            return
        if detail:
            row.set_detail(detail)
        row.set_ok(key, on)

    @staticmethod
    def _norm_role_to_key(role: str) -> Optional[str]:
        s = (role or "").strip().lower()
        if s in ("assignment", "mission", "mmr", "mission_planning", "assignment_planning"):
            return "assignment"
        if s in ("monitoring", "msm"):
            return "monitoring"
        if s in ("decision", "mob", "decision_support"):
            return "decision"
        return None

    @staticmethod
    def _norm_mid(mid: str) -> str:
        m = str(mid).strip()
        return m.zfill(4) if m.isdigit() and len(m) < 4 else m

    def _hook_dash_pulse(self):
        try:
            self.orch.dashPulse.connect(self._on_dash_pulse)
        except Exception:
            pass

    def _on_dash_pulse(self, role: str, kind: str, msg_id: str):
        key = self._norm_role_to_key(role)
        if not key:
            return
        k = (kind or "").lower()
        mid = self._norm_mid(msg_id)

        if k == "rx" and mid == "0402":
            if not self._rx0402.get(key, False):
                self._rx0402[key] = True
                self.sig_set.emit(1, key, True, "")

        if k == "rx" and mid == "0401":
            if not self._rx0401.get(key, False):
                self._rx0401[key] = True
                self.sig_set.emit(2, key, True, "")

        if k == "tx" and key == "monitoring" and mid == "0501":
            if not self._monitoring_tx_0501:
                self._monitoring_tx_0501 = True
                self.sig_set.emit(3, "monitoring", True, "")

        if k == "rx" and mid == "0501" and key in ("assignment", "decision"):
            if not self._rx0501.get(key, False):
                self._rx0501[key] = True
                self.sig_set.emit(4, key, True, "")


def ensure_s210_checklist(orch: Any) -> S210ChecklistController:
    ctrl = getattr(orch, "_s210_checklist_ctrl", None)
    if isinstance(ctrl, S210ChecklistController):
        try:
            ctrl.ui.show()
            ctrl.ui.raise_(); ctrl.ui.activateWindow()
        except Exception:
            pass
        return ctrl
    ctrl = S210ChecklistController(orch)
    setattr(orch, "_s210_checklist_ctrl", ctrl)
    return ctrl
