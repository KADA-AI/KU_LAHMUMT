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
        self.setMinimumWidth(520)

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
        self.r3.set_detail("각 모듈의 UDP mode=대기모드 감지")

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

    # ---------- 훅 3: dashPulse(role, kind, msg_id) — UDP ----------
    def _hook_dash_pulse(self):
        try:
            self.orch.dashPulse.connect(self._on_dash_pulse)
        except Exception:
            pass

    def _on_dash_pulse(self, role: str, kind: str, msg_id: str):
        key = self._norm_role_to_key(role)
        if not key:
            return
        # 어떤 UDP 펄스든 오면 "해당 모듈 실행"으로 간주
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
                is_standby = (t in ("대기모드", "대기", "standby", "2"))
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
        self.setMinimumWidth(620)
        self.resize(720, 540)

        legend = QLabel("● 초록=생성(TX)   ● 파랑=수신(RX)")
        legend.setStyleSheet("color:#666; font-size:11px;")

        # ── 체크 항목 생성: style_map으로 TX/RX 지정 ──
        self.r1  = _Row("1) 0203(비행참조정보) 수신",
                        expected_keys=("assignment","monitoring","decision"),
                        style_map={"assignment":"rx","monitoring":"rx","decision":"rx"})
        self.r2  = _Row("2) 0201(협업기저임무계획) 수신",
                        expected_keys=("assignment","monitoring","decision"),
                        style_map={"assignment":"rx","monitoring":"rx","decision":"rx"})
        self.r3  = _Row("3) 초기임무계획 모드 전환(0101)",
                        expected_keys=("assignment","monitoring","decision"),
                        style_map={"assignment":"rx","monitoring":"rx","decision":"rx"})
        self.r4  = _Row("4) [모니터링] 0902 생성",
                        expected_keys=("monitoring",),
                        style_map={"monitoring":"tx"})
        self.r5  = _Row("5) [할당] 0305 재계획 수립 '중' 생성",
                        expected_keys=("assignment",),
                        style_map={"assignment":"tx"})
        self.r6  = _Row("6) [할당] 0301~0304 생성",
                        expected_keys=("assignment",),
                        style_map={"assignment":"tx"})
        self.r7  = _Row("7) [할당] 0301 임무계획 전달",
                        expected_keys=("assignment",),
                        style_map={"assignment":"tx"})
        self.r8  = _Row("8) 0301 수신(모니터링·의사결정)",
                        expected_keys=("monitoring","decision"),
                        style_map={"monitoring":"rx","decision":"rx"})
        self.r9  = _Row("9) [할당] 0305 재계획 수립 '완료' 생성",
                        expected_keys=("assignment",),
                        style_map={"assignment":"tx"})
        self.r10 = _Row("10) [할당] 0901 옵션정보 생성 요청",
                        expected_keys=("assignment",),
                        style_map={"assignment":"tx"})
        self.r11 = _Row("11) [의사결정] 0701 옵션정보 생성",
                        expected_keys=("decision",),
                        style_map={"decision":"tx"})
        self.r12 = _Row("12) 0702(의사결정 결과) 수신",
                        expected_keys=("assignment","monitoring","decision"),
                        style_map={"assignment":"rx","monitoring":"rx","decision":"rx"})
        self.r13 = _Row("13) 시스템운용모드(대기) 수신",
                        expected_keys=("assignment","monitoring","decision"),
                        style_map={"assignment":"rx","monitoring":"rx","decision":"rx"})

        self.r1.set_detail("각 모듈이 0203을 수신(rx)했는지")
        self.r2.set_detail("각 모듈이 0201을 수신(rx)했는지")
        self.r3.set_detail("각 모듈이 초기임무계획 모드로 전환했는지")
        self.r4.set_detail("모니터링 모듈이 0902를 생성(tx)")
        self.r5.set_detail("할당 모듈이 0305('중')를 생성(tx) — 0301 이전")
        self.r6.set_detail("할당 모듈이 0301/0302/0303/0304 모두 생성(tx)")
        self.r7.set_detail("할당 모듈이 0301을 송신(tx)")
        self.r8.set_detail("모니터링·의사결정 모듈이 0301을 수신(rx)")
        self.r9.set_detail("할당 모듈이 0305('완료')를 생성(tx) — 0301 이후")
        self.r10.set_detail("할당 모듈이 0901을 송신(tx)")
        self.r11.set_detail("의사결정 모듈이 0701을 송신(tx)")
        self.r12.set_detail("세 모듈이 0702를 수신(rx)")
        self.r13.set_detail("세 모듈이 대기모드(standby)로 전환(rx)")

        for r in (self.r1,self.r2,self.r3,self.r4,self.r5,self.r6,self.r7,self.r8,self.r9,self.r10,self.r11,self.r12,self.r13):
            r.set_all(False)

        # ── 버튼 영역 ──
        self.btn_reset = QPushButton("초기화")
        self.btn_close = QPushButton("닫기")
        btns = QHBoxLayout()
        btns.addStretch(1)
        btns.addWidget(self.btn_reset)
        btns.addWidget(self.btn_close)

        # ── 스크롤 컨테이너 ──
        content = QWidget()
        content_v = QVBoxLayout(content)
        content_v.addWidget(legend)
        for r in (self.r1,self.r2,self.r3,self.r4,self.r5,self.r6,self.r7,self.r8,self.r9,self.r10,self.r11,self.r12,self.r13):
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

    def _on_reset(self):
        for r in (self.r1,self.r2,self.r3,self.r4,self.r5,self.r6,self.r7,self.r8,self.r9,self.r10,self.r11,self.r12,self.r13):
            r.reset()

class S110ChecklistController(QObject):
    """
    UDP dashPulse + 오케스트레이터 모드 이벤트를 후킹하여
    13개 항목을 자동 평가한다.
    """
    sig_set = pyqtSignal(int, str, bool, str)  # (row_idx, module_key, on, detail)

    def __init__(self, orch: Any):
        super().__init__()
        self.orch = orch
        self.ui = S110ChecklistDialog()
        self.ui.show()

        # 내부 상태
        self._rx0203: Dict[str,bool] = {}
        self._rx0201: Dict[str,bool] = {}
        self._mode_init: Dict[str,bool] = {}
        self._mode_standby: Dict[str,bool] = {}

        self._mon_tx_0902_done = False

        self._as_tx_0301 = False
        self._as_tx_0302 = False
        self._as_tx_0303 = False
        self._as_tx_0304 = False
        self._as_tx_0301_seen = False
        self._as_tx_0305_pre = False     # 0301 이전
        self._as_tx_0305_post = False    # 0301 이후
        self._as_tx_0901 = False

        self._de_tx_0701 = False

        self._rx0301: Dict[str,bool] = {}   # monitoring/decision
        self._rx0702: Dict[str,bool] = {}   # assignment/monitoring/decision

        # 신호 연결
        self.sig_set.connect(self._on_sig_set)

        # 훅 장착
        self._hook_dash_pulse()
        self._hook_mode_event()

    # UI 업데이트
    def _on_sig_set(self, idx: int, key: str, on: bool, detail: str):
        row_map = {
            1:self.ui.r1, 2:self.ui.r2, 3:self.ui.r3, 4:self.ui.r4, 5:self.ui.r5,
            6:self.ui.r6, 7:self.ui.r7, 8:self.ui.r8, 9:self.ui.r9, 10:self.ui.r10,
            11:self.ui.r11, 12:self.ui.r12, 13:self.ui.r13
        }
        row = row_map.get(int(idx))
        if not row:
            return
        if detail:
            row.set_detail(detail)
        row.set_ok(key, on)

    # 공용: role 정규화
    @staticmethod
    def _norm_role_to_key(role: str) -> Optional[str]:
        s = (role or "").strip().lower()
        if s in ("assignment","mission","mmr","mission_planning","assignment_planning"): return "assignment"
        if s in ("monitoring","msm"): return "monitoring"
        if s in ("decision","mob","decision_support"): return "decision"
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
        k = kind.lower()
        mid = self._norm_mid(msg_id)

        # 1) 0203 수신
        if k == "rx" and mid == "0203":
            if not self._rx0203.get(key, False):
                self._rx0203[key] = True
                self.sig_set.emit(1, key, True, "")

        # 2) 0201 수신
        if k == "rx" and mid == "0201":
            if not self._rx0201.get(key, False):
                self._rx0201[key] = True
                self.sig_set.emit(2, key, True, "")

        # 4) MSM 0902 생성
        if k == "tx" and mid == "0902" and key == "monitoring":
            if not self._mon_tx_0902_done:
                self._mon_tx_0902_done = True
                self.sig_set.emit(4, "monitoring", True, "")

        # 6) MMR 0301~0304 생성
        if k == "tx" and key == "assignment" and mid in ("0301","0302","0303","0304"):
            if mid == "0301":
                self._as_tx_0301 = True
                self._as_tx_0301_seen = True
                self.sig_set.emit(7, "assignment", True, "")  # 7) 0301 전달
            elif mid == "0302":
                self._as_tx_0302 = True
            elif mid == "0303":
                self._as_tx_0303 = True
            elif mid == "0304":
                self._as_tx_0304 = True
            # Row6 상세 갱신
            done = [m for m, ok in (("0301",self._as_tx_0301),("0302",self._as_tx_0302),("0303",self._as_tx_0303),("0304",self._as_tx_0304)) if ok]
            self.sig_set.emit(6, "assignment", len(done)==4, f"생성됨: {', '.join(done) or '-'}")

        # 5 & 9) 0305 (재계획 수립 중/완료)
        if k == "tx" and key == "assignment" and mid == "0305":
            if not self._as_tx_0301_seen and not self._as_tx_0305_pre:
                self._as_tx_0305_pre = True
                self.sig_set.emit(5, "assignment", True, "0301 이전 발생")
            elif self._as_tx_0301_seen and not self._as_tx_0305_post:
                self._as_tx_0305_post = True
                self.sig_set.emit(9, "assignment", True, "0301 이후 발생")

        # 8) 0301 수신(모니터링·의사결정)
        if k == "rx" and mid == "0301" and key in ("monitoring","decision"):
            if not self._rx0301.get(key, False):
                self._rx0301[key] = True
                self.sig_set.emit(8, key, True, "")

        # 10) 0901 옵션정보 생성 요청 by 할당
        if k == "tx" and key == "assignment" and mid == "0901":
            if not self._as_tx_0901:
                self._as_tx_0901 = True
                self.sig_set.emit(10, "assignment", True, "")

        # 11) 0701 옵션정보 생성 by 의사결정
        if k == "tx" and key == "decision" and mid == "0701":
            if not self._de_tx_0701:
                self._de_tx_0701 = True
                self.sig_set.emit(11, "decision", True, "")

        # 12) 0702 수신(전 모듈)
        if k == "rx" and mid == "0702":
            if not self._rx0702.get(key, False):
                self._rx0702[key] = True
                self.sig_set.emit(12, key, True, "")

        # 13)은 모드 이벤트에서 처리

    # 모드 이벤트 훅
    def _hook_mode_event(self):
        orig = getattr(self.orch, "_handle_mode_event", None)
        if not callable(orig):
            return
        def wrapper(src_role: str, text: str):
            try:
                key = self._norm_role_to_key(src_role)
                t = self._normalize_mode(text)
                if key:
                    if self._is_initial_plan(t) and not self._mode_init.get(key, False):
                        self._mode_init[key] = True
                        self.sig_set.emit(3, key, True, t)
                    if self._is_standby(t) and not self._mode_standby.get(key, False):
                        self._mode_standby[key] = True
                        self.sig_set.emit(13, key, True, t)
            except Exception:
                pass
            return orig(src_role, text)
        self.orch._handle_mode_event = wrapper  # type: ignore

    def _normalize_mode(self, text: str) -> str:
        # 오케스트레이터 제공 함수가 있으면 사용
        if hasattr(self.orch, "_normalize_mode_text"):
            try:
                text = str(self.orch._normalize_mode_text(text))
            except Exception:
                pass
        t = "".join(str(text or "").split()).lower()
        return t

    @staticmethod
    def _is_initial_plan(norm_text: str) -> bool:
        return norm_text in ("초기임무계획", "초기임무", "initplan", "initialplan", "3")

    @staticmethod
    def _is_standby(norm_text: str) -> bool:
        return norm_text in ("대기모드","대기","standby","2")

# 외부 진입점
def ensure_s110_checklist(orch: Any) -> S110ChecklistController:
    ctrl = getattr(orch, "_s110_checklist_ctrl", None)
    if isinstance(ctrl, S110ChecklistController):
        try:
            ctrl.ui.raise_(); ctrl.ui.activateWindow()
        except Exception:
            pass
        return ctrl
    ctrl = S110ChecklistController(orch)
    setattr(orch, "_s110_checklist_ctrl", ctrl)
    return ctrl
