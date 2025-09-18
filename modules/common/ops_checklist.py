# 파일: modules/common/ops_checklist.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, Optional
import re, json

from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtWidgets import (
    QDialog, QLabel, QVBoxLayout, QHBoxLayout,
    QGridLayout, QPushButton, QFrame
)

# ─────────────────────────────────────────────────────────────
# 간단한 UI 위젯들
class _Row(QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setFrameShadow(QFrame.Raised)

        self._title = QLabel(title)
        self._title.setStyleSheet("font-weight: 600;")

        self._count = QLabel("0/3")
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
        lay.addWidget(self._dot_a, 1, 0)
        lay.addWidget(self._dot_b, 1, 1)
        lay.addWidget(self._dot_c, 1, 2)
        lay.addWidget(self._detail, 2, 0, 1, 5)

        self._done_flags: Dict[str, bool] = {"assignment": False, "monitoring": False, "decision": False}

    def set_flag(self, key: str, on: bool):
        if key in self._done_flags:
            self._done_flags[key] = bool(on)
            self._recalc()
            self._update_dots()

    def set_all(self, on: bool):
        for k in list(self._done_flags.keys()):
            self._done_flags[k] = bool(on)
        self._recalc()
        self._update_dots()

    def _update_dots(self):
        keys = ["assignment", "monitoring", "decision"]
        dots = [self._dot_a, self._dot_b, self._dot_c]
        for k, d in zip(keys, dots):
            ok = self._done_flags.get(k, False)
            d.setStyleSheet("color: #2aa745; font-weight:700;" if ok else "color:#999;")

    def set_detail(self, text: str):
        self._detail.setText(str(text or "-"))

    def _recalc(self):
        n = sum(1 for v in self._done_flags.values() if v)
        self._count.setText(f"{n}/3")
        if n >= 3:
            self._status.setText("완료")
        elif n == 0:
            self._status.setText("미완료")
        else:
            self._status.setText("진행중")

    def reset(self):
        for k in self._done_flags:
            self._done_flags[k] = False
        self._recalc()
        self._detail.setText("-")


class S100ChecklistDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("S100 운용 체크리스트")
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.setMinimumWidth(520)

        self.r1 = _Row("1) SW 실행 여부 확인 - 3개중 3개완료")
        self.r2 = _Row("2) 모듈 상태정보 송신(0102) - 3개중 3개 완료")
        self.r3 = _Row("3) 대기모드 전환(Mode=대기모드) - 3개중 3개 완료")

        self.r1.set_detail("mission / monitoring / decision 실행 감시")
        self.r2.set_detail("각 모듈의 0102 최초 송신 감지")
        self.r3.set_detail("각 모듈의 UDP mode=대기모드 감지")

        for r in (self.r1, self.r2, self.r3):
            r.set_flag("assignment", False)
            r.set_flag("monitoring", False)
            r.set_flag("decision",   False)

        self.btn_reset = QPushButton("초기화")
        self.btn_close = QPushButton("닫기")

        btns = QHBoxLayout()
        btns.addStretch(1)
        btns.addWidget(self.btn_reset)
        btns.addWidget(self.btn_close)

        lay = QVBoxLayout(self)
        lay.addWidget(self.r1)
        lay.addWidget(self.r2)
        lay.addWidget(self.r3)
        lay.addLayout(btns)

        self.btn_reset.clicked.connect(self._on_reset)
        self.btn_close.clicked.connect(self.close)

    def _on_reset(self):
        self.r1.reset(); self.r2.reset(); self.r3.reset()


# ─────────────────────────────────────────────────────────────
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
        "decision-support":        "decision",
        "decision":                "decision",
        "mob":                     "decision",
    }

    def __init__(self, orch: Any, parent=None):
        super().__init__(parent)
        self.orch = orch
        self.ui = S100ChecklistDialog(getattr(orch, "win", None))
        self.ui.show()

        # 상태
        self._launched: Dict[str, bool]  = {"assignment": False, "monitoring": False, "decision": False}
        self._sent0102: Dict[str, bool]  = {"assignment": False, "monitoring": False, "decision": False}
        self._standby:  Dict[str, bool]  = {"assignment": False, "monitoring": False, "decision": False}

        # UI 신호 연결
        self.sig_set_launch.connect(lambda k, v: (self.ui.r1.set_flag(k, v), self._refresh_r1()))
        self.sig_set_0102.connect(lambda k, v: (self.ui.r2.set_flag(k, v), self._refresh_r2()))
        self.sig_set_standby.connect(lambda k, v: (self.ui.r3.set_flag(k, v), self._refresh_r3()))

        # orch 훅
        self._hook_logging()        # [RUN] 로그 (백업 신호)
        self._hook_mark_received()  # 버스 수신 0102 (백업)
        self._hook_dash_pulse()     # UDP 모니터(tx/rx)
        self._hook_mode_event()     # ★ UDP mode 이벤트(대기모드 전환) 추적

    # ---------- 유틸 ----------
    @staticmethod
    def _norm_role_to_key(role: str) -> Optional[str]:
        return S100ChecklistController.SCRIPT_TO_KEY.get((role or "").strip().lower())

    def _refresh_r1(self):
        on = [k for k, v in self._launched.items() if v]
        off = [k for k, v in self._launched.items() if not v]
        self.ui.r1.set_detail(f"ON: {', '.join(on) or '-'} / WAIT: {', '.join(off) or '-'}")

    def _refresh_r2(self):
        on = [k for k, v in self._sent0102.items() if v]
        off = [k for k, v in self._sent0102.items() if not v]
        self.ui.r2.set_detail(f"FIRST 0102: {', '.join(on) or '-'} / WAIT: {', '.join(off) or '-'}")

    def _refresh_r3(self):
        on = [k for k, v in self._standby.items() if v]
        off = [k for k, v in self._standby.items() if not v]
        self.ui.r3.set_detail(f"STANDBY: {', '.join(on) or '-'} / WAIT: {', '.join(off) or '-'}")

    # ---------- 훅 1: 대시보드 전역 로그 → [RUN] 감지 ----------
    def _hook_logging(self):
        orig = getattr(self.orch, "_log_everywhere", None)
        if not callable(orig):
            return
        def wrapper(text: str):
            try:
                self._on_log(text)
            except Exception:
                pass
            return orig(text)
        self.orch._log_everywhere = wrapper  # type: ignore

    def _on_log(self, text: str):
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
        src = (obj.get("Source") or obj.get("source") or obj.get("SourceModuleName")
               or obj.get("requestModuleName") or "")
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
