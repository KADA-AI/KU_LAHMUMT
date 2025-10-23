# /mnt/data/modules/common/checklists/base.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Set, Optional
from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QWidget, QGridLayout
from PyQt5.QtCore import Qt

@dataclass
class ItemState:
    done: bool = False
    note: str = ""

class BaseChecklistDialog(QDialog):
    def __init__(self, title: str, item_desc: Dict[int, str], parent=None):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.setModal(False)
        self.setMinimumWidth(520)
        self._desc = item_desc
        self._labels: Dict[int, QLabel] = {}
        lay = QVBoxLayout()
        title_w = QLabel(title)
        title_w.setAlignment(Qt.AlignCenter)
        title_w.setStyleSheet("font-weight:600; font-size:15px;")
        lay.addWidget(title_w)
        grid = QGridLayout()
        for idx in sorted(item_desc.keys()):
            lbl = QLabel(f"[ ] {idx:02d}. {item_desc[idx]}")
            lbl.setStyleSheet("font-size:13px;")
            grid.addWidget(lbl, idx-1, 0)
            self._labels[idx] = lbl
        wrap = QWidget()
        wrap.setLayout(grid)
        lay.addWidget(wrap)
        self.setLayout(lay)

    def mark(self, idx: int, note: str = ""):
        if idx not in self._labels:
            return
        self._labels[idx].setText(f"[✔] {idx:02d}. {self._desc[idx]}  {note}")
        self._labels[idx].setStyleSheet("color:#1a7f37; font-size:13px;")

class BaseChecklistController:
    """공통 이벤트 훅/마킹 로직. 서브클래스에서 _recalc/handle_* 구현."""
    ROLES = {"monitoring": "MSM", "mission": "MMR", "decision": "MOB"}

    def __init__(self, orch, title: str, item_desc: Dict[int, str]):
        self.orch = orch
        # UI
        parent = getattr(orch, "get_main_window", lambda: None)()
        self.view = BaseChecklistDialog(title, item_desc, parent=parent)
        self.view.show()
        # 상태
        self._state: Dict[int, ItemState] = {i: ItemState() for i in item_desc.keys()}
        self._rx_seen: Dict[str, Set[str]] = {}     # code -> {MSM/MMR/MOB}
        self._tx_created: Set[str] = set()          # "MSM:0301" 식
        self._mode_seen: Set[str] = set()           # "MSM:INIT_PLAN" 식
        self._gui_seen: Set[str]  = set()           # "MSM","MMR","MOB"
        # 훅
        if hasattr(orch, "hook_on_bus"):
            orch.hook_on_bus(self._on_bus)
        if hasattr(orch, "hook_on_mode"):
            orch.hook_on_mode(self._on_mode)
        if hasattr(orch, "hook_on_gui_launch"):
            orch.hook_on_gui_launch(self._on_gui)

    # ── 공통 헬퍼 ──
    def _role_abbr(self, role: str) -> str:
        return self.ROLES.get(role, role).upper()

    def _mark_once(self, idx: int, note: str = ""):
        st = self._state.get(idx)
        if st and not st.done:
            st.done, st.note = True, note
            self.view.mark(idx, note)

    # ── 이벤트 엔트리 ──
    def _on_bus(self, role: str, direction: str, code: str, payload: dict, t_ms: int):
        self.handle_bus(role, direction, code, payload, t_ms)
        self._recalc()

    def _on_mode(self, role: str, mode_code: str, t_ms: int):
        self.handle_mode(role, mode_code, t_ms)
        self._recalc()

    def _on_gui(self, role: str, ok: bool, t_ms: int):
        self.handle_gui(role, ok, t_ms)
        self._recalc()

    # ── 서브클래스가 오버라이드 ──
    def handle_bus(self, role: str, direction: str, code: str, payload: dict, t_ms: int):
        pass

    def handle_mode(self, role: str, mode_code: str, t_ms: int):
        pass

    def handle_gui(self, role: str, ok: bool, t_ms: int):
        pass

    def _recalc(self):
        pass
