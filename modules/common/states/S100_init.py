# /modules/common/states/S100_init.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from PyQt5.QtCore import QTimer
from . import register_state
from ..ops_checklist import ensure_s100_checklist  # ← 추가

@register_state("S100")
def run(orch):
    orch._safe_log("[OPS] S100: 초기화")
    orch._set_mode_text_all("초기화")
    orch._launch_all_guis()

    ensure_s100_checklist(orch)  # ← S100 체크리스트 창/모니터링 시작

    QTimer.singleShot(2000, lambda: orch._self_check_all(True))
