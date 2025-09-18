# /modules/common/states/S100_init.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from PyQt5.QtCore import QTimer
from . import register_state

@register_state("S100")
def run(orch):
    orch._safe_log("[OPS] S100: 초기화")
    orch._set_mode_text_all("초기화")
    orch._launch_all_guis()
    QTimer.singleShot(2000, lambda: orch._self_check_all(True))
