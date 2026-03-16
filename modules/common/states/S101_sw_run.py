# /modules/common/states/S101_sw_run.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from PyQt5.QtCore import QTimer

from . import register_state

@register_state("S101")
def run(orch):
    """Launch all GUIs when the SW 실행 button is pressed."""
    orch._safe_log("[OPS] S101: SW 실행 시작")
    orch._launch_all_guis()
    # short delay before toggling self-check so modules have time to boot
    QTimer.singleShot(1000, lambda: orch._self_check_all(True))
