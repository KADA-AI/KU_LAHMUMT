# /modules/common/states/S300_mission_mode.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from . import register_state

@register_state("S300")
def run(orch):
    """Switch the system into mission execution mode."""
    orch._set_mode_text_all("임무 수행")
    orch._broadcast_ctrl({"cmd": "mode", "text": "임무 수행"})
    orch._safe_log("모든 SW 임무 수행 모드 진입")
