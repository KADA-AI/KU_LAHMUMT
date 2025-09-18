# /modules/common/states/S101_run_all.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state

@register_state("S101")
def run(orch):
    orch._safe_log("[OPS] S101: SW 실행(전체 GUI)")
    orch._launch_all_guis()
