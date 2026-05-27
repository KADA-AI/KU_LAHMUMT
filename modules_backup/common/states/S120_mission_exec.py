# /modules/common/states/S120_mission_exec.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state
from ..ops_checklist import ensure_s120_checklist

@register_state("S120")
def run(orch):
    try:
        ensure_s120_checklist(orch)
    except Exception:
        pass
