# 파일: /modules/common/states/S110_initial_prep.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state
try:
    from ..ops_checklist import ensure_s110_checklist
except Exception:
    ensure_s110_checklist = lambda _orch: None

@register_state("S110")
def run(orch):
    try:
        ensure_s110_checklist(orch)
    except Exception:
        pass
