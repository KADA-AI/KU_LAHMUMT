# /modules/common/states/S100_init.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state
from ..ops_checklist import ensure_s100_checklist

@register_state("S100")
def run(orch):
    try:
        ensure_s100_checklist(orch)
    except Exception:
        pass
