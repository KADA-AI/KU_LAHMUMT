# /modules/common/states/S210_normal_exec.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state
from ..ops_checklist import ensure_s210_checklist

@register_state("S210")
def run(orch):
    try:
        ensure_s210_checklist(orch)
    except Exception:
        pass

