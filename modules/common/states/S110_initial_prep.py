# 파일: /modules/common/states/S110_initial_prep.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state
try:
    # S110 체크리스트가 있다면 함께 띄움(없어도 동작)
    from ..ops_checklist import ensure_s110_checklist
except Exception:
    ensure_s110_checklist = lambda _orch: None

@register_state("S110")
def run(orch):
    orch._safe_log("[OPS] S110: 초기임무계획 진입 요청")
    try:
        orch._set_mode_text_all("초기임무계획")
    except Exception:
        pass
    try:
        ensure_s110_checklist(orch)
    except Exception:
        pass

