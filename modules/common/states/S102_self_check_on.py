# /modules/common/states/S102_self_check_on.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from . import register_state

@register_state("S102")
def run(orch):
    orch._self_check_all(True)
