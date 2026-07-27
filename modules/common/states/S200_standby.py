# /modules/common/states/S200_standby.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from . import register_state

@register_state("S200")
def run(orch):
    """Enter standby mode across all modules."""
    orch._enter_standby()
