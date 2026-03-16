# /modules/common/states/manager.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any
from . import discover, get_state

class StateManager:
    def __init__(self):
        self._ready = False

    def _ensure_discovered(self):
        if not self._ready:
            discover()
            self._ready = True

    def dispatch(self, code: str, orch: Any) -> bool:
        self._ensure_discovered()
        fn = get_state(code)
        if fn is None:
            return False
        try:
            fn(orch)
            return True
        except Exception as e:
            try:
                orch._safe_log(f"[ERR] {code} 상태 처리 중 예외: {e}")
            except Exception:
                pass
            return False
