# /modules/common/states/__init__.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Callable, Dict

STATE_REGISTRY: Dict[str, Callable] = {}

def register_state(code: str):
    code = str(code).upper().strip()
    def _decorator(fn: Callable):
        STATE_REGISTRY[code] = fn
        return fn
    return _decorator

def get_state(code: str):
    return STATE_REGISTRY.get(str(code).upper().strip())

def discover() -> Dict[str, Callable]:
    import pkgutil, importlib
    pkg_name = __name__
    for m in pkgutil.iter_modules(__path__):  # type: ignore[name-defined]
        if m.ispkg: continue
        importlib.import_module(f"{pkg_name}.{m.name}")
    return dict(STATE_REGISTRY)
