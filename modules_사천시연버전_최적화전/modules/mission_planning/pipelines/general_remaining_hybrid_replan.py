"""Compatibility wrapper for general remaining-mission hybrid replanning."""

from __future__ import annotations

from importlib import import_module

_MODULE = import_module("modules.mission_planning.replanning.triggers.remaining_hybrid.general")

for _name, _value in vars(_MODULE).items():
    if _name.startswith("__") and _name.endswith("__"):
        continue
    globals()[_name] = _value

if hasattr(_MODULE, "__all__"):
    __all__ = list(_MODULE.__all__)
else:
    __all__ = [
        _name
        for _name in vars(_MODULE).keys()
        if not (_name.startswith("__") and _name.endswith("__"))
    ]
