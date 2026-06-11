"""Compatibility wrapper for replan ID reservation helpers."""

from __future__ import annotations

try:
    from ._compat_import import import_runtime_compat_module
except Exception:
    from _compat_import import import_runtime_compat_module

_MODULE = import_runtime_compat_module("modules.mission_planning.runtime.ids.replan_reservation", __file__)
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
