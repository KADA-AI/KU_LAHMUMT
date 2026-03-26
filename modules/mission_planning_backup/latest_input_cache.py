"""Backward-compatible wrapper for latest input cache helper."""

from __future__ import annotations

import sys
from importlib import import_module
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

_MODULE = import_module("modules.mission_planning.runtime.latest_input_cache")
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
