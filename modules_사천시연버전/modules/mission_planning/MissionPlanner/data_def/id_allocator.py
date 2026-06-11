"""Compatibility proxy for mission generation ID allocation."""

from __future__ import annotations

import sys
from importlib import import_module
from pathlib import Path
from types import ModuleType

_PROJECT_ROOT = next(
    (
        parent
        for parent in Path(__file__).resolve().parents
        if (parent / "modules" / "common").exists()
    ),
    Path(__file__).resolve().parents[4],
)
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

_MODULE = import_module("modules.mission_planning.engine.mission_generation.id_allocation.allocator")
_PROXY_LOCAL_NAMES = {
    "_MODULE",
    "_PROXY_LOCAL_NAMES",
    "_AllocatorProxy",
    "__all__",
    "__builtins__",
    "__cached__",
    "__class__",
    "__dict__",
    "__doc__",
    "__file__",
    "__loader__",
    "__name__",
    "__package__",
    "__path__",
    "__spec__",
}


class _AllocatorProxy(ModuleType):
    def __getattribute__(self, name: str):
        if name in _PROXY_LOCAL_NAMES or (name.startswith("__") and name.endswith("__")):
            return ModuleType.__getattribute__(self, name)
        return getattr(_MODULE, name)

    def __getattr__(self, name: str):
        return getattr(_MODULE, name)

    def __setattr__(self, name: str, value):
        if name in _PROXY_LOCAL_NAMES or (name.startswith("__") and name.endswith("__")):
            ModuleType.__setattr__(self, name, value)
            return
        setattr(_MODULE, name, value)


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

sys.modules[__name__].__class__ = _AllocatorProxy
