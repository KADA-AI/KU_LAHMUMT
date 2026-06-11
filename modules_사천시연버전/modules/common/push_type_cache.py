from __future__ import annotations

import importlib
from threading import RLock
from types import ModuleType
from typing import Any, Dict, Iterable, Iterator, Optional, Tuple


_LOCK = RLock()
_MODULE_CACHE: Dict[str, ModuleType] = {}
_TYPE_CACHE: Dict[Tuple[Tuple[str, ...], str], Any] = {}


def resolve_module(module_name: str) -> ModuleType:
    name = str(module_name or "").strip()
    if not name:
        raise ImportError("empty module name")
    with _LOCK:
        cached = _MODULE_CACHE.get(name)
        if cached is not None:
            return cached
    module = importlib.import_module(name)
    with _LOCK:
        return _MODULE_CACHE.setdefault(name, module)


def _module_names_for_message(msg_id: str) -> tuple[str, ...]:
    mid = str(msg_id or "").strip()
    return (
        f"nFusion.Model.msg_{mid}",
        "nFusion.Model.CommonType",
        "nFusion.Model",
    )


def resolve_csharp_type(
    msg_id: str,
    type_name: str,
    *,
    globals_dict: Optional[Dict[str, Any]] = None,
    module_names: Optional[Iterable[str]] = None,
) -> Any:
    name = str(type_name or "").strip()
    if not name:
        return None
    if isinstance(globals_dict, dict):
        value = globals_dict.get(name)
        if value is not None:
            return value

    modules = tuple(
        str(item)
        for item in (module_names or _module_names_for_message(msg_id))
        if str(item or "").strip()
    )
    key = (modules, name)
    with _LOCK:
        cached = _TYPE_CACHE.get(key)
        if cached is not None:
            return cached

    for module_name in modules:
        try:
            module = resolve_module(module_name)
        except Exception:
            continue
        value = getattr(module, name, None)
        if value is not None:
            with _LOCK:
                _TYPE_CACHE[key] = value
            return value
    return None


def iter_csharp_types(module_names: Iterable[str]) -> Iterator[tuple[str, str, Any]]:
    for module_name in tuple(str(item) for item in module_names if str(item or "").strip()):
        try:
            module = resolve_module(module_name)
        except Exception:
            continue
        for name, value in list(getattr(module, "__dict__", {}).items()):
            yield module_name, str(name), value


def cache_stats() -> Dict[str, int]:
    with _LOCK:
        return {
            "modules": len(_MODULE_CACHE),
            "types": len(_TYPE_CACHE),
        }


def clear_cache() -> None:
    with _LOCK:
        _MODULE_CACHE.clear()
        _TYPE_CACHE.clear()
