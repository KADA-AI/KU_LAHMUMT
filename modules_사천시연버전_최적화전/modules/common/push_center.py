import importlib
import json
import os
import socket
import sys

try:
    from modules.common.push_type_cache import resolve_module
except Exception:
    resolve_module = None


_DASH_PORT = int(os.getenv("KU_DASHBOARD_PORT", "45991"))

SEARCH_PREFIXES = [
    "generator",
    "push_info",
    "push",
    "modules.common.push_info",
    "modules.common.push",
    "modules.decision_support.push_info",
    "modules.decision_support.push",
]

_PUSH_MODULE_CACHE = {}


def _resolve_push_module(msg_id: str):
    mid = str(msg_id or "").strip()
    if not mid:
        raise ImportError("empty message id")

    cached = _PUSH_MODULE_CACHE.get(mid)
    if cached is not None:
        return cached

    last_exc = None
    for pref in SEARCH_PREFIXES:
        module_name = f"{pref}.message{mid}_push"
        try:
            mod = resolve_module(module_name) if callable(resolve_module) else importlib.import_module(module_name)
            _PUSH_MODULE_CACHE[mid] = mod
            return mod
        except Exception as exc:
            last_exc = exc

    raise last_exc or ImportError(f"message{mid}_push not found in {SEARCH_PREFIXES}")


def push_message(msg_id: str, messenger, *, on_done=None, body_dict=None) -> bool:
    try:
        mod = _resolve_push_module(msg_id)
        use_manual = isinstance(body_dict, dict) and len(body_dict) > 0

        if use_manual and hasattr(mod, "make_and_push"):
            raw = mod.make_and_push(body_dict, messenger)
        elif hasattr(mod, "make_random_and_push"):
            raw = mod.make_random_and_push(messenger)
        elif hasattr(mod, "push"):
            raw = mod.push(messenger)
        else:
            raise AttributeError(
                f"{mod.__name__} has no push entrypoint (make_and_push/make_random_and_push/push)"
            )
    except Exception as exc:
        try:
            sys.stderr.write(f"[WARN] push_message({msg_id}) failed: {exc}\n")
        except Exception:
            pass
        return False

    if on_done:
        on_done(msg_id, raw)
    return True
