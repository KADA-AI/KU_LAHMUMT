"""Common receive center bridging Qt tabs and Python handlers.

This module unifies the listener model so legacy tabs that expect
`mark_received` callbacks and newer code that registers plain Python
callables can coexist. Incoming notifications are also propagated
through a Qt `SignalEmitter` for GUI updates.
"""

from __future__ import annotations

from functools import partial
import json
import sys
from typing import Callable, Dict, List, Optional

from PyQt5.QtCore import QObject, QTimer, QCoreApplication, pyqtSignal

# Keep a single receive_center module instance regardless of import path.
_this_module = sys.modules.get(__name__)
if _this_module is not None:
    sys.modules.setdefault("receive_center", _this_module)
    sys.modules.setdefault("modules.common.receive_center", _this_module)

# ── Listener registries ─────────────────────────────────────────────────────

_tab_registry: Dict[str, List[object]] = {}
_handler_registry: Dict[str, List[Callable[[str, object], None]]] = {}


class SignalEmitter(QObject):
    """Qt signal wrapper to deliver message updates on the GUI thread."""

    message_received = pyqtSignal(str, object)


_global_signal_emitter: Optional[SignalEmitter] = None


def set_global_signal_emitter(emitter: SignalEmitter) -> None:
    """Register the emitter used to broadcast message updates."""

    global _global_signal_emitter
    _global_signal_emitter = emitter


def get_global_signal_emitter() -> Optional[SignalEmitter]:
    return _global_signal_emitter


def _norm(msg_id: object) -> str:
    s = str(msg_id)
    return s.zfill(4) if s.isdigit() and len(s) < 4 else s


def register_listener(msg_id: str, listener) -> None:
    """Register either a tab (with mark_received) or a callable handler."""

    key = _norm(msg_id)
    if callable(listener):
        _handler_registry.setdefault(key, []).append(listener)
    else:
        _tab_registry.setdefault(key, []).append(listener)

    # print(
    #     f"[receive_center] registered listener for {key}: handlers={len(_handler_registry.get(key, []))}, tabs={len(_tab_registry.get(key, []))}",
    #     flush=True,
    # )


def unregister_listener(msg_id: str, listener) -> None:
    key = _norm(msg_id)
    registry = _handler_registry if callable(listener) else _tab_registry
    listeners = registry.get(key)
    if not listeners:
        return
    try:
        listeners.remove(listener)
        if not listeners:
            del registry[key]
    except ValueError:
        pass


def _decode_payload(raw: Optional[bytes]):
    if raw is None:
        return None
    if isinstance(raw, bytes):
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return raw
    return raw


def notify(msg_id: str, raw: Optional[bytes] = None) -> None:
    """Fan-out notification to registered listeners on the GUI thread."""

    key = _norm(msg_id)
    payload = _decode_payload(raw)

    tabs = _tab_registry.get(key, [])
    handlers = _handler_registry.get(key, [])

    def _dispatch(fn):
        if QCoreApplication.instance() is None:
            try:
                fn()
            except Exception:
                return
        else:
            QTimer.singleShot(0, fn)

    for tab in tabs:
        _dispatch(partial(tab.mark_received, key, raw))

    for handler in handlers:
        _dispatch(partial(handler, key, payload))

    # print(
    #     f"[receive_center] notify {key}: handlers={len(handlers)}, tabs={len(tabs)}, emitter={'set' if _global_signal_emitter else 'none'}",
    #     flush=True,
    # )

    if _global_signal_emitter is not None:
        _dispatch(lambda: _global_signal_emitter.message_received.emit(key, payload))

    if not tabs and not handlers and _global_signal_emitter is None:
        # Nothing registered for incoming message; drop silently to avoid noisy output.
        return
