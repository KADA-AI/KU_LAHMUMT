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
import threading
import weakref
from typing import Callable, Dict, List, Optional

from PyQt5.QtCore import QObject, QCoreApplication, QTimer, pyqtSignal

from modules.common.qt_safety import is_qt_object_alive

# Keep a single receive_center module instance regardless of import path.
_this_module = sys.modules.get(__name__)
if _this_module is not None:
    sys.modules.setdefault("receive_center", _this_module)
    sys.modules.setdefault("modules.common.receive_center", _this_module)

# ── Listener registries ─────────────────────────────────────────────────────

_tab_registry: Dict[str, List[object]] = {}
_handler_registry: Dict[str, List[Callable[[str, object], None]]] = {}
_coalesce_lock = threading.Lock()
_coalesced_dispatches: Dict[tuple[str, str, int], dict[str, object]] = {}


class SignalEmitter(QObject):
    """Qt signal wrapper to deliver message updates on the GUI thread."""

    message_received = pyqtSignal(str, object)


class _GuiDispatchEmitter(QObject):
    invoke = pyqtSignal(object)

    def __init__(self) -> None:
        super().__init__()
        self.invoke.connect(self._run)

    def _run(self, fn: object) -> None:
        try:
            if callable(fn):
                fn()
        except Exception:
            return


_global_signal_emitter: Optional[SignalEmitter] = None
_gui_dispatch_emitter: Optional[_GuiDispatchEmitter] = None


def _ensure_gui_dispatch_emitter() -> Optional[_GuiDispatchEmitter]:
    app = QCoreApplication.instance()
    if app is None:
        return None
    global _gui_dispatch_emitter
    if _gui_dispatch_emitter is None:
        emitter = _GuiDispatchEmitter()
        try:
            emitter.moveToThread(app.thread())
        except Exception:
            pass
        _gui_dispatch_emitter = emitter
    return _gui_dispatch_emitter


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
    _ensure_gui_dispatch_emitter()
    if callable(listener):
        bucket = _handler_registry.setdefault(key, [])
        if listener not in bucket:
            bucket.append(listener)
    else:
        bucket = _tab_registry.setdefault(key, [])
        if listener not in bucket:
            bucket.append(listener)
            try:
                listener_ref = weakref.ref(listener)

                def _cleanup(_obj=None, *, _msg_id=key, _ref=listener_ref) -> None:
                    obj = _ref()
                    if obj is not None:
                        unregister_listener(_msg_id, obj)

                listener.destroyed.connect(_cleanup)
            except Exception:
                pass

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


def _coalesce_interval_ms(listener: object, msg_id: str) -> int:
    """Return listener-requested GUI dispatch coalescing interval for msg_id."""

    configs = (
        getattr(listener, "RECEIVE_COALESCE_MESSAGES", None),
        getattr(listener, "receive_coalesce_messages", None),
    )
    raw_value = None
    for config in configs:
        if isinstance(config, dict):
            raw_value = config.get(msg_id)
            if raw_value is None:
                raw_value = config.get("*")
        elif isinstance(config, (set, list, tuple)) and msg_id in {str(v) for v in config}:
            raw_value = getattr(listener, "RECEIVE_COALESCE_MS", None)
            if raw_value is None:
                raw_value = getattr(listener, "receive_coalesce_ms", 80)
        if raw_value is not None:
            break

    if raw_value is None:
        raw_value = getattr(listener, f"receive_coalesce_{msg_id}_ms", None)
    if raw_value is None:
        return 0
    try:
        return max(1, min(2000, int(raw_value)))
    except Exception:
        return 0


def _listener_ref(listener: object):
    try:
        return weakref.ref(listener)
    except Exception:
        return lambda: listener


def _drop_coalesced_dispatch(state_key: tuple[str, str, int]) -> None:
    with _coalesce_lock:
        _coalesced_dispatches.pop(state_key, None)


def _dispatch_coalesced(
    *,
    kind: str,
    msg_id: str,
    listener: object,
    payload: object,
    interval_ms: int,
    dispatch,
    invoke,
) -> None:
    if interval_ms <= 0 or QCoreApplication.instance() is None:
        dispatch(lambda: invoke(payload))
        return

    state_key = (kind, msg_id, id(listener))
    with _coalesce_lock:
        state = _coalesced_dispatches.get(state_key)
        if state is None:
            state = {
                "listener_ref": _listener_ref(listener),
                "payload": payload,
                "scheduled": False,
            }
            _coalesced_dispatches[state_key] = state
        else:
            state["payload"] = payload
        if bool(state.get("scheduled")):
            return
        state["scheduled"] = True

    def _drain() -> None:
        with _coalesce_lock:
            state = _coalesced_dispatches.get(state_key)
            if state is None:
                return
            state["scheduled"] = False
            latest_payload = state.get("payload")
            state["payload"] = None
            ref = state.get("listener_ref")

        target = ref() if callable(ref) else None
        if target is None:
            _drop_coalesced_dispatch(state_key)
            return
        if kind == "tab" and not is_qt_object_alive(target):
            unregister_listener(msg_id, target)
            _drop_coalesced_dispatch(state_key)
            return
        try:
            invoke(latest_payload)
        except Exception:
            return

    def _schedule() -> None:
        QTimer.singleShot(int(interval_ms), _drain)

    dispatch(_schedule)


def notify(msg_id: str, raw: Optional[bytes] = None) -> None:
    """Fan-out notification to registered listeners on the GUI thread."""

    key = _norm(msg_id)

    tabs = list(_tab_registry.get(key, []))
    handlers = list(_handler_registry.get(key, []))
    payload_decoded = False
    payload_cache = None

    def _payload_value():
        nonlocal payload_decoded, payload_cache
        if not payload_decoded:
            payload_cache = _decode_payload(raw)
            payload_decoded = True
        return payload_cache

    def _dispatch(fn):
        dispatcher = _ensure_gui_dispatch_emitter()
        if dispatcher is None:
            try:
                fn()
            except Exception:
                return
        else:
            dispatcher.invoke.emit(fn)

    for tab in tabs:
        if not is_qt_object_alive(tab):
            unregister_listener(key, tab)
            continue
        interval_ms = _coalesce_interval_ms(tab, key)
        if interval_ms > 0:
            _dispatch_coalesced(
                kind="tab",
                msg_id=key,
                listener=tab,
                payload=raw,
                interval_ms=interval_ms,
                dispatch=_dispatch,
                invoke=lambda latest, _tab=tab: _tab.mark_received(key, latest),
            )
        else:
            _dispatch(partial(tab.mark_received, key, raw))

    for handler in handlers:
        interval_ms = _coalesce_interval_ms(handler, key)
        if interval_ms > 0:
            _dispatch_coalesced(
                kind="handler",
                msg_id=key,
                listener=handler,
                payload=raw,
                interval_ms=interval_ms,
                dispatch=_dispatch,
                invoke=lambda latest, _handler=handler: _handler(key, _decode_payload(latest)),
            )
        else:
            _dispatch(partial(handler, key, _payload_value()))

    # print(
    #     f"[receive_center] notify {key}: handlers={len(handlers)}, tabs={len(tabs)}, emitter={'set' if _global_signal_emitter else 'none'}",
    #     flush=True,
    # )

    if _global_signal_emitter is not None:
        emitter_payload = _payload_value()
        _dispatch(lambda: _global_signal_emitter.message_received.emit(key, emitter_payload))

    if not tabs and not handlers and _global_signal_emitter is None:
        # Nothing registered for incoming message; drop silently to avoid noisy output.
        return
