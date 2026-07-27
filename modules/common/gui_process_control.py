# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import time
from typing import Callable

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QApplication


_TRUE_VALUES = {"1", "true", "yes", "on"}


def env_flag(name: str, default: bool = False) -> bool:
    raw = str(os.environ.get(name, "")).strip().lower()
    if not raw:
        return bool(default)
    return raw in _TRUE_VALUES


def should_start_hidden() -> bool:
    return env_flag("KU_START_HIDDEN", False)


def should_hide_on_close() -> bool:
    return env_flag("KU_HIDE_ON_CLOSE", env_flag("KU_LAUNCHED_BY_DASHBOARD", False))


def should_keep_alive_when_hidden() -> bool:
    return should_start_hidden() or should_hide_on_close()


def keep_process_alive_when_hidden(app=None) -> None:
    if not should_keep_alive_when_hidden():
        return
    if app is None:
        try:
            app = QApplication.instance()
        except Exception:
            app = None
    if app is None:
        return
    try:
        app.setQuitOnLastWindowClosed(False)
    except Exception:
        pass


def show_window(window) -> None:
    keep_process_alive_when_hidden()
    _ensure_window_on_screen(window)
    _show_window_once(window)
    try:
        QTimer.singleShot(120, lambda: _show_window_once(window))
        QTimer.singleShot(420, lambda: _show_window_once(window))
    except Exception:
        pass


def _show_window_once(window) -> None:
    try:
        if window.windowState() & Qt.WindowMinimized:
            window.setWindowState(window.windowState() & ~Qt.WindowMinimized)
    except Exception:
        pass
    try:
        if window.isMaximized():
            window.showMaximized()
        else:
            window.showNormal()
    except Exception:
        try:
            window.show()
        except Exception:
            pass
    try:
        _ensure_window_on_screen(window)
    except Exception:
        pass
    try:
        window.raise_()
    except Exception:
        pass
    try:
        window.activateWindow()
    except Exception:
        pass
    _force_windows_foreground(window)


def _ensure_window_on_screen(window) -> None:
    app = QApplication.instance()
    if app is None:
        return
    try:
        frame = window.frameGeometry()
    except Exception:
        return
    try:
        width = max(100, int(frame.width()) or int(window.width()))
        height = max(100, int(frame.height()) or int(window.height()))
        left = int(frame.x())
        top = int(frame.y())
    except Exception:
        return
    for screen in app.screens():
        try:
            area = screen.availableGeometry()
            if area.intersects(frame):
                return
        except Exception:
            continue
    try:
        screen = app.primaryScreen()
        area = screen.availableGeometry() if screen is not None else None
    except Exception:
        area = None
    if area is None:
        return
    try:
        x = max(area.x(), min(left, area.x() + area.width() - width))
        y = max(area.y(), min(top, area.y() + area.height() - height))
        if x == left and y == top:
            x = area.x() + 72
            y = area.y() + 72
        window.move(int(x), int(y))
    except Exception:
        pass


def _force_windows_foreground(window) -> None:
    if os.name != "nt":
        return
    try:
        hwnd = int(window.winId())
    except Exception:
        hwnd = 0
    if not hwnd:
        return
    try:
        import ctypes

        user32 = ctypes.windll.user32
        sw_restore = 9
        sw_show = 5
        hwnd_topmost = -1
        hwnd_notopmost = -2
        swp_nosize = 0x0001
        swp_nomove = 0x0002
        swp_showwindow = 0x0040
        try:
            if user32.IsIconic(hwnd):
                user32.ShowWindowAsync(hwnd, sw_restore)
            else:
                user32.ShowWindowAsync(hwnd, sw_show)
        except Exception:
            pass
        flags = swp_nomove | swp_nosize | swp_showwindow
        try:
            user32.SetWindowPos(hwnd, hwnd_topmost, 0, 0, 0, 0, flags)
            user32.SetWindowPos(hwnd, hwnd_notopmost, 0, 0, 0, 0, flags)
        except Exception:
            pass
        try:
            user32.BringWindowToTop(hwnd)
        except Exception:
            pass
        try:
            user32.SetForegroundWindow(hwnd)
        except Exception:
            pass
    except Exception:
        pass


def window_state_summary(window) -> str:
    parts: list[str] = []
    for name, getter in (
        ("visible", lambda: window.isVisible()),
        ("minimized", lambda: bool(window.windowState() & Qt.WindowMinimized)),
        ("active", lambda: window.isActiveWindow()),
    ):
        try:
            parts.append(f"{name}={bool(getter())}")
        except Exception:
            pass
    try:
        geom = window.frameGeometry()
        parts.append(
            f"geom={int(geom.x())},{int(geom.y())},{int(geom.width())}x{int(geom.height())}"
        )
    except Exception:
        pass
    return ", ".join(parts)


def hide_window(window) -> None:
    try:
        window.hide()
    except Exception:
        pass


def apply_initial_visibility(app, window, position_window_from_env: Callable) -> None:
    keep_process_alive_when_hidden(app)
    try:
        position_window_from_env(app, window)
    except Exception:
        pass
    if should_start_hidden():
        try:
            window.hide()
        except Exception:
            pass
        return
    show_window(window)


def handle_window_control(window, payload: dict, *, role: str, log: Callable[[str], None] | None = None) -> bool:
    if not isinstance(payload, dict):
        return False
    target = str(payload.get("role") or payload.get("target") or "").strip().lower()
    aliases = {str(role).strip().lower(), "all", "*"}
    if role == "monitor":
        aliases.add("monitoring")
    if role == "mission":
        aliases.update({"assignment", "mission_planning", "mmr"})
    if role == "decision":
        aliases.update({"mob", "decision_support"})
    if role == "info":
        aliases.update({"inf", "info_manage"})
    if target and target not in aliases:
        return False

    cmd = str(payload.get("cmd") or "").strip().lower()
    if cmd in {"show_window", "show_gui", "open_gui", "raise_window"}:
        request_id = str(payload.get("request_id") or payload.get("requestId") or "").strip()
        if request_id:
            now = time.monotonic()
            try:
                seen = getattr(window, "_ku_seen_show_request_ids", None)
                if not isinstance(seen, dict):
                    seen = {}
                    setattr(window, "_ku_seen_show_request_ids", seen)
                for key, seen_at in list(seen.items()):
                    try:
                        if now - float(seen_at) > 10.0:
                            seen.pop(key, None)
                    except Exception:
                        seen.pop(key, None)
                if request_id in seen:
                    return True
                seen[request_id] = now
            except Exception:
                pass
        delay_ms = 0
        try:
            delay_ms = max(0, int(payload.get("delay_ms") or 0))
        except Exception:
            delay_ms = 0
        if delay_ms > 0:
            QTimer.singleShot(delay_ms, lambda: show_window(window))
        else:
            show_window(window)
        if log:
            suffix = f" after {delay_ms} ms" if delay_ms > 0 else ""
            state = window_state_summary(window)
            detail = f" ({state})" if state else ""
            log(f"[CTRL] GUI window shown{suffix}{detail}")
        return True
    if cmd in {"hide_window", "hide_gui"}:
        hide_window(window)
        if log:
            log("[CTRL] GUI window hidden")
        return True
    return False


def hide_instead_of_close(window, event, *, log: Callable[[str], None] | None = None) -> bool:
    if not should_hide_on_close():
        return False
    keep_process_alive_when_hidden()
    try:
        event.ignore()
    except Exception:
        pass
    hide_window(window)
    if log:
        log("[CTRL] GUI close intercepted -> hidden; process remains running")
    return True
