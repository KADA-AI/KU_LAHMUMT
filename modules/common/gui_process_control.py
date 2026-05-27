# -*- coding: utf-8 -*-
from __future__ import annotations

import os
from typing import Callable

from PyQt5.QtCore import QTimer, Qt


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


def show_window(window) -> None:
    try:
        window.show()
    except Exception:
        pass
    try:
        if window.windowState() & Qt.WindowMinimized:
            window.setWindowState(window.windowState() & ~Qt.WindowMinimized)
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


def hide_window(window) -> None:
    try:
        window.hide()
    except Exception:
        pass


def apply_initial_visibility(app, window, position_window_from_env: Callable) -> None:
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
            log(f"[CTRL] GUI window shown{suffix}")
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
    try:
        event.ignore()
    except Exception:
        pass
    hide_window(window)
    if log:
        log("[CTRL] GUI close intercepted -> hidden; process remains running")
    return True
