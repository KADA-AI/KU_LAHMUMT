# -*- coding: utf-8 -*-
from __future__ import annotations

import atexit
import os
import queue
import sys
import threading
from datetime import datetime
from pathlib import Path


def env_flag(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() in {"1", "true", "yes", "y", "on"}


def should_show_run_console() -> bool:
    return env_flag("KU_SHOW_RUN_CONSOLE", True)


def should_show_module_consoles() -> bool:
    return env_flag("KU_SHOW_MODULE_CONSOLES", True)


def preferred_console_python(executable: str | None = None) -> str:
    exe = Path(executable or sys.executable)
    if os.name == "nt" and exe.name.lower() == "pythonw.exe":
        candidate = exe.with_name("python.exe")
        if candidate.exists():
            return str(candidate)
    return str(exe)


def creationflags_for_subprocess(
    *,
    show_console: bool,
    new_process_group: bool = False,
) -> int:
    if os.name != "nt":
        return 0
    import subprocess

    flags = 0
    if show_console:
        flags |= getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
    else:
        flags |= getattr(subprocess, "CREATE_NO_WINDOW", 0)
    if new_process_group:
        flags |= getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    return flags


def ensure_console(title: str | None = None) -> bool:
    if os.name != "nt":
        return False
    if not should_show_run_console():
        return False
    try:
        import ctypes

        kernel32 = ctypes.windll.kernel32
        if kernel32.GetConsoleWindow():
            if title:
                try:
                    kernel32.SetConsoleTitleW(str(title))
                except Exception:
                    pass
            return True
        if not kernel32.AllocConsole():
            return False
        if title:
            try:
                kernel32.SetConsoleTitleW(str(title))
            except Exception:
                pass
        try:
            sys.stdin = open("CONIN$", "r", encoding="utf-8", errors="replace")
        except Exception:
            pass
        try:
            sys.stdout = open("CONOUT$", "w", encoding="utf-8", buffering=1, errors="replace")
        except Exception:
            pass
        try:
            sys.stderr = open("CONOUT$", "w", encoding="utf-8", buffering=1, errors="replace")
        except Exception:
            pass
        return True
    except Exception:
        return False


def _slugify_module_name(name: str | None) -> str:
    raw = str(name or "process").strip().lower()
    if not raw:
        return "process"
    chars: list[str] = []
    for ch in raw:
        if ch.isalnum():
            chars.append(ch)
        elif ch in (" ", "-", ".", "_"):
            chars.append("_")
    text = "".join(chars).strip("_")
    return text or "process"


class _AsyncProcessFileSink:
    def __init__(self, module_name: str) -> None:
        self.module_name = _slugify_module_name(module_name)
        self._queue: queue.Queue[str | None] = queue.Queue()
        self._thread = threading.Thread(
            target=self._worker,
            name=f"process-log-{self.module_name}",
            daemon=True,
        )
        self._current_path: Path | None = None
        self._handle = None
        self._thread.start()
        atexit.register(self.close)

    def emit(self, text: str) -> None:
        if text is None:
            return
        try:
            self._queue.put_nowait(str(text))
        except Exception:
            pass

    def close(self) -> None:
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass

    def _resolve_path(self) -> Path:
        try:
            from modules.common import db_paths

            return db_paths.get_db_subpath("DSS_Internal", "module_logs", f"{self.module_name}.log")
        except Exception:
            root = Path(os.getenv("KU_MISSION_DB_ROOT") or Path.cwd())
            return root / "DSS_Internal" / "module_logs" / f"{self.module_name}.log"

    def _ensure_handle(self) -> None:
        path = self._resolve_path()
        if self._handle is not None and self._current_path == path:
            return
        self._close_handle()
        path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = path.open("a", encoding="utf-8", buffering=1, errors="replace")
        self._current_path = path
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self._handle.write(f"\n[{stamp}] process-log attached: {self.module_name}\n")

    def _close_handle(self) -> None:
        try:
            if self._handle is not None:
                self._handle.flush()
                self._handle.close()
        except Exception:
            pass
        self._handle = None

    def _worker(self) -> None:
        while True:
            try:
                item = self._queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if item is None:
                break
            batch = [item]
            while True:
                try:
                    nxt = self._queue.get_nowait()
                except queue.Empty:
                    break
                if nxt is None:
                    item = None
                    break
                batch.append(nxt)
            try:
                self._ensure_handle()
                if self._handle is not None:
                    self._handle.write("".join(batch))
                    self._handle.flush()
            except Exception:
                pass
            if item is None:
                break
        self._close_handle()


class _TeeStream:
    def __init__(self, original, sink: _AsyncProcessFileSink) -> None:
        self._original = original
        self._sink = sink

    def write(self, data):
        text = "" if data is None else str(data)
        try:
            written = self._original.write(text)
        except Exception:
            written = len(text)
        if text:
            self._sink.emit(text)
        return written

    def flush(self) -> None:
        try:
            self._original.flush()
        except Exception:
            pass

    def isatty(self):
        try:
            return self._original.isatty()
        except Exception:
            return False

    def fileno(self):
        return self._original.fileno()

    @property
    def encoding(self):
        return getattr(self._original, "encoding", "utf-8")

    def __getattr__(self, name: str):
        return getattr(self._original, name)


_ACTIVE_SINKS: dict[str, _AsyncProcessFileSink] = {}


def install_process_file_logging(module_name: str) -> _AsyncProcessFileSink:
    key = _slugify_module_name(module_name)
    existing = _ACTIVE_SINKS.get(key)
    if existing is not None:
        return existing
    sink = _AsyncProcessFileSink(key)
    if not isinstance(sys.stdout, _TeeStream):
        sys.stdout = _TeeStream(sys.stdout, sink)
    if not isinstance(sys.stderr, _TeeStream):
        sys.stderr = _TeeStream(sys.stderr, sink)
    _ACTIVE_SINKS[key] = sink
    return sink


def emit_process_log(module_name: str, text: str) -> None:
    sink = _ACTIVE_SINKS.get(_slugify_module_name(module_name))
    if sink is None:
        sink = install_process_file_logging(module_name)
    if text:
        sink.emit(f"{text}\n")
