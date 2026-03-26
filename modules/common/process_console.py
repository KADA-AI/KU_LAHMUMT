# -*- coding: utf-8 -*-
from __future__ import annotations

import atexit
import json
import os
import queue
import sys
import threading
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
PROCESS_LOG_STATE_PATH = PROJECT_ROOT / "temp" / "process_log_session.json"
PENDING_PROCESS_LOG_ROOT = PROJECT_ROOT / "temp" / "process_log_pending"
ENV_PROCESS_LOG_SESSION_ID = "KU_PROCESS_LOG_SESSION_ID"
ENV_PROCESS_LOG_PHASE = "KU_PROCESS_LOG_PHASE"
_PROCESS_LOG_STATE_LOCK = threading.Lock()


def _normalize_process_log_phase(value: str | None) -> str:
    raw = str(value or "").strip().lower()
    if raw == "active":
        return "active"
    return "pending"


def _default_process_log_session_id() -> str:
    return f"{datetime.now().strftime('%Y%m%dT%H%M%S_%f')}_{os.getpid()}"


def _set_process_log_env(*, session_id: str | None, phase: str) -> None:
    if session_id:
        os.environ[ENV_PROCESS_LOG_SESSION_ID] = str(session_id)
    else:
        os.environ.pop(ENV_PROCESS_LOG_SESSION_ID, None)
    os.environ[ENV_PROCESS_LOG_PHASE] = _normalize_process_log_phase(phase)


def _read_process_log_state_unlocked() -> dict:
    data: dict = {}
    try:
        with PROCESS_LOG_STATE_PATH.open("r", encoding="utf-8") as fh:
            loaded = json.load(fh)
        if isinstance(loaded, dict):
            data = dict(loaded)
    except Exception:
        data = {}
    session_id = str(
        data.get("session_id")
        or os.getenv(ENV_PROCESS_LOG_SESSION_ID)
        or ""
    ).strip()
    if session_id:
        data["session_id"] = session_id
    phase = _normalize_process_log_phase(data.get("phase") or os.getenv(ENV_PROCESS_LOG_PHASE))
    data["phase"] = phase
    return data


def _write_process_log_state_unlocked(data: dict) -> dict:
    payload = dict(data)
    payload["session_id"] = str(payload.get("session_id") or _default_process_log_session_id()).strip()
    payload["phase"] = _normalize_process_log_phase(payload.get("phase"))
    payload["updated_at"] = datetime.now().isoformat(timespec="seconds")
    PROCESS_LOG_STATE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with PROCESS_LOG_STATE_PATH.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2, sort_keys=True)
    _set_process_log_env(session_id=payload["session_id"], phase=payload["phase"])
    return payload


def begin_process_log_session(*, session_id: str | None = None, force_new: bool = True) -> dict:
    with _PROCESS_LOG_STATE_LOCK:
        current = _read_process_log_state_unlocked()
        current_id = str(current.get("session_id") or "").strip()
        target_id = str(session_id or "").strip()
        if not target_id:
            target_id = _default_process_log_session_id() if force_new or not current_id else current_id
        payload = {
            "session_id": target_id,
            "phase": "pending",
            "owner_pid": os.getpid(),
            "pending_flushed": False,
        }
        return _write_process_log_state_unlocked(payload)


def activate_process_log_session(*, db_root: str | os.PathLike | None = None) -> dict:
    with _PROCESS_LOG_STATE_LOCK:
        current = _read_process_log_state_unlocked()
        payload = {
            **current,
            "session_id": str(current.get("session_id") or _default_process_log_session_id()).strip(),
            "phase": "active",
            "owner_pid": os.getpid(),
        }
        if db_root:
            payload["db_root"] = str(db_root)
        return _write_process_log_state_unlocked(payload)


def get_process_log_session() -> dict:
    with _PROCESS_LOG_STATE_LOCK:
        current = _read_process_log_state_unlocked()
        session_id = str(current.get("session_id") or "").strip()
        if session_id:
            _set_process_log_env(session_id=session_id, phase=current.get("phase") or "pending")
        return current


def _pending_process_log_path(module_name: str, session_id: str | None) -> Path:
    safe_session = str(session_id or "default").strip() or "default"
    return PENDING_PROCESS_LOG_ROOT / safe_session / f"{module_name}.log"


def flush_pending_process_logs(*, db_root: str | os.PathLike | None = None) -> int:
    state = get_process_log_session()
    session_id = str(state.get("session_id") or "").strip()
    if not session_id:
        return 0
    pending_dir = _pending_process_log_path("module", session_id).parent
    if not pending_dir.exists():
        return 0
    target_root = Path(str(db_root)).resolve() if db_root else None
    if target_root is None:
        try:
            from modules.common import db_paths

            target_root = Path(db_paths.get_active_db_root())
        except Exception:
            return 0
    dest_dir = target_root / "DSS_Internal" / "module_logs"
    dest_dir.mkdir(parents=True, exist_ok=True)
    copied = 0
    for src in sorted(pending_dir.glob("*.log")):
        try:
            text = src.read_text(encoding="utf-8", errors="replace")
        except Exception:
            continue
        if not text:
            continue
        try:
            with (dest_dir / src.name).open("a", encoding="utf-8", buffering=1, errors="replace") as fh:
                fh.write(text)
            copied += 1
        except Exception:
            continue
    with _PROCESS_LOG_STATE_LOCK:
        current = _read_process_log_state_unlocked()
        if str(current.get("session_id") or "").strip() == session_id:
            current["pending_flushed"] = True
            current["pending_flush_count"] = copied
            _write_process_log_state_unlocked(current)
    return copied


def get_process_log_path(module_name: str) -> Path:
    state = get_process_log_session()
    session_id = str(state.get("session_id") or "").strip()
    if state.get("phase") != "active":
        return _pending_process_log_path(module_name, session_id)
    state_db_root = str(state.get("db_root") or "").strip()
    if state_db_root:
        return Path(state_db_root) / "DSS_Internal" / "module_logs" / f"{module_name}.log"
    try:
        from modules.common import db_paths

        return db_paths.get_db_subpath("DSS_Internal", "module_logs", f"{module_name}.log")
    except Exception:
        root = Path(os.getenv("KU_MISSION_DB_ROOT") or Path.cwd())
        return root / "DSS_Internal" / "module_logs" / f"{module_name}.log"


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
        return get_process_log_path(self.module_name)

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
