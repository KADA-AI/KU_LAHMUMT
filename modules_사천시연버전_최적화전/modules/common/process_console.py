# -*- coding: utf-8 -*-
from __future__ import annotations

import atexit
import json
import os
import queue
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


PROCESS_LOG_ENQUEUE_P95_BUDGET_MS = 1.0
PROCESS_LOG_REPLAY_P95_OVERHEAD_BUDGET_MS = 100.0
PROCESS_LOG_QUEUE_BACKLOG_RATIO = 0.8

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


def _same_path(left: Path, right: Path) -> bool:
    try:
        return left.resolve() == right.resolve()
    except Exception:
        return left == right


def _coerce_fallback_root(candidate: Path, project_root: Path) -> Path | None:
    if (
        _same_path(candidate, project_root)
        or _same_path(candidate, project_root / "Logs")
    ):
        return None
    if candidate.name.startswith("Scenario_"):
        return candidate / os.getenv("KU_AGENCY_CODE", "SBC3")
    return candidate


def _fallback_db_root() -> Path:
    project_root = Path(__file__).resolve().parents[2]
    raw_env = os.getenv("KU_MISSION_DB_ROOT")
    if raw_env:
        try:
            candidate = _coerce_fallback_root(Path(raw_env), project_root)
            if candidate is not None:
                return candidate
        except Exception:
            pass

    try:
        from modules.common.settings_paths import existing_scenario_info_path

        info_path = existing_scenario_info_path()
    except Exception:
        info_path = project_root / "settings" / "current_scenario.json"
    try:
        data = json.loads(info_path.read_text(encoding="utf-8"))
        raw_db_root = data.get("db_root") if isinstance(data, dict) else None
        if raw_db_root:
            candidate = _coerce_fallback_root(Path(str(raw_db_root)), project_root)
            if candidate is not None:
                return candidate
        raw_scenario_dir = data.get("scenario_dir") if isinstance(data, dict) else None
        if raw_scenario_dir:
            candidate = Path(str(raw_scenario_dir)) / str(data.get("agency") or os.getenv("KU_AGENCY_CODE", "SBC3"))
            return candidate
    except Exception:
        pass

    return project_root / "Logs" / "ProcessFallback" / os.getenv("KU_AGENCY_CODE", "SBC3")


def process_logging_budget() -> dict[str, Any]:
    return {
        "singleLogEnqueueP95BudgetMs": float(PROCESS_LOG_ENQUEUE_P95_BUDGET_MS),
        "replayP95OverheadBudgetMs": float(PROCESS_LOG_REPLAY_P95_OVERHEAD_BUDGET_MS),
        "queueBacklogRatio": float(PROCESS_LOG_QUEUE_BACKLOG_RATIO),
        "hotPathDirectFileWrite": False,
    }


def _format_process_event(
    module_name: str,
    *,
    lifecycle: str,
    component: str | None = None,
    outcome: str | None = None,
    reason: str | None = None,
    extra: dict[str, Any] | None = None,
) -> str:
    payload: dict[str, Any] = {
        "event": "process_lifecycle",
        "module": _slugify_module_name(module_name),
        "processId": int(os.getpid()),
        "threadName": str(threading.current_thread().name),
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "lifecycle": str(lifecycle or "event"),
    }
    if component:
        payload["component"] = str(component)
    if outcome:
        payload["outcome"] = str(outcome)
    if reason:
        payload["reason"] = str(reason)
    if extra:
        payload.update(extra)
    return "[PROCESS][EVENT] " + json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"


class _AsyncProcessFileSink:
    def __init__(self, module_name: str) -> None:
        self.module_name = _slugify_module_name(module_name)
        try:
            maxsize = int(os.getenv("KU_PROCESS_LOG_QUEUE_SIZE", "5000") or "5000")
        except Exception:
            maxsize = 5000
        self._queue_maxsize = max(64, maxsize)
        self._queue: queue.Queue[str | None] = queue.Queue(maxsize=self._queue_maxsize)
        self._dropped = 0
        self._flush_failures = 0
        self._last_drop_report = 0.0
        self._last_failure_report = 0.0
        self._last_backlog_report = 0.0
        self._close_requested = False
        self._stats_lock = threading.Lock()
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
        except queue.Full:
            with self._stats_lock:
                self._dropped += 1
        except Exception:
            with self._stats_lock:
                self._dropped += 1

    def close(self) -> None:
        with self._stats_lock:
            if self._close_requested:
                return
            self._close_requested = True
        self.emit(
            _format_process_event(
                self.module_name,
                lifecycle="process_stop",
                component="process",
                outcome="graceful_shutdown",
                extra=self.stats(),
            )
        )
        try:
            self._queue.put_nowait(None)
        except Exception:
            pass

    def _resolve_path(self) -> Path:
        try:
            from modules.common import db_paths

            return db_paths.get_db_subpath("DSS_Internal", "module_logs", f"{self.module_name}.log")
        except Exception:
            root = _fallback_db_root()
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

    def stats(self) -> dict[str, Any]:
        with self._stats_lock:
            return {
                "module": self.module_name,
                "queued": int(self._queue.qsize()),
                "dropped": int(self._dropped),
                "flushFailures": int(self._flush_failures),
                "queueMaxSize": int(getattr(self, "_queue_maxsize", 0) or 0),
            }

    def _consume_drop_notice(self) -> str:
        with self._stats_lock:
            dropped = int(self._dropped)
            now = time.monotonic()
            if dropped <= 0 or now - self._last_drop_report < 5.0:
                return ""
            self._last_drop_report = now
            self._dropped = 0
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        return f"[{stamp}] process-log dropped messages: module={self.module_name} count={dropped}\n"

    def _consume_backlog_notice(self) -> str:
        try:
            queued = int(self._queue.qsize())
            maxsize = int(getattr(self, "_queue_maxsize", 0) or 0)
        except Exception:
            return ""
        if maxsize <= 0 or queued < max(1, int(maxsize * PROCESS_LOG_QUEUE_BACKLOG_RATIO)):
            return ""
        with self._stats_lock:
            now = time.monotonic()
            if now - self._last_backlog_report < 5.0:
                return ""
            self._last_backlog_report = now
        return _format_process_event(
            self.module_name,
            lifecycle="queue_backlog",
            component="process_log_queue",
            outcome="warning",
            extra={"queued": queued, "queueMaxSize": maxsize},
        )

    def _record_flush_failure(self) -> None:
        with self._stats_lock:
            self._flush_failures += 1
            now = time.monotonic()
            if now - self._last_failure_report >= 5.0:
                self._last_failure_report = now

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
                    drop_notice = self._consume_drop_notice()
                    if drop_notice:
                        self._handle.write(drop_notice)
                    backlog_notice = self._consume_backlog_notice()
                    if backlog_notice:
                        self._handle.write(backlog_notice)
                    self._handle.write("".join(batch))
                    self._handle.flush()
            except Exception:
                self._record_flush_failure()
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
    sink.emit(
        _format_process_event(
            key,
            lifecycle="process_start",
            component="process",
            outcome="ok",
            extra=process_logging_budget(),
        )
    )
    return sink


def emit_process_log(module_name: str, text: str) -> None:
    sink = _ACTIVE_SINKS.get(_slugify_module_name(module_name))
    if sink is None:
        sink = install_process_file_logging(module_name)
    if text:
        sink.emit(f"{text}\n")


def emit_process_lifecycle_event(
    module_name: str,
    lifecycle: str,
    *,
    component: str | None = None,
    outcome: str | None = None,
    reason: str | None = None,
    extra: dict[str, Any] | None = None,
) -> None:
    sink = _ACTIVE_SINKS.get(_slugify_module_name(module_name))
    if sink is None:
        sink = install_process_file_logging(module_name)
    sink.emit(
        _format_process_event(
            module_name,
            lifecycle=lifecycle,
            component=component,
            outcome=outcome,
            reason=reason,
            extra=extra,
        )
    )


def get_process_log_stats(module_name: str | None = None) -> dict[str, Any]:
    if module_name:
        sink = _ACTIVE_SINKS.get(_slugify_module_name(module_name))
        return sink.stats() if sink is not None else {}
    return {key: sink.stats() for key, sink in sorted(_ACTIVE_SINKS.items())}
