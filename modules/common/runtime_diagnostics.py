# -*- coding: utf-8 -*-
"""Low-overhead, best-effort runtime diagnostics.

The recorder deliberately owns no thread.  ``process_console`` polls it from
the existing asynchronous file-writer thread, so mission/planning call sites
only enqueue a small trigger and never execute hardware probes or file I/O.
"""
from __future__ import annotations

import hashlib
import hmac
import json
import math
import multiprocessing
import os
import platform
import re
import secrets
import struct
import threading
import time
from collections import deque
from datetime import datetime, timezone
from itertools import islice
from pathlib import Path
from typing import Any, Callable


RUNTIME_DIAGNOSTIC_SCHEMA_VERSION = 1
RUNTIME_DIAGNOSTIC_IDLE_INTERVAL_SEC = 15.0
RUNTIME_DIAGNOSTIC_ACTIVE_INTERVAL_SEC = 1.0
RUNTIME_DIAGNOSTIC_TREE_INTERVAL_SEC = 5.0
RUNTIME_DIAGNOSTIC_DETAIL_INTERVAL_SEC = 30.0
RUNTIME_DIAGNOSTIC_MAX_RECORD_BYTES = 4096
RUNTIME_DIAGNOSTIC_MAX_FILE_BYTES = 8 * 1024 * 1024
RUNTIME_DIAGNOSTIC_BACKUP_COUNT = 3
RUNTIME_DIAGNOSTIC_RETENTION_DAYS = 14
RUNTIME_DIAGNOSTIC_MODULE_MAX_BYTES = 256 * 1024 * 1024

_TRIGGER_MIN_INTERVAL_SEC = {
    # These may originate from high-frequency GUI/timer paths. Preserve the
    # first occurrence and a coalesced count without turning an outage into a
    # 5 Hz hardware-probe/write loop.
    "0102_heartbeat_threshold": 5.0,
    "gui_log_write_slow": 5.0,
}

_AUTO = object()
_SENSITIVE_KEY_PARTS = (
    "authorization",
    "cmdline",
    "commandline",
    "cookie",
    "credential",
    "cwd",
    "environment",
    "password",
    "path",
    "secret",
    "token",
)
_CONTEXT_ALLOWED_KEYS = {
    "aircraftid",
    "component",
    "coreworkers",
    "durationms",
    "elapsedms",
    "event",
    "extra",
    "futures",
    "lastsuccessagems",
    "lifecycle",
    "missionplanid",
    "option",
    "outcome",
    "phase",
    "pipeline",
    "reason",
    "replantransactionid",
    "runid",
    "sessionid",
    "sendms",
    "status",
    "storecommitworkers",
    "storeprepareworkers",
    "trigger",
    "triggertype",
    "ticklagms",
    "success",
    "variant",
    "variants",
    "workers",
    "writeelapsedms",
    "writeretry",
}
_URL_PATTERN = re.compile(r"\b[a-z][a-z0-9+.-]*://\S+", re.IGNORECASE)
_WINDOWS_PATH_PATTERN = re.compile(r"(?i)(?:\b[a-z]:[\\/]|\\\\)[^\s,;]+")
_UNIX_PATH_PATTERN = re.compile(r"(?<!\w)/(?:home|Users|var|tmp|etc)/[^\s,;]+")
_IPV4_PATTERN = re.compile(r"(?<![\d.])(?:\d{1,3}\.){3}\d{1,3}(?![\d.])")
_MAC_PATTERN = re.compile(r"(?i)(?<![0-9a-f])(?:[0-9a-f]{2}[:-]){5}[0-9a-f]{2}(?![0-9a-f])")
_IPV6_PATTERN = re.compile(r"(?i)(?<![0-9a-f:])(?:[0-9a-f]{0,4}:){2,7}[0-9a-f]{0,4}(?![0-9a-f:])")
_EMAIL_PATTERN = re.compile(r"(?i)\b[a-z0-9._%+-]+@[a-z0-9.-]+\.[a-z]{2,}\b")
_NAMED_HOST_PATTERN = re.compile(
    r"(?i)\b(host|server|machine|computer|user)\s*[=:]?\s+[a-z0-9._-]+"
)


def _env_flag(name: str, default: bool = False) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() in {"1", "true", "yes", "y", "on"}


def _env_float(name: str, default: float, *, minimum: float, maximum: float) -> float:
    try:
        value = float(os.getenv(name, str(default)) or default)
    except Exception:
        value = float(default)
    if not math.isfinite(value):
        value = float(default)
    return max(float(minimum), min(float(maximum), value))


def _env_int(name: str, default: int, *, minimum: int, maximum: int) -> int:
    try:
        value = int(os.getenv(name, str(default)) or default)
    except Exception:
        value = int(default)
    return max(int(minimum), min(int(maximum), value))


def _slugify(name: str | None) -> str:
    raw = str(name or "process").strip().lower()
    chars = [ch if ch.isalnum() else "_" for ch in raw]
    return "".join(chars).strip("_") or "process"


def _utc_iso(timestamp: float | None = None) -> str:
    return datetime.fromtimestamp(
        time.time() if timestamp is None else float(timestamp),
        tz=timezone.utc,
    ).isoformat()


def _finite_number(value: Any, *, digits: int = 3) -> float | int | None:
    if isinstance(value, bool):
        return int(value)
    try:
        number = float(value)
    except Exception:
        return None
    if not math.isfinite(number):
        return None
    if number.is_integer() and abs(number) < 9_007_199_254_740_992:
        return int(number)
    return round(number, digits)


def _bounded_text(value: Any, limit: int = 160) -> str:
    try:
        text = str(value)
    except Exception:
        return "<unavailable>"
    text = text.replace("\r", " ").replace("\n", " ").strip()
    if len(text) > limit:
        return text[: max(0, limit - 3)] + "..."
    return text


def _looks_like_path(text: str) -> bool:
    value = str(text or "").strip()
    if value.startswith(("\\\\", "/home/", "/Users/", "/var/", "/tmp/")):
        return True
    return len(value) >= 3 and value[1:3] in {":\\", ":/"} and value[0].isalpha()


def _redact_context_text(value: Any) -> str:
    text = _bounded_text(value)
    if _looks_like_path(text):
        return "<path-redacted>"
    text = _URL_PATTERN.sub("<url-redacted>", text)
    text = _WINDOWS_PATH_PATTERN.sub("<path-redacted>", text)
    text = _UNIX_PATH_PATTERN.sub("<path-redacted>", text)
    text = _IPV4_PATTERN.sub("<ip-redacted>", text)
    text = _IPV6_PATTERN.sub("<ip-redacted>", text)
    text = _MAC_PATTERN.sub("<mac-redacted>", text)
    text = _EMAIL_PATTERN.sub("<email-redacted>", text)
    text = _NAMED_HOST_PATTERN.sub(lambda match: f"{match.group(1)}=<name-redacted>", text)
    return text


def _sanitize_context(value: Any, *, depth: int = 0) -> Any:
    """Return a small privacy-allowlisted JSON value for trigger metadata."""
    if depth > 2:
        return "<depth-limit>"
    if value is None or isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return _finite_number(value)
    if isinstance(value, str):
        return _redact_context_text(value)
    if isinstance(value, dict):
        safe: dict[str, Any] = {}
        for raw_key, raw_value in islice(value.items(), 16):
            key = _bounded_text(raw_key, 48)
            lower = key.replace("_", "").lower()
            if lower not in _CONTEXT_ALLOWED_KEYS:
                continue
            if any(part in lower for part in _SENSITIVE_KEY_PARTS):
                continue
            safe[key] = _sanitize_context(raw_value, depth=depth + 1)
        return safe
    if isinstance(value, (list, tuple, set)):
        return [_sanitize_context(item, depth=depth + 1) for item in islice(value, 16)]
    return _bounded_text(type(value).__name__, 64)


def _load_psutil() -> Any | None:
    try:
        import psutil  # type: ignore

        return psutil
    except Exception:
        return None


def _is_main_process() -> bool:
    try:
        return str(multiprocessing.current_process().name) == "MainProcess"
    except Exception:
        return True


def _processor_model() -> str | None:
    if os.name == "nt":
        try:
            import winreg

            with winreg.OpenKey(
                winreg.HKEY_LOCAL_MACHINE,
                r"HARDWARE\DESCRIPTION\System\CentralProcessor\0",
            ) as key:
                value, _ = winreg.QueryValueEx(key, "ProcessorNameString")
                if str(value or "").strip():
                    return _bounded_text(value, 160)
        except Exception:
            pass
    for candidate in (platform.processor(), os.getenv("PROCESSOR_IDENTIFIER", "")):
        if str(candidate or "").strip():
            return _bounded_text(candidate, 160)
    return None


class _WindowsFallback:
    """Cheap Windows metrics used when psutil is absent or partially fails."""

    def __init__(self) -> None:
        self.available = False
        self._last_system_times: tuple[int, int, int] | None = None
        if os.name != "nt":
            return
        try:
            import ctypes
            from ctypes import wintypes

            self.ctypes = ctypes
            self.wintypes = wintypes
            self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            self.psapi = ctypes.WinDLL("psapi", use_last_error=True)
            self.kernel32.GetCurrentProcess.argtypes = []
            self.kernel32.GetCurrentProcess.restype = wintypes.HANDLE
            self.kernel32.GetSystemTimes.argtypes = [
                ctypes.POINTER(wintypes.FILETIME),
                ctypes.POINTER(wintypes.FILETIME),
                ctypes.POINTER(wintypes.FILETIME),
            ]
            self.kernel32.GetSystemTimes.restype = wintypes.BOOL
            self.kernel32.GlobalMemoryStatusEx.argtypes = [ctypes.c_void_p]
            self.kernel32.GlobalMemoryStatusEx.restype = wintypes.BOOL
            self.psapi.GetProcessMemoryInfo.argtypes = [
                wintypes.HANDLE,
                ctypes.c_void_p,
                wintypes.DWORD,
            ]
            self.psapi.GetProcessMemoryInfo.restype = wintypes.BOOL
            self.kernel32.GetProcessIoCounters.argtypes = [wintypes.HANDLE, ctypes.c_void_p]
            self.kernel32.GetProcessIoCounters.restype = wintypes.BOOL
            self.kernel32.GetProcessAffinityMask.argtypes = [
                wintypes.HANDLE,
                ctypes.POINTER(ctypes.c_size_t),
                ctypes.POINTER(ctypes.c_size_t),
            ]
            self.kernel32.GetProcessAffinityMask.restype = wintypes.BOOL
            self.kernel32.GetProcessHandleCount.argtypes = [
                wintypes.HANDLE,
                ctypes.POINTER(wintypes.DWORD),
            ]
            self.kernel32.GetProcessHandleCount.restype = wintypes.BOOL
            self.process_handle = self.kernel32.GetCurrentProcess()
            self.available = True
        except Exception:
            self.available = False

    @staticmethod
    def _filetime_value(filetime: Any) -> int:
        return (int(filetime.dwHighDateTime) << 32) | int(filetime.dwLowDateTime)

    def _system_times(self) -> tuple[int, int, int] | None:
        if not self.available:
            return None
        try:
            idle = self.wintypes.FILETIME()
            kernel = self.wintypes.FILETIME()
            user = self.wintypes.FILETIME()
            if not self.kernel32.GetSystemTimes(
                self.ctypes.byref(idle),
                self.ctypes.byref(kernel),
                self.ctypes.byref(user),
            ):
                return None
            return (
                self._filetime_value(idle),
                self._filetime_value(kernel),
                self._filetime_value(user),
            )
        except Exception:
            return None

    def prime(self) -> None:
        self._last_system_times = self._system_times()

    def system_cpu_percent(self) -> float | None:
        current = self._system_times()
        previous = self._last_system_times
        self._last_system_times = current
        if current is None or previous is None:
            return None
        idle_delta = current[0] - previous[0]
        total_delta = (current[1] - previous[1]) + (current[2] - previous[2])
        if total_delta <= 0:
            return None
        return max(0.0, min(100.0, (total_delta - idle_delta) * 100.0 / total_delta))

    def memory(self) -> dict[str, int | float | None]:
        if not self.available:
            return {}
        try:
            class MEMORYSTATUSEX(self.ctypes.Structure):
                _fields_ = [
                    ("dwLength", self.wintypes.DWORD),
                    ("dwMemoryLoad", self.wintypes.DWORD),
                    ("ullTotalPhys", self.ctypes.c_ulonglong),
                    ("ullAvailPhys", self.ctypes.c_ulonglong),
                    ("ullTotalPageFile", self.ctypes.c_ulonglong),
                    ("ullAvailPageFile", self.ctypes.c_ulonglong),
                    ("ullTotalVirtual", self.ctypes.c_ulonglong),
                    ("ullAvailVirtual", self.ctypes.c_ulonglong),
                    ("ullAvailExtendedVirtual", self.ctypes.c_ulonglong),
                ]

            status = MEMORYSTATUSEX()
            status.dwLength = self.ctypes.sizeof(MEMORYSTATUSEX)
            if not self.kernel32.GlobalMemoryStatusEx(self.ctypes.byref(status)):
                return {}
            return {
                "systemMemoryTotalBytes": int(status.ullTotalPhys),
                "systemMemoryAvailableBytes": int(status.ullAvailPhys),
                "systemMemoryPercent": float(status.dwMemoryLoad),
                # Windows reports commit limit/available here (RAM included), not pagefile size.
                "systemCommitLimitBytes": int(status.ullTotalPageFile),
                "systemCommitAvailableBytes": int(status.ullAvailPageFile),
            }
        except Exception:
            return {}

    def process_memory(self) -> dict[str, int]:
        if not self.available:
            return {}
        try:
            class PROCESS_MEMORY_COUNTERS_EX(self.ctypes.Structure):
                _fields_ = [
                    ("cb", self.wintypes.DWORD),
                    ("PageFaultCount", self.wintypes.DWORD),
                    ("PeakWorkingSetSize", self.ctypes.c_size_t),
                    ("WorkingSetSize", self.ctypes.c_size_t),
                    ("QuotaPeakPagedPoolUsage", self.ctypes.c_size_t),
                    ("QuotaPagedPoolUsage", self.ctypes.c_size_t),
                    ("QuotaPeakNonPagedPoolUsage", self.ctypes.c_size_t),
                    ("QuotaNonPagedPoolUsage", self.ctypes.c_size_t),
                    ("PagefileUsage", self.ctypes.c_size_t),
                    ("PeakPagefileUsage", self.ctypes.c_size_t),
                    ("PrivateUsage", self.ctypes.c_size_t),
                ]

            counters = PROCESS_MEMORY_COUNTERS_EX()
            counters.cb = self.ctypes.sizeof(PROCESS_MEMORY_COUNTERS_EX)
            if not self.psapi.GetProcessMemoryInfo(
                self.process_handle,
                self.ctypes.byref(counters),
                counters.cb,
            ):
                return {}
            return {
                "processRssBytes": int(counters.WorkingSetSize),
                "processVmsBytes": int(counters.PrivateUsage),
                "processPeakRssBytes": int(counters.PeakWorkingSetSize),
            }
        except Exception:
            return {}

    def process_io(self) -> dict[str, int]:
        if not self.available:
            return {}
        try:
            class IO_COUNTERS(self.ctypes.Structure):
                _fields_ = [
                    ("ReadOperationCount", self.ctypes.c_ulonglong),
                    ("WriteOperationCount", self.ctypes.c_ulonglong),
                    ("OtherOperationCount", self.ctypes.c_ulonglong),
                    ("ReadTransferCount", self.ctypes.c_ulonglong),
                    ("WriteTransferCount", self.ctypes.c_ulonglong),
                    ("OtherTransferCount", self.ctypes.c_ulonglong),
                ]

            counters = IO_COUNTERS()
            if not self.kernel32.GetProcessIoCounters(
                self.process_handle,
                self.ctypes.byref(counters),
            ):
                return {}
            return {
                "processReadBytes": int(counters.ReadTransferCount),
                "processWriteBytes": int(counters.WriteTransferCount),
            }
        except Exception:
            return {}

    def affinity_count(self) -> int | None:
        if not self.available:
            return None
        try:
            process_mask = self.ctypes.c_size_t()
            system_mask = self.ctypes.c_size_t()
            if not self.kernel32.GetProcessAffinityMask(
                self.process_handle,
                self.ctypes.byref(process_mask),
                self.ctypes.byref(system_mask),
            ):
                return None
            return int(int(process_mask.value).bit_count()) or None
        except Exception:
            return None

    def handle_count(self) -> int | None:
        if not self.available:
            return None
        try:
            count = self.wintypes.DWORD()
            if self.kernel32.GetProcessHandleCount(self.process_handle, self.ctypes.byref(count)):
                return int(count.value)
        except Exception:
            pass
        return None


class RuntimeDiagnosticsRecorder:
    """Collect and persist bounded JSONL diagnostics from an async worker."""

    def __init__(
        self,
        module_name: str,
        *,
        psutil_module: Any = _AUTO,
        force_enabled: bool | None = None,
        path_resolver: Callable[[str], Path] | None = None,
        monotonic: Callable[[], float] | None = None,
        wall_clock: Callable[[], float] | None = None,
    ) -> None:
        self.module_name = _slugify(module_name)
        self.process_id = int(os.getpid())
        self._owner_pid = self.process_id
        self._monotonic = monotonic or time.monotonic
        self._wall_clock = wall_clock or time.time
        enabled_by_env = _env_flag("KU_RUNTIME_DIAGNOSTICS", True)
        main_process_ok = _is_main_process() or _env_flag("KU_RUNTIME_DIAGNOSTICS_WORKERS", False)
        self.enabled = bool(
            force_enabled if force_enabled is not None else enabled_by_env and main_process_ok
        )
        if not self.enabled:
            # A real opt-out: no psutil import, DLL load, registry access, or HW probe.
            self._psutil = None
            self._backend = "disabled"
            self._process = None
            self._lock = threading.Lock()
            self._triggers = deque(maxlen=16)
            self._dropped_triggers = 0
            self._trigger_last_accepted = {}
            self._trigger_coalesced_pending = {}
            self._coalesced_triggers = 0
            self._active = False
            self._handle = None
            self._samples = 0
            self._write_failures = 0
            self._collection_failures = 0
            self._rotation_failures = 0
            self._truncated_records = 0
            self._capability_failures = {}
            self._retention_deleted_files = 0
            self._retention_failures = 0
            return
        self._capability_failures: dict[str, int] = {}
        self._psutil = _load_psutil() if psutil_module is _AUTO else psutil_module
        self._backend = "psutil" if self._psutil is not None else (
            "windows_ctypes" if os.name == "nt" else "stdlib"
        )
        if self._psutil is None:
            for capability in (
                "process_tree",
                "process_thread_count",
                "process_context_switches",
            ):
                self._mark_capability_failure(capability)
        self._fallback = _WindowsFallback()
        self._process = None
        if self.enabled and self._psutil is not None:
            try:
                self._process = self._psutil.Process(self.process_id)
            except Exception:
                self._mark_capability_failure("process_handle")
                self._process = None

        self._idle_interval = _env_float(
            "KU_RUNTIME_DIAGNOSTICS_IDLE_SEC",
            RUNTIME_DIAGNOSTIC_IDLE_INTERVAL_SEC,
            minimum=5.0,
            maximum=300.0,
        )
        self._active_interval = _env_float(
            "KU_RUNTIME_DIAGNOSTICS_ACTIVE_SEC",
            RUNTIME_DIAGNOSTIC_ACTIVE_INTERVAL_SEC,
            minimum=0.5,
            maximum=30.0,
        )
        self._tree_interval = _env_float(
            "KU_RUNTIME_DIAGNOSTICS_TREE_SEC",
            RUNTIME_DIAGNOSTIC_TREE_INTERVAL_SEC,
            minimum=2.0,
            maximum=120.0,
        )
        self._detail_interval = _env_float(
            "KU_RUNTIME_DIAGNOSTICS_DETAIL_SEC",
            RUNTIME_DIAGNOSTIC_DETAIL_INTERVAL_SEC,
            minimum=10.0,
            maximum=600.0,
        )
        self._active_timeout = _env_float(
            "KU_RUNTIME_DIAGNOSTICS_ACTIVE_TIMEOUT_SEC",
            1800.0,
            minimum=30.0,
            maximum=7200.0,
        )
        self._max_record_bytes = _env_int(
            "KU_RUNTIME_DIAGNOSTICS_RECORD_BYTES",
            RUNTIME_DIAGNOSTIC_MAX_RECORD_BYTES,
            minimum=1024,
            maximum=65536,
        )
        self._max_file_bytes = _env_int(
            "KU_RUNTIME_DIAGNOSTICS_MAX_FILE_BYTES",
            RUNTIME_DIAGNOSTIC_MAX_FILE_BYTES,
            minimum=256 * 1024,
            maximum=512 * 1024 * 1024,
        )
        self._backup_count = _env_int(
            "KU_RUNTIME_DIAGNOSTICS_BACKUPS",
            RUNTIME_DIAGNOSTIC_BACKUP_COUNT,
            # Keep the per-file size limit effective even when misconfigured.
            minimum=1,
            maximum=10,
        )
        self._retention_days = _env_int(
            "KU_RUNTIME_DIAGNOSTICS_RETENTION_DAYS",
            RUNTIME_DIAGNOSTIC_RETENTION_DAYS,
            minimum=1,
            maximum=3650,
        )
        self._module_max_bytes = _env_int(
            "KU_RUNTIME_DIAGNOSTICS_MODULE_MAX_BYTES",
            RUNTIME_DIAGNOSTIC_MODULE_MAX_BYTES,
            minimum=32 * 1024 * 1024,
            maximum=10 * 1024 * 1024 * 1024,
        )

        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self._start_token = stamp
        filename = f"{self.module_name}_{self.process_id}_{stamp}.jsonl"
        self._path_resolver = path_resolver or self._default_path_resolver
        self._filename = filename

        self._lock = threading.Lock()
        self._triggers: deque[dict[str, Any]] = deque(maxlen=16)
        self._dropped_triggers = 0
        self._trigger_last_accepted: dict[str, float] = {}
        self._trigger_coalesced_pending: dict[str, int] = {}
        self._coalesced_triggers = 0
        now = self._monotonic()
        self._next_due = now
        self._force_due = True
        self._active = False
        self._active_deadline = 0.0
        self._last_sample_mono: float | None = None
        self._last_process_cpu_time = self._process_cpu_time()
        self._last_io: tuple[int | None, int | None] = (None, None)
        self._last_tree_mono: float | None = None
        self._last_tree_parent_cpu_time: float | None = None
        self._last_detail_mono: float | None = None
        self._child_cpu_totals: dict[int, float] = {}
        self._tree_cache: dict[str, Any] = {}
        self._detail_cache: dict[str, Any] = {}
        self._system_high_since: float | None = None
        self._tree_high_since: float | None = None

        self._handle = None
        self._current_path: Path | None = None
        self._current_bytes = 0
        self._inventory_written_for_path: set[str] = set()
        self._retention_checked_dirs: set[str] = set()
        self._rotation_retry_after = 0.0
        self._last_write_ms: float | None = None
        self._samples = 0
        self._write_failures = 0
        self._collection_failures = 0
        self._rotation_failures = 0
        self._truncated_records = 0
        self._retention_deleted_files = 0
        self._retention_failures = 0
        self._static_inventory = self._build_inventory()
        self._prime_cpu_counters()
        self._static_inventory["collectorBackend"] = self._effective_backend()
        self._static_inventory["missingCapabilities"] = sorted(self._capability_failures)[:16]

    def _mark_capability_failure(self, name: str) -> None:
        key = _bounded_text(name or "unknown", 48)
        self._capability_failures[key] = int(self._capability_failures.get(key, 0)) + 1

    def _effective_backend(self) -> str:
        if self._backend == "psutil" and self._capability_failures:
            return "psutil_partial"
        return self._backend

    def _default_path_resolver(self, filename: str) -> Path:
        try:
            from modules.common import db_paths

            return db_paths.get_db_subpath("DSS_Internal", "runtime_diagnostics", filename)
        except Exception:
            project_root = Path(__file__).resolve().parents[2]
            agency = os.getenv("KU_AGENCY_CODE", "SBC3")
            return project_root / "Logs" / "ProcessFallback" / agency / "DSS_Internal" / "runtime_diagnostics" / filename

    def _prime_cpu_counters(self) -> None:
        if not self.enabled:
            return
        if self._psutil is not None:
            try:
                self._psutil.cpu_percent(interval=None, percpu=True)
            except Exception:
                self._mark_capability_failure("system_cpu")
            try:
                if self._process is not None:
                    self._process.cpu_percent(interval=None)
            except Exception:
                self._mark_capability_failure("process_cpu")
        try:
            self._fallback.prime()
        except Exception:
            pass

    def _process_cpu_time(self) -> float | None:
        if self._process is not None:
            try:
                values = self._process.cpu_times()
                return float(values.user) + float(values.system)
            except Exception:
                self._mark_capability_failure("process_cpu_times")
        try:
            return float(time.process_time())
        except Exception:
            return None

    def _host_id(self) -> str:
        try:
            source = platform.node().strip()
        except Exception:
            source = ""
        if not source:
            source = f"{platform.system()}|{platform.machine()}|{os.cpu_count()}"
        configured_salt = os.getenv("KU_RUNTIME_DIAGNOSTICS_HOST_SALT")
        if not configured_salt:
            return secrets.token_hex(8)
        return hmac.new(
            configured_salt.encode("utf-8", errors="replace"),
            source.encode("utf-8", errors="replace"),
            hashlib.sha256,
        ).hexdigest()[:16]

    def _build_inventory(self) -> dict[str, Any]:
        logical = int(os.cpu_count() or 0) or None
        physical = None
        affinity = None
        memory_total = None
        swap_total = None
        priority = None
        process_create_time = None
        cpu_frequency: dict[str, Any] | None = None
        psutil_version = None
        if self._psutil is not None:
            psutil_version = _bounded_text(getattr(self._psutil, "__version__", "unknown"), 32)
            try:
                logical = self._psutil.cpu_count(logical=True) or logical
            except Exception:
                self._mark_capability_failure("cpu_logical_count")
            try:
                physical = self._psutil.cpu_count(logical=False)
            except Exception:
                self._mark_capability_failure("cpu_physical_count")
            try:
                vm = self._psutil.virtual_memory()
                memory_total = int(vm.total)
            except Exception:
                self._mark_capability_failure("system_memory")
            try:
                swap_total = int(self._psutil.swap_memory().total)
            except Exception:
                self._mark_capability_failure("system_swap")
            try:
                freq = self._psutil.cpu_freq()
                if freq is not None:
                    cpu_frequency = {
                        "currentMhz": _finite_number(getattr(freq, "current", None)),
                        "minMhz": _finite_number(getattr(freq, "min", None)),
                        "maxMhz": _finite_number(getattr(freq, "max", None)),
                    }
            except Exception:
                self._mark_capability_failure("cpu_frequency")
            if self._process is not None:
                try:
                    affinity = len(self._process.cpu_affinity())
                except Exception:
                    self._mark_capability_failure("process_affinity")
                try:
                    priority = _bounded_text(self._process.nice(), 48)
                except Exception:
                    self._mark_capability_failure("process_priority")
                try:
                    process_create_time = _utc_iso(float(self._process.create_time()))
                except Exception:
                    self._mark_capability_failure("process_create_time")
        if affinity is None:
            affinity = self._fallback.affinity_count()
        if memory_total is None:
            memory_total = self._fallback.memory().get("systemMemoryTotalBytes")
        for capability, value in (
            ("cpu_physical_count", physical),
            ("process_affinity", affinity),
            ("system_memory", memory_total),
            ("system_swap", swap_total),
            ("cpu_frequency", cpu_frequency),
            ("process_priority", priority),
            ("process_create_time", process_create_time),
        ):
            if value is None:
                self._mark_capability_failure(capability)

        thread_limits: dict[str, str] = {}
        for name in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
            value = os.getenv(name)
            if value is not None:
                thread_limits[name] = _bounded_text(value, 24)

        return {
            "schemaVersion": RUNTIME_DIAGNOSTIC_SCHEMA_VERSION,
            "event": "runtime_inventory",
            "timestamp": _utc_iso(self._wall_clock()),
            "module": self.module_name,
            "processId": self.process_id,
            "processStartToken": self._start_token,
            "processCreateTime": process_create_time,
            "hostId": self._host_id(),
            "hostIdScope": (
                "configured_salt"
                if os.getenv("KU_RUNTIME_DIAGNOSTICS_HOST_SALT")
                else "process_session_random"
            ),
            "collectorBackend": self._effective_backend(),
            "psutilVersion": psutil_version,
            "osFamily": _bounded_text(platform.system(), 48),
            "osRelease": _bounded_text(platform.release(), 64),
            "architecture": _bounded_text(platform.machine(), 64),
            "pythonVersion": _bounded_text(platform.python_version(), 32),
            "pythonBits": int(struct.calcsize("P") * 8),
            "cpuModel": _processor_model(),
            "cpuPhysicalCount": _finite_number(physical),
            "cpuLogicalCount": _finite_number(logical),
            "effectiveCpuAffinityCount": _finite_number(affinity),
            "cpuFrequency": cpu_frequency,
            "memoryTotalBytes": _finite_number(memory_total),
            "swapTotalBytes": _finite_number(swap_total),
            "processPriority": priority,
            "threadRuntimeLimits": thread_limits,
            "missingCapabilities": sorted(self._capability_failures)[:16],
            "sampling": {
                "idleIntervalSec": self._idle_interval,
                "activeIntervalSec": self._active_interval,
                "treeIntervalSec": self._tree_interval,
                "detailIntervalSec": self._detail_interval,
            },
            "retention": {
                "days": self._retention_days,
                "moduleMaxBytes": self._module_max_bytes,
                "fileMaxBytes": self._max_file_bytes,
                "backupCount": self._backup_count,
            },
            "privacy": {
                "rawHostname": False,
                "username": False,
                "networkIdentifiers": False,
                "commandLine": False,
            },
        }

    def request_snapshot(
        self,
        trigger: str,
        *,
        context: dict[str, Any] | None = None,
        active: bool | None = None,
    ) -> None:
        """Schedule a boundary snapshot; never performs collection or I/O."""
        if not self.enabled or os.getpid() != self._owner_pid:
            return
        now = self._monotonic()
        normalized_trigger = _bounded_text(trigger or "event", 64)
        coalesced_count = 0
        minimum_interval = float(_TRIGGER_MIN_INTERVAL_SEC.get(normalized_trigger, 0.0))
        if minimum_interval > 0.0:
            with self._lock:
                previous = self._trigger_last_accepted.get(normalized_trigger)
                if previous is not None and now - previous < minimum_interval:
                    self._trigger_coalesced_pending[normalized_trigger] = (
                        int(self._trigger_coalesced_pending.get(normalized_trigger, 0)) + 1
                    )
                    self._coalesced_triggers += 1
                    return
                self._trigger_last_accepted[normalized_trigger] = now
                coalesced_count = int(self._trigger_coalesced_pending.pop(normalized_trigger, 0))
        item = {
            "trigger": normalized_trigger,
            "timestamp": _utc_iso(self._wall_clock()),
            "_requestedMonotonic": now,
        }
        if coalesced_count > 0:
            item["coalescedCount"] = coalesced_count
        safe_context = _sanitize_context(context or {})
        if safe_context:
            item["context"] = safe_context
        with self._lock:
            if self._triggers.maxlen is not None and len(self._triggers) >= self._triggers.maxlen:
                self._dropped_triggers += 1
            self._triggers.append(item)
            self._force_due = True
            if active is True:
                self._active = True
                self._active_deadline = now + self._active_timeout
            elif active is False:
                self._active = False
                self._active_deadline = 0.0

    def _psutil_system_snapshot(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        if self._psutil is None:
            return result
        try:
            per_cpu = self._psutil.cpu_percent(interval=None, percpu=True)
            values = [float(value) for value in list(per_cpu or []) if _finite_number(value) is not None]
            if values:
                result["perLogicalCpuPercent"] = [round(value, 1) for value in values]
                result["systemCpuPercent"] = round(sum(values) / len(values), 2)
                result["busyLogicalCpuCount"] = sum(1 for value in values if value >= 75.0)
        except Exception:
            self._mark_capability_failure("system_cpu")
        try:
            vm = self._psutil.virtual_memory()
            result.update(
                {
                    "systemMemoryTotalBytes": int(vm.total),
                    "systemMemoryAvailableBytes": int(vm.available),
                    "systemMemoryPercent": round(float(vm.percent), 2),
                }
            )
        except Exception:
            self._mark_capability_failure("system_memory")
        try:
            swap = self._psutil.swap_memory()
            result.update(
                {
                    "systemSwapTotalBytes": int(swap.total),
                    "systemSwapUsedBytes": int(swap.used),
                    "systemSwapPercent": round(float(swap.percent), 2),
                }
            )
        except Exception:
            self._mark_capability_failure("system_swap")
        return result

    def _process_fast_snapshot(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        if self._process is not None:
            try:
                memory = self._process.memory_info()
                result["processRssBytes"] = int(memory.rss)
                result["processVmsBytes"] = int(memory.vms)
            except Exception:
                self._mark_capability_failure("process_memory")
            try:
                io = self._process.io_counters()
                result["processReadBytes"] = int(io.read_bytes)
                result["processWriteBytes"] = int(io.write_bytes)
            except Exception:
                self._mark_capability_failure("process_io")
        if "processRssBytes" not in result:
            result.update(self._fallback.process_memory())
        if "processReadBytes" not in result:
            result.update(self._fallback.process_io())
        read_now = result.get("processReadBytes")
        write_now = result.get("processWriteBytes")
        read_prev, write_prev = self._last_io
        if isinstance(read_now, int) and isinstance(read_prev, int):
            result["processReadDeltaBytes"] = max(0, read_now - read_prev)
        if isinstance(write_now, int) and isinstance(write_prev, int):
            result["processWriteDeltaBytes"] = max(0, write_now - write_prev)
        self._last_io = (
            read_now if isinstance(read_now, int) else read_prev,
            write_now if isinstance(write_now, int) else write_prev,
        )
        return result

    def _refresh_tree_snapshot(
        self,
        now: float,
        parent_rss: int | None,
        parent_cpu_time: float | None,
        capacity: float | int | None,
    ) -> bool:
        if self._process is None:
            return False
        if self._last_tree_mono is not None and now - self._last_tree_mono < self._tree_interval:
            return False
        started = time.perf_counter()
        previous_mono = self._last_tree_mono
        current_cpu: dict[int, float] = {}
        child_rss = 0
        child_vms = 0
        active_children = 0
        child_count = 0
        truncated = False
        child_cpu_delta = 0.0
        try:
            children = list(self._process.children(recursive=False))
        except Exception:
            children = []
        if len(children) > 64:
            children = children[:64]
            truncated = True
        for child in children:
            try:
                times = child.cpu_times()
                total = float(times.user) + float(times.system)
                current_cpu[int(child.pid)] = total
                previous = self._child_cpu_totals.get(int(child.pid))
                if previous is not None and total >= previous:
                    delta = total - previous
                    child_cpu_delta += delta
                    if delta > 0.01:
                        active_children += 1
            except Exception:
                pass
            try:
                memory = child.memory_info()
                child_rss += int(memory.rss)
                child_vms += int(memory.vms)
            except Exception:
                pass
            child_count += 1
        elapsed = None if previous_mono is None else max(0.000001, now - previous_mono)
        parent_cpu_delta = None
        if (
            parent_cpu_time is not None
            and self._last_tree_parent_cpu_time is not None
            and parent_cpu_time >= self._last_tree_parent_cpu_time
        ):
            parent_cpu_delta = parent_cpu_time - self._last_tree_parent_cpu_time
        child_raw_cpu = None if elapsed is None else child_cpu_delta * 100.0 / elapsed
        parent_raw_cpu = (
            None
            if elapsed is None or parent_cpu_delta is None
            else parent_cpu_delta * 100.0 / elapsed
        )
        tree_raw_cpu = (
            None
            if child_raw_cpu is None or parent_raw_cpu is None
            else max(0.0, child_raw_cpu + parent_raw_cpu)
        )
        self._child_cpu_totals = current_cpu
        self._last_tree_mono = now
        self._last_tree_parent_cpu_time = parent_cpu_time
        self._tree_cache = {
            "childProcessCount": child_count,
            "activeChildProcessCount": active_children,
            "childProcessListTruncated": truncated,
            "childProcessRssBytes": child_rss,
            "childProcessVmsBytes": child_vms,
            "childCpuPercentRaw": _finite_number(child_raw_cpu),
            "processTreeParentCpuPercentRaw": _finite_number(parent_raw_cpu),
            "processTreeCpuPercentRaw": _finite_number(tree_raw_cpu),
            "processTreeCpuCoreEquivalent": _finite_number(
                None if tree_raw_cpu is None else tree_raw_cpu / 100.0
            ),
            "processTreeCpuPercentOfAffinity": _finite_number(
                None
                if tree_raw_cpu is None or not isinstance(capacity, (int, float)) or capacity <= 0
                else tree_raw_cpu / float(capacity)
            ),
            "processTreeRssBytes": (int(parent_rss or 0) + child_rss) if parent_rss is not None else None,
            "treeScanMs": round((time.perf_counter() - started) * 1000.0, 3),
        }
        return True

    def _refresh_detail_snapshot(self, now: float, *, force: bool) -> None:
        minimum = 5.0 if force else self._detail_interval
        if self._last_detail_mono is not None and now - self._last_detail_mono < minimum:
            return
        detail: dict[str, Any] = {}
        if self._process is not None:
            try:
                detail["processThreadCount"] = int(self._process.num_threads())
            except Exception:
                pass
            try:
                detail["processHandleCount"] = int(self._process.num_handles())
            except Exception:
                pass
            try:
                switches = self._process.num_ctx_switches()
                detail["processVoluntaryContextSwitches"] = int(switches.voluntary)
                detail["processInvoluntaryContextSwitches"] = int(switches.involuntary)
            except Exception:
                pass
        if "processHandleCount" not in detail:
            handle_count = self._fallback.handle_count()
            if handle_count is not None:
                detail["processHandleCount"] = handle_count
        self._detail_cache = detail
        self._last_detail_mono = now

    def _warning_codes(self, record: dict[str, Any]) -> list[str]:
        warnings: list[str] = []
        if record.get("missingCapabilities"):
            warnings.append("diagnostic_capability_partial")
        if int(record.get("diagnosticCollectionFailures") or 0) > 0:
            warnings.append("diagnostic_collection_failure_seen")
        if int(record.get("diagnosticWriteFailures") or 0) > 0:
            warnings.append("diagnostic_write_failure_seen")
        if int(record.get("diagnosticRotationFailures") or 0) > 0:
            warnings.append("diagnostic_rotation_failure_seen")
        if int(record.get("diagnosticRetentionFailures") or 0) > 0:
            warnings.append("diagnostic_retention_failure_seen")
        if int(record.get("logDroppedMessagesTotal") or 0) > 0:
            warnings.append("process_log_messages_dropped")
        if int(record.get("logFlushFailures") or 0) > 0:
            warnings.append("process_log_flush_failure_seen")
        lag = float(record.get("sampleLagMs") or 0.0)
        if lag >= 1000.0:
            warnings.append("diagnostic_sampler_lag_critical")
        elif lag >= 250.0:
            warnings.append("diagnostic_sampler_lag")
        trigger_lag = float(record.get("triggerToSampleMs") or 0.0)
        # A boundary just after a sample is intentionally coalesced for 200 ms,
        # then may wait for the writer's 500 ms idle poll. Do not report that
        # normal bounded delay as contention.
        if trigger_lag >= 2000.0:
            warnings.append("diagnostic_trigger_delay_critical")
        elif trigger_lag >= 750.0:
            warnings.append("diagnostic_trigger_delay")

        system_cpu = record.get("systemCpuPercent")
        warning_now = self._monotonic()
        if isinstance(system_cpu, (int, float)) and system_cpu >= 90.0:
            if self._system_high_since is None:
                self._system_high_since = warning_now
        else:
            self._system_high_since = None
        if self._system_high_since is not None and warning_now - self._system_high_since >= 3.0:
            warnings.append("system_cpu_sustained_high")

        memory_percent = record.get("systemMemoryPercent")
        memory_available = record.get("systemMemoryAvailableBytes")
        if (
            isinstance(memory_percent, (int, float)) and memory_percent >= 95.0
        ) or (
            isinstance(memory_available, int) and memory_available < 512 * 1024 * 1024
        ):
            warnings.append("system_memory_critical")
        elif (
            isinstance(memory_percent, (int, float)) and memory_percent >= 90.0
        ) or (
            isinstance(memory_available, int) and memory_available < 1024 * 1024 * 1024
        ):
            warnings.append("system_memory_low")

        tree_cpu = record.get("processTreeCpuPercentOfAffinity")
        if record.get("processTreeSampleFresh"):
            if isinstance(tree_cpu, (int, float)) and tree_cpu >= 85.0:
                if self._tree_high_since is None:
                    self._tree_high_since = warning_now
            else:
                self._tree_high_since = None
            if self._tree_high_since is not None and warning_now - self._tree_high_since >= 10.0:
                warnings.append("process_tree_cpu_capacity_high")

        queue_size = record.get("logQueueDepth")
        queue_max = record.get("logQueueMaxSize")
        if isinstance(queue_size, int) and isinstance(queue_max, int) and queue_max > 0:
            if queue_size / queue_max >= 0.8:
                warnings.append("process_log_queue_backlog")
        module_write = record.get("moduleLogWriteMs")
        module_write_age = record.get("moduleLogWriteAgeMs")
        scheduled_ms = float(record.get("scheduledIntervalMs") or 0.0)
        recent_write_limit_ms = max(5000.0, scheduled_ms + 500.0)
        if (
            isinstance(module_write, (int, float))
            and module_write >= 100.0
            and isinstance(module_write_age, (int, float))
            and module_write_age <= recent_write_limit_ms
        ):
            warnings.append("module_log_write_slow")
        diagnostic_write = record.get("diagnosticsWriteMs")
        if isinstance(diagnostic_write, (int, float)) and diagnostic_write >= 100.0:
            warnings.append("diagnostics_write_slow")
        return warnings

    def _collect_sample(
        self,
        *,
        now: float,
        lag_ms: float,
        scheduled_interval: float,
        active: bool,
        triggers: list[dict[str, Any]],
        process_log_stats: dict[str, Any] | None,
        module_log_write_ms: float | None,
    ) -> dict[str, Any]:
        collection_started = time.perf_counter()
        previous_sample = self._last_sample_mono
        wall_delta = None if previous_sample is None else max(0.000001, now - previous_sample)
        process_cpu_time = self._process_cpu_time()
        cpu_delta = None
        if (
            process_cpu_time is not None
            and self._last_process_cpu_time is not None
            and process_cpu_time >= self._last_process_cpu_time
        ):
            cpu_delta = process_cpu_time - self._last_process_cpu_time
        self._last_process_cpu_time = process_cpu_time
        process_cpu_raw = None
        if cpu_delta is not None and wall_delta is not None:
            process_cpu_raw = cpu_delta * 100.0 / wall_delta

        record: dict[str, Any] = {
            "schemaVersion": RUNTIME_DIAGNOSTIC_SCHEMA_VERSION,
            "event": "runtime_sample",
            "timestamp": _utc_iso(self._wall_clock()),
            "module": self.module_name,
            "processId": self.process_id,
            "processStartToken": self._start_token,
            "hostId": self._static_inventory.get("hostId"),
            "collectorBackend": self._effective_backend(),
            "activePlanningWindow": bool(active),
            "sampleReason": "triggered" if triggers else "scheduled",
            "sampleIntervalMs": _finite_number(None if wall_delta is None else wall_delta * 1000.0),
            "scheduledIntervalMs": _finite_number(scheduled_interval * 1000.0),
            "sampleLagMs": _finite_number(lag_ms),
            "processCpuTimeDeltaSec": _finite_number(cpu_delta, digits=6),
            "processWallDeltaSec": _finite_number(wall_delta, digits=6),
            "processCpuPercentRaw": _finite_number(process_cpu_raw),
            "diagnosticTriggers": triggers,
            "triggerToSampleMs": _finite_number(
                max(
                    (
                        float(item.get("triggerToSampleMs", 0.0) or 0.0)
                        for item in triggers
                        if isinstance(item, dict)
                    ),
                    default=0.0,
                )
            ) if triggers else None,
            "moduleLogWriteMs": _finite_number(module_log_write_ms),
            "diagnosticsWriteMs": _finite_number(self._last_write_ms),
            "diagnosticWriteFailures": self._write_failures,
            "diagnosticCollectionFailures": self._collection_failures,
            "diagnosticRotationFailures": self._rotation_failures,
            "diagnosticDroppedTriggers": self._dropped_triggers,
            "diagnosticCoalescedTriggers": self._coalesced_triggers,
            "diagnosticRetentionDeletedFiles": self._retention_deleted_files,
            "diagnosticRetentionFailures": self._retention_failures,
            "pythonThreadCount": int(threading.active_count()),
        }
        record.update(self._psutil_system_snapshot())
        if "systemCpuPercent" not in record:
            fallback_cpu = self._fallback.system_cpu_percent()
            if fallback_cpu is not None:
                record["systemCpuPercent"] = round(fallback_cpu, 2)
        if "systemMemoryTotalBytes" not in record:
            record.update(self._fallback.memory())
        record.update(self._process_fast_snapshot())

        affinity = self._static_inventory.get("effectiveCpuAffinityCount")
        logical = self._static_inventory.get("cpuLogicalCount")
        capacity = affinity if isinstance(affinity, (int, float)) and affinity > 0 else logical
        if process_cpu_raw is not None:
            record["processCpuCoreEquivalent"] = round(process_cpu_raw / 100.0, 3)
            if isinstance(capacity, (int, float)) and capacity > 0:
                record["processCpuPercentOfAffinity"] = round(process_cpu_raw / capacity, 3)

        parent_rss = record.get("processRssBytes")
        tree_refreshed = self._refresh_tree_snapshot(
            now,
            parent_rss if isinstance(parent_rss, int) else None,
            process_cpu_time,
            capacity,
        )
        detail_boundary = False
        for trigger_item in triggers:
            context = trigger_item.get("context") if isinstance(trigger_item, dict) else None
            lifecycle = str(context.get("lifecycle") or "").lower() if isinstance(context, dict) else ""
            if any(token in lifecycle for token in ("worker_thread_start", "worker_thread_stop", "exception")):
                detail_boundary = True
                break
        self._refresh_detail_snapshot(now, force=detail_boundary)
        record["processTreeSampleFresh"] = bool(tree_refreshed)
        if self._last_tree_mono is not None:
            record["processTreeSampleAgeMs"] = round(max(0.0, now - self._last_tree_mono) * 1000.0, 3)
        record.update(self._tree_cache)
        record.update(self._detail_cache)
        record["collectorBackend"] = self._effective_backend()
        record["missingCapabilities"] = sorted(self._capability_failures)[:16]
        record["capabilityFailureCount"] = int(sum(self._capability_failures.values()))

        stats = dict(process_log_stats or {})
        if stats:
            record["logQueueDepth"] = int(stats.get("queued", 0) or 0)
            record["logQueueMaxSize"] = int(stats.get("queueMaxSize", 0) or 0)
            record["logDroppedMessages"] = int(stats.get("dropped", 0) or 0)
            record["logDroppedMessagesTotal"] = int(stats.get("droppedTotal", 0) or 0)
            record["logFlushFailures"] = int(stats.get("flushFailures", 0) or 0)
            record["moduleLogBatchItems"] = int(stats.get("lastBatchItems", 0) or 0)
            record["moduleLogBatchChars"] = int(stats.get("lastBatchChars", 0) or 0)
            record["moduleLogWriteAgeMs"] = _finite_number(stats.get("lastBatchAgeMs"))
        record["collectionMs"] = round((time.perf_counter() - collection_started) * 1000.0, 3)
        warnings = self._warning_codes(record)
        if record["collectionMs"] >= 50.0:
            warnings.append("diagnostic_collection_slow")
        if self._dropped_triggers > 0:
            warnings.append("diagnostic_trigger_queue_overflow")
        if self._coalesced_triggers > 0:
            warnings.append("diagnostic_trigger_rate_limited")
        record["warningCodes"] = warnings
        self._last_sample_mono = now
        return record

    def poll(
        self,
        *,
        process_log_stats: dict[str, Any] | None = None,
        module_log_write_ms: float | None = None,
        force: bool = False,
    ) -> bool:
        """Collect/write one due sample. Must be called only by the sink worker."""
        if not self.enabled or os.getpid() != self._owner_pid:
            return False
        now = self._monotonic()
        with self._lock:
            if self._active and now >= self._active_deadline:
                self._active = False
                self._active_deadline = 0.0
            active = self._active
            scheduled_interval = self._active_interval if active else self._idle_interval
            forced_due = self._force_due
            min_force_time = -math.inf if self._last_sample_mono is None else self._last_sample_mono + 0.2
            due = bool(force or now >= self._next_due or (forced_due and now >= min_force_time))
            if not due:
                return False
            lag_ms = max(0.0, (now - self._next_due) * 1000.0)
            triggers = list(self._triggers)
            self._triggers.clear()
            self._force_due = False
            self._next_due = now + scheduled_interval
        trigger_delays: list[float] = []
        for item in triggers:
            requested = item.pop("_requestedMonotonic", None)
            if isinstance(requested, (int, float)):
                delay_ms = max(0.0, (now - float(requested)) * 1000.0)
                trigger_delays.append(delay_ms)
                item.setdefault("triggerToSampleMs", round(delay_ms, 3))
        try:
            record = self._collect_sample(
                now=now,
                lag_ms=lag_ms,
                scheduled_interval=scheduled_interval,
                active=active,
                triggers=triggers,
                process_log_stats=process_log_stats,
                module_log_write_ms=module_log_write_ms,
            )
        except Exception:
            self._collection_failures += 1
            record = {
                "schemaVersion": RUNTIME_DIAGNOSTIC_SCHEMA_VERSION,
                "event": "runtime_collection_error",
                "timestamp": _utc_iso(self._wall_clock()),
                "module": self.module_name,
                "processId": self.process_id,
                "processStartToken": self._start_token,
                "collectorBackend": self._effective_backend(),
                "diagnosticCollectionFailures": self._collection_failures,
            }
        self._samples += 1
        self._write_records([record])
        return True

    @staticmethod
    def _compact_trigger_for_record(item: dict[str, Any]) -> dict[str, Any]:
        compact: dict[str, Any] = {}
        for key in ("trigger", "timestamp", "triggerToSampleMs", "coalescedCount"):
            value = item.get(key)
            if value is not None:
                compact[key] = value
        context = item.get("context")
        if not isinstance(context, dict):
            return compact
        compact_context: dict[str, Any] = {}
        for key in (
            "event",
            "lifecycle",
            "component",
            "replanTransactionId",
            "trigger",
            "triggerType",
            "pipeline",
            "phase",
            "missionPlanID",
            "aircraftID",
            "elapsedMs",
            "outcome",
            "reason",
            "success",
            "tickLagMs",
            "sendMs",
            "lastSuccessAgeMs",
            "writeElapsedMs",
            "writeRetry",
        ):
            value = context.get(key)
            if value is not None:
                compact_context[key] = (
                    _bounded_text(value, 96) if key == "reason" and isinstance(value, str) else value
                )
        extra = context.get("extra")
        if isinstance(extra, dict):
            compact_extra = {
                key: extra[key]
                for key in (
                    "workers",
                    "futures",
                    "variants",
                    "variant",
                    "core_workers",
                    "store_prepare_workers",
                    "store_commit_workers",
                    "duration_ms",
                    "status",
                    "option",
                )
                if key in extra and extra[key] is not None
            }
            if compact_extra:
                compact_context["extra"] = compact_extra
        if compact_context:
            compact["context"] = compact_context
        return compact

    @staticmethod
    def _trigger_record_priority(index: int, total: int, item: dict[str, Any]) -> tuple[int, int]:
        context = item.get("context") if isinstance(item.get("context"), dict) else {}
        trigger = str(item.get("trigger") or "").lower()
        phase = str(context.get("phase") or "").lower()
        outcome = str(context.get("outcome") or "").lower()
        lifecycle = str(context.get("lifecycle") or "").lower()
        score = 0
        if any(
            token in f"{trigger} {outcome} {lifecycle}"
            for token in ("fail", "error", "exception", "crash")
        ):
            score += 1000
        if phase == "parallel_executor_submitted":
            score += 900
        if index == total - 1:
            score += 800
        extra = context.get("extra")
        if isinstance(extra, dict) and any(
            key in extra for key in ("workers", "futures", "variants")
        ):
            score += 400
        return score, index

    def _encode_record(self, record: dict[str, Any]) -> bytes:
        try:
            encoded = (
                json.dumps(record, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n"
            ).encode("utf-8", errors="replace")
        except Exception:
            safe = {
                "schemaVersion": RUNTIME_DIAGNOSTIC_SCHEMA_VERSION,
                "event": "runtime_serialization_error",
                "timestamp": _utc_iso(self._wall_clock()),
                "module": self.module_name,
                "processId": self.process_id,
            }
            encoded = (json.dumps(safe, separators=(",", ":")) + "\n").encode("utf-8")
        if len(encoded) <= self._max_record_bytes:
            return encoded
        compact = dict(record)
        compact.pop("perLogicalCpuPercent", None)
        triggers = [
            item
            for item in list(compact.get("diagnosticTriggers") or [])
            if isinstance(item, dict)
        ]
        compact_triggers = [self._compact_trigger_for_record(item) for item in triggers]
        compact["diagnosticTriggerCount"] = len(compact_triggers)
        compact["diagnosticTriggersOmitted"] = 0
        compact["diagnosticTriggers"] = compact_triggers
        compact["recordTruncated"] = True
        encoded = (
            json.dumps(compact, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n"
        ).encode("utf-8", errors="replace")
        if len(encoded) <= self._max_record_bytes:
            self._truncated_records += 1
            return encoded

        # Fit compact contexts by importance. Failure, parallel submission and
        # the latest boundary are retained before routine phases.
        base = dict(compact)
        base["diagnosticTriggers"] = []
        base["diagnosticTriggersOmitted"] = len(compact_triggers)
        selected: list[tuple[int, dict[str, Any]]] = []
        ranked_indices = sorted(
            range(len(triggers)),
            key=lambda index: self._trigger_record_priority(index, len(triggers), triggers[index]),
            reverse=True,
        )
        for index in ranked_indices:
            trial_selected = selected + [(index, compact_triggers[index])]
            trial = dict(base)
            trial["diagnosticTriggers"] = [
                value for _, value in sorted(trial_selected, key=lambda pair: pair[0])
            ]
            trial["diagnosticTriggersOmitted"] = len(compact_triggers) - len(trial_selected)
            trial_encoded = (
                json.dumps(trial, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
                + "\n"
            ).encode("utf-8", errors="replace")
            if len(trial_encoded) <= self._max_record_bytes:
                selected = trial_selected
                encoded = trial_encoded
        if selected and len(encoded) <= self._max_record_bytes:
            self._truncated_records += 1
            return encoded

        priority_trigger = (
            compact_triggers[ranked_indices[0]]
            if compact_triggers and ranked_indices
            else None
        )
        minimal = {
            key: compact.get(key)
            for key in (
                "schemaVersion",
                "event",
                "timestamp",
                "module",
                "processId",
                "processStartToken",
                "collectorBackend",
                "sampleLagMs",
                "collectionMs",
                "warningCodes",
            )
        }
        minimal["diagnosticTriggerCount"] = len(compact_triggers)
        minimal["diagnosticTriggersOmitted"] = max(
            0,
            len(compact_triggers) - int(priority_trigger is not None),
        )
        minimal["diagnosticTriggers"] = [priority_trigger] if priority_trigger is not None else []
        minimal["recordTruncated"] = True
        self._truncated_records += 1
        minimal_encoded = (
            json.dumps(minimal, ensure_ascii=False, separators=(",", ":"), allow_nan=False) + "\n"
        ).encode("utf-8", errors="replace")
        if len(minimal_encoded) <= self._max_record_bytes:
            return minimal_encoded
        # The configured minimum is 1 KiB, but keep an identifier-only fallback
        # so the recorder can never violate its per-line contract.
        minimal.pop("diagnosticTriggers", None)
        minimal["diagnosticTriggersOmitted"] = len(compact_triggers)
        return (json.dumps(minimal, separators=(",", ":")) + "\n").encode("utf-8")

    def _resolve_path(self) -> Path:
        return Path(self._path_resolver(self._filename))

    def _diagnostic_file_pid(self, path: Path) -> int | None:
        # A full match prevents similarly prefixed modules (for example
        # ``foo`` and ``foo_0303``) from sharing a retention boundary.
        match = re.fullmatch(
            rf"{re.escape(self.module_name)}_(\d+)_\d{{8}}T\d{{6}}_\d{{6}}Z(?:\.\d+)?\.jsonl",
            path.name,
        )
        if match is None:
            return None
        try:
            return int(match.group(1))
        except Exception:
            return None

    def _pid_is_alive(self, pid: int, *, age_sec: float) -> bool:
        if int(pid) == self.process_id:
            return True
        if self._psutil is not None:
            try:
                return bool(self._psutil.pid_exists(int(pid)))
            except Exception:
                return True
        # Without a process API, never touch a file that could be from a live recent run.
        return age_sec < 24.0 * 60.0 * 60.0

    def _prune_old_diagnostic_files(self, directory: Path, current_path: Path) -> None:
        directory_key = str(directory)
        if directory_key in self._retention_checked_dirs:
            return
        self._retention_checked_dirs.add(directory_key)
        now_wall = self._wall_clock()
        cutoff_age = float(self._retention_days) * 24.0 * 60.0 * 60.0
        try:
            candidates = []
            for candidate in directory.glob(f"{self.module_name}_*.jsonl"):
                if candidate == current_path or not candidate.is_file():
                    continue
                try:
                    stat = candidate.stat()
                    candidates.append((candidate, int(stat.st_size), float(stat.st_mtime)))
                except Exception:
                    self._retention_failures += 1
            candidates.sort(key=lambda item: item[2], reverse=True)
        except Exception:
            self._retention_failures += 1
            return

        retained_bytes = 0
        for candidate, size, modified in candidates:
            age_sec = max(0.0, now_wall - modified)
            pid = self._diagnostic_file_pid(candidate)
            if pid is None:
                # This file is outside the exact module filename grammar.
                continue
            if self._pid_is_alive(pid, age_sec=age_sec):
                retained_bytes += max(0, size)
                continue
            expired = age_sec >= cutoff_age
            over_budget = retained_bytes + max(0, size) > self._module_max_bytes
            if not expired and not over_budget:
                retained_bytes += max(0, size)
                continue
            try:
                candidate.unlink()
                self._retention_deleted_files += 1
            except Exception:
                self._retention_failures += 1
                retained_bytes += max(0, size)

    def _close_handle(self) -> None:
        try:
            if self._handle is not None:
                self._handle.flush()
                self._handle.close()
        except Exception:
            pass
        self._handle = None

    def _ensure_handle(self) -> bool:
        try:
            path = self._resolve_path()
            if self._handle is not None and self._current_path == path:
                return True
            self._close_handle()
            path.parent.mkdir(parents=True, exist_ok=True)
            self._prune_old_diagnostic_files(path.parent, path)
            self._handle = path.open("ab", buffering=0)
            self._current_path = path
            self._current_bytes = int(path.stat().st_size) if path.exists() else 0
            path_key = str(path)
            if path_key not in self._inventory_written_for_path:
                inventory = dict(self._static_inventory)
                inventory["timestamp"] = _utc_iso(self._wall_clock())
                line = self._encode_record(inventory)
                self._handle.write(line)
                self._current_bytes += len(line)
                self._inventory_written_for_path.add(path_key)
            return True
        except Exception:
            self._write_failures += 1
            self._close_handle()
            return False

    def _backup_path(self, base: Path, index: int) -> Path:
        return base.with_name(f"{base.stem}.{index}{base.suffix}")

    def _rotate(self) -> bool:
        path = self._current_path
        if path is None or self._backup_count <= 0:
            return False
        now = self._monotonic()
        if now < self._rotation_retry_after:
            return False
        self._close_handle()
        base_moved = False
        try:
            oldest = self._backup_path(path, self._backup_count)
            if oldest.exists():
                oldest.unlink()
            for index in range(self._backup_count - 1, 0, -1):
                source = self._backup_path(path, index)
                if source.exists():
                    os.replace(source, self._backup_path(path, index + 1))
            if path.exists():
                os.replace(path, self._backup_path(path, 1))
                base_moved = True
                self._inventory_written_for_path.discard(str(path))
            self._handle = path.open("ab", buffering=0)
            self._current_bytes = 0
            inventory = dict(self._static_inventory)
            inventory["timestamp"] = _utc_iso(self._wall_clock())
            inventory["rotatedFileStart"] = True
            line = self._encode_record(inventory)
            self._handle.write(line)
            self._current_bytes += len(line)
            self._inventory_written_for_path.add(str(path))
            return True
        except Exception:
            self._rotation_failures += 1
            self._rotation_retry_after = now + 30.0
            self._close_handle()
            try:
                self._handle = path.open("ab", buffering=0)
                self._current_bytes = int(path.stat().st_size) if path.exists() else 0
                if base_moved:
                    inventory = dict(self._static_inventory)
                    inventory["timestamp"] = _utc_iso(self._wall_clock())
                    inventory["rotationRecovery"] = True
                    line = self._encode_record(inventory)
                    self._handle.write(line)
                    self._current_bytes += len(line)
                    self._inventory_written_for_path.add(str(path))
            except Exception:
                self._close_handle()
                self._handle = None
            return False

    def _write_records(self, records: list[dict[str, Any]]) -> None:
        started = time.perf_counter()
        if not self._ensure_handle():
            return
        try:
            for record in records:
                line = self._encode_record(record)
                if self._current_bytes > 0 and self._current_bytes + len(line) > self._max_file_bytes:
                    self._rotate()
                if self._handle is None and not self._ensure_handle():
                    return
                self._handle.write(line)
                self._current_bytes += len(line)
            self._handle.flush()
            self._last_write_ms = round((time.perf_counter() - started) * 1000.0, 3)
        except Exception:
            self._write_failures += 1
            self._close_handle()

    def stats(self) -> dict[str, Any]:
        with self._lock:
            active = bool(self._active)
            pending = len(self._triggers)
        return {
            "enabled": bool(self.enabled),
            "backend": self._effective_backend(),
            "missingCapabilities": sorted(self._capability_failures)[:16],
            "capabilityFailureCount": int(sum(self._capability_failures.values())),
            "samples": int(self._samples),
            "pendingTriggers": int(pending),
            "active": active,
            "writeFailures": int(self._write_failures),
            "collectionFailures": int(self._collection_failures),
            "rotationFailures": int(self._rotation_failures),
            "truncatedRecords": int(self._truncated_records),
            "droppedTriggers": int(self._dropped_triggers),
            "coalescedTriggers": int(self._coalesced_triggers),
            "retentionDeletedFiles": int(self._retention_deleted_files),
            "retentionFailures": int(self._retention_failures),
        }

    def close(self) -> None:
        self._close_handle()


def create_runtime_diagnostics(module_name: str) -> RuntimeDiagnosticsRecorder | None:
    """Factory isolated so process logging can always survive diagnostics failure."""
    try:
        recorder = RuntimeDiagnosticsRecorder(module_name)
        return recorder if recorder.enabled else None
    except Exception:
        return None
