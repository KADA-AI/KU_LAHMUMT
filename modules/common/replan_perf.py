from __future__ import annotations

import atexit
import json
import os
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator

_ENV_ENABLED = "DSS_REPLAN_PERF"
_ENV_LOG = "DSS_REPLAN_PERF_LOG"
_TRUE_VALUES = {"1", "true", "yes", "y", "on", "perf", "profile", "profiling"}

_LOCK = threading.RLock()
_METRICS: dict[str, dict[str, Any]] = {}


def is_enabled() -> bool:
    return str(os.getenv(_ENV_ENABLED, "")).strip().lower() in _TRUE_VALUES


def start_timer() -> float | None:
    if not is_enabled():
        return None
    return time.perf_counter()


def add(name: str, *, elapsed_ms: float = 0.0, **counters: Any) -> None:
    if not is_enabled():
        return

    key = str(name or "unnamed")
    elapsed = _safe_float(elapsed_ms)
    with _LOCK:
        row = _METRICS.setdefault(
            key,
            {
                "count": 0,
                "totalMs": 0.0,
                "maxMs": 0.0,
                "counters": {},
            },
        )
        row["count"] = int(row.get("count", 0)) + 1
        row["totalMs"] = float(row.get("totalMs", 0.0)) + elapsed
        row["maxMs"] = max(float(row.get("maxMs", 0.0)), elapsed)
        counter_map = row.setdefault("counters", {})
        if not isinstance(counter_map, dict):
            counter_map = {}
            row["counters"] = counter_map
        for counter_name, value in counters.items():
            numeric = _safe_numeric(value)
            if numeric is None:
                continue
            counter_map[str(counter_name)] = float(counter_map.get(str(counter_name), 0.0)) + numeric


def add_elapsed(name: str, start: float | None, **counters: Any) -> None:
    if start is None or not is_enabled():
        return
    add(name, elapsed_ms=(time.perf_counter() - start) * 1000.0, **counters)


@contextmanager
def measure(name: str, **counters: Any) -> Iterator[None]:
    start = start_timer()
    try:
        yield
    finally:
        add_elapsed(name, start, **counters)


def snapshot(*, reset: bool = False) -> dict[str, Any]:
    with _LOCK:
        metrics = json.loads(json.dumps(_METRICS, ensure_ascii=False))
        if reset:
            _METRICS.clear()
    return {
        "generatedAt": datetime.now(timezone.utc).isoformat(),
        "enabled": is_enabled(),
        "metrics": metrics,
    }


def write_snapshot(path: str | Path | None = None, *, reset: bool = False) -> Path | None:
    target_raw = path if path is not None else os.getenv(_ENV_LOG)
    if not target_raw:
        return None
    target = Path(target_raw)
    data = snapshot(reset=reset)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    return target


def _safe_float(value: Any) -> float:
    try:
        return float(value)
    except Exception:
        return 0.0


def _safe_numeric(value: Any) -> float | None:
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    try:
        return float(value)
    except Exception:
        return None


def _flush_at_exit() -> None:
    if not is_enabled():
        return
    try:
        write_snapshot()
    except Exception:
        pass


atexit.register(_flush_at_exit)
