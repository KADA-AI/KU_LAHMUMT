"""Small reload-stable registry for mission-planning process pools.

The planner hot-reloads its runtime modules.  Keeping an executor directly in
one of those modules loses the last strong reference on reload and, more
importantly, can submit new code to workers that still have the old module
loaded.  This module is deliberately outside the planner reload watch list and
keys pools by the caller's reload generation.
"""

from __future__ import annotations

import atexit
import concurrent.futures
import os
import threading
from dataclasses import dataclass
from typing import Any, Callable, Dict


@dataclass
class _PoolEntry:
    executor: concurrent.futures.ProcessPoolExecutor
    family: str
    generation: str
    workers: int


_LOCK = threading.RLock()
_OWNER_PID = os.getpid()
_POOLS: Dict[tuple[str, str, int], _PoolEntry] = {}


def _shutdown_executor(
    executor: concurrent.futures.ProcessPoolExecutor,
    *,
    cancel_futures: bool,
) -> None:
    try:
        executor.shutdown(wait=False, cancel_futures=bool(cancel_futures))
    except TypeError:
        # Python < 3.9 compatibility for packaged deployments.
        try:
            executor.shutdown(wait=False)
        except Exception:
            pass
    except Exception:
        pass


def _reset_after_fork_unlocked() -> None:
    global _OWNER_PID, _POOLS
    current_pid = os.getpid()
    if current_pid == _OWNER_PID:
        return
    # A ProcessPoolExecutor inherited through fork is not usable in the child.
    # Do not call shutdown on the inherited object: it belongs to the parent.
    _OWNER_PID = current_pid
    _POOLS = {}


def acquire_process_pool(
    *,
    family: str,
    generation: str,
    max_workers: int,
    initializer: Callable[..., Any] | None = None,
    initargs: tuple[Any, ...] = (),
) -> concurrent.futures.ProcessPoolExecutor:
    """Return a lazy persistent pool for one reload generation/worker count."""

    family_key = str(family or "mission-planning")
    generation_key = str(generation or "default")
    workers = max(1, int(max_workers))
    stale: list[_PoolEntry] = []
    with _LOCK:
        _reset_after_fork_unlocked()
        key = (family_key, generation_key, workers)
        entry = _POOLS.get(key)
        if entry is not None:
            return entry.executor

        # A new planner reload generation must never reuse workers that imported
        # the previous source.  Worker-count variants in the same generation are
        # allowed to coexist so concurrent callers cannot invalidate each other.
        for old_key, old_entry in list(_POOLS.items()):
            if old_entry.family == family_key and old_entry.generation != generation_key:
                stale.append(_POOLS.pop(old_key))

        executor = concurrent.futures.ProcessPoolExecutor(
            max_workers=workers,
            initializer=initializer,
            initargs=tuple(initargs or ()),
        )
        _POOLS[key] = _PoolEntry(
            executor=executor,
            family=family_key,
            generation=generation_key,
            workers=workers,
        )

    for old_entry in stale:
        # Already-submitted work may finish; new submissions fail promptly and
        # are handled by the caller's correctness-preserving fallback.
        _shutdown_executor(old_entry.executor, cancel_futures=False)
    return executor


def invalidate_process_pool(executor: object) -> bool:
    """Remove and stop the exact executor after a submit/worker failure."""

    removed: _PoolEntry | None = None
    with _LOCK:
        _reset_after_fork_unlocked()
        for key, entry in list(_POOLS.items()):
            if entry.executor is executor:
                removed = _POOLS.pop(key)
                break
    if removed is None:
        return False
    _shutdown_executor(removed.executor, cancel_futures=True)
    return True


def shutdown_process_pools(*, wait: bool = False, cancel_futures: bool = True) -> int:
    """Shutdown all registered pools (GUI exit and test cleanup hook)."""

    with _LOCK:
        _reset_after_fork_unlocked()
        entries = list(_POOLS.values())
        _POOLS.clear()
    for entry in entries:
        try:
            entry.executor.shutdown(
                wait=bool(wait),
                cancel_futures=bool(cancel_futures),
            )
        except TypeError:
            try:
                entry.executor.shutdown(wait=bool(wait))
            except Exception:
                pass
        except Exception:
            pass
    return len(entries)


def process_pool_registry_size() -> int:
    """Read-only diagnostic used by focused tests and shutdown logging."""

    with _LOCK:
        _reset_after_fork_unlocked()
        return len(_POOLS)


atexit.register(shutdown_process_pools, wait=False, cancel_futures=True)
