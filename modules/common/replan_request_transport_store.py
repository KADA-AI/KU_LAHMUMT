from __future__ import annotations

import copy
import json
import os
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from modules.common import db_paths

try:
    from modules.common import replan_perf
except Exception:
    import sys as _sys

    _COMMON_DIR = next(
        (
            parent / "common"
            for parent in Path(__file__).resolve().parents
            if (parent / "common" / "replan_perf.py").exists()
        ),
        None,
    )
    if _COMMON_DIR is not None and str(_COMMON_DIR) not in _sys.path:
        _sys.path.insert(0, str(_COMMON_DIR))
    import replan_perf  # type: ignore

_MODE_ENV = "REPLAN_0902_SIDECAR_MODE"
_FALLBACK_MODE_ENV = "REPLAN_SIDECAR_MODE"
_OFF_MODES = {"0", "false", "no", "off", "skip", "disabled", "disable", "none"}
_COMPACT_MODES = {"compact", "performance", "perf", "fast", "min", "minimal"}
_ENTRY_CACHE: dict[str, tuple[tuple[int, int], List[Dict[str, Any]], str]] = {}


def _base_dir() -> Path:
    raw = db_paths.get_db_subpath("DSS_Internal", "replan_request_transport")
    return _normalize_path(raw)


def _payload_path(timestamp_ms: int) -> Path:
    return _base_dir() / f"replan_request_{int(timestamp_ms)}.json"


def payload_path_for_timestamp(timestamp_ms: int) -> Path:
    return _payload_path(int(timestamp_ms))


def payload_path_for_payload(payload: Dict[str, Any]) -> Optional[Path]:
    timestamp_ms = _coerce_timestamp(dict(payload or {}))
    if timestamp_ms is None:
        return None
    return _payload_path(timestamp_ms)


def sidecar_mode() -> str:
    raw = os.environ.get(_MODE_ENV)
    if raw is None:
        raw = os.environ.get(_FALLBACK_MODE_ENV, "")
    value = str(raw or "").strip().lower()
    if value in _OFF_MODES:
        return "off"
    if value in _COMPACT_MODES:
        return "compact"
    return "pretty"


def sidecar_enabled() -> bool:
    return sidecar_mode() != "off"


def _serialize_entries(entries: List[Dict[str, Any]], mode: str) -> str:
    if mode == "compact":
        return json.dumps(entries, ensure_ascii=False, separators=(",", ":"))
    return json.dumps(entries, ensure_ascii=False, indent=2)


def _path_signature(path: Path) -> tuple[int, int] | None:
    try:
        stat = path.stat()
    except OSError:
        return None
    return int(getattr(stat, "st_mtime_ns", int(stat.st_mtime * 1_000_000_000))), int(stat.st_size)


def _clone_entries(entries: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return copy.deepcopy(entries)


def _cache_key(path: Path) -> str:
    try:
        return str(path.resolve())
    except Exception:
        return str(path)


def _get_cached_entries(path: Path) -> tuple[List[Dict[str, Any]], str] | None:
    signature = _path_signature(path)
    if signature is None:
        return None
    cached = _ENTRY_CACHE.get(_cache_key(path))
    if cached is None:
        return None
    cached_signature, entries, text = cached
    if cached_signature != signature:
        return None
    return _clone_entries(entries), text


def _store_cached_entries(path: Path, entries: List[Dict[str, Any]], text: str) -> None:
    signature = _path_signature(path)
    if signature is None:
        _ENTRY_CACHE.pop(_cache_key(path), None)
        return
    _ENTRY_CACHE[_cache_key(path)] = (signature, _clone_entries(entries), str(text))


def _read_entries(
    path: Path,
    *,
    attempts: int,
    retry_delay_ms: float = 5.0,
) -> tuple[List[Dict[str, Any]], str, int, bool, bool]:
    cached = _get_cached_entries(path)
    if cached is not None:
        entries, text = cached
        return entries, text, 0, True, False

    last_error = False
    max_attempts = max(1, int(attempts or 1))
    for attempt in range(max_attempts):
        try:
            text = path.read_text(encoding="utf-8")
            entries = _normalize_entries(json.loads(text))
            _store_cached_entries(path, entries, text)
            return entries, text, attempt + 1, False, False
        except Exception:
            last_error = True
            if attempt >= max_attempts - 1:
                break
            time.sleep((retry_delay_ms / 1000.0) * (attempt + 1))
    return [], "", max_attempts, False, last_error


def _coerce_timestamp(payload: Dict[str, Any]) -> Optional[int]:
    candidates = [payload.get("timestamp")]
    request_time = payload.get("replanRequestTime")
    if isinstance(request_time, dict):
        candidates.append(request_time.get("replanRequestTimestamp"))
    for value in candidates:
        try:
            timestamp_ms = int(value)
        except Exception:
            continue
        if timestamp_ms > 0:
            return timestamp_ms
    return None


def _normalize_entries(raw: Any) -> List[Dict[str, Any]]:
    if isinstance(raw, dict):
        return [dict(raw)]
    if isinstance(raw, list):
        return [dict(item) for item in raw if isinstance(item, dict)]
    return []


def _payload_identity(payload: Dict[str, Any]) -> tuple[str, int, str]:
    reason = str(payload.get("replanRequest") or payload.get("replanReason") or "").strip()
    try:
        level = int(payload.get("replanLevel") or 0)
    except Exception:
        level = 0
    try:
        detail = json.dumps(payload.get("replanDetail"), ensure_ascii=False, sort_keys=True)
    except Exception:
        detail = str(payload.get("replanDetail") or "")
    return reason, level, detail


def save_payload(payload: Dict[str, Any]) -> Optional[Path]:
    perf_start = replan_perf.start_timer()
    mode = sidecar_mode()
    if mode == "off":
        replan_perf.add_elapsed("common.replan_sidecar.save", perf_start, mode_off=1)
        return None

    data = dict(payload or {})
    timestamp_ms = _coerce_timestamp(data)
    if timestamp_ms is None:
        replan_perf.add_elapsed("common.replan_sidecar.save", perf_start, no_timestamp=1)
        return None
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    path = _payload_path(timestamp_ms)
    path.parent.mkdir(parents=True, exist_ok=True)

    existing_entries: List[Dict[str, Any]] = []
    existing_text = ""
    if path.exists():
        read_start = replan_perf.start_timer()
        existing_entries, existing_text, read_attempts, cache_hit, read_error = _read_entries(path, attempts=1)
        replan_perf.add_elapsed(
            "common.replan_sidecar.save.read",
            read_start,
            read_chars=len(existing_text),
            attempts=read_attempts,
            cache_hit=cache_hit,
            error=read_error,
        )

    identity = _payload_identity(data)
    duplicate = any(_payload_identity(entry) == identity for entry in existing_entries)
    if not duplicate:
        existing_entries.append(data)

    last_error: Optional[BaseException] = None
    for attempt in range(5):
        tmp = path.with_name(f"{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
        try:
            text = _serialize_entries(existing_entries, mode)
            if duplicate and existing_text == text:
                replan_perf.add_elapsed(
                    "common.replan_sidecar.save",
                    perf_start,
                    skipped_duplicate=1,
                    entries=len(existing_entries),
                    attempts=attempt + 1,
                )
                return path
            tmp.write_text(text, encoding="utf-8")
            tmp.replace(path)
            _store_cached_entries(path, existing_entries, text)
            replan_perf.add_elapsed(
                "common.replan_sidecar.save",
                perf_start,
                written=1,
                entries=len(existing_entries),
                write_chars=len(text),
                attempts=attempt + 1,
            )
            return path
        except OSError as exc:
            last_error = exc
            try:
                if tmp.exists():
                    tmp.unlink()
            except Exception:
                pass
            if path.exists():
                replan_perf.add_elapsed(
                    "common.replan_sidecar.save",
                    perf_start,
                    existing_after_error=1,
                    attempts=attempt + 1,
                )
                return path
            time.sleep(0.01 * (attempt + 1))

    if path.exists():
        replan_perf.add_elapsed("common.replan_sidecar.save", perf_start, existing_after_retries=1)
        return path
    if last_error is not None:
        replan_perf.add_elapsed("common.replan_sidecar.save", perf_start, error=1)
        raise last_error
    replan_perf.add_elapsed("common.replan_sidecar.save", perf_start, no_write=1)
    return path


def load_payload(
    timestamp_ms: int,
    *,
    reason: object | None = None,
    replan_level: object | None = None,
) -> Optional[Dict[str, Any]]:
    perf_start = replan_perf.start_timer()
    path = _payload_path(int(timestamp_ms))
    if not path.exists():
        replan_perf.add_elapsed("common.replan_sidecar.load", perf_start, missing=1)
        return None
    read_start = replan_perf.start_timer()
    entries, text, read_attempts, cache_hit, read_error = _read_entries(path, attempts=3, retry_delay_ms=5.0)
    replan_perf.add_elapsed(
        "common.replan_sidecar.load.read",
        read_start,
        read_chars=len(text),
        attempts=read_attempts,
        cache_hit=cache_hit,
        error=read_error,
    )
    if read_error:
        replan_perf.add_elapsed("common.replan_sidecar.load", perf_start, error=1, attempts=read_attempts)
        return None
    if not entries:
        replan_perf.add_elapsed("common.replan_sidecar.load", perf_start, empty=1)
        return None

    normalized_reason = str(reason or "").strip()
    try:
        normalized_level = int(replan_level) if replan_level is not None else None
    except Exception:
        normalized_level = None

    for entry in entries:
        entry_reason = str(entry.get("replanRequest") or entry.get("replanReason") or "").strip()
        try:
            entry_level = int(entry.get("replanLevel") or 0)
        except Exception:
            entry_level = 0
        if normalized_reason and entry_reason != normalized_reason:
            continue
        if normalized_level is not None and entry_level != normalized_level:
            continue
        replan_perf.add_elapsed(
            "common.replan_sidecar.load",
            perf_start,
            matched=1,
            entries=len(entries),
        )
        return copy.deepcopy(entry)

    replan_perf.add_elapsed(
        "common.replan_sidecar.load",
        perf_start,
        fallback_latest=1,
        entries=len(entries),
    )
    return copy.deepcopy(entries[-1])


def payload_identity(payload: Dict[str, Any]) -> tuple[str, int, str]:
    return _payload_identity(dict(payload or {}))


def load_latest_payload() -> Optional[Dict[str, Any]]:
    perf_start = replan_perf.start_timer()
    base = _base_dir()
    if not base.exists():
        replan_perf.add_elapsed("common.replan_sidecar.load_latest", perf_start, missing_base=1)
        return None

    candidate_paths: list[tuple[int, Path]] = []
    for path in base.glob("replan_request_*.json"):
        stem = path.stem
        try:
            timestamp_ms = int(stem.rsplit("_", 1)[-1])
        except Exception:
            continue
        candidate_paths.append((timestamp_ms, path))

    if not candidate_paths:
        replan_perf.add_elapsed("common.replan_sidecar.load_latest", perf_start, no_candidates=1)
        return None

    candidate_paths.sort(key=lambda item: item[0], reverse=True)
    for _timestamp_ms, path in candidate_paths:
        read_start = replan_perf.start_timer()
        entries, text, read_attempts, cache_hit, read_error = _read_entries(path, attempts=1)
        replan_perf.add_elapsed(
            "common.replan_sidecar.load_latest.read",
            read_start,
            read_chars=len(text),
            attempts=read_attempts,
            cache_hit=cache_hit,
            error=read_error,
        )
        if read_error:
            continue
        if not entries:
            continue
        replan_perf.add_elapsed(
            "common.replan_sidecar.load_latest",
            perf_start,
            candidates=len(candidate_paths),
            matched=1,
        )
        return copy.deepcopy(entries[-1])
    replan_perf.add_elapsed("common.replan_sidecar.load_latest", perf_start, candidates=len(candidate_paths), miss=1)
    return None


def _normalize_path(path: Path) -> Path:
    if os.name == "nt":
        return Path(str(path))
    text = str(path)
    if len(text) >= 2 and text[1] == ":":
        drive = text[0].lower()
        rest = text[2:].replace("\\", "/")
        rest = rest.lstrip("/")
        return Path(f"/mnt/{drive}/{rest}")
    return Path(text)
