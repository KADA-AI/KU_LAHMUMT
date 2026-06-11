from __future__ import annotations

import json
import os
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from modules.common import db_paths

_MODE_ENV = "REPLAN_0902_SIDECAR_MODE"
_FALLBACK_MODE_ENV = "REPLAN_SIDECAR_MODE"
_OFF_MODES = {"0", "false", "no", "off", "skip", "disabled", "disable", "none"}
_COMPACT_MODES = {"compact", "performance", "perf", "fast", "min", "minimal"}


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
    mode = sidecar_mode()
    if mode == "off":
        return None

    data = dict(payload or {})
    timestamp_ms = _coerce_timestamp(data)
    if timestamp_ms is None:
        return None
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    path = _payload_path(timestamp_ms)
    path.parent.mkdir(parents=True, exist_ok=True)

    existing_entries: List[Dict[str, Any]] = []
    if path.exists():
        try:
            existing_entries = _normalize_entries(json.loads(path.read_text(encoding="utf-8")))
        except Exception:
            existing_entries = []

    identity = _payload_identity(data)
    if not any(_payload_identity(entry) == identity for entry in existing_entries):
        existing_entries.append(data)

    last_error: Optional[BaseException] = None
    for attempt in range(5):
        tmp = path.with_name(f"{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
        try:
            tmp.write_text(_serialize_entries(existing_entries, mode), encoding="utf-8")
            tmp.replace(path)
            return path
        except OSError as exc:
            last_error = exc
            try:
                if tmp.exists():
                    tmp.unlink()
            except Exception:
                pass
            if path.exists():
                return path
            time.sleep(0.01 * (attempt + 1))

    if path.exists():
        return path
    if last_error is not None:
        raise last_error
    return path


def load_payload(
    timestamp_ms: int,
    *,
    reason: object | None = None,
    replan_level: object | None = None,
) -> Optional[Dict[str, Any]]:
    path = _payload_path(int(timestamp_ms))
    if not path.exists():
        return None
    entries: List[Dict[str, Any]] = []
    for attempt in range(3):
        try:
            entries = _normalize_entries(json.loads(path.read_text(encoding="utf-8")))
            break
        except Exception:
            if attempt >= 2:
                return None
            time.sleep(0.005 * (attempt + 1))
    if not entries:
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
        return dict(entry)

    return dict(entries[-1])


def payload_identity(payload: Dict[str, Any]) -> tuple[str, int, str]:
    return _payload_identity(dict(payload or {}))


def load_latest_payload() -> Optional[Dict[str, Any]]:
    base = _base_dir()
    if not base.exists():
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
        return None

    candidate_paths.sort(key=lambda item: item[0], reverse=True)
    for _timestamp_ms, path in candidate_paths:
        try:
            entries = _normalize_entries(json.loads(path.read_text(encoding="utf-8")))
        except Exception:
            continue
        if not entries:
            continue
        return dict(entries[-1])
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
