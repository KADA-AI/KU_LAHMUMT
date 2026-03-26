from __future__ import annotations

import json
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from modules.common import db_paths


def _base_dir() -> Path:
    raw = db_paths.get_db_subpath("DSS_Internal", "replan_request_transport")
    return _normalize_path(raw)


def _payload_path(timestamp_ms: int) -> Path:
    return _base_dir() / f"replan_request_{int(timestamp_ms)}.json"


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

    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(existing_entries, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(path)
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
    try:
        entries = _normalize_entries(json.loads(path.read_text(encoding="utf-8")))
    except Exception:
        return None
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
