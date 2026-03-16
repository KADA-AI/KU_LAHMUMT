from __future__ import annotations

import json
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import os
from modules.common import db_paths

try:
    from modules.mission_planning.runtime.attack_tracking_state import (
        update_from_agent_states as _update_attack_tracking_state,
    )
except Exception:
    _update_attack_tracking_state = None

SNAPSHOT_FILENAME = "latest_0401_agent_status.json"
LOG_FILENAME = "log_0401_agent_status_sim.jsonl"


def _snapshot_path() -> Path:
    base = _normalize_path(db_paths.get_active_db_root())
    return base / "DSS_Internal" / SNAPSHOT_FILENAME


def _log_path() -> Path:
    base = _normalize_path(db_paths.get_active_db_root())
    return base / "DSS_Internal" / LOG_FILENAME


def _to_plain_dict(obj: Any) -> Dict[str, Any]:
    if obj is None:
        return {}
    if isinstance(obj, dict):
        return obj
    if is_dataclass(obj):
        try:
            return asdict(obj)
        except Exception:
            pass
    if hasattr(obj, "_asdict"):
        try:
            return obj._asdict()
        except Exception:
            pass
    try:
        text = json.dumps(obj, ensure_ascii=False, default=lambda o: asdict(o) if is_dataclass(o) else str(o))
        return json.loads(text)
    except Exception:
        return {}


def save_agent_status_snapshot(agent_status_obj: Any) -> None:
    """
    Persist the latest 0401 AgentStatus payload for other modules (e.g., mission planning) to consume.
    """
    payload = _to_plain_dict(agent_status_obj)
    if not payload:
        return

    agent_states = payload.get("agentStateList") or payload.get("agent_states") or []
    wrapper = {
        "saved_at": datetime.now(timezone.utc).isoformat(),
        "agent_states": agent_states,
        "raw": payload,
    }

    try:
        if callable(_update_attack_tracking_state):
            _update_attack_tracking_state(agent_states)
    except Exception:
        pass

    path = _snapshot_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        tmp.write_text(json.dumps(wrapper, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(path)
    except Exception:
        # Persistence failure should not break the monitoring module.
        return


def append_agent_status_log(agent_status_obj: Any, *, source: str = "SIM") -> Optional[Path]:
    """
    Append one 0401 payload as a JSONL record.
    Intended for high-frequency simulation logging.
    """
    payload = _to_plain_dict(agent_status_obj)
    if not payload:
        return None

    agent_states = payload.get("agentStateList") or payload.get("agent_states") or []
    record = {
        "logged_at": datetime.now(timezone.utc).isoformat(),
        "source": str(source or "SIM"),
        "timestamp": payload.get("timestamp"),
        "agentStateList": agent_states,
        "raw": payload,
    }

    try:
        if callable(_update_attack_tracking_state):
            _update_attack_tracking_state(agent_states)
    except Exception:
        pass

    path = _log_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")))
            handle.write("\n")
        return path
    except Exception:
        return None


def load_agent_status_snapshot() -> Optional[Dict[str, Any]]:
    """
    Load the most recent agent-status snapshot captured from 0401.
    Returns None when the snapshot file is missing or unreadable.
    """
    path = _snapshot_path()
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
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
