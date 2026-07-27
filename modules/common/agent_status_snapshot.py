from __future__ import annotations

import json
import threading
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional

import os
import time
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

try:
    from modules.mission_planning.runtime.attack_tracking_state import (
        update_from_agent_states as _update_attack_tracking_state,
    )
except Exception:
    _update_attack_tracking_state = None

try:
    from modules.mission_planning.runtime.prior_tracking_state import (
        update_from_agent_states as _update_prior_tracking_state,
    )
except Exception:
    _update_prior_tracking_state = None

SNAPSHOT_FILENAME = "latest_0401_agent_status.json"
LOG_FILENAME = "log_0401_agent_status_sim.jsonl"
SIM_0401_LOG_DIRNAME = "simlog_0401"
SIM_0401_LOG_BASENAME = "0401"
SIM_0401_LOG_MAX_BYTES = 5 * 1024 * 1024
_sim_0401_json_log_lock = threading.Lock()


def _snapshot_path() -> Path:
    base = _normalize_path(db_paths.get_active_db_root())
    return base / "DSS_Internal" / SNAPSHOT_FILENAME


def _log_path() -> Path:
    base = _normalize_path(db_paths.get_active_db_root())
    return base / "DSS_Internal" / LOG_FILENAME


def _sim_0401_log_dir() -> Path:
    override = os.getenv("KU_SIM_0401_LOG_DIR")
    if override:
        return _normalize_path(Path(override))
    base = _normalize_path(db_paths.get_active_db_root())
    return base / SIM_0401_LOG_DIRNAME


def _sim_0401_log_max_bytes() -> int:
    raw = os.getenv("KU_SIM_0401_LOG_MAX_BYTES")
    if raw:
        try:
            value = int(float(raw))
            if value > 0:
                return value
        except Exception:
            pass
    return SIM_0401_LOG_MAX_BYTES


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


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _extract_aircraft_id(entry: Any) -> Optional[int]:
    if not isinstance(entry, dict):
        return None
    return _to_int(entry.get("aircraftID") or entry.get("aircraftId"))


def _extract_current_waypoint(entry: Any) -> Optional[int]:
    if not isinstance(entry, dict):
        return None
    for container in (entry, entry.get("unmannedInfo"), entry.get("flightMode")):
        if not isinstance(container, dict):
            continue
        for key in ("currentWaypointID", "CurrentWaypointID", "currentWaypointId"):
            if key not in container:
                continue
            value = container.get(key)
            if isinstance(value, dict):
                for sub_key in ("waypointID", "WaypointID", "waypointId"):
                    if sub_key in value:
                        waypoint_id = _to_int(value.get(sub_key))
                        if waypoint_id is not None and waypoint_id > 0:
                            return waypoint_id
                continue
            waypoint_id = _to_int(value)
            if waypoint_id is not None and waypoint_id > 0:
                return waypoint_id
    waypoint_id = None
    if waypoint_id is not None and waypoint_id <= 0:
        return None
    return waypoint_id


def _normalize_waypoint_memory(value: Any) -> Dict[str, int]:
    if not isinstance(value, dict):
        return {}
    normalized: Dict[str, int] = {}
    for key, item in value.items():
        aid = _to_int(key)
        waypoint_id = _to_int(item)
        if aid is None or waypoint_id is None or waypoint_id <= 0:
            continue
        normalized[str(aid)] = int(waypoint_id)
    return normalized


def _build_last_nonzero_waypoint_memory(
    agent_states: Any,
    *,
    previous_memory: Optional[Dict[str, int]] = None,
) -> Dict[str, int]:
    memory = dict(previous_memory or {})
    if not isinstance(agent_states, list):
        return memory
    for entry in agent_states:
        aircraft_id = _extract_aircraft_id(entry)
        if aircraft_id is None:
            continue
        current_wp = _extract_current_waypoint(entry)
        if current_wp is None or current_wp <= 0:
            continue
        memory[str(aircraft_id)] = int(current_wp)
    return memory


def save_agent_status_snapshot(agent_status_obj: Any) -> None:
    """
    Persist the latest 0401 AgentStatus payload for other modules (e.g., mission planning) to consume.
    """
    perf_start = replan_perf.start_timer()
    payload = _to_plain_dict(agent_status_obj)
    if not payload:
        replan_perf.add_elapsed("common.agent_status_snapshot.save", perf_start, empty_payload=1)
        return

    agent_states = payload.get("agentStateList") or payload.get("agent_states") or []
    previous_start = time.perf_counter() if replan_perf.is_enabled() else None
    previous_snapshot = load_agent_status_snapshot() or {}
    previous_ms = 0.0
    if previous_start is not None:
        previous_ms = (time.perf_counter() - previous_start) * 1000.0
    previous_memory = _normalize_waypoint_memory(previous_snapshot.get("last_nonzero_waypoint_by_aircraft"))
    waypoint_memory = _build_last_nonzero_waypoint_memory(
        agent_states,
        previous_memory=previous_memory,
    )
    wrapper = {
        "saved_at": datetime.now(timezone.utc).isoformat(),
        "agent_states": agent_states,
        "last_nonzero_waypoint_by_aircraft": waypoint_memory,
        "raw": payload,
    }

    update_start = time.perf_counter() if replan_perf.is_enabled() else None
    try:
        if callable(_update_attack_tracking_state):
            _update_attack_tracking_state(agent_states)
    except Exception:
        pass
    try:
        if callable(_update_prior_tracking_state):
            _update_prior_tracking_state(agent_states)
    except Exception:
        pass
    update_ms = 0.0
    if update_start is not None:
        update_ms = (time.perf_counter() - update_start) * 1000.0

    path = _snapshot_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        text = json.dumps(wrapper, ensure_ascii=False, separators=(",", ":"))
        tmp.write_text(text, encoding="utf-8")
        tmp.replace(path)
        replan_perf.add_elapsed(
            "common.agent_status_snapshot.save",
            perf_start,
            state_count=len(agent_states) if isinstance(agent_states, list) else 0,
            previous_read_ms=previous_ms,
            tracking_update_ms=update_ms,
            write_chars=len(text),
            written=1,
        )
    except Exception:
        # Persistence failure should not break the monitoring module.
        replan_perf.add_elapsed(
            "common.agent_status_snapshot.save",
            perf_start,
            state_count=len(agent_states) if isinstance(agent_states, list) else 0,
            previous_read_ms=previous_ms,
            tracking_update_ms=update_ms,
            error=1,
        )
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
    try:
        if callable(_update_prior_tracking_state):
            _update_prior_tracking_state(agent_states)
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


def append_agent_status_json_log(agent_status_obj: Any) -> Optional[Path]:
    """
    Append one generated 0401 payload to simlog_0401 as rotating JSON-array files.
    The file layout mirrors the middleware logs: 0401.json, 0401_1.json, ...
    """
    payload = _to_plain_dict(agent_status_obj)
    if not payload:
        return None

    try:
        with _sim_0401_json_log_lock:
            log_dir = _sim_0401_log_dir()
            log_dir.mkdir(parents=True, exist_ok=True)
            max_bytes = _sim_0401_log_max_bytes()
            path, index = _select_sim_0401_log_file(log_dir, max_bytes)
            try:
                _append_json_array_record(path, payload)
            except Exception:
                index += 1
                path = _sim_0401_log_file(log_dir, index)
                _write_json_array_file(path, payload)
            return path
    except Exception:
        return None


def load_agent_status_snapshot() -> Optional[Dict[str, Any]]:
    """
    Load the most recent agent-status snapshot captured from 0401.
    Returns None when the snapshot file is missing or unreadable.
    """
    perf_start = replan_perf.start_timer()
    path = _snapshot_path()
    if not path.exists():
        replan_perf.add_elapsed("common.agent_status_snapshot.load", perf_start, missing=1)
        return None
    try:
        text = path.read_text(encoding="utf-8")
        payload = json.loads(text)
        replan_perf.add_elapsed(
            "common.agent_status_snapshot.load",
            perf_start,
            loaded=1,
            read_chars=len(text),
        )
        return payload
    except Exception:
        replan_perf.add_elapsed("common.agent_status_snapshot.load", perf_start, error=1)
        return None


def _select_sim_0401_log_file(log_dir: Path, max_bytes: int) -> tuple[Path, int]:
    latest_path, latest_index = _latest_sim_0401_log_file(log_dir)
    if latest_path is None:
        return _sim_0401_log_file(log_dir, 0), 0
    try:
        if latest_path.stat().st_size >= max_bytes:
            latest_index += 1
            return _sim_0401_log_file(log_dir, latest_index), latest_index
    except Exception:
        latest_index += 1
        return _sim_0401_log_file(log_dir, latest_index), latest_index
    return latest_path, latest_index


def _latest_sim_0401_log_file(log_dir: Path) -> tuple[Optional[Path], int]:
    latest_path: Optional[Path] = None
    latest_index = -1
    for path in log_dir.glob(f"{SIM_0401_LOG_BASENAME}*.json"):
        index = _sim_0401_log_index(path)
        if index is None:
            continue
        if index > latest_index:
            latest_path = path
            latest_index = index
    return latest_path, latest_index


def _sim_0401_log_index(path: Path) -> Optional[int]:
    stem = path.stem
    if stem == SIM_0401_LOG_BASENAME:
        return 0
    prefix = f"{SIM_0401_LOG_BASENAME}_"
    if stem.startswith(prefix):
        suffix = stem[len(prefix):]
        if suffix.isdigit():
            return int(suffix)
    return None


def _sim_0401_log_file(log_dir: Path, index: int) -> Path:
    if index <= 0:
        return log_dir / f"{SIM_0401_LOG_BASENAME}.json"
    return log_dir / f"{SIM_0401_LOG_BASENAME}_{int(index)}.json"


def _format_json_array_record(record: Dict[str, Any]) -> str:
    text = json.dumps(record, ensure_ascii=False, indent=2)
    return "  " + text.replace("\n", "\n  ")


def _write_json_array_file(path: Path, record: Dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    body = _format_json_array_record(record)
    path.write_text(f"[\n{body}\n]\n", encoding="utf-8")


def _append_json_array_record(path: Path, record: Dict[str, Any]) -> None:
    if not path.exists() or path.stat().st_size <= 0:
        _write_json_array_file(path, record)
        return

    body = _format_json_array_record(record).encode("utf-8")
    with path.open("rb+") as handle:
        end_pos = _find_json_array_end(handle)
        has_records = _json_array_has_records(handle, end_pos)
        handle.seek(end_pos)
        handle.truncate()
        if has_records:
            handle.write(b",\n")
        else:
            handle.write(b"\n")
        handle.write(body)
        handle.write(b"\n]\n")


def _find_json_array_end(handle) -> int:
    handle.seek(0, os.SEEK_END)
    pos = handle.tell() - 1
    while pos >= 0:
        handle.seek(pos)
        char = handle.read(1)
        if char in b" \r\n\t":
            pos -= 1
            continue
        if char != b"]":
            raise ValueError("JSON array log does not end with ]")
        return pos
    raise ValueError("Empty JSON array log")


def _json_array_has_records(handle, end_pos: int) -> bool:
    if end_pos <= 0:
        return False
    handle.seek(0)
    chunk = handle.read(min(end_pos, 256))
    stripped = chunk.strip()
    return stripped not in (b"", b"[")


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
