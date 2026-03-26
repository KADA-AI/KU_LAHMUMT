from __future__ import annotations

import json
import math
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from modules.common import db_paths

_COORD_MATCH_RADIUS_M = float(os.getenv("MSM_PRIOR_COORD_REDISCOVERY_RADIUS_M", "500.0"))
_LOCK = threading.Lock()


def _base_dir() -> Path:
    raw = db_paths.get_db_subpath("DSS_Internal", "prior_target_rediscovery")
    return _normalize_path(raw)


def _state_path() -> Path:
    return _base_dir() / "state.json"


def _normalize_path(path: Path) -> Path:
    if os.name == "nt":
        return Path(str(path))
    text = str(path)
    if len(text) >= 2 and text[1] == ":":
        drive = text[0].lower()
        rest = text[2:].replace("\\", "/").lstrip("/")
        return Path(f"/mnt/{drive}/{rest}")
    return Path(text)


def _to_int(value: Any) -> Optional[int]:
    try:
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> Optional[float]:
    try:
        return float(value)
    except Exception:
        return None


def _normalize_coordinate(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    lat = _to_float(value.get("latitude"))
    lon = _to_float(value.get("longitude"))
    if lat is None or lon is None:
        return None
    coord: Dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    alt = _to_float(value.get("altitude"))
    if alt is not None:
        coord["altitude"] = float(alt)
    return coord


def _sanitize_stage(stage: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("_", "-") else "_" for ch in str(stage or "event"))


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _haversine_m(left: Dict[str, float], right: Dict[str, float]) -> float:
    lat1 = math.radians(float(left["latitude"]))
    lon1 = math.radians(float(left["longitude"]))
    lat2 = math.radians(float(right["latitude"]))
    lon2 = math.radians(float(right["longitude"]))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return 6_371_000.0 * c


def _load_state_unlocked() -> Dict[str, Any]:
    path = _state_path()
    if not path.exists():
        return {"entries": {}}
    try:
        raw = path.read_text(encoding="utf-8")
        data = json.loads(raw) if raw else {}
    except Exception:
        data = {}
    entries_raw = data.get("entries")
    if not isinstance(entries_raw, dict):
        entries_raw = {}
    normalized: Dict[str, Dict[str, Any]] = {}
    for key, value in entries_raw.items():
        if not isinstance(value, dict):
            continue
        prior_mission_id = _to_int(value.get("priorMissionID"))
        if prior_mission_id is None:
            prior_mission_id = _to_int(key)
        if prior_mission_id is None or prior_mission_id <= 0:
            continue
        entry = dict(value)
        entry["priorMissionID"] = int(prior_mission_id)
        mission_type = _to_int(entry.get("missionType"))
        if mission_type is not None:
            entry["missionType"] = int(mission_type)
        else:
            entry.pop("missionType", None)
        target_id = _to_int(entry.get("targetID"))
        entry["targetID"] = int(target_id) if target_id is not None and target_id > 0 else None
        coord = _normalize_coordinate(entry.get("coordinate"))
        if coord:
            entry["coordinate"] = coord
        else:
            entry.pop("coordinate", None)
        last_coord = _normalize_coordinate(entry.get("lastCoordinate"))
        if last_coord:
            entry["lastCoordinate"] = last_coord
        else:
            entry.pop("lastCoordinate", None)
        status = str(entry.get("status") or "armed").strip().lower()
        if status not in {"armed", "matched", "consumed"}:
            status = "armed"
        entry["status"] = status
        normalized[str(int(prior_mission_id))] = entry
    return {"entries": normalized}


def _write_state_unlocked(state: Dict[str, Any]) -> None:
    path = _state_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "entries": state.get("entries") if isinstance(state.get("entries"), dict) else {},
        "savedAt": _now_iso(),
    }
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)


def load_state() -> Dict[str, Any]:
    with _LOCK:
        return _load_state_unlocked()


def save_event(stage: str, payload: Dict[str, Any]) -> Path:
    stage_name = _sanitize_stage(stage)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    path = _base_dir() / f"{stage_name}_{timestamp}.json"
    data = dict(payload or {})
    data.setdefault("savedAt", _now_iso())
    data.setdefault("stage", stage_name)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True), encoding="utf-8")
    tmp.replace(path)
    return path


def arm_target_rediscovery(
    *,
    prior_mission_id: int,
    mission_type: int,
    timestamp: int | None = None,
    target_id: int | None = None,
    coordinate: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    prior_id = _to_int(prior_mission_id)
    mission_type_int = _to_int(mission_type)
    if prior_id is None or prior_id <= 0:
        return {}
    if mission_type_int is None or mission_type_int <= 0:
        return {}
    coord = _normalize_coordinate(coordinate)
    target_id_int = _to_int(target_id)
    target_id_norm = int(target_id_int) if target_id_int is not None and target_id_int > 0 else None
    with _LOCK:
        state = _load_state_unlocked()
        entries = state.setdefault("entries", {})
        entry = dict(entries.get(str(int(prior_id))) or {})
        entry.update(
            {
                "priorMissionID": int(prior_id),
                "missionType": int(mission_type_int),
                "targetID": target_id_norm,
                "status": "armed",
                "armedAt": int(timestamp) if timestamp is not None else None,
                "matchedAt": None,
                "consumedAt": None,
                "consumeReason": None,
                "lastTargetKey": None,
                "lastWatcherID": None,
                "lastMatchedTargetID": None,
            }
        )
        if coord:
            entry["coordinate"] = coord
        else:
            entry.pop("coordinate", None)
        existing_count = _to_int(entry.get("matchCount"))
        entry["matchCount"] = int(existing_count) if existing_count is not None and existing_count >= 0 else 0
        entries[str(int(prior_id))] = entry
        _write_state_unlocked(state)
    save_event(
        "armed",
        {
            "priorMissionID": int(prior_id),
            "missionType": int(mission_type_int),
            "targetID": target_id_norm,
            "coordinate": coord,
            "timestamp": int(timestamp) if timestamp is not None else None,
        },
    )
    return dict(entry)


def _entry_matches(
    entry: Dict[str, Any],
    *,
    target_id: int | None,
    coordinate: Dict[str, float] | None,
) -> bool:
    entry_target_id = _to_int(entry.get("targetID"))
    if entry_target_id is not None and target_id is not None and int(entry_target_id) == int(target_id):
        return True
    mission_type = _to_int(entry.get("missionType"))
    if mission_type != 1:
        return False
    entry_coord = _normalize_coordinate(entry.get("coordinate"))
    if not entry_coord or not coordinate:
        return False
    return _haversine_m(entry_coord, coordinate) <= float(_COORD_MATCH_RADIUS_M)


def match_detection(
    *,
    target_id: int | None,
    coordinate: Dict[str, Any] | None,
    watcher_id: int | None = None,
    key: str | None = None,
    timestamp: int | None = None,
) -> List[Dict[str, Any]]:
    coord = _normalize_coordinate(coordinate)
    target_id_int = _to_int(target_id)
    target_id_norm = int(target_id_int) if target_id_int is not None and target_id_int > 0 else None
    watcher_id_int = _to_int(watcher_id)
    matched: List[Dict[str, Any]] = []
    event_payloads: List[Dict[str, Any]] = []
    with _LOCK:
        state = _load_state_unlocked()
        entries = state.setdefault("entries", {})
        changed = False
        for entry in entries.values():
            if not isinstance(entry, dict):
                continue
            if str(entry.get("status") or "armed").strip().lower() == "consumed":
                continue
            if not _entry_matches(entry, target_id=target_id_norm, coordinate=coord):
                continue
            was_armed = str(entry.get("status") or "armed").strip().lower() == "armed"
            if was_armed:
                entry["status"] = "matched"
                entry["matchedAt"] = int(timestamp) if timestamp is not None else None
                entry["matchCount"] = int(_to_int(entry.get("matchCount")) or 0) + 1
                changed = True
            if timestamp is not None:
                entry["lastSeenAt"] = int(timestamp)
            if target_id_norm is not None:
                entry["lastMatchedTargetID"] = int(target_id_norm)
            if watcher_id_int is not None:
                entry["lastWatcherID"] = int(watcher_id_int)
            if key:
                entry["lastTargetKey"] = str(key)
            if coord:
                entry["lastCoordinate"] = coord
            matched.append(dict(entry))
            if was_armed:
                event_payloads.append(
                    {
                        "priorMissionID": entry.get("priorMissionID"),
                        "missionType": entry.get("missionType"),
                        "targetID": target_id_norm,
                        "watcherID": watcher_id_int,
                        "key": key,
                        "timestamp": int(timestamp) if timestamp is not None else None,
                    }
                )
        if changed:
            _write_state_unlocked(state)
    for payload in event_payloads:
        save_event("matched", payload)
    return matched


def consume_target(
    target_id: int | None,
    *,
    timestamp: int | None = None,
    reason: str | None = None,
    mission_plan_ids: List[int] | None = None,
    watcher_id: int | None = None,
    key: str | None = None,
) -> List[Dict[str, Any]]:
    target_id_int = _to_int(target_id)
    if target_id_int is None or target_id_int <= 0:
        return []
    target_id_norm = int(target_id_int)
    watcher_id_int = _to_int(watcher_id)
    consumed: List[Dict[str, Any]] = []
    with _LOCK:
        state = _load_state_unlocked()
        entries = state.setdefault("entries", {})
        changed = False
        for entry in entries.values():
            if not isinstance(entry, dict):
                continue
            if str(entry.get("status") or "armed").strip().lower() == "consumed":
                continue
            entry_target_id = _to_int(entry.get("targetID"))
            matched_target_id = _to_int(entry.get("lastMatchedTargetID"))
            if entry_target_id != target_id_norm and matched_target_id != target_id_norm:
                continue
            entry["status"] = "consumed"
            if timestamp is not None:
                entry["consumedAt"] = int(timestamp)
            if reason:
                entry["consumeReason"] = str(reason)
            if mission_plan_ids:
                entry["missionPlanIDs"] = [int(mid) for mid in mission_plan_ids if _to_int(mid) is not None]
            if watcher_id_int is not None:
                entry["lastWatcherID"] = int(watcher_id_int)
            if key:
                entry["lastTargetKey"] = str(key)
            consumed.append(dict(entry))
            changed = True
        if changed:
            _write_state_unlocked(state)
    if consumed:
        save_event(
            "consumed",
            {
                "targetID": int(target_id_norm),
                "timestamp": int(timestamp) if timestamp is not None else None,
                "reason": str(reason or ""),
                "missionPlanIDs": [int(mid) for mid in mission_plan_ids or [] if _to_int(mid) is not None],
                "watcherID": watcher_id_int,
                "key": key,
                "entries": consumed,
            },
        )
    return consumed
