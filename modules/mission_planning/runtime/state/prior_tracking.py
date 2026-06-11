from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from modules.common import db_paths

_STATE_FILENAME = "prior_tracking_state.json"


def _state_path():
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    return directory / _STATE_FILENAME


def _load_state() -> dict:
    path = _state_path()
    if not path.exists():
        return {"assignments": {}}
    try:
        text = path.read_text(encoding="utf-8")
        data = json.loads(text)
    except Exception:
        try:
            text = path.read_text(encoding="utf-8")
            data, end = json.JSONDecoder().raw_decode(text)
            if str(text[end:]).strip():
                _save_state(data)
        except Exception:
            return {"assignments": {}}
    if not isinstance(data, dict):
        return {"assignments": {}}
    assignments = data.get("assignments")
    if not isinstance(assignments, dict):
        data["assignments"] = {}
    return data


def _save_state(payload: dict) -> None:
    path = _state_path()
    tmp = path.with_name(f"{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
        tmp.replace(path)
    except Exception:
        try:
            tmp.unlink(missing_ok=True)
        except Exception:
            pass
        pass


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _normalize_coord(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    def _pick(*keys: str) -> Any:
        for key in keys:
            if key in value:
                return value.get(key)
        return None

    lat = _to_float(_pick("latitude", "Latitude", "lat"))
    lon = _to_float(_pick("longitude", "Longitude", "lon"))
    alt = _to_float(_pick("altitude", "Altitude", "alt"))
    if lat is None or lon is None:
        return None
    result = {"latitude": float(lat), "longitude": float(lon)}
    if alt is not None:
        result["altitude"] = float(alt)
    return result


def _extract_aircraft_id(entry: Any) -> Optional[int]:
    if not isinstance(entry, dict):
        return None
    return _to_int(
        entry.get("aircraft_id")
        or entry.get("aircraftID")
        or entry.get("AircraftID")
    )


def _extract_current_waypoint(entry: Any) -> Optional[int]:
    if not isinstance(entry, dict):
        return None
    for container in (entry, entry.get("unmannedInfo"), entry.get("flightMode")):
        if not isinstance(container, dict):
            continue
        for key in ("current_waypoint_id", "currentWaypointID", "CurrentWaypointID", "currentWaypointId"):
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
    return None


def _extract_coordinate(entry: Any) -> Optional[Dict[str, float]]:
    if not isinstance(entry, dict):
        return None
    return (
        _normalize_coord(entry.get("coordinate"))
        or _normalize_coord(entry.get("Coordinate"))
        or _normalize_coord((entry.get("unmannedInfo") or {}).get("coordinate"))
        or _normalize_coord((entry.get("unmannedInfo") or {}).get("Coordinate"))
        or _normalize_coord((entry.get("mannedInfo") or {}).get("coordinate"))
        or _normalize_coord((entry.get("mannedInfo") or {}).get("Coordinate"))
    )


def _normalize_int_list(value: Any) -> List[int]:
    source = value if isinstance(value, (list, tuple, set)) else []
    out: List[int] = []
    seen: set[int] = set()
    for item in source:
        parsed = _to_int(item)
        if parsed is None or parsed <= 0 or parsed in seen:
            continue
        seen.add(int(parsed))
        out.append(int(parsed))
    return out


def register_prior_assignment(
    *,
    aircraft_id: int,
    source_plan_id: int,
    prior_plan_id: int,
    current_input_mission_id: Optional[int],
    original_path_id: int,
    original_individual_mission_id: int,
    original_current_waypoint_id: Optional[int],
    original_coordinate: Optional[Dict[str, Any]],
    prior_path_id: Optional[int],
    prior_individual_mission_id: Optional[int],
    prior_waypoint_ids: Any,
    resume_path_id: Optional[int],
    resume_individual_mission_id: Optional[int],
    resume_first_waypoint_id: Optional[int],
    resume_first_active_waypoint_id: Optional[int],
    resume_waypoint_ids: Any,
    prior_mission_id: Optional[int],
    mission_type: Optional[int],
    target_id: Optional[int],
    target_coordinate: Optional[Dict[str, Any]],
) -> None:
    aircraft_key = str(int(aircraft_id))
    data = _load_state()
    assignments = data.setdefault("assignments", {})
    coord = _normalize_coord(original_coordinate)
    target_coord = _normalize_coord(target_coordinate)
    assignments[aircraft_key] = {
        "aircraft_id": int(aircraft_id),
        "active": True,
        "source_plan_id": int(source_plan_id),
        "prior_plan_id": int(prior_plan_id),
        "current_input_mission_id": _to_int(current_input_mission_id),
        "original_path_id": int(original_path_id),
        "original_individual_mission_id": int(original_individual_mission_id),
        "original_current_waypoint_id": _to_int(original_current_waypoint_id),
        "original_coordinate": coord,
        "last_nonzero_waypoint_id": _to_int(original_current_waypoint_id),
        "last_nonzero_coordinate": coord,
        "handoff_waypoint_id": None,
        "handoff_coordinate": None,
        "prior_path_id": _to_int(prior_path_id),
        "prior_individual_mission_id": _to_int(prior_individual_mission_id),
        "prior_waypoint_ids": _normalize_int_list(prior_waypoint_ids),
        "resume_path_id": _to_int(resume_path_id),
        "resume_individual_mission_id": _to_int(resume_individual_mission_id),
        "resume_first_waypoint_id": _to_int(resume_first_waypoint_id),
        "resume_first_active_waypoint_id": _to_int(resume_first_active_waypoint_id),
        "resume_waypoint_ids": _normalize_int_list(resume_waypoint_ids),
        "prior_mission_id": _to_int(prior_mission_id),
        "mission_type": _to_int(mission_type),
        "target_id": _to_int(target_id),
        "target_coordinate": target_coord,
        "registered_at": _now_iso(),
        "updated_at": _now_iso(),
    }
    _save_state(data)


def get_prior_assignment(aircraft_id: Optional[int]) -> Optional[Dict[str, Any]]:
    aid = _to_int(aircraft_id)
    if aid is None:
        return None
    data = _load_state()
    assignments = data.get("assignments")
    if not isinstance(assignments, dict):
        return None
    entry = assignments.get(str(aid))
    return dict(entry) if isinstance(entry, dict) else None


def list_active_prior_assignments() -> List[Dict[str, Any]]:
    data = _load_state()
    assignments = data.get("assignments")
    if not isinstance(assignments, dict):
        return []
    active: List[Dict[str, Any]] = []
    for entry in assignments.values():
        if not isinstance(entry, dict):
            continue
        if not bool(entry.get("active")):
            continue
        active.append(dict(entry))
    return active


def clear_prior_assignment(aircraft_id: Optional[int], *, reason: str | None = None) -> None:
    aid = _to_int(aircraft_id)
    if aid is None:
        return
    data = _load_state()
    assignments = data.get("assignments")
    if not isinstance(assignments, dict):
        return
    entry = assignments.get(str(aid))
    if not isinstance(entry, dict):
        return
    entry["active"] = False
    entry["cleared_at"] = _now_iso()
    if reason:
        entry["clear_reason"] = str(reason)
    assignments[str(aid)] = entry
    _save_state(data)


def mark_prior_handoff(
    aircraft_id: Optional[int],
    *,
    handoff_waypoint_id: Optional[int],
    handoff_coordinate: Optional[Dict[str, Any]],
) -> None:
    aid = _to_int(aircraft_id)
    if aid is None:
        return
    data = _load_state()
    assignments = data.get("assignments")
    if not isinstance(assignments, dict):
        return
    entry = assignments.get(str(aid))
    if not isinstance(entry, dict) or not bool(entry.get("active")):
        return
    waypoint_id = _to_int(handoff_waypoint_id)
    coordinate = _normalize_coord(handoff_coordinate)
    changed = False
    if waypoint_id is not None and waypoint_id > 0:
        entry["handoff_waypoint_id"] = int(waypoint_id)
        changed = True
    if coordinate is not None:
        entry["handoff_coordinate"] = coordinate
        changed = True
    if changed:
        entry["updated_at"] = _now_iso()
        assignments[str(aid)] = entry
        _save_state(data)


def update_from_agent_states(agent_states: Any) -> None:
    if not isinstance(agent_states, list):
        return
    data = _load_state()
    assignments = data.get("assignments")
    if not isinstance(assignments, dict) or not assignments:
        return

    changed = False
    index: Dict[str, Dict[str, Any]] = {}
    for entry in agent_states:
        if not isinstance(entry, dict):
            continue
        aircraft_id = _extract_aircraft_id(entry)
        if aircraft_id is None:
            continue
        index[str(aircraft_id)] = entry

    for aircraft_key, assignment in list(assignments.items()):
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        state_entry = index.get(str(aircraft_key))
        if not isinstance(state_entry, dict):
            continue
        assignment_changed = False
        current_wp = _extract_current_waypoint(state_entry)
        current_coord = _extract_coordinate(state_entry)

        if current_wp is not None and current_wp > 0:
            if _to_int(assignment.get("last_nonzero_waypoint_id")) != int(current_wp):
                assignment["last_nonzero_waypoint_id"] = int(current_wp)
                assignment_changed = True
            if current_coord is not None:
                assignment["last_nonzero_coordinate"] = current_coord
                assignment_changed = True

        if assignment_changed:
            assignment["updated_at"] = _now_iso()
            assignments[aircraft_key] = assignment
            changed = True

    if changed:
        _save_state(data)
