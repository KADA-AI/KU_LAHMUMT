from __future__ import annotations

import json
from datetime import datetime, timezone
from typing import Any, Dict, Optional

from modules.common import db_paths

_STATE_FILENAME = "attack_tracking_state.json"


def _state_path():
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    return directory / _STATE_FILENAME


def _load_state() -> dict:
    path = _state_path()
    if not path.exists():
        return {"assignments": {}}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
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
    try:
        path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    except Exception:
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
    lat = _to_float(value.get("latitude") or value.get("lat"))
    lon = _to_float(value.get("longitude") or value.get("lon"))
    alt = _to_float(value.get("altitude") or value.get("alt"))
    if lat is None or lon is None:
        return None
    result = {"latitude": lat, "longitude": lon}
    if alt is not None:
        result["altitude"] = alt
    return result


def _extract_current_waypoint(entry: Any) -> Optional[int]:
    if not isinstance(entry, dict):
        return None
    wp_block = entry.get("currentWaypointID") or {}
    if not wp_block:
        wp_block = (entry.get("unmannedInfo") or {}).get("currentWaypointID") or {}
    waypoint_id = _to_int((wp_block or {}).get("waypointID"))
    if waypoint_id is not None and waypoint_id <= 0:
        return None
    return waypoint_id


def _extract_coordinate(entry: Any) -> Optional[Dict[str, float]]:
    if not isinstance(entry, dict):
        return None
    return (
        _normalize_coord(entry.get("coordinate"))
        or _normalize_coord((entry.get("unmannedInfo") or {}).get("coordinate"))
        or _normalize_coord((entry.get("mannedInfo") or {}).get("coordinate"))
    )


def register_tracking_assignment(
    *,
    aircraft_id: int,
    source_plan_id: int,
    attack_plan_id: int,
    original_path_id: int,
    original_individual_mission_id: int,
    original_current_waypoint_id: Optional[int],
    original_coordinate: Optional[Dict[str, Any]],
    tracking_path_id: Optional[int],
    tracking_individual_mission_id: Optional[int],
    resume_path_id: Optional[int],
    resume_individual_mission_id: Optional[int],
    target_id: Optional[int],
) -> None:
    aircraft_key = str(int(aircraft_id))
    data = _load_state()
    assignments = data.setdefault("assignments", {})
    coord = _normalize_coord(original_coordinate)
    assignments[aircraft_key] = {
        "aircraft_id": int(aircraft_id),
        "active": True,
        "source_plan_id": int(source_plan_id),
        "attack_plan_id": int(attack_plan_id),
        "original_path_id": int(original_path_id),
        "original_individual_mission_id": int(original_individual_mission_id),
        "original_current_waypoint_id": _to_int(original_current_waypoint_id),
        "last_nonzero_waypoint_id": _to_int(original_current_waypoint_id),
        "last_nonzero_coordinate": coord,
        "handoff_waypoint_id": None,
        "handoff_coordinate": None,
        "tracking_path_id": _to_int(tracking_path_id),
        "tracking_individual_mission_id": _to_int(tracking_individual_mission_id),
        "resume_path_id": _to_int(resume_path_id),
        "resume_individual_mission_id": _to_int(resume_individual_mission_id),
        "target_id": _to_int(target_id),
        "auto_tracking_engaged": False,
        "registered_at": _now_iso(),
        "updated_at": _now_iso(),
    }
    _save_state(data)


def get_tracking_assignment(aircraft_id: Optional[int]) -> Optional[Dict[str, Any]]:
    aid = _to_int(aircraft_id)
    if aid is None:
        return None
    data = _load_state()
    assignments = data.get("assignments")
    if not isinstance(assignments, dict):
        return None
    entry = assignments.get(str(aid))
    return dict(entry) if isinstance(entry, dict) else None


def clear_tracking_assignment(aircraft_id: Optional[int]) -> None:
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
        aircraft_id = _to_int(entry.get("aircraftID") or entry.get("aircraftId"))
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

        if current_wp is not None:
            if _to_int(assignment.get("last_nonzero_waypoint_id")) != current_wp:
                assignment["last_nonzero_waypoint_id"] = current_wp
                assignment_changed = True
            if current_coord is not None:
                assignment["last_nonzero_coordinate"] = current_coord
                assignment_changed = True
        else:
            handoff_wp = _to_int(assignment.get("handoff_waypoint_id"))
            fallback_wp = _to_int(assignment.get("last_nonzero_waypoint_id")) or _to_int(
                assignment.get("original_current_waypoint_id")
            )
            if handoff_wp is None and fallback_wp is not None:
                assignment["handoff_waypoint_id"] = fallback_wp
                assignment["auto_tracking_engaged"] = True
                assignment["handoff_at"] = _now_iso()
                assignment_changed = True
                if current_coord is not None:
                    assignment["handoff_coordinate"] = current_coord
                elif _normalize_coord(assignment.get("last_nonzero_coordinate")) is not None:
                    assignment["handoff_coordinate"] = _normalize_coord(
                        assignment.get("last_nonzero_coordinate")
                    )
                assignment_changed = True

        if assignment_changed:
            assignment["updated_at"] = _now_iso()
            assignments[aircraft_key] = assignment
            changed = True

    if changed:
        _save_state(data)
