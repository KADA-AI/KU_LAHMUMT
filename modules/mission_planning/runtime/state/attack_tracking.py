from __future__ import annotations

import json
import os
import threading
import time
from contextlib import contextmanager
from datetime import datetime, timezone
from threading import RLock
from typing import Any, Dict, Optional

from modules.common import db_paths

_STATE_FILENAME = "attack_tracking_state.json"
_LOCK = RLock()
_FILE_LOCK_LOCAL = threading.local()

try:  # Windows
    import msvcrt  # type: ignore
except Exception:  # pragma: no cover - non-Windows fallback
    msvcrt = None  # type: ignore

try:  # POSIX
    import fcntl  # type: ignore
except Exception:  # pragma: no cover - Windows fallback
    fcntl = None  # type: ignore


def _state_path():
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    return directory / _STATE_FILENAME


def _mission_plan_log_dir():
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _lock_file_handle(lock_file) -> None:
    if msvcrt is not None:
        # LK_LOCK sleeps for one whole second between retries on Windows.  The
        # monitoring updater holds this lock only for a tiny JSON transaction,
        # so that retry granularity used to add an artificial ~1 s to post-
        # attack replanning whenever the two operations briefly overlapped.
        # Keep the same exclusive cross-process lock, but retry non-blocking at
        # a short interval (with the same approximate 10 s upper bound).
        deadline = time.monotonic() + 10.0
        while True:
            try:
                msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
                return
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.002)
    elif fcntl is not None:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)


def _unlock_file_handle(lock_file) -> None:
    if msvcrt is not None:
        msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)
    elif fcntl is not None:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


@contextmanager
def _state_file_lock():
    """Serialize attack-tracking read/modify/write across module processes."""

    depth = int(getattr(_FILE_LOCK_LOCAL, "depth", 0) or 0)
    if depth > 0:
        _FILE_LOCK_LOCAL.depth = depth + 1
        try:
            yield
        finally:
            _FILE_LOCK_LOCAL.depth = depth
        return

    state_path = _state_path()
    lock_path = state_path.with_name(f"{state_path.name}.lock")
    lock_file = lock_path.open("a+b")
    acquired = False
    _FILE_LOCK_LOCAL.depth = 1
    try:
        lock_file.seek(0, os.SEEK_END)
        if lock_file.tell() == 0:
            lock_file.write(b"\0")
            lock_file.flush()
        lock_file.seek(0)
        _lock_file_handle(lock_file)
        acquired = True
        yield
    finally:
        try:
            if acquired:
                lock_file.seek(0)
                _unlock_file_handle(lock_file)
        finally:
            _FILE_LOCK_LOCAL.depth = 0
            lock_file.close()


def _load_state() -> dict:
    with _LOCK:
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
    with _LOCK:
        path = _state_path()
        tmp = path.with_name(f"{path.name}.{os.getpid()}.{threading.get_ident()}.tmp")
        try:
            tmp.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
            tmp.replace(path)
        except Exception:
            try:
                tmp.unlink()
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


def _load_plan_run_context(plan_id: int) -> Dict[str, Any]:
    try:
        base_dir = _mission_plan_log_dir()
    except Exception:
        return {}

    candidates = []
    primary = base_dir / f"missionPlan_{int(plan_id)}.json"
    if primary.exists():
        candidates.append(primary)
    try:
        tokened = sorted(
            base_dir.glob(f"missionPlan_{int(plan_id)}_*.json"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
    except Exception:
        tokened = []
    candidates.extend(path for path in tokened if path not in candidates)

    for path in candidates:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(payload, dict):
            continue
        context = payload.get("context")
        if isinstance(context, dict):
            return dict(context)
    return {}


def resolve_plan_lineage_ids(plan_id: Optional[int], *, max_depth: int = 12) -> set[int]:
    root_plan_id = _to_int(plan_id)
    if root_plan_id is None or root_plan_id <= 0:
        return set()

    lineage: set[int] = set()
    current_plan_id = int(root_plan_id)
    depth = 0
    while current_plan_id > 0 and current_plan_id not in lineage and depth < max_depth:
        lineage.add(int(current_plan_id))
        context = _load_plan_run_context(int(current_plan_id))
        if not isinstance(context, dict) or not context:
            break

        parent_candidates = [
            context.get("sourceMissionPlanID"),
            context.get("currentMissionPlanID"),
        ]
        detail = context.get("replan_detail")
        if isinstance(detail, dict):
            parent_candidates.extend(
                [
                    detail.get("sourceMissionPlanID"),
                    detail.get("currentMissionPlanID"),
                ]
            )

        next_plan_id: Optional[int] = None
        for candidate in parent_candidates:
            parent_id = _to_int(candidate)
            if parent_id is None or parent_id <= 0 or parent_id == int(current_plan_id):
                continue
            next_plan_id = int(parent_id)
            break
        if next_plan_id is None:
            break
        current_plan_id = int(next_plan_id)
        depth += 1
    return lineage


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
    current_input_mission_id: Optional[int],
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
    with _LOCK:
        with _state_file_lock():
            aircraft_key = str(int(aircraft_id))
            data = _load_state()
            assignments = data.setdefault("assignments", {})
            coord = _normalize_coord(original_coordinate)
            assignments[aircraft_key] = {
                "aircraft_id": int(aircraft_id),
                "active": True,
                "source_plan_id": int(source_plan_id),
                "attack_plan_id": int(attack_plan_id),
                "current_input_mission_id": _to_int(current_input_mission_id),
                "original_path_id": int(original_path_id),
                "original_individual_mission_id": int(original_individual_mission_id),
                "original_current_waypoint_id": _to_int(original_current_waypoint_id),
                "original_coordinate": coord,
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


def list_active_tracking_assignments() -> list[Dict[str, Any]]:
    data = _load_state()
    assignments = data.get("assignments")
    if not isinstance(assignments, dict):
        return []
    active: list[Dict[str, Any]] = []
    for entry in assignments.values():
        if not isinstance(entry, dict):
            continue
        if not bool(entry.get("active")):
            continue
        active.append(dict(entry))
    return active


def clear_tracking_assignments(aircraft_ids: Any) -> None:
    if isinstance(aircraft_ids, (list, tuple, set)):
        normalized_ids = {
            int(aid)
            for aid in (_to_int(value) for value in aircraft_ids)
            if aid is not None
        }
    else:
        aid = _to_int(aircraft_ids)
        normalized_ids = {int(aid)} if aid is not None else set()
    if not normalized_ids:
        return

    with _LOCK:
        with _state_file_lock():
            data = _load_state()
            assignments = data.get("assignments")
            if not isinstance(assignments, dict):
                return
            cleared_at = _now_iso()
            changed = False
            for aid in sorted(normalized_ids):
                entry = assignments.get(str(aid))
                if not isinstance(entry, dict):
                    continue
                entry["active"] = False
                entry["cleared_at"] = cleared_at
                assignments[str(aid)] = entry
                changed = True
            if changed:
                _save_state(data)


def clear_tracking_assignment(aircraft_id: Optional[int]) -> None:
    clear_tracking_assignments(aircraft_id)


def rebind_tracking_assignments_to_plan(
    *,
    old_attack_plan_id: Optional[int],
    new_attack_plan_id: Optional[int],
    aircraft_ids: Any = None,
) -> list[int]:
    old_plan_id = _to_int(old_attack_plan_id)
    new_plan_id = _to_int(new_attack_plan_id)
    if old_plan_id is None or new_plan_id is None or old_plan_id <= 0 or new_plan_id <= 0:
        return []
    if old_plan_id == new_plan_id:
        return []

    selected_aircraft_ids: set[int] = set()
    if aircraft_ids is None:
        selected_aircraft_ids = set()
    elif isinstance(aircraft_ids, (list, tuple, set)):
        for item in aircraft_ids:
            aid = _to_int(item)
            if aid is not None and aid > 0:
                selected_aircraft_ids.add(int(aid))
    else:
        aid = _to_int(aircraft_ids)
        if aid is not None and aid > 0:
            selected_aircraft_ids.add(int(aid))

    with _LOCK:
        with _state_file_lock():
            data = _load_state()
            assignments = data.get("assignments")
            if not isinstance(assignments, dict):
                return []

            rebound: list[int] = []
            changed = False
            for aircraft_key, entry in assignments.items():
                if not isinstance(entry, dict) or not bool(entry.get("active")):
                    continue
                aircraft_id = _to_int(entry.get("aircraft_id") or aircraft_key)
                if aircraft_id is None:
                    continue
                if selected_aircraft_ids and int(aircraft_id) not in selected_aircraft_ids:
                    continue
                if _to_int(entry.get("attack_plan_id")) != int(old_plan_id):
                    continue
                entry["attack_plan_id"] = int(new_plan_id)
                entry["updated_at"] = _now_iso()
                assignments[str(aircraft_id)] = entry
                rebound.append(int(aircraft_id))
                changed = True

            if changed:
                _save_state(data)
            return rebound


def update_from_agent_states(agent_states: Any) -> None:
    if not isinstance(agent_states, list):
        return
    with _LOCK:
        with _state_file_lock():
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
