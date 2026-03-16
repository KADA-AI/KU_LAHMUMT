from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Optional

try:
    from modules.common import db_paths
except ModuleNotFoundError:
    _root = Path(__file__).resolve().parents[3]
    _root_str = str(_root)
    if _root_str not in sys.path:
        sys.path.insert(0, _root_str)
    from modules.common import db_paths

_STATE_FILENAME = "attack_assignment_state.json"
_KEY_LAST_MANNED = "last_manned_aircraft_id"
_KEY_USED_BY_INPUT = "used_manned_by_input_package"
_KEY_PENDING_BY_PLAN = "pending_manned_by_plan_id"


def _to_int(value: object) -> Optional[int]:
    try:
        return int(value) if value is not None else None
    except Exception:
        return None


def _load_state() -> dict:
    path = db_paths.get_db_subpath("DSS_Internal") / _STATE_FILENAME
    if not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _save_state(payload: dict) -> None:
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / _STATE_FILENAME
    try:
        path.write_text(json.dumps(payload, ensure_ascii=True, indent=2), encoding="utf-8")
    except Exception:
        pass


def get_last_assigned_manned_id() -> Optional[int]:
    data = _load_state()
    value = data.get(_KEY_LAST_MANNED) if isinstance(data, dict) else None
    try:
        return int(value) if value is not None else None
    except Exception:
        return None


def set_last_assigned_manned_id(aircraft_id: Optional[int]) -> None:
    if aircraft_id is None:
        return
    try:
        aircraft_id_int = int(aircraft_id)
    except Exception:
        return
    data = _load_state()
    if not isinstance(data, dict):
        data = {}
    data[_KEY_LAST_MANNED] = aircraft_id_int
    _save_state(data)


def get_used_manned_ids(input_package_id: Optional[int]) -> set[int]:
    if input_package_id is None:
        return set()
    try:
        key = str(int(input_package_id))
    except Exception:
        return set()
    data = _load_state()
    used_map = data.get(_KEY_USED_BY_INPUT)
    if not isinstance(used_map, dict):
        return set()
    raw = used_map.get(key)
    if raw is None:
        return set()
    used: set[int] = set()
    for item in raw if isinstance(raw, (list, tuple, set)) else [raw]:
        try:
            used.add(int(item))
        except Exception:
            continue
    return used


def mark_manned_used(input_package_id: Optional[int], aircraft_id: Optional[int]) -> None:
    if input_package_id is None or aircraft_id is None:
        return
    try:
        key = str(int(input_package_id))
        aircraft_id_int = int(aircraft_id)
    except Exception:
        return
    data = _load_state()
    if not isinstance(data, dict):
        data = {}
    used_map = data.get(_KEY_USED_BY_INPUT)
    if not isinstance(used_map, dict):
        used_map = {}
    raw = used_map.get(key)
    used_list: list[int] = []
    if isinstance(raw, (list, tuple, set)):
        for item in raw:
            try:
                used_list.append(int(item))
            except Exception:
                continue
    elif raw is not None:
        try:
            used_list.append(int(raw))
        except Exception:
            pass
    if aircraft_id_int not in used_list:
        used_list.append(aircraft_id_int)
    used_map[key] = used_list
    data[_KEY_USED_BY_INPUT] = used_map
    _save_state(data)


def has_available_manned(input_package_id: Optional[int], *, candidates: tuple[int, ...] = (2, 3)) -> bool:
    if input_package_id is None:
        return True
    used = get_used_manned_ids(input_package_id)
    for candidate in candidates:
        if candidate not in used:
            return True
    return False


def set_pending_manned_assignment(
    mission_plan_id: Optional[int],
    input_package_id: Optional[int],
    aircraft_id: Optional[int],
) -> None:
    plan_id = _to_int(mission_plan_id)
    package_id = _to_int(input_package_id)
    aircraft_id_int = _to_int(aircraft_id)
    if plan_id is None or package_id is None or aircraft_id_int is None:
        return
    data = _load_state()
    if not isinstance(data, dict):
        data = {}
    pending_map = data.get(_KEY_PENDING_BY_PLAN)
    if not isinstance(pending_map, dict):
        pending_map = {}
    pending_map[str(plan_id)] = {
        "mission_plan_id": int(plan_id),
        "input_package_id": int(package_id),
        "aircraft_id": int(aircraft_id_int),
    }
    data[_KEY_PENDING_BY_PLAN] = pending_map
    _save_state(data)


def commit_pending_manned_assignment(mission_plan_id: Optional[int]) -> Optional[int]:
    plan_id = _to_int(mission_plan_id)
    if plan_id is None:
        return None
    data = _load_state()
    pending_map = data.get(_KEY_PENDING_BY_PLAN)
    if not isinstance(pending_map, dict):
        return None
    entry = pending_map.pop(str(plan_id), None)
    if not isinstance(entry, dict):
        if pending_map:
            data[_KEY_PENDING_BY_PLAN] = pending_map
        else:
            data.pop(_KEY_PENDING_BY_PLAN, None)
        _save_state(data)
        return None

    package_id = _to_int(entry.get("input_package_id"))
    aircraft_id = _to_int(entry.get("aircraft_id"))
    if package_id is None or aircraft_id is None:
        if pending_map:
            data[_KEY_PENDING_BY_PLAN] = pending_map
        else:
            data.pop(_KEY_PENDING_BY_PLAN, None)
        _save_state(data)
        return None

    used_map = data.get(_KEY_USED_BY_INPUT)
    if not isinstance(used_map, dict):
        used_map = {}
    raw = used_map.get(str(package_id))
    used_list: list[int] = []
    if isinstance(raw, (list, tuple, set)):
        for item in raw:
            item_int = _to_int(item)
            if item_int is not None:
                used_list.append(item_int)
    else:
        item_int = _to_int(raw)
        if item_int is not None:
            used_list.append(item_int)
    if aircraft_id not in used_list:
        used_list.append(aircraft_id)
    used_map[str(package_id)] = used_list
    data[_KEY_USED_BY_INPUT] = used_map

    if pending_map:
        data[_KEY_PENDING_BY_PLAN] = pending_map
    else:
        data.pop(_KEY_PENDING_BY_PLAN, None)
    _save_state(data)
    return aircraft_id
