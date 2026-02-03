from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Optional

try:
    from modules.common import db_paths
except ModuleNotFoundError:
    _root = Path(__file__).resolve().parents[2]
    _root_str = str(_root)
    if _root_str not in sys.path:
        sys.path.insert(0, _root_str)
    from modules.common import db_paths

_STATE_FILENAME = "attack_assignment_state.json"
_KEY_LAST_MANNED = "last_manned_aircraft_id"
_KEY_USED_BY_INPUT = "used_manned_by_input_package"


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
