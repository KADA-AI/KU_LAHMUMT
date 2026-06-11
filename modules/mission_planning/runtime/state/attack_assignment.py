from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Optional

try:
    from modules.common import db_paths
except ModuleNotFoundError:
    _root = next(
        (
            parent
            for parent in Path(__file__).resolve().parents
            if (parent / "modules" / "common").exists()
        ),
        Path(__file__).resolve().parents[4],
    )
    _root_str = str(_root)
    if _root_str not in sys.path:
        sys.path.insert(0, _root_str)
    from modules.common import db_paths

_STATE_FILENAME = "attack_assignment_state.json"
_KEY_LAST_MANNED = "last_manned_aircraft_id"
_KEY_USED_BY_INPUT = "used_manned_by_input_package"
_KEY_PENDING_BY_PLAN = "pending_manned_by_plan_id"
_KEY_DEFERRED_ATTACK_TARGETS = "deferred_attack_targets_by_input_package"


def _to_int(value: object) -> Optional[int]:
    try:
        return int(value) if value is not None else None
    except Exception:
        return None


def _normalize_aircraft_ids(values: object) -> list[int]:
    normalized: list[int] = []
    iterable = values if isinstance(values, (list, tuple, set)) else [values]
    for item in iterable:
        aircraft_id = _to_int(item)
        if aircraft_id is None or aircraft_id <= 0:
            continue
        if aircraft_id not in normalized:
            normalized.append(int(aircraft_id))
    return normalized


def _normalize_target_id(value: object) -> Optional[int]:
    target_id = _to_int(value)
    if target_id is None or target_id <= 0:
        return None
    return int(target_id)


def _normalize_deferred_target_entry(entry: object) -> dict | None:
    if not isinstance(entry, dict):
        return None
    target_id = _normalize_target_id(
        entry.get("targetID")
        or entry.get("targetId")
        or entry.get("target_id")
    )
    if target_id is None:
        return None

    normalized: dict = {"targetID": int(target_id)}
    for src_key, dst_key in (
        ("targetType", "targetType"),
        ("target_type", "targetType"),
        ("watcherID", "watcherID"),
        ("watcherId", "watcherID"),
        ("watcher_id", "watcherID"),
        ("isUsed", "isUsed"),
        ("isIgnored", "isIgnored"),
        ("isDestroyed", "isDestroyed"),
        ("targetInFrame", "targetInFrame"),
        ("threat", "threat"),
        ("firstDetected", "firstDetected"),
        ("lastUpdated", "lastUpdated"),
        ("timestamp", "timestamp"),
        ("selectionOrder", "selectionOrder"),
    ):
        if src_key in entry and entry.get(src_key) is not None:
            normalized[dst_key] = entry.get(src_key)

    key_value = entry.get("targetKey") or entry.get("key") or entry.get("sourceKey")
    if key_value:
        normalized["targetKey"] = str(key_value)
        normalized["key"] = str(key_value)

    coord = entry.get("coordinate") or entry.get("targetCoordinate")
    if isinstance(coord, dict):
        normalized["coordinate"] = dict(coord)
    return normalized


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


def release_manned_used(input_package_id: Optional[int], aircraft_ids: object = None) -> list[int]:
    if input_package_id is None:
        return []
    try:
        key = str(int(input_package_id))
    except Exception:
        return []

    data = _load_state()
    if not isinstance(data, dict):
        return []
    used_map = data.get(_KEY_USED_BY_INPUT)
    if not isinstance(used_map, dict):
        return []

    raw = used_map.get(key)
    if raw is None:
        return []
    used_list = _normalize_aircraft_ids(raw)
    if not used_list:
        used_map.pop(key, None)
        data[_KEY_USED_BY_INPUT] = used_map
        _save_state(data)
        return []

    release_ids = _normalize_aircraft_ids(aircraft_ids) if aircraft_ids is not None else list(used_list)
    if not release_ids:
        return []

    release_set = set(int(item) for item in release_ids)
    kept = [int(item) for item in used_list if int(item) not in release_set]
    released = [int(item) for item in used_list if int(item) in release_set]
    if not released:
        return []

    if kept:
        used_map[key] = kept
    else:
        used_map.pop(key, None)
    if used_map:
        data[_KEY_USED_BY_INPUT] = used_map
    else:
        data.pop(_KEY_USED_BY_INPUT, None)
    _save_state(data)
    return released


def defer_attack_targets(
    input_package_id: Optional[int],
    source_plan_id: Optional[int],
    entries: object,
    *,
    now_ms: Optional[int] = None,
    reason: str | None = None,
) -> list[int]:
    if input_package_id is None:
        return []
    try:
        package_key = str(int(input_package_id))
    except Exception:
        return []

    iterable = entries if isinstance(entries, (list, tuple, set)) else [entries]
    normalized_entries = [
        normalized
        for normalized in (_normalize_deferred_target_entry(item) for item in iterable)
        if isinstance(normalized, dict)
    ]
    if not normalized_entries:
        return []

    data = _load_state()
    if not isinstance(data, dict):
        data = {}
    deferred_map = data.get(_KEY_DEFERRED_ATTACK_TARGETS)
    if not isinstance(deferred_map, dict):
        deferred_map = {}
    package_map = deferred_map.get(package_key)
    if not isinstance(package_map, dict):
        package_map = {}

    try:
        now_int = int(now_ms) if now_ms is not None else None
    except Exception:
        now_int = None
    try:
        source_plan_id_int = int(source_plan_id) if source_plan_id is not None else None
    except Exception:
        source_plan_id_int = None

    changed_ids: list[int] = []
    for normalized in normalized_entries:
        target_id = _normalize_target_id(normalized.get("targetID"))
        if target_id is None:
            continue
        key = str(int(target_id))
        existing = package_map.get(key)
        merged = dict(existing) if isinstance(existing, dict) else {}
        if now_int is not None and merged.get("firstDeferredMs") is None:
            merged["firstDeferredMs"] = int(now_int)
        merged.update(normalized)
        if now_int is not None:
            merged["lastSeenMs"] = int(now_int)
        if source_plan_id_int is not None:
            merged["sourceMissionPlanID"] = int(source_plan_id_int)
        if reason:
            merged["deferredReason"] = str(reason)
        package_map[key] = merged
        changed_ids.append(int(target_id))

    if not changed_ids:
        return []
    deferred_map[package_key] = package_map
    data[_KEY_DEFERRED_ATTACK_TARGETS] = deferred_map
    _save_state(data)
    return sorted(set(changed_ids))


def list_deferred_attack_targets(input_package_id: Optional[int]) -> list[dict]:
    if input_package_id is None:
        return []
    try:
        package_key = str(int(input_package_id))
    except Exception:
        return []
    data = _load_state()
    deferred_map = data.get(_KEY_DEFERRED_ATTACK_TARGETS) if isinstance(data, dict) else None
    if not isinstance(deferred_map, dict):
        return []
    package_map = deferred_map.get(package_key)
    if not isinstance(package_map, dict):
        return []
    out: list[dict] = []
    for value in package_map.values():
        normalized = _normalize_deferred_target_entry(value)
        if normalized is None:
            continue
        if isinstance(value, dict):
            merged = dict(value)
            merged.update(normalized)
        else:
            merged = normalized
        out.append(merged)
    out.sort(
        key=lambda item: (
            _to_int(item.get("firstDeferredMs")) or _to_int(item.get("lastSeenMs")) or 0,
            _to_int(item.get("targetID")) or 0,
        )
    )
    return out


def clear_deferred_attack_targets(
    input_package_id: Optional[int],
    target_ids: object = None,
) -> list[int]:
    if input_package_id is None:
        return []
    try:
        package_key = str(int(input_package_id))
    except Exception:
        return []

    data = _load_state()
    if not isinstance(data, dict):
        return []
    deferred_map = data.get(_KEY_DEFERRED_ATTACK_TARGETS)
    if not isinstance(deferred_map, dict):
        return []
    package_map = deferred_map.get(package_key)
    if not isinstance(package_map, dict) or not package_map:
        return []

    if target_ids is None:
        clear_ids = {
            int(tid)
            for tid in (_normalize_target_id(item.get("targetID")) for item in package_map.values() if isinstance(item, dict))
            if tid is not None
        }
        package_map = {}
    else:
        clear_ids = set(_normalize_aircraft_ids(target_ids))
        if not clear_ids:
            return []
        for target_id in list(clear_ids):
            package_map.pop(str(int(target_id)), None)

    if package_map:
        deferred_map[package_key] = package_map
    else:
        deferred_map.pop(package_key, None)
    if deferred_map:
        data[_KEY_DEFERRED_ATTACK_TARGETS] = deferred_map
    else:
        data.pop(_KEY_DEFERRED_ATTACK_TARGETS, None)
    _save_state(data)
    return sorted(clear_ids)


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
    set_pending_manned_assignments(mission_plan_id, input_package_id, [aircraft_id])


def set_pending_manned_assignments(
    mission_plan_id: Optional[int],
    input_package_id: Optional[int],
    aircraft_ids: object,
) -> None:
    plan_id = _to_int(mission_plan_id)
    package_id = _to_int(input_package_id)
    normalized_aircraft_ids = _normalize_aircraft_ids(aircraft_ids)
    if plan_id is None or package_id is None or not normalized_aircraft_ids:
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
        "aircraft_id": int(normalized_aircraft_ids[0]),
        "aircraft_ids": list(normalized_aircraft_ids),
    }
    data[_KEY_PENDING_BY_PLAN] = pending_map
    _save_state(data)


def clear_pending_manned_assignments(mission_plan_ids: object) -> list[int]:
    plan_ids = _normalize_aircraft_ids(mission_plan_ids)
    if not plan_ids:
        return []
    data = _load_state()
    if not isinstance(data, dict):
        return []
    pending_map = data.get(_KEY_PENDING_BY_PLAN)
    if not isinstance(pending_map, dict):
        return []

    cleared: list[int] = []
    for plan_id in plan_ids:
        if pending_map.pop(str(int(plan_id)), None) is not None:
            cleared.append(int(plan_id))
    if not cleared:
        return []

    if pending_map:
        data[_KEY_PENDING_BY_PLAN] = pending_map
    else:
        data.pop(_KEY_PENDING_BY_PLAN, None)
    _save_state(data)
    return cleared


def commit_pending_manned_assignment(mission_plan_id: Optional[int]) -> Optional[int]:
    committed = commit_pending_manned_assignments(mission_plan_id)
    return committed[0] if committed else None


def commit_pending_manned_assignments(mission_plan_id: Optional[int]) -> list[int]:
    plan_id = _to_int(mission_plan_id)
    if plan_id is None:
        return []
    data = _load_state()
    pending_map = data.get(_KEY_PENDING_BY_PLAN)
    if not isinstance(pending_map, dict):
        return []
    entry = pending_map.pop(str(plan_id), None)
    if not isinstance(entry, dict):
        if pending_map:
            data[_KEY_PENDING_BY_PLAN] = pending_map
        else:
            data.pop(_KEY_PENDING_BY_PLAN, None)
        _save_state(data)
        return []

    package_id = _to_int(entry.get("input_package_id"))
    aircraft_ids = _normalize_aircraft_ids(entry.get("aircraft_ids") or entry.get("aircraft_id"))
    if package_id is None or not aircraft_ids:
        if pending_map:
            data[_KEY_PENDING_BY_PLAN] = pending_map
        else:
            data.pop(_KEY_PENDING_BY_PLAN, None)
        _save_state(data)
        return []

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
    for aircraft_id in aircraft_ids:
        if aircraft_id not in used_list:
            used_list.append(aircraft_id)
    used_map[str(package_id)] = used_list
    data[_KEY_USED_BY_INPUT] = used_map

    if pending_map:
        data[_KEY_PENDING_BY_PLAN] = pending_map
    else:
        data.pop(_KEY_PENDING_BY_PLAN, None)
    _save_state(data)
    return aircraft_ids
