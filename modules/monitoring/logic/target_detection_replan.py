# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import math
import os
import threading
from typing import Any, Callable

from modules.common import prior_target_rediscovery_store
from modules.monitoring.logic.init_replan import allocate_mission_plan_ids, collect_input_mission_ids
from modules.monitoring.logic.mission_update import load_db_json
from modules.monitoring.logic.replan_runtime_settings import (
    get_post_attack_rejoin_settings,
    get_replan_toggle,
    get_target_detection_settings,
)
from modules.monitoring.logic.target_info import (
    load_target_info,
    update_target_info_from_0402,
    save_target_info,
)
try:
    from modules.mission_planning.runtime.attack_assignment_state import (
        clear_deferred_attack_targets,
        defer_attack_targets,
        get_used_manned_ids,
        list_deferred_attack_targets,
        release_manned_used,
    )
except Exception:  # pragma: no cover - optional dependency
    def get_used_manned_ids(_input_package_id: int | None) -> set[int]:
        return set()

    def release_manned_used(
        _input_package_id: int | None,
        _aircraft_ids: object = None,
    ) -> list[int]:
        return []

    def defer_attack_targets(
        _input_package_id: int | None,
        _source_plan_id: int | None,
        _entries: object,
        *,
        now_ms: int | None = None,
        reason: str | None = None,
    ) -> list[int]:
        return []

    def list_deferred_attack_targets(_input_package_id: int | None) -> list[dict[str, Any]]:
        return []

    def clear_deferred_attack_targets(
        _input_package_id: int | None,
        _target_ids: object = None,
    ) -> list[int]:
        return []
try:
    from modules.mission_planning.runtime.attack_tracking_state import (
        list_active_tracking_assignments,
        resolve_plan_lineage_ids,
    )
except Exception:  # pragma: no cover - optional dependency
    def list_active_tracking_assignments() -> list[dict[str, Any]]:
        return []

    def resolve_plan_lineage_ids(_plan_id: int | None, *, max_depth: int = 12) -> set[int]:
        return set()

TARGET_TYPE_LABELS = {
    None: "표적",
    0: "표적",
    1: "전차",
    2: "장갑차",
    3: "방사포",
    4: "곡사포",
    5: "고정고사포",
    6: "군인",
}

WATCHER_CALLSIGN_MAP = {
    4: "무인기 1번",
    5: "무인기 2번",
    6: "무인기 3번",
}

WATCHER_UAV_IDS = tuple(sorted(WATCHER_CALLSIGN_MAP.keys())) or (4, 5, 6)
ATTACK_MANNED_IDS = (2, 3)

OPTION_PRESETS = (
    (2, "공격 특화"),
    (3, "공격 배제"),
)
POST_ATTACK_REJOIN_REASON = "공격후 협업복귀"
POST_ATTACK_CLOSE_COOLDOWN_MS = 30_000


def _target_detection_config() -> dict[str, Any]:
    return get_target_detection_settings()


def _target_detection_enabled() -> bool:
    return bool(get_replan_toggle("target_detection", True))


def _post_attack_rejoin_config() -> dict[str, Any]:
    return get_post_attack_rejoin_settings()


def _post_attack_rejoin_enabled() -> bool:
    return bool(get_replan_toggle("post_attack_rejoin", True))


def _watcher_uav_ids() -> tuple[int, ...]:
    values = _target_detection_config().get("watcher_uav_ids") or list(WATCHER_UAV_IDS)
    out = [
        int(item)
        for item in values
        if _coerce_int(item) is not None and int(_coerce_int(item)) > 0
    ]
    return tuple(out) or tuple(WATCHER_UAV_IDS)


def _stale_attack_slots(
    input_package_id: int | None,
    mission_plan_id: int | None,
    used_manned: set[int],
    logs: list[str],
) -> set[int]:
    """Occupied slots whose aircraft no longer carries an attack, now released.

    Judging occupancy by the live plan rather than the flag is what keeps a
    finished engagement from holding a manned aircraft out of the fight.
    """

    if not used_manned or not input_package_id or not mission_plan_id:
        return set()
    try:
        from modules.monitoring.logic.mission_update import mission_plan_json_path

        plan_path = mission_plan_json_path(int(mission_plan_id))
        if plan_path is None or not plan_path.exists():
            return set()
        plan = json.loads(plan_path.read_text(encoding="utf-8"))
        busy: set[int] = set()
        for aircraft in (plan or {}).get("aircraftList") or []:
            if not isinstance(aircraft, dict):
                continue
            aircraft_id = _coerce_int(aircraft.get("aircraftID"))
            if aircraft_id is None or int(aircraft_id) not in used_manned:
                continue
            for mission in aircraft.get("individualMissionList") or []:
                if not isinstance(mission, dict):
                    continue
                path_id = _coerce_int(mission.get("pathID"))
                if not path_id:
                    continue
                path_file = plan_path.parent.parent / "FlightPath" / f"{int(path_id)}.json"
                if not path_file.exists():
                    continue
                payload = json.loads(path_file.read_text(encoding="utf-8"))
                for waypoint in (payload or {}).get("lahWaypointList") or []:
                    attack = waypoint.get("attack") if isinstance(waypoint, dict) else None
                    target_id = _coerce_int((attack or {}).get("targetID")) if isinstance(attack, dict) else None
                    if target_id and int(target_id) > 0:
                        busy.add(int(aircraft_id))
                        break
                if int(aircraft_id) in busy:
                    break
        stale = {int(aid) for aid in used_manned if int(aid) not in busy}
        if not stale:
            return set()
        released = release_manned_used(int(input_package_id), sorted(stale))
        logs.append(
            "[0402] stale manned attack slots reclaimed: no attack left in their plan "
            f"(inputMissionPackageID={input_package_id}, aircraft={sorted(stale)}, "
            f"released={released})"
        )
        return stale
    except Exception as exc:  # pragma: no cover - defensive
        logs.append(f"[0402] manned attack slot reconciliation skipped: {exc}")
        return set()


def _attack_manned_ids() -> tuple[int, ...]:
    values = _target_detection_config().get("attack_manned_ids") or list(ATTACK_MANNED_IDS)
    out = [
        int(item)
        for item in values
        if _coerce_int(item) is not None and int(_coerce_int(item)) > 0
    ]
    return tuple(out) or tuple(ATTACK_MANNED_IDS)


def _option_presets() -> tuple[tuple[int, str], ...]:
    raw_items = _target_detection_config().get("option_presets") or []
    out: list[tuple[int, str]] = []
    for item in raw_items:
        if not isinstance(item, dict):
            continue
        option_id = _coerce_int(item.get("option_id") or item.get("optionID"))
        option_name = str(item.get("option_name") or item.get("optionName") or "").strip()
        if option_id is None or option_id <= 0 or not option_name:
            continue
        out.append((int(option_id), option_name))
    return tuple(out) or OPTION_PRESETS


def _target_type_priority_order() -> list[int]:
    raw = _target_detection_config().get("target_type_priority") or [1, 2, 3, 4, 5, 6]
    order: list[int] = []
    seen: set[int] = set()
    for item in raw:
        value = _coerce_int(item)
        if value is None or value <= 0 or value in seen:
            continue
        seen.add(value)
        order.append(int(value))
    for value in (1, 2, 3, 4, 5, 6):
        if value not in seen:
            order.append(value)
    return order


def _target_type_priority_rank(target_type: object) -> int:
    value = _coerce_int(target_type)
    order = _target_type_priority_order()
    if value is None:
        return len(order) + 100
    try:
        return order.index(int(value))
    except ValueError:
        return len(order) + 100


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _find_target_key_by_id(target_map: dict, target_id: int) -> str | None:
    for key, entry in target_map.items():
        if not isinstance(entry, dict):
            continue
        if _coerce_int(entry.get("targetID")) == target_id:
            return str(key)
    return None


def _collect_active_watchers(info: dict[str, Any]) -> set[int]:
    target_map = info.get("targetList") if isinstance(info, dict) else None
    if not isinstance(target_map, dict):
        return set()
    used: set[int] = set()
    for entry in target_map.values():
        if not isinstance(entry, dict):
            continue
        target_id = _coerce_int(entry.get("targetID"))
        if target_id is None or target_id <= 0:
            continue
        if entry.get("isDestroyed"):
            continue
        is_ignored = _coerce_int(entry.get("isIgnored"))
        if is_ignored is not None and is_ignored != 0:
            continue
        watcher_id = _coerce_int(entry.get("watcherID"))
        if watcher_id is not None:
            used.add(watcher_id)
    return used


def _assign_watcher_id(entry: dict[str, Any], used_watchers: set[int]) -> int | None:
    watcher_id = _coerce_int(entry.get("watcherID"))
    if watcher_id is not None:
        used_watchers.add(watcher_id)
        return watcher_id
    for candidate in _watcher_uav_ids():
        if candidate not in used_watchers:
            used_watchers.add(candidate)
            return candidate
    return None


def _sync_target_watcher(
    info: dict[str, Any],
    key: str,
    entry: dict[str, Any],
) -> tuple[str, bool]:
    watcher_id = _coerce_int(entry.get("watcherID"))
    target_id = _coerce_int(entry.get("targetID"))
    if watcher_id is None or target_id is None:
        return key, False
    target_map = info.get("targetList") if isinstance(info, dict) else None
    if not isinstance(target_map, dict):
        return key, False
    map_key = key if key in target_map else _find_target_key_by_id(target_map, target_id)
    if map_key is None:
        return key, False
    target_entry = target_map.get(map_key)
    if not isinstance(target_entry, dict):
        return key, False
    target_entry["watcherID"] = watcher_id
    new_key = f"{target_id}-{watcher_id}"
    if new_key != map_key:
        existing = target_map.get(new_key)
        if isinstance(existing, dict):
            merged = dict(existing)
            merged.update({k: v for k, v in target_entry.items() if v is not None})
            target_map[new_key] = merged
        else:
            target_map[new_key] = target_entry
        try:
            del target_map[map_key]
        except Exception:
            pass
        return new_key, True
    return map_key, True


def _extract_input_package_id(mission_plan_id: int | None) -> int | None:
    if mission_plan_id is None:
        return None
    plan = load_db_json("MissionPlan", mission_plan_id)
    return _coerce_int(
        plan.get("inputMissionPackageID")
        or plan.get("InputMissionPackageID")
        or plan.get("inputMissionPackageId")
    )


def _pending_input_ids_from_package(package_id: int | None) -> list[int]:
    if package_id is None:
        return []
    payload = load_db_json("InputMissionPlan", package_id)
    items = payload.get("inputMissionList") or []
    pending: list[int] = []
    seen: set[int] = set()
    for mission in items:
        if not isinstance(mission, dict):
            continue
        if mission.get("isDone"):
            continue
        mission_id = _coerce_int(mission.get("inputMissionID"))
        if mission_id is None or mission_id <= 0 or mission_id in seen:
            continue
        seen.add(mission_id)
        pending.append(mission_id)
    return pending


def _format_target_reason(watcher_id: int | None, target_type: int | None, target_id: int) -> str:
    watcher_label = WATCHER_CALLSIGN_MAP.get(watcher_id)
    if not watcher_label:
        watcher_label = f"무인기 {watcher_id}번" if watcher_id else "미상 감시기"
    target_label = TARGET_TYPE_LABELS.get(target_type, TARGET_TYPE_LABELS.get(None, "표적"))
    return f"표적{int(target_id)} 발견 재계획"


def _format_target_bundle_reason(
    watcher_id: int | None,
    target_type: int | None,
    target_id: int,
    *,
    bundle_count: int,
) -> str:
    if int(bundle_count or 0) <= 1:
        return _format_target_reason(watcher_id, target_type, target_id)
    return f"표적{int(target_id)} 등 {int(bundle_count)}개 발견"


def _extract_target_timestamp(entry: dict[str, Any]) -> int | None:
    for field in ("lastUpdated", "LastUpdated", "timestamp", "Timestamp", "lastObserved"):
        value = entry.get(field)
        ts = _coerce_int(value)
        if ts is not None:
            return ts
    return _coerce_int(entry.get("firstDetected"))


def _coerce_float(value: object) -> float | None:
    try:
        fval = float(value)
    except Exception:
        return None
    if not math.isfinite(fval):
        return None
    return fval


def _normalize_coordinate_payload(value: object) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None
    lat = _coerce_float(value.get("latitude"))
    lon = _coerce_float(value.get("longitude"))
    alt = _coerce_float(value.get("altitude"))
    if lat is None or lon is None:
        return None
    payload = {
        "latitude": float(lat),
        "longitude": float(lon),
        "altitude": float(alt) if alt is not None else 0.0,
    }
    if abs(payload["latitude"]) < 1e-9 and abs(payload["longitude"]) < 1e-9:
        return None
    return payload


def _coerce_bool(value: object) -> bool | None:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    try:
        lowered = str(value).strip().lower()
    except Exception:
        return None
    if lowered in {"1", "true", "t", "y", "yes", "on"}:
        return True
    if lowered in {"0", "false", "f", "n", "no", "off"}:
        return False
    return None


def _extract_message_target_entries(message: object) -> list[dict[str, Any]]:
    if isinstance(message, (bytes, bytearray, str)):
        try:
            message = json.loads(message)
        except Exception:
            message = {}
    if not isinstance(message, dict):
        return []
    message_ts = _coerce_int(message.get("timestamp") or message.get("Timestamp"))

    raw_targets = (
        message.get("targetList")
        or message.get("TargetList")
        or message.get("targets")
        or message.get("Targets")
        or message.get("situationAwarenessInfoList")
        or message.get("SituationAwarenessInfoList")
        or []
    )
    if not isinstance(raw_targets, list):
        raw_targets = [raw_targets]

    entries: list[dict[str, Any]] = []
    for item in raw_targets:
        if not isinstance(item, dict):
            continue
        target_id = _coerce_int(item.get("targetID") or item.get("targetId"))
        if target_id is None or target_id <= 0:
            continue
        watcher_block = item.get("watcher") if isinstance(item.get("watcher"), dict) else {}
        watcher_id = _coerce_int(
            item.get("watcherID")
            or item.get("watcherId")
            or item.get("aircraftID")
            or watcher_block.get("aircraftID")
        )
        entry = {
            "targetID": int(target_id),
            "watcherID": watcher_id,
            "targetType": _coerce_int(item.get("targetType")),
            "coordinate": _normalize_coordinate_payload(item.get("coordinate")),
            "targetInFrame": _coerce_bool(item.get("targetInFrame")),
            "isDestroyed": _coerce_bool(item.get("isDestroyed")),
            "threat": _coerce_float(item.get("threat")),
        }
        if message_ts is not None:
            entry["timestamp"] = int(message_ts)
        entries.append(entry)
    return entries


def destroyed_target_ids_from_message(message: object | None) -> set[int]:
    """Return target IDs carrying an explicit destroyed transition in 0402."""

    return {
        int(target_id)
        for entry in _extract_message_target_entries(message)
        if _coerce_bool(entry.get("isDestroyed")) is True
        and (target_id := _coerce_int(entry.get("targetID"))) is not None
        and int(target_id) > 0
    }


def _current_plan_tracking_target_pairs(
    current_mission_plan_id: int | None,
    active_assignments: list[dict[str, Any]],
) -> set[tuple[int, int]]:
    """Find active tracker/target pairs that are still executable in the plan.

    Incremental attack replans can preserve an older tracker while replacing
    its IMP/path IDs. The runtime assignment then legitimately points at the
    older attack plan even though the current plan still contains the same
    aircraft/target tracking branch. This artifact check is the safe fallback
    for that lineage gap.
    """

    plan_id = _coerce_int(current_mission_plan_id)
    if plan_id is None or plan_id <= 0:
        return set()

    target_by_aircraft: dict[int, int] = {}
    for assignment in active_assignments or []:
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        aircraft_id = _coerce_int(assignment.get("aircraft_id"))
        target_id = _coerce_int(assignment.get("target_id"))
        if (
            aircraft_id is None
            or aircraft_id <= 0
            or target_id is None
            or target_id <= 0
        ):
            continue
        target_by_aircraft[int(aircraft_id)] = int(target_id)
    if not target_by_aircraft:
        return set()

    plan_data = load_db_json("MissionPlan", int(plan_id))
    if not isinstance(plan_data, dict) or not plan_data:
        return set()

    pairs: set[tuple[int, int]] = set()
    for aircraft_entry in plan_data.get("aircraftList") or []:
        if not isinstance(aircraft_entry, dict):
            continue
        aircraft_id = _coerce_int(aircraft_entry.get("aircraftID"))
        if aircraft_id is None or int(aircraft_id) not in target_by_aircraft:
            continue
        package_id = _coerce_int(
            aircraft_entry.get("individualMissionPackageID")
            or aircraft_entry.get("individualMissionPlanPackageID")
            or aircraft_entry.get("individualMissionPackageId")
        )
        if package_id is None or package_id <= 0:
            continue
        imp_data = load_db_json("IndividualMissionPlan", int(package_id))
        target_id = int(target_by_aircraft[int(aircraft_id)])
        for mission in imp_data.get("individualMissionList") or []:
            if not isinstance(mission, dict) or bool(mission.get("isDone")):
                continue
            mission_info = (
                mission.get("individualMissionInfo")
                if isinstance(mission.get("individualMissionInfo"), dict)
                else {}
            )
            related = (
                mission.get("relatedMission")
                if isinstance(mission.get("relatedMission"), dict)
                else {}
            )
            mission_target_id = _coerce_int(
                mission_info.get("targetID")
                or related.get("targetID")
                or mission.get("targetID")
            )
            if mission_target_id == target_id:
                pairs.add((int(aircraft_id), int(target_id)))
                break
    return pairs


def _tracking_assignment_matches_current_plan(
    assignment: dict[str, Any],
    *,
    current_plan_lineage: set[int],
    current_plan_target_pairs: set[tuple[int, int]],
) -> bool:
    if _tracking_assignment_matches_plan_lineage(
        assignment,
        current_plan_lineage=current_plan_lineage,
    ):
        return True
    aircraft_id = _coerce_int(assignment.get("aircraft_id"))
    target_id = _coerce_int(assignment.get("target_id"))
    if aircraft_id is None or target_id is None:
        return False
    return (int(aircraft_id), int(target_id)) in current_plan_target_pairs


def _extract_destroyed_tracking_entries_from_info(
    info: dict[str, Any] | None,
    *,
    active_assignments: list[dict[str, Any]],
    current_plan_lineage: set[int],
    current_plan_target_pairs: set[tuple[int, int]] | None = None,
) -> list[dict[str, Any]]:
    target_map = info.get("targetList") if isinstance(info, dict) else None
    if not isinstance(target_map, dict):
        return []

    tracked_by_target: dict[int, list[dict[str, Any]]] = {}
    for assignment in active_assignments or []:
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        if not _tracking_assignment_matches_current_plan(
            assignment,
            current_plan_lineage=current_plan_lineage,
            current_plan_target_pairs=set(current_plan_target_pairs or set()),
        ):
            continue
        target_id = _coerce_int(assignment.get("target_id"))
        if target_id is None or target_id <= 0:
            continue
        tracked_by_target.setdefault(int(target_id), []).append(dict(assignment))
    if not tracked_by_target:
        return []

    out: list[dict[str, Any]] = []
    for target_id, assignments in sorted(tracked_by_target.items()):
        assignment_aircraft_ids = {
            int(_coerce_int(item.get("aircraft_id")))
            for item in assignments
            if _coerce_int(item.get("aircraft_id")) is not None
        }
        candidates: list[dict[str, Any]] = []
        for key, raw in target_map.items():
            if not isinstance(raw, dict):
                continue
            if _coerce_int(raw.get("targetID")) != int(target_id):
                continue
            if _coerce_bool(raw.get("isDestroyed")) is not True:
                continue
            entry = dict(raw)
            entry["key"] = str(key)
            candidates.append(entry)
        if not candidates:
            continue

        def _candidate_key(item: dict[str, Any]) -> tuple[int, int, int]:
            watcher_id = _coerce_int(item.get("watcherID"))
            return (
                0 if watcher_id in assignment_aircraft_ids else 1,
                0 if _normalize_coordinate_payload(item.get("coordinate")) is not None else 1,
                -(_extract_target_timestamp(item) or 0),
            )

        selected = sorted(candidates, key=_candidate_key)[0]
        watcher_id = _coerce_int(selected.get("watcherID"))
        if watcher_id is None and len(assignment_aircraft_ids) == 1:
            watcher_id = next(iter(assignment_aircraft_ids))
        out.append(
            {
                "targetID": int(target_id),
                "watcherID": watcher_id,
                "targetType": _coerce_int(selected.get("targetType")),
                "coordinate": _normalize_coordinate_payload(selected.get("coordinate")) or {},
                "targetInFrame": _coerce_bool(selected.get("targetInFrame")),
                "isDestroyed": True,
                "threat": _coerce_float(selected.get("threat")),
                "timestamp": _extract_target_timestamp(selected),
                "fromTargetInfo": True,
            }
        )
    return out


def _has_actionable_coordinate(entry: dict[str, Any]) -> bool:
    coord = entry.get("coordinate")
    if not isinstance(coord, dict):
        return False
    lat = _coerce_float(coord.get("latitude"))
    lon = _coerce_float(coord.get("longitude"))
    if lat is None or lon is None:
        return False
    # Sentinel noise from 0402 payloads.
    if abs(lat) < 1e-9 and abs(lon) < 1e-9:
        return False
    return True


def _is_actionable_target_entry(key: str, entry: dict[str, Any]) -> bool:
    if key.startswith("unknown-"):
        return False
    target_id = _coerce_int(entry.get("targetID"))
    if target_id is None or target_id <= 0:
        return False
    is_used = _coerce_int(entry.get("isUsed"))
    if is_used is not None and is_used != 0:
        return False
    is_ignored = _coerce_int(entry.get("isIgnored"))
    if is_ignored is not None and is_ignored != 0:
        return False
    if bool(entry.get("isDestroyed")):
        return False
    if not _has_actionable_coordinate(entry):
        return False
    return True


def _is_bundle_candidate_entry(entry: dict[str, Any]) -> bool:
    target_id = _coerce_int(entry.get("targetID"))
    if target_id is None or target_id <= 0:
        return False
    if bool(entry.get("isDestroyed")):
        return False
    is_ignored = _coerce_int(entry.get("isIgnored"))
    if is_ignored is not None and is_ignored != 0:
        return False
    return _normalize_coordinate_payload(entry.get("coordinate")) is not None


def _bundle_order_key(entry: dict[str, Any], *, trigger_target_id: int | None) -> tuple[int, int, int, int, int]:
    target_id = _coerce_int(entry.get("targetID")) or 0
    is_used = 1 if (_coerce_int(entry.get("isUsed")) or 0) != 0 else 0
    trigger_rank = 0 if (trigger_target_id is not None and target_id == int(trigger_target_id)) else 1
    used_rank = 0 if is_used == 1 else 1
    detected_ts = _coerce_int(entry.get("firstDetected")) or _coerce_int(entry.get("timestamp")) or 0
    type_rank = _target_type_priority_rank(entry.get("targetType"))
    return (used_rank, trigger_rank, type_rank, detected_ts, target_id)


def _select_bundle_entries(
    info: dict[str, Any],
    *,
    primary_entry: dict[str, Any],
    limit: int = 3,
) -> tuple[list[dict[str, Any]], int]:
    target_map = info.get("targetList") if isinstance(info, dict) else None
    merged: dict[int, dict[str, Any]] = {}

    def _merge_entry(raw: dict[str, Any]) -> None:
        if not isinstance(raw, dict):
            return
        target_id = _coerce_int(raw.get("targetID"))
        if target_id is None or target_id <= 0:
            return
        existing = merged.get(int(target_id))
        candidate = dict(existing or {})
        candidate.update(dict(raw))
        if existing is not None:
            prev_coord = _normalize_coordinate_payload(existing.get("coordinate"))
            if prev_coord is not None and _normalize_coordinate_payload(candidate.get("coordinate")) is None:
                candidate["coordinate"] = prev_coord
            if (_coerce_int(existing.get("isUsed")) or 0) != 0:
                candidate["isUsed"] = 1
            if (_coerce_int(existing.get("isIgnored")) or 0) != 0:
                candidate["isIgnored"] = 1
        merged[int(target_id)] = candidate

    if isinstance(target_map, dict):
        for key, entry in target_map.items():
            if not isinstance(entry, dict):
                continue
            candidate = dict(entry)
            candidate.setdefault("key", str(key))
            _merge_entry(candidate)

    primary_copy = dict(primary_entry or {})
    if primary_copy:
        _merge_entry(primary_copy)

    trigger_target_id = _coerce_int(primary_copy.get("targetID"))
    candidates = [
        dict(entry)
        for entry in merged.values()
        if isinstance(entry, dict) and _is_bundle_candidate_entry(entry)
    ]
    candidates.sort(key=lambda entry: _bundle_order_key(entry, trigger_target_id=trigger_target_id))
    selected = candidates[: max(1, int(limit))]
    return selected, len(candidates)


def _assign_bundle_watchers(entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    allowed = list(_watcher_uav_ids())
    if not entries or not allowed:
        return entries
    priority_sorted = sorted(
        range(len(entries)),
        key=lambda idx: _bundle_order_key(entries[idx], trigger_target_id=_coerce_int(entries[idx].get("targetID"))),
    )
    assigned: set[int] = set()
    for idx in priority_sorted:
        watcher_id = _coerce_int(entries[idx].get("watcherID"))
        if watcher_id is not None and watcher_id in allowed and watcher_id not in assigned:
            entries[idx]["watcherID"] = int(watcher_id)
            assigned.add(int(watcher_id))
        else:
            entries[idx]["watcherID"] = None
    for idx in priority_sorted:
        if _coerce_int(entries[idx].get("watcherID")) is not None:
            continue
        for watcher_id in allowed:
            if watcher_id in assigned:
                continue
            entries[idx]["watcherID"] = int(watcher_id)
            assigned.add(int(watcher_id))
            break
    return entries


def _serialize_bundle_target(entry: dict[str, Any]) -> dict[str, Any] | None:
    target_id = _coerce_int(entry.get("targetID"))
    coord = _normalize_coordinate_payload(entry.get("coordinate"))
    if target_id is None or coord is None:
        return None
    payload: dict[str, Any] = {
        "targetID": int(target_id),
        "targetType": int(_coerce_int(entry.get("targetType")) or 0),
        "coordinate": coord,
        "isUsed": int(_coerce_int(entry.get("isUsed")) or 0),
        "isIgnored": int(_coerce_int(entry.get("isIgnored")) or 0),
        "isDestroyed": bool(entry.get("isDestroyed")),
        "targetInFrame": bool(entry.get("targetInFrame")),
        "threat": float(_coerce_float(entry.get("threat")) or 0.0),
    }
    watcher_id = _coerce_int(entry.get("watcherID"))
    if watcher_id is not None:
        payload["watcherID"] = int(watcher_id)
    key = str(entry.get("key") or "").strip()
    if key:
        payload["targetKey"] = key
    first_detected = _coerce_int(entry.get("firstDetected"))
    if first_detected is not None:
        payload["firstDetected"] = int(first_detected)
    last_updated = _coerce_int(entry.get("lastUpdated"))
    if last_updated is not None:
        payload["lastUpdated"] = int(last_updated)
    return payload


def _build_attack_target_bundle(
    info: dict[str, Any],
    *,
    primary_entry: dict[str, Any],
    limit: int = 3,
) -> tuple[list[dict[str, Any]], int]:
    selected, total_count = _select_bundle_entries(info, primary_entry=primary_entry, limit=limit)
    selected = _assign_bundle_watchers([dict(entry) for entry in selected])
    bundle: list[dict[str, Any]] = []
    for order, entry in enumerate(selected, start=1):
        payload = _serialize_bundle_target(entry)
        if payload is None:
            continue
        payload["selectionOrder"] = int(order)
        bundle.append(payload)
    return bundle, int(total_count)


def _target_is_blocked_by_state(info: dict[str, Any], target_id: int) -> bool:
    target_map = info.get("targetList") if isinstance(info, dict) else None
    if not isinstance(target_map, dict):
        return False
    seen_alive = False
    seen_destroyed = False
    for entry in target_map.values():
        if not isinstance(entry, dict):
            continue
        if _coerce_int(entry.get("targetID")) != int(target_id):
            continue
        is_used = _coerce_int(entry.get("isUsed"))
        if is_used is not None and is_used != 0:
            return True
        is_ignored = _coerce_int(entry.get("isIgnored"))
        if is_ignored is not None and is_ignored != 0:
            return True
        is_destroyed = entry.get("isDestroyed")
        if is_destroyed is True:
            seen_destroyed = True
        elif is_destroyed is False:
            seen_alive = True
    # If we have any alive view of the same target, do not block by stale
    # destroyed snapshots from other watchers.
    return seen_destroyed and not seen_alive


def _mark_target_used_in_info(
    info: dict[str, Any],
    *,
    key: str,
    target_id: int,
    watcher_id: int | None,
) -> bool:
    target_map = info.get("targetList") if isinstance(info, dict) else None
    if not isinstance(target_map, dict):
        return False

    changed = False
    candidate_keys: list[str] = []
    if key:
        candidate_keys.append(str(key))
    if watcher_id is not None:
        candidate_keys.append(f"{int(target_id)}-{int(watcher_id)}")
    candidate_keys.append(str(int(target_id)))

    seen: set[str] = set()
    for key_str in candidate_keys:
        if not key_str or key_str in seen:
            continue
        seen.add(key_str)
        target_entry = target_map.get(key_str)
        if isinstance(target_entry, dict) and _coerce_int(target_entry.get("isUsed")) != 1:
            target_entry["isUsed"] = 1
            changed = True

    for target_entry in target_map.values():
        if not isinstance(target_entry, dict):
            continue
        if _coerce_int(target_entry.get("targetID")) != int(target_id):
            continue
        if _coerce_int(target_entry.get("isUsed")) != 1:
            target_entry["isUsed"] = 1
            changed = True
    return changed


def _dedupe_candidates_by_target_id(candidates: list[dict[str, Any]]) -> list[dict[str, Any]]:
    best_by_target: dict[int, tuple[tuple[tuple[int, int, int, int, int, int], int], dict[str, Any]]] = {}
    for entry in candidates:
        if not isinstance(entry, dict):
            continue
        target_id = _coerce_int(entry.get("targetID"))
        if target_id is None:
            continue
        priority = _tracking_watcher_preference_key(entry)
        current = best_by_target.get(target_id)
        if current is None or priority < current[0]:
            best_by_target[target_id] = (priority, dict(entry))
    return [item[1] for item in best_by_target.values()]


def _candidate_priority_key(entry: dict[str, Any]) -> tuple[int, int, float, int]:
    priority_order = _target_type_priority_order()
    priority_rank = {int(target_type): idx for idx, target_type in enumerate(priority_order)}
    target_type = _coerce_int(entry.get("targetType"))
    threat = _coerce_float(entry.get("threat")) or 0.0
    timestamp = _extract_target_timestamp(entry) or -1
    return (
        priority_rank.get(int(target_type or 0), len(priority_rank)),
        0 if _is_prior_mission_rediscovery(entry) else 1,
        -float(threat),
        -int(timestamp),
    )


def _is_prior_mission_rediscovery(entry: dict[str, Any]) -> bool:
    return bool(entry.get("priorMissionRediscovered")) or bool(entry.get("priorMissionMatched"))


def _tracking_watcher_preference_key(entry: dict[str, Any]) -> tuple[int, int, int, int, int, int]:
    watcher_id = _coerce_int(entry.get("watcherID"))
    first_detected = _coerce_int(entry.get("firstDetected"))
    last_updated = _extract_target_timestamp(entry)
    return (
        0 if watcher_id is not None else 1,
        0 if bool(entry.get("targetInFrame")) else 1,
        0 if isinstance(entry.get("coordinate"), dict) else 1,
        int(first_detected) if first_detected is not None else 10**18,
        -(int(last_updated) if last_updated is not None else -1),
        int(watcher_id) if watcher_id is not None else 10**9,
    )


def build_target_bundle_from_target_info(
    info: dict[str, Any] | None = None,
    *,
    preferred_target_id: int | None = None,
    limit: int = 3,
) -> list[dict[str, Any]]:
    if not isinstance(info, dict):
        info = load_target_info()
    target_map = info.get("targetList") if isinstance(info, dict) else None
    if not isinstance(target_map, dict) or not target_map:
        return []

    candidates: list[dict[str, Any]] = []
    for key, entry in target_map.items():
        if not isinstance(entry, dict):
            continue
        target_id = _coerce_int(entry.get("targetID"))
        if target_id is None or target_id <= 0:
            continue
        if bool(entry.get("isDestroyed")):
            continue
        if _coerce_int(entry.get("isIgnored")) not in (None, 0):
            continue
        coord = entry.get("coordinate")
        if not isinstance(coord, dict):
            continue
        if not _has_actionable_coordinate(entry):
            continue

        watcher_id = _coerce_int(entry.get("watcherID"))
        if watcher_id is None:
            watcher_id = _extract_watcher_from_key(str(key))

        normalized = {
            "key": str(key),
            "targetID": int(target_id),
            "targetType": _coerce_int(entry.get("targetType")),
            "watcherID": watcher_id,
            "coordinate": {
                "latitude": _coerce_float(coord.get("latitude")),
                "longitude": _coerce_float(coord.get("longitude")),
            },
            "isUsed": 1 if _coerce_int(entry.get("isUsed")) == 1 else 0,
            "isIgnored": 1 if _coerce_int(entry.get("isIgnored")) == 1 else 0,
            "isDestroyed": bool(entry.get("isDestroyed")),
            "targetInFrame": bool(entry.get("targetInFrame")),
            "threat": _coerce_float(entry.get("threat")),
            "firstDetected": _coerce_int(entry.get("firstDetected")),
            "lastUpdated": _coerce_int(entry.get("lastUpdated")),
        }
        alt = _coerce_float(coord.get("altitude"))
        if alt is not None:
            normalized["coordinate"]["altitude"] = alt
        normalized["sourceKey"] = str(key)
        candidates.append(normalized)

    if not candidates:
        return []

    best_by_target: dict[int, tuple[tuple[int, int, int, int, int, int], dict[str, Any]]] = {}
    for entry in candidates:
        target_id = _coerce_int(entry.get("targetID"))
        if target_id is None:
            continue
        priority = (
            _tracking_watcher_preference_key(entry),
            0 if _coerce_int(entry.get("isUsed")) == 1 else 1,
        )
        current = best_by_target.get(target_id)
        if current is None or priority < current[0]:
            best_by_target[target_id] = (priority, dict(entry))

    deduped = [item[1] for item in best_by_target.values()]
    if not deduped:
        return []

    priority_order = _target_type_priority_order()
    priority_rank = {value: index for index, value in enumerate(priority_order)}

    def _bundle_key(item: dict[str, Any]) -> tuple[int, int, int, int, int]:
        target_type = _coerce_int(item.get("targetType"))
        target_id = _coerce_int(item.get("targetID")) or 0
        return (
            0 if _coerce_int(item.get("isUsed")) == 1 else 1,
            priority_rank.get(target_type or 0, len(priority_rank) + 100),
            0 if bool(item.get("targetInFrame")) else 1,
            -(_coerce_int(item.get("lastUpdated")) or _coerce_int(item.get("firstDetected")) or -1),
            -int(target_id),
        )

    deduped.sort(key=_bundle_key)

    if preferred_target_id is not None:
        preferred_target_id = _coerce_int(preferred_target_id)
        if preferred_target_id is not None:
            for idx, item in enumerate(deduped):
                if _coerce_int(item.get("targetID")) == preferred_target_id:
                    if idx != 0:
                        deduped.insert(0, deduped.pop(idx))
                    break

    if limit is not None and limit > 0:
        deduped = deduped[: int(limit)]
    return deduped


def _tracking_assignment_matches_close_event(
    assignment: dict[str, Any],
    *,
    current_plan_lineage: set[int],
    current_plan_target_pairs: set[tuple[int, int]],
    target_id: int,
) -> bool:
    if not isinstance(assignment, dict) or not bool(assignment.get("active")):
        return False
    if _coerce_int(assignment.get("target_id")) != int(target_id):
        return False
    return _tracking_assignment_matches_current_plan(
        assignment,
        current_plan_lineage=current_plan_lineage,
        current_plan_target_pairs=current_plan_target_pairs,
    )


def _tracking_assignment_matches_plan_lineage(
    assignment: dict[str, Any],
    *,
    current_plan_lineage: set[int],
) -> bool:
    if not isinstance(assignment, dict):
        return False
    if not current_plan_lineage:
        return True
    for key in ("attack_plan_id", "source_plan_id"):
        plan_id = _coerce_int(assignment.get(key))
        if plan_id is not None and int(plan_id) in current_plan_lineage:
            return True
    return False


def _active_attack_tracking_exists_for_plan(current_mission_plan_id: int | None) -> bool:
    plan_id = _coerce_int(current_mission_plan_id)
    if plan_id is None or plan_id <= 0:
        return False
    current_plan_lineage = resolve_plan_lineage_ids(int(plan_id)) or {int(plan_id)}
    for assignment in list_active_tracking_assignments():
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        if _tracking_assignment_matches_plan_lineage(
            assignment,
            current_plan_lineage=current_plan_lineage,
        ):
            return True
    return False


def _resolve_close_event_assignments(
    *,
    active_assignments: list[dict[str, Any]],
    current_plan_lineage: set[int],
    current_plan_target_pairs: set[tuple[int, int]] | None = None,
    target_id: int,
    watcher_id: int | None,
) -> dict[str, Any]:
    executable_pairs = set(current_plan_target_pairs or set())
    candidate_assignments = [
        assignment
        for assignment in active_assignments
        if _tracking_assignment_matches_close_event(
            assignment,
            current_plan_lineage=current_plan_lineage,
            current_plan_target_pairs=executable_pairs,
            target_id=int(target_id),
        )
    ]
    lineage_recovered = any(
        not _tracking_assignment_matches_plan_lineage(
            assignment,
            current_plan_lineage=current_plan_lineage,
        )
        for assignment in candidate_assignments
    )
    candidate_aircraft_ids = sorted(
        {
            int(_coerce_int(item.get("aircraft_id")))
            for item in candidate_assignments
            if _coerce_int(item.get("aircraft_id")) is not None
        }
    )
    watcher_uav_ids = set(_watcher_uav_ids())
    if not candidate_assignments:
        return {
            "assignments": [],
            "candidate_aircraft_ids": candidate_aircraft_ids,
            "resolved_watcher_id": watcher_id,
            "match_strategy": "no_target_plan_match",
            "lineage_recovered": False,
        }
    if watcher_id is None or watcher_id not in watcher_uav_ids:
        return {
            "assignments": candidate_assignments,
            "candidate_aircraft_ids": candidate_aircraft_ids,
            "resolved_watcher_id": watcher_id,
            "match_strategy": "watcher_unavailable",
            "lineage_recovered": bool(lineage_recovered),
        }

    watcher_matched = [
        assignment
        for assignment in candidate_assignments
        if _coerce_int(assignment.get("aircraft_id")) == int(watcher_id)
    ]
    if watcher_matched:
        return {
            "assignments": watcher_matched,
            "candidate_aircraft_ids": candidate_aircraft_ids,
            "resolved_watcher_id": watcher_id,
            "match_strategy": "watcher_match",
            "lineage_recovered": bool(lineage_recovered),
        }

    if len(candidate_aircraft_ids) == 1:
        return {
            "assignments": candidate_assignments,
            "candidate_aircraft_ids": candidate_aircraft_ids,
            "resolved_watcher_id": int(candidate_aircraft_ids[0]),
            "match_strategy": "target_plan_fallback",
            "lineage_recovered": bool(lineage_recovered),
        }

    return {
        "assignments": [],
        "candidate_aircraft_ids": candidate_aircraft_ids,
        "resolved_watcher_id": watcher_id,
        "match_strategy": "ambiguous_watcher_mismatch",
        "lineage_recovered": bool(lineage_recovered),
    }


def _build_attack_close_reason(target_id: int, watcher_id: int | None) -> str:
    return f"표적{int(target_id)} 격파 후 복귀"


class TargetDetectionCoordinator:
    REPLAN_LEVEL = 2

    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._target_trigger_history: dict[int, int] = {}
        self._attack_close_trigger_history: dict[str, int] = {}
        self._lock = threading.Lock()

    def _log_line(self, text: str) -> None:
        if self._log:
            self._log(text)

    def clear_target_trigger_history(self, target_ids: object) -> list[int]:
        normalized_ids: set[int] = set()
        for value in target_ids or []:
            try:
                target_id = int(value)
            except Exception:
                continue
            if target_id > 0:
                normalized_ids.add(target_id)
        if not normalized_ids:
            return []

        removed: list[int] = []
        with self._lock:
            for target_id in sorted(normalized_ids):
                if target_id in self._target_trigger_history:
                    self._target_trigger_history.pop(target_id, None)
                    removed.append(target_id)
        return removed

    def _build_attack_close_payloads(
        self,
        *,
        message: object | None,
        target_info: dict[str, Any] | None = None,
        current_mission_plan_id: int | None,
        mission_ids: list[int],
        package_id: int | None,
        now_ts: int,
        logs: list[str],
    ) -> list[dict[str, Any]]:
        if not _post_attack_rejoin_enabled():
            return []
        if current_mission_plan_id is None or current_mission_plan_id <= 0:
            return []

        active_assignments = [
            dict(item)
            for item in list_active_tracking_assignments()
            if isinstance(item, dict) and bool(item.get("active"))
        ]
        if not active_assignments:
            return []
        current_plan_lineage = resolve_plan_lineage_ids(int(current_mission_plan_id)) or {
            int(current_mission_plan_id)
        }
        current_plan_target_pairs = _current_plan_tracking_target_pairs(
            int(current_mission_plan_id),
            active_assignments,
        )
        close_cfg = _post_attack_rejoin_config()
        close_cooldown_ms = max(
            0,
            _coerce_int(close_cfg.get("closure_cooldown_ms")) or POST_ATTACK_CLOSE_COOLDOWN_MS,
        )

        payloads: list[dict[str, Any]] = []
        message_entries = _extract_message_target_entries(message)
        info_entries = _extract_destroyed_tracking_entries_from_info(
            target_info,
            active_assignments=active_assignments,
            current_plan_lineage=current_plan_lineage,
            current_plan_target_pairs=current_plan_target_pairs,
        )
        close_entries_by_target: dict[int, dict[str, Any]] = {}
        for entry in sorted(
            [*message_entries, *info_entries],
            key=lambda item: _extract_target_timestamp(item) or int(now_ts),
        ):
            target_id = _coerce_int(entry.get("targetID"))
            if target_id is None or target_id <= 0:
                continue
            previous = close_entries_by_target.get(int(target_id))
            if previous is None or (
                bool(previous.get("fromTargetInfo")) and not bool(entry.get("fromTargetInfo"))
            ):
                close_entries_by_target[int(target_id)] = dict(entry)

        for entry in close_entries_by_target.values():
            target_id = _coerce_int(entry.get("targetID"))
            if target_id is None or target_id <= 0:
                continue
            if not bool(entry.get("isDestroyed")):
                continue

            watcher_id = _coerce_int(entry.get("watcherID"))
            close_match = _resolve_close_event_assignments(
                active_assignments=active_assignments,
                current_plan_lineage=current_plan_lineage,
                current_plan_target_pairs=current_plan_target_pairs,
                target_id=int(target_id),
                watcher_id=watcher_id,
            )
            matched_assignments = list(close_match.get("assignments") or [])
            resolved_watcher_id = _coerce_int(close_match.get("resolved_watcher_id"))
            candidate_aircraft_ids = [
                int(item)
                for item in (close_match.get("candidate_aircraft_ids") or [])
                if _coerce_int(item) is not None
            ]
            match_strategy = str(close_match.get("match_strategy") or "")
            lineage_recovered = bool(close_match.get("lineage_recovered"))
            if not matched_assignments:
                if match_strategy == "ambiguous_watcher_mismatch":
                    logs.append(
                        "[0402-close] watcher mismatch ambiguous; skip "
                        f"targetID={target_id}, eventWatcher={watcher_id}, "
                        f"candidateTrackingAircraft={candidate_aircraft_ids}"
                    )
                else:
                    logs.append(
                        "[0402-close] no active tracking assignment matched destroyed target "
                        f"(targetID={target_id}, eventWatcher={watcher_id})"
                    )
                continue
            if lineage_recovered:
                logs.append(
                    "[0402-close] stale tracking-plan lineage recovered from current "
                    "MissionPlan artifact "
                    f"(targetID={target_id}, trackingAircraft={candidate_aircraft_ids}, "
                    f"currentPlanID={current_mission_plan_id})"
                )
            if (
                match_strategy == "target_plan_fallback"
                and watcher_id is not None
                and resolved_watcher_id is not None
                and int(watcher_id) != int(resolved_watcher_id)
            ):
                logs.append(
                    "[0402-close] watcher mismatch on destroyed target -> fallback by target/plan lineage "
                    f"(targetID={target_id}, eventWatcher={watcher_id}, "
                    f"trackingAircraft={candidate_aircraft_ids})"
                )

            tracking_aircraft_key = ",".join(
                str(int(aid))
                for aid in sorted(
                    {
                        int(_coerce_int(item.get("aircraft_id")) or 0)
                        for item in matched_assignments
                        if _coerce_int(item.get("aircraft_id")) is not None
                    }
                )
            )
            dedupe_keys = [
                "|".join(
                    [
                        "plan",
                        str(int(current_mission_plan_id)),
                        str(int(target_id)),
                        tracking_aircraft_key,
                        "destroyed",
                    ]
                ),
                "|".join(["target", str(int(target_id)), tracking_aircraft_key, "destroyed"]),
            ]
            duplicate_key = None
            for dedupe_key in dedupe_keys:
                prev_ts = self._attack_close_trigger_history.get(dedupe_key)
                if prev_ts is not None and (int(now_ts) - int(prev_ts)) < int(close_cooldown_ms):
                    duplicate_key = dedupe_key
                    break
            if duplicate_key is not None:
                logs.append(
                    f"[0402-close] duplicate close event suppressed "
                    f"(targetID={target_id}, trackingAircraft={tracking_aircraft_key})"
                )
                continue
            for dedupe_key in dedupe_keys:
                self._attack_close_trigger_history[dedupe_key] = int(now_ts)

            plan_ids = allocate_mission_plan_ids(1)
            if not plan_ids:
                logs.append(f"[0402-close] missionPlanID allocation failed; skip targetID={target_id}")
                continue

            detail_payload = {
                "trigger": "0402",
                "triggerType": "attackClosedDestroyed",
                "closureReason": "destroyed",
                "sourceMissionPlanID": int(current_mission_plan_id),
                "currentMissionPlanID": int(current_mission_plan_id),
                "missionPlanID": int(plan_ids[0]),
                "targetID": int(target_id),
                "watcherID": resolved_watcher_id if resolved_watcher_id is not None else watcher_id,
                "targetType": _coerce_int(entry.get("targetType")),
                "coordinate": dict(entry.get("coordinate") or {}),
                "targetInFrame": entry.get("targetInFrame"),
                "isDestroyed": True,
                "threat": entry.get("threat"),
                "matchStrategy": match_strategy,
                "trackingLineageRecovered": bool(lineage_recovered),
                "trackingAircraftIDList": [
                    int(_coerce_int(item.get("aircraft_id")))
                    for item in matched_assignments
                    if _coerce_int(item.get("aircraft_id")) is not None
                ],
            }
            if (
                watcher_id is not None
                and resolved_watcher_id is not None
                and int(watcher_id) != int(resolved_watcher_id)
            ):
                detail_payload["observedWatcherID"] = int(watcher_id)
            payloads.append(
                {
                    "timestamp": int(now_ts),
                    "source": "MSM",
                    "inputMissionPackageID": int(package_id) if package_id is not None else 0,
                    "replanRequestTime": {"replanRequestTimestamp": int(now_ts)},
                    "replanLevel": int(self.REPLAN_LEVEL),
                    "replanRequest": _build_attack_close_reason(
                        int(target_id),
                        resolved_watcher_id if resolved_watcher_id is not None else watcher_id,
                    ),
                    "inputMissionIDList": [{"inputMissionID": int(mid)} for mid in mission_ids],
                    "missionPlanIDList": [{"missionPlanID": int(plan_ids[0])}],
                    "replanDetail": detail_payload,
                }
            )
            logs.append(
                f"[0402-close] attack target destroyed -> post-attack rejoin request queued "
                f"(targetID={target_id}, trackingAircraft={[item.get('aircraft_id') for item in matched_assignments]})"
            )
        return payloads

    def _pick_candidate(
        self,
        info: dict[str, Any],
        new_detections: list[dict[str, Any]],
    ) -> tuple[str | None, dict[str, Any] | None]:
        if new_detections:
            entry = new_detections[0]
            key = str(entry.get("key") or "")
            if key:
                return key, dict(entry)

        target_map = info.get("targetList") or {}
        if isinstance(target_map, dict):
            for key, entry in target_map.items():
                if not isinstance(entry, dict):
                    continue
                key_str = str(key)
                if not _is_actionable_target_entry(key_str, entry):
                    continue
                candidate = dict(entry)
                candidate["key"] = key_str
                return key_str, candidate
        return None, None

    def _deferred_candidates(
        self,
        info: dict[str, Any],
        package_id: int | None,
        logs: list[str],
    ) -> list[dict[str, Any]]:
        stored_entries = list_deferred_attack_targets(package_id)
        if not stored_entries:
            return []

        target_map = info.get("targetList") if isinstance(info, dict) else None
        target_map = target_map if isinstance(target_map, dict) else {}
        out: list[dict[str, Any]] = []
        stale_ids: list[int] = []

        for stored in stored_entries:
            target_id = _coerce_int(stored.get("targetID") if isinstance(stored, dict) else None)
            if target_id is None or target_id <= 0:
                continue

            latest_candidates: list[dict[str, Any]] = []
            target_seen = False
            terminal_seen = False
            for key, entry in target_map.items():
                if not isinstance(entry, dict):
                    continue
                if _coerce_int(entry.get("targetID")) != int(target_id):
                    continue
                target_seen = True
                is_ignored = _coerce_int(entry.get("isIgnored"))
                if bool(entry.get("isDestroyed")) or (is_ignored is not None and is_ignored != 0):
                    terminal_seen = True
                key_str = str(key)
                if not _is_actionable_target_entry(key_str, entry):
                    continue
                candidate = dict(entry)
                candidate["key"] = key_str
                latest_candidates.append(candidate)

            if latest_candidates:
                latest_candidates.sort(key=_tracking_watcher_preference_key)
                candidate = dict(latest_candidates[0])
            elif isinstance(stored, dict):
                if target_seen:
                    if terminal_seen:
                        stale_ids.append(int(target_id))
                    continue
                key_str = str(stored.get("key") or stored.get("targetKey") or target_id)
                candidate = dict(stored)
                candidate["key"] = key_str
                if not _is_actionable_target_entry(key_str, candidate):
                    continue
            else:
                continue

            candidate["deferredAttackResume"] = True
            out.append(candidate)

        if stale_ids:
            try:
                cleared = clear_deferred_attack_targets(package_id, stale_ids)
            except Exception:
                cleared = []
            if cleared:
                logs.append(
                    "[0402] stale deferred attack targets cleared: "
                    + ",".join(str(target_id) for target_id in cleared)
                )

        return _dedupe_candidates_by_target_id(out)

    def resume_deferred_attacks(
        self,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        mission_ids, package_id = self._collect_pending_input_ids(current_mission_plan_id)
        if package_id is None or not list_deferred_attack_targets(package_id):
            return [], []
        return self.on_situation_awareness(
            None,
            system_mode=system_mode,
            current_mission_plan_id=current_mission_plan_id,
        )

    def on_situation_awareness(
        self,
        payload: object | None,
        *,
        system_mode: int | None,
        current_mission_plan_id: int | None,
    ) -> tuple[list[dict[str, Any]], list[str]]:
        with self._lock:
            logs: list[str] = []
            now_ts = int(self._now_ms())
            info, new_detections = update_target_info_from_0402(payload)
            mission_ids, package_id = self._collect_pending_input_ids(current_mission_plan_id)
            if not mission_ids:
                mission_ids = collect_input_mission_ids()
            close_payloads: list[dict[str, Any]] = []
            if system_mode in (3, 4):
                close_payloads = self._build_attack_close_payloads(
                    message=payload,
                    target_info=info,
                    current_mission_plan_id=current_mission_plan_id,
                    mission_ids=mission_ids,
                    package_id=package_id,
                    now_ts=now_ts,
                    logs=logs,
                )
            if not _target_detection_enabled():
                return close_payloads, logs

            if new_detections:
                logs.append(f"[0402] new detections: {len(new_detections)}")

            candidates: list[dict[str, Any]] = []
            if new_detections:
                target_map = info.get("targetList") if isinstance(info, dict) else {}
                for raw in new_detections:
                    key = str(raw.get("key") or "")
                    if not key:
                        continue
                    entry = None
                    if isinstance(target_map, dict):
                        entry = target_map.get(key)
                    if not isinstance(entry, dict):
                        entry = dict(raw)
                    else:
                        entry = dict(entry)
                        for extra_key, extra_value in dict(raw).items():
                            if extra_key in {"key", "priorMissionMatched", "priorMissionRediscovered", "handledStateReset", "priorMissionIDList", "priorMissionTypeList"}:
                                entry[extra_key] = extra_value
                    entry["key"] = key
                    candidates.append(entry)
            else:
                deferred_candidates = self._deferred_candidates(info, package_id, logs)
                if deferred_candidates:
                    candidates.extend(deferred_candidates)
                    logs.append(
                        "[0402] deferred attack candidates loaded: "
                        + ",".join(
                            str(_coerce_int(item.get("targetID")))
                            for item in deferred_candidates
                            if _coerce_int(item.get("targetID")) is not None
                        )
                    )
                else:
                    key, entry = self._pick_candidate(info, [])
                    if key and isinstance(entry, dict):
                        entry.setdefault("key", key)
                        candidates.append(entry)

            if new_detections:
                existing_target_ids = {
                    int(target_id)
                    for target_id in (
                        _coerce_int(item.get("targetID"))
                        for item in candidates
                        if isinstance(item, dict)
                    )
                    if target_id is not None
                }
                deferred_candidates = self._deferred_candidates(info, package_id, logs)
                appended_deferred_ids: list[int] = []
                for entry in deferred_candidates:
                    target_id = _coerce_int(entry.get("targetID"))
                    if target_id is None or int(target_id) in existing_target_ids:
                        continue
                    candidates.append(entry)
                    existing_target_ids.add(int(target_id))
                    appended_deferred_ids.append(int(target_id))
                if appended_deferred_ids:
                    logs.append(
                        "[0402] deferred attack candidates appended: "
                        + ",".join(str(target_id) for target_id in appended_deferred_ids)
                    )

            if not candidates:
                return close_payloads, logs

            filtered_candidates: list[dict[str, Any]] = []
            for entry in candidates:
                if not isinstance(entry, dict):
                    continue
                key = str(entry.get("key") or "")
                if not key:
                    target_id = _coerce_int(entry.get("targetID"))
                    if target_id is not None:
                        key = str(target_id)
                        entry["key"] = key
                if not key:
                    continue
                if not _is_actionable_target_entry(key, entry):
                    continue
                filtered_candidates.append(entry)
            if len(filtered_candidates) != len(candidates):
                logs.append(
                    f"[0402] actionable filter: {len(candidates)} -> {len(filtered_candidates)}"
                )
            candidates = filtered_candidates
            if not candidates:
                return close_payloads, logs

            deduped = _dedupe_candidates_by_target_id(candidates)
            if len(deduped) != len(candidates):
                logs.append(f"[0402] candidate dedupe by targetID: {len(candidates)} -> {len(deduped)}")
            candidates = deduped
            if not candidates:
                return close_payloads, logs
            if len(candidates) > 1:
                candidates.sort(key=_candidate_priority_key)
                logs.append("[0402] candidate order refreshed by target-type priority")
            if len(candidates) > 3:
                logs.append(f"[0402] candidate cap applied: {len(candidates)} -> 3")
                candidates = candidates[:3]

            if system_mode not in (3, 4):
                logs.append(f"[0402] replan skipped: mode={system_mode} (need 3/4)")
                return close_payloads, logs

            used_manned = get_used_manned_ids(package_id)
            # A slot is only genuinely occupied while that aircraft still holds
            # an attack. Bookkeeping can outlive the attack - a post-attack
            # replan that never ran leaves the flag set - and with every slot
            # stale-marked no attack can start, so no post-attack replan can
            # ever run to clear it. Reconcile against the live plan first.
            stale_manned = _stale_attack_slots(
                package_id, current_mission_plan_id, used_manned, logs
            )
            if stale_manned:
                used_manned = {aid for aid in used_manned if aid not in stale_manned}
            available_slots = sum(1 for aid in _attack_manned_ids() if aid not in used_manned)
            updated_info = False
            follow_up_attack_mode = False
            if available_slots <= 0:
                follow_up_candidates = [
                    entry
                    for entry in candidates
                    if isinstance(entry, dict)
                    and (
                        bool(entry.get("deferredAttackResume"))
                        or (_coerce_int(entry.get("isUsed")) or 0) == 0
                    )
                ]
                active_follow_up_attack = bool(follow_up_candidates) and _active_attack_tracking_exists_for_plan(
                    current_mission_plan_id
                )
                if active_follow_up_attack:
                    follow_up_attack_mode = True
                    logs.append(
                        "[0402] attack slots occupied -> follow-up attack candidate evaluating "
                        f"(inputMissionPackageID={package_id}, usedManned={sorted(int(aid) for aid in used_manned)})"
                    )
                else:
                    deferred_ids: list[int] = []
                    try:
                        deferred_ids = defer_attack_targets(
                            package_id,
                            current_mission_plan_id,
                            candidates,
                            now_ms=now_ts,
                            reason="attack_slots_exhausted",
                        )
                    except Exception as exc:
                        logs.append(f"[0402] deferred attack target save failed: {exc}")
                    if deferred_ids:
                        logs.append(
                            "[0402] attack slots exhausted -> deferred targets retained "
                            f"(inputMissionPackageID={package_id}, targets={deferred_ids})"
                        )
                    else:
                        logs.append(
                            f"[0402] replan skipped: attack slots exhausted (inputMissionPackageID={package_id})"
                        )
                    return close_payloads, logs

            eligible_candidates: list[dict[str, Any]] = []
            for entry in candidates:
                key = str(entry.get("key") or "")
                if not key or not isinstance(entry, dict):
                    continue
                target_id = _coerce_int(entry.get("targetID"))
                if target_id is None:
                    continue
                if _target_is_blocked_by_state(info, target_id):
                    continue
                is_prior_rediscovery = _is_prior_mission_rediscovery(entry)
                prev_ts = self._target_trigger_history.get(target_id)
                cooldown_ms = int(_target_detection_config().get("cooldown_ms", 10000))
                if (not is_prior_rediscovery) and prev_ts is not None and (now_ts - prev_ts) < cooldown_ms:
                    continue
                if is_prior_rediscovery:
                    prior_ids = entry.get("priorMissionIDList") or []
                    logs.append(
                        f"[0402] prior-mission rediscovery accepted: targetID={target_id}, priorMissionID={prior_ids}"
                    )
                eligible_candidates.append(dict(entry))

            if not eligible_candidates:
                return close_payloads, logs

            if len(eligible_candidates) > 1:
                eligible_candidates.sort(key=_candidate_priority_key)

            deferred_resume_ids = [
                int(target_id)
                for target_id in (
                    _coerce_int(entry.get("targetID"))
                    for entry in eligible_candidates
                    if isinstance(entry, dict) and bool(entry.get("deferredAttackResume"))
                )
                if target_id is not None
            ]

            primary_entry = dict(eligible_candidates[0])
            primary_target_id = _coerce_int(primary_entry.get("targetID"))
            if primary_target_id is None:
                return close_payloads, logs

            bundle_targets, total_bundle_count = _build_attack_target_bundle(
                info,
                primary_entry=primary_entry,
                limit=3,
            )
            if not bundle_targets:
                return close_payloads, logs
            bundle_target_ids = {
                int(target_id)
                for target_id in (
                    _coerce_int(item.get("targetID"))
                    for item in bundle_targets
                    if isinstance(item, dict)
                )
                if target_id is not None
            }
            if follow_up_attack_mode and int(primary_target_id) not in bundle_target_ids:
                try:
                    deferred_ids = defer_attack_targets(
                        package_id,
                        current_mission_plan_id,
                        [primary_entry],
                        now_ms=now_ts,
                        reason="max_tracked_targets_reached",
                    )
                except Exception as exc:
                    deferred_ids = []
                    logs.append(f"[0402] follow-up deferred target retain failed: {exc}")
                target_text = ",".join(str(target_id) for target_id in sorted(bundle_target_ids)) or "-"
                if deferred_ids:
                    logs.append(
                        "[0402] follow-up attack skipped: deferred target outside max-tracked bundle "
                        f"(targetID={primary_target_id}, bundleTargets={target_text}, deferred={deferred_ids})"
                    )
                else:
                    logs.append(
                        "[0402] follow-up attack skipped: deferred target outside max-tracked bundle "
                        f"(targetID={primary_target_id}, bundleTargets={target_text})"
                    )
                return close_payloads, logs
            if follow_up_attack_mode:
                target_text = ",".join(str(target_id) for target_id in sorted(bundle_target_ids)) or "-"
                logs.append(
                    "[0402] attack slots occupied -> follow-up attack replan allowed "
                    f"(inputMissionPackageID={package_id}, bundleTargets={target_text})"
                )

            bundle_ids: list[int] = []
            bundle_changed = False
            primary_bundle_entry: dict[str, Any] | None = None
            for bundle_entry in bundle_targets:
                bundle_target_id = _coerce_int(bundle_entry.get("targetID"))
                bundle_watcher_id = _coerce_int(bundle_entry.get("watcherID"))
                bundle_key = str(bundle_entry.get("targetKey") or "")
                if bundle_target_id is not None and bundle_target_id not in bundle_ids:
                    bundle_ids.append(int(bundle_target_id))
                if bundle_target_id is not None and bundle_target_id == int(primary_target_id):
                    primary_bundle_entry = bundle_entry
                if bundle_target_id is None or bundle_watcher_id is None:
                    continue
                sync_entry = {
                    "targetID": int(bundle_target_id),
                    "watcherID": int(bundle_watcher_id),
                }
                if bundle_key:
                    sync_entry["key"] = bundle_key
                synced_key, changed = _sync_target_watcher(
                    info,
                    bundle_key,
                    sync_entry,
                )
                if synced_key:
                    bundle_entry["targetKey"] = synced_key
                    if bundle_target_id == int(primary_target_id):
                        primary_entry["key"] = synced_key
                bundle_changed = bundle_changed or changed
            if bundle_changed:
                updated_info = True

            if len(bundle_targets) > 1:
                logs.append(
                    f"[0402] accumulated attack bundle prepared: count={len(bundle_targets)}, targets={bundle_ids}"
                )

            for bundle_entry in bundle_targets:
                bundle_target_id = _coerce_int(bundle_entry.get("targetID"))
                if bundle_target_id is None:
                    continue
                bundle_watcher_id = _coerce_int(bundle_entry.get("watcherID"))
                bundle_key = str(bundle_entry.get("targetKey") or bundle_entry.get("key") or "")
                if _mark_target_used_in_info(
                    info,
                    key=bundle_key,
                    target_id=int(bundle_target_id),
                    watcher_id=bundle_watcher_id,
                ):
                    updated_info = True
                bundle_entry["isUsed"] = 1
                if int(bundle_target_id) == int(primary_target_id):
                    primary_entry["isUsed"] = 1

            primary_bundle_entry = primary_bundle_entry or dict(bundle_targets[0])
            key = str(primary_bundle_entry.get("targetKey") or primary_entry.get("key") or "")
            target_id = _coerce_int(primary_bundle_entry.get("targetID")) or int(primary_target_id)
            watcher_id = _coerce_int(primary_bundle_entry.get("watcherID"))
            target_type = _coerce_int(primary_bundle_entry.get("targetType"))
            if target_type is None:
                target_type = _coerce_int(primary_entry.get("targetType"))

            if watcher_id is None:
                logs.append(f"[0402] watcher unavailable; skip targetID={target_id}")
                self._target_trigger_history[target_id] = now_ts
                return close_payloads, logs

            reason_text = _format_target_bundle_reason(
                watcher_id,
                target_type,
                target_id,
                bundle_count=len(bundle_targets),
            )

            presets = _option_presets()
            plan_ids = allocate_mission_plan_ids(len(presets))
            if not plan_ids:
                logs.append("[0402] missionPlanID allocation failed; skip")
                return close_payloads, logs

            pending_options: list[dict[str, Any]] = []
            for (option_id, name), plan_id in zip(presets, plan_ids):
                pending_options.append(
                    {
                        "optionID": int(option_id),
                        "optionName": str(name),
                        "missionPlanID": int(plan_id),
                    }
                )

            detail_payload = {
                "trigger": "0402",
                "sourceMissionPlanID": int(current_mission_plan_id) if current_mission_plan_id is not None else None,
                "currentMissionPlanID": int(current_mission_plan_id) if current_mission_plan_id is not None else None,
                "targetKey": key,
                "targetID": target_id,
                "watcherID": watcher_id,
                "targetType": target_type,
                "threat": primary_entry.get("threat"),
                "targetInFrame": primary_entry.get("targetInFrame"),
                "coordinate": primary_entry.get("coordinate"),
                "preferredOptionCount": len(presets),
                "attackTargetList": list(bundle_targets),
                "attackTargetCount": len(bundle_targets),
                "attackTargetTotalCount": int(total_bundle_count),
                "attackTargetOverflow": bool(total_bundle_count > len(bundle_targets)),
                "attackTargetIDs": list(bundle_ids),
                "targetBundle": list(bundle_targets),
                "targetBundleCount": len(bundle_targets),
                "targetBundleMode": (
                    "follow_up"
                    if follow_up_attack_mode
                    else ("bundle" if len(bundle_targets) > 1 else "single")
                ),
                "followUpAttackMode": bool(follow_up_attack_mode),
                "maxTrackedTargets": 3,
                "snapshot": primary_entry,
            }
            if follow_up_attack_mode:
                detail_payload["followUpReason"] = "attack_slots_occupied"
                detail_payload["occupiedMannedAircraftIDs"] = sorted(int(aid) for aid in used_manned)
            if deferred_resume_ids:
                detail_payload["deferredAttackResume"] = True
                detail_payload["deferredAttackTargetIDs"] = sorted(set(deferred_resume_ids))
                detail_payload["deferredReason"] = "attack_slots_exhausted"

            ts = now_ts
            payload_0902: dict[str, Any] = {
                "timestamp": ts,
                "source": "MSM",
                "inputMissionPackageID": int(package_id) if package_id is not None else 0,
                "replanRequestTime": {"replanRequestTimestamp": ts},
                "replanLevel": int(self.REPLAN_LEVEL),
                "replanRequest": reason_text,
                "inputMissionIDList": [{"inputMissionID": int(mid)} for mid in mission_ids],
                "pendingOptionList": pending_options,
                "replanDetail": detail_payload,
            }

            payloads: list[dict[str, Any]] = list(close_payloads)
            payloads.append(payload_0902)
            history_targets = list(bundle_ids) if bundle_ids else [int(target_id)]
            for history_target_id in history_targets:
                self._target_trigger_history[int(history_target_id)] = ts
            try:
                cleared_deferred = clear_deferred_attack_targets(package_id, history_targets)
            except Exception as exc:
                cleared_deferred = []
                logs.append(f"[0402] deferred attack clear failed: {exc}")
            if cleared_deferred:
                logs.append(
                    "[0402] deferred attack targets resumed: "
                    + ",".join(str(target_id) for target_id in cleared_deferred)
                )
            if _is_prior_mission_rediscovery(primary_entry):
                consumed = prior_target_rediscovery_store.consume_target(
                    target_id,
                    timestamp=ts,
                    reason="0402_replan_dispatched",
                    mission_plan_ids=plan_ids,
                    watcher_id=watcher_id,
                    key=key,
                )
                logs.append(
                    f"[0402] prior-mission rediscovery consumed: targetID={target_id}, entries={len(consumed)}"
                )
            logs.append(
                f"[0402] bundled target replan prepared: key={key}, primaryTargetID={target_id}, targets={history_targets}"
            )

            if updated_info:
                try:
                    save_target_info(info)
                except Exception as exc:
                    logs.append(f"[0402] targetInfo save failed: {exc}")

            return payloads, logs

    def _collect_pending_input_ids(
        self, current_mission_plan_id: int | None
    ) -> tuple[list[int], int | None]:
        package_id = _extract_input_package_id(current_mission_plan_id)
        pending = _pending_input_ids_from_package(package_id)
        return pending, package_id


