# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import os
import threading
from typing import Any, Callable

from modules.monitoring.logic.init_replan import allocate_mission_plan_ids, collect_input_mission_ids
from modules.monitoring.logic.mission_update import load_db_json
from modules.monitoring.logic.target_info import (
    update_target_info_from_0402,
    save_target_info,
)
try:
    from modules.mission_planning.attack_assignment_state import get_used_manned_ids
except Exception:  # pragma: no cover - optional dependency
    def get_used_manned_ids(_input_package_id: int | None) -> set[int]:
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
    for candidate in WATCHER_UAV_IDS:
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
    return f"{watcher_label} - {target_label}(ID-{target_id}) 발견으로 인한 재계획"


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
    best_by_target: dict[int, tuple[tuple[int, int, int, int], dict[str, Any]]] = {}
    for entry in candidates:
        if not isinstance(entry, dict):
            continue
        target_id = _coerce_int(entry.get("targetID"))
        if target_id is None:
            continue
        priority = (
            1 if _coerce_int(entry.get("watcherID")) is not None else 0,
            1 if bool(entry.get("targetInFrame")) else 0,
            1 if isinstance(entry.get("coordinate"), dict) else 0,
            _extract_target_timestamp(entry) or -1,
        )
        current = best_by_target.get(target_id)
        if current is None or priority > current[0]:
            best_by_target[target_id] = (priority, dict(entry))
    return [item[1] for item in best_by_target.values()]


class TargetDetectionCoordinator:
    REPLAN_LEVEL = 2
    REPLAN_COOLDOWN_MS = int(os.getenv("MSM_0402_REPLAN_COOLDOWN_MS", "10000"))

    def __init__(
        self,
        *,
        now_fn: Callable[[], int],
        logger: Callable[[str], None] | None = None,
    ) -> None:
        self._now_ms = now_fn
        self._log = logger
        self._target_trigger_history: dict[int, int] = {}
        self._lock = threading.Lock()

    def _log_line(self, text: str) -> None:
        if self._log:
            self._log(text)

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
                    entry["key"] = key
                    candidates.append(entry)
            else:
                key, entry = self._pick_candidate(info, [])
                if key and isinstance(entry, dict):
                    entry.setdefault("key", key)
                    candidates.append(entry)

            if not candidates:
                return [], logs

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
                return [], logs

            deduped = _dedupe_candidates_by_target_id(candidates)
            if len(deduped) != len(candidates):
                logs.append(f"[0402] candidate dedupe by targetID: {len(candidates)} -> {len(deduped)}")
            candidates = deduped
            if not candidates:
                return [], logs

            if system_mode not in (3, 4):
                logs.append(f"[0402] replan skipped: mode={system_mode} (need 3/4)")
                return [], logs

            mission_ids, package_id = self._collect_pending_input_ids(current_mission_plan_id)
            if not mission_ids:
                mission_ids = collect_input_mission_ids()

            used_manned = get_used_manned_ids(package_id)
            available_slots = sum(1 for aid in ATTACK_MANNED_IDS if aid not in used_manned)
            updated_info = False
            if available_slots <= 0:
                logs.append(
                    f"[0402] replan skipped: attack slots exhausted (inputMissionPackageID={package_id})"
                )
                for entry in candidates:
                    target_id = _coerce_int(entry.get("targetID"))
                    if target_id is not None:
                        self._target_trigger_history[target_id] = now_ts
                return [], logs

            if len(candidates) > available_slots:
                logs.append(
                    f"[0402] target replan limited by attack slots ({available_slots}/{len(candidates)})"
                )
                overflow = candidates[available_slots:]
                for entry in overflow:
                    if not isinstance(entry, dict):
                        continue
                    target_id = _coerce_int(entry.get("targetID"))
                    if target_id is None or target_id <= 0:
                        continue
                    key = str(entry.get("key") or target_id)
                    watcher_id = _coerce_int(entry.get("watcherID"))
                    self._target_trigger_history[target_id] = now_ts
                    if _mark_target_used_in_info(
                        info,
                        key=key,
                        target_id=target_id,
                        watcher_id=watcher_id,
                    ):
                        updated_info = True
                candidates = candidates[:available_slots]
                logs.append(
                    f"[0402] overflow targets marked used: {len(overflow)}"
                )

            used_watchers = _collect_active_watchers(info)
            payloads: list[dict[str, Any]] = []

            for entry in candidates:
                key = str(entry.get("key") or "")
                if not key or not isinstance(entry, dict):
                    continue
                target_id = _coerce_int(entry.get("targetID"))
                if target_id is None:
                    continue
                if _target_is_blocked_by_state(info, target_id):
                    continue
                prev_ts = self._target_trigger_history.get(target_id)
                if prev_ts is not None and (now_ts - prev_ts) < self.REPLAN_COOLDOWN_MS:
                    continue

                watcher_id = _assign_watcher_id(entry, used_watchers)
                entry["watcherID"] = watcher_id
                key, changed = _sync_target_watcher(info, key, entry)
                if changed:
                    updated_info = True
                    entry["key"] = key

                if watcher_id is None:
                    logs.append(f"[0402] watcher unavailable; skip targetID={target_id}")
                    self._target_trigger_history[target_id] = now_ts
                    continue

                target_type = _coerce_int(entry.get("targetType"))
                reason_text = _format_target_reason(watcher_id, target_type, target_id)

                plan_ids = allocate_mission_plan_ids(len(OPTION_PRESETS))
                if not plan_ids:
                    logs.append("[0402] missionPlanID allocation failed; skip")
                    continue

                pending_options: list[dict[str, Any]] = []
                for (option_id, name), plan_id in zip(OPTION_PRESETS, plan_ids):
                    pending_options.append(
                        {
                            "optionID": int(option_id),
                            "optionName": str(name),
                            "missionPlanID": int(plan_id),
                        }
                    )

                detail_payload = {
                    "trigger": "0402",
                    "targetKey": key,
                    "targetID": target_id,
                    "watcherID": watcher_id,
                    "targetType": target_type,
                    "threat": entry.get("threat"),
                    "targetInFrame": entry.get("targetInFrame"),
                    "coordinate": entry.get("coordinate"),
                    "preferredOptionCount": len(OPTION_PRESETS),
                    "snapshot": entry,
                }

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

                payloads.append(payload_0902)
                self._target_trigger_history[target_id] = ts
                if _mark_target_used_in_info(
                    info,
                    key=key,
                    target_id=target_id,
                    watcher_id=watcher_id,
                ):
                    updated_info = True
                logs.append(f"[0402] target replan prepared: key={key}, targetID={target_id}")

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


