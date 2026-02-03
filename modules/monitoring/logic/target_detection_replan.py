# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Callable

from modules.monitoring.logic.init_replan import allocate_mission_plan_ids, collect_input_mission_ids
from modules.monitoring.logic.mission_update import load_db_json
from modules.monitoring.logic.target_info import (
    update_target_info_from_0402,
    mark_targets_as_used,
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
        if entry.get("isDestroyed"):
            continue
        if _coerce_int(entry.get("isIgnored")) == 1:
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


def _is_actionable_target_entry(key: str, entry: dict[str, Any]) -> bool:
    if key.startswith("unknown-"):
        return False
    target_id = _coerce_int(entry.get("targetID"))
    if target_id is None:
        return False
    if _coerce_int(entry.get("isUsed")) == 1:
        return False
    if _coerce_int(entry.get("isIgnored")) == 1:
        return False
    if bool(entry.get("isDestroyed")):
        return False
    return True


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
        self._target_trigger_history: dict[str, int] = {}

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
        logs: list[str] = []
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

        if system_mode not in (3, 4):
            logs.append(f"[0402] replan skipped: mode={system_mode} (need 3/4)")
            return [], logs

        mission_ids, package_id = self._collect_pending_input_ids(current_mission_plan_id)
        if not mission_ids:
            mission_ids = collect_input_mission_ids()

        used_manned = get_used_manned_ids(package_id)
        available_slots = sum(1 for aid in ATTACK_MANNED_IDS if aid not in used_manned)
        if available_slots <= 0:
            logs.append(
                f"[0402] replan skipped: attack slots exhausted (inputMissionPackageID={package_id})"
            )
            for entry in candidates:
                key = str(entry.get("key") or "")
                last_updated = _extract_target_timestamp(entry)
                if key and last_updated is not None:
                    self._target_trigger_history[key] = last_updated
            return [], logs

        if len(candidates) > available_slots:
            logs.append(
                f"[0402] target replan limited by attack slots ({available_slots}/{len(candidates)})"
            )
            candidates = candidates[:available_slots]

        used_watchers = _collect_active_watchers(info)
        payloads: list[dict[str, Any]] = []
        updated_info = False

        for entry in candidates:
            key = str(entry.get("key") or "")
            if not key or not isinstance(entry, dict):
                continue
            last_updated = _extract_target_timestamp(entry)
            if last_updated is not None:
                prev_ts = self._target_trigger_history.get(key)
                if prev_ts is not None and last_updated <= prev_ts:
                    continue

            target_id = _coerce_int(entry.get("targetID"))
            if target_id is None:
                continue

            watcher_id = _assign_watcher_id(entry, used_watchers)
            entry["watcherID"] = watcher_id
            key, changed = _sync_target_watcher(info, key, entry)
            if changed:
                updated_info = True
                entry["key"] = key

            if watcher_id is None:
                logs.append(f"[0402] watcher unavailable; skip targetID={target_id}")
                if last_updated is not None:
                    self._target_trigger_history[key] = last_updated
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

            ts = int(self._now_ms())
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
            self._target_trigger_history[key] = last_updated or ts
            try:
                mark_targets_as_used(
                    [
                        {
                            "key": key,
                            "targetID": target_id,
                            "watcherID": watcher_id,
                        }
                    ]
                )
            except Exception:
                pass
            logs.append(f"[0402] target replan prepared: key={key}, targetID={target_id}")

        if updated_info:
            try:
                save_target_info(info)
            except Exception:
                pass

        return payloads, logs

    def _collect_pending_input_ids(
        self, current_mission_plan_id: int | None
    ) -> tuple[list[int], int | None]:
        package_id = _extract_input_package_id(current_mission_plan_id)
        pending = _pending_input_ids_from_package(package_id)
        return pending, package_id


