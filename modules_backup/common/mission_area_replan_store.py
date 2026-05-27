from __future__ import annotations

import json
import time
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

from modules.common import db_paths


_STORE_DIR = "mission_area_replan"
_DETAIL_PREFIX = "mission_area_snapshot"
_AUDIT_BASENAME = "mission_area_snapshot_audit.jsonl"
_AREA_EPSILON_M2 = 10.0
_AREA_GROWTH_TOLERANCE_RATIO = 0.01


def _detail_dir() -> Path:
    return db_paths.get_db_subpath("DSS_Internal", _STORE_DIR)


def _detail_path(mission_plan_id: int) -> Path:
    return _detail_dir() / f"{_DETAIL_PREFIX}_{int(mission_plan_id)}.json"


def _audit_path() -> Path:
    return _detail_dir() / _AUDIT_BASENAME


def _to_int(value: Any) -> Optional[int]:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        parsed = float(value)
    except Exception:
        return None
    if parsed != parsed:
        return None
    return parsed


def _remaining_detail_has_geometry(detail: Any) -> bool:
    if not isinstance(detail, dict):
        return False
    line_list = detail.get("lineList")
    if isinstance(line_list, list) and line_list:
        return True
    area_list = detail.get("areaList")
    if isinstance(area_list, list) and area_list:
        return True
    coordinate_list = detail.get("coordinateList")
    return isinstance(coordinate_list, list) and len(coordinate_list) >= 2


def _mission_input_id(mission: Any) -> Optional[int]:
    if not isinstance(mission, dict):
        return None
    input_id = _to_int(mission.get("inputMissionID"))
    return int(input_id) if input_id is not None and input_id > 0 else None


def _mission_remaining_area(mission: Dict[str, Any]) -> Optional[float]:
    value = _to_float(mission.get("remainingAreaM2"))
    return float(value) if value is not None and value >= 0.0 else None


def _mission_is_done(mission: Dict[str, Any]) -> bool:
    return bool(mission.get("isDone"))


def _growth_tolerance(existing_area_m2: float) -> float:
    return max(float(_AREA_EPSILON_M2), float(existing_area_m2) * float(_AREA_GROWTH_TOLERANCE_RATIO))


def _audit(event: str, payload: Dict[str, Any]) -> None:
    try:
        path = _audit_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        row = {
            "savedAt": datetime.now(timezone.utc).isoformat(),
            "event": str(event),
            **dict(payload or {}),
        }
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
    except Exception:
        pass


def _find_entry(snapshot: Any, input_mission_id: int) -> Optional[Dict[str, Any]]:
    if not isinstance(snapshot, dict):
        return None
    for mission in snapshot.get("missions") or []:
        if not isinstance(mission, dict):
            continue
        if _mission_input_id(mission) == int(input_mission_id):
            return mission
    return None


def _iter_snapshot_paths() -> Iterable[Path]:
    try:
        paths = list(_detail_dir().glob(f"{_DETAIL_PREFIX}_*.json"))
    except Exception:
        return []
    return sorted(
        paths,
        key=lambda path: (
            path.stat().st_mtime if path.exists() else 0.0,
            path.name,
        ),
        reverse=True,
    )


def _merge_with_existing_snapshot(
    mission_plan_id: int,
    payload: Dict[str, Any],
    existing: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    if not isinstance(existing, dict):
        return payload
    existing_by_input: Dict[int, Dict[str, Any]] = {}
    for mission in existing.get("missions") or []:
        input_id = _mission_input_id(mission)
        if input_id is not None:
            existing_by_input[int(input_id)] = mission

    missions = payload.get("missions")
    if not isinstance(missions, list):
        return payload

    merged_missions = []
    changed = False
    for mission in missions:
        if not isinstance(mission, dict):
            merged_missions.append(mission)
            continue
        input_id = _mission_input_id(mission)
        if input_id is None or input_id not in existing_by_input:
            merged_missions.append(mission)
            continue

        previous = existing_by_input[int(input_id)]
        previous_done = _mission_is_done(previous)
        incoming_done = _mission_is_done(mission)
        previous_has_geometry = _remaining_detail_has_geometry(previous.get("remainingDetail"))
        incoming_has_geometry = _remaining_detail_has_geometry(mission.get("remainingDetail"))
        previous_area = _mission_remaining_area(previous)
        incoming_area = _mission_remaining_area(mission)

        keep_previous = False
        reason = ""
        if previous_done and not incoming_done:
            keep_previous = True
            reason = "previous_done"
        elif incoming_done:
            keep_previous = False
        elif previous_has_geometry and not incoming_has_geometry:
            keep_previous = True
            reason = "incoming_empty_not_done"
        elif (
            previous_area is not None
            and incoming_area is not None
            and incoming_area > previous_area + _growth_tolerance(previous_area)
        ):
            keep_previous = True
            reason = "remaining_area_grew"

        if keep_previous:
            preserved = deepcopy(previous)
            preserved["missionPlanID"] = int(mission_plan_id)
            merged_missions.append(preserved)
            changed = True
            _audit(
                "snapshot_entry_preserved",
                {
                    "missionPlanID": int(mission_plan_id),
                    "inputMissionID": int(input_id),
                    "reason": reason,
                    "previousRemainingAreaM2": previous_area,
                    "incomingRemainingAreaM2": incoming_area,
                },
            )
        else:
            merged_missions.append(mission)

    if changed:
        payload = dict(payload)
        payload["missions"] = merged_missions
        payload["missionCount"] = len([item for item in merged_missions if isinstance(item, dict)])
    return payload


def save_snapshot(mission_plan_id: int, payload: Dict[str, Any]) -> Path:
    data = dict(payload or {})
    data.setdefault("missionPlanID", int(mission_plan_id))
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    path = _detail_path(mission_plan_id)
    path.parent.mkdir(parents=True, exist_ok=True)
    existing = load_snapshot(int(mission_plan_id))
    data = _merge_with_existing_snapshot(int(mission_plan_id), data, existing)

    serialized = json.dumps(data, ensure_ascii=False, indent=2)
    try:
        if path.exists() and path.read_text(encoding="utf-8") == serialized:
            return path
    except Exception:
        pass

    tmp = path.with_suffix(".tmp")
    tmp.write_text(serialized, encoding="utf-8")
    tmp.replace(path)
    return path


def load_snapshot(mission_plan_id: int | None) -> Optional[Dict[str, Any]]:
    if mission_plan_id is None:
        return None
    path = _detail_path(int(mission_plan_id))
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def load_snapshot_entry(
    mission_plan_id: int | None,
    input_mission_id: int,
    *,
    allow_latest: bool = True,
) -> Optional[Dict[str, Any]]:
    input_id = _to_int(input_mission_id)
    if input_id is None or input_id <= 0:
        return None

    if mission_plan_id is not None:
        exact_snapshot = load_snapshot(int(mission_plan_id))
        exact_entry = _find_entry(exact_snapshot, int(input_id))
        if isinstance(exact_snapshot, dict) and isinstance(exact_entry, dict):
            return {
                "snapshot": deepcopy(exact_snapshot),
                "entry": deepcopy(exact_entry),
                "snapshotMissionPlanID": _to_int(exact_snapshot.get("missionPlanID")) or int(mission_plan_id),
                "exact": True,
            }

    if not allow_latest:
        return None

    for path in _iter_snapshot_paths():
        try:
            snapshot = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        entry = _find_entry(snapshot, int(input_id))
        if not isinstance(entry, dict):
            continue
        snapshot_plan_id = _to_int(snapshot.get("missionPlanID"))
        if mission_plan_id is not None and snapshot_plan_id == int(mission_plan_id):
            continue
        _audit(
            "snapshot_entry_latest_fallback",
            {
                "requestedMissionPlanID": int(mission_plan_id) if mission_plan_id is not None else None,
                "snapshotMissionPlanID": snapshot_plan_id,
                "inputMissionID": int(input_id),
            },
        )
        return {
            "snapshot": deepcopy(snapshot),
            "entry": deepcopy(entry),
            "snapshotMissionPlanID": snapshot_plan_id,
            "exact": False,
        }
    return None


def carry_forward_snapshot(
    source_plan_id: int | None,
    target_plan_id: int | None,
    *,
    reason: str = "",
) -> Optional[Path]:
    source_id = _to_int(source_plan_id)
    target_id = _to_int(target_plan_id)
    if source_id is None or target_id is None or source_id <= 0 or target_id <= 0:
        return None
    if int(source_id) == int(target_id):
        return _detail_path(int(target_id)) if _detail_path(int(target_id)).exists() else None

    snapshot = load_snapshot(int(source_id))
    if not isinstance(snapshot, dict):
        _audit(
            "snapshot_carry_missing_source",
            {
                "sourceMissionPlanID": int(source_id),
                "targetMissionPlanID": int(target_id),
                "reason": str(reason or ""),
            },
        )
        return None

    carried = deepcopy(snapshot)
    carried["missionPlanID"] = int(target_id)
    carried["carriedFromMissionPlanID"] = int(source_id)
    carried["carryForwardReason"] = str(reason or "")
    carried["carriedAt"] = datetime.now(timezone.utc).isoformat()
    carried["carriedAtEpochMs"] = int(time.time() * 1000)
    path = save_snapshot(int(target_id), carried)
    _audit(
        "snapshot_carried_forward",
        {
            "sourceMissionPlanID": int(source_id),
            "targetMissionPlanID": int(target_id),
            "missionCount": int(carried.get("missionCount") or 0),
            "reason": str(reason or ""),
        },
    )
    return path
