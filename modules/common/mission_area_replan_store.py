from __future__ import annotations

import json
import threading
import time
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

from modules.common import db_paths


_STORE_DIR = "mission_area_replan"
_DETAIL_PREFIX = "mission_area_snapshot"
_AUDIT_BASENAME = "mission_area_snapshot_audit.jsonl"
_CENTRAL_LEDGER_BASENAME = "mission_area_central_ledger.json"
_CENTRAL_LEDGER_SCHEMA_VERSION = 1
_CENTRAL_LEDGER_LOCK = threading.RLock()
_AREA_EPSILON_M2 = 10.0
_AREA_GROWTH_TOLERANCE_RATIO = 0.01
_AREA_READINESS_SCHEMA_VERSION = 2
_AREA_PROGRESS_REQUIRED_KEYS = (
    "progressSource",
    "sourceMissionPlanID",
    "pathID",
    "currentWaypointID",
    "sweepProgressPoints",
    "sweepPointCount",
    "mappedBoundaryLineIndex",
    "confidence",
)
_AREA_OWNERSHIP_REQUIRED_KEYS = (
    "aircraftID",
    "individualMissionID",
    "inputMissionID",
    "sourceMissionPlanID",
    "pathID",
    "takeoverPolicy",
    "remainingDetail",
)
_AREA_FIELD_CATEGORY_ORDER = (
    "areaProgressDetails",
    "areaProgressDetails.requiredKeys",
    "areaOwnershipDetails",
    "areaOwnershipDetails.requiredKeys",
    "areaSegmentList",
    "areaSegmentList.validRows",
    "geometryDiagnostics",
)


def _detail_dir() -> Path:
    return db_paths.get_db_subpath("DSS_Internal", _STORE_DIR)


def _detail_path(mission_plan_id: int) -> Path:
    return _detail_dir() / f"{_DETAIL_PREFIX}_{int(mission_plan_id)}.json"


def _audit_path() -> Path:
    return _detail_dir() / _AUDIT_BASENAME


def _central_ledger_path() -> Path:
    return _detail_dir() / _CENTRAL_LEDGER_BASENAME


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
    area_segment_list = detail.get("areaSegmentList")
    if isinstance(area_segment_list, list) and area_segment_list:
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


def _mission_type(mission: Dict[str, Any]) -> str:
    return str(mission.get("missionType") or "").strip().lower()


def _area_progress_missing_keys(progress_rows: Any) -> list[list[str]]:
    if not isinstance(progress_rows, list):
        return []
    missing_by_row: list[list[str]] = []
    for row in progress_rows:
        if not isinstance(row, dict):
            continue
        missing_by_row.append(
            sorted(key for key in _AREA_PROGRESS_REQUIRED_KEYS if row.get(key) is None)
        )
    return missing_by_row


def _invalid_area_segment_indexes(area_segments: Any) -> list[int]:
    if not isinstance(area_segments, list):
        return []
    invalid: list[int] = []
    for index, row in enumerate(area_segments):
        if (
            not isinstance(row, dict)
            or row.get("source") != "planned_sweep_row"
            or row.get("lineIndex") is None
            or row.get("aircraftID") is None
            or row.get("individualMissionID") is None
            or row.get("inputMissionID") is None
            or row.get("areaM2") is None
            or len(row.get("coordinateList") or []) < 3
        ):
            invalid.append(int(index))
    return invalid


def _area_entry_completed_without_remaining_geometry(
    mission: Dict[str, Any],
    remaining_detail: Dict[str, Any],
) -> bool:
    if not _mission_is_done(mission):
        return False
    remaining_area = _mission_remaining_area(mission)
    if remaining_area is not None and remaining_area > _AREA_EPSILON_M2:
        return False
    return not _remaining_detail_has_geometry(remaining_detail)


def _area_ownership_missing_keys(
    ownership_rows: Any,
    *,
    allow_completed_empty_remaining: bool = False,
) -> list[list[str]]:
    if not isinstance(ownership_rows, list):
        return []
    missing_by_row: list[list[str]] = []
    for row in ownership_rows:
        if not isinstance(row, dict):
            continue
        missing = sorted(key for key in _AREA_OWNERSHIP_REQUIRED_KEYS if row.get(key) is None)
        if row.get("takeoverPolicy") != "piece_only":
            missing.append("takeoverPolicy.piece_only")
        if not _remaining_detail_has_geometry(row.get("remainingDetail")):
            row_remaining_area = _to_float(row.get("remainingAreaM2"))
            completed_owner = bool(row.get("isDone")) and (
                row_remaining_area is None or row_remaining_area <= _AREA_EPSILON_M2
            )
            if not (allow_completed_empty_remaining and completed_owner):
                missing.append("remainingDetail.geometry")
        missing_by_row.append(missing)
    return missing_by_row


def _snapshot_entry_area_field_summary(entry: Any) -> Dict[str, Any]:
    mission = entry if isinstance(entry, dict) else {}
    is_area_entry = _mission_type(mission) == "area"
    remaining_detail = mission.get("remainingDetail") if isinstance(mission.get("remainingDetail"), dict) else {}
    progress_details = mission.get("areaProgressDetails")
    ownership_details = mission.get("areaOwnershipDetails")
    area_segments = (
        remaining_detail.get("areaSegmentList")
        if isinstance(remaining_detail.get("areaSegmentList"), list)
        else []
    )
    area_list = remaining_detail.get("areaList") if isinstance(remaining_detail.get("areaList"), list) else []
    coordinate_list = (
        remaining_detail.get("coordinateList")
        if isinstance(remaining_detail.get("coordinateList"), list)
        else []
    )
    diagnostics = mission.get("geometryDiagnostics") if isinstance(mission.get("geometryDiagnostics"), dict) else {}
    completed_without_remaining = bool(
        is_area_entry
        and _area_entry_completed_without_remaining_geometry(
            mission,
            remaining_detail,
        )
    )

    progress_rows = [row for row in (progress_details or []) if isinstance(row, dict)] if isinstance(progress_details, list) else []
    ownership_rows = [row for row in (ownership_details or []) if isinstance(row, dict)] if isinstance(ownership_details, list) else []
    segment_rows = [row for row in (area_segments or []) if isinstance(row, dict)] if isinstance(area_segments, list) else []
    has_clean_area_geometry = bool(area_list or len(coordinate_list) >= 3)
    progress_missing_keys = _area_progress_missing_keys(progress_rows)
    ownership_missing_keys = _area_ownership_missing_keys(
        ownership_rows,
        allow_completed_empty_remaining=bool(completed_without_remaining),
    )
    invalid_segment_indexes = _invalid_area_segment_indexes(segment_rows)

    progress_count = len(progress_rows)
    ownership_count = len(ownership_rows)
    segment_count = len(segment_rows)
    missing: list[str] = []
    if is_area_entry:
        if progress_count <= 0:
            missing.append("areaProgressDetails")
        elif any(bool(missing_keys) for missing_keys in progress_missing_keys):
            missing.append("areaProgressDetails.requiredKeys")
        if ownership_count <= 0:
            missing.append("areaOwnershipDetails")
        elif any(bool(missing_keys) for missing_keys in ownership_missing_keys):
            missing.append("areaOwnershipDetails.requiredKeys")
        if segment_count <= 0 and not completed_without_remaining and not has_clean_area_geometry:
            missing.append("areaSegmentList")
        elif invalid_segment_indexes:
            missing.append("areaSegmentList.validRows")
        if not diagnostics:
            missing.append("geometryDiagnostics")

    return {
        "inputMissionID": _mission_input_id(mission),
        "missionType": _mission_type(mission),
        "isAreaEntry": bool(is_area_entry),
        "areaEntryCompletedWithoutRemainingGeometry": bool(completed_without_remaining),
        "areaReadinessSchemaVersion": int(_AREA_READINESS_SCHEMA_VERSION),
        "areaProgressDetailCount": int(progress_count),
        "areaOwnershipDetailCount": int(ownership_count),
        "areaSegmentCount": int(segment_count),
        "geometryDiagnosticsPresent": bool(diagnostics),
        "replanInputGeometry": str(diagnostics.get("replanInputGeometry") or "") if diagnostics else "",
        "areaSegmentPolicy": str(remaining_detail.get("areaSegmentPolicy") or ""),
        "areaProgressMissingKeys": progress_missing_keys,
        "areaOwnershipMissingKeys": ownership_missing_keys,
        "invalidAreaSegmentIndexes": invalid_segment_indexes,
        "areaEntryMissingNewFieldCategories": list(missing),
        "areaEntryNewFieldReady": bool(is_area_entry and not missing),
    }


def _snapshot_area_field_summary(snapshot: Any) -> Dict[str, Any]:
    summary: Dict[str, Any] = {
        "missionCount": 0,
        "areaMissionCount": 0,
        "areaProgressDetailMissionCount": 0,
        "areaProgressDetailCount": 0,
        "areaOwnershipDetailMissionCount": 0,
        "areaOwnershipDetailCount": 0,
        "areaSegmentMissionCount": 0,
        "areaSegmentCount": 0,
        "geometryDiagnosticsMissionCount": 0,
        "areaNewFieldReadyMissionCount": 0,
        "areaNewFieldIncompleteMissionCount": 0,
        "replanInputGeometryCounts": {},
        "areaSegmentPolicyCounts": {},
        "missingNewFieldCategories": [],
        "areaSnapshotNewFieldReady": False,
        "areaReadinessSchemaVersion": int(_AREA_READINESS_SCHEMA_VERSION),
    }
    if not isinstance(snapshot, dict):
        summary["missingNewFieldCategories"] = [
            "areaProgressDetails",
            "areaOwnershipDetails",
            "areaSegmentList",
            "geometryDiagnostics",
        ]
        return summary

    replan_geometry_counts: Dict[str, int] = {}
    segment_policy_counts: Dict[str, int] = {}
    missing_category_set: set[str] = set()
    for mission in snapshot.get("missions") or []:
        if not isinstance(mission, dict):
            continue
        summary["missionCount"] = int(summary["missionCount"]) + 1
        entry_summary = _snapshot_entry_area_field_summary(mission)
        if not bool(entry_summary.get("isAreaEntry")):
            continue
        summary["areaMissionCount"] = int(summary["areaMissionCount"]) + 1
        if bool(entry_summary.get("areaEntryNewFieldReady")):
            summary["areaNewFieldReadyMissionCount"] = int(summary["areaNewFieldReadyMissionCount"]) + 1
        else:
            summary["areaNewFieldIncompleteMissionCount"] = int(summary["areaNewFieldIncompleteMissionCount"]) + 1
            for category in entry_summary.get("areaEntryMissingNewFieldCategories") or []:
                missing_category_set.add(str(category))

        progress_count = int(entry_summary.get("areaProgressDetailCount") or 0)
        if progress_count > 0:
            summary["areaProgressDetailMissionCount"] = int(summary["areaProgressDetailMissionCount"]) + 1
            summary["areaProgressDetailCount"] = int(summary["areaProgressDetailCount"]) + int(progress_count)

        ownership_count = int(entry_summary.get("areaOwnershipDetailCount") or 0)
        if ownership_count > 0:
            summary["areaOwnershipDetailMissionCount"] = int(summary["areaOwnershipDetailMissionCount"]) + 1
            summary["areaOwnershipDetailCount"] = int(summary["areaOwnershipDetailCount"]) + int(ownership_count)

        segment_count = int(entry_summary.get("areaSegmentCount") or 0)
        if segment_count > 0:
            summary["areaSegmentMissionCount"] = int(summary["areaSegmentMissionCount"]) + 1
            summary["areaSegmentCount"] = int(summary["areaSegmentCount"]) + int(segment_count)
        segment_policy = str(entry_summary.get("areaSegmentPolicy") or "")
        if segment_policy:
            segment_policy_counts[segment_policy] = int(segment_policy_counts.get(segment_policy, 0)) + 1

        if bool(entry_summary.get("geometryDiagnosticsPresent")):
            summary["geometryDiagnosticsMissionCount"] = int(summary["geometryDiagnosticsMissionCount"]) + 1
            replan_input = str(entry_summary.get("replanInputGeometry") or "")
            if replan_input:
                replan_geometry_counts[replan_input] = int(replan_geometry_counts.get(replan_input, 0)) + 1

    missing_categories: list[str] = []
    if int(summary["areaMissionCount"]) > 0:
        missing_categories = [
            category
            for category in _AREA_FIELD_CATEGORY_ORDER
            if category in missing_category_set
        ]
        missing_categories.extend(
            category
            for category in sorted(missing_category_set)
            if category not in missing_categories
        )
    summary["missingNewFieldCategories"] = list(missing_categories)
    summary["areaSnapshotNewFieldReady"] = bool(
        int(summary["areaMissionCount"]) > 0
        and int(summary["areaNewFieldReadyMissionCount"]) == int(summary["areaMissionCount"])
        and not missing_categories
    )
    summary["replanInputGeometryCounts"] = dict(sorted(replan_geometry_counts.items()))
    summary["areaSegmentPolicyCounts"] = dict(sorted(segment_policy_counts.items()))
    return summary


def _growth_tolerance(existing_area_m2: float) -> float:
    return max(float(_AREA_EPSILON_M2), float(existing_area_m2) * float(_AREA_GROWTH_TOLERANCE_RATIO))


def _area_entry_has_replan_geometry_contract(entry: Any, summary: Optional[Dict[str, Any]] = None) -> bool:
    if not isinstance(entry, dict):
        return False
    entry_summary = summary if isinstance(summary, dict) else _snapshot_entry_area_field_summary(entry)
    if not bool(entry_summary.get("isAreaEntry")):
        return False
    if not _remaining_detail_has_geometry(entry.get("remainingDetail")):
        return False
    remaining_area = _mission_remaining_area(entry)
    if remaining_area is not None and remaining_area <= _AREA_EPSILON_M2:
        return False
    return bool(
        int(entry_summary.get("areaOwnershipDetailCount") or 0) > 0
        or bool(entry_summary.get("geometryDiagnosticsPresent"))
    )


def _central_ledger_key(entry: Any) -> Optional[str]:
    if not isinstance(entry, dict):
        return None
    input_id = _mission_input_id(entry)
    if input_id is None:
        return None
    mission_type = _mission_type(entry)
    if mission_type != "area":
        return None
    return f"{mission_type}:{int(input_id)}"


def _load_central_ledger() -> Dict[str, Any]:
    path = _central_ledger_path()
    if not path.exists():
        return {
            "schemaVersion": int(_CENTRAL_LEDGER_SCHEMA_VERSION),
            "entries": {},
        }
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {
            "schemaVersion": int(_CENTRAL_LEDGER_SCHEMA_VERSION),
            "entries": {},
        }
    if not isinstance(data, dict):
        return {
            "schemaVersion": int(_CENTRAL_LEDGER_SCHEMA_VERSION),
            "entries": {},
        }
    entries = data.get("entries")
    if not isinstance(entries, dict):
        data["entries"] = {}
    data["schemaVersion"] = int(data.get("schemaVersion") or _CENTRAL_LEDGER_SCHEMA_VERSION)
    return data


def _write_central_ledger(data: Dict[str, Any]) -> None:
    ledger = dict(data or {})
    ledger["schemaVersion"] = int(_CENTRAL_LEDGER_SCHEMA_VERSION)
    ledger["updatedAt"] = datetime.now(timezone.utc).isoformat()
    path = _central_ledger_path()
    _write_snapshot_file(path, ledger)


def _central_record_mission(record: Any) -> Optional[Dict[str, Any]]:
    if not isinstance(record, dict):
        return None
    mission = record.get("mission")
    if isinstance(mission, dict):
        return mission
    if _mission_input_id(record) is not None:
        return record
    return None


def _entry_done_without_remaining(entry: Dict[str, Any]) -> bool:
    if _mission_is_done(entry):
        return True
    remaining_area = _mission_remaining_area(entry)
    return bool(
        remaining_area is not None
        and remaining_area <= _AREA_EPSILON_M2
        and not _remaining_detail_has_geometry(entry.get("remainingDetail"))
    )


def _central_done_entry_verified(entry: Any) -> bool:
    if not isinstance(entry, dict) or not _entry_done_without_remaining(entry):
        return False
    try:
        return snapshot_entry_ready_for_replan(entry)
    except Exception:
        return False


def _central_should_replace_incoming(central_entry: Any, incoming_entry: Any) -> bool:
    return False


def _central_should_update(central_entry: Any, incoming_entry: Any) -> bool:
    if not isinstance(incoming_entry, dict):
        return False
    if _central_ledger_key(incoming_entry) is None:
        return False
    if not isinstance(central_entry, dict):
        return True
    if _central_ledger_key(central_entry) != _central_ledger_key(incoming_entry):
        return True

    central_done = _entry_done_without_remaining(central_entry)
    incoming_done = _entry_done_without_remaining(incoming_entry)
    if incoming_done:
        return True
    if central_done and not incoming_done:
        return bool(
            not _central_done_entry_verified(central_entry)
            and _area_entry_has_replan_geometry_contract(incoming_entry)
        )

    central_has_geometry = _remaining_detail_has_geometry(central_entry.get("remainingDetail"))
    incoming_has_geometry = _remaining_detail_has_geometry(incoming_entry.get("remainingDetail"))
    if incoming_has_geometry and not central_has_geometry:
        return True
    if not incoming_has_geometry:
        return False

    return True


def _entry_for_snapshot_plan(entry: Dict[str, Any], mission_plan_id: int, *, applied_from_central: bool = False) -> Dict[str, Any]:
    out = deepcopy(entry)
    original_plan_id = _to_int(out.get("missionPlanID"))
    out["missionPlanID"] = int(mission_plan_id)
    if applied_from_central:
        out["centralAreaLedgerApplied"] = True
        if original_plan_id is not None and original_plan_id > 0:
            out["centralAreaSourceMissionPlanID"] = int(original_plan_id)
    return out


def _central_record_from_entry(entry: Dict[str, Any], mission_plan_id: int) -> Dict[str, Any]:
    mission = _entry_for_snapshot_plan(entry, int(mission_plan_id), applied_from_central=False)
    mission.pop("centralAreaLedgerApplied", None)
    mission.pop("centralAreaSourceMissionPlanID", None)
    return {
        "inputMissionID": _mission_input_id(mission),
        "missionType": _mission_type(mission),
        "missionPlanID": int(mission_plan_id),
        "remainingAreaM2": _mission_remaining_area(mission),
        "updatedAt": datetime.now(timezone.utc).isoformat(),
        "mission": mission,
    }


def _merge_snapshot_with_central_area_ledger_unlocked(
    mission_plan_id: int,
    snapshot: Dict[str, Any],
    *,
    update_ledger: bool,
    audit_context: str = "",
    audit: bool = True,
) -> Dict[str, Any]:
    missions = snapshot.get("missions") if isinstance(snapshot, dict) else None
    if not isinstance(missions, list):
        return snapshot

    ledger = _load_central_ledger()
    entries = ledger.setdefault("entries", {})
    if not isinstance(entries, dict):
        entries = {}
        ledger["entries"] = entries

    merged_missions: list[Any] = []
    merged_input_ids: set[int] = set()
    changed = False
    applied_count = 0
    updated_count = 0
    for mission in missions:
        if not isinstance(mission, dict):
            merged_missions.append(mission)
            continue
        key = _central_ledger_key(mission)
        input_id = _mission_input_id(mission)
        if input_id is not None:
            merged_input_ids.add(int(input_id))
        if key is None:
            merged_missions.append(mission)
            continue

        central_entry = _central_record_mission(entries.get(key))
        output_mission = mission
        if _central_should_replace_incoming(central_entry, mission):
            output_mission = _entry_for_snapshot_plan(
                central_entry,
                int(mission_plan_id),
                applied_from_central=True,
            )
            applied_count += 1
            changed = True
            if audit:
                _audit(
                    "central_area_entry_applied",
                    {
                        "missionPlanID": int(mission_plan_id),
                        "inputMissionID": _mission_input_id(mission),
                        "centralRemainingAreaM2": _mission_remaining_area(central_entry),
                        "incomingRemainingAreaM2": _mission_remaining_area(mission),
                        "centralSourceMissionPlanID": _to_int((central_entry or {}).get("missionPlanID")),
                        "auditContext": str(audit_context or ""),
                    },
                )

        if update_ledger and _central_should_update(central_entry, mission):
            entries[str(key)] = _central_record_from_entry(mission, int(mission_plan_id))
            updated_count += 1

        merged_missions.append(output_mission)

    if changed:
        snapshot = dict(snapshot)
        snapshot["missions"] = merged_missions
        snapshot["missionCount"] = len([item for item in merged_missions if isinstance(item, dict)])

    if update_ledger and updated_count > 0:
        _write_central_ledger(ledger)
        if audit:
            _audit(
                "central_area_ledger_updated",
                {
                    "missionPlanID": int(mission_plan_id),
                    "updatedEntryCount": int(updated_count),
                    "appliedEntryCount": int(applied_count),
                    "auditContext": str(audit_context or ""),
                },
            )
    elif applied_count > 0 and audit:
        _audit(
            "central_area_snapshot_merged",
            {
                "missionPlanID": int(mission_plan_id),
                "appliedEntryCount": int(applied_count),
                "auditContext": str(audit_context or ""),
            },
        )
    return snapshot


def _merge_snapshot_with_central_area_ledger(
    mission_plan_id: int,
    snapshot: Dict[str, Any],
    *,
    update_ledger: bool,
    audit_context: str = "",
    audit: bool = True,
) -> Dict[str, Any]:
    with _CENTRAL_LEDGER_LOCK:
        return _merge_snapshot_with_central_area_ledger_unlocked(
            mission_plan_id,
            snapshot,
            update_ledger=update_ledger,
            audit_context=audit_context,
            audit=audit,
        )


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


def audit_snapshot_entry_access(
    entry: Any,
    *,
    requested_mission_plan_id: int | None = None,
    snapshot_mission_plan_id: int | None = None,
    audit_context: Any = "",
    event: str = "snapshot_entry_exact",
) -> None:
    entry_summary = _snapshot_entry_area_field_summary(entry)
    if not bool(entry_summary.get("isAreaEntry")):
        return
    payload = {
        "requestedMissionPlanID": _to_int(requested_mission_plan_id),
        "snapshotMissionPlanID": _to_int(snapshot_mission_plan_id),
        **entry_summary,
    }
    if audit_context:
        payload["auditContext"] = audit_context
    _audit(str(event or "snapshot_entry_exact"), payload)


def snapshot_entry_replan_reject_reason(
    entry: Any,
    *,
    exact: bool | None = None,
    allow_latest_area: bool = False,
) -> str:
    entry_summary = _snapshot_entry_area_field_summary(entry)
    if not bool(entry_summary.get("isAreaEntry")):
        return ""
    if exact is False and not bool(allow_latest_area):
        return "area_snapshot_latest_fallback_not_allowed"
    if not bool(entry_summary.get("areaEntryNewFieldReady")):
        if _area_entry_has_replan_geometry_contract(entry, entry_summary):
            return ""
        return "area_snapshot_not_ready_for_replan"
    return ""


def snapshot_entry_ready_for_replan(
    entry: Any,
    *,
    exact: bool | None = None,
    allow_latest_area: bool = False,
) -> bool:
    return not snapshot_entry_replan_reject_reason(
        entry,
        exact=exact,
        allow_latest_area=bool(allow_latest_area),
    )


def audit_snapshot_entry_rejected(
    entry: Any,
    *,
    requested_mission_plan_id: int | None = None,
    snapshot_mission_plan_id: int | None = None,
    audit_context: Any = "",
    reason: str = "area_snapshot_not_ready_for_replan",
) -> None:
    entry_summary = _snapshot_entry_area_field_summary(entry)
    if not bool(entry_summary.get("isAreaEntry")):
        return
    payload = {
        "requestedMissionPlanID": _to_int(requested_mission_plan_id),
        "snapshotMissionPlanID": _to_int(snapshot_mission_plan_id),
        "rejectReason": str(reason or "area_snapshot_not_ready_for_replan"),
        **entry_summary,
    }
    if audit_context:
        payload["auditContext"] = audit_context
    _audit("snapshot_entry_rejected_unready", payload)


def _find_entry(snapshot: Any, input_mission_id: int) -> Optional[Dict[str, Any]]:
    if not isinstance(snapshot, dict):
        return None
    for mission in snapshot.get("missions") or []:
        if not isinstance(mission, dict):
            continue
        if _mission_input_id(mission) == int(input_mission_id):
            return mission
    return None


def _input_mission_kind(input_mission: Any) -> str:
    if not isinstance(input_mission, dict):
        return ""
    mission_type = _to_int(input_mission.get("inputMissionType"))
    if mission_type == 2:
        return "area"
    if mission_type in {1, 4, 5, 7}:
        return "line"
    detail = input_mission.get("missionDetail") if isinstance(input_mission.get("missionDetail"), dict) else {}
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    if area_list and not line_list:
        return "area"
    if line_list:
        return "line"
    return ""


def _target_plan_input_kinds(mission_plan_id: int) -> Dict[int, str]:
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(mission_plan_id)}.json")
        plan_data = json.loads(plan_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    if not isinstance(plan_data, dict):
        return {}
    input_pkg_id = _to_int(
        plan_data.get("inputMissionPackageID")
        or plan_data.get("inputMissionPackageId")
        or plan_data.get("InputMissionPackageID")
    )
    if input_pkg_id is None or input_pkg_id <= 0:
        return {}
    try:
        input_path = db_paths.get_db_subpath("InputMissionPlan", f"{int(input_pkg_id)}.json")
        input_data = json.loads(input_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    if not isinstance(input_data, dict):
        return {}
    out: Dict[int, str] = {}
    for mission in input_data.get("inputMissionList") or []:
        input_id = _mission_input_id(mission)
        if input_id is None:
            continue
        kind = _input_mission_kind(mission)
        if kind:
            out[int(input_id)] = str(kind)
    return out


def _align_carried_snapshot_entries_to_target_inputs(
    carried: Dict[str, Any],
    *,
    source_plan_id: int,
    target_plan_id: int,
) -> Dict[str, Any]:
    target_input_kinds = _target_plan_input_kinds(int(target_plan_id))
    if not target_input_kinds:
        return carried
    target_input_ids = set(int(input_id) for input_id in target_input_kinds)
    missions = carried.get("missions")
    if not isinstance(missions, list):
        return carried

    source_entries = [mission for mission in missions if isinstance(mission, dict)]

    aligned: list[Dict[str, Any]] = []
    for mission in source_entries:
        input_id = _mission_input_id(mission)
        if input_id is None or int(input_id) not in target_input_ids:
            continue
        aligned.append(mission)

    aligned_input_ids = {
        int(input_id)
        for input_id in (_mission_input_id(mission) for mission in aligned)
        if input_id is not None
    }
    missing_area_ids = [
        int(input_id)
        for input_id, kind in sorted(target_input_kinds.items())
        if str(kind) == "area" and int(input_id) not in aligned_input_ids
    ]
    if len(aligned) == len(source_entries):
        return carried

    updated = dict(carried)
    updated["missions"] = aligned
    updated["missionCount"] = len(aligned)
    _audit(
        "snapshot_carried_entries_aligned_to_target_inputs",
        {
            "sourceMissionPlanID": int(source_plan_id),
            "targetMissionPlanID": int(target_plan_id),
            "targetInputIDs": sorted(int(input_id) for input_id in target_input_ids),
            "missingAreaInputIDs": missing_area_ids,
            "keptMissionCount": len(aligned),
            "droppedMissionCount": max(0, len(source_entries) - len(aligned)),
            "aliasCount": 0,
        },
    )
    return updated


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


def _mission_plan_id_from_snapshot_path(path: Path) -> Optional[int]:
    try:
        return _to_int(str(path.stem).rsplit("_", 1)[-1])
    except Exception:
        return None


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
    incoming_input_ids: set[int] = set()
    changed = False
    for mission in missions:
        if not isinstance(mission, dict):
            merged_missions.append(mission)
            continue
        input_id = _mission_input_id(mission)
        if input_id is not None:
            incoming_input_ids.add(int(input_id))
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
        is_area_snapshot = _mission_type(previous) == "area" or _mission_type(mission) == "area"
        incoming_has_area_remaining = (
            incoming_has_geometry
            and incoming_area is not None
            and incoming_area > _AREA_EPSILON_M2
        )

        keep_previous = False
        reason = ""
        if is_area_snapshot:
            keep_previous = False
        elif previous_done and not incoming_done:
            keep_previous = True
            reason = "previous_done"
        elif incoming_done:
            keep_previous = False
        elif previous_has_geometry and not incoming_has_geometry:
            keep_previous = True
            reason = "incoming_empty_not_done"

        if keep_previous:
            preserved = deepcopy(previous)
            preserved["missionPlanID"] = int(mission_plan_id)
            merged_missions.append(preserved)
            changed = True
            entry_summary = _snapshot_entry_area_field_summary(preserved)
            _audit(
                "snapshot_entry_preserved",
                {
                    "missionPlanID": int(mission_plan_id),
                    "inputMissionID": int(input_id),
                    "reason": reason,
                    "previousRemainingAreaM2": previous_area,
                    "incomingRemainingAreaM2": incoming_area,
                    **entry_summary,
                },
            )
        else:
            merged_missions.append(mission)

    if changed:
        payload = dict(payload)
        payload["missions"] = merged_missions
        payload["missionCount"] = len([item for item in merged_missions if isinstance(item, dict)])
    return payload


def _write_snapshot_file(path: Path, data: Dict[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
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


def save_snapshot(mission_plan_id: int, payload: Dict[str, Any]) -> Path:
    data = dict(payload or {})
    data.setdefault("missionPlanID", int(mission_plan_id))
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    path = _detail_path(mission_plan_id)
    existing = load_snapshot(int(mission_plan_id))
    data = _merge_with_existing_snapshot(int(mission_plan_id), data, existing)
    data = _merge_snapshot_with_central_area_ledger(
        int(mission_plan_id),
        data,
        update_ledger=True,
        audit_context="save_snapshot",
    )
    written_path = _write_snapshot_file(path, data)
    field_summary = _snapshot_area_field_summary(data)
    _audit(
        "snapshot_saved",
        {
            "missionPlanID": int(mission_plan_id),
            "path": str(written_path),
            **field_summary,
        },
    )
    return written_path


def load_snapshot(mission_plan_id: int | None) -> Optional[Dict[str, Any]]:
    if mission_plan_id is None:
        return None
    path = _detail_path(int(mission_plan_id))
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    if isinstance(data, dict):
        return _merge_snapshot_with_central_area_ledger(
            int(mission_plan_id),
            data,
            update_ledger=False,
            audit_context="load_snapshot",
            audit=False,
        )
    return data


def load_snapshot_entry(
    mission_plan_id: int | None,
    input_mission_id: int,
    *,
    allow_latest: bool = True,
    audit_context: Any = "",
) -> Optional[Dict[str, Any]]:
    input_id = _to_int(input_mission_id)
    if input_id is None or input_id <= 0:
        return None

    if mission_plan_id is not None:
        exact_snapshot = load_snapshot(int(mission_plan_id))
        exact_entry = _find_entry(exact_snapshot, int(input_id))
        if isinstance(exact_snapshot, dict) and isinstance(exact_entry, dict):
            snapshot_plan_id = _to_int(exact_snapshot.get("missionPlanID")) or int(mission_plan_id)
            audit_snapshot_entry_access(
                exact_entry,
                requested_mission_plan_id=int(mission_plan_id),
                snapshot_mission_plan_id=snapshot_plan_id,
                audit_context=audit_context,
                event="snapshot_entry_exact",
            )
            return {
                "snapshot": deepcopy(exact_snapshot),
                "entry": deepcopy(exact_entry),
                "snapshotMissionPlanID": snapshot_plan_id,
                "exact": True,
            }

    if not allow_latest:
        return None

    for path in _iter_snapshot_paths():
        try:
            snapshot = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        snapshot_plan_id = _to_int(snapshot.get("missionPlanID")) if isinstance(snapshot, dict) else None
        if isinstance(snapshot, dict):
            snapshot = _merge_snapshot_with_central_area_ledger(
                int(snapshot_plan_id or _mission_plan_id_from_snapshot_path(path) or 0),
                snapshot,
                update_ledger=False,
                audit_context="load_snapshot_entry_latest",
                audit=False,
            )
        entry = _find_entry(snapshot, int(input_id))
        if not isinstance(entry, dict):
            continue
        snapshot_plan_id = _to_int(snapshot.get("missionPlanID"))
        if mission_plan_id is not None and snapshot_plan_id == int(mission_plan_id):
            continue
        audit_snapshot_entry_access(
            entry,
            requested_mission_plan_id=int(mission_plan_id) if mission_plan_id is not None else None,
            snapshot_mission_plan_id=snapshot_plan_id,
            audit_context=audit_context,
            event="snapshot_entry_latest_fallback",
        )
        return {
            "snapshot": deepcopy(snapshot),
            "entry": deepcopy(entry),
            "snapshotMissionPlanID": snapshot_plan_id,
            "exact": False,
        }
    return None


def load_replan_ready_snapshot_entry(
    mission_plan_id: int | None,
    input_mission_id: int,
    *,
    allow_latest: bool = True,
    allow_latest_area: bool = True,
    audit_context: Any = "",
) -> Optional[Dict[str, Any]]:
    primary = load_snapshot_entry(
        mission_plan_id,
        int(input_mission_id),
        allow_latest=False,
        audit_context=audit_context,
    )
    primary_reject_reason = ""
    if isinstance(primary, dict):
        primary_entry = primary.get("entry")
        if isinstance(primary_entry, dict):
            primary_reject_reason = snapshot_entry_replan_reject_reason(
                primary_entry,
                exact=bool(primary.get("exact")) if "exact" in primary else None,
                allow_latest_area=bool(allow_latest_area),
            )
            if not primary_reject_reason:
                return primary
    if not allow_latest:
        return primary

    input_id = _to_int(input_mission_id)
    if input_id is None or input_id <= 0:
        return primary

    for path in _iter_snapshot_paths():
        try:
            snapshot = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        snapshot_plan_id = _to_int(snapshot.get("missionPlanID")) if isinstance(snapshot, dict) else None
        if isinstance(snapshot, dict):
            snapshot = _merge_snapshot_with_central_area_ledger(
                int(snapshot_plan_id or _mission_plan_id_from_snapshot_path(path) or 0),
                snapshot,
                update_ledger=False,
                audit_context="load_replan_ready_snapshot_entry_latest",
                audit=False,
            )
        entry = _find_entry(snapshot, int(input_id))
        if not isinstance(entry, dict):
            continue
        snapshot_plan_id = _to_int(snapshot.get("missionPlanID"))
        if mission_plan_id is not None and snapshot_plan_id == int(mission_plan_id):
            continue
        if (
            mission_plan_id is not None
            and snapshot_plan_id is not None
            and int(snapshot_plan_id) < int(mission_plan_id)
            and _mission_type(entry) == "area"
        ):
            audit_snapshot_entry_rejected(
                entry,
                requested_mission_plan_id=int(mission_plan_id),
                snapshot_mission_plan_id=int(snapshot_plan_id),
                audit_context=audit_context,
                reason="area_snapshot_older_than_requested_plan",
            )
            continue
        reject_reason = snapshot_entry_replan_reject_reason(
            entry,
            exact=False,
            allow_latest_area=bool(allow_latest_area),
        )
        if reject_reason:
            continue
        audit_snapshot_entry_access(
            entry,
            requested_mission_plan_id=int(mission_plan_id) if mission_plan_id is not None else None,
            snapshot_mission_plan_id=snapshot_plan_id,
            audit_context=audit_context,
            event="snapshot_entry_replan_ready_fallback",
        )
        return {
            "snapshot": deepcopy(snapshot),
            "entry": deepcopy(entry),
            "snapshotMissionPlanID": snapshot_plan_id,
            "exact": False,
            "primaryRejectReason": str(primary_reject_reason or ""),
        }
    return primary


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
    carried_missions = carried.get("missions")
    if isinstance(carried_missions, list):
        filtered_missions: list[Any] = []
        for mission in carried_missions:
            if isinstance(mission, dict):
                mission["missionPlanID"] = int(target_id)
                if _mission_type(mission) == "area":
                    continue
            filtered_missions.append(mission)
        carried["missions"] = filtered_missions
        carried["missionCount"] = len([item for item in filtered_missions if isinstance(item, dict)])
    carried = _align_carried_snapshot_entries_to_target_inputs(
        carried,
        source_plan_id=int(source_id),
        target_plan_id=int(target_id),
    )
    carried = _merge_snapshot_with_central_area_ledger(
        int(target_id),
        carried,
        update_ledger=True,
        audit_context=f"carry_forward:{reason or ''}",
    )
    path = _write_snapshot_file(_detail_path(int(target_id)), carried)
    field_summary = _snapshot_area_field_summary(carried)
    _audit(
        "snapshot_carried_forward",
        {
            "sourceMissionPlanID": int(source_id),
            "targetMissionPlanID": int(target_id),
            "missionCount": int(carried.get("missionCount") or 0),
            "reason": str(reason or ""),
            "path": str(path),
            **field_summary,
        },
    )
    return path
