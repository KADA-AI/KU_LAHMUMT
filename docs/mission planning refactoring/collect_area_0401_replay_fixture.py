from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
FLOW_AUDIT_CONTEXT_GROUPS = {
    "current_remaining_snapshot_apply": (
        "mission_planning_gui_current_remaining_snapshot_apply",
        "mission_planning_gui_apply_remaining_snapshot",
    ),
    "reexecute_first_snapshot_apply": (
        "mission_planning_gui_reexecute_first_snapshot_apply",
    ),
    "prior_collaborative_resume": (
        "prior_collaborative_resume_remaining_input",
    ),
    "attack_collaborative_resume": (
        "attack_collaborative_resume_remaining_input",
    ),
    "post_attack_collaborative_resume": (
        "post_attack_collaborative_resume_remaining_input",
        "post_attack_active_only_remaining_input",
    ),
    "post_attack_snapshot_reads": (
        "post_attack_remaining_area_detail",
        "post_attack_remaining_snapshot_geometry_check",
    ),
}
REQUIRED_PROGRESS_KEYS = {
    "progressSource",
    "sourceMissionPlanID",
    "pathID",
    "currentWaypointID",
    "sweepProgressPoints",
    "sweepPointCount",
    "mappedBoundaryLineIndex",
    "confidence",
}
REQUIRED_OWNERSHIP_KEYS = {
    "aircraftID",
    "individualMissionID",
    "inputMissionID",
    "sourceMissionPlanID",
    "pathID",
    "takeoverPolicy",
    "remainingDetail",
}
READINESS_SCHEMA_VERSION = 2
AREA_EPSILON_M2 = 10.0


def _load_json(path: Path) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def _area_missions(payload: dict[str, Any]) -> list[dict[str, Any]]:
    missions: list[dict[str, Any]] = []
    for mission in payload.get("missions") or []:
        if not isinstance(mission, dict):
            continue
        if str(mission.get("missionType") or "").lower() != "area":
            continue
        missions.append(mission)
    return missions


def _detail_has_geometry(detail: Any) -> bool:
    if not isinstance(detail, dict):
        return False
    for key in ("lineList", "areaList", "areaSegmentList"):
        value = detail.get(key)
        if isinstance(value, list) and value:
            return True
    coords = detail.get("coordinateList")
    return isinstance(coords, list) and len(coords) >= 2


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        parsed = float(value)
    except Exception:
        return None
    if parsed != parsed:
        return None
    return parsed


def _mission_completed_without_remaining_geometry(
    mission: dict[str, Any],
    remaining_detail: dict[str, Any],
) -> bool:
    if not bool(mission.get("isDone")):
        return False
    remaining_area = _to_float(mission.get("remainingAreaM2"))
    if remaining_area is not None and remaining_area > AREA_EPSILON_M2:
        return False
    return not _detail_has_geometry(remaining_detail)


def _mission_field_status(mission: dict[str, Any]) -> dict[str, Any]:
    remaining_detail = mission.get("remainingDetail") if isinstance(mission.get("remainingDetail"), dict) else {}
    completed_without_remaining = _mission_completed_without_remaining_geometry(
        mission,
        remaining_detail,
    )
    progress_details = [
        item
        for item in mission.get("areaProgressDetails") or []
        if isinstance(item, dict)
    ]
    ownership_details = [
        item
        for item in mission.get("areaOwnershipDetails") or []
        if isinstance(item, dict)
    ]
    area_segments = [
        item
        for item in remaining_detail.get("areaSegmentList") or []
        if isinstance(item, dict)
    ]
    diagnostics = mission.get("geometryDiagnostics") if isinstance(mission.get("geometryDiagnostics"), dict) else {}

    progress_missing_keys = [
        sorted(key for key in REQUIRED_PROGRESS_KEYS if detail.get(key) is None)
        for detail in progress_details
    ]
    ownership_missing_keys = []
    for detail in ownership_details:
        missing = sorted(key for key in REQUIRED_OWNERSHIP_KEYS if detail.get(key) is None)
        if detail.get("takeoverPolicy") != "piece_only":
            missing.append("takeoverPolicy.piece_only")
        if not _detail_has_geometry(detail.get("remainingDetail")):
            row_remaining_area = _to_float(detail.get("remainingAreaM2"))
            completed_owner = bool(detail.get("isDone")) and (
                row_remaining_area is None or row_remaining_area <= AREA_EPSILON_M2
            )
            if not (completed_without_remaining and completed_owner):
                missing.append("remainingDetail.geometry")
        ownership_missing_keys.append(missing)
    invalid_segment_indexes = [
        index
        for index, row in enumerate(area_segments)
        if row.get("source") != "planned_sweep_row"
        or row.get("lineIndex") is None
        or row.get("aircraftID") is None
        or row.get("individualMissionID") is None
        or row.get("inputMissionID") is None
        or row.get("areaM2") is None
        or len(row.get("coordinateList") or []) < 3
    ]
    missing_categories = []
    if not progress_details:
        missing_categories.append("areaProgressDetails")
    elif any(bool(missing) for missing in progress_missing_keys):
        missing_categories.append("areaProgressDetails.requiredKeys")
    if not ownership_details:
        missing_categories.append("areaOwnershipDetails")
    elif any(bool(missing) for missing in ownership_missing_keys):
        missing_categories.append("areaOwnershipDetails.requiredKeys")
    if not area_segments and not completed_without_remaining:
        missing_categories.append("areaSegmentList")
    elif invalid_segment_indexes:
        missing_categories.append("areaSegmentList.validRows")
    if not diagnostics:
        missing_categories.append("geometryDiagnostics")

    return {
        "areaProgressDetailCount": len(progress_details),
        "areaOwnershipDetailCount": len(ownership_details),
        "areaSegmentCount": len(area_segments),
        "areaEntryCompletedWithoutRemainingGeometry": bool(completed_without_remaining),
        "geometryDiagnosticsPresent": bool(diagnostics),
        "replanInputGeometry": str(diagnostics.get("replanInputGeometry") or "") if diagnostics else "",
        "areaSegmentPolicy": str(remaining_detail.get("areaSegmentPolicy") or ""),
        "operatorDecisionCount": len(diagnostics.get("operatorDecisions") or []) if diagnostics else 0,
        "progressMissingKeys": progress_missing_keys,
        "ownershipMissingKeys": ownership_missing_keys,
        "invalidSegmentIndexes": invalid_segment_indexes,
        "missingCategories": missing_categories,
        "ready": not missing_categories
        and all(not missing for missing in progress_missing_keys)
        and not invalid_segment_indexes,
    }


def _audit_row_ready(row: dict[str, Any]) -> bool:
    try:
        schema_version = int(row.get("areaReadinessSchemaVersion") or 0)
    except Exception:
        schema_version = 0
    if schema_version < int(READINESS_SCHEMA_VERSION):
        return False
    progress_missing = row.get("areaProgressMissingKeys")
    if isinstance(progress_missing, list) and any(bool(item) for item in progress_missing):
        return False
    ownership_missing = row.get("areaOwnershipMissingKeys")
    if isinstance(ownership_missing, list) and any(bool(item) for item in ownership_missing):
        return False
    if row.get("invalidAreaSegmentIndexes"):
        return False
    return bool(row.get("areaSnapshotNewFieldReady")) or bool(row.get("areaEntryNewFieldReady"))


def _flow_coverage(
    ready_context_counts: dict[str, int],
    partial_context_counts: dict[str, int],
) -> dict[str, Any]:
    coverage: dict[str, Any] = {}
    for flow_name, contexts in FLOW_AUDIT_CONTEXT_GROUPS.items():
        ready_contexts = [
            context
            for context in contexts
            if int(ready_context_counts.get(context) or 0) > 0
        ]
        partial_contexts = [
            context
            for context in contexts
            if int(partial_context_counts.get(context) or 0) > 0
        ]
        coverage[flow_name] = {
            "contexts": list(contexts),
            "ready": bool(ready_contexts),
            "readyContexts": ready_contexts,
            "partialContexts": partial_contexts,
        }
    return coverage


def _flow_coverage_line(flow_name: str, detail: dict[str, Any]) -> str:
    ready_contexts = [
        str(context)
        for context in detail.get("readyContexts") or []
        if str(context)
    ]
    partial_contexts = [
        str(context)
        for context in detail.get("partialContexts") or []
        if str(context)
    ]
    status = "ready" if bool(detail.get("ready")) else "missing"
    ready_text = ",".join(ready_contexts) if ready_contexts else "-"
    partial_text = ",".join(partial_contexts) if partial_contexts else "-"
    return f"{flow_name}: {status} ready={ready_text} partial={partial_text}"


def collect_candidates(logs_root: Path) -> dict[str, Any]:
    counts = {
        "files": 0,
        "areaMissions": 0,
        "ready": 0,
        "partial": 0,
        "auditRows": 0,
        "auditReady": 0,
        "auditPartial": 0,
        "carryReady": 0,
        "carryPartial": 0,
        "preservedReady": 0,
        "preservedPartial": 0,
        "rejectedUnready": 0,
    }
    ready: list[dict[str, Any]] = []
    partial: list[dict[str, Any]] = []
    audit_ready: list[dict[str, Any]] = []
    audit_partial: list[dict[str, Any]] = []
    audit_context_counts: dict[str, int] = {}
    audit_ready_context_counts: dict[str, int] = {}
    audit_partial_context_counts: dict[str, int] = {}

    for path in sorted(logs_root.rglob("mission_area_snapshot_*.json")):
        payload = _load_json(path)
        if not isinstance(payload, dict):
            continue
        counts["files"] += 1
        for mission in _area_missions(payload):
            counts["areaMissions"] += 1
            status = _mission_field_status(mission)
            record = {
                "sourcePath": str(path),
                "missionPlanID": payload.get("missionPlanID"),
                "timestamp": payload.get("timestamp"),
                "inputMissionID": mission.get("inputMissionID"),
                "individualMissionIDs": mission.get("individualMissionIDs"),
                "aircraftIDs": mission.get("aircraftIDs"),
                "coveragePercent": mission.get("coveragePercent"),
                "isDone": mission.get("isDone"),
                "fieldStatus": status,
            }
            if bool(status.get("ready")):
                counts["ready"] += 1
                ready.append(record)
            elif (
                status.get("areaProgressDetailCount")
                or status.get("areaOwnershipDetailCount")
                or status.get("areaSegmentCount")
                or status.get("geometryDiagnosticsPresent")
            ):
                counts["partial"] += 1
                partial.append(record)

    for path in sorted(logs_root.rglob("mission_area_snapshot_audit.jsonl")):
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except Exception:
            continue
        for line in lines:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except Exception:
                continue
            if not isinstance(row, dict):
                continue
            event = str(row.get("event") or "")
            if event not in {
                "snapshot_saved",
                "snapshot_entry_exact",
                "snapshot_entry_latest_fallback",
                "snapshot_carried_forward",
                "snapshot_entry_preserved",
                "snapshot_entry_rejected_unready",
            }:
                continue
            is_area_related = bool(row.get("isAreaEntry")) or int(row.get("areaMissionCount") or 0) > 0
            if not is_area_related:
                continue
            counts["auditRows"] += 1
            context = str(row.get("auditContext") or "")
            if context:
                audit_context_counts[context] = int(audit_context_counts.get(context, 0)) + 1
            row_ready = _audit_row_ready(row)
            if event == "snapshot_carried_forward":
                if row_ready:
                    counts["carryReady"] += 1
                else:
                    counts["carryPartial"] += 1
            elif event == "snapshot_entry_preserved":
                if row_ready:
                    counts["preservedReady"] += 1
                else:
                    counts["preservedPartial"] += 1
            elif event == "snapshot_entry_rejected_unready":
                counts["rejectedUnready"] += 1
            if context and row_ready:
                audit_ready_context_counts[context] = int(audit_ready_context_counts.get(context, 0)) + 1
            elif context:
                audit_partial_context_counts[context] = int(audit_partial_context_counts.get(context, 0)) + 1
            record = {
                "sourcePath": str(path),
                "event": event,
                "savedAt": row.get("savedAt"),
                "auditContext": row.get("auditContext"),
                "missionPlanID": row.get("missionPlanID"),
                "requestedMissionPlanID": row.get("requestedMissionPlanID"),
                "snapshotMissionPlanID": row.get("snapshotMissionPlanID"),
                "sourceMissionPlanID": row.get("sourceMissionPlanID"),
                "targetMissionPlanID": row.get("targetMissionPlanID"),
                "inputMissionID": row.get("inputMissionID"),
                "areaSnapshotNewFieldReady": row.get("areaSnapshotNewFieldReady"),
                "areaEntryNewFieldReady": row.get("areaEntryNewFieldReady"),
                "missingNewFieldCategories": row.get("missingNewFieldCategories"),
                "areaEntryMissingNewFieldCategories": row.get("areaEntryMissingNewFieldCategories"),
                "replanInputGeometry": row.get("replanInputGeometry"),
                "areaSegmentPolicy": row.get("areaSegmentPolicy"),
                "areaSegmentCount": row.get("areaSegmentCount"),
                "areaReadinessSchemaVersion": row.get("areaReadinessSchemaVersion"),
                "areaProgressMissingKeys": row.get("areaProgressMissingKeys"),
                "areaOwnershipMissingKeys": row.get("areaOwnershipMissingKeys"),
                "invalidAreaSegmentIndexes": row.get("invalidAreaSegmentIndexes"),
                "rejectReason": row.get("rejectReason"),
            }
            if row_ready:
                counts["auditReady"] += 1
                audit_ready.append(record)
            else:
                counts["auditPartial"] += 1
                audit_partial.append(record)

    flow_coverage = _flow_coverage(audit_ready_context_counts, audit_partial_context_counts)
    missing_flow_groups = [
        flow_name
        for flow_name, detail in flow_coverage.items()
        if not bool(detail.get("ready"))
    ]
    flow_coverage_lines = [
        _flow_coverage_line(flow_name, detail)
        for flow_name, detail in sorted(flow_coverage.items())
    ]

    return {
        "logsRoot": str(logs_root),
        "counts": counts,
        "auditContextCounts": dict(sorted(audit_context_counts.items())),
        "auditReadyContextCounts": dict(sorted(audit_ready_context_counts.items())),
        "auditPartialContextCounts": dict(sorted(audit_partial_context_counts.items())),
        "flowAuditContextGroups": {
            key: list(value)
            for key, value in sorted(FLOW_AUDIT_CONTEXT_GROUPS.items())
        },
        "flowCoverage": flow_coverage,
        "flowCoverageLines": flow_coverage_lines,
        "missingFlowGroups": missing_flow_groups,
        "readyCandidates": ready,
        "partialCandidates": partial,
        "auditReadyCandidates": audit_ready,
        "auditPartialCandidates": audit_partial,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Collect area 0401 replay fixture candidates from mission_area_snapshot logs."
    )
    parser.add_argument(
        "--logs-root",
        default=str(PROJECT_ROOT / "Logs"),
        help="Root directory to scan for mission_area_snapshot_*.json files.",
    )
    parser.add_argument(
        "--write",
        default="",
        help="Optional JSON output path for the collected candidate report.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return a non-zero exit code when no ready candidate is found.",
    )
    parser.add_argument(
        "--require-flow-contexts",
        action="store_true",
        help="In strict mode, also require at least one ready audit row for each replan flow context group.",
    )
    args = parser.parse_args(argv)

    logs_root = Path(str(args.logs_root))
    report = collect_candidates(logs_root)
    counts = report.get("counts") or {}
    print(
        "area 0401 replay candidates: "
        f"files={counts.get('files', 0)} "
        f"areaMissions={counts.get('areaMissions', 0)} "
        f"ready={counts.get('ready', 0)} "
        f"partial={counts.get('partial', 0)} "
        f"auditReady={counts.get('auditReady', 0)} "
        f"auditPartial={counts.get('auditPartial', 0)} "
        f"carryReady={counts.get('carryReady', 0)} "
        f"carryPartial={counts.get('carryPartial', 0)} "
        f"preservedReady={counts.get('preservedReady', 0)} "
        f"preservedPartial={counts.get('preservedPartial', 0)} "
        f"rejectedUnready={counts.get('rejectedUnready', 0)} "
        f"readyFlowGroups={len((report.get('flowAuditContextGroups') or {})) - len(report.get('missingFlowGroups') or [])}"
    )
    if report.get("missingFlowGroups"):
        print(
            "missing ready flow groups: "
            + ", ".join(str(name) for name in report.get("missingFlowGroups") or [])
        )
    if report.get("flowCoverageLines"):
        print("flow coverage:")
        for line in report.get("flowCoverageLines") or []:
            print(f"  {line}")

    if args.write:
        output_path = Path(str(args.write))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        print(f"wrote {output_path}")

    if args.strict and int(counts.get("ready") or 0) <= 0 and int(counts.get("auditReady") or 0) <= 0:
        print("no ready area 0401 replay candidate found", file=sys.stderr)
        return 1
    if args.strict and args.require_flow_contexts and report.get("missingFlowGroups"):
        print(
            "missing ready area 0401 replay flow groups: "
            + ", ".join(str(name) for name in report.get("missingFlowGroups") or []),
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
