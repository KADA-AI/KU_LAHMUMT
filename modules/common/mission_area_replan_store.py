from __future__ import annotations

import json
import math
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
_SNAPSHOT_FILE_LOCK = threading.RLock()
_AREA_EPSILON_M2 = 10.0
_AREA_GROWTH_TOLERANCE_RATIO = 0.01
_AREA_READINESS_SCHEMA_VERSION = 2
_AREA_COVERAGE_PASS_CONTRACT_VERSION = 1
_AREA_COVERAGE_DEPTH_CONTRACT_VERSION = 1
_AREA_LOGICAL_REGION_CONTRACT_VERSION = 1
# AREA is currently a single-acquisition mission.  Keep this constant at one
# so legacy snapshots cannot silently restore the retired OUT/RETURN workload.
_DEFAULT_REQUIRED_COVERAGE_DEPTH = 1
_AREA_GEOMETRY_DETAIL_KEYS = (
    "coordinateList",
    "lineList",
    "areaList",
    "areaSegmentList",
    "areaSegmentPolicy",
)
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


def area_geometry_detail(detail: Any) -> Optional[Dict[str, Any]]:
    """Return a portable copy of only the geometry carried by an Area detail."""

    if not isinstance(detail, dict):
        return None
    geometry = {
        key: deepcopy(detail[key])
        for key in _AREA_GEOMETRY_DETAIL_KEYS
        if key in detail
    }
    return geometry if _remaining_detail_has_geometry(geometry) else None


def area_assignment_detail(
    source: Any,
    *,
    fallback: Any = None,
) -> Optional[Dict[str, Any]]:
    """Resolve the stable ownership geometry independently of capture workload.

    Spatial depth details intentionally contain many exact fragments.  Those
    fragments are route obligations, not UAV ownership polygons.  A replan must
    therefore prefer the explicit ``areaAssignmentDetail`` and only fall back to
    the mission's original geometry when handling a legacy payload.
    """

    candidates: list[Any] = []
    for value in (source, fallback):
        if not isinstance(value, dict):
            continue
        explicit = value.get("areaAssignmentDetail")
        if isinstance(explicit, dict):
            candidates.append(explicit)
        mission_detail = value.get("missionDetail")
        if isinstance(mission_detail, dict):
            nested_explicit = mission_detail.get("areaAssignmentDetail")
            if isinstance(nested_explicit, dict):
                candidates.append(nested_explicit)
            candidates.append(mission_detail)
        candidates.append(value)
    for candidate in candidates:
        geometry = area_geometry_detail(candidate)
        if geometry is not None:
            return geometry
    return None


def apply_area_assignment_geometry(target: Any, assignment_detail: Any) -> bool:
    """Replace only Area geometry keys and retain every non-geometry contract."""

    if not isinstance(target, dict):
        return False
    geometry = area_geometry_detail(assignment_detail)
    if geometry is None:
        return False
    for key in _AREA_GEOMETRY_DETAIL_KEYS:
        if key in geometry:
            target[key] = deepcopy(geometry[key])
        else:
            target.pop(key, None)
    target["areaAssignmentDetail"] = deepcopy(geometry)
    return True


def attach_area_coverage_workload(
    target: Any,
    source_entry: Any,
    *,
    fallback_assignment: Any = None,
) -> Dict[str, Any]:
    """Attach exact depth work without replacing the stable ownership geometry."""

    if not isinstance(target, dict):
        return {}
    depth_contract = coverage_depth_replan_contract(source_entry)
    if not depth_contract:
        return {}
    assignment = area_assignment_detail(source_entry, fallback=fallback_assignment or target)
    if assignment is not None:
        target["areaAssignmentDetail"] = deepcopy(assignment)
    workload = coverage_depth_pending_remaining_detail(source_entry)
    target["areaCoverageWorkloadDetail"] = (
        deepcopy(workload)
        if isinstance(workload, dict)
        else {"coordinateList": [], "lineList": [], "areaList": []}
    )
    return {
        "assignment": deepcopy(assignment) if isinstance(assignment, dict) else None,
        "workload": deepcopy(workload) if isinstance(workload, dict) else None,
        "depth": depth_contract,
    }


def _normalize_coverage_pass(value: Any) -> Optional[str]:
    text = str(value or "").strip().lower()
    if text in {"forward", "reverse"}:
        return text
    return None


_AREA_MULTI_CAPTURE_CONTRACT_KEYS = (
    "areaCoveragePassContractVersion",
    "coveragePassPolicy",
    "coveragePassOrder",
    "coveragePassDetails",
    "coveragePassObligations",
    "remainingCoveragePasses",
    "completedCoveragePasses",
    "currentCoveragePass",
    "activeCoveragePass",
    "areaCoveragePhase",
    "coveragePassAttributionDetails",
    "coveragePassCount",
    "coveragePassRequirementMode",
    "areaPassAssignmentMode",
    "areaAssignedCoveragePass",
    "coveragePass",
    "areaCoverageDepthContractVersion",
    "coverageDepthPolicy",
    "coverageDepthDetails",
    "coverageDepthObligations",
    "remainingCoverageDepth",
    "completedCoverageDepth",
    "coverageDepthSatisfied",
    "coverageDepthUnresolvedGeometryCount",
    "coverageDepthAreaM2",
    "coverageObservationDetails",
    "activeCoverageAcquisitionIDs",
    "areaLogicalRegionContractVersion",
    "areaLogicalRegionPolicy",
    "areaLogicalRegionDetails",
    "areaLogicalRegionCount",
    "remainingAreaLogicalRegionCount",
)


def strip_area_multi_capture_contracts(target: Any) -> bool:
    """Remove the retired reciprocal/depth contract from an AREA payload."""

    if not isinstance(target, dict):
        return False
    changed = False
    for key in _AREA_MULTI_CAPTURE_CONTRACT_KEYS:
        if key in target:
            target.pop(key, None)
            changed = True
    target["requiredCoverageDepth"] = 1
    target["areaCapturePolicy"] = "single_capture"
    return changed


def _legacy_single_capture_pending_state(
    entry: Any,
) -> tuple[bool, Optional[Dict[str, Any]], bool]:
    """Translate an old two-pass/depth snapshot to one-capture remaining work.

    Returns ``(legacy_contract_present, remaining_detail, completed)``.  Only
    never-captured geometry is retained: the old forward pass is the single
    capture, and a spatial depth of one already satisfies the new policy.
    """

    source = entry if isinstance(entry, dict) else {}
    pass_rows_raw = source.get("coveragePassDetails")
    if not isinstance(pass_rows_raw, list):
        pass_rows_raw = source.get("coverage_pass_details")
    pass_obligations_raw = source.get("coveragePassObligations")
    if not isinstance(pass_obligations_raw, list):
        pass_obligations_raw = source.get("coverage_pass_obligations")
    depth_rows_raw = source.get("coverageDepthDetails")
    if not isinstance(depth_rows_raw, list):
        depth_rows_raw = source.get("coverage_depth_details")
    depth_obligations_raw = source.get("coverageDepthObligations")
    if not isinstance(depth_obligations_raw, list):
        depth_obligations_raw = source.get("coverage_depth_obligations")

    pass_explicit = bool(
        source.get("areaCoveragePassContractVersion") is not None
        or isinstance(pass_rows_raw, list)
        or isinstance(pass_obligations_raw, list)
        or isinstance(source.get("coveragePassOrder"), list)
        or isinstance(source.get("coverage_pass_order"), list)
    )
    depth_explicit = bool(
        source.get("areaCoverageDepthContractVersion") is not None
        or isinstance(depth_rows_raw, list)
        or isinstance(depth_obligations_raw, list)
    )
    explicit = bool(pass_explicit or depth_explicit)
    if not explicit:
        detail = source.get("remainingDetail")
        return False, deepcopy(detail) if isinstance(detail, dict) else None, bool(
            source.get("isDone")
        )
    if bool(source.get("isDone")):
        return True, None, True

    rows_by_pass: Dict[str, Dict[str, Any]] = {}
    for raw_row in list(pass_rows_raw or []) + list(pass_obligations_raw or []):
        if not isinstance(raw_row, dict):
            continue
        pass_name = _normalize_coverage_pass(
            raw_row.get("coveragePass", raw_row.get("coverage_pass"))
        )
        if pass_name is None:
            continue
        row = deepcopy(rows_by_pass.get(pass_name) or {})
        row.update(deepcopy(raw_row))
        rows_by_pass[pass_name] = row

    forward = rows_by_pass.get("forward")
    if isinstance(forward, dict):
        remaining_detail = forward.get("remainingDetail")
        if _remaining_detail_has_geometry(remaining_detail):
            return True, deepcopy(remaining_detail), False
        remaining_area = _to_float(forward.get("remainingAreaM2"))
        if bool(forward.get("isDone")) or (
            remaining_area is not None and remaining_area <= _AREA_EPSILON_M2
        ):
            return True, None, True

    # Depth zero means the terrain has never been captured.  Depth one was the
    # former "one more return pass" band and is complete under single capture.
    depth_rows: list[Dict[str, Any]] = []
    seen_depth_rows: set[str] = set()
    for raw_row in list(depth_rows_raw or []) + list(depth_obligations_raw or []):
        if not isinstance(raw_row, dict):
            continue
        try:
            token = json.dumps(
                raw_row.get("remainingDetail") or raw_row,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
        except Exception:
            token = repr(raw_row)
        if token in seen_depth_rows:
            continue
        seen_depth_rows.add(token)
        depth_rows.append(raw_row)
    never_captured = [
        row.get("remainingDetail")
        for row in depth_rows
        if int(_to_int(row.get("coverageDepth", row.get("coverage_depth"))) or 0) < 1
        and not bool(row.get("isDone"))
        and _remaining_detail_has_geometry(row.get("remainingDetail"))
    ]
    merged_never_captured = _merge_remaining_detail_rows(never_captured)
    if isinstance(merged_never_captured, dict):
        return True, merged_never_captured, False
    if depth_rows:
        return True, None, True

    phase = str(
        source.get("areaCoveragePhase")
        or source.get("area_coverage_phase")
        or ""
    ).strip().lower()
    if phase in {"turn", "return", "completed"} or (
        pass_explicit and "forward" not in rows_by_pass and "reverse" in rows_by_pass
    ):
        return True, None, True

    fallback = source.get("remainingDetail")
    if _remaining_detail_has_geometry(fallback):
        return True, deepcopy(fallback), False
    return True, None, False


def _legacy_single_capture_progress(entry: Any) -> Dict[str, Any]:
    """Extract single-capture progress from the legacy forward-pass summary."""

    source = entry if isinstance(entry, dict) else {}
    rows = source.get("coveragePassDetails")
    if not isinstance(rows, list):
        rows = source.get("coverage_pass_details")
    forward = next(
        (
            row
            for row in (rows or [])
            if isinstance(row, dict)
            and _normalize_coverage_pass(
                row.get("coveragePass", row.get("coverage_pass"))
            )
            == "forward"
        ),
        None,
    )
    if not isinstance(forward, dict):
        return {}
    planned = _to_float(forward.get("plannedAreaM2"))
    covered = _to_float(forward.get("coveredAreaM2"))
    remaining = _to_float(forward.get("remainingAreaM2"))
    percent = _to_int(forward.get("coveragePercent"))
    result: Dict[str, Any] = {}
    if planned is not None:
        result["coverageWorkPlannedM2"] = float(max(0.0, planned))
    if covered is not None:
        result["coveredAreaM2"] = float(max(0.0, covered))
        result["coverageWorkCoveredM2"] = float(max(0.0, covered))
    if remaining is not None:
        result["remainingAreaM2"] = float(max(0.0, remaining))
        result["coverageWorkRemainingM2"] = float(max(0.0, remaining))
    if percent is None and planned is not None and covered is not None and planned > 1e-9:
        percent = int(round((covered / planned) * 100.0))
    if percent is not None:
        result["coveragePercent"] = max(0, min(100, int(percent)))
        result["spatialCoveragePercent"] = max(0, min(100, int(percent)))
    return result


def _merge_remaining_detail_rows(details: Iterable[Any]) -> Optional[Dict[str, Any]]:
    """Concatenate portable remaining-geometry fragments without losing lineage.

    The Area route builder performs its exact union/clip later.  Keeping the
    fragments separate here preserves capture-depth lineage during
    attack/takeover; these rows are workload only and must never be reused as
    UAV ownership polygons.
    """

    rows = [deepcopy(row) for row in details if _remaining_detail_has_geometry(row)]
    if not rows:
        return None
    if len(rows) == 1:
        return rows[0]

    merged: Dict[str, Any] = {
        "coordinateList": [],
        "lineList": [],
        "areaList": [],
    }
    segments: list[Dict[str, Any]] = []
    for detail in rows:
        for row in detail.get("areaList") or []:
            if isinstance(row, dict):
                merged["areaList"].append(deepcopy(row))
        for row in detail.get("lineList") or []:
            if isinstance(row, dict):
                merged["lineList"].append(deepcopy(row))
        for row in detail.get("areaSegmentList") or []:
            if isinstance(row, dict):
                segments.append(deepcopy(row))
        if not detail.get("areaList") and not detail.get("areaSegmentList"):
            coordinate_list = detail.get("coordinateList")
            if isinstance(coordinate_list, list) and len(coordinate_list) >= 3:
                merged["areaList"].append(
                    {"isHole": False, "coordinateList": deepcopy(coordinate_list)}
                )
    if segments:
        merged["areaSegmentList"] = segments
        merged["areaSegmentPolicy"] = "planned_sweep_row_remaining"
    return merged if _remaining_detail_has_geometry(merged) else None


def _empty_area_geometry_detail() -> Dict[str, Any]:
    return {"coordinateList": [], "lineList": [], "areaList": []}


def _dedupe_geometry_rows(rows: Any) -> list[Dict[str, Any]]:
    """Deduplicate portable geometry rows without interpreting their shape."""

    result: list[Dict[str, Any]] = []
    seen: set[str] = set()
    for raw_row in rows or []:
        if not isinstance(raw_row, dict):
            continue
        try:
            key = json.dumps(
                raw_row,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
        except Exception:
            key = repr(raw_row)
        if key in seen:
            continue
        seen.add(key)
        result.append(deepcopy(raw_row))
    return result


def _merge_logical_geometry_details(details: Iterable[Any]) -> Optional[Dict[str, Any]]:
    """Merge geometry inside one logical region while keeping one contract row.

    A concave assignment or a clipped workload may still contain multiple
    polygon components.  Those components are geometry of *one* UAV/pass
    region; they must not become independent ownership/route-region records.
    """

    merged = _merge_remaining_detail_rows(details)
    if not isinstance(merged, dict):
        return None
    for key in ("areaList", "lineList", "areaSegmentList"):
        if key in merged:
            merged[key] = _dedupe_geometry_rows(merged.get(key))
    area_rows = list(merged.get("areaList") or [])
    outer_rows = [row for row in area_rows if not bool(row.get("isHole"))]
    if len(outer_rows) == 1:
        merged["coordinateList"] = deepcopy(outer_rows[0].get("coordinateList") or [])
    elif area_rows:
        merged["coordinateList"] = []
    return merged if _remaining_detail_has_geometry(merged) else None


def area_logical_region_replan_contract(entry: Any) -> Dict[str, Any]:
    """Return one stable single-capture Area region per UAV.

    Legacy OUT/RETURN owners are collapsed to their forward (never-captured)
    remainder.  Exact geometry components stay nested in that one UAV region.
    """

    source = entry if isinstance(entry, dict) else {}
    input_id = _mission_input_id(source)
    raw_regions = source.get("areaLogicalRegionDetails")
    raw_owners = [
        owner
        for owner in (source.get("areaOwnershipDetails") or [])
        if isinstance(owner, dict)
    ]
    region_sources: list[Dict[str, Any]] = list(raw_owners)
    if not region_sources and isinstance(raw_regions, list):
        region_sources = [row for row in raw_regions if isinstance(row, dict)]

    grouped_single: Dict[int, Dict[str, Any]] = {}
    for raw_region in region_sources:
        aircraft_id = _to_int(raw_region.get("aircraftID"))
        if aircraft_id is None:
            continue
        pass_name = _normalize_coverage_pass(
            raw_region.get("coveragePass")
            or raw_region.get("areaAssignedCoveragePass")
        )
        # A legacy RETURN-only row is already complete under one capture.
        if pass_name == "reverse":
            continue
        legacy_present, single_remaining, single_completed = (
            _legacy_single_capture_pending_state(raw_region)
        )
        if not legacy_present:
            single_remaining = area_geometry_detail(raw_region.get("remainingDetail"))
            single_completed = bool(raw_region.get("isDone"))
        assignment = area_assignment_detail(raw_region, fallback=source)
        bucket = grouped_single.setdefault(
            int(aircraft_id),
            {
                "rows": [],
                "assignmentDetails": [],
                "remainingDetails": [],
                "missionIDs": set(),
            },
        )
        bucket["rows"].append(
            {**deepcopy(raw_region), "isDone": bool(single_completed)}
        )
        if assignment is not None:
            bucket["assignmentDetails"].append(assignment)
        if isinstance(single_remaining, dict) and _remaining_detail_has_geometry(
            single_remaining
        ):
            bucket["remainingDetails"].append(single_remaining)
        for value in list(raw_region.get("individualMissionIDs") or []) + [
            raw_region.get("individualMissionID")
        ]:
            mission_id = _to_int(value)
            if mission_id is not None:
                bucket["missionIDs"].add(int(mission_id))

    logical_rows: list[Dict[str, Any]] = []
    for aircraft_id, bucket in sorted(grouped_single.items()):
        assignment = _merge_logical_geometry_details(
            bucket.get("assignmentDetails") or []
        )
        remaining = _merge_logical_geometry_details(
            bucket.get("remainingDetails") or []
        )
        rows = list(bucket.get("rows") or [])
        is_done = bool(rows) and all(bool(row.get("isDone")) for row in rows)
        if remaining is not None:
            is_done = False
        mission_ids = sorted(int(value) for value in bucket.get("missionIDs") or set())
        logical_id = (
            f"area:{int(input_id)}:uav:{int(aircraft_id)}:single"
            if input_id is not None
            else f"area:uav:{int(aircraft_id)}:single"
        )
        logical_rows.append(
            {
                "logicalRegionID": logical_id,
                "logicalRegionRole": "SINGLE_CAPTURE",
                "aircraftID": int(aircraft_id),
                "individualMissionIDs": mission_ids,
                "isDone": bool(is_done),
                "geometryRole": "logical_aircraft_single_capture_assignment",
                "areaAssignmentDetail": deepcopy(
                    assignment or remaining or _empty_area_geometry_detail()
                ),
                "remainingDetail": deepcopy(
                    _empty_area_geometry_detail()
                    if is_done
                    else remaining or assignment or _empty_area_geometry_detail()
                ),
                "areaCoverageWorkloadDetail": deepcopy(
                    _empty_area_geometry_detail()
                    if is_done
                    else remaining or _empty_area_geometry_detail()
                ),
            }
        )

    if not logical_rows:
        return {}
    pending_count = len([row for row in logical_rows if not bool(row.get("isDone"))])
    return {
        "areaLogicalRegionContractVersion": int(_AREA_LOGICAL_REGION_CONTRACT_VERSION),
        "areaLogicalRegionPolicy": "one_single_capture_region_per_aircraft",
        "areaLogicalRegionDetails": logical_rows,
        "areaLogicalRegionCount": len(logical_rows),
        "remainingAreaLogicalRegionCount": int(pending_count),
    }

    source_rows: list[Dict[str, Any]] = []
    if raw_owners:
        # Ownership rows are rebuilt by monitoring from the current plan and
        # current progress.  Logical rows, on the other hand, are portable
        # carry-forward metadata and can describe an older plan generation.
        # Always rebuild from fresh owners when they are available so a region
        # completed after a prior/attack/UAV-loss replan cannot be resurrected
        # by a stale logical row copied from the source input mission.
        for owner in raw_owners:
            if not isinstance(owner, dict):
                continue
            pass_rows = owner.get("coveragePassDetails")
            if not isinstance(pass_rows, list) or not pass_rows:
                owner_pass = _normalize_coverage_pass(
                    owner.get("coveragePass")
                    or owner.get("areaAssignedCoveragePass")
                )
                pass_rows = (
                    [{"coveragePass": owner_pass, "isDone": owner.get("isDone")}]
                    if owner_pass is not None
                    else []
                )
            for pass_row in pass_rows:
                if not isinstance(pass_row, dict):
                    continue
                pass_name = _normalize_coverage_pass(pass_row.get("coveragePass"))
                aircraft_id = _to_int(owner.get("aircraftID"))
                if pass_name is None or aircraft_id is None:
                    continue
                assignment = area_assignment_detail(owner, fallback=source)
                workload = area_geometry_detail(
                    pass_row.get("areaCoverageWorkloadDetail")
                    or pass_row.get("remainingDetail")
                    or owner.get("areaCoverageWorkloadDetail")
                    or owner.get("remainingDetail")
                )
                source_rows.append(
                    {
                        "aircraftID": int(aircraft_id),
                        "coveragePass": str(pass_name),
                        "passIndex": 1 if pass_name == "forward" else 2,
                        "individualMissionID": _to_int(owner.get("individualMissionID")),
                        "individualMissionIDs": [
                            int(value)
                            for value in [_to_int(owner.get("individualMissionID"))]
                            if value is not None
                        ],
                        "plannedAreaM2": _to_float(
                            pass_row.get("plannedAreaM2", owner.get("plannedAreaM2"))
                        ),
                        "remainingAreaM2": _to_float(
                            pass_row.get("remainingAreaM2", owner.get("remainingAreaM2"))
                        ),
                        "isDone": bool(pass_row.get("isDone")),
                        "coverageDepthContractPresent": bool(
                            owner.get("areaCoverageDepthContractVersion") is not None
                            or isinstance(owner.get("coverageDepthDetails"), list)
                        ),
                        "coverageDepthSatisfied": bool(
                            owner.get("coverageDepthSatisfied")
                        ),
                        "areaAssignmentDetail": deepcopy(assignment),
                        "areaCoverageWorkloadDetail": deepcopy(workload),
                    }
                )
    elif isinstance(raw_regions, list) and raw_regions:
        source_rows = [deepcopy(row) for row in raw_regions if isinstance(row, dict)]

    grouped: Dict[tuple[int, str], Dict[str, Any]] = {}
    for row in source_rows:
        aircraft_id = _to_int(row.get("aircraftID"))
        pass_name = _normalize_coverage_pass(row.get("coveragePass"))
        if aircraft_id is None or pass_name is None:
            continue
        key = (int(aircraft_id), str(pass_name))
        bucket = grouped.setdefault(
            key,
            {
                "rows": [],
                "assignmentDetails": [],
                "workloadDetails": [],
                "missionIDs": set(),
            },
        )
        bucket["rows"].append(row)
        assignment = area_assignment_detail(row, fallback=source)
        if assignment is not None:
            bucket["assignmentDetails"].append(assignment)
        workload = area_geometry_detail(
            row.get("areaCoverageWorkloadDetail") or row.get("remainingDetail")
        )
        if workload is not None:
            bucket["workloadDetails"].append(workload)
        for value in list(row.get("individualMissionIDs") or []) + [
            row.get("individualMissionID")
        ]:
            mission_id = _to_int(value)
            if mission_id is not None:
                bucket["missionIDs"].add(int(mission_id))

    logical_rows: list[Dict[str, Any]] = []
    for (aircraft_id, pass_name), bucket in sorted(
        grouped.items(),
        key=lambda item: (int(item[0][0]), 1 if item[0][1] == "forward" else 2),
    ):
        rows = list(bucket.get("rows") or [])
        assignment = _merge_logical_geometry_details(
            bucket.get("assignmentDetails") or []
        )
        workload = _merge_logical_geometry_details(
            bucket.get("workloadDetails") or []
        )
        if assignment is None:
            # Legacy snapshots may not have explicit ownership geometry.  Keep
            # them usable, but still expose one logical row rather than one row
            # per footprint component.
            assignment = deepcopy(workload) if isinstance(workload, dict) else None
        is_done = bool(rows) and all(bool(row.get("isDone")) for row in rows)
        if any(
            bool(row.get("coverageDepthContractPresent"))
            and not bool(row.get("coverageDepthSatisfied"))
            for row in rows
        ) and _remaining_detail_has_geometry(workload):
            is_done = False
        remaining_values = [
            value
            for value in (_to_float(row.get("remainingAreaM2")) for row in rows)
            if value is not None
        ]
        planned_values = [
            value
            for value in (_to_float(row.get("plannedAreaM2")) for row in rows)
            if value is not None
        ]
        mission_ids = sorted(int(value) for value in bucket.get("missionIDs") or set())
        logical_id = (
            f"area:{int(input_id)}:uav:{int(aircraft_id)}:{pass_name}"
            if input_id is not None
            else f"area:uav:{int(aircraft_id)}:{pass_name}"
        )
        logical_rows.append(
            {
                "logicalRegionID": logical_id,
                "logicalRegionRole": "OUT" if pass_name == "forward" else "RETURN",
                "aircraftID": int(aircraft_id),
                "coveragePass": str(pass_name),
                "passIndex": 1 if pass_name == "forward" else 2,
                "individualMissionIDs": mission_ids,
                "plannedAreaM2": float(sum(planned_values)) if planned_values else None,
                "remainingAreaM2": float(sum(remaining_values)) if remaining_values else None,
                "isDone": bool(is_done),
                "geometryRole": "logical_aircraft_pass_assignment",
                "areaAssignmentDetail": (
                    deepcopy(assignment)
                    if isinstance(assignment, dict)
                    else _empty_area_geometry_detail()
                ),
                "remainingDetail": (
                    _empty_area_geometry_detail()
                    if is_done
                    else deepcopy(assignment or workload or _empty_area_geometry_detail())
                ),
                "areaCoverageWorkloadDetail": (
                    _empty_area_geometry_detail()
                    if is_done
                    else deepcopy(workload)
                    if isinstance(workload, dict)
                    else _empty_area_geometry_detail()
                ),
            }
        )

    if not logical_rows:
        return {}
    pending_count = len([row for row in logical_rows if not bool(row.get("isDone"))])
    return {
        "areaLogicalRegionContractVersion": int(_AREA_LOGICAL_REGION_CONTRACT_VERSION),
        "areaLogicalRegionPolicy": "one_out_and_one_return_per_aircraft",
        "areaLogicalRegionDetails": logical_rows,
        "areaLogicalRegionCount": len(logical_rows),
        "remainingAreaLogicalRegionCount": int(pending_count),
    }


def apply_area_logical_region_replan_contract(
    target: Any,
    source_entry: Any,
) -> Dict[str, Any]:
    contract = area_logical_region_replan_contract(source_entry)
    if not isinstance(target, dict) or not contract:
        return contract
    for key, value in contract.items():
        target[key] = deepcopy(value)
    return contract


def coverage_depth_replan_contract(entry: Any) -> Dict[str, Any]:
    """Return no depth contract while AREA uses one capture.

    Old snapshots are translated by ``coverage_replan_pending_remaining_detail``;
    exposing their depth ledger again would recreate the retired return pass.
    """

    return {}

    source = entry if isinstance(entry, dict) else {}
    raw_details = source.get("coverageDepthDetails")
    raw_obligations = source.get("coverageDepthObligations")
    explicit = bool(
        source.get("areaCoverageDepthContractVersion") is not None
        or isinstance(raw_details, list)
        or isinstance(raw_obligations, list)
    )
    if not explicit:
        return {}

    required_depth = _to_int(source.get("requiredCoverageDepth"))
    required_depth = max(1, int(required_depth or _DEFAULT_REQUIRED_COVERAGE_DEPTH))
    def _depth_row_identity(raw_row: Dict[str, Any], fallback_index: int) -> str:
        for key in ("depthBandID", "fragmentID", "coverageFragmentID", "segmentID"):
            value = str(raw_row.get(key) or "").strip()
            if value:
                return f"id:{value}"
        try:
            geometry_token = json.dumps(
                raw_row.get("remainingDetail") or {},
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
        except Exception:
            geometry_token = f"index:{int(fallback_index)}"
        return f"geometry:{geometry_token}"

    rows_by_identity: Dict[str, Dict[str, Any]] = {}
    row_order: list[str] = []
    for source_index, raw_row in enumerate(raw_details or []):
        if not isinstance(raw_row, dict):
            continue
        identity = _depth_row_identity(raw_row, source_index)
        if identity not in rows_by_identity:
            row_order.append(identity)
        rows_by_identity[identity] = deepcopy(raw_row)
    for source_index, raw_row in enumerate(raw_obligations or []):
        if not isinstance(raw_row, dict):
            continue
        identity = _depth_row_identity(raw_row, source_index)
        if identity not in rows_by_identity:
            row_order.append(identity)
            rows_by_identity[identity] = {}
        rows_by_identity[identity].update(deepcopy(raw_row))
    source_rows = [rows_by_identity[key] for key in row_order]
    normalized_rows: list[Dict[str, Any]] = []
    active_rows: list[Dict[str, Any]] = []
    unresolved_rows = 0
    for row_index, raw_row in enumerate(source_rows or []):
        if not isinstance(raw_row, dict):
            continue
        row = deepcopy(raw_row)
        completed_depth = _to_int(
            row.get("coverageDepth", row.get("capturesCompleted"))
        )
        remaining_count = _to_int(row.get("remainingCaptureCount"))
        if completed_depth is None and remaining_count is not None:
            completed_depth = int(required_depth) - int(remaining_count)
        completed_depth = max(0, min(int(required_depth), int(completed_depth or 0)))
        if remaining_count is None:
            remaining_count = int(required_depth) - int(completed_depth)
        remaining_count = max(0, min(int(required_depth), int(remaining_count)))
        is_done = bool(row.get("isDone")) or remaining_count <= 0
        if is_done:
            completed_depth = int(required_depth)
            remaining_count = 0

        row["coverageDepth"] = int(completed_depth)
        row["remainingCaptureCount"] = int(remaining_count)
        row["requiredCoverageDepth"] = int(required_depth)
        row["isDone"] = bool(is_done)
        row.setdefault("depthBandIndex", int(row_index))
        normalized_rows.append(row)
        if is_done:
            continue
        if _remaining_detail_has_geometry(row.get("remainingDetail")):
            active_rows.append(deepcopy(row))
        else:
            # Never silently turn a geometry-loss event into mission complete.
            unresolved_rows += 1

    empty_contract_unresolved = bool(
        not normalized_rows
        and not bool(source.get("coverageDepthSatisfied"))
        and not bool(source.get("isDone"))
    )
    if empty_contract_unresolved:
        unresolved_rows += 1

    remaining_depth_value = _to_int(source.get("remainingCoverageDepth"))
    if remaining_depth_value is None:
        remaining_depth_value = max(
            [int(row.get("remainingCaptureCount") or 0) for row in normalized_rows]
            or [0]
        )
    if empty_contract_unresolved:
        remaining_depth_value = max(1, int(remaining_depth_value or required_depth))
    completed_depth_value = _to_int(source.get("completedCoverageDepth"))
    if completed_depth_value is None:
        completed_depth_value = min(
            [int(row.get("coverageDepth") or 0) for row in normalized_rows]
            or [int(required_depth) if not active_rows and not unresolved_rows else 0]
        )
    explicit_satisfied = source.get("coverageDepthSatisfied")
    satisfied = bool(explicit_satisfied) if explicit_satisfied is not None else bool(
        int(remaining_depth_value) <= 0 and not active_rows and unresolved_rows <= 0
    )
    if active_rows or unresolved_rows:
        satisfied = False

    contract = {
        "areaCoverageDepthContractVersion": int(_AREA_COVERAGE_DEPTH_CONTRACT_VERSION),
        "coverageDepthPolicy": "spatial_capture_depth",
        "requiredCoverageDepth": int(required_depth),
        "coverageDepthDetails": normalized_rows,
        "coverageDepthObligations": active_rows,
        "remainingCoverageDepth": max(0, int(remaining_depth_value)),
        "completedCoverageDepth": max(0, min(int(required_depth), int(completed_depth_value))),
        "coverageDepthSatisfied": bool(satisfied),
        "coverageDepthUnresolvedGeometryCount": int(unresolved_rows),
        "coverageObservationDetails": [
            deepcopy(row)
            for row in (source.get("coverageObservationDetails") or [])
            if isinstance(row, dict)
        ],
        "activeCoverageAcquisitionIDs": {
            str(key): str(value)
            for key, value in (source.get("activeCoverageAcquisitionIDs") or {}).items()
            if str(value or "").strip()
        }
        if isinstance(source.get("activeCoverageAcquisitionIDs"), dict)
        else {},
    }
    acquisition_namespace = str(source.get("coverageAcquisitionNamespace") or "").strip()
    if acquisition_namespace:
        contract["coverageAcquisitionNamespace"] = acquisition_namespace
    return contract


def apply_coverage_depth_replan_contract(
    target: Any,
    source_entry: Any,
) -> Dict[str, Any]:
    """Copy the portable spatial-depth ledger to a mission/detail/path row."""

    contract = coverage_depth_replan_contract(source_entry)
    if not isinstance(target, dict) or not contract:
        return contract
    for key, value in contract.items():
        target[key] = deepcopy(value)
    return contract


def coverage_depth_pending_remaining_detail(entry: Any) -> Optional[Dict[str, Any]]:
    """Return every fragment that still needs at least one acquisition."""

    contract = coverage_depth_replan_contract(entry)
    if not contract:
        return None
    return _merge_remaining_detail_rows(
        row.get("remainingDetail")
        for row in contract.get("coverageDepthObligations") or []
        if isinstance(row, dict)
    )


def _coverage_depth_route_pass_contract(
    depth_contract: Dict[str, Any],
    source: Dict[str, Any],
) -> Dict[str, Any]:
    """Project spatial work onto at most two route traversals.

    Layer A visits need-1 and need-2 fragments.  Layer B visits only need-2
    fragments.  This is a planning projection, not the completion ledger.
    """

    obligations = [
        row
        for row in depth_contract.get("coverageDepthObligations") or []
        if isinstance(row, dict)
    ]
    layer_a = _merge_remaining_detail_rows(
        row.get("remainingDetail")
        for row in obligations
        if int(_to_int(row.get("remainingCaptureCount")) or 0) >= 1
    )
    layer_b = _merge_remaining_detail_rows(
        row.get("remainingDetail")
        for row in obligations
        if int(_to_int(row.get("remainingCaptureCount")) or 0) >= 2
    )

    route_rows: list[Dict[str, Any]] = []
    preferred_single_pass = "forward"
    if (
        _normalize_coverage_pass(
            source.get("activeCoveragePass") or source.get("currentCoveragePass")
        )
        == "reverse"
        or str(source.get("areaCoveragePhase") or "").strip().lower()
        in {"return", "returning", "turn"}
    ):
        preferred_single_pass = "reverse"
    if _remaining_detail_has_geometry(layer_a):
        layer_a_pass = (
            preferred_single_pass
            if not _remaining_detail_has_geometry(layer_b)
            else "forward"
        )
        route_rows.append(
            {
                "coveragePass": str(layer_a_pass),
                "passIndex": 2 if layer_a_pass == "reverse" else 1,
                "obligationKind": "remaining",
                "remainingDetail": deepcopy(layer_a),
                "isDone": False,
                "coverageDepthRouteLayer": 1,
                "forceNewCoverageAcquisition": True,
            }
        )
    if _remaining_detail_has_geometry(layer_b):
        route_rows.append(
            {
                "coveragePass": "reverse",
                "passIndex": 2,
                "obligationKind": "remaining",
                "remainingDetail": deepcopy(layer_b),
                "isDone": False,
                "coverageDepthRouteLayer": 2,
                "forceNewCoverageAcquisition": True,
            }
        )

    remaining_passes = [str(row["coveragePass"]) for row in route_rows]
    if remaining_passes:
        phase = "outbound" if remaining_passes[0] == "forward" else "return"
    else:
        phase = "completed"
    return {
        "areaCoveragePassContractVersion": int(_AREA_COVERAGE_PASS_CONTRACT_VERSION),
        "coveragePassPolicy": "capture_depth_route_projection",
        "coveragePassOrder": list(remaining_passes),
        "coveragePassDetails": deepcopy(route_rows),
        "coveragePassObligations": deepcopy(route_rows),
        "remainingCoveragePasses": list(remaining_passes),
        # A missing route layer means "not needed for this depth workload",
        # never "historically completed".  Completion truth lives only in the
        # spatial depth ledger; declaring the absent layer completed can cause
        # legacy monitoring to seed a fictitious second acquisition.
        "completedCoveragePasses": [],
        "currentCoveragePass": remaining_passes[0] if remaining_passes else None,
        "activeCoveragePass": remaining_passes[0] if remaining_passes else None,
        "areaCoveragePhase": phase,
    }


def coverage_pass_replan_contract(entry: Any) -> Dict[str, Any]:
    """Return the portable reciprocal-Area obligation carried into a replan.

    ``remainingDetail`` is the spatial union needed by legacy planners.  It is
    intentionally not sufficient for reciprocal capture: while OUT is active
    the forward remainder *and* the complete return obligation are required;
    after OUT completes only the reverse remainder is required.  This compact
    contract keeps that distinction across a new mission/plan ID without
    asking every replan path to reinterpret progress percentages.
    """

    # Reciprocal OUT/RETURN coverage is retired.  Legacy payloads are converted
    # to their never-captured first-pass remainder by the unified resolver.
    return {}

    source = entry if isinstance(entry, dict) else {}
    depth_contract = coverage_depth_replan_contract(source)
    raw_rows = source.get("coveragePassDetails")
    if not isinstance(raw_rows, list):
        raw_rows = source.get("coverage_pass_details")
    raw_obligations = source.get("coveragePassObligations")
    if not isinstance(raw_obligations, list):
        raw_obligations = source.get("coverage_pass_obligations")
    explicit_order = source.get("coveragePassOrder")
    if not isinstance(explicit_order, list):
        explicit_order = source.get("coverage_pass_order")
    explicit_pass_contract = bool(
        source.get("areaCoveragePassContractVersion") is not None
        or isinstance(raw_rows, list)
        or isinstance(raw_obligations, list)
        or isinstance(explicit_order, list)
    )
    if not explicit_pass_contract and depth_contract:
        return _coverage_depth_route_pass_contract(depth_contract, source)
    rows_by_pass: Dict[str, Dict[str, Any]] = {}
    order: list[str] = []
    for raw_row in raw_rows or []:
        if not isinstance(raw_row, dict):
            continue
        pass_name = _normalize_coverage_pass(
            raw_row.get("coveragePass", raw_row.get("coverage_pass"))
        )
        if pass_name is None:
            continue
        row = deepcopy(raw_row)
        row["coveragePass"] = str(pass_name)
        row.setdefault("passIndex", len(order) + 1)
        rows_by_pass[pass_name] = row
        if pass_name not in order:
            order.append(pass_name)
    for raw_row in raw_obligations or []:
        if not isinstance(raw_row, dict):
            continue
        pass_name = _normalize_coverage_pass(
            raw_row.get("coveragePass", raw_row.get("coverage_pass"))
        )
        if pass_name is None:
            continue
        merged = deepcopy(rows_by_pass.get(pass_name) or {})
        merged.update(deepcopy(raw_row))
        merged["coveragePass"] = str(pass_name)
        rows_by_pass[pass_name] = merged
        if pass_name not in order:
            order.append(pass_name)
    for value in explicit_order or []:
        pass_name = _normalize_coverage_pass(value)
        if pass_name is not None and pass_name not in order:
            order.append(pass_name)
    if not order:
        return {}

    active_rows: list[Dict[str, Any]] = []
    completed_passes: list[str] = []
    normalized_rows: list[Dict[str, Any]] = []
    for pass_index, pass_name in enumerate(order, start=1):
        row = deepcopy(rows_by_pass.get(pass_name) or {"coveragePass": pass_name})
        row["coveragePass"] = str(pass_name)
        row["passIndex"] = int(_to_int(row.get("passIndex")) or pass_index)
        remaining_detail = row.get("remainingDetail")
        remaining_area = _to_float(row.get("remainingAreaM2"))
        planned_area = _to_float(row.get("plannedAreaM2"))
        covered_area = _to_float(row.get("coveredAreaM2"))
        is_done = bool(row.get("isDone")) or (
            remaining_area is not None
            and remaining_area <= _AREA_EPSILON_M2
            and not _remaining_detail_has_geometry(remaining_detail)
        )
        row["isDone"] = bool(is_done)
        normalized_rows.append(row)
        if is_done:
            completed_passes.append(str(pass_name))
            continue
        obligation = deepcopy(row)
        full_remaining = bool(
            (planned_area is not None and remaining_area is not None
             and remaining_area >= planned_area - _AREA_EPSILON_M2)
            or (covered_area is not None and covered_area <= _AREA_EPSILON_M2)
        )
        obligation["obligationKind"] = "full" if full_remaining else "remaining"
        active_rows.append(obligation)

    remaining_passes = [str(row["coveragePass"]) for row in active_rows]
    phase = str(
        source.get("areaCoveragePhase")
        or source.get("area_coverage_phase")
        or ""
    ).strip().lower()
    active_pass = _normalize_coverage_pass(
        source.get("activeCoveragePass", source.get("active_coverage_pass"))
    )
    current_pass = active_pass or (remaining_passes[0] if remaining_passes else None)
    return {
        "areaCoveragePassContractVersion": int(_AREA_COVERAGE_PASS_CONTRACT_VERSION),
        "coveragePassPolicy": str(
            source.get("coveragePassPolicy")
            or source.get("coverage_pass_policy")
            or "all_passes_required"
        ),
        "coveragePassOrder": list(order),
        "coveragePassDetails": normalized_rows,
        "coveragePassObligations": active_rows,
        "remainingCoveragePasses": remaining_passes,
        "completedCoveragePasses": completed_passes,
        "currentCoveragePass": current_pass,
        "activeCoveragePass": active_pass,
        "areaCoveragePhase": phase or None,
    }


def apply_coverage_pass_replan_contract(
    target: Any,
    source_entry: Any,
) -> Dict[str, Any]:
    """Copy the reciprocal-Area contract to a mission or missionDetail."""

    contract = coverage_pass_replan_contract(source_entry)
    if not isinstance(target, dict) or not contract:
        return contract
    for key, value in contract.items():
        target[key] = deepcopy(value)
    return contract


def apply_area_coverage_replan_contracts(
    target: Any,
    source_entry: Any,
) -> Dict[str, Dict[str, Any]]:
    """Copy completion truth, logical UAV/pass regions and route projection."""

    if isinstance(target, dict):
        strip_area_multi_capture_contracts(target)
    depth_contract = apply_coverage_depth_replan_contract(target, source_entry)
    logical_region_contract = apply_area_logical_region_replan_contract(
        target,
        source_entry,
    )
    if (
        depth_contract
        and isinstance(target, dict)
        and isinstance(source_entry, dict)
        and isinstance(source_entry.get("coveragePassDetails"), list)
    ):
        target["coveragePassAttributionDetails"] = deepcopy(
            source_entry.get("coveragePassDetails") or []
        )
    pass_contract = apply_coverage_pass_replan_contract(target, source_entry)
    return {
        "depth": depth_contract,
        "logicalRegions": logical_region_contract,
        "passes": pass_contract,
    }


def coverage_pass_pending_remaining_detail(entry: Any) -> Optional[Dict[str, Any]]:
    """Build planner geometry from pending pass rows, never completed passes."""

    contract = coverage_pass_replan_contract(entry)
    obligations = contract.get("coveragePassObligations") if contract else None
    if not isinstance(obligations, list) or not obligations:
        return None
    details = [
        row.get("remainingDetail")
        for row in obligations
        if isinstance(row, dict) and _remaining_detail_has_geometry(row.get("remainingDetail"))
    ]
    details = [detail for detail in details if isinstance(detail, dict)]
    if not details:
        return None
    return _merge_remaining_detail_rows(details)


def coverage_replan_pending_remaining_detail(entry: Any) -> Optional[Dict[str, Any]]:
    """Resolve planner geometry with depth-ledger precedence.

    Callers must use this function as the sole source instead of ``... or
    entry['remainingDetail']``.  That legacy fallback resurrects completed
    geometry when an explicit depth ledger correctly reports no obligations.
    """

    legacy_contract, single_remaining, single_completed = (
        _legacy_single_capture_pending_state(entry)
    )
    if legacy_contract:
        if single_completed:
            return None
        return deepcopy(single_remaining) if isinstance(single_remaining, dict) else None
    if isinstance(entry, dict) and isinstance(entry.get("remainingDetail"), dict):
        return deepcopy(entry.get("remainingDetail"))
    return None


def _normalize_done_area_depth_state(target: Any) -> None:
    """Make an explicitly completed Area row internally self-consistent.

    Monitoring can finish a route while a final footprint intersection leaves
    a tiny depth-1 diagnostic band.  Replan consumers already respect
    ``isDone``, but the SIM depth layer reads ``coverageDepthDetails`` directly.
    Normalize only the explicit completion path so a large unfinished mission
    is never collapsed by an area/percentage heuristic.
    """

    if not isinstance(target, dict) or not _mission_is_done(target):
        return

    depth_contract = coverage_depth_replan_contract(target)
    required_depth = max(
        1,
        int(
            _to_int(
                (depth_contract or {}).get(
                    "requiredCoverageDepth",
                    target.get("requiredCoverageDepth"),
                )
            )
            or _DEFAULT_REQUIRED_COVERAGE_DEPTH
        ),
    )
    if depth_contract:
        completed_rows: list[Dict[str, Any]] = []
        for raw_row in depth_contract.get("coverageDepthDetails") or []:
            if not isinstance(raw_row, dict):
                continue
            row_depth = int(_to_int(raw_row.get("coverageDepth")) or 0)
            remaining_count = _to_int(raw_row.get("remainingCaptureCount"))
            if not (
                bool(raw_row.get("isDone"))
                or row_depth >= int(required_depth)
                or (remaining_count is not None and int(remaining_count) <= 0)
            ):
                # This is the stale pending band that must not survive explicit
                # route completion or become a SIM NEED-1/NEED-2 feature.
                continue
            row = deepcopy(raw_row)
            row["coverageDepth"] = int(required_depth)
            row["remainingCaptureCount"] = 0
            row["requiredCoverageDepth"] = int(required_depth)
            row["coveragePercent"] = 100
            row["isDone"] = True
            completed_rows.append(row)

        target["areaCoverageDepthContractVersion"] = int(
            _AREA_COVERAGE_DEPTH_CONTRACT_VERSION
        )
        target["coverageDepthPolicy"] = "spatial_capture_depth"
        target["requiredCoverageDepth"] = int(required_depth)
        target["coverageDepthDetails"] = completed_rows
        target["coverageDepthObligations"] = []
        target["remainingCoverageDepth"] = 0
        target["completedCoverageDepth"] = int(required_depth)
        target["coverageDepthSatisfied"] = True
        target["coverageDepthUnresolvedGeometryCount"] = 0
        target["activeCoverageAcquisitionIDs"] = {}

    planned_area_m2 = _to_float(target.get("plannedAreaM2"))
    if planned_area_m2 is not None:
        target["coveredAreaM2"] = max(0.0, float(planned_area_m2))
    target["remainingAreaM2"] = 0.0
    target["coveragePercent"] = 100
    target["spatialCoveragePercent"] = 100
    target["coverageWorkRemainingM2"] = 0.0
    work_planned_m2 = _to_float(target.get("coverageWorkPlannedM2"))
    if work_planned_m2 is None and planned_area_m2 is not None:
        work_planned_m2 = max(0.0, float(planned_area_m2)) * int(required_depth)
        target["coverageWorkPlannedM2"] = float(work_planned_m2)
    if work_planned_m2 is not None:
        target["coverageWorkCoveredM2"] = max(0.0, float(work_planned_m2))
    if isinstance(target.get("coverageDepthAreaM2"), dict) or planned_area_m2 is not None:
        target["coverageDepthAreaM2"] = {
            str(depth): (
                max(0.0, float(planned_area_m2 or 0.0))
                if depth == int(required_depth)
                else 0.0
            )
            for depth in range(int(required_depth) + 1)
        }
    target["remainingAreaLogicalRegionCount"] = 0
    target["remainingDetail"] = _empty_area_geometry_detail()
    target["areaCoverageWorkloadDetail"] = _empty_area_geometry_detail()


def _normalize_area_entry_logical_regions(entry: Any) -> Any:
    """Keep exact footprint work internal and expose only logical route regions.

    ``remainingAreaM2`` and the spatial-depth rows stay exact.  Only the primary
    geometry surface used by the central ledger, visualization and legacy
    ownership takeover is changed from hundreds of footprint components to a
    bounded set of UAV/pass assignment regions.
    """

    if not isinstance(entry, dict) or _mission_type(entry) != "area":
        return entry
    out = deepcopy(entry)
    legacy_present, single_remaining, single_completed = (
        _legacy_single_capture_pending_state(out)
    )
    single_progress = _legacy_single_capture_progress(out)
    strip_area_multi_capture_contracts(out)
    out.update(single_progress)
    if legacy_present:
        if single_completed:
            out["remainingDetail"] = _empty_area_geometry_detail()
            out["areaCoverageWorkloadDetail"] = _empty_area_geometry_detail()
            out["remainingAreaM2"] = 0.0
            out["isDone"] = True
        elif isinstance(single_remaining, dict):
            out["remainingDetail"] = deepcopy(single_remaining)
            out["areaCoverageWorkloadDetail"] = deepcopy(single_remaining)
            out["isDone"] = False
    elif bool(out.get("isDone")):
        out["remainingDetail"] = _empty_area_geometry_detail()
        out["areaCoverageWorkloadDetail"] = _empty_area_geometry_detail()
    elif isinstance(out.get("remainingDetail"), dict):
        out["areaCoverageWorkloadDetail"] = deepcopy(out.get("remainingDetail"))

    owners_by_aircraft: Dict[int, Dict[str, Any]] = {}
    owners_without_aircraft: list[Dict[str, Any]] = []
    for raw_owner in out.get("areaOwnershipDetails") or []:
        if not isinstance(raw_owner, dict):
            continue
        owner_pass = _normalize_coverage_pass(
            raw_owner.get("coveragePass")
            or raw_owner.get("areaAssignedCoveragePass")
        )
        if owner_pass == "reverse":
            continue
        owner_legacy, owner_remaining, owner_completed = (
            _legacy_single_capture_pending_state(raw_owner)
        )
        owner_progress = _legacy_single_capture_progress(raw_owner)
        owner = deepcopy(raw_owner)
        strip_area_multi_capture_contracts(owner)
        owner.update(owner_progress)
        if owner_legacy and owner_completed:
            owner["remainingDetail"] = _empty_area_geometry_detail()
            owner["areaCoverageWorkloadDetail"] = _empty_area_geometry_detail()
            owner["remainingAreaM2"] = 0.0
            owner["isDone"] = True
        elif owner_legacy and isinstance(owner_remaining, dict):
            owner["remainingDetail"] = deepcopy(owner_remaining)
            owner["areaCoverageWorkloadDetail"] = deepcopy(owner_remaining)
            owner["isDone"] = False
        elif isinstance(owner.get("remainingDetail"), dict):
            owner["areaCoverageWorkloadDetail"] = deepcopy(owner.get("remainingDetail"))
        aircraft_id = _to_int(owner.get("aircraftID"))
        if aircraft_id is None:
            owners_without_aircraft.append(owner)
            continue
        bucket = owners_by_aircraft.setdefault(
            int(aircraft_id),
            {
                "template": owner,
                "assignmentDetails": [],
                "remainingDetails": [],
                "missionIDs": set(),
                "doneFlags": [],
            },
        )
        assignment = area_assignment_detail(owner, fallback=owner.get("remainingDetail"))
        if assignment is not None:
            bucket["assignmentDetails"].append(assignment)
        remaining_geometry = area_geometry_detail(owner.get("remainingDetail"))
        if remaining_geometry is not None and not bool(owner.get("isDone")):
            bucket["remainingDetails"].append(remaining_geometry)
        bucket["doneFlags"].append(bool(owner.get("isDone")))
        for value in list(owner.get("individualMissionIDs") or []) + [
            owner.get("individualMissionID")
        ]:
            mission_id = _to_int(value)
            if mission_id is not None:
                bucket["missionIDs"].add(int(mission_id))

    normalized_owners: list[Dict[str, Any]] = []
    for aircraft_id, bucket in sorted(owners_by_aircraft.items()):
        owner = deepcopy(bucket["template"])
        assignment = _merge_logical_geometry_details(bucket["assignmentDetails"])
        remaining = _merge_logical_geometry_details(bucket["remainingDetails"])
        is_done = bool(bucket["doneFlags"]) and all(bucket["doneFlags"])
        if remaining is not None:
            is_done = False
        if assignment is not None:
            owner["areaAssignmentDetail"] = deepcopy(assignment)
        owner["remainingDetail"] = deepcopy(
            _empty_area_geometry_detail() if is_done else remaining or assignment or _empty_area_geometry_detail()
        )
        owner["areaCoverageWorkloadDetail"] = deepcopy(
            _empty_area_geometry_detail() if is_done else remaining or _empty_area_geometry_detail()
        )
        owner["individualMissionIDs"] = sorted(bucket["missionIDs"])
        owner["aircraftID"] = int(aircraft_id)
        owner["isDone"] = bool(is_done)
        owner["remainingGeometryPolicy"] = "single_capture_region"
        normalized_owners.append(owner)
    normalized_owners.extend(owners_without_aircraft)
    if normalized_owners:
        out["areaOwnershipDetails"] = normalized_owners

    contract = area_logical_region_replan_contract(out)
    for key, value in contract.items():
        out[key] = deepcopy(value)
    out["remainingGeometryPolicy"] = "single_capture_remaining_area"
    diagnostics = dict(out.get("geometryDiagnostics") or {})
    diagnostics["replanInputGeometry"] = "single_capture_remaining_area"
    diagnostics["coverageWorkloadGeometry"] = "single_capture_union"
    diagnostics["areaLogicalRegionCount"] = int(
        contract.get("areaLogicalRegionCount") or 0
    )
    out["geometryDiagnostics"] = diagnostics
    return out

    _normalize_done_area_depth_state(out)
    mission_done = _mission_is_done(out)
    normalized_done_owners: list[Any] = []
    for raw_owner in out.get("areaOwnershipDetails") or []:
        if not isinstance(raw_owner, dict):
            normalized_done_owners.append(raw_owner)
            continue
        owner = deepcopy(raw_owner)
        owner_depth_contract = coverage_depth_replan_contract(owner)
        owner_depth_satisfied = bool(
            (owner_depth_contract or {}).get("coverageDepthSatisfied")
        )
        # An owner can finish its planned route while a real capture hole is
        # still pending.  Only collapse the owner ledger when the aggregate
        # mission is complete or the owner's own depth contract is satisfied.
        if mission_done or owner_depth_satisfied:
            _normalize_done_area_depth_state(owner)
        normalized_done_owners.append(owner)
    if normalized_done_owners:
        out["areaOwnershipDetails"] = normalized_done_owners
    contract = area_logical_region_replan_contract(out)
    if not contract:
        return out
    for key, value in contract.items():
        out[key] = deepcopy(value)

    depth_contract = coverage_depth_replan_contract(out)
    if _mission_is_done(out):
        # Route/pass completion is authoritative for plan hand-off.  A final
        # footprint intersection can leave a tiny diagnostic depth band behind;
        # retaining it on the public workload surface makes the completed piece
        # appear again in simulation and in a later carried snapshot.
        exact_workload = None
    elif depth_contract:
        # The spatial-depth ledger is the completion authority.  Do not fall
        # back to an older workload/remaining surface when it intentionally
        # contains no pending geometry; that would make already swept slivers
        # reappear after carry-forward or a later replan.
        exact_workload = coverage_depth_pending_remaining_detail(out)
    else:
        exact_workload = area_geometry_detail(out.get("areaCoverageWorkloadDetail"))
        if exact_workload is None:
            exact_workload = area_geometry_detail(out.get("remainingDetail"))
    out["areaCoverageWorkloadDetail"] = deepcopy(
        exact_workload or _empty_area_geometry_detail()
    )

    logical_rows = [
        row
        for row in contract.get("areaLogicalRegionDetails") or []
        if isinstance(row, dict)
    ]
    logical_assignment_detail = _merge_logical_geometry_details(
        row.get("areaAssignmentDetail") for row in logical_rows
    )
    if logical_assignment_detail is not None:
        # Owner-level allocated polygons are authoritative.  This also repairs
        # legacy entries whose mission-level assignment was polluted by dense
        # sweep-strip reconstruction (often one outer ring + thousands of
        # artificial holes).
        out["areaAssignmentDetail"] = deepcopy(logical_assignment_detail)
    pending_logical_detail = _merge_logical_geometry_details(
        row.get("remainingDetail")
        for row in logical_rows
        if not bool(row.get("isDone"))
    )
    depth_satisfied = bool(depth_contract.get("coverageDepthSatisfied")) if depth_contract else None
    if pending_logical_detail is not None:
        out["remainingDetail"] = deepcopy(pending_logical_detail)
    elif bool(depth_satisfied) or _mission_is_done(out):
        out["remainingDetail"] = _empty_area_geometry_detail()

    normalized_owners: list[Any] = []
    for raw_owner in out.get("areaOwnershipDetails") or []:
        if not isinstance(raw_owner, dict):
            normalized_owners.append(raw_owner)
            continue
        owner = deepcopy(raw_owner)
        owner_depth_contract = coverage_depth_replan_contract(owner)
        owner_route_done = _mission_is_done(owner)
        owner_depth_satisfied = bool(
            (owner_depth_contract or {}).get("coverageDepthSatisfied")
        )
        owner_done = bool(mission_done) or bool(
            owner_route_done
            and (not owner_depth_contract or owner_depth_satisfied)
        )
        if owner_done:
            owner_workload = None
        elif owner_depth_contract:
            owner_workload = coverage_depth_pending_remaining_detail(owner)
        else:
            owner_workload = area_geometry_detail(owner.get("areaCoverageWorkloadDetail"))
            if owner_workload is None:
                owner_workload = area_geometry_detail(owner.get("remainingDetail"))
        owner["areaCoverageWorkloadDetail"] = deepcopy(
            owner_workload or _empty_area_geometry_detail()
        )
        owner_aircraft_id = _to_int(owner.get("aircraftID"))
        owner_mission_id = _to_int(owner.get("individualMissionID"))
        owner_rows = [
            row
            for row in logical_rows
            if _to_int(row.get("aircraftID")) == owner_aircraft_id
            and (
                owner_mission_id is None
                or owner_mission_id
                in {
                    int(value)
                    for value in row.get("individualMissionIDs") or []
                    if _to_int(value) is not None
                }
            )
        ]
        owner_pending = _merge_logical_geometry_details(
            row.get("remainingDetail")
            for row in owner_rows
            if not bool(row.get("isDone"))
        )
        if owner_pending is not None:
            owner["remainingDetail"] = deepcopy(owner_pending)
        elif owner_rows and all(bool(row.get("isDone")) for row in owner_rows):
            owner["remainingDetail"] = _empty_area_geometry_detail()
        owner["logicalRegionIDs"] = [
            str(row.get("logicalRegionID"))
            for row in owner_rows
            if str(row.get("logicalRegionID") or "").strip()
        ]
        owner["remainingGeometryPolicy"] = "logical_aircraft_pass_region"
        normalized_owners.append(owner)
    if normalized_owners:
        out["areaOwnershipDetails"] = normalized_owners

    out["remainingGeometryPolicy"] = "logical_aircraft_pass_regions"
    diagnostics = dict(out.get("geometryDiagnostics") or {})
    diagnostics["replanInputGeometry"] = "area_logical_region_contract"
    diagnostics["areaLogicalRegionCount"] = int(
        contract.get("areaLogicalRegionCount") or 0
    )
    diagnostics["remainingAreaLogicalRegionCount"] = int(
        contract.get("remainingAreaLogicalRegionCount") or 0
    )
    diagnostics["coverageWorkloadGeometry"] = "coverage_depth_internal"
    out["geometryDiagnostics"] = diagnostics
    return out


def normalize_area_single_capture_entry(entry: Any) -> Any:
    """Public in-memory migration used by planning, monitoring, and SIM."""

    return _normalize_area_entry_logical_regions(entry)


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


def reset_central_area_coverage_entry(
    input_mission_id: int,
    *,
    mission_plan_id: int | None = None,
    reason: str = "explicit_coverage_reset",
) -> bool:
    """Remove one Area depth ledger entry after an explicit mission reset.

    This is intentionally not called by normal replans.  It exists for the
    operator-authorized 0803 execute=2 reset so the monotonic central ledger
    cannot restore observations that were deliberately cleared in memory.
    """

    input_id = _to_int(input_mission_id)
    if input_id is None or input_id <= 0:
        return False
    removed = False
    with _CENTRAL_LEDGER_LOCK:
        ledger = _load_central_ledger()
        entries = ledger.get("entries")
        if not isinstance(entries, dict):
            entries = {}
            ledger["entries"] = entries
        removed = entries.pop(f"area:{int(input_id)}", None) is not None
        if removed:
            _write_central_ledger(ledger)
    _audit(
        "central_area_coverage_reset",
        {
            "inputMissionID": int(input_id),
            "missionPlanID": _to_int(mission_plan_id),
            "reason": str(reason or "explicit_coverage_reset"),
            "removed": bool(removed),
        },
    )
    return bool(removed)


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


def _boundary_guard_cycle_vector(entry: Any) -> Optional[Dict[str, int]]:
    if not isinstance(entry, dict) or not bool(entry.get("boundaryGuardLoop")):
        return None
    rows = entry.get("boundaryGuardSetProgress")
    if not isinstance(rows, list):
        rows = entry.get("areaOwnershipDetails")
    vector: Dict[str, int] = {}
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        set_id = str(
            row.get("boundaryGuardSetID")
            or row.get("boundary_guard_set_id")
            or ""
        ).strip()
        if not set_id:
            continue
        cycle_count = _to_int(
            row.get(
                "boundaryGuardCycleCount",
                row.get("boundary_guard_cycle_count"),
            )
        )
        vector[set_id] = max(
            int(vector.get(set_id, 0)),
            max(0, int(cycle_count or 0)),
        )
    return vector or None


def _central_should_replace_incoming(central_entry: Any, incoming_entry: Any) -> bool:
    if not isinstance(central_entry, dict) or not isinstance(incoming_entry, dict):
        return False
    if _central_ledger_key(central_entry) != _central_ledger_key(incoming_entry):
        return False
    central_guard_cycles = _boundary_guard_cycle_vector(central_entry)
    incoming_guard_cycles = _boundary_guard_cycle_vector(incoming_entry)
    if central_guard_cycles is not None and incoming_guard_cycles is not None:
        # Contract membership changes are authoritative in the incoming plan.
        # With the same set namespace, however, a lower counter is necessarily
        # an older current-cycle snapshot and must not resurrect prior work.
        if set(central_guard_cycles) != set(incoming_guard_cycles):
            return False
        if any(
            int(incoming_guard_cycles[set_id])
            < int(central_guard_cycles[set_id])
            for set_id in central_guard_cycles
        ):
            return True
        if any(
            int(incoming_guard_cycles[set_id])
            > int(central_guard_cycles[set_id])
            for set_id in central_guard_cycles
        ):
            return False
    central_depth = coverage_depth_replan_contract(central_entry)
    if not central_depth:
        return False
    incoming_depth = coverage_depth_replan_contract(incoming_entry)
    if not incoming_depth:
        return True
    if bool(central_depth.get("coverageDepthSatisfied")) and not bool(
        incoming_depth.get("coverageDepthSatisfied")
    ):
        return True
    central_observations = _coverage_observation_ids(central_entry)
    incoming_observations = _coverage_observation_ids(incoming_entry)
    if not central_observations.issubset(incoming_observations):
        return True
    central_work = _coverage_depth_remaining_work_m2(central_entry)
    incoming_work = _coverage_depth_remaining_work_m2(incoming_entry)
    return bool(
        central_work is not None
        and incoming_work is not None
        and incoming_work > central_work + _growth_tolerance(central_work)
    )


def _coverage_observation_ids(entry: Any) -> set[str]:
    contract = coverage_depth_replan_contract(entry)
    return {
        str(row.get("acquisitionID") or row.get("coverageAcquisitionID") or "").strip()
        for row in contract.get("coverageObservationDetails") or []
        if isinstance(row, dict)
        and str(row.get("acquisitionID") or row.get("coverageAcquisitionID") or "").strip()
    }


def _coverage_depth_remaining_work_m2(entry: Any) -> Optional[float]:
    contract = coverage_depth_replan_contract(entry)
    if not contract:
        return None
    work_m2 = 0.0
    saw_area = False
    for row in contract.get("coverageDepthDetails") or []:
        if not isinstance(row, dict) or bool(row.get("isDone")):
            continue
        area_m2 = _to_float(row.get("remainingAreaM2"))
        if area_m2 is None:
            continue
        saw_area = True
        work_m2 += max(0.0, float(area_m2)) * max(
            0,
            int(_to_int(row.get("remainingCaptureCount")) or 0),
        )
    return float(work_m2) if saw_area else None


def _central_should_update(central_entry: Any, incoming_entry: Any) -> bool:
    if not isinstance(incoming_entry, dict):
        return False
    if _central_ledger_key(incoming_entry) is None:
        return False
    if not isinstance(central_entry, dict):
        return True
    if _central_ledger_key(central_entry) != _central_ledger_key(incoming_entry):
        return True
    central_guard_cycles = _boundary_guard_cycle_vector(central_entry)
    incoming_guard_cycles = _boundary_guard_cycle_vector(incoming_entry)
    if incoming_guard_cycles is not None:
        if central_guard_cycles is None:
            return True
        if set(central_guard_cycles) != set(incoming_guard_cycles):
            return True
        if any(
            int(incoming_guard_cycles[set_id])
            < int(central_guard_cycles[set_id])
            for set_id in central_guard_cycles
        ):
            return False
        if any(
            int(incoming_guard_cycles[set_id])
            > int(central_guard_cycles[set_id])
            for set_id in central_guard_cycles
        ):
            # A higher cycle intentionally grows current-cycle remaining area
            # and replaces the prior cycle's observations.
            return True

    central_depth = coverage_depth_replan_contract(central_entry)
    incoming_depth = coverage_depth_replan_contract(incoming_entry)
    if central_depth and not incoming_depth:
        return False
    if central_depth and incoming_depth:
        if bool(central_depth.get("coverageDepthSatisfied")) and not bool(
            incoming_depth.get("coverageDepthSatisfied")
        ):
            return False
        central_observations = _coverage_observation_ids(central_entry)
        incoming_observations = _coverage_observation_ids(incoming_entry)
        if not central_observations.issubset(incoming_observations):
            return False
        central_work = _coverage_depth_remaining_work_m2(central_entry)
        incoming_work = _coverage_depth_remaining_work_m2(incoming_entry)
        if central_work is not None and incoming_work is not None:
            if incoming_work > central_work + _growth_tolerance(central_work):
                return False

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
    normalized = _normalize_area_entry_logical_regions(entry)
    out = deepcopy(normalized if isinstance(normalized, dict) else entry)
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
    migrated_count = 0
    for mission in missions:
        if not isinstance(mission, dict):
            merged_missions.append(mission)
            continue
        normalized_mission = _normalize_area_entry_logical_regions(mission)
        if isinstance(normalized_mission, dict):
            if normalized_mission != mission:
                changed = True
            mission = normalized_mission
        key = _central_ledger_key(mission)
        input_id = _mission_input_id(mission)
        if input_id is not None:
            merged_input_ids.add(int(input_id))
        if key is None:
            merged_missions.append(mission)
            continue

        central_record = entries.get(key)
        central_entry = _central_record_mission(central_record)
        normalized_central = _normalize_area_entry_logical_regions(central_entry)
        if (
            isinstance(central_entry, dict)
            and isinstance(normalized_central, dict)
            and normalized_central != central_entry
        ):
            central_entry = normalized_central
            if isinstance(central_record, dict) and isinstance(
                central_record.get("mission"), dict
            ):
                migrated_record = deepcopy(central_record)
                migrated_record["mission"] = deepcopy(normalized_central)
                entries[str(key)] = migrated_record
            else:
                entries[str(key)] = deepcopy(normalized_central)
            migrated_count += 1
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

        if (
            update_ledger
            # ledger에서 복원된 엔트리를 다시 ledger에 기록하면 updatedAt/plan이
            # 재스탬프되어 출처(원본 플랜·시점)가 세탁된다 — 에코백은 기록하지 않는다.
            and not bool(mission.get("centralAreaLedgerApplied"))
            and _central_should_update(central_entry, mission)
        ):
            entries[str(key)] = _central_record_from_entry(mission, int(mission_plan_id))
            updated_count += 1

        merged_missions.append(output_mission)

    if changed:
        snapshot = dict(snapshot)
        snapshot["missions"] = merged_missions
        snapshot["missionCount"] = len([item for item in merged_missions if isinstance(item, dict)])

    if update_ledger and (updated_count > 0 or migrated_count > 0):
        _write_central_ledger(ledger)
        if audit:
            _audit(
                "central_area_ledger_updated",
                {
                    "missionPlanID": int(mission_plan_id),
                    "updatedEntryCount": int(updated_count),
                    "migratedLogicalRegionEntryCount": int(migrated_count),
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
    depth_contract = coverage_depth_replan_contract(entry)
    if (
        depth_contract
        and not bool(depth_contract.get("coverageDepthSatisfied"))
        and int(depth_contract.get("coverageDepthUnresolvedGeometryCount") or 0) > 0
    ):
        return "area_coverage_depth_geometry_unresolved"
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
    if mission_type in {1, 7}:
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


def _individual_mission_input_id(mission: Any) -> Optional[int]:
    if not isinstance(mission, dict):
        return None
    related = mission.get("relatedMission")
    if not isinstance(related, dict):
        related = {}
    for value in (
        related.get("inputMissionID"),
        related.get("inputMissionId"),
        mission.get("inputMissionID"),
        mission.get("inputMissionId"),
    ):
        input_id = _to_int(value)
        if input_id is not None and input_id > 0:
            return int(input_id)
    return None


def _target_plan_active_area_input_ids(mission_plan_id: int) -> set[int]:
    """Return unfinished AREA inputs referenced by executable target-plan missions.

    A target-attack plan can temporarily replace the current AREA sweep with
    tracking and boundary-hold missions.  Those missions still reference the
    interrupted input ID, while completed inputs are marked done in the input
    package and future missions are execution-blocked.  This narrow set is the
    only safe authority for restoring an AREA entry omitted by a live monitor
    snapshot.
    """

    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(mission_plan_id)}.json")
        plan_data = json.loads(plan_path.read_text(encoding="utf-8"))
    except Exception:
        return set()
    if not isinstance(plan_data, dict):
        return set()

    input_pkg_id = _to_int(
        plan_data.get("inputMissionPackageID")
        or plan_data.get("inputMissionPackageId")
        or plan_data.get("InputMissionPackageID")
    )
    if input_pkg_id is None or input_pkg_id <= 0:
        return set()
    try:
        input_path = db_paths.get_db_subpath("InputMissionPlan", f"{int(input_pkg_id)}.json")
        input_data = json.loads(input_path.read_text(encoding="utf-8"))
    except Exception:
        return set()
    if not isinstance(input_data, dict):
        return set()

    unfinished_area_ids: set[int] = set()
    for input_mission in input_data.get("inputMissionList") or []:
        if not isinstance(input_mission, dict) or bool(input_mission.get("isDone")):
            continue
        input_id = _mission_input_id(input_mission)
        if input_id is None or _input_mission_kind(input_mission) != "area":
            continue
        unfinished_area_ids.add(int(input_id))
    if not unfinished_area_ids:
        return set()

    referenced_ids: set[int] = set()
    aircraft_rows = plan_data.get("aircraftList")
    if not isinstance(aircraft_rows, list):
        aircraft_rows = plan_data.get("AircraftList")
    for aircraft_row in aircraft_rows or []:
        if not isinstance(aircraft_row, dict):
            continue
        aircraft_id = _to_int(
            aircraft_row.get("aircraftID")
            or aircraft_row.get("aircraftId")
            or aircraft_row.get("AircraftID")
        )
        if aircraft_id not in {4, 5, 6}:
            continue
        package_id = _to_int(
            aircraft_row.get("individualMissionPackageID")
            or aircraft_row.get("individualMissionPackageId")
            or aircraft_row.get("IndividualMissionPackageID")
        )
        if package_id is None or package_id <= 0:
            continue
        try:
            package_path = db_paths.get_db_subpath(
                "IndividualMissionPlan",
                f"{int(package_id)}.json",
            )
            package_data = json.loads(package_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(package_data, dict):
            continue
        for individual_mission in package_data.get("individualMissionList") or []:
            if not isinstance(individual_mission, dict):
                continue
            if bool(individual_mission.get("executionBlockedUntilNextCollab")):
                continue
            input_id = _individual_mission_input_id(individual_mission)
            if input_id is not None and int(input_id) in unfinished_area_ids:
                referenced_ids.add(int(input_id))
    return referenced_ids


def _area_detail_relative_size(detail: Any) -> float:
    """Return a stable relative polygon size without adding a GIS dependency."""

    geometry = area_geometry_detail(detail)
    if not isinstance(geometry, dict):
        return 0.0
    area_rows = geometry.get("areaList")
    if not isinstance(area_rows, list) or not area_rows:
        coordinate_list = geometry.get("coordinateList")
        area_rows = (
            [{"isHole": False, "coordinateList": coordinate_list}]
            if isinstance(coordinate_list, list)
            else []
        )

    total = 0.0
    for row in area_rows:
        if not isinstance(row, dict):
            continue
        coordinates = row.get("coordinateList")
        if not isinstance(coordinates, list) or len(coordinates) < 3:
            continue
        points: list[tuple[float, float]] = []
        for coordinate in coordinates:
            if not isinstance(coordinate, dict):
                continue
            latitude = _to_float(coordinate.get("latitude"))
            longitude = _to_float(coordinate.get("longitude"))
            if latitude is None or longitude is None:
                continue
            points.append((float(longitude), float(latitude)))
        if len(points) < 3:
            continue
        latitude_scale = max(
            0.01,
            abs(math.cos(math.radians(sum(y for _, y in points) / len(points)))),
        )
        signed_twice_area = 0.0
        for index, (longitude, latitude) in enumerate(points):
            next_longitude, next_latitude = points[(index + 1) % len(points)]
            signed_twice_area += (
                longitude * latitude_scale * next_latitude
                - next_longitude * latitude_scale * latitude
            )
        polygon_size = abs(signed_twice_area) * 0.5
        if bool(row.get("isHole")):
            total -= polygon_size
        else:
            total += polygon_size
    return max(0.0, float(total))


def _target_plan_area_ownership_assignments(
    mission_plan_id: int,
    *,
    input_mission_ids: Optional[set[int]] = None,
) -> Dict[int, list[Dict[str, Any]]]:
    """Read the AREA ownership that is actually executable in a target plan."""

    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(mission_plan_id)}.json")
        plan_data = json.loads(plan_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    if not isinstance(plan_data, dict):
        return {}

    requested_ids = {
        int(value)
        for value in (input_mission_ids or set())
        if _to_int(value) is not None and int(value) > 0
    }
    assignments: Dict[int, list[Dict[str, Any]]] = {}
    for aircraft in plan_data.get("aircraftList") or []:
        if not isinstance(aircraft, dict):
            continue
        aircraft_id = _to_int(aircraft.get("aircraftID"))
        package_id = _to_int(aircraft.get("individualMissionPackageID"))
        if aircraft_id is None or package_id is None:
            continue
        try:
            imp_path = db_paths.get_db_subpath(
                "IndividualMissionPlan",
                f"{int(package_id)}.json",
            )
            imp_data = json.loads(imp_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(imp_data, dict):
            continue
        for mission in imp_data.get("individualMissionList") or []:
            if not isinstance(mission, dict) or bool(mission.get("isDone")):
                continue
            input_id = _individual_mission_input_id(mission)
            if input_id is None or (requested_ids and int(input_id) not in requested_ids):
                continue
            mission_info = mission.get("individualMissionInfo")
            if not isinstance(mission_info, dict):
                continue
            area_list = mission_info.get("areaList")
            mission_type = _to_int(mission_info.get("individualMissionType"))
            if mission_type != 3 or not isinstance(area_list, list) or not area_list:
                continue
            assignment_detail = {
                "coordinateList": [],
                "lineList": [],
                "areaList": deepcopy(area_list),
            }
            assignments.setdefault(int(input_id), []).append(
                {
                    "aircraftID": int(aircraft_id),
                    "individualMissionID": _to_int(mission.get("individualMissionID")),
                    "inputMissionID": int(input_id),
                    "sourceMissionPlanID": int(mission_plan_id),
                    "pathID": _to_int(mission.get("pathID")),
                    "isDone": False,
                    "areaAssignmentDetail": deepcopy(assignment_detail),
                    "remainingDetail": deepcopy(assignment_detail),
                    "takeoverPolicy": "target_plan_assignment",
                }
            )
    return assignments


def _rebind_area_snapshot_ownership_to_target_plan(
    snapshot: Dict[str, Any],
    target_plan_id: int,
    *,
    input_mission_ids: Optional[set[int]] = None,
    assignments_by_input: Optional[Dict[int, list[Dict[str, Any]]]] = None,
) -> tuple[Dict[str, Any], list[int]]:
    """Make snapshot ownership match newly divided AREA missions in the plan.

    Coverage/remaining totals stay monotonic.  Only the executable owner/path
    partition is replaced, preventing a carried pre-attack partition from being
    rendered on top of a new two/three-UAV collaborative division.
    """

    assignments = assignments_by_input
    if assignments is None:
        assignments = _target_plan_area_ownership_assignments(
            int(target_plan_id),
            input_mission_ids=input_mission_ids,
        )
    if not assignments:
        return snapshot, []
    requested_ids = {
        int(value)
        for value in (input_mission_ids or set())
        if _to_int(value) is not None and int(value) > 0
    }
    missions = snapshot.get("missions")
    if not isinstance(missions, list):
        return snapshot, []

    rebound_ids: list[int] = []
    rebound_missions: list[Any] = []
    for raw_mission in missions:
        input_id = _mission_input_id(raw_mission)
        target_rows = assignments.get(int(input_id)) if input_id is not None else None
        if (
            not isinstance(raw_mission, dict)
            or _mission_type(raw_mission) != "area"
            or input_id is None
            or (requested_ids and int(input_id) not in requested_ids)
            or not target_rows
        ):
            rebound_missions.append(raw_mission)
            continue

        valid_rows = [deepcopy(row) for row in target_rows if isinstance(row, dict)]
        if not valid_rows:
            rebound_missions.append(raw_mission)
            continue
        total_remaining_area = _mission_remaining_area(raw_mission)
        weights = [
            _area_detail_relative_size(row.get("areaAssignmentDetail"))
            for row in valid_rows
        ]
        weight_total = sum(weights)
        if weight_total <= 0.0:
            weights = [1.0 for _ in valid_rows]
            weight_total = float(len(valid_rows))

        owners: list[Dict[str, Any]] = []
        progress_rows: list[Dict[str, Any]] = []
        for index, row in enumerate(valid_rows):
            owner = deepcopy(row)
            owner_area = (
                max(0.0, float(total_remaining_area)) * float(weights[index]) / float(weight_total)
                if total_remaining_area is not None
                else None
            )
            if owner_area is not None:
                owner["plannedAreaM2"] = float(owner_area)
                owner["remainingAreaM2"] = float(owner_area)
            owner["sourceMissionPlanID"] = int(target_plan_id)
            owner["isDone"] = False
            owner["completedLineCount"] = 0
            owner["progressBoundaryLineIndex"] = None
            owner["remainingGeometryPolicy"] = "single_capture_region"
            owners.append(owner)
            progress_rows.append(
                {
                    "aircraftID": _to_int(owner.get("aircraftID")),
                    "individualMissionID": _to_int(owner.get("individualMissionID")),
                    "inputMissionID": int(input_id),
                    "sourceMissionPlanID": int(target_plan_id),
                    "pathID": _to_int(owner.get("pathID")),
                    "currentWaypointID": None,
                    "sweepProgressPoints": 0,
                    "sweepPointCount": 0,
                    "mappedBoundaryLineIndex": None,
                    "confidence": 0.0,
                    "progressSource": "target_plan_assignment",
                }
            )

        merged_assignment = _merge_logical_geometry_details(
            owner.get("areaAssignmentDetail") for owner in owners
        )
        if merged_assignment is None:
            rebound_missions.append(raw_mission)
            continue
        mission = deepcopy(raw_mission)
        mission["missionPlanID"] = int(target_plan_id)
        mission["sourceMissionPlanID"] = int(target_plan_id)
        mission["individualMissionIDs"] = sorted(
            int(value)
            for value in (_to_int(owner.get("individualMissionID")) for owner in owners)
            if value is not None
        )
        mission["aircraftIDs"] = sorted(
            int(value)
            for value in (_to_int(owner.get("aircraftID")) for owner in owners)
            if value is not None
        )
        mission["areaOwnershipDetails"] = owners
        mission["areaProgressDetails"] = progress_rows
        mission["areaAssignmentDetail"] = deepcopy(merged_assignment)
        mission["remainingDetail"] = deepcopy(merged_assignment)
        mission["areaCoverageWorkloadDetail"] = deepcopy(merged_assignment)
        mission["areaOwnershipPolicy"] = "target_plan_assignment"
        mission["remainingGeometryPolicy"] = "single_capture_remaining_area"
        mission.pop("centralAreaLedgerApplied", None)
        mission.pop("centralAreaSourceMissionPlanID", None)
        diagnostics = dict(mission.get("geometryDiagnostics") or {})
        diagnostics["ownershipReboundToMissionPlanID"] = int(target_plan_id)
        diagnostics["ownershipReboundOwnerCount"] = len(owners)
        diagnostics["replanInputGeometry"] = "target_plan_area_assignment"
        mission["geometryDiagnostics"] = diagnostics
        mission = _normalize_area_entry_logical_regions(mission)
        rebound_missions.append(mission)
        rebound_ids.append(int(input_id))

    if not rebound_ids:
        return snapshot, []
    updated = dict(snapshot)
    updated["missions"] = rebound_missions
    updated["missionCount"] = len([item for item in rebound_missions if isinstance(item, dict)])
    updated["areaOwnershipReboundToMissionPlanID"] = int(target_plan_id)
    updated["areaOwnershipReboundInputMissionIDs"] = sorted(set(rebound_ids))
    return updated, sorted(set(rebound_ids))


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


def _restore_missing_area_entries_from_ledger(
    carried: Dict[str, Any],
    target_plan_id: int,
    *,
    audit_context: str = "",
) -> Dict[str, Any]:
    """carry가 area 엔트리를 제거한 뒤 모니터가 새 플랜 스냅샷을 저장하기 전까지,
    타깃 플랜에 포함된 area 입력의 잔여 기하가 스냅샷에서 보이지 않는 공백이 생긴다.
    이 공백에서 rejoin 기하 게이트가 눈멀어 잔여 영역을 통째로 버릴 수 있으므로,
    중앙 ledger에 남은 마지막 잔여 기하(미완료·기하 보유 엔트리만)로 메운다."""
    target_input_kinds = _target_plan_input_kinds(int(target_plan_id))
    if not target_input_kinds:
        return carried
    active_area_input_ids = _target_plan_active_area_input_ids(int(target_plan_id))
    if not active_area_input_ids:
        return carried
    missions = carried.get("missions")
    if not isinstance(missions, list):
        return carried
    present_area_ids = {
        int(input_id)
        for input_id in (
            _mission_input_id(mission)
            for mission in missions
            if isinstance(mission, dict) and _mission_type(mission) == "area"
        )
        if input_id is not None
    }
    missing_area_ids = [
        int(input_id)
        for input_id, kind in sorted(target_input_kinds.items())
        if (
            str(kind) == "area"
            and int(input_id) in active_area_input_ids
            and int(input_id) not in present_area_ids
        )
    ]
    if not missing_area_ids:
        return carried
    with _CENTRAL_LEDGER_LOCK:
        ledger = _load_central_ledger()
    entries = ledger.get("entries")
    if not isinstance(entries, dict):
        return carried
    restored: list[Dict[str, Any]] = []
    for input_id in missing_area_ids:
        mission = _central_record_mission(entries.get(f"area:{int(input_id)}"))
        if not isinstance(mission, dict):
            continue
        if _entry_done_without_remaining(mission):
            continue
        if not _remaining_detail_has_geometry(mission.get("remainingDetail")):
            continue
        restored.append(
            _entry_for_snapshot_plan(mission, int(target_plan_id), applied_from_central=True)
        )
    if not restored:
        return carried
    updated = dict(carried)
    updated_missions = list(missions) + restored
    updated["missions"] = updated_missions
    updated["missionCount"] = len([item for item in updated_missions if isinstance(item, dict)])
    audit_event = (
        "central_area_entry_restored_on_carry"
        if str(audit_context or "").startswith("carry_forward:")
        else "central_area_entry_restored_on_save"
    )
    _audit(
        audit_event,
        {
            "targetMissionPlanID": int(target_plan_id),
            "restoredInputIDs": sorted(
                int(_mission_input_id(mission) or 0) for mission in restored
            ),
            "auditContext": str(audit_context or ""),
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
    # A carry-forward seed and a live monitoring update can be produced by
    # different executors.  Serialize their read/merge/write transactions so a
    # late seed can never race a newer live snapshot.
    with _SNAPSHOT_FILE_LOCK:
        return _save_snapshot_unlocked(mission_plan_id, payload)


def _save_snapshot_unlocked(mission_plan_id: int, payload: Dict[str, Any]) -> Path:
    data = dict(payload or {})
    data.setdefault("missionPlanID", int(mission_plan_id))
    data.setdefault("savedAt", datetime.now(timezone.utc).isoformat())
    data.setdefault("snapshotOrigin", "monitor")
    path = _detail_path(mission_plan_id)
    existing = load_snapshot(int(mission_plan_id))
    data = _merge_with_existing_snapshot(int(mission_plan_id), data, existing)
    # Attack/hold missions can temporarily hide the current AREA sweep from a
    # non-empty live snapshot.  Restore only an unfinished AREA that is still
    # referenced by an executable UAV mission in this exact target plan.  This
    # excludes completed inputs, future collaboration-blocked inputs and AREA
    # inputs removed from the plan.
    data = _restore_missing_area_entries_from_ledger(
        data,
        int(mission_plan_id),
        audit_context="save_snapshot",
    )
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


def snapshot_area_entry_done(
    mission_plan_id: int | None,
    input_mission_id: int | None,
) -> bool:
    """해당 플랜 스냅샷에 area 엔트리가 존재하고 잔여 없이 완료 상태인지 여부.

    '엔트리 부재/미준비'(일시 결손 → 재시도 대상)와 '완료'(종결 대상)를 구분하는
    용도이므로, 엔트리가 없으면 False를 반환한다.
    """
    plan_id = _to_int(mission_plan_id)
    input_id = _to_int(input_mission_id)
    if plan_id is None or input_id is None or plan_id <= 0 or input_id <= 0:
        return False
    snapshot = load_snapshot(int(plan_id))
    if not isinstance(snapshot, dict):
        return False
    for mission in snapshot.get("missions") or []:
        if not isinstance(mission, dict) or _mission_type(mission) != "area":
            continue
        if _mission_input_id(mission) != int(input_id):
            continue
        return _entry_done_without_remaining(mission)
    return False


def carry_forward_snapshot(
    source_plan_id: int | None,
    target_plan_id: int | None,
    *,
    reason: str = "",
    area_ownership_target_input_ids: Optional[Iterable[int]] = None,
) -> Optional[Path]:
    source_id = _to_int(source_plan_id)
    target_id = _to_int(target_plan_id)
    if source_id is None or target_id is None or source_id <= 0 or target_id <= 0:
        return None
    if int(source_id) == int(target_id):
        target_path = _detail_path(int(target_id))
        return target_path if target_path.exists() else None

    with _SNAPSHOT_FILE_LOCK:
        target_path = _detail_path(int(target_id))
        if target_path.exists():
            _audit(
                "snapshot_carry_skipped_existing_target",
                {
                    "sourceMissionPlanID": int(source_id),
                    "targetMissionPlanID": int(target_id),
                    "reason": str(reason or ""),
                    "path": str(target_path),
                },
            )
            return target_path
        return _carry_forward_snapshot_seed_unlocked(
            int(source_id),
            int(target_id),
            reason=reason,
            area_ownership_target_input_ids=area_ownership_target_input_ids,
        )


def _carry_forward_snapshot_seed_unlocked(
    source_plan_id: int | None,
    target_plan_id: int | None,
    *,
    reason: str = "",
    area_ownership_target_input_ids: Optional[Iterable[int]] = None,
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
    carried["snapshotOrigin"] = "carry_forward_seed"
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
    carried = _restore_missing_area_entries_from_ledger(
        carried,
        int(target_id),
        audit_context=f"carry_forward:{reason or ''}",
    )
    requested_area_input_ids = {
        int(value)
        for value in (area_ownership_target_input_ids or [])
        if _to_int(value) is not None and int(value) > 0
    }
    rebound_input_ids: list[int] = []
    if requested_area_input_ids:
        carried, rebound_input_ids = _rebind_area_snapshot_ownership_to_target_plan(
            carried,
            int(target_id),
            input_mission_ids=requested_area_input_ids,
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
            "areaOwnershipReboundInputMissionIDs": list(rebound_input_ids),
            **field_summary,
        },
    )
    return path
