from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set

from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
    prepare_next_collab_input_replacements,
)
from modules.mission_planning.pipelines.line_scan_remaining_adapter import (
    apply_line_scan_remaining_to_input_mission,
)
from modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first import (
    prepare_reexecute_first_mission_replacements,
    reexecute_first_mission_generic_skip_policy,
)

CURRENT_REMAINING_TRACKED_UAV_IDS = (4, 5, 6)


@dataclass
class CurrentRemainingHybridRequest:
    source_plan_id: int
    current_input_id: int
    current_input_mission: Dict[str, Any]
    next_input_mission: Dict[str, Any] | None
    entry_coord_map: Dict[int, Dict[str, Any]]
    heading_map: Dict[int, float]
    representative_entry: Dict[str, Any] | None
    turn_radius_scale: float
    apply_option_ordinals: Set[int] | None = None
    planner_mode: str = "current_remaining"
    source_template_input_id: int | None = None
    entry_aircraft_context_map: Dict[int, Dict[str, Any]] = field(default_factory=dict)


@dataclass
class CurrentRemainingHybridGeometry:
    current_input_id: int
    source_plan_id: int
    planner_mode: str
    aircraft_ids: Set[int]
    generated_path_ids: Set[int]
    path_aircraft_by_id: Dict[int, int] = field(default_factory=dict)
    source_template_input_id: int | None = None
    mission_mode: str = ""


@dataclass
class CurrentRemainingHybridRuntimeResult:
    planner_workflow: str
    planner_result_text: str
    prepare_timing_ms: Dict[str, float] = field(default_factory=dict)
    id_reservation: Dict[str, Any] = field(default_factory=dict)
    uav_work_summary: Dict[int, int] = field(default_factory=dict)


@dataclass
class CurrentRemainingHybridResult:
    current_input_id: int
    missions: List[Dict[str, Any]]
    flight_plans_0303: List[Dict[str, Any]]
    generated_path_ids: Set[int]
    aircraft_ids: Set[int]
    planner_workflow: str
    planner_result_text: str
    flight_plans_0304: List[Dict[str, Any]] = field(default_factory=list)
    prepare_timing_ms: Dict[str, float] = field(default_factory=dict)
    geometry: CurrentRemainingHybridGeometry | None = None
    runtime_result: CurrentRemainingHybridRuntimeResult | None = None


@dataclass
class GenericFlightPathSkipResult:
    missions: List[Dict[str, Any]]
    skipped_path_ids: Set[int]
    skipped_aircraft_ids: Set[int]
    skipped_count: int
    skip_policy: Dict[str, Any] = field(default_factory=dict)


def _mission_input_id(mission: Dict[str, Any]) -> int | None:
    if not isinstance(mission, dict):
        return None
    related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    try:
        value = related.get("inputMissionID")
        return int(value) if value is not None else None
    except Exception:
        return None


def _path_ids_from_payloads(payloads: List[Dict[str, Any]] | Set[int] | tuple | None) -> Set[int]:
    results: Set[int] = set()
    for item in payloads or []:
        try:
            if isinstance(item, dict):
                value = item.get("pathID")
            else:
                value = item
            path_id = int(value)
        except Exception:
            continue
        if path_id > 0:
            results.add(int(path_id))
    return results


def validate_current_remaining_hybrid_request(
    request: CurrentRemainingHybridRequest,
    *,
    trigger_detail: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    """Return a compact validation summary for the trigger/request contract."""

    detail_current_input_id = None
    if isinstance(trigger_detail, dict):
        try:
            raw_current_input_id = trigger_detail.get("currentInputMissionID")
            if raw_current_input_id is not None:
                detail_current_input_id = int(raw_current_input_id)
        except Exception:
            detail_current_input_id = None
    mission_current_input_id = _mission_input_id(request.current_input_mission)
    entry_aircraft_ids = sorted(int(aid) for aid in (request.entry_coord_map or {}).keys())
    invalid_aircraft_ids = [
        int(aid)
        for aid in entry_aircraft_ids
        if int(aid) not in CURRENT_REMAINING_TRACKED_UAV_IDS
    ]
    current_matches_trigger = (
        detail_current_input_id is None
        or int(detail_current_input_id) == int(request.current_input_id)
    )
    current_matches_mission = (
        mission_current_input_id is None
        or int(mission_current_input_id) == int(request.current_input_id)
    )
    valid = (
        bool(entry_aircraft_ids)
        and current_matches_trigger
        and current_matches_mission
        and not invalid_aircraft_ids
    )
    return {
        "valid": bool(valid),
        "currentInputMissionID": int(request.current_input_id),
        "triggerCurrentInputMissionID": detail_current_input_id,
        "missionCurrentInputMissionID": mission_current_input_id,
        "currentInputMatchesTrigger": bool(current_matches_trigger),
        "currentInputMatchesMission": bool(current_matches_mission),
        "entryAircraftIDs": entry_aircraft_ids,
        "entryAircraftUavOnly": not invalid_aircraft_ids,
        "invalidEntryAircraftIDs": invalid_aircraft_ids,
        "sourceMissionPlanID": int(request.source_plan_id),
    }


def validate_current_remaining_hybrid_paths(
    *,
    generic_path_ids: List[Dict[str, Any]] | Set[int] | tuple | None,
    hybrid_path_ids: List[Dict[str, Any]] | Set[int] | tuple | None,
) -> Dict[str, Any]:
    generic_ids = _path_ids_from_payloads(generic_path_ids)
    hybrid_ids = _path_ids_from_payloads(hybrid_path_ids)
    overlap = sorted(int(pid) for pid in generic_ids.intersection(hybrid_ids))
    return {
        "valid": not overlap,
        "genericPathIDs": sorted(int(pid) for pid in generic_ids),
        "hybridPathIDs": sorted(int(pid) for pid in hybrid_ids),
        "overlapPathIDs": overlap,
    }


def materialize_current_remaining_hybrid_result(
    request: CurrentRemainingHybridRequest,
    prepared: Any,
    *,
    planner_mode: str | None = None,
) -> CurrentRemainingHybridResult | None:
    if prepared is None:
        return None
    mode = str(planner_mode or getattr(request, "planner_mode", "") or "current_remaining")

    missions: List[Dict[str, Any]] = []
    replacement_by_aircraft = getattr(prepared, "replacement_by_aircraft", {}) or {}
    for aircraft_id in sorted(replacement_by_aircraft):
        for mission in replacement_by_aircraft.get(int(aircraft_id)) or []:
            if not isinstance(mission, dict):
                continue
            mission_entry = deepcopy(mission)
            mission_entry["aircraftID"] = int(aircraft_id)
            missions.append(mission_entry)

    path_aircraft_by_id: Dict[int, int] = {}
    for mission in missions:
        try:
            aircraft_id = int(mission.get("aircraftID", 0))
            path_id = int(mission.get("pathID", 0))
        except Exception:
            continue
        if aircraft_id > 0 and path_id > 0:
            path_aircraft_by_id[int(path_id)] = int(aircraft_id)

    flight_plans_0303: List[Dict[str, Any]] = []
    flight_plans_0304: List[Dict[str, Any]] = []
    generated_fp_by_path = getattr(prepared, "generated_fp_by_path", {}) or {}
    for path_id, payload in sorted(generated_fp_by_path.items()):
        if not isinstance(payload, dict):
            continue
        flight_path = deepcopy(payload)
        try:
            aircraft_id = int(flight_path.get("aircraftID") or path_aircraft_by_id.get(int(path_id)) or 0)
        except Exception:
            aircraft_id = 0
        if aircraft_id > 0:
            flight_path["aircraftID"] = int(aircraft_id)
        if 1 <= int(aircraft_id) <= 3:
            flight_plans_0304.append(flight_path)
        else:
            flight_plans_0303.append(flight_path)
    if not missions or not (flight_plans_0303 or flight_plans_0304):
        return None

    aircraft_ids = {
            int(mission.get("aircraftID"))
            for mission in missions
            if mission.get("aircraftID") is not None
    }
    generated_path_ids = {
        int(path_id)
        for path_id in (getattr(prepared, "generated_path_ids", set()) or set())
        if path_id is not None
    }
    normalized_aircraft_ids = {int(aid) for aid in aircraft_ids if int(aid) > 0}
    prepare_timing_ms = dict(getattr(prepared, "timing_ms", {}) or {})
    workflow = f"{mode}:{str(getattr(prepared, 'planner_workflow', '') or '')}"
    result_text = str(getattr(prepared, "planner_result_text", "") or "")
    geometry = CurrentRemainingHybridGeometry(
        current_input_id=int(request.current_input_id),
        source_plan_id=int(request.source_plan_id),
        planner_mode=str(mode),
        aircraft_ids=set(normalized_aircraft_ids),
        generated_path_ids=set(generated_path_ids),
        path_aircraft_by_id=dict(path_aircraft_by_id),
        source_template_input_id=(
            int(request.source_template_input_id)
            if request.source_template_input_id is not None
            else None
        ),
        mission_mode=str(getattr(prepared, "mission_mode", "") or ""),
    )
    runtime_result = CurrentRemainingHybridRuntimeResult(
        planner_workflow=str(workflow),
        planner_result_text=str(result_text),
        prepare_timing_ms=dict(prepare_timing_ms),
        id_reservation=dict(getattr(prepared, "id_reservation", {}) or {}),
        uav_work_summary={
            int(aid): int(count)
            for aid, count in dict(getattr(prepared, "uav_work_summary", {}) or {}).items()
        },
    )
    return CurrentRemainingHybridResult(
        current_input_id=int(request.current_input_id),
        missions=missions,
        flight_plans_0303=flight_plans_0303,
        generated_path_ids=set(generated_path_ids),
        aircraft_ids=set(normalized_aircraft_ids),
        planner_workflow=str(workflow),
        planner_result_text=str(result_text),
        flight_plans_0304=flight_plans_0304,
        prepare_timing_ms=dict(prepare_timing_ms),
        geometry=geometry,
        runtime_result=runtime_result,
    )


def build_current_remaining_hybrid(
    request: CurrentRemainingHybridRequest,
    *,
    log: Callable[[str], None] | None = None,
) -> CurrentRemainingHybridResult | None:
    emit = log or (lambda _msg: None)
    planner_mode = str(getattr(request, "planner_mode", "") or "current_remaining")
    target_input_mission, line_detail = apply_line_scan_remaining_to_input_mission(
        request.current_input_mission,
        source_plan_id=int(request.source_plan_id),
        input_mission_id=int(request.current_input_id),
    )
    if line_detail:
        emit(
            "[CURRENT-HYBRID][LINE] line_scan remaining injected "
            f"fragments={int(line_detail.get('lineRemainingFragmentCount') or 0)} "
            f"aircraft={line_detail.get('lineScanContributingAircraftIDs') or []}"
        )
    if planner_mode == "reexecute_first_mission":
        prepared = prepare_reexecute_first_mission_replacements(
            source_plan_id=int(request.source_plan_id),
            current_input_mission=deepcopy(target_input_mission),
            entry_coord_map={int(aid): dict(coord) for aid, coord in request.entry_coord_map.items()},
            heading_map={int(aid): float(val) for aid, val in request.heading_map.items()},
            entry_aircraft_context_map={
                int(aid): dict(row)
                for aid, row in (request.entry_aircraft_context_map or {}).items()
                if isinstance(row, dict)
            },
            representative_entry=deepcopy(request.representative_entry),
            next_input_mission=deepcopy(request.next_input_mission),
            turn_radius_scale=float(request.turn_radius_scale),
            source_template_input_id=request.source_template_input_id,
            log=emit,
        )
    else:
        prepared = prepare_next_collab_input_replacements(
            source_plan_id=int(request.source_plan_id),
            target_input_mission=deepcopy(target_input_mission),
            entry_coord_map={int(aid): dict(coord) for aid, coord in request.entry_coord_map.items()},
            heading_map={int(aid): float(val) for aid, val in request.heading_map.items()},
            entry_aircraft_context_map={
                int(aid): dict(row)
                for aid, row in (request.entry_aircraft_context_map or {}).items()
                if isinstance(row, dict)
            },
            representative_entry=deepcopy(request.representative_entry),
            next_input_mission=deepcopy(request.next_input_mission),
            turn_radius_scale=float(request.turn_radius_scale),
            log=emit,
        )
    return materialize_current_remaining_hybrid_result(
        request,
        prepared,
        planner_mode=planner_mode,
    )


def filter_generic_flightpath_missions_for_hybrid(
    missions: List[Dict[str, Any]],
    *,
    request: CurrentRemainingHybridRequest,
    hybrid: CurrentRemainingHybridResult | None = None,
) -> GenericFlightPathSkipResult:
    """Drop generic FlightPath inputs that the current-remaining hybrid replaces.

    This is intentionally applied only after a hybrid result has been built
    successfully. If hybrid preparation fails, callers should keep the original
    generic path generation for functional fallback.
    """

    planner_mode = str(getattr(request, "planner_mode", "") or "current_remaining")
    skip_policy: Dict[str, Any] = {}
    if planner_mode == "reexecute_first_mission":
        skip_policy = reexecute_first_mission_generic_skip_policy(
            current_input_id=hybrid.current_input_id if hybrid is not None else request.current_input_id,
            source_template_input_id=getattr(request, "source_template_input_id", None),
        )
        fallback_current_input_id = hybrid.current_input_id if hybrid is not None else request.current_input_id
        current_input_id = int(skip_policy.get("genericSkipInputMissionID") or fallback_current_input_id)
    else:
        current_input_id = int(
            hybrid.current_input_id if hybrid is not None else request.current_input_id
        )
    replace_aircraft_ids = {
        int(aid)
        for aid in (
            hybrid.aircraft_ids if hybrid is not None else request.entry_coord_map.keys()
        )
        if int(aid) > 0
    }
    kept: List[Dict[str, Any]] = []
    skipped_path_ids: Set[int] = set()
    skipped_aircraft_ids: Set[int] = set()
    skipped_count = 0
    for mission in missions:
        if not isinstance(mission, dict):
            kept.append(mission)
            continue
        try:
            aircraft_id = int(mission.get("aircraftID", 0))
        except Exception:
            aircraft_id = 0
        if aircraft_id in replace_aircraft_ids and _mission_input_id(mission) == current_input_id:
            skipped_count += 1
            skipped_aircraft_ids.add(int(aircraft_id))
            try:
                path_id = int(mission.get("pathID"))
                if path_id > 0:
                    skipped_path_ids.add(path_id)
            except Exception:
                pass
            continue
        kept.append(mission)
    return GenericFlightPathSkipResult(
        missions=kept,
        skipped_path_ids=skipped_path_ids,
        skipped_aircraft_ids=skipped_aircraft_ids,
        skipped_count=int(skipped_count),
        skip_policy=skip_policy,
    )


def merge_current_remaining_hybrid(
    *,
    missions: List[Dict[str, Any]],
    flight_plans_0303: List[Dict[str, Any]],
    flight_plans_0304: List[Dict[str, Any]],
    hybrid: CurrentRemainingHybridResult,
) -> Dict[str, Any]:
    def _temporary_path_id_for_collision(old_path_id: int, used_path_ids: Set[int]) -> int:
        try:
            aircraft_id = int(old_path_id) // 100_000_000
        except Exception:
            aircraft_id = 0
        base = int(aircraft_id) * 100_000_000 if 1 <= int(aircraft_id) <= 6 else 900_000_000
        aircraft_ids = [
            int(pid)
            for pid in used_path_ids
            if int(pid) // 100_000_000 == int(aircraft_id)
        ]
        candidate = max(aircraft_ids or [base]) + 1
        if candidate <= base:
            candidate = base + 1
        while candidate in used_path_ids:
            candidate += 1
        used_path_ids.add(int(candidate))
        return int(candidate)

    def _rewrite_path_ids(payloads: List[Dict[str, Any]], remap: Dict[int, int]) -> None:
        if not remap:
            return
        for payload in payloads or []:
            if not isinstance(payload, dict):
                continue
            try:
                path_id = int(payload.get("pathID", 0))
            except Exception:
                continue
            if path_id in remap:
                payload["pathID"] = int(remap[path_id])

    replace_aircraft_ids = {int(aid) for aid in hybrid.aircraft_ids if int(aid) > 0}
    removed_path_ids: Set[int] = set()
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        try:
            aircraft_id = int(mission.get("aircraftID", 0))
        except Exception:
            aircraft_id = 0
        mission_input_id = _mission_input_id(mission)
        if aircraft_id in replace_aircraft_ids and mission_input_id == int(hybrid.current_input_id):
            try:
                removed_path_ids.add(int(mission.get("pathID")))
            except Exception:
                pass

    filtered_0303 = [
        fp
        for fp in flight_plans_0303
        if isinstance(fp, dict) and int(fp.get("pathID") or 0) not in removed_path_ids
    ]
    filtered_0304 = [
        fp
        for fp in flight_plans_0304
        if isinstance(fp, dict) and int(fp.get("pathID") or 0) not in removed_path_ids
    ]

    hybrid_missions = [
        deepcopy(mission)
        for mission in hybrid.missions
        if isinstance(mission, dict)
    ]
    hybrid_0303 = [
        deepcopy(fp)
        for fp in hybrid.flight_plans_0303
        if isinstance(fp, dict)
    ]
    hybrid_0304 = [
        deepcopy(fp)
        for fp in getattr(hybrid, "flight_plans_0304", []) or []
        if isinstance(fp, dict)
    ]
    hybrid_path_ids = _path_ids_from_payloads(hybrid.generated_path_ids)
    generic_path_ids = _path_ids_from_payloads(list(filtered_0303) + list(filtered_0304))
    temporary_path_id_remap: Dict[int, int] = {}
    used_path_ids = set(generic_path_ids).union(hybrid_path_ids)
    for old_path_id in sorted(generic_path_ids.intersection(hybrid_path_ids)):
        new_path_id = _temporary_path_id_for_collision(int(old_path_id), used_path_ids)
        temporary_path_id_remap[int(old_path_id)] = int(new_path_id)
        hybrid_path_ids.discard(int(old_path_id))
        hybrid_path_ids.add(int(new_path_id))
    if temporary_path_id_remap:
        _rewrite_path_ids(hybrid_missions, temporary_path_id_remap)
        _rewrite_path_ids(hybrid_0303, temporary_path_id_remap)
        _rewrite_path_ids(hybrid_0304, temporary_path_id_remap)

    preserved_missions: List[Dict[str, Any]] = []
    hybrid_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    for mission in hybrid_missions:
        try:
            aircraft_id = int(mission.get("aircraftID", 0))
        except Exception:
            aircraft_id = 0
        if aircraft_id <= 0:
            continue
        hybrid_by_aircraft.setdefault(aircraft_id, []).append(mission)
    inserted_aircraft_ids: Set[int] = set()
    for mission in missions:
        if not isinstance(mission, dict):
            preserved_missions.append(mission)
            continue
        try:
            aircraft_id = int(mission.get("aircraftID", 0))
        except Exception:
            aircraft_id = 0
        mission_input_id = _mission_input_id(mission)
        if aircraft_id in replace_aircraft_ids and mission_input_id == int(hybrid.current_input_id):
            continue
        if aircraft_id in replace_aircraft_ids and aircraft_id not in inserted_aircraft_ids:
            preserved_missions.extend(
                item
                for item in hybrid_by_aircraft.get(int(aircraft_id), [])
                if isinstance(item, dict)
            )
            inserted_aircraft_ids.add(int(aircraft_id))
        preserved_missions.append(mission)

    for aircraft_id in sorted(replace_aircraft_ids.difference(inserted_aircraft_ids)):
        preserved_missions.extend(
            item
            for item in hybrid_by_aircraft.get(int(aircraft_id), [])
            if isinstance(item, dict)
        )

    path_validation = validate_current_remaining_hybrid_paths(
        generic_path_ids=list(filtered_0303) + list(filtered_0304),
        hybrid_path_ids=hybrid_path_ids,
    )
    merged_missions = list(preserved_missions)
    merged_0303 = filtered_0303 + list(hybrid_0303)
    merged_0304 = filtered_0304 + list(hybrid_0304)
    return {
        "missions": merged_missions,
        "flight_plans_0303": merged_0303,
        "flight_plans_0304": merged_0304,
        "removed_path_ids": sorted(int(pid) for pid in removed_path_ids),
        "replace_aircraft_ids": sorted(int(aid) for aid in replace_aircraft_ids),
        "planner_workflow": str(hybrid.planner_workflow or ""),
        "planner_result_text": str(hybrid.planner_result_text or ""),
        "generated_path_ids": set(int(path_id) for path_id in hybrid_path_ids),
        "pathValidation": path_validation,
        "temporaryPathIdRemap": {
            int(old_path_id): int(new_path_id)
            for old_path_id, new_path_id in temporary_path_id_remap.items()
        },
    }
