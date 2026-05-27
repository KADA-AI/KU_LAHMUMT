from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Set

from modules.mission_planning.pipelines.next_collab_replan_pipeline_impl import (
    prepare_next_collab_input_replacements,
)
from modules.mission_planning.pipelines.reexecute_first_mission_hybrid import (
    prepare_reexecute_first_mission_replacements,
)


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


@dataclass
class GenericFlightPathSkipResult:
    missions: List[Dict[str, Any]]
    skipped_path_ids: Set[int]
    skipped_aircraft_ids: Set[int]
    skipped_count: int


def _mission_input_id(mission: Dict[str, Any]) -> int | None:
    if not isinstance(mission, dict):
        return None
    related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    try:
        value = related.get("inputMissionID")
        return int(value) if value is not None else None
    except Exception:
        return None


def build_current_remaining_hybrid(
    request: CurrentRemainingHybridRequest,
    *,
    log: Callable[[str], None] | None = None,
) -> CurrentRemainingHybridResult | None:
    emit = log or (lambda _msg: None)
    planner_mode = str(getattr(request, "planner_mode", "") or "current_remaining")
    if planner_mode == "reexecute_first_mission":
        prepared = prepare_reexecute_first_mission_replacements(
            source_plan_id=int(request.source_plan_id),
            current_input_mission=deepcopy(request.current_input_mission),
            entry_coord_map={int(aid): dict(coord) for aid, coord in request.entry_coord_map.items()},
            heading_map={int(aid): float(val) for aid, val in request.heading_map.items()},
            representative_entry=deepcopy(request.representative_entry)
            if isinstance(request.representative_entry, dict)
            else None,
            next_input_mission=deepcopy(request.next_input_mission)
            if isinstance(request.next_input_mission, dict)
            else None,
            turn_radius_scale=float(request.turn_radius_scale),
            source_template_input_id=request.source_template_input_id,
            log=emit,
        )
    else:
        prepared = prepare_next_collab_input_replacements(
            source_plan_id=int(request.source_plan_id),
            target_input_mission=deepcopy(request.current_input_mission),
            entry_coord_map={int(aid): dict(coord) for aid, coord in request.entry_coord_map.items()},
            heading_map={int(aid): float(val) for aid, val in request.heading_map.items()},
            representative_entry=deepcopy(request.representative_entry)
            if isinstance(request.representative_entry, dict)
            else None,
            next_input_mission=deepcopy(request.next_input_mission)
            if isinstance(request.next_input_mission, dict)
            else None,
            turn_radius_scale=float(request.turn_radius_scale),
            log=emit,
        )
    if prepared is None:
        return None

    missions: List[Dict[str, Any]] = []
    for aircraft_id in sorted(prepared.replacement_by_aircraft):
        for mission in prepared.replacement_by_aircraft.get(int(aircraft_id)) or []:
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
    for path_id, payload in sorted(prepared.generated_fp_by_path.items()):
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
    return CurrentRemainingHybridResult(
        current_input_id=int(request.current_input_id),
        missions=missions,
        flight_plans_0303=flight_plans_0303,
        generated_path_ids={int(path_id) for path_id in prepared.generated_path_ids},
        aircraft_ids={int(aid) for aid in aircraft_ids if int(aid) > 0},
        planner_workflow=f"{planner_mode}:{str(prepared.planner_workflow or '')}",
        planner_result_text=str(prepared.planner_result_text or ""),
        flight_plans_0304=flight_plans_0304,
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
    )


def merge_current_remaining_hybrid(
    *,
    missions: List[Dict[str, Any]],
    flight_plans_0303: List[Dict[str, Any]],
    flight_plans_0304: List[Dict[str, Any]],
    hybrid: CurrentRemainingHybridResult,
) -> Dict[str, Any]:
    replace_aircraft_ids = {int(aid) for aid in hybrid.aircraft_ids if int(aid) > 0}
    removed_path_ids: Set[int] = set()
    preserved_missions: List[Dict[str, Any]] = []
    hybrid_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    for mission in hybrid.missions:
        if not isinstance(mission, dict):
            continue
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
            try:
                removed_path_ids.add(int(mission.get("pathID")))
            except Exception:
                pass
            continue
        if aircraft_id in replace_aircraft_ids and aircraft_id not in inserted_aircraft_ids:
            preserved_missions.extend(
                deepcopy(item)
                for item in hybrid_by_aircraft.get(int(aircraft_id), [])
                if isinstance(item, dict)
            )
            inserted_aircraft_ids.add(int(aircraft_id))
        preserved_missions.append(mission)

    for aircraft_id in sorted(replace_aircraft_ids.difference(inserted_aircraft_ids)):
        preserved_missions.extend(
            deepcopy(item)
            for item in hybrid_by_aircraft.get(int(aircraft_id), [])
            if isinstance(item, dict)
        )

    filtered_0303 = [
        deepcopy(fp)
        for fp in flight_plans_0303
        if isinstance(fp, dict) and int(fp.get("pathID") or 0) not in removed_path_ids
    ]
    filtered_0304 = [
        deepcopy(fp)
        for fp in flight_plans_0304
        if isinstance(fp, dict) and int(fp.get("pathID") or 0) not in removed_path_ids
    ]
    merged_missions = [deepcopy(mission) for mission in preserved_missions]
    merged_0303 = filtered_0303 + [deepcopy(fp) for fp in hybrid.flight_plans_0303]
    merged_0304 = filtered_0304 + [
        deepcopy(fp)
        for fp in getattr(hybrid, "flight_plans_0304", []) or []
        if isinstance(fp, dict)
    ]
    return {
        "missions": merged_missions,
        "flight_plans_0303": merged_0303,
        "flight_plans_0304": merged_0304,
        "removed_path_ids": sorted(int(pid) for pid in removed_path_ids),
        "replace_aircraft_ids": sorted(int(aid) for aid in replace_aircraft_ids),
        "planner_workflow": str(hybrid.planner_workflow or ""),
        "planner_result_text": str(hybrid.planner_result_text or ""),
        "generated_path_ids": set(int(path_id) for path_id in hybrid.generated_path_ids),
    }
