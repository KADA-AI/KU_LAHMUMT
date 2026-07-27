from __future__ import annotations

import json
import math
import time
import importlib
import concurrent.futures
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from modules.common import db_paths, agent_status_snapshot, prior_replan_store, mission_area_replan_store
from modules.monitoring.logic.replan_runtime_settings import (
    get_post_attack_rejoin_settings,
    get_replan_toggle,
)
from modules.mission_planning._paths import mission_planner_data_def_root
from modules.mission_planning.MissionPlanner.runtime_settings import (
    get_runtime_effective_fov_deg,
    get_runtime_float,
    pop_runtime_camera_fov_adjustment_logs,
    get_runtime_prior_float,
    get_runtime_prior_int,
    get_runtime_prior_mission_profile,
)
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    sanitize_flight_path_payload_filming_altitudes,
)
from modules.mission_planning.runtime.debug_artifacts import debug_artifact_mode, write_debug_json
from modules.mission_planning.runtime.replan_transaction import (
    write_json_transaction as write_json,
    write_json_transaction_batch as write_json_batch,
)
from modules.mission_planning.runtime.cache.source_artifacts import (
    call_with_source_artifact_cache,
    get_active_source_artifact_cache,
    read_json_cached,
)
from modules.mission_planning.runtime.logging.pipeline_events import (
    PipelinePhaseTimer,
    new_replan_transaction_id,
)
from modules.mission_planning.runtime.ids.replan_reservation import (
    ReplanIdReservation,
    ReservedIdBlock,
    summarize_used_reserved_ids,
)
from modules.mission_planning.runtime.validation.replan_payloads import (
    validate_generated_artifact_payloads,
    validate_replan_payloads,
)
from modules.mission_planning.runtime.state.attack_tracking import resolve_plan_lineage_ids
from modules.mission_planning.runtime.state.prior_tracking import (
    clear_prior_assignment,
    list_active_prior_assignments,
    register_prior_assignment,
)
from modules.mission_planning.pipelines.mission_path_trim import (
    DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
    count_sweep_points_in_waypoints,
    load_sweep_progress,
    merge_small_adjacent_line_search_waypoints,
    physical_sweep_cut_points,
    preserve_first_waypoint_altitude_from_reference,
    reassign_unique_waypoint_ids_inplace,
    realign_line_search_waypoints_to_first_sweep,
    recompute_line_search_speed_from_geometry,
    scale_line_search_speed,
    trim_waypoints_by_sweep_points,
    relink_waypoints,
)
from modules.mission_planning.pipelines.line_scan_remaining_adapter import (
    has_line_remaining_geometry,
    load_line_scan_remaining_detail,
)
from modules.mission_planning.replanning.line_entry_context import (
    build_line_entry_context_map,
)
import importlib.util
from types import ModuleType

_EPOCH_2000_MS = 946_684_800_000
_RTB_FLIGHT_MODE = 5
_RELEASE_RESUME_FAST_SPEED_MPS = 58.0
_PRIOR_POST_REJOIN_OPTION_NAME = "선행임무 후 복귀 재계획"
_PRIOR_POST_REJOIN_LOG_BASENAME = "log_prior_post_rejoin"
_ID_ALLOCATOR_MOD: Optional[ModuleType] = None
_MISSION_HELPERS_MOD: Optional[ModuleType] = None
_POST_ATTACK_REJOIN_MOD: Optional[ModuleType] = None


def _load_id_allocator() -> ModuleType:
    global _ID_ALLOCATOR_MOD
    if _ID_ALLOCATOR_MOD is not None:
        return _ID_ALLOCATOR_MOD
    _ID_ALLOCATOR_MOD = importlib.import_module(
        "modules.mission_planning.engine.mission_generation.id_allocation.allocator"
    )
    return _ID_ALLOCATOR_MOD


def _next_imp_id() -> int:
    return int(_load_id_allocator().reserve_imp_ids(1)[0])


def _next_individual_mission_id() -> int:
    return int(_load_id_allocator().reserve_individual_mission_ids(1)[0])


def _next_path_id(aircraft_id: int) -> int:
    return int(_load_id_allocator().reserve_path_ids(aircraft_id, 1)[0])


def _reserve_imp_ids(count: int) -> List[int]:
    return [int(v) for v in _load_id_allocator().reserve_imp_ids(count)]


def _reserve_individual_mission_ids(count: int) -> List[int]:
    return [int(v) for v in _load_id_allocator().reserve_individual_mission_ids(count)]


def _reserve_path_ids(aircraft_id: int, count: int) -> List[int]:
    return [int(v) for v in _load_id_allocator().reserve_path_ids(aircraft_id, count)]


def _next_waypoint_id() -> int:
    return int(_load_id_allocator().reserve_waypoint_block(1))


def _reserve_waypoint_block(count: int) -> int:
    return int(_load_id_allocator().reserve_waypoint_block(count))


def _load_mission_helpers_module() -> Optional[ModuleType]:
    global _MISSION_HELPERS_MOD
    if _MISSION_HELPERS_MOD is not None:
        return _MISSION_HELPERS_MOD
    try:
        module = importlib.import_module(
            "modules.mission_planning.MissionPlanner.data_def.mission_helpers"
        )
    except Exception:
        return None
    _MISSION_HELPERS_MOD = module
    return module


def _sample_dem_altitude(lat: float, lon: float) -> Optional[float]:
    module = _load_mission_helpers_module()
    if module is None:
        return None
    terrain_func = getattr(module, "terrain_elev", None)
    if not callable(terrain_func):
        return None
    try:
        value = float(terrain_func(lat, lon))
    except Exception:
        return None
    if not math.isfinite(value):
        return None
    return value


def warm_prior_mission_pipeline() -> Dict[str, Any]:
    """Preload lazy dependencies used by the prior-mission replan path."""
    status: Dict[str, Any] = {
        "id_allocator_loaded": False,
        "mission_helpers_loaded": False,
        "terrain_elev_available": False,
    }
    allocator = _load_id_allocator()
    status["id_allocator_loaded"] = allocator is not None
    helpers = _load_mission_helpers_module()
    status["mission_helpers_loaded"] = helpers is not None
    status["terrain_elev_available"] = callable(getattr(helpers, "terrain_elev", None)) if helpers else False
    return status


def _load_post_attack_rejoin_module() -> Optional[ModuleType]:
    global _POST_ATTACK_REJOIN_MOD
    if _POST_ATTACK_REJOIN_MOD is not None:
        return _POST_ATTACK_REJOIN_MOD
    try:
        _POST_ATTACK_REJOIN_MOD = importlib.import_module(
            "modules.mission_planning.replanning.triggers.post_attack.pipeline"
        )
    except Exception:
        return None
    return _POST_ATTACK_REJOIN_MOD


def warm_prior_post_rejoin_pipeline() -> Dict[str, Any]:
    module = _load_post_attack_rejoin_module()
    return {
        "prior_tracking_assignments": len(list_active_prior_assignments()),
        "post_attack_helpers_loaded": module is not None,
        "agent_snapshot_available": bool(agent_status_snapshot.load_agent_status_snapshot()),
    }


def _prior_post_rejoin_enabled() -> bool:
    return bool(get_replan_toggle("prior_mission", True))


def _prior_post_rejoin_config() -> Dict[str, Any]:
    return dict(get_post_attack_rejoin_settings() or {})


def _resolve_requested_plan_id(ctx: Dict[str, Any]) -> Optional[int]:
    for value in ctx.get("plan_ids") or []:
        plan_id = _to_int(value)
        if plan_id is not None and plan_id > 0:
            return int(plan_id)
    fallback = _to_int(ctx.get("missionPlanID") or ctx.get("mission_plan_id"))
    if fallback is not None and fallback > 0:
        return int(fallback)
    return None


def _write_prior_post_rejoin_log_payload(payload: Dict[str, Any]) -> Path:
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    path = directory / f"{_PRIOR_POST_REJOIN_LOG_BASENAME}_{timestamp}.json"
    payload["logArtifactMode"] = debug_artifact_mode()
    payload["logArtifactWritten"] = write_debug_json(
        path,
        payload,
        pretty=True,
        ensure_ascii=False,
        skip_if_unchanged=False,
    )
    return path


def _prior_post_rejoin_state_key_policy() -> Dict[str, Any]:
    return {
        "priorStateFile": "prior_tracking_state.json",
        "attackStateFile": "attack_tracking_state.json",
        "assignmentKey": "aircraft_id",
        "priorPlanField": "prior_plan_id",
        "attackPlanField": "attack_plan_id",
        "separateStateStores": True,
    }


def _prior_post_rejoin_noop_delivery_policy() -> Dict[str, Any]:
    return {
        "status0305": 2,
        "notice0001": True,
        "sameAsPostAttackRejoin": True,
        "skipReasons": ["rejoin_not_needed", "remaining_work_too_small"],
    }


def _prior_post_rejoin_id_reservation_policy() -> Dict[str, Any]:
    return {
        "scope": "priorPostRejoin",
        "separateFromPostAttack": True,
        "collaborativeReplacementImp": "ReplanIdReservation",
    }


def _summarize_prior_post_rejoin_lineage(
    *,
    current_plan_id: Optional[int],
    assignments: List[Dict[str, Any]],
) -> Dict[str, Any]:
    def _sorted_ints(key: str) -> List[int]:
        values: Set[int] = set()
        for assignment in assignments or []:
            value = _to_int((assignment or {}).get(key))
            if value is not None and value > 0:
                values.add(int(value))
        return sorted(values)

    return {
        "currentMissionPlanID": _to_int(current_plan_id),
        "priorPlanIDs": _sorted_ints("prior_plan_id"),
        "sourceMissionPlanIDs": _sorted_ints("source_plan_id"),
        "priorMissionIDs": _sorted_ints("prior_mission_id"),
        "currentInputMissionIDs": _sorted_ints("current_input_mission_id"),
        "originalPathIDs": _sorted_ints("original_path_id"),
        "resumePathIDs": _sorted_ints("resume_path_id"),
    }


def _finish_prior_post_rejoin_result(
    *,
    requested_plan_id: Optional[int],
    status: str,
    summary: Dict[str, Any],
    result_payload: Dict[str, Any],
    generated_imp_ids: Optional[Set[int]] = None,
    generated_path_ids: Optional[Set[int]] = None,
    plan_ids: Optional[List[int]] = None,
    option_names: Optional[List[str]] = None,
    plan_meta_map: Optional[Dict[int, Dict[str, Any]]] = None,
) -> PriorPostRejoinPipelineResult:
    log_path = _write_prior_post_rejoin_log_payload(result_payload)
    summary.setdefault("logArtifactMode", result_payload.get("logArtifactMode"))
    summary.setdefault("logArtifactWritten", result_payload.get("logArtifactWritten"))
    normalized_plan_ids = [
        int(pid)
        for pid in (plan_ids or [])
        if _to_int(pid) is not None and int(_to_int(pid) or 0) > 0
    ]
    normalized_option_names = list(option_names or [])
    normalized_meta = dict(plan_meta_map or {})
    if normalized_plan_ids:
        meta_entry = normalized_meta.setdefault(int(normalized_plan_ids[0]), {})
        meta_entry["priorPostRejoin"] = True
        meta_entry["priorPostRejoinContext"] = {
            "status": str(status),
            **{k: v for k, v in dict(summary or {}).items() if k != "option_names"},
            "logPath": str(log_path),
        }
    return PriorPostRejoinPipelineResult(
        plan_ids=normalized_plan_ids,
        option_names=normalized_option_names,
        plan_meta_map=normalized_meta,
        generated_imp_ids={int(val) for val in (generated_imp_ids or set()) if _to_int(val) is not None},
        generated_path_ids={int(val) for val in (generated_path_ids or set()) if _to_int(val) is not None},
        log_path=str(log_path),
        status=str(status),
        summary=dict(summary or {}),
    )


def _match_prior_post_rejoin_assignments(
    *,
    current_plan_id: int,
    aircraft_id: Optional[int],
    preferred_aircraft_ids: Any = None,
    emit: Optional[Callable[[str], None]] = None,
) -> List[Dict[str, Any]]:
    active_aircraft_ids = _active_prior_aircraft_ids_for_source_plan(int(current_plan_id))
    candidate_assignments: List[Dict[str, Any]] = []
    for assignment in list_active_prior_assignments():
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        assignment_aircraft_id = _to_int(assignment.get("aircraft_id"))
        if assignment_aircraft_id is None or int(assignment_aircraft_id) not in active_aircraft_ids:
            continue
        candidate_assignments.append(dict(assignment))
    if not candidate_assignments:
        return []

    raw_preferred_ids = (
        list(preferred_aircraft_ids)
        if isinstance(preferred_aircraft_ids, (list, tuple, set))
        else ([preferred_aircraft_ids] if preferred_aircraft_ids is not None else [])
    )
    if aircraft_id is not None:
        raw_preferred_ids.append(int(aircraft_id))
    preferred_ids = {
        int(value)
        for value in (_to_int(item) for item in raw_preferred_ids)
        if value is not None and int(value) > 0
    }
    if preferred_ids:
        preferred_matches = [
            assignment
            for assignment in candidate_assignments
            if _to_int(assignment.get("aircraft_id")) in preferred_ids
        ]
        if preferred_matches:
            return preferred_matches
        if emit:
            emit(
                "preferred prior-close aircraft missing from active prior assignments; "
                f"fallback candidates={sorted(int(_to_int(item.get('aircraft_id')) or 0) for item in candidate_assignments)}."
            )

    if aircraft_id is None or aircraft_id <= 0:
        return candidate_assignments

    direct_matches = [
        assignment
        for assignment in candidate_assignments
        if _to_int(assignment.get("aircraft_id")) == int(aircraft_id)
    ]
    if direct_matches:
        return direct_matches
    if len(candidate_assignments) == 1:
        return candidate_assignments
    return []


def _resolve_prior_assignment_input_mission_id(assignment: Dict[str, Any]) -> Optional[int]:
    current_input_id = _to_int(assignment.get("current_input_mission_id"))
    if current_input_id is not None and current_input_id > 0:
        return int(current_input_id)

    source_plan_id = _to_int(assignment.get("source_plan_id"))
    aircraft_id = _to_int(assignment.get("aircraft_id"))
    original_individual_mission_id = _to_int(assignment.get("original_individual_mission_id"))
    if source_plan_id is None or aircraft_id is None or original_individual_mission_id is None:
        return None

    imp_data = _load_imp_package_for_aircraft(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
    )
    if not isinstance(imp_data, dict):
        return None
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _to_int(mission.get("individualMissionID")) != int(original_individual_mission_id):
            continue
        input_mission_id = _extract_related_input_mission_id(mission)
        if input_mission_id is not None and input_mission_id > 0:
            return int(input_mission_id)
    return None


def _resolve_prior_group_source_plan_id(
    group_assignments: List[Dict[str, Any]],
    *,
    fallback_plan_id: int,
) -> int:
    for assignment in group_assignments or []:
        if not isinstance(assignment, dict):
            continue
        source_plan_id = _to_int(assignment.get("source_plan_id"))
        if source_plan_id is not None and source_plan_id > 0:
            return int(source_plan_id)
    return int(fallback_plan_id)


def _plan_carries_prior_assignment(
    source_plan_id: int,
    assignment: Dict[str, Any],
) -> bool:
    """Return whether the current plan still references an active prior IMP entry."""

    aircraft_id = _to_int(assignment.get("aircraft_id"))
    prior_individual_id = _to_int(assignment.get("prior_individual_mission_id"))
    prior_path_id = _to_int(assignment.get("prior_path_id"))
    if aircraft_id is None or (prior_individual_id is None and prior_path_id is None):
        return False
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = read_json_cached(plan_path, copy_result=False, kind="MissionPlan")
        package_id = None
        for aircraft in plan_data.get("aircraftList") or []:
            if isinstance(aircraft, dict) and _to_int(aircraft.get("aircraftID")) == int(aircraft_id):
                package_id = _to_int(aircraft.get("individualMissionPackageID"))
                break
        if package_id is None:
            return False
        imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(package_id)}.json")
        imp_data = read_json_cached(imp_path, copy_result=False, kind="IndividualMissionPlan")
    except Exception:
        return False
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if (
            prior_individual_id is not None
            and _to_int(mission.get("individualMissionID")) == int(prior_individual_id)
        ) or (
            prior_path_id is not None
            and _to_int(mission.get("pathID")) == int(prior_path_id)
        ):
            return True
    return False


def _active_prior_aircraft_ids_for_source_plan(source_plan_id: Optional[int]) -> Set[int]:
    """Return active prior-mission UAVs carried by the current plan."""

    normalized_plan_id = _to_int(source_plan_id)
    if normalized_plan_id is None or normalized_plan_id <= 0:
        return set()
    try:
        plan_lineage = resolve_plan_lineage_ids(int(normalized_plan_id)) or {int(normalized_plan_id)}
    except Exception:
        plan_lineage = {int(normalized_plan_id)}
    try:
        assignments = [
            dict(assignment)
            for assignment in list_active_prior_assignments()
            if isinstance(assignment, dict) and bool(assignment.get("active"))
        ]
    except Exception:
        assignments = []

    # Active assignment records themselves form source -> prior edges. Walk
    # those edges backwards so correctness does not depend on optional run logs.
    related_plan_ids = set(int(value) for value in plan_lineage)
    changed = True
    while changed:
        changed = False
        for assignment in assignments:
            prior_plan_id = _to_int(assignment.get("prior_plan_id"))
            assignment_source_id = _to_int(assignment.get("source_plan_id"))
            if (
                prior_plan_id is None
                or int(prior_plan_id) not in related_plan_ids
                or assignment_source_id is None
                or int(assignment_source_id) in related_plan_ids
            ):
                continue
            related_plan_ids.add(int(assignment_source_id))
            changed = True

    aircraft_ids: Set[int] = set()
    for assignment in assignments:
        prior_plan_id = _to_int(assignment.get("prior_plan_id"))
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        if (
            prior_plan_id is None
            or aircraft_id is None
            or aircraft_id <= 3
        ):
            continue
        if int(prior_plan_id) not in related_plan_ids and not _plan_carries_prior_assignment(
            int(normalized_plan_id),
            assignment,
        ):
            continue
        aircraft_ids.add(int(aircraft_id))
    return aircraft_ids


def _rebase_prior_source_plan_to_latest_applied(source_plan_id: Optional[int]) -> Optional[int]:
    """Use a newer applied descendant when a queued prior request carries an older source."""

    normalized_plan_id = _to_int(source_plan_id)
    if normalized_plan_id is None or normalized_plan_id <= 0:
        return normalized_plan_id
    latest_plan_id = _load_latest_mission_progress_plan_id()
    if latest_plan_id is None or int(latest_plan_id) == int(normalized_plan_id):
        return int(normalized_plan_id)
    try:
        latest_lineage = resolve_plan_lineage_ids(int(latest_plan_id)) or {int(latest_plan_id)}
    except Exception:
        latest_lineage = {int(latest_plan_id)}
    if int(normalized_plan_id) in latest_lineage:
        return int(latest_plan_id)

    try:
        assignments = [
            dict(assignment)
            for assignment in list_active_prior_assignments()
            if isinstance(assignment, dict) and bool(assignment.get("active"))
        ]
    except Exception:
        assignments = []
    reachable_plan_ids = {int(normalized_plan_id)}
    changed = True
    while changed:
        changed = False
        for assignment in assignments:
            assignment_source_id = _to_int(assignment.get("source_plan_id"))
            prior_plan_id = _to_int(assignment.get("prior_plan_id"))
            if (
                assignment_source_id is None
                or int(assignment_source_id) not in reachable_plan_ids
                or prior_plan_id is None
                or int(prior_plan_id) in reachable_plan_ids
            ):
                continue
            reachable_plan_ids.add(int(prior_plan_id))
            changed = True
    if int(latest_plan_id) in reachable_plan_ids:
        return int(latest_plan_id)
    for assignment in assignments:
        assignment_source_id = _to_int(assignment.get("source_plan_id"))
        prior_plan_id = _to_int(assignment.get("prior_plan_id"))
        if not (
            (assignment_source_id is not None and int(assignment_source_id) in reachable_plan_ids)
            or (prior_plan_id is not None and int(prior_plan_id) in reachable_plan_ids)
        ):
            continue
        if _plan_carries_prior_assignment(int(latest_plan_id), assignment):
            return int(latest_plan_id)
    return int(normalized_plan_id)


def _evaluate_prior_rejoin_group(
    *,
    current_plan_id: int,
    current_input_id: int,
    group_assignments: List[Dict[str, Any]],
    agent_state_map: Dict[int, Dict[str, Any]],
    config: Dict[str, Any],
    emit: Callable[[str], None],
) -> Dict[str, Any]:
    post_attack = _load_post_attack_rejoin_module()
    if post_attack is None:
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "post_attack_helpers_unavailable",
        }

    team_aircraft_ids = _aircraft_ids_for_input_mission(
        source_plan_id=int(current_plan_id),
        input_mission_id=int(current_input_id),
    )
    active_prior_aircraft_ids = _active_prior_aircraft_ids_for_source_plan(int(current_plan_id))
    closed_aircraft_ids = {
        int(aid)
        for aid in (_to_int(item.get("aircraft_id")) for item in group_assignments)
        if aid is not None and aid > 3
    }
    if not team_aircraft_ids:
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "team_aircraft_unavailable",
        }

    ongoing_prior_aircraft_ids: Set[int] = set()
    for assignment in list_active_prior_assignments():
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        if _resolve_prior_assignment_input_mission_id(assignment) != int(current_input_id):
            continue
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        if (
            aircraft_id is None
            or int(aircraft_id) not in active_prior_aircraft_ids
            or aircraft_id in closed_aircraft_ids
        ):
            continue
        ongoing_prior_aircraft_ids.add(int(aircraft_id))

    available_aircraft_ids = {
        int(aid)
        for aid in team_aircraft_ids
        if int(aid) > 3 and int(aid) not in ongoing_prior_aircraft_ids
    }
    active_aircraft_ids = sorted(int(aid) for aid in available_aircraft_ids if int(aid) not in closed_aircraft_ids)
    returning_aircraft_ids = sorted(int(aid) for aid in closed_aircraft_ids if int(aid) in available_aircraft_ids)

    if not returning_aircraft_ids:
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "returning_aircraft_missing",
            "ongoing_tracking_aircraft_ids": sorted(ongoing_prior_aircraft_ids),
        }
    if not active_aircraft_ids:
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "active_aircraft_missing",
            "ongoing_tracking_aircraft_ids": sorted(ongoing_prior_aircraft_ids),
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    progress_summary = post_attack._summarize_active_group_progress(
        current_input_id=int(current_input_id),
        active_aircraft_ids=active_aircraft_ids,
    )
    active_progress_skip_percent = max(
        0,
        min(
            100,
            _to_int(config.get("active_progress_skip_percent"))
            or int(getattr(post_attack, "_DEFAULT_ACTIVE_PROGRESS_SKIP_PERCENT", 70)),
        ),
    )
    active_avg_progress_percent = progress_summary.get("active_avg_progress_percent")
    active_progress_sample_count = int(progress_summary.get("active_progress_sample_count") or 0)
    if (
        active_progress_sample_count > 0
        and active_avg_progress_percent is not None
        and float(active_avg_progress_percent) >= float(active_progress_skip_percent)
    ):
        emit(
            f"[PRIOR-REJOIN] inputMissionID={current_input_id} rejoin skipped: "
            f"active UAV avg progress {float(active_avg_progress_percent):.1f}% >= "
            f"{int(active_progress_skip_percent)}%."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "active_group_progress_high",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            **progress_summary,
            "ongoing_tracking_aircraft_ids": sorted(ongoing_prior_aircraft_ids),
            "available_aircraft_ids": sorted(available_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    if not post_attack._has_remaining_snapshot_geometry(int(current_plan_id), int(current_input_id)):
        emit(
            f"[PRIOR-REJOIN] inputMissionID={current_input_id} rejoin skipped: "
            "current remaining snapshot geometry unavailable."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "remaining_snapshot_unavailable",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            **progress_summary,
            "ongoing_tracking_aircraft_ids": sorted(ongoing_prior_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    reference_coord = post_attack._select_rejoin_reference_coordinate(
        active_aircraft_ids=active_aircraft_ids,
        agent_state_map=agent_state_map,
        current_plan_id=int(current_plan_id),
        current_input_id=int(current_input_id),
    )
    if reference_coord is None:
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "rejoin_reference_unavailable",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            **progress_summary,
            "ongoing_tracking_aircraft_ids": sorted(ongoing_prior_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    active_remaining_eta_s = post_attack._estimate_group_remaining_eta_s(
        source_plan_id=int(current_plan_id),
        current_input_id=int(current_input_id),
        aircraft_ids=active_aircraft_ids,
        agent_state_map=agent_state_map,
        emit=emit,
    )
    return_eta_map: Dict[int, int] = {}
    for assignment in group_assignments:
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        if aircraft_id is None or aircraft_id not in returning_aircraft_ids:
            continue
        state = agent_state_map.get(int(aircraft_id)) or {}
        coord = post_attack._normalize_coordinate(state.get("coordinate"))
        if coord is None:
            coord = (
                post_attack._normalize_coordinate(assignment.get("handoff_coordinate"))
                or post_attack._normalize_coordinate(assignment.get("last_nonzero_coordinate"))
                or post_attack._normalize_coordinate(assignment.get("original_coordinate"))
            )
        heading = _to_float(state.get("heading"))
        speed = _to_float(state.get("speed"))
        return_eta_map[int(aircraft_id)] = post_attack._estimate_turn_aware_eta_s(
            origin=coord,
            destination=reference_coord,
            heading_deg=heading,
            speed_value=speed,
            turn_radius_m=_to_float(config.get("turn_radius_m"))
            or float(getattr(post_attack, "_DEFAULT_TURN_RADIUS_M", 180.0)),
            default_cruise_speed_mps=_to_float(config.get("default_cruise_speed_mps"))
            or float(getattr(post_attack, "_DEFAULT_CRUISE_SPEED_MPS", 35.0)),
        )

    max_return_eta_s = max(return_eta_map.values()) if return_eta_map else 0
    min_remaining_eta_s = max(
        0,
        _to_int(config.get("min_remaining_eta_s"))
        or int(getattr(post_attack, "_DEFAULT_MIN_REMAINING_ETA_S", 120)),
    )
    rejoin_margin_s = max(
        0,
        _to_int(config.get("rejoin_margin_s"))
        or int(getattr(post_attack, "_DEFAULT_REJOIN_MARGIN_S", 45)),
    )
    replan_needed = bool(
        active_remaining_eta_s >= int(min_remaining_eta_s)
        and active_remaining_eta_s > (max_return_eta_s + int(rejoin_margin_s))
    )
    skip_reason = None if replan_needed else "remaining_work_too_small"
    return {
        "input_mission_id": int(current_input_id),
        "replan_needed": replan_needed,
        "skip_reason": skip_reason,
        "active_progress_skip_percent": int(active_progress_skip_percent),
        **progress_summary,
        "active_remaining_eta_s": int(active_remaining_eta_s),
        "max_return_eta_s": int(max_return_eta_s),
        "min_remaining_eta_s": int(min_remaining_eta_s),
        "rejoin_margin_s": int(rejoin_margin_s),
        "rejoin_reference": dict(reference_coord),
        "available_aircraft_ids": sorted(available_aircraft_ids),
        "ongoing_tracking_aircraft_ids": sorted(ongoing_prior_aircraft_ids),
        "active_aircraft_ids": active_aircraft_ids,
        "returning_aircraft_ids": returning_aircraft_ids,
        "return_eta_by_aircraft": {int(aid): int(val) for aid, val in return_eta_map.items()},
    }


def run_prior_post_rejoin_pipeline(
    ctx: Dict[str, Any],
    detail: Dict[str, Any],
    reason: str,
    *,
    log: Optional[Callable[[str], None]] = None,
) -> PriorPostRejoinPipelineResult:
    emit = log or (lambda _msg: None)
    detail = dict(detail or {})
    transaction_id = new_replan_transaction_id("prior-rejoin")
    phase_timer = PipelinePhaseTimer(
        pipeline="prior_post_rejoin",
        replan_transaction_id=transaction_id,
        emit_events=True,
    )
    now_ms = _now_ms_since_2000()
    log_messages: List[str] = []

    def _emit(message: str) -> None:
        log_messages.append(str(message))
        emit(f"[PRIOR-REJOIN] {message}")

    post_attack = _load_post_attack_rejoin_module()
    requested_plan_id = _resolve_requested_plan_id(ctx)
    current_plan_id = _to_int(
        detail.get("currentMissionPlanID")
        or detail.get("priorPlanID")
        or detail.get("sourceMissionPlanID")
        or ctx.get("currentMissionPlanID")
        or ctx.get("sourceMissionPlanID")
    )
    trigger = str(detail.get("trigger") or "").strip()
    trigger_type = str(detail.get("triggerType") or "").strip()
    aircraft_id = _to_int(detail.get("aircraftID") or detail.get("aircraftId"))
    result_payload: Dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "reason": str(reason or ctx.get("reason") or ""),
        "trigger": trigger,
        "triggerType": trigger_type,
        "currentPlanID": current_plan_id,
        "aircraftID": aircraft_id,
        "evaluations": [],
        "logMessages": log_messages,
        "replanTransactionId": transaction_id,
        "stateKeyPolicy": _prior_post_rejoin_state_key_policy(),
        "noopDeliveryPolicy": _prior_post_rejoin_noop_delivery_policy(),
        "idReservationPolicy": _prior_post_rejoin_id_reservation_policy(),
    }
    config = _prior_post_rejoin_config()

    def _finish_with_timing(**kwargs: Any) -> PriorPostRejoinPipelineResult:
        result_payload["timingMs"] = phase_timer.snapshot(include_total=True)
        summary = kwargs.get("summary")
        if isinstance(summary, dict):
            summary.setdefault("timingMs", dict(result_payload.get("timingMs") or {}))
            summary.setdefault("replanTransactionId", transaction_id)
            for key in (
                "stateKeyPolicy",
                "noopDeliveryPolicy",
                "idReservationPolicy",
                "lineage",
                "collaborativeReservationSummaries",
            ):
                if key in result_payload:
                    summary.setdefault(key, result_payload.get(key))
        return _finish_prior_post_rejoin_result(**kwargs)

    if trigger != "0401" or trigger_type != "priorClosedResume":
        _emit("ignored: detail is not a prior-close resume trigger.")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "not_prior_close_trigger"},
            result_payload=result_payload,
        )
    if not _prior_post_rejoin_enabled():
        _emit("skipped: prior mission trigger is disabled in monitoring settings.")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "prior_mission_disabled"},
            result_payload=result_payload,
        )
    if post_attack is None:
        _emit("skipped: post-attack rejoin helper module unavailable.")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "post_attack_helpers_unavailable"},
            result_payload=result_payload,
        )
    if current_plan_id is None or current_plan_id <= 0 or aircraft_id is None or aircraft_id <= 0:
        _emit("skipped: missing currentMissionPlanID/aircraftID in prior-close detail.")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "missing_trigger_identifiers"},
            result_payload=result_payload,
        )

    assignments = _match_prior_post_rejoin_assignments(
        current_plan_id=int(current_plan_id),
        aircraft_id=int(aircraft_id),
        preferred_aircraft_ids=detail.get("returningAircraftIDList"),
        emit=_emit,
    )
    if not assignments:
        _emit(
            f"skipped: no active prior assignment matched aircraft={int(aircraft_id)} "
            f"on priorPlan={int(current_plan_id)}."
        )
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "prior_assignment_not_found"},
            result_payload=result_payload,
        )
    result_payload["lineage"] = _summarize_prior_post_rejoin_lineage(
        current_plan_id=int(current_plan_id),
        assignments=assignments,
    )
    phase_timer.mark("state_key_assignment_load")

    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(current_plan_id)}.json")
        plan_data = read_json_cached(plan_path, kind="MissionPlan")
    except Exception as exc:
        _emit(f"failed to load current MissionPlan {current_plan_id}: {exc}")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "current_plan_load_failed"},
            result_payload=result_payload,
        )
    phase_timer.mark("current_mission_plan_load")

    snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    agent_state_map = post_attack._index_agent_states(
        snapshot.get("agent_states") or [],
        snapshot.get("last_nonzero_waypoint_by_aircraft"),
    )
    phase_timer.mark("agent_snapshot_load")
    allocate_fresh_plan_id = getattr(post_attack, "_allocate_fresh_plan_id", None)
    new_plan_id = int(requested_plan_id or (allocate_fresh_plan_id() if callable(allocate_fresh_plan_id) else 700000001))
    # plan_data는 read_json_cached 반환본(호출자 소유)이고 이후 재사용 없음
    new_plan_data = plan_data
    new_plan_data["missionPlanID"] = int(new_plan_id)
    new_plan_data["timestamp"] = int(now_ms)
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = int(now_ms)

    generated_imp_ids: Set[int] = set()
    generated_path_ids: Set[int] = set()
    updated_aircraft_ids: Set[int] = set()
    cleared_aircraft_ids: Set[int] = set()
    group_summaries: List[Dict[str, Any]] = []
    collaborative_reservation_summaries: List[Dict[str, Any]] = []
    assignments_by_input: Dict[int, List[Dict[str, Any]]] = {}
    skipped_assignments: List[Dict[str, Any]] = []
    for assignment in assignments:
        current_input_id = _resolve_prior_assignment_input_mission_id(assignment)
        if current_input_id is None or current_input_id <= 0:
            skipped_assignments.append(
                {
                    "aircraftID": _to_int(assignment.get("aircraft_id")),
                    "reason": "inputMissionID_unavailable",
                }
            )
            continue
        normalized = dict(assignment)
        normalized["current_input_mission_id"] = int(current_input_id)
        assignments_by_input.setdefault(int(current_input_id), []).append(normalized)
    if skipped_assignments:
        result_payload["skippedAssignments"] = skipped_assignments

    def _clear_group_prior_assignments(
        grouped_assignments: List[Dict[str, Any]],
        *,
        skip_reason: str,
    ) -> List[int]:
        released_ids: Set[int] = set()
        for grouped in grouped_assignments:
            grouped_aircraft_id = _to_int(grouped.get("aircraft_id"))
            if grouped_aircraft_id is None or grouped_aircraft_id <= 0:
                continue
            clear_prior_assignment(int(grouped_aircraft_id), reason=skip_reason or "rejoin_not_needed")
            released_ids.add(int(grouped_aircraft_id))
        if released_ids:
            _emit(
                "Prior assignments cleared without collaborative replan -> "
                f"aircraft={sorted(released_ids)} (reason={skip_reason or 'rejoin_not_needed'})."
            )
        return sorted(int(aid) for aid in released_ids)

    for current_input_id, group_assignments in assignments_by_input.items():
        evaluation = _evaluate_prior_rejoin_group(
            current_plan_id=int(current_plan_id),
            current_input_id=int(current_input_id),
            group_assignments=group_assignments,
            agent_state_map=agent_state_map,
            config=config,
            emit=_emit,
        )
        if _source_input_mission_is_locked_type2_branch(
            int(current_plan_id),
            int(current_input_id),
        ):
            evaluation["replan_needed"] = False
            evaluation["skip_reason"] = "type2_branch_owner_resume_preserved"
            _emit(
                "[PRIOR-REJOIN][TYPE2] immutable branch ownership preserved; "
                "the returning UAV continues its own stored resume chain."
            )
        group_summaries.append(evaluation)
        result_payload["evaluations"].append(evaluation)

        if not bool(evaluation.get("replan_needed")):
            active_completed_updates: List[Dict[str, Any]] = []
            active_completed_failed_aircraft_ids: Set[int] = set()
            active_progress_by_aircraft = evaluation.get("active_progress_by_aircraft")
            active_progress_by_aircraft = (
                active_progress_by_aircraft if isinstance(active_progress_by_aircraft, dict) else {}
            )
            active_done_hold_seconds = post_attack._estimate_active_done_hold_seconds(
                current_plan_id=int(current_plan_id),
                current_input_id=int(current_input_id),
                evaluation=evaluation,
                group_assignments=group_assignments,
                agent_state_map=agent_state_map,
                config=config,
            )
            if int(active_done_hold_seconds) > int(getattr(post_attack, "_POST_ATTACK_COMPLETE_HOLD_SECONDS", 5)):
                _emit(
                    "[PRIOR-REJOIN][ACTIVE-DONE] completion hold extended for returning UAV rejoin "
                    f"(hold={int(active_done_hold_seconds)}s)."
                )
            active_candidate_ids = {
                int(aid)
                for aid in (evaluation.get("active_aircraft_ids") or [])
                if _to_int(aid) is not None and int(aid) > 0
            }
            for active_aircraft_id in sorted(active_candidate_ids):
                state = agent_state_map.get(int(active_aircraft_id)) or {}
                progress_percent = _to_int(
                    active_progress_by_aircraft.get(active_aircraft_id)
                    if active_aircraft_id in active_progress_by_aircraft
                    else active_progress_by_aircraft.get(str(active_aircraft_id))
                )
                path_all_done = post_attack._active_current_input_path_all_done(
                    source_plan_id=int(current_plan_id),
                    current_input_id=int(current_input_id),
                    aircraft_id=int(active_aircraft_id),
                )
                completed_by_on_mission = False
                if progress_percent is not None and int(progress_percent) >= 100:
                    completed_by_on_mission = post_attack._active_current_input_on_mission_complete(
                        source_plan_id=int(current_plan_id),
                        current_input_id=int(current_input_id),
                        aircraft_id=int(active_aircraft_id),
                        state=state,
                    )
                if (
                    (progress_percent is None or int(progress_percent) < 100)
                    and not path_all_done
                    and not completed_by_on_mission
                ):
                    continue
                if completed_by_on_mission:
                    _emit(
                        "[PRIOR-REJOIN][ACTIVE-DONE] active imaging mission reports onMission=2; "
                        "using completion hold instead of clearing directly "
                        f"(aircraft={active_aircraft_id}, "
                        f"progress={progress_percent if progress_percent is not None else 'n/a'}%)."
                    )
                elif path_all_done and (progress_percent is None or int(progress_percent) < 100):
                    _emit(
                        "[PRIOR-REJOIN][ACTIVE-DONE] active path waypoints already done; "
                        f"using done/follow-up update (aircraft={active_aircraft_id}, "
                        f"progress={progress_percent if progress_percent is not None else 'n/a'}%)."
                    )
                update = post_attack._build_post_attack_active_done_followup_update(
                    source_plan_id=int(current_plan_id),
                    current_input_id=int(current_input_id),
                    aircraft_id=int(active_aircraft_id),
                    hold_seconds=int(active_done_hold_seconds),
                    now_ms=int(now_ms),
                    emit=_emit,
                    log_prefix="[PRIOR-REJOIN][ACTIVE-DONE]",
                )
                if not isinstance(update, dict):
                    active_completed_failed_aircraft_ids.add(int(active_aircraft_id))
                    continue
                new_imp_id = _to_int(update.get("individualMissionPackageID"))
                if new_imp_id is None or new_imp_id <= 0:
                    active_completed_failed_aircraft_ids.add(int(active_aircraft_id))
                    continue
                updated_entry = False
                for entry in new_plan_data.get("aircraftList") or []:
                    if _to_int(entry.get("aircraftID")) == int(active_aircraft_id):
                        entry["individualMissionPackageID"] = int(new_imp_id)
                        updated_entry = True
                        break
                if not updated_entry:
                    active_completed_failed_aircraft_ids.add(int(active_aircraft_id))
                    continue
                active_completed_updates.append(dict(update))
                active_completed_updates[-1]["completedByOnMission2"] = bool(completed_by_on_mission)
                updated_aircraft_ids.add(int(active_aircraft_id))
                generated_imp_ids.add(int(new_imp_id))
                generated_path_ids.update(
                    int(path_id)
                    for path_id in (update.get("generatedPathIDs") or [])
                    if _to_int(path_id) is not None
                )
            if active_completed_updates:
                evaluation["active_completed_updates"] = active_completed_updates
            if active_completed_failed_aircraft_ids:
                evaluation["active_completed_failed_aircraft_ids"] = sorted(
                    int(aid) for aid in active_completed_failed_aircraft_ids
                )
            released_ids = _clear_group_prior_assignments(
                group_assignments,
                skip_reason=str(evaluation.get("skip_reason") or ""),
            )
            cleared_aircraft_ids.update(int(aid) for aid in released_ids)
            continue

        ongoing_unavailable = {
            int(aid)
            for aid in (evaluation.get("ongoing_tracking_aircraft_ids") or [])
            if _to_int(aid) is not None
        }
        available_aircraft_ids = {
            int(aid)
            for aid in (evaluation.get("available_aircraft_ids") or [])
            if _to_int(aid) is not None
        }
        available_state_map = {
            int(aid): dict(agent_state_map.get(int(aid)) or {})
            for aid in available_aircraft_ids
            if int(aid) not in ongoing_unavailable
        }
        for assignment in group_assignments:
            grouped_aircraft_id = _to_int(assignment.get("aircraft_id"))
            if grouped_aircraft_id is None or grouped_aircraft_id in ongoing_unavailable:
                continue
            state = available_state_map.setdefault(int(grouped_aircraft_id), {})
            if not post_attack._normalize_coordinate(state.get("coordinate")):
                fallback_coord = (
                    post_attack._normalize_coordinate(assignment.get("handoff_coordinate"))
                    or post_attack._normalize_coordinate(assignment.get("last_nonzero_coordinate"))
                    or post_attack._normalize_coordinate(assignment.get("original_coordinate"))
                )
                if fallback_coord is not None:
                    state["coordinate"] = fallback_coord

        planning_source_plan_id = _resolve_prior_group_source_plan_id(
            group_assignments,
            fallback_plan_id=int(current_plan_id),
        )
        collab = post_attack._prepare_post_attack_collaborative_update(
            source_plan_id=int(planning_source_plan_id),
            runtime_plan_id=int(current_plan_id),
            current_input_id=int(current_input_id),
            evaluation=evaluation,
            group_assignments=[dict(item) for item in group_assignments],
            unavailable_aircraft_ids={int(aid) for aid in ongoing_unavailable},
            agent_state_map=available_state_map,
            now_ms=int(now_ms),
            emit=_emit,
            log_prefix="[PRIOR-REJOIN][COLLAB]",
            reservation_summaries=collaborative_reservation_summaries,
        )
        if collab is None:
            evaluation["replan_needed"] = False
            evaluation["skip_reason"] = "collaborative_replan_unavailable"
            evaluation["prior_assignment_retained"] = True
            _emit(
                "Collaborative prior rejoin unavailable; prior assignments retained "
                "until a rejoin update succeeds."
            )
            continue

        for grouped_aircraft_id, imp_id in collab.aircraft_imp_ids.items():
            for entry in new_plan_data.get("aircraftList") or []:
                if _to_int(entry.get("aircraftID")) == int(grouped_aircraft_id):
                    entry["individualMissionPackageID"] = int(imp_id)
                    generated_imp_ids.add(int(imp_id))
                    updated_aircraft_ids.add(int(grouped_aircraft_id))
                    break
        generated_path_ids.update(int(path_id) for path_id in collab.generated_path_ids)
        for assignment in group_assignments:
            grouped_aircraft_id = _to_int(assignment.get("aircraft_id"))
            if grouped_aircraft_id is None:
                continue
            clear_prior_assignment(int(grouped_aircraft_id), reason="rejoined")
            cleared_aircraft_ids.add(int(grouped_aircraft_id))

    result_payload["groupSummaries"] = group_summaries
    result_payload["updatedAircraftIDs"] = sorted(updated_aircraft_ids)
    result_payload["clearedPriorAircraftIDs"] = sorted(cleared_aircraft_ids)
    result_payload["collaborativeReservationSummaries"] = collaborative_reservation_summaries
    phase_timer.mark("group_evaluation")

    if not updated_aircraft_ids:
        if cleared_aircraft_ids:
            _emit(
                "skipped: collaborative rejoin update was unnecessary; "
                "kept current prior resume chain and cleared closed prior assignments."
            )
        else:
            _emit("skipped: no prior collaborative rejoin update was necessary.")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={
                "status": "skipped",
                "reason": "rejoin_not_needed",
                "group_evaluations": group_summaries,
                "current_plan_id": int(current_plan_id),
                "cleared_prior_aircraft_ids": sorted(cleared_aircraft_ids),
                "mode": "priorPostRejoin",
            },
            result_payload=result_payload,
        )

    validation_summary = validate_replan_payloads(
        mission_plan=new_plan_data,
        individual_mission_plans=[],
        flight_paths=[],
        scope=f"priorPostRejoin:{new_plan_id}",
        allow_existing_db_artifacts=True,
        log=_emit,
    )
    result_payload["validation"] = validation_summary
    phase_timer.mark("validation")

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{int(new_plan_id)}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    write_json(plan_dest, new_plan_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    phase_timer.mark("mission_plan_write")
    carried_snapshot = mission_area_replan_store.carry_forward_snapshot(
        int(current_plan_id),
        int(new_plan_id),
        reason="prior_post_rejoin",
    )
    if carried_snapshot is not None:
        _emit(
            "carried area remaining snapshot -> "
            f"{carried_snapshot.name} (sourcePlan={current_plan_id}, plan={new_plan_id})"
        )
    _emit(
        "stored prior post-rejoin plan update -> "
        f"plan:{plan_dest.name}, updatedAircraft={sorted(updated_aircraft_ids)}"
    )

    plan_meta_map = dict(ctx.get("_option_meta") or {})
    plan_meta_map[int(new_plan_id)] = {
        "optionName": _PRIOR_POST_REJOIN_OPTION_NAME,
        "replanDetail": dict(detail),
        "priorPostRejoin": True,
        "sourceMissionPlanID": int(current_plan_id),
        "lineage": dict(result_payload.get("lineage") or {}),
        "stateKeyPolicy": dict(result_payload.get("stateKeyPolicy") or {}),
        "noopDeliveryPolicy": dict(result_payload.get("noopDeliveryPolicy") or {}),
        "idReservationPolicy": dict(result_payload.get("idReservationPolicy") or {}),
        "collaborativeReservationSummaries": collaborative_reservation_summaries,
        "updatedAircraftIDs": sorted(updated_aircraft_ids),
        "clearedPriorAircraftIDs": sorted(cleared_aircraft_ids),
        "groupEvaluations": group_summaries,
        "timingMs": phase_timer.snapshot(include_total=False),
        "replanTransactionId": transaction_id,
    }
    return _finish_with_timing(
        requested_plan_id=requested_plan_id,
        status="success",
        summary={
            "status": "success",
            "reason": "prior_post_rejoin_applied",
            "plan_ids": [int(new_plan_id)],
            "option_names": [_PRIOR_POST_REJOIN_OPTION_NAME],
            "current_plan_id": int(current_plan_id),
            "updated_aircraft_ids": sorted(updated_aircraft_ids),
            "cleared_prior_aircraft_ids": sorted(cleared_aircraft_ids),
            "group_evaluations": group_summaries,
            "mode": "priorPostRejoin",
        },
        result_payload=result_payload,
        generated_imp_ids=generated_imp_ids,
        generated_path_ids=generated_path_ids,
        plan_ids=[int(new_plan_id)],
        option_names=[_PRIOR_POST_REJOIN_OPTION_NAME],
        plan_meta_map=plan_meta_map,
    )


def _orientation_altitude(lat: Optional[float], lon: Optional[float], *, fallback: int = 0) -> int:
    if lat is None or lon is None:
        return int(fallback)
    dem_alt = _sample_dem_altitude(float(lat), float(lon))
    dem_alt_int = _normalize_altitude_value(dem_alt)
    if dem_alt_int is None:
        return int(fallback)
    return dem_alt_int


def _apply_runtime_flyover_to_flight_path_payload(payload: Dict[str, Any]) -> Dict[str, Any]:
    waypoint_list = payload.get("waypointList") if isinstance(payload, dict) else None
    if not isinstance(waypoint_list, list) or not waypoint_list:
        return payload
    aircraft_id = _to_int(payload.get("aircraftID") if isinstance(payload, dict) else None)
    if aircraft_id in (4, 5, 6):
        try:
            from modules.mission_planning.pipelines.next_collab_path_builder import (
                _apply_legacy_altitude_profile_to_waypoints,
            )

            _apply_legacy_altitude_profile_to_waypoints(
                waypoint_list,
                aircraft_id=int(aircraft_id),
                mission_info=None,
            )
        except Exception:
            pass
    try:
        from modules.mission_planning.pipelines.next_collab_path_builder import (
            _apply_runtime_flyover_to_waypoints,
        )
        _apply_runtime_flyover_to_waypoints(waypoint_list)
    except Exception:
        return payload
    payload["waypointList"] = waypoint_list
    if "lahWaypointList" in payload:
        payload["lahWaypointList"] = deepcopy(waypoint_list)
    if all(isinstance(wp, dict) and bool(wp.get("isDone")) for wp in waypoint_list):
        try:
            from modules.common.eta import annotate_eta_done_flight_plan

            annotate_eta_done_flight_plan(payload, waypoint_list_keys=("waypointList",))
            if "lahWaypointList" in payload:
                payload["lahWaypointList"] = deepcopy(payload.get("waypointList") or [])
        except Exception:
            pass
    return payload


def _set_flight_path_waypoints_done(payload: Dict[str, Any], is_done: bool) -> None:
    if not isinstance(payload, dict):
        return
    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        waypoints = payload.get(key)
        if not isinstance(waypoints, list):
            continue
        for waypoint in waypoints:
            if isinstance(waypoint, dict):
                waypoint["isDone"] = bool(is_done)


def _project_coordinate(
    coord: Optional[Dict[str, Any]],
    heading_deg: Optional[float],
    distance_m: float,
) -> Optional[Dict[str, float]]:
    if not coord:
        return None
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    heading = heading_deg if heading_deg is not None else 0.0
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    heading_rad = math.radians(heading)
    angular_distance = distance_m / 6_371_000.0
    new_lat = math.asin(
        math.sin(lat_rad) * math.cos(angular_distance)
        + math.cos(lat_rad) * math.sin(angular_distance) * math.cos(heading_rad)
    )
    new_lon = lon_rad + math.atan2(
        math.sin(heading_rad) * math.sin(angular_distance) * math.cos(lat_rad),
        math.cos(angular_distance) - math.sin(lat_rad) * math.sin(new_lat),
    )
    return {
        "latitude": math.degrees(new_lat),
        "longitude": math.degrees(new_lon),
    }


def _bearing_between(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_lambda = math.radians(lon2 - lon1)
    y = math.sin(d_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360.0) % 360.0


@dataclass
class PriorMissionPipelineResult:
    plan_ids: List[int]
    option_names: List[str]
    plan_meta_map: Dict[int, Dict[str, Any]]
    generated_imp_ids: Set[int]
    generated_path_ids: Set[int]
    new_imp_id: int
    new_path_id: int
    new_individual_id: int
    resume_path_id: int
    resume_individual_id: int
    log_path: Path
    removed_waypoint_id: Optional[int]
    inserted_waypoint_id: int
    approach_waypoint_id: int
    target_waypoint_id: int


@dataclass
class PriorPostRejoinPipelineResult:
    plan_ids: List[int]
    option_names: List[str]
    plan_meta_map: Dict[int, Dict[str, Any]]
    generated_imp_ids: Set[int]
    generated_path_ids: Set[int]
    log_path: str
    status: str
    summary: Dict[str, Any]


@dataclass
class AgentSnapshotSummary:
    aircraft_id: int
    latitude: Optional[float]
    longitude: Optional[float]
    altitude: Optional[float]
    current_waypoint_id: Optional[int]
    heading: Optional[float]
    flight_mode: Optional[int]


@dataclass
class PlanMissionArtifacts:
    source_plan_id: int
    aircraft_id: int
    individual_mission_package_id: int
    individual_mission_id: int
    path_id: int
    current_waypoint_id: Optional[int]
    previous_waypoint_id: Optional[int]


@dataclass
class CollaborativeResumeReplanResult:
    current_input_id: int
    unavailable_aircraft_ids: Set[int]
    replacement_aircraft_ids: Set[int]
    aircraft_imp_ids: Dict[int, int]
    generated_path_ids: Set[int]
    finish_eta_s: int
    planner_workflow: str
    planner_result_text: str
    timing_ms: Dict[str, Any] = field(default_factory=dict)
    deferred_write_entries: List[Tuple[Path, Dict[str, Any]]] = field(default_factory=list)


def _selected_prior_waypoint_reservation_count(flight_path: Dict[str, Any]) -> int:
    """Return a safe upper bound for the selected UAV's prior-replan WP IDs.

    The pipeline consumes two IDs for the prior approach/target waypoints. The
    resume splitter can then allocate a temporary replan anchor before it
    assigns fresh IDs to every source waypoint and to that anchor. For a
    source path with N waypoints the worst-case requirement is therefore N+4.
    """

    waypoints = flight_path.get("waypointList") if isinstance(flight_path, dict) else None
    source_waypoint_count = len(waypoints) if isinstance(waypoints, list) else 0
    return int(source_waypoint_count + 4)


def _now_ms_since_2000() -> int:
    return int(time.time() * 1000) - _EPOCH_2000_MS


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _to_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _normalize_altitude_value(value: Any) -> Optional[int]:
    alt = _to_float(value)
    if alt is None:
        return None
    try:
        return int(round(alt))
    except (TypeError, ValueError, OverflowError):
        return None


def _apply_dem_altitude_if_needed(
    coord: Dict[str, Any],
    lat: Optional[float],
    lon: Optional[float],
    emit: Optional[Callable[[str], None]] = None,
    *,
    context: str = "Target coordinate",
) -> Optional[int]:
    current_alt = _normalize_altitude_value(coord.get("altitude"))
    if current_alt is not None:
        coord["altitude"] = current_alt
    if lat is None or lon is None:
        return current_alt
    if current_alt is not None and current_alt != 0:
        return current_alt
    dem_alt = _sample_dem_altitude(lat, lon)
    if dem_alt is None:
        return current_alt
    dem_alt_int = _normalize_altitude_value(dem_alt)
    if dem_alt_int is None:
        return current_alt
    coord["altitude"] = dem_alt_int
    if emit is not None:
        emit(f"[PRIOR][STEP2] {context} altitude resolved via DEM ({dem_alt_int}m).")
    return dem_alt_int


def _ensure_option_names(plan_ids: List[int], option_names: List[str]) -> List[str]:
    if not option_names:
        option_names = ["선행임무 재계획"]
    if len(option_names) == len(plan_ids):
        return option_names
    if len(option_names) < len(plan_ids):
        filler = option_names[-1]
        option_names = option_names + [filler] * (len(plan_ids) - len(option_names))
        return option_names
    return option_names[: len(plan_ids)]


def run_prior_mission_pipeline(
    ctx: Dict[str, Any],
    detail: Dict[str, Any],
    reason: str,
    *,
    log: Callable[[str], None],
) -> Optional[PriorMissionPipelineResult]:
    log_messages: List[str] = []
    transaction_id = new_replan_transaction_id("prior")
    phase_timer = PipelinePhaseTimer(
        pipeline="prior_mission",
        replan_transaction_id=transaction_id,
        emit_events=True,
    )

    def emit(message: str) -> None:
        log_messages.append(message)
        log(message)

    plan_ids_raw = ctx.get("plan_ids") or []
    plan_ids: List[int] = []
    success: bool = False
    error_text: Optional[str] = None
    new_plan_id: Optional[int] = None
    new_imp_id: Optional[int] = None
    done_path_id: Optional[int] = None
    prior_path_id: Optional[int] = None
    resume_path_id: Optional[int] = None
    prior_individual_id: Optional[int] = None
    resume_individual_id: Optional[int] = None
    removed_wp_id: Optional[int] = None
    inserted_wp_id: Optional[int] = None
    prior_approach_wp_id: Optional[int] = None
    prior_target_wp_id: Optional[int] = None
    prior_mission_id: Optional[int] = None
    mission_type: Optional[int] = None
    aircraft_id: Optional[int] = None
    path_id: Optional[int] = None
    source_plan_id: Optional[int] = None
    imp_package_id: Optional[int] = None
    individual_mission_id: Optional[int] = None
    current_waypoint_id: Optional[int] = None
    previous_waypoint_id: Optional[int] = None
    target_coord: Dict[str, Any] = {}
    detail_keys: List[str] = []
    missing_required_fields: List[str] = []
    detail_summary: Dict[str, Any] = {}
    agent_snapshot_payload: Optional[Dict[str, Any]] = None
    agent_summaries: List[AgentSnapshotSummary] = []
    active_prior_aircraft_ids: Set[int] = set()
    selected_agent_summary: Optional[AgentSnapshotSummary] = None
    selected_agent_distance_m: Optional[float] = None
    selected_reservation_summary: Dict[str, Any] = {}
    collaborative_reservation_summaries: List[Dict[str, Any]] = []
    related_mission_policy: Dict[str, Any] = {}
    stored_detail = _load_detail_from_store(plan_ids_raw)
    if not isinstance(detail, dict) or not detail:
        detail = stored_detail or {}
        detail_provided = bool(detail)
        detail_raw = detail
    else:
        detail_provided = True
        detail_raw = dict(detail)
        if stored_detail:
            detail_raw.setdefault("sourceMissionPlanID", stored_detail.get("sourceMissionPlanID"))
            detail_raw.setdefault("targetCoordinate", stored_detail.get("targetCoordinate"))
            detail_raw.setdefault("priorMissionID", stored_detail.get("priorMissionID"))
            detail_raw.setdefault("missionType", stored_detail.get("missionType"))
    detail_raw_preview = _preview_value(detail_raw)

    try:
        detail = dict(detail_raw) if isinstance(detail_raw, dict) else {}
        detail_keys = sorted(detail.keys())
        emit(f"[PRIOR] detail keys: {detail_keys or '∅'}")

        prior_mission_id = _to_int(detail.get("priorMissionID"))
        mission_type = _to_int(detail.get("missionType"))
        target_orientation = detail.get("targetOrientation") or {}
        target_id = _to_int(target_orientation.get("targetID"))
        if target_id is None:
            target_id = _to_int(target_orientation.get("targetId"))
        if target_id is None:
            target_id = _to_int(detail.get("targetID") or detail.get("targetId"))
        aircraft_id = _to_int(detail.get("aircraftID"))
        path_id = _to_int(detail.get("pathID"))
        source_plan_id = _to_int(detail.get("sourceMissionPlanID"))
        imp_package_id = _to_int(detail.get("individualMissionPackageID"))
        individual_mission_id = _to_int(detail.get("individualMissionID"))
        current_waypoint_id = _to_int(detail.get("currentWaypointID"))
        previous_waypoint_id = _to_int(detail.get("previousWaypointID"))
        target_coord = dict(detail.get("targetCoordinate") or {})
        if "altitude" in target_coord:
            target_coord["altitude"] = _normalize_altitude_value(target_coord.get("altitude"))
        coord_source_label = "Target coordinate (0202 payload)"
        detail_summary = _build_detail_summary(detail)

        source_plan_id = _to_int(detail.get("sourceMissionPlanID"))
        if source_plan_id is None:
            source_plan_id = _load_latest_mission_progress_plan_id()
            if source_plan_id is not None:
                emit(f"[PRIOR] sourceMissionPlanID resolved from mission_progress (planID={source_plan_id}).")
        if source_plan_id is None:
            source_plan_id = _scan_latest_source_plan_id()
            if source_plan_id is None:
                emit("[PRIOR] sourceMissionPlanID unavailable and no fallback plan found.")
                return None

        requested_source_plan_id = int(source_plan_id)
        source_plan_id = _rebase_prior_source_plan_to_latest_applied(source_plan_id)
        if source_plan_id is None:
            emit("[PRIOR] sourceMissionPlanID unavailable after applied-plan resolution.")
            return None
        if int(source_plan_id) != requested_source_plan_id:
            detail["sourceMissionPlanID"] = int(source_plan_id)
            detail["currentMissionPlanID"] = int(source_plan_id)
            ctx["sourceMissionPlanID"] = int(source_plan_id)
            ctx["currentMissionPlanID"] = int(source_plan_id)
            ctx_detail = ctx.get("replan_detail")
            if isinstance(ctx_detail, dict):
                ctx_detail["sourceMissionPlanID"] = int(source_plan_id)
                ctx_detail["currentMissionPlanID"] = int(source_plan_id)
            emit(
                "[PRIOR] Queued prior request sourcePlan rebound to latest applied descendant "
                f"({requested_source_plan_id} -> {source_plan_id})."
            )

        active_prior_aircraft_ids = _active_prior_aircraft_ids_for_source_plan(source_plan_id)
        if active_prior_aircraft_ids:
            emit(
                "[PRIOR][STEP1] Active prior-mission UAVs excluded from new assignment "
                f"and collaborative remainder (aircraft={sorted(active_prior_aircraft_ids)})."
            )

        agent_snapshot_payload = agent_status_snapshot.load_agent_status_snapshot()
        agent_summaries = _summarize_agent_states(agent_snapshot_payload)
        _log_step1_agent_snapshot(emit, agent_snapshot_payload, agent_summaries)
        availability_known, available_aircraft_ids = _load_vehicle_status_available_ids()
        unavailable_aircraft_ids: Set[int] = {
            int(summary.aircraft_id)
            for summary in agent_summaries
            if _is_unavailable_agent(
                summary,
                availability_known=availability_known,
                available_aircraft_ids=available_aircraft_ids,
                excluded_aircraft_ids=active_prior_aircraft_ids,
            )
        }
        if availability_known:
            emit(
                "[PRIOR][STEP1] VehicleStatus availability applied "
                f"(available={sorted(int(aid) for aid in available_aircraft_ids)}, "
                f"excluded={sorted(unavailable_aircraft_ids)})."
            )
        else:
            emit("[PRIOR][STEP1] VehicleStatus unavailable; falling back to RTB-only candidate guard.")
        phase_timer.mark("agent_snapshot_load")

        if not plan_ids_raw:
            emit("[PRIOR] plan_ids missing in context.")
            return None
        try:
            plan_ids = [int(v) for v in plan_ids_raw]
        except Exception:
            emit(f"[PRIOR] plan_ids malformed: {plan_ids_raw!r}")
            return None
        if len(plan_ids) != 1:
            emit(f"[PRIOR] Expected exactly one plan option, got {len(plan_ids)}.")
            return None
        option_names = _ensure_option_names(plan_ids, ctx.get("option_names") or [])

        prior_record = None
        if (
            prior_mission_id is None
            or mission_type is None
            or target_id is None
            or target_coord.get("latitude") is None
            or target_coord.get("longitude") is None
        ):
            prior_record = _load_prior_record_from_db(prior_mission_id)
            if prior_record:
                if prior_mission_id is None:
                    prior_mission_id = prior_record.get("priorMissionID")
                if mission_type is None:
                    mission_type = prior_record.get("missionType")
                if target_id is None:
                    target_id = prior_record.get("targetID")
                coord_block = prior_record.get("coordinate")
                if coord_block and (
                    target_coord.get("latitude") is None or target_coord.get("longitude") is None
                ):
                    target_coord["latitude"] = coord_block.get("latitude")
                    target_coord["longitude"] = coord_block.get("longitude")
                    target_coord["altitude"] = _normalize_altitude_value(coord_block.get("altitude"))
                    emit("[PRIOR][STEP2] Target coordinate 보강: PriorMissionInfo 최신 기록에서 좌표 복구.")
                    coord_source_label = "CoordinateOrientation (PriorMissionInfo latest)"

        target_tracking_entry = None
        if mission_type == 2:
            target_tracking_entry = _load_target_tracking_entry(target_id)
            if target_tracking_entry:
                coord_block = target_tracking_entry.get("coordinate") or {}
                if coord_block:
                    target_coord["latitude"] = coord_block.get("latitude")
                    target_coord["longitude"] = coord_block.get("longitude")
                    target_coord["altitude"] = _normalize_altitude_value(coord_block.get("altitude"))
                    coord_source_label = "Target tracking coordinate"
            if target_id is None or int(target_id) <= 0:
                missing_required_fields.append("targetID")
                error_text = "missionType=2 requires a positive targetID"
                emit(f"[PRIOR] {error_text}; prior mission planning aborted.")
                return None
        if target_coord.get("latitude") is None or target_coord.get("longitude") is None:
            fallback_coord = _load_prior_coordinate_from_db(prior_mission_id)
            if fallback_coord:
                target_coord["latitude"] = fallback_coord.get("latitude")
                target_coord["longitude"] = fallback_coord.get("longitude")
                target_coord["altitude"] = _normalize_altitude_value(fallback_coord.get("altitude"))
                emit(
                    f"[PRIOR][STEP2] Target coordinate 보강: PriorMissionInfo/{prior_mission_id}.json에서 좌표 복구."
                )
                coord_source_label = "CoordinateOrientation (PriorMissionInfo fallback)"
        lat = _to_float(target_coord.get("latitude"))
        lon = _to_float(target_coord.get("longitude"))
        _apply_dem_altitude_if_needed(
            target_coord,
            lat,
            lon,
            emit,
            context=coord_source_label,
        )
        _log_step2_target_coordinate(emit, lat, lon, target_coord.get("altitude"))
        if lat is None or lon is None:
            emit("[PRIOR] Target coordinate missing latitude/longitude.")
            return None
        if "altitude" not in target_coord:
            target_coord["altitude"] = None
        phase_timer.mark("target_prior_record_resolve")

        if mission_type == 2:
            if target_tracking_entry:
                watcher_id = _to_int(target_tracking_entry.get("watcherID"))
                if watcher_id is not None:
                    selected_agent_summary = next(
                        (summary for summary in agent_summaries if summary.aircraft_id == watcher_id),
                        None,
                    )
                    selected_agent_distance_m = None
                    if selected_agent_summary:
                        if _is_rtb_agent(selected_agent_summary):
                            emit(
                                f"[PRIOR][STEP3] Target-tracking watcher UAV {watcher_id} is RTB; "
                                "excluding it from prior mission candidate selection."
                            )
                            selected_agent_summary = None
                        elif _is_unavailable_agent(
                            selected_agent_summary,
                            availability_known=availability_known,
                            available_aircraft_ids=available_aircraft_ids,
                            excluded_aircraft_ids=active_prior_aircraft_ids,
                        ):
                            emit(
                                f"[PRIOR][STEP3] Target-tracking watcher UAV {watcher_id} is unavailable "
                                "or already executing another prior mission; "
                                "excluding it from prior mission candidate selection."
                            )
                            selected_agent_summary = None
                        else:
                            emit(
                                f"[PRIOR][STEP3] Target-tracking watcher UAV {watcher_id} selected (targetID={target_id})."
                            )
                else:
                    emit(
                        f"[PRIOR][STEP3] targetID={target_id} found in targetInfo, but watcherID is missing."
                    )
            else:
                emit(
                    f"[PRIOR][STEP3] targetID={target_id} not found in targetInfo. Falling back to nearest UAV."
                )
        if selected_agent_summary is None:
            selected_agent_summary, selected_agent_distance_m = _select_nearest_agent(
                lat,
                lon,
                agent_summaries,
                availability_known=availability_known,
                available_aircraft_ids=available_aircraft_ids,
                excluded_aircraft_ids=active_prior_aircraft_ids,
            )
            _log_step3_nearest_agent(emit, selected_agent_summary, selected_agent_distance_m)

        if selected_agent_summary is None:
            emit("[PRIOR][STEP3] No eligible UAV found after excluding RTB/unavailable aircraft.")
            return None
        phase_timer.mark("nearest_uav_select")

        aircraft_id = selected_agent_summary.aircraft_id
        current_waypoint_id = selected_agent_summary.current_waypoint_id
        artifacts = _resolve_plan_artifacts(
            source_plan_id=source_plan_id,
            aircraft_id=aircraft_id,
            current_waypoint_id=current_waypoint_id,
            emit=emit,
        )
        if artifacts is None:
            emit("[PRIOR] Failed to resolve mission artifacts from MissionPlan/IMP data.")
            return None
        imp_package_id = artifacts.individual_mission_package_id
        individual_mission_id = artifacts.individual_mission_id
        path_id = artifacts.path_id
        current_waypoint_id = artifacts.current_waypoint_id
        previous_waypoint_id = artifacts.previous_waypoint_id

        plan_src = db_paths.get_db_subpath("MissionPlan", f"{source_plan_id}.json")
        imp_src = db_paths.get_db_subpath("IndividualMissionPlan", f"{imp_package_id}.json")
        fp_src = db_paths.get_db_subpath("FlightPath", f"{path_id}.json")

        for label, path in (("MissionPlan", plan_src), ("IndividualMissionPlan", imp_src), ("FlightPath", fp_src)):
            if not path.exists():
                emit(f"[PRIOR] {label} source file missing: {path}")
                return None

        try:
            plan_data = read_json_cached(plan_src, kind="MissionPlan")
            imp_data = read_json_cached(imp_src, kind="IndividualMissionPlan")
            fp_data = read_json_cached(fp_src, kind="FlightPath")
        except Exception as exc:
            emit(f"[PRIOR] Failed to load source artifacts: {exc}")
            return None
        phase_timer.mark("resolve_artifacts")

        new_plan_id = plan_ids[0]
        selected_waypoint_count = _selected_prior_waypoint_reservation_count(fp_data)
        selected_reservation = ReplanIdReservation.reserve(
            imp_count=1,
            individual_count=2,
            path_count_by_aircraft={int(aircraft_id): 3},
            waypoint_count=selected_waypoint_count,
        )
        emit(
            "[PRIOR][ID] Reserved selected-UAV waypoint block "
            f"(sourceWaypoints={max(0, selected_waypoint_count - 4)}, "
            f"reserved={selected_waypoint_count})."
        )
        new_imp_id = selected_reservation.next_imp()
        prior_individual_id = selected_reservation.next_individual()
        resume_individual_id = selected_reservation.next_individual()
        done_path_id = selected_reservation.next_path(int(aircraft_id))
        prior_path_id = selected_reservation.next_path(int(aircraft_id))
        resume_path_id = selected_reservation.next_path(int(aircraft_id))
        prior_approach_wp_id = selected_reservation.next_waypoint()
        prior_target_wp_id = selected_reservation.next_waypoint()
        _log_step4_waypoint_allocation(
            emit,
            prior_target_wp_id,
            selected_agent_summary,
            selected_agent_distance_m,
        )
        emit(
            "[PRIOR] Allocated IDs -> "
            f"plan:{new_plan_id} imp:{new_imp_id} "
            f"path(done/prior/resume):{done_path_id}/{prior_path_id}/{resume_path_id} "
            f"indiv(prior/resume):{prior_individual_id}/{resume_individual_id} "
            f"wp(approach/target):{prior_approach_wp_id}/{prior_target_wp_id}"
        )
        phase_timer.mark("allocate_ids")

        # plan_data/imp_data는 read_json_cached가 반환한 호출자 소유 사본이고 이후 재사용 없음
        new_plan_data = plan_data
        new_imp_data = imp_data
        resume_fp_data = deepcopy(fp_data)
        sweep_progress = load_sweep_progress()

        now_ms = _now_ms_since_2000()
        new_plan_data["missionPlanID"] = new_plan_id
        new_plan_data["timestamp"] = now_ms
        if "missionPlanTimestamp" in new_plan_data:
            new_plan_data["missionPlanTimestamp"] = now_ms
        updated_aircraft = False
        for aircraft_entry in new_plan_data.get("aircraftList", []):
            if _to_int(aircraft_entry.get("aircraftID")) == aircraft_id:
                aircraft_entry["individualMissionPackageID"] = new_imp_id
                updated_aircraft = True
                break
        if not updated_aircraft:
            emit(f"[PRIOR] Aircraft {aircraft_id} not found inside missionPlan {source_plan_id}.")
            return None

        target_index = None
        mission_list = new_imp_data.get("individualMissionList", [])
        for idx, mission in enumerate(mission_list):
            if _to_int(mission.get("individualMissionID")) == individual_mission_id:
                target_index = idx
                break
        if target_index is None:
            emit(f"[PRIOR] Individual mission {individual_mission_id} not found in package {imp_package_id}.")
            return None

        original_mission_template = deepcopy(mission_list[target_index])

        base_rel_block = dict(original_mission_template.get("relatedMission") or {})
        input_mission_id = _to_int(detail.get("inputMissionID"))
        if input_mission_id is None:
            input_mission_id = _to_int(base_rel_block.get("inputMissionID"))

        collaborative_resume: Optional[CollaborativeResumeReplanResult] = None
        preserve_type2_branch = bool(
            input_mission_id is not None
            and input_mission_id > 0
            and _source_input_mission_is_locked_type2_branch(
                int(source_plan_id),
                int(input_mission_id),
            )
        )
        if preserve_type2_branch:
            emit(
                "[PRIOR][TYPE2] selected UAV keeps its branch suffix; "
                "collaborative redistribution skipped."
            )
        if input_mission_id is not None and input_mission_id > 0 and not preserve_type2_branch:
            agent_state_map: Dict[int, Dict[str, Any]] = {}
            for summary in agent_summaries:
                if _is_unavailable_agent(
                    summary,
                    availability_known=availability_known,
                    available_aircraft_ids=available_aircraft_ids,
                    excluded_aircraft_ids=active_prior_aircraft_ids,
                ):
                    continue
                coord: Dict[str, Any] = {}
                if summary.latitude is not None and summary.longitude is not None:
                    coord["latitude"] = summary.latitude
                    coord["longitude"] = summary.longitude
                if summary.altitude is not None:
                    coord["altitude"] = summary.altitude
                agent_state_map[int(summary.aircraft_id)] = {
                    "coordinate": coord,
                    "heading": summary.heading,
                }
            collaborative_resume = _prepare_uav_collaborative_resume_replan(
                source_plan_id=int(source_plan_id),
                current_input_id=int(input_mission_id),
                unavailable_aircraft_ids={int(aircraft_id)}.union(unavailable_aircraft_ids),
                agent_state_map=agent_state_map,
                now_ms=int(now_ms),
                emit=emit,
                log_prefix="[PRIOR][COLLAB]",
                drop_prefix_missions=False,
                reservation_summaries=collaborative_reservation_summaries,
                flight_path_transform=lambda aircraft_id, path_id, payload: (
                    _boost_prior_collab_first_sweep_search_speed(
                        int(aircraft_id),
                        int(path_id),
                        payload,
                        emit=emit,
                    )
                ),
                # Match attack replanning: one remaining UAV executes two
                # sequential area pieces instead of one oversized area task.
                split_single_aircraft_area_into_two=True,
            )
            if collaborative_resume is not None:
                for aid, imp_id in collaborative_resume.aircraft_imp_ids.items():
                    updated = False
                    for aircraft_entry in new_plan_data.get("aircraftList", []):
                        if _to_int(aircraft_entry.get("aircraftID")) == int(aid):
                            aircraft_entry["individualMissionPackageID"] = int(imp_id)
                            updated = True
                            break
                    if not updated:
                        emit(f"[PRIOR][COLLAB] Aircraft {aid} not found in MissionPlan for collaborative update.")
        phase_timer.mark("collaborative_resume_build")

        prior_rel_block = dict(base_rel_block)
        prior_rel_block["priorMissionID"] = prior_mission_id or 0
        prior_rel_block["relatedMissionType"] = 2
        if input_mission_id is not None:
            prior_rel_block["inputMissionID"] = input_mission_id

        resume_rel_block = dict(base_rel_block)
        resume_rel_block["priorMissionID"] = 0
        if input_mission_id is not None and "inputMissionID" not in resume_rel_block:
            resume_rel_block["inputMissionID"] = input_mission_id

        related_mission_policy = {
            "missionType": mission_type,
            "priorMissionID": prior_mission_id or 0,
            "priorRelatedMissionType": 2,
            "resumePriorMissionID": 0,
            "inputMissionID": input_mission_id,
            "targetID": target_id if mission_type == 2 and target_id is not None else 0,
            "autoTracking": bool(mission_type == 2 and target_id is not None),
        }

        prior_mission_entry = deepcopy(original_mission_template)
        prior_mission_entry["individualMissionID"] = prior_individual_id
        prior_mission_entry["pathID"] = prior_path_id
        prior_mission_entry["relatedMission"] = prior_rel_block
        prior_mission_entry["isDone"] = False

        resume_mission_entry = deepcopy(original_mission_template)
        resume_mission_entry["individualMissionID"] = resume_individual_id
        resume_mission_entry["pathID"] = resume_path_id
        resume_mission_entry["relatedMission"] = resume_rel_block
        resume_mission_entry["isDone"] = False

        target_tracking_payload = {"targetID": target_id} if mission_type == 2 and target_id is not None else None

        selected_current_coord: Dict[str, Any] = {}
        if selected_agent_summary.latitude is not None and selected_agent_summary.longitude is not None:
            selected_current_coord["latitude"] = selected_agent_summary.latitude
            selected_current_coord["longitude"] = selected_agent_summary.longitude
        if selected_agent_summary.altitude is not None:
            selected_current_coord["altitude"] = selected_agent_summary.altitude

        done_waypoints, resume_waypoints, removed_wp_id = _apply_resume_path_trimming(
            resume_fp_data,
            artifacts=artifacts,
            sweep_progress=sweep_progress,
            emit=emit,
            current_coord=selected_current_coord,
            waypoint_allocator=selected_reservation.next_waypoint,
        )
        selected_reservation_summary = selected_reservation.summary()
        if not resume_waypoints and collaborative_resume is None:
            emit("[PRIOR] FlightPath trimming produced an empty waypoint list.")
            return None

        has_done_segment = bool(done_waypoints)
        if not has_done_segment:
            done_path_id = None

        preserved_done_entry = None
        done_fp_data = None
        if has_done_segment and done_path_id is not None:
            preserved_done_entry = _build_done_reference_mission(
                original_mission_template,
                path_id=int(done_path_id),
                done_waypoints=done_waypoints,
            )

            done_fp_data = deepcopy(fp_data)
            done_fp_data["pathID"] = done_path_id
            done_fp_data["timestamp"] = now_ms
            done_fp_data["Source"] = done_fp_data.get("Source") or "MMR"
            done_fp_data["aircraftID"] = aircraft_id
            done_fp_data["individualMissionID"] = _to_int(original_mission_template.get("individualMissionID"))
            done_fp_data["waypointList"] = done_waypoints

        resume_fp_data["waypointList"] = resume_waypoints
        resume_fp_data["pathID"] = resume_path_id
        resume_fp_data["timestamp"] = now_ms
        resume_fp_data["Source"] = resume_fp_data.get("Source") or "MMR"
        resume_fp_data["aircraftID"] = aircraft_id
        resume_fp_data["individualMissionID"] = resume_individual_id

        prior_approach_base_offset_m = get_runtime_prior_float("approach_base_offset_m", 250.0)
        prior_approach_far_offset_m = get_runtime_prior_float("approach_far_offset_m", 450.0)
        prior_far_trigger_distance_m = get_runtime_prior_float("approach_far_trigger_distance_m", 400.0)
        prior_orientation_offset_m = get_runtime_prior_float("orientation_offset_m", 100.0)
        prior_approach_speed = get_runtime_prior_float("approach_speed_mps", 40.0)
        prior_target_speed = get_runtime_prior_float("target_speed_mps", 30.0)
        prior_loiter_seconds = (
            get_runtime_prior_int("tracking_loiter_seconds", 300)
            if mission_type == 2
            else get_runtime_prior_int("default_loiter_seconds", 50)
        )

        agent_coord = None
        if (
            selected_agent_summary.latitude is not None
            and selected_agent_summary.longitude is not None
        ):
            agent_coord = {
                "latitude": selected_agent_summary.latitude,
                "longitude": selected_agent_summary.longitude,
            }
        approach_coord = None
        if agent_coord:
            approach_coord = _project_coordinate(
                agent_coord,
                selected_agent_summary.heading,
                prior_approach_base_offset_m,
            )
            if approach_coord is None:
                try:
                    bearing = _bearing_between(
                        agent_coord["latitude"],
                        agent_coord["longitude"],
                        target_coord["latitude"],
                        target_coord["longitude"],
                    )
                    approach_coord = _project_coordinate(
                        agent_coord,
                        bearing,
                        prior_approach_base_offset_m,
                    )
                except Exception:
                    approach_coord = None
        if approach_coord is None:
            approach_coord = {
                "latitude": agent_coord["latitude"] if agent_coord else target_coord["latitude"],
                "longitude": agent_coord["longitude"] if agent_coord else target_coord["longitude"],
            }
        approach_alt = _normalize_altitude_value(selected_agent_summary.altitude)
        if approach_alt is None:
            approach_alt = _normalize_altitude_value(target_coord.get("altitude")) or 700
        approach_coord["altitude"] = approach_alt

        agent_to_target_distance = None
        if (
            agent_coord
            and target_coord.get("latitude") is not None
            and target_coord.get("longitude") is not None
        ):
            try:
                agent_to_target_distance = _haversine_distance(
                    float(agent_coord["latitude"]),
                    float(agent_coord["longitude"]),
                    float(target_coord["latitude"]),
                    float(target_coord["longitude"]),
                )
            except Exception:
                agent_to_target_distance = None

        # Prior mission path now uses only one loiter waypoint (no separate entry waypoint).
        use_single_tracking_wp = True
        if mission_type == 2:
            emit("[PRIOR][STEP3] missionType=2 target tracking -> using single auto-tracking waypoint.")
        elif (
            isinstance(agent_to_target_distance, (int, float))
            and agent_to_target_distance > float(prior_far_trigger_distance_m)
        ):
            try:
                bearing = _bearing_between(
                    agent_coord["latitude"],
                    agent_coord["longitude"],
                    target_coord["latitude"],
                    target_coord["longitude"],
                )
                approach_override = _project_coordinate(
                    agent_coord,
                    bearing,
                    prior_approach_far_offset_m,
                )
                if approach_override:
                    approach_override["altitude"] = approach_alt
                    approach_coord = approach_override
                    emit(
                        "[PRIOR][STEP3] Approach waypoint adjusted: "
                        f"{agent_to_target_distance:.1f}m -> {prior_approach_far_offset_m:.0f}m ahead."
                    )
            except Exception:
                pass

        target_altitude_value = _normalize_altitude_value(target_coord.get("altitude")) or 0
        coord_list = [
            {
                "latitude": target_coord["latitude"],
                "longitude": target_coord["longitude"],
                "altitude": target_altitude_value,
            }
        ]

        prior_mission_entry["individualMissionInfo"] = {
            "individualMissionType": 1 if mission_type == 2 else 5,
            "patternType": 1,
            "autoZoomIn": True,
            "coordinateList": coord_list,
            "lineList": [],
            "areaList": [],
            "targetID": target_id if mission_type == 2 and target_id is not None else 0,
        }

        # 접근 WP 시선 방향: 현재 접근 좌표에서 목표 좌표 방향으로 100m 앞 좌표
        orientation_coord = _project_coordinate(
            approach_coord,
            _bearing_between(
                approach_coord["latitude"],
                approach_coord["longitude"],
                target_coord["latitude"],
                target_coord["longitude"],
            ),
            prior_orientation_offset_m,
        ) or dict(target_coord)
        orientation_altitude = _orientation_altitude(
            orientation_coord.get("latitude"),
            orientation_coord.get("longitude"),
            fallback=_normalize_altitude_value(orientation_coord.get("altitude")) or 0,
        )

        target_altitude = (
            _normalize_altitude_value(approach_coord.get("altitude"))
            or _normalize_altitude_value(target_coord.get("altitude"))
            or 700
        )

        approach_speed = float(prior_approach_speed)
        target_speed = float(prior_target_speed)
        loiter_seconds = int(prior_loiter_seconds)
        prior_profile = get_runtime_prior_mission_profile(
            default_turn_radius_m=400.0,
            default_fov_deg=5.0,
        )
        prior_fov_deg = float(prior_profile.get("fov_deg", 5.0) or 5.0)
        prior_turn_radius_m = float(prior_profile.get("turn_radius_m", 400.0) or 400.0)
        distance_m = None
        if use_single_tracking_wp and isinstance(agent_to_target_distance, (int, float)):
            distance_m = float(agent_to_target_distance)
        elif (
            approach_coord.get("latitude") is not None
            and approach_coord.get("longitude") is not None
            and target_coord.get("latitude") is not None
            and target_coord.get("longitude") is not None
        ):
            try:
                distance_m = _haversine_distance(
                    float(approach_coord["latitude"]),
                    float(approach_coord["longitude"]),
                    float(target_coord["latitude"]),
                    float(target_coord["longitude"]),
                )
            except Exception:
                distance_m = None
        eta_to_target = 0
        if isinstance(distance_m, (int, float)) and target_speed > 0:
            try:
                eta_to_target = int(round(float(distance_m) / float(target_speed)))
            except Exception:
                eta_to_target = 0
        target_eta = max(0, int(eta_to_target) + int(loiter_seconds))

        approach_wp = {
            "waypointID": prior_approach_wp_id,
            "coordinate": {
                "latitude": approach_coord["latitude"],
                "longitude": approach_coord["longitude"],
                "altitude": approach_coord["altitude"],
            },
            "speed": approach_speed,
            "eta": 0,
            "ecf": 0.0,
            "nextWaypointID": prior_target_wp_id,
            "waypointPassType": 1,
            "filmingProperty": {
                "fieldOfView": prior_fov_deg,
                "sensorType": 1,
                "operationMode": 1,
                "coordinateOrientation": {
                    "coordinate": {
                        "latitude": orientation_coord.get("latitude", target_coord["latitude"]),
                        "longitude": orientation_coord.get("longitude", target_coord["longitude"]),
                        "altitude": orientation_altitude,
                    }
                },
            },
            "loiterProperty": {},
            "isDone": False,
        }

        target_wp = {
            "waypointID": prior_target_wp_id,
            "coordinate": {
                "latitude": target_coord["latitude"],
                "longitude": target_coord["longitude"],
                "altitude": target_altitude,
            },
            "speed": target_speed,
            "eta": target_eta,
            "ecf": 0.0,
            "nextWaypointID": 0,
            "waypointPassType": 2,
            "filmingProperty": {
                "fieldOfView": prior_fov_deg,
                "sensorType": 1,
                "operationMode": 3 if mission_type == 2 else 1,
                "coordinateOrientation": {
                    "coordinate": {
                        "latitude": target_coord["latitude"],
                        "longitude": target_coord["longitude"],
                        "altitude": _orientation_altitude(
                            target_coord.get("latitude"),
                            target_coord.get("longitude"),
                            fallback=0,
                        ),
                    }
                },
            },
            "loiterProperty": {
                "radius": int(prior_turn_radius_m),
                "direction": 1,
                "time": loiter_seconds,
                "speed": 30,
            },
            "isDone": False,
        }
        if mission_type == 2 and target_tracking_payload:
            filming = target_wp.get("filmingProperty") or {}
            filming["autoTracking"] = {"targetID": target_tracking_payload.get("targetID")}
            if use_single_tracking_wp and "coordinateOrientation" in filming:
                del filming["coordinateOrientation"]
            target_wp["filmingProperty"] = filming

        prior_fp_data = {
            key: deepcopy(value)
            for key, value in fp_data.items()
            if key not in {"waypointList", "pathID", "timestamp", "individualMissionID"}
        }
        prior_fp_data["pathID"] = prior_path_id
        prior_fp_data["timestamp"] = now_ms
        prior_fp_data["Source"] = fp_data.get("Source") or prior_fp_data.get("Source") or "MMR"
        prior_fp_data["aircraftID"] = aircraft_id
        prior_fp_data["individualMissionID"] = prior_individual_id
        prior_fp_data["isFormationFlight"] = fp_data.get("isFormationFlight", False)
        prior_fp_data["waypointList"] = [target_wp] if use_single_tracking_wp else [approach_wp, target_wp]

        if collaborative_resume is not None:
            release_end_coord = _extract_final_uav_coordinate(fp_data)
            release_start_coord = _normalize_coordinate_dict(target_coord)
            if release_end_coord is None and resume_waypoints:
                release_end_coord = _normalize_coordinate_dict((resume_waypoints[-1] or {}).get("coordinate"))
            if release_start_coord is not None and release_end_coord is not None:
                release_waypoints, release_speed_mps = _build_uav_release_resume_waypoints(
                    start_coord=release_start_coord,
                    end_coord=release_end_coord,
                    release_eta_s=int(target_eta),
                    target_finish_eta_s=int(collaborative_resume.finish_eta_s),
                    force_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
                )
                if release_waypoints:
                    resume_waypoints = release_waypoints
                    _apply_release_resume_mission_info(
                        resume_mission_entry,
                        start_coord=release_start_coord,
                        end_coord=release_end_coord,
                    )
                    emit(
                        "[PRIOR][COLLAB] Selected UAV resume replaced with release transit "
                        f"(aircraft={aircraft_id}, targetFinishEta={collaborative_resume.finish_eta_s}, "
                        f"speed={release_speed_mps:.1f}m/s)."
                    )
                    resume_fp_data["waypointList"] = resume_waypoints

        prefix_missions = mission_list[:target_index]
        suffix_missions = mission_list[target_index + 1 :]
        rebuilt_list = prefix_missions
        if preserved_done_entry is not None:
            rebuilt_list.append(preserved_done_entry)
        rebuilt_list.extend([prior_mission_entry, resume_mission_entry])
        rebuilt_list.extend(suffix_missions)
        mission_list[:] = rebuilt_list
        new_imp_data["individualMissionPackageID"] = new_imp_id

        other_updates: List[Dict[str, Any]] = []
        other_generated_imp_ids: Set[int] = set()
        other_generated_path_ids: Set[int] = set()
        collaborative_generated_imp_ids: Set[int] = (
            set(int(value) for value in collaborative_resume.aircraft_imp_ids.values())
            if collaborative_resume is not None
            else set()
        )
        collaborative_generated_path_ids: Set[int] = (
            set(int(value) for value in collaborative_resume.generated_path_ids)
            if collaborative_resume is not None
            else set()
        )
        other_resume_jobs: List[Dict[str, Any]] = []
        for summary in agent_summaries:
            aid = summary.aircraft_id
            if aid == aircraft_id:
                continue
            if _is_unavailable_agent(
                summary,
                availability_known=availability_known,
                available_aircraft_ids=available_aircraft_ids,
                excluded_aircraft_ids=active_prior_aircraft_ids,
            ):
                continue
            if collaborative_resume is not None and int(aid) in collaborative_resume.replacement_aircraft_ids:
                continue
            current_coord = None
            if summary.latitude is not None and summary.longitude is not None:
                current_coord = {
                    "latitude": summary.latitude,
                    "longitude": summary.longitude,
                }
                if summary.altitude is not None:
                    current_coord["altitude"] = summary.altitude
            other_resume_jobs.append(
                {
                    "aircraft_id": int(aid),
                    "current_waypoint_id": summary.current_waypoint_id,
                    "current_coord": current_coord,
                }
            )

        active_source_cache = get_active_source_artifact_cache()
        reservation_specs: List[Tuple[Dict[str, Any], int]] = []
        for job in other_resume_jobs:
            try:
                if active_source_cache is not None:
                    prepass_artifacts = call_with_source_artifact_cache(
                        active_source_cache,
                        _resolve_plan_artifacts,
                        source_plan_id=int(source_plan_id),
                        aircraft_id=int(job["aircraft_id"]),
                        current_waypoint_id=_to_int(job.get("current_waypoint_id")),
                        emit=lambda _message: None,
                        allow_first_mission_fallback=True,
                    )
                else:
                    prepass_artifacts = _resolve_plan_artifacts(
                        source_plan_id=int(source_plan_id),
                        aircraft_id=int(job["aircraft_id"]),
                        current_waypoint_id=_to_int(job.get("current_waypoint_id")),
                        emit=lambda _message: None,
                        allow_first_mission_fallback=True,
                    )
                if prepass_artifacts is None:
                    continue
                fp_path = db_paths.get_db_subpath(
                    "FlightPath",
                    f"{int(prepass_artifacts.path_id)}.json",
                )
                if active_source_cache is not None:
                    prepass_fp = active_source_cache.read_json(fp_path, kind="FlightPath")
                else:
                    prepass_fp = read_json_cached(fp_path, kind="FlightPath")
                waypoint_count = max(1, len(prepass_fp.get("waypointList") or []) + 2)
            except Exception:
                continue
            reservation_specs.append((job, int(waypoint_count)))

        if reservation_specs:
            parent_reservation = ReplanIdReservation.reserve(
                imp_count=len(reservation_specs),
                individual_count=len(reservation_specs),
                path_count_by_aircraft={
                    int(job["aircraft_id"]): 2
                    for job, _waypoint_count in reservation_specs
                },
                waypoint_count=sum(
                    int(waypoint_count)
                    for _job, waypoint_count in reservation_specs
                ),
            )
            for job, waypoint_count in reservation_specs:
                aid = int(job["aircraft_id"])
                job["id_reservation"] = ReplanIdReservation(
                    imp_ids=ReservedIdBlock(
                        "individualMissionPackage",
                        [parent_reservation.next_imp()],
                    ),
                    individual_ids=ReservedIdBlock(
                        "individualMission",
                        [parent_reservation.next_individual()],
                    ),
                    waypoint_ids=ReservedIdBlock(
                        "waypoint",
                        [
                            parent_reservation.next_waypoint()
                            for _ in range(int(waypoint_count))
                        ],
                    ),
                    path_ids_by_aircraft={
                        aid: ReservedIdBlock(
                            f"pathID[{aid}]",
                            [
                                parent_reservation.next_path(aid),
                                parent_reservation.next_path(aid),
                            ],
                        )
                    },
                )

        def _run_other_resume_job(job: Dict[str, Any]) -> Optional[Dict[str, Any]]:
            kwargs = {
                "source_plan_id": int(source_plan_id),
                "aircraft_id": int(job["aircraft_id"]),
                "current_waypoint_id": _to_int(job.get("current_waypoint_id")),
                "current_coord": job.get("current_coord"),
                "emit": emit,
                "now_ms": int(now_ms),
                "sweep_progress": sweep_progress,
                "id_reservation": job.get("id_reservation"),
            }
            if active_source_cache is not None:
                return call_with_source_artifact_cache(
                    active_source_cache,
                    _build_other_uav_resume_package,
                    **kwargs,
                )
            return _build_other_uav_resume_package(**kwargs)

        other_resume_results: Dict[int, Optional[Dict[str, Any]]] = {}
        if len(other_resume_jobs) > 1:
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=min(3, len(other_resume_jobs)),
                thread_name_prefix="PriorOtherUAV",
            ) as executor:
                futures = {
                    int(job["aircraft_id"]): executor.submit(_run_other_resume_job, job)
                    for job in other_resume_jobs
                }
                for job in other_resume_jobs:
                    aid = int(job["aircraft_id"])
                    other_resume_results[aid] = futures[aid].result()
        else:
            for job in other_resume_jobs:
                aid = int(job["aircraft_id"])
                other_resume_results[aid] = _run_other_resume_job(job)

        for job in other_resume_jobs:
            aid = int(job["aircraft_id"])
            update = other_resume_results.get(aid)
            if not update:
                continue
            other_updates.append(update)
            other_generated_imp_ids.add(int(update["individualMissionPackageID"]))
            resume_meta = update.get("resume") or {}
            if "pathID" in resume_meta:
                try:
                    other_generated_path_ids.add(int(resume_meta["pathID"]))
                except Exception:
                    pass
            done_path_value = _to_int(update.get("donePathID"))
            if done_path_value is not None:
                other_generated_path_ids.add(done_path_value)
            updated = False
            for aircraft_entry in new_plan_data.get("aircraftList", []):
                if _to_int(aircraft_entry.get("aircraftID")) == aid:
                    aircraft_entry["individualMissionPackageID"] = int(update["individualMissionPackageID"])
                    updated = True
                    break
            if not updated:
                emit(f"[PRIOR][UAV] Aircraft {aid} not found in MissionPlan for resume update.")
        phase_timer.mark("other_uav_resume_package")

        inserted_wp = target_wp
        inserted_wp_id = prior_target_wp_id
        phase_timer.mark("build_artifacts")

        plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
        imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
        done_fp_dest = (
            db_paths.get_db_subpath("FlightPath", f"{done_path_id}.json")
            if (done_path_id is not None and done_fp_data is not None)
            else None
        )
        prior_fp_dest = db_paths.get_db_subpath("FlightPath", f"{prior_path_id}.json")
        resume_fp_dest = db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json")
        prior_mission_entry["isDone"] = False
        resume_mission_entry["isDone"] = False
        _set_flight_path_waypoints_done(prior_fp_data, False)
        _set_flight_path_waypoints_done(resume_fp_data, False)
        flight_path_payloads: List[Dict[str, Any]] = []
        write_entries: List[tuple[Path, Dict[str, Any]]] = [(plan_dest, new_plan_data), (imp_dest, new_imp_data)]
        if done_fp_dest is not None and done_fp_data is not None:
            _apply_runtime_flyover_to_flight_path_payload(done_fp_data)
            sanitize_flight_path_payload_filming_altitudes(done_fp_data)
            flight_path_payloads.append(done_fp_data)
            write_entries.append((done_fp_dest, done_fp_data))
        _apply_runtime_flyover_to_flight_path_payload(prior_fp_data)
        _set_flight_path_waypoints_done(prior_fp_data, False)
        sanitize_flight_path_payload_filming_altitudes(prior_fp_data)
        flight_path_payloads.append(prior_fp_data)
        write_entries.append((prior_fp_dest, prior_fp_data))
        _apply_runtime_flyover_to_flight_path_payload(resume_fp_data)
        _set_flight_path_waypoints_done(resume_fp_data, False)
        sanitize_flight_path_payload_filming_altitudes(resume_fp_data)
        flight_path_payloads.append(resume_fp_data)
        write_entries.append((resume_fp_dest, resume_fp_data))

        validation_summary = validate_replan_payloads(
            mission_plan=new_plan_data,
            individual_mission_plans=[new_imp_data],
            flight_paths=flight_path_payloads,
            scope=f"priorMission:{new_plan_id}",
            allow_existing_db_artifacts=True,
            log=emit,
        )
        phase_timer.mark("validation")

        write_results = write_json_batch(
            write_entries,
            pretty=True,
            ensure_ascii=False,
            skip_if_unchanged=True,
            log=emit,
        )
        carried_snapshot = mission_area_replan_store.carry_forward_snapshot(
            int(source_plan_id),
            int(new_plan_id),
            reason="prior_mission_replan",
        )
        if carried_snapshot is not None:
            emit(
                "[PRIOR] carried area remaining snapshot -> "
                f"{carried_snapshot.name} (sourcePlan={source_plan_id}, plan={new_plan_id})"
            )
        written_fp_names = [prior_fp_dest.name, resume_fp_dest.name]
        if done_fp_dest is not None:
            written_fp_names.insert(0, done_fp_dest.name)
        write_count = sum(1 for row in write_results if row.get("written"))
        emit(
            "[PRIOR] Stored new artifacts -> "
            f"plan:{plan_dest.name}, imp:{imp_dest.name}, fp:{'/'.join(written_fp_names)} "
            f"(written={write_count}/{len(write_results)})"
        )
        phase_timer.mark("write_artifacts")
        phase_timings_ms = phase_timer.snapshot()
        emit(f"[PRIOR][TIME] timingMs={phase_timings_ms}")
        for fov_adjust_message in pop_runtime_camera_fov_adjustment_logs():
            emit(str(fov_adjust_message))

        log_dir = db_paths.get_db_subpath("DSS_Internal")
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"PriorMission_{prior_mission_id or 0}_{now_ms}.json"
        log_payload = {
            "timestamp": now_ms,
            "reason": reason,
            "priorMissionID": prior_mission_id,
            "missionType": mission_type,
            "targetID": target_id,
            "targetCoordinate": target_coord,
            "selectedAircraftID": aircraft_id,
            "activePriorAircraftIDs": sorted(int(aid) for aid in active_prior_aircraft_ids),
            "sourceMissionPlanID": source_plan_id,
            "sourceIndividualMissionPackageID": imp_package_id,
            "sourcePathID": path_id,
            "currentWaypointID": current_waypoint_id,
            "removedWaypointID": removed_wp_id,
            "insertedWaypoint": inserted_wp,
            "telemetrySnapshot": detail.get("telemetrySnapshot"),
            "generatedMissionPlanID": new_plan_id,
            "generatedIndividualMissionPackageID": new_imp_id,
            "generatedPriorIndividualMissionID": prior_individual_id,
            "generatedResumeIndividualMissionID": resume_individual_id,
            "generatedDonePathID": done_path_id,
            "generatedPriorPathID": prior_path_id,
            "generatedResumePathID": resume_path_id,
            "priorApproachWaypointID": prior_approach_wp_id,
            "priorTargetWaypointID": prior_target_wp_id,
            "reservedIds": selected_reservation_summary,
            "relatedMissionPolicy": related_mission_policy,
            "otherAircraftUpdates": other_updates,
            "otherReservationSummaries": [
                {
                    "aircraftID": int(update.get("aircraft_id")),
                    "reservedIds": update.get("reservedIds") or {},
                }
                for update in other_updates
                if update.get("aircraft_id") is not None
            ],
            "collaborativeReservationSummaries": collaborative_reservation_summaries,
            "replanTransactionId": transaction_id,
            "writeResults": write_results,
            "validation": validation_summary,
            "timingMs": phase_timings_ms,
            "logMessages": log_messages,
            "logArtifactMode": debug_artifact_mode(),
        }
        log_payload["logArtifactWritten"] = write_debug_json(
            log_path,
            log_payload,
            pretty=True,
            ensure_ascii=False,
            skip_if_unchanged=False,
        )
        if log_payload["logArtifactWritten"]:
            emit(f"[PRIOR] Log captured -> {log_path}")
        else:
            emit("[PRIOR] Log artifact skipped by runtime artifact mode.")

        plan_meta_map = dict(ctx.get("_option_meta") or {})
        plan_meta_entry = plan_meta_map.setdefault(new_plan_id, {})
        plan_meta_entry.update(
            {
                "priorMissionID": prior_mission_id,
                "missionType": mission_type,
                "targetID": target_id,
                "sourceMissionPlanID": source_plan_id,
                "individualMissionPackageID": new_imp_id,
                "individualMissionID": prior_individual_id,
                "pathID": prior_path_id,
                "logPath": str(log_path),
                "logArtifactMode": debug_artifact_mode(),
                "logArtifactWritten": bool(log_payload.get("logArtifactWritten")),
                "removedWaypointID": removed_wp_id,
                "insertedWaypointID": prior_target_wp_id,
                "approachWaypointID": prior_approach_wp_id,
                "resumeIndividualMissionID": resume_individual_id,
                "resumePathID": resume_path_id,
                "targetCoordinate": target_coord,
                "reservedIds": selected_reservation_summary,
                "relatedMissionPolicy": related_mission_policy,
                "otherReservationSummaries": [
                    {
                        "aircraftID": int(update.get("aircraft_id")),
                        "reservedIds": update.get("reservedIds") or {},
                    }
                    for update in other_updates
                    if update.get("aircraft_id") is not None
                ],
                "collaborativeReservationSummaries": collaborative_reservation_summaries,
                "replanTransactionId": transaction_id,
                "writeResults": write_results,
                "validation": validation_summary,
                "timingMs": phase_timings_ms,
            }
        )
        if done_path_id is not None:
            plan_meta_entry["donePathID"] = done_path_id
        if collaborative_resume is not None:
            plan_meta_entry["collaborativeRemainingReplan"] = {
                "currentInputMissionID": int(collaborative_resume.current_input_id),
                "replacementAircraftIDs": sorted(int(aid) for aid in collaborative_resume.replacement_aircraft_ids),
                "finishEtaS": int(collaborative_resume.finish_eta_s),
                "plannerWorkflow": str(collaborative_resume.planner_workflow or ""),
            }

        prior_waypoint_ids = [
            int(wp_id)
            for wp_id in (
                _to_int((waypoint or {}).get("waypointID"))
                for waypoint in (prior_fp_data.get("waypointList") or [])
            )
            if wp_id is not None and int(wp_id) > 0
        ]
        resume_first_waypoint_id = None
        resume_first_active_waypoint_id = None
        resume_waypoint_ids: list[int] = []
        for waypoint in (resume_fp_data.get("waypointList") or []):
            waypoint_id = _to_int((waypoint or {}).get("waypointID"))
            if waypoint_id is not None and waypoint_id > 0:
                resume_waypoint_ids.append(int(waypoint_id))
            if resume_first_waypoint_id is None and waypoint_id is not None and waypoint_id > 0:
                resume_first_waypoint_id = waypoint_id
            if (
                resume_first_active_waypoint_id is None
                and waypoint_id is not None
                and waypoint_id > 0
                and not bool((waypoint or {}).get("isDone"))
            ):
                resume_first_active_waypoint_id = int(waypoint_id)
        if resume_first_active_waypoint_id is None:
            resume_first_active_waypoint_id = resume_first_waypoint_id
        if aircraft_id is not None and source_plan_id is not None:
            register_prior_assignment(
                aircraft_id=int(aircraft_id),
                source_plan_id=int(source_plan_id),
                prior_plan_id=int(new_plan_id),
                current_input_mission_id=input_mission_id,
                original_path_id=int(path_id),
                original_individual_mission_id=int(individual_mission_id),
                original_current_waypoint_id=current_waypoint_id,
                original_coordinate=selected_current_coord,
                prior_path_id=prior_path_id,
                prior_individual_mission_id=prior_individual_id,
                prior_waypoint_ids=prior_waypoint_ids,
                resume_path_id=resume_path_id,
                resume_individual_mission_id=resume_individual_id,
                resume_first_waypoint_id=resume_first_waypoint_id,
                resume_first_active_waypoint_id=resume_first_active_waypoint_id,
                resume_waypoint_ids=resume_waypoint_ids,
                prior_mission_id=prior_mission_id,
                mission_type=mission_type,
                target_id=target_id,
                target_coordinate=target_coord,
            )

        success = True
        generated_path_ids: Set[int] = (
            {prior_path_id, resume_path_id}
            .union(other_generated_path_ids)
            .union(collaborative_generated_path_ids)
        )
        if done_path_id is not None:
            generated_path_ids.add(done_path_id)
        result = PriorMissionPipelineResult(
            plan_ids=plan_ids,
            option_names=option_names,
            plan_meta_map=plan_meta_map,
            generated_imp_ids={new_imp_id}.union(other_generated_imp_ids).union(collaborative_generated_imp_ids),
            generated_path_ids=generated_path_ids,
            new_imp_id=new_imp_id,
            new_path_id=prior_path_id,
            new_individual_id=prior_individual_id,
            resume_path_id=resume_path_id,
            resume_individual_id=resume_individual_id,
            approach_waypoint_id=prior_approach_wp_id,
            target_waypoint_id=prior_target_wp_id,
            log_path=log_path,
            removed_waypoint_id=removed_wp_id,
            inserted_waypoint_id=prior_target_wp_id,
        )
        return result
    except Exception as exc:
        emit(f"[PRIOR] Unexpected failure: {exc}")
        error_text = str(exc)
        return None
    finally:
        final_timings_ms = phase_timer.snapshot()
        for fov_adjust_message in pop_runtime_camera_fov_adjustment_logs():
            emit(str(fov_adjust_message))
        entry = {
            "timestamp": _now_ms_since_2000(),
            "status": "success" if success else "error",
            "reason": reason,
            "priorMissionID": prior_mission_id,
            "missionType": mission_type,
            "aircraftID": aircraft_id,
            "sourceMissionPlanID": source_plan_id,
            "planIDs": plan_ids,
            "newMissionPlanID": new_plan_id,
            "newIndividualMissionPackageID": new_imp_id,
            "priorIndividualMissionID": prior_individual_id,
            "resumeIndividualMissionID": resume_individual_id,
            "donePathID": done_path_id,
            "priorPathID": prior_path_id,
            "resumePathID": resume_path_id,
            "removedWaypointID": removed_wp_id,
            "insertedWaypointID": inserted_wp_id,
            "approachWaypointID": prior_approach_wp_id,
            "targetWaypointID": prior_target_wp_id,
            "targetCoordinate": target_coord,
            "reservedIds": selected_reservation_summary,
            "relatedMissionPolicy": related_mission_policy,
            "collaborativeReservationSummaries": collaborative_reservation_summaries,
            "logMessages": list(log_messages),
            "detailSummary": detail_summary if detail_provided else {},
            "detailProvided": detail_provided,
            "detailKeys": detail_keys if detail_provided else [],
            "detailRawPreview": detail_raw_preview,
            "missingDetailFields": missing_required_fields,
            "snapshotAgentCount": len(agent_summaries),
            "selectedAgentID": selected_agent_summary.aircraft_id if selected_agent_summary else None,
            "selectedAgentDistanceMeters": selected_agent_distance_m,
            "error": error_text,
            "replanTransactionId": transaction_id,
            "timingMs": final_timings_ms,
        }
        _persist_prior_algorithm_log(entry)


def _log_step1_agent_snapshot(
    emit: Callable[[str], None],
    snapshot_payload: Optional[Dict[str, Any]],
    summaries: List[AgentSnapshotSummary],
) -> None:
    if not summaries:
        emit(
            f"[PRIOR][STEP1] 0401 snapshot unavailable or empty — expected file '{agent_status_snapshot.SNAPSHOT_FILENAME}'."
        )
        return
    saved_at = (snapshot_payload or {}).get("saved_at")
    emit(
        f"[PRIOR][STEP1] Loaded 0401 snapshot with {len(summaries)} unmanned entries (saved_at={saved_at})."
    )
    for summary in summaries:
        emit(
            "[PRIOR][STEP1] UAV "
            f"{summary.aircraft_id}: currentWP={summary.current_waypoint_id}, "
            f"coord={_format_coord(summary.latitude, summary.longitude, summary.altitude)}"
        )


def _log_step2_target_coordinate(
    emit: Callable[[str], None],
    lat: Optional[float],
    lon: Optional[float],
    alt: Optional[float],
) -> None:
    if lat is None or lon is None:
        emit("[PRIOR][STEP2] Target coordinate (0202) missing latitude/longitude.")
        return
    emit(
        f"[PRIOR][STEP2] Target coordinate (0202) → {_format_coord(lat, lon, alt)}"
    )


def _log_step3_nearest_agent(
    emit: Callable[[str], None],
    agent: Optional[AgentSnapshotSummary],
    distance_m: Optional[float],
) -> None:
    if agent is None:
        emit("[PRIOR][STEP3] Unable to determine nearest UAV (no valid agent coordinates).")
        return
    distance_text = f"{distance_m:.1f} m" if isinstance(distance_m, (int, float)) else "n/a"
    emit(
        "[PRIOR][STEP3] Nearest UAV "
        f"{agent.aircraft_id} selected based on 0401 snapshot "
        f"(currentWP={agent.current_waypoint_id}, distance≈{distance_text})."
    )


def _log_step4_waypoint_allocation(
    emit: Callable[[str], None],
    waypoint_id: int,
    agent: Optional[AgentSnapshotSummary],
    distance_m: Optional[float],
) -> None:
    if agent is None:
        emit(f"[PRIOR][STEP4] Reserved new waypoint ID {waypoint_id} (no UAV context available).")
        return
    distance_text = f"{distance_m:.1f} m" if isinstance(distance_m, (int, float)) else "n/a"
    emit(
        "[PRIOR][STEP4] Reserved new waypoint ID "
        f"{waypoint_id} for UAV {agent.aircraft_id} "
        f"(currentWP={agent.current_waypoint_id}, distance≈{distance_text})."
    )


def _summarize_agent_states(
    snapshot_payload: Optional[Dict[str, Any]]
) -> List[AgentSnapshotSummary]:
    if not snapshot_payload:
        return []
    states = snapshot_payload.get("agent_states") or snapshot_payload.get("raw", {}).get("agentStateList") or []
    summaries: List[AgentSnapshotSummary] = []
    for entry in states:
        if not isinstance(entry, dict):
            continue
        if not bool(entry.get("isUnmanned")):
            continue
        aircraft_id = _to_int(entry.get("aircraftID"))
        if aircraft_id is None:
            continue
        coord = entry.get("coordinate") or {}
        if not coord:
            unmanned_info = entry.get("unmannedInfo") or {}
            coord = unmanned_info.get("coordinate") or coord
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        alt = _to_float(coord.get("altitude"))
        wp_block = entry.get("currentWaypointID") or {}
        if not wp_block:
            unmanned_info = entry.get("unmannedInfo") or {}
            wp_block = unmanned_info.get("currentWaypointID") or {}
        current_wp = _to_int(wp_block.get("waypointID"))
        flight_mode = None
        raw_flight_mode = entry.get("flightMode")
        if isinstance(raw_flight_mode, dict):
            flight_mode = _to_int(
                raw_flight_mode.get("flightMode")
                or raw_flight_mode.get("FlightMode")
            )
        else:
            flight_mode = _to_int(raw_flight_mode)
        if flight_mode is None:
            unmanned_info = entry.get("unmannedInfo") or {}
            raw_flight_mode = unmanned_info.get("flightMode") or unmanned_info.get("FlightMode")
            if isinstance(raw_flight_mode, dict):
                flight_mode = _to_int(
                    raw_flight_mode.get("flightMode")
                    or raw_flight_mode.get("FlightMode")
                )
            else:
                flight_mode = _to_int(raw_flight_mode)
        heading = None
        velocity = entry.get("velocity") or {}
        if velocity:
            heading = _to_float(velocity.get("heading"))
        if heading is None:
            heading = _to_float(entry.get("heading"))
        summaries.append(
            AgentSnapshotSummary(
                aircraft_id=aircraft_id,
                latitude=lat,
                longitude=lon,
                altitude=alt,
                current_waypoint_id=current_wp,
                heading=heading,
                flight_mode=flight_mode,
            )
        )
    return summaries


def _is_rtb_agent(summary: Optional[AgentSnapshotSummary]) -> bool:
    if summary is None:
        return False
    return _to_int(summary.flight_mode) == _RTB_FLIGHT_MODE


def _load_vehicle_status_available_ids() -> Tuple[bool, Set[int]]:
    try:
        status_path = db_paths.get_db_subpath("VehicleStatus", "status.json")
    except Exception:
        return False, set()
    if not status_path.exists():
        return False, set()
    try:
        payload = json.loads(status_path.read_text(encoding="utf-8"))
    except Exception:
        return False, set()
    raw_available = payload.get("available")
    if not isinstance(raw_available, list):
        return False, set()
    available_ids: Set[int] = set()
    for item in raw_available:
        value = _to_int(item)
        if value is None or value <= 0:
            continue
        available_ids.add(int(value))
    return True, available_ids


def _is_unavailable_agent(
    summary: Optional[AgentSnapshotSummary],
    *,
    availability_known: bool = False,
    available_aircraft_ids: Optional[Set[int]] = None,
    excluded_aircraft_ids: Optional[Set[int]] = None,
) -> bool:
    if summary is None:
        return False
    if int(summary.aircraft_id) in set(excluded_aircraft_ids or set()):
        return True
    if _is_rtb_agent(summary):
        return True
    if availability_known and int(summary.aircraft_id) not in set(available_aircraft_ids or set()):
        return True
    return False


def _select_nearest_agent(
    target_lat: Optional[float],
    target_lon: Optional[float],
    summaries: List[AgentSnapshotSummary],
    *,
    availability_known: bool = False,
    available_aircraft_ids: Optional[Set[int]] = None,
    excluded_aircraft_ids: Optional[Set[int]] = None,
) -> Tuple[Optional[AgentSnapshotSummary], Optional[float]]:
    if target_lat is None or target_lon is None:
        return None, None
    best_agent: Optional[AgentSnapshotSummary] = None
    best_distance: Optional[float] = None
    for summary in summaries:
        if _is_unavailable_agent(
            summary,
            availability_known=availability_known,
            available_aircraft_ids=available_aircraft_ids,
            excluded_aircraft_ids=excluded_aircraft_ids,
        ):
            continue
        if summary.latitude is None or summary.longitude is None:
            continue
        distance = _haversine_distance(
            target_lat, target_lon, summary.latitude, summary.longitude
        )
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_agent = summary
    return best_agent, best_distance


def _format_coord(
    lat: Optional[float],
    lon: Optional[float],
    alt: Optional[float],
) -> str:
    if lat is None or lon is None:
        return "lat/lon unavailable"
    if alt is None:
        return f"({lat:.6f}, {lon:.6f})"
    return f"({lat:.6f}, {lon:.6f}, alt={alt:.1f})"


def _haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6_371_000  # Earth radius in meters
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return r * c


def _inject_prior_waypoint(
    flight_path: Dict[str, Any],
    current_waypoint_id: int,
    previous_waypoint_id: Optional[int],
    target_coord: Dict[str, Any],
    new_waypoint_id: int,
    *,
    mission_type: Optional[int] = None,
    target_tracking: Optional[Dict[str, Any]] = None,
) -> Tuple[Optional[int], Dict[str, Any]]:
    waypoint_list = list(flight_path.get("waypointList") or [])
    current_index = None
    for idx, waypoint in enumerate(waypoint_list):
        if _to_int(waypoint.get("waypointID")) == current_waypoint_id:
            current_index = idx
            break
    if current_index is None:
        raise ValueError(f"Current waypoint {current_waypoint_id} not found in flight path.")

    removed_waypoint_id = None
    inherited_altitude: Optional[int] = None
    if current_index > 0:
        completed_segment = waypoint_list[:current_index]
        waypoint_list = waypoint_list[current_index:]
        current_index = 0
        last_completed = completed_segment[-1]
        removed_waypoint_id = _to_int(last_completed.get("waypointID"))
        inherited_altitude = _normalize_altitude_value(
            (last_completed.get("coordinate") or {}).get("altitude")
        )
    elif previous_waypoint_id:
        # ensure previous pointer is cleared when explicit ID provided
        removed_waypoint_id = previous_waypoint_id

    preceding_index = current_index - 1
    altitude = _normalize_altitude_value(target_coord.get("altitude"))
    if altitude is None and inherited_altitude is not None:
        altitude = inherited_altitude
    if altitude is None:
        altitude = 700
    prior_profile = get_runtime_prior_mission_profile(
        default_turn_radius_m=400.0,
        default_fov_deg=5.0,
    )
    prior_fov_deg = float(prior_profile.get("fov_deg", 5.0) or 5.0)
    prior_turn_radius_m = float(prior_profile.get("turn_radius_m", 400.0) or 400.0)

    inserted_wp = {
        "waypointID": new_waypoint_id,
        "coordinate": {
            "latitude": target_coord["latitude"],
            "longitude": target_coord["longitude"],
            "altitude": altitude,
        },
        "speed": get_runtime_prior_float("target_speed_mps", 30.0),
        "eta": 700,
        "ecf": 0.0,
        "nextWaypointID": current_waypoint_id,
        "waypointPassType": 2,
        "filmingProperty": {
            "fieldOfView": prior_fov_deg,
            "sensorType": 1,
            "operationMode": 1,
            "coordinateOrientation": {
                    "coordinate": {
                        "latitude": target_coord["latitude"],
                        "longitude": target_coord["longitude"],
                        "altitude": target_coord["altitude"]
                        if target_coord.get("altitude") is not None
                        else 0,
                    }
                },
        },
        "loiterProperty": {
            "radius": int(prior_turn_radius_m),
            "direction": 1,
            "time": get_runtime_prior_int("reinsert_loiter_seconds", 100),
            "speed": int(get_runtime_prior_float("target_speed_mps", 30.0)),
        },
    }

    if mission_type == 2:
        filming = inserted_wp.get("filmingProperty") or {}
        filming["operationMode"] = 3
        inserted_wp["filmingProperty"] = filming
        target_track_id = _to_int((target_tracking or {}).get("targetID"))
        if target_track_id is not None:
            filming["autoTracking"] = {"targetID": target_track_id}
            inserted_wp["filmingProperty"] = filming

    waypoint_list.insert(current_index, inserted_wp)
    if preceding_index >= 0:
        waypoint_list[preceding_index]["nextWaypointID"] = new_waypoint_id
    flight_path["waypointList"] = waypoint_list
    return removed_waypoint_id, inserted_wp


def _trim_completed_waypoints(
    flight_path: Dict[str, Any],
    *,
    current_waypoint_id: Optional[int],
    previous_waypoint_id: Optional[int],
) -> Optional[int]:
    if current_waypoint_id is None:
        return previous_waypoint_id
    waypoint_list = list(flight_path.get("waypointList") or [])
    if not waypoint_list:
        return previous_waypoint_id
    current_index = None
    for idx, waypoint in enumerate(waypoint_list):
        if _to_int(waypoint.get("waypointID")) == current_waypoint_id:
            current_index = idx
            break
    if current_index is None:
        return previous_waypoint_id
    removed_waypoint_id = None
    if current_index > 0:
        completed_segment = waypoint_list[:current_index]
        waypoint_list = waypoint_list[current_index:]
        last_completed = completed_segment[-1]
        removed_waypoint_id = _to_int(last_completed.get("waypointID"))
    elif previous_waypoint_id:
        removed_waypoint_id = previous_waypoint_id
    flight_path["waypointList"] = waypoint_list
    return removed_waypoint_id


def _apply_resume_capture_buffer(
    resume_waypoints: List[Dict[str, Any]],
    *,
    emit: Callable[[str], None],
) -> None:
    return


def _apply_resume_path_trimming(
    resume_fp_data: Dict[str, Any],
    *,
    artifacts: PlanMissionArtifacts,
    sweep_progress: Dict[int, Dict[str, Any]] | None,
    emit: Callable[[str], None],
    current_coord: Optional[Dict[str, Any]] = None,
    log_prefix: str = "[PRIOR][UAV]",
    waypoint_allocator: Optional[Callable[[], int]] = None,
    timing: Optional[Dict[str, Any]] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], Optional[int]]:
    trim_started_total = time.perf_counter()
    waypoints = list(resume_fp_data.get("waypointList") or [])
    done_waypoints: List[Dict[str, Any]] = []
    resume_waypoints: List[Dict[str, Any]] = []
    removed_wp_id: Optional[int] = None

    def _record_trim_stage(name: str, started_at: float, **extra: Any) -> None:
        if timing is None:
            return
        row: Dict[str, Any] = {"elapsedMs": round((time.perf_counter() - started_at) * 1000.0, 3)}
        if extra:
            row.update(extra)
        timing[str(name)] = row

    split_started = time.perf_counter()
    curr_wp = _to_int(artifacts.current_waypoint_id)
    prev_wp = _to_int(artifacts.previous_waypoint_id)
    curr_idx = next(
        (idx for idx, wp in enumerate(waypoints) if _to_int(wp.get("waypointID")) == curr_wp),
        None,
    )

    if curr_idx is not None:
        done_waypoints = deepcopy(waypoints[:curr_idx]) if curr_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[curr_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_wp_id = prev_wp
        if removed_wp_id is not None:
            emit(f"{log_prefix} Resume trimmed by currentWP (lastRemovedWP={removed_wp_id}).")
    elif any(bool(wp.get("isDone")) for wp in waypoints):
        idx = 0
        while idx < len(waypoints) and bool(waypoints[idx].get("isDone")):
            idx += 1
        done_waypoints = deepcopy(waypoints[:idx]) if idx > 0 else []
        resume_waypoints = deepcopy(waypoints[idx:]) if idx > 0 else deepcopy(waypoints)
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        if removed_wp_id is not None:
            emit(f"{log_prefix} Resume trimmed by isDone (lastRemovedWP={removed_wp_id}).")
    else:
        done_waypoints = []
        resume_waypoints = deepcopy(waypoints)
        removed_wp_id = prev_wp

    # Keep resume non-empty so downstream mission chain remains valid.
    if not resume_waypoints and waypoints:
        forced_start_idx: Optional[int] = None
        if prev_wp is not None:
            for idx, waypoint in enumerate(waypoints):
                if _to_int(waypoint.get("waypointID")) == prev_wp:
                    forced_start_idx = min(idx + 1, len(waypoints) - 1)
                    break
        if forced_start_idx is None:
            for idx, waypoint in enumerate(waypoints):
                if not bool(waypoint.get("isDone")):
                    forced_start_idx = idx
                    break
        if forced_start_idx is None:
            forced_start_idx = len(waypoints) - 1
        done_waypoints = deepcopy(waypoints[:forced_start_idx]) if forced_start_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[forced_start_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_wp_id = prev_wp
        emit(
            f"{log_prefix} Resume fallback applied "
            f"(forcedStartWP={_to_int((resume_waypoints[0] or {}).get('waypointID'))})."
        )
    _record_trim_stage(
        "split_current_progress",
        split_started,
        waypointCount=len(waypoints),
        currentWaypointID=curr_wp,
        previousWaypointID=prev_wp,
        currentIndex=curr_idx,
        doneWaypointCount=len(done_waypoints),
        resumeWaypointCount=len(resume_waypoints),
        removedWaypointID=removed_wp_id,
    )

    done_count_started = time.perf_counter()
    done_sweep_points = count_sweep_points_in_waypoints(done_waypoints)
    _record_trim_stage(
        "count_done_sweep_points",
        done_count_started,
        doneSweepPoints=done_sweep_points,
        doneWaypointCount=len(done_waypoints),
    )

    # Append replan anchor waypoint to done path to preserve visualization continuity.
    anchor_started = time.perf_counter()
    anchor_added = False
    prior_anchor_fov_deg = float(
        get_runtime_prior_mission_profile(
            default_turn_radius_m=400.0,
            default_fov_deg=5.0,
        ).get("fov_deg", 5.0)
        or 5.0
    )
    if done_waypoints and resume_waypoints and isinstance(current_coord, dict):
        anchor_lat = _to_float(current_coord.get("latitude"))
        anchor_lon = _to_float(current_coord.get("longitude"))
        anchor_alt = _normalize_altitude_value(current_coord.get("altitude"))
        if anchor_alt is None:
            anchor_alt = _normalize_altitude_value((done_waypoints[-1].get("coordinate") or {}).get("altitude"))
        if anchor_alt is None:
            anchor_alt = _normalize_altitude_value((resume_waypoints[0].get("coordinate") or {}).get("altitude"))
        if anchor_alt is None:
            anchor_alt = 0

        if anchor_lat is not None and anchor_lon is not None:
            prev_coord = done_waypoints[-1].get("coordinate") if isinstance(done_waypoints[-1], dict) else {}
            prev_lat = _to_float((prev_coord or {}).get("latitude")) if isinstance(prev_coord, dict) else None
            prev_lon = _to_float((prev_coord or {}).get("longitude")) if isinstance(prev_coord, dict) else None
            prev_alt = _normalize_altitude_value((prev_coord or {}).get("altitude")) if isinstance(prev_coord, dict) else None
            same_as_prev = (
                prev_lat is not None
                and prev_lon is not None
                and abs(prev_lat - anchor_lat) <= 1e-7
                and abs(prev_lon - anchor_lon) <= 1e-7
                and (prev_alt or 0) == anchor_alt
            )
            if not same_as_prev:
                template_wp = done_waypoints[-1] if isinstance(done_waypoints[-1], dict) else {}
                anchor_wp: Dict[str, Any] = {
                    "waypointID": int(waypoint_allocator()) if waypoint_allocator is not None else int(_reserve_waypoint_block(1)),
                    "coordinate": {
                        "latitude": anchor_lat,
                        "longitude": anchor_lon,
                        "altitude": anchor_alt,
                    },
                    "speed": template_wp.get("speed", 30.0),
                    "eta": template_wp.get("eta", 0),
                    "ecf": template_wp.get("ecf", 0.0),
                    "nextWaypointID": 0,
                }
                if "waypointPassType" in template_wp:
                    anchor_wp["waypointPassType"] = template_wp.get("waypointPassType")
                if "filmingProperty" in template_wp:
                    template_fp = template_wp.get("filmingProperty")
                    if isinstance(template_fp, dict) and template_fp:
                        anchor_fp = deepcopy(template_fp)
                        coord_orient = anchor_fp.get("coordinateOrientation")
                        if isinstance(coord_orient, dict):
                            coord_orient = dict(coord_orient)
                            coord_orient["coordinate"] = {
                                "latitude": anchor_lat,
                                "longitude": anchor_lon,
                                "altitude": anchor_alt,
                            }
                            anchor_fp["coordinateOrientation"] = coord_orient
                        anchor_wp["filmingProperty"] = anchor_fp
                    else:
                        anchor_wp["filmingProperty"] = {
                            "fieldOfView": prior_anchor_fov_deg,
                            "sensorType": 1,
                            "operationMode": 1,
                            "coordinateOrientation": {
                                "coordinate": {
                                    "latitude": anchor_lat,
                                    "longitude": anchor_lon,
                                    "altitude": anchor_alt,
                                }
                            },
                        }
                if "loiterProperty" in template_wp:
                    template_loiter = template_wp.get("loiterProperty")
                    if isinstance(template_loiter, dict) and template_loiter:
                        anchor_wp["loiterProperty"] = deepcopy(template_loiter)
                done_waypoints.append(anchor_wp)
                anchor_added = True
                emit(
                    f"{log_prefix} Added replan anchor waypoint to done path "
                    f"(anchorWP={anchor_wp.get('waypointID')})."
                )
    _record_trim_stage(
        "append_replan_anchor",
        anchor_started,
        anchorAdded=bool(anchor_added),
        currentCoordPresent=isinstance(current_coord, dict),
        doneWaypointCount=len(done_waypoints),
        resumeWaypointCount=len(resume_waypoints),
    )

    cut_started = time.perf_counter()
    progress_entry = None
    if sweep_progress and artifacts.path_id is not None:
        progress_entry = sweep_progress.get(int(artifacts.path_id))
    resume_offset_reference_coord = current_coord if isinstance(current_coord, dict) else None
    raw_cut_points = physical_sweep_cut_points(
        progress_entry,
        default_buffer_seconds=DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
    )
    cut_points = max(0, int(raw_cut_points) - int(done_sweep_points))
    _record_trim_stage(
        "resolve_sweep_cut_points",
        cut_started,
        pathID=artifacts.path_id,
        hasProgressEntry=isinstance(progress_entry, dict),
        rawCutPoints=raw_cut_points,
        doneSweepPoints=done_sweep_points,
        cutPoints=cut_points,
    )
    if cut_points > 0 and resume_waypoints:
        trim_started = time.perf_counter()
        resume_waypoints, removed_points = trim_waypoints_by_sweep_points(
            resume_waypoints,
            cut_points,
            preserve_waypoints=True,
            reference_coord_for_offset=resume_offset_reference_coord,
        )
        _record_trim_stage(
            "trim_waypoints_by_sweep_points",
            trim_started,
            requestedCutPoints=cut_points,
            removedPoints=removed_points,
            resumeWaypointCount=len(resume_waypoints),
        )
        if removed_points > 0:
            emit(
                f"{log_prefix} Resume sweep trim applied "
                f"(cutPoints={removed_points}, rawCutPoints={raw_cut_points}, "
                f"doneSweepPoints={done_sweep_points}, pathID={artifacts.path_id})."
            )
    else:
        _record_trim_stage(
            "trim_waypoints_by_sweep_points",
            time.perf_counter(),
            skipped=True,
            requestedCutPoints=cut_points,
            resumeWaypointCount=len(resume_waypoints),
        )
    if resume_waypoints:
        merge_started = time.perf_counter()
        resume_waypoints, merged_groups = merge_small_adjacent_line_search_waypoints(
            resume_waypoints,
            max_sweeps=2,
            reference_coord_for_offset=resume_offset_reference_coord,
        )
        _record_trim_stage(
            "merge_small_adjacent_line_search_waypoints",
            merge_started,
            mergedWaypoints=merged_groups,
            resumeWaypointCount=len(resume_waypoints),
        )
        if merged_groups > 0:
            emit(
                f"{log_prefix} Resume lineSearch tail groups merged "
                f"(mergedWaypoints={merged_groups})."
            )
    else:
        _record_trim_stage(
            "merge_small_adjacent_line_search_waypoints",
            time.perf_counter(),
            skipped=True,
            resumeWaypointCount=0,
        )

    for wp in done_waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = True
    for wp in resume_waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = False

    if done_waypoints:
        reassign_done_started = time.perf_counter()
        reassign_unique_waypoint_ids_inplace(
            done_waypoints,
            waypoint_id_provider=waypoint_allocator,
        )
        _record_trim_stage(
            "reassign_done_waypoint_ids",
            reassign_done_started,
            doneWaypointCount=len(done_waypoints),
        )
    if resume_waypoints:
        capture_started = time.perf_counter()
        _apply_resume_capture_buffer(
            resume_waypoints,
            emit=emit,
        )
        _record_trim_stage(
            "apply_resume_capture_buffer",
            capture_started,
            resumeWaypointCount=len(resume_waypoints),
        )
        realign_started = time.perf_counter()
        reanchored = realign_line_search_waypoints_to_first_sweep(
            resume_waypoints,
            reference_coord_for_offset=resume_offset_reference_coord,
        )
        _record_trim_stage(
            "realign_line_search_waypoints_to_first_sweep",
            realign_started,
            reanchoredWaypoints=reanchored,
            resumeWaypointCount=len(resume_waypoints),
        )
        if reanchored > 0:
            emit(
                f"{log_prefix} Resume lineSearch anchors reoriented from UAV entry "
                f"(waypoints={reanchored})."
            )
        preserve_alt_started = time.perf_counter()
        altitude_preserved = preserve_first_waypoint_altitude_from_reference(resume_waypoints, current_coord)
        _record_trim_stage(
            "preserve_first_waypoint_altitude",
            preserve_alt_started,
            preserved=bool(altitude_preserved),
        )
        if altitude_preserved:
            emit(f"{log_prefix} Resume first waypoint altitude preserved from current UAV.")
        search_speed_weight = get_runtime_float("search_speed_weight", 1.1)
        recompute_started = time.perf_counter()
        recomputed = recompute_line_search_speed_from_geometry(
            resume_waypoints,
            first_reference_coord=current_coord,
            speed_scale=search_speed_weight,
            only_increase=True,
        )
        _record_trim_stage(
            "recompute_line_search_speed_from_geometry",
            recompute_started,
            weight=float(search_speed_weight),
            recomputedWaypoints=recomputed,
        )
        if recomputed > 0:
            emit(
                f"{log_prefix} Resume searchSpeed geometry recomputed "
                f"(weight={float(search_speed_weight):.2f}, waypoints={recomputed})."
            )
        resume_speed_scale = get_runtime_prior_float("resume_search_speed_scale", 1.3)
        scale_started = time.perf_counter()
        scaled = scale_line_search_speed(resume_waypoints, resume_speed_scale)
        _record_trim_stage(
            "scale_line_search_speed",
            scale_started,
            factor=float(resume_speed_scale),
            scaledWaypoints=scaled,
        )
        if scaled > 0:
            emit(
                f"{log_prefix} Resume searchSpeed scaled "
                f"(factor={resume_speed_scale:.2f}, waypoints={scaled})."
            )
        reassign_resume_started = time.perf_counter()
        reassign_unique_waypoint_ids_inplace(
            resume_waypoints,
            waypoint_id_provider=waypoint_allocator,
        )
        _record_trim_stage(
            "reassign_resume_waypoint_ids",
            reassign_resume_started,
            resumeWaypointCount=len(resume_waypoints),
        )
    elif timing is not None:
        timing["resume_finalize"] = {"elapsedMs": 0.0, "skipped": True, "resumeWaypointCount": 0}
    resume_fp_data["waypointList"] = resume_waypoints
    if timing is not None:
        timing["totalMs"] = round((time.perf_counter() - trim_started_total) * 1000.0, 3)
    return done_waypoints, resume_waypoints, removed_wp_id


def _clone_follow_up_replan_artifacts(
    *,
    missions: List[Dict[str, Any]],
    aircraft_id: int,
    now_ms: int,
    emit: Callable[[str], None],
    log_prefix: str,
    excluded_input_ids: Optional[Set[int]] = None,
    individual_id_provider: Optional[Callable[[], int]] = None,
    path_id_provider: Optional[Callable[[int], int]] = None,
    waypoint_id_provider: Optional[Callable[[], int]] = None,
    reservation_summaries: Optional[List[Dict[str, Any]]] = None,
    reservation_scope: str = "followUpClone",
) -> Optional[Tuple[List[Dict[str, Any]], List[Tuple[Path, Dict[str, Any]]]]]:
    pending: List[Dict[str, Any]] = []
    excluded_inputs = {int(value) for value in (excluded_input_ids or set())}
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        skip_reason = _skip_replan_follow_up_reason(mission, excluded_input_ids=excluded_inputs)
        if skip_reason is not None:
            emit(
                f"{log_prefix} Skipping follow-up mission "
                f"{_to_int(mission.get('individualMissionID'))} ({skip_reason})."
            )
            continue
        pending.append(mission)
    if not pending:
        return [], []

    if individual_id_provider is not None:
        reserved_mission_ids = [int(individual_id_provider()) for _ in pending]
    else:
        reserved_mission_ids = _reserve_individual_mission_ids(len(pending))
    if path_id_provider is not None:
        reserved_path_ids = [int(path_id_provider(int(aircraft_id))) for _ in pending]
    else:
        reserved_path_ids = _reserve_path_ids(aircraft_id, len(pending))
    cloned_missions: List[Dict[str, Any]] = []
    cloned_paths: List[Tuple[Path, Dict[str, Any]]] = []
    reassigned_waypoint_ids: List[int] = []
    waypoint_keys = ("waypointList", "uavWaypointList", "lahWaypointList")

    for mission, mission_id, path_id in zip(pending, reserved_mission_ids, reserved_path_ids):
        source_path_id = _to_int(mission.get("pathID"))
        if source_path_id is None:
            emit(
                f"{log_prefix} Follow-up mission pathID missing for aircraft {aircraft_id}; "
                "aborting artifact clone."
            )
            return None

        try:
            src = db_paths.get_db_subpath("FlightPath", f"{source_path_id}.json")
            fp_data = read_json_cached(src, kind="FlightPath")
        except Exception as exc:
            emit(
                f"{log_prefix} Failed to load follow-up FlightPath {source_path_id} "
                f"for aircraft {aircraft_id}: {exc}"
            )
            return None

        mission_copy = deepcopy(mission)
        mission_copy["individualMissionID"] = int(mission_id)
        mission_copy["pathID"] = int(path_id)
        mission_copy["isDone"] = False
        cloned_missions.append(mission_copy)

        # fp_data는 read_json_cached 반환본(호출자 소유)이고 이후 재사용 없음
        fp_copy = fp_data
        fp_copy["pathID"] = int(path_id)
        fp_copy["timestamp"] = now_ms
        fp_copy["Source"] = fp_copy.get("Source") or "MMR"
        fp_copy["aircraftID"] = aircraft_id
        fp_copy["individualMissionID"] = int(mission_id)
        for key in waypoint_keys:
            waypoints = fp_copy.get(key)
            if not isinstance(waypoints, list):
                continue
            # read_json_cached() returned an owned FlightPath copy above, so
            # cloning this waypoint list again only repeats a potentially
            # large deep copy without adding isolation.
            copied_waypoints = waypoints
            copied_wp_dicts = [wp for wp in copied_waypoints if isinstance(wp, dict)]
            for wp in copied_wp_dicts:
                wp["isDone"] = False
            if copied_waypoints:
                reassign_unique_waypoint_ids_inplace(
                    copied_waypoints,
                    waypoint_id_provider=waypoint_id_provider,
                )
                for wp in copied_wp_dicts:
                    waypoint_id = _to_int(wp.get("waypointID"))
                    if waypoint_id is not None and waypoint_id > 0:
                        reassigned_waypoint_ids.append(int(waypoint_id))
            fp_copy[key] = copied_waypoints

        dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        cloned_paths.append((dest, fp_copy))

    if isinstance(reservation_summaries, list):
        reservation_summaries.append(
            {
                "scope": str(reservation_scope or "followUpClone"),
                "aircraftID": int(aircraft_id),
                "reservedIds": {
                    "individualMission": summarize_used_reserved_ids(
                        "individualMission",
                        [int(value) for value in reserved_mission_ids],
                    ),
                    "pathID": {
                        int(aircraft_id): summarize_used_reserved_ids(
                            f"pathID[{int(aircraft_id)}]",
                            [int(value) for value in reserved_path_ids],
                        )
                    },
                    "waypoint": summarize_used_reserved_ids(
                        "waypoint",
                        reassigned_waypoint_ids,
                    ),
                },
            }
        )

    return cloned_missions, cloned_paths


def _extract_related_input_mission_id(mission: Dict[str, Any]) -> Optional[int]:
    related = mission.get("relatedMission") or {}
    if not isinstance(related, dict):
        return None
    return _to_int(related.get("inputMissionID"))


def _should_skip_replan_follow_up_mission(
    mission: Dict[str, Any],
    *,
    excluded_input_ids: Set[int],
) -> bool:
    return _skip_replan_follow_up_reason(mission, excluded_input_ids=excluded_input_ids) is not None


def _skip_replan_follow_up_reason(
    mission: Dict[str, Any],
    *,
    excluded_input_ids: Set[int],
) -> Optional[str]:
    if bool(mission.get("isDone")):
        return "individual mission already done"
    input_id = _extract_related_input_mission_id(mission)
    if input_id is not None and int(input_id) in excluded_input_ids:
        return f"input mission {int(input_id)} already done"
    return None


def _preserve_follow_up_replan_artifacts(
    *,
    missions: List[Dict[str, Any]],
    aircraft_id: int,
    emit: Callable[[str], None],
    log_prefix: str,
    excluded_input_ids: Optional[Set[int]] = None,
) -> Optional[Tuple[List[Dict[str, Any]], Dict[str, Any]]]:
    preserved_missions: List[Dict[str, Any]] = []
    skipped_count = 0
    candidate_count = 0
    excluded_inputs = {int(value) for value in (excluded_input_ids or set())}

    for mission in missions or []:
        if not isinstance(mission, dict):
            continue
        skip_reason = _skip_replan_follow_up_reason(mission, excluded_input_ids=excluded_inputs)
        if skip_reason is not None:
            skipped_count += 1
            emit(
                f"{log_prefix} Skipping follow-up mission "
                f"{_to_int(mission.get('individualMissionID'))} ({skip_reason})."
            )
            continue

        candidate_count += 1
        mission_id = _to_int(mission.get("individualMissionID"))
        source_path_id = _to_int(mission.get("pathID"))
        if mission_id is None or source_path_id is None:
            emit(
                f"{log_prefix} Follow-up mission missing ID/path for aircraft {aircraft_id}; "
                "falling back to clone."
            )
            return None

        try:
            fp_src = db_paths.get_db_subpath("FlightPath", f"{int(source_path_id)}.json")
            fp_data = read_json_cached(fp_src, copy_result=False, kind="FlightPath")
        except Exception as exc:
            emit(
                f"{log_prefix} Follow-up path {source_path_id} cannot be verified for preservation; "
                f"falling back to clone ({exc})."
            )
            return None
        if not isinstance(fp_data, dict):
            return None

        fp_path_id = _to_int(fp_data.get("pathID"))
        fp_aircraft_id = _to_int(fp_data.get("aircraftID"))
        fp_mission_id = _to_int(fp_data.get("individualMissionID"))
        if fp_path_id is not None and int(fp_path_id) != int(source_path_id):
            return None
        if fp_aircraft_id is not None and int(fp_aircraft_id) != int(aircraft_id):
            return None
        if fp_mission_id is not None and int(fp_mission_id) != int(mission_id):
            return None

        for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
            waypoints = fp_data.get(key)
            if not isinstance(waypoints, list):
                continue
            if any(isinstance(wp, dict) and bool(wp.get("isDone")) for wp in waypoints):
                emit(
                    f"{log_prefix} Follow-up path {source_path_id} has completed waypoint state; "
                    "falling back to clone."
                )
                return None

        preserved = deepcopy(mission)
        preserved["isDone"] = False
        preserved_missions.append(preserved)

    return preserved_missions, {
        "candidateCount": int(candidate_count),
        "preservedCount": len(preserved_missions),
        "clonedCount": 0,
        "skippedCount": int(skipped_count),
    }


def _load_done_input_ids_for_plan(source_plan_id: int) -> Set[int]:
    done_input_ids: Set[int] = set()
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = read_json_cached(plan_path, kind="MissionPlan")
        input_package_id = _to_int(plan_data.get("inputMissionPackageID"))
        if input_package_id is None:
            return done_input_ids
        input_path = db_paths.get_db_subpath("InputMissionPlan", f"{int(input_package_id)}.json")
        input_data = read_json_cached(input_path, kind="InputMissionPlan")
    except Exception:
        return done_input_ids

    for item in input_data.get("inputMissionList") or []:
        if not isinstance(item, dict):
            continue
        if not bool(item.get("isDone")):
            continue
        input_id = _to_int(item.get("inputMissionID"))
        if input_id is not None:
            done_input_ids.add(int(input_id))
    return done_input_ids


def _normalize_coordinate_dict(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    lat = _to_float(value.get("latitude") or value.get("lat"))
    lon = _to_float(value.get("longitude") or value.get("lon"))
    alt = _normalize_altitude_value(value.get("altitude") or value.get("alt"))
    if lat is None or lon is None:
        return None
    return {"latitude": float(lat), "longitude": float(lon), "altitude": alt}


def _waypoint_coordinate_list(waypoints: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    coordinates: List[Dict[str, Any]] = []
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        coord = _normalize_coordinate_dict(waypoint.get("coordinate"))
        if coord is not None:
            coordinates.append(coord)
    return coordinates


def _build_done_reference_mission(
    template: Dict[str, Any],
    *,
    path_id: int,
    done_waypoints: List[Dict[str, Any]],
) -> Dict[str, Any]:
    mission = deepcopy(template)
    mission["pathID"] = int(path_id)
    mission["isDone"] = True
    info = deepcopy(mission.get("individualMissionInfo") or {})
    if not isinstance(info, dict):
        info = {}
    info["individualMissionType"] = 7
    info["patternType"] = 10
    info["coordinateList"] = _waypoint_coordinate_list(done_waypoints)
    info["lineList"] = []
    info["areaList"] = []
    mission["individualMissionInfo"] = info
    mission["doneReferenceOnly"] = True
    return mission


def _normalize_coord_list(value: Any, *, min_len: int = 0) -> List[Dict[str, Any]]:
    if not isinstance(value, list):
        return []
    out: List[Dict[str, Any]] = []
    for item in value:
        coord = _normalize_coordinate_dict(item)
        if coord is None:
            continue
        out.append(coord)
    if len(out) < int(min_len):
        return []
    return out


def _dedupe_coord_path(
    coords: List[Dict[str, Any]],
    *,
    closed: bool,
) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    last_key: Tuple[float, float, int] | None = None
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        alt = _normalize_altitude_value(coord.get("altitude")) or 0
        key = (round(float(lat), 9), round(float(lon), 9), int(alt))
        if key == last_key:
            continue
        out.append({"latitude": float(lat), "longitude": float(lon), "altitude": int(alt)})
        last_key = key
    if closed and len(out) >= 3:
        first = out[0]
        last = out[-1]
        if (
            abs(float(first["latitude"]) - float(last["latitude"])) <= 1e-9
            and abs(float(first["longitude"]) - float(last["longitude"])) <= 1e-9
        ):
            out = out[:-1]
    return out


def _template_line_width_m(template_mission: Optional[Dict[str, Any]]) -> Optional[float]:
    """Return the width of this aircraft's assigned LINE strip.

    ``sourceLineWidthM`` can describe the full collaborative parent corridor
    while ``lineList[*].width`` is the narrower strip actually assigned to one
    UAV.  Resume missions must keep the assigned width; otherwise every attack
    replan expands each UAV strip back to the full parent width.
    """
    if not isinstance(template_mission, dict):
        return None
    info = template_mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        return None
    assigned_width = _to_float(info.get("assignedLineWidthM"))
    if assigned_width is not None and assigned_width > 0.0:
        return float(assigned_width)
    line_list = info.get("lineList")
    if isinstance(line_list, list):
        for row in line_list:
            if not isinstance(row, dict):
                continue
            width = _to_float(row.get("width"))
            if width is not None and width > 0.0:
                return float(width)
    for key in ("width", "sourceLineWidthM"):
        width = _to_float(info.get(key))
        if width is not None and width > 0.0:
            return float(width)
    return None


def _template_source_line_width_m(template_mission: Optional[Dict[str, Any]]) -> Optional[float]:
    if not isinstance(template_mission, dict):
        return None
    info = template_mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        return None
    source_width = _to_float(info.get("sourceLineWidthM"))
    if source_width is not None and source_width > 0.0:
        return float(source_width)
    return _template_line_width_m(template_mission)


def _template_line_source_coords(template_mission: Optional[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if not isinstance(template_mission, dict):
        return []
    info = template_mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        return []
    candidates: List[Any] = [
        info.get("sourceCoordinateList"),
        info.get("coordinateList"),
    ]
    line_list = info.get("lineList")
    if isinstance(line_list, list):
        for row in line_list:
            if isinstance(row, dict):
                candidates.append(row.get("coordinateList"))
    for candidate in candidates:
        coords = _dedupe_coord_path(_normalize_coord_list(candidate, min_len=2), closed=False)
        if len(coords) >= 2:
            return coords
    return []


def _project_remaining_scan_to_source_line(
    source_coords: List[Dict[str, Any]],
    scan_coords: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    """Return the source-centerline interval covered by the remaining scan path.

    ``lineSearch.coordinateList`` is an executable zigzag/sweep path.  It must
    never be published as ``individualMissionInfo.lineList`` because consumers
    buffer that geometry by the LINE width; buffering an already-expanded
    zigzag produces a very large, irregular allocation area.  Projecting the
    remaining sweep points onto the canonical source centerline keeps the
    executable path intact while publishing the proper one-dimensional LINE
    geometry for progress/reallocation consumers.
    """
    source = _dedupe_coord_path(
        _normalize_coord_list(source_coords, min_len=2),
        closed=False,
    )
    samples = _dedupe_coord_path(
        _normalize_coord_list(scan_coords, min_len=2),
        closed=False,
    )
    if len(source) < 2 or len(samples) < 2:
        return []

    earth_radius_m = 6_371_000.0
    lat0_rad = math.radians(
        sum(float(coord["latitude"]) for coord in source) / float(len(source))
    )
    lon0_rad = math.radians(
        sum(float(coord["longitude"]) for coord in source) / float(len(source))
    )

    def to_xy(coord: Dict[str, Any]) -> Tuple[float, float]:
        lat_rad = math.radians(float(coord["latitude"]))
        lon_rad = math.radians(float(coord["longitude"]))
        return (
            earth_radius_m * (lon_rad - lon0_rad) * math.cos(lat0_rad),
            earth_radius_m * (lat_rad - lat0_rad),
        )

    source_xy = [to_xy(coord) for coord in source]
    segment_lengths: List[float] = []
    cumulative = [0.0]
    for index in range(len(source_xy) - 1):
        x1, y1 = source_xy[index]
        x2, y2 = source_xy[index + 1]
        length = math.hypot(x2 - x1, y2 - y1)
        segment_lengths.append(float(length))
        cumulative.append(cumulative[-1] + float(length))
    if cumulative[-1] <= 0.5:
        return []

    projected_along_m: List[float] = []
    for sample in samples:
        px, py = to_xy(sample)
        best_distance_sq: Optional[float] = None
        best_along_m: Optional[float] = None
        for index, segment_length in enumerate(segment_lengths):
            if segment_length <= 1e-6:
                continue
            x1, y1 = source_xy[index]
            x2, y2 = source_xy[index + 1]
            dx = x2 - x1
            dy = y2 - y1
            t = max(
                0.0,
                min(1.0, ((px - x1) * dx + (py - y1) * dy) / (segment_length ** 2)),
            )
            qx = x1 + (t * dx)
            qy = y1 + (t * dy)
            distance_sq = ((px - qx) ** 2) + ((py - qy) ** 2)
            if best_distance_sq is None or distance_sq < best_distance_sq:
                best_distance_sq = float(distance_sq)
                best_along_m = cumulative[index] + (t * segment_length)
        if best_along_m is not None:
            projected_along_m.append(float(best_along_m))
    if len(projected_along_m) < 2:
        return []

    start_m = max(0.0, min(projected_along_m))
    end_m = min(cumulative[-1], max(projected_along_m))
    if end_m - start_m <= 0.5:
        return []

    def coordinate_at(distance_m: float) -> Dict[str, Any]:
        clamped = max(0.0, min(float(distance_m), cumulative[-1]))
        segment_index = max(0, len(segment_lengths) - 1)
        for index, segment_end_m in enumerate(cumulative[1:]):
            if clamped <= segment_end_m + 1e-6:
                segment_index = index
                break
        segment_length = segment_lengths[segment_index]
        segment_start_m = cumulative[segment_index]
        ratio = 0.0 if segment_length <= 1e-6 else (clamped - segment_start_m) / segment_length
        ratio = max(0.0, min(1.0, ratio))
        first = source[segment_index]
        second = source[segment_index + 1]
        first_alt = _to_float(first.get("altitude")) or 0.0
        second_alt = _to_float(second.get("altitude")) or first_alt
        return {
            "latitude": float(first["latitude"]) + ratio * (
                float(second["latitude"]) - float(first["latitude"])
            ),
            "longitude": float(first["longitude"]) + ratio * (
                float(second["longitude"]) - float(first["longitude"])
            ),
            "altitude": int(round(first_alt + ratio * (second_alt - first_alt))),
        }

    remaining = [coordinate_at(start_m)]
    for vertex_index in range(1, len(source) - 1):
        if start_m + 1e-6 < cumulative[vertex_index] < end_m - 1e-6:
            remaining.append(deepcopy(source[vertex_index]))
    remaining.append(coordinate_at(end_m))
    return _dedupe_coord_path(remaining, closed=False)


def _remaining_line_detail_from_waypoints(
    waypoints: List[Dict[str, Any]],
    template_mission: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    width_m = _template_line_width_m(template_mission)
    source_width_m = _template_source_line_width_m(template_mission)
    rows: List[Dict[str, Any]] = []
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty")
        if not isinstance(filming, dict):
            continue
        line_search = filming.get("lineSearch")
        if not isinstance(line_search, dict):
            continue
        coords = _dedupe_coord_path(
            _normalize_coord_list(line_search.get("coordinateList"), min_len=2),
            closed=False,
        )
        if len(coords) < 2:
            continue
        row_width = _to_float(line_search.get("width"))
        rows.append(
            {
                "coordinateList": deepcopy(coords),
                "width": max(0, min(50000, int(round(float(width_m or row_width or 1.0))))),
            }
        )
    if not rows:
        return None

    flattened_coords = _dedupe_coord_path(
        [
            dict(coord)
            for row in rows
            for coord in (row.get("coordinateList") or [])
            if isinstance(coord, dict)
        ],
        closed=False,
    )
    source_coords = _template_line_source_coords(template_mission)
    remaining_source_coords = _project_remaining_scan_to_source_line(
        source_coords,
        flattened_coords,
    )
    if len(remaining_source_coords) >= 2:
        published_width = max(
            0,
            min(50000, int(round(float(width_m or rows[0].get("width") or 1.0)))),
        )
        detail = {
            "coordinateList": deepcopy(remaining_source_coords),
            "lineList": [
                {
                    "coordinateList": deepcopy(remaining_source_coords),
                    "width": published_width,
                }
            ],
            "areaList": [],
            "sourceCoordinateList": deepcopy(source_coords),
            "lineGeometrySource": "projected_source_centerline",
        }
        if width_m is not None and width_m > 0.0:
            detail["assignedLineWidthM"] = float(width_m)
        if source_width_m is not None and source_width_m > 0.0:
            detail["sourceLineWidthM"] = float(source_width_m)
        return detail

    detail: Dict[str, Any] = {
        "coordinateList": deepcopy(flattened_coords),
        "lineList": rows,
        "areaList": [],
        "lineGeometrySource": "line_search_fallback",
    }
    if width_m is not None and width_m > 0.0:
        detail["assignedLineWidthM"] = float(width_m)
    if source_width_m is not None and source_width_m > 0.0:
        detail["sourceLineWidthM"] = float(source_width_m)
    if len(source_coords) >= 2:
        detail["sourceCoordinateList"] = deepcopy(source_coords)
    return detail


def _sync_resume_mission_info_with_waypoints(
    mission: Dict[str, Any],
    waypoints: List[Dict[str, Any]],
) -> bool:
    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo")
    info = deepcopy(info) if isinstance(info, dict) else {}
    mission_type = _to_int(info.get("individualMissionType"))
    if mission_type == 3 or (isinstance(info.get("areaList"), list) and info.get("areaList")):
        # AREA is single capture.  A retained legacy path may still contain a
        # reciprocal turn and reverse suffix; cut that suffix before publishing
        # the resume mission and remove every portable two-pass/depth field.
        cut_index = len(waypoints or [])
        for waypoint_index, waypoint in enumerate(waypoints or []):
            if not isinstance(waypoint, dict):
                continue
            pass_name = str(
                waypoint.get("areaCoveragePass")
                or waypoint.get("area_coverage_pass")
                or ""
            ).strip().lower()
            turn_role = str(
                waypoint.get("areaTurnRole")
                or waypoint.get("area_turn_role")
                or ""
            ).strip().lower()
            if pass_name == "reverse" or turn_role == "reciprocal_turn":
                cut_index = int(waypoint_index)
                break
        if isinstance(waypoints, list):
            del waypoints[cut_index:]
            for waypoint in waypoints:
                if isinstance(waypoint, dict):
                    waypoint.pop("areaCoveragePass", None)
                    waypoint.pop("area_coverage_pass", None)
                    waypoint.pop("areaTurnRole", None)
                    waypoint.pop("area_turn_role", None)
        mission_area_replan_store.strip_area_multi_capture_contracts(info)
        mission_area_replan_store.strip_area_multi_capture_contracts(mission)
        mission["individualMissionInfo"] = info
        has_single_capture_work = any(
            isinstance((waypoint.get("filmingProperty") or {}).get("lineSearch"), dict)
            for waypoint in (waypoints or [])
            if isinstance(waypoint, dict)
        )
        if not has_single_capture_work:
            mission["isDone"] = True
        return True

        remaining_passes: List[str] = []
        first_pass_index: Optional[int] = None
        has_leading_turn = False
        for waypoint_index, waypoint in enumerate(waypoints or []):
            if not isinstance(waypoint, dict):
                continue
            pass_name = str(
                waypoint.get("areaCoveragePass")
                or waypoint.get("area_coverage_pass")
                or ""
            ).strip().lower()
            if pass_name in {"forward", "reverse"}:
                if first_pass_index is None:
                    first_pass_index = int(waypoint_index)
                if pass_name not in remaining_passes:
                    remaining_passes.append(pass_name)
            elif first_pass_index is None and str(
                waypoint.get("areaTurnRole")
                or waypoint.get("area_turn_role")
                or ""
            ).strip().lower() == "reciprocal_turn":
                has_leading_turn = True
        if not remaining_passes:
            remaining_passes = [
                pass_name
                for pass_name in (
                    str(value or "").strip().lower()
                    for value in (info.get("remainingCoveragePasses") or [])
                )
                if pass_name in {"forward", "reverse"}
            ]
        if not remaining_passes:
            return False

        original_order = [
            pass_name
            for pass_name in (
                str(value or "").strip().lower()
                for value in (info.get("coveragePassOrder") or [])
            )
            if pass_name in {"forward", "reverse"}
        ]
        # Planner extension tags are emitted only for the reciprocal Area
        # contract.  A reverse-only suffix therefore still has a completed
        # forward obligation even when older missionInfo lacked the metadata.
        if not original_order:
            original_order = ["forward", "reverse"]
        for pass_name in remaining_passes:
            if pass_name not in original_order:
                original_order.append(pass_name)
        completed_passes = [
            pass_name for pass_name in original_order if pass_name not in remaining_passes
        ]

        source_rows = info.get("coveragePassDetails")
        rows_by_pass = {
            str(row.get("coveragePass") or "").strip().lower(): deepcopy(row)
            for row in (source_rows or [])
            if isinstance(row, dict)
            and str(row.get("coveragePass") or "").strip().lower() in {"forward", "reverse"}
        }
        detail_rows: List[Dict[str, Any]] = []
        obligation_rows: List[Dict[str, Any]] = []
        for pass_index, pass_name in enumerate(original_order, start=1):
            row = deepcopy(rows_by_pass.get(pass_name) or {})
            row.update(
                {
                    "coveragePass": str(pass_name),
                    "passIndex": int(row.get("passIndex") or pass_index),
                    "isDone": bool(pass_name in completed_passes),
                }
            )
            detail_rows.append(row)
            if pass_name in remaining_passes:
                obligation = deepcopy(row)
                obligation["obligationKind"] = str(
                    obligation.get("obligationKind")
                    or (
                        "full"
                        if pass_name == "reverse"
                        and "forward" in completed_passes
                        and has_leading_turn
                        else "remaining"
                    )
                )
                obligation_rows.append(obligation)

        area_phase = (
            "turn"
            if has_leading_turn and remaining_passes[0] == "reverse"
            else "outbound"
            if remaining_passes[0] == "forward"
            else "return"
        )
        active_pass = None if area_phase == "turn" else remaining_passes[0]
        pass_contract = {
            "areaCoveragePassContractVersion": 1,
            "coveragePassPolicy": "all_passes_required",
            "coveragePassOrder": list(original_order),
            "coveragePassDetails": detail_rows,
            "coveragePassObligations": obligation_rows,
            "remainingCoveragePasses": list(remaining_passes),
            "completedCoveragePasses": completed_passes,
            "currentCoveragePass": remaining_passes[0],
            "activeCoveragePass": active_pass,
            "areaCoveragePhase": area_phase,
        }
        info.update(deepcopy(pass_contract))
        mission.update(deepcopy(pass_contract))
        mission["individualMissionInfo"] = info
        return True
    detail = _remaining_line_detail_from_waypoints(waypoints, mission)
    if not isinstance(detail, dict) or not _remaining_detail_has_geometry(detail):
        return False
    info["coordinateList"] = deepcopy(detail.get("coordinateList") or [])
    info["lineList"] = deepcopy(detail.get("lineList") or [])
    info["areaList"] = []
    for key in (
        "sourceCoordinateList",
        "sourceLineWidthM",
        "assignedLineWidthM",
        "lineGeometrySource",
    ):
        if key in detail:
            info[key] = deepcopy(detail[key])
    mission["individualMissionInfo"] = info
    return True


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
    coord_list = detail.get("coordinateList")
    return isinstance(coord_list, list) and len(coord_list) >= 2


def _load_input_plan_for_source_plan(source_plan_id: int) -> Optional[Dict[str, Any]]:
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = read_json_cached(plan_path, kind="MissionPlan")
        input_package_id = _to_int(plan_data.get("inputMissionPackageID"))
        if input_package_id is None or input_package_id <= 0:
            return None
        input_path = db_paths.get_db_subpath("InputMissionPlan", f"{int(input_package_id)}.json")
        return read_json_cached(input_path, kind="InputMissionPlan")
    except Exception:
        return None


def _source_input_mission_is_locked_type2_branch(
    source_plan_id: int,
    input_mission_id: int,
) -> bool:
    input_data = _load_input_plan_for_source_plan(int(source_plan_id))
    if not isinstance(input_data, dict):
        return False
    try:
        from modules.mission_planning.runtime.state import branch_ownership as store

        return store.is_locked_type2_branch_mission(
            input_data,
            int(input_mission_id),
        )
    except Exception:
        return False


def _find_input_mission_in_package(
    input_data: Dict[str, Any],
    input_mission_id: int,
) -> Optional[Dict[str, Any]]:
    for mission in input_data.get("inputMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _to_int(mission.get("inputMissionID")) == int(input_mission_id):
            return mission
    return None


def _find_next_input_mission_in_package(
    input_data: Dict[str, Any],
    current_input_id: int,
) -> Optional[Dict[str, Any]]:
    seen_current = False
    for mission in input_data.get("inputMissionList") or []:
        if not isinstance(mission, dict):
            continue
        input_id = _to_int(mission.get("inputMissionID"))
        if input_id is None or input_id <= 0:
            continue
        if int(input_id) == int(current_input_id):
            seen_current = True
            continue
        if seen_current and not bool(mission.get("isDone")):
            return mission
    return None


def _remaining_snapshot_geometry_bucket(
    *,
    mission_type: str,
    mission_detail: Dict[str, Any],
) -> str:
    mission_type_norm = str(mission_type or "").strip().lower()
    if mission_type_norm == "line":
        return "line"
    if mission_type_norm == "area":
        return "area"
    if isinstance(mission_detail.get("lineList"), list) and mission_detail.get("lineList"):
        return "line"
    if isinstance(mission_detail.get("areaList"), list) and mission_detail.get("areaList"):
        return "area"
    coord_list = mission_detail.get("coordinateList")
    if isinstance(coord_list, list) and len(coord_list) >= 2:
        return "line"
    return "area"


def _remaining_source_line_metadata(detail: Dict[str, Any]) -> Tuple[float | None, List[Dict[str, Any]]]:
    source_width_m: float | None = None
    try:
        raw_width = detail.get("sourceLineWidthM")
        if raw_width is not None:
            parsed_width = float(raw_width)
            if parsed_width > 0.0:
                source_width_m = float(parsed_width)
    except Exception:
        source_width_m = None

    coord_candidates: List[Any] = [detail.get("sourceCoordinateList")]
    source_line_list = detail.get("sourceLineList")
    if isinstance(source_line_list, list):
        for row in source_line_list:
            if isinstance(row, dict):
                coord_candidates.append(row.get("coordinateList"))
    line_list = detail.get("lineList")
    if isinstance(line_list, list):
        for row in line_list:
            if isinstance(row, dict):
                coord_candidates.append(row.get("coordinateList"))
    coord_candidates.append(detail.get("coordinateList"))

    source_coords: List[Dict[str, Any]] = []
    for candidate in coord_candidates:
        coords = _dedupe_coord_path(_normalize_coord_list(candidate, min_len=2), closed=False)
        if len(coords) >= 2:
            source_coords = coords
            break
    return source_width_m, source_coords


def _merge_line_remaining_detail(detail: Dict[str, Any]) -> Dict[str, Any]:
    try:
        from shapely.geometry import LineString, Point
    except Exception:
        LineString = None  # type: ignore[assignment]
        Point = None  # type: ignore[assignment]

    segments: List[Dict[str, Any]] = []
    width_values: List[float] = []
    line_list = detail.get("lineList")
    if isinstance(line_list, list):
        for row in line_list:
            if not isinstance(row, dict):
                continue
            coords = _dedupe_coord_path(
                _normalize_coord_list(row.get("coordinateList"), min_len=2),
                closed=False,
            )
            if len(coords) < 2:
                continue
            width = _to_float(row.get("width"))
            if width is not None and width > 0.0:
                width_values.append(float(width))
            segments.append(
                {
                    "coordinateList": coords,
                    "width": max(0, min(50000, int(round(float(width))))) if width is not None and width > 0.0 else None,
                }
            )
    if not segments:
        coords = _dedupe_coord_path(
            _normalize_coord_list(detail.get("coordinateList"), min_len=2),
            closed=False,
        )
        if len(coords) >= 2:
            segments.append({"coordinateList": coords, "width": None})
    if not segments:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    all_coords = [
        dict(coord)
        for seg in segments
        for coord in (seg.get("coordinateList") or [])
        if isinstance(coord, dict)
    ]
    ref_lat = (
        sum(float(coord["latitude"]) for coord in all_coords) / float(len(all_coords))
        if all_coords
        else 0.0
    )
    ref_lon = (
        sum(float(coord["longitude"]) for coord in all_coords) / float(len(all_coords))
        if all_coords
        else 0.0
    )

    def _coord_to_local_xy(coord: Dict[str, Any]) -> Tuple[float, float]:
        m_per_deg_lat = 111_320.0
        m_per_deg_lon = 111_320.0 * math.cos(math.radians(float(ref_lat)))
        return (
            (float(coord["longitude"]) - float(ref_lon)) * float(m_per_deg_lon),
            (float(coord["latitude"]) - float(ref_lat)) * float(m_per_deg_lat),
        )

    def _path_len(coords: List[Dict[str, Any]]) -> float:
        total = 0.0
        for idx in range(1, len(coords)):
            x0, y0 = _coord_to_local_xy(coords[idx - 1])
            x1, y1 = _coord_to_local_xy(coords[idx])
            total += math.hypot(float(x1) - float(x0), float(y1) - float(y0))
        return float(total)

    def _path_midpoint_xy(coords: List[Dict[str, Any]]) -> Optional[Tuple[float, float]]:
        points_xy = [_coord_to_local_xy(coord) for coord in coords if isinstance(coord, dict)]
        if len(points_xy) < 2:
            return points_xy[0] if points_xy else None
        seg_lengths: List[float] = []
        total = 0.0
        for idx in range(1, len(points_xy)):
            x0, y0 = points_xy[idx - 1]
            x1, y1 = points_xy[idx]
            seg_len = math.hypot(float(x1) - float(x0), float(y1) - float(y0))
            seg_lengths.append(float(seg_len))
            total += float(seg_len)
        if total <= 1e-6:
            return points_xy[len(points_xy) // 2]
        target = float(total) / 2.0
        walked = 0.0
        for idx, seg_len in enumerate(seg_lengths, start=1):
            if walked + float(seg_len) < target:
                walked += float(seg_len)
                continue
            x0, y0 = points_xy[idx - 1]
            x1, y1 = points_xy[idx]
            ratio = 0.0 if seg_len <= 1e-6 else (target - walked) / float(seg_len)
            return (
                float(x0) + (float(x1) - float(x0)) * float(ratio),
                float(y0) + (float(y1) - float(y0)) * float(ratio),
            )
        return points_xy[-1]

    source_width_m, source_coords = _remaining_source_line_metadata(detail)

    representative = max(
        segments,
        key=lambda row: (
            _path_len(row.get("coordinateList") or []),
            len(row.get("coordinateList") or []),
        ),
    )
    centerline_candidate = representative
    if len(source_coords) >= 2 and LineString is not None and Point is not None:
        try:
            source_line = LineString([_coord_to_local_xy(coord) for coord in source_coords])
        except Exception:
            source_line = None
        if source_line is not None and not source_line.is_empty:
            try:
                centerline_candidate = min(
                    segments,
                    key=lambda row: (
                        float(
                            source_line.distance(
                                Point(_path_midpoint_xy(row.get("coordinateList") or []))
                            )
                        )
                        if _path_midpoint_xy(row.get("coordinateList") or []) is not None
                        else float("inf"),
                        -_path_len(row.get("coordinateList") or []),
                        -len(row.get("coordinateList") or []),
                    ),
                )
            except Exception:
                centerline_candidate = representative
    merged_coords = deepcopy(centerline_candidate.get("coordinateList") or [])
    merged_width = max(
        [float(width) for width in width_values if width > 0.0]
        + [float(representative.get("width") or 1.0)]
    )
    if source_width_m is not None and source_width_m > 0.0:
        merged_width = float(source_width_m)
    elif LineString is not None and Point is not None and len(merged_coords) >= 2:
        try:
            rep_line = LineString([_coord_to_local_xy(coord) for coord in merged_coords])
        except Exception:
            rep_line = None
        if rep_line is not None and not rep_line.is_empty:
            half_width = float(merged_width) / 2.0
            for seg in segments:
                midpoint_xy = _path_midpoint_xy(seg.get("coordinateList") or [])
                if midpoint_xy is None:
                    continue
                seg_width = _to_float(seg.get("width")) or merged_width
                try:
                    offset_m = float(rep_line.distance(Point(midpoint_xy)))
                except Exception:
                    offset_m = 0.0
                half_width = max(
                    float(half_width),
                    float(offset_m) + (float(seg_width) / 2.0),
                )
            merged_width = max(float(merged_width), float(half_width) * 2.0)
    merged_detail = {
        "coordinateList": deepcopy(merged_coords),
        "lineList": [
            {
                "width": max(0, min(50000, int(round(float(merged_width))))),
                "coordinateList": deepcopy(merged_coords),
            }
        ],
        "areaList": [],
    }
    if source_width_m is not None and source_width_m > 0.0:
        merged_detail["sourceLineWidthM"] = float(source_width_m)
    if len(source_coords) >= 2:
        merged_detail["sourceCoordinateList"] = deepcopy(source_coords)
    return merged_detail


def _merge_area_remaining_detail(detail: Dict[str, Any]) -> Dict[str, Any]:
    try:
        from shapely.geometry import GeometryCollection, MultiPolygon, Polygon
        from shapely.ops import unary_union
    except Exception:
        area_list = detail.get("areaList")
        if isinstance(area_list, list) and area_list:
            return {
                "coordinateList": [],
                "lineList": [],
                "areaList": deepcopy(area_list[:1]),
            }
        coords = _normalize_coord_list(detail.get("coordinateList"), min_len=3)
        return {
            "coordinateList": [],
            "lineList": [],
            "areaList": [{"isHole": False, "coordinateList": deepcopy(coords)}] if coords else [],
        }

    def _coord_list_to_polygon(coords: Any) -> Optional[Polygon]:
        coord_list = _dedupe_coord_path(_normalize_coord_list(coords, min_len=3), closed=True)
        if len(coord_list) < 3:
            return None
        xy = [(float(item["longitude"]), float(item["latitude"])) for item in coord_list]
        try:
            poly = Polygon(xy)
        except Exception:
            return None
        if poly.is_empty:
            return None
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            return None
        if isinstance(poly, Polygon):
            return poly
        if isinstance(poly, MultiPolygon):
            polys = [child for child in poly.geoms if isinstance(child, Polygon) and not child.is_empty]
            return max(polys, key=lambda child: float(child.area or 0.0)) if polys else None
        return None

    outer_polys: List[Polygon] = []
    hole_polys: List[Polygon] = []
    altitude = 0
    area_list = detail.get("areaList")
    if isinstance(area_list, list) and area_list:
        for row in area_list:
            if not isinstance(row, dict):
                continue
            poly = _coord_list_to_polygon(row.get("coordinateList"))
            if poly is None:
                continue
            coords = _normalize_coord_list(row.get("coordinateList"))
            if coords:
                altitude = _normalize_altitude_value(coords[0].get("altitude")) or altitude
            if bool(row.get("isHole")):
                hole_polys.append(poly)
            else:
                outer_polys.append(poly)
    else:
        poly = _coord_list_to_polygon(detail.get("coordinateList"))
        if poly is not None:
            coords = _normalize_coord_list(detail.get("coordinateList"))
            if coords:
                altitude = _normalize_altitude_value(coords[0].get("altitude")) or altitude
            outer_polys.append(poly)
    if not outer_polys:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    geometry = unary_union(outer_polys)
    if hole_polys:
        try:
            geometry = geometry.difference(unary_union(hole_polys))
        except Exception:
            pass
    polygons: List[Polygon] = []
    if isinstance(geometry, Polygon):
        polygons = [geometry]
    elif isinstance(geometry, MultiPolygon):
        polygons = [child for child in geometry.geoms if isinstance(child, Polygon) and not child.is_empty]
    elif isinstance(geometry, GeometryCollection):
        polygons = [child for child in geometry.geoms if isinstance(child, Polygon) and not child.is_empty]
    if not polygons:
        polygons = [max(outer_polys, key=lambda poly: float(poly.area or 0.0))]

    def _ring_to_coords(ring: Any) -> List[Dict[str, Any]]:
        coords_out: List[Dict[str, Any]] = []
        for lon_val, lat_val in list(ring.coords)[:-1]:
            coords_out.append(
                {
                    "latitude": float(lat_val),
                    "longitude": float(lon_val),
                    "altitude": int(altitude),
                }
            )
        return _dedupe_coord_path(coords_out, closed=True)

    area_rows: List[Dict[str, Any]] = []
    for polygon in sorted(polygons, key=lambda poly: float(poly.area or 0.0), reverse=True):
        if polygon.is_empty:
            continue
        outer_coords = _ring_to_coords(polygon.exterior)
        if len(outer_coords) >= 3:
            area_rows.append({"isHole": False, "coordinateList": deepcopy(outer_coords)})
        for interior in polygon.interiors:
            hole_coords = _ring_to_coords(interior)
            if len(hole_coords) >= 3:
                area_rows.append({"isHole": True, "coordinateList": deepcopy(hole_coords)})
    return {
        "coordinateList": [],
        "lineList": [],
        "areaList": area_rows,
    }


def _area_owner_remaining_detail_for_unavailable(
    snapshot_entry: Dict[str, Any],
    unavailable_aircraft_ids: Set[int] | None,
) -> Tuple[Optional[Dict[str, Any]], List[int]]:
    unavailable_ids = {
        int(aid)
        for aid in (unavailable_aircraft_ids or set())
        if _to_int(aid) is not None and int(aid) > 3
    }
    if not unavailable_ids or not isinstance(snapshot_entry, dict):
        return None, []
    ownership_details = snapshot_entry.get("areaOwnershipDetails")
    if not isinstance(ownership_details, list) or not ownership_details:
        return None, []

    combined_detail: Dict[str, Any] = {
        "coordinateList": [],
        "lineList": [],
        "areaList": [],
        "areaSegmentList": [],
    }
    pass_rows_by_name: Dict[str, Dict[str, Any]] = {}
    pass_geometry_by_name: Dict[str, Dict[str, Any]] = {}
    owner_depth_rows: List[Dict[str, Any]] = []
    owner_observation_rows: List[Dict[str, Any]] = []
    owner_assignment_details: List[Dict[str, Any]] = []
    matched_aircraft_ids: List[int] = []
    for owner in ownership_details:
        if not isinstance(owner, dict):
            continue
        aircraft_id = _to_int(owner.get("aircraftID"))
        if aircraft_id is None or int(aircraft_id) not in unavailable_ids:
            continue
        raw_detail = owner.get("remainingDetail")
        if not isinstance(raw_detail, dict):
            continue

        added_geometry = False
        area_rows = raw_detail.get("areaList")
        if isinstance(area_rows, list) and area_rows:
            for row in area_rows:
                if isinstance(row, dict):
                    combined_detail["areaList"].append(deepcopy(row))
                    added_geometry = True
        segment_rows = raw_detail.get("areaSegmentList")
        if isinstance(segment_rows, list) and segment_rows:
            for row in segment_rows:
                if isinstance(row, dict):
                    combined_detail["areaSegmentList"].append(deepcopy(row))
                    added_geometry = True
        if not added_geometry:
            coords = _normalize_coord_list(raw_detail.get("coordinateList"), min_len=3)
            if len(coords) >= 3:
                combined_detail["areaList"].append(
                    {
                        "isHole": False,
                        "coordinateList": deepcopy(coords),
                    }
                )
                added_geometry = True

        if added_geometry:
            matched_aircraft_ids.append(int(aircraft_id))
            assignment_detail = mission_area_replan_store.area_assignment_detail(
                owner,
                fallback=raw_detail,
            )
            if isinstance(assignment_detail, dict):
                owner_assignment_details.append(deepcopy(assignment_detail))
            depth_contract = mission_area_replan_store.coverage_depth_replan_contract(owner)
            for depth_row in depth_contract.get("coverageDepthDetails") or []:
                if isinstance(depth_row, dict):
                    carried_depth_row = deepcopy(depth_row)
                    carried_depth_row.setdefault("sourceAircraftID", int(aircraft_id))
                    owner_depth_rows.append(carried_depth_row)
            for observation_row in depth_contract.get("coverageObservationDetails") or []:
                if isinstance(observation_row, dict):
                    carried_observation = deepcopy(observation_row)
                    carried_observation.setdefault("sourceAircraftID", int(aircraft_id))
                    owner_observation_rows.append(carried_observation)
            owner_pass_rows = owner.get("coveragePassDetails")
            if isinstance(owner_pass_rows, list):
                for owner_pass_row in owner_pass_rows:
                    if not isinstance(owner_pass_row, dict):
                        continue
                    pass_name = str(owner_pass_row.get("coveragePass") or "").strip().lower()
                    if pass_name not in {"forward", "reverse"}:
                        continue
                    aggregate = pass_rows_by_name.setdefault(
                        pass_name,
                        {
                            "coveragePass": pass_name,
                            "passIndex": _to_int(owner_pass_row.get("passIndex")) or len(pass_rows_by_name) + 1,
                            "plannedAreaM2": 0.0,
                            "coveredAreaM2": 0.0,
                            "remainingAreaM2": 0.0,
                            "isDone": True,
                        },
                    )
                    for field_name in ("plannedAreaM2", "coveredAreaM2", "remainingAreaM2"):
                        aggregate[field_name] = float(aggregate.get(field_name) or 0.0) + float(
                            _to_float(owner_pass_row.get(field_name)) or 0.0
                        )
                    aggregate["isDone"] = bool(aggregate.get("isDone")) and bool(
                        owner_pass_row.get("isDone")
                    )
                    pass_raw_detail = owner_pass_row.get("remainingDetail")
                    if not isinstance(pass_raw_detail, dict):
                        continue
                    pass_combined = pass_geometry_by_name.setdefault(
                        pass_name,
                        {"coordinateList": [], "lineList": [], "areaList": [], "areaSegmentList": []},
                    )
                    for row in pass_raw_detail.get("areaList") or []:
                        if isinstance(row, dict):
                            pass_combined["areaList"].append(deepcopy(row))
                    for row in pass_raw_detail.get("areaSegmentList") or []:
                        if isinstance(row, dict):
                            pass_combined["areaSegmentList"].append(deepcopy(row))
                    pass_coords = _normalize_coord_list(pass_raw_detail.get("coordinateList"), min_len=3)
                    if pass_coords and not pass_raw_detail.get("areaList"):
                        pass_combined["areaList"].append(
                            {"isHole": False, "coordinateList": deepcopy(pass_coords)}
                        )

    if not matched_aircraft_ids:
        return None, []

    merged_detail = _merge_area_remaining_detail(combined_detail)
    if combined_detail.get("areaSegmentList") and not merged_detail.get("areaList") and not merged_detail.get("coordinateList"):
        merged_detail["areaSegmentList"] = deepcopy(combined_detail.get("areaSegmentList") or [])
        merged_detail["areaSegmentPolicy"] = "planned_sweep_row_remaining"
    if not _remaining_detail_has_geometry(merged_detail):
        return None, sorted({int(aid) for aid in matched_aircraft_ids})
    owner_contract_source: Dict[str, Any] = {}
    if pass_rows_by_name:
        owner_pass_rows: List[Dict[str, Any]] = []
        for pass_name, aggregate in sorted(
            pass_rows_by_name.items(),
            key=lambda item: int(item[1].get("passIndex") or 0),
        ):
            pass_detail = _merge_area_remaining_detail(pass_geometry_by_name.get(pass_name) or {})
            segment_rows = list(
                (pass_geometry_by_name.get(pass_name) or {}).get("areaSegmentList") or []
            )
            if segment_rows and not _remaining_detail_has_geometry(pass_detail):
                pass_detail["areaSegmentList"] = deepcopy(segment_rows)
                pass_detail["areaSegmentPolicy"] = "planned_sweep_row_remaining"
            pass_row = deepcopy(aggregate)
            pass_row["remainingDetail"] = pass_detail
            owner_pass_rows.append(pass_row)
        owner_contract_source.update(
            {
                "coveragePassPolicy": "all_passes_required",
                "coveragePassDetails": owner_pass_rows,
            }
        )
    if owner_depth_rows:
        owner_contract_source.update(
            {
                "areaCoverageDepthContractVersion": 1,
                "coverageDepthPolicy": "spatial_capture_depth",
                "requiredCoverageDepth": 2,
                "coverageDepthDetails": owner_depth_rows,
                "coverageObservationDetails": owner_observation_rows,
            }
        )
    if owner_contract_source:
        mission_area_replan_store.apply_area_coverage_replan_contracts(
            merged_detail,
            owner_contract_source,
        )
    combined_assignment = _merge_area_remaining_detail(
        {
            "coordinateList": [],
            "lineList": [],
            "areaList": [
                deepcopy(row)
                for detail in owner_assignment_details
                for row in (detail.get("areaList") or [])
                if isinstance(row, dict)
            ],
        }
    )
    if not _remaining_detail_has_geometry(combined_assignment):
        coordinate_candidates = [
            detail.get("coordinateList")
            for detail in owner_assignment_details
            if isinstance(detail.get("coordinateList"), list)
            and len(detail.get("coordinateList") or []) >= 3
        ]
        if len(coordinate_candidates) == 1:
            combined_assignment = {
                "coordinateList": deepcopy(coordinate_candidates[0]),
                "lineList": [],
                "areaList": [],
            }
    if _remaining_detail_has_geometry(combined_assignment):
        merged_detail["areaAssignmentDetail"] = deepcopy(combined_assignment)
    return merged_detail, sorted({int(aid) for aid in matched_aircraft_ids})


def _build_remaining_input_mission_for_collaborative_replan(
    *,
    source_plan_id: int,
    current_input_id: int,
    unavailable_aircraft_ids: Set[int] | None = None,
    area_takeover_scope: str = "full_remaining",
    audit_context: str = "prior_collaborative_resume_remaining_input",
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    input_data = _load_input_plan_for_source_plan(int(source_plan_id))
    if not isinstance(input_data, dict):
        return None, None
    current_input = _find_input_mission_in_package(input_data, int(current_input_id))
    next_input = _find_next_input_mission_in_package(input_data, int(current_input_id))
    if not isinstance(current_input, dict):
        return None, next_input

    current_copy = deepcopy(current_input)
    unavailable_uav_ids = {
        int(aid)
        for aid in (unavailable_aircraft_ids or set())
        if _to_int(aid) is not None and int(aid) > 3
    }
    current_source_detail = (
        current_copy.get("missionDetail")
        if isinstance(current_copy.get("missionDetail"), dict)
        else {}
    )
    current_input_type = _to_int(current_copy.get("inputMissionType"))
    current_area_rows = (
        current_source_detail.get("areaList")
        if isinstance(current_source_detail.get("areaList"), list)
        else []
    )
    current_line_rows = (
        current_source_detail.get("lineList")
        if isinstance(current_source_detail.get("lineList"), list)
        else []
    )
    current_is_area_mission = bool(
        current_input_type == 2
        or (current_area_rows and not current_line_rows)
    )
    line_scan_detail = (
        {}
        if current_is_area_mission
        else load_line_scan_remaining_detail(
            source_plan_id=int(source_plan_id),
            input_mission_id=int(current_input_id),
            source_detail=dict(current_source_detail or {}),
        )
    )
    line_progress_completed = bool(line_scan_detail.get("lineRemainingCompleted"))
    snapshot_info = mission_area_replan_store.load_replan_ready_snapshot_entry(
        int(source_plan_id),
        int(current_input_id),
        # LINE snapshots are plan-specific.  Falling back to a globally newer
        # (possibly unselected option) plan can reverse or revive its remainder.
        # AREA retains its central-ledger/latest compatibility path.
        allow_latest=bool(current_is_area_mission),
        allow_latest_area=True,
        audit_context=str(audit_context or "prior_collaborative_resume_remaining_input"),
    )
    snapshot_entry: Optional[Dict[str, Any]] = None
    if isinstance(snapshot_info, dict):
        entry = snapshot_info.get("entry")
        if isinstance(entry, dict):
            snapshot_entry = entry

    reject_reason = ""
    remaining_detail: Dict[str, Any] = {}
    exact_snapshot_has_line_remaining = False
    if isinstance(snapshot_entry, dict):
        reject_reason = mission_area_replan_store.snapshot_entry_replan_reject_reason(
            snapshot_entry,
            exact=(
                bool(snapshot_info.get("exact"))
                if isinstance(snapshot_info, dict) and "exact" in snapshot_info
                else None
            ),
            allow_latest_area=True,
        )
        remaining_detail = mission_area_replan_store.coverage_replan_pending_remaining_detail(
            snapshot_entry
        )
        snapshot_plan_id = _to_int((snapshot_info or {}).get("snapshotMissionPlanID"))
        snapshot_is_exact = bool((snapshot_info or {}).get("exact")) or (
            snapshot_plan_id is not None and int(snapshot_plan_id) == int(source_plan_id)
        )
        exact_snapshot_has_line_remaining = bool(
            snapshot_is_exact
            and not reject_reason
            and not bool(snapshot_entry.get("isDone"))
            and _remaining_snapshot_geometry_bucket(
                mission_type=str(snapshot_entry.get("missionType") or ""),
                mission_detail=remaining_detail,
            )
            == "line"
            and _remaining_detail_has_geometry(remaining_detail)
        )

    if line_progress_completed and not exact_snapshot_has_line_remaining:
        current_copy["isDone"] = True
        current_copy["lineTakeoverSource"] = "line_scan_progress_completed"
        current_copy["lineTakeoverScope"] = "full_remaining"
        current_copy["lineTakeoverSourceAircraftIDs"] = list(
            line_scan_detail.get("lineScanContributingAircraftIDs") or []
        )
        current_copy["lineRemainingPolicy"] = str(
            line_scan_detail.get("lineRemainingPolicy") or "centerline_interval_union"
        )
        return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None

    if line_progress_completed and exact_snapshot_has_line_remaining:
        current_copy["lineCompletionConflictResolution"] = "exact_snapshot_remaining"

    if not isinstance(snapshot_entry, dict):
        if has_line_remaining_geometry(line_scan_detail):
            mission_detail = dict(current_source_detail or {})
            mission_detail.update(line_scan_detail)
            current_copy["missionDetail"] = mission_detail
            current_copy["isDone"] = False
            current_copy["lineTakeoverSource"] = "line_scan_progress"
            current_copy["lineTakeoverScope"] = "full_remaining"
            current_copy["lineTakeoverUnavailableAircraftIDs"] = sorted(int(aid) for aid in unavailable_uav_ids)
            current_copy["lineTakeoverSourceAircraftIDs"] = list(
                line_scan_detail.get("lineScanContributingAircraftIDs") or []
            )
            current_copy["lineRemainingPolicy"] = str(
                line_scan_detail.get("lineRemainingPolicy") or "centerline_interval_union"
            )
            return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None
        return None, deepcopy(next_input) if isinstance(next_input, dict) else None

    if reject_reason == "area_snapshot_latest_fallback_not_allowed":
        mission_area_replan_store.audit_snapshot_entry_rejected(
            snapshot_entry,
            requested_mission_plan_id=int(source_plan_id),
            snapshot_mission_plan_id=_to_int((snapshot_info or {}).get("snapshotMissionPlanID")),
            audit_context=str(audit_context or "prior_collaborative_resume_remaining_input"),
            reason=str(reject_reason),
        )
        if has_line_remaining_geometry(line_scan_detail):
            mission_detail = dict(current_source_detail or {})
            mission_detail.update(line_scan_detail)
            current_copy["missionDetail"] = mission_detail
            current_copy["isDone"] = False
            current_copy["lineTakeoverSource"] = "line_scan_progress"
            current_copy["lineTakeoverScope"] = "full_remaining"
            current_copy["lineTakeoverUnavailableAircraftIDs"] = sorted(int(aid) for aid in unavailable_uav_ids)
            current_copy["lineTakeoverSourceAircraftIDs"] = list(
                line_scan_detail.get("lineScanContributingAircraftIDs") or []
            )
            current_copy["lineRemainingPolicy"] = str(
                line_scan_detail.get("lineRemainingPolicy") or "centerline_interval_union"
            )
            return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None
        current_copy["isDone"] = True
        current_copy["areaOwnershipPolicy"] = "piece_only_takeover"
        current_copy["areaTakeoverSourceAircraftIDs"] = []
        current_copy["areaTakeoverSkippedReason"] = str(reject_reason)
        return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None

    if not _remaining_detail_has_geometry(remaining_detail):
        depth_contract = mission_area_replan_store.coverage_depth_replan_contract(
            snapshot_entry
        )
        if depth_contract and not bool(depth_contract.get("coverageDepthSatisfied")):
            current_copy["isDone"] = False
            current_copy["areaSnapshotReadyForReplan"] = False
            current_copy["areaSnapshotUnreadyReason"] = (
                "area_coverage_depth_geometry_unresolved"
            )
            return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None
        if has_line_remaining_geometry(line_scan_detail):
            mission_detail = dict(current_source_detail or {})
            mission_detail.update(line_scan_detail)
            current_copy["missionDetail"] = mission_detail
            current_copy["isDone"] = False
            current_copy["lineTakeoverSource"] = "line_scan_progress"
            current_copy["lineTakeoverScope"] = "full_remaining"
            current_copy["lineTakeoverUnavailableAircraftIDs"] = sorted(int(aid) for aid in unavailable_uav_ids)
            current_copy["lineTakeoverSourceAircraftIDs"] = list(
                line_scan_detail.get("lineScanContributingAircraftIDs") or []
            )
            current_copy["lineRemainingPolicy"] = str(
                line_scan_detail.get("lineRemainingPolicy") or "centerline_interval_union"
            )
            return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None
        current_copy["isDone"] = True
        return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None

    if reject_reason:
        mission_area_replan_store.audit_snapshot_entry_rejected(
            snapshot_entry,
            requested_mission_plan_id=int(source_plan_id),
            snapshot_mission_plan_id=_to_int((snapshot_info or {}).get("snapshotMissionPlanID")),
            audit_context=str(audit_context or "prior_collaborative_resume_remaining_input"),
            reason=str(reject_reason),
        )
        if has_line_remaining_geometry(line_scan_detail):
            mission_detail = dict(current_source_detail or {})
            mission_detail.update(line_scan_detail)
            current_copy["missionDetail"] = mission_detail
            current_copy["isDone"] = False
            current_copy["lineTakeoverSource"] = "line_scan_progress"
            current_copy["lineTakeoverScope"] = "full_remaining"
            current_copy["lineTakeoverUnavailableAircraftIDs"] = sorted(int(aid) for aid in unavailable_uav_ids)
            current_copy["lineTakeoverSourceAircraftIDs"] = list(
                line_scan_detail.get("lineScanContributingAircraftIDs") or []
            )
            current_copy["lineRemainingPolicy"] = str(
                line_scan_detail.get("lineRemainingPolicy") or "centerline_interval_union"
            )
            return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None
        if str(reject_reason) != "area_snapshot_not_ready_for_replan":
            current_copy["isDone"] = True
            current_copy["areaOwnershipPolicy"] = "piece_only_takeover"
            current_copy["areaTakeoverSourceAircraftIDs"] = []
            current_copy["areaTakeoverSkippedReason"] = str(reject_reason)
            return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None
        current_copy["areaSnapshotReadyForReplan"] = False
        current_copy["areaSnapshotUnreadyReason"] = str(reject_reason)

    mission_detail = dict(current_copy.get("missionDetail") or {})
    geometry_bucket = _remaining_snapshot_geometry_bucket(
        mission_type=str(snapshot_entry.get("missionType") or ""),
        mission_detail=remaining_detail if isinstance(remaining_detail, dict) else {},
    )
    if geometry_bucket == "line":
        line_detail = dict(remaining_detail or {})
        source_width_m = _to_float(snapshot_entry.get("sourceLineWidthM"))
        source_coords = _normalize_coord_list(snapshot_entry.get("sourceCoordinateList"), min_len=2)
        if source_width_m is not None and source_width_m > 0.0:
            line_detail["sourceLineWidthM"] = float(source_width_m)
        if len(source_coords) >= 2:
            line_detail["sourceCoordinateList"] = deepcopy(source_coords)
        line_scan_detail = load_line_scan_remaining_detail(
            source_plan_id=int(source_plan_id),
            input_mission_id=int(current_input_id),
            source_detail=line_detail,
        )
        if has_line_remaining_geometry(line_scan_detail):
            merged_detail = line_scan_detail
            current_copy["lineTakeoverSource"] = "line_scan_progress"
            current_copy["lineTakeoverScope"] = "full_remaining"
            current_copy["lineTakeoverUnavailableAircraftIDs"] = sorted(int(aid) for aid in unavailable_uav_ids)
            current_copy["lineTakeoverSourceAircraftIDs"] = list(
                line_scan_detail.get("lineScanContributingAircraftIDs") or []
            )
            current_copy["lineRemainingPolicy"] = str(
                line_scan_detail.get("lineRemainingPolicy") or "centerline_interval_union"
            )
        else:
            merged_detail = _merge_line_remaining_detail(line_detail)
            current_copy["lineTakeoverSource"] = "mission_area_replan_snapshot"
            current_copy["lineTakeoverScope"] = "full_remaining"
            current_copy["lineTakeoverUnavailableAircraftIDs"] = sorted(
                int(aid) for aid in unavailable_uav_ids
            )
            current_copy["lineTakeoverSourceAircraftIDs"] = list(
                snapshot_entry.get("aircraftIDs") or []
            )
            current_copy["lineRemainingPolicy"] = "exact_snapshot_remaining"
    else:
        owner_detail: Optional[Dict[str, Any]] = None
        owner_aircraft_ids: List[int] = []
        use_owner_only_area = str(area_takeover_scope or "full_remaining") != "full_remaining"
        if unavailable_uav_ids and use_owner_only_area:
            owner_detail, owner_aircraft_ids = _area_owner_remaining_detail_for_unavailable(
                snapshot_entry,
                unavailable_uav_ids,
            )
            if not owner_aircraft_ids or not _remaining_detail_has_geometry(owner_detail):
                current_copy["isDone"] = True
                current_copy["areaOwnershipPolicy"] = "piece_only_takeover"
                current_copy["areaTakeoverSourceAircraftIDs"] = []
                current_copy["areaTakeoverSkippedReason"] = "missing_unavailable_owner_remaining_detail"
                return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None

        if owner_detail is not None:
            merged_detail = owner_detail
            current_copy["areaOwnershipPolicy"] = "piece_only_takeover"
            current_copy["areaTakeoverSourceAircraftIDs"] = [int(aid) for aid in owner_aircraft_ids]
            current_copy["areaTakeoverSource"] = "mission_area_replan_snapshot"
        else:
            merged_detail = _merge_area_remaining_detail(dict(remaining_detail or {}))
            segment_rows = (
                remaining_detail.get("areaSegmentList")
                if isinstance(remaining_detail, dict)
                and isinstance(remaining_detail.get("areaSegmentList"), list)
                else []
            )
            if segment_rows and not merged_detail.get("areaList") and not merged_detail.get("coordinateList"):
                merged_detail["areaSegmentList"] = deepcopy(segment_rows)
                merged_detail["areaSegmentPolicy"] = str(
                    remaining_detail.get("areaSegmentPolicy") or "planned_sweep_row_remaining"
                )
            current_copy["areaTakeoverSource"] = "mission_area_replan_snapshot"
            current_copy["areaOwnershipPolicy"] = (
                "full_remaining_takeover" if unavailable_uav_ids else "snapshot_remaining"
            )
            current_copy["areaTakeoverSourceAircraftIDs"] = []
    pass_contract_source = owner_detail if geometry_bucket == "area" and owner_detail is not None else snapshot_entry
    depth_contract = (
        mission_area_replan_store.coverage_depth_replan_contract(pass_contract_source)
        if geometry_bucket == "area"
        else {}
    )
    if geometry_bucket == "area" and depth_contract:
        assignment_detail = mission_area_replan_store.area_assignment_detail(
            pass_contract_source,
            fallback=mission_detail,
        )
        if assignment_detail is not None:
            mission_area_replan_store.apply_area_assignment_geometry(
                mission_detail,
                assignment_detail,
            )
        mission_detail["areaCoverageWorkloadDetail"] = deepcopy(merged_detail)
    else:
        mission_detail.update(merged_detail)
        if geometry_bucket == "area" and merged_detail.get("areaList"):
            mission_detail["coordinateList"] = []
    contracts = mission_area_replan_store.apply_area_coverage_replan_contracts(
        mission_detail,
        pass_contract_source,
    )
    current_copy["missionDetail"] = mission_detail
    if contracts.get("passes") or contracts.get("depth"):
        mission_area_replan_store.apply_area_coverage_replan_contracts(
            current_copy,
            pass_contract_source,
        )
    current_copy["isDone"] = not _remaining_detail_has_geometry(merged_detail)
    return current_copy, deepcopy(next_input) if isinstance(next_input, dict) else None


def _estimate_uav_flight_path_final_eta_s(fp_data: Dict[str, Any]) -> int:
    from modules.common.eta import annotate_eta_flight_plan

    if not isinstance(fp_data, dict):
        return 0
    waypoint_list = fp_data.get("waypointList")
    if not isinstance(waypoint_list, list) or not waypoint_list:
        return 0
    existing_final_eta = 0.0
    for waypoint in waypoint_list:
        if not isinstance(waypoint, dict):
            continue
        eta_val = _to_float(waypoint.get("eta"))
        if eta_val is not None:
            existing_final_eta = max(float(existing_final_eta), float(eta_val))
    if existing_final_eta > 0.0:
        return int(round(existing_final_eta))
    payload = deepcopy(fp_data)
    try:
        annotate_eta_flight_plan(payload, waypoint_list_keys=("waypointList",))
    except Exception:
        payload = deepcopy(fp_data)
    final_eta = 0.0
    for waypoint in payload.get("waypointList") or []:
        if not isinstance(waypoint, dict):
            continue
        eta_val = _to_float(waypoint.get("eta"))
        if eta_val is not None:
            final_eta = max(float(final_eta), float(eta_val))
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        search_speed = _to_float(line_search.get("searchSpeed"))
        coords = _normalize_coord_list(line_search.get("coordinateList"), min_len=2)
        if search_speed is None or search_speed <= 0.0 or len(coords) < 2:
            continue
        line_distance_m = 0.0
        for prev_coord, next_coord in zip(coords, coords[1:]):
            line_distance_m += _haversine_distance(
                float(prev_coord["latitude"]),
                float(prev_coord["longitude"]),
                float(next_coord["latitude"]),
                float(next_coord["longitude"]),
            )
        final_eta = max(float(final_eta), float(line_distance_m) / float(search_speed))
    return int(round(final_eta))


def _boost_prior_collab_first_sweep_search_speed(
    aircraft_id: int,
    path_id: int,
    payload: Dict[str, Any],
    *,
    emit: Callable[[str], None],
    speed_scale: float | None = None,
) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        return payload
    waypoints = payload.get("waypointList")
    if not isinstance(waypoints, list) or not waypoints:
        return payload

    if speed_scale is None:
        scale = get_runtime_float(
            "replan_sweep_speed_scale",
            1.3,
        )
    else:
        try:
            scale = float(speed_scale)
        except Exception:
            scale = 1.0
    if scale <= 0.0 or abs(scale - 1.0) <= 1e-9:
        return payload

    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty")
        if not isinstance(filming, dict):
            continue
        line_search = filming.get("lineSearch")
        if not isinstance(line_search, dict):
            continue
        search_speed = _to_float(line_search.get("searchSpeed"))
        if search_speed is None or search_speed <= 0.0:
            continue
        boosted_speed = round(float(search_speed) * float(scale), 2)
        line_search["searchSpeed"] = float(boosted_speed)
        filming["lineSearch"] = line_search
        waypoint["filmingProperty"] = filming
        payload["waypointList"] = waypoints
        if "lahWaypointList" in payload:
            payload["lahWaypointList"] = deepcopy(waypoints)
        try:
            from modules.common.eta import annotate_eta_flight_plan

            annotate_eta_flight_plan(payload, waypoint_list_keys=("waypointList",))
        except Exception:
            pass
        emit(
            "[PRIOR][COLLAB] First sweep searchSpeed boosted "
            f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
            f"waypointID={_to_int(waypoint.get('waypointID'))}, "
            f"factor={scale:.2f}, old={float(search_speed):.2f}, new={float(boosted_speed):.2f})."
        )
        return payload
    return payload


def _load_imp_package_for_aircraft(
    *,
    source_plan_id: int,
    aircraft_id: int,
) -> Optional[Dict[str, Any]]:
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        # aircraftList 읽기 전용 스캔 — 사본 불필요
        plan_data = read_json_cached(plan_path, copy_result=False, kind="MissionPlan")
    except Exception:
        return None
    for aircraft_entry in plan_data.get("aircraftList") or []:
        if not isinstance(aircraft_entry, dict):
            continue
        if _to_int(aircraft_entry.get("aircraftID")) != int(aircraft_id):
            continue
        imp_id = _to_int(aircraft_entry.get("individualMissionPackageID"))
        if imp_id is None or imp_id <= 0:
            return None
        try:
            imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(imp_id)}.json")
            return read_json_cached(imp_path, kind="IndividualMissionPlan")
        except Exception:
            return None
    return None


def _aircraft_ids_for_input_mission(
    *,
    source_plan_id: int,
    input_mission_id: int,
) -> Set[int]:
    out: Set[int] = set()
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        # aircraftList 읽기 전용 스캔 — 사본 불필요
        plan_data = read_json_cached(plan_path, copy_result=False, kind="MissionPlan")
    except Exception:
        return out
    for aircraft_entry in plan_data.get("aircraftList") or []:
        if not isinstance(aircraft_entry, dict):
            continue
        aircraft_id = _to_int(aircraft_entry.get("aircraftID"))
        imp_id = _to_int(aircraft_entry.get("individualMissionPackageID"))
        if aircraft_id is None or aircraft_id <= 0 or imp_id is None or imp_id <= 0:
            continue
        try:
            imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(imp_id)}.json")
            # individualMissionList 읽기 전용 스캔 — 사본 불필요
            imp_data = read_json_cached(imp_path, copy_result=False, kind="IndividualMissionPlan")
        except Exception:
            continue
        for mission in imp_data.get("individualMissionList") or []:
            if not isinstance(mission, dict):
                continue
            if _extract_related_input_mission_id(mission) == int(input_mission_id):
                out.add(int(aircraft_id))
                break
    return out


@dataclass
class CollaborativeRemainingImpUpdate:
    aircraft_id: int
    imp_id: int
    destination: Path
    payload: Dict[str, Any]
    replacement_count: int
    flight_path_payloads: List[Dict[str, Any]] = field(default_factory=list)


def _build_collaborative_remaining_imp_update_payload(
    *,
    source_plan_id: int,
    aircraft_id: int,
    current_input_id: int,
    replacement_missions: List[Dict[str, Any]],
    now_ms: int,
    emit: Callable[[str], None],
    log_prefix: str,
    drop_prefix_missions: bool,
    reservation_summaries: Optional[List[Dict[str, Any]]] = None,
    flight_path_payloads: Optional[List[Dict[str, Any]]] = None,
) -> Optional[CollaborativeRemainingImpUpdate]:
    imp_data = _load_imp_package_for_aircraft(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
    )
    if not isinstance(imp_data, dict):
        emit(f"{log_prefix} IMP load failed for aircraft {aircraft_id}.")
        return None
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        emit(f"{log_prefix} IMP mission list missing for aircraft {aircraft_id}.")
        return None

    target_indices = [
        idx
        for idx, mission in enumerate(mission_list)
        if isinstance(mission, dict) and _extract_related_input_mission_id(mission) == int(current_input_id)
    ]
    if not target_indices:
        emit(
            f"{log_prefix} No source missions found for current inputMissionID={current_input_id} "
            f"(aircraft={aircraft_id})."
        )
        return None

    first_idx = min(target_indices)
    last_idx = max(target_indices)
    prefix = [] if drop_prefix_missions else deepcopy(mission_list[:first_idx])
    suffix = deepcopy(mission_list[last_idx + 1 :])
    active_replacements = [
        deepcopy(mission)
        for mission in replacement_missions
        if isinstance(mission, dict)
    ]
    for mission in active_replacements:
        mission["isDone"] = False
    reservation = ReplanIdReservation.reserve(imp_count=1)
    new_imp_id = reservation.next_imp()
    if isinstance(reservation_summaries, list):
        reservation_summaries.append(
            {
                "scope": "collaborativeRemainingImp",
                "aircraftID": int(aircraft_id),
                "reservedIds": reservation.summary(),
            }
        )
    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = int(now_ms)
    imp_data["individualMissionList"] = prefix + active_replacements + suffix
    imp_dest = db_paths.get_db_subpath(
        "IndividualMissionPlan",
        f"{int(imp_data['individualMissionPackageID'])}.json",
    )
    return CollaborativeRemainingImpUpdate(
        aircraft_id=int(aircraft_id),
        imp_id=int(imp_data["individualMissionPackageID"]),
        destination=imp_dest,
        payload=imp_data,
        replacement_count=len(replacement_missions),
        flight_path_payloads=[
            payload for payload in (flight_path_payloads or []) if isinstance(payload, dict)
        ],
    )


def _write_collaborative_remaining_imp_update(
    *,
    source_plan_id: int,
    aircraft_id: int,
    current_input_id: int,
    replacement_missions: List[Dict[str, Any]],
    now_ms: int,
    emit: Callable[[str], None],
    log_prefix: str,
    drop_prefix_missions: bool,
    reservation_summaries: Optional[List[Dict[str, Any]]] = None,
    flight_path_payloads: Optional[List[Dict[str, Any]]] = None,
) -> Optional[int]:
    update = _build_collaborative_remaining_imp_update_payload(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        current_input_id=int(current_input_id),
        replacement_missions=replacement_missions,
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
        drop_prefix_missions=bool(drop_prefix_missions),
        reservation_summaries=reservation_summaries,
        flight_path_payloads=flight_path_payloads,
    )
    if update is None:
        return None
    update.destination.parent.mkdir(parents=True, exist_ok=True)
    if update.flight_path_payloads:
        validate_generated_artifact_payloads(
            individual_mission_plans=[update.payload],
            flight_paths=update.flight_path_payloads,
            scope=f"collaborativeRemainingImp:{update.imp_id}",
            allow_existing_db_artifacts=True,
            log=emit,
        )
    write_json(update.destination, update.payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    emit(
        f"{log_prefix} Collaborative replacement IMP written "
        f"(aircraft={aircraft_id}, imp={update.destination.name}, missions={len(replacement_missions)})."
    )
    return int(update.imp_id)


def _prepare_uav_collaborative_resume_replan(
    *,
    source_plan_id: int,
    current_input_id: int,
    unavailable_aircraft_ids: Set[int],
    agent_state_map: Dict[int, Dict[str, Any]],
    now_ms: int,
    emit: Callable[[str], None],
    log_prefix: str,
    drop_prefix_missions: bool,
    reservation_summaries: Optional[List[Dict[str, Any]]] = None,
    replacement_mission_transform: Optional[
        Callable[[int, List[Dict[str, Any]]], List[Dict[str, Any]]]
    ] = None,
    flight_path_transform: Optional[
        Callable[[int, int, Dict[str, Any]], Dict[str, Any]]
    ] = None,
    current_input_mission_override: Optional[Dict[str, Any]] = None,
    next_input_mission_override: Optional[Dict[str, Any]] = None,
    entry_coord_map_override: Optional[Dict[int, Dict[str, Any]]] = None,
    heading_map_override: Optional[Dict[int, float]] = None,
    entry_aircraft_context_map_override: Optional[Dict[int, Dict[str, Any]]] = None,
    template_aircraft_ids_override: Optional[Set[int]] = None,
    area_takeover_scope: str = "full_remaining",
    audit_context: str = "prior_collaborative_resume_remaining_input",
    defer_writes: bool = False,
    split_single_aircraft_area_into_two: bool = False,
) -> Optional[CollaborativeResumeReplanResult]:
    collab_total_started = time.perf_counter()
    collab_import_started = time.perf_counter()
    from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
        prepare_next_collab_input_replacements,
    )

    collab_timing: Dict[str, Any] = {}

    def _record_collab_stage(name: str, started_at: float, **extra: Any) -> None:
        row: Dict[str, Any] = {"elapsedMs": round((time.perf_counter() - started_at) * 1000.0, 3)}
        if extra:
            row.update(extra)
        collab_timing[str(name)] = row

    _record_collab_stage("next_collab_import", collab_import_started)

    remaining_started = time.perf_counter()
    current_input_mission, next_input_mission = _build_remaining_input_mission_for_collaborative_replan(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids},
        area_takeover_scope=str(area_takeover_scope or "full_remaining"),
        audit_context=str(audit_context or "prior_collaborative_resume_remaining_input"),
    )
    _record_collab_stage(
        "remaining_input_build",
        remaining_started,
        hasCurrentInput=isinstance(current_input_mission, dict),
        hasNextInput=isinstance(next_input_mission, dict),
    )
    if isinstance(current_input_mission_override, dict):
        current_input_mission = deepcopy(current_input_mission_override)
    if isinstance(next_input_mission_override, dict):
        next_input_mission = deepcopy(next_input_mission_override)
    if not isinstance(current_input_mission, dict) or bool(current_input_mission.get("isDone")):
        skip_reason = ""
        if isinstance(current_input_mission, dict):
            skip_reason = str(current_input_mission.get("areaTakeoverSkippedReason") or "")
        if skip_reason:
            emit(
                f"{log_prefix} Collaborative replan skipped: current remaining mission unavailable "
                f"({skip_reason})."
            )
        else:
            emit(f"{log_prefix} Collaborative replan skipped: current remaining mission unavailable.")
        return None

    template_started = time.perf_counter()
    source_template_aircraft_ids = _aircraft_ids_for_input_mission(
        source_plan_id=int(source_plan_id),
        input_mission_id=int(current_input_id),
    )
    override_template_aircraft_ids = {
        int(aid)
        for aid in (template_aircraft_ids_override or set())
        if _to_int(aid) is not None and int(aid) > 3
    }
    template_aircraft_ids = {
        int(aid) for aid in source_template_aircraft_ids if _to_int(aid) is not None
    } | override_template_aircraft_ids
    remaining_template_aircraft_ids = {
        int(aid)
        for aid in template_aircraft_ids
        if int(aid) > 3 and int(aid) not in unavailable_aircraft_ids
    }
    _record_collab_stage(
        "template_aircraft_resolve",
        template_started,
        sourceTemplateAircraftCount=len(source_template_aircraft_ids),
        overrideTemplateAircraftCount=len(override_template_aircraft_ids),
        templateAircraftCount=len(template_aircraft_ids),
        remainingTemplateAircraftCount=len(remaining_template_aircraft_ids),
    )
    if not remaining_template_aircraft_ids:
        emit(f"{log_prefix} Collaborative replan skipped: no remaining source UAVs for current input.")
        return None

    entry_started = time.perf_counter()
    entry_coord_map: Dict[int, Dict[str, Any]] = {}
    heading_map: Dict[int, float] = {}
    raw_entry_override = entry_coord_map_override if isinstance(entry_coord_map_override, dict) else {}
    raw_heading_override = heading_map_override if isinstance(heading_map_override, dict) else {}

    for aircraft_id, state in agent_state_map.items():
        aid = _to_int(aircraft_id)
        if (
            aid is None
            or aid <= 3
            or int(aid) in unavailable_aircraft_ids
            or int(aid) not in remaining_template_aircraft_ids
        ):
            continue
        coord = _normalize_coordinate_dict(raw_entry_override.get(int(aid)))
        if coord is None:
            coord = _normalize_coordinate_dict((state or {}).get("coordinate"))
        if coord is None:
            continue
        entry_coord_map[int(aid)] = coord
        heading = _to_float(raw_heading_override.get(int(aid)))
        if heading is None:
            heading = _to_float((state or {}).get("heading"))
        if heading is not None:
            heading_map[int(aid)] = float(heading)
    _record_collab_stage(
        "entry_state_resolve",
        entry_started,
        entryAircraftCount=len(entry_coord_map),
        headingCount=len(heading_map),
    )
    if not entry_coord_map:
        emit(f"{log_prefix} Collaborative replan skipped: no remaining UAV entry coordinates.")
        return None
    entry_context_started = time.perf_counter()
    entry_aircraft_context_map = build_line_entry_context_map(
        state_map={int(aid): dict(state or {}) for aid, state in agent_state_map.items()},
        entry_coord_map={int(aid): dict(coord) for aid, coord in entry_coord_map.items()},
        heading_map={int(aid): float(val) for aid, val in heading_map.items()},
        base_context_map={
            int(_to_int(aid) or 0): dict(row)
            for aid, row in (entry_aircraft_context_map_override or {}).items()
            if _to_int(aid) is not None and int(_to_int(aid) or 0) > 0 and isinstance(row, dict)
        },
    )
    _record_collab_stage(
        "entry_context_build",
        entry_context_started,
        contextAircraftCount=len(entry_aircraft_context_map),
    )

    prepare_started = time.perf_counter()
    prepared = prepare_next_collab_input_replacements(
        source_plan_id=int(source_plan_id),
        target_input_mission=deepcopy(current_input_mission),
        entry_coord_map={int(aid): dict(coord) for aid, coord in entry_coord_map.items()},
        heading_map={int(aid): float(val) for aid, val in heading_map.items()},
        entry_aircraft_context_map=entry_aircraft_context_map,
        representative_entry=next(iter(entry_coord_map.values())),
        next_input_mission=deepcopy(next_input_mission) if isinstance(next_input_mission, dict) else None,
        turn_radius_scale=None,
        split_single_aircraft_area_into_two=bool(split_single_aircraft_area_into_two),
        now_ms=int(now_ms),
        log=emit,
    )
    _record_collab_stage(
        "next_collab_prepare",
        prepare_started,
        status="ok" if prepared is not None else "skipped",
        workflow=str(getattr(prepared, "planner_workflow", "") or ""),
        preparedTimingMs=dict(getattr(prepared, "timing_ms", {}) or {}) if prepared is not None else {},
        sourceCache=dict(getattr(prepared, "source_cache", {}) or {}) if prepared is not None else {},
    )
    if prepared is None or not getattr(prepared, "replacement_by_aircraft", None):
        emit(f"{log_prefix} Collaborative replan skipped: next-collab replacements unavailable.")
        return None
    prepared_id_reservation = dict(getattr(prepared, "id_reservation", {}) or {})
    if isinstance(reservation_summaries, list) and prepared_id_reservation:
        reservation_summaries.append(
            {
                "scope": "collaborativeReplacementPaths",
                "currentInputMissionID": int(current_input_id),
                "reservedIds": prepared_id_reservation,
            }
        )

    path_owner_by_id: Dict[int, int] = {}
    for aircraft_id, replacement_missions in prepared.replacement_by_aircraft.items():
        for mission in replacement_missions or []:
            if not isinstance(mission, dict):
                continue
            path_id = _to_int(mission.get("pathID"))
            if path_id is None:
                continue
            path_owner_by_id[int(path_id)] = int(aircraft_id)

    prepared_fp_by_path: Dict[int, Dict[str, Any]] = {}
    prepared_fp_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    finish_eta_s = 0
    normalize_started = time.perf_counter()
    for path_id, payload in (prepared.generated_fp_by_path or {}).items():
        if not isinstance(payload, dict):
            continue
        path_payload = deepcopy(payload)
        owner_aircraft_id = path_owner_by_id.get(int(path_id))
        _apply_runtime_flyover_to_flight_path_payload(path_payload)
        if owner_aircraft_id is not None and callable(flight_path_transform):
            transformed = flight_path_transform(int(owner_aircraft_id), int(path_id), path_payload)
            if isinstance(transformed, dict):
                path_payload = transformed
        _set_flight_path_waypoints_done(path_payload, False)
        sanitize_flight_path_payload_filming_altitudes(path_payload)
        prepared_fp_by_path[int(path_id)] = path_payload
        if owner_aircraft_id is not None:
            prepared_fp_by_aircraft.setdefault(int(owner_aircraft_id), []).append(path_payload)
        finish_eta_s = max(int(finish_eta_s), int(_estimate_uav_flight_path_final_eta_s(path_payload)))
    _record_collab_stage(
        "flight_path_normalize",
        normalize_started,
        pathCount=len(prepared_fp_by_path),
        ownerAircraftCount=len(prepared_fp_by_aircraft),
        finishEtaS=int(finish_eta_s),
    )

    aircraft_imp_ids: Dict[int, int] = {}
    imp_build_started = time.perf_counter()
    imp_updates: List[CollaborativeRemainingImpUpdate] = []
    for aircraft_id, replacement_missions in prepared.replacement_by_aircraft.items():
        normalized_replacements = [
            dict(item) for item in (replacement_missions or []) if isinstance(item, dict)
        ]
        if callable(replacement_mission_transform):
            normalized_replacements = list(
                replacement_mission_transform(int(aircraft_id), normalized_replacements) or []
            )
        if not normalized_replacements:
            continue
        update = _build_collaborative_remaining_imp_update_payload(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
            current_input_id=int(current_input_id),
            replacement_missions=normalized_replacements,
            now_ms=int(now_ms),
            emit=emit,
            log_prefix=log_prefix,
            drop_prefix_missions=bool(drop_prefix_missions),
            reservation_summaries=reservation_summaries,
            flight_path_payloads=prepared_fp_by_aircraft.get(int(aircraft_id), []),
        )
        if update is None:
            continue
        imp_updates.append(update)
        aircraft_imp_ids[int(aircraft_id)] = int(update.imp_id)
    _record_collab_stage(
        "imp_update_build",
        imp_build_started,
        aircraftCount=len(aircraft_imp_ids),
        replacementAircraftCount=len(prepared.replacement_by_aircraft or {}),
    )
    if imp_updates:
        imp_validate_started = time.perf_counter()
        validate_generated_artifact_payloads(
            individual_mission_plans=[update.payload for update in imp_updates],
            flight_paths=[
                payload
                for update in imp_updates
                for payload in update.flight_path_payloads
                if isinstance(payload, dict)
            ],
            scope="collaborativeRemainingImpBatch",
            allow_existing_db_artifacts=True,
            log=emit,
        )
        _record_collab_stage(
            "imp_update_validate",
            imp_validate_started,
            aircraftCount=len(imp_updates),
            flightPathPayloadCount=sum(len(update.flight_path_payloads) for update in imp_updates),
        )
        imp_write_started = time.perf_counter()
        if defer_writes:
            for update in imp_updates:
                emit(
                    f"{log_prefix} Collaborative replacement IMP queued "
                    f"(aircraft={update.aircraft_id}, imp={update.destination.name}, "
                    f"missions={update.replacement_count})."
                )
            _record_collab_stage(
                "imp_update_write",
                imp_write_started,
                aircraftCount=len(imp_updates),
                fileCount=len(imp_updates),
                writtenCount=0,
                skippedCount=0,
                deferred=True,
            )
        else:
            write_results = write_json_batch(
                [(update.destination, update.payload) for update in imp_updates],
                pretty=True,
                ensure_ascii=False,
                skip_if_unchanged=True,
            )
            for update in imp_updates:
                emit(
                    f"{log_prefix} Collaborative replacement IMP written "
                    f"(aircraft={update.aircraft_id}, imp={update.destination.name}, "
                    f"missions={update.replacement_count})."
                )
            _record_collab_stage(
                "imp_update_write",
                imp_write_started,
                aircraftCount=len(imp_updates),
                fileCount=len(write_results),
                writtenCount=sum(1 for row in write_results if row.get("written")),
                skippedCount=sum(1 for row in write_results if row.get("skipped")),
            )
    else:
        _record_collab_stage(
            "imp_update_validate",
            time.perf_counter(),
            skipped=True,
            aircraftCount=0,
        )
        _record_collab_stage(
            "imp_update_write",
            time.perf_counter(),
            skipped=True,
            aircraftCount=0,
        )

    if not aircraft_imp_ids:
        emit(f"{log_prefix} Collaborative replan skipped: no IMP updates were written.")
        return None

    generated_path_ids: Set[int] = set()
    path_write_started = time.perf_counter()
    deferred_write_entries: List[Tuple[Path, Dict[str, Any]]] = []
    if defer_writes:
        for update in imp_updates:
            deferred_write_entries.append((update.destination, update.payload))
        for path_id, path_payload in prepared_fp_by_path.items():
            if not isinstance(path_payload, dict):
                continue
            path_dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
            deferred_write_entries.append((path_dest, path_payload))
            generated_path_ids.add(int(path_id))
    else:
        for path_id, path_payload in prepared_fp_by_path.items():
            if not isinstance(path_payload, dict):
                continue
            path_dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
            path_dest.parent.mkdir(parents=True, exist_ok=True)
            write_json(path_dest, path_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
            generated_path_ids.add(int(path_id))
    _record_collab_stage(
        "flight_path_write",
        path_write_started,
        pathCount=len(generated_path_ids),
        deferred=bool(defer_writes),
    )
    collab_timing["totalMs"] = round((time.perf_counter() - collab_total_started) * 1000.0, 3)

    emit(
        f"{log_prefix} Collaborative remaining mission replanned "
        f"(inputMissionID={current_input_id}, unavailable={sorted(int(aid) for aid in unavailable_aircraft_ids)}, "
        f"remaining={sorted(aircraft_imp_ids.keys())}, workflow={prepared.planner_workflow})."
    )
    return CollaborativeResumeReplanResult(
        current_input_id=int(current_input_id),
        unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids},
        replacement_aircraft_ids=set(int(aid) for aid in aircraft_imp_ids.keys()),
        aircraft_imp_ids={int(aid): int(imp_id) for aid, imp_id in aircraft_imp_ids.items()},
        generated_path_ids=set(int(path_id) for path_id in generated_path_ids),
        finish_eta_s=int(finish_eta_s),
        planner_workflow=str(prepared.planner_workflow or ""),
        planner_result_text=str(prepared.planner_result_text or ""),
        timing_ms=dict(collab_timing),
        deferred_write_entries=deferred_write_entries,
    )


def _extract_final_uav_coordinate(fp_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    waypoints = fp_data.get("waypointList") if isinstance(fp_data, dict) else None
    if not isinstance(waypoints, list):
        return None
    for waypoint in reversed(waypoints):
        if not isinstance(waypoint, dict):
            continue
        flight_coord = _normalize_coordinate_dict(waypoint.get("coordinate"))
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        coords = line_search.get("coordinateList")
        if isinstance(coords, list) and coords:
            final_coord = _normalize_coordinate_dict(coords[-1])
            if final_coord is not None:
                flight_alt = _normalize_altitude_value((flight_coord or {}).get("altitude"))
                if flight_alt is not None:
                    final_coord["altitude"] = int(flight_alt)
                return final_coord
        if flight_coord is not None:
            return flight_coord
    return None




def _midpoint_coordinate(start: Dict[str, Any], end: Dict[str, Any]) -> Dict[str, Any]:
    start_alt = _normalize_altitude_value(start.get("altitude")) or 0
    end_alt = _normalize_altitude_value(end.get("altitude")) or start_alt
    return {
        "latitude": (float(start["latitude"]) + float(end["latitude"])) / 2.0,
        "longitude": (float(start["longitude"]) + float(end["longitude"])) / 2.0,
        "altitude": int(round((float(start_alt) + float(end_alt)) / 2.0)),
    }


def _build_uav_transit_waypoint(
    *,
    coordinate: Dict[str, Any],
    speed_mps: float,
    eta_s: int,
    orientation_coordinate: Dict[str, Any],
    waypoint_pass_type: int,
) -> Dict[str, Any]:
    transit_fov_deg = get_runtime_effective_fov_deg("global_manual_fov_deg", 5.0)
    return {
        "waypointID": 0,
        "coordinate": {
            "latitude": float(coordinate["latitude"]),
            "longitude": float(coordinate["longitude"]),
            "altitude": int(_normalize_altitude_value(coordinate.get("altitude")) or 0),
        },
        "speed": float(speed_mps),
        "eta": int(max(0, eta_s)),
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": int(waypoint_pass_type),
        "filmingProperty": {
            "fieldOfView": float(transit_fov_deg),
            "sensorType": 1,
            "operationMode": 1,
            "coordinateOrientation": {
                "coordinate": {
                    "latitude": float(orientation_coordinate["latitude"]),
                    "longitude": float(orientation_coordinate["longitude"]),
                    "altitude": int(_normalize_altitude_value(orientation_coordinate.get("altitude")) or 0),
                }
            },
        },
        "loiterProperty": {},
        "isDone": False,
    }


def _build_uav_release_resume_waypoints(
    *,
    start_coord: Dict[str, Any],
    end_coord: Dict[str, Any],
    release_eta_s: int,
    target_finish_eta_s: int,
    default_speed_mps: float = 40.0,
    min_speed_mps: float = 20.0,
    max_speed_mps: float = _RELEASE_RESUME_FAST_SPEED_MPS,
    force_speed_mps: Optional[float] = None,
    assign_waypoint_ids: bool = True,
    waypoint_id_provider: Optional[Callable[[], int]] = None,
) -> Tuple[List[Dict[str, Any]], float]:
    start_norm = _normalize_coordinate_dict(start_coord)
    end_norm = _normalize_coordinate_dict(end_coord)
    if start_norm is None or end_norm is None:
        return [], 0.0
    min_speed = float(min_speed_mps)
    max_speed = max(min_speed, float(max_speed_mps))
    forced_speed = _to_float(force_speed_mps)

    def _clamp_speed(value: float) -> float:
        return max(min_speed, min(max_speed, float(value)))

    def _effective_default_speed() -> float:
        if forced_speed is not None and forced_speed > 0.0:
            return _clamp_speed(float(forced_speed))
        return _clamp_speed(float(default_speed_mps))

    total_distance_m = _haversine_distance(
        float(start_norm["latitude"]),
        float(start_norm["longitude"]),
        float(end_norm["latitude"]),
        float(end_norm["longitude"]),
    )
    if total_distance_m <= 1.0:
        speed_mps = _effective_default_speed()
        waypoint = _build_uav_transit_waypoint(
            coordinate=end_norm,
            speed_mps=float(speed_mps),
            eta_s=0,
            orientation_coordinate=end_norm,
            waypoint_pass_type=3,
        )
        if assign_waypoint_ids:
            reassign_unique_waypoint_ids_inplace(
                [waypoint],
                waypoint_id_provider=waypoint_id_provider,
            )
        return [waypoint], float(speed_mps)

    available_duration_s = int(target_finish_eta_s) - int(release_eta_s)
    if forced_speed is not None and forced_speed > 0.0:
        speed_mps = _clamp_speed(float(forced_speed))
    elif available_duration_s > 0:
        speed_mps = max(
            float(min_speed),
            min(float(max_speed), float(total_distance_m) / float(available_duration_s)),
        )
    else:
        speed_mps = _effective_default_speed()

    midpoint = _midpoint_coordinate(start_norm, end_norm)
    leg1_m = _haversine_distance(
        float(start_norm["latitude"]),
        float(start_norm["longitude"]),
        float(midpoint["latitude"]),
        float(midpoint["longitude"]),
    )
    leg2_m = _haversine_distance(
        float(midpoint["latitude"]),
        float(midpoint["longitude"]),
        float(end_norm["latitude"]),
        float(end_norm["longitude"]),
    )
    eta_mid = int(round(float(leg1_m) / max(float(speed_mps), 1.0)))
    eta_end = int(round((float(leg1_m) + float(leg2_m)) / max(float(speed_mps), 1.0)))
    waypoints = [
        _build_uav_transit_waypoint(
            coordinate=midpoint,
            speed_mps=float(speed_mps),
            eta_s=int(eta_mid),
            orientation_coordinate=end_norm,
            waypoint_pass_type=1,
        ),
        _build_uav_transit_waypoint(
            coordinate=end_norm,
            speed_mps=float(speed_mps),
            eta_s=int(eta_end),
            orientation_coordinate=end_norm,
            waypoint_pass_type=3,
        ),
    ]
    if assign_waypoint_ids:
        reassign_unique_waypoint_ids_inplace(
            waypoints,
            waypoint_id_provider=waypoint_id_provider,
        )
    return waypoints, float(speed_mps)


def _apply_release_resume_mission_info(
    mission_entry: Dict[str, Any],
    *,
    start_coord: Dict[str, Any],
    end_coord: Dict[str, Any],
) -> None:
    info = deepcopy(mission_entry.get("individualMissionInfo") or {})
    midpoint = _midpoint_coordinate(start_coord, end_coord)
    info["autoZoomIn"] = False
    info["targetID"] = None
    # Release/resume waypoints are transit-only.  If the source mission was an
    # area/line imaging mission, keeping its original geometry makes monitoring
    # count the same photographed area again as a new assignment.
    info["individualMissionType"] = 7
    info["patternType"] = 10
    info["coordinateList"] = [deepcopy(midpoint), deepcopy(end_coord)]
    info["lineList"] = []
    info["areaList"] = []
    mission_entry["individualMissionInfo"] = info


def _line_remaining_rows_from_detail(detail: Dict[str, Any] | None) -> List[Dict[str, Any]]:
    if not isinstance(detail, dict):
        return []
    width_fallback = _to_float(detail.get("sourceLineWidthM")) or 1.0
    rows: List[Dict[str, Any]] = []
    for row in detail.get("lineList") or []:
        if not isinstance(row, dict):
            continue
        coords = _dedupe_coord_path(
            _normalize_coord_list(row.get("coordinateList"), min_len=2),
            closed=False,
        )
        if len(coords) < 2:
            continue
        width = _to_float(row.get("width")) or float(width_fallback)
        line_index = _to_int(row.get("lineIndex"))
        normalized = {
            "coordinateList": deepcopy(coords),
            "width": max(0, min(50000, int(round(float(width))))),
        }
        if line_index is not None:
            normalized["lineIndex"] = int(line_index)
        rows.append(normalized)
    if rows:
        return rows

    coords = _dedupe_coord_path(
        _normalize_coord_list(detail.get("coordinateList"), min_len=2),
        closed=False,
    )
    if len(coords) >= 2:
        return [{"coordinateList": deepcopy(coords), "width": max(0, min(50000, int(round(float(width_fallback)))))}]
    return []


def _line_search_waypoint_indices(waypoints: List[Dict[str, Any]]) -> List[int]:
    indices: List[int] = []
    for idx, waypoint in enumerate(waypoints or []):
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty")
        if not isinstance(filming, dict):
            continue
        if isinstance(filming.get("lineSearch"), dict):
            indices.append(int(idx))
    return indices


def _apply_line_remaining_detail_to_resume_waypoints(
    resume_waypoints: List[Dict[str, Any]],
    line_remaining_detail: Dict[str, Any] | None,
    *,
    aircraft_id: int,
    path_id: int,
    current_coord: Optional[Dict[str, Any]],
    emit: Callable[[str], None],
    log_prefix: str,
) -> Tuple[List[Dict[str, Any]], bool]:
    if not has_line_remaining_geometry(line_remaining_detail):
        return resume_waypoints, False
    remaining_rows = _line_remaining_rows_from_detail(line_remaining_detail)
    if not remaining_rows:
        return resume_waypoints, False
    line_indices = _line_search_waypoint_indices(resume_waypoints)
    if not line_indices:
        emit(
            f"{log_prefix} LINE remaining detail found but resume path has no lineSearch waypoints "
            f"(aircraft={aircraft_id}, pathID={path_id})."
        )
        return resume_waypoints, False

    first_line_idx = int(line_indices[0])
    last_line_idx = int(line_indices[-1])
    prefix = [deepcopy(wp) for wp in resume_waypoints[:first_line_idx] if isinstance(wp, dict)]
    suffix = [deepcopy(wp) for wp in resume_waypoints[last_line_idx + 1 :] if isinstance(wp, dict)]
    templates = [
        deepcopy(resume_waypoints[idx])
        for idx in line_indices
        if 0 <= int(idx) < len(resume_waypoints) and isinstance(resume_waypoints[idx], dict)
    ]
    if not templates:
        return resume_waypoints, False

    rebuilt_line_waypoints: List[Dict[str, Any]] = []
    for row_idx, row in enumerate(remaining_rows):
        template = deepcopy(templates[min(int(row_idx), len(templates) - 1)])
        filming = template.get("filmingProperty")
        filming = deepcopy(filming) if isinstance(filming, dict) else {}
        line_search = filming.get("lineSearch")
        line_search = deepcopy(line_search) if isinstance(line_search, dict) else {}
        coords = deepcopy(row.get("coordinateList") or [])
        if len(coords) < 2:
            continue
        line_search["coordinateList"] = coords
        line_search["width"] = float(_to_float(row.get("width")) or 1.0)
        filming["lineSearch"] = line_search
        template["filmingProperty"] = filming
        template["isDone"] = False
        if not isinstance(template.get("coordinate"), dict):
            template["coordinate"] = deepcopy(coords[0])
        rebuilt_line_waypoints.append(template)

    if not rebuilt_line_waypoints:
        return resume_waypoints, False

    rebuilt_waypoints = prefix + rebuilt_line_waypoints + suffix
    for waypoint in rebuilt_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    reassign_unique_waypoint_ids_inplace(rebuilt_waypoints)
    relink_waypoints(rebuilt_waypoints)

    reanchored = realign_line_search_waypoints_to_first_sweep(
        rebuilt_waypoints,
        reference_coord_for_offset=current_coord if isinstance(current_coord, dict) else None,
    )
    if preserve_first_waypoint_altitude_from_reference(rebuilt_waypoints, current_coord):
        emit(f"{log_prefix} Resume first waypoint altitude preserved from current UAV after LINE remaining rebuild.")

    emit(
        f"{log_prefix} Resume lineSearch rebuilt from LINE progress "
        f"(aircraft={aircraft_id}, pathID={path_id}, rows={len(rebuilt_line_waypoints)}, "
        f"source={line_remaining_detail.get('lineRemainingSource')}, "
        f"fallback={bool(line_remaining_detail.get('lineScanSourcePlanFallback'))}, "
        f"reanchored={reanchored})."
    )
    return rebuilt_waypoints, True


def _build_other_uav_resume_package(
    *,
    source_plan_id: int,
    aircraft_id: int,
    current_waypoint_id: Optional[int],
    current_coord: Optional[Dict[str, float]],
    emit: Callable[[str], None],
    now_ms: int,
    sweep_progress: Dict[int, Dict[str, Any]] | None,
    clone_follow_up_artifacts: bool = False,
    preserve_follow_up_artifacts: bool = False,
    drop_prefix_missions: bool = False,
    allow_first_mission_fallback: bool = True,
    include_done_reference_mission: bool = True,
    line_remaining_detail: Optional[Dict[str, Any]] = None,
    log_prefix: str = "[PRIOR][UAV]",
    id_reservation: Optional[ReplanIdReservation] = None,
) -> Optional[Dict[str, Any]]:
    package_started_total = time.perf_counter()
    package_timing: Dict[str, Any] = {}

    def _record_package_stage(name: str, started_at: float, **extra: Any) -> None:
        row: Dict[str, Any] = {"elapsedMs": round((time.perf_counter() - started_at) * 1000.0, 3)}
        if extra:
            row.update(extra)
        package_timing[str(name)] = row

    resolve_started = time.perf_counter()
    artifacts = _resolve_plan_artifacts(
        source_plan_id=source_plan_id,
        aircraft_id=aircraft_id,
        current_waypoint_id=current_waypoint_id,
        emit=emit,
        allow_first_mission_fallback=allow_first_mission_fallback,
    )
    _record_package_stage(
        "resolve_plan_artifacts",
        resolve_started,
        sourcePlanID=source_plan_id,
        aircraftID=aircraft_id,
        currentWaypointID=current_waypoint_id,
        artifactIndividualMissionID=artifacts.individual_mission_id if artifacts is not None else None,
        artifactPathID=artifacts.path_id if artifacts is not None else None,
        found=artifacts is not None,
    )
    if artifacts is None:
        package_timing["totalMs"] = round((time.perf_counter() - package_started_total) * 1000.0, 3)
        emit(
            f"{log_prefix} Resume package timing aircraft={aircraft_id} "
            f"timingMs={json.dumps(package_timing, ensure_ascii=False, default=str)}"
        )
        return None

    try:
        load_started = time.perf_counter()
        imp_src = db_paths.get_db_subpath(
            "IndividualMissionPlan", f"{artifacts.individual_mission_package_id}.json"
        )
        fp_src = db_paths.get_db_subpath("FlightPath", f"{artifacts.path_id}.json")
        imp_data = read_json_cached(imp_src, kind="IndividualMissionPlan")
        fp_data = read_json_cached(fp_src, kind="FlightPath")
        _record_package_stage(
            "load_artifacts",
            load_started,
            individualMissionPackageID=artifacts.individual_mission_package_id,
            pathID=artifacts.path_id,
            waypointCount=len(fp_data.get("waypointList") or []),
            missionCount=len(imp_data.get("individualMissionList") or []),
        )
    except Exception as exc:
        emit(f"[PRIOR][UAV] Failed to load artifacts for aircraft {aircraft_id}: {exc}")
        package_timing["totalMs"] = round((time.perf_counter() - package_started_total) * 1000.0, 3)
        emit(
            f"{log_prefix} Resume package timing aircraft={aircraft_id} "
            f"timingMs={json.dumps(package_timing, ensure_ascii=False, default=str)}"
        )
        return None

    path_reservation_count = 2 if include_done_reference_mission else 1
    reserve_started = time.perf_counter()
    reservation = id_reservation
    if reservation is None:
        reservation = ReplanIdReservation.reserve(
            imp_count=1,
            individual_count=1,
            path_count_by_aircraft={int(aircraft_id): path_reservation_count},
            # Reserve the optional anchor and all final done/resume waypoint IDs
            # in one transaction instead of reacquiring the global ID lock for
            # each list during trimming.
            # The optional anchor receives an ID when it is created and is then
            # included in the final list-wide reassignment.  N+2 covers both that
            # temporary ID and the final N(+anchor) IDs without another lock.
            waypoint_count=max(1, len(fp_data.get("waypointList") or []) + 2),
        )
    new_imp_id = reservation.next_imp()
    resume_individual_id = reservation.next_individual()
    done_path_id = reservation.next_path(int(aircraft_id)) if include_done_reference_mission else None
    resume_path_id = reservation.next_path(int(aircraft_id))
    _record_package_stage(
        "reserve_ids",
        reserve_started,
        pathReservationCount=path_reservation_count,
        newImpID=new_imp_id,
        resumeIndividualID=resume_individual_id,
        donePathID=done_path_id,
        resumePathID=resume_path_id,
        preReserved=bool(id_reservation is not None),
    )

    locate_started = time.perf_counter()
    mission_list = imp_data.get("individualMissionList") or []
    target_index = None
    target_mission = None
    for idx, mission in enumerate(mission_list):
        if _to_int(mission.get("individualMissionID")) == artifacts.individual_mission_id:
            target_index = idx
            target_mission = mission
            break
    _record_package_stage(
        "locate_target_mission",
        locate_started,
        targetIndex=target_index,
        targetIndividualMissionID=artifacts.individual_mission_id,
        missionCount=len(mission_list),
        found=target_mission is not None,
    )
    if target_mission is None:
        emit(
            f"[PRIOR][UAV] Individual mission {artifacts.individual_mission_id} "
            f"not found for aircraft {aircraft_id}."
        )
        package_timing["totalMs"] = round((time.perf_counter() - package_started_total) * 1000.0, 3)
        emit(
            f"{log_prefix} Resume package timing aircraft={aircraft_id} "
            f"timingMs={json.dumps(package_timing, ensure_ascii=False, default=str)}"
        )
        return None

    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    done_input_started = time.perf_counter()
    follow_up_policy_enabled = bool(clone_follow_up_artifacts or preserve_follow_up_artifacts)
    done_input_ids = _load_done_input_ids_for_plan(source_plan_id) if follow_up_policy_enabled else set()
    _record_package_stage(
        "load_done_input_ids",
        done_input_started,
        enabled=bool(follow_up_policy_enabled),
        doneInputCount=len(done_input_ids),
    )
    if clone_follow_up_artifacts and target_index is not None:
        follow_up_rows = mission_list[target_index + 1 :]
        follow_up_preserved = False
        if preserve_follow_up_artifacts:
            preserve_started = time.perf_counter()
            preserved_artifacts = _preserve_follow_up_replan_artifacts(
                missions=follow_up_rows,
                aircraft_id=aircraft_id,
                emit=emit,
                log_prefix=log_prefix,
                excluded_input_ids=done_input_ids,
            )
            if preserved_artifacts is not None:
                follow_up_missions, preserve_stats = preserved_artifacts
                follow_up_paths = []
                follow_up_preserved = True
                _record_package_stage(
                    "clone_followups",
                    preserve_started,
                    followUpMissionCount=len(follow_up_missions),
                    followUpPathCount=0,
                    preservedFollowUpCount=preserve_stats.get("preservedCount"),
                    clonedFollowUpCount=0,
                    skippedFollowUpCount=preserve_stats.get("skippedCount"),
                    preserveMode="existing_id_path",
                )
                if follow_up_missions:
                    emit(
                        f"{log_prefix} Preserved {len(follow_up_missions)} follow-up mission(s) "
                        "by existing ID/path."
                    )
            else:
                emit(f"{log_prefix} Follow-up preservation unavailable; falling back to clone.")

        if not follow_up_preserved:
            clone_started = time.perf_counter()
            cloned_artifacts = _clone_follow_up_replan_artifacts(
                missions=follow_up_rows,
                aircraft_id=aircraft_id,
                now_ms=now_ms,
                emit=emit,
                log_prefix="[PRIOR][UAV]",
                excluded_input_ids=done_input_ids,
            )
            if cloned_artifacts is None:
                package_timing["totalMs"] = round((time.perf_counter() - package_started_total) * 1000.0, 3)
                emit(
                    f"{log_prefix} Resume package timing aircraft={aircraft_id} "
                    f"timingMs={json.dumps(package_timing, ensure_ascii=False, default=str)}"
                )
                return None
            follow_up_missions, follow_up_paths = cloned_artifacts
            _record_package_stage(
                "clone_followups",
                clone_started,
                followUpMissionCount=len(follow_up_missions),
                followUpPathCount=len(follow_up_paths),
                preservedFollowUpCount=0,
                clonedFollowUpCount=len(follow_up_missions),
                preserveMode="cloned",
            )

    build_resume_started = time.perf_counter()
    resume_mission = deepcopy(target_mission)
    resume_mission["individualMissionID"] = resume_individual_id
    resume_mission["pathID"] = resume_path_id
    resume_mission["isDone"] = False

    resume_fp_data = deepcopy(fp_data)
    trim_timing: Dict[str, Any] = {}
    trim_started = time.perf_counter()
    done_waypoints, resume_waypoints, removed_wp_id = _apply_resume_path_trimming(
        resume_fp_data,
        artifacts=artifacts,
        sweep_progress=sweep_progress,
        emit=emit,
        current_coord=current_coord,
        log_prefix=log_prefix,
        waypoint_allocator=reservation.next_waypoint,
        timing=trim_timing,
    )
    _record_package_stage(
        "apply_resume_path_trimming",
        trim_started,
        doneWaypointCount=len(done_waypoints),
        resumeWaypointCount=len(resume_waypoints),
        removedWaypointID=removed_wp_id,
        detail=trim_timing,
    )
    if not resume_waypoints:
        emit(f"{log_prefix} Resume path became empty for aircraft {aircraft_id}; skipping update.")
        package_timing["totalMs"] = round((time.perf_counter() - package_started_total) * 1000.0, 3)
        emit(
            f"{log_prefix} Resume package timing aircraft={aircraft_id} "
            f"timingMs={json.dumps(package_timing, ensure_ascii=False, default=str)}"
        )
        return None

    line_remaining_applied = False
    if has_line_remaining_geometry(line_remaining_detail):
        line_remaining_started = time.perf_counter()
        resume_waypoints, line_remaining_applied = _apply_line_remaining_detail_to_resume_waypoints(
            resume_waypoints,
            line_remaining_detail,
            aircraft_id=int(aircraft_id),
            path_id=int(resume_path_id),
            current_coord=current_coord,
            emit=emit,
            log_prefix=log_prefix,
        )
        _record_package_stage(
            "apply_line_remaining_detail",
            line_remaining_started,
            applied=bool(line_remaining_applied),
            resumeWaypointCount=len(resume_waypoints),
        )
        if not resume_waypoints:
            emit(f"{log_prefix} Resume path became empty after LINE remaining rebuild for aircraft {aircraft_id}.")
            package_timing["totalMs"] = round((time.perf_counter() - package_started_total) * 1000.0, 3)
            emit(
                f"{log_prefix} Resume package timing aircraft={aircraft_id} "
                f"timingMs={json.dumps(package_timing, ensure_ascii=False, default=str)}"
            )
            return None
    else:
        _record_package_stage(
            "apply_line_remaining_detail",
            time.perf_counter(),
            skipped=True,
            resumeWaypointCount=len(resume_waypoints),
        )

    has_done_segment = bool(done_waypoints)
    if not has_done_segment:
        done_path_id = None
    elif not include_done_reference_mission:
        done_path_id = None
        emit(
            f"{log_prefix} Done-reference mission skipped "
            f"(aircraft={aircraft_id}, removedWaypointID={removed_wp_id})."
        )

    preserved_done_mission = None
    if has_done_segment and done_path_id is not None:
        preserved_done_mission = _build_done_reference_mission(
            target_mission,
            path_id=int(done_path_id),
            done_waypoints=done_waypoints,
        )

    done_fp_data = None
    if has_done_segment and done_path_id is not None:
        done_fp_data = deepcopy(fp_data)
        done_fp_data["pathID"] = done_path_id
        done_fp_data["timestamp"] = now_ms
        done_fp_data["Source"] = done_fp_data.get("Source") or "MMR"
        done_fp_data["aircraftID"] = aircraft_id
        done_fp_data["individualMissionID"] = _to_int(target_mission.get("individualMissionID"))
        done_fp_data["waypointList"] = done_waypoints

    resume_fp_data["waypointList"] = resume_waypoints
    _record_package_stage(
        "build_resume_payload",
        build_resume_started,
        hasDoneSegment=bool(has_done_segment),
        includeDoneReferenceMission=bool(include_done_reference_mission),
        lineRemainingApplied=bool(line_remaining_applied),
        followUpMissionCount=len(follow_up_missions),
        followUpPathCount=len(follow_up_paths),
    )

    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = resume_fp_data.get("Source") or "MMR"
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = resume_individual_id

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    if 0 <= target_index < len(mission_list):
        if clone_follow_up_artifacts:
            if drop_prefix_missions:
                rebuilt = []
                if preserved_done_mission is not None:
                    rebuilt.append(preserved_done_mission)
                rebuilt.append(resume_mission)
            else:
                prefix = deepcopy(mission_list[:target_index])
                rebuilt = list(prefix)
                if preserved_done_mission is not None:
                    rebuilt.append(preserved_done_mission)
                rebuilt.append(resume_mission)
            rebuilt.extend(follow_up_missions)
            mission_list[:] = rebuilt
        else:
            if drop_prefix_missions:
                rebuilt = []
                if preserved_done_mission is not None:
                    rebuilt.append(preserved_done_mission)
                rebuilt.append(resume_mission)
                mission_list[:] = rebuilt
            else:
                if preserved_done_mission is not None:
                    mission_list[target_index] = preserved_done_mission
                    mission_list.insert(target_index + 1, resume_mission)
                else:
                    mission_list[target_index] = resume_mission
    else:
        mission_list.insert(0, resume_mission)
        emit(
            f"{log_prefix} Target mission index invalid; appended resume at head (aircraft {aircraft_id})."
        )

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    done_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{done_path_id}.json")
        if done_path_id is not None and done_fp_data is not None
        else None
    )
    resume_fp_dest = db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json")
    for path in (
        imp_dest,
        *( [done_fp_dest] if done_fp_dest is not None else [] ),
        resume_fp_dest,
        *(dest for dest, _ in follow_up_paths),
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
    resume_mission["isDone"] = False
    normalize_started = time.perf_counter()
    _set_flight_path_waypoints_done(resume_fp_data, False)
    generated_flight_paths: List[Dict[str, Any]] = []
    if done_fp_dest is not None and done_fp_data is not None:
        _apply_runtime_flyover_to_flight_path_payload(done_fp_data)
        sanitize_flight_path_payload_filming_altitudes(done_fp_data)
        generated_flight_paths.append(done_fp_data)
    _apply_runtime_flyover_to_flight_path_payload(resume_fp_data)
    _set_flight_path_waypoints_done(resume_fp_data, False)
    sanitize_flight_path_payload_filming_altitudes(resume_fp_data)
    if _sync_resume_mission_info_with_waypoints(
        resume_mission,
        resume_fp_data.get("waypointList") if isinstance(resume_fp_data.get("waypointList"), list) else [],
    ):
        emit(
            f"{log_prefix} Resume missionInfo synced with trimmed lineSearch "
            f"(aircraft={aircraft_id}, pathID={resume_path_id})."
        )
    generated_flight_paths.append(resume_fp_data)
    for dest, payload in follow_up_paths:
        _apply_runtime_flyover_to_flight_path_payload(payload)
        _set_flight_path_waypoints_done(payload, False)
        sanitize_flight_path_payload_filming_altitudes(payload)
        if isinstance(payload, dict):
            generated_flight_paths.append(payload)
    _record_package_stage(
        "normalize_generated_payloads",
        normalize_started,
        generatedFlightPathCount=len(generated_flight_paths),
        followUpPathCount=len(follow_up_paths),
    )
    validation_started = time.perf_counter()
    validate_generated_artifact_payloads(
        individual_mission_plans=[imp_data],
        flight_paths=generated_flight_paths,
        scope=f"priorOtherUavResume:{new_imp_id}",
        allow_existing_db_artifacts=True,
        log=emit,
    )
    _record_package_stage(
        "validate_generated_artifacts",
        validation_started,
        generatedFlightPathCount=len(generated_flight_paths),
    )
    write_started = time.perf_counter()
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [(imp_dest, imp_data)]
    if done_fp_dest is not None and done_fp_data is not None:
        write_entries.append((done_fp_dest, done_fp_data))
    write_entries.append((resume_fp_dest, resume_fp_data))
    write_entries.extend((dest, payload) for dest, payload in follow_up_paths)
    write_results = write_json_batch(
        write_entries,
        pretty=True,
        ensure_ascii=False,
        skip_if_unchanged=True,
    )
    _record_package_stage(
        "write_json",
        write_started,
        fileCount=len(write_entries),
        followUpPathCount=len(follow_up_paths),
        writtenCount=sum(1 for row in write_results if row.get("written")),
        skippedCount=sum(1 for row in write_results if row.get("skipped")),
    )

    path_summary = (
        f"{done_fp_dest.name}/{resume_fp_dest.name}"
        if done_fp_dest is not None
        else f"{resume_fp_dest.name}"
    )
    emit(
        f"{log_prefix} Generated done/resume mission -> "
        f"aircraft={aircraft_id} IMP:{imp_dest.name} PATHS:{path_summary}"
    )

    reservation_summary = reservation.summary()
    package_timing["totalMs"] = round((time.perf_counter() - package_started_total) * 1000.0, 3)
    emit(
        f"{log_prefix} Resume package timing aircraft={aircraft_id} "
        f"timingMs={json.dumps(package_timing, ensure_ascii=False, default=str)}"
    )
    return {
        "aircraft_id": aircraft_id,
        "individualMissionPackageID": new_imp_id,
        "resume": {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        },
        "removedWaypointID": removed_wp_id,
        "donePathID": done_path_id,
        "donePath": str(done_fp_dest) if done_fp_dest is not None else None,
        "resumePath": str(resume_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
        "lineRemainingApplied": bool(line_remaining_applied),
        "reservedIds": reservation_summary,
        "timingMs": package_timing,
    }


def _build_detail_summary(detail: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(detail, dict):
        return {}
    summary: Dict[str, Any] = {}
    for key in (
        "priorMissionID",
        "missionType",
        "aircraftID",
        "pathID",
        "sourceMissionPlanID",
        "individualMissionPackageID",
        "individualMissionID",
        "currentWaypointID",
        "previousWaypointID",
    ):
        value = detail.get(key)
        if value is not None:
            summary[key] = value
    target = detail.get("targetCoordinate")
    if isinstance(target, dict):
        summary["targetCoordinate"] = {
            k: target.get(k) for k in ("latitude", "longitude", "altitude") if target.get(k) is not None
        }
    telemetry = detail.get("telemetrySnapshot")
    if telemetry is not None:
        summary["telemetrySnapshot"] = telemetry
    return summary


def _persist_prior_algorithm_log(entry: Dict[str, Any]) -> None:
    log_path = db_paths.get_db_subpath("DSS_Internal", "log_prior_algorithm.json")
    try:
        data = []
        if log_path.exists():
            raw = log_path.read_text(encoding="utf-8").strip()
            if raw:
                parsed = json.loads(raw)
                if isinstance(parsed, list):
                    data = parsed
        data.append(entry)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        write_debug_json(log_path, data, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
    except Exception:
        pass


def _preview_value(value: Any, limit: int = 256) -> str:
    try:
        text = json.dumps(value, ensure_ascii=False)
    except Exception:
        text = repr(value)
    if text is None:
        return ""
    if len(text) > limit:
        return text[: limit - 3] + "..."
    return text


def _extract_watcher_from_target_key(key: str) -> Optional[int]:
    if "-" not in key:
        return None
    try:
        _, watcher = key.split("-", 1)
        return int(watcher)
    except Exception:
        return None


def _load_target_tracking_entry(target_id: Optional[int]) -> Optional[Dict[str, Any]]:
    if target_id is None:
        return None
    try:
        target_path = db_paths.get_db_subpath("DSS_Internal", "targetInfo.json")
        data = json.loads(target_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    target_list = data.get("targetList")
    if not isinstance(target_list, dict):
        return None
    candidates: List[Dict[str, Any]] = []
    for key, entry in target_list.items():
        if not isinstance(entry, dict):
            continue
        entry_target_id = _to_int(entry.get("targetID"))
        if entry_target_id != target_id:
            continue
        watcher_id = _to_int(entry.get("watcherID"))
        if watcher_id is None:
            watcher_field = entry.get("watcher")
            if isinstance(watcher_field, dict):
                watcher_id = _to_int(
                    watcher_field.get("aircraftID")
                    or watcher_field.get("watcherID")
                    or watcher_field.get("id")
                )
            else:
                watcher_id = _to_int(watcher_field)
        if watcher_id is None:
            watcher_id = _extract_watcher_from_target_key(str(key))

        item = dict(entry)
        if watcher_id is not None:
            item["watcherID"] = watcher_id
        item["_key"] = str(key)
        candidates.append(item)

    if not candidates:
        return None

    candidates.sort(
        key=lambda item: (
            not bool(item.get("isDestroyed")),
            bool(item.get("targetInFrame")),
            _to_int(item.get("watcherID")) is not None,
            _to_int(item.get("lastUpdated")) or 0,
        ),
        reverse=True,
    )
    return candidates[0]


def _load_prior_coordinate_from_db(prior_mission_id: Optional[int]) -> Optional[Dict[str, float]]:
    if prior_mission_id is None:
        return None
    try:
        info_dir = db_paths.get_db_subpath("PriorMissionInfo")
    except Exception:
        return None
    path = info_dir / f"{int(prior_mission_id)}.json"
    if not path.exists():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    coordinate_orientation = payload.get("coordinateOrientation") or {}
    coordinate = coordinate_orientation.get("coordinate") or {}
    lat = _to_float(coordinate.get("latitude"))
    lon = _to_float(coordinate.get("longitude"))
    alt = _normalize_altitude_value(coordinate.get("altitude"))
    if lat is None or lon is None:
        return None
    result = {"latitude": lat, "longitude": lon}
    if alt is not None and alt != 0:
        result["altitude"] = alt
    else:
        dem_alt = _sample_dem_altitude(lat, lon)
        dem_alt_int = _normalize_altitude_value(dem_alt)
        if dem_alt_int is not None:
            result["altitude"] = dem_alt_int
    return result


def _load_prior_record_from_db(prior_mission_id: Optional[int]) -> Optional[Dict[str, Any]]:
    try:
        info_dir = db_paths.get_db_subpath("PriorMissionInfo")
    except Exception:
        return None

    candidates = []
    if prior_mission_id is not None:
        path = info_dir / f"{int(prior_mission_id)}.json"
        if path.exists():
            candidates.append(path)
    if not candidates:
        try:
            candidates = sorted(
                info_dir.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True
            )
        except Exception:
            return None
    for path in candidates:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        coord_block = (
            ((payload.get("coordinateOrientation") or {}).get("coordinate"))
            if isinstance(payload.get("coordinateOrientation"), dict)
            else None
        )
        target_block = payload.get("targetOrientation") or {}
        record = {
            "priorMissionID": payload.get("priorMissionID"),
            "missionType": payload.get("missionType"),
            "coordinate": coord_block,
            "targetID": _to_int(target_block.get("targetID")),
        }
        return record
    return None


def _load_latest_mission_progress_plan_id() -> Optional[int]:
    try:
        progress_dir = db_paths.get_db_subpath("DSS_Internal", "mission_progress")
    except Exception:
        return None
    try:
        candidates = list(progress_dir.glob("*.json"))
    except Exception:
        return None
    if not candidates:
        return None
    try:
        latest = max(candidates, key=lambda p: p.stat().st_mtime)
        data = json.loads(latest.read_text(encoding="utf-8"))
        plan_id = data.get("missionPlanID")
        return int(plan_id) if plan_id is not None else None
    except Exception:
        return None


def _load_detail_from_store(plan_ids: List[int]) -> Optional[Dict[str, Any]]:
    for value in plan_ids:
        try:
            plan_id = int(value)
        except Exception:
            continue
        payload = prior_replan_store.load_detail(plan_id)
        if payload:
            return payload
    return None


def _scan_latest_source_plan_id() -> Optional[int]:
    try:
        plan_dir = db_paths.get_db_subpath("MissionPlan")
    except Exception:
        return None
    try:
        candidates = list(plan_dir.glob("*.json"))
    except Exception:
        return None
    if not candidates:
        return None
    try:
        latest = max(candidates, key=lambda path: path.stat().st_mtime)
        return int(latest.stem)
    except Exception:
        return None


def _resolve_plan_artifacts(
    *,
    source_plan_id: Optional[int],
    aircraft_id: Optional[int],
    current_waypoint_id: Optional[int],
    emit: Callable[[str], None],
    allow_first_mission_fallback: bool = True,
) -> Optional[PlanMissionArtifacts]:
    if source_plan_id is None or aircraft_id is None:
        return None
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = read_json_cached(plan_path, kind="MissionPlan")
    except FileNotFoundError:
        emit(f"[PRIOR] MissionPlan {source_plan_id} not found.")
        return None
    except Exception as exc:
        emit(f"[PRIOR] MissionPlan {source_plan_id} load failed: {exc}")
        return None

    aircraft_entry = None
    for entry in plan_data.get("aircraftList", []):
        try:
            entry_aircraft_id = int(entry.get("aircraftID"))
        except (TypeError, ValueError):
            continue
        if entry_aircraft_id == aircraft_id:
            aircraft_entry = entry
            break
    if aircraft_entry is None:
        emit(f"[PRIOR] Aircraft {aircraft_id} not present in MissionPlan {source_plan_id}.")
        return None

    try:
        package_id = int(aircraft_entry.get("individualMissionPackageID"))
    except (TypeError, ValueError, AttributeError):
        emit(f"[PRIOR] Aircraft {aircraft_id} missing IndividualMissionPackageID.")
        return None

    try:
        imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{package_id}.json")
        imp_data = read_json_cached(imp_path, kind="IndividualMissionPlan")
    except FileNotFoundError:
        emit(f"[PRIOR] IndividualMissionPlan {package_id} not found.")
        return None
    except Exception as exc:
        emit(f"[PRIOR] IndividualMissionPlan {package_id} load failed: {exc}")
        return None

    missions = imp_data.get("individualMissionList") or []
    target_mission = None
    previous_wp = None
    current_wp_int = _to_int(current_waypoint_id)
    resolved_current_wp = current_wp_int

    if current_wp_int is not None:
        for mission in missions:
            if isinstance(mission, dict) and bool(mission.get("isDone")):
                continue
            path_id = mission.get("pathID")
            individual_mission_id = mission.get("individualMissionID")
            if path_id is None or individual_mission_id is None:
                continue
            waypoints = _load_waypoint_ids(path_id)
            if not waypoints:
                continue
            waypoint_index_by_id: Dict[int, int] = {}
            for idx, waypoint_id in enumerate(waypoints):
                waypoint_index_by_id.setdefault(int(waypoint_id), int(idx))
            current_index = waypoint_index_by_id.get(int(current_wp_int))
            if current_index is not None:
                idx = int(current_index)
                previous_wp = waypoints[idx - 1] if idx > 0 else None
                target_mission = (
                    int(individual_mission_id),
                    int(path_id),
                )
                break

    if target_mission is None and missions and allow_first_mission_fallback:
        fallback_mission = None
        fallback_path_id = None
        fallback_waypoints: List[int] = []
        fallback_label = "first pending mission"

        for candidate in missions:
            if not isinstance(candidate, dict) or bool(candidate.get("isDone")):
                continue
            candidate_path_id = _to_int(candidate.get("pathID"))
            candidate_mission_id = _to_int(candidate.get("individualMissionID"))
            if candidate_path_id is None or candidate_mission_id is None:
                continue
            candidate_waypoints = _load_waypoint_ids(candidate_path_id)
            if not candidate_waypoints:
                continue
            fallback_mission = candidate
            fallback_path_id = int(candidate_path_id)
            fallback_waypoints = candidate_waypoints
            break

        if fallback_mission is None:
            fallback_label = "first mission"
            for candidate in missions:
                if not isinstance(candidate, dict):
                    continue
                candidate_path_id = _to_int(candidate.get("pathID"))
                candidate_mission_id = _to_int(candidate.get("individualMissionID"))
                if candidate_path_id is None or candidate_mission_id is None:
                    continue
                candidate_waypoints = _load_waypoint_ids(candidate_path_id)
                if not candidate_waypoints:
                    continue
                fallback_mission = candidate
                fallback_path_id = int(candidate_path_id)
                fallback_waypoints = candidate_waypoints
                break

        if fallback_mission is not None and fallback_path_id is not None:
            mission_id = int(_to_int(fallback_mission.get("individualMissionID")) or 0)
            path_id = int(fallback_path_id)
            if fallback_waypoints:
                resolved_current_wp = int(fallback_waypoints[0])
                previous_wp = None
            target_mission = (mission_id, path_id)
            emit(
                f"[PRIOR] Falling back to {fallback_label} for aircraft "
                f"{aircraft_id} (current waypoint not found in plan)."
            )

    if target_mission is None:
        return None

    mission_id, path_id = target_mission
    return PlanMissionArtifacts(
        source_plan_id=source_plan_id,
        aircraft_id=aircraft_id,
        individual_mission_package_id=package_id,
        individual_mission_id=mission_id,
        path_id=path_id,
        current_waypoint_id=resolved_current_wp,
        previous_waypoint_id=previous_wp,
    )


def _load_waypoint_ids(path_id: int) -> List[int]:
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        data = read_json_cached(path, copy_result=False, kind="FlightPath")
    except Exception:
        return []
    waypoints: List[int] = []
    waypoint_items: List[Any] = []
    for key in ("waypointList", "lahWaypointList", "uavWaypointList"):
        items = data.get(key)
        if isinstance(items, list) and items:
            waypoint_items = items
            break
    for wp in waypoint_items:
        if not isinstance(wp, dict):
            continue
        value = wp.get("waypointID")
        try:
            waypoints.append(int(value))
        except (TypeError, ValueError):
            continue
    return waypoints
