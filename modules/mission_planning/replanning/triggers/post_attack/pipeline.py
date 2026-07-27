from __future__ import annotations

import json
import math
import threading
import time
import concurrent.futures
from collections import defaultdict
from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Set, Tuple

from modules.common import agent_status_snapshot, db_paths, mission_area_replan_store
from modules.monitoring.logic.replan_runtime_settings import (
    get_post_attack_rejoin_settings,
    get_replan_toggle,
    get_target_detection_settings,
)
from modules.mission_planning.MissionPlanner.runtime_settings import (
    get_runtime_attack_float,
    get_runtime_float,
    pop_runtime_camera_fov_adjustment_logs,
)
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    normalize_filming_target_altitudes_in_waypoints,
    sanitize_flight_path_payload_filming_altitudes,
)
from modules.mission_planning.runtime.validation.attack_continuity import (
    collect_lah_attack_rows,
    compare_post_attack_pairs,
)
from modules.mission_planning.engine.mission_generation.id_allocation.allocator import reserve_mission_plan_ids
from modules.mission_planning.pipelines.mission_path_trim import (
    DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
    count_sweep_points_in_waypoints,
    is_line_scan_progress_entry,
    physical_sweep_buffer_points,
    physical_sweep_cut_points,
    reassign_unique_waypoint_ids_inplace,
    relink_waypoints,
    sweep_progress_points,
    trim_waypoints_by_sweep_points,
)
from modules.mission_planning.pipelines.line_scan_remaining_adapter import (
    build_line_scan_remaining_detail,
    has_line_remaining_geometry,
    load_line_scan_aircraft_remaining_detail,
    load_line_scan_remaining_detail,
)
from modules.mission_planning.pipelines.line_search_speed_guard import (
    clamp_line_search_speed_mps,
    effective_line_search_transit_m,
)
from modules.mission_planning.pipelines.ground_maneuver_mode import (
    TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
    TYPE2_SELF_RELIANCE_RETURN_LINE,
    resolve_type2_self_reliance_phase,
)
from modules.mission_planning.replanning.line_entry_context import (
    build_line_entry_context_map,
)
from modules.mission_planning.replanning.triggers.next_collab.pipeline import (
    prepare_next_collab_input_replacements,
)
from modules.mission_planning.replanning.triggers.prior.pipeline import (
    CollaborativeResumeReplanResult,
    _apply_release_resume_mission_info,
    _apply_runtime_flyover_to_flight_path_payload,
    _aircraft_ids_for_input_mission,
    _RELEASE_RESUME_FAST_SPEED_MPS,
    _build_uav_transit_waypoint,
    _build_uav_release_resume_waypoints,
    _build_other_uav_resume_package,
    _clone_follow_up_replan_artifacts,
    _build_remaining_input_mission_for_collaborative_replan,
    _estimate_uav_flight_path_final_eta_s,
    _extract_final_uav_coordinate,
    _extract_related_input_mission_id,
    _load_imp_package_for_aircraft,
    _load_input_plan_for_source_plan,
    _prepare_uav_collaborative_resume_replan,
    _remaining_detail_has_geometry,
    _resolve_plan_artifacts,
    _skip_replan_follow_up_reason,
    _source_input_mission_is_locked_type2_branch,
    _sync_resume_mission_info_with_waypoints,
    _write_collaborative_remaining_imp_update,
)
from modules.mission_planning.runtime.state.attack_tracking import (
    clear_tracking_assignments,
    list_active_tracking_assignments,
    rebind_tracking_assignments_to_plan,
    resolve_plan_lineage_ids,
)
from modules.mission_planning.runtime.state.attack_assignment import release_manned_used
from modules.mission_planning.runtime.cache.source_artifacts import (
    call_with_source_artifact_cache,
    get_active_source_artifact_cache,
    read_json_cached,
)
from modules.mission_planning.runtime.debug_artifacts import debug_artifact_mode, write_debug_json
from modules.mission_planning.runtime.replan_transaction import (
    write_json_transaction as write_json,
    write_json_transaction_batch as write_json_batch,
)
from modules.mission_planning.runtime.logging.pipeline_events import (
    PipelinePhaseTimer,
    new_replan_transaction_id,
)
from modules.mission_planning.runtime.ids.replan_reservation import ReplanIdReservation, summarize_used_reserved_ids
from modules.mission_planning.runtime.validation.replan_payloads import (
    normalize_flight_path_waypoint_altitudes_inplace,
    normalize_flight_path_waypoint_speeds_inplace,
    validate_generated_artifact_payloads,
    validate_replan_payloads,
)

LogCallback = Callable[[str], None]

_POST_ATTACK_OPTION_NAME = "공격 후 복귀 재계획"
_DEFAULT_MIN_REMAINING_ETA_S = 60
_DEFAULT_REJOIN_MARGIN_S = 45
_DEFAULT_TURN_RADIUS_M = 180.0
_DEFAULT_CRUISE_SPEED_MPS = 35.0
_DEFAULT_COLLAB_ENTRY_SPEED_MPS = 40.0
_DEFAULT_ACTIVE_PROGRESS_SKIP_PERCENT = 70
# 진행률이 이 값 미만인데 경로 기반 잔여 ETA가 "충분히 작다"고 나오면 모순으로
# 보고 ETA를 불신한다(잔여 스윕 전체가 WP 1개에 압축된 축약 경로에서 비행 ETA가
# 실제 촬영 잔여 시간보다 훨씬 짧게 계산되는 문제 방어 — 0604 로그: 진행률 0%
# 인데 잔여 87s로 판정되어 재분할이 생략되고 영역이 done 처리됨).
_DEFAULT_LOW_PROGRESS_ETA_GUARD_PERCENT = 25
_POST_ATTACK_SHORT_RETURN_DEFAULT_M = 2000.0
_POST_ATTACK_COMPLETE_HOLD_SECONDS = 15
_POST_ATTACK_COMPLETE_HOLD_RADIUS_M = 180
_POST_ATTACK_COMPLETE_HOLD_SPEED_MPS = 30.0
_FORMATION_FLIGHT_INPUT_MISSION_TYPE = 7
_LOG_BASENAME = "log_post_attack_rejoin"
_PRESERVE_ACTIVE_ASSIGNMENT_SKIP_REASONS = frozenset(
    {
        "active_group_progress_high",
        "remaining_work_too_small",
        # The evaluator checked the current plan and could not find usable
        # remaining geometry.  Do not retry against an ancestor/source plan:
        # that can resurrect an already completed LINE sweep.
        "remaining_snapshot_unavailable",
        "remaining_snapshot_completed",
        "type2_branch_owner_resume_preserved",
        # A tracking UAV still owns an executable suffix for this same input.
        # Redistributing the aggregate remaining AREA to the other UAVs while
        # leaving that package untouched assigns the same ground twice.  Keep
        # every current owner intact until the last tracker is released; the
        # returning UAV resumes only its recorded individual suffix.
        "ongoing_tracker_partition_preserved",
    }
)


def _allow_post_attack_active_only_replan(skip_reason: Any) -> bool:
    return str(skip_reason or "") not in _PRESERVE_ACTIVE_ASSIGNMENT_SKIP_REASONS


def _requires_type2_individual_suffix_refresh(
    skip_reason: Any,
    self_reliance_phase: Any,
) -> bool:
    """Refresh only an exact Type-2 outbound/return branch LINE suffix."""

    return bool(
        str(skip_reason or "") == "type2_branch_owner_resume_preserved"
        and str(self_reliance_phase or "")
        in {TYPE2_SELF_RELIANCE_OUTBOUND_LINE, TYPE2_SELF_RELIANCE_RETURN_LINE}
    )


def _source_type2_self_reliance_phase(
    *,
    source_plan_id: Optional[int],
    input_mission_id: Optional[int],
) -> Optional[str]:
    """Return a fresh exact phase only when the branch ownership guard agrees."""

    source_id = _to_int(source_plan_id)
    input_id = _to_int(input_mission_id)
    if source_id is None or input_id is None:
        return None
    input_data = _load_input_plan_for_source_plan(int(source_id))
    if not isinstance(input_data, dict):
        return None
    if not _source_input_mission_is_locked_type2_branch(
        source_plan_id=int(source_id),
        input_mission_id=int(input_id),
    ):
        return None
    return resolve_type2_self_reliance_phase(input_data, int(input_id))


def _active_progress_is_complete(progress_percent: Any) -> bool:
    try:
        return float(progress_percent) >= 100.0
    except (TypeError, ValueError):
        return False


def _include_active_completion_boundary_hold(
    progress_percent: Any,
    *,
    preserve_current_mission: bool,
    pending_area_pass_reassignment: bool,
) -> bool:
    return bool(
        _active_progress_is_complete(progress_percent)
        or preserve_current_mission
        or pending_area_pass_reassignment
    )


def _block_post_attack_followups_until_next_collab(
    *,
    current_mission_completed: bool,
    pending_area_pass_reassignment: bool,
) -> bool:
    """Keep the next input mission non-executable until collaboration handoff."""

    return bool(current_mission_completed or pending_area_pass_reassignment)


def _mark_post_attack_followups_execution_blocked(
    missions: List[Dict[str, Any]],
    *,
    current_input_id: int,
) -> int:
    """Retain future paths for display/template use while blocking SIM execution."""

    blocked = 0
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        input_id = _extract_related_input_mission_id(mission)
        if input_id is None or int(input_id) == int(current_input_id):
            mission.pop("executionBlockedUntilNextCollab", None)
            continue
        mission["executionBlockedUntilNextCollab"] = True
        blocked += 1
    return int(blocked)


def _post_attack_reserved_ids_summary(
    *,
    imp_ids: Optional[List[int]] = None,
    individual_ids: Optional[List[int]] = None,
    waypoint_ids: Optional[List[int]] = None,
    path_ids_by_aircraft: Optional[Dict[int, List[int]]] = None,
) -> Dict[str, Any]:
    summary: Dict[str, Any] = {}
    if imp_ids:
        summary["individualMissionPackage"] = summarize_used_reserved_ids(
            "individualMissionPackage",
            [int(value) for value in imp_ids],
        )
    if individual_ids:
        summary["individualMission"] = summarize_used_reserved_ids(
            "individualMission",
            [int(value) for value in individual_ids],
        )
    if waypoint_ids:
        summary["waypoint"] = summarize_used_reserved_ids(
            "waypoint",
            [int(value) for value in waypoint_ids],
        )
    path_summary: Dict[int, Dict[str, Any]] = {}
    for aircraft_id, ids in sorted((path_ids_by_aircraft or {}).items()):
        normalized_ids = [int(value) for value in ids if value is not None]
        if not normalized_ids:
            continue
        path_summary[int(aircraft_id)] = summarize_used_reserved_ids(
            f"pathID[{int(aircraft_id)}]",
            normalized_ids,
        )
    if path_summary:
        summary["pathID"] = path_summary
    return summary


def _post_attack_reservation_event(
    *,
    scope: str,
    aircraft_id: Optional[int],
    imp_ids: Optional[List[int]] = None,
    individual_ids: Optional[List[int]] = None,
    waypoint_ids: Optional[List[int]] = None,
    path_ids_by_aircraft: Optional[Dict[int, List[int]]] = None,
) -> Dict[str, Any]:
    event: Dict[str, Any] = {
        "scope": str(scope),
        "reservedIds": _post_attack_reserved_ids_summary(
            imp_ids=imp_ids,
            individual_ids=individual_ids,
            waypoint_ids=waypoint_ids,
            path_ids_by_aircraft=path_ids_by_aircraft,
        ),
    }
    if aircraft_id is not None:
        event["aircraftID"] = int(aircraft_id)
    return event


def _extend_reservation_summaries(target: List[Dict[str, Any]], value: Any) -> None:
    if not isinstance(target, list):
        return
    if isinstance(value, list):
        for item in value:
            if isinstance(item, dict):
                target.append(dict(item))
    elif isinstance(value, dict):
        target.append(dict(value))


def _post_attack_follow_up_clone_count(
    missions: List[Dict[str, Any]],
    *,
    excluded_input_ids: Optional[Set[int]] = None,
) -> int:
    excluded_inputs = {int(value) for value in (excluded_input_ids or set())}
    count = 0
    for mission in missions or []:
        if not isinstance(mission, dict):
            continue
        if _skip_replan_follow_up_reason(mission, excluded_input_ids=excluded_inputs) is None:
            count += 1
    return int(count)


def _copy_post_attack_imp_shell(imp_data: Dict[str, Any]) -> Dict[str, Any]:
    """Copy an IMP without its mission list when that list will be replaced."""

    return {
        key: deepcopy(value)
        for key, value in (imp_data or {}).items()
        if key != "individualMissionList"
    }


@dataclass
class PostAttackRejoinPipelineResult:
    plan_ids: List[int]
    option_names: List[str]
    plan_meta_map: Dict[int, Dict[str, Any]]
    generated_imp_ids: Set[int]
    generated_path_ids: Set[int]
    log_path: str
    status: str
    summary: Dict[str, Any]


@dataclass
class _PhasedLineSource:
    aircraft_id: int
    template_mission: Dict[str, Any]
    template_path: Dict[str, Any]
    prefix_waypoints: List[Dict[str, Any]]
    suffix_waypoints: List[Dict[str, Any]]
    predicted_entry_coordinate: Optional[Dict[str, Any]]
    predicted_heading_deg: Optional[float]


@dataclass
class _PostAttackRunCache:
    mission_plans: Dict[int, Dict[str, Any]]
    imp_by_aircraft: Dict[Tuple[int, int], Optional[Dict[str, Any]]]
    flight_paths: Dict[int, Optional[Dict[str, Any]]]
    sweep_progress: Optional[Dict[int, Dict[str, Any]]] = None
    coverage_progress: Optional[Dict[str, Any]] = None
    line_scan_progress: Optional[Dict[str, Any]] = None
    remaining_snapshot_geometry: Optional[Dict[Tuple[int, int], bool]] = None
    remaining_snapshot_completed: Optional[Dict[Tuple[int, int], bool]] = None
    remaining_snapshot_details: Optional[Dict[Tuple[int, int], Dict[str, Any]]] = None
    line_remaining_details: Optional[Dict[Tuple[int, int, Tuple[int, ...]], Dict[str, Any]]] = None
    deferred_compact_write_entries: List[Tuple[Path, Dict[str, Any]]] = field(default_factory=list)
    deferred_write_lock: threading.Lock = field(default_factory=threading.Lock)


def _write_or_defer_post_attack_json_batch(
    entries: List[Tuple[Path, Dict[str, Any]]],
    *,
    run_cache: Optional[_PostAttackRunCache],
) -> None:
    for path, payload in entries:
        if Path(path).parent.name == "FlightPath" and isinstance(payload, dict):
            normalize_flight_path_waypoint_altitudes_inplace(payload)
            normalize_flight_path_waypoint_speeds_inplace(payload)
    if run_cache is None:
        write_json_batch(
            entries,
            pretty=False,
            ensure_ascii=False,
            skip_if_unchanged=True,
        )
        return
    with run_cache.deferred_write_lock:
        run_cache.deferred_compact_write_entries.extend(entries)


def _validate_generated_post_attack_artifact_payloads(
    *,
    individual_mission_plans: Iterable[dict] = (),
    flight_paths: Iterable[dict] = (),
    scope: str,
    allow_existing_db_artifacts: bool,
    log: Optional[Callable[[str], None]] = None,
) -> dict[str, Any]:
    """Normalize only newly emitted paths, then apply the strict ICD checks."""

    path_rows = [row for row in flight_paths if isinstance(row, dict)]
    for payload in path_rows:
        normalize_flight_path_waypoint_altitudes_inplace(payload)
        normalize_flight_path_waypoint_speeds_inplace(payload)
    return validate_generated_artifact_payloads(
        individual_mission_plans=individual_mission_plans,
        flight_paths=path_rows,
        scope=scope,
        allow_existing_db_artifacts=allow_existing_db_artifacts,
        log=log,
    )


def _flush_post_attack_json_batches(
    run_cache: Optional[_PostAttackRunCache],
) -> List[Dict[str, Any]]:
    if run_cache is None:
        return []
    with run_cache.deferred_write_lock:
        entries = list(run_cache.deferred_compact_write_entries)
        run_cache.deferred_compact_write_entries.clear()
    if not entries:
        return []
    return write_json_batch(
        entries,
        pretty=False,
        ensure_ascii=False,
        skip_if_unchanged=True,
    )


_POST_ATTACK_PRIOR_REJOIN_HELPER_DIFFERENCES = {
    "state_store": "post-attack clears attack_tracking_state; prior-post-rejoin clears prior assignment state.",
    "slot_release": "post-attack may release manned attack slots after tracking clear; prior-post-rejoin has no attack slot release.",
    "delivery": "post-attack forces direct 0301+0903 and suppresses 0702 fallback; prior direct delivery may keep 0702 fallback.",
    "return_only": "post-attack writes tracking return-only packages with terminal hold before clearing tracking assignments.",
}

_POST_ATTACK_SNAPSHOT_CARRY_EXECUTOR = concurrent.futures.ThreadPoolExecutor(
    max_workers=1,
    thread_name_prefix="PostAttackSnapshotCarry",
)


def _queue_post_attack_snapshot_carry(
    source_plan_id: int,
    target_plan_id: int,
    *,
    reason: str,
    area_ownership_target_input_ids: Iterable[int] = (),
) -> concurrent.futures.Future:
    return _POST_ATTACK_SNAPSHOT_CARRY_EXECUTOR.submit(
        mission_area_replan_store.carry_forward_snapshot,
        int(source_plan_id),
        int(target_plan_id),
        reason=str(reason),
        area_ownership_target_input_ids=tuple(int(value) for value in area_ownership_target_input_ids),
    )


def warm_post_attack_rejoin_pipeline() -> Dict[str, Any]:
    return {
        "attack_tracking_assignments": len(list_active_tracking_assignments()),
        "agent_snapshot_available": bool(agent_status_snapshot.load_agent_status_snapshot()),
    }


def _post_attack_rejoin_enabled() -> bool:
    return bool(get_replan_toggle("post_attack_rejoin", True))


def _post_attack_rejoin_config() -> Dict[str, Any]:
    return dict(get_post_attack_rejoin_settings() or {})


def _commit_closed_tracking_assignments(
    aircraft_ids: Any,
    *,
    emit: LogCallback,
    completion_context: str,
) -> Tuple[Set[int], Set[int]]:
    """Deactivate tracking state after an attack-close event is handled.

    A destroyed 0402 can arrive after an operator has already reassigned the
    tracking UAV. In that case no MissionPlan update is needed, but the stale
    tracking assignment must still be closed so the same persistent destroyed
    state cannot enqueue another post-attack replan.
    """

    normalized_ids = {
        int(aircraft_id)
        for aircraft_id in (_to_int(value) for value in (aircraft_ids or []))
        if aircraft_id is not None and int(aircraft_id) > 0
    }
    if not normalized_ids:
        return set(), set()

    try:
        clear_tracking_assignments(sorted(normalized_ids))
    except Exception as exc:
        emit(
            f"Tracking assignment clear failed {completion_context} "
            f"(aircraft={sorted(normalized_ids)}): {exc}"
        )
        return set(), set(normalized_ids)

    emit(
        f"Tracking assignments cleared {completion_context} -> "
        f"aircraft={sorted(normalized_ids)}."
    )
    return set(normalized_ids), set()


def _watcher_uav_ids() -> Set[int]:
    raw = (get_target_detection_settings() or {}).get("watcher_uav_ids") or [4, 5, 6]
    out: Set[int] = set()
    for item in raw:
        try:
            value = int(item)
        except Exception:
            continue
        if value > 0:
            out.add(int(value))
    return out or {4, 5, 6}


def _attack_manned_ids() -> List[int]:
    raw = (get_target_detection_settings() or {}).get("attack_manned_ids") or [2, 3]
    out: List[int] = []
    for item in raw:
        value = _to_int(item)
        if value is None or value <= 0 or value in out:
            continue
        out.append(int(value))
    return out or [2, 3]


def _extract_input_package_id(
    ctx: Dict[str, Any],
    detail: Dict[str, Any],
    plan_data: Optional[Dict[str, Any]] = None,
) -> Optional[int]:
    for value in (
        detail.get("inputMissionPackageID"),
        detail.get("inputMissionPackageId"),
        ctx.get("inputMissionPackageID"),
        ctx.get("inputMissionPackageId"),
        ctx.get("input_mission_package_id"),
        (plan_data or {}).get("inputMissionPackageID") if isinstance(plan_data, dict) else None,
        (plan_data or {}).get("inputMissionPackageId") if isinstance(plan_data, dict) else None,
    ):
        input_package_id = _to_int(value)
        if input_package_id is not None and input_package_id > 0:
            return int(input_package_id)
    return None


def _manned_aircraft_still_attacking(current_plan_id: Optional[int]) -> set[int]:
    """Manned aircraft whose current plan still carries an attack waypoint.

    Occupancy is a property of the plan, not of the bookkeeping: an aircraft is
    busy only while it actually holds an attack.  Once a post-attack replan has
    swapped the attack out for a conceal-and-hold leg, that aircraft is free for
    the next target even though other aircraft are still engaged.
    """

    plan_id = _to_int(current_plan_id)
    if plan_id is None or plan_id <= 0:
        return set()
    manned_ids = set(_attack_manned_ids())
    busy: set[int] = set()
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(plan_id)}.json")
        plan = json.loads(Path(plan_path).read_text(encoding="utf-8"))
    except Exception:
        return set()
    for aircraft in (plan or {}).get("aircraftList") or []:
        if not isinstance(aircraft, dict):
            continue
        aircraft_id = _to_int(aircraft.get("aircraftID"))
        if aircraft_id is None or int(aircraft_id) not in manned_ids:
            continue
        mission_list = aircraft.get("individualMissionList")
        if not isinstance(mission_list, list):
            imp_id = _to_int(aircraft.get("individualMissionPackageID"))
            if imp_id is None or imp_id <= 0:
                continue
            try:
                imp_file = db_paths.get_db_subpath(
                    "IndividualMissionPlan", f"{int(imp_id)}.json"
                )
                imp_payload = json.loads(Path(imp_file).read_text(encoding="utf-8"))
                mission_list = (imp_payload or {}).get("individualMissionList")
            except Exception:
                continue
        for mission in mission_list or []:
            if not isinstance(mission, dict):
                continue
            path_id = _to_int(mission.get("pathID"))
            if path_id is None or path_id <= 0:
                continue
            try:
                path_file = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
                payload = json.loads(Path(path_file).read_text(encoding="utf-8"))
            except Exception:
                continue
            if _flight_path_attack_target_ids(payload):
                busy.add(int(aircraft_id))
                break
            if int(aircraft_id) in busy:
                break
    return busy


def _release_attack_slots_if_tracking_closed(
    *,
    input_package_id: Optional[int],
    current_plan_id: Optional[int],
    emit: LogCallback,
) -> List[int]:
    if input_package_id is None or input_package_id <= 0:
        emit("[ATTACK-SLOT] release skipped: inputMissionPackageID unavailable.")
        return []

    # Reaching a post-attack rejoin means the attack has been flown, so the slot
    # has to come back.  Only an aircraft that still holds an attack of its own
    # is kept occupied; another aircraft's tracking never blocks this, which is
    # what used to deadlock the package once both manned slots were marked used.
    busy = _manned_aircraft_still_attacking(current_plan_id)
    releasable = [aircraft_id for aircraft_id in _attack_manned_ids() if aircraft_id not in busy]
    if busy:
        emit(
            "[ATTACK-SLOT] manned aircraft still carrying an attack: "
            f"{sorted(busy)}; releasing the rest."
        )
    if not releasable:
        emit(
            "[ATTACK-SLOT] every manned attack slot is still flying its own attack "
            f"(inputMissionPackageID={int(input_package_id)})."
        )
        return []

    released = release_manned_used(int(input_package_id), releasable)
    if released:
        emit(
            "[ATTACK-SLOT] released manned attack slots after the attack was flown -> "
            f"inputMissionPackageID={int(input_package_id)}, aircraft={released}."
        )
    else:
        emit(
            "[ATTACK-SLOT] no occupied manned attack slot to release "
            f"(inputMissionPackageID={int(input_package_id)})."
        )
    return released


def run_post_attack_rejoin_pipeline(
    ctx: Dict[str, Any],
    detail: Dict[str, Any],
    reason: str,
    *,
    log: Optional[LogCallback] = None,
) -> PostAttackRejoinPipelineResult:
    emit = log or (lambda _msg: None)
    detail = dict(detail or {})
    transaction_id = new_replan_transaction_id("post-attack")
    phase_timer = PipelinePhaseTimer(
        pipeline="post_attack_rejoin",
        replan_transaction_id=transaction_id,
        emit_events=True,
    )
    now_ms = _now_timestamp_ms()
    log_messages: List[str] = []
    id_reservation_summaries: List[Dict[str, Any]] = []

    def _emit(message: str) -> None:
        log_messages.append(str(message))
        emit(f"[POSTATTACK] {message}")

    requested_plan_id = _resolve_requested_plan_id(ctx)
    current_plan_id = _to_int(
        detail.get("currentMissionPlanID")
        or detail.get("sourceMissionPlanID")
        or ctx.get("currentMissionPlanID")
        or ctx.get("sourceMissionPlanID")
        or getattr(ctx, "_last_mission_plan_id", None)
    )
    trigger = str(detail.get("trigger") or "").strip()
    trigger_type = str(detail.get("triggerType") or "").strip()
    target_id = _to_int(detail.get("targetID") or detail.get("targetId"))

    result_payload: Dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "reason": str(reason or ctx.get("reason") or ""),
        "trigger": trigger,
        "triggerType": trigger_type,
        "currentPlanID": current_plan_id,
        "targetID": target_id,
        "evaluations": [],
        "logMessages": log_messages,
        "replanTransactionId": transaction_id,
        "idReservationSummaries": id_reservation_summaries,
    }
    config = _post_attack_rejoin_config()

    def _finish_with_timing(**kwargs: Any) -> PostAttackRejoinPipelineResult:
        result_payload["timingMs"] = phase_timer.snapshot(include_total=True)
        summary = kwargs.get("summary")
        if isinstance(summary, dict):
            summary.setdefault("timingMs", dict(result_payload.get("timingMs") or {}))
            summary.setdefault("replanTransactionId", transaction_id)
        return _finish_result(**kwargs)

    if trigger != "0402" or trigger_type != "attackClosedDestroyed":
        _emit("ignored: detail is not an attack-close destroyed trigger.")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "not_attack_close_trigger"},
            result_payload=result_payload,
        )
    if not _post_attack_rejoin_enabled():
        _emit("skipped: post-attack rejoin is disabled in monitoring settings.")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "post_attack_rejoin_disabled"},
            result_payload=result_payload,
        )
    if current_plan_id is None or current_plan_id <= 0 or target_id is None or target_id <= 0:
        _emit("skipped: missing currentMissionPlanID/targetID in closure detail.")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "missing_trigger_identifiers"},
            result_payload=result_payload,
        )
    phase_timer.mark("detail_validation")

    assignments = _match_tracking_assignments(
        current_plan_id=int(current_plan_id),
        target_id=int(target_id),
        watcher_id=_to_int(detail.get("watcherID")),
        preferred_aircraft_ids=detail.get("trackingAircraftIDList"),
        emit=_emit,
    )
    if not assignments:
        assignments = _recover_tracking_assignments_from_current_plan(
            current_plan_id=int(current_plan_id),
            target_id=int(target_id),
            watcher_id=_to_int(detail.get("watcherID")),
            preferred_aircraft_ids=detail.get("trackingAircraftIDList"),
            emit=_emit,
        )
    if not assignments:
        _emit(
            f"skipped: no active tracking assignment matched targetID={target_id} "
            f"on attackPlan={current_plan_id}."
        )
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "tracking_assignment_not_found"},
            result_payload=result_payload,
        )
    phase_timer.mark("tracking_state_load")

    matched_rebound_aircraft_ids: Set[int] = set()
    legacy_attack_plan_ids = sorted(
        {
            int(plan_id)
            for plan_id in (_to_int(item.get("attack_plan_id")) for item in assignments)
            if plan_id is not None and int(plan_id) > 0 and int(plan_id) != int(current_plan_id)
        }
    )
    for old_attack_plan_id in legacy_attack_plan_ids:
        rebound_ids = rebind_tracking_assignments_to_plan(
            old_attack_plan_id=int(old_attack_plan_id),
            new_attack_plan_id=int(current_plan_id),
        )
        if not rebound_ids:
            continue
        matched_rebound_aircraft_ids.update(int(aid) for aid in rebound_ids)
        for assignment in assignments:
            if _to_int(assignment.get("attack_plan_id")) == int(old_attack_plan_id):
                assignment["attack_plan_id"] = int(current_plan_id)
        _emit(
            "Tracking assignment plan binding recovered from lineage -> "
            f"{int(old_attack_plan_id)} -> {int(current_plan_id)} "
            f"(aircraft={sorted(int(aid) for aid in rebound_ids)})."
        )

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
    input_package_id = _extract_input_package_id(ctx, detail, plan_data)
    run_cache = _PostAttackRunCache(
        mission_plans={int(current_plan_id): plan_data},
        imp_by_aircraft={},
        flight_paths={},
        remaining_snapshot_geometry={},
        remaining_snapshot_completed={},
        remaining_snapshot_details={},
        line_remaining_details={},
    )

    snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    agent_state_map = _index_agent_states(
        snapshot.get("agent_states") or [],
        snapshot.get("last_nonzero_waypoint_by_aircraft"),
    )
    new_plan_id = int(requested_plan_id or _allocate_fresh_plan_id())
    new_plan_data = deepcopy(plan_data)
    new_plan_data["missionPlanID"] = int(new_plan_id)
    new_plan_data["timestamp"] = int(now_ms)
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = int(now_ms)

    generated_imp_ids: Set[int] = set()
    generated_path_ids: Set[int] = set()
    updated_aircraft_ids: Set[int] = set()
    cleared_aircraft_ids: Set[int] = set()
    pending_clear_aircraft_ids: Set[int] = set()
    rebound_aircraft_ids: Set[int] = set(matched_rebound_aircraft_ids)
    group_summaries: List[Dict[str, Any]] = []

    assignments_by_input: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    skipped_assignments: List[Dict[str, Any]] = []
    for assignment in assignments:
        current_input_id = _resolve_assignment_input_mission_id(assignment)
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
        assignments_by_input[int(current_input_id)].append(normalized)

    if skipped_assignments:
        result_payload["skippedAssignments"] = skipped_assignments

    def _clear_group_tracking_assignments(
        grouped_assignments: List[Dict[str, Any]],
        *,
        skip_reason: str,
    ) -> List[int]:
        queued_ids: Set[int] = set()
        for grouped in grouped_assignments:
            aircraft_id = _to_int(grouped.get("aircraft_id"))
            if aircraft_id is None or aircraft_id <= 0:
                continue
            pending_clear_aircraft_ids.add(int(aircraft_id))
            queued_ids.add(int(aircraft_id))
        if queued_ids:
            _emit(
                "Tracking assignment clear queued until post-attack handling completes -> "
                f"aircraft={sorted(queued_ids)} (reason={skip_reason or 'rejoin_not_needed'})."
            )
        return sorted(int(aid) for aid in queued_ids)

    for current_input_id, group_assignments in assignments_by_input.items():
        returning_lah_updates: List[Dict[str, Any]] = []
        returning_lah_ids = _find_returning_manned_attack_aircraft_ids(
            current_plan_id=int(current_plan_id),
            current_input_id=int(current_input_id),
            target_id=int(target_id),
            plan_data=plan_data,
            run_cache=run_cache,
        )

        def _build_returning_lah_update(aircraft_id: int) -> Optional[Dict[str, Any]]:
            try:
                return _build_post_attack_lah_resume_update(
                    source_plan_id=int(current_plan_id),
                    current_input_id=int(current_input_id),
                    target_id=int(target_id),
                    aircraft_id=int(aircraft_id),
                    current_state=agent_state_map.get(int(aircraft_id)) or {},
                    now_ms=int(now_ms),
                    emit=_emit,
                    log_prefix="[POSTATTACK][LAH]",
                    run_cache=run_cache,
                )
            except Exception as exc:
                _emit(
                    "[POSTATTACK][LAH] parallel resume generation failed "
                    f"(aircraft={aircraft_id}, error={exc!r})."
                )
                return None

        returning_lah_executor: Optional[concurrent.futures.ThreadPoolExecutor] = None
        returning_lah_futures: Dict[int, concurrent.futures.Future] = {}
        if returning_lah_ids:
            returning_lah_executor = concurrent.futures.ThreadPoolExecutor(
                max_workers=min(3, len(returning_lah_ids)),
                thread_name_prefix="PostAttackLAH",
            )
            active_source_cache = get_active_source_artifact_cache()
            if active_source_cache is not None:
                returning_lah_futures = {
                    int(aircraft_id): returning_lah_executor.submit(
                        call_with_source_artifact_cache,
                        active_source_cache,
                        _build_returning_lah_update,
                        int(aircraft_id),
                    )
                    for aircraft_id in returning_lah_ids
                }
            else:
                returning_lah_futures = {
                    int(aircraft_id): returning_lah_executor.submit(
                        _build_returning_lah_update,
                        int(aircraft_id),
                    )
                    for aircraft_id in returning_lah_ids
                }

        lah_updates_collected = False

        def _collect_returning_lah_updates() -> None:
            nonlocal lah_updates_collected, returning_lah_executor
            if lah_updates_collected:
                return
            lah_updates_collected = True
            try:
                for aircraft_id in returning_lah_ids:
                    future = returning_lah_futures.get(int(aircraft_id))
                    lah_update = future.result() if future is not None else None
                    if not isinstance(lah_update, dict):
                        continue
                    if bool(lah_update.get("preserveExistingPackage")):
                        # A second/third attack is already queued in this exact
                        # LAH package.  Rebuilding the package here would assign
                        # new mission/path/WP IDs and reset that attack's
                        # execution progress.  The completed attack prefix and
                        # its certified descent remain harmless; keep the whole
                        # package byte-for-byte until the final queued attack
                        # closes.
                        returning_lah_updates.append(dict(lah_update))
                        _emit(
                            "[POSTATTACK][LAH][CONTINUITY] Existing sequential attack "
                            f"package retained (aircraft={int(aircraft_id)}, "
                            f"remainingTargets={lah_update.get('remainingAttackTargetIDs') or []})."
                        )
                        continue
                    if _update_plan_aircraft_entry(
                        new_plan_data,
                        int(aircraft_id),
                        int(lah_update["individualMissionPackageID"]),
                    ):
                        updated_aircraft_ids.add(int(aircraft_id))
                        generated_imp_ids.add(int(lah_update["individualMissionPackageID"]))
                        generated_path_ids.update(
                            int(path_id)
                            for path_id in (lah_update.get("generatedPathIDs") or [])
                            if _to_int(path_id) is not None
                        )
                        returning_lah_updates.append(dict(lah_update))
                        _extend_reservation_summaries(
                            id_reservation_summaries,
                            lah_update.get("reservationSummaries"),
                        )
                if returning_lah_updates:
                    evaluation["returning_lah_updates"] = returning_lah_updates
            finally:
                if returning_lah_executor is not None:
                    returning_lah_executor.shutdown(wait=True, cancel_futures=False)
                    returning_lah_executor = None

        # LAH package generation runs while threshold/group evaluation and,
        # when needed, the collaborative UAV replan are being prepared.
        evaluation = _evaluate_rejoin_group(
            current_plan_id=int(current_plan_id),
            current_input_id=int(current_input_id),
            group_assignments=group_assignments,
            agent_state_map=agent_state_map,
            config=config,
            emit=_emit,
            run_cache=run_cache,
        )
        group_summaries.append(evaluation)
        result_payload["evaluations"].append(evaluation)

        if not bool(evaluation.get("replan_needed")):
            _collect_returning_lah_updates()
            tracking_release_aircraft_ids: Set[int] = set()
            active_only_updated_aircraft_ids: Set[int] = set()
            tracking_source_plan_id = _resolve_group_source_plan_id(
                group_assignments,
                fallback_plan_id=int(current_plan_id),
            )
            # Assignment source IDs are provenance and may point at the plan in
            # which tracking began.  Geometry, phase and suffix decisions must
            # use the currently applied plan; otherwise an older AREA/LINE can
            # be resurrected after one or more intervening attack replans.
            planning_source_plan_id = int(current_plan_id)
            if int(tracking_source_plan_id) != int(planning_source_plan_id):
                _emit(
                    "[POSTATTACK][SOURCE] Current applied plan overrides historical "
                    f"tracking source for resume decisions ({tracking_source_plan_id} -> "
                    f"{planning_source_plan_id}, inputMissionID={current_input_id})."
                )
            skip_reason = str(evaluation.get("skip_reason") or "")
            allow_active_only_collab = _allow_post_attack_active_only_replan(skip_reason)
            self_reliance_phase = _source_type2_self_reliance_phase(
                source_plan_id=int(planning_source_plan_id),
                input_mission_id=int(current_input_id),
            )
            force_type2_individual_suffix_refresh = (
                _requires_type2_individual_suffix_refresh(
                    skip_reason,
                    self_reliance_phase,
                )
            )
            if (
                allow_active_only_collab
                and _has_remaining_snapshot_geometry(
                    int(planning_source_plan_id),
                    int(current_input_id),
                    run_cache=run_cache,
                )
            ):
                active_only_reservation_start = len(id_reservation_summaries)
                active_only_collab = _prepare_post_attack_active_only_remaining_update(
                    source_plan_id=int(planning_source_plan_id),
                    current_input_id=int(current_input_id),
                    evaluation=evaluation,
                    group_assignments=[dict(item) for item in group_assignments],
                    agent_state_map=agent_state_map,
                    now_ms=int(now_ms),
                    emit=_emit,
                    log_prefix="[POSTATTACK][ACTIVEONLY]",
                    run_cache=run_cache,
                    reservation_summaries=id_reservation_summaries,
                )
                if active_only_collab is not None:
                    active_only_reservation_blocks = [
                        dict(item)
                        for item in id_reservation_summaries[active_only_reservation_start:]
                        if isinstance(item, dict)
                    ]
                    evaluation["active_only_remaining_update"] = {
                        "applied": True,
                        "replacement_aircraft_ids": sorted(
                            int(aid) for aid in active_only_collab.replacement_aircraft_ids
                        ),
                        "generated_path_ids": sorted(
                            int(pid) for pid in active_only_collab.generated_path_ids
                        ),
                        "planner_workflow": str(active_only_collab.planner_workflow or ""),
                    }
                    if active_only_reservation_blocks:
                        evaluation["active_only_remaining_update"]["idReservation"] = {
                            "blocks": active_only_reservation_blocks,
                        }
                    for aircraft_id, imp_id in active_only_collab.aircraft_imp_ids.items():
                        if _update_plan_aircraft_entry(new_plan_data, int(aircraft_id), int(imp_id)):
                            generated_imp_ids.add(int(imp_id))
                            updated_aircraft_ids.add(int(aircraft_id))
                            active_only_updated_aircraft_ids.add(int(aircraft_id))
                    generated_path_ids.update(
                        int(path_id) for path_id in active_only_collab.generated_path_ids
                    )
                else:
                    evaluation["active_only_remaining_update"] = {
                        "applied": False,
                        "reason": "active_only_remaining_unavailable",
                    }
            elif not allow_active_only_collab:
                evaluation["active_only_remaining_update"] = {
                    "applied": False,
                    "reason": (
                        "type2_branch_line_individual_suffix_refresh"
                        if force_type2_individual_suffix_refresh
                        else f"{skip_reason or 'rejoin_not_needed'}_preserves_existing_active_assignments"
                    ),
                }
            else:
                evaluation["active_only_remaining_update"] = {
                    "applied": False,
                    "reason": "remaining_snapshot_unavailable",
                }

            active_suffix_candidate_ids = {
                int(aid)
                for aid in (evaluation.get("active_aircraft_ids") or [])
                if _to_int(aid) is not None and int(aid) > 0
            }
            active_suffix_candidate_ids.difference_update(active_only_updated_aircraft_ids)
            active_progress_by_aircraft = evaluation.get("active_progress_by_aircraft")
            active_progress_by_aircraft = (
                active_progress_by_aircraft if isinstance(active_progress_by_aircraft, dict) else {}
            )
            if force_type2_individual_suffix_refresh:
                # The Type2 owner set is immutable, but each owner's currently
                # executable branch still needs to be cut at its own capture
                # progress.  Do not trust top-level waypoint isDone here: SIM
                # can mark the carrier WPs done while lineSearch still has
                # substantial unphotographed geometry.
                evaluation["active_path_resume_skipped_reason"] = None
                _emit(
                    "[POSTATTACK][TYPE2-LINE-SUFFIX] preserving branch ownership while "
                    "rebuilding each active UAV from its individual remaining LINE geometry "
                    f"(inputMissionID={current_input_id}, "
                    f"aircraft={sorted(active_suffix_candidate_ids)})."
                )
            elif not allow_active_only_collab:
                completed_progress_aircraft_ids = {
                    int(aircraft_id)
                    for aircraft_id in active_suffix_candidate_ids
                    if _active_progress_is_complete(
                        active_progress_by_aircraft.get(aircraft_id)
                        if aircraft_id in active_progress_by_aircraft
                        else active_progress_by_aircraft.get(str(aircraft_id))
                    )
                }
                completed_path_aircraft_ids = {
                    int(aircraft_id)
                    for aircraft_id in active_suffix_candidate_ids
                    if _active_current_input_path_all_done(
                        source_plan_id=int(current_plan_id),
                        current_input_id=int(current_input_id),
                        aircraft_id=int(aircraft_id),
                        run_cache=run_cache,
                    )
                }
                completed_aircraft_ids = (
                    completed_progress_aircraft_ids | completed_path_aircraft_ids
                )
                preserved_active_aircraft_ids = (
                    set(active_suffix_candidate_ids) - completed_aircraft_ids
                )
                if preserved_active_aircraft_ids:
                    evaluation["active_path_resume_updates"] = []
                    evaluation["active_path_resume_skipped_reason"] = (
                        f"{skip_reason}_preserve_existing_active_assignments"
                    )
                    _emit(
                        "[POSTATTACK][ACTIVE-SUFFIX] preserving existing active UAV assignments "
                        "because the rejoin skip reason does not provide safe current-plan "
                        "remaining geometry "
                        f"(inputMissionID={current_input_id}, reason={skip_reason}, "
                        f"aircraft={sorted(preserved_active_aircraft_ids)})."
                    )
                if completed_progress_aircraft_ids:
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] 100% LINE progress will create a "
                        "completion-boundary loiter instead of reviving the sweep "
                        f"(inputMissionID={current_input_id}, "
                        f"aircraft={sorted(completed_progress_aircraft_ids)})."
                    )
                completed_by_path_only_ids = (
                    completed_path_aircraft_ids - completed_progress_aircraft_ids
                )
                if completed_by_path_only_ids:
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] completed current-input paths will be "
                        "replaced with fresh completion-boundary loiters even though the "
                        "coverage progress sample is unavailable "
                        f"(inputMissionID={current_input_id}, "
                        f"aircraft={sorted(completed_by_path_only_ids)})."
                    )
                active_suffix_candidate_ids.intersection_update(
                    completed_aircraft_ids
                )
            active_completed_updates: List[Dict[str, Any]] = []
            active_completed_failed_aircraft_ids: Set[int] = set()
            active_done_hold_seconds = _estimate_active_done_hold_seconds(
                current_plan_id=int(current_plan_id),
                current_input_id=int(current_input_id),
                evaluation=evaluation,
                group_assignments=group_assignments,
                agent_state_map=agent_state_map,
                config=config,
                run_cache=run_cache,
            )
            pending_area_pass_reassignment = _area_pass_reassignment_pending(
                source_plan_id=int(planning_source_plan_id),
                current_input_id=int(current_input_id),
                run_cache=run_cache,
            )
            if pending_area_pass_reassignment:
                _emit(
                    "[POSTATTACK][ACTIVE-DONE] pending Area OUT/RETURN work detected; "
                    "completed UAVs will hold for reassignment instead of advancing "
                    f"to the next input mission (inputMissionID={current_input_id})."
                )
            progress_only_active_aircraft_ids: Set[int] = set()
            hold_group_until_next_collab = _block_post_attack_followups_until_next_collab(
                # A missing remaining-geometry snapshot means "unknown", not
                # "completed".  Existing active aircraft keep their current
                # assignment and a released tracker executes its recorded return
                # path.  Only authoritative completion detected below may turn
                # this into a collaboration-boundary hold.
                current_mission_completed=(
                    str(skip_reason or "") == "remaining_snapshot_completed"
                ),
                pending_area_pass_reassignment=bool(pending_area_pass_reassignment),
            )
            for aircraft_id in sorted(active_suffix_candidate_ids):
                if force_type2_individual_suffix_refresh:
                    # Exact sweep/LINE progress is applied by
                    # _build_other_uav_resume_package below.  The carrier WP
                    # completion flags are not authoritative for Type2 scan
                    # geometry and must not convert this branch into a hold.
                    continue
                state = agent_state_map.get(int(aircraft_id)) or {}
                progress_percent = _to_int(
                    active_progress_by_aircraft.get(aircraft_id)
                    if aircraft_id in active_progress_by_aircraft
                    else active_progress_by_aircraft.get(str(aircraft_id))
                )
                completed_by_progress = _active_progress_is_complete(progress_percent)
                path_all_done = _active_current_input_path_all_done(
                    source_plan_id=int(current_plan_id),
                    current_input_id=int(current_input_id),
                    aircraft_id=int(aircraft_id),
                    run_cache=run_cache,
                )
                completed_by_on_mission = False
                if progress_percent is not None and int(progress_percent) >= 100:
                    completed_by_on_mission = _active_current_input_on_mission_complete(
                        source_plan_id=int(current_plan_id),
                        current_input_id=int(current_input_id),
                        aircraft_id=int(aircraft_id),
                        state=state,
                        run_cache=run_cache,
                    )
                if completed_by_on_mission and not path_all_done and not completed_by_progress:
                    progress_only_active_aircraft_ids.add(int(aircraft_id))
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] active mission reported onMission=2 "
                        "but executable waypoints are not all done; preserving active suffix "
                        f"instead of replacing capture geometry (aircraft={aircraft_id})."
                    )
                    completed_by_on_mission = False
                if not path_all_done and not completed_by_on_mission and not completed_by_progress:
                    continue
                if completed_by_progress or completed_by_on_mission:
                    hold_group_until_next_collab = True
                if completed_by_progress:
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] LINE coverage progress reached 100%; "
                        "creating completion-boundary loiter instead of advancing directly "
                        f"to the next mission (aircraft={aircraft_id})."
                    )
                elif completed_by_on_mission:
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] active imaging mission reports onMission=2; "
                        "using completion hold instead of reviving remaining sweep "
                        f"(aircraft={aircraft_id}, "
                        f"progress={progress_percent if progress_percent is not None else 'n/a'}%)."
                    )
                preserve_current_active_mission = bool(
                    path_all_done
                    and not completed_by_on_mission
                    and progress_percent is not None
                    and int(progress_percent) < 100
                )
                if preserve_current_active_mission:
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] active path waypoints already done; "
                        "coverage progress is below 100%, so current imaging mission will be "
                        "kept after the post-attack boundary marker "
                        f"(aircraft={aircraft_id}, progress={int(progress_percent)}%)."
                    )
                elif path_all_done and progress_percent is None:
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] active path waypoints already done with no "
                        "coverage progress sample; using done/follow-up update "
                        f"(aircraft={aircraft_id}, progress=n/a%)."
                    )
                update = _build_post_attack_active_done_followup_update(
                    source_plan_id=int(current_plan_id),
                    current_input_id=int(current_input_id),
                    aircraft_id=int(aircraft_id),
                    hold_seconds=int(active_done_hold_seconds),
                    now_ms=int(now_ms),
                    emit=_emit,
                    log_prefix="[POSTATTACK][ACTIVE-DONE]",
                    run_cache=run_cache,
                    preserve_current_mission=bool(preserve_current_active_mission),
                    include_completion_boundary_hold=_include_active_completion_boundary_hold(
                        progress_percent,
                        preserve_current_mission=bool(preserve_current_active_mission),
                        pending_area_pass_reassignment=bool(
                            pending_area_pass_reassignment
                        ),
                    ),
                    block_follow_up_until_reassignment=bool(
                        hold_group_until_next_collab
                    ),
                )
                if not isinstance(update, dict):
                    active_completed_failed_aircraft_ids.add(int(aircraft_id))
                    continue
                new_imp_id = _to_int(update.get("individualMissionPackageID"))
                if new_imp_id is None or new_imp_id <= 0:
                    active_completed_failed_aircraft_ids.add(int(aircraft_id))
                    continue
                if not _update_plan_aircraft_entry(new_plan_data, int(aircraft_id), int(new_imp_id)):
                    active_completed_failed_aircraft_ids.add(int(aircraft_id))
                    continue
                active_completed_updates.append(dict(update))
                active_completed_updates[-1]["completedByProgress100"] = bool(
                    completed_by_progress
                )
                active_completed_updates[-1]["completedByOnMission2"] = bool(completed_by_on_mission)
                active_completed_updates[-1]["preservedCurrentMission"] = bool(
                    preserve_current_active_mission
                )
                _extend_reservation_summaries(
                    id_reservation_summaries,
                    update.get("reservationSummaries"),
                )
                active_suffix_candidate_ids.discard(int(aircraft_id))
                updated_aircraft_ids.add(int(aircraft_id))
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

            if active_suffix_candidate_ids:
                sweep_progress = _load_sweep_progress_safe(run_cache=run_cache)
                active_path_resume_updates: List[Dict[str, Any]] = []
                active_path_resume_failed_aircraft_ids: Set[int] = set()
                for aircraft_id in sorted(active_suffix_candidate_ids):
                    state = agent_state_map.get(int(aircraft_id)) or {}
                    aircraft_sweep_progress = sweep_progress
                    if int(aircraft_id) in progress_only_active_aircraft_ids:
                        ignored_path_ids = _active_current_input_path_ids(
                            source_plan_id=int(current_plan_id),
                            current_input_id=int(current_input_id),
                            aircraft_id=int(aircraft_id),
                            run_cache=run_cache,
                        )
                        if ignored_path_ids:
                            aircraft_sweep_progress = {
                                int(path_id): dict(entry or {})
                                for path_id, entry in sweep_progress.items()
                                if int(path_id) not in ignored_path_ids
                            }
                            _emit(
                                "[POSTATTACK][ACTIVE-SUFFIX] ignoring time-based 100% sweep progress "
                                "because active completion was not confirmed "
                                f"(aircraft={aircraft_id}, pathIDs={sorted(ignored_path_ids)})."
                            )
                    line_remaining_detail = _load_line_scan_aircraft_remaining_detail_cached(
                        source_plan_id=int(current_plan_id),
                        input_mission_id=int(current_input_id),
                        aircraft_ids=[int(aircraft_id)],
                        source_detail={},
                        allow_latest_plan_fallback=True,
                        run_cache=run_cache,
                    )
                    line_remaining_completed = False
                    if has_line_remaining_geometry(line_remaining_detail):
                        _emit(
                            "[POSTATTACK][ACTIVE-SUFFIX] applying row-level LINE remaining "
                            f"(aircraft={aircraft_id}, fragments="
                            f"{line_remaining_detail.get('lineRemainingFragmentCount')}, "
                            f"fallback={bool(line_remaining_detail.get('lineScanSourcePlanFallback'))})."
                        )
                    else:
                        line_remaining_completed = bool(
                            isinstance(line_remaining_detail, dict)
                            and line_remaining_detail.get("lineRemainingCompleted")
                        )
                        if line_remaining_completed and force_type2_individual_suffix_refresh:
                            current_input_path_ids = _active_current_input_path_ids(
                                source_plan_id=int(current_plan_id),
                                current_input_id=int(current_input_id),
                                aircraft_id=int(aircraft_id),
                                run_cache=run_cache,
                            )
                            remaining_sweep_path_ids = sorted(
                                int(path_id)
                                for path_id in current_input_path_ids
                                if _sweep_progress_entry_has_remaining_imaging(
                                    aircraft_sweep_progress.get(int(path_id))
                                )
                            )
                            if remaining_sweep_path_ids:
                                line_remaining_completed = False
                                line_remaining_detail = None
                                _emit(
                                    "[POSTATTACK][ACTIVE-SUFFIX] stale LINE-complete snapshot "
                                    "overridden by exact path sweep progress "
                                    f"(aircraft={aircraft_id}, "
                                    f"pathIDs={remaining_sweep_path_ids})."
                                )
                        if line_remaining_completed:
                            _emit(
                                "[POSTATTACK][ACTIVE-SUFFIX] LINE progress has no remaining rows; "
                                "dropping completed current mission and preserving follow-ups "
                                f"(aircraft={aircraft_id}, inputMissionID={current_input_id}, "
                                f"fallback={bool(line_remaining_detail.get('lineScanSourcePlanFallback'))})."
                            )
                        line_remaining_detail = None
                    if line_remaining_completed:
                        update = _build_post_attack_active_done_followup_update(
                            source_plan_id=int(current_plan_id),
                            current_input_id=int(current_input_id),
                            aircraft_id=int(aircraft_id),
                            hold_seconds=int(active_done_hold_seconds),
                            now_ms=int(now_ms),
                            emit=_emit,
                            log_prefix="[POSTATTACK][ACTIVE-SUFFIX]",
                            run_cache=run_cache,
                            preserve_current_mission=False,
                            include_completion_boundary_hold=False,
                            block_follow_up_until_reassignment=False,
                        )
                        if isinstance(update, dict):
                            update["completedCurrentMissionDroppedByLineProgress"] = True
                    else:
                        update = _build_other_uav_resume_package(
                            source_plan_id=int(current_plan_id),
                            aircraft_id=int(aircraft_id),
                            current_waypoint_id=_to_int(state.get("current_waypoint_id")),
                            current_coord=_normalize_coordinate(state.get("coordinate")),
                            emit=_emit,
                            now_ms=int(now_ms),
                            sweep_progress=aircraft_sweep_progress,
                            clone_follow_up_artifacts=True,
                            drop_prefix_missions=True,
                            allow_first_mission_fallback=skip_reason != "active_group_progress_high",
                            include_done_reference_mission=False,
                            line_remaining_detail=line_remaining_detail,
                            log_prefix="[POSTATTACK][ACTIVE-SUFFIX]",
                        )
                    if not isinstance(update, dict):
                        active_path_resume_failed_aircraft_ids.add(int(aircraft_id))
                        continue
                    new_imp_id = _to_int(update.get("individualMissionPackageID"))
                    if new_imp_id is None or new_imp_id <= 0:
                        active_path_resume_failed_aircraft_ids.add(int(aircraft_id))
                        continue
                    if not _update_plan_aircraft_entry(new_plan_data, int(aircraft_id), int(new_imp_id)):
                        active_path_resume_failed_aircraft_ids.add(int(aircraft_id))
                        continue
                    path_ids: Set[int] = set()
                    resume_info = update.get("resume") if isinstance(update.get("resume"), dict) else {}
                    reported_generated_path_ids = (
                        update.get("generatedPathIDs")
                        if isinstance(update.get("generatedPathIDs"), list)
                        else []
                    )
                    for path_id in [
                        *reported_generated_path_ids,
                        resume_info.get("pathID"),
                        update.get("donePathID"),
                    ]:
                        normalized_path_id = _to_int(path_id)
                        if normalized_path_id is not None and normalized_path_id > 0:
                            path_ids.add(int(normalized_path_id))
                    needs_imp_path_scan = bool(
                        not path_ids
                        or (
                            update.get("followUpMissionCount")
                            and not reported_generated_path_ids
                        )
                    )
                    if needs_imp_path_scan:
                        try:
                            imp_payload = read_json_cached(
                                db_paths.get_db_subpath(
                                    "IndividualMissionPlan",
                                    f"{int(new_imp_id)}.json",
                                ),
                                kind="IndividualMissionPlan",
                            )
                            for mission in imp_payload.get("individualMissionList") or []:
                                if not isinstance(mission, dict):
                                    continue
                                path_id = _to_int(mission.get("pathID"))
                                if path_id is not None and path_id > 0:
                                    path_ids.add(int(path_id))
                        except Exception:
                            pass
                    update["generatedPathIDs"] = sorted(int(path_id) for path_id in path_ids)
                    if update.get("reservationSummaries"):
                        _extend_reservation_summaries(
                            id_reservation_summaries,
                            update.get("reservationSummaries"),
                        )
                    elif isinstance(update.get("reservedIds"), dict):
                        id_reservation_summaries.append(
                            {
                                "scope": "postAttackActiveSuffixResume",
                                "aircraftID": int(aircraft_id),
                                "reservedIds": dict(update.get("reservedIds") or {}),
                            }
                        )
                    active_path_resume_updates.append(dict(update))
                    updated_aircraft_ids.add(int(aircraft_id))
                    generated_imp_ids.add(int(new_imp_id))
                    generated_path_ids.update(int(path_id) for path_id in path_ids)
                if active_path_resume_updates:
                    evaluation["active_path_resume_updates"] = active_path_resume_updates
                if active_path_resume_failed_aircraft_ids:
                    evaluation["active_path_resume_failed_aircraft_ids"] = sorted(
                        int(aid) for aid in active_path_resume_failed_aircraft_ids
                    )

            tracking_release_updates: List[Dict[str, Any]] = []
            tracking_release_failed_aircraft_ids: Set[int] = set()
            for assignment in group_assignments:
                aircraft_id = _to_int(assignment.get("aircraft_id"))
                if aircraft_id is None or aircraft_id <= 0:
                    continue
                tracking_release = _build_post_attack_tracking_return_only_update(
                    attack_plan_id=int(current_plan_id),
                    current_input_id=int(current_input_id),
                    assignment=assignment,
                    current_state=agent_state_map.get(int(aircraft_id)) or {},
                    hold_seconds=int(active_done_hold_seconds),
                    now_ms=int(now_ms),
                    emit=_emit,
                    log_prefix="[POSTATTACK][TRACK-RETURN]",
                    run_cache=run_cache,
                    block_follow_up_until_reassignment=bool(
                        hold_group_until_next_collab
                    ),
                )
                if not isinstance(tracking_release, dict):
                    tracking_release_failed_aircraft_ids.add(int(aircraft_id))
                    continue
                new_imp_id = _to_int(tracking_release.get("individualMissionPackageID"))
                if new_imp_id is None or new_imp_id <= 0:
                    tracking_release_failed_aircraft_ids.add(int(aircraft_id))
                    continue
                if _update_plan_aircraft_entry(new_plan_data, int(aircraft_id), int(new_imp_id)):
                    updated_aircraft_ids.add(int(aircraft_id))
                    generated_imp_ids.add(int(new_imp_id))
                    generated_path_ids.update(
                        int(path_id)
                        for path_id in (tracking_release.get("generatedPathIDs") or [])
                        if _to_int(path_id) is not None
                    )
                    tracking_release_updates.append(dict(tracking_release))
                    _extend_reservation_summaries(
                        id_reservation_summaries,
                        tracking_release.get("reservationSummaries"),
                    )
                    tracking_release_aircraft_ids.add(int(aircraft_id))
                    pending_clear_aircraft_ids.add(int(aircraft_id))
                else:
                    tracking_release_failed_aircraft_ids.add(int(aircraft_id))
            if tracking_release_updates:
                evaluation["tracking_release_updates"] = tracking_release_updates
            if tracking_release_failed_aircraft_ids:
                evaluation["tracking_release_failed_aircraft_ids"] = sorted(
                    int(aid) for aid in tracking_release_failed_aircraft_ids
                )

            remaining_group_assignments = [
                dict(item)
                for item in group_assignments
                if _to_int(item.get("aircraft_id")) not in tracking_release_aircraft_ids
                and _to_int(item.get("aircraft_id"))
                not in tracking_release_failed_aircraft_ids
            ]
            released_ids = _clear_group_tracking_assignments(
                remaining_group_assignments,
                skip_reason=str(evaluation.get("skip_reason") or ""),
            )
            pending_clear_aircraft_ids.update(int(aid) for aid in released_ids)
            if tracking_release_failed_aircraft_ids:
                _emit(
                    "[POSTATTACK][TRACK-RETURN][ERR] Return package generation failed; "
                    "tracking assignment retained and current package left applied "
                    f"(aircraft={sorted(tracking_release_failed_aircraft_ids)})."
                )
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
            aircraft_id = _to_int(assignment.get("aircraft_id"))
            if aircraft_id is None or aircraft_id in ongoing_unavailable:
                continue
            state = available_state_map.setdefault(int(aircraft_id), {})
            if not _normalize_coordinate(state.get("coordinate")):
                fallback_coord = (
                    _normalize_coordinate(assignment.get("handoff_coordinate"))
                    or _normalize_coordinate(assignment.get("last_nonzero_coordinate"))
                    or _normalize_coordinate(assignment.get("original_coordinate"))
                )
                if fallback_coord is not None:
                    state["coordinate"] = fallback_coord

        planning_source_plan_id = _resolve_group_source_plan_id(
            group_assignments,
            fallback_plan_id=int(current_plan_id),
        )
        collab_reservation_start = len(id_reservation_summaries)
        collab = _prepare_post_attack_collaborative_update(
            source_plan_id=int(planning_source_plan_id),
            runtime_plan_id=int(current_plan_id),
            current_input_id=int(current_input_id),
            evaluation=evaluation,
            group_assignments=[dict(item) for item in group_assignments],
            unavailable_aircraft_ids={int(aid) for aid in ongoing_unavailable},
            agent_state_map=available_state_map,
            now_ms=int(now_ms),
            emit=_emit,
            log_prefix="[POSTATTACK][COLLAB]",
            run_cache=run_cache,
            reservation_summaries=id_reservation_summaries,
        )
        if collab is None:
            _collect_returning_lah_updates()
            evaluation["replan_needed"] = False
            evaluation["skip_reason"] = "collaborative_replan_unavailable"
            evaluation["tracking_assignment_retained"] = True
            _emit(
                "Collaborative replan unavailable; tracking assignments retained "
                "until a return-only or collaborative update succeeds."
            )
            continue
        collab_reservation_blocks = [
            dict(item)
            for item in id_reservation_summaries[collab_reservation_start:]
            if isinstance(item, dict)
        ]
        if collab_reservation_blocks:
            evaluation["collaborative_id_reservation"] = {
                "blocks": collab_reservation_blocks,
            }

        for aircraft_id, imp_id in collab.aircraft_imp_ids.items():
            if _update_plan_aircraft_entry(new_plan_data, int(aircraft_id), int(imp_id)):
                generated_imp_ids.add(int(imp_id))
                updated_aircraft_ids.add(int(aircraft_id))
        generated_path_ids.update(int(path_id) for path_id in collab.generated_path_ids)
        for assignment in group_assignments:
            aircraft_id = _to_int(assignment.get("aircraft_id"))
            if aircraft_id is None or int(aircraft_id) not in collab.aircraft_imp_ids:
                continue
            pending_clear_aircraft_ids.add(int(aircraft_id))
        _collect_returning_lah_updates()

    collaborative_replanned_input_ids: Set[int] = set()
    for evaluation in group_summaries:
        if not isinstance(evaluation, dict) or not bool(evaluation.get("replan_needed")):
            continue
        input_id = _to_int(evaluation.get("input_mission_id"))
        if input_id is not None and input_id > 0:
            collaborative_replanned_input_ids.add(int(input_id))

    released_attack_manned_ids: List[int] = []
    result_payload["groupSummaries"] = group_summaries
    result_payload["updatedAircraftIDs"] = sorted(updated_aircraft_ids)
    result_payload["clearedTrackingAircraftIDs"] = sorted(cleared_aircraft_ids)
    result_payload["pendingClearTrackingAircraftIDs"] = sorted(pending_clear_aircraft_ids)
    result_payload["reboundTrackingAircraftIDs"] = sorted(rebound_aircraft_ids)
    result_payload["releasedAttackMannedAircraftIDs"] = list(released_attack_manned_ids)
    result_payload["idReservationSummaries"] = [
        dict(item) for item in id_reservation_summaries if isinstance(item, dict)
    ]
    phase_timer.mark("group_evaluation")
    artifact_write_results = _flush_post_attack_json_batches(run_cache)
    result_payload["artifactWriteFileCount"] = len(artifact_write_results)
    phase_timer.mark("artifact_write")

    if updated_aircraft_ids:
        source_attack_rows, source_attack_scan_errors = collect_lah_attack_rows(plan_data)
        candidate_attack_rows, candidate_attack_scan_errors = collect_lah_attack_rows(
            new_plan_data
        )
        live_attack_invariant = compare_post_attack_pairs(
            source_attack_rows,
            candidate_attack_rows,
            closed_target_ids={int(target_id)},
        )
        live_attack_invariant["sourceScanErrors"] = list(source_attack_scan_errors)
        live_attack_invariant["candidateScanErrors"] = list(candidate_attack_scan_errors)
        if source_attack_scan_errors or candidate_attack_scan_errors:
            live_attack_invariant["ok"] = False
        result_payload["liveAttackInvariant"] = live_attack_invariant
        phase_timer.mark("live_attack_invariant")
        if not bool(live_attack_invariant.get("ok")):
            _emit(
                "[POSTATTACK][LAH][CONTINUITY][ERR] Candidate MissionPlan rejected; "
                "an unfinished attack was deleted, moved to another LAH, duplicated, "
                "or could not be verified "
                f"(expected={live_attack_invariant.get('expected')}, "
                f"actual={live_attack_invariant.get('actual')}, "
                f"missing={live_attack_invariant.get('missing')}, "
                f"unexpected={live_attack_invariant.get('unexpected')}, "
                f"errors={(source_attack_scan_errors + candidate_attack_scan_errors)[:3]})."
            )
            # Generated IMP/FlightPath files are harmless orphan artifacts.  Do
            # not publish the MissionPlan and, critically, do not clear tracking
            # assignments or release attack slots.  The currently applied plan
            # therefore keeps executing its surviving attack paths.
            return _finish_with_timing(
                requested_plan_id=requested_plan_id,
                status="skipped",
                summary={
                    "status": "skipped",
                    "reason": "live_attack_continuity_invariant_failed",
                    "current_plan_id": int(current_plan_id),
                    "candidate_plan_id": int(new_plan_id),
                    "live_attack_invariant": dict(live_attack_invariant),
                    "tracking_assignments_retained": True,
                },
                result_payload=result_payload,
            )
        _emit(
            "[POSTATTACK][LAH][CONTINUITY] Remaining attack mission/path/WP identity set "
            f"certified -> {live_attack_invariant.get('actual')}."
        )

    if not updated_aircraft_ids:
        if pending_clear_aircraft_ids:
            noop_cleared_ids, clear_failed_aircraft_ids = _commit_closed_tracking_assignments(
                pending_clear_aircraft_ids,
                emit=_emit,
                completion_context="after no-op post-attack close",
            )
            cleared_aircraft_ids.update(noop_cleared_ids)
            pending_clear_aircraft_ids.difference_update(noop_cleared_ids)
            if clear_failed_aircraft_ids:
                result_payload["trackingAssignmentClearFailedAircraftIDs"] = sorted(
                    clear_failed_aircraft_ids
                )
            if cleared_aircraft_ids:
                released_attack_manned_ids = _release_attack_slots_if_tracking_closed(
                    input_package_id=input_package_id,
                    current_plan_id=current_plan_id,
                    emit=_emit,
                )
            if clear_failed_aircraft_ids:
                _emit(
                    "skipped: collaborative rejoin update was unnecessary; kept the "
                    "operator-selected current plan, but tracking-state clear will be retried."
                )
            else:
                _emit(
                    "skipped: collaborative rejoin update was unnecessary; kept the "
                    "operator-selected current plan and closed the stale tracking assignment."
                )
        else:
            _emit("skipped: no post-attack collaborative rejoin update was necessary.")
        result_payload["clearedTrackingAircraftIDs"] = sorted(cleared_aircraft_ids)
        result_payload["pendingClearTrackingAircraftIDs"] = sorted(pending_clear_aircraft_ids)
        result_payload["releasedAttackMannedAircraftIDs"] = list(released_attack_manned_ids)
        phase_timer.mark("tracking_assignment_clear")
        phase_timer.mark("state_release")
        return _finish_with_timing(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={
                "status": "skipped",
                "reason": "rejoin_not_needed",
                "group_evaluations": group_summaries,
                "current_plan_id": int(current_plan_id),
                "cleared_tracking_aircraft_ids": sorted(cleared_aircraft_ids),
                "pending_clear_tracking_aircraft_ids": sorted(pending_clear_aircraft_ids),
                "rebound_tracking_aircraft_ids": sorted(rebound_aircraft_ids),
                "released_attack_manned_aircraft_ids": list(released_attack_manned_ids),
            },
            result_payload=result_payload,
        )

    validation_summary = validate_replan_payloads(
        mission_plan=new_plan_data,
        individual_mission_plans=[],
        flight_paths=[],
        scope=f"postAttackRejoin:{new_plan_id}",
        allow_existing_db_artifacts=True,
        # Newly generated paths are validated before their packages are
        # written.  Keep the final cross-file link check, but do not rescan
        # waypoint chains from unchanged DB paths.
        validate_existing_flight_path_waypoints=False,
        validate_existing_flight_path_links=False,
        log=_emit,
    )
    result_payload["validation"] = validation_summary
    phase_timer.mark("validation")

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{int(new_plan_id)}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    write_json(plan_dest, new_plan_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    phase_timer.mark("mission_plan_write")
    clear_failed_aircraft_ids: Set[int] = set()
    if pending_clear_aircraft_ids:
        try:
            clear_tracking_assignments(sorted(pending_clear_aircraft_ids))
            cleared_aircraft_ids.update(int(aid) for aid in pending_clear_aircraft_ids)
        except Exception as exc:
            clear_failed_aircraft_ids.update(int(aid) for aid in pending_clear_aircraft_ids)
            _emit(
                "Tracking assignment clear failed after plan write "
                f"(aircraft={sorted(pending_clear_aircraft_ids)}): {exc}"
            )
    if cleared_aircraft_ids:
        _emit(
            "Tracking assignments cleared after MissionPlan write -> "
            f"aircraft={sorted(cleared_aircraft_ids)}."
        )
    if clear_failed_aircraft_ids:
        result_payload["trackingAssignmentClearFailedAircraftIDs"] = sorted(clear_failed_aircraft_ids)
    phase_timer.mark("tracking_assignment_clear")
    if cleared_aircraft_ids:
        released_attack_manned_ids = _release_attack_slots_if_tracking_closed(
            input_package_id=input_package_id,
            # Inspect the plan just written.  The source plan still contains
            # the closed attack, while the new plan accurately shows either a
            # second sequential attack (slot stays busy) or no attack (release).
            current_plan_id=new_plan_id,
            emit=_emit,
        )
    phase_timer.mark("state_release")
    _queue_post_attack_snapshot_carry(
        int(current_plan_id),
        int(new_plan_id),
        reason="post_attack_rejoin",
        area_ownership_target_input_ids=collaborative_replanned_input_ids,
    )
    _emit(
        "area remaining snapshot carry queued -> "
        f"sourcePlan={current_plan_id}, plan={new_plan_id}."
    )
    rebound_aircraft_ids.update(
        rebind_tracking_assignments_to_plan(
            old_attack_plan_id=int(current_plan_id),
            new_attack_plan_id=int(new_plan_id),
        )
    )
    if rebound_aircraft_ids:
        _emit(
            "Tracking assignment plan binding migrated -> "
            f"{int(current_plan_id)} -> {int(new_plan_id)} "
            f"(aircraft={sorted(rebound_aircraft_ids)})."
        )
    _emit(
        f"MissionPlan updated for post-attack rejoin -> {plan_dest.name} "
        f"(sourcePlan={current_plan_id}, updatedAircraft={sorted(updated_aircraft_ids)})."
    )
    result_payload["clearedTrackingAircraftIDs"] = sorted(cleared_aircraft_ids)
    result_payload["releasedAttackMannedAircraftIDs"] = list(released_attack_manned_ids)

    summary = {
        "status": "success",
        "plan_ids": [int(new_plan_id)],
        "source_plan_id": int(current_plan_id),
        "updated_aircraft_ids": sorted(updated_aircraft_ids),
        "generated_imp_ids": sorted(generated_imp_ids),
        "generated_path_ids": sorted(generated_path_ids),
        "cleared_tracking_aircraft_ids": sorted(cleared_aircraft_ids),
        "rebound_tracking_aircraft_ids": sorted(rebound_aircraft_ids),
        "released_attack_manned_aircraft_ids": list(released_attack_manned_ids),
        "group_evaluations": group_summaries,
        "option_names": [_POST_ATTACK_OPTION_NAME],
        "idReservationSummaries": [dict(item) for item in id_reservation_summaries if isinstance(item, dict)],
    }
    result_payload["result"] = summary
    return _finish_with_timing(
        requested_plan_id=int(new_plan_id),
        status="success",
        summary=summary,
        result_payload=result_payload,
        generated_imp_ids=generated_imp_ids,
        generated_path_ids=generated_path_ids,
    )


def _finish_result(
    *,
    requested_plan_id: Optional[int],
    status: str,
    summary: Dict[str, Any],
    result_payload: Dict[str, Any],
    generated_imp_ids: Optional[Set[int]] = None,
    generated_path_ids: Optional[Set[int]] = None,
) -> PostAttackRejoinPipelineResult:
    log_path = _write_log_payload(result_payload)
    summary.setdefault("logArtifactMode", result_payload.get("logArtifactMode"))
    summary.setdefault("logArtifactWritten", result_payload.get("logArtifactWritten"))
    plan_ids = list(
        summary.get("plan_ids")
        or ([] if status != "success" else ([int(requested_plan_id)] if requested_plan_id else []))
    )
    option_names = list(summary.get("option_names") or ([] if not plan_ids else [_POST_ATTACK_OPTION_NAME]))
    plan_meta_map: Dict[int, Dict[str, Any]] = {}
    if plan_ids:
        plan_meta_map[int(plan_ids[0])] = {
            "postAttackRejoin": True,
            "postAttackRejoinContext": {
                "status": str(status),
                **{k: v for k, v in dict(summary or {}).items() if k != "option_names"},
                "logPath": str(log_path),
            },
        }
    return PostAttackRejoinPipelineResult(
        plan_ids=[int(pid) for pid in plan_ids if _to_int(pid) is not None and int(pid) > 0],
        option_names=option_names,
        plan_meta_map=plan_meta_map,
        generated_imp_ids={int(val) for val in (generated_imp_ids or set())},
        generated_path_ids={int(val) for val in (generated_path_ids or set())},
        log_path=str(log_path),
        status=str(status),
        summary=dict(summary or {}),
    )


def _match_tracking_assignments(
    *,
    current_plan_id: int,
    target_id: int,
    watcher_id: Optional[int],
    preferred_aircraft_ids: Any = None,
    emit: Optional[LogCallback] = None,
) -> List[Dict[str, Any]]:
    watcher_uav_ids = _watcher_uav_ids()
    current_plan_lineage = resolve_plan_lineage_ids(int(current_plan_id)) or {int(current_plan_id)}
    candidate_assignments: List[Dict[str, Any]] = []
    for assignment in list_active_tracking_assignments():
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        attack_plan_id = _to_int(assignment.get("attack_plan_id"))
        if attack_plan_id is None or int(attack_plan_id) not in current_plan_lineage:
            continue
        if _to_int(assignment.get("target_id")) != int(target_id):
            continue
        candidate_assignments.append(dict(assignment))
    if not candidate_assignments:
        return []
    lineage_matched_ids = sorted(
        {
            int(_to_int(item.get("aircraft_id")))
            for item in candidate_assignments
            if _to_int(item.get("aircraft_id")) is not None
            and _to_int(item.get("attack_plan_id")) != int(current_plan_id)
        }
    )
    if lineage_matched_ids and emit:
        emit(
            "tracking assignment matched through plan lineage -> "
            f"currentPlan={int(current_plan_id)}, lineage={sorted(current_plan_lineage)}, "
            f"aircraft={lineage_matched_ids}."
        )

    raw_preferred_ids = (
        list(preferred_aircraft_ids)
        if isinstance(preferred_aircraft_ids, (list, tuple, set))
        else ([preferred_aircraft_ids] if preferred_aircraft_ids is not None else [])
    )
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
            if emit:
                matched_ids = sorted(
                    {
                        int(_to_int(item.get("aircraft_id")))
                        for item in preferred_matches
                        if _to_int(item.get("aircraft_id")) is not None
                    }
                )
                if watcher_id is not None and len(matched_ids) == 1 and int(matched_ids[0]) != int(watcher_id):
                    emit(
                        "watcher mismatch on closure detail -> using trackingAircraftIDList fallback "
                        f"(eventWatcher={watcher_id}, trackingAircraft={matched_ids[0]})."
                    )
            return preferred_matches

    if watcher_id is None or watcher_id not in watcher_uav_ids:
        return candidate_assignments

    watcher_matches = [
        assignment
        for assignment in candidate_assignments
        if _to_int(assignment.get("aircraft_id")) == int(watcher_id)
    ]
    if watcher_matches:
        return watcher_matches

    candidate_aircraft_ids = sorted(
        {
            int(_to_int(item.get("aircraft_id")))
            for item in candidate_assignments
            if _to_int(item.get("aircraft_id")) is not None
        }
    )
    if len(candidate_aircraft_ids) == 1:
        if emit:
            emit(
                "watcher mismatch on closure detail -> falling back to sole active tracking assignment "
                f"(eventWatcher={watcher_id}, trackingAircraft={candidate_aircraft_ids[0]})."
            )
        return candidate_assignments

    if emit:
        emit(
            "watcher mismatch on closure detail -> ambiguous active tracking assignments; skip "
            f"(eventWatcher={watcher_id}, candidateTrackingAircraft={candidate_aircraft_ids})."
        )
    return []


def _mission_target_id(mission: Dict[str, Any]) -> Optional[int]:
    if not isinstance(mission, dict):
        return None
    info = mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        return None
    return _to_int(info.get("targetID") or info.get("targetId"))


def _clear_waypoint_attacks_for_targets(
    flight_paths: Iterable[Dict[str, Any]],
    target_ids: Any,
) -> int:
    """Blank ``attack`` blocks that still name an already-serviced target.

    The waypoint keeps its geometry - the aircraft still flies the leg - but it
    no longer carries a firing command for a target that is gone.
    """

    wanted: set[int] = set()
    for value in target_ids or []:
        parsed = _to_int(value)
        if parsed is not None and int(parsed) > 0:
            wanted.add(int(parsed))
    if not wanted:
        return 0

    cleared = 0
    for payload in flight_paths or []:
        if not isinstance(payload, dict):
            continue
        for key in ("lahWaypointList", "waypointList", "uavWaypointList"):
            rows = payload.get(key)
            if not isinstance(rows, list):
                continue
            for waypoint in rows:
                if not isinstance(waypoint, dict):
                    continue
                attack = waypoint.get("attack")
                if not isinstance(attack, dict):
                    continue
                attack_target = _to_int(attack.get("targetID"))
                if attack_target is None or int(attack_target) not in wanted:
                    continue
                attack["targetID"] = 0
                attack["weaponType"] = 0
                cleared += 1
    return int(cleared)


def _flight_path_attack_target_ids(payload: Dict[str, Any]) -> set[int]:
    """Return executable target IDs carried by a generated flight path."""

    target_ids: set[int] = set()
    if not isinstance(payload, dict):
        return target_ids
    for key in ("lahWaypointList", "waypointList", "uavWaypointList"):
        rows = payload.get(key)
        if not isinstance(rows, list):
            continue
        for waypoint in rows:
            if not isinstance(waypoint, dict):
                continue
            attack = waypoint.get("attack")
            target_id = _to_int((attack or {}).get("targetID")) if isinstance(attack, dict) else None
            if target_id is not None and int(target_id) > 0:
                target_ids.add(int(target_id))
    return target_ids


def _is_post_attack_resume_mission(mission: Dict[str, Any]) -> bool:
    """Whether an attack planner explicitly marked this as its return leg.

    Attack and support missions intentionally retain ``targetID`` for
    provenance.  The return mission used to retain it as well, which made the
    post-attack cleanup mistake the already-built return route for another
    target branch and delete it.  New artifacts carry an explicit marker; the
    legacy fallback below handles plans produced before that marker existed.
    """

    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo")
    return bool(
        mission.get("postAttackResume")
        or (
            isinstance(info, dict)
            and info.get("postAttackResume")
        )
    )


def _preserve_legacy_lah_target_bound_resumes(
    mission_list: List[Dict[str, Any]],
    attack_target_indices: Iterable[int],
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
    emit: Optional[LogCallback] = None,
    log_prefix: str = "[POSTATTACK][LAH]",
) -> List[int]:
    """Keep unmarked return legs emitted by older attack planners.

    A generated LAH engagement is laid out as one or more target-bound action
    missions followed by a target-bound resume mission.  The resume is the last
    member of its ``(inputMissionID, targetID)`` group and carries no executable
    attack waypoint.  Only that tail is preserved; holds and actual sequential
    attacks ahead of it remain eligible for removal.
    """

    selected = sorted(
        {
            int(index)
            for index in attack_target_indices or []
            if 0 <= int(index) < len(mission_list or [])
        }
    )
    if not selected:
        return []

    grouped: Dict[Tuple[int, int], List[int]] = defaultdict(list)
    for index in selected:
        mission = mission_list[index]
        # A run-to-cover leg is target-bound but is never a return route, and
        # counting it here would turn a lone hold into a two-member group whose
        # tail - carrying no attack command - then reads as a legacy return leg
        # and survives the sweep it should not survive.
        if bool(mission.get("lahCoverIngress")):
            continue
        input_id = _extract_related_input_mission_id(mission)
        target_id = _mission_target_id(mission)
        if input_id is None or target_id is None or int(target_id) <= 0:
            continue
        grouped[(int(input_id), int(target_id))].append(int(index))

    preserved: set[int] = set()
    for (input_id, target_id), group_indices in grouped.items():
        # A singleton may be a hold/support action with no attack command.  It
        # is not safe to call it a return leg without the preceding action.
        if len(group_indices) < 2:
            continue
        tail_index = max(group_indices)
        tail_mission = mission_list[tail_index]
        path_payload = _load_path_payload(
            _to_int(tail_mission.get("pathID")),
            run_cache=run_cache,
            copy_result=False,
        )
        if not isinstance(path_payload, dict):
            continue
        if _flight_path_attack_target_ids(path_payload):
            continue
        # Upgrade the legacy artifact in-memory.  The cloned return mission then
        # carries the explicit marker, so a delayed duplicate event cannot
        # classify this now-singleton return leg as an attack branch again.
        tail_mission["postAttackResume"] = True
        tail_mission["postAttackSourceTargetID"] = int(target_id)
        tail_info = tail_mission.get("individualMissionInfo")
        if isinstance(tail_info, dict):
            tail_info = deepcopy(tail_info)
            tail_info["targetID"] = None
            tail_mission["individualMissionInfo"] = tail_info
        preserved.add(int(tail_index))
        if emit is not None:
            emit(
                f"{log_prefix} preserved legacy target-bound return route "
                f"(input={input_id}, target={target_id}, "
                f"mission={_to_int(tail_mission.get('individualMissionID'))}, "
                f"path={_to_int(tail_mission.get('pathID'))})."
            )

    return [index for index in selected if int(index) not in preserved]


def _remaining_lah_live_attack_target_ids(
    mission_list: List[Dict[str, Any]],
    *,
    start_index: int,
    removed_target_ids: set[int],
    run_cache: Optional[_PostAttackRunCache] = None,
) -> set[int]:
    """Find attacks that must survive this target-specific close/replan."""

    remaining: set[int] = set()
    for mission in (mission_list or [])[max(0, int(start_index)) :]:
        if not isinstance(mission, dict):
            continue
        path_payload = _load_path_payload(
            _to_int(mission.get("pathID")),
            run_cache=run_cache,
            copy_result=False,
        )
        if not isinstance(path_payload, dict):
            continue
        remaining.update(_flight_path_attack_target_ids(path_payload))
    return {
        int(target_id)
        for target_id in remaining
        if int(target_id) not in {int(value) for value in removed_target_ids}
    }


def _lah_attack_target_mission_indices(
    mission_list: List[Dict[str, Any]],
    *,
    current_input_id: int,
    target_id: int,
    exclude_all_target_missions: bool = False,
    retained_target_ids: Any = None,
) -> List[int]:
    """Locate LAH attack/attack-support branches to remove before resuming.

    ``exclude_all_target_missions`` sweeps every target-bound branch, which is
    what a whole-package attack exclusion wants.  ``retained_target_ids`` carves
    out the engagements that must survive it: finishing one target must never
    cancel another aircraft's attack on a different target, which otherwise
    leaves that target tracked forever and never shot.
    """

    retained: set[int] = set()
    for value in retained_target_ids or []:
        retained_id = _to_int(value)
        if retained_id is not None and int(retained_id) > 0:
            retained.add(int(retained_id))

    indices: List[int] = []
    for idx, mission in enumerate(mission_list or []):
        if not isinstance(mission, dict):
            continue
        if _is_post_attack_resume_mission(mission):
            continue
        mission_target_id = _mission_target_id(mission)
        if mission_target_id is None or mission_target_id <= 0:
            continue
        if int(mission_target_id) in retained:
            continue
        mission_info = mission.get("individualMissionInfo")
        mission_type = _to_int(
            (mission_info or {}).get("individualMissionType")
            if isinstance(mission_info, dict)
            else None
        )
        if exclude_all_target_missions:
            # LAH attack plans use type 2 for the shooter and target-bound
            # type 9 for the supporting/holding manned aircraft, each preceded
            # by its own type-7 run-to-cover leg.  The ingress is part of the
            # branch and has to be swept with it.
            if mission_type in {2, 9} or bool(mission.get("lahCoverIngress")):
                indices.append(int(idx))
            continue
        if (
            _extract_related_input_mission_id(mission) == int(current_input_id)
            and int(mission_target_id) == int(target_id)
        ):
            indices.append(int(idx))
    return indices


def _recover_tracking_assignments_from_current_plan(
    *,
    current_plan_id: int,
    target_id: int,
    watcher_id: Optional[int] = None,
    preferred_aircraft_ids: Any = None,
    emit: Optional[LogCallback] = None,
) -> List[Dict[str, Any]]:
    """Recover an active tracking branch from authoritative plan artifacts.

    The runtime tracking-state file is auxiliary state and can briefly lag the
    selected attack plan when attack/attack-exclusion options are produced in
    parallel.  The selected MissionPlan and IMP still contain the target-specific
    tracking mission, so use those artifacts as the fallback source of truth.
    """

    def _read_payload(kind: str, numeric_id: Any) -> Optional[Dict[str, Any]]:
        value = _to_int(numeric_id)
        if value is None or value <= 0:
            return None
        try:
            path = db_paths.get_db_subpath(str(kind), f"{int(value)}.json")
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None
        return payload if isinstance(payload, dict) else None

    plan_data = _read_payload("MissionPlan", int(current_plan_id))
    if not isinstance(plan_data, dict):
        return []

    recovered: List[Dict[str, Any]] = []
    for aircraft_entry in plan_data.get("aircraftList") or []:
        if not isinstance(aircraft_entry, dict):
            continue
        aircraft_id = _to_int(aircraft_entry.get("aircraftID"))
        if aircraft_id is None or aircraft_id <= 3:
            continue
        imp_data = _read_payload(
            "IndividualMissionPlan",
            aircraft_entry.get("individualMissionPackageID"),
        )
        mission_list = (
            imp_data.get("individualMissionList") if isinstance(imp_data, dict) else None
        )
        if not isinstance(mission_list, list):
            continue

        tracking_index = next(
            (
                idx
                for idx, mission in enumerate(mission_list)
                if isinstance(mission, dict)
                and not bool(mission.get("isDone"))
                and _mission_target_id(mission) == int(target_id)
                and _to_int((mission.get("individualMissionInfo") or {}).get("individualMissionType"))
                == 1
            ),
            None,
        )
        if tracking_index is None:
            continue

        tracking_mission = mission_list[int(tracking_index)]
        current_input_id = _extract_related_input_mission_id(tracking_mission)
        resume_index = next(
            (
                idx
                for idx in range(int(tracking_index) + 1, len(mission_list))
                if isinstance(mission_list[idx], dict)
                and not bool(mission_list[idx].get("isDone"))
                and _extract_related_input_mission_id(mission_list[idx]) == current_input_id
                and _mission_target_id(mission_list[idx]) != int(target_id)
            ),
            None,
        )
        resume_mission = (
            mission_list[int(resume_index)]
            if resume_index is not None and isinstance(mission_list[int(resume_index)], dict)
            else None
        )

        tracking_path_id = _to_int(tracking_mission.get("pathID"))
        resume_path_id = _to_int((resume_mission or {}).get("pathID"))
        original_mission = resume_mission or tracking_mission
        original_path_id = resume_path_id or tracking_path_id
        original_path = _read_payload("FlightPath", original_path_id)
        tracking_path = _read_payload("FlightPath", tracking_path_id)
        original_waypoints = (
            original_path.get("waypointList") if isinstance(original_path, dict) else []
        )
        original_waypoints = original_waypoints if isinstance(original_waypoints, list) else []
        current_waypoint = next(
            (
                item
                for item in original_waypoints
                if isinstance(item, dict) and not bool(item.get("isDone"))
            ),
            None,
        )
        original_coord = (
            _normalize_coordinate((current_waypoint or {}).get("coordinate"))
            or _normalize_coordinate(_extract_final_uav_coordinate(original_path or {}))
            or _normalize_coordinate(_extract_final_uav_coordinate(tracking_path or {}))
        )
        recovered.append(
            {
                "aircraft_id": int(aircraft_id),
                "active": True,
                "source_plan_id": int(current_plan_id),
                "attack_plan_id": int(current_plan_id),
                "current_input_mission_id": _to_int(current_input_id),
                "original_path_id": _to_int(original_path_id),
                "original_individual_mission_id": _to_int(
                    original_mission.get("individualMissionID")
                ),
                "original_current_waypoint_id": _to_int(
                    (current_waypoint or {}).get("waypointID")
                ),
                "original_coordinate": original_coord,
                "tracking_path_id": tracking_path_id,
                "tracking_individual_mission_id": _to_int(
                    tracking_mission.get("individualMissionID")
                ),
                "resume_path_id": resume_path_id,
                "resume_individual_mission_id": _to_int(
                    (resume_mission or {}).get("individualMissionID")
                ),
                "target_id": int(target_id),
                "recovered_from_plan_artifacts": True,
            }
        )

    if not recovered:
        return []

    raw_preferred_ids = (
        list(preferred_aircraft_ids)
        if isinstance(preferred_aircraft_ids, (list, tuple, set))
        else ([preferred_aircraft_ids] if preferred_aircraft_ids is not None else [])
    )
    preferred_ids = {
        int(value)
        for value in (_to_int(item) for item in raw_preferred_ids)
        if value is not None and int(value) > 0
    }
    preferred_matches = [
        item for item in recovered if _to_int(item.get("aircraft_id")) in preferred_ids
    ]
    if preferred_matches:
        recovered = preferred_matches
    elif watcher_id is not None and int(watcher_id) in _watcher_uav_ids():
        watcher_matches = [
            item
            for item in recovered
            if _to_int(item.get("aircraft_id")) == int(watcher_id)
        ]
        if watcher_matches:
            recovered = watcher_matches
        elif len(recovered) > 1:
            if emit:
                emit(
                    "tracking artifact recovery remained ambiguous after watcher filter -> "
                    f"targetID={target_id}, aircraft="
                    f"{sorted(int(item['aircraft_id']) for item in recovered)}."
                )
            return []

    if emit:
        emit(
            "active tracking assignment recovered from selected MissionPlan artifacts -> "
            f"plan={int(current_plan_id)}, targetID={int(target_id)}, aircraft="
            f"{sorted(int(item['aircraft_id']) for item in recovered)}."
        )
    return recovered


def _extract_lah_waypoint_list(path_payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    for key in ("lahWaypointList", "waypointList", "uavWaypointList"):
        items = path_payload.get(key)
        if isinstance(items, list):
            return [deepcopy(item) for item in items if isinstance(item, dict)]
    return []


def _build_lah_path_payload_from_waypoints(
    *,
    template_path: Dict[str, Any],
    aircraft_id: int,
    path_id: int,
    individual_mission_id: int,
    waypoints: List[Dict[str, Any]],
    now_ms: int,
    waypoint_id_provider: Optional[Callable[[], int]] = None,
) -> Dict[str, Any]:
    template = template_path if isinstance(template_path, dict) else {}
    # The source waypoint arrays are replaced below.  Excluding them from the
    # shell copy avoids deep-copying a dense path only to discard it.
    payload = {
        key: deepcopy(value)
        for key, value in template.items()
        if key not in {"lahWaypointList", "waypointList"}
    }
    payload["pathID"] = int(path_id)
    payload["aircraftID"] = int(aircraft_id)
    payload["individualMissionID"] = int(individual_mission_id)
    payload["timestamp"] = int(now_ms)
    if "Source" in payload or "source" not in payload:
        payload["Source"] = str(payload.get("Source") or payload.get("source") or "MMR")
        payload.pop("source", None)
    else:
        payload["source"] = str(payload.get("source") or payload.get("Source") or "MMR")

    copied = [deepcopy(item) for item in waypoints if isinstance(item, dict)]
    for waypoint in copied:
        waypoint["isDone"] = False
    if copied:
        reassign_unique_waypoint_ids_inplace(
            copied,
            waypoint_id_provider=waypoint_id_provider,
        )

    payload["lahWaypointList"] = copied
    if "waypointList" in template:
        payload["waypointList"] = deepcopy(copied)
    sanitize_flight_path_payload_filming_altitudes(payload)
    normalize_flight_path_waypoint_altitudes_inplace(payload)
    normalize_flight_path_waypoint_speeds_inplace(payload)
    return payload


def _find_returning_manned_attack_aircraft_ids(
    *,
    current_plan_id: int,
    current_input_id: int,
    target_id: int,
    plan_data: Dict[str, Any],
    run_cache: Optional[_PostAttackRunCache] = None,
) -> List[int]:
    matched_ids: List[int] = []
    for entry in plan_data.get("aircraftList") or []:
        aircraft_id = _to_int((entry or {}).get("aircraftID"))
        if aircraft_id is None or aircraft_id <= 0 or aircraft_id > 3:
            continue
        imp_data = _load_imp_package_for_aircraft_cached(
            source_plan_id=int(current_plan_id),
            aircraft_id=int(aircraft_id),
            run_cache=run_cache,
        )
        if not isinstance(imp_data, dict):
            continue
        mission_list = imp_data.get("individualMissionList")
        if not isinstance(mission_list, list):
            continue
        has_target_attack = any(
            isinstance(mission, dict)
            and _extract_related_input_mission_id(mission) == int(current_input_id)
            and _mission_target_id(mission) == int(target_id)
            for mission in mission_list
        )
        if has_target_attack:
            matched_ids.append(int(aircraft_id))
    return matched_ids


def _splice_post_attack_cover_prelude(
    resume_waypoints: List[Dict[str, Any]],
    *,
    aircraft_id: int,
    plan: Optional[Dict[str, Any]],
    hold_seconds: int,
    emit: LogCallback,
    log_prefix: str,
) -> Tuple[List[Dict[str, Any]], Optional[Dict[str, Any]], str]:
    """Put the concealment route in front of the return route.

    Returns ``(waypoints, plan_or_None, role)``.  ``plan`` comes back as
    ``None`` when no certified cover could be serialized, which is the signal
    that the route was not re-timed and holds no concealment point.
    """

    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _build_lah_tactical_route_waypoints,
        get_runtime_attack_int,
    )

    if not resume_waypoints:
        return resume_waypoints, None, "hold"
    role = (
        "relay"
        if int(aircraft_id) == int(get_runtime_attack_int("command_aircraft_id", 1))
        else "hold"
    )
    # Placeholder IDs: the payload builder renumbers and relinks the whole
    # chain once the reservation block is known.
    counter = {"value": 0}

    def _placeholder_id() -> int:
        counter["value"] += 1
        return int(counter["value"])

    cover_waypoints = _build_lah_tactical_route_waypoints(
        template_wp=deepcopy(resume_waypoints[0]),
        plan=plan,
        waypoint_id_provider=_placeholder_id,
        terminal_hover_seconds=int(hold_seconds),
    )
    if cover_waypoints:
        return cover_waypoints + deepcopy(resume_waypoints), plan, role

    # No certified concealment reachable - wait where the aircraft already is
    # rather than driving it off into a live threat.
    spliced = deepcopy(resume_waypoints)
    if hold_seconds > 0 and isinstance(spliced[0], dict):
        hovering = spliced[0].get("hovering")
        existing_s = (_to_int(hovering.get("time")) if isinstance(hovering, dict) else 0) or 0
        spliced[0]["hovering"] = {"time": int(max(existing_s, hold_seconds))}
        emit(
            f"{log_prefix} aircraft {aircraft_id} holds in place for "
            f"{hold_seconds}s (no certified cover)."
        )
    return spliced, None, role


def _append_post_attack_cover_route(
    resume_waypoints: List[Dict[str, Any]],
    *,
    aircraft_id: int,
    plan: Optional[Dict[str, Any]],
    hold_seconds: int,
    emit: LogCallback,
    log_prefix: str,
) -> Tuple[List[Dict[str, Any]], Optional[Dict[str, Any]], str]:
    """Append a freshly certified cover route after an already-planned descent."""

    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _build_lah_tactical_route_waypoints,
        _extract_lah_waypoint_coordinate,
        _haversine_distance_m,
        _normalize_altitude_value,
        get_runtime_attack_int,
    )

    if not resume_waypoints:
        return resume_waypoints, None, "hold"
    role = (
        "relay"
        if int(aircraft_id) == int(get_runtime_attack_int("command_aircraft_id", 1))
        else "hold"
    )
    counter = {"value": 0}

    def _placeholder_id() -> int:
        counter["value"] += 1
        return int(counter["value"])

    cover_waypoints = _build_lah_tactical_route_waypoints(
        template_wp=deepcopy(resume_waypoints[-1]),
        plan=plan,
        waypoint_id_provider=_placeholder_id,
        terminal_hover_seconds=int(hold_seconds),
    )
    if cover_waypoints:
        combined = deepcopy(resume_waypoints)
        last_coord = _extract_lah_waypoint_coordinate(combined[-1])
        first_coord = _extract_lah_waypoint_coordinate(cover_waypoints[0])
        same_endpoint = False
        if last_coord is not None and first_coord is not None:
            distance_m = _haversine_distance_m(last_coord, first_coord)
            last_altitude_m = _normalize_altitude_value(last_coord.get("altitude"))
            first_altitude_m = _normalize_altitude_value(first_coord.get("altitude"))
            same_endpoint = bool(
                distance_m is not None
                and float(distance_m) <= 1.0
                and last_altitude_m is not None
                and first_altitude_m is not None
                and abs(int(last_altitude_m) - int(first_altitude_m)) <= 1
            )
        if same_endpoint:
            first_hover = (
                cover_waypoints[0].get("hovering")
                if isinstance(cover_waypoints[0].get("hovering"), dict)
                else {}
            )
            existing_hover = (
                combined[-1].get("hovering")
                if isinstance(combined[-1].get("hovering"), dict)
                else {}
            )
            combined[-1]["hovering"] = {
                "time": int(
                    max(
                        _to_int(existing_hover.get("time")) or 0,
                        _to_int(first_hover.get("time")) or 0,
                    )
                )
            }
            cover_waypoints = cover_waypoints[1:]
        combined.extend(deepcopy(cover_waypoints))
        return combined, plan, role

    # The known descent is still safer than remaining at the exposed firing
    # altitude.  If no new certified position exists, wait at its endpoint but
    # do not label that point as certified against the new threat set.
    held = deepcopy(resume_waypoints)
    if hold_seconds > 0 and isinstance(held[-1], dict):
        hovering = held[-1].get("hovering")
        existing_s = (_to_int(hovering.get("time")) if isinstance(hovering, dict) else 0) or 0
        held[-1]["hovering"] = {"time": int(max(existing_s, hold_seconds))}
        emit(
            f"{log_prefix} aircraft {aircraft_id} descends first and holds for "
            f"{hold_seconds}s (no cover certified for the current threat set)."
        )
    return held, None, role


def _rebase_post_attack_cover_timing(payload: Dict[str, Any]) -> None:
    """Re-time a resume route that now starts with a concealment prelude.

    The resume waypoints carry ETAs from the plan they were cut out of, so
    splicing cover in front of them leaves the chain non-monotonic.  ICD eta is
    cumulative from a fixed zero at the first waypoint, so the whole chain is
    recomputed and the per-leg fuel with it.
    """

    waypoints = payload.get("lahWaypointList")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        return
    try:
        from modules.common.ecf import apply_leg_fuel_inplace
        from modules.common.eta import annotate_eta_flight_plan

        annotate_eta_flight_plan(
            payload,
            default_speed_mps=40.0,
            waypoint_list_keys=("lahWaypointList",),
        )
        apply_leg_fuel_inplace(waypoints)
    except Exception:
        return
    if isinstance(payload.get("waypointList"), list):
        payload["waypointList"] = deepcopy(waypoints)


def _lah_coordinates_are_continuous(
    left: Optional[Dict[str, Any]],
    right: Optional[Dict[str, Any]],
    *,
    horizontal_tolerance_m: float = 1.0,
    altitude_tolerance_m: float = 1.0,
) -> bool:
    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _haversine_distance_m,
        _normalize_altitude_value,
    )

    if not isinstance(left, dict) or not isinstance(right, dict):
        return False
    horizontal_m = _haversine_distance_m(left, right)
    left_altitude_m = _normalize_altitude_value(left.get("altitude"))
    right_altitude_m = _normalize_altitude_value(right.get("altitude"))
    return bool(
        horizontal_m is not None
        and float(horizontal_m) <= float(horizontal_tolerance_m)
        and left_altitude_m is not None
        and right_altitude_m is not None
        and abs(int(left_altitude_m) - int(right_altitude_m))
        <= float(altitude_tolerance_m)
    )


def _build_post_attack_lah_transition_prefix(
    *,
    start_waypoint: Dict[str, Any],
    destination_waypoint: Dict[str, Any],
    aircraft_id: int,
) -> Optional[Dict[str, Any]]:
    """Build only the terrain-safe prefix needed to close a path boundary."""

    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _build_lah_low_level_waypoint_route,
        _extract_lah_waypoint_coordinate,
        _haversine_distance_m,
        _normalize_altitude_value,
    )

    start_coord = _extract_lah_waypoint_coordinate(start_waypoint)
    destination_coord = _extract_lah_waypoint_coordinate(destination_waypoint)
    if start_coord is None or destination_coord is None:
        raise ValueError(
            f"post-attack LAH path boundary coordinate missing for aircraft {aircraft_id}"
        )

    horizontal_gap_m = _haversine_distance_m(start_coord, destination_coord)
    start_altitude_m = _normalize_altitude_value(start_coord.get("altitude"))
    destination_altitude_m = _normalize_altitude_value(destination_coord.get("altitude"))
    if (
        horizontal_gap_m is None
        or start_altitude_m is None
        or destination_altitude_m is None
    ):
        raise ValueError(
            f"post-attack LAH path boundary is not measurable for aircraft {aircraft_id}"
        )
    altitude_gap_m = int(destination_altitude_m) - int(start_altitude_m)
    if _lah_coordinates_are_continuous(start_coord, destination_coord):
        return None

    placeholder_id = 0

    def _next_placeholder_id() -> int:
        nonlocal placeholder_id
        placeholder_id += 1
        return int(placeholder_id)

    speed_mps = (
        _to_float(destination_waypoint.get("speed"))
        or _to_float(start_waypoint.get("speed"))
        or 40.0
    )
    connector = _build_lah_low_level_waypoint_route(
        template_wp=destination_waypoint,
        route_coordinates=[start_coord, destination_coord],
        waypoint_id_provider=_next_placeholder_id,
        speed_mps=float(speed_mps),
    )
    if not connector:
        raise RuntimeError(
            f"DEM-safe post-attack connector generation failed for aircraft {aircraft_id}"
        )

    connector_start = _extract_lah_waypoint_coordinate(connector[0])
    connector_end = _extract_lah_waypoint_coordinate(connector[-1])
    if not _lah_coordinates_are_continuous(
        connector_start,
        start_coord,
        horizontal_tolerance_m=2.0,
        altitude_tolerance_m=1.0,
    ):
        raise RuntimeError(
            f"post-attack connector does not start at the prior endpoint for aircraft {aircraft_id}"
        )
    connector_destination_gap_m = (
        _haversine_distance_m(connector_end, destination_coord)
        if connector_end is not None
        else None
    )
    if (
        connector_destination_gap_m is None
        or float(connector_destination_gap_m) > 2.0
    ):
        raise RuntimeError(
            f"post-attack connector does not reach the retained mission for aircraft {aircraft_id}"
        )

    # If the DEM route reaches the exact original first waypoint, retain that
    # original waypoint as the terminal of the connector.  It may carry mission
    # semantics not copied by the terrain builder.  If the DEM raised the same
    # XY for safety, keep both points so the subsequent vertical leg is explicit.
    prefix_waypoints = (
        connector[:-1]
        if _lah_coordinates_are_continuous(connector_end, destination_coord)
        else connector
    )
    if not prefix_waypoints:
        raise RuntimeError(
            f"post-attack connector collapsed despite a non-zero boundary gap "
            f"for aircraft {aircraft_id}"
        )

    return {
        "prefixWaypoints": [deepcopy(item) for item in prefix_waypoints],
        "horizontalGapM": float(horizontal_gap_m),
        "altitudeGapM": int(altitude_gap_m),
    }


def _prepare_post_attack_lah_follow_up_connector(
    *,
    primary_resume: Optional[
        Tuple[Dict[str, Any], Dict[str, Any], List[Dict[str, Any]]]
    ],
    follow_up_source_missions: List[Dict[str, Any]],
    aircraft_id: int,
    run_cache: Optional[_PostAttackRunCache],
    emit: LogCallback,
    log_prefix: str,
) -> Optional[Dict[str, Any]]:
    """Prepare a DEM-safe boundary leg into the first retained mission.

    Flight paths are executed independently by SIM, so the first waypoint of a
    follow-up path must repeat the terminal point of the preceding path.  The
    attack planner's legacy return paths occasionally started tens of metres
    away; deleting the return path made that gap kilometres long.  This prefix
    closes either case while retaining the already-planned return geometry.
    """

    if primary_resume is None or not primary_resume[2]:
        return None

    first_follow_up_mission: Optional[Dict[str, Any]] = None
    first_follow_up_path: Optional[Dict[str, Any]] = None
    first_follow_up_waypoints: List[Dict[str, Any]] = []
    for mission in follow_up_source_missions or []:
        if not isinstance(mission, dict):
            continue
        if _skip_replan_follow_up_reason(mission, excluded_input_ids=set()) is not None:
            continue
        path_payload = _load_path_payload(
            _to_int(mission.get("pathID")),
            run_cache=run_cache,
            copy_result=True,
        )
        if not isinstance(path_payload, dict):
            # The clone stage reports the authoritative load error.
            return None
        waypoints = _extract_lah_waypoint_list(path_payload)
        if not waypoints:
            continue
        first_follow_up_mission = mission
        first_follow_up_path = path_payload
        first_follow_up_waypoints = waypoints
        break

    if (
        first_follow_up_mission is None
        or first_follow_up_path is None
        or not first_follow_up_waypoints
    ):
        return None

    prepared = _build_post_attack_lah_transition_prefix(
        start_waypoint=primary_resume[2][-1],
        destination_waypoint=first_follow_up_waypoints[0],
        aircraft_id=int(aircraft_id),
    )
    if prepared is None:
        return None
    prepared.update(
        {
            "sourceMissionID": _to_int(
                first_follow_up_mission.get("individualMissionID")
            ),
            "sourcePathID": _to_int(first_follow_up_mission.get("pathID")),
        }
    )
    return prepared


def _prepend_post_attack_lah_follow_up_connector(
    *,
    prepared: Optional[Dict[str, Any]],
    follow_up_missions: List[Dict[str, Any]],
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]],
    waypoint_id_provider: Optional[Callable[[], int]],
    aircraft_id: int,
    emit: LogCallback,
    log_prefix: str,
) -> int:
    """Attach a prepared connector to the first cloned LAH follow-up path."""

    prefix = [
        deepcopy(item)
        for item in ((prepared or {}).get("prefixWaypoints") or [])
        if isinstance(item, dict)
    ]
    if not prefix:
        return 0

    selected_payload: Optional[Dict[str, Any]] = None
    for _destination, payload in follow_up_paths or []:
        if (
            isinstance(payload, dict)
            and isinstance(payload.get("lahWaypointList"), list)
            and any(isinstance(item, dict) for item in payload.get("lahWaypointList") or [])
        ):
            selected_payload = payload
            break
    if selected_payload is None:
        raise RuntimeError(
            f"cloned LAH follow-up path unavailable for aircraft {aircraft_id}"
        )

    existing_waypoints = [
        item
        for item in (selected_payload.get("lahWaypointList") or [])
        if isinstance(item, dict)
    ]
    if not existing_waypoints:
        raise RuntimeError(
            f"cloned LAH follow-up waypoint list is empty for aircraft {aircraft_id}"
        )

    if waypoint_id_provider is not None:
        reassign_unique_waypoint_ids_inplace(
            prefix,
            waypoint_id_provider=waypoint_id_provider,
        )
    combined = prefix + existing_waypoints
    relink_waypoints(combined)
    selected_payload["lahWaypointList"] = combined
    if isinstance(selected_payload.get("waypointList"), list):
        selected_payload["waypointList"] = deepcopy(combined)
    _rebase_post_attack_cover_timing(selected_payload)

    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _lah_waypoints_to_coordinate_list,
    )

    selected_path_id = _to_int(selected_payload.get("pathID"))
    for mission in follow_up_missions or []:
        if not isinstance(mission, dict) or _to_int(mission.get("pathID")) != selected_path_id:
            continue
        info = mission.get("individualMissionInfo")
        info = deepcopy(info) if isinstance(info, dict) else {}
        info["coordinateList"] = _lah_waypoints_to_coordinate_list(combined)
        mission["individualMissionInfo"] = info
        break

    emit(
        f"{log_prefix} inserted DEM-safe post-attack return connector "
        f"(aircraft={aircraft_id}, fromPath=attack/cover, "
        f"toSourcePath={_to_int((prepared or {}).get('sourcePathID'))}, "
        f"horizontal={float((prepared or {}).get('horizontalGapM') or 0.0):.1f}m, "
        f"altitudeDelta={int((prepared or {}).get('altitudeGapM') or 0)}m, "
        f"addedWaypoints={len(prefix)})."
    )
    return int(len(prefix))


def _remaining_enemy_coordinates(destroyed_target_id: Optional[int]) -> List[Dict[str, Any]]:
    """Live contacts still on the board after this kill."""

    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _load_target_entries,
        _normalize_coordinate,
    )

    entries, error = _load_target_entries()
    if error or not entries:
        return []
    killed = _to_int(destroyed_target_id)
    # targetInfo is keyed by target+watcher, so one target has a row per
    # observer.  A kill reported by any one of them settles it for all.
    destroyed_ids = {
        int(value)
        for value in (
            _to_int((entry or {}).get("target_id"))
            for entry in entries
            if isinstance(entry, dict) and bool(entry.get("is_destroyed"))
        )
        if value is not None
    }
    if killed is not None:
        destroyed_ids.add(int(killed))
    seen: set[tuple[float, float]] = set()
    out: List[Dict[str, Any]] = []
    for entry in entries:
        if not isinstance(entry, dict) or bool(entry.get("is_destroyed")):
            continue
        entry_id = _to_int(entry.get("target_id"))
        if entry_id is not None and int(entry_id) in destroyed_ids:
            continue
        coord = _normalize_coordinate(entry.get("coordinate"))
        if coord is None:
            continue
        key = (round(float(coord["latitude"]), 7), round(float(coord["longitude"]), 7))
        if key in seen:
            continue
        seen.add(key)
        out.append({"coordinate": coord})
    return out


def _post_attack_hold_seconds_for_enemies(
    current_state: Dict[str, Any],
    enemies: List[Dict[str, Any]],
) -> int:
    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _attack_cover_hold_seconds,
        _average_coordinate,
        _haversine_distance_m,
        _lah_max_attack_speed_mps,
        _normalize_coordinate,
        get_runtime_attack_int,
    )

    own = _normalize_coordinate((current_state or {}).get("coordinate"))
    centre = _average_coordinate(
        [row.get("coordinate") for row in enemies if isinstance(row, dict)]
    )
    if own is None or centre is None:
        return 0
    distance_m = _haversine_distance_m(own, centre)
    speed_mps = _lah_max_attack_speed_mps()
    if distance_m is None or speed_mps <= 0.0:
        return 0
    minimum_s = max(0, get_runtime_attack_int("lah_wait_hold_min_seconds", 30))
    maximum_s = max(minimum_s, get_runtime_attack_int("lah_wait_hold_max_seconds", 600))
    estimate_s = (float(distance_m) / float(speed_mps)) + 2.0 * float(
        _attack_cover_hold_seconds()
    )
    return int(min(maximum_s, max(minimum_s, math.ceil(estimate_s))))


def _current_waypoint_is_planned_regain_cover(
    waypoints: List[Dict[str, Any]],
    current_waypoint_id: Optional[int],
) -> bool:
    """Whether delayed kill processing finds the aircraft already back in cover."""

    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _extract_lah_waypoint_coordinate,
        _normalize_altitude_value,
    )

    current_id = _to_int(current_waypoint_id)
    if current_id is None:
        return False
    current_index = next(
        (
            index
            for index, waypoint in enumerate(waypoints or [])
            if isinstance(waypoint, dict)
            and _to_int(waypoint.get("waypointID")) == int(current_id)
        ),
        None,
    )
    if current_index is None:
        return False
    current = waypoints[int(current_index)]
    attack = current.get("attack") if isinstance(current.get("attack"), dict) else {}
    hover = current.get("hovering") if isinstance(current.get("hovering"), dict) else {}
    if (_to_int(attack.get("targetID")) or 0) > 0 or (_to_int(hover.get("time")) or 0) <= 0:
        return False
    current_coord = _extract_lah_waypoint_coordinate(current)
    current_altitude_m = _normalize_altitude_value((current_coord or {}).get("altitude"))
    if current_altitude_m is None:
        return False
    for previous in reversed(waypoints[: int(current_index)]):
        previous_attack = (
            previous.get("attack") if isinstance(previous.get("attack"), dict) else {}
        )
        if (_to_int(previous_attack.get("targetID")) or 0) <= 0:
            continue
        previous_coord = _extract_lah_waypoint_coordinate(previous)
        previous_altitude_m = _normalize_altitude_value(
            (previous_coord or {}).get("altitude")
        )
        return bool(
            previous_altitude_m is not None
            and int(previous_altitude_m) > int(current_altitude_m)
        )
    return False


def _preserved_regain_cover_waypoint(
    waypoints: List[Dict[str, Any]],
    *,
    current_coord: Optional[Dict[str, Any]],
) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
    """Find the planned post-shot hide waypoint in a current attack suffix."""

    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _extract_lah_waypoint_coordinate,
        _normalize_altitude_value,
    )

    current_altitude_m = _normalize_altitude_value(
        (current_coord or {}).get("altitude") if isinstance(current_coord, dict) else None
    )
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        attack = waypoint.get("attack") if isinstance(waypoint.get("attack"), dict) else {}
        if (_to_int(attack.get("targetID")) or 0) > 0:
            continue
        hover = waypoint.get("hovering") if isinstance(waypoint.get("hovering"), dict) else {}
        if (_to_int(hover.get("time")) or 0) <= 0:
            continue
        coord = _extract_lah_waypoint_coordinate(waypoint)
        altitude_m = _normalize_altitude_value((coord or {}).get("altitude"))
        if coord is None or altitude_m is None:
            continue
        # Equality means a delayed kill event arrived after the aircraft had
        # already reached this exact cover waypoint; it still must be retained.
        if current_altitude_m is not None and int(altitude_m) > int(current_altitude_m):
            continue
        return waypoint, coord
    return None, None


def _state_at_post_attack_cover(
    current_state: Dict[str, Any],
    cover_coord: Dict[str, Any],
) -> Dict[str, Any]:
    """Build the settled state used to validate cover against current contacts."""

    state = deepcopy(current_state or {})
    state["coordinate"] = deepcopy(cover_coord)
    state["speed"] = 0.0
    for key in ("mannedInfo", "unmannedInfo"):
        nested = state.get(key)
        if not isinstance(nested, dict):
            continue
        nested_copy = deepcopy(nested)
        nested_copy["coordinate"] = deepcopy(cover_coord)
        nested_copy["speed"] = 0.0
        state[key] = nested_copy
    return state


def _post_attack_cover_prelude(
    *,
    aircraft_id: int,
    current_state: Dict[str, Any],
    destroyed_target_id: Optional[int],
    emit: LogCallback,
    log_prefix: str,
) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]], int]:
    """Plan cover for the run home while contacts are still alive.

    Returns ``(plan, enemy_coordinates, hold_seconds)``.  The command aircraft
    keeps its relay links; wingmen only need to be hidden.  With no live
    contact the return is unchanged.
    """

    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _plan_lah_enemy_contact_response,
        get_runtime_attack_int,
    )

    enemies = _remaining_enemy_coordinates(destroyed_target_id)
    if not enemies:
        return None, [], 0

    command_aircraft_id = get_runtime_attack_int("command_aircraft_id", 1)
    is_command_relay = int(aircraft_id) == int(command_aircraft_id)
    descriptor = {
        "aircraft_id": int(aircraft_id),
        "mode": "LAH_RELAY" if is_command_relay else "LAH_HOLD_RESUME",
        "enemy_contact": {
            "uav_states": _post_attack_uav_states(),
            "enemy_coordinates": [row["coordinate"] for row in enemies],
            "enemy_input_count": len(enemies),
            "enemy_coordinate_count": len(enemies),
        },
    }
    plan = _plan_lah_enemy_contact_response(
        descriptor,
        current_state or {},
        role="relay" if is_command_relay else "attacker",
        emit=emit,
    )

    hold_seconds = _post_attack_hold_seconds_for_enemies(current_state, enemies)

    emit(
        f"{log_prefix} aircraft {aircraft_id} returns under cover "
        f"(enemies={len(enemies)}, role={'relay' if is_command_relay else 'hide'}, "
        f"hold={hold_seconds}s, certified={bool((plan or {}).get('applied'))})."
    )
    return plan, enemies, hold_seconds


def _post_attack_uav_states() -> List[Dict[str, Any]]:
    """Live UAV positions for the relay link check."""

    from modules.common import agent_status_snapshot
    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _index_agent_states,
        _normalize_coordinate,
    )

    try:
        snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
        index = _index_agent_states(
            snapshot.get("agent_states") or [],
            waypoint_memory=snapshot.get("last_nonzero_waypoint_by_aircraft"),
        )
    except Exception:
        return []
    out: List[Dict[str, Any]] = []
    for aircraft_id, state in sorted((index or {}).items()):
        try:
            identifier = int(aircraft_id)
        except (TypeError, ValueError):
            continue
        if identifier <= 3:  # 1-3 are the manned flight, not relay endpoints
            continue
        coord = _normalize_coordinate((state or {}).get("coordinate"))
        if coord is not None:
            out.append({"aircraft_id": identifier, "coordinate": coord})
    return out


def _build_post_attack_lah_resume_update(
    *,
    source_plan_id: int,
    current_input_id: int,
    target_id: int,
    aircraft_id: int,
    current_state: Dict[str, Any],
    now_ms: int,
    emit: LogCallback,
    log_prefix: str,
    run_cache: Optional[_PostAttackRunCache] = None,
    exclude_all_target_missions: bool = False,
    retained_target_ids: Any = None,
) -> Optional[Dict[str, Any]]:
    from modules.mission_planning.replanning.triggers.attack.pipeline import (
        _lah_waypoints_to_coordinate_list,
        _predict_lah_followup_anchor,
        _split_done_resume_lah_path,
        _trim_lah_waypoints_before_anchor,
    )

    imp_data = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
    )
    if not isinstance(imp_data, dict):
        emit(f"{log_prefix} IMP load failed for aircraft {aircraft_id}.")
        return None
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        emit(f"{log_prefix} IMP mission list missing for aircraft {aircraft_id}.")
        return None

    attack_target_indices = _lah_attack_target_mission_indices(
        mission_list,
        current_input_id=int(current_input_id),
        target_id=int(target_id),
        exclude_all_target_missions=bool(exclude_all_target_missions),
        retained_target_ids=retained_target_ids,
    )
    attack_target_indices = _preserve_legacy_lah_target_bound_resumes(
        mission_list,
        attack_target_indices,
        run_cache=run_cache,
        emit=emit,
        log_prefix=log_prefix,
    )
    if not attack_target_indices:
        return None

    current_wp = _to_int((current_state or {}).get("current_waypoint_id"))
    artifacts = _resolve_plan_artifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        current_waypoint_id=current_wp,
        emit=emit,
        allow_first_mission_fallback=True,
    )
    if artifacts is None:
        emit(f"{log_prefix} plan artifacts unavailable for aircraft {aircraft_id}.")
        return None

    current_idx = next(
        (
            idx
            for idx, mission in enumerate(mission_list)
            if isinstance(mission, dict)
            and _to_int(mission.get("individualMissionID"))
            == _to_int(getattr(artifacts, "individual_mission_id", None))
        ),
        None,
    )
    attack_last_idx = max(int(idx) for idx in attack_target_indices)
    preserve_current_attack_suffix = bool(
        current_idx is not None and int(current_idx) in attack_target_indices
    )
    if current_idx is None:
        start_idx = int(attack_last_idx + 1)
    elif current_idx in attack_target_indices:
        # The current attack path ends with the certified descent/cover hold.
        # Start from this mission and split after the active attack waypoint so
        # a kill notification cannot discard that safety-critical suffix.
        start_idx = int(current_idx)
    else:
        start_idx = int(current_idx)

    removed_attack_indices = {int(idx) for idx in attack_target_indices}
    removed_target_ids = {
        int(mission_target_id)
        for idx in removed_attack_indices
        for mission_target_id in [_mission_target_id(mission_list[idx])]
        if mission_target_id is not None and int(mission_target_id) > 0
    }
    remaining_live_attack_target_ids = _remaining_lah_live_attack_target_ids(
        mission_list,
        start_index=int(current_idx if current_idx is not None else start_idx),
        removed_target_ids=removed_target_ids,
        run_cache=run_cache,
    )
    if remaining_live_attack_target_ids:
        emit(
            f"{log_prefix} target-specific close keeps sequential attack(s) "
            f"on aircraft {aircraft_id}: {sorted(remaining_live_attack_target_ids)}."
        )
        return {
            "aircraft_id": int(aircraft_id),
            "preserveExistingPackage": True,
            "continuingAttack": True,
            "remainingAttackTargetIDs": sorted(remaining_live_attack_target_ids),
            "generatedPathIDs": [],
            "reservationSummaries": [],
        }

    current_coord = (
        _normalize_coordinate((current_state or {}).get("coordinate"))
        or _normalize_coordinate(((current_state or {}).get("mannedInfo") or {}).get("coordinate"))
        or _normalize_coordinate(((current_state or {}).get("unmannedInfo") or {}).get("coordinate"))
    )
    split_trim_anchor = (
        _predict_lah_followup_anchor(
            current_coord,
            current_state or {},
            enable_prediction=True,
        )
        if current_coord is not None
        else None
    )
    replacement_missions: List[Dict[str, Any]] = []
    primary_path_rows: List[Tuple[Path, Dict[str, Any]]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    generated_path_ids: Set[int] = set()
    reservation_summaries: List[Dict[str, Any]] = []
    reserved_individual_ids: List[int] = []
    reserved_path_ids: List[int] = []
    reserved_imp_ids: List[int] = []
    removed_wp_id: Optional[int] = None
    first_kept_index: Optional[int] = None
    primary_resume: Optional[Tuple[Dict[str, Any], Dict[str, Any], List[Dict[str, Any]]]] = None
    preserved_cover_coord: Optional[Dict[str, Any]] = None
    primary_transition_waypoint_count = 0

    for candidate_idx in range(max(0, int(start_idx)), len(mission_list)):
        if (
            int(candidate_idx) in removed_attack_indices
            and (current_idx is None or int(candidate_idx) != int(current_idx))
        ):
            continue
        source_mission = mission_list[candidate_idx]
        if not isinstance(source_mission, dict):
            continue
        source_path_id = _to_int(source_mission.get("pathID"))
        if source_path_id is None or source_path_id <= 0:
            continue
        try:
            source_path = read_json_cached(
                db_paths.get_db_subpath("FlightPath", f"{int(source_path_id)}.json"),
                kind="FlightPath",
            )
        except Exception as exc:
            emit(
                f"{log_prefix} failed to load FlightPath {source_path_id} "
                f"for aircraft {aircraft_id}: {exc}"
            )
            return None

        resume_waypoints = _extract_lah_waypoint_list(source_path)
        if not resume_waypoints:
            continue

        if (
            current_idx is not None
            and candidate_idx == int(current_idx)
            and _to_int(getattr(artifacts, "path_id", None)) == int(source_path_id)
        ):
            preserving_current_suffix = bool(
                preserve_current_attack_suffix and candidate_idx == int(current_idx)
            )
            current_is_regain_cover = bool(
                preserving_current_suffix
                and _current_waypoint_is_planned_regain_cover(
                    resume_waypoints,
                    current_wp,
                )
            )
            _, resume_waypoints, removed_wp_id = _split_done_resume_lah_path(
                source_path,
                artifacts=artifacts,
                current_coord=current_coord,
                emit=emit,
                force_nonempty_resume=False,
                # At the firing waypoint, drop that exposed point and keep the
                # descent.  If a delayed kill event arrives after the aircraft
                # has already reached cover, keep the current cover waypoint.
                exclude_current_from_resume=(
                    preserving_current_suffix and not current_is_regain_cover
                ),
                resume_trim_anchor_coord=(
                    None if preserving_current_suffix else split_trim_anchor
                ),
            )
            if not resume_waypoints:
                continue
            if preserving_current_suffix:
                _cover_wp, candidate_cover_coord = _preserved_regain_cover_waypoint(
                    resume_waypoints,
                    current_coord=current_coord,
                )
                if candidate_cover_coord is not None:
                    preserved_cover_coord = dict(candidate_cover_coord)

        if current_coord is not None:
            preserving_cover_suffix = bool(
                preserved_cover_coord is not None
                and candidate_idx == int(current_idx)
            )
            trim_anchor = (
                current_coord
                if preserving_cover_suffix
                else (split_trim_anchor or current_coord)
            )
            if (
                not preserving_cover_suffix
                and _extract_related_input_mission_id(source_mission) == int(current_input_id)
            ):
                resume_waypoints, _ = _trim_lah_waypoints_before_anchor(
                    resume_waypoints,
                    trim_anchor,
                    emit=emit,
                    log_prefix=log_prefix,
                    aircraft_id=int(aircraft_id),
                    path_id=int(source_path_id),
                )
            if resume_waypoints:
                anchor = _normalize_coordinate(trim_anchor or current_coord)
                if anchor is not None:
                    prepared_primary_transition = _build_post_attack_lah_transition_prefix(
                        start_waypoint={
                            "coordinate": dict(anchor),
                            "speed": resume_waypoints[0].get("speed", 40.0),
                        },
                        destination_waypoint=resume_waypoints[0],
                        aircraft_id=int(aircraft_id),
                    )
                    if prepared_primary_transition is not None:
                        transition_prefix = [
                            deepcopy(item)
                            for item in prepared_primary_transition.get("prefixWaypoints") or []
                            if isinstance(item, dict)
                        ]
                        resume_waypoints = transition_prefix + resume_waypoints
                        relink_waypoints(resume_waypoints)
                        primary_transition_waypoint_count += len(transition_prefix)
                        emit(
                            f"{log_prefix} inserted DEM-safe current-to-resume connector "
                            f"(aircraft={aircraft_id}, path={source_path_id}, "
                            f"horizontal={float(prepared_primary_transition.get('horizontalGapM') or 0.0):.1f}m, "
                            f"altitudeDelta={int(prepared_primary_transition.get('altitudeGapM') or 0)}m, "
                            f"addedWaypoints={len(transition_prefix)})."
                        )
        if not resume_waypoints:
            continue

        primary_resume = (source_mission, source_path, resume_waypoints)
        first_kept_index = int(candidate_idx)
        break

    cover_plan: Optional[Dict[str, Any]] = None
    cover_role = "hold"
    if primary_resume is not None:
        source_mission, source_path, resume_waypoints = primary_resume
        cover_enemies = _remaining_enemy_coordinates(int(target_id))
        if preserved_cover_coord is not None:
            # Always execute the already-planned descent first.  Then validate
            # from that settled point against the *current* live contact set;
            # contacts discovered during the attack were not part of the old
            # certificate and may require one additional, justified cover move.
            cover_state = _state_at_post_attack_cover(
                current_state or {},
                preserved_cover_coord,
            )
            cover_plan, cover_enemies, cover_hold_s = _post_attack_cover_prelude(
                aircraft_id=int(aircraft_id),
                current_state=cover_state,
                destroyed_target_id=int(target_id),
                emit=emit,
                log_prefix=log_prefix,
            )
            from modules.mission_planning.replanning.triggers.attack.pipeline import (
                _lah_tactical_endpoint_coordinate,
                get_runtime_attack_int,
            )

            command_aircraft_id = get_runtime_attack_int("command_aircraft_id", 1)
            cover_role = (
                "relay" if int(aircraft_id) == int(command_aircraft_id) else "hold"
            )
            if cover_enemies:
                resume_waypoints, cover_plan, cover_role = _append_post_attack_cover_route(
                    resume_waypoints,
                    aircraft_id=int(aircraft_id),
                    plan=cover_plan,
                    hold_seconds=int(cover_hold_s),
                    emit=emit,
                    log_prefix=log_prefix,
                )
                revalidated_endpoint = _lah_tactical_endpoint_coordinate(cover_plan)
                if revalidated_endpoint is not None:
                    preserved_cover_coord = dict(revalidated_endpoint)
                    emit(
                        f"{log_prefix} post-attack cover revalidated against current "
                        f"contacts (aircraft={aircraft_id}, enemies={len(cover_enemies)}, "
                        f"hold={cover_hold_s}s)."
                    )
                else:
                    # Do not publish a stale point as certified.  The descent
                    # and fail-closed hold remain in the route.
                    preserved_cover_coord = None
                    emit(
                        f"{log_prefix} no cover point certified against the current "
                        f"contact set after descent (aircraft={aircraft_id}, "
                        f"enemies={len(cover_enemies)})."
                    )
            else:
                # With no live contact there is nothing left to revalidate, but
                # retain the descent marker for SIM/display provenance.
                cover_plan = {
                    "applied": True,
                    "status": "preserved_cover_no_live_contacts",
                    "enemyVisibleCount": 0,
                    "enemyAnalyzedCount": 0,
                    "endpoint": dict(preserved_cover_coord),
                }
            primary_resume = (source_mission, source_path, resume_waypoints)
            emit(
                f"{log_prefix} preserved planned attack descent before further "
                f"movement (aircraft={aircraft_id}, "
                f"remainingEnemies={len(cover_enemies)}, hold={cover_hold_s}s)."
            )
        else:
            cover_plan, cover_enemies, cover_hold_s = _post_attack_cover_prelude(
                aircraft_id=int(aircraft_id),
                current_state=current_state or {},
                destroyed_target_id=int(target_id),
                emit=emit,
                log_prefix=log_prefix,
            )
            if cover_enemies:
                resume_waypoints, cover_plan, cover_role = _splice_post_attack_cover_prelude(
                    resume_waypoints,
                    aircraft_id=int(aircraft_id),
                    plan=cover_plan,
                    hold_seconds=int(cover_hold_s),
                    emit=emit,
                    log_prefix=log_prefix,
                )
                primary_resume = (source_mission, source_path, resume_waypoints)

    clone_start_idx = int(first_kept_index + 1) if first_kept_index is not None else max(0, int(start_idx))
    follow_up_source_missions = [
        mission
        for absolute_idx, mission in enumerate(
            mission_list[clone_start_idx:], start=int(clone_start_idx)
        )
        if int(absolute_idx) not in removed_attack_indices
    ]
    try:
        prepared_follow_up_connector = _prepare_post_attack_lah_follow_up_connector(
            primary_resume=primary_resume,
            follow_up_source_missions=follow_up_source_missions,
            aircraft_id=int(aircraft_id),
            run_cache=run_cache,
            emit=emit,
            log_prefix=log_prefix,
        )
    except Exception as exc:
        emit(
            f"{log_prefix} refusing a discontinuous LAH return package "
            f"(aircraft={aircraft_id}, error={exc!r})."
        )
        return None
    clone_count = _post_attack_follow_up_clone_count(follow_up_source_missions)
    reserved_path_count = int(clone_count) + (1 if primary_resume is not None else 0)
    reserved_waypoint_count = len(primary_resume[2]) if primary_resume is not None else 0
    for follow_up_mission in follow_up_source_missions:
        if not isinstance(follow_up_mission, dict):
            continue
        if _skip_replan_follow_up_reason(follow_up_mission, excluded_input_ids=set()) is not None:
            continue
        follow_up_path_id = _to_int(follow_up_mission.get("pathID"))
        if follow_up_path_id is None or follow_up_path_id <= 0:
            continue
        try:
            follow_up_path = read_json_cached(
                db_paths.get_db_subpath("FlightPath", f"{int(follow_up_path_id)}.json"),
                kind="FlightPath",
            )
        except Exception:
            continue
        for waypoint_key in ("waypointList", "uavWaypointList", "lahWaypointList"):
            follow_up_waypoints = follow_up_path.get(waypoint_key)
            if isinstance(follow_up_waypoints, list):
                reserved_waypoint_count += sum(1 for item in follow_up_waypoints if isinstance(item, dict))
    reserved_waypoint_count += sum(
        1
        for item in ((prepared_follow_up_connector or {}).get("prefixWaypoints") or [])
        if isinstance(item, dict)
    )
    id_reservation = (
        ReplanIdReservation.reserve(
            imp_count=1,
            individual_count=reserved_path_count,
            path_count_by_aircraft={int(aircraft_id): reserved_path_count},
            waypoint_count=int(reserved_waypoint_count),
        )
        if reserved_path_count > 0
        else None
    )

    if primary_resume is not None:
        source_mission, source_path, resume_waypoints = primary_resume
        if id_reservation is None:
            return None
        individual_id = int(id_reservation.next_individual())
        path_id = int(id_reservation.next_path(int(aircraft_id)))
        reserved_individual_ids.append(int(individual_id))
        reserved_path_ids.append(int(path_id))
        mission_entry = deepcopy(source_mission)
        mission_entry["individualMissionID"] = int(individual_id)
        mission_entry["pathID"] = int(path_id)
        mission_entry["isDone"] = False
        info = (
            deepcopy(mission_entry.get("individualMissionInfo"))
            if isinstance(mission_entry.get("individualMissionInfo"), dict)
            else {}
        )
        info["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)
        mission_entry["individualMissionInfo"] = info
        replacement_missions.append(mission_entry)

        path_payload = _build_lah_path_payload_from_waypoints(
            template_path=source_path,
            aircraft_id=int(aircraft_id),
            path_id=int(path_id),
            individual_mission_id=int(individual_id),
            waypoints=resume_waypoints,
            now_ms=int(now_ms),
            waypoint_id_provider=id_reservation.next_waypoint,
        )
        _apply_runtime_flyover_to_flight_path_payload(path_payload)
        sanitize_flight_path_payload_filming_altitudes(path_payload)
        if cover_plan is not None or primary_transition_waypoint_count > 0:
            _rebase_post_attack_cover_timing(path_payload)
        if cover_plan is not None:
            # The concealment endpoint has no ICD field; record it out of band
            # so SIM can draw it.
            from modules.mission_planning.replanning.triggers.attack.pipeline import (
                _lah_tactical_endpoint_coordinate,
                _record_lah_tactical_points,
            )

            _record_lah_tactical_points(
                path_id=int(path_id),
                waypoints=path_payload.get("lahWaypointList") or [],
                plan=cover_plan,
                role=cover_role,
                # Without this the resume route's last waypoint - kilometres
                # away - would be labelled as the concealment point.
                conceal_coordinate=(
                    preserved_cover_coord
                    or _lah_tactical_endpoint_coordinate(cover_plan)
                ),
            )
        path_dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        primary_path_rows.append((path_dest, path_payload))
        generated_path_ids.add(int(path_id))

    if prepared_follow_up_connector is not None:
        if id_reservation is None:
            return None
        connector_prefix = [
            item
            for item in prepared_follow_up_connector.get("prefixWaypoints") or []
            if isinstance(item, dict)
        ]
        if connector_prefix:
            # Allocate in execution order: primary path, connector prefix, then
            # the cloned follow-up waypoints.  This keeps IDs monotonic within
            # the prepended path as well as globally unique.
            reassign_unique_waypoint_ids_inplace(
                connector_prefix,
                waypoint_id_provider=id_reservation.next_waypoint,
            )
            prepared_follow_up_connector["prefixWaypoints"] = connector_prefix

    cloned_artifacts = _clone_follow_up_replan_artifacts(
        missions=follow_up_source_missions,
        aircraft_id=int(aircraft_id),
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
        individual_id_provider=id_reservation.next_individual if id_reservation is not None and clone_count > 0 else None,
        path_id_provider=id_reservation.next_path if id_reservation is not None and clone_count > 0 else None,
        waypoint_id_provider=id_reservation.next_waypoint if id_reservation is not None and clone_count > 0 else None,
        reservation_summaries=reservation_summaries,
        reservation_scope="postAttackLahResumeFollowUp",
    )
    if cloned_artifacts is None:
        return None
    follow_up_missions, follow_up_paths = cloned_artifacts
    return_connector_waypoint_count = 0
    if prepared_follow_up_connector is not None:
        if id_reservation is None:
            return None
        try:
            return_connector_waypoint_count = _prepend_post_attack_lah_follow_up_connector(
                prepared=prepared_follow_up_connector,
                follow_up_missions=follow_up_missions,
                follow_up_paths=follow_up_paths,
                waypoint_id_provider=None,
                aircraft_id=int(aircraft_id),
                emit=emit,
                log_prefix=log_prefix,
            )
        except Exception as exc:
            emit(
                f"{log_prefix} refusing a discontinuous cloned LAH return package "
                f"(aircraft={aircraft_id}, error={exc!r})."
            )
            return None
    replacement_missions.extend(follow_up_missions)
    generated_path_ids.update(
        int(_to_int((payload or {}).get("pathID")) or 0)
        for _, payload in follow_up_paths
        if _to_int((payload or {}).get("pathID")) is not None
    )

    if not replacement_missions:
        target_text = "all active attack targets" if exclude_all_target_missions else f"targetID={target_id}"
        emit(
            f"{log_prefix} no remaining missions left after dropping closed attack branch "
            f"(aircraft={aircraft_id}, {target_text})."
        )
        return None

    # The original mission list is replaced wholesale below; copy only the
    # package shell to avoid duplicating every nested mission twice.
    new_imp_data = _copy_post_attack_imp_shell(imp_data)
    if id_reservation is None:
        return None
    new_imp_id = int(id_reservation.next_imp())
    reserved_imp_ids.append(int(new_imp_id))
    new_imp_data["individualMissionPackageID"] = int(new_imp_id)
    new_imp_data["timestamp"] = int(now_ms)
    new_imp_data["individualMissionList"] = [deepcopy(mission) for mission in replacement_missions]
    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(new_imp_id)}.json")
    imp_dest.parent.mkdir(parents=True, exist_ok=True)
    generated_flight_paths = [
        payload
        for _dest, payload in [*primary_path_rows, *follow_up_paths]
        if isinstance(payload, dict)
    ]
    # Mission-level removal is keyed on the branch's declared targetID, but a
    # waypoint carried over from an earlier engagement can still name the target
    # that has just been serviced. Scrub by waypoint too, or the manned aircraft
    # resumes carrying a live attack on a target that is already gone.
    finished_target_ids = {int(target_id)} if int(target_id) > 0 else set()
    finished_target_ids -= {
        int(value) for value in (retained_target_ids or []) if _to_int(value)
    }
    scrubbed = _clear_waypoint_attacks_for_targets(
        generated_flight_paths, finished_target_ids
    )
    if scrubbed:
        emit(
            f"{log_prefix} cleared {scrubbed} stale attack waypoint(s) for "
            f"target(s) {sorted(finished_target_ids)} on aircraft {aircraft_id}."
        )
    _validate_generated_post_attack_artifact_payloads(
        individual_mission_plans=[new_imp_data],
        flight_paths=generated_flight_paths,
        scope=f"postAttackLahResume:{new_imp_id}",
        allow_existing_db_artifacts=True,
        log=emit,
    )
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [(imp_dest, new_imp_data)]
    for dest, payload in primary_path_rows:
        dest.parent.mkdir(parents=True, exist_ok=True)
        sanitize_flight_path_payload_filming_altitudes(payload)
        write_entries.append((dest, payload))
    for dest, payload in follow_up_paths:
        dest.parent.mkdir(parents=True, exist_ok=True)
        sanitize_flight_path_payload_filming_altitudes(payload)
        write_entries.append((dest, payload))
    _write_or_defer_post_attack_json_batch(
        write_entries,
        run_cache=run_cache,
    )

    emit(
        f"{log_prefix} returning LAH resume package written "
        f"(aircraft={aircraft_id}, imp={imp_dest.name}, missions={len(replacement_missions)})."
    )
    direct_reservation = _post_attack_reservation_event(
        scope="postAttackLahResume",
        aircraft_id=int(aircraft_id),
        imp_ids=reserved_imp_ids,
        individual_ids=reserved_individual_ids,
        path_ids_by_aircraft={int(aircraft_id): reserved_path_ids},
    )
    reservation_summaries.insert(0, direct_reservation)
    return {
        "aircraft_id": int(aircraft_id),
        "individualMissionPackageID": int(new_imp_id),
        "generatedPathIDs": sorted(int(pid) for pid in generated_path_ids if int(pid) > 0),
        "removedWaypointID": removed_wp_id,
        "followUpMissionCount": len(follow_up_missions),
        "returnConnectorWaypointCount": int(return_connector_waypoint_count),
        "primaryTransitionWaypointCount": int(primary_transition_waypoint_count),
        "returnConnectorHorizontalM": (
            round(float(prepared_follow_up_connector.get("horizontalGapM") or 0.0), 1)
            if prepared_follow_up_connector is not None
            else 0.0
        ),
        "continuingAttack": bool(remaining_live_attack_target_ids),
        "remainingAttackTargetIDs": sorted(remaining_live_attack_target_ids),
        "reservedIds": direct_reservation.get("reservedIds", {}),
        "reservationSummaries": reservation_summaries,
    }


def _evaluate_rejoin_group(
    *,
    current_plan_id: int,
    current_input_id: int,
    group_assignments: List[Dict[str, Any]],
    agent_state_map: Dict[int, Dict[str, Any]],
    config: Dict[str, Any],
    emit: LogCallback,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[str, Any]:
    eval_start = time.perf_counter()
    team_aircraft_ids = _aircraft_ids_for_input_mission(
        source_plan_id=int(current_plan_id),
        input_mission_id=int(current_input_id),
    )
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

    ongoing_tracking_aircraft_ids: Set[int] = set()
    for assignment in list_active_tracking_assignments():
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        if _to_int(assignment.get("attack_plan_id")) != int(current_plan_id):
            continue
        if _resolve_assignment_input_mission_id(assignment) != int(current_input_id):
            continue
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        if aircraft_id is None or aircraft_id in closed_aircraft_ids:
            continue
        ongoing_tracking_aircraft_ids.add(int(aircraft_id))

    available_aircraft_ids = {
        int(aid)
        for aid in team_aircraft_ids
        if int(aid) > 3 and int(aid) not in ongoing_tracking_aircraft_ids
    }
    active_aircraft_ids = sorted(int(aid) for aid in available_aircraft_ids if int(aid) not in closed_aircraft_ids)
    returning_aircraft_ids = sorted(int(aid) for aid in closed_aircraft_ids if int(aid) in available_aircraft_ids)

    if not returning_aircraft_ids:
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "returning_aircraft_missing",
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
        }
    if _source_input_mission_is_locked_type2_branch(
        source_plan_id=int(current_plan_id),
        input_mission_id=int(current_input_id),
    ):
        emit(
            f"[POSTATTACK][TYPE2] inputMissionID={current_input_id} immutable branch "
            "ownership preserved; active UAVs stay on their branches and the "
            "returning UAV resumes its own stored suffix."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "type2_branch_owner_resume_preserved",
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "available_aircraft_ids": sorted(available_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
            "evaluation_elapsed_ms": round((time.perf_counter() - eval_start) * 1000.0, 3),
        }
    progress_summary = _summarize_active_group_progress(
        current_plan_id=int(current_plan_id),
        current_input_id=int(current_input_id),
        active_aircraft_ids=active_aircraft_ids,
        run_cache=run_cache,
    )
    active_progress_skip_percent = max(
        0,
        min(
            100,
            _to_int(config.get("active_progress_skip_percent")) or _DEFAULT_ACTIVE_PROGRESS_SKIP_PERCENT,
        ),
    )
    active_avg_progress_percent = progress_summary.get("active_avg_progress_percent")
    active_progress_sample_count = int(progress_summary.get("active_progress_sample_count") or 0)
    active_progress_high = bool(
        active_progress_sample_count > 0
        and active_avg_progress_percent is not None
        and float(active_avg_progress_percent) >= float(active_progress_skip_percent)
    )
    if active_progress_high:
        elapsed_ms = round((time.perf_counter() - eval_start) * 1000.0, 3)
        emit(
            f"[POSTATTACK] inputMissionID={current_input_id} active UAV avg progress high: "
            f"{float(active_avg_progress_percent):.1f}% >= "
            f"{int(active_progress_skip_percent)}%; validating remaining geometry/ETA before skip "
            f"(evalMs={elapsed_ms:.1f})."
        )
    if _remaining_snapshot_explicitly_completed(
        int(current_plan_id),
        int(current_input_id),
        run_cache=run_cache,
    ):
        emit(
            f"[POSTATTACK] inputMissionID={current_input_id} rejoin skipped: "
            "current snapshot explicitly reports mission completion with no remaining geometry."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "remaining_snapshot_completed",
            "remaining_snapshot_state": "completed",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            "evaluation_elapsed_ms": round((time.perf_counter() - eval_start) * 1000.0, 3),
            **progress_summary,
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }
    if not _has_remaining_snapshot_geometry(int(current_plan_id), int(current_input_id), run_cache=run_cache):
        emit(
            f"[POSTATTACK] inputMissionID={current_input_id} rejoin skipped: "
            "current remaining snapshot geometry unavailable."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "remaining_snapshot_unavailable",
            "remaining_snapshot_state": "unavailable",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            "evaluation_elapsed_ms": round((time.perf_counter() - eval_start) * 1000.0, 3),
            **progress_summary,
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    if ongoing_tracking_aircraft_ids:
        # The remaining-area snapshot is an aggregate union.  It does not
        # subtract the executable current-input suffixes retained by ongoing
        # trackers.  Planning that union for the available/returning UAVs would
        # therefore duplicate the trackers' work (the 0727 trace produced a
        # 10.07 km2 suffix twice).  Preserve the existing partition atomically:
        # active UAVs keep their packages and the just-released tracker uses its
        # own recorded return suffix via the normal no-collab branch.
        elapsed_ms = round((time.perf_counter() - eval_start) * 1000.0, 3)
        emit(
            f"[POSTATTACK][AREA-OWNERSHIP] inputMissionID={current_input_id} "
            "collaborative full-remaining redistribution skipped while another "
            "tracker still owns an executable suffix; returning UAV resumes only "
            f"its recorded share (returning={returning_aircraft_ids}, "
            f"active={active_aircraft_ids}, "
            f"tracking={sorted(ongoing_tracking_aircraft_ids)}, evalMs={elapsed_ms:.1f})."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "ongoing_tracker_partition_preserved",
            "remaining_snapshot_state": "partition_preserved",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            "evaluation_elapsed_ms": elapsed_ms,
            **progress_summary,
            "available_aircraft_ids": sorted(available_aircraft_ids),
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    if not active_aircraft_ids:
        # All UAVs that previously owned this input may currently be occupied by
        # target tracking.  When the first one returns there is no active sweep
        # UAV to compare ETA against, but valid remaining geometry is itself
        # sufficient evidence that the returning UAV must take over.  Reuse the
        # normal collaborative path so ongoing trackers remain untouched and a
        # failed plan build cannot partially replace the current plan.
        elapsed_ms = round((time.perf_counter() - eval_start) * 1000.0, 3)
        emit(
            f"[POSTATTACK] inputMissionID={current_input_id} returning-only takeover: "
            f"returningAircraft={returning_aircraft_ids}, "
            f"ongoingTracking={sorted(ongoing_tracking_aircraft_ids)} "
            f"(evalMs={elapsed_ms:.1f})."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": True,
            "skip_reason": None,
            "remaining_snapshot_state": "available",
            "returning_only_takeover": True,
            "active_progress_skip_percent": int(active_progress_skip_percent),
            "evaluation_elapsed_ms": elapsed_ms,
            **progress_summary,
            "available_aircraft_ids": sorted(available_aircraft_ids),
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "active_aircraft_ids": [],
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    reference_coord = _select_rejoin_reference_coordinate(
        active_aircraft_ids=active_aircraft_ids,
        agent_state_map=agent_state_map,
        current_plan_id=int(current_plan_id),
        current_input_id=int(current_input_id),
        run_cache=run_cache,
    )
    if reference_coord is None:
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "rejoin_reference_unavailable",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            "evaluation_elapsed_ms": round((time.perf_counter() - eval_start) * 1000.0, 3),
            **progress_summary,
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    active_remaining_eta_s = _estimate_group_remaining_eta_s(
        source_plan_id=int(current_plan_id),
        current_input_id=int(current_input_id),
        aircraft_ids=active_aircraft_ids,
        agent_state_map=agent_state_map,
        emit=emit,
        run_cache=run_cache,
    )
    return_eta_map: Dict[int, int] = {}
    for assignment in group_assignments:
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        if aircraft_id is None or aircraft_id not in returning_aircraft_ids:
            continue
        state = agent_state_map.get(int(aircraft_id)) or {}
        coord = _normalize_coordinate(state.get("coordinate"))
        if coord is None:
            coord = (
                _normalize_coordinate(assignment.get("handoff_coordinate"))
                or _normalize_coordinate(assignment.get("last_nonzero_coordinate"))
                or _normalize_coordinate(assignment.get("original_coordinate"))
            )
        heading = _to_float(state.get("heading"))
        speed = _to_float(state.get("speed"))
        return_eta_map[int(aircraft_id)] = _estimate_turn_aware_eta_s(
            origin=coord,
            destination=reference_coord,
            heading_deg=heading,
            speed_value=speed,
            turn_radius_m=_to_float(config.get("turn_radius_m")) or _DEFAULT_TURN_RADIUS_M,
            default_cruise_speed_mps=(
                _to_float(config.get("default_cruise_speed_mps")) or _DEFAULT_CRUISE_SPEED_MPS
            ),
        )
    max_return_eta_s = max(return_eta_map.values()) if return_eta_map else 0
    min_remaining_eta_s = max(
        0,
        _to_int(config.get("min_remaining_eta_s")) or _DEFAULT_MIN_REMAINING_ETA_S,
    )
    rejoin_margin_s = max(
        0,
        _to_int(config.get("rejoin_margin_s")) or _DEFAULT_REJOIN_MARGIN_S,
    )

    replan_needed = bool(
        active_remaining_eta_s >= int(min_remaining_eta_s)
        and active_remaining_eta_s > (max_return_eta_s + int(rejoin_margin_s))
    )
    low_progress_guard_percent = max(
        0,
        min(
            100,
            _to_int(config.get("low_progress_eta_guard_percent"))
            or _DEFAULT_LOW_PROGRESS_ETA_GUARD_PERCENT,
        ),
    )
    if (
        not replan_needed
        and not active_progress_high
        and active_progress_sample_count > 0
        and active_avg_progress_percent is not None
        and float(active_avg_progress_percent) < float(low_progress_guard_percent)
    ):
        # 진행률이 거의 0인데 잔여 ETA가 작게 나오는 조합은 경로 축약(전체 스윕이
        # 소수 WP에 압축)으로 ETA가 과소평가된 것 — 스킵하지 말고 재분할한다.
        replan_needed = True
        emit(
            f"[POSTATTACK][THRESHOLD] inputMissionID={current_input_id} low-progress guard: "
            f"avgProgress={float(active_avg_progress_percent):.1f}% < {int(low_progress_guard_percent)}% "
            f"but remainingEta={int(active_remaining_eta_s)}s looked small -> forcing redistribute replan."
        )
    skip_reason = None if replan_needed else (
        "active_group_progress_high" if active_progress_high else "remaining_work_too_small"
    )
    elapsed_ms = round((time.perf_counter() - eval_start) * 1000.0, 3)
    progress_text = ""
    if active_avg_progress_percent is not None:
        progress_text = (
            f" activeAvgProgress={float(active_avg_progress_percent):.1f}%"
            f" activeProgressSkip={int(active_progress_skip_percent)}%"
        )
    emit(
        f"[POSTATTACK][THRESHOLD] inputMissionID={current_input_id} "
        f"activeRemainingEta={int(active_remaining_eta_s)}s "
        f"maxReturnEta={int(max_return_eta_s)}s "
        f"minRemainingEta={int(min_remaining_eta_s)}s "
        f"rejoinMargin={int(rejoin_margin_s)}s "
        f"{progress_text} "
        f"replanNeeded={int(bool(replan_needed))} "
        f"skipReason={skip_reason or '-'} evalMs={elapsed_ms:.1f}."
    )
    return {
        "input_mission_id": int(current_input_id),
        "replan_needed": replan_needed,
        "skip_reason": skip_reason,
        "active_progress_skip_percent": int(active_progress_skip_percent),
        "evaluation_elapsed_ms": elapsed_ms,
        **progress_summary,
        "active_remaining_eta_s": int(active_remaining_eta_s),
        "max_return_eta_s": int(max_return_eta_s),
        "min_remaining_eta_s": int(min_remaining_eta_s),
        "rejoin_margin_s": int(rejoin_margin_s),
        "rejoin_reference": dict(reference_coord),
        "available_aircraft_ids": sorted(available_aircraft_ids),
        "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
        "active_aircraft_ids": active_aircraft_ids,
        "returning_aircraft_ids": returning_aircraft_ids,
        "return_eta_by_aircraft": {int(aid): int(val) for aid, val in return_eta_map.items()},
    }


def _estimate_active_done_hold_seconds(
    *,
    current_plan_id: int,
    current_input_id: int,
    evaluation: Dict[str, Any],
    group_assignments: List[Dict[str, Any]],
    agent_state_map: Dict[int, Dict[str, Any]],
    config: Dict[str, Any],
    run_cache: Optional[_PostAttackRunCache] = None,
) -> int:
    # Completion-boundary holds are an execution acknowledgement window, not
    # a rendezvous ETA.  Keeping this duration invariant prevents one UAV's
    # return distance from turning the other completed UAVs into long holds.
    return int(_POST_ATTACK_COMPLETE_HOLD_SECONDS)


def _post_attack_collab_lookahead_s() -> float:
    return max(
        0.0,
        get_runtime_attack_float(
            "collab_replan_lookahead_s",
            DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
        ),
    )


def _interpolate_post_attack_coordinate(
    start_coord: Dict[str, Any],
    end_coord: Dict[str, Any],
    ratio: float,
) -> Dict[str, Any]:
    clamped = max(0.0, min(1.0, float(ratio)))
    start_alt = _to_float(start_coord.get("altitude"))
    end_alt = _to_float(end_coord.get("altitude"))
    if start_alt is None:
        start_alt = end_alt
    if end_alt is None:
        end_alt = start_alt
    result: Dict[str, Any] = {
        "latitude": float(start_coord["latitude"])
        + (float(end_coord["latitude"]) - float(start_coord["latitude"])) * clamped,
        "longitude": float(start_coord["longitude"])
        + (float(end_coord["longitude"]) - float(start_coord["longitude"])) * clamped,
    }
    if start_alt is not None and end_alt is not None:
        result["altitude"] = int(round(float(start_alt) + (float(end_alt) - float(start_alt)) * clamped))
    return result


def _predict_post_attack_position_linear(
    coord: Dict[str, Any],
    heading_deg: Optional[float],
    speed_value: Optional[float],
    *,
    lookahead_s: float,
) -> Dict[str, Any]:
    if heading_deg is None or coord is None:
        return dict(coord or {})
    speed_mps = _to_mps(speed_value) or _DEFAULT_COLLAB_ENTRY_SPEED_MPS
    dist_m = max(0.0, float(speed_mps) * float(lookahead_s))
    lat = float(coord.get("latitude") or 0.0)
    lon = float(coord.get("longitude") or 0.0)
    heading_rad = math.radians(float(heading_deg))
    lat_scale = 111_132.0
    lon_scale = max(lat_scale * math.cos(math.radians(lat)), 1.0)
    return {
        "latitude": lat + (dist_m * math.cos(heading_rad)) / lat_scale,
        "longitude": lon + (dist_m * math.sin(heading_rad)) / lon_scale,
        "altitude": coord.get("altitude"),
    }


def _predict_post_attack_position_along_current_path(
    *,
    source_plan_id: int,
    aircraft_id: int,
    state: Dict[str, Any],
    lookahead_s: float,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Optional[Dict[str, Any]]:
    current_coord = _normalize_coordinate((state or {}).get("coordinate"))
    if current_coord is None or lookahead_s <= 1e-9:
        return current_coord

    artifacts = _resolve_plan_artifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        current_waypoint_id=_to_int((state or {}).get("current_waypoint_id")),
        emit=lambda _msg: None,
        allow_first_mission_fallback=False,
    )
    if artifacts is None:
        return None
    fp_data = _load_path_payload(
        getattr(artifacts, "path_id", None),
        run_cache=run_cache,
        copy_result=False,
    )
    if not isinstance(fp_data, dict):
        return None

    raw_waypoints: List[Dict[str, Any]] = []
    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        items = fp_data.get(key)
        if isinstance(items, list):
            raw_waypoints = [dict(item) for item in items if isinstance(item, dict)]
            break
    if not raw_waypoints:
        return None

    start_idx = 0
    resolved_wp = _to_int(getattr(artifacts, "current_waypoint_id", None))
    if resolved_wp is not None:
        for idx, waypoint in enumerate(raw_waypoints):
            if _to_int((waypoint or {}).get("waypointID")) == int(resolved_wp):
                start_idx = int(idx)
                break

    path_coords: List[Dict[str, Any]] = [dict(current_coord)]
    for waypoint in raw_waypoints[start_idx:]:
        waypoint_coord = _normalize_coordinate((waypoint or {}).get("coordinate"))
        if waypoint_coord is None:
            continue
        segment_m = _haversine_m(
            float(path_coords[-1]["latitude"]),
            float(path_coords[-1]["longitude"]),
            float(waypoint_coord["latitude"]),
            float(waypoint_coord["longitude"]),
        )
        if segment_m <= 3.0:
            continue
        path_coords.append(dict(waypoint_coord))

    if len(path_coords) < 2:
        return dict(current_coord)

    speed_mps = _to_mps((state or {}).get("speed")) or _DEFAULT_COLLAB_ENTRY_SPEED_MPS
    remaining_distance_m = max(0.0, float(speed_mps) * float(lookahead_s))
    previous_coord = path_coords[0]
    for next_coord in path_coords[1:]:
        segment_distance_m = _haversine_m(
            float(previous_coord["latitude"]),
            float(previous_coord["longitude"]),
            float(next_coord["latitude"]),
            float(next_coord["longitude"]),
        )
        if segment_distance_m <= 1e-6:
            previous_coord = next_coord
            continue
        if remaining_distance_m <= segment_distance_m:
            return _interpolate_post_attack_coordinate(
                previous_coord,
                next_coord,
                remaining_distance_m / segment_distance_m,
            )
        remaining_distance_m -= segment_distance_m
        previous_coord = next_coord
    return dict(path_coords[-1])


def _build_post_attack_collab_agent_state_map(
    *,
    agent_state_map: Dict[int, Dict[str, Any]],
    source_plan_id: int,
    emit: LogCallback,
    log_prefix: str,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[int, Dict[str, Any]]:
    lookahead_s = _post_attack_collab_lookahead_s()
    predicted_map: Dict[int, Dict[str, Any]] = {}
    path_follow_count = 0
    linear_fallback_count = 0
    current_only_count = 0

    for aircraft_id, state in agent_state_map.items():
        aid = _to_int(aircraft_id)
        if aid is None:
            continue
        state_copy = dict(state or {})
        coord = _normalize_coordinate(state_copy.get("coordinate"))
        heading = _to_float(state_copy.get("heading"))
        speed = _to_float(state_copy.get("speed"))
        predicted_coord = None
        if coord is not None:
            state_copy["currentCoordinate"] = dict(coord)
        if coord is not None and lookahead_s > 1e-9:
            predicted_coord = _predict_post_attack_position_along_current_path(
                source_plan_id=int(source_plan_id),
                aircraft_id=int(aid),
                state=state_copy,
                lookahead_s=float(lookahead_s),
                run_cache=run_cache,
            )
            if predicted_coord is not None:
                path_follow_count += 1
            else:
                predicted_coord = _predict_post_attack_position_linear(
                    coord,
                    heading,
                    speed,
                    lookahead_s=float(lookahead_s),
                )
                if predicted_coord != coord:
                    linear_fallback_count += 1
        if predicted_coord is None:
            predicted_coord = coord
            current_only_count += 1
        if predicted_coord is not None:
            state_copy["coordinate"] = dict(predicted_coord)
            state_copy["predictedEntryCoordinate"] = dict(predicted_coord)
            state_copy["predictedEntryEtaS"] = float(lookahead_s)
        else:
            state_copy.pop("predictedEntryCoordinate", None)
            state_copy.pop("predictedEntryEtaS", None)
        predicted_map[int(aid)] = state_copy

    emit(
        f"{log_prefix} Remaining UAV entry lookahead applied: "
        f"{lookahead_s:.1f}s (pathFollow={path_follow_count}, "
        f"linearFallback={linear_fallback_count}, currentOnly={current_only_count})."
    )
    return predicted_map


def _post_attack_authoritative_source_plan_id(
    source_plan_id: int,
    runtime_plan_id: int,
) -> int:
    """Prefer the currently applied plan over the attack's historical source."""

    runtime_id = _to_int(runtime_plan_id)
    if runtime_id is not None and runtime_id > 0:
        return int(runtime_id)
    return int(source_plan_id)


def _prepare_post_attack_collaborative_update(
    *,
    source_plan_id: int,
    runtime_plan_id: int,
    current_input_id: int,
    evaluation: Dict[str, Any],
    group_assignments: List[Dict[str, Any]],
    unavailable_aircraft_ids: Set[int],
    agent_state_map: Dict[int, Dict[str, Any]],
    now_ms: int,
    emit: LogCallback,
    log_prefix: str,
    run_cache: Optional[_PostAttackRunCache] = None,
    reservation_summaries: Optional[List[Dict[str, Any]]] = None,
) -> Optional[CollaborativeResumeReplanResult]:
    authoritative_source_plan_id = _post_attack_authoritative_source_plan_id(
        int(source_plan_id),
        int(runtime_plan_id),
    )
    evaluation["remaining_geometry_source_plan_id"] = int(authoritative_source_plan_id)
    if int(authoritative_source_plan_id) != int(source_plan_id):
        emit(
            f"{log_prefix} Current applied plan selected as remaining-geometry source "
            f"(trackingSourcePlan={int(source_plan_id)}, "
            f"runtimePlan={int(authoritative_source_plan_id)})."
        )
    current_input_mission, next_input_mission = _build_remaining_input_mission_for_collaborative_replan(
        source_plan_id=int(authoritative_source_plan_id),
        current_input_id=int(current_input_id),
        unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids},
    )
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

    active_line_aircraft_ids = sorted(
        int(aid)
        for aid in (evaluation.get("active_aircraft_ids") or [])
        if _to_int(aid) is not None and int(aid) > 0
    )
    if _is_line_rejoin_target(current_input_mission) and active_line_aircraft_ids:
        current_detail = (
            current_input_mission.get("missionDetail")
            if isinstance(current_input_mission.get("missionDetail"), dict)
            else {}
        )
        live_line_detail = build_line_scan_remaining_detail(
            # Rejoin geometry is frozen several seconds after group evaluation.
            # A run-cache read here can therefore be older than the exact
            # current-plan snapshot and move the remaining start backward.
            _load_sweep_progress_safe(run_cache=None),
            source_plan_id=int(runtime_plan_id or source_plan_id),
            input_mission_id=int(current_input_id),
            aircraft_ids=active_line_aircraft_ids,
            source_detail=dict(current_detail or {}),
            common_aircraft_coverage=True,
        )
        if bool(live_line_detail.get("lineRemainingCompleted")):
            emit(
                f"{log_prefix} LINE collaborative replan skipped: "
                "current-plan progress is already complete."
            )
            return None
        if has_line_remaining_geometry(live_line_detail):
            selected_line_detail = deepcopy(live_line_detail)
            takeover_source = "post_attack_cached_active_line_progress"
            remaining_policy = str(
                live_line_detail.get("lineRemainingPolicy")
                or "centerline_interval_union"
            )
            snapshot_guard = _load_exact_line_rejoin_snapshot_guard(
                source_plan_id=int(runtime_plan_id or source_plan_id),
                current_input_id=int(current_input_id),
            )
            if _line_rejoin_snapshot_conflicts_with_live_progress(
                live_line_detail=live_line_detail,
                snapshot_guard=snapshot_guard,
                evaluation=evaluation,
            ):
                selected_line_detail = deepcopy(snapshot_guard["detail"])
                takeover_source = "post_attack_exact_snapshot_conflict_guard"
                remaining_policy = "exact_current_plan_snapshot_conflict_guard"
                emit(
                    f"{log_prefix} LINE progress conflict guarded by exact current-plan snapshot: "
                    f"liveProgress={_active_line_progress_percent(evaluation)}, "
                    f"snapshotProgress={snapshot_guard.get('coveragePercent')}, "
                    f"liveRemaining={_line_detail_length_m(live_line_detail):.1f}m, "
                    f"snapshotRemaining={_line_detail_length_m(selected_line_detail):.1f}m."
                )

            merged_detail = dict(current_detail or {})
            merged_detail.update(selected_line_detail)
            current_input_mission = deepcopy(current_input_mission)
            current_input_mission["missionDetail"] = merged_detail
            current_input_mission["isDone"] = False
            current_input_mission["lineTakeoverSource"] = takeover_source
            current_input_mission["lineTakeoverSourceAircraftIDs"] = list(
                active_line_aircraft_ids
            )
            current_input_mission["lineRemainingPolicy"] = remaining_policy
            first_coords = (
                (selected_line_detail.get("lineList") or [{}])[0].get("coordinateList")
                or []
            )
            first_coord = first_coords[0] if first_coords else {}
            emit(
                f"{log_prefix} LINE rejoin uses exact active assignment domain: "
                f"aircraft={active_line_aircraft_ids}, "
                f"source={takeover_source}, "
                f"fragments={int(selected_line_detail.get('lineRemainingFragmentCount') or len(selected_line_detail.get('lineList') or []))}, "
                f"discardedShortFragments={int(selected_line_detail.get('lineRemainingDiscardedFragmentCount') or 0)}, "
                f"discardedShortLength={float(selected_line_detail.get('lineRemainingDiscardedFragmentLengthM') or 0.0):.1f}m, "
                f"start=({first_coord.get('latitude')}, {first_coord.get('longitude')})."
            )

    # LINE progress is volatile while the attack-close replan is being built.
    # Reuse the exact remaining mission resolved above instead of letting the
    # lower collaborative helper read line_scan_progress / the carried snapshot
    # a second time.  A second read can race the monitor and fall back to the
    # original pre-attack line, resurrecting the already photographed prefix
    # when the active two-UAV assignment is split across three UAVs again.
    current_input_override: Optional[Dict[str, Any]] = (
        deepcopy(current_input_mission)
        if _is_line_rejoin_target(current_input_mission)
        else None
    )
    entry_coord_map_override: Optional[Dict[int, Dict[str, Any]]] = None
    heading_map_override: Optional[Dict[int, float]] = None
    returning_aircraft_ids = {
        int(aid)
        for aid in (evaluation.get("returning_aircraft_ids") or [])
        if _to_int(aid) is not None and int(aid) > 0
    }
    rejoin_template_aircraft_ids: Optional[Set[int]] = None
    if current_input_override is not None and active_line_aircraft_ids:
        # The current attack-time plan intentionally contains only the UAVs
        # that kept scanning.  On rejoin, add the returning UAV explicitly;
        # otherwise the source-plan template lookup silently plans the old
        # two-aircraft set again and the third UAV never receives a share.
        rejoin_template_aircraft_ids = set(active_line_aircraft_ids) | set(
            returning_aircraft_ids
        )
    predicted_agent_state_map = _build_post_attack_collab_agent_state_map(
        agent_state_map={int(aid): dict(state or {}) for aid, state in agent_state_map.items()},
        source_plan_id=int(authoritative_source_plan_id),
        emit=emit,
        log_prefix=log_prefix,
        run_cache=run_cache,
    )

    def _post_attack_flight_path_transform(
        aircraft_id: int,
        path_id: int,
        payload: Dict[str, Any],
    ) -> Dict[str, Any]:
        transformed = _drop_post_attack_collab_leading_prefix_waypoints(
            int(aircraft_id),
            int(path_id),
            payload,
            emit=emit,
        )
        transformed = _boost_post_attack_collab_first_sweep_search_speed(
            int(aircraft_id),
            int(path_id),
            transformed,
            emit=emit,
            reference_coord=(predicted_agent_state_map.get(int(aircraft_id)) or {}).get("coordinate"),
        )
        if int(aircraft_id) in returning_aircraft_ids:
            transformed = _scale_post_attack_returning_first_sweep_fov(
                int(aircraft_id),
                int(path_id),
                transformed,
                emit=emit,
                log_prefix=log_prefix,
            )
        return _reset_post_attack_replacement_path_state(transformed)

    return _prepare_uav_collaborative_resume_replan(
        source_plan_id=int(authoritative_source_plan_id),
        current_input_id=int(current_input_id),
        unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids},
        agent_state_map=predicted_agent_state_map,
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
        drop_prefix_missions=True,
        audit_context="post_attack_collaborative_resume_remaining_input",
        replacement_mission_transform=lambda aircraft_id, missions: _sanitize_post_attack_collaborative_replacements(
            aircraft_id=int(aircraft_id),
            current_input_id=int(current_input_id),
            replacement_missions=missions,
        ),
        flight_path_transform=_post_attack_flight_path_transform,
        current_input_mission_override=current_input_override,
        next_input_mission_override=deepcopy(next_input_mission) if isinstance(next_input_mission, dict) else None,
        entry_coord_map_override=entry_coord_map_override,
        heading_map_override=heading_map_override,
        template_aircraft_ids_override=rejoin_template_aircraft_ids,
        reservation_summaries=reservation_summaries,
    )


def _prepare_post_attack_active_only_remaining_update(
    *,
    source_plan_id: int,
    current_input_id: int,
    evaluation: Dict[str, Any],
    group_assignments: List[Dict[str, Any]],
    agent_state_map: Dict[int, Dict[str, Any]],
    now_ms: int,
    emit: LogCallback,
    log_prefix: str,
    run_cache: Optional[_PostAttackRunCache] = None,
    reservation_summaries: Optional[List[Dict[str, Any]]] = None,
) -> Optional[CollaborativeResumeReplanResult]:
    active_aircraft_ids = {
        int(aid)
        for aid in (evaluation.get("active_aircraft_ids") or [])
        if _to_int(aid) is not None and int(aid) > 0
    }
    if not active_aircraft_ids:
        emit(f"{log_prefix} active-only remaining replan skipped: no active UAVs.")
        return None

    unavailable_aircraft_ids = {
        int(aid)
        for aid in (evaluation.get("ongoing_tracking_aircraft_ids") or [])
        if _to_int(aid) is not None and int(aid) > 0
    }
    unavailable_aircraft_ids.update(
        int(aid)
        for aid in (evaluation.get("returning_aircraft_ids") or [])
        if _to_int(aid) is not None and int(aid) > 0
    )
    unavailable_aircraft_ids.update(
        int(aid)
        for aid in (_to_int(item.get("aircraft_id")) for item in group_assignments)
        if aid is not None and aid > 0
    )

    filtered_agent_state_map = {
        int(aid): dict(agent_state_map.get(int(aid)) or {})
        for aid in sorted(active_aircraft_ids)
    }
    predicted_agent_state_map = _build_post_attack_collab_agent_state_map(
        agent_state_map=filtered_agent_state_map,
        source_plan_id=int(source_plan_id),
        emit=emit,
        log_prefix=log_prefix,
        run_cache=run_cache,
    )

    def _active_only_flight_path_transform(
        aircraft_id: int,
        path_id: int,
        payload: Dict[str, Any],
    ) -> Dict[str, Any]:
        transformed = _drop_post_attack_collab_leading_prefix_waypoints(
            int(aircraft_id),
            int(path_id),
            payload,
            emit=emit,
        )
        transformed = _boost_post_attack_collab_first_sweep_search_speed(
            int(aircraft_id),
            int(path_id),
            transformed,
            emit=emit,
            reference_coord=(predicted_agent_state_map.get(int(aircraft_id)) or {}).get("coordinate"),
        )
        return _reset_post_attack_replacement_path_state(transformed)

    return _prepare_uav_collaborative_resume_replan(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids},
        agent_state_map=predicted_agent_state_map,
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
        drop_prefix_missions=True,
        audit_context="post_attack_active_only_remaining_input",
        replacement_mission_transform=lambda aircraft_id, missions: _sanitize_post_attack_collaborative_replacements(
            aircraft_id=int(aircraft_id),
            current_input_id=int(current_input_id),
            replacement_missions=missions,
        ),
        flight_path_transform=_active_only_flight_path_transform,
        reservation_summaries=reservation_summaries,
    )


def _drop_post_attack_collab_leading_prefix_waypoints(
    aircraft_id: int,
    path_id: int,
    payload: Dict[str, Any],
    *,
    emit: LogCallback,
) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        return payload
    waypoints = payload.get("waypointList")
    if not isinstance(waypoints, list) or len(waypoints) <= 1:
        return payload

    filtered = [item for item in waypoints if isinstance(item, dict)]
    if len(filtered) <= 1:
        return payload

    keep_idx = 0
    while (
        keep_idx < len(filtered) - 1
        and _is_post_attack_collab_entry_prefix_waypoint(filtered[keep_idx])
        and _is_post_attack_collab_sweep_waypoint(filtered[keep_idx + 1])
    ):
        keep_idx += 1

    if keep_idx <= 0:
        return payload

    removed = filtered[:keep_idx]
    # 복사는 트림 확정 후 잔존 구간에만 (조기 반환 경로는 복사 0회)
    trimmed = [deepcopy(item) for item in filtered[keep_idx:]]
    relink_waypoints(trimmed)
    payload["waypointList"] = trimmed
    if "lahWaypointList" in payload:
        payload["lahWaypointList"] = deepcopy(trimmed)

    try:
        from modules.common.eta import annotate_eta_flight_plan

        annotate_eta_flight_plan(
            payload,
            default_speed_mps=40.0,
            waypoint_list_keys=("waypointList",),
            line_search_timing="incoming",
        )
        _repair_post_attack_single_line_search_eta(payload, default_speed_mps=40.0)
        _normalize_post_attack_collab_waypoint_ecf(trimmed)
        if "lahWaypointList" in payload:
            payload["lahWaypointList"] = deepcopy(trimmed)
    except Exception:
        pass

    removed_ids = [
        int(wp_id)
        for wp_id in (_to_int((item or {}).get("waypointID")) for item in removed)
        if wp_id is not None
    ]
    first_kept_id = _to_int((trimmed[0] if trimmed else {}).get("waypointID"))
    emit(
        "[POSTATTACK][COLLAB] Leading entry waypoint prefix removed "
        f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
        f"removedWaypointIDs={removed_ids}, firstWaypointID={first_kept_id})."
    )
    return payload


def _is_post_attack_collab_entry_prefix_waypoint(waypoint: Dict[str, Any]) -> bool:
    if not isinstance(waypoint, dict):
        return False
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return False
    if isinstance(filming.get("lineSearch"), dict):
        return False
    if _to_int(filming.get("operationMode")) != 1:
        return False
    return isinstance(filming.get("coordinateOrientation"), dict)


def _is_post_attack_collab_sweep_waypoint(waypoint: Dict[str, Any]) -> bool:
    if not isinstance(waypoint, dict):
        return False
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return False
    line_search = filming.get("lineSearch")
    if isinstance(line_search, dict):
        coords = line_search.get("coordinateList")
        return not isinstance(coords, list) or len(coords) >= 1
    return _to_int(filming.get("operationMode")) == 2


def _normalize_post_attack_collab_waypoint_ecf(waypoints: List[Dict[str, Any]]) -> None:
    """ICD ecf: litres burned on the leg into each waypoint, not progress."""

    if not waypoints:
        return
    from modules.common.ecf import apply_leg_fuel_inplace

    apply_leg_fuel_inplace(waypoints)


def _repair_post_attack_single_line_search_eta(
    payload: Dict[str, Any],
    *,
    default_speed_mps: float = 40.0,
) -> None:
    if not isinstance(payload, dict):
        return
    waypoints = payload.get("waypointList")
    if not isinstance(waypoints, list) or len(waypoints) != 1:
        return
    waypoint = waypoints[0]
    if not isinstance(waypoint, dict):
        return

    eta_s = _estimate_single_line_search_waypoint_eta_s(
        waypoint,
        default_speed_mps=default_speed_mps,
    )
    if eta_s is None:
        return

    from modules.common.ecf import apply_leg_fuel_inplace

    waypoint["eta"] = int(eta_s)
    # ICD ecf is the fuel burned from the previous waypoint.  This is the only
    # waypoint of the plan, so there is no leg into it - it is not a "100%
    # complete" marker.
    apply_leg_fuel_inplace(waypoints)
    waypoint["nextWaypointID"] = 0
    payload["waypointList"] = waypoints
    if isinstance(payload.get("lahWaypointList"), list) and len(payload.get("lahWaypointList") or []) == 1:
        payload["lahWaypointList"] = deepcopy(waypoints)


def _estimate_single_line_search_waypoint_eta_s(
    waypoint: Dict[str, Any],
    *,
    default_speed_mps: float = 40.0,
) -> Optional[int]:
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    coords = [
        coord
        for coord in (_normalize_coordinate(item) for item in (line_search.get("coordinateList") or []))
        if coord is not None
    ]
    if len(coords) < 2:
        return None

    speed_mps = _to_float(line_search.get("searchSpeed"))
    if speed_mps is None or speed_mps <= 0.0:
        speed_mps = _to_float(waypoint.get("speed"))
    if speed_mps is None or speed_mps <= 0.0:
        speed_mps = float(default_speed_mps)
    if speed_mps <= 0.0:
        return None

    distance_m = 0.0
    for prev_coord, next_coord in zip(coords, coords[1:]):
        distance_m += _haversine_m(
            float(prev_coord["latitude"]),
            float(prev_coord["longitude"]),
            float(next_coord["latitude"]),
            float(next_coord["longitude"]),
        )
    if distance_m <= 0.0:
        return None
    return max(1, int(round(float(distance_m) / float(speed_mps))))


def _boost_post_attack_collab_first_sweep_search_speed(
    aircraft_id: int,
    path_id: int,
    payload: Dict[str, Any],
    *,
    emit: LogCallback,
    speed_scale: float | None = None,
    reference_coord: Dict[str, Any] | None = None,
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
        # AREA capture waypoints carry this stable public marker.  Their speed
        # is already synchronized to the actual incoming flight leg by the
        # next-collab builder (with only its small completion margin).  Applying
        # the generic replan boost again made AREA filming finish halfway to the
        # waypoint.
        if str(waypoint.get("coverageAcquisitionID") or "").startswith(
            "areaMission:"
        ):
            emit(
                "[POSTATTACK][COLLAB] AREA first sweep searchSpeed kept "
                f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
                f"waypointID={_to_int(waypoint.get('waypointID'))}, "
                f"speed={float(search_speed):.2f})."
            )
            return payload
        reference_speed, reference_distance_m = _estimate_post_attack_collab_first_sweep_search_speed_from_reference(
            waypoint,
            reference_coord,
        )
        base_speed = float(search_speed)
        used_reference_base = False
        if reference_speed is not None and float(reference_speed) > 0.0:
            base_speed = float(reference_speed)
            used_reference_base = True
        cruise_speed_mps = max(
            float(_to_float(waypoint.get("speed")) or _DEFAULT_COLLAB_ENTRY_SPEED_MPS),
            float(base_speed),
        )
        boosted_speed = round(
            clamp_line_search_speed_mps(
                base_speed * float(scale),
                cruise_speed_mps=float(cruise_speed_mps),
                speed_scale=float(scale),
                multiplier_cap_enabled=False,
            ),
            2,
        )
        line_search["searchSpeed"] = float(boosted_speed)
        filming["lineSearch"] = line_search
        waypoint["filmingProperty"] = filming
        payload["waypointList"] = waypoints
        if "lahWaypointList" in payload:
            payload["lahWaypointList"] = deepcopy(waypoints)
        try:
            from modules.common.eta import annotate_eta_flight_plan

            annotate_eta_flight_plan(
                payload,
                default_speed_mps=40.0,
                waypoint_list_keys=("waypointList",),
                line_search_timing="incoming",
            )
            _repair_post_attack_single_line_search_eta(payload, default_speed_mps=40.0)
        except Exception:
            pass
        if used_reference_base:
            emit(
                "[POSTATTACK][COLLAB] First sweep searchSpeed boosted "
                f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
                f"waypointID={_to_int(waypoint.get('waypointID'))}, "
                f"factor={scale:.2f}, old={float(search_speed):.2f}, "
                f"refBase={base_speed:.2f}, refDist={float(reference_distance_m or 0.0):.1f}m, "
                f"new={float(boosted_speed):.2f})."
            )
        else:
            emit(
                "[POSTATTACK][COLLAB] First sweep searchSpeed boosted "
                f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
                f"waypointID={_to_int(waypoint.get('waypointID'))}, "
                f"factor={scale:.2f}, old={float(search_speed):.2f}, new={float(boosted_speed):.2f})."
            )
        return payload
    return payload


def _estimate_post_attack_collab_first_sweep_search_speed_from_reference(
    waypoint: Dict[str, Any],
    reference_coord: Dict[str, Any] | None,
) -> tuple[Optional[float], Optional[float]]:
    ref = _normalize_coordinate(reference_coord)
    if ref is None or not isinstance(waypoint, dict):
        return None, None
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return None, None
    line_search = filming.get("lineSearch")
    if not isinstance(line_search, dict):
        return None, None
    raw_coords = line_search.get("coordinateList")
    if not isinstance(raw_coords, list) or len(raw_coords) < 2:
        return None, None
    sweep_coords = [_normalize_coordinate(coord) for coord in raw_coords]
    sweep_coords = [coord for coord in sweep_coords if coord is not None]
    if len(sweep_coords) < 2:
        return None, None

    anchor_coord = _normalize_coordinate(waypoint.get("coordinate"))
    transit_distance_m = _coord_distance_m(ref, anchor_coord) if anchor_coord is not None else None
    if transit_distance_m is None or transit_distance_m <= 1e-6:
        transit_distance_m = _coord_distance_m(ref, sweep_coords[0])
    if transit_distance_m is None or transit_distance_m <= 1e-6:
        return None, None

    sweep_distance_m = 0.0
    prev_coord: Optional[Dict[str, Any]] = None
    for coord in sweep_coords:
        if prev_coord is not None:
            segment_m = _coord_distance_m(prev_coord, coord)
            if segment_m is not None and segment_m > 0.0:
                sweep_distance_m += float(segment_m)
        prev_coord = coord
    if sweep_distance_m <= 1e-6:
        return None, None

    transit_speed_mps = _to_float(waypoint.get("speed")) or _DEFAULT_COLLAB_ENTRY_SPEED_MPS
    if transit_speed_mps <= 0.0:
        transit_speed_mps = _DEFAULT_COLLAB_ENTRY_SPEED_MPS
    effective_transit_distance_m = effective_line_search_transit_m(transit_distance_m)
    if effective_transit_distance_m <= 1e-6:
        return None, None
    transit_time_s = float(effective_transit_distance_m) / float(transit_speed_mps)
    if transit_time_s <= 1e-6:
        return None, None
    search_speed_weight = get_runtime_float("search_speed_weight", 1.1)
    try:
        search_speed_weight = max(0.1, float(search_speed_weight))
    except Exception:
        search_speed_weight = 1.1
    estimated_speed = float(sweep_distance_m) / float(transit_time_s) * float(search_speed_weight)
    return (
        clamp_line_search_speed_mps(
            estimated_speed,
            cruise_speed_mps=float(transit_speed_mps),
            speed_scale=float(search_speed_weight),
            multiplier_cap_enabled=False,
        ),
        float(transit_distance_m),
    )


def _coord_distance_m(a: Dict[str, Any] | None, b: Dict[str, Any] | None) -> Optional[float]:
    if not isinstance(a, dict) or not isinstance(b, dict):
        return None
    lat1 = _to_float(a.get("latitude"))
    lon1 = _to_float(a.get("longitude"))
    lat2 = _to_float(b.get("latitude"))
    lon2 = _to_float(b.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    return _haversine_m(float(lat1), float(lon1), float(lat2), float(lon2))


def _scale_post_attack_returning_first_sweep_fov(
    aircraft_id: int,
    path_id: int,
    payload: Dict[str, Any],
    *,
    emit: LogCallback,
    log_prefix: str = "[POSTATTACK][COLLAB]",
    fov_scale: float | None = None,
) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        return payload
    waypoints = payload.get("waypointList")
    if not isinstance(waypoints, list) or not waypoints:
        return payload

    if fov_scale is None:
        scale = get_runtime_float(
            "post_attack_return_first_fov_scale",
            0.70,
        )
    else:
        try:
            scale = float(fov_scale)
        except Exception:
            scale = 1.0
    scale = max(0.10, min(1.00, float(scale)))
    if abs(scale - 1.0) <= 1e-9:
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
        fov = _to_float(filming.get("fieldOfView"))
        if fov is None or fov <= 0.0:
            continue
        scaled_fov = round(max(0.1, float(fov) * float(scale)), 3)
        filming["fieldOfView"] = float(scaled_fov)
        waypoint["filmingProperty"] = filming
        payload["waypointList"] = waypoints
        if "lahWaypointList" in payload:
            payload["lahWaypointList"] = deepcopy(waypoints)
        emit(
            f"{log_prefix} Returning UAV first sweep FOV scaled "
            f"(aircraft={int(aircraft_id)}, pathID={int(path_id)}, "
            f"waypointID={_to_int(waypoint.get('waypointID'))}, "
            f"factor={scale:.2f}, old={float(fov):.3f}, new={float(scaled_fov):.3f})."
        )
        return payload
    return payload


def _prepend_post_attack_returning_uav_anchor(
    payload: Dict[str, Any],
    *,
    current_coord: Optional[Dict[str, Any]],
) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        return payload

    start_coord = _normalize_coordinate(current_coord)
    if start_coord is None:
        return payload

    waypoints = payload.get("waypointList")
    if not isinstance(waypoints, list) or not waypoints:
        return payload

    first_coord = _normalize_coordinate((waypoints[0] or {}).get("coordinate"))
    if first_coord is not None:
        lat_gap = abs(float(first_coord["latitude"]) - float(start_coord["latitude"]))
        lon_gap = abs(float(first_coord["longitude"]) - float(start_coord["longitude"]))
        alt_gap = abs(float(first_coord.get("altitude") or 0.0) - float(start_coord.get("altitude") or 0.0))
        if lat_gap <= 1e-7 and lon_gap <= 1e-7 and alt_gap <= 1e-3:
            return payload

    anchor_speed = _to_float((waypoints[0] or {}).get("speed")) or 40.0
    anchor_orientation = first_coord or start_coord
    anchor_wp = _build_uav_transit_waypoint(
        coordinate=start_coord,
        speed_mps=float(anchor_speed),
        eta_s=0,
        orientation_coordinate=anchor_orientation,
        waypoint_pass_type=1,
    )
    anchored = [anchor_wp]
    anchored.extend(deepcopy(item) for item in waypoints if isinstance(item, dict))
    reassign_unique_waypoint_ids_inplace(anchored)
    relink_waypoints(anchored)
    payload["waypointList"] = anchored

    try:
        from modules.common.eta import annotate_eta_flight_plan

        annotate_eta_flight_plan(
            payload,
            default_speed_mps=float(anchor_speed),
            waypoint_list_keys=("waypointList",),
            line_search_timing="incoming",
        )
    except Exception:
        pass
    return payload


def _prepare_post_attack_line_phased_rejoin(
    *,
    source_plan_id: int,
    current_input_id: int,
    current_input_mission: Dict[str, Any],
    next_input_mission: Optional[Dict[str, Any]],
    evaluation: Dict[str, Any],
    group_assignments: List[Dict[str, Any]],
    active_aircraft_ids: List[int],
    returning_aircraft_ids: List[int],
    unavailable_aircraft_ids: Set[int],
    agent_state_map: Dict[int, Dict[str, Any]],
    now_ms: int,
    emit: LogCallback,
    log_prefix: str,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Optional[CollaborativeResumeReplanResult]:
    join_delay_s = max(0, _to_int(evaluation.get("max_return_eta_s")) or 0)
    if join_delay_s <= 0:
        return None

    sweep_progress = _load_sweep_progress_safe(run_cache=run_cache)
    phased_sources: Dict[int, _PhasedLineSource] = {}
    suffix_rows: List[Dict[str, Any]] = []
    future_entry_map: Dict[int, Dict[str, Any]] = {}
    future_heading_map: Dict[int, float] = {}

    for aircraft_id in active_aircraft_ids:
        split = _build_active_line_phase_source(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
            current_input_id=int(current_input_id),
            join_delay_s=int(join_delay_s),
            agent_state_map=agent_state_map,
            sweep_progress=sweep_progress,
            emit=emit,
            log_prefix=log_prefix,
            run_cache=run_cache,
        )
        if split is None:
            return None
        phased_sources[int(aircraft_id)] = split
        suffix_row = _build_future_line_row_from_suffix(split)
        if suffix_row is not None:
            suffix_rows.append(suffix_row)
        entry_coord = (
            _normalize_coordinate(split.predicted_entry_coordinate)
            or _normalize_coordinate((agent_state_map.get(int(aircraft_id)) or {}).get("coordinate"))
        )
        if entry_coord is not None:
            future_entry_map[int(aircraft_id)] = entry_coord
        heading_deg = split.predicted_heading_deg
        if heading_deg is None:
            heading_deg = _to_float((agent_state_map.get(int(aircraft_id)) or {}).get("heading"))
        if heading_deg is not None:
            future_heading_map[int(aircraft_id)] = float(heading_deg) % 360.0

    future_remaining_mission = deepcopy(current_input_mission)
    remaining_future_detail = _build_future_line_detail_from_remaining_mission(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        current_input_mission=current_input_mission,
        evaluation=evaluation,
    )
    future_detail = deepcopy(remaining_future_detail) if isinstance(remaining_future_detail, dict) else {}
    if _remaining_detail_has_geometry(future_detail):
        emit(
            f"{log_prefix} phased line rejoin: using ETA-trimmed future centerline "
            f"(futureRows={len(suffix_rows)}, joinDelay={join_delay_s}s)."
        )
    if not _remaining_detail_has_geometry(future_detail):
        if not suffix_rows:
            emit(f"{log_prefix} phased line rejoin skipped: no future line suffix rows remained after join ETA.")
            return None
        emit(f"{log_prefix} phased line rejoin fallback: using current remaining detail.")
        current_detail = (
            current_input_mission.get("missionDetail")
            if isinstance(current_input_mission.get("missionDetail"), dict)
            else {}
        )
        future_detail = deepcopy(current_detail if isinstance(current_detail, dict) else {})
    future_remaining_mission["missionDetail"] = future_detail
    future_remaining_mission["isDone"] = not _remaining_detail_has_geometry(future_detail)
    if bool(future_remaining_mission.get("isDone")):
        emit(f"{log_prefix} phased line rejoin skipped: future suffix geometry collapsed.")
        return None

    for assignment in group_assignments:
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        if aircraft_id is None or aircraft_id not in returning_aircraft_ids:
            continue
        reference_coord = (
            _normalize_coordinate((agent_state_map.get(int(aircraft_id)) or {}).get("coordinate"))
            or _normalize_coordinate(assignment.get("handoff_coordinate"))
            or _normalize_coordinate(assignment.get("last_nonzero_coordinate"))
            or _normalize_coordinate(assignment.get("original_coordinate"))
            or _normalize_coordinate(evaluation.get("rejoin_reference"))
        )
        if reference_coord is None:
            continue
        future_entry_map[int(aircraft_id)] = reference_coord
        heading_deg = _to_float((agent_state_map.get(int(aircraft_id)) or {}).get("heading"))
        if heading_deg is not None:
            future_heading_map[int(aircraft_id)] = float(heading_deg) % 360.0

    representative_entry = _centroid_coordinate(list(future_entry_map.values()))
    future_entry_context_map = build_line_entry_context_map(
        state_map={int(aid): dict(state or {}) for aid, state in agent_state_map.items()},
        entry_coord_map={int(aid): dict(coord) for aid, coord in future_entry_map.items()},
        heading_map={int(aid): float(val) for aid, val in future_heading_map.items()},
    )
    prepared = prepare_next_collab_input_replacements(
        source_plan_id=int(source_plan_id),
        target_input_mission=deepcopy(future_remaining_mission),
        entry_coord_map={int(aid): dict(coord) for aid, coord in future_entry_map.items()},
        heading_map={int(aid): float(val) for aid, val in future_heading_map.items()},
        entry_aircraft_context_map=future_entry_context_map,
        representative_entry=deepcopy(representative_entry) if isinstance(representative_entry, dict) else None,
        next_input_mission=deepcopy(next_input_mission) if isinstance(next_input_mission, dict) else None,
        turn_radius_scale=None,
        now_ms=int(now_ms),
        log=emit,
    )
    if prepared is None or not getattr(prepared, "replacement_by_aircraft", None):
        emit(f"{log_prefix} phased line rejoin skipped: next-collab future suffix preparation failed.")
        return None

    generated_path_ids: Set[int] = set()
    aircraft_imp_ids: Dict[int, int] = {}
    replacement_aircraft_ids: Set[int] = set()
    finish_eta_s = 0

    path_owner_by_id: Dict[int, int] = {}
    for aircraft_id, replacement_missions in (prepared.replacement_by_aircraft or {}).items():
        for mission in replacement_missions or []:
            if not isinstance(mission, dict):
                continue
            path_id = _to_int(mission.get("pathID"))
            if path_id is not None:
                path_owner_by_id[int(path_id)] = int(aircraft_id)

    generated_fp_by_path: Dict[int, Dict[str, Any]] = {}
    generated_fp_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}
    for path_id, payload in (prepared.generated_fp_by_path or {}).items():
        if not isinstance(payload, dict):
            continue
        path_payload = deepcopy(payload)
        _apply_runtime_flyover_to_flight_path_payload(path_payload)
        owner_aircraft_id = path_owner_by_id.get(int(path_id))
        if owner_aircraft_id is not None:
            path_payload = _drop_post_attack_collab_leading_prefix_waypoints(
                int(owner_aircraft_id),
                int(path_id),
                path_payload,
                emit=emit,
            )
            path_payload = _boost_post_attack_collab_first_sweep_search_speed(
                int(owner_aircraft_id),
                int(path_id),
                path_payload,
                emit=emit,
                reference_coord=future_entry_map.get(int(owner_aircraft_id)),
            )
        path_payload = _reset_post_attack_replacement_path_state(path_payload)
        sanitize_flight_path_payload_filming_altitudes(path_payload)
        generated_fp_by_path[int(path_id)] = deepcopy(path_payload)
        if owner_aircraft_id is not None:
            generated_fp_by_aircraft.setdefault(int(owner_aircraft_id), []).append(deepcopy(path_payload))
        finish_eta_s = max(int(finish_eta_s), int(_estimate_uav_flight_path_final_eta_s(path_payload)))

    pending_fp_rows: List[Tuple[Path, Dict[str, Any]]] = []
    for aircraft_id in sorted(int(aid) for aid in (prepared.replacement_by_aircraft or {}).keys()):
        replacements = [
            _sanitize_post_attack_mission_entry(dict(item), current_input_id=int(current_input_id))
            for item in (prepared.replacement_by_aircraft.get(int(aircraft_id)) or [])
            if isinstance(item, dict)
        ]
        if not replacements:
            continue

        phased_replacements: List[Dict[str, Any]] = []
        phased_extra_fp_rows: List[Tuple[Path, Dict[str, Any]]] = []
        if int(aircraft_id) in phased_sources:
            prefix = _build_active_prefix_replacement(
                phased_source=phased_sources[int(aircraft_id)],
                current_input_id=int(current_input_id),
                now_ms=int(now_ms),
            )
            if prefix is not None:
                mission_entry, path_payload = prefix
                phased_replacements.append(mission_entry)
                path_id = _to_int(path_payload.get("pathID"))
                if path_id is not None:
                    path_dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
                    _apply_runtime_flyover_to_flight_path_payload(path_payload)
                    sanitize_flight_path_payload_filming_altitudes(path_payload)
                    phased_extra_fp_rows.append((path_dest, path_payload))
        elif int(aircraft_id) in returning_aircraft_ids:
            transit = _build_returning_transit_replacement(
                source_plan_id=int(source_plan_id),
                aircraft_id=int(aircraft_id),
                current_input_id=int(current_input_id),
                first_replacement=replacements[0],
                generated_fp_by_path=generated_fp_by_path,
                current_state=agent_state_map.get(int(aircraft_id)) or {},
                join_delay_s=int(join_delay_s),
                now_ms=int(now_ms),
                run_cache=run_cache,
            )
            if transit is not None:
                mission_entry, path_payload = transit
                phased_replacements.append(mission_entry)
                path_id = _to_int(path_payload.get("pathID"))
                if path_id is not None:
                    path_dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
                    _apply_runtime_flyover_to_flight_path_payload(path_payload)
                    sanitize_flight_path_payload_filming_altitudes(path_payload)
                    phased_extra_fp_rows.append((path_dest, path_payload))

        phased_replacements.extend(replacements)
        new_imp_id = _write_collaborative_remaining_imp_update(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
            current_input_id=int(current_input_id),
            replacement_missions=phased_replacements,
            now_ms=int(now_ms),
            emit=emit,
            log_prefix=log_prefix,
            drop_prefix_missions=True,
            flight_path_payloads=[
                *generated_fp_by_aircraft.get(int(aircraft_id), []),
                *[payload for _dest, payload in phased_extra_fp_rows if isinstance(payload, dict)],
            ],
        )
        if new_imp_id is None:
            continue
        aircraft_imp_ids[int(aircraft_id)] = int(new_imp_id)
        replacement_aircraft_ids.add(int(aircraft_id))
        for payload in generated_fp_by_aircraft.get(int(aircraft_id), []):
            path_id = _to_int((payload or {}).get("pathID"))
            if path_id is None:
                continue
            pending_fp_rows.append((db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json"), payload))
        pending_fp_rows.extend(phased_extra_fp_rows)

    if not aircraft_imp_ids:
        emit(f"{log_prefix} phased line rejoin skipped: no IMP updates were written.")
        return None

    write_entries: List[Tuple[Path, Dict[str, Any]]] = []
    for path_dest, path_payload in pending_fp_rows:
        if not isinstance(path_payload, dict):
            continue
        path_dest.parent.mkdir(parents=True, exist_ok=True)
        write_entries.append((path_dest, path_payload))
        path_id = _to_int((path_payload or {}).get("pathID"))
        if path_id is not None:
            generated_path_ids.add(int(path_id))
    if write_entries:
        _write_or_defer_post_attack_json_batch(
            write_entries,
            run_cache=run_cache,
        )

    emit(
        f"{log_prefix} phased line rejoin prepared "
        f"(inputMissionID={current_input_id}, joinDelay={join_delay_s}s, "
        f"active={sorted(active_aircraft_ids)}, returning={sorted(returning_aircraft_ids)})."
    )
    return CollaborativeResumeReplanResult(
        current_input_id=int(current_input_id),
        unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids},
        replacement_aircraft_ids=set(int(aid) for aid in replacement_aircraft_ids),
        aircraft_imp_ids={int(aid): int(imp_id) for aid, imp_id in aircraft_imp_ids.items()},
        generated_path_ids=set(int(path_id) for path_id in generated_path_ids),
        finish_eta_s=int(finish_eta_s),
        planner_workflow=str(prepared.planner_workflow or ""),
        planner_result_text=str(prepared.planner_result_text or ""),
    )


def _estimate_group_remaining_eta_s(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_ids: List[int],
    agent_state_map: Dict[int, Dict[str, Any]],
    emit: LogCallback,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> int:
    max_eta_s = 0
    for aircraft_id in aircraft_ids:
        eta_s = _estimate_aircraft_remaining_eta_s(
            source_plan_id=int(source_plan_id),
            current_input_id=int(current_input_id),
            aircraft_id=int(aircraft_id),
            state=dict(agent_state_map.get(int(aircraft_id)) or {}),
            emit=emit,
            run_cache=run_cache,
        )
        max_eta_s = max(int(max_eta_s), int(eta_s))
    return int(max_eta_s)


def _estimate_aircraft_remaining_eta_s(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
    state: Dict[str, Any],
    emit: LogCallback,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> int:
    current_waypoint_id = _to_int(state.get("current_waypoint_id"))
    artifacts = _resolve_plan_artifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        current_waypoint_id=current_waypoint_id,
        emit=lambda _msg: None,
        allow_first_mission_fallback=True,
    )
    if artifacts is None:
        package_eta_s = _estimate_aircraft_package_remaining_eta_s(
            source_plan_id=int(source_plan_id),
            current_input_id=int(current_input_id),
            aircraft_id=int(aircraft_id),
            state=state,
            artifacts=None,
            current_path_eta_s=0,
            run_cache=run_cache,
        )
        emit(
            f"[POSTATTACK] aircraft={aircraft_id} remaining ETA fallback -> "
            f"plan artifacts unavailable, packageEta={int(package_eta_s)}s."
        )
        return int(package_eta_s)

    current_path_eta_s = 0
    fp_data = _load_path_payload(int(artifacts.path_id), run_cache=run_cache, copy_result=False)
    if not isinstance(fp_data, dict):
        current_path_eta_s = 0
    else:
        waypoints = deepcopy(fp_data.get("waypointList") or [])
        if isinstance(waypoints, list) and waypoints:
            start_idx = _find_current_waypoint_index(waypoints, current_waypoint_id)
            if start_idx is None:
                start_idx = _first_not_done_waypoint_index(waypoints)
            if start_idx is None:
                current_path_eta_s = _estimate_line_scan_remaining_eta_s(
                    source_plan_id=int(source_plan_id),
                    current_input_id=int(current_input_id),
                    aircraft_id=int(aircraft_id),
                    state=state,
                    run_cache=run_cache,
                )
                waypoints = []

            if waypoints:
                done_prefix = deepcopy(waypoints[:start_idx]) if start_idx > 0 else []
                resume_waypoints = deepcopy(waypoints[start_idx:])
            else:
                done_prefix = []
                resume_waypoints = []

            sweep_progress = _load_sweep_progress_safe(run_cache=run_cache)
            progress_entry = sweep_progress.get(int(artifacts.path_id)) if isinstance(sweep_progress, dict) else None

            if resume_waypoints and isinstance(progress_entry, dict):
                raw_cut_points = physical_sweep_cut_points(
                    progress_entry,
                    default_buffer_seconds=DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
                )
                done_sweep_points = count_sweep_points_in_waypoints(done_prefix)
                cut_points = max(0, int(raw_cut_points) - int(done_sweep_points))
                if cut_points > 0:
                    resume_waypoints, _ = trim_waypoints_by_sweep_points(
                        resume_waypoints,
                        cut_points,
                        preserve_waypoints=True,
                    )
            if resume_waypoints:
                current_path_eta_s = int(
                    _estimate_uav_flight_path_final_eta_s({"waypointList": resume_waypoints})
                )

    package_eta_s = _estimate_aircraft_package_remaining_eta_s(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        aircraft_id=int(aircraft_id),
        state=state,
        artifacts=artifacts,
        current_path_eta_s=int(current_path_eta_s),
        run_cache=run_cache,
    )
    if int(package_eta_s) > int(current_path_eta_s):
        emit(
            f"[POSTATTACK] aircraft={aircraft_id} active remaining ETA includes follow-up missions "
            f"(currentPathEta={int(current_path_eta_s)}s, packageEta={int(package_eta_s)}s)."
        )
    return max(int(current_path_eta_s), int(package_eta_s))


def _estimate_aircraft_package_remaining_eta_s(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
    state: Dict[str, Any],
    artifacts: Optional[Any],
    current_path_eta_s: int,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> int:
    imp_data = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
        copy_result=False,
    )
    mission_list = imp_data.get("individualMissionList") if isinstance(imp_data, dict) else None
    if not isinstance(mission_list, list) or not mission_list:
        return max(0, int(current_path_eta_s))

    start_idx: Optional[int] = None
    for idx, mission in enumerate(mission_list):
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) == int(current_input_id):
            start_idx = int(idx)
            break
    if start_idx is None and artifacts is not None:
        artifact_mission_id = _to_int(getattr(artifacts, "individual_mission_id", None))
        if artifact_mission_id is not None:
            for idx, mission in enumerate(mission_list):
                if not isinstance(mission, dict):
                    continue
                if _to_int(mission.get("individualMissionID")) == int(artifact_mission_id):
                    start_idx = int(idx)
                    break
    if start_idx is None:
        for idx, mission in enumerate(mission_list):
            if isinstance(mission, dict) and not bool(mission.get("isDone")):
                start_idx = int(idx)
                break
    if start_idx is None:
        return 0

    total_eta_s = 0
    artifact_path_id = _to_int(getattr(artifacts, "path_id", None)) if artifacts is not None else None
    for idx in range(int(start_idx), len(mission_list)):
        mission = mission_list[idx]
        if not isinstance(mission, dict) or bool(mission.get("isDone")):
            continue
        path_id = _to_int(mission.get("pathID"))
        if path_id is None or path_id <= 0:
            continue
        mission_input_id = _extract_related_input_mission_id(mission)
        is_current_mission = idx == int(start_idx)
        if is_current_mission and (artifact_path_id is None or int(path_id) == int(artifact_path_id)):
            eta_s = max(0, int(current_path_eta_s))
            if eta_s <= 0:
                eta_s = _estimate_line_scan_remaining_eta_s(
                    source_plan_id=int(source_plan_id),
                    current_input_id=int(mission_input_id or current_input_id),
                    aircraft_id=int(aircraft_id),
                    state=state,
                    run_cache=run_cache,
                )
        else:
            eta_s = _estimate_path_total_eta_s(int(path_id), run_cache=run_cache)
        total_eta_s += max(0, int(eta_s))
    return int(total_eta_s)


def _estimate_path_total_eta_s(
    path_id: int,
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> int:
    fp_data = _load_path_payload(int(path_id), run_cache=run_cache, copy_result=False)
    if not isinstance(fp_data, dict):
        return 0
    return int(_estimate_uav_flight_path_final_eta_s(fp_data))


def _estimate_line_scan_remaining_eta_s(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
    state: Dict[str, Any],
    run_cache: Optional[_PostAttackRunCache] = None,
) -> int:
    rows_by_aircraft, _ = _collect_line_scan_progress_rows_by_aircraft(
        current_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        active_aircraft_ids={int(aircraft_id)},
        allow_plan_mismatch=False,
        run_cache=run_cache,
    )
    rows = list(rows_by_aircraft.get(int(aircraft_id)) or [])
    if not rows:
        rows_by_aircraft, _ = _collect_line_scan_progress_rows_by_aircraft(
            current_plan_id=int(source_plan_id),
            current_input_id=int(current_input_id),
            active_aircraft_ids={int(aircraft_id)},
            allow_plan_mismatch=True,
            run_cache=run_cache,
        )
        rows = list(rows_by_aircraft.get(int(aircraft_id)) or [])
    remaining_m = 0.0
    for row in rows:
        if not isinstance(row, dict):
            continue
        value = _first_float_field(
            row,
            (
                "remainingLengthM",
                "remaining_length_m",
            ),
        )
        if value is None:
            planned = _first_float_field(row, ("plannedLengthM", "planned_length_m"))
            covered = _first_float_field(row, ("coveredLengthM", "covered_length_m"))
            if planned is not None and covered is not None:
                value = max(0.0, float(planned) - float(covered))
        if value is None:
            line_total = 0.0
            for line_row in row.get("lineList") or []:
                if not isinstance(line_row, dict):
                    continue
                line_value = _first_float_field(
                    line_row,
                    (
                        "remainingLengthM",
                        "remaining_length_m",
                    ),
                )
                if line_value is not None and line_value > 0.0:
                    line_total += float(line_value)
            value = line_total if line_total > 0.0 else None
        if value is not None and value > 0.0:
            remaining_m += float(value)
    if remaining_m <= 1e-6:
        return 0
    speed_mps = _to_mps((state or {}).get("speed")) or _DEFAULT_COLLAB_ENTRY_SPEED_MPS
    return max(1, int(math.ceil(float(remaining_m) / max(1.0, float(speed_mps)))))


def _is_line_rejoin_target(mission: Dict[str, Any]) -> bool:
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []
    return bool(line_list) or (not area_list and len(coord_list) >= 2)


def _load_sweep_progress_safe(
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[int, Dict[str, Any]]:
    return _load_sweep_progress_safe_impl(run_cache=run_cache)


def _load_sweep_progress_safe_impl(
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[int, Dict[str, Any]]:
    if run_cache is not None and run_cache.sweep_progress is not None:
        return {int(pid): dict(entry or {}) for pid, entry in run_cache.sweep_progress.items()}
    try:
        from modules.mission_planning.pipelines.mission_path_trim import load_sweep_progress

        payload = load_sweep_progress()
        if isinstance(payload, dict):
            normalized = {int(pid): dict(entry or {}) for pid, entry in payload.items()}
            if run_cache is not None:
                run_cache.sweep_progress = {int(pid): dict(entry or {}) for pid, entry in normalized.items()}
            return normalized
    except Exception:
        pass
    if run_cache is not None:
        run_cache.sweep_progress = {}
    return {}


def _sweep_progress_entry_has_remaining_imaging(entry: Any) -> bool:
    """Treat an exact path progress sample as authoritative over carrier WPs."""

    if not isinstance(entry, dict):
        return False
    total_raw = (
        entry.get("sweep_point_count")
        if "sweep_point_count" in entry
        else entry.get("sweepPointCount")
    )
    progress_raw = (
        entry.get("progress_points")
        if "progress_points" in entry
        else entry.get("progressPoints")
    )
    total = _to_int(total_raw)
    progress = _to_int(progress_raw)
    remaining_signals: List[bool] = []
    if total is not None and total > 0 and progress is not None:
        remaining_signals.append(int(progress) < int(total))

    percent_raw = (
        entry.get("progress_percent")
        if "progress_percent" in entry
        else entry.get("progressPercent")
    )
    percent = _to_float(percent_raw)
    if percent is not None:
        remaining_signals.append(float(percent) < 100.0)

    remaining_raw = (
        entry.get("remaining_seconds")
        if "remaining_seconds" in entry
        else entry.get("remainingSeconds")
    )
    remaining_seconds = _to_float(remaining_raw)
    if remaining_seconds is not None:
        remaining_signals.append(float(remaining_seconds) > 0.0)
    # Conflicting progress fields fail closed: any credible remaining-work
    # signal keeps the current branch instead of deleting it.
    return any(remaining_signals)


def _sweep_progress_entry_is_authoritative(entry: Any) -> bool:
    if not isinstance(entry, dict):
        return False
    integer_keys = (
        "sweep_point_count",
        "sweepPointCount",
        "progress_points",
        "progressPoints",
    )
    float_keys = (
        "progress_percent",
        "progressPercent",
        "remaining_seconds",
        "remainingSeconds",
    )
    return bool(
        any(key in entry and _to_int(entry.get(key)) is not None for key in integer_keys)
        or any(key in entry and _to_float(entry.get(key)) is not None for key in float_keys)
    )


def _trim_waypoints_for_exact_sweep_progress(
    waypoint_list: List[Dict[str, Any]],
    progress_entry: Dict[str, Any],
    *,
    reference_coord: Optional[Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], int]:
    """Cut photographed points while never returning an empty remaining branch."""

    original = [deepcopy(item) for item in waypoint_list if isinstance(item, dict)]
    if not original:
        return [], 0
    cut_points = max(
        0,
        int(
            physical_sweep_cut_points(
                progress_entry,
                default_buffer_seconds=0.0,
            )
        ),
    )
    if cut_points <= 0 and not is_line_scan_progress_entry(progress_entry):
        cut_points = max(0, int(sweep_progress_points(progress_entry)))
    trimmed, removed_points = trim_waypoints_by_sweep_points(
        original,
        int(cut_points),
        preserve_waypoints=True,
        reference_coord_for_offset=reference_coord,
    )
    # A contradictory sample must fail closed: retaining some geometry is safer
    # than deleting the active LINE and advancing into the next input mission.
    if not trimmed and _sweep_progress_entry_has_remaining_imaging(progress_entry):
        trimmed = [deepcopy(item) for item in waypoint_list if isinstance(item, dict)]
        removed_points = 0
    for waypoint in trimmed:
        waypoint["isDone"] = False
    relink_waypoints(trimmed)
    return trimmed, int(removed_points)


def _load_coverage_progress_safe(
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[str, Any]:
    if run_cache is not None and run_cache.coverage_progress is not None:
        return deepcopy(run_cache.coverage_progress)
    try:
        path = db_paths.get_db_subpath("DSS_Internal", "coverage_progress.json")
        payload = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(payload, dict):
            if run_cache is not None:
                run_cache.coverage_progress = deepcopy(payload)
            return payload
    except Exception:
        pass
    if run_cache is not None:
        run_cache.coverage_progress = {}
    return {}


def _load_line_scan_progress_safe(
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[str, Any]:
    if run_cache is not None and run_cache.line_scan_progress is not None:
        return deepcopy(run_cache.line_scan_progress)
    try:
        path = db_paths.get_db_subpath("DSS_Internal", "line_scan_progress.json")
        payload = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(payload, dict):
            if run_cache is not None:
                run_cache.line_scan_progress = deepcopy(payload)
            return payload
    except Exception:
        pass
    if run_cache is not None:
        run_cache.line_scan_progress = {}
    return {}


def _first_int_field(payload: Dict[str, Any], keys: Tuple[str, ...]) -> Optional[int]:
    for key in keys:
        value = _to_int(payload.get(key))
        if value is not None:
            return int(value)
    return None


def _first_float_field(payload: Dict[str, Any], keys: Tuple[str, ...]) -> Optional[float]:
    for key in keys:
        value = _to_float(payload.get(key))
        if value is not None:
            return float(value)
    return None


def _clamp_progress_percent(value: float) -> int:
    return max(0, min(100, int(round(float(value)))))


def _progress_percent_from_rows(rows: List[Dict[str, Any]]) -> Optional[int]:
    planned_total = 0.0
    covered_total = 0.0
    explicit_values: List[int] = []
    for row in rows:
        if not isinstance(row, dict):
            continue
        percent = _first_int_field(
            row,
            (
                "progressPercent",
                "progress_percent",
                "coveragePercent",
                "coverage_percent",
            ),
        )
        if percent is not None:
            explicit_values.append(_clamp_progress_percent(float(percent)))

        planned = _first_float_field(
            row,
            (
                "plannedLengthM",
                "planned_length_m",
                "planned_area_m2",
            ),
        )
        covered = _first_float_field(
            row,
            (
                "coveredLengthM",
                "covered_length_m",
                "covered_area_m2",
            ),
        )
        remaining = _first_float_field(
            row,
            (
                "remainingLengthM",
                "remaining_length_m",
            ),
        )
        if covered is None and planned is not None and remaining is not None:
            covered = max(0.0, float(planned) - max(0.0, float(remaining)))
        if planned is None or planned <= 0.0 or covered is None:
            continue
        planned_total += float(planned)
        covered_total += max(0.0, min(float(covered), float(planned)))

    if planned_total > 0.0:
        return _clamp_progress_percent((covered_total / planned_total) * 100.0)
    if explicit_values:
        return min(explicit_values)
    return None


def _collect_line_scan_progress_rows_by_aircraft(
    *,
    current_plan_id: Optional[int],
    current_input_id: int,
    active_aircraft_ids: Set[int],
    allow_plan_mismatch: bool,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Tuple[Dict[int, List[Dict[str, Any]]], Dict[int, str]]:
    wanted_aircraft_ids = {
        int(aid)
        for aid in active_aircraft_ids
        if _to_int(aid) is not None and int(aid) > 0
    }
    if not wanted_aircraft_ids:
        return {}, {}

    payload = _load_line_scan_progress_safe(run_cache=run_cache)
    if not isinstance(payload, dict):
        return {}, {}

    entries = payload.get("entries") if isinstance(payload.get("entries"), list) else []
    expected_plan_id = _to_int(current_plan_id)
    top_plan_id = _first_int_field(
        payload,
        ("missionPlanID", "missionPlanId", "mission_plan_id"),
    )
    candidates: List[Tuple[int, Optional[int], Dict[str, Any]]] = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        entry_input_id = _first_int_field(
            entry,
            ("inputMissionID", "inputMissionId", "input_id"),
        )
        if entry_input_id != int(current_input_id):
            continue
        aircraft_id = _first_int_field(
            entry,
            ("aircraftID", "aircraftId", "aircraft_id"),
        )
        if aircraft_id is None or int(aircraft_id) not in wanted_aircraft_ids:
            continue
        entry_plan_id = _first_int_field(
            entry,
            ("missionPlanID", "missionPlanId", "mission_plan_id"),
        )
        candidate_plan_id = entry_plan_id if entry_plan_id is not None else top_plan_id
        if (
            not allow_plan_mismatch
            and expected_plan_id is not None
            and candidate_plan_id is not None
            and int(candidate_plan_id) != int(expected_plan_id)
        ):
            continue
        candidates.append((int(aircraft_id), candidate_plan_id, entry))

    if allow_plan_mismatch and candidates:
        plan_ids = [
            int(plan_id)
            for _, plan_id, _ in candidates
            if plan_id is not None and int(plan_id) > 0
        ]
        if plan_ids:
            latest_plan_id = max(plan_ids)
            candidates = [
                (aircraft_id, plan_id, entry)
                for aircraft_id, plan_id, entry in candidates
                if plan_id is not None and int(plan_id) == int(latest_plan_id)
            ]

    rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    source_by_aircraft: Dict[int, str] = {}
    source_name = "line_scan_progress_latest_fallback" if allow_plan_mismatch else "line_scan_progress"
    for aircraft_id, _plan_id, entry in candidates:
        rows_by_aircraft[int(aircraft_id)].append(entry)
        source_by_aircraft[int(aircraft_id)] = source_name
    return dict(rows_by_aircraft), dict(source_by_aircraft)


def _summarize_active_group_progress(
    *,
    current_plan_id: Optional[int] = None,
    current_input_id: int,
    active_aircraft_ids: List[int],
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[str, Any]:
    active_id_set = {
        int(aid)
        for aid in active_aircraft_ids
        if _to_int(aid) is not None and int(aid) > 0
    }
    if not active_id_set:
        return {
            "active_progress_by_aircraft": {},
            "active_progress_aircraft_ids": [],
            "active_progress_sample_count": 0,
            "active_avg_progress_percent": None,
            "active_progress_source_by_aircraft": {},
            "active_progress_row_count_by_aircraft": {},
        }

    expected_plan_id = _to_int(current_plan_id)
    line_rows_by_aircraft, line_source_by_aircraft = _collect_line_scan_progress_rows_by_aircraft(
        current_plan_id=expected_plan_id,
        current_input_id=int(current_input_id),
        active_aircraft_ids=active_id_set,
        allow_plan_mismatch=False,
        run_cache=run_cache,
    )
    missing_line_aircraft_ids = active_id_set.difference(set(line_rows_by_aircraft))
    if missing_line_aircraft_ids:
        fallback_rows, fallback_sources = _collect_line_scan_progress_rows_by_aircraft(
            current_plan_id=expected_plan_id,
            current_input_id=int(current_input_id),
            active_aircraft_ids=missing_line_aircraft_ids,
            allow_plan_mismatch=True,
            run_cache=run_cache,
        )
        for aircraft_id, rows in fallback_rows.items():
            if rows and int(aircraft_id) not in line_rows_by_aircraft:
                line_rows_by_aircraft[int(aircraft_id)] = rows
                line_source_by_aircraft[int(aircraft_id)] = fallback_sources.get(
                    int(aircraft_id),
                    "line_scan_progress_latest_fallback",
                )

    coverage_payload = _load_coverage_progress_safe(run_cache=run_cache)
    coverage_top_plan_id = _first_int_field(
        coverage_payload,
        ("missionPlanID", "missionPlanId", "mission_plan_id"),
    ) if isinstance(coverage_payload, dict) else None
    if (
        isinstance(coverage_payload, dict)
        and not (
            expected_plan_id is not None
            and coverage_top_plan_id is not None
            and int(coverage_top_plan_id) != int(expected_plan_id)
        )
    ):
        missions = coverage_payload.get("missions") if isinstance(coverage_payload.get("missions"), list) else []
    else:
        missions = []
    progress_by_aircraft: Dict[int, int] = {}
    source_by_aircraft: Dict[int, str] = {}
    row_count_by_aircraft: Dict[int, int] = {}
    for aircraft_id, rows in sorted(line_rows_by_aircraft.items()):
        percent = _progress_percent_from_rows(rows)
        if percent is None:
            continue
        progress_by_aircraft[int(aircraft_id)] = int(percent)
        source_by_aircraft[int(aircraft_id)] = str(
            line_source_by_aircraft.get(int(aircraft_id)) or "line_scan_progress"
        )
        row_count_by_aircraft[int(aircraft_id)] = len(rows)

    coverage_rows_by_aircraft: Dict[int, List[Dict[str, Any]]] = defaultdict(list)
    for entry in missions:
        if not isinstance(entry, dict):
            continue
        entry_plan_id = _first_int_field(
            entry,
            ("missionPlanID", "missionPlanId", "mission_plan_id"),
        )
        if (
            expected_plan_id is not None
            and entry_plan_id is not None
            and int(entry_plan_id) != int(expected_plan_id)
        ):
            continue
        if _to_int(entry.get("input_id")) != int(current_input_id):
            continue
        aircraft_id = _to_int(entry.get("aircraft_id"))
        if aircraft_id is None or int(aircraft_id) not in active_id_set:
            continue
        if int(aircraft_id) in progress_by_aircraft:
            continue
        coverage_rows_by_aircraft[int(aircraft_id)].append(entry)

    for aircraft_id, rows in sorted(coverage_rows_by_aircraft.items()):
        percent = _progress_percent_from_rows(rows)
        if percent is None:
            continue
        progress_by_aircraft[int(aircraft_id)] = int(percent)
        source_by_aircraft[int(aircraft_id)] = "coverage_progress"
        row_count_by_aircraft[int(aircraft_id)] = len(rows)

    values = [int(progress_by_aircraft[aid]) for aid in sorted(progress_by_aircraft)]
    avg_percent = (sum(values) / float(len(values))) if values else None
    return {
        "active_progress_by_aircraft": {
            int(aid): int(progress_by_aircraft[aid])
            for aid in sorted(progress_by_aircraft)
        },
        "active_progress_aircraft_ids": sorted(int(aid) for aid in progress_by_aircraft),
        "active_progress_sample_count": len(values),
        "active_avg_progress_percent": float(avg_percent) if avg_percent is not None else None,
        "active_progress_source_by_aircraft": {
            int(aid): str(source_by_aircraft.get(int(aid)) or "")
            for aid in sorted(progress_by_aircraft)
        },
        "active_progress_row_count_by_aircraft": {
            int(aid): int(row_count_by_aircraft.get(int(aid), 0))
            for aid in sorted(progress_by_aircraft)
        },
    }


def _build_active_line_phase_source(
    *,
    source_plan_id: int,
    aircraft_id: int,
    current_input_id: int,
    join_delay_s: int,
    agent_state_map: Dict[int, Dict[str, Any]],
    sweep_progress: Dict[int, Dict[str, Any]],
    emit: LogCallback,
    log_prefix: str,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Optional[_PhasedLineSource]:
    current_state = dict(agent_state_map.get(int(aircraft_id)) or {})
    artifacts = _resolve_plan_artifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        current_waypoint_id=_to_int(current_state.get("current_waypoint_id")),
        emit=lambda _msg: None,
        allow_first_mission_fallback=True,
    )
    if artifacts is None:
        emit(f"{log_prefix} aircraft {aircraft_id} phased line source unavailable: plan artifacts unresolved.")
        return None

    imp_data = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
        copy_result=False,
    )
    if not isinstance(imp_data, dict):
        emit(f"{log_prefix} aircraft {aircraft_id} phased line source unavailable: IMP load failed.")
        return None
    mission_list = imp_data.get("individualMissionList") or []
    template_mission = None
    for mission in mission_list:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        if _to_int(mission.get("individualMissionID")) == _to_int(getattr(artifacts, "individual_mission_id", None)):
            template_mission = deepcopy(mission)
            break
        if template_mission is None and not bool(mission.get("isDone")):
            template_mission = deepcopy(mission)
    if not isinstance(template_mission, dict):
        emit(f"{log_prefix} aircraft {aircraft_id} phased line source unavailable: template mission missing.")
        return None

    try:
        template_path = _load_path_payload(int(artifacts.path_id), run_cache=run_cache)
        if not isinstance(template_path, dict):
            raise ValueError("payload missing")
    except Exception as exc:
        emit(f"{log_prefix} aircraft {aircraft_id} phased line source unavailable: path load failed ({exc}).")
        return None
    waypoints = deepcopy(template_path.get("waypointList") or [])
    if not isinstance(waypoints, list) or not waypoints:
        emit(f"{log_prefix} aircraft {aircraft_id} phased line source unavailable: waypointList missing.")
        return None

    prefix_waypoints, suffix_waypoints = _split_resume_waypoints_for_join_delay(
        waypoints=waypoints,
        current_waypoint_id=_to_int(current_state.get("current_waypoint_id")),
        sweep_progress_entry=sweep_progress.get(int(artifacts.path_id)),
        join_delay_s=int(join_delay_s),
    )
    predicted_entry_coordinate = _last_path_coordinate(prefix_waypoints)
    if predicted_entry_coordinate is None:
        predicted_entry_coordinate = _normalize_coordinate(current_state.get("coordinate"))
    predicted_heading_deg = _path_heading_from_waypoints(prefix_waypoints)
    if predicted_heading_deg is None:
        predicted_heading_deg = _to_float(current_state.get("heading"))
    return _PhasedLineSource(
        aircraft_id=int(aircraft_id),
        template_mission=template_mission,
        template_path=template_path if isinstance(template_path, dict) else {},
        prefix_waypoints=prefix_waypoints,
        suffix_waypoints=suffix_waypoints,
        predicted_entry_coordinate=predicted_entry_coordinate,
        predicted_heading_deg=predicted_heading_deg,
    )


def _split_resume_waypoints_for_join_delay(
    *,
    waypoints: List[Dict[str, Any]],
    current_waypoint_id: Optional[int],
    sweep_progress_entry: Optional[Dict[str, Any]],
    join_delay_s: int,
) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    if not waypoints:
        return [], []
    start_idx = _find_current_waypoint_index(waypoints, current_waypoint_id)
    first_not_done_idx = _first_not_done_waypoint_index(waypoints)
    if start_idx is None:
        start_idx = first_not_done_idx
    elif (
        first_not_done_idx is not None
        and first_not_done_idx < start_idx
        and isinstance(sweep_progress_entry, dict)
    ):
        progress_points = max(0, int(sweep_progress_points(sweep_progress_entry)))
        current_done_points = count_sweep_points_in_waypoints(waypoints[:start_idx])
        if current_done_points > (progress_points + 3):
            start_idx = int(first_not_done_idx)
    if start_idx is None:
        start_idx = 0
    resume_waypoints = deepcopy(waypoints[start_idx:])
    if not resume_waypoints:
        return [], []
    # Do not move the flight waypoint onto the current sensor center here.
    # Post-attack rejoin uses these waypoints as executable flight anchors;
    # trimming their coordinates to camera sweep points can create U-turns
    # and immediate path-deviation replans.
    done_sweep_points = count_sweep_points_in_waypoints(waypoints[:start_idx])
    lookahead_s = max(0.0, float(join_delay_s)) + float(DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS)
    cut_total = physical_sweep_buffer_points(sweep_progress_entry, lookahead_s)
    cut_points = max(0, int(cut_total) - int(done_sweep_points))
    if cut_points <= 0:
        return [], resume_waypoints
    prefix_waypoints, suffix_waypoints = _split_waypoints_by_sweep_points(resume_waypoints, cut_points)
    if not suffix_waypoints:
        return resume_waypoints, []
    if prefix_waypoints:
        reassign_unique_waypoint_ids_inplace(prefix_waypoints)
    if suffix_waypoints:
        reassign_unique_waypoint_ids_inplace(suffix_waypoints)
    return prefix_waypoints, suffix_waypoints


def _split_waypoints_by_sweep_points(
    waypoints: List[Dict[str, Any]],
    cut_points: int,
) -> tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    if not waypoints:
        return [], []
    if cut_points <= 0:
        return [], deepcopy(waypoints)

    remaining = int(cut_points)
    prefix: List[Dict[str, Any]] = []
    suffix: List[Dict[str, Any]] = []
    suffix_started = False
    for waypoint in deepcopy(waypoints):
        fp = waypoint.get("filmingProperty") if isinstance(waypoint, dict) else None
        line_search = fp.get("lineSearch") if isinstance(fp, dict) else None
        coords = line_search.get("coordinateList") if isinstance(line_search, dict) else None
        if not isinstance(coords, list) or not coords:
            if suffix_started or remaining <= 0:
                suffix.append(waypoint)
            else:
                prefix.append(waypoint)
            continue

        coord_count = len(coords)
        if remaining >= coord_count:
            prefix.append(waypoint)
            remaining -= coord_count
            continue
        if remaining <= 0:
            suffix.append(waypoint)
            suffix_started = True
            continue

        prefix_wp = _clone_waypoint_with_line_search_coords(waypoint, coords[:remaining])
        suffix_wp = _clone_waypoint_with_line_search_coords(waypoint, coords[remaining:])
        if prefix_wp is not None:
            prefix.append(prefix_wp)
        if suffix_wp is not None:
            suffix.append(suffix_wp)
            suffix_started = True
        remaining = 0

    for waypoint in prefix:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    for waypoint in suffix:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    return prefix, suffix


def _clone_waypoint_with_line_search_coords(
    waypoint: Dict[str, Any],
    coords: List[Any],
) -> Optional[Dict[str, Any]]:
    usable = [dict(item) for item in coords if isinstance(item, dict)]
    if not usable:
        return None
    cloned = deepcopy(waypoint)
    fp = cloned.get("filmingProperty") if isinstance(cloned.get("filmingProperty"), dict) else {}
    line_search = fp.get("lineSearch") if isinstance(fp.get("lineSearch"), dict) else {}
    line_search_removed = False
    if len(usable) >= 2:
        line_search["coordinateList"] = usable
        fp["lineSearch"] = line_search
    else:
        try:
            fp.pop("lineSearch", None)
        except Exception:
            pass
        line_search_removed = True
    first_coord = usable[0]
    coord = cloned.get("coordinate") if isinstance(cloned.get("coordinate"), dict) else {}
    original_alt = _to_float(coord.get("altitude"))
    coord["latitude"] = float(first_coord.get("latitude"))
    coord["longitude"] = float(first_coord.get("longitude"))
    if original_alt is not None:
        # lineSearch coordinates carry terrain/sensor target altitude; the
        # waypoint coordinate must keep the aircraft flight altitude.
        coord["altitude"] = int(round(float(original_alt)))
    else:
        first_alt = _to_float(first_coord.get("altitude"))
        if first_alt is not None:
            coord["altitude"] = int(round(float(first_alt)))
    if line_search_removed:
        fp["operationMode"] = 1
        fp["coordinateOrientation"] = {"coordinate": deepcopy(first_coord)}
    cloned["coordinate"] = coord
    cloned["filmingProperty"] = fp
    normalize_filming_target_altitudes_in_waypoints([cloned])
    return cloned


def _line_width_from_template_mission(template_mission: Optional[Dict[str, Any]]) -> float:
    if not isinstance(template_mission, dict):
        return 1.0
    info = template_mission.get("individualMissionInfo") if isinstance(template_mission.get("individualMissionInfo"), dict) else {}
    line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
    for row in line_list:
        if not isinstance(row, dict):
            continue
        width = _to_float(row.get("width"))
        if width is not None and width > 0.0:
            return float(width)
    return 1.0


def _coords_close(left: Dict[str, Any], right: Dict[str, Any], *, tol: float = 1e-8) -> bool:
    return (
        abs(float(left.get("latitude", 0.0)) - float(right.get("latitude", 0.0))) <= float(tol)
        and abs(float(left.get("longitude", 0.0)) - float(right.get("longitude", 0.0))) <= float(tol)
    )


def _collect_line_search_coordinates(waypoints: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    coords: List[Dict[str, Any]] = []
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        row_coords = [
            coord
            for coord in (_normalize_coordinate(item) for item in (line_search.get("coordinateList") or []))
            if coord is not None
        ]
        if len(row_coords) < 2:
            continue
        if coords and _coords_close(coords[-1], row_coords[0]):
            coords.extend(dict(item) for item in row_coords[1:])
        else:
            coords.extend(dict(item) for item in row_coords)
    deduped: List[Dict[str, Any]] = []
    for coord in coords:
        if not deduped or not _coords_close(deduped[-1], coord):
            deduped.append(dict(coord))
    return deduped


def _build_future_line_row_from_suffix(
    phased_source: _PhasedLineSource,
) -> Optional[Dict[str, Any]]:
    coords = _collect_line_search_coordinates(phased_source.suffix_waypoints)
    if len(coords) < 2:
        return None
    return {
        "coordinateList": deepcopy(coords),
        "width": max(0, min(50000, int(round(float(_line_width_from_template_mission(phased_source.template_mission)))))),
    }


def _build_future_line_detail(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    line_rows: List[Dict[str, Any]] = []
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        coords = [
            coord
            for coord in (_normalize_coordinate(item) for item in (row.get("coordinateList") or []))
            if coord is not None
        ]
        if len(coords) < 2:
            continue
        width = _to_float(row.get("width"))
        if width is None or width <= 0.0:
            width = 1.0
        line_rows.append({
            "coordinateList": deepcopy(coords),
            "width": max(0, min(50000, int(round(float(width))))),
        })
    if not line_rows:
        return {"coordinateList": [], "lineList": [], "areaList": []}
    return {
        "coordinateList": deepcopy((line_rows[0] or {}).get("coordinateList") or []),
        "lineList": deepcopy(line_rows),
        "areaList": [],
    }


def _build_future_line_detail_from_remaining_mission(
    *,
    source_plan_id: int,
    current_input_id: int,
    current_input_mission: Dict[str, Any],
    evaluation: Dict[str, Any],
) -> Dict[str, Any]:
    detail = _load_remaining_line_detail_from_snapshot(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
    )
    if not detail:
        detail = current_input_mission.get("missionDetail") if isinstance(current_input_mission.get("missionDetail"), dict) else {}
    line_rows = _normalize_line_rows_from_detail(detail)
    if not line_rows:
        return {"coordinateList": [], "lineList": [], "areaList": []}
    restored = _restore_source_line_detail_metadata(_build_future_line_detail(line_rows), detail)
    future_centerline = [
        coord
        for coord in (_normalize_coordinate(item) for item in (restored.get("coordinateList") or []))
        if coord is not None
    ]
    existing_source = [
        coord
        for coord in (_normalize_coordinate(item) for item in (restored.get("sourceCoordinateList") or []))
        if coord is not None
    ]
    if len(existing_source) < 2 and len(future_centerline) >= 2:
        restored["sourceCoordinateList"] = deepcopy(future_centerline)
    return restored


def _restore_source_line_detail_metadata(
    detail: Dict[str, Any],
    source_detail: Dict[str, Any],
) -> Dict[str, Any]:
    restored = deepcopy(detail if isinstance(detail, dict) else {})
    source_detail = source_detail if isinstance(source_detail, dict) else {}

    source_width_m = _to_float(source_detail.get("sourceLineWidthM"))
    if source_width_m is None or source_width_m <= 0.0:
        for row in source_detail.get("lineList") or []:
            if not isinstance(row, dict):
                continue
            source_width_m = _to_float(row.get("width"))
            if source_width_m is not None and source_width_m > 0.0:
                break

    source_coords = [
        coord
        for coord in (_normalize_coordinate(item) for item in (source_detail.get("sourceCoordinateList") or []))
        if coord is not None
    ]
    if len(source_coords) < 2:
        for row in source_detail.get("lineList") or []:
            if not isinstance(row, dict):
                continue
            row_coords = [
                coord
                for coord in (_normalize_coordinate(item) for item in (row.get("coordinateList") or []))
                if coord is not None
            ]
            if len(row_coords) >= 2:
                source_coords = row_coords
                break
    if len(source_coords) < 2:
        source_coords = [
            coord
            for coord in (_normalize_coordinate(item) for item in (source_detail.get("coordinateList") or []))
            if coord is not None
        ]

    if source_width_m is not None and source_width_m > 0.0:
        restored["sourceLineWidthM"] = float(source_width_m)
    if len(source_coords) >= 2:
        restored["sourceCoordinateList"] = deepcopy(source_coords)
    return restored


def _load_remaining_line_detail_from_snapshot(
    *,
    source_plan_id: int,
    current_input_id: int,
) -> Dict[str, Any]:
    source_detail: Dict[str, Any] = {}
    try:
        snapshot_info = mission_area_replan_store.load_snapshot_entry(
            int(source_plan_id),
            int(current_input_id),
            allow_latest=True,
            audit_context="post_attack_remaining_line_detail",
        )
    except Exception:
        snapshot_info = None
    if isinstance(snapshot_info, dict):
        mission = snapshot_info.get("entry")
        if isinstance(mission, dict):
            detail = mission.get("remainingDetail")
            if isinstance(detail, dict):
                source_detail = deepcopy(detail)

    line_scan_detail = load_line_scan_remaining_detail(
        source_plan_id=int(source_plan_id),
        input_mission_id=int(current_input_id),
        source_detail=source_detail,
    )
    if has_line_remaining_geometry(line_scan_detail) or bool(
        line_scan_detail.get("lineRemainingCompleted")
    ):
        return line_scan_detail
    return source_detail


def _line_detail_length_m(detail: Any) -> float:
    if not isinstance(detail, dict):
        return 0.0
    raw_rows = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    coordinate_rows: List[List[Dict[str, Any]]] = []
    for row in raw_rows:
        if not isinstance(row, dict) or not isinstance(row.get("coordinateList"), list):
            continue
        coordinate_rows.append(row["coordinateList"])
    if not coordinate_rows and isinstance(detail.get("coordinateList"), list):
        coordinate_rows.append(detail["coordinateList"])

    total_m = 0.0
    for raw_coords in coordinate_rows:
        coords: List[Dict[str, Any]] = []
        for raw_coord in raw_coords:
            if not isinstance(raw_coord, dict):
                continue
            lat = _to_float(raw_coord.get("latitude"))
            lon = _to_float(raw_coord.get("longitude"))
            if lat is None or lon is None:
                continue
            coords.append({"latitude": float(lat), "longitude": float(lon)})
        total_m += _coord_path_length_m(coords)
    return float(total_m)


def _active_line_progress_percent(evaluation: Any) -> Optional[float]:
    if not isinstance(evaluation, dict):
        return None
    direct = _to_float(evaluation.get("active_avg_progress_percent"))
    if direct is not None:
        return max(0.0, min(100.0, float(direct)))

    progress_by_aircraft = evaluation.get("active_progress_by_aircraft")
    if not isinstance(progress_by_aircraft, dict):
        return None
    active_ids = {
        int(aid)
        for aid in (evaluation.get("active_aircraft_ids") or [])
        if _to_int(aid) is not None and int(aid) > 0
    }
    values: List[float] = []
    for raw_aid, raw_value in progress_by_aircraft.items():
        aid = _to_int(raw_aid)
        if active_ids and (aid is None or int(aid) not in active_ids):
            continue
        value = _to_float(raw_value)
        if value is not None:
            values.append(max(0.0, min(100.0, float(value))))
    if not values:
        return None
    return float(sum(values) / len(values))


def _load_exact_line_rejoin_snapshot_guard(
    *,
    source_plan_id: int,
    current_input_id: int,
) -> Dict[str, Any]:
    """Load only the current plan's exact spatial-coverage snapshot.

    A latest/ancestor fallback is intentionally forbidden here.  This guard
    protects against a transient LINE progress spike, so an older plan's
    geometry must never override the active two-UAV assignment.
    """

    try:
        snapshot_info = mission_area_replan_store.load_snapshot_entry(
            int(source_plan_id),
            int(current_input_id),
            allow_latest=False,
            audit_context="post_attack_line_rejoin_conflict_guard",
        )
    except Exception:
        snapshot_info = None
    if not isinstance(snapshot_info, dict) or not bool(snapshot_info.get("exact")):
        return {}
    entry = snapshot_info.get("entry")
    if not isinstance(entry, dict) or str(entry.get("missionType") or "").strip().lower() != "line":
        return {}
    detail = entry.get("remainingDetail")
    if not isinstance(detail, dict) or not has_line_remaining_geometry(detail):
        return {}

    guarded_detail = deepcopy(detail)
    source_coords = entry.get("sourceCoordinateList")
    if isinstance(source_coords, list) and len(source_coords) >= 2:
        guarded_detail["sourceCoordinateList"] = deepcopy(source_coords)
    source_width = _to_float(entry.get("sourceLineWidthM"))
    if source_width is not None and source_width > 0.0:
        guarded_detail["sourceLineWidthM"] = float(source_width)
    guarded_detail.setdefault(
        "lineRemainingFragmentCount",
        len(guarded_detail.get("lineList") or []),
    )
    return {
        "detail": guarded_detail,
        "coveragePercent": _to_float(entry.get("coveragePercent")),
        "snapshotMissionPlanID": _to_int(snapshot_info.get("snapshotMissionPlanID")),
        "snapshotTimestampMs": _to_int(
            (snapshot_info.get("snapshot") or {}).get("timestamp")
            if isinstance(snapshot_info.get("snapshot"), dict)
            else None
        ),
    }


def _line_rejoin_snapshot_conflicts_with_live_progress(
    *,
    live_line_detail: Any,
    snapshot_guard: Any,
    evaluation: Any,
) -> bool:
    if not isinstance(live_line_detail, dict) or not isinstance(snapshot_guard, dict):
        return False
    snapshot_detail = snapshot_guard.get("detail")
    if not isinstance(snapshot_detail, dict) or not has_line_remaining_geometry(snapshot_detail):
        return False

    active_progress = _active_line_progress_percent(evaluation)
    snapshot_progress = _to_float(snapshot_guard.get("coveragePercent"))
    progress_conflict = bool(
        active_progress is not None
        and snapshot_progress is not None
        and float(active_progress) - float(snapshot_progress) >= 25.0
    )

    live_length_m = _line_detail_length_m(live_line_detail)
    snapshot_length_m = _line_detail_length_m(snapshot_detail)
    live_timestamp_ms = _to_int(live_line_detail.get("lineScanLatestTimestampMs"))
    snapshot_timestamp_ms = _to_int(snapshot_guard.get("snapshotTimestampMs"))
    newer_snapshot_regression = bool(
        live_timestamp_ms is not None
        and snapshot_timestamp_ms is not None
        and int(snapshot_timestamp_ms) > int(live_timestamp_ms)
        and live_length_m > snapshot_length_m + max(50.0, snapshot_length_m * 0.05)
    )
    geometry_conflict = bool(
        snapshot_length_m >= 500.0
        and live_length_m <= max(150.0, snapshot_length_m * 0.35)
        and snapshot_length_m - live_length_m >= 500.0
    )
    return bool(progress_conflict or geometry_conflict or newer_snapshot_regression)


def _normalize_line_rows_from_detail(detail: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    width_fallback = 1.0
    for row in detail.get("lineList") or []:
        if not isinstance(row, dict):
            continue
        width = _to_float(row.get("width"))
        if width is not None and width > 0.0:
            width_fallback = float(width)
            break
    for row in detail.get("lineList") or []:
        if not isinstance(row, dict):
            continue
        coords = [
            coord
            for coord in (_normalize_coordinate(item) for item in (row.get("coordinateList") or []))
            if coord is not None
        ]
        if len(coords) < 2:
            continue
        width = _to_float(row.get("width"))
        rows.append({
            "coordinateList": deepcopy(coords),
            "width": max(0, min(50000, int(round(float(width))))) if width is not None and width > 0.0 else max(0, min(50000, int(round(float(width_fallback))))),
        })
    if rows:
        return rows

    coord_list = [
        coord
        for coord in (_normalize_coordinate(item) for item in (detail.get("coordinateList") or []))
        if coord is not None
    ]
    if len(coord_list) >= 2:
        return [{"coordinateList": deepcopy(coord_list), "width": max(0, min(50000, int(round(float(width_fallback)))))}]
    return []


def _coord_step_length_m(left: Dict[str, Any], right: Dict[str, Any]) -> float:
    mean_lat_rad = math.radians((float(left["latitude"]) + float(right["latitude"])) * 0.5)
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(mean_lat_rad)
    dx = (float(right["longitude"]) - float(left["longitude"])) * m_per_deg_lon
    dy = (float(right["latitude"]) - float(left["latitude"])) * m_per_deg_lat
    return math.hypot(dx, dy)


def _coord_path_length_m(coords: List[Dict[str, Any]]) -> float:
    total = 0.0
    usable = [coord for coord in coords if isinstance(coord, dict)]
    for idx in range(len(usable) - 1):
        total += _coord_step_length_m(usable[idx], usable[idx + 1])
    return float(total)


def _interpolate_coord(
    left: Dict[str, Any],
    right: Dict[str, Any],
    ratio: float,
) -> Dict[str, Any]:
    clamped = max(0.0, min(1.0, float(ratio)))
    coord = {
        "latitude": float(left["latitude"]) + ((float(right["latitude"]) - float(left["latitude"])) * clamped),
        "longitude": float(left["longitude"]) + ((float(right["longitude"]) - float(left["longitude"])) * clamped),
    }
    left_alt = _to_float(left.get("altitude"))
    right_alt = _to_float(right.get("altitude"))
    if left_alt is not None or right_alt is not None:
        alt0 = float(left_alt if left_alt is not None else right_alt or 0.0)
        alt1 = float(right_alt if right_alt is not None else left_alt or 0.0)
        coord["altitude"] = int(round(alt0 + ((alt1 - alt0) * clamped)))
    return coord


def _trim_coord_path_by_distance(
    coords: List[Dict[str, Any]],
    trim_distance_m: float,
) -> List[Dict[str, Any]]:
    usable = [dict(coord) for coord in coords if isinstance(coord, dict)]
    if len(usable) < 2:
        return usable
    remaining = max(0.0, float(trim_distance_m))
    if remaining <= 0.0:
        return usable
    idx = 0
    while idx < len(usable) - 1:
        seg_len = _coord_step_length_m(usable[idx], usable[idx + 1])
        if seg_len <= 1e-6:
            idx += 1
            continue
        if remaining + 1e-6 >= seg_len:
            remaining -= seg_len
            idx += 1
            continue
        start_coord = _interpolate_coord(usable[idx], usable[idx + 1], remaining / seg_len)
        return [start_coord] + [dict(coord) for coord in usable[idx + 1:]]
    return []


def _trim_line_rows_by_distance(
    rows: List[Dict[str, Any]],
    trim_distance_m: float,
) -> List[Dict[str, Any]]:
    remaining = max(0.0, float(trim_distance_m))
    trimmed_rows: List[Dict[str, Any]] = []
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        coords = [dict(coord) for coord in (row.get("coordinateList") or []) if isinstance(coord, dict)]
        if len(coords) < 2:
            continue
        row_length_m = _coord_path_length_m(coords)
        if remaining > 1e-6:
            if remaining + 1e-6 >= row_length_m:
                remaining -= row_length_m
                continue
            coords = _trim_coord_path_by_distance(coords, remaining)
            remaining = 0.0
        if len(coords) < 2:
            continue
        trimmed_rows.append({
            "coordinateList": deepcopy(coords),
            "width": max(0, min(50000, int(round(float(_to_float(row.get("width")) or 1.0))))),
        })
    return trimmed_rows


def _extract_line_rows_from_waypoints(
    waypoints: List[Dict[str, Any]],
    template_mission: Optional[Dict[str, Any]] = None,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    template_width = None
    if isinstance(template_mission, dict):
        info = template_mission.get("individualMissionInfo") if isinstance(template_mission.get("individualMissionInfo"), dict) else {}
        line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
        if line_list:
            template_width = _to_float((line_list[0] or {}).get("width"))
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        coords = [dict(item) for item in (line_search.get("coordinateList") or []) if isinstance(item, dict)]
        if len(coords) < 2:
            continue
        row = {
            "coordinateList": coords,
            "width": max(0, min(50000, int(round(float(template_width))))) if template_width is not None and template_width > 0.0 else 1,
        }
        rows.append(row)
    return rows


def _merge_future_line_rows(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    detail = {
        "coordinateList": [],
        "lineList": [dict(row) for row in rows if isinstance(row, dict)],
        "areaList": [],
    }
    try:
        from modules.mission_planning.replanning.triggers.prior.pipeline import _merge_line_remaining_detail

        return _merge_line_remaining_detail(detail)
    except Exception:
        line_list = detail["lineList"]
        if line_list:
            first = line_list[0]
            return {
                "coordinateList": deepcopy(first.get("coordinateList") or []),
                "lineList": deepcopy(line_list),
                "areaList": [],
            }
        return {"coordinateList": [], "lineList": [], "areaList": []}


def _build_active_prefix_replacement(
    *,
    phased_source: _PhasedLineSource,
    current_input_id: int,
    now_ms: int,
    individual_id_provider: Optional[Callable[[], int]] = None,
    path_id_provider: Optional[Callable[[int], int]] = None,
) -> Optional[tuple[Dict[str, Any], Dict[str, Any]]]:
    if not phased_source.prefix_waypoints:
        return None
    aircraft_id = int(phased_source.aircraft_id)
    if individual_id_provider is not None and path_id_provider is not None:
        individual_id = int(individual_id_provider())
        path_id = int(path_id_provider(aircraft_id))
    else:
        reservation = ReplanIdReservation.reserve(
            individual_count=1,
            path_count_by_aircraft={aircraft_id: 1},
        )
        individual_id = int(reservation.next_individual())
        path_id = int(reservation.next_path(aircraft_id))
    mission_entry = deepcopy(phased_source.template_mission)
    mission_entry["individualMissionID"] = int(individual_id)
    mission_entry["isDone"] = False
    mission_entry["pathID"] = int(path_id)
    mission_entry = _sanitize_post_attack_mission_entry(mission_entry, current_input_id=int(current_input_id))
    path_payload = _build_path_payload_from_waypoints(
        template_path=phased_source.template_path,
        aircraft_id=int(phased_source.aircraft_id),
        path_id=int(path_id),
        individual_mission_id=int(individual_id),
        waypoints=phased_source.prefix_waypoints,
        now_ms=int(now_ms),
    )
    return mission_entry, path_payload


def _build_returning_transit_replacement(
    *,
    source_plan_id: int,
    aircraft_id: int,
    current_input_id: int,
    first_replacement: Dict[str, Any],
    generated_fp_by_path: Dict[int, Dict[str, Any]],
    current_state: Dict[str, Any],
    join_delay_s: int,
    now_ms: int,
    run_cache: Optional[_PostAttackRunCache] = None,
    individual_id_provider: Optional[Callable[[], int]] = None,
    path_id_provider: Optional[Callable[[int], int]] = None,
) -> Optional[tuple[Dict[str, Any], Dict[str, Any]]]:
    template_imp = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
    )
    if not isinstance(template_imp, dict):
        return None
    template_mission = None
    for mission in template_imp.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        if not bool(mission.get("isDone")):
            template_mission = deepcopy(mission)
            break
        if template_mission is None:
            template_mission = deepcopy(mission)
    if not isinstance(template_mission, dict):
        return None
    try:
        template_path_id = _to_int(template_mission.get("pathID"))
        template_path = (
            _load_path_payload(int(template_path_id), run_cache=run_cache)
            if template_path_id is not None and template_path_id > 0
            else {}
        )
    except Exception:
        template_path = {}

    start_coord = _normalize_coordinate(current_state.get("coordinate"))
    replacement_path_id = _to_int(first_replacement.get("pathID"))
    replacement_path = generated_fp_by_path.get(int(replacement_path_id or 0)) if replacement_path_id is not None else None
    end_coord = _first_waypoint_flight_coordinate((replacement_path or {}).get("waypointList") or [])
    if start_coord is None or end_coord is None:
        return None

    transit_waypoints, speed_mps = _build_uav_release_resume_waypoints(
        start_coord=start_coord,
        end_coord=end_coord,
        release_eta_s=0,
        target_finish_eta_s=max(1, int(join_delay_s)),
        default_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
        min_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
        max_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
        force_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
    )
    if not transit_waypoints:
        return None

    aircraft_id = int(aircraft_id)
    if individual_id_provider is not None and path_id_provider is not None:
        individual_id = int(individual_id_provider())
        path_id = int(path_id_provider(aircraft_id))
    else:
        reservation = ReplanIdReservation.reserve(
            individual_count=1,
            path_count_by_aircraft={aircraft_id: 1},
        )
        individual_id = int(reservation.next_individual())
        path_id = int(reservation.next_path(aircraft_id))
    mission_entry = deepcopy(template_mission)
    mission_entry["individualMissionID"] = int(individual_id)
    mission_entry["isDone"] = False
    mission_entry["pathID"] = int(path_id)
    mission_entry = _sanitize_post_attack_mission_entry(mission_entry, current_input_id=int(current_input_id))
    _apply_release_resume_mission_info(
        mission_entry,
        start_coord=start_coord,
        end_coord=end_coord,
    )
    info = mission_entry.get("individualMissionInfo") if isinstance(mission_entry.get("individualMissionInfo"), dict) else {}
    info["SPEED"] = float(speed_mps)
    mission_entry["individualMissionInfo"] = info
    bearing_deg = _bearing_between(start_coord, end_coord)
    mission_entry["bearing_deg"] = float(bearing_deg)

    path_payload = _build_path_payload_from_waypoints(
        template_path=template_path if isinstance(template_path, dict) else {},
        aircraft_id=int(aircraft_id),
        path_id=int(path_id),
        individual_mission_id=int(individual_id),
        waypoints=transit_waypoints,
        now_ms=int(now_ms),
    )
    return mission_entry, path_payload


def _build_path_payload_from_waypoints(
    *,
    template_path: Dict[str, Any],
    aircraft_id: int,
    path_id: int,
    individual_mission_id: int,
    waypoints: List[Dict[str, Any]],
    now_ms: int,
) -> Dict[str, Any]:
    payload = deepcopy(template_path if isinstance(template_path, dict) else {})
    payload["pathID"] = int(path_id)
    payload["aircraftID"] = int(aircraft_id)
    payload["individualMissionID"] = int(individual_mission_id)
    payload["timestamp"] = int(now_ms)
    if "Source" in payload or "source" not in payload:
        payload["Source"] = str(payload.get("Source") or payload.get("source") or "MMR")
        payload.pop("source", None)
    else:
        payload["source"] = str(payload.get("source") or payload.get("Source") or "MMR")
    copied = [deepcopy(item) for item in waypoints if isinstance(item, dict)]
    for idx, waypoint in enumerate(copied):
        next_id = _to_int((copied[idx + 1] or {}).get("waypointID")) if idx + 1 < len(copied) else 0
        waypoint["nextWaypointID"] = int(next_id or 0)
        waypoint["isDone"] = False
    payload["waypointList"] = copied
    _repair_post_attack_single_line_search_eta(payload)
    sanitize_flight_path_payload_filming_altitudes(payload)
    return payload


def _first_waypoint_flight_coordinate(waypoints: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        coord = _normalize_coordinate(waypoint.get("coordinate"))
        if coord is not None:
            return dict(coord)
    return None


def _first_follow_up_reuses_boundary_point(
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]],
    boundary_coord: Dict[str, Any],
) -> bool:
    if not follow_up_paths or not isinstance(boundary_coord, dict):
        return False
    first_payload = follow_up_paths[0][1] if isinstance(follow_up_paths[0], tuple) and len(follow_up_paths[0]) >= 2 else None
    if not isinstance(first_payload, dict):
        return False
    waypoints = first_payload.get("waypointList")
    if not isinstance(waypoints, list) or len(waypoints) != 1:
        return False
    waypoint = waypoints[0]
    if not isinstance(waypoint, dict):
        return False
    if isinstance(waypoint.get("loiterProperty"), dict) or bool(waypoint.get("postAttackBoundaryHold")):
        return False
    coord = _normalize_coordinate(waypoint.get("coordinate"))
    if coord is None:
        return False
    try:
        return _haversine_m(
            float(coord["latitude"]),
            float(coord["longitude"]),
            float(boundary_coord["latitude"]),
            float(boundary_coord["longitude"]),
        ) <= 5.0
    except Exception:
        return False


def _sanitize_post_attack_mission_entry(
    mission_entry: Dict[str, Any],
    *,
    current_input_id: int,
) -> Dict[str, Any]:
    mission_entry["isDone"] = False
    related = mission_entry.get("relatedMission") if isinstance(mission_entry.get("relatedMission"), dict) else {}
    related = deepcopy(related)
    related["relatedMissionType"] = 1
    related["inputMissionID"] = int(current_input_id)
    related["priorMissionID"] = _to_int(related.get("priorMissionID")) or 0
    related.pop("attackReason", None)
    related.pop("targetID", None)
    mission_entry["relatedMission"] = related
    info = mission_entry.get("individualMissionInfo") if isinstance(mission_entry.get("individualMissionInfo"), dict) else {}
    info = deepcopy(info)
    info["targetID"] = None
    mission_entry["individualMissionInfo"] = info
    return mission_entry


def _reset_post_attack_replacement_path_state(payload: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(payload, dict):
        return payload

    touched = False
    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        waypoints = payload.get(key)
        if not isinstance(waypoints, list) or not waypoints:
            continue
        copied = [deepcopy(item) for item in waypoints if isinstance(item, dict)]
        if not copied:
            continue
        for waypoint in copied:
            waypoint["isDone"] = False
        relink_waypoints(copied)
        payload[key] = copied
        touched = True

    if touched:
        payload["isDone"] = False
    return payload


def _sanitize_post_attack_collaborative_replacements(
    *,
    aircraft_id: int,
    current_input_id: int,
    replacement_missions: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    sanitized: List[Dict[str, Any]] = []
    for mission_entry in replacement_missions or []:
        if not isinstance(mission_entry, dict):
            continue
        sanitized.append(
            _sanitize_post_attack_mission_entry(
                dict(mission_entry),
                current_input_id=int(current_input_id),
            )
        )
    return sanitized


def _load_imp_package_for_aircraft_cached(
    *,
    source_plan_id: int,
    aircraft_id: int,
    run_cache: Optional[_PostAttackRunCache] = None,
    copy_result: bool = True,
) -> Optional[Dict[str, Any]]:
    if run_cache is None:
        payload = _load_imp_package_for_aircraft(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
        )
        if not isinstance(payload, dict):
            return None
        return deepcopy(payload) if copy_result else payload
    key = (int(source_plan_id), int(aircraft_id))
    if key not in run_cache.imp_by_aircraft:
        payload = _load_imp_package_for_aircraft(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
        )
        # payload는 로더가 반환한 호출자 소유 사본 — 재복사 없이 보관
        run_cache.imp_by_aircraft[key] = payload if isinstance(payload, dict) else None
    cached = run_cache.imp_by_aircraft.get(key)
    if not isinstance(cached, dict):
        return None
    return deepcopy(cached) if copy_result else cached


def _load_path_payload(
    path_id: Optional[int],
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
    copy_result: bool = True,
) -> Optional[Dict[str, Any]]:
    pid = _to_int(path_id)
    if pid is None or pid <= 0:
        return None
    if run_cache is not None and int(pid) in run_cache.flight_paths:
        cached = run_cache.flight_paths.get(int(pid))
        if not isinstance(cached, dict):
            return None
        return deepcopy(cached) if copy_result else cached
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{int(pid)}.json")
        payload = read_json_cached(path, kind="FlightPath")
        if run_cache is not None:
            run_cache.flight_paths[int(pid)] = deepcopy(payload) if isinstance(payload, dict) else None
        return payload if isinstance(payload, dict) else None
    except Exception:
        if run_cache is not None:
            run_cache.flight_paths[int(pid)] = None
        return None


def _load_line_scan_aircraft_remaining_detail_cached(
    *,
    source_plan_id: int,
    input_mission_id: int,
    aircraft_ids: List[int],
    source_detail: Dict[str, Any],
    allow_latest_plan_fallback: bool,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[str, Any]:
    normalized_aircraft_ids = tuple(sorted(int(aid) for aid in aircraft_ids if _to_int(aid) is not None))
    key = (int(source_plan_id), int(input_mission_id), normalized_aircraft_ids)
    cache_enabled = not bool(source_detail)
    if cache_enabled and run_cache is not None and run_cache.line_remaining_details is not None:
        cached = run_cache.line_remaining_details.get(key)
        if isinstance(cached, dict):
            return deepcopy(cached)
    detail = load_line_scan_aircraft_remaining_detail(
        source_plan_id=int(source_plan_id),
        input_mission_id=int(input_mission_id),
        aircraft_ids=list(normalized_aircraft_ids),
        source_detail=source_detail,
        allow_latest_plan_fallback=bool(allow_latest_plan_fallback),
    )
    payload = detail if isinstance(detail, dict) else {}
    if cache_enabled and run_cache is not None and run_cache.line_remaining_details is not None:
        run_cache.line_remaining_details[key] = deepcopy(payload)
    return payload


def _active_current_input_path_all_done(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> bool:
    imp_data = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
    )
    if not isinstance(imp_data, dict):
        return False
    found_current_path = False
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        path_payload = _load_path_payload(
            _to_int(mission.get("pathID")),
            run_cache=run_cache,
            copy_result=False,
        )
        waypoints = (path_payload or {}).get("waypointList")
        if not isinstance(waypoints, list) or not waypoints:
            continue
        found_current_path = True
        if _first_not_done_waypoint_index(waypoints) is not None:
            return False
    return bool(found_current_path)


def _active_current_input_path_ids(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Set[int]:
    imp_data = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
        copy_result=False,
    )
    if not isinstance(imp_data, dict):
        return set()
    path_ids: Set[int] = set()
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        path_id = _to_int(mission.get("pathID"))
        if path_id is not None and path_id > 0:
            path_ids.add(int(path_id))
    return path_ids


def _active_current_input_on_mission_complete(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
    state: Dict[str, Any],
    run_cache: Optional[_PostAttackRunCache] = None,
) -> bool:
    if _to_int((state or {}).get("on_mission") or (state or {}).get("onMission")) != 2:
        return False
    imp_data = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
        copy_result=False,
    )
    if not isinstance(imp_data, dict):
        return False
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        path_payload = _load_path_payload(
            _to_int(mission.get("pathID")),
            run_cache=run_cache,
            copy_result=False,
        )
        waypoints = (path_payload or {}).get("waypointList")
        if _mission_has_post_attack_imaging_geometry(
            mission,
            waypoints if isinstance(waypoints, list) else [],
        ):
            return True
    return False


def _mission_has_post_attack_imaging_geometry(
    mission: Optional[Dict[str, Any]],
    waypoint_list: Optional[List[Dict[str, Any]]],
) -> bool:
    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo")
    info = info if isinstance(info, dict) else {}
    mission_type = _to_int(info.get("individualMissionType"))
    if mission_type in (3, 6):
        return True
    if isinstance(info.get("lineList"), list) and info.get("lineList"):
        return True
    if isinstance(info.get("areaList"), list) and info.get("areaList"):
        return True
    return count_sweep_points_in_waypoints(list(waypoint_list or [])) > 0


def _short_return_merge_distance_m() -> float:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float

        configured = _to_float(get_runtime_float("uav_wp_interval_m", _POST_ATTACK_SHORT_RETURN_DEFAULT_M))
    except Exception:
        configured = None
    return max(float(_POST_ATTACK_SHORT_RETURN_DEFAULT_M), float(configured or 0.0))


def _collapse_short_return_waypoints(
    waypoints: List[Dict[str, Any]],
    *,
    aircraft_id: int,
    emit: LogCallback,
    log_prefix: str,
) -> Tuple[List[Dict[str, Any]], bool]:
    if not isinstance(waypoints, list) or len(waypoints) <= 1:
        return waypoints, False
    first_coord = _normalize_coordinate((waypoints[0] or {}).get("coordinate"))
    final_coord = _normalize_coordinate((waypoints[-1] or {}).get("coordinate"))
    if first_coord is None or final_coord is None:
        return waypoints, False
    distance_m = _haversine_m(
        float(first_coord["latitude"]),
        float(first_coord["longitude"]),
        float(final_coord["latitude"]),
        float(final_coord["longitude"]),
    )
    threshold_m = _short_return_merge_distance_m()
    if float(distance_m) > float(threshold_m):
        return waypoints, False
    final_wp = deepcopy(waypoints[-1])
    final_wp["coordinate"] = deepcopy(final_coord)
    final_wp["isDone"] = False
    final_wp["nextWaypointID"] = 0
    emit(
        f"{log_prefix} short return path collapsed "
        f"(aircraft={aircraft_id}, distance={float(distance_m):.1f}m, threshold={float(threshold_m):.1f}m)."
    )
    return [final_wp], True


def _build_post_attack_active_done_followup_update(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
    hold_seconds: int,
    now_ms: int,
    emit: LogCallback,
    log_prefix: str,
    run_cache: Optional[_PostAttackRunCache] = None,
    preserve_current_mission: bool = False,
    include_completion_boundary_hold: bool = True,
    block_follow_up_until_reassignment: bool = False,
) -> Optional[Dict[str, Any]]:
    imp_data = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
    )
    if not isinstance(imp_data, dict):
        emit(f"{log_prefix} IMP load failed for completed active aircraft {aircraft_id}.")
        return None
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        emit(f"{log_prefix} IMP mission list missing for completed active aircraft {aircraft_id}.")
        return None

    target_indices = [
        idx
        for idx, mission in enumerate(mission_list)
        if isinstance(mission, dict)
        and _extract_related_input_mission_id(mission) == int(current_input_id)
    ]
    if not target_indices:
        emit(
            f"{log_prefix} completed active mission missing "
            f"(aircraft={aircraft_id}, inputMissionID={current_input_id})."
        )
        return None

    target_idx = max(int(idx) for idx in target_indices)
    target_mission = deepcopy(mission_list[target_idx])
    clone_start_idx = int(target_idx if preserve_current_mission else target_idx + 1)
    point_only_active_done = bool(block_follow_up_until_reassignment) or not _has_mission_entries(
        mission_list[clone_start_idx:]
    )
    source_path_id = _to_int(target_mission.get("pathID"))
    template_path = _load_path_payload(source_path_id, run_cache=run_cache)
    if not isinstance(template_path, dict):
        emit(
            f"{log_prefix} completed active path load failed "
            f"(aircraft={aircraft_id}, pathID={source_path_id})."
        )
        return None

    waypoints = [
        deepcopy(item)
        for item in (template_path.get("waypointList") or [])
        if isinstance(item, dict)
    ]
    marker_wp = deepcopy(waypoints[-1]) if waypoints else {}
    final_flight_coord = _normalize_coordinate(
        marker_wp.get("coordinate") if isinstance(marker_wp, dict) else None
    )
    if final_flight_coord is None:
        final_flight_coord = _normalize_coordinate(
            _extract_final_uav_coordinate(template_path) or _last_path_coordinate(waypoints)
        )
    if final_flight_coord is None:
        emit(
            f"{log_prefix} completed active final coordinate unavailable "
            f"(aircraft={aircraft_id}, pathID={source_path_id})."
        )
        return None
    area_internal_point_used = False
    if point_only_active_done:
        area_coord = _input_area_internal_hold_coordinate(
            source_plan_id=int(source_plan_id),
            current_input_id=int(current_input_id),
            altitude_sources=[final_flight_coord, marker_wp.get("coordinate")],
            run_cache=run_cache,
        )
        if area_coord is not None:
            final_flight_coord = deepcopy(area_coord)
            area_internal_point_used = True
            emit(
                f"{log_prefix} active point-only hold moved inside current area "
                f"(aircraft={aircraft_id}, inputMissionID={current_input_id})."
            )
    final_orientation_coord = (
        deepcopy(final_flight_coord)
        if area_internal_point_used
        else (_final_filming_orientation_coordinate(waypoints) or deepcopy(final_flight_coord))
    )
    hold_seconds = int(_POST_ATTACK_COMPLETE_HOLD_SECONDS)

    reservation_summaries: List[Dict[str, Any]] = []

    # Keep future collaborative inputs in the normal mission list for
    # planning/visualization. If the first preserved follow-up already is the
    # same single boundary point, do not add an artificial loiter hold in front
    # of it.
    follow_up_source_missions = _post_attack_follow_up_source_missions(
        mission_list,
        clone_start_idx,
    )
    clone_count = _post_attack_follow_up_clone_count(follow_up_source_missions)
    follow_up_waypoint_count = 0
    follow_up_waypoint_count_complete = True
    for follow_up_mission in follow_up_source_missions:
        if not isinstance(follow_up_mission, dict):
            continue
        if _skip_replan_follow_up_reason(follow_up_mission, excluded_input_ids=set()) is not None:
            continue
        follow_up_path_id = _to_int(follow_up_mission.get("pathID"))
        if follow_up_path_id is None or follow_up_path_id <= 0:
            follow_up_waypoint_count_complete = False
            break
        try:
            follow_up_path = read_json_cached(
                db_paths.get_db_subpath("FlightPath", f"{int(follow_up_path_id)}.json"),
                kind="FlightPath",
            )
        except Exception:
            follow_up_waypoint_count_complete = False
            break
        for waypoint_key in ("waypointList", "uavWaypointList", "lahWaypointList"):
            follow_up_waypoints = follow_up_path.get(waypoint_key)
            if isinstance(follow_up_waypoints, list):
                follow_up_waypoint_count += sum(
                    1 for item in follow_up_waypoints if isinstance(item, dict)
                )
    if not follow_up_waypoint_count_complete:
        # Keep the clone helper's existing per-path allocator fallback when a
        # prepass cannot prove the complete waypoint budget.
        follow_up_waypoint_count = 0
    clone_id_reservation = (
        ReplanIdReservation.reserve(
            individual_count=int(clone_count),
            path_count_by_aircraft={int(aircraft_id): int(clone_count)},
            waypoint_count=int(follow_up_waypoint_count),
        )
        if int(clone_count) > 0
        else None
    )
    cloned_artifacts = _clone_follow_up_replan_artifacts(
        missions=follow_up_source_missions,
        aircraft_id=int(aircraft_id),
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
        individual_id_provider=clone_id_reservation.next_individual if clone_id_reservation is not None else None,
        path_id_provider=clone_id_reservation.next_path if clone_id_reservation is not None else None,
        waypoint_id_provider=(
            clone_id_reservation.next_waypoint
            if clone_id_reservation is not None and follow_up_waypoint_count_complete
            else None
        ),
        reservation_summaries=reservation_summaries,
        reservation_scope="postAttackActiveDoneFollowUpClone",
    )
    if cloned_artifacts is None:
        return None
    follow_up_missions, follow_up_paths = cloned_artifacts

    if block_follow_up_until_reassignment:
        include_completion_boundary_hold = True
        blocked_follow_up_count = _mark_post_attack_followups_execution_blocked(
            follow_up_missions,
            current_input_id=int(current_input_id),
        )
        emit(
            f"{log_prefix} future input missions preserved for visualization and "
            "execution-blocked until the next collaborative mission handoff "
            f"(aircraft={aircraft_id}, inputMissionID={current_input_id}, "
            f"blockedFollowUps={int(blocked_follow_up_count)})."
        )

    if not include_completion_boundary_hold and not preserve_current_mission:
        imp_reservation = ReplanIdReservation.reserve(imp_count=1)
        new_imp_id = int(imp_reservation.next_imp())
        new_imp_data = _copy_post_attack_imp_shell(imp_data)
        new_imp_data["individualMissionPackageID"] = int(new_imp_id)
        new_imp_data["timestamp"] = int(now_ms)
        new_imp_data["individualMissionList"] = [
            deepcopy(mission) for mission in follow_up_missions
        ]
        new_imp_data.pop("deferredIndividualMissionList", None)

        generated_flight_paths = [
            payload for _dest, payload in follow_up_paths if isinstance(payload, dict)
        ]
        _validate_generated_post_attack_artifact_payloads(
            individual_mission_plans=[new_imp_data],
            flight_paths=generated_flight_paths,
            scope=f"postAttackActiveDoneFollowupNoCurrent:{new_imp_id}",
            allow_existing_db_artifacts=True,
            log=emit,
        )

        imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(new_imp_id)}.json")
        imp_dest.parent.mkdir(parents=True, exist_ok=True)

        generated_path_ids: Set[int] = set()
        write_entries: List[Tuple[Path, Dict[str, Any]]] = [(imp_dest, new_imp_data)]
        for dest, payload in follow_up_paths:
            dest.parent.mkdir(parents=True, exist_ok=True)
            _apply_runtime_flyover_to_flight_path_payload(payload)
            sanitize_flight_path_payload_filming_altitudes(payload)
            write_entries.append((dest, payload))
            path_id = _to_int((payload or {}).get("pathID"))
            if path_id is not None and path_id > 0:
                generated_path_ids.add(int(path_id))
        _write_or_defer_post_attack_json_batch(
            write_entries,
            run_cache=run_cache,
        )

        emit(
            f"{log_prefix} completed current input omitted from regenerated package "
            f"(aircraft={aircraft_id}, inputMissionID={current_input_id}, "
            f"imp={imp_dest.name}, followUps={len(follow_up_missions)})."
        )
        direct_reservation = _post_attack_reservation_event(
            scope="postAttackActiveDoneFollowupNoCurrent",
            aircraft_id=int(aircraft_id),
            imp_ids=[int(new_imp_id)],
        )
        reservation_summaries.insert(0, direct_reservation)
        return {
            "aircraft_id": int(aircraft_id),
            "individualMissionPackageID": int(new_imp_id),
            "doneIndividualMissionID": None,
            "donePathID": None,
            "generatedPathIDs": sorted(int(pid) for pid in generated_path_ids),
            "followUpMissionCount": len(follow_up_missions),
            "preservedCurrentMission": False,
            "holdSkipped": True,
            "holdSkipReason": "completed_current_input_omitted",
            "completedCurrentMissionDropped": True,
            "finalCoordinate": deepcopy(final_flight_coord),
            "orientationCoordinate": deepcopy(final_orientation_coord),
            "reservedIds": direct_reservation.get("reservedIds", {}),
            "reservationSummaries": reservation_summaries,
        }

    if (
        preserve_current_mission
        and not include_completion_boundary_hold
        and follow_up_missions
        and _first_follow_up_reuses_boundary_point(follow_up_paths, final_flight_coord)
    ):
        imp_reservation = ReplanIdReservation.reserve(imp_count=1)
        new_imp_id = int(imp_reservation.next_imp())
        new_imp_data = _copy_post_attack_imp_shell(imp_data)
        new_imp_data["individualMissionPackageID"] = int(new_imp_id)
        new_imp_data["timestamp"] = int(now_ms)
        new_imp_data["individualMissionList"] = [
            deepcopy(mission) for mission in follow_up_missions
        ]
        new_imp_data.pop("deferredIndividualMissionList", None)

        generated_flight_paths = [
            payload for _dest, payload in follow_up_paths if isinstance(payload, dict)
        ]
        _validate_generated_post_attack_artifact_payloads(
            individual_mission_plans=[new_imp_data],
            flight_paths=generated_flight_paths,
            scope=f"postAttackActiveDoneFollowupNoHold:{new_imp_id}",
            allow_existing_db_artifacts=True,
            log=emit,
        )

        imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(new_imp_id)}.json")
        imp_dest.parent.mkdir(parents=True, exist_ok=True)

        generated_path_ids: Set[int] = set()
        write_entries = [(imp_dest, new_imp_data)]
        for dest, payload in follow_up_paths:
            dest.parent.mkdir(parents=True, exist_ok=True)
            _apply_runtime_flyover_to_flight_path_payload(payload)
            sanitize_flight_path_payload_filming_altitudes(payload)
            write_entries.append((dest, payload))
            path_id = _to_int((payload or {}).get("pathID"))
            if path_id is not None and path_id > 0:
                generated_path_ids.add(int(path_id))
        _write_or_defer_post_attack_json_batch(
            write_entries,
            run_cache=run_cache,
        )

        emit(
            f"{log_prefix} redundant boundary loiter skipped; preserved first follow-up point "
            f"(aircraft={aircraft_id}, inputMissionID={current_input_id}, "
            f"imp={imp_dest.name}, firstPath={min(generated_path_ids) if generated_path_ids else None}, "
            f"followUps={len(follow_up_missions)})."
        )
        direct_reservation = _post_attack_reservation_event(
            scope="postAttackActiveDoneFollowupNoHold",
            aircraft_id=int(aircraft_id),
            imp_ids=[int(new_imp_id)],
        )
        reservation_summaries.insert(0, direct_reservation)
        return {
            "aircraft_id": int(aircraft_id),
            "individualMissionPackageID": int(new_imp_id),
            "doneIndividualMissionID": None,
            "donePathID": None,
            "generatedPathIDs": sorted(int(pid) for pid in generated_path_ids),
            "followUpMissionCount": len(follow_up_missions),
            "preservedCurrentMission": bool(preserve_current_mission),
            "holdSkipped": True,
            "holdSkipReason": "first_follow_up_reuses_boundary_point",
            "finalCoordinate": deepcopy(final_flight_coord),
            "orientationCoordinate": deepcopy(final_orientation_coord),
            "reservedIds": direct_reservation.get("reservedIds", {}),
            "reservationSummaries": reservation_summaries,
        }

    direct_reservation = ReplanIdReservation.reserve(
        imp_count=1,
        individual_count=1,
        path_count_by_aircraft={int(aircraft_id): 1},
        waypoint_count=1,
    )
    done_individual_id = int(direct_reservation.next_individual())
    done_path_id = int(direct_reservation.next_path(int(aircraft_id)))
    new_imp_id = int(direct_reservation.next_imp())

    if not marker_wp:
        marker_wp = _build_uav_transit_waypoint(
            coordinate=final_flight_coord,
            speed_mps=float(_POST_ATTACK_COMPLETE_HOLD_SPEED_MPS),
            eta_s=int(hold_seconds),
            orientation_coordinate=final_orientation_coord,
            waypoint_pass_type=2,
        )
    marker_waypoint_id = int(direct_reservation.next_waypoint())
    marker_wp["waypointID"] = int(marker_waypoint_id)
    marker_wp["coordinate"] = deepcopy(final_flight_coord)
    marker_wp["speed"] = float(_POST_ATTACK_COMPLETE_HOLD_SPEED_MPS)
    marker_wp["eta"] = int(hold_seconds)
    marker_wp["waypointPassType"] = 2
    marker_wp["loiterProperty"] = {
        "radius": int(_POST_ATTACK_COMPLETE_HOLD_RADIUS_M),
        "direction": 1,
        "time": int(hold_seconds),
        "speed": int(round(_POST_ATTACK_COMPLETE_HOLD_SPEED_MPS)),
    }
    marker_wp["isDone"] = False
    marker_wp["nextWaypointID"] = 0
    filming = marker_wp.get("filmingProperty")
    filming = deepcopy(filming) if isinstance(filming, dict) else {}
    filming["operationMode"] = 1
    filming["sensorType"] = _to_int(filming.get("sensorType")) or 1
    if _to_float(filming.get("fieldOfView")) is None:
        filming["fieldOfView"] = 5.0
    filming.pop("lineSearch", None)
    filming.pop("areaSearch", None)
    filming.pop("autoTracking", None)
    coord_orientation = filming.get("coordinateOrientation")
    coord_orientation = deepcopy(coord_orientation) if isinstance(coord_orientation, dict) else {}
    coord_orientation["coordinate"] = deepcopy(final_orientation_coord)
    filming["coordinateOrientation"] = coord_orientation
    marker_wp["filmingProperty"] = filming

    done_mission = deepcopy(target_mission)
    done_mission.pop("executionBlockedUntilNextCollab", None)
    done_mission["individualMissionID"] = int(done_individual_id)
    done_mission["pathID"] = int(done_path_id)
    done_mission["isDone"] = False
    done_mission = _sanitize_post_attack_mission_entry(
        done_mission,
        current_input_id=int(current_input_id),
    )
    info = done_mission.get("individualMissionInfo") if isinstance(done_mission.get("individualMissionInfo"), dict) else {}
    info = deepcopy(info)
    info["individualMissionType"] = 7
    info["patternType"] = 10
    info["autoZoomIn"] = False
    info["targetID"] = None
    info["coordinateList"] = [deepcopy(final_flight_coord)]
    info["lineList"] = []
    info["areaList"] = []
    info["SPEED"] = float(_POST_ATTACK_COMPLETE_HOLD_SPEED_MPS)
    done_mission["individualMissionInfo"] = info

    path_payload = _build_path_payload_from_waypoints(
        template_path=template_path,
        aircraft_id=int(aircraft_id),
        path_id=int(done_path_id),
        individual_mission_id=int(done_individual_id),
        waypoints=[marker_wp],
        now_ms=int(now_ms),
    )
    for waypoint in path_payload.get("waypointList") or []:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
            waypoint["nextWaypointID"] = 0
            waypoint["postAttackBoundaryHold"] = True
    done_mission["postAttackBoundaryHold"] = True
    path_payload["postAttackBoundaryHold"] = True

    new_imp_data = _copy_post_attack_imp_shell(imp_data)
    new_imp_data["individualMissionPackageID"] = int(new_imp_id)
    new_imp_data["timestamp"] = int(now_ms)
    new_imp_data["individualMissionList"] = [done_mission] + [
        deepcopy(mission) for mission in follow_up_missions
    ]
    new_imp_data.pop("deferredIndividualMissionList", None)

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(new_imp_id)}.json")
    path_dest = db_paths.get_db_subpath("FlightPath", f"{int(done_path_id)}.json")
    imp_dest.parent.mkdir(parents=True, exist_ok=True)
    path_dest.parent.mkdir(parents=True, exist_ok=True)
    _apply_runtime_flyover_to_flight_path_payload(path_payload)
    for waypoint in path_payload.get("waypointList") or []:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
            waypoint["nextWaypointID"] = 0
            waypoint["postAttackBoundaryHold"] = True
    sanitize_flight_path_payload_filming_altitudes(path_payload)
    generated_flight_paths = [path_payload] + [
        payload for _dest, payload in follow_up_paths if isinstance(payload, dict)
    ]
    _validate_generated_post_attack_artifact_payloads(
        individual_mission_plans=[new_imp_data],
        flight_paths=generated_flight_paths,
        scope=f"postAttackActiveDoneFollowup:{new_imp_id}",
        allow_existing_db_artifacts=True,
        log=emit,
    )
    generated_path_ids: Set[int] = {int(done_path_id)}
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [
        (imp_dest, new_imp_data),
        (path_dest, path_payload),
    ]
    for dest, payload in follow_up_paths:
        dest.parent.mkdir(parents=True, exist_ok=True)
        _apply_runtime_flyover_to_flight_path_payload(payload)
        sanitize_flight_path_payload_filming_altitudes(payload)
        write_entries.append((dest, payload))
        path_id = _to_int((payload or {}).get("pathID"))
        if path_id is not None and path_id > 0:
            generated_path_ids.add(int(path_id))
    _write_or_defer_post_attack_json_batch(
        write_entries,
        run_cache=run_cache,
    )

    emit(
        f"{log_prefix} completed current input preserved as active boundary marker "
        f"(aircraft={aircraft_id}, inputMissionID={current_input_id}, "
        f"imp={imp_dest.name}, donePath={path_dest.name}, "
        f"hold={int(hold_seconds)}s, followUps={len(follow_up_missions)}, "
        f"preserveCurrent={int(bool(preserve_current_mission))})."
    )
    direct_reservation = _post_attack_reservation_event(
        scope="postAttackActiveDoneFollowup",
        aircraft_id=int(aircraft_id),
        imp_ids=[int(new_imp_id)],
        individual_ids=[int(done_individual_id)],
        waypoint_ids=[int(marker_waypoint_id)],
        path_ids_by_aircraft={int(aircraft_id): [int(done_path_id)]},
    )
    reservation_summaries.insert(0, direct_reservation)
    return {
        "aircraft_id": int(aircraft_id),
        "individualMissionPackageID": int(new_imp_id),
        "doneIndividualMissionID": int(done_individual_id),
        "donePathID": int(done_path_id),
        "generatedPathIDs": sorted(int(pid) for pid in generated_path_ids),
        "followUpMissionCount": len(follow_up_missions),
        "preservedCurrentMission": bool(preserve_current_mission),
        "areaPassReassignmentHold": bool(block_follow_up_until_reassignment),
        "followUpsBlockedUntilNextCollab": bool(block_follow_up_until_reassignment),
        "holdSeconds": int(hold_seconds),
        "finalCoordinate": deepcopy(final_flight_coord),
        "orientationCoordinate": deepcopy(final_orientation_coord),
        "reservedIds": direct_reservation.get("reservedIds", {}),
        "reservationSummaries": reservation_summaries,
    }


def _formation_follow_up_missions_after_current(
    *,
    source_plan_id: int,
    missions: List[Dict[str, Any]],
    emit: LogCallback,
    log_prefix: str,
) -> List[Dict[str, Any]]:
    input_data = _load_input_plan_for_source_plan(int(source_plan_id))
    if not isinstance(input_data, dict):
        return []

    input_type_by_id: Dict[int, int] = {}
    for item in input_data.get("inputMissionList") or []:
        if not isinstance(item, dict):
            continue
        input_id = _to_int(item.get("inputMissionID"))
        mission_type = _to_int(item.get("inputMissionType"))
        if input_id is None or mission_type is None:
            continue
        input_type_by_id[int(input_id)] = int(mission_type)

    preserved: List[Dict[str, Any]] = []
    for mission in missions or []:
        if not isinstance(mission, dict):
            continue
        input_id = _extract_related_input_mission_id(mission)
        if input_id is None:
            continue
        if input_type_by_id.get(int(input_id)) != int(_FORMATION_FLIGHT_INPUT_MISSION_TYPE):
            continue
        preserved.append(mission)

    if preserved:
        ids = [
            int(_extract_related_input_mission_id(mission) or 0)
            for mission in preserved
            if _extract_related_input_mission_id(mission) is not None
        ]
        emit(
            f"{log_prefix} preserving formation-flight follow-up missions "
            f"(inputMissionIDs={sorted(set(ids))})."
        )
    return preserved


def _final_filming_orientation_coordinate(
    waypoints: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    for waypoint in reversed(waypoints or []):
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        coords = line_search.get("coordinateList")
        if isinstance(coords, list) and coords:
            coord = _normalize_coordinate(coords[-1])
            if coord is not None:
                return coord
        coord_orientation = filming.get("coordinateOrientation")
        if isinstance(coord_orientation, dict):
            coord = _normalize_coordinate(coord_orientation.get("coordinate"))
            if coord is not None:
                return coord
    return None


def _build_post_attack_tracking_return_only_update(
    *,
    attack_plan_id: int,
    current_input_id: int,
    assignment: Dict[str, Any],
    current_state: Dict[str, Any],
    hold_seconds: int | None = None,
    now_ms: int,
    emit: LogCallback,
    log_prefix: str,
    run_cache: Optional[_PostAttackRunCache] = None,
    block_follow_up_until_reassignment: bool = False,
) -> Optional[Dict[str, Any]]:
    aircraft_id = _to_int(assignment.get("aircraft_id"))
    if aircraft_id is None or aircraft_id <= 0:
        return None

    self_reliance_phase = _source_type2_self_reliance_phase(
        source_plan_id=int(attack_plan_id),
        input_mission_id=int(current_input_id),
    )
    type2_branch_line = self_reliance_phase in {
        TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
        TYPE2_SELF_RELIANCE_RETURN_LINE,
    }
    if type2_branch_line:
        emit(
            f"{log_prefix} Type2 branch LINE suffix will use exact individual progress "
            f"without blocking its following AREA/LINE missions "
            f"(aircraft={aircraft_id}, inputMissionID={current_input_id}, "
            f"phase={self_reliance_phase})."
        )

    imp_data = _load_imp_package_for_aircraft_cached(
        source_plan_id=int(attack_plan_id),
        aircraft_id=int(aircraft_id),
        run_cache=run_cache,
    )
    if not isinstance(imp_data, dict):
        emit(f"{log_prefix} IMP load failed for aircraft {aircraft_id}.")
        return None
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list) or not mission_list:
        emit(f"{log_prefix} IMP mission list missing for aircraft {aircraft_id}.")
        return None

    tracking_individual_id = _to_int(assignment.get("tracking_individual_mission_id"))
    resume_individual_id = _to_int(assignment.get("resume_individual_mission_id"))
    tracking_index = next(
        (
            idx
            for idx, mission in enumerate(mission_list)
            if isinstance(mission, dict)
            and tracking_individual_id is not None
            and _to_int(mission.get("individualMissionID")) == int(tracking_individual_id)
        ),
        None,
    )
    resume_index = next(
        (
            idx
            for idx, mission in enumerate(mission_list)
            if isinstance(mission, dict)
            and resume_individual_id is not None
            and _to_int(mission.get("individualMissionID")) == int(resume_individual_id)
        ),
        None,
    )

    template_mission = None
    if resume_index is not None and 0 <= int(resume_index) < len(mission_list):
        template_mission = mission_list[int(resume_index)]
    elif tracking_index is not None and 0 <= int(tracking_index) < len(mission_list):
        template_mission = mission_list[int(tracking_index)]
    if not isinstance(template_mission, dict):
        emit(f"{log_prefix} resume/tracking mission template unavailable for aircraft {aircraft_id}.")
        return None

    resume_path_payload = _load_path_payload(assignment.get("resume_path_id"), run_cache=run_cache)
    original_path_payload = _load_path_payload(assignment.get("original_path_id"), run_cache=run_cache)
    template_path_payload = resume_path_payload or original_path_payload
    if not isinstance(template_path_payload, dict):
        emit(f"{log_prefix} template path unavailable for aircraft {aircraft_id}.")
        return None

    current_coord = (
        _normalize_coordinate((current_state or {}).get("coordinate"))
        or _normalize_coordinate(assignment.get("handoff_coordinate"))
        or _normalize_coordinate(assignment.get("last_nonzero_coordinate"))
        or _normalize_coordinate(assignment.get("original_coordinate"))
    )
    if current_coord is None:
        emit(f"{log_prefix} current coordinate unavailable for aircraft {aircraft_id}.")
        return None

    resume_mission = None
    resume_mission_done = False
    if resume_index is not None and 0 <= int(resume_index) < len(mission_list):
        resume_mission = mission_list[int(resume_index)]
        resume_mission_done = isinstance(resume_mission, dict) and bool(resume_mission.get("isDone"))
    resume_waypoints = [
        deepcopy(item)
        for item in (
            (resume_path_payload or {}).get("waypointList")
            if isinstance((resume_path_payload or {}).get("waypointList"), list)
            else []
        )
        if isinstance(item, dict)
    ]
    resume_has_imaging_geometry = _mission_has_post_attack_imaging_geometry(
        resume_mission,
        resume_waypoints,
    )
    resume_path_id = _to_int(assignment.get("resume_path_id"))
    sweep_progress = (
        _load_sweep_progress_safe(run_cache=run_cache)
        if type2_branch_line
        else {}
    )
    resume_progress_entry = (
        sweep_progress.get(int(resume_path_id))
        if resume_path_id is not None
        else None
    )
    progress_is_authoritative = _sweep_progress_entry_is_authoritative(
        resume_progress_entry
    )
    progress_has_remaining = _sweep_progress_entry_has_remaining_imaging(
        resume_progress_entry
    )
    if progress_has_remaining and resume_has_imaging_geometry:
        resume_waypoints, removed_sweep_points = (
            _trim_waypoints_for_exact_sweep_progress(
                resume_waypoints,
                resume_progress_entry,
                reference_coord=current_coord,
            )
        )
        emit(
            f"{log_prefix} exact resume-path progress overrides stale waypoint completion "
            f"(aircraft={aircraft_id}, pathID={resume_path_id}, "
            f"removedSweepPoints={removed_sweep_points}, "
            f"remainingSweepPoints={count_sweep_points_in_waypoints(resume_waypoints)})."
        )
    carrier_resume_path_done = bool(
        resume_path_payload
        and resume_waypoints
        and _first_not_done_waypoint_index(resume_waypoints) is None
    )
    if progress_is_authoritative and resume_has_imaging_geometry:
        resume_path_done = not progress_has_remaining
    else:
        resume_path_done = bool(carrier_resume_path_done)
    preserve_type2_without_progress = bool(
        type2_branch_line
        and not progress_is_authoritative
        and isinstance(resume_mission, dict)
        and not resume_mission_done
        and resume_has_imaging_geometry
    )
    # A LINE-search waypoint carries two different coordinate domains:
    # ``waypoint.coordinate`` is the aircraft flight position, while
    # ``lineSearch.coordinateList`` contains the camera's ground sweep points.
    # The return leg must target the former; using the first sweep point here
    # can send the aircraft far away from the route before imaging resumes.
    first_resume_coord = _first_waypoint_flight_coordinate(resume_waypoints)
    resume_branch_has_remaining_imaging = bool(
        resume_index is not None
        and isinstance(resume_mission, dict)
        and not resume_mission_done
        and first_resume_coord is not None
        and resume_has_imaging_geometry
        and (
            progress_has_remaining
            or preserve_type2_without_progress
            or not resume_path_done
        )
    )
    follow_up_start_idx = (
        int(resume_index)
        if resume_branch_has_remaining_imaging
        else (
            max(
                idx
                for idx in (tracking_index, resume_index)
                if idx is not None
            ) + 1 if (tracking_index is not None or resume_index is not None) else 0
        )
    )
    follow_up_source_missions = [
        mission
        for mission in mission_list[int(follow_up_start_idx) :]
        if isinstance(mission, dict)
    ]
    if block_follow_up_until_reassignment:
        emit(
            f"{log_prefix} future input missions will be retained but execution-blocked "
            f"until the next collaborative mission handoff "
            f"(aircraft={aircraft_id}, inputMissionID={current_input_id})."
        )
    first_follow_up_path_payload = None
    if follow_up_source_missions and not block_follow_up_until_reassignment:
        first_follow_up_mission = follow_up_source_missions[0]
        if isinstance(first_follow_up_mission, dict):
            first_follow_up_path_payload = _load_path_payload(
                first_follow_up_mission.get("pathID"),
                run_cache=run_cache,
            )
    first_follow_up_coord = _first_waypoint_flight_coordinate(
        (first_follow_up_path_payload or {}).get("waypointList") or []
    )
    use_follow_up_return_target = bool(
        not block_follow_up_until_reassignment
        and (resume_mission_done or resume_path_done)
        and first_follow_up_coord is not None
    )
    follow_up_source_has_missions = bool(
        resume_branch_has_remaining_imaging
        if block_follow_up_until_reassignment
        else _has_mission_entries(follow_up_source_missions)
    )
    template_final_coord = (
        _normalize_coordinate(_extract_final_uav_coordinate(resume_path_payload or {}))
        or _normalize_coordinate(_extract_final_uav_coordinate(original_path_payload or {}))
    )
    point_only_return = bool(
        not resume_branch_has_remaining_imaging
        and not use_follow_up_return_target
        and not follow_up_source_has_missions
    )
    area_internal_return_coord = None
    if point_only_return:
        area_internal_return_coord = _input_area_internal_hold_coordinate(
            source_plan_id=int(attack_plan_id),
            current_input_id=int(current_input_id),
            altitude_sources=[template_final_coord, current_coord],
            run_cache=run_cache,
        )
    final_coord = (
        first_resume_coord
        if resume_branch_has_remaining_imaging and first_resume_coord is not None
        else first_follow_up_coord
        if use_follow_up_return_target and first_follow_up_coord is not None
        else area_internal_return_coord
        if area_internal_return_coord is not None
        else (
            template_final_coord
        )
    )
    if area_internal_return_coord is not None:
        emit(
            f"{log_prefix} point-only return target moved inside current area "
            f"(aircraft={aircraft_id}, inputMissionID={current_input_id})."
        )
    if resume_branch_has_remaining_imaging:
        emit(
            f"{log_prefix} resume imaging branch preserved after return-only boundary "
            f"(aircraft={aircraft_id}, resumeIndex={resume_index})."
        )
    if use_follow_up_return_target:
        emit(
            f"{log_prefix} resume branch already done; return target moved to first follow-up path "
            f"(aircraft={aircraft_id})."
        )
    if final_coord is None:
        final_coord = _normalize_coordinate((template_path_payload.get("waypointList") or [{}])[-1].get("coordinate"))
    if final_coord is None:
        emit(f"{log_prefix} final coordinate unavailable for aircraft {aircraft_id}.")
        return None

    return_waypoints, return_speed_mps = _build_uav_release_resume_waypoints(
        start_coord=current_coord,
        end_coord=final_coord,
        release_eta_s=0,
        target_finish_eta_s=0,
        default_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
        min_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
        max_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
        force_speed_mps=_RELEASE_RESUME_FAST_SPEED_MPS,
        # Assign after the follow-up prepass so this return path and every
        # cloned follow-up share one global ID reservation/lock acquisition.
        assign_waypoint_ids=False,
    )
    raw_return_waypoint_count = sum(1 for item in return_waypoints if isinstance(item, dict))

    clone_count = _post_attack_follow_up_clone_count(follow_up_source_missions)
    follow_up_waypoint_count = 0
    follow_up_waypoint_count_complete = True
    for follow_up_mission in follow_up_source_missions:
        if not isinstance(follow_up_mission, dict):
            continue
        if _skip_replan_follow_up_reason(follow_up_mission, excluded_input_ids=set()) is not None:
            continue
        follow_up_path_id = _to_int(follow_up_mission.get("pathID"))
        if follow_up_path_id is None or follow_up_path_id <= 0:
            # Preserve the clone helper's existing validation/error path.
            follow_up_waypoint_count_complete = False
            break
        try:
            follow_up_path = read_json_cached(
                db_paths.get_db_subpath("FlightPath", f"{int(follow_up_path_id)}.json"),
                kind="FlightPath",
            )
        except Exception:
            # Preserve the clone helper's existing validation/error path.
            follow_up_waypoint_count_complete = False
            break
        for waypoint_key in ("waypointList", "uavWaypointList", "lahWaypointList"):
            follow_up_waypoints = follow_up_path.get(waypoint_key)
            if isinstance(follow_up_waypoints, list):
                follow_up_waypoint_count += sum(
                    1 for item in follow_up_waypoints if isinstance(item, dict)
                )
    if not follow_up_waypoint_count_complete:
        follow_up_waypoint_count = 0

    reservation_summaries: List[Dict[str, Any]] = []
    id_reservation = ReplanIdReservation.reserve(
        imp_count=1,
        individual_count=int(clone_count) + 1,
        path_count_by_aircraft={int(aircraft_id): int(clone_count) + 1},
        waypoint_count=int(raw_return_waypoint_count) + int(follow_up_waypoint_count),
    )
    # Assign before short-path collapse.  The discarded midpoint therefore
    # consumes exactly the same ID as before, preserving every emitted ID.
    if return_waypoints:
        reassign_unique_waypoint_ids_inplace(
            return_waypoints,
            waypoint_id_provider=id_reservation.next_waypoint,
        )
    return_waypoints, return_path_collapsed = _collapse_short_return_waypoints(
        return_waypoints,
        aircraft_id=int(aircraft_id),
        emit=emit,
        log_prefix=log_prefix,
    )
    if not return_waypoints:
        anchor_wp = _build_uav_transit_waypoint(
            coordinate=final_coord,
            speed_mps=float(_RELEASE_RESUME_FAST_SPEED_MPS),
            eta_s=0,
            orientation_coordinate=final_coord,
            waypoint_pass_type=3,
        )
        return_waypoints = [anchor_wp]
        return_speed_mps = float(_RELEASE_RESUME_FAST_SPEED_MPS)
        return_path_collapsed = False

    # A tracking UAV's return-only branch is the same collaborative completion
    # boundary as the non-tracking UAV holds produced above.  Always emit the
    # same executable terminal-loiter contract; leaving a collapsed short
    # return as passType=3 + empty loiter makes this aircraft the only member
    # whose completion still depends on physically crossing its marker.
    completion_boundary_hold = True
    _apply_post_attack_terminal_hold(
        return_waypoints,
        hold_seconds=int(_POST_ATTACK_COMPLETE_HOLD_SECONDS),
        orientation_coordinate=final_coord,
    )

    new_individual_id = int(id_reservation.next_individual())
    new_path_id = int(id_reservation.next_path(int(aircraft_id)))

    return_mission = _sanitize_post_attack_mission_entry(
        deepcopy(template_mission),
        current_input_id=int(current_input_id),
    )
    return_mission.pop("executionBlockedUntilNextCollab", None)
    return_mission["individualMissionID"] = int(new_individual_id)
    return_mission["pathID"] = int(new_path_id)
    return_mission["isDone"] = False
    return_info = return_mission.get("individualMissionInfo")
    return_info = deepcopy(return_info if isinstance(return_info, dict) else {})
    return_info["individualMissionType"] = 7
    return_info["patternType"] = 10
    return_info["targetID"] = None
    return_info["lineList"] = []
    return_info["areaList"] = []
    return_info["SPEED"] = float(return_speed_mps)
    return_mission["individualMissionInfo"] = return_info
    _apply_release_resume_mission_info(
        return_mission,
        start_coord=current_coord,
        end_coord=final_coord,
    )
    if return_path_collapsed:
        return_info = return_mission.get("individualMissionInfo")
        return_info = deepcopy(return_info if isinstance(return_info, dict) else {})
        return_info["coordinateList"] = [deepcopy(final_coord)]
        return_info["lineList"] = []
        return_info["areaList"] = []
        return_mission["individualMissionInfo"] = return_info

    return_path_payload = deepcopy(template_path_payload)
    return_path_payload["pathID"] = int(new_path_id)
    return_path_payload["timestamp"] = int(now_ms)
    return_path_payload["Source"] = return_path_payload.get("Source") or "MMR"
    return_path_payload["aircraftID"] = int(aircraft_id)
    return_path_payload["individualMissionID"] = int(new_individual_id)
    return_path_payload["waypointList"] = [deepcopy(item) for item in return_waypoints if isinstance(item, dict)]
    _apply_runtime_flyover_to_flight_path_payload(return_path_payload)
    sanitize_flight_path_payload_filming_altitudes(return_path_payload)
    if completion_boundary_hold:
        return_mission["postAttackBoundaryHold"] = True
        return_path_payload["postAttackBoundaryHold"] = True
        for waypoint in return_path_payload.get("waypointList") or []:
            if isinstance(waypoint, dict):
                waypoint["postAttackBoundaryHold"] = True
    cloned_artifacts = _clone_follow_up_replan_artifacts(
        missions=follow_up_source_missions,
        aircraft_id=int(aircraft_id),
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
        individual_id_provider=id_reservation.next_individual if int(clone_count) > 0 else None,
        path_id_provider=id_reservation.next_path if int(clone_count) > 0 else None,
        waypoint_id_provider=(
            id_reservation.next_waypoint
            if int(clone_count) > 0 and follow_up_waypoint_count_complete
            else None
        ),
        reservation_summaries=reservation_summaries,
        reservation_scope="postAttackTrackingReturnFollowUpClone",
    )
    if cloned_artifacts is None:
        return None
    follow_up_missions, follow_up_paths = cloned_artifacts
    if (
        type2_branch_line
        and resume_branch_has_remaining_imaging
        and progress_has_remaining
        and isinstance(resume_progress_entry, dict)
    ):
        cloned_resume_mission = next(
            (
                mission
                for mission in follow_up_missions
                if isinstance(mission, dict)
                and _extract_related_input_mission_id(mission) == int(current_input_id)
                and _mission_has_post_attack_imaging_geometry(mission, [])
            ),
            None,
        )
        cloned_resume_path_id = _to_int(
            (cloned_resume_mission or {}).get("pathID")
        )
        cloned_resume_path = next(
            (
                payload
                for _dest, payload in follow_up_paths
                if isinstance(payload, dict)
                and cloned_resume_path_id is not None
                and _to_int(payload.get("pathID")) == int(cloned_resume_path_id)
            ),
            None,
        )
        cloned_resume_waypoints = (
            cloned_resume_path.get("waypointList")
            if isinstance(cloned_resume_path, dict)
            and isinstance(cloned_resume_path.get("waypointList"), list)
            else []
        )
        if isinstance(cloned_resume_mission, dict) and cloned_resume_waypoints:
            trimmed_clone_waypoints, removed_clone_sweep_points = (
                _trim_waypoints_for_exact_sweep_progress(
                    cloned_resume_waypoints,
                    resume_progress_entry,
                    reference_coord=current_coord,
                )
            )
            cloned_resume_path["waypointList"] = trimmed_clone_waypoints
            _sync_resume_mission_info_with_waypoints(
                cloned_resume_mission,
                trimmed_clone_waypoints,
            )
            emit(
                f"{log_prefix} cloned current LINE trimmed at post-attack progress "
                f"(aircraft={aircraft_id}, sourcePathID={resume_path_id}, "
                f"newPathID={cloned_resume_path_id}, "
                f"removedSweepPoints={removed_clone_sweep_points})."
            )
    if block_follow_up_until_reassignment:
        blocked_follow_up_count = _mark_post_attack_followups_execution_blocked(
            follow_up_missions,
            current_input_id=int(current_input_id),
        )
        emit(
            f"{log_prefix} retained future mission artifacts "
            f"(aircraft={aircraft_id}, blockedFollowUps={int(blocked_follow_up_count)})."
        )

    new_imp_data = _copy_post_attack_imp_shell(imp_data)
    new_imp_id = int(id_reservation.next_imp())
    new_imp_data["individualMissionPackageID"] = int(new_imp_id)
    new_imp_data["timestamp"] = int(now_ms)
    new_imp_data["individualMissionList"] = [return_mission] + [
        deepcopy(mission) for mission in follow_up_missions
    ]
    new_imp_data.pop("deferredIndividualMissionList", None)

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(new_imp_id)}.json")
    path_dest = db_paths.get_db_subpath("FlightPath", f"{int(new_path_id)}.json")
    imp_dest.parent.mkdir(parents=True, exist_ok=True)
    path_dest.parent.mkdir(parents=True, exist_ok=True)
    sanitize_flight_path_payload_filming_altitudes(return_path_payload)
    generated_flight_paths = [return_path_payload] + [
        payload for _dest, payload in follow_up_paths if isinstance(payload, dict)
    ]
    _validate_generated_post_attack_artifact_payloads(
        individual_mission_plans=[new_imp_data],
        flight_paths=generated_flight_paths,
        scope=f"postAttackTrackingReturnOnly:{new_imp_id}",
        allow_existing_db_artifacts=True,
        log=emit,
    )
    generated_path_ids: Set[int] = {int(new_path_id)}
    write_entries: List[Tuple[Path, Dict[str, Any]]] = [
        (imp_dest, new_imp_data),
        (path_dest, return_path_payload),
    ]
    for dest, payload in follow_up_paths:
        dest.parent.mkdir(parents=True, exist_ok=True)
        _apply_runtime_flyover_to_flight_path_payload(payload)
        sanitize_flight_path_payload_filming_altitudes(payload)
        write_entries.append((dest, payload))
        path_id = _to_int((payload or {}).get("pathID"))
        if path_id is not None and path_id > 0:
            generated_path_ids.add(int(path_id))
    _write_or_defer_post_attack_json_batch(
        write_entries,
        run_cache=run_cache,
    )

    emit(
        f"{log_prefix} tracking branch replaced with return-only package "
        f"(aircraft={aircraft_id}, imp={imp_dest.name}, path={path_dest.name}, "
        f"hold={int(_POST_ATTACK_COMPLETE_HOLD_SECONDS) if completion_boundary_hold else 0}s, "
        f"followUps={len(follow_up_missions)}, "
        f"speed={float(return_speed_mps):.1f})."
    )
    direct_reservation = _post_attack_reservation_event(
        scope="postAttackTrackingReturnOnly",
        aircraft_id=int(aircraft_id),
        imp_ids=[int(new_imp_id)],
        individual_ids=[int(new_individual_id)],
        path_ids_by_aircraft={int(aircraft_id): [int(new_path_id)]},
    )
    reservation_summaries.insert(0, direct_reservation)
    return {
        "aircraft_id": int(aircraft_id),
        "individualMissionPackageID": int(new_imp_id),
        "generatedPathIDs": sorted(int(pid) for pid in generated_path_ids if int(pid) > 0),
        "returnMission": {
            "individualMissionID": int(new_individual_id),
            "pathID": int(new_path_id),
            "finalCoordinate": dict(final_coord),
        },
        "followUpMissionCount": len(follow_up_missions),
        "completionBoundaryHold": bool(completion_boundary_hold),
        "followUpsBlockedUntilNextCollab": bool(block_follow_up_until_reassignment),
        "reservedIds": direct_reservation.get("reservedIds", {}),
        "reservationSummaries": reservation_summaries,
    }


def _apply_post_attack_terminal_hold(
    waypoints: List[Dict[str, Any]],
    *,
    hold_seconds: int,
    orientation_coordinate: Dict[str, Any],
) -> None:
    if not isinstance(waypoints, list) or not waypoints:
        return
    terminal = waypoints[-1]
    if not isinstance(terminal, dict):
        return
    hold_seconds = int(_POST_ATTACK_COMPLETE_HOLD_SECONDS)
    terminal["waypointPassType"] = 2
    terminal["eta"] = max(int(_to_float(terminal.get("eta")) or 0), int(hold_seconds))
    terminal["nextWaypointID"] = 0
    terminal["isDone"] = False
    terminal["loiterProperty"] = {
        "radius": int(_POST_ATTACK_COMPLETE_HOLD_RADIUS_M),
        "direction": 1,
        "time": int(hold_seconds),
        "speed": int(round(_POST_ATTACK_COMPLETE_HOLD_SPEED_MPS)),
    }
    filming = terminal.get("filmingProperty")
    filming = deepcopy(filming) if isinstance(filming, dict) else {}
    filming["operationMode"] = 1
    filming["sensorType"] = _to_int(filming.get("sensorType")) or 1
    if _to_float(filming.get("fieldOfView")) is None:
        filming["fieldOfView"] = 5.0
    filming.pop("lineSearch", None)
    filming.pop("areaSearch", None)
    filming.pop("autoTracking", None)
    coord_orientation = filming.get("coordinateOrientation")
    coord_orientation = deepcopy(coord_orientation) if isinstance(coord_orientation, dict) else {}
    coord_orientation["coordinate"] = deepcopy(orientation_coordinate)
    filming["coordinateOrientation"] = coord_orientation
    terminal["filmingProperty"] = filming


def _collect_path_coordinates(waypoints: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    coords: List[Dict[str, Any]] = []
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        for item in line_search.get("coordinateList") or []:
            coord = _normalize_coordinate(item)
            if coord is not None:
                coords.append(coord)
        coord = _normalize_coordinate(waypoint.get("coordinate"))
        if coord is not None:
            coords.append(coord)
    deduped: List[Dict[str, Any]] = []
    for coord in coords:
        if not deduped:
            deduped.append(dict(coord))
            continue
        prev = deduped[-1]
        if (
            abs(float(prev["latitude"]) - float(coord["latitude"])) <= 1e-8
            and abs(float(prev["longitude"]) - float(coord["longitude"])) <= 1e-8
        ):
            continue
        deduped.append(dict(coord))
    return deduped


def _last_path_coordinate(waypoints: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    coords = _collect_path_coordinates(waypoints)
    return dict(coords[-1]) if coords else None


def _path_heading_from_waypoints(waypoints: List[Dict[str, Any]]) -> Optional[float]:
    coords = _collect_path_coordinates(waypoints)
    if len(coords) < 2:
        return None
    return _bearing_between(coords[-2], coords[-1])


def _find_current_waypoint_index(waypoints: List[Dict[str, Any]], current_waypoint_id: Optional[int]) -> Optional[int]:
    current_waypoint_id = _to_int(current_waypoint_id)
    if current_waypoint_id is None:
        return None
    for idx, waypoint in enumerate(waypoints):
        if _to_int((waypoint or {}).get("waypointID")) == int(current_waypoint_id):
            return idx
    return None


def _first_not_done_waypoint_index(waypoints: List[Dict[str, Any]]) -> Optional[int]:
    for idx, waypoint in enumerate(waypoints):
        if not bool((waypoint or {}).get("isDone")):
            return idx
    return None


def _select_rejoin_reference_coordinate(
    *,
    active_aircraft_ids: List[int],
    agent_state_map: Dict[int, Dict[str, Any]],
    current_plan_id: int,
    current_input_id: int,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Optional[Dict[str, Any]]:
    active_coords = [
        _normalize_coordinate((agent_state_map.get(int(aid)) or {}).get("coordinate"))
        for aid in active_aircraft_ids
    ]
    active_coords = [coord for coord in active_coords if coord is not None]
    if active_coords:
        return _centroid_coordinate(active_coords)

    current_input_mission, _ = _build_remaining_input_mission_for_collaborative_replan(
        source_plan_id=int(current_plan_id),
        current_input_id=int(current_input_id),
    )
    if not isinstance(current_input_mission, dict):
        return None
    return _extract_input_mission_reference_coordinate(current_input_mission)


def _extract_input_mission_reference_coordinate(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    for key in ("coordinateList",):
        values = detail.get(key)
        if isinstance(values, list):
            for entry in values:
                coord = _normalize_coordinate(entry)
                if coord is not None:
                    return coord
    for key in ("lineList", "areaList"):
        values = detail.get(key)
        if not isinstance(values, list):
            continue
        for entry in values:
            if not isinstance(entry, dict):
                continue
            coord_lists = []
            for nested_key in ("coordinateList", "coordList", "pointList"):
                nested = entry.get(nested_key)
                if isinstance(nested, list):
                    coord_lists.append(nested)
            for coord_list in coord_lists:
                for coord_entry in coord_list:
                    coord = _normalize_coordinate(coord_entry)
                    if coord is not None:
                        return coord
    return None


def _has_mission_entries(missions: Any) -> bool:
    return any(isinstance(mission, dict) for mission in (missions or []))


def _post_attack_follow_up_source_missions(
    mission_list: List[Dict[str, Any]],
    clone_start_idx: int,
) -> List[Dict[str, Any]]:
    return [
        mission
        for mission in mission_list[max(0, int(clone_start_idx)) :]
        if isinstance(mission, dict)
    ]


def _input_area_internal_hold_coordinate(
    *,
    source_plan_id: int,
    current_input_id: int,
    altitude_sources: Optional[List[Any]] = None,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Optional[Dict[str, Any]]:
    detail = _remaining_snapshot_area_detail(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        run_cache=run_cache,
    )
    area_rows = _area_rows_from_detail(detail)
    if not area_rows:
        input_data = _load_input_plan_for_source_plan(int(source_plan_id))
        if isinstance(input_data, dict):
            for mission in input_data.get("inputMissionList") or []:
                if not isinstance(mission, dict):
                    continue
                if _to_int(mission.get("inputMissionID")) != int(current_input_id):
                    continue
                mission_detail = (
                    mission.get("missionDetail")
                    if isinstance(mission.get("missionDetail"), dict)
                    else {}
                )
                area_rows = _area_rows_from_detail(mission_detail)
                break
    if not area_rows:
        return None

    coord = _representative_area_coordinate(area_rows)
    if coord is None:
        return None

    altitude = _preferred_hold_altitude(altitude_sources or [], area_rows)
    if altitude is not None:
        coord["altitude"] = int(round(float(altitude)))
    return coord


def _remaining_snapshot_area_detail(
    *,
    source_plan_id: int,
    current_input_id: int,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> Dict[str, Any]:
    key = (int(source_plan_id), int(current_input_id))
    if run_cache is not None and run_cache.remaining_snapshot_details is not None:
        cached = run_cache.remaining_snapshot_details.get(key)
        if isinstance(cached, dict):
            return deepcopy(cached)
    try:
        snapshot_info = mission_area_replan_store.load_replan_ready_snapshot_entry(
            int(source_plan_id),
            int(current_input_id),
            allow_latest=True,
            allow_latest_area=True,
            audit_context="post_attack_remaining_area_detail",
        )
    except Exception:
        return {}
    if not isinstance(snapshot_info, dict):
        return {}
    entry = snapshot_info.get("entry")
    if not isinstance(entry, dict):
        return {}
    reject_reason = mission_area_replan_store.snapshot_entry_replan_reject_reason(
        entry,
        exact=bool(snapshot_info.get("exact")) if "exact" in snapshot_info else None,
        allow_latest_area=True,
    )
    if reject_reason == "area_snapshot_latest_fallback_not_allowed":
        mission_area_replan_store.audit_snapshot_entry_rejected(
            entry,
            requested_mission_plan_id=int(source_plan_id),
            snapshot_mission_plan_id=_to_int(snapshot_info.get("snapshotMissionPlanID")),
            audit_context="post_attack_remaining_area_detail",
            reason=str(reject_reason),
        )
        return {}
    detail = mission_area_replan_store.coverage_replan_pending_remaining_detail(entry)
    if not _remaining_detail_has_geometry(detail):
        result = detail if isinstance(detail, dict) else {}
        if run_cache is not None and run_cache.remaining_snapshot_details is not None:
            run_cache.remaining_snapshot_details[key] = deepcopy(result)
        return result
    if reject_reason:
        mission_area_replan_store.audit_snapshot_entry_rejected(
            entry,
            requested_mission_plan_id=int(source_plan_id),
            snapshot_mission_plan_id=_to_int(snapshot_info.get("snapshotMissionPlanID")),
            audit_context="post_attack_remaining_area_detail",
            reason=str(reject_reason),
        )
        if str(reject_reason) != "area_snapshot_not_ready_for_replan":
            return {}
    result = deepcopy(detail) if isinstance(detail, dict) else {}
    mission_area_replan_store.apply_area_coverage_replan_contracts(result, entry)
    if run_cache is not None and run_cache.remaining_snapshot_details is not None:
        run_cache.remaining_snapshot_details[key] = deepcopy(result)
    return result


def _area_pass_reassignment_pending(
    *,
    source_plan_id: int,
    current_input_id: int,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> bool:
    detail = _remaining_snapshot_area_detail(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        run_cache=run_cache,
    )
    if not isinstance(detail, dict):
        return False
    contract = mission_area_replan_store.coverage_pass_replan_contract(detail)
    remaining_passes = {
        str(value or "").strip().lower()
        for value in (contract.get("remainingCoveragePasses") or [])
    }
    return bool(remaining_passes.intersection({"forward", "reverse"}))


def _area_rows_from_detail(detail: Any) -> List[Dict[str, Any]]:
    if not isinstance(detail, dict):
        return []
    rows: List[Dict[str, Any]] = []
    area_list = detail.get("areaList")
    if isinstance(area_list, list):
        for row in area_list:
            if not isinstance(row, dict):
                continue
            coords = _normalize_area_coordinate_list(row.get("coordinateList"))
            if len(coords) >= 3:
                rows.append({"isHole": bool(row.get("isHole")), "coordinateList": coords})
    if rows:
        return rows
    area_segment_list = detail.get("areaSegmentList")
    if isinstance(area_segment_list, list):
        for row in area_segment_list:
            if not isinstance(row, dict):
                continue
            coords = _normalize_area_coordinate_list(row.get("coordinateList"))
            if len(coords) >= 3:
                rows.append({"isHole": False, "coordinateList": coords})
    if rows:
        return rows
    coords = _normalize_area_coordinate_list(detail.get("coordinateList"))
    if len(coords) >= 3:
        rows.append({"isHole": False, "coordinateList": coords})
    return rows


def _normalize_area_coordinate_list(values: Any) -> List[Dict[str, Any]]:
    if not isinstance(values, list):
        return []
    coords: List[Dict[str, Any]] = []
    for item in values:
        coord = _normalize_coordinate(item)
        if coord is None:
            continue
        altitude = _coordinate_altitude(item)
        if altitude is not None:
            coord["altitude"] = int(round(float(altitude)))
        if coords:
            prev = coords[-1]
            if (
                abs(float(prev["latitude"]) - float(coord["latitude"])) < 1e-12
                and abs(float(prev["longitude"]) - float(coord["longitude"])) < 1e-12
            ):
                continue
        coords.append(coord)
    if len(coords) >= 2:
        first = coords[0]
        last = coords[-1]
        if (
            abs(float(first["latitude"]) - float(last["latitude"])) < 1e-12
            and abs(float(first["longitude"]) - float(last["longitude"])) < 1e-12
        ):
            coords = coords[:-1]
    return coords


def _representative_area_coordinate(area_rows: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    try:
        from shapely.geometry import Polygon
        from shapely.ops import unary_union

        outer_polys = []
        hole_polys = []
        for row in area_rows or []:
            coords = row.get("coordinateList") if isinstance(row, dict) else None
            if not isinstance(coords, list) or len(coords) < 3:
                continue
            xy = [(float(coord["longitude"]), float(coord["latitude"])) for coord in coords]
            poly = Polygon(xy)
            if poly.is_empty:
                continue
            if not poly.is_valid:
                poly = poly.buffer(0)
            poly = _largest_shapely_polygon(poly)
            if poly is None or poly.is_empty:
                continue
            if bool(row.get("isHole")):
                hole_polys.append(poly)
            else:
                outer_polys.append(poly)
        if outer_polys:
            geom = unary_union(outer_polys)
            if hole_polys:
                geom = geom.difference(unary_union(hole_polys))
            poly = _largest_shapely_polygon(geom)
            if poly is not None and not poly.is_empty:
                point = poly.representative_point()
                return {"latitude": float(point.y), "longitude": float(point.x)}
    except Exception:
        pass
    return _fallback_area_coordinate(area_rows)


def _largest_shapely_polygon(geometry: Any) -> Any:
    if geometry is None or bool(getattr(geometry, "is_empty", False)):
        return None
    if str(getattr(geometry, "geom_type", "")) == "Polygon":
        return geometry
    geoms = getattr(geometry, "geoms", None)
    if geoms is None:
        return None
    polygons = [
        item
        for item in geoms
        if str(getattr(item, "geom_type", "")) == "Polygon"
        and not bool(getattr(item, "is_empty", False))
    ]
    if not polygons:
        return None
    return max(polygons, key=lambda item: float(getattr(item, "area", 0.0) or 0.0))


def _fallback_area_coordinate(area_rows: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for row in area_rows or []:
        if not isinstance(row, dict) or bool(row.get("isHole")):
            continue
        coords = row.get("coordinateList")
        if not isinstance(coords, list) or len(coords) < 3:
            continue
        coord = _centroid_coordinate(coords)
        if coord is not None:
            return {
                "latitude": float(coord["latitude"]),
                "longitude": float(coord["longitude"]),
            }
    return None


def _preferred_hold_altitude(
    altitude_sources: List[Any],
    area_rows: List[Dict[str, Any]],
) -> Optional[float]:
    for source in altitude_sources or []:
        altitude = _coordinate_altitude(source)
        if altitude is not None:
            return float(altitude)
    altitudes: List[float] = []
    for row in area_rows or []:
        coords = row.get("coordinateList") if isinstance(row, dict) else None
        if not isinstance(coords, list):
            continue
        for coord in coords:
            altitude = _coordinate_altitude(coord)
            if altitude is not None:
                altitudes.append(float(altitude))
    if not altitudes:
        return None
    return sum(altitudes) / float(len(altitudes))


def _coordinate_altitude(value: Any) -> Optional[float]:
    if not isinstance(value, dict):
        return None
    for key in ("altitude", "alt"):
        if key not in value:
            continue
        altitude = _to_float(value.get(key))
        if altitude is not None and math.isfinite(float(altitude)):
            return float(altitude)
    return None


def _has_remaining_snapshot_geometry(
    source_plan_id: int,
    current_input_id: int,
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> bool:
    return _has_remaining_snapshot_geometry_cached(
        int(source_plan_id),
        int(current_input_id),
        run_cache=run_cache,
    )


def _remaining_snapshot_explicitly_completed(
    source_plan_id: int,
    current_input_id: int,
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> bool:
    """Distinguish an authoritative completed entry from a missing snapshot."""

    key = (int(source_plan_id), int(current_input_id))
    if run_cache is not None and run_cache.remaining_snapshot_completed is not None:
        cached = run_cache.remaining_snapshot_completed.get(key)
        if cached is not None:
            return bool(cached)

    snapshot_info = mission_area_replan_store.load_snapshot_entry(
        int(source_plan_id),
        int(current_input_id),
        allow_latest=False,
        audit_context="post_attack_remaining_snapshot_completion_check",
    )
    completed = False
    if isinstance(snapshot_info, dict) and bool(snapshot_info.get("exact")):
        entry = snapshot_info.get("entry")
        if isinstance(entry, dict):
            has_geometry = bool(_remaining_detail_has_geometry(entry.get("remainingDetail")))
            remaining_area = _to_float(entry.get("remainingAreaM2"))
            completed = bool(
                not has_geometry
                and (
                    bool(entry.get("isDone"))
                    or (
                        remaining_area is not None
                        and float(remaining_area) <= 10.0
                    )
                )
            )

    if run_cache is not None and run_cache.remaining_snapshot_completed is not None:
        run_cache.remaining_snapshot_completed[key] = bool(completed)
    return bool(completed)


def _has_remaining_snapshot_geometry_cached(
    source_plan_id: int,
    current_input_id: int,
    *,
    run_cache: Optional[_PostAttackRunCache] = None,
) -> bool:
    key = (int(source_plan_id), int(current_input_id))
    if run_cache is not None and run_cache.remaining_snapshot_geometry is not None:
        cached = run_cache.remaining_snapshot_geometry.get(key)
        if cached is not None:
            return bool(cached)
    snapshot_info = mission_area_replan_store.load_replan_ready_snapshot_entry(
        int(source_plan_id),
        int(current_input_id),
        allow_latest=True,
        allow_latest_area=True,
        audit_context="post_attack_remaining_snapshot_geometry_check",
    )
    result = False
    if isinstance(snapshot_info, dict):
        item = snapshot_info.get("entry")
        if isinstance(item, dict):
            has_geometry = bool(_remaining_detail_has_geometry(item.get("remainingDetail")))
            reject_reason = mission_area_replan_store.snapshot_entry_replan_reject_reason(
                item,
                exact=bool(snapshot_info.get("exact")) if "exact" in snapshot_info else None,
                allow_latest_area=True,
            )
            if has_geometry and reject_reason:
                mission_area_replan_store.audit_snapshot_entry_rejected(
                    item,
                    requested_mission_plan_id=int(source_plan_id),
                    snapshot_mission_plan_id=_to_int(snapshot_info.get("snapshotMissionPlanID")),
                    audit_context="post_attack_remaining_snapshot_geometry_check",
                    reason=str(reject_reason),
                )
                if str(reject_reason) == "area_snapshot_not_ready_for_replan":
                    result = True
            else:
                result = bool(has_geometry)
    if run_cache is not None and run_cache.remaining_snapshot_geometry is not None:
        run_cache.remaining_snapshot_geometry[key] = bool(result)
    return bool(result)


def _resolve_assignment_input_mission_id(assignment: Dict[str, Any]) -> Optional[int]:
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


def _resolve_group_source_plan_id(
    group_assignments: List[Dict[str, Any]],
    *,
    fallback_plan_id: int,
) -> int:
    candidate_ids: List[int] = []
    for assignment in group_assignments or []:
        if not isinstance(assignment, dict):
            continue
        source_plan_id = _to_int(assignment.get("source_plan_id"))
        if source_plan_id is None or source_plan_id <= 0:
            continue
        candidate_ids.append(int(source_plan_id))
    if not candidate_ids:
        return int(fallback_plan_id)
    # Tracking assignments for the same closed target should share one source plan.
    return int(candidate_ids[0])


def _index_agent_states(
    agent_states: List[Any],
    waypoint_memory: Optional[Dict[str, Any]],
) -> Dict[int, Dict[str, Any]]:
    index: Dict[int, Dict[str, Any]] = {}
    waypoint_memory = waypoint_memory if isinstance(waypoint_memory, dict) else {}
    for entry in agent_states:
        if not isinstance(entry, dict):
            continue
        aircraft_id = _to_int(entry.get("aircraftID") or entry.get("aircraftId"))
        if aircraft_id is None:
            continue
        coord = (
            _normalize_coordinate(entry.get("coordinate"))
            or _normalize_coordinate((entry.get("mannedInfo") or {}).get("coordinate"))
            or _normalize_coordinate((entry.get("unmannedInfo") or {}).get("coordinate"))
        )
        wp_block = entry.get("currentWaypointID") or {}
        if not wp_block:
            wp_block = ((entry.get("unmannedInfo") or {}).get("currentWaypointID")) or {}
        current_wp = _to_int((wp_block or {}).get("waypointID"))
        if current_wp is None:
            current_wp = _to_int(
                waypoint_memory.get(str(aircraft_id))
                if str(aircraft_id) in waypoint_memory
                else waypoint_memory.get(aircraft_id)
            )
        velocity = entry.get("velocity") if isinstance(entry.get("velocity"), dict) else {}
        unmanned = entry.get("unmannedInfo") if isinstance(entry.get("unmannedInfo"), dict) else {}
        unmanned_velocity = unmanned.get("velocity") if isinstance(unmanned.get("velocity"), dict) else {}
        if not velocity and unmanned_velocity:
            velocity = unmanned_velocity
        heading = _to_float(
            entry.get("headingDeg")
            if entry.get("headingDeg") is not None
            else entry.get("heading")
        )
        if heading is None:
            heading = _to_float(velocity.get("heading"))
        if heading is not None:
            heading = heading % 360.0
        speed = _to_float(
            entry.get("speedMps")
            if entry.get("speedMps") is not None
            else entry.get("speed")
        )
        if speed is None:
            speed = _to_float(velocity.get("speed"))
        sensor_info = unmanned.get("sensorInfo") if isinstance(unmanned.get("sensorInfo"), dict) else {}
        sensor_center_coord = _normalize_coordinate(sensor_info.get("centerCoordinate"))
        on_mission = _to_int(unmanned.get("onMission") or entry.get("onMission"))
        state_row = {
            "coordinate": coord,
            "sensor_center_coordinate": sensor_center_coord,
            "current_waypoint_id": current_wp,
            "heading": heading,
            "speed": speed,
            "on_mission": on_mission,
        }
        if velocity:
            state_row["velocity"] = deepcopy(velocity)
        if heading is not None:
            state_row["headingDeg"] = float(heading) % 360.0
        if speed is not None and speed > 0.0:
            state_row["speedMps" if speed <= 70.0 else "speed"] = float(speed)
        for key in (
            "turnRateDps",
            "turn_rate_dps",
            "yawRateDps",
            "yaw_rate_dps",
            "turnSign",
            "turnRadiusM",
            "turnCircleRadiusM",
            "actualTurnRadiusM",
            "idealTurnRadiusM",
        ):
            value = entry.get(key)
            if value is None and velocity:
                value = velocity.get(key)
            if value is None and unmanned_velocity:
                value = unmanned_velocity.get(key)
            if value is not None:
                state_row[key] = value
        index[int(aircraft_id)] = state_row
    return index


def _estimate_turn_aware_eta_s(
    *,
    origin: Optional[Dict[str, Any]],
    destination: Optional[Dict[str, Any]],
    heading_deg: Optional[float],
    speed_value: Optional[float],
    turn_radius_m: float = _DEFAULT_TURN_RADIUS_M,
    default_cruise_speed_mps: float = _DEFAULT_CRUISE_SPEED_MPS,
) -> int:
    origin_coord = _normalize_coordinate(origin)
    dest_coord = _normalize_coordinate(destination)
    if origin_coord is None or dest_coord is None:
        return 0
    cruise_speed_mps = _to_mps(speed_value) or max(float(default_cruise_speed_mps), 1.0)
    bearing_deg = _bearing_between(origin_coord, dest_coord)
    turn_deg = abs(_wrap_delta_deg(float(bearing_deg) - float(heading_deg or bearing_deg)))
    turn_len_m = math.radians(turn_deg) * max(float(turn_radius_m), 1.0)
    cruise_len_m = _haversine_m(
        float(origin_coord["latitude"]),
        float(origin_coord["longitude"]),
        float(dest_coord["latitude"]),
        float(dest_coord["longitude"]),
    )
    return int(round((turn_len_m + cruise_len_m) / max(float(cruise_speed_mps), 1.0)))


def _update_plan_aircraft_entry(plan_data: Dict[str, Any], aircraft_id: int, new_package_id: int) -> bool:
    for entry in plan_data.get("aircraftList") or []:
        if _to_int(entry.get("aircraftID")) == int(aircraft_id):
            entry["individualMissionPackageID"] = int(new_package_id)
            return True
    return False


def _resolve_requested_plan_id(ctx: Dict[str, Any]) -> Optional[int]:
    for value in ctx.get("plan_ids") or []:
        plan_id = _to_int(value)
        if plan_id is not None and plan_id > 0:
            return int(plan_id)
    fallback = _to_int(ctx.get("missionPlanID") or ctx.get("mission_plan_id"))
    if fallback is not None and fallback > 0:
        return int(fallback)
    return None


def _allocate_fresh_plan_id() -> int:
    try:
        return int(reserve_mission_plan_ids(1)[0])
    except Exception:
        pass
    plan_dir = db_paths.get_db_subpath("MissionPlan")
    try:
        plan_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    used: set[int] = set()
    try:
        for item in plan_dir.glob("*.json"):
            if item.stem.isdigit():
                used.add(int(item.stem))
    except Exception:
        pass
    return int(max(used) + 1) if used else 700_000_001


def _write_log_payload(payload: Dict[str, Any]) -> Path:
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
    path = directory / f"{_LOG_BASENAME}_{timestamp}.json"
    log_messages = payload.setdefault("logMessages", [])
    if isinstance(log_messages, list):
        for fov_adjust_message in pop_runtime_camera_fov_adjustment_logs():
            log_messages.append(str(fov_adjust_message))
    payload["logArtifactMode"] = debug_artifact_mode()
    payload["logArtifactWritten"] = write_debug_json(
        path,
        payload,
        pretty=True,
        ensure_ascii=False,
        skip_if_unchanged=False,
    )
    return path


def _centroid_coordinate(coords: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    valid = [_normalize_coordinate(coord) for coord in coords]
    valid = [coord for coord in valid if coord is not None]
    if not valid:
        return None
    lat = sum(float(coord["latitude"]) for coord in valid) / len(valid)
    lon = sum(float(coord["longitude"]) for coord in valid) / len(valid)
    alt_values = [float(coord.get("altitude") or 0.0) for coord in valid]
    alt = sum(alt_values) / len(alt_values) if alt_values else 0.0
    return {"latitude": lat, "longitude": lon, "altitude": int(round(float(alt)))}


def _normalize_coordinate(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    lat = _to_float(value.get("latitude") or value.get("lat"))
    lon = _to_float(value.get("longitude") or value.get("lon"))
    alt = _to_float(value.get("altitude") or value.get("alt"))
    if lat is None or lon is None:
        return None
    result = {"latitude": float(lat), "longitude": float(lon)}
    if alt is not None:
        result["altitude"] = int(round(float(alt)))
    return result


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _to_mps(value: Optional[float]) -> Optional[float]:
    speed = _to_float(value)
    if speed is None or speed <= 0:
        return None
    return (float(speed) * 1000.0 / 3600.0) if float(speed) > 70.0 else float(speed)


def _now_timestamp_ms() -> int:
    epoch = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.now(timezone.utc) - epoch).total_seconds() * 1000)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    to_rad = math.radians
    dlat = to_rad(lat2 - lat1)
    dlon = to_rad(lon2 - lon1)
    a = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(to_rad(lat1)) * math.cos(to_rad(lat2)) * math.sin(dlon / 2.0) ** 2
    )
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return 6_371_000.0 * c


def _bearing_between(origin: Dict[str, Any], destination: Dict[str, Any]) -> float:
    lat1 = math.radians(float(origin["latitude"]))
    lat2 = math.radians(float(destination["latitude"]))
    dlon = math.radians(float(destination["longitude"]) - float(origin["longitude"]))
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360.0) % 360.0


def _wrap_delta_deg(value: float) -> float:
    return ((float(value) + 180.0) % 360.0) - 180.0
