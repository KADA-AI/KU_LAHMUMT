from __future__ import annotations

import json
import math
from collections import defaultdict
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

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
from modules.mission_planning.pipelines.mission_path_trim import (
    DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
    count_sweep_points_in_waypoints,
    estimate_sweep_buffer_points,
    reassign_unique_waypoint_ids_inplace,
    relink_waypoints,
    sweep_progress_points,
    sweep_cut_points,
    trim_waypoints_by_sweep_points,
)
from modules.mission_planning.pipelines.next_collab_replan_pipeline_impl import (
    prepare_next_collab_input_replacements,
)
from modules.mission_planning.pipelines.prior_mission_pipeline_impl import (
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
    _next_imp_id,
    _prepare_uav_collaborative_resume_replan,
    _reserve_individual_mission_ids,
    _reserve_path_ids,
    _next_waypoint_id,
    _remaining_detail_has_geometry,
    _resolve_plan_artifacts,
    _write_collaborative_remaining_imp_update,
)
from modules.mission_planning.runtime.attack_tracking_state import (
    clear_tracking_assignment,
    list_active_tracking_assignments,
    rebind_tracking_assignments_to_plan,
    resolve_plan_lineage_ids,
)
from modules.mission_planning.runtime.attack_assignment_state import release_manned_used
from modules.mission_planning.runtime.debug_artifacts import debug_artifact_mode, write_debug_json
from modules.mission_planning.runtime.json_io import write_json

LogCallback = Callable[[str], None]

_POST_ATTACK_OPTION_NAME = "공격 후 복귀 재계획"
_DEFAULT_MIN_REMAINING_ETA_S = 120
_DEFAULT_REJOIN_MARGIN_S = 45
_DEFAULT_TURN_RADIUS_M = 180.0
_DEFAULT_CRUISE_SPEED_MPS = 35.0
_DEFAULT_COLLAB_ENTRY_SPEED_MPS = 40.0
_DEFAULT_ACTIVE_PROGRESS_SKIP_PERCENT = 70
_POST_ATTACK_SHORT_RETURN_DEFAULT_M = 2000.0
_POST_ATTACK_COMPLETE_HOLD_SECONDS = 5
_POST_ATTACK_COMPLETE_HOLD_JOIN_BUFFER_S = 5
_POST_ATTACK_COMPLETE_HOLD_RADIUS_M = 180
_POST_ATTACK_COMPLETE_HOLD_SPEED_MPS = 30.0
_FORMATION_FLIGHT_INPUT_MISSION_TYPE = 7
_LOG_BASENAME = "log_post_attack_rejoin"


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


def warm_post_attack_rejoin_pipeline() -> Dict[str, Any]:
    return {
        "attack_tracking_assignments": len(list_active_tracking_assignments()),
        "agent_snapshot_available": bool(agent_status_snapshot.load_agent_status_snapshot()),
    }


def _post_attack_rejoin_enabled() -> bool:
    return bool(get_replan_toggle("post_attack_rejoin", True))


def _post_attack_rejoin_config() -> Dict[str, Any]:
    return dict(get_post_attack_rejoin_settings() or {})


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


def _active_tracking_remains_for_plan_lineage(current_plan_id: Optional[int]) -> bool:
    plan_id = _to_int(current_plan_id)
    if plan_id is None or plan_id <= 0:
        return False
    plan_lineage = resolve_plan_lineage_ids(int(plan_id)) or {int(plan_id)}
    for assignment in list_active_tracking_assignments():
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        attack_plan_id = _to_int(assignment.get("attack_plan_id"))
        if attack_plan_id is not None and int(attack_plan_id) in plan_lineage:
            return True
    return False


def _release_attack_slots_if_tracking_closed(
    *,
    input_package_id: Optional[int],
    current_plan_id: Optional[int],
    emit: LogCallback,
) -> List[int]:
    if input_package_id is None or input_package_id <= 0:
        emit("[ATTACK-SLOT] release skipped: inputMissionPackageID unavailable.")
        return []
    if _active_tracking_remains_for_plan_lineage(current_plan_id):
        emit("[ATTACK-SLOT] release deferred: active target-tracking assignment remains.")
        return []

    released = release_manned_used(int(input_package_id), _attack_manned_ids())
    if released:
        emit(
            "[ATTACK-SLOT] released manned attack slots after closed tracking assignments -> "
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
    now_ms = _now_timestamp_ms()
    log_messages: List[str] = []

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
    }
    config = _post_attack_rejoin_config()

    if trigger != "0402" or trigger_type != "attackClosedDestroyed":
        _emit("ignored: detail is not an attack-close destroyed trigger.")
        return _finish_result(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "not_attack_close_trigger"},
            result_payload=result_payload,
        )
    if not _post_attack_rejoin_enabled():
        _emit("skipped: post-attack rejoin is disabled in monitoring settings.")
        return _finish_result(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "post_attack_rejoin_disabled"},
            result_payload=result_payload,
        )
    if current_plan_id is None or current_plan_id <= 0 or target_id is None or target_id <= 0:
        _emit("skipped: missing currentMissionPlanID/targetID in closure detail.")
        return _finish_result(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "missing_trigger_identifiers"},
            result_payload=result_payload,
        )

    assignments = _match_tracking_assignments(
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
        return _finish_result(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "tracking_assignment_not_found"},
            result_payload=result_payload,
        )

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
        plan_data = json.loads(plan_path.read_text(encoding="utf-8"))
    except Exception as exc:
        _emit(f"failed to load current MissionPlan {current_plan_id}: {exc}")
        return _finish_result(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={"status": "skipped", "reason": "current_plan_load_failed"},
            result_payload=result_payload,
        )
    input_package_id = _extract_input_package_id(ctx, detail, plan_data)

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
        released_ids: Set[int] = set()
        for grouped in grouped_assignments:
            aircraft_id = _to_int(grouped.get("aircraft_id"))
            if aircraft_id is None or aircraft_id <= 0:
                continue
            clear_tracking_assignment(int(aircraft_id))
            released_ids.add(int(aircraft_id))
        if released_ids:
            _emit(
                "Tracking assignments cleared without collaborative replan -> "
                f"aircraft={sorted(released_ids)} (reason={skip_reason or 'rejoin_not_needed'})."
            )
        return sorted(int(aid) for aid in released_ids)

    for current_input_id, group_assignments in assignments_by_input.items():
        evaluation = _evaluate_rejoin_group(
            current_plan_id=int(current_plan_id),
            current_input_id=int(current_input_id),
            group_assignments=group_assignments,
            agent_state_map=agent_state_map,
            config=config,
            emit=_emit,
        )
        returning_lah_updates: List[Dict[str, Any]] = []
        returning_lah_ids = _find_returning_manned_attack_aircraft_ids(
            current_plan_id=int(current_plan_id),
            current_input_id=int(current_input_id),
            target_id=int(target_id),
            plan_data=plan_data,
        )
        for aircraft_id in returning_lah_ids:
            lah_update = _build_post_attack_lah_resume_update(
                source_plan_id=int(current_plan_id),
                current_input_id=int(current_input_id),
                target_id=int(target_id),
                aircraft_id=int(aircraft_id),
                current_state=agent_state_map.get(int(aircraft_id)) or {},
                now_ms=int(now_ms),
                emit=_emit,
                log_prefix="[POSTATTACK][LAH]",
            )
            if not isinstance(lah_update, dict):
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
        if returning_lah_updates:
            evaluation["returning_lah_updates"] = returning_lah_updates
        group_summaries.append(evaluation)
        result_payload["evaluations"].append(evaluation)

        if not bool(evaluation.get("replan_needed")):
            tracking_release_aircraft_ids: Set[int] = set()
            active_only_updated_aircraft_ids: Set[int] = set()
            planning_source_plan_id = _resolve_group_source_plan_id(
                group_assignments,
                fallback_plan_id=int(current_plan_id),
            )
            skip_reason = str(evaluation.get("skip_reason") or "")
            suffix_only_skip_reasons = {
                "active_group_progress_high",
                "remaining_work_too_small",
            }
            allow_active_only_collab = skip_reason not in suffix_only_skip_reasons
            if (
                allow_active_only_collab
                and _has_remaining_snapshot_geometry(int(planning_source_plan_id), int(current_input_id))
            ):
                active_only_collab = _prepare_post_attack_active_only_remaining_update(
                    source_plan_id=int(planning_source_plan_id),
                    current_input_id=int(current_input_id),
                    evaluation=evaluation,
                    group_assignments=[dict(item) for item in group_assignments],
                    agent_state_map=agent_state_map,
                    now_ms=int(now_ms),
                    emit=_emit,
                    log_prefix="[POSTATTACK][ACTIVEONLY]",
                )
                if active_only_collab is not None:
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
                    "reason": f"{skip_reason or 'rejoin_not_needed'}_uses_done_or_suffix_update",
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
            active_completed_updates: List[Dict[str, Any]] = []
            active_completed_failed_aircraft_ids: Set[int] = set()
            active_progress_by_aircraft = evaluation.get("active_progress_by_aircraft")
            active_progress_by_aircraft = (
                active_progress_by_aircraft if isinstance(active_progress_by_aircraft, dict) else {}
            )
            active_done_hold_seconds = _estimate_active_done_hold_seconds(
                current_plan_id=int(current_plan_id),
                current_input_id=int(current_input_id),
                evaluation=evaluation,
                group_assignments=group_assignments,
                agent_state_map=agent_state_map,
                config=config,
            )
            if int(active_done_hold_seconds) > int(_POST_ATTACK_COMPLETE_HOLD_SECONDS):
                _emit(
                    "[POSTATTACK][ACTIVE-DONE] completion hold extended for returning UAV rejoin "
                    f"(hold={int(active_done_hold_seconds)}s)."
                )
            progress_only_active_aircraft_ids: Set[int] = set()
            for aircraft_id in sorted(active_suffix_candidate_ids):
                state = agent_state_map.get(int(aircraft_id)) or {}
                progress_percent = _to_int(
                    active_progress_by_aircraft.get(aircraft_id)
                    if aircraft_id in active_progress_by_aircraft
                    else active_progress_by_aircraft.get(str(aircraft_id))
                )
                path_all_done = _active_current_input_path_all_done(
                    source_plan_id=int(current_plan_id),
                    current_input_id=int(current_input_id),
                    aircraft_id=int(aircraft_id),
                )
                completed_by_on_mission = False
                if progress_percent is not None and int(progress_percent) >= 100:
                    completed_by_on_mission = _active_current_input_on_mission_complete(
                        source_plan_id=int(current_plan_id),
                        current_input_id=int(current_input_id),
                        aircraft_id=int(aircraft_id),
                        state=state,
                    )
                if not path_all_done and not completed_by_on_mission:
                    if progress_percent is not None and int(progress_percent) >= 100:
                        progress_only_active_aircraft_ids.add(int(aircraft_id))
                        _emit(
                            "[POSTATTACK][ACTIVE-DONE] sweep progress reached 100% "
                            "but completion is not confirmed by onMission=2 or all-done waypoints; "
                            "keeping active suffix instead of replacing capture geometry "
                            f"(aircraft={aircraft_id})."
                        )
                    continue
                if completed_by_on_mission:
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] active imaging mission reports onMission=2; "
                        "using completion hold instead of reviving remaining sweep "
                        f"(aircraft={aircraft_id}, "
                        f"progress={progress_percent if progress_percent is not None else 'n/a'}%)."
                    )
                elif path_all_done and (progress_percent is None or int(progress_percent) < 100):
                    _emit(
                        "[POSTATTACK][ACTIVE-DONE] active path waypoints already done; "
                        f"using done/follow-up update (aircraft={aircraft_id}, "
                        f"progress={progress_percent if progress_percent is not None else 'n/a'}%)."
                    )
                update = _build_post_attack_active_done_followup_update(
                    source_plan_id=int(current_plan_id),
                    current_input_id=int(current_input_id),
                    aircraft_id=int(aircraft_id),
                    hold_seconds=int(active_done_hold_seconds),
                    now_ms=int(now_ms),
                    emit=_emit,
                    log_prefix="[POSTATTACK][ACTIVE-DONE]",
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
                active_completed_updates[-1]["completedByOnMission2"] = bool(completed_by_on_mission)
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
                sweep_progress = _load_sweep_progress_safe()
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
                    for path_id in (resume_info.get("pathID"), update.get("donePathID")):
                        normalized_path_id = _to_int(path_id)
                        if normalized_path_id is not None and normalized_path_id > 0:
                            path_ids.add(int(normalized_path_id))
                    try:
                        imp_payload = json.loads(
                            db_paths.get_db_subpath(
                                "IndividualMissionPlan",
                                f"{int(new_imp_id)}.json",
                            ).read_text(encoding="utf-8")
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
                    tracking_release_aircraft_ids.add(int(aircraft_id))
                    clear_tracking_assignment(int(aircraft_id))
                    cleared_aircraft_ids.add(int(aircraft_id))
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
            ]
            released_ids = _clear_group_tracking_assignments(
                remaining_group_assignments,
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
        )
        if collab is None:
            evaluation["replan_needed"] = False
            evaluation["skip_reason"] = "collaborative_replan_unavailable"
            released_ids = _clear_group_tracking_assignments(
                group_assignments,
                skip_reason=str(evaluation.get("skip_reason") or ""),
            )
            cleared_aircraft_ids.update(int(aid) for aid in released_ids)
            continue

        for aircraft_id, imp_id in collab.aircraft_imp_ids.items():
            if _update_plan_aircraft_entry(new_plan_data, int(aircraft_id), int(imp_id)):
                generated_imp_ids.add(int(imp_id))
                updated_aircraft_ids.add(int(aircraft_id))
        generated_path_ids.update(int(path_id) for path_id in collab.generated_path_ids)
        for assignment in group_assignments:
            aircraft_id = _to_int(assignment.get("aircraft_id"))
            if aircraft_id is None or int(aircraft_id) not in collab.aircraft_imp_ids:
                continue
            clear_tracking_assignment(int(aircraft_id))
            cleared_aircraft_ids.add(int(aircraft_id))

    released_attack_manned_ids: List[int] = []
    if cleared_aircraft_ids:
        released_attack_manned_ids = _release_attack_slots_if_tracking_closed(
            input_package_id=input_package_id,
            current_plan_id=current_plan_id,
            emit=_emit,
        )

    result_payload["groupSummaries"] = group_summaries
    result_payload["updatedAircraftIDs"] = sorted(updated_aircraft_ids)
    result_payload["clearedTrackingAircraftIDs"] = sorted(cleared_aircraft_ids)
    result_payload["reboundTrackingAircraftIDs"] = sorted(rebound_aircraft_ids)
    result_payload["releasedAttackMannedAircraftIDs"] = list(released_attack_manned_ids)

    if not updated_aircraft_ids:
        if cleared_aircraft_ids:
            _emit(
                "skipped: collaborative rejoin update was unnecessary; "
                "kept current plan resume chain and cleared closed tracking assignments."
            )
        else:
            _emit("skipped: no post-attack collaborative rejoin update was necessary.")
        return _finish_result(
            requested_plan_id=requested_plan_id,
            status="skipped",
            summary={
                "status": "skipped",
                "reason": "rejoin_not_needed",
                "group_evaluations": group_summaries,
                "current_plan_id": int(current_plan_id),
                "cleared_tracking_aircraft_ids": sorted(cleared_aircraft_ids),
                "rebound_tracking_aircraft_ids": sorted(rebound_aircraft_ids),
                "released_attack_manned_aircraft_ids": list(released_attack_manned_ids),
            },
            result_payload=result_payload,
        )

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{int(new_plan_id)}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    write_json(plan_dest, new_plan_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    carried_snapshot = mission_area_replan_store.carry_forward_snapshot(
        int(current_plan_id),
        int(new_plan_id),
        reason="post_attack_rejoin",
    )
    if carried_snapshot is not None:
        _emit(
            "carried area remaining snapshot -> "
            f"{carried_snapshot.name} (sourcePlan={current_plan_id}, plan={new_plan_id})."
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
    }
    result_payload["result"] = summary
    return _finish_result(
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
    for waypoint in copied:
        waypoint["isDone"] = False
    if copied:
        reassign_unique_waypoint_ids_inplace(copied)

    payload["lahWaypointList"] = copied
    if "waypointList" in payload:
        payload["waypointList"] = deepcopy(copied)
    sanitize_flight_path_payload_filming_altitudes(payload)
    return payload


def _find_returning_manned_attack_aircraft_ids(
    *,
    current_plan_id: int,
    current_input_id: int,
    target_id: int,
    plan_data: Dict[str, Any],
) -> List[int]:
    matched_ids: List[int] = []
    for entry in plan_data.get("aircraftList") or []:
        aircraft_id = _to_int((entry or {}).get("aircraftID"))
        if aircraft_id is None or aircraft_id <= 0 or aircraft_id > 3:
            continue
        imp_data = _load_imp_package_for_aircraft(
            source_plan_id=int(current_plan_id),
            aircraft_id=int(aircraft_id),
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
) -> Optional[Dict[str, Any]]:
    from modules.mission_planning.pipelines.attack_plan_pipeline import (
        _lah_waypoints_to_coordinate_list,
        _prepend_lah_transition_waypoint,
        _predict_lah_followup_anchor,
        _split_done_resume_lah_path,
        _trim_lah_waypoints_before_anchor,
    )

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

    attack_target_indices = [
        idx
        for idx, mission in enumerate(mission_list)
        if isinstance(mission, dict)
        and _extract_related_input_mission_id(mission) == int(current_input_id)
        and _mission_target_id(mission) == int(target_id)
    ]
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
    if current_idx is None:
        start_idx = int(attack_last_idx + 1)
    elif current_idx in attack_target_indices:
        start_idx = int(current_idx + 1)
    else:
        start_idx = int(current_idx)

    current_coord = (
        _normalize_coordinate((current_state or {}).get("coordinate"))
        or _normalize_coordinate(((current_state or {}).get("mannedInfo") or {}).get("coordinate"))
        or _normalize_coordinate(((current_state or {}).get("unmannedInfo") or {}).get("coordinate"))
    )
    replacement_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    generated_path_ids: Set[int] = set()
    removed_wp_id: Optional[int] = None
    first_kept_index: Optional[int] = None

    for candidate_idx in range(max(0, int(start_idx)), len(mission_list)):
        source_mission = mission_list[candidate_idx]
        if not isinstance(source_mission, dict):
            continue
        source_path_id = _to_int(source_mission.get("pathID"))
        if source_path_id is None or source_path_id <= 0:
            continue
        try:
            source_path = json.loads(
                db_paths.get_db_subpath("FlightPath", f"{int(source_path_id)}.json").read_text(encoding="utf-8")
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
            _, resume_waypoints, removed_wp_id = _split_done_resume_lah_path(
                source_path,
                artifacts=artifacts,
                current_coord=current_coord,
                emit=emit,
                force_nonempty_resume=False,
                exclude_current_from_resume=False,
            )
            if not resume_waypoints:
                continue

        if current_coord is not None:
            trim_anchor = _predict_lah_followup_anchor(
                current_coord,
                current_state or {},
                enable_prediction=True,
            )
            if _extract_related_input_mission_id(source_mission) == int(current_input_id):
                resume_waypoints, _ = _trim_lah_waypoints_before_anchor(
                    resume_waypoints,
                    trim_anchor,
                    emit=emit,
                    log_prefix=log_prefix,
                    aircraft_id=int(aircraft_id),
                    path_id=int(source_path_id),
                )
            template_wp = deepcopy((resume_waypoints or [None])[0]) if resume_waypoints else {}
            resume_waypoints = _prepend_lah_transition_waypoint(
                resume_waypoints,
                template_wp=template_wp,
                anchor_coord=trim_anchor or current_coord,
            )
        if not resume_waypoints:
            continue

        [individual_id] = _reserve_individual_mission_ids(1)
        [path_id] = _reserve_path_ids(int(aircraft_id), 1)
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
        )
        _apply_runtime_flyover_to_flight_path_payload(path_payload)
        sanitize_flight_path_payload_filming_altitudes(path_payload)
        path_dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        path_dest.parent.mkdir(parents=True, exist_ok=True)
        write_json(path_dest, path_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        generated_path_ids.add(int(path_id))
        first_kept_index = int(candidate_idx)
        break

    clone_start_idx = int(first_kept_index + 1) if first_kept_index is not None else max(0, int(start_idx))
    cloned_artifacts = _clone_follow_up_replan_artifacts(
        missions=mission_list[clone_start_idx:],
        aircraft_id=int(aircraft_id),
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
    )
    if cloned_artifacts is None:
        return None
    follow_up_missions, follow_up_paths = cloned_artifacts
    replacement_missions.extend(follow_up_missions)
    generated_path_ids.update(
        int(_to_int((payload or {}).get("pathID")) or 0)
        for _, payload in follow_up_paths
        if _to_int((payload or {}).get("pathID")) is not None
    )

    if not replacement_missions:
        emit(
            f"{log_prefix} no remaining missions left after dropping closed attack branch "
            f"(aircraft={aircraft_id}, targetID={target_id})."
        )
        return None

    new_imp_data = deepcopy(imp_data)
    new_imp_id = int(_next_imp_id())
    new_imp_data["individualMissionPackageID"] = int(new_imp_id)
    new_imp_data["timestamp"] = int(now_ms)
    new_imp_data["individualMissionList"] = [deepcopy(mission) for mission in replacement_missions]
    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(new_imp_id)}.json")
    imp_dest.parent.mkdir(parents=True, exist_ok=True)
    write_json(imp_dest, new_imp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)

    for dest, payload in follow_up_paths:
        dest.parent.mkdir(parents=True, exist_ok=True)
        sanitize_flight_path_payload_filming_altitudes(payload)
        write_json(dest, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)

    emit(
        f"{log_prefix} returning LAH resume package written "
        f"(aircraft={aircraft_id}, imp={imp_dest.name}, missions={len(replacement_missions)})."
    )
    return {
        "aircraft_id": int(aircraft_id),
        "individualMissionPackageID": int(new_imp_id),
        "generatedPathIDs": sorted(int(pid) for pid in generated_path_ids if int(pid) > 0),
        "removedWaypointID": removed_wp_id,
        "followUpMissionCount": len(follow_up_missions),
    }


def _evaluate_rejoin_group(
    *,
    current_plan_id: int,
    current_input_id: int,
    group_assignments: List[Dict[str, Any]],
    agent_state_map: Dict[int, Dict[str, Any]],
    config: Dict[str, Any],
    emit: LogCallback,
) -> Dict[str, Any]:
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
    if not active_aircraft_ids:
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "active_aircraft_missing",
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "returning_aircraft_ids": returning_aircraft_ids,
        }
    progress_summary = _summarize_active_group_progress(
        current_input_id=int(current_input_id),
        active_aircraft_ids=active_aircraft_ids,
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
    if (
        active_progress_sample_count > 0
        and active_avg_progress_percent is not None
        and float(active_avg_progress_percent) >= float(active_progress_skip_percent)
    ):
        emit(
            f"[POSTATTACK] inputMissionID={current_input_id} rejoin skipped: "
            f"active UAV avg progress {float(active_avg_progress_percent):.1f}% >= "
            f"{int(active_progress_skip_percent)}%."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "active_group_progress_high",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            **progress_summary,
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "available_aircraft_ids": sorted(available_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }
    if not _has_remaining_snapshot_geometry(int(current_plan_id), int(current_input_id)):
        emit(
            f"[POSTATTACK] inputMissionID={current_input_id} rejoin skipped: "
            "current remaining snapshot geometry unavailable."
        )
        return {
            "input_mission_id": int(current_input_id),
            "replan_needed": False,
            "skip_reason": "remaining_snapshot_unavailable",
            "active_progress_skip_percent": int(active_progress_skip_percent),
            **progress_summary,
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    reference_coord = _select_rejoin_reference_coordinate(
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
            "ongoing_tracking_aircraft_ids": sorted(ongoing_tracking_aircraft_ids),
            "active_aircraft_ids": active_aircraft_ids,
            "returning_aircraft_ids": returning_aircraft_ids,
        }

    active_remaining_eta_s = _estimate_group_remaining_eta_s(
        source_plan_id=int(current_plan_id),
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
) -> int:
    hold_seconds = int(_POST_ATTACK_COMPLETE_HOLD_SECONDS)
    max_return_eta_s = _to_int(evaluation.get("max_return_eta_s"))
    if max_return_eta_s is None:
        active_aircraft_ids = [
            int(aid)
            for aid in (evaluation.get("active_aircraft_ids") or [])
            if _to_int(aid) is not None and int(aid) > 0
        ]
        returning_aircraft_ids = {
            int(aid)
            for aid in (evaluation.get("returning_aircraft_ids") or [])
            if _to_int(aid) is not None and int(aid) > 0
        }
        if not active_aircraft_ids or not returning_aircraft_ids:
            return hold_seconds
        reference_coord = _normalize_coordinate(evaluation.get("rejoin_reference"))
        if reference_coord is None:
            reference_coord = _select_rejoin_reference_coordinate(
                active_aircraft_ids=active_aircraft_ids,
                agent_state_map=agent_state_map,
                current_plan_id=int(current_plan_id),
                current_input_id=int(current_input_id),
            )
        if reference_coord is None:
            return hold_seconds
        estimated: List[int] = []
        for assignment in group_assignments:
            aircraft_id = _to_int(assignment.get("aircraft_id"))
            if aircraft_id is None or int(aircraft_id) not in returning_aircraft_ids:
                continue
            state = agent_state_map.get(int(aircraft_id)) or {}
            coord = (
                _normalize_coordinate(state.get("coordinate"))
                or _normalize_coordinate(assignment.get("handoff_coordinate"))
                or _normalize_coordinate(assignment.get("last_nonzero_coordinate"))
                or _normalize_coordinate(assignment.get("original_coordinate"))
            )
            if coord is None:
                continue
            estimated.append(
                int(
                    _estimate_turn_aware_eta_s(
                        origin=coord,
                        destination=reference_coord,
                        heading_deg=_to_float(state.get("heading")),
                        speed_value=_to_float(state.get("speed")),
                        turn_radius_m=_to_float(config.get("turn_radius_m")) or _DEFAULT_TURN_RADIUS_M,
                        default_cruise_speed_mps=(
                            _to_float(config.get("default_cruise_speed_mps"))
                            or _DEFAULT_CRUISE_SPEED_MPS
                        ),
                    )
                )
            )
        max_return_eta_s = max(estimated) if estimated else None
    if max_return_eta_s is None or int(max_return_eta_s) <= 0:
        return hold_seconds
    return max(
        hold_seconds,
        int(max_return_eta_s) + int(_POST_ATTACK_COMPLETE_HOLD_JOIN_BUFFER_S),
    )


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
    fp_data = _load_path_payload(getattr(artifacts, "path_id", None))
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
        if coord is not None and lookahead_s > 1e-9:
            predicted_coord = _predict_post_attack_position_along_current_path(
                source_plan_id=int(source_plan_id),
                aircraft_id=int(aid),
                state=state_copy,
                lookahead_s=float(lookahead_s),
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
        predicted_map[int(aid)] = state_copy

    emit(
        f"{log_prefix} Remaining UAV entry lookahead applied: "
        f"{lookahead_s:.1f}s (pathFollow={path_follow_count}, "
        f"linearFallback={linear_fallback_count}, currentOnly={current_only_count})"
    )
    return predicted_map


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
) -> Optional[CollaborativeResumeReplanResult]:
    current_input_mission, next_input_mission = _build_remaining_input_mission_for_collaborative_replan(
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
    )
    if not isinstance(current_input_mission, dict) or bool(current_input_mission.get("isDone")):
        emit(f"{log_prefix} Collaborative replan skipped: current remaining mission unavailable.")
        return None

    current_input_override: Optional[Dict[str, Any]] = None
    entry_coord_map_override: Optional[Dict[int, Dict[str, Any]]] = None
    heading_map_override: Optional[Dict[int, float]] = None
    returning_aircraft_ids = {
        int(aid)
        for aid in (evaluation.get("returning_aircraft_ids") or [])
        if _to_int(aid) is not None and int(aid) > 0
    }
    predicted_agent_state_map = _build_post_attack_collab_agent_state_map(
        agent_state_map={int(aid): dict(state or {}) for aid, state in agent_state_map.items()},
        source_plan_id=int(runtime_plan_id or source_plan_id),
        emit=emit,
        log_prefix=log_prefix,
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
        source_plan_id=int(source_plan_id),
        current_input_id=int(current_input_id),
        unavailable_aircraft_ids={int(aid) for aid in unavailable_aircraft_ids},
        agent_state_map=predicted_agent_state_map,
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
        drop_prefix_missions=True,
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
        replacement_mission_transform=lambda aircraft_id, missions: _sanitize_post_attack_collaborative_replacements(
            aircraft_id=int(aircraft_id),
            current_input_id=int(current_input_id),
            replacement_missions=missions,
        ),
        flight_path_transform=_active_only_flight_path_transform,
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

    copied = [deepcopy(item) for item in waypoints if isinstance(item, dict)]
    if len(copied) <= 1:
        return payload

    keep_idx = 0
    while (
        keep_idx < len(copied) - 1
        and _is_post_attack_collab_entry_prefix_waypoint(copied[keep_idx])
        and _is_post_attack_collab_sweep_waypoint(copied[keep_idx + 1])
    ):
        keep_idx += 1

    if keep_idx <= 0:
        return payload

    removed = copied[:keep_idx]
    trimmed = copied[keep_idx:]
    relink_waypoints(trimmed)
    payload["waypointList"] = trimmed
    if "lahWaypointList" in payload:
        payload["lahWaypointList"] = deepcopy(trimmed)

    try:
        from modules.common.eta import annotate_eta_flight_plan

        annotate_eta_flight_plan(payload, default_speed_mps=40.0, waypoint_list_keys=("waypointList",))
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
    if not waypoints:
        return
    final_eta = 0.0
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        eta_value = _to_float(waypoint.get("eta"))
        if eta_value is not None:
            final_eta = max(final_eta, float(eta_value))
    for idx, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            continue
        if idx >= len(waypoints) - 1:
            waypoint["ecf"] = 1.0
            continue
        eta_value = _to_float(waypoint.get("eta")) or 0.0
        waypoint["ecf"] = (
            0.0
            if final_eta <= 0.0
            else max(0.0, min(1.0, float(eta_value) / float(final_eta)))
        )


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
        reference_speed, reference_distance_m = _estimate_post_attack_collab_first_sweep_search_speed_from_reference(
            waypoint,
            reference_coord,
        )
        base_speed = float(search_speed)
        used_reference_base = False
        if reference_speed is not None and float(reference_speed) > base_speed:
            base_speed = float(reference_speed)
            used_reference_base = True
        boosted_speed = round(base_speed * float(scale), 2)
        line_search["searchSpeed"] = float(boosted_speed)
        filming["lineSearch"] = line_search
        waypoint["filmingProperty"] = filming
        payload["waypointList"] = waypoints
        if "lahWaypointList" in payload:
            payload["lahWaypointList"] = deepcopy(waypoints)
        try:
            from modules.common.eta import annotate_eta_flight_plan

            annotate_eta_flight_plan(payload, default_speed_mps=40.0, waypoint_list_keys=("waypointList",))
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
    transit_time_s = float(transit_distance_m) / float(transit_speed_mps)
    if transit_time_s <= 1e-6:
        return None, None
    search_speed_weight = get_runtime_float("search_speed_weight", 1.1)
    try:
        search_speed_weight = max(0.1, float(search_speed_weight))
    except Exception:
        search_speed_weight = 1.1
    return float(sweep_distance_m) / float(transit_time_s) * float(search_speed_weight), float(transit_distance_m)


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
) -> Optional[CollaborativeResumeReplanResult]:
    join_delay_s = max(0, _to_int(evaluation.get("max_return_eta_s")) or 0)
    if join_delay_s <= 0:
        return None

    sweep_progress = _load_sweep_progress_safe()
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
    prepared = prepare_next_collab_input_replacements(
        source_plan_id=int(source_plan_id),
        target_input_mission=deepcopy(future_remaining_mission),
        entry_coord_map={int(aid): dict(coord) for aid, coord in future_entry_map.items()},
        heading_map={int(aid): float(val) for aid, val in future_heading_map.items()},
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
        path_dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        path_dest.parent.mkdir(parents=True, exist_ok=True)
        write_json(path_dest, path_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        generated_fp_by_path[int(path_id)] = deepcopy(path_payload)
        generated_path_ids.add(int(path_id))
        finish_eta_s = max(int(finish_eta_s), int(_estimate_uav_flight_path_final_eta_s(path_payload)))

    for aircraft_id in sorted(int(aid) for aid in (prepared.replacement_by_aircraft or {}).keys()):
        replacements = [
            _sanitize_post_attack_mission_entry(dict(item), current_input_id=int(current_input_id))
            for item in (prepared.replacement_by_aircraft.get(int(aircraft_id)) or [])
            if isinstance(item, dict)
        ]
        if not replacements:
            continue

        phased_replacements: List[Dict[str, Any]] = []
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
                    path_dest.parent.mkdir(parents=True, exist_ok=True)
                    _apply_runtime_flyover_to_flight_path_payload(path_payload)
                    sanitize_flight_path_payload_filming_altitudes(path_payload)
                    write_json(path_dest, path_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
                    generated_path_ids.add(int(path_id))
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
            )
            if transit is not None:
                mission_entry, path_payload = transit
                phased_replacements.append(mission_entry)
                path_id = _to_int(path_payload.get("pathID"))
                if path_id is not None:
                    path_dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
                    path_dest.parent.mkdir(parents=True, exist_ok=True)
                    _apply_runtime_flyover_to_flight_path_payload(path_payload)
                    sanitize_flight_path_payload_filming_altitudes(path_payload)
                    write_json(path_dest, path_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
                    generated_path_ids.add(int(path_id))

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
        )
        if new_imp_id is None:
            continue
        aircraft_imp_ids[int(aircraft_id)] = int(new_imp_id)
        replacement_aircraft_ids.add(int(aircraft_id))

    if not aircraft_imp_ids:
        emit(f"{log_prefix} phased line rejoin skipped: no IMP updates were written.")
        return None

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
    aircraft_ids: List[int],
    agent_state_map: Dict[int, Dict[str, Any]],
    emit: LogCallback,
) -> int:
    max_eta_s = 0
    for aircraft_id in aircraft_ids:
        eta_s = _estimate_aircraft_remaining_eta_s(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
            state=dict(agent_state_map.get(int(aircraft_id)) or {}),
            emit=emit,
        )
        max_eta_s = max(int(max_eta_s), int(eta_s))
    return int(max_eta_s)


def _estimate_aircraft_remaining_eta_s(
    *,
    source_plan_id: int,
    aircraft_id: int,
    state: Dict[str, Any],
    emit: LogCallback,
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
        emit(f"[POSTATTACK] aircraft={aircraft_id} remaining ETA fallback -> plan artifacts unavailable.")
        return 0
    try:
        fp_path = db_paths.get_db_subpath("FlightPath", f"{int(artifacts.path_id)}.json")
        fp_data = json.loads(fp_path.read_text(encoding="utf-8"))
    except Exception:
        return 0

    waypoints = deepcopy(fp_data.get("waypointList") or [])
    if not isinstance(waypoints, list) or not waypoints:
        return 0

    start_idx = _find_current_waypoint_index(waypoints, current_waypoint_id)
    if start_idx is None:
        start_idx = _first_not_done_waypoint_index(waypoints)
    if start_idx is None:
        start_idx = 0

    done_prefix = deepcopy(waypoints[:start_idx]) if start_idx > 0 else []
    resume_waypoints = deepcopy(waypoints[start_idx:])
    if not resume_waypoints:
        return 0

    progress_entry = None
    try:
        from modules.mission_planning.pipelines.mission_path_trim import load_sweep_progress

        sweep_progress = load_sweep_progress()
        progress_entry = sweep_progress.get(int(artifacts.path_id)) if isinstance(sweep_progress, dict) else None
    except Exception:
        progress_entry = None

    if isinstance(progress_entry, dict):
        raw_cut_points = sweep_cut_points(
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
    if not resume_waypoints:
        return 0
    return int(_estimate_uav_flight_path_final_eta_s({"waypointList": resume_waypoints}))


def _is_line_rejoin_target(mission: Dict[str, Any]) -> bool:
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []
    return bool(line_list) or (not area_list and len(coord_list) >= 2)


def _load_sweep_progress_safe() -> Dict[int, Dict[str, Any]]:
    try:
        from modules.mission_planning.pipelines.mission_path_trim import load_sweep_progress

        payload = load_sweep_progress()
        if isinstance(payload, dict):
            return {int(pid): dict(entry or {}) for pid, entry in payload.items()}
    except Exception:
        pass
    return {}


def _load_coverage_progress_safe() -> Dict[str, Any]:
    try:
        path = db_paths.get_db_subpath("DSS_Internal", "coverage_progress.json")
        payload = json.loads(path.read_text(encoding="utf-8"))
        if isinstance(payload, dict):
            return payload
    except Exception:
        pass
    return {}


def _summarize_active_group_progress(
    *,
    current_input_id: int,
    active_aircraft_ids: List[int],
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
        }

    coverage_payload = _load_coverage_progress_safe()
    missions = coverage_payload.get("missions") if isinstance(coverage_payload.get("missions"), list) else []
    progress_by_aircraft: Dict[int, int] = {}
    for entry in missions:
        if not isinstance(entry, dict):
            continue
        if _to_int(entry.get("input_id")) != int(current_input_id):
            continue
        aircraft_id = _to_int(entry.get("aircraft_id"))
        if aircraft_id is None or int(aircraft_id) not in active_id_set:
            continue
        percent = _to_int(entry.get("coverage_percent"))
        if percent is None and bool(entry.get("coverage_enabled")):
            percent = _to_int(entry.get("progress_percent"))
        if percent is None:
            continue
        progress_by_aircraft[int(aircraft_id)] = max(
            int(progress_by_aircraft.get(int(aircraft_id), 0)),
            max(0, min(100, int(percent))),
        )

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

    imp_data = _load_imp_package_for_aircraft(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
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
        path_src = db_paths.get_db_subpath("FlightPath", f"{int(artifacts.path_id)}.json")
        template_path = json.loads(path_src.read_text(encoding="utf-8"))
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
    cut_total = estimate_sweep_buffer_points(sweep_progress_entry, lookahead_s)
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
        "width": float(_line_width_from_template_mission(phased_source.template_mission)),
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
            "width": float(width),
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
    if len(future_centerline) >= 2:
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
    snapshot_info = mission_area_replan_store.load_snapshot_entry(
        int(source_plan_id),
        int(current_input_id),
        allow_latest=True,
    )
    if isinstance(snapshot_info, dict):
        mission = snapshot_info.get("entry")
        if isinstance(mission, dict):
            detail = mission.get("remainingDetail")
            if isinstance(detail, dict):
                return deepcopy(detail)
    return {}


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
            "width": float(width) if width is not None and width > 0.0 else float(width_fallback),
        })
    if rows:
        return rows

    coord_list = [
        coord
        for coord in (_normalize_coordinate(item) for item in (detail.get("coordinateList") or []))
        if coord is not None
    ]
    if len(coord_list) >= 2:
        return [{"coordinateList": deepcopy(coord_list), "width": float(width_fallback)}]
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
            "width": float(_to_float(row.get("width")) or 1.0),
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
            "width": float(template_width) if template_width is not None and template_width > 0.0 else 1.0,
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
        from modules.mission_planning.pipelines.prior_mission_pipeline_impl import _merge_line_remaining_detail

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
) -> Optional[tuple[Dict[str, Any], Dict[str, Any]]]:
    if not phased_source.prefix_waypoints:
        return None
    [individual_id] = _reserve_individual_mission_ids(1)
    [path_id] = _reserve_path_ids(int(phased_source.aircraft_id), 1)
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
) -> Optional[tuple[Dict[str, Any], Dict[str, Any]]]:
    template_imp = _load_imp_package_for_aircraft(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
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
            json.loads(
                db_paths.get_db_subpath("FlightPath", f"{int(template_path_id)}.json").read_text(encoding="utf-8")
            )
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

    [individual_id] = _reserve_individual_mission_ids(1)
    [path_id] = _reserve_path_ids(int(aircraft_id), 1)
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


def _load_path_payload(path_id: Optional[int]) -> Optional[Dict[str, Any]]:
    pid = _to_int(path_id)
    if pid is None or pid <= 0:
        return None
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{int(pid)}.json")
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _active_current_input_path_all_done(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
) -> bool:
    imp_data = _load_imp_package_for_aircraft(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
    )
    if not isinstance(imp_data, dict):
        return False
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        path_payload = _load_path_payload(_to_int(mission.get("pathID")))
        waypoints = (path_payload or {}).get("waypointList")
        if isinstance(waypoints, list) and waypoints and _first_not_done_waypoint_index(waypoints) is None:
            return True
    return False


def _active_current_input_path_ids(
    *,
    source_plan_id: int,
    current_input_id: int,
    aircraft_id: int,
) -> Set[int]:
    imp_data = _load_imp_package_for_aircraft(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
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
) -> bool:
    if _to_int((state or {}).get("on_mission") or (state or {}).get("onMission")) != 2:
        return False
    imp_data = _load_imp_package_for_aircraft(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
    )
    if not isinstance(imp_data, dict):
        return False
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        if _extract_related_input_mission_id(mission) != int(current_input_id):
            continue
        path_payload = _load_path_payload(_to_int(mission.get("pathID")))
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
) -> Optional[Dict[str, Any]]:
    imp_data = _load_imp_package_for_aircraft(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
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
    source_path_id = _to_int(target_mission.get("pathID"))
    template_path = _load_path_payload(source_path_id)
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
    final_orientation_coord = _final_filming_orientation_coordinate(waypoints) or deepcopy(final_flight_coord)
    hold_seconds = max(int(_POST_ATTACK_COMPLETE_HOLD_SECONDS), int(hold_seconds or 0))

    [done_individual_id] = _reserve_individual_mission_ids(1)
    [done_path_id] = _reserve_path_ids(int(aircraft_id), 1)
    new_imp_id = int(_next_imp_id())

    if not marker_wp:
        marker_wp = _build_uav_transit_waypoint(
            coordinate=final_flight_coord,
            speed_mps=float(_POST_ATTACK_COMPLETE_HOLD_SPEED_MPS),
            eta_s=int(hold_seconds),
            orientation_coordinate=final_orientation_coord,
            waypoint_pass_type=2,
        )
    marker_wp["waypointID"] = int(_next_waypoint_id())
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

    # Keep post-attack completed active aircraft parked on the boundary
    # marker. Future collaborative inputs stay in the normal mission list for
    # planning/visualization; the simulator stops execution at this boundary.
    cloned_artifacts = _clone_follow_up_replan_artifacts(
        missions=mission_list[target_idx + 1 :],
        aircraft_id=int(aircraft_id),
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
    )
    if cloned_artifacts is None:
        return None
    follow_up_missions, follow_up_paths = cloned_artifacts

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

    new_imp_data = deepcopy(imp_data)
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
    write_json(imp_dest, new_imp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    _apply_runtime_flyover_to_flight_path_payload(path_payload)
    for waypoint in path_payload.get("waypointList") or []:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
            waypoint["nextWaypointID"] = 0
            waypoint["postAttackBoundaryHold"] = True
    sanitize_flight_path_payload_filming_altitudes(path_payload)
    write_json(path_dest, path_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)

    generated_path_ids: Set[int] = {int(done_path_id)}
    for dest, payload in follow_up_paths:
        dest.parent.mkdir(parents=True, exist_ok=True)
        _apply_runtime_flyover_to_flight_path_payload(payload)
        sanitize_flight_path_payload_filming_altitudes(payload)
        write_json(dest, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        path_id = _to_int((payload or {}).get("pathID"))
        if path_id is not None and path_id > 0:
            generated_path_ids.add(int(path_id))

    emit(
        f"{log_prefix} completed current input preserved as active boundary marker "
        f"(aircraft={aircraft_id}, inputMissionID={current_input_id}, "
        f"imp={imp_dest.name}, donePath={path_dest.name}, "
        f"hold={int(hold_seconds)}s, followUps={len(follow_up_missions)})."
    )
    return {
        "aircraft_id": int(aircraft_id),
        "individualMissionPackageID": int(new_imp_id),
        "doneIndividualMissionID": int(done_individual_id),
        "donePathID": int(done_path_id),
        "generatedPathIDs": sorted(int(pid) for pid in generated_path_ids),
        "followUpMissionCount": len(follow_up_missions),
        "holdSeconds": int(hold_seconds),
        "finalCoordinate": deepcopy(final_flight_coord),
        "orientationCoordinate": deepcopy(final_orientation_coord),
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
) -> Optional[Dict[str, Any]]:
    aircraft_id = _to_int(assignment.get("aircraft_id"))
    if aircraft_id is None or aircraft_id <= 0:
        return None

    imp_data = _load_imp_package_for_aircraft(
        source_plan_id=int(attack_plan_id),
        aircraft_id=int(aircraft_id),
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

    resume_path_payload = _load_path_payload(assignment.get("resume_path_id"))
    original_path_payload = _load_path_payload(assignment.get("original_path_id"))
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

    follow_up_start_idx = max(
        idx
        for idx in (tracking_index, resume_index)
        if idx is not None
    ) + 1 if (tracking_index is not None or resume_index is not None) else 0
    first_follow_up_path_payload = None
    if 0 <= int(follow_up_start_idx) < len(mission_list):
        first_follow_up_mission = mission_list[int(follow_up_start_idx)]
        if isinstance(first_follow_up_mission, dict):
            first_follow_up_path_payload = _load_path_payload(first_follow_up_mission.get("pathID"))
    first_follow_up_coord = _first_path_coordinate(
        (first_follow_up_path_payload or {}).get("waypointList") or []
    )
    resume_mission_done = False
    if resume_index is not None and 0 <= int(resume_index) < len(mission_list):
        resume_mission = mission_list[int(resume_index)]
        resume_mission_done = isinstance(resume_mission, dict) and bool(resume_mission.get("isDone"))
    resume_path_done = bool(
        resume_path_payload
        and _first_not_done_waypoint_index((resume_path_payload or {}).get("waypointList") or []) is None
    )
    use_follow_up_return_target = bool(
        (resume_mission_done or resume_path_done) and first_follow_up_coord is not None
    )
    final_coord = (
        first_follow_up_coord
        if use_follow_up_return_target
        else (
            _extract_final_uav_coordinate(resume_path_payload or {})
            or _extract_final_uav_coordinate(original_path_payload or {})
        )
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

    _apply_post_attack_terminal_hold(
        return_waypoints,
        hold_seconds=max(int(_POST_ATTACK_COMPLETE_HOLD_SECONDS), int(hold_seconds or 0)),
        orientation_coordinate=final_coord,
    )

    [new_individual_id] = _reserve_individual_mission_ids(1)
    [new_path_id] = _reserve_path_ids(int(aircraft_id), 1)

    return_mission = _sanitize_post_attack_mission_entry(
        deepcopy(template_mission),
        current_input_id=int(current_input_id),
    )
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
    return_mission["postAttackBoundaryHold"] = True
    return_path_payload["postAttackBoundaryHold"] = True
    for waypoint in return_path_payload.get("waypointList") or []:
        if isinstance(waypoint, dict):
            waypoint["postAttackBoundaryHold"] = True

    cloned_artifacts = _clone_follow_up_replan_artifacts(
        missions=mission_list[follow_up_start_idx:],
        aircraft_id=int(aircraft_id),
        now_ms=int(now_ms),
        emit=emit,
        log_prefix=log_prefix,
    )
    if cloned_artifacts is None:
        return None
    follow_up_missions, follow_up_paths = cloned_artifacts

    new_imp_data = deepcopy(imp_data)
    new_imp_id = int(_next_imp_id())
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
    write_json(imp_dest, new_imp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    sanitize_flight_path_payload_filming_altitudes(return_path_payload)
    write_json(path_dest, return_path_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    generated_path_ids: Set[int] = {int(new_path_id)}
    for dest, payload in follow_up_paths:
        dest.parent.mkdir(parents=True, exist_ok=True)
        _apply_runtime_flyover_to_flight_path_payload(payload)
        sanitize_flight_path_payload_filming_altitudes(payload)
        write_json(dest, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        path_id = _to_int((payload or {}).get("pathID"))
        if path_id is not None and path_id > 0:
            generated_path_ids.add(int(path_id))

    emit(
        f"{log_prefix} tracking branch replaced with return-only package "
        f"(aircraft={aircraft_id}, imp={imp_dest.name}, path={path_dest.name}, "
        f"hold={int(max(int(_POST_ATTACK_COMPLETE_HOLD_SECONDS), int(hold_seconds or 0)))}s, "
        f"followUps={len(follow_up_missions)}, "
        f"speed={float(return_speed_mps):.1f})."
    )
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
    hold_seconds = max(int(_POST_ATTACK_COMPLETE_HOLD_SECONDS), int(hold_seconds or 0))
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


def _first_path_coordinate(waypoints: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    coords = _collect_path_coordinates(waypoints)
    return dict(coords[0]) if coords else None


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


def _has_remaining_snapshot_geometry(source_plan_id: int, current_input_id: int) -> bool:
    snapshot_info = mission_area_replan_store.load_snapshot_entry(
        int(source_plan_id),
        int(current_input_id),
        allow_latest=True,
    )
    if isinstance(snapshot_info, dict):
        item = snapshot_info.get("entry")
        if isinstance(item, dict):
            return bool(_remaining_detail_has_geometry(item.get("remainingDetail")))
    return False


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
        heading = _to_float(velocity.get("heading"))
        if heading is not None:
            heading = heading % 360.0
        speed = _to_float(velocity.get("speed"))
        unmanned = entry.get("unmannedInfo") if isinstance(entry.get("unmannedInfo"), dict) else {}
        sensor_info = unmanned.get("sensorInfo") if isinstance(unmanned.get("sensorInfo"), dict) else {}
        sensor_center_coord = _normalize_coordinate(sensor_info.get("centerCoordinate"))
        on_mission = _to_int(unmanned.get("onMission") or entry.get("onMission"))
        index[int(aircraft_id)] = {
            "coordinate": coord,
            "sensor_center_coordinate": sensor_center_coord,
            "current_waypoint_id": current_wp,
            "heading": heading,
            "speed": speed,
            "on_mission": on_mission,
        }
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
