from __future__ import annotations

import json
from copy import deepcopy
import importlib.util
import math
from collections import OrderedDict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple
import sys
from types import ModuleType

from modules.common import agent_status_snapshot, db_paths
from modules.mission_planning._paths import mission_planner_root, mission_planning_root, project_root
from modules.mission_planning.runtime.json_io import write_json
from modules.mission_planning.runtime.attack_assignment_state import (
    get_last_assigned_manned_id,
    get_used_manned_ids,
    set_pending_manned_assignment,
    set_last_assigned_manned_id,
)
from modules.mission_planning.runtime.attack_tracking_state import (
    clear_tracking_assignment,
    get_tracking_assignment,
    register_tracking_assignment,
)

_ATTACK_ROOT = mission_planning_root()
_MP_DIR = mission_planner_root()
_PROJECT_ROOT = project_root()
_RECEIVE_DB_MOD: Optional[ModuleType] = None
for _candidate in (_PROJECT_ROOT, _ATTACK_ROOT, _MP_DIR):
    _candidate_str = str(_candidate)
    if _candidate.exists() and _candidate_str not in sys.path:
        sys.path.insert(0, _candidate_str)

from modules.mission_planning.pipelines.prior_mission_pipeline_impl import (
    _build_other_uav_resume_package,
    _clone_follow_up_replan_artifacts,
    _load_done_input_ids_for_plan,
    _load_latest_mission_progress_plan_id,
    _normalize_altitude_value,
    _project_coordinate,
    _resolve_plan_artifacts,
    _scan_latest_source_plan_id,
    _bearing_between,
    _next_imp_id,
    _next_waypoint_id,
    _reserve_individual_mission_ids,
    _reserve_path_ids,
    warm_prior_mission_pipeline,
)
from modules.mission_planning.pipelines.mission_path_trim import (
    count_sweep_points_in_waypoints,
    load_sweep_progress,
    reassign_unique_waypoint_ids_inplace,
    relink_waypoints,
    scale_line_search_speed,
    sweep_cut_points,
    trim_waypoints_by_sweep_points,
)

LogCallback = Callable[[str], None]

LOG_FILENAME = "log_attack_algorithm.json"
ATTACK_ENTRY_OFFSET_METERS = 100.0
ATTACK_RESUME_OFFSET_METERS = 20.0
ATTACK_WEAPON_TYPE = 2
LAH_HOLD_SECONDS = 50
LAH_HOLD_NEAR_RESUME_OFFSET_METERS = 30.0
ATTACK_MANNED_CANDIDATES = (2, 3)
_MISSION_PLAN_START = 700_000_001
_RESUME_SEARCH_SPEED_SCALE = 1.3
_ATTACK_FAST_NUM_ARC_RAYS = 360
_ATTACK_POINT_CACHE_MAX = 16
_ATTACK_POINT_CACHE: "OrderedDict[Tuple[float, ...], Dict[str, Any]]" = OrderedDict()


def warm_attack_plan_pipeline() -> Dict[str, Any]:
    """Preload lazy dependencies used by the attack replan path."""
    status: Dict[str, Any] = {"prior_pipeline": warm_prior_mission_pipeline()}
    try:
        from modules.mission_planning.MissionPlanner.data_def import (
            lah_attack_assistance as attack_assist,
        )
    except BaseException as exc:
        status["lah_attack_assistance_loaded"] = False
        status["lah_attack_assistance_error"] = str(exc)
        return status

    status["lah_attack_assistance_loaded"] = attack_assist is not None
    status["compute_attack_point_available"] = callable(
        getattr(attack_assist, "calculate_attack_point", None)
    )
    try:
        raster_paths = attack_assist.detect_raster_paths()
        status["attack_raster_count"] = len(raster_paths)
        gather = getattr(attack_assist, "_gather_raster_infos", None)
        if callable(gather):
            infos = gather(raster_paths)
            status["attack_raster_info_cached"] = len(infos)
    except BaseException as exc:
        status["attack_raster_warm_error"] = str(exc)
    return status


def _allocate_fresh_plan_id() -> int:
    plan_dir = db_paths.get_db_subpath("MissionPlan")
    try:
        plan_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    used: set[int] = set()
    try:
        for item in plan_dir.glob("*.json"):
            stem = item.stem
            if stem.isdigit():
                used.add(int(stem))
    except Exception:
        pass
    if not used:
        return int(_MISSION_PLAN_START)
    return int(max(used) + 1)


def _resolve_attack_exclusion_source_plan_id(
    *,
    source_plan_id: Optional[int],
    aircraft_id: Optional[int],
    current_waypoint_id: Optional[int],
    emit: LogCallback,
) -> Optional[int]:
    base_source_plan_id = _to_int(source_plan_id)
    if aircraft_id is None or current_waypoint_id is None:
        return base_source_plan_id

    candidate_plan_ids: List[int] = []
    seen_plan_ids: set[int] = set()
    for value in (
        base_source_plan_id,
        _load_latest_mission_progress_plan_id(),
        _scan_latest_source_plan_id(),
    ):
        plan_id = _to_int(value)
        if plan_id is None or plan_id in seen_plan_ids:
            continue
        seen_plan_ids.add(plan_id)
        candidate_plan_ids.append(plan_id)

    silent_emit: LogCallback = lambda _message: None
    for candidate_plan_id in candidate_plan_ids:
        artifacts = _resolve_plan_artifacts(
            source_plan_id=candidate_plan_id,
            aircraft_id=aircraft_id,
            current_waypoint_id=current_waypoint_id,
            emit=silent_emit,
            allow_first_mission_fallback=False,
        )
        if artifacts is None:
            continue
        if base_source_plan_id is not None and candidate_plan_id != base_source_plan_id:
            emit(
                f"UAV {aircraft_id} resume source adjusted "
                f"{base_source_plan_id} -> {candidate_plan_id} "
                f"(matched waypoint {current_waypoint_id})."
            )
        return candidate_plan_id

    return base_source_plan_id


def run_attack_plan_pipeline(
    ctx: Dict[str, Any],
    log_callback: Optional[LogCallback] = None,
) -> Dict[str, Any]:
    """
    Execute the specialized attack-planning pre-processing flow.
    Returns a dictionary that is also persisted to DSS_Internal/log_attack_algorithm.json.
    """

    log_messages: List[str] = []
    attack_log: Dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "context": _json_safe(
            {
                "reason": ctx.get("reason"),
                "replan_level": ctx.get("replan_level"),
                "mission_ids": ctx.get("mission_ids"),
                "option_names": ctx.get("option_names"),
                "replan_detail": ctx.get("replan_detail"),
            }
        ),
        "steps": [],
        "logMessages": log_messages,
        "log_text": "",
        "result": {},
    }

    def _emit(message: str) -> None:
        log_messages.append(message)
        attack_log["log_text"] = "\n".join(log_messages)
        if log_callback:
            log_callback(f"[ATTACK] {message}")

    # Step 1) Select the manned aircraft (aircraft 2 or 3) with the greatest fuel.
    input_pkg_id = _to_int(
        ctx.get("inputMissionPackageID")
        or ctx.get("inputMissionPackageId")
        or ctx.get("input_mission_package_id")
    )
    agent_snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    agent_states = agent_snapshot.get("agent_states") or []
    best_aircraft, candidates = _select_preferred_manned_aircraft(
        agent_states,
        input_package_id=input_pkg_id,
    )
    attack_log["result"]["manned_candidates"] = candidates
    attack_log["result"]["selected_aircraft"] = best_aircraft
    if best_aircraft:
        _emit(
            "STEP1 Selected manned aircraft: "
            f"aircraft {best_aircraft['aircraft_id']} "
            f"(fuel={best_aircraft.get('fuel')}, "
            f"coord={_coord_text(best_aircraft.get('coordinate'))})"
        )
        attack_log["steps"].append(
            {
                "name": "select_manned_aircraft",
                "status": "ok",
                "selected": best_aircraft,
                "candidates": candidates,
            }
        )
    else:
        if candidates and input_pkg_id is not None:
            message = (
                "all candidates already used "
                f"(inputMissionPackageID={input_pkg_id})"
            )
            _emit(f"STEP1 Manned-aircraft selection failed: {message}")
        else:
            message = "latest_0401_agent_status.json unavailable"
            _emit(f"STEP1 Manned-aircraft selection failed: {message}")
        attack_log["steps"].append(
            {
                "name": "select_manned_aircraft",
                "status": "error",
                "message": message,
                "candidates": candidates,
            }
        )

    # Step 2) Determine which UAVs are currently tracking targets.
    target_entries, target_error = _load_target_entries()
    attack_log["result"]["target_tracking"] = target_entries
    if target_entries:
        tracking_summary = ", ".join(
            f"watcher {entry.get('watcher_id')}?’target {entry.get('target_id') or entry.get('key')}"
            for entry in target_entries
        )
        _emit(f"STEP2 UAV tracking summary: {tracking_summary or 'no active tracking'}")
        attack_log["steps"].append(
            {
                "name": "analyze_uav_tracking",
                "status": "ok",
                "entries": target_entries,
            }
        )
    else:
        _emit(f"STEP2 UAV tracking info unavailable: {target_error or 'targetInfo.json missing'}")
        attack_log["steps"].append(
            {
                "name": "analyze_uav_tracking",
                "status": "warn",
                "message": target_error or "targetInfo.json missing",
            }
        )

    # Step 3) Attempt to build an attack mission snapshot using lah_attack_assistance.
    friendly_coord = (best_aircraft or {}).get("coordinate")
    detail_override = _build_primary_target_from_detail(ctx.get("replan_detail"), target_entries)
    if detail_override:
        primary_target = detail_override
        _emit(
            "STEP2.5 Using 0402 target override: "
            f"target={primary_target.get('target_id')} watcher={primary_target.get('watcher_id')}"
        )
    else:
        primary_target = _pick_primary_target(target_entries)
    attack_log["result"]["primary_target"] = primary_target
    if not friendly_coord:
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "error",
                "message": "Missing friendly coordinate (cannot derive aircraft position)",
            }
        )
        _emit("STEP3 Attack plan failed: manned aircraft coordinate missing")
        return _persist_attack_log(attack_log)
    if not primary_target or not primary_target.get("coordinate"):
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "warn",
                "message": "No active target with coordinates found.",
            }
        )
        _emit("STEP3 Attack plan deferred: no active target with coordinates")
        return _persist_attack_log(attack_log)

    attack_point, attack_error = _compute_attack_point(
        friendly_coord,
        primary_target["coordinate"],
    )
    attack_log["result"]["attack_point"] = attack_point
    mission_updates: Optional[Dict[str, Any]] = None
    if attack_point:
        altitude_display = (
            f"alt={attack_point['altitude']}m" if attack_point.get("altitude") is not None else "alt=unknown"
        )
        _emit(
            "STEP3 Attack plan completed: "
            f"lat={attack_point['latitude']:.6f}, lon={attack_point['longitude']:.6f}, "
            f"{altitude_display}"
        )
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "ok",
                "attack_point": attack_point,
            }
        )
        mission_updates = _apply_attack_plan_overrides(
            ctx=ctx,
            attack_point=attack_point,
            manned_aircraft=best_aircraft,
            primary_target=primary_target,
            agent_states=agent_states,
            emit=_emit,
        )
        if mission_updates:
            attack_log["result"]["missionUpdates"] = mission_updates
            manned_id = _extract_assigned_manned_id(mission_updates)
            if manned_id is not None:
                set_last_assigned_manned_id(manned_id)
                attack_plan_id = _to_int(mission_updates.get("mission_plan_id"))
                if attack_plan_id is not None:
                    set_pending_manned_assignment(attack_plan_id, input_pkg_id, manned_id)
    else:
        _emit(f"STEP3 Attack plan failed: {attack_error}")
        attack_log["steps"].append(
            {
                "name": "generate_attack_point",
                "status": "error",
                "message": attack_error,
            }
        )

    return _persist_attack_log(attack_log)


def run_attack_exclusion_pipeline(
    ctx: Dict[str, Any],
    log_callback: Optional[LogCallback] = None,
) -> Dict[str, Any]:
    """
    Build an attack-exclusion plan by trimming each UAV mission to its
    current resume portion, using the same logic as the non-selected UAV
    branch of the prior-mission pipeline.
    """

    log_messages: List[str] = []
    result_payload: Dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "context": _json_safe(
            {
                "reason": ctx.get("reason"),
                "replan_level": ctx.get("replan_level"),
                "mission_ids": ctx.get("mission_ids"),
                "option_names": ctx.get("option_names"),
            }
        ),
        "result": {},
        "logMessages": log_messages,
    }

    def _emit(message: str) -> None:
        log_messages.append(message)
        if log_callback:
            log_callback(f"[ATTACK-EXCLUDE] {message}")

    source_plan_id = _to_int(
        ctx.get("sourceMissionPlanID")
        or ctx.get("source_plan_id")
        or ctx.get("currentMissionPlanID")
        or ctx.get("missionPlanID")
    )
    if source_plan_id is None:
        source_plan_id = _load_latest_mission_progress_plan_id() or _scan_latest_source_plan_id()
    if source_plan_id is None:
        _emit("원본 MissionPlan을 찾지 못해 공격 배제 계획을 생성할 수 없습니다.")
        result_payload["result"] = {"error": "source_plan_not_found"}
        return result_payload

    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = json.loads(plan_src.read_text(encoding="utf-8"))
    except Exception as exc:
        _emit(f"원본 MissionPlan {source_plan_id} 로드 실패: {exc}")
        result_payload["result"] = {
            "error": "source_plan_load_failed",
            "sourcePlanID": int(source_plan_id),
        }
        return result_payload

    agent_snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
    agent_states = agent_snapshot.get("agent_states") or []
    agent_index = _index_agent_states(agent_states)
    sweep_progress = load_sweep_progress()

    requested_plan_id = _resolve_requested_plan_id(
        ctx,
        preferred_option_names={"공격 배제"},
    )
    new_plan_id = requested_plan_id or _allocate_fresh_plan_id()
    if requested_plan_id is not None:
        _emit(
            f"ATTACK-EXCLUDE using requested missionPlanID {new_plan_id} "
            f"(sourcePlanID={source_plan_id})"
        )
    else:
        _emit(
            f"ATTACK-EXCLUDE allocated fresh missionPlanID {new_plan_id} "
            f"(sourcePlanID={source_plan_id})"
        )
    now_ms = _now_timestamp_ms()
    new_plan_data = deepcopy(plan_data)
    new_plan_data["missionPlanID"] = new_plan_id
    new_plan_data["timestamp"] = now_ms
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = now_ms

    aircraft_updates: List[Dict[str, Any]] = []
    unchanged_aircraft: List[int] = []
    for entry in new_plan_data.get("aircraftList", []):
        aircraft_id = _to_int(entry.get("aircraftID"))
        if aircraft_id is None:
            continue
        if aircraft_id <= 3:
            unchanged_aircraft.append(aircraft_id)
            continue

        state = agent_index.get(aircraft_id) or {}
        current_wp = _to_int(state.get("current_waypoint_id"))
        current_coord = state.get("coordinate") if isinstance(state, dict) else None
        recovery = _resolve_attack_tracking_recovery(
            aircraft_id=int(aircraft_id),
            source_plan_id=int(source_plan_id),
            current_coord=current_coord,
            emit=_emit,
        )
        if recovery is not None:
            update = _build_other_uav_resume_package(
                source_plan_id=int(recovery["source_plan_id"]),
                aircraft_id=int(aircraft_id),
                current_waypoint_id=_to_int(recovery["split_waypoint_id"]),
                current_coord=recovery.get("done_anchor_coord"),
                emit=_emit,
                now_ms=now_ms,
                sweep_progress=sweep_progress,
                clone_follow_up_artifacts=True,
                allow_first_mission_fallback=False,
            )
            if not update:
                _emit(
                    f"UAV {aircraft_id} tracking recovery resume mission generation failed; "
                    "keeping existing individual mission."
                )
                unchanged_aircraft.append(aircraft_id)
                continue

            clear_tracking_assignment(aircraft_id)
            entry["individualMissionPackageID"] = int(update["individualMissionPackageID"])
            aircraft_updates.append(update)
            continue

        resume_source_hint = int(source_plan_id)
        if current_wp is None:
            inferred_plan_id, inferred_wp = _infer_attack_exclusion_resume_state(
                source_plan_id=int(source_plan_id),
                aircraft_id=int(aircraft_id),
                current_coord=current_coord,
                emit=_emit,
            )
            if inferred_wp is not None:
                current_wp = inferred_wp
                if inferred_plan_id is not None:
                    resume_source_hint = int(inferred_plan_id)
        if current_wp is None:
            _emit(f"UAV {aircraft_id} currentWaypointID가 없어 기존 개별임무를 유지합니다.")
            unchanged_aircraft.append(aircraft_id)
            continue

        resume_source_plan_id = _resolve_attack_exclusion_source_plan_id(
            source_plan_id=int(resume_source_hint),
            aircraft_id=int(aircraft_id),
            current_waypoint_id=current_wp,
            emit=_emit,
        )
        if resume_source_plan_id is None:
            _emit(
                f"UAV {aircraft_id} resume source MissionPlan을 찾지 못해 "
                "기존 개별임무를 유지합니다."
            )
            unchanged_aircraft.append(aircraft_id)
            continue

        update = _build_other_uav_resume_package(
            source_plan_id=int(resume_source_plan_id),
            aircraft_id=int(aircraft_id),
            current_waypoint_id=current_wp,
            current_coord=current_coord,
            emit=_emit,
            now_ms=now_ms,
            sweep_progress=sweep_progress,
            clone_follow_up_artifacts=True,
            allow_first_mission_fallback=False,
        )
        if not update:
            _emit(f"UAV {aircraft_id} resume 임무 생성에 실패하여 기존 개별임무를 유지합니다.")
            unchanged_aircraft.append(aircraft_id)
            continue

        entry["individualMissionPackageID"] = int(update["individualMissionPackageID"])
        aircraft_updates.append(update)
        _clear_attack_tracking_assignment_if_attached_to_plan(
            aircraft_id=int(aircraft_id),
            source_plan_id=int(source_plan_id),
            emit=_emit,
        )

    if not aircraft_updates:
        _emit("공격 배제용 UAV 재개 임무가 생성되지 않았습니다.")
        result_payload["result"] = {
            "error": "no_updates",
            "sourcePlanID": int(source_plan_id),
            "unchangedAircraft": unchanged_aircraft,
        }
        return result_payload

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(plan_dest, new_plan_data)
    _emit(f"공격 배제 MissionPlan 저장 -> {plan_dest.name} (planID={new_plan_id})")

    result_payload["result"] = {
        "sourcePlanID": int(source_plan_id),
        "missionPlanID": int(new_plan_id),
        "planPath": str(plan_dest),
        "missionUpdates": {
            "mode": "attack_exclusion",
            "aircraft": aircraft_updates,
            "unchangedAircraft": unchanged_aircraft,
        },
    }
    return result_payload


def _select_preferred_manned_aircraft(
    agent_states: List[Any],
    *,
    input_package_id: Optional[int] = None,
) -> Tuple[Optional[Dict[str, Any]], List[Dict[str, Any]]]:
    candidates: List[Dict[str, Any]] = []
    for state in agent_states:
        aircraft_id = _to_int(
            (state.get("aircraftID") if isinstance(state, dict) else None)
            or (state.get("aircraftId") if isinstance(state, dict) else None)
        )
        if aircraft_id not in ATTACK_MANNED_CANDIDATES:
            continue
        is_unmanned = _to_bool(state.get("isUnmanned")) if isinstance(state, dict) else None
        if is_unmanned:
            continue
        fuel = _to_float(state.get("fuel")) if isinstance(state, dict) else None
        coordinate = _normalize_coordinate(state.get("coordinate")) if isinstance(state, dict) else None
        candidates.append(
            {
                "aircraft_id": aircraft_id,
                "fuel": fuel,
                "coordinate": coordinate,
            }
        )
    if not candidates:
        return None, candidates
    candidates.sort(
        key=lambda item: (
            item["fuel"] is not None,
            item["fuel"] if item["fuel"] is not None else float("-inf"),
        ),
        reverse=True,
    )
    used = get_used_manned_ids(input_package_id)
    if used:
        unused = [c for c in candidates if c["aircraft_id"] not in used]
        if not unused:
            return None, candidates
        candidates = unused

    last_assigned = get_last_assigned_manned_id()
    if last_assigned is not None:
        for candidate in candidates:
            if candidate["aircraft_id"] != last_assigned:
                return candidate, candidates
    return candidates[0], candidates


def _load_target_entries() -> Tuple[List[Dict[str, Any]], Optional[str]]:
    target_entries: List[Dict[str, Any]] = []
    target_path = db_paths.get_db_subpath("DSS_Internal") / "targetInfo.json"
    if not target_path.exists():
        return [], f"{target_path} does not exist"
    try:
        raw = json.loads(target_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return [], f"Failed to parse targetInfo.json: {exc}"
    target_map = raw.get("targetList") if isinstance(raw, dict) else None
    if not isinstance(target_map, dict):
        return [], "targetInfo.json lacks targetList"
    for key, value in target_map.items():
        entry = value or {}
        target_id = _to_int(entry.get("targetID"))
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
            watcher_id = _extract_watcher_from_key(str(key))
        is_destroyed = bool(entry.get("isDestroyed"))
        target_entries.append(
            {
                "key": str(key),
                "target_id": target_id,
                "watcher_id": watcher_id,
                "coordinate": _normalize_coordinate(entry.get("coordinate")),
                "is_destroyed": is_destroyed,
                "is_used": _to_int(entry.get("isUsed")),
                "target_in_frame": bool(entry.get("targetInFrame")),
                "threat": _to_float(entry.get("threat")),
                "raw": entry,
            }
        )
    target_entries.sort(
        key=lambda item: (
            not item["is_destroyed"],
            item["target_in_frame"],
            item["target_id"] if item["target_id"] is not None else -1,
        ),
        reverse=True,
    )
    return target_entries, None


def _normalize_replan_detail(detail: Any) -> Optional[Dict[str, Any]]:
    if isinstance(detail, dict):
        return detail
    if isinstance(detail, (bytes, bytearray)):
        try:
            detail = detail.decode("utf-8", "ignore")
        except Exception:
            return None
    if isinstance(detail, str):
        try:
            parsed = json.loads(detail)
        except Exception:
            return None
        return parsed if isinstance(parsed, dict) else None
    return None


def _build_primary_target_from_detail(
    detail: Any,
    target_entries: List[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    detail = _normalize_replan_detail(detail)
    if not detail:
        return None
    trigger = detail.get("trigger")
    has_watcher = detail.get("watcherID") is not None or detail.get("watcherId") is not None
    has_target = detail.get("targetID") is not None or detail.get("targetId") is not None
    has_coord = detail.get("coordinate") is not None or detail.get("targetCoordinate") is not None
    if not (isinstance(trigger, str) and trigger.strip() == "0402") and not (has_watcher and (has_target or has_coord)):
        return None
    target_id = _to_int(detail.get("targetID") or detail.get("targetId"))
    watcher_id = _to_int(detail.get("watcherID") or detail.get("watcherId"))
    coord = _normalize_coordinate(detail.get("coordinate") or detail.get("targetCoordinate"))
    if coord is None and target_id is not None:
        for entry in target_entries:
            if entry.get("target_id") == target_id:
                if entry.get("is_destroyed"):
                    return None
                coord = entry.get("coordinate")
                if watcher_id is None:
                    watcher_id = entry.get("watcher_id")
                break
    if target_id is None and coord is None and watcher_id is None:
        return None
    return {
        "key": detail.get("targetKey") or detail.get("targetID"),
        "target_id": target_id,
        "watcher_id": watcher_id,
        "coordinate": coord,
        "is_destroyed": False,
        "is_used": 0,
        "target_in_frame": True,
        "raw": detail,
    }


def _pick_primary_target(target_entries: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    for entry in target_entries:
        if entry["is_destroyed"]:
            continue
        if entry["target_in_frame"] and entry.get("coordinate"):
            return entry
    for entry in target_entries:
        if not entry["is_destroyed"] and entry.get("coordinate"):
            return entry
    return None


def _compute_attack_point(
    friendly_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
    friendly = _coordinate_to_world(friendly_coord)
    enemy = _coordinate_to_world(enemy_coord)
    if not friendly or not enemy:
        return None, "Insufficient coordinate data for attack calculation."
    try:
        from modules.mission_planning.MissionPlanner.data_def import (
            lah_attack_assistance as attack_assist,
        )
    except SystemExit as exc:
        return None, f"GDAL/Shapely dependencies missing: {exc}"
    except Exception as exc:
        return None, f"Failed to import lah_attack_assistance: {exc}"

    cache_key = (
        round(float(friendly[1]), 5),
        round(float(friendly[0]), 5),
        round(float(enemy[1]), 5),
        round(float(enemy[0]), 5),
        float(_ATTACK_FAST_NUM_ARC_RAYS),
    )
    cached = _ATTACK_POINT_CACHE.get(cache_key)
    if isinstance(cached, dict):
        _ATTACK_POINT_CACHE.move_to_end(cache_key)
        return dict(cached), None

    try:
        raster_paths = attack_assist.detect_raster_paths()
        elevation, geotransform, used_rasters = attack_assist.load_elevation(
            raster_paths,
            enemy,
            radius_m=attack_assist.ANALYSIS_RADIUS_METERS,
        )
        enemy_px = attack_assist.ensure_point_inside(enemy, geotransform, elevation)
        num_rays = int(getattr(attack_assist, "NUM_ARC_RAYS", _ATTACK_FAST_NUM_ARC_RAYS) or _ATTACK_FAST_NUM_ARC_RAYS)
        num_rays = max(180, min(num_rays, _ATTACK_FAST_NUM_ARC_RAYS))
        arc = attack_assist.compute_cover_disk(
            elevation,
            geotransform,
            enemy_px,
            radius_m=attack_assist.ANALYSIS_RADIUS_METERS,
            num_rays=num_rays,
        )
        cell_data = attack_assist.compute_cell_data(arc)
        polygons = attack_assist.build_danger_polygons(
            cell_data,
            arc.world_x,
            arc.world_y,
            arc,
            geotransform,
        )
        if not polygons:
            return None, "No LOS polygons generated from terrain data."
        best = attack_assist.choose_attack_point(
            polygons,
            friendly,
            enemy,
            geotransform,
        )
        if not best:
            return None, "Attack candidate selection failed."
        altitude = attack_assist.sample_elevation_at_world(
            elevation,
            best["centroid"],
            geotransform,
        )
        # Use terrain altitude with a fixed safety offset (DEM + 300m)
        altitude_int = _normalize_altitude_value(altitude + 300.0 if altitude is not None else 300.0)
        result = {
            "latitude": best["centroid"][1],
            "longitude": best["centroid"][0],
            "altitude": altitude_int,
            "friendly_distance_m": best["friendly_distance"],
            "enemy_distance_m": best["enemy_distance"],
            "raster_sources": used_rasters,
        }
        _ATTACK_POINT_CACHE[cache_key] = dict(result)
        _ATTACK_POINT_CACHE.move_to_end(cache_key)
        while len(_ATTACK_POINT_CACHE) > _ATTACK_POINT_CACHE_MAX:
            _ATTACK_POINT_CACHE.popitem(last=False)
        return (result, None)
    except Exception as exc:
        return None, f"Attack point computation error: {exc}"


def _persist_attack_log(payload: Dict[str, Any]) -> Dict[str, Any]:
    directory = db_paths.get_db_subpath("DSS_Internal")
    directory.mkdir(parents=True, exist_ok=True)
    timestamp_token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%f")
    target_path = directory / f"log_attack_algorithm_{timestamp_token}.json"
    payload["log_text"] = "\n".join(payload.get("logMessages") or [])
    try:
        write_json(target_path, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
    except Exception as exc:
        if isinstance(payload.setdefault("steps", []), list):
            payload["steps"].append(
                {
                    "name": "persist_log",
                    "status": "error",
                    "message": f"Failed to write {target_path}: {exc}",
                }
        )
        return payload
    payload["log_path"] = str(target_path)
    payload.setdefault("logMessages", []).append(f"[LOG] Saved attack analysis -> {target_path}")
    payload["log_text"] = "\n".join(payload.get("logMessages") or [])
    return payload


def _normalize_coordinate(value: Any) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    lat = _to_float(value.get("latitude") or value.get("lat"))
    lon = _to_float(value.get("longitude") or value.get("lon"))
    alt = _normalize_altitude_value(value.get("altitude") or value.get("alt"))
    if lat is None or lon is None:
        return None
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def _coordinate_to_world(coord: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    if not coord:
        return None
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    return (lon, lat)


def _coord_text(coord: Optional[Dict[str, Any]]) -> str:
    if not coord:
        return "-"
    return f"({coord.get('latitude')}, {coord.get('longitude')}, alt={coord.get('altitude')})"


def _extract_watcher_from_key(key: str) -> Optional[int]:
    if "-" not in key:
        return None
    try:
        _, watcher = key.split("-", 1)
        return int(watcher)
    except Exception:
        return None


def _haversine_distance_m(a: Dict[str, Any], b: Dict[str, Any]) -> Optional[float]:
    lat1 = _to_float(a.get("latitude"))
    lon1 = _to_float(a.get("longitude"))
    lat2 = _to_float(b.get("latitude"))
    lon2 = _to_float(b.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    r = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a_val = math.sin(dphi / 2.0) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a_val), math.sqrt(1.0 - a_val))
    return r * c



def _json_safe(value: Any) -> Any:
    try:
        json.dumps(value, ensure_ascii=False)
        return value
    except TypeError:
        if isinstance(value, dict):
            return {str(k): _json_safe(v) for k, v in value.items()}
        if isinstance(value, (list, tuple, set)):
            return [_json_safe(v) for v in value]
        return str(value)


def _to_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _to_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _normalize_option_name(value: Any) -> str:
    return "".join(str(value or "").split()).lower()


def _resolve_requested_plan_id(
    ctx: Dict[str, Any],
    *,
    preferred_option_names: Optional[set[str]] = None,
) -> Optional[int]:
    plan_ids = list(ctx.get("plan_ids") or [])
    option_names = list(ctx.get("option_names") or [])
    normalized_pref = {
        _normalize_option_name(name)
        for name in (preferred_option_names or set())
        if str(name or "").strip()
    }

    if normalized_pref and option_names:
        for idx, name in enumerate(option_names):
            if _normalize_option_name(name) not in normalized_pref:
                continue
            if idx >= len(plan_ids):
                continue
            plan_id = _to_int(plan_ids[idx])
            if plan_id is not None and plan_id > 0:
                return int(plan_id)

    for value in plan_ids:
        plan_id = _to_int(value)
        if plan_id is not None and plan_id > 0:
            return int(plan_id)

    fallback = _to_int(
        ctx.get("missionPlanID")
        or ctx.get("mission_plan_id")
    )
    if fallback is not None and fallback > 0:
        return int(fallback)
    return None


def _to_bool(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "y", "yes"}:
            return True
        if lowered in {"false", "0", "n", "no"}:
            return False
    return None


def _extract_assigned_manned_id(mission_updates: Dict[str, Any]) -> Optional[int]:
    aircraft_entries = mission_updates.get("aircraft") if isinstance(mission_updates, dict) else None
    if not isinstance(aircraft_entries, list):
        return None
    for entry in aircraft_entries:
        if not isinstance(entry, dict):
            continue
        if str(entry.get("role") or "") != "manned":
            continue
        return _to_int(entry.get("aircraft_id"))
    return None


def _get_received_db():
    global _RECEIVE_DB_MOD
    for module_name in ("receive.database", "modules.common.receive.database", "database"):
        module = sys.modules.get(module_name)
        if module is not None:
            received = getattr(module, "received_db", None)
            if received is not None:
                return received

    if _RECEIVE_DB_MOD is None:
        database_path = _PROJECT_ROOT / "modules" / "common" / "receive" / "database.py"
        spec = importlib.util.spec_from_file_location(
            "ku_receive_database_standalone",
            database_path,
        )
        if spec is None or spec.loader is None:
            return None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _RECEIVE_DB_MOD = module

    return getattr(_RECEIVE_DB_MOD, "received_db", None)


def _normalize_waypoint_id(value: Any) -> Optional[int]:
    waypoint_id = _to_int(value)
    if waypoint_id is not None and waypoint_id <= 0:
        return None
    return waypoint_id


def _safe_get_value(obj: Any, *names: str) -> Any:
    if obj is None:
        return None
    if isinstance(obj, dict):
        for name in names:
            if name in obj:
                return obj.get(name)
    for name in names:
        try:
            if hasattr(obj, name):
                return getattr(obj, name)
        except Exception:
            pass
    return None


def _iter_safe_items(coll: Any):
    if coll is None:
        return
    try:
        for item in coll:
            yield item
    except Exception:
        return


def _load_latest_mission_progress_state(aircraft_id: int) -> Optional[Dict[str, Optional[int]]]:
    received_db = _get_received_db()
    if received_db is None:
        return None
    try:
        raw = received_db.get_received_0501()
    except Exception:
        raw = None
    if raw is None:
        return None

    current_plan_id = _to_int(_safe_get_value(raw, "currentMissionPlanID", "CurrentMissionPlanID"))
    progress_items = _safe_get_value(
        raw,
        "individualMissionProgressStatusList",
        "IndividualMissionProgressStatusList",
    )
    for item in _iter_safe_items(progress_items):
        aid = _to_int(_safe_get_value(item, "aircraftID", "AircraftID"))
        if aid != aircraft_id:
            continue
        mission = _safe_get_value(item, "currentIndividualMission", "CurrentIndividualMission")
        return {
            "currentMissionPlanID": current_plan_id,
            "individualMissionID": _to_int(
                _safe_get_value(mission, "individualMissionID", "IndividualMissionID")
            ),
            "progress": _to_int(
                _safe_get_value(
                    item,
                    "currentIndividualMissionProgress",
                    "CurrentIndividualMissionProgress",
                )
            ),
        }
    return None


def _load_path_waypoints(path_id: Optional[int]) -> List[Dict[str, Any]]:
    pid = _to_int(path_id)
    if pid is None:
        return []
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{pid}.json")
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return []
    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        items = data.get(key)
        if isinstance(items, list):
            return [item for item in items if isinstance(item, dict)]
    return []


def _load_attack_exclusion_plan_context(
    plan_id: Optional[int],
    aircraft_id: int,
) -> Optional[Dict[str, Any]]:
    resolved_plan_id = _to_int(plan_id)
    if resolved_plan_id is None:
        return None
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{resolved_plan_id}.json")
        plan_data = json.loads(plan_path.read_text(encoding="utf-8"))
    except Exception:
        return None

    aircraft_entry = None
    for entry in plan_data.get("aircraftList", []):
        if _to_int((entry or {}).get("aircraftID")) == aircraft_id:
            aircraft_entry = entry
            break
    if not isinstance(aircraft_entry, dict):
        return None

    package_id = _to_int(aircraft_entry.get("individualMissionPackageID"))
    if package_id is None:
        return None

    try:
        imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{package_id}.json")
        imp_data = json.loads(imp_path.read_text(encoding="utf-8"))
    except Exception:
        return None

    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        return None

    return {
        "plan_id": resolved_plan_id,
        "individual_mission_package_id": package_id,
        "individualMissionList": mission_list,
    }


def _infer_attack_exclusion_resume_state(
    *,
    source_plan_id: Optional[int],
    aircraft_id: int,
    current_coord: Optional[Dict[str, Any]],
    emit: LogCallback,
) -> Tuple[Optional[int], Optional[int]]:
    coord_norm = _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None
    progress_state = _load_latest_mission_progress_state(aircraft_id) or {}
    progress_plan_id = _to_int(progress_state.get("currentMissionPlanID"))
    progress_individual_id = _to_int(progress_state.get("individualMissionID"))

    candidate_plan_ids: List[int] = []
    seen_plan_ids: set[int] = set()
    for value in (
        source_plan_id,
        progress_plan_id,
        _load_latest_mission_progress_plan_id(),
        _scan_latest_source_plan_id(),
    ):
        plan_id = _to_int(value)
        if plan_id is None or plan_id in seen_plan_ids:
            continue
        seen_plan_ids.add(plan_id)
        candidate_plan_ids.append(plan_id)

    best_match: Optional[Dict[str, Any]] = None
    for plan_order, candidate_plan_id in enumerate(candidate_plan_ids):
        context = _load_attack_exclusion_plan_context(candidate_plan_id, aircraft_id)
        if context is None:
            continue
        mission_list = context.get("individualMissionList") or []
        for mission_index, mission in enumerate(mission_list):
            if not isinstance(mission, dict):
                continue
            mission_id = _to_int(mission.get("individualMissionID"))
            path_id = _to_int(mission.get("pathID"))
            waypoints = _load_path_waypoints(path_id)
            if mission_id is None or path_id is None or not waypoints:
                continue

            mission_priority = 2
            if progress_individual_id is not None and mission_id == progress_individual_id:
                mission_priority = 0
            elif not bool(mission.get("isDone")):
                mission_priority = 1

            active_idx = next(
                (idx for idx, wp in enumerate(waypoints) if not bool(wp.get("isDone"))),
                0,
            )
            for wp_index, waypoint in enumerate(waypoints):
                waypoint_id = _to_int(waypoint.get("waypointID"))
                if waypoint_id is None:
                    continue
                done_priority = 1 if bool(waypoint.get("isDone")) else 0
                coord = _normalize_coordinate(waypoint.get("coordinate"))
                if coord_norm:
                    distance_m = _haversine_distance_m(coord_norm, coord) if coord else None
                    distance_score = float(distance_m) if isinstance(distance_m, (int, float)) else 1.0e9
                    tie_break = abs(wp_index - active_idx)
                    score = (
                        mission_priority,
                        done_priority,
                        distance_score,
                        plan_order,
                        tie_break,
                        wp_index,
                    )
                else:
                    if wp_index != active_idx and done_priority > 0:
                        continue
                    distance_m = None
                    score = (
                        mission_priority,
                        done_priority,
                        abs(wp_index - active_idx),
                        plan_order,
                        wp_index,
                    )

                candidate = {
                    "score": score,
                    "plan_id": candidate_plan_id,
                    "individual_mission_id": mission_id,
                    "path_id": path_id,
                    "waypoint_id": waypoint_id,
                    "distance_m": distance_m,
                }
                if best_match is None or candidate["score"] < best_match["score"]:
                    best_match = candidate

    if best_match is None:
        return None, None

    distance_m = best_match.get("distance_m")
    distance_text = f", distance~{distance_m:.1f}m" if isinstance(distance_m, (int, float)) else ""
    emit(
        f"UAV {aircraft_id} currentWaypointID missing -> inferred waypoint "
        f"{best_match['waypoint_id']} from MissionPlan {best_match['plan_id']} "
        f"(mission={best_match['individual_mission_id']}, path={best_match['path_id']}{distance_text})."
    )
    return _to_int(best_match.get("plan_id")), _to_int(best_match.get("waypoint_id"))


def _resolve_attack_tracking_recovery(
    *,
    aircraft_id: int,
    source_plan_id: Optional[int],
    current_coord: Optional[Dict[str, Any]],
    emit: LogCallback,
) -> Optional[Dict[str, Any]]:
    assignment = get_tracking_assignment(aircraft_id)
    if not isinstance(assignment, dict) or not bool(assignment.get("active")):
        return None

    attack_plan_id = _to_int(assignment.get("attack_plan_id"))
    source_plan_id_int = _to_int(source_plan_id)
    if attack_plan_id is None or source_plan_id_int is None or attack_plan_id != source_plan_id_int:
        return None

    tracked_source_plan_id = _to_int(assignment.get("source_plan_id"))
    split_wp = (
        _normalize_waypoint_id(assignment.get("handoff_waypoint_id"))
        or _normalize_waypoint_id(assignment.get("last_nonzero_waypoint_id"))
        or _normalize_waypoint_id(assignment.get("original_current_waypoint_id"))
    )
    if tracked_source_plan_id is None or split_wp is None:
        return None

    handoff_coord = _normalize_coordinate(assignment.get("handoff_coordinate"))
    if handoff_coord is None:
        handoff_coord = _normalize_coordinate(assignment.get("last_nonzero_coordinate"))
    if handoff_coord is None:
        handoff_coord = _normalize_coordinate(current_coord)

    emit(
        f"UAV {aircraft_id} attack-exclusion recovery -> "
        f"remove tracking branch and resume source mission "
        f"(sourcePlan={tracked_source_plan_id}, splitWP={split_wp})."
    )
    return {
        "source_plan_id": tracked_source_plan_id,
        "split_waypoint_id": split_wp,
        "done_anchor_coord": handoff_coord,
        "tracking_assignment": assignment,
    }


def _clear_attack_tracking_assignment_if_attached_to_plan(
    *,
    aircraft_id: int,
    source_plan_id: Optional[int],
    emit: LogCallback,
) -> None:
    assignment = get_tracking_assignment(aircraft_id)
    if not isinstance(assignment, dict) or not bool(assignment.get("active")):
        return

    attack_plan_id = _to_int(assignment.get("attack_plan_id"))
    source_plan_id_int = _to_int(source_plan_id)
    if attack_plan_id is None or source_plan_id_int is None or attack_plan_id != source_plan_id_int:
        return

    clear_tracking_assignment(aircraft_id)
    emit(
        f"UAV {aircraft_id} attack tracking assignment cleared after exclusion "
        f"(attackPlan={attack_plan_id})."
    )


def _apply_resume_capture_buffer(
    resume_waypoints: List[Dict[str, Any]],
    *,
    emit: Callable[[str], None],
    log_prefix: str,
) -> None:
    return



def _apply_attack_plan_overrides(
    *,
    ctx: Dict[str, Any],
    attack_point: Dict[str, Any],
    manned_aircraft: Optional[Dict[str, Any]],
    primary_target: Optional[Dict[str, Any]],
    agent_states: List[Any],
    emit: Callable[[str], None],
) -> Optional[Dict[str, Any]]:
    if not attack_point or not manned_aircraft or not primary_target:
        emit("[ATTACK] Mission override skipped (insufficient attack metadata).")
        return None

    manned_id = _to_int(manned_aircraft.get("aircraft_id"))
    watcher_id = _to_int(primary_target.get("watcher_id"))
    if manned_id is None or watcher_id is None:
        emit("[ATTACK] Mission override skipped (aircraft IDs missing).")
        return None

    agent_index = _index_agent_states(agent_states)
    manned_state = agent_index.get(manned_id)
    uav_state = agent_index.get(watcher_id)
    if not manned_state or not manned_state.get("coordinate"):
        emit(f"[ATTACK] Coordinate unavailable for manned aircraft {manned_id}.")
        return None
    if not uav_state or not uav_state.get("current_waypoint_id"):
        emit(f"[ATTACK] Current waypoint unavailable for UAV {watcher_id}.")
        return None

    detail = _normalize_replan_detail(ctx.get("replan_detail")) or {}
    trigger = str(detail.get("trigger") or "").strip()
    has_watcher = detail.get("watcherID") is not None or detail.get("watcherId") is not None
    has_target = detail.get("targetID") is not None or detail.get("targetId") is not None
    has_coord = detail.get("coordinate") is not None or detail.get("targetCoordinate") is not None
    use_detection_tracking = bool(trigger == "0402" or (has_watcher and (has_target or has_coord)))
    if not use_detection_tracking:
        replan_level = _to_int(ctx.get("replan_level") or ctx.get("replanLevel"))
        watcher_fallback = _to_int(primary_target.get("watcher_id")) if isinstance(primary_target, dict) else None
        if replan_level == 2 and watcher_fallback is not None:
            use_detection_tracking = True
            emit(
                f"[ATTACK][ETA] Using detection fallback (replan_level=2, watcher={watcher_fallback})."
            )

    tracking_eta_s: Optional[int] = None
    if use_detection_tracking:
        manned_coord = manned_state.get("coordinate") if isinstance(manned_state, dict) else None
        if manned_coord and attack_point:
            distance_m = _haversine_distance_m(manned_coord, attack_point)
            speed_mps = _to_float(manned_state.get("speed")) if isinstance(manned_state, dict) else None
            if speed_mps is None or speed_mps <= 0:
                speed_mps = 40.0
                emit("[ATTACK][ETA] Manned speed missing; using default 40 m/s.")
            if distance_m is not None:
                tracking_eta_s = int(round(distance_m / speed_mps + 30.0))
                emit(
                    f"[ATTACK][ETA] Tracking ETA computed (dist={distance_m:.1f}m, speed={speed_mps:.1f}m/s, +30s) -> {tracking_eta_s}s"
                )
            else:
                emit("[ATTACK][ETA] Distance calc failed; using default 30s.")
                tracking_eta_s = 30
        else:
            emit("[ATTACK][ETA] Manned/attack coordinate missing; using default 30s.")
            tracking_eta_s = 30

    source_plan_id = _load_latest_mission_progress_plan_id() or _scan_latest_source_plan_id()
    if source_plan_id is None:
        emit("[ATTACK] Mission override skipped (no MissionPlan found).")
        return None

    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = json.loads(plan_src.read_text(encoding="utf-8"))
    except Exception as exc:
        emit(f"[ATTACK] MissionPlan {source_plan_id} load failed: {exc}")
        return None

    sweep_progress = load_sweep_progress()

    requested_plan_id = _resolve_requested_plan_id(
        ctx,
        preferred_option_names={"공격 특화", "공격특화", "공격추천"},
    )
    new_plan_id = requested_plan_id or _allocate_fresh_plan_id()
    if requested_plan_id is not None:
        emit(
            "[ATTACK] Using requested missionPlanID "
            f"{new_plan_id} (sourcePlanID={source_plan_id})"
        )
    else:
        emit(
            "[ATTACK] Allocated fresh missionPlanID "
            f"{new_plan_id} (sourcePlanID={source_plan_id})"
        )

    new_plan_data = deepcopy(plan_data)
    now_ms = _now_timestamp_ms()
    new_plan_data["missionPlanID"] = new_plan_id
    new_plan_data["timestamp"] = now_ms
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = now_ms

    attack_coord = {
        "latitude": attack_point.get("latitude"),
        "longitude": attack_point.get("longitude"),
        "altitude": attack_point.get("altitude"),
    }
    uav_target_coord = primary_target.get("coordinate") or attack_coord

    descriptors = [
        {
            "label": "manned",
            "aircraft_id": manned_id,
            "state": manned_state,
            "target_coord": attack_coord,
            "target_id": primary_target.get("target_id"),
            "mode": "LAH_ATTACK",
        },
        {
            "label": "uav_tracking",
            "aircraft_id": watcher_id,
            "state": uav_state,
            "target_coord": uav_target_coord,
            "target_id": primary_target.get("target_id"),
            "mode": "UAV_TRACK",
        },
    ]

    other_lah_ids: List[int] = []
    for entry in plan_data.get("aircraftList", []):
        aid = _to_int(entry.get("aircraftID"))
        if aid is None or aid > 3 or aid == manned_id:
            continue
        if aid in other_lah_ids:
            continue
        other_lah_ids.append(aid)

    for aid in other_lah_ids:
        descriptors.append(
            {
                "label": f"lah_hold_{aid}",
                "aircraft_id": aid,
                "state": agent_index.get(aid),
                "target_coord": None,
                "target_id": None,
                "mode": "LAH_HOLD_RESUME",
            }
        )

    other_uav_ids: List[int] = []
    for entry in plan_data.get("aircraftList", []):
        aid = _to_int(entry.get("aircraftID"))
        if aid is None or aid <= 3:
            continue
        if aid == watcher_id:
            continue
        if aid in other_uav_ids:
            continue
        other_uav_ids.append(aid)

    for aid in other_uav_ids:
        descriptors.append(
            {
                "label": f"uav_resume_{aid}",
                "aircraft_id": aid,
                "state": agent_index.get(aid),
                "target_coord": None,
                "target_id": None,
                "mode": "UAV_RESUME",
            }
        )

    aircraft_updates: List[Dict[str, Any]] = []
    for descriptor in descriptors:
        aircraft_id = descriptor["aircraft_id"]
        state = descriptor["state"] or {}
        current_wp = _to_int(state.get("current_waypoint_id"))
        if aircraft_id is None:
            emit(f"[ATTACK] {descriptor['label']} aircraft lacks identifier; skipping.")
            continue
        if descriptor["mode"] == "UAV_TRACK" and current_wp is None:
            emit(f"[ATTACK] {descriptor['label']} aircraft lacks waypoint context; skipping.")
            continue

        artifacts = _resolve_plan_artifacts(
            source_plan_id=source_plan_id,
            aircraft_id=aircraft_id,
            current_waypoint_id=current_wp,
            emit=emit,
        )
        if artifacts is None:
            continue

        try:
            imp_src = db_paths.get_db_subpath(
                "IndividualMissionPlan", f"{artifacts.individual_mission_package_id}.json"
            )
            fp_src = db_paths.get_db_subpath("FlightPath", f"{artifacts.path_id}.json")
            imp_data = json.loads(imp_src.read_text(encoding="utf-8"))
            fp_data = json.loads(fp_src.read_text(encoding="utf-8"))
        except Exception as exc:
            emit(f"[ATTACK] Failed to load artifacts for aircraft {aircraft_id}: {exc}")
            continue

        new_imp_id = _next_imp_id()
        if not _update_plan_aircraft_entry(new_plan_data, aircraft_id, new_imp_id, emit):
            continue

        new_imp_data = deepcopy(imp_data)
        mission_list = new_imp_data.get("individualMissionList", [])
        target_mission = None
        target_index = None
        for idx, mission in enumerate(mission_list):
            if _to_int(mission.get("individualMissionID")) == artifacts.individual_mission_id:
                target_mission = mission
                target_index = idx
                break
        if target_mission is None:
            emit(
                f"[ATTACK] Individual mission {artifacts.individual_mission_id} "
                f"not found for aircraft {aircraft_id}."
            )
            continue

        if descriptor["mode"] == "LAH_ATTACK":
            update = _build_lah_attack_package(
                descriptor=descriptor,
                new_imp_id=new_imp_id,
                imp_data=new_imp_data,
                fp_data=fp_data,
                target_mission=target_mission,
                target_index=target_index,
                attack_coord=attack_coord,
                ctx=ctx,
                state=state,
                aircraft_id=aircraft_id,
                artifacts=artifacts,
                emit=emit,
                now_ms=now_ms,
            )
            if update:
                aircraft_updates.append(update)
            continue

        if descriptor["mode"] == "LAH_HOLD_RESUME":
            update = _build_lah_hold_resume_package(
                descriptor=descriptor,
                new_imp_id=new_imp_id,
                imp_data=new_imp_data,
                fp_data=fp_data,
                target_mission=target_mission,
                target_index=target_index,
                ctx=ctx,
                state=state,
                aircraft_id=aircraft_id,
                artifacts=artifacts,
                emit=emit,
                now_ms=now_ms,
            )
            if update:
                aircraft_updates.append(update)
            continue

        if descriptor["mode"] == "UAV_TRACK":
            update = _build_uav_attack_tracking_package(
                descriptor=descriptor,
                new_imp_id=new_imp_id,
                imp_data=new_imp_data,
                fp_data=fp_data,
                target_mission_template=target_mission,
                target_index=target_index,
                attack_coord=attack_coord,
                ctx=ctx,
                state=state,
                artifacts=artifacts,
                emit=emit,
                now_ms=now_ms,
                force_start_at_current=use_detection_tracking,
                tracking_eta_s=tracking_eta_s,
                sweep_progress=sweep_progress,
            )
        else:
            update = _build_uav_attack_resume_package(
                descriptor=descriptor,
                new_imp_id=new_imp_id,
                imp_data=new_imp_data,
                fp_data=fp_data,
                target_mission_template=target_mission,
                target_index=target_index,
                ctx=ctx,
                state=state,
                artifacts=artifacts,
                emit=emit,
                now_ms=now_ms,
                sweep_progress=sweep_progress,
            )
        if update:
            if descriptor["mode"] == "UAV_TRACK":
                tracking_meta = update.get("tracking") if isinstance(update, dict) else {}
                resume_meta = update.get("resume") if isinstance(update, dict) else {}
                register_tracking_assignment(
                    aircraft_id=int(aircraft_id),
                    source_plan_id=int(source_plan_id),
                    attack_plan_id=int(new_plan_id),
                    original_path_id=int(artifacts.path_id),
                    original_individual_mission_id=int(artifacts.individual_mission_id),
                    original_current_waypoint_id=_normalize_waypoint_id(artifacts.current_waypoint_id),
                    original_coordinate=state.get("coordinate") if isinstance(state, dict) else None,
                    tracking_path_id=_to_int((tracking_meta or {}).get("pathID")),
                    tracking_individual_mission_id=_to_int(
                        (tracking_meta or {}).get("individualMissionID")
                    ),
                    resume_path_id=_to_int((resume_meta or {}).get("pathID")),
                    resume_individual_mission_id=_to_int(
                        (resume_meta or {}).get("individualMissionID")
                    ),
                    target_id=_to_int(primary_target.get("target_id")),
                )
                emit(
                    f"[ATTACK][UAV] Tracking assignment state saved "
                    f"(aircraft={aircraft_id}, sourcePlan={source_plan_id}, attackPlan={new_plan_id})."
                )
            aircraft_updates.append(update)

    if not aircraft_updates:
        emit("[ATTACK] Mission override produced no artifacts.")
        return None

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
    plan_dest.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(plan_dest, new_plan_data)
    emit(f"[ATTACK][PLAN] MissionPlan saved -> {plan_dest.name} (planID={new_plan_id})")

    plan_meta_map = dict(ctx.get("_option_meta") or {})
    plan_meta_entry = plan_meta_map.setdefault(new_plan_id, {})
    plan_meta_entry.update(
        {
            "attack": True,
            "attackTargets": {
                "targetID": primary_target.get("target_id"),
                "watcherID": watcher_id,
                "attackPoint": attack_coord,
            },
        }
    )
    ctx["_option_meta"] = plan_meta_map

    return {
        "source_plan_id": source_plan_id,
        "mission_plan_id": new_plan_id,
        "plan_path": str(plan_dest),
        "aircraft": aircraft_updates,
    }


def _index_agent_states(agent_states: List[Any]) -> Dict[int, Dict[str, Any]]:
    index: Dict[int, Dict[str, Any]] = {}
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
            unm_info = entry.get("unmannedInfo") or {}
            wp_block = unm_info.get("currentWaypointID") or {}
        current_wp = _normalize_waypoint_id(wp_block.get("waypointID"))
        velocity = entry.get("velocity") or {}
        heading = _to_float(velocity.get("heading"))
        speed = _to_float(velocity.get("speed"))
        if heading is not None:
            heading = heading % 360.0
        index[aircraft_id] = {
            "aircraft_id": aircraft_id,
            "coordinate": coord,
            "current_waypoint_id": current_wp,
            "is_unmanned": bool(entry.get("isUnmanned")),
            "heading": heading,
            "speed": speed,
        }
    return index


def _insert_lah_attack_waypoint(
    flight_path: Dict[str, Any],
    current_waypoint_id: int,
    attack_coord: Dict[str, Any],
    new_waypoint_id: int,
    target_id: Optional[int],
) -> Dict[str, Any]:
    waypoints = list(flight_path.get("lahWaypointList") or [])
    if not waypoints:
        raise ValueError("LAH flight path is empty.")
    current_index = next(
        (idx for idx, wp in enumerate(waypoints) if _to_int(wp.get("waypointID")) == current_waypoint_id),
        None,
    )
    if current_index is None:
        raise ValueError(f"Current waypoint {current_waypoint_id} not found in LAH flight path.")

    template = deepcopy(waypoints[current_index])
    prev_index = current_index - 1
    if prev_index >= 0 and "nextWaypointID" in waypoints[prev_index]:
        waypoints[prev_index]["nextWaypointID"] = new_waypoint_id

    altitude = _normalize_altitude_value(attack_coord.get("altitude"))
    if altitude is None:
        altitude = _normalize_altitude_value((template.get("coordinate") or {}).get("altitude"))
    if altitude is None:
        altitude = 800
    coordinate = {
        "latitude": attack_coord.get("latitude"),
        "longitude": attack_coord.get("longitude"),
        "altitude": altitude,
    }
    new_wp = {
        "waypointID": new_waypoint_id,
        "coordinate": coordinate,
        "speed": template.get("speed", 40),
        "eta": template.get("eta", 0),
        "ecf": template.get("ecf", 0.0),
        "nextWaypointID": current_waypoint_id,
        "hovering": {"time": 0},
        "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
        "attack": deepcopy(template.get("attack") or {}),
    }
    attack_block = new_wp["attack"] or {}
    attack_block["targetID"] = target_id or 0
    attack_block["weaponType"] = attack_block.get("weaponType") or ATTACK_WEAPON_TYPE
    new_wp["attack"] = attack_block

    waypoints.insert(current_index, new_wp)
    flight_path["lahWaypointList"] = waypoints
    return new_wp


def _extract_path_source(fp_data: Dict[str, Any]) -> str:
    return str(fp_data.get("Source") or fp_data.get("source") or "MMR")


def _extract_lah_waypoint_coordinate(waypoint: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    if not isinstance(waypoint, dict):
        return None
    return _normalize_coordinate(waypoint.get("coordinate"))


def _infer_lah_current_waypoint_id(
    waypoints: List[Dict[str, Any]],
    current_coord: Optional[Dict[str, Any]],
) -> Optional[int]:
    coord_norm = _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None
    if coord_norm is None:
        return None

    best_waypoint_id: Optional[int] = None
    best_score: Optional[Tuple[int, float, int]] = None
    for idx, waypoint in enumerate(waypoints):
        waypoint_id = _to_int(waypoint.get("waypointID"))
        waypoint_coord = _extract_lah_waypoint_coordinate(waypoint)
        if waypoint_id is None or waypoint_coord is None:
            continue
        distance_m = _haversine_distance_m(coord_norm, waypoint_coord)
        if distance_m is None:
            continue
        score = (
            1 if bool(waypoint.get("isDone")) else 0,
            float(distance_m),
            idx,
        )
        if best_score is None or score < best_score:
            best_score = score
            best_waypoint_id = waypoint_id
    return best_waypoint_id


def _lah_waypoints_to_coordinate_list(
    waypoints: List[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    coordinate_list: List[Dict[str, Any]] = []
    for waypoint in waypoints:
        coord = _extract_lah_waypoint_coordinate(waypoint)
        if coord:
            coordinate_list.append(coord)
    return coordinate_list


def _build_lah_anchor_waypoint(
    template_wp: Dict[str, Any],
    *,
    coord: Dict[str, Any],
    next_id: int = 0,
    hovering_time: int = 0,
) -> Dict[str, Any]:
    anchor_wp = _build_lah_waypoint_from_template(
        template_wp,
        _next_waypoint_id(),
        coord,
        next_id,
        mark_attack=False,
        target_id=None,
    )
    anchor_wp["isDone"] = False
    if hovering_time > 0:
        anchor_wp["hovering"] = {"time": int(hovering_time)}
    elif "hovering" in anchor_wp:
        anchor_wp["hovering"] = {"time": 0}
    return anchor_wp


def _same_normalized_coordinate(
    left: Optional[Dict[str, Any]],
    right: Optional[Dict[str, Any]],
) -> bool:
    if left is None or right is None:
        return False
    left_lat = _to_float(left.get("latitude"))
    left_lon = _to_float(left.get("longitude"))
    right_lat = _to_float(right.get("latitude"))
    right_lon = _to_float(right.get("longitude"))
    if None in (left_lat, left_lon, right_lat, right_lon):
        return False
    if abs(float(left_lat) - float(right_lat)) > 1e-7:
        return False
    if abs(float(left_lon) - float(right_lon)) > 1e-7:
        return False
    return (_normalize_altitude_value(left.get("altitude")) or 0) == (
        _normalize_altitude_value(right.get("altitude")) or 0
    )


def _append_lah_done_anchor(
    done_waypoints: List[Dict[str, Any]],
    *,
    template_wp: Dict[str, Any],
    anchor_coord: Optional[Dict[str, Any]],
) -> None:
    anchor = _normalize_coordinate(anchor_coord) if isinstance(anchor_coord, dict) else None
    if anchor is None:
        return
    last_coord = _extract_lah_waypoint_coordinate(done_waypoints[-1]) if done_waypoints else None
    if _same_normalized_coordinate(last_coord, anchor):
        return
    done_waypoints.append(
        _build_lah_anchor_waypoint(
            template_wp,
            coord=anchor,
            next_id=0,
            hovering_time=0,
        )
    )


def _prepend_lah_transition_waypoint(
    waypoints: List[Dict[str, Any]],
    *,
    template_wp: Dict[str, Any],
    anchor_coord: Optional[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    if not waypoints:
        return waypoints
    anchor = _normalize_coordinate(anchor_coord) if isinstance(anchor_coord, dict) else None
    if anchor is None:
        return waypoints
    first_coord = _extract_lah_waypoint_coordinate(waypoints[0])
    if _same_normalized_coordinate(first_coord, anchor):
        return waypoints
    anchored = [
        _build_lah_anchor_waypoint(
            template_wp,
            coord=anchor,
            next_id=_to_int((waypoints[0] or {}).get("waypointID")) or 0,
            hovering_time=0,
        )
    ]
    anchored.extend(deepcopy(waypoints))
    relink_waypoints(anchored)
    return anchored


def _build_lah_hold_coordinate_near_resume(
    *,
    resume_waypoints: List[Dict[str, Any]],
    current_coord: Optional[Dict[str, Any]],
) -> Optional[Dict[str, Any]]:
    first_resume = _extract_lah_waypoint_coordinate(resume_waypoints[0]) if resume_waypoints else None
    if first_resume is None:
        return _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None

    second_resume = (
        _extract_lah_waypoint_coordinate(resume_waypoints[1])
        if len(resume_waypoints) >= 2
        else None
    )

    base_bearing: Optional[float] = None
    if second_resume is not None:
        base_bearing = _bearing_between(
            float(first_resume["latitude"]),
            float(first_resume["longitude"]),
            float(second_resume["latitude"]),
            float(second_resume["longitude"]),
        )
    elif isinstance(current_coord, dict):
        current_norm = _normalize_coordinate(current_coord)
        if current_norm is not None:
            base_bearing = _bearing_between(
                float(current_norm["latitude"]),
                float(current_norm["longitude"]),
                float(first_resume["latitude"]),
                float(first_resume["longitude"]),
            )

    if base_bearing is None:
        base_bearing = 90.0

    hold_coord = dict(first_resume)
    projected = _project_coordinate(
        first_resume,
        (float(base_bearing) + 90.0) % 360.0,
        LAH_HOLD_NEAR_RESUME_OFFSET_METERS,
    )
    if projected:
        hold_coord.update(
            {
                "latitude": projected.get("latitude", hold_coord.get("latitude")),
                "longitude": projected.get("longitude", hold_coord.get("longitude")),
            }
        )
    hold_coord["altitude"] = first_resume.get("altitude")
    return _normalize_coordinate(hold_coord)


def _split_done_resume_lah_path(
    source_fp_data: Dict[str, Any],
    *,
    artifacts: Any,
    current_coord: Optional[Dict[str, Any]],
    emit: Callable[[str], None],
    force_nonempty_resume: bool = False,
    exclude_current_from_resume: bool = False,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], Optional[int]]:
    waypoints = list(source_fp_data.get("lahWaypointList") or [])
    done_waypoints: List[Dict[str, Any]] = []
    resume_waypoints: List[Dict[str, Any]] = []
    removed_wp_id: Optional[int] = None

    if not waypoints:
        return done_waypoints, resume_waypoints, removed_wp_id

    curr_wp = _to_int(getattr(artifacts, "current_waypoint_id", None))
    prev_wp = _to_int(getattr(artifacts, "previous_waypoint_id", None))
    if curr_wp is None:
        inferred_wp = _infer_lah_current_waypoint_id(waypoints, current_coord)
        if inferred_wp is not None:
            curr_wp = inferred_wp
            emit(f"[ATTACK][LAH] Inferred current waypoint from position -> {curr_wp}.")

    curr_idx = next(
        (idx for idx, waypoint in enumerate(waypoints) if _to_int(waypoint.get("waypointID")) == curr_wp),
        None,
    )

    if curr_idx is not None:
        done_waypoints = deepcopy(waypoints[:curr_idx]) if curr_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[curr_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_wp_id = prev_wp

        done_prefix_idx = 0
        while done_prefix_idx < len(waypoints) and bool(waypoints[done_prefix_idx].get("isDone")):
            done_prefix_idx += 1
        if done_prefix_idx != curr_idx:
            emit(
                "[ATTACK][LAH] isDone/currentWP mismatch; "
                f"using currentWP split (isDonePrefix={done_prefix_idx}, currentIdx={curr_idx})."
            )
    elif any(bool(waypoint.get("isDone")) for waypoint in waypoints):
        idx = 0
        while idx < len(waypoints) and bool(waypoint.get("isDone")):
            idx += 1
        done_waypoints = deepcopy(waypoints[:idx]) if idx > 0 else []
        resume_waypoints = deepcopy(waypoints[idx:]) if idx > 0 else deepcopy(waypoints)
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
    else:
        resume_waypoints = deepcopy(waypoints)
        removed_wp_id = prev_wp

    if force_nonempty_resume and not resume_waypoints and waypoints:
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
            "[ATTACK][LAH] Resume fallback applied "
            f"(forcedStartWP={_to_int((resume_waypoints[0] or {}).get('waypointID'))})."
        )

    if exclude_current_from_resume and resume_waypoints:
        first_resume_wp_id = _to_int((resume_waypoints[0] or {}).get("waypointID"))
        if curr_wp is not None and first_resume_wp_id == curr_wp:
            removed_wp_id = int(curr_wp)
            resume_waypoints = deepcopy(resume_waypoints[1:]) if len(resume_waypoints) > 1 else []
            emit(
                "[ATTACK][LAH] Dropped current waypoint from resume path "
                f"(currentWP={curr_wp}, nextWP={_to_int((resume_waypoints[0] or {}).get('waypointID')) if resume_waypoints else None})."
            )

    template_wp = deepcopy((waypoints or [None])[0]) if waypoints else _default_lah_waypoint_template()
    if not done_waypoints:
        current_anchor = _normalize_coordinate(current_coord) if isinstance(current_coord, dict) else None
        if current_anchor is not None:
            done_waypoints = [
                _build_lah_anchor_waypoint(
                    template_wp,
                    coord=current_anchor,
                    next_id=0,
                    hovering_time=0,
                )
            ]
    else:
        _append_lah_done_anchor(
            done_waypoints,
            template_wp=template_wp,
            anchor_coord=current_coord,
        )

    for waypoint in done_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = True
    for waypoint in resume_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False

    if done_waypoints:
        reassign_unique_waypoint_ids_inplace(done_waypoints)
    if resume_waypoints:
        reassign_unique_waypoint_ids_inplace(resume_waypoints)
    return done_waypoints, resume_waypoints, removed_wp_id


def _split_done_resume_path(
    source_fp_data: Dict[str, Any],
    *,
    artifacts: Any,
    sweep_progress: Dict[int, Dict[str, Any]] | None,
    emit: Callable[[str], None],
    force_nonempty_resume: bool = False,
    append_replan_anchor: bool = False,
    replan_coordinate: Optional[Dict[str, Any]] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], Optional[int]]:
    waypoints = list(source_fp_data.get("waypointList") or [])
    done_waypoints: List[Dict[str, Any]] = []
    resume_waypoints: List[Dict[str, Any]] = []
    removed_wp_id: Optional[int] = None

    if not waypoints:
        return done_waypoints, resume_waypoints, removed_wp_id

    curr_wp = _to_int(getattr(artifacts, "current_waypoint_id", None))
    prev_wp = _to_int(getattr(artifacts, "previous_waypoint_id", None))
    curr_idx = next(
        (i for i, wp in enumerate(waypoints) if _to_int(wp.get("waypointID")) == curr_wp),
        None,
    )

    # Prefer current waypoint based split whenever available:
    # [done ...] + [current ... remaining]
    if curr_idx is not None:
        done_waypoints = deepcopy(waypoints[:curr_idx]) if curr_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[curr_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp:
            removed_wp_id = prev_wp

        # Diagnostic when waypoint isDone flags disagree with current waypoint progress.
        done_prefix_idx = 0
        while done_prefix_idx < len(waypoints) and bool(waypoints[done_prefix_idx].get("isDone")):
            done_prefix_idx += 1
        if done_prefix_idx != curr_idx:
            emit(
                "[ATTACK][UAV] isDone/currentWP mismatch; "
                f"using currentWP split (isDonePrefix={done_prefix_idx}, currentIdx={curr_idx})."
            )
        if removed_wp_id is not None:
            emit(f"[ATTACK][UAV] Resume trimmed by currentWP (lastRemovedWP={removed_wp_id}).")
    elif any(bool(wp.get("isDone")) for wp in waypoints):
        idx = 0
        while idx < len(waypoints) and bool(waypoints[idx].get("isDone")):
            idx += 1
        done_waypoints = deepcopy(waypoints[:idx]) if idx > 0 else []
        resume_waypoints = deepcopy(waypoints[idx:]) if idx > 0 else deepcopy(waypoints)
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        if removed_wp_id is not None and not (force_nonempty_resume and idx >= len(waypoints)):
            emit(f"[ATTACK][UAV] Resume trimmed by isDone (lastRemovedWP={removed_wp_id}).")
    else:
        done_waypoints = []
        resume_waypoints = deepcopy(waypoints)
        removed_wp_id = prev_wp

    if force_nonempty_resume and not resume_waypoints and waypoints:
        forced_start_idx: Optional[int] = None

        # 1) Prefer a split after previous waypoint when available.
        if prev_wp is not None:
            for idx, waypoint in enumerate(waypoints):
                if _to_int(waypoint.get("waypointID")) == prev_wp:
                    forced_start_idx = min(idx + 1, len(waypoints) - 1)
                    break

        # 2) Otherwise keep the first not-done waypoint (if any).
        if forced_start_idx is None:
            for idx, waypoint in enumerate(waypoints):
                if not bool(waypoint.get("isDone")):
                    forced_start_idx = idx
                    break

        # 3) Last fallback: keep the terminal waypoint as resume anchor.
        if forced_start_idx is None:
            forced_start_idx = len(waypoints) - 1

        done_waypoints = deepcopy(waypoints[:forced_start_idx]) if forced_start_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[forced_start_idx:])
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_wp_id = prev_wp
        emit(
            "[ATTACK][UAV] Resume fallback applied "
            f"(forcedStartWP={_to_int((resume_waypoints[0] or {}).get('waypointID'))})."
        )

    done_sweep_points = count_sweep_points_in_waypoints(done_waypoints)

    if append_replan_anchor and done_waypoints and resume_waypoints:
        anchor_coord_src = replan_coordinate if isinstance(replan_coordinate, dict) else {}
        anchor_lat = _to_float(anchor_coord_src.get("latitude"))
        anchor_lon = _to_float(anchor_coord_src.get("longitude"))
        anchor_alt = _normalize_altitude_value(anchor_coord_src.get("altitude"))
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
                    "waypointID": _next_waypoint_id(),
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
                            "fieldOfView": 10.0,
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
                if "hovering" in template_wp:
                    anchor_wp["hovering"] = {"time": 0}
                if "loiter" in template_wp:
                    anchor_wp["loiter"] = {"radius": 0, "direction": 0, "time": 0, "speed": 0}
                if "attack" in template_wp:
                    anchor_wp["attack"] = {"targetID": 0, "weaponType": 0}

                done_waypoints.append(anchor_wp)
                emit(
                    "[ATTACK][UAV] Added replan anchor waypoint to done path "
                    f"(anchorWP={anchor_wp.get('waypointID')})."
                )

    progress_entry = None
    if sweep_progress and artifacts.path_id is not None:
        progress_entry = sweep_progress.get(int(artifacts.path_id))
    raw_cut_points = sweep_cut_points(progress_entry)
    cut_points = max(0, int(raw_cut_points) - int(done_sweep_points))
    if cut_points > 0 and resume_waypoints:
        resume_waypoints, removed_points = trim_waypoints_by_sweep_points(
            resume_waypoints,
            cut_points,
            preserve_waypoints=True,
        )
        if removed_points > 0:
            emit(
                f"[ATTACK][UAV] Resume sweep trim applied "
                f"(cutPoints={removed_points}, rawCutPoints={raw_cut_points}, "
                f"doneSweepPoints={done_sweep_points}, pathID={artifacts.path_id})."
            )

    for wp in done_waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = True
    for wp in resume_waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = False

    if done_waypoints:
        reassign_unique_waypoint_ids_inplace(done_waypoints)
    if resume_waypoints:
        _apply_resume_capture_buffer(
            resume_waypoints,
            emit=emit,
            log_prefix="[ATTACK][UAV]",
        )
        scaled = scale_line_search_speed(resume_waypoints, _RESUME_SEARCH_SPEED_SCALE)
        if scaled > 0:
            emit(
                f"[ATTACK][UAV] Resume searchSpeed scaled "
                f"(factor={_RESUME_SEARCH_SPEED_SCALE:.2f}, waypoints={scaled})."
            )
        reassign_unique_waypoint_ids_inplace(resume_waypoints)
    return done_waypoints, resume_waypoints, removed_wp_id


def _resolve_path_start_waypoint_id(path_id: Optional[int]) -> Optional[int]:
    pid = _to_int(path_id)
    if pid is None:
        return None
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{pid}.json")
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    waypoints = []
    for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
        lst = data.get(key)
        if isinstance(lst, list):
            waypoints = lst
            break
    if not waypoints:
        return None
    fallback: Optional[int] = None
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        wid = _to_int(wp.get("waypointID"))
        if wid is None:
            continue
        if fallback is None:
            fallback = wid
        if not bool(wp.get("isDone")):
            return wid
    return fallback


def _build_uav_attack_tracking_package(
    *,
    descriptor: Dict[str, Any],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission_template: Dict[str, Any],
    target_index: Optional[int],
    attack_coord: Dict[str, Any],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
    force_start_at_current: bool = False,
    tracking_eta_s: Optional[int] = None,
    sweep_progress: Dict[int, Dict[str, Any]] | None = None,
) -> Optional[Dict[str, Any]]:
    if target_index is None:
        emit("[ATTACK][UAV] Target mission index unavailable; skipping UAV override.")
        return None
    agent_coord = _normalize_coordinate(state.get("coordinate"))
    if not agent_coord:
        emit(f"[ATTACK][UAV] Coordinate missing for aircraft {descriptor['aircraft_id']}.")
        return None

    target_coord_norm = _normalize_coordinate(descriptor.get("target_coord") or attack_coord)
    if not target_coord_norm:
        emit(f"[ATTACK][UAV] Target coordinate unavailable for aircraft {descriptor['aircraft_id']}.")
        return None
    if target_coord_norm.get("altitude") is None:
        target_coord_norm["altitude"] = _normalize_altitude_value(agent_coord.get("altitude")) or 700

    done_path_id, attack_path_id, resume_path_id = _reserve_path_ids(descriptor["aircraft_id"], 3)
    tracking_individual_id, resume_individual_id = _reserve_individual_mission_ids(2)
    target_wp_id = _next_waypoint_id()
    tracking_target_id = _to_int(descriptor.get("target_id"))
    if tracking_target_id is None:
        detail = ctx.get("replan_detail") if isinstance(ctx, dict) else {}
        if isinstance(detail, dict):
            tracking_target_id = _to_int(detail.get("targetID") or detail.get("targetId"))
            if tracking_target_id is None:
                orient = detail.get("targetOrientation") or {}
                if isinstance(orient, dict):
                    tracking_target_id = _to_int(orient.get("targetID") or orient.get("targetId"))
    tracking_target_id_value = tracking_target_id if tracking_target_id is not None else 0

    original_entry = deepcopy(target_mission_template)
    base_rel_block = dict(original_entry.get("relatedMission") or {})
    input_mission_id = _to_int(base_rel_block.get("inputMissionID")) or _to_int((ctx.get("mission_ids") or [None])[0]) or 0
    prior_mission_id = _to_int(base_rel_block.get("priorMissionID")) or 0
    attack_reason = ctx.get("reason")

    tracking_rel = dict(base_rel_block)
    tracking_rel["relatedMissionType"] = 3
    tracking_rel["inputMissionID"] = input_mission_id
    tracking_rel["priorMissionID"] = prior_mission_id
    tracking_rel["attackReason"] = attack_reason
    tracking_rel["targetID"] = tracking_target_id_value

    resume_rel = dict(base_rel_block)
    resume_rel["relatedMissionType"] = base_rel_block.get("relatedMissionType", 1)
    resume_rel["inputMissionID"] = input_mission_id
    resume_rel["priorMissionID"] = prior_mission_id

    mission_attack = {
        "individualMissionID": tracking_individual_id,
        "isDone": False,
        "relatedMission": tracking_rel,
        "individualMissionInfo": {
            "individualMissionType": 1,
            "patternType": 1,
            "autoZoomIn": True,
            "coordinateList": [
                {
                    "latitude": target_coord_norm["latitude"],
                    "longitude": target_coord_norm["longitude"],
                    "altitude": target_coord_norm["altitude"],
                },
            ],
            "lineList": [],
            "areaList": [],
            "targetID": tracking_target_id_value,
        },
        "pathID": attack_path_id,
    }

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = resume_rel
    mission_resume["isDone"] = False
    source_waypoints = list(fp_data.get("waypointList") or [])
    source_single_point = len(source_waypoints) <= 1
    if source_single_point:
        done_waypoints = deepcopy(source_waypoints)
        for wp in done_waypoints:
            if isinstance(wp, dict):
                wp["isDone"] = True
        resume_waypoints = []
        removed_wp_id = _to_int((done_waypoints[-1] or {}).get("waypointID")) if done_waypoints else None
        emit(
            "[ATTACK][UAV] Source path has a single waypoint; "
            "preserving it as done and skipping done/resume split."
        )
    else:
        done_waypoints, resume_waypoints, removed_wp_id = _split_done_resume_path(
            fp_data,
            artifacts=artifacts,
            sweep_progress=sweep_progress,
            emit=emit,
            force_nonempty_resume=True,
            append_replan_anchor=True,
            replan_coordinate=agent_coord,
        )

    preserved_individual_id = _to_int(original_entry.get("individualMissionID"))
    done_fp_data = deepcopy(fp_data)
    done_fp_data["pathID"] = done_path_id
    done_fp_data["timestamp"] = now_ms
    done_fp_data["Source"] = done_fp_data.get("Source") or "MMR"
    done_fp_data["aircraftID"] = descriptor["aircraft_id"]
    if preserved_individual_id is not None:
        done_fp_data["individualMissionID"] = preserved_individual_id
    done_fp_data["waypointList"] = done_waypoints

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["waypointList"] = resume_waypoints

    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = resume_fp_data.get("Source") or "MMR"
    resume_fp_data["aircraftID"] = descriptor["aircraft_id"]
    resume_fp_data["individualMissionID"] = resume_individual_id

    has_resume = bool(resume_waypoints)
    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    done_input_ids = _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    source_mission_list = imp_data.get("individualMissionList")
    if (
        isinstance(source_mission_list, list)
        and target_index is not None
        and 0 <= target_index < len(source_mission_list)
    ):
        cloned_artifacts = _clone_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][UAV]",
            excluded_input_ids=done_input_ids,
        )
        if cloned_artifacts is None:
            return None
        follow_up_missions, follow_up_paths = cloned_artifacts

    target_eta = int(tracking_eta_s) if isinstance(tracking_eta_s, int) and tracking_eta_s >= 0 else 30
    target_loiter_time = target_eta

    target_wp = {
        "waypointID": target_wp_id,
        "coordinate": {
            "latitude": target_coord_norm["latitude"],
            "longitude": target_coord_norm["longitude"],
            "altitude": target_coord_norm["altitude"],
        },
        "speed": 30.0,
        "eta": target_eta,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": 2,
        "filmingProperty": {
            "fieldOfView": 5.0,
            "sensorType": 1,
            "operationMode": 3,
            "coordinateOrientation": {
                "coordinate": {
                    "latitude": target_coord_norm["latitude"],
                    "longitude": target_coord_norm["longitude"],
                    "altitude": target_coord_norm["altitude"],
                }
            },
        },
    }
    if target_loiter_time > 0:
        target_wp["loiterProperty"] = {
            "radius": 400,
            "direction": 1,
            "time": target_loiter_time,
            "speed": 30,
        }
    if tracking_target_id is not None:
        filming = target_wp.get("filmingProperty") or {}
        filming["autoTracking"] = {"targetID": tracking_target_id}
        if "coordinateOrientation" in filming:
            del filming["coordinateOrientation"]
        target_wp["filmingProperty"] = filming

    tracking_fp_data = {
        "timestamp": now_ms,
        "Source": fp_data.get("Source") or "MMR",
        "pathID": attack_path_id,
        "aircraftID": descriptor["aircraft_id"],
        "individualMissionID": tracking_individual_id,
        "isFormationFlight": fp_data.get("isFormationFlight", False),
        "waypointList": [target_wp],
    }

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    write_done_path = False
    write_resume_path = False
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    if 0 <= target_index < len(mission_list):
        prefix = deepcopy(mission_list[:target_index])
        preserved = deepcopy(mission_list[target_index] or {})
        preserved["isDone"] = True
        preserved["pathID"] = done_path_id
        rebuilt = prefix + [preserved, mission_attack]
        if has_resume:
            rebuilt.append(mission_resume)
            write_resume_path = True
        rebuilt.extend(follow_up_missions)
        mission_list[:] = rebuilt
        write_done_path = True
        emit(
            f"[ATTACK][UAV] Preserved done segment, inserted tracking, and reattached "
            f"{len(follow_up_missions)} follow-up mission(s)."
        )
    else:
        mission_list.insert(0, mission_attack)
        if has_resume:
            mission_list.insert(1, mission_resume)
            write_resume_path = True
        emit("[ATTACK][UAV] Target mission index invalid; appended tracking branch at head.")

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    done_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{done_path_id}.json") if write_done_path else None
    )
    tracking_fp_dest = db_paths.get_db_subpath("FlightPath", f"{attack_path_id}.json")
    resume_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json") if write_resume_path else None
    )
    write_targets = [imp_dest, tracking_fp_dest]
    if resume_fp_dest is not None:
        write_targets.append(resume_fp_dest)
    if done_fp_dest is not None:
        write_targets.append(done_fp_dest)
    write_targets.extend(dest for dest, _ in follow_up_paths)
    for path in write_targets:
        path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(imp_dest, imp_data)
    if done_fp_dest is not None:
        _write_json_file(done_fp_dest, done_fp_data)
    _write_json_file(tracking_fp_dest, tracking_fp_data)
    if resume_fp_dest is not None:
        _write_json_file(resume_fp_dest, resume_fp_data)
    for dest, payload in follow_up_paths:
        _write_json_file(dest, payload)

    if resume_fp_dest is not None:
        emit(
            f"[ATTACK][UAV] Generated tracking/resume missions -> "
            f"IMP:{imp_dest.name} PATHS:{tracking_fp_dest.name}/{resume_fp_dest.name} "
            f"(followUps={len(follow_up_missions)})"
        )
    else:
        emit(
            f"[ATTACK][UAV] Generated tracking-only mission -> "
            f"IMP:{imp_dest.name} PATH:{tracking_fp_dest.name} "
            f"(followUps={len(follow_up_missions)}, resumeSkipped=true)"
        )

    result: Dict[str, Any] = {
        "aircraft_id": descriptor["aircraft_id"],
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "tracking": {
            "individualMissionID": tracking_individual_id,
            "pathID": attack_path_id,
            "targetWaypointID": target_wp_id,
        },
        "removedWaypointID": removed_wp_id,
        "donePath": str(done_fp_dest) if done_fp_dest is not None else None,
        "trackingPath": str(tracking_fp_dest),
    }
    if write_resume_path and resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
        result["resumePath"] = str(resume_fp_dest)
    result["followUpMissionCount"] = len(follow_up_missions)
    return result


def _build_uav_attack_resume_package(
    *,
    descriptor: Dict[str, Any],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission_template: Dict[str, Any],
    target_index: Optional[int],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
    sweep_progress: Dict[int, Dict[str, Any]] | None = None,
) -> Optional[Dict[str, Any]]:
    if target_index is None:
        emit("[ATTACK][UAV] Target mission index unavailable; skipping UAV resume.")
        return None

    done_path_id, resume_path_id = _reserve_path_ids(descriptor["aircraft_id"], 2)
    [resume_individual_id] = _reserve_individual_mission_ids(1)

    original_entry = deepcopy(target_mission_template)
    base_rel_block = dict(original_entry.get("relatedMission") or {})
    input_mission_id = (
        _to_int(base_rel_block.get("inputMissionID"))
        or _to_int((ctx.get("mission_ids") or [None])[0])
        or 0
    )
    prior_mission_id = _to_int(base_rel_block.get("priorMissionID")) or 0

    resume_rel = dict(base_rel_block)
    resume_rel["relatedMissionType"] = base_rel_block.get("relatedMissionType", 1)
    resume_rel["inputMissionID"] = input_mission_id
    resume_rel["priorMissionID"] = prior_mission_id

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = resume_rel
    mission_resume["isDone"] = False

    replan_coord = _normalize_coordinate((state or {}).get("coordinate")) or {}

    done_waypoints, resume_waypoints, removed_wp_id = _split_done_resume_path(
        fp_data,
        artifacts=artifacts,
        sweep_progress=sweep_progress,
        emit=emit,
        append_replan_anchor=True,
        replan_coordinate=replan_coord,
    )

    preserved_individual_id = _to_int(original_entry.get("individualMissionID"))
    done_fp_data = deepcopy(fp_data)
    done_fp_data["pathID"] = done_path_id
    done_fp_data["timestamp"] = now_ms
    done_fp_data["Source"] = done_fp_data.get("Source") or "MMR"
    done_fp_data["aircraftID"] = descriptor["aircraft_id"]
    if preserved_individual_id is not None:
        done_fp_data["individualMissionID"] = preserved_individual_id
    done_fp_data["waypointList"] = done_waypoints

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["waypointList"] = resume_waypoints

    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = resume_fp_data.get("Source") or "MMR"
    resume_fp_data["aircraftID"] = descriptor["aircraft_id"]
    resume_fp_data["individualMissionID"] = resume_individual_id

    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    done_input_ids = _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    source_mission_list = imp_data.get("individualMissionList")
    if isinstance(source_mission_list, list) and 0 <= target_index < len(source_mission_list):
        cloned_artifacts = _clone_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][UAV]",
            excluded_input_ids=done_input_ids,
        )
        if cloned_artifacts is None:
            return None
        follow_up_missions, follow_up_paths = cloned_artifacts

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    write_done_path = False
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    if 0 <= target_index < len(mission_list):
        prefix = deepcopy(mission_list[:target_index])
        preserved = deepcopy(mission_list[target_index] or {})
        preserved["isDone"] = True
        preserved["pathID"] = done_path_id
        rebuilt = prefix + [preserved, mission_resume]
        rebuilt.extend(follow_up_missions)
        mission_list[:] = rebuilt
        write_done_path = True
        emit(
            "[ATTACK][UAV] Preserved prior mission as done, inserted resume, "
            f"and reattached {len(follow_up_missions)} follow-up mission(s)."
        )
    else:
        mission_list.insert(0, mission_resume)
        emit("[ATTACK][UAV] Target mission index invalid; appended resume at head.")

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    done_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{done_path_id}.json") if write_done_path else None
    )
    resume_fp_dest = db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json")
    write_targets = [imp_dest, resume_fp_dest]
    if done_fp_dest is not None:
        write_targets.append(done_fp_dest)
    write_targets.extend(dest for dest, _ in follow_up_paths)
    for path in write_targets:
        path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(imp_dest, imp_data)
    if done_fp_dest is not None:
        _write_json_file(done_fp_dest, done_fp_data)
    _write_json_file(resume_fp_dest, resume_fp_data)
    for dest, payload in follow_up_paths:
        _write_json_file(dest, payload)

    emit(
        f"[ATTACK][UAV] Generated resume-only mission -> "
        f"IMP:{imp_dest.name} PATH:{resume_fp_dest.name} "
        f"(followUps={len(follow_up_missions)})"
    )

    return {
        "aircraft_id": descriptor["aircraft_id"],
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "resume": {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        },
        "removedWaypointID": removed_wp_id,
        "donePath": str(done_fp_dest) if done_fp_dest is not None else None,
        "resumePath": str(resume_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
    }


def _build_lah_attack_package(
    *,
    descriptor: Dict[str, Any],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission: Dict[str, Any],
    target_index: Optional[int],
    attack_coord: Dict[str, Any],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    aircraft_id: int,
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
) -> Optional[Dict[str, Any]]:
    if target_index is None:
        emit(f"[ATTACK][LAH] Target mission index unavailable for aircraft {aircraft_id}.")
        return None

    current_coord = _normalize_coordinate(state.get("coordinate"))
    if not current_coord:
        emit(f"[ATTACK][LAH] Coordinate missing for aircraft {aircraft_id}.")
        return None
    heading = _to_float(state.get("heading"))
    if heading is None:
        emit(f"[ATTACK][LAH] Heading missing for aircraft {aircraft_id}; defaulting to north.")
        heading = 0.0

    entry_coord = _project_coordinate(current_coord, heading, ATTACK_ENTRY_OFFSET_METERS) or dict(current_coord)
    entry_alt = _normalize_altitude_value(entry_coord.get("altitude")) or _normalize_altitude_value(current_coord.get("altitude")) or 800
    entry_coord["altitude"] = entry_alt

    attack_coord_norm = _normalize_coordinate(attack_coord)
    if not attack_coord_norm:
        emit("[ATTACK][LAH] Attack coordinate unavailable for manned aircraft.")
        return None
    attack_alt = _normalize_altitude_value(attack_coord_norm.get("altitude"))
    if attack_alt is None:
        attack_alt = entry_alt
    attack_coord_norm["altitude"] = attack_alt

    attack_path_id, resume_path_id = _reserve_path_ids(aircraft_id, 2)
    attack_individual_id, resume_individual_id = _reserve_individual_mission_ids(2)
    entry_wp_id = _next_waypoint_id()
    attack_wp_id = _next_waypoint_id()

    attack_target_id = _to_int(descriptor.get("target_id"))
    if attack_target_id is None:
        detail = ctx.get("replan_detail") if isinstance(ctx, dict) else {}
        if isinstance(detail, dict):
            attack_target_id = _to_int(detail.get("targetID") or detail.get("targetId"))
            if attack_target_id is None:
                orient = detail.get("targetOrientation") or {}
                if isinstance(orient, dict):
                    attack_target_id = _to_int(orient.get("targetID") or orient.get("targetId"))
    attack_target_id_value = attack_target_id if attack_target_id is not None else 0

    rel_info = dict(target_mission.get("relatedMission") or {})
    input_mission_id = _to_int(rel_info.get("inputMissionID")) or _to_int((ctx.get("mission_ids") or [None])[0]) or 0
    prior_mission_id = _to_int(rel_info.get("priorMissionID")) or 0
    related_template = {
        "relatedMissionType": 1,
        "inputMissionID": input_mission_id,
        "priorMissionID": prior_mission_id,
    }

    template_wp = deepcopy((fp_data.get("lahWaypointList") or [None])[0]) if fp_data.get("lahWaypointList") else _default_lah_waypoint_template()
    _, resume_waypoints, removed_wp_id = _split_done_resume_lah_path(
        fp_data,
        artifacts=artifacts,
        current_coord=current_coord,
        emit=emit,
        force_nonempty_resume=True,
        exclude_current_from_resume=True,
    )

    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    source_mission_list = imp_data.get("individualMissionList")
    done_input_ids = _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    if (
        isinstance(source_mission_list, list)
        and 0 <= target_index < len(source_mission_list)
    ):
        cloned_artifacts = _clone_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][LAH]",
            excluded_input_ids=done_input_ids,
        )
        if cloned_artifacts is None:
            return None
        follow_up_missions, follow_up_paths = cloned_artifacts

    resume_target_coord = _extract_lah_waypoint_coordinate(resume_waypoints[0]) if resume_waypoints else None
    if resume_target_coord is not None:
        resume_bearing = _bearing_between(
            float(attack_coord_norm["latitude"]),
            float(attack_coord_norm["longitude"]),
            float(resume_target_coord["latitude"]),
            float(resume_target_coord["longitude"]),
        )
        resume_start_coord = dict(attack_coord_norm)
        projected_resume = _project_coordinate(
            attack_coord_norm,
            resume_bearing,
            ATTACK_RESUME_OFFSET_METERS,
        )
        if projected_resume:
            resume_start_coord.update(
                {
                    "latitude": projected_resume.get("latitude", resume_start_coord.get("latitude")),
                    "longitude": projected_resume.get("longitude", resume_start_coord.get("longitude")),
                }
            )
        resume_start_coord["altitude"] = attack_coord_norm.get("altitude")
        resume_waypoints = _prepend_lah_transition_waypoint(
            resume_waypoints,
            template_wp=template_wp,
            anchor_coord=resume_start_coord,
        )
    has_resume = bool(resume_waypoints)

    mission_attack = {
        "individualMissionID": attack_individual_id,
        "isDone": False,
        "relatedMission": dict(related_template),
        "individualMissionInfo": {
            "individualMissionType": 2,
            "patternType": 2,
            "autoZoomIn": False,
            "targetID": attack_target_id_value,
            "coordinateList": [
                {
                    "latitude": entry_coord["latitude"],
                    "longitude": entry_coord["longitude"],
                    "altitude": entry_coord["altitude"],
                },
                {
                    "latitude": attack_coord_norm["latitude"],
                    "longitude": attack_coord_norm["longitude"],
                    "altitude": attack_coord_norm["altitude"],
                },
            ],
        },
        "pathID": attack_path_id,
    }

    entry_wp = _build_lah_waypoint_from_template(template_wp, entry_wp_id, entry_coord, attack_wp_id, mark_attack=False, target_id=None)
    attack_wp = _build_lah_waypoint_from_template(
        template_wp,
        attack_wp_id,
        attack_coord_norm,
        0,
        mark_attack=True,
        target_id=attack_target_id,
    )
    attack_fp_data = {
        "timestamp": now_ms,
        "Source": _extract_path_source(fp_data),
        "pathID": attack_path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": attack_individual_id,
        "lahWaypointList": [entry_wp, attack_wp],
    }

    original_entry = deepcopy(target_mission)

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = dict(related_template)
    mission_resume["isDone"] = False
    mission_resume_info = mission_resume.get("individualMissionInfo")
    if isinstance(mission_resume_info, dict):
        mission_resume["individualMissionInfo"] = deepcopy(mission_resume_info)
        mission_resume["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = _extract_path_source(fp_data)
    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = resume_individual_id
    resume_fp_data["lahWaypointList"] = resume_waypoints

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    if 0 <= target_index < len(mission_list):
        rebuilt = [mission_attack]
        if has_resume:
            rebuilt.append(mission_resume)
        rebuilt.extend(follow_up_missions)
        mission_list[:] = rebuilt
        emit(
            "[ATTACK][LAH] Dropped completed missions, inserted attack, and reattached "
            f"{len(follow_up_missions)} follow-up mission(s)."
        )
    else:
        mission_list.insert(0, mission_attack)
        if has_resume:
            mission_list.append(mission_resume)
        mission_list.extend(follow_up_missions)
        emit(f"[ATTACK][LAH] Target mission index invalid; appended attack branch at head (aircraft {aircraft_id}).")

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    attack_fp_dest = db_paths.get_db_subpath("FlightPath", f"{attack_path_id}.json")
    resume_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json") if has_resume else None
    )
    write_targets = [imp_dest, attack_fp_dest]
    if resume_fp_dest is not None:
        write_targets.append(resume_fp_dest)
    write_targets.extend(dest for dest, _ in follow_up_paths)
    for path in write_targets:
        path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(imp_dest, imp_data)
    _write_json_file(attack_fp_dest, attack_fp_data)
    if resume_fp_dest is not None:
        _write_json_file(resume_fp_dest, resume_fp_data)
    for dest, payload in follow_up_paths:
        _write_json_file(dest, payload)

    emit(
        "[ATTACK][LAH] Generated attack/resume missions -> "
        f"IMP:{imp_dest.name} PATHS:{attack_fp_dest.name}"
        f"{'/' + resume_fp_dest.name if resume_fp_dest is not None else ''} "
        f"(followUps={len(follow_up_missions)})"
    )

    result: Dict[str, Any] = {
        "aircraft_id": aircraft_id,
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "attack": {
            "individualMissionID": attack_individual_id,
            "pathID": attack_path_id,
            "waypointIDs": [entry_wp_id, attack_wp_id],
        },
        "removedWaypointID": removed_wp_id,
        "attackPath": str(attack_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
        "attackEntryCoordinate": dict(entry_coord),
        "attackCoordinate": dict(attack_coord_norm),
    }
    if resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
        result["resumePath"] = str(resume_fp_dest)
    return result


def _build_lah_hold_resume_package(
    *,
    descriptor: Dict[str, Any],
    new_imp_id: int,
    imp_data: Dict[str, Any],
    fp_data: Dict[str, Any],
    target_mission: Dict[str, Any],
    target_index: Optional[int],
    ctx: Dict[str, Any],
    state: Dict[str, Any],
    aircraft_id: int,
    artifacts: Any,
    emit: Callable[[str], None],
    now_ms: int,
) -> Optional[Dict[str, Any]]:
    if target_index is None:
        emit(f"[ATTACK][LAH] Target mission index unavailable for aircraft {aircraft_id}.")
        return None

    current_coord = _normalize_coordinate(state.get("coordinate"))
    if current_coord is None:
        current_coord = _extract_final_lah_coordinate(fp_data)
    if current_coord is None:
        emit(f"[ATTACK][LAH] Hold/resume coordinate missing for aircraft {aircraft_id}.")
        return None

    hold_path_id, resume_path_id = _reserve_path_ids(aircraft_id, 2)
    hold_individual_id, resume_individual_id = _reserve_individual_mission_ids(2)
    template_wp = deepcopy((fp_data.get("lahWaypointList") or [None])[0]) if fp_data.get("lahWaypointList") else _default_lah_waypoint_template()

    _, resume_waypoints, removed_wp_id = _split_done_resume_lah_path(
        fp_data,
        artifacts=artifacts,
        current_coord=current_coord,
        emit=emit,
        force_nonempty_resume=True,
        exclude_current_from_resume=True,
    )
    has_resume = bool(resume_waypoints)
    hold_coord = _build_lah_hold_coordinate_near_resume(
        resume_waypoints=resume_waypoints,
        current_coord=current_coord,
    )
    if hold_coord is None:
        hold_coord = current_coord

    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    source_mission_list = imp_data.get("individualMissionList")
    done_input_ids = _load_done_input_ids_for_plan(int(artifacts.source_plan_id))
    if (
        isinstance(source_mission_list, list)
        and 0 <= target_index < len(source_mission_list)
    ):
        cloned_artifacts = _clone_follow_up_replan_artifacts(
            missions=source_mission_list[target_index + 1 :],
            aircraft_id=descriptor["aircraft_id"],
            now_ms=now_ms,
            emit=emit,
            log_prefix="[ATTACK][LAH]",
            excluded_input_ids=done_input_ids,
        )
        if cloned_artifacts is None:
            return None
        follow_up_missions, follow_up_paths = cloned_artifacts

    original_entry = deepcopy(target_mission)
    rel_info = dict(original_entry.get("relatedMission") or {})
    input_mission_id = _to_int(rel_info.get("inputMissionID")) or _to_int((ctx.get("mission_ids") or [None])[0]) or 0
    prior_mission_id = _to_int(rel_info.get("priorMissionID")) or 0
    related_template = {
        "relatedMissionType": rel_info.get("relatedMissionType", 1),
        "inputMissionID": input_mission_id,
        "priorMissionID": prior_mission_id,
    }

    mission_hold = {
        "individualMissionID": hold_individual_id,
        "isDone": False,
        "relatedMission": dict(related_template),
        "individualMissionInfo": {
            "individualMissionType": 9,
            "patternType": 12,
            "autoZoomIn": False,
            "coordinateList": [dict(hold_coord)],
            "targetID": None,
        },
        "pathID": hold_path_id,
    }

    mission_resume = deepcopy(original_entry)
    mission_resume["individualMissionID"] = resume_individual_id
    mission_resume["pathID"] = resume_path_id
    mission_resume["relatedMission"] = dict(related_template)
    mission_resume["isDone"] = False
    mission_resume_info = mission_resume.get("individualMissionInfo")
    if isinstance(mission_resume_info, dict):
        mission_resume["individualMissionInfo"] = deepcopy(mission_resume_info)
        mission_resume["individualMissionInfo"]["coordinateList"] = _lah_waypoints_to_coordinate_list(resume_waypoints)

    hold_wp = _build_lah_anchor_waypoint(
        template_wp,
        coord=hold_coord,
        next_id=0,
        hovering_time=LAH_HOLD_SECONDS,
    )
    hold_fp_data = {
        "timestamp": now_ms,
        "Source": _extract_path_source(fp_data),
        "pathID": hold_path_id,
        "aircraftID": aircraft_id,
        "individualMissionID": hold_individual_id,
        "lahWaypointList": [hold_wp],
    }

    resume_fp_data = deepcopy(fp_data)
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = _extract_path_source(fp_data)
    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = resume_individual_id
    resume_fp_data["lahWaypointList"] = resume_waypoints

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    mission_list = imp_data.get("individualMissionList")
    if not isinstance(mission_list, list):
        mission_list = []
        imp_data["individualMissionList"] = mission_list
    if 0 <= target_index < len(mission_list):
        rebuilt = [mission_hold]
        if has_resume:
            rebuilt.append(mission_resume)
        rebuilt.extend(follow_up_missions)
        mission_list[:] = rebuilt
        emit(
            "[ATTACK][LAH] Dropped completed missions, inserted hold, and reattached "
            f"{len(follow_up_missions)} follow-up mission(s)."
        )
    else:
        mission_list.insert(0, mission_hold)
        if has_resume:
            mission_list.append(mission_resume)
        mission_list.extend(follow_up_missions)
        emit(
            f"[ATTACK][LAH] Target mission index invalid; appended hold branch at head (aircraft {aircraft_id})."
        )

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    hold_fp_dest = db_paths.get_db_subpath("FlightPath", f"{hold_path_id}.json")
    resume_fp_dest = (
        db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json") if has_resume else None
    )
    write_targets = [imp_dest, hold_fp_dest]
    if resume_fp_dest is not None:
        write_targets.append(resume_fp_dest)
    write_targets.extend(dest for dest, _ in follow_up_paths)
    for path in write_targets:
        path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_file(imp_dest, imp_data)
    _write_json_file(hold_fp_dest, hold_fp_data)
    if resume_fp_dest is not None:
        _write_json_file(resume_fp_dest, resume_fp_data)
    for dest, payload in follow_up_paths:
        _write_json_file(dest, payload)

    emit(
        "[ATTACK][LAH] Generated hold/resume missions -> "
        f"IMP:{imp_dest.name} PATHS:{hold_fp_dest.name}"
        f"{'/' + resume_fp_dest.name if resume_fp_dest is not None else ''} "
        f"(followUps={len(follow_up_missions)})"
    )

    result: Dict[str, Any] = {
        "aircraft_id": aircraft_id,
        "role": descriptor["label"],
        "individualMissionPackageID": new_imp_id,
        "hold": {
            "individualMissionID": hold_individual_id,
            "pathID": hold_path_id,
            "waypointID": _to_int(hold_wp.get("waypointID")),
            "durationSeconds": LAH_HOLD_SECONDS,
        },
        "removedWaypointID": removed_wp_id,
        "holdPath": str(hold_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
    }
    if resume_fp_dest is not None:
        result["resume"] = {
            "individualMissionID": resume_individual_id,
            "pathID": resume_path_id,
        }
        result["resumePath"] = str(resume_fp_dest)
    return result


def _build_lah_waypoint_from_template(
    template: Dict[str, Any],
    waypoint_id: int,
    coord: Dict[str, Any],
    next_id: int,
    *,
    mark_attack: bool,
    target_id: Optional[int],
) -> Dict[str, Any]:
    waypoint = deepcopy(template)
    waypoint["waypointID"] = waypoint_id
    waypoint["nextWaypointID"] = next_id or 0
    coordinate = dict(template.get("coordinate") or {})
    coordinate["latitude"] = coord.get("latitude")
    coordinate["longitude"] = coord.get("longitude")
    coordinate["altitude"] = _normalize_altitude_value(coord.get("altitude")) or coordinate.get("altitude") or 800
    waypoint["coordinate"] = coordinate
    waypoint["speed"] = _to_float(template.get("speed")) or 30.0
    attack_block = dict(template.get("attack") or {"targetID": 0, "weaponType": 0})
    if mark_attack:
        attack_block["targetID"] = _to_int(target_id) or 0
        attack_block["weaponType"] = ATTACK_WEAPON_TYPE
    else:
        attack_block["targetID"] = 0
        attack_block["weaponType"] = attack_block.get("weaponType", 0)
    waypoint["attack"] = attack_block
    return waypoint


def _default_lah_waypoint_template() -> Dict[str, Any]:
    return {
        "waypointID": 0,
        "coordinate": {"latitude": 0.0, "longitude": 0.0, "altitude": 800},
        "speed": 40.0,
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "hovering": {"time": 0},
        "loiter": {"radius": 0, "direction": 0, "time": 0, "speed": 0},
        "attack": {"targetID": 0, "weaponType": 0},
    }


def _extract_final_lah_coordinate(fp_data: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    waypoints = fp_data.get("lahWaypointList") or []
    if not waypoints:
        return None
    last = waypoints[-1].get("coordinate")
    return _normalize_coordinate(last)


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


def _update_plan_aircraft_entry(
    plan_data: Dict[str, Any],
    aircraft_id: int,
    new_package_id: int,
    emit: Callable[[str], None],
) -> bool:
    for entry in plan_data.get("aircraftList", []):
        if _to_int(entry.get("aircraftID")) == aircraft_id:
            entry["individualMissionPackageID"] = new_package_id
            return True
    emit(f"[ATTACK] Aircraft {aircraft_id} not found in mission plan.")
    return False


def _write_json_file(path: Path, data: Dict[str, Any]) -> None:
    write_json(path, data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)


def _now_timestamp_ms() -> int:
    epoch = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.now(timezone.utc) - epoch).total_seconds() * 1000)

