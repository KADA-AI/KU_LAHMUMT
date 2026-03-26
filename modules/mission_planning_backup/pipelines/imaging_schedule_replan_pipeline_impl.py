from __future__ import annotations

import json
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set

from modules.common import db_paths, imaging_schedule_replan_store
from modules.mission_planning.runtime.json_io import write_json
from modules.mission_planning.pipelines.mission_path_trim import (
    count_sweep_points_in_waypoints,
    load_sweep_progress,
    reassign_unique_waypoint_ids_inplace,
    scale_line_search_speed,
    sweep_cut_points,
    trim_waypoints_by_sweep_points,
)
from modules.mission_planning.pipelines.prior_mission_pipeline_impl import (
    _next_waypoint_id,
    _now_ms_since_2000,
    _to_float,
    _reserve_imp_ids,
    _reserve_individual_mission_ids,
    _reserve_path_ids,
    _resolve_plan_artifacts,
    _to_int,
)


DEFAULT_OPTION_NAME = "비행/촬영"
IMAGING_TRIGGER_TYPE = "imagingScheduleDeviation"
QUALITY_TRIGGER_TYPE = "qualityMonitorSep"


@dataclass
class ImagingSchedulePipelineResult:
    plan_ids: List[int]
    option_names: List[str]
    plan_meta_map: Dict[int, Dict[str, Any]]
    generated_imp_ids: Set[int]
    generated_path_ids: Set[int]
    new_imp_id: int
    new_path_id: int
    new_individual_id: int
    replaced_waypoint_id: int
    new_waypoint_id: int
    log_path: Path
    trigger_type: str = IMAGING_TRIGGER_TYPE
    removed_waypoint_id: Optional[int] = None
    anchor_waypoint_id: Optional[int] = None
    search_speed_scale: Optional[float] = None
    speed_adjustment_direction: Optional[str] = None
    trimmed_sweep_points: int = 0


def warm_imaging_schedule_replan_pipeline() -> Dict[str, Any]:
    return {"ready": True}


def _ensure_option_names(plan_ids: List[int], option_names: List[str] | None) -> List[str]:
    names = [str(name) for name in (option_names or []) if name is not None]
    if not names:
        names = [DEFAULT_OPTION_NAME]
    while len(names) < len(plan_ids):
        names.append(names[-1])
    return names[: len(plan_ids)]


def _collect_option_names(option_names: List[str] | None) -> List[str]:
    return [str(name) for name in (option_names or []) if name is not None]


def _set_source_field(payload: Dict[str, Any], source: str) -> None:
    if "Source" in payload or "source" not in payload:
        payload["Source"] = str(payload.get("Source") or payload.get("source") or source)
    else:
        payload["source"] = str(payload.get("source") or payload.get("Source") or source)


def _coerce_float(value: Any) -> Optional[float]:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _normalize_coordinate(value: object | None) -> Optional[Dict[str, float]]:
    if not isinstance(value, dict):
        return None
    lat = _coerce_float(value.get("latitude") or value.get("Latitude"))
    lon = _coerce_float(value.get("longitude") or value.get("Longitude"))
    alt = _coerce_float(value.get("altitude") or value.get("Altitude"))
    if lat is None or lon is None:
        return None
    out: Dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        out["altitude"] = float(alt)
    return out


def _load_detail_from_store(plan_ids: List[int]) -> Optional[Dict[str, Any]]:
    for value in plan_ids:
        try:
            plan_id = int(value)
        except Exception:
            continue
        payload = imaging_schedule_replan_store.load_detail(plan_id)
        if payload:
            return payload
    return None


def _build_quality_replanned_waypoints(
    waypoint_list: List[Dict[str, Any]],
    *,
    artifacts: Any,
    current_coordinate: Optional[Dict[str, float]],
    search_speed_scale: float,
    emit: Callable[[str], None],
) -> tuple[List[Dict[str, Any]], Optional[int], Optional[int], int]:
    waypoints = [deepcopy(item) for item in (waypoint_list or []) if isinstance(item, dict)]
    if not waypoints:
        raise RuntimeError("source path has no waypointList")

    done_waypoints: List[Dict[str, Any]] = []
    resume_waypoints: List[Dict[str, Any]] = []
    removed_waypoint_id: Optional[int] = None
    anchor_waypoint_id: Optional[int] = None
    anchor_wp_ref: Optional[Dict[str, Any]] = None

    curr_wp = _to_int(getattr(artifacts, "current_waypoint_id", None))
    prev_wp = _to_int(getattr(artifacts, "previous_waypoint_id", None))
    curr_idx = next(
        (idx for idx, wp in enumerate(waypoints) if _to_int((wp or {}).get("waypointID")) == curr_wp),
        None,
    )

    if curr_idx is not None:
        done_waypoints = deepcopy(waypoints[:curr_idx]) if curr_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[curr_idx:])
        if done_waypoints:
            removed_waypoint_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_waypoint_id = prev_wp
        if removed_waypoint_id is not None:
            emit(f"[QUALITY] Resume trimmed by currentWP (lastRemovedWP={removed_waypoint_id}).")
    elif any(bool((wp or {}).get("isDone")) for wp in waypoints):
        idx = 0
        while idx < len(waypoints) and bool((waypoints[idx] or {}).get("isDone")):
            idx += 1
        done_waypoints = deepcopy(waypoints[:idx]) if idx > 0 else []
        resume_waypoints = deepcopy(waypoints[idx:]) if idx > 0 else deepcopy(waypoints)
        if done_waypoints:
            removed_waypoint_id = _to_int(done_waypoints[-1].get("waypointID"))
        if removed_waypoint_id is not None:
            emit(f"[QUALITY] Resume trimmed by isDone (lastRemovedWP={removed_waypoint_id}).")
    else:
        done_waypoints = []
        resume_waypoints = deepcopy(waypoints)
        removed_waypoint_id = prev_wp

    if not resume_waypoints and waypoints:
        forced_start_idx: Optional[int] = None
        if prev_wp is not None:
            for idx, waypoint in enumerate(waypoints):
                if _to_int((waypoint or {}).get("waypointID")) == prev_wp:
                    forced_start_idx = min(idx + 1, len(waypoints) - 1)
                    break
        if forced_start_idx is None:
            for idx, waypoint in enumerate(waypoints):
                if not bool((waypoint or {}).get("isDone")):
                    forced_start_idx = idx
                    break
        if forced_start_idx is None:
            forced_start_idx = len(waypoints) - 1
        done_waypoints = deepcopy(waypoints[:forced_start_idx]) if forced_start_idx > 0 else []
        resume_waypoints = deepcopy(waypoints[forced_start_idx:])
        if done_waypoints:
            removed_waypoint_id = _to_int(done_waypoints[-1].get("waypointID"))
        elif prev_wp is not None:
            removed_waypoint_id = prev_wp
        emit(
            "[QUALITY] Resume fallback applied "
            f"(forcedStartWP={_to_int((resume_waypoints[0] or {}).get('waypointID'))})."
        )

    done_sweep_points = count_sweep_points_in_waypoints(done_waypoints)
    if done_waypoints and resume_waypoints and isinstance(current_coordinate, dict):
        anchor_lat = _to_float(current_coordinate.get("latitude"))
        anchor_lon = _to_float(current_coordinate.get("longitude"))
        anchor_alt = _to_float(current_coordinate.get("altitude"))
        if anchor_alt is None:
            anchor_alt = _to_float(((done_waypoints[-1].get("coordinate") or {}) if done_waypoints else {}).get("altitude"))
        if anchor_alt is None:
            anchor_alt = _to_float(((resume_waypoints[0].get("coordinate") or {}) if resume_waypoints else {}).get("altitude"))
        if anchor_alt is None:
            anchor_alt = 0.0

        if anchor_lat is not None and anchor_lon is not None:
            prev_coord = done_waypoints[-1].get("coordinate") if isinstance(done_waypoints[-1], dict) else {}
            prev_lat = _to_float((prev_coord or {}).get("latitude")) if isinstance(prev_coord, dict) else None
            prev_lon = _to_float((prev_coord or {}).get("longitude")) if isinstance(prev_coord, dict) else None
            prev_alt = _to_float((prev_coord or {}).get("altitude")) if isinstance(prev_coord, dict) else None
            same_as_prev = (
                prev_lat is not None
                and prev_lon is not None
                and abs(prev_lat - anchor_lat) <= 1e-7
                and abs(prev_lon - anchor_lon) <= 1e-7
                and abs((prev_alt or 0.0) - float(anchor_alt)) <= 1e-6
            )
            if not same_as_prev:
                template_wp = done_waypoints[-1] if isinstance(done_waypoints[-1], dict) else {}
                anchor_waypoint_id = _next_waypoint_id()
                anchor_wp: Dict[str, Any] = {
                    "waypointID": int(anchor_waypoint_id),
                    "coordinate": {
                        "latitude": float(anchor_lat),
                        "longitude": float(anchor_lon),
                        "altitude": float(anchor_alt),
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
                                "latitude": float(anchor_lat),
                                "longitude": float(anchor_lon),
                                "altitude": float(anchor_alt),
                            }
                            anchor_fp["coordinateOrientation"] = coord_orient
                        anchor_wp["filmingProperty"] = anchor_fp
                if "loiterProperty" in template_wp:
                    template_loiter = template_wp.get("loiterProperty")
                    if isinstance(template_loiter, dict) and template_loiter:
                        anchor_wp["loiterProperty"] = deepcopy(template_loiter)
                done_waypoints.append(anchor_wp)
                anchor_wp_ref = anchor_wp
                emit(
                    "[QUALITY] Added replan anchor waypoint to done path "
                    f"(anchorWP={anchor_waypoint_id})."
                )

    progress_entry = None
    sweep_progress = load_sweep_progress()
    if sweep_progress and getattr(artifacts, "path_id", None) is not None:
        progress_entry = sweep_progress.get(int(artifacts.path_id))
    raw_cut_points = sweep_cut_points(progress_entry)
    cut_points = max(0, int(raw_cut_points) - int(done_sweep_points))
    trimmed_sweep_points = 0
    if cut_points > 0 and resume_waypoints:
        resume_waypoints, trimmed_sweep_points = trim_waypoints_by_sweep_points(
            resume_waypoints,
            cut_points,
            preserve_waypoints=True,
        )
        if trimmed_sweep_points > 0:
            emit(
                f"[QUALITY] Resume sweep trim applied "
                f"(cutPoints={trimmed_sweep_points}, rawCutPoints={raw_cut_points}, "
                f"doneSweepPoints={done_sweep_points}, pathID={getattr(artifacts, 'path_id', None)})."
            )

    for wp in done_waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = True
    for wp in resume_waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = False

    if resume_waypoints:
        scaled = scale_line_search_speed(resume_waypoints, search_speed_scale)
        if scaled > 0:
            emit(
                f"[QUALITY] searchSpeed scaled "
                f"(factor={float(search_speed_scale):.2f}, waypoints={scaled})."
            )

    new_waypoints = done_waypoints + resume_waypoints
    if not new_waypoints:
        raise RuntimeError("quality resume path became empty after trimming")
    reassign_unique_waypoint_ids_inplace(new_waypoints)
    if isinstance(anchor_wp_ref, dict):
        anchor_waypoint_id = _to_int(anchor_wp_ref.get("waypointID"))
    return new_waypoints, removed_waypoint_id, anchor_waypoint_id, int(trimmed_sweep_points)


def _build_replanned_waypoints(
    waypoint_list: List[Dict[str, Any]],
    *,
    current_waypoint_id: int,
    replacement_waypoint_id: Optional[int],
    emit: Callable[[str], None],
) -> tuple[List[Dict[str, Any]], int, int]:
    current_index = None
    for idx, waypoint in enumerate(waypoint_list):
        if _to_int((waypoint or {}).get("waypointID")) == int(current_waypoint_id):
            current_index = idx
            break
    if current_index is None:
        emit(f"[IMGSCH] currentWaypointID {current_waypoint_id} not found in source FlightPath.")
        raise RuntimeError("current waypoint not found in source path")

    current_waypoint = deepcopy(waypoint_list[current_index])
    remaining_waypoints = [deepcopy(item) for item in waypoint_list[current_index + 1 :]]
    existing_ids = {
        int(wp_id)
        for wp_id in (_to_int((item or {}).get("waypointID")) for item in waypoint_list)
        if wp_id is not None and int(wp_id) > 0
    }
    candidate_waypoint_id = _to_int(replacement_waypoint_id)
    if (
        candidate_waypoint_id is None
        or candidate_waypoint_id <= 0
        or candidate_waypoint_id == int(current_waypoint_id)
        or candidate_waypoint_id in existing_ids
    ):
        candidate_waypoint_id = _next_waypoint_id()
    current_waypoint["waypointID"] = int(candidate_waypoint_id)
    current_waypoint["isDone"] = False

    new_waypoints = [current_waypoint] + remaining_waypoints
    for waypoint in new_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    reassign_unique_waypoint_ids_inplace(new_waypoints)
    new_waypoint_id = _to_int(current_waypoint.get("waypointID")) or int(candidate_waypoint_id)
    return new_waypoints, int(current_waypoint_id), int(new_waypoint_id)


def run_imaging_schedule_replan_pipeline(
    ctx: Dict[str, Any],
    detail: Dict[str, Any],
    reason: str,
    *,
    log: Callable[[str], None],
) -> Optional[ImagingSchedulePipelineResult]:
    log_messages: List[str] = []

    def emit(message: str) -> None:
        log_messages.append(message)
        log(message)

    plan_ids_raw = list(ctx.get("plan_ids") or [])
    try:
        plan_ids = [int(value) for value in plan_ids_raw if value is not None]
    except Exception:
        emit(f"[IMGSCH] invalid plan_ids in context: {plan_ids_raw!r}")
        return None
    if len(plan_ids) != 1:
        emit(f"[IMGSCH] expected exactly one pending missionPlanID, got {len(plan_ids)}.")
        return None

    stored_detail = _load_detail_from_store(plan_ids)
    if not isinstance(detail, dict) or not detail:
        detail = stored_detail or {}
    elif stored_detail:
        merged_detail = dict(stored_detail)
        merged_detail.update(detail)
        detail = merged_detail
    if not isinstance(detail, dict) or not detail:
        emit("[IMGSCH] replanDetail missing and store lookup failed.")
        try:
            imaging_schedule_replan_store.save_event(
                "mission_pipeline_missing_detail",
                {
                    "planIDs": list(plan_ids),
                    "reason": str(reason or ""),
                    "context": dict(ctx or {}),
                },
            )
        except Exception:
            pass
        return None

    source_plan_id = _to_int(detail.get("sourceMissionPlanID"))
    aircraft_id = _to_int(detail.get("aircraftID"))
    current_waypoint_id = _to_int(detail.get("currentWaypointID"))
    replacement_waypoint_id = _to_int(detail.get("replacementWaypointID"))
    trigger_type = str(detail.get("triggerType") or IMAGING_TRIGGER_TYPE).strip() or IMAGING_TRIGGER_TYPE
    is_quality_trigger = trigger_type == QUALITY_TRIGGER_TYPE
    log_prefix = "[QUALITY]" if is_quality_trigger else "[IMGSCH]"
    if current_waypoint_id is None or current_waypoint_id <= 0:
        emit(f"{log_prefix} currentWaypointID is 0/invalid -> not a replan target.")
        return None
    if source_plan_id is None or source_plan_id <= 0 or aircraft_id is None or aircraft_id <= 0:
        emit(
            f"{log_prefix} required source identifiers missing "
            f"(sourcePlan={source_plan_id}, aircraftID={aircraft_id})."
        )
        return None

    artifacts = _resolve_plan_artifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        current_waypoint_id=int(current_waypoint_id),
        emit=emit,
        allow_first_mission_fallback=False,
    )
    if artifacts is None:
        emit(f"{log_prefix} failed to resolve source mission artifacts.")
        return None

    plan_src = db_paths.get_db_subpath("MissionPlan", f"{source_plan_id}.json")
    imp_src = db_paths.get_db_subpath(
        "IndividualMissionPlan", f"{artifacts.individual_mission_package_id}.json"
    )
    fp_src = db_paths.get_db_subpath("FlightPath", f"{artifacts.path_id}.json")

    try:
        plan_data = json.loads(plan_src.read_text(encoding="utf-8"))
        imp_data = json.loads(imp_src.read_text(encoding="utf-8"))
        fp_data = json.loads(fp_src.read_text(encoding="utf-8"))
    except Exception as exc:
        emit(f"{log_prefix} failed to load source artifacts: {exc}")
        return None

    new_plan_id = int(plan_ids[0])
    [new_imp_id] = _reserve_imp_ids(1)
    [new_individual_id] = _reserve_individual_mission_ids(1)
    [new_path_id] = _reserve_path_ids(int(aircraft_id), 1)
    option_names = (
        _collect_option_names(ctx.get("option_names"))
        if is_quality_trigger
        else _ensure_option_names([new_plan_id], ctx.get("option_names"))
    )
    now_ms = _now_ms_since_2000()

    new_plan_data = deepcopy(plan_data)
    new_plan_data["missionPlanID"] = int(new_plan_id)
    new_plan_data["timestamp"] = int(now_ms)
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = int(now_ms)

    plan_aircraft_updated = False
    for aircraft_entry in new_plan_data.get("aircraftList") or []:
        if _to_int((aircraft_entry or {}).get("aircraftID")) == int(aircraft_id):
            aircraft_entry["individualMissionPackageID"] = int(new_imp_id)
            plan_aircraft_updated = True
            break
    if not plan_aircraft_updated:
        emit(f"{log_prefix} aircraft {aircraft_id} not found in source MissionPlan {source_plan_id}.")
        return None

    new_imp_data = deepcopy(imp_data)
    new_imp_data["individualMissionPackageID"] = int(new_imp_id)
    new_imp_data["timestamp"] = int(now_ms)
    _set_source_field(new_imp_data, "MMR")

    target_index = None
    mission_list = new_imp_data.get("individualMissionList") or []
    for idx, mission in enumerate(mission_list):
        if _to_int((mission or {}).get("individualMissionID")) == int(artifacts.individual_mission_id):
            target_index = idx
            break
    if target_index is None:
        emit(
            f"{log_prefix} individualMissionID {artifacts.individual_mission_id} "
            f"not found in package {artifacts.individual_mission_package_id}."
        )
        return None

    source_mission = deepcopy(mission_list[target_index])
    source_mission["individualMissionID"] = int(new_individual_id)
    source_mission["pathID"] = int(new_path_id)
    source_mission["isDone"] = False
    mission_list[target_index] = source_mission

    source_waypoints = list(fp_data.get("waypointList") or fp_data.get("lahWaypointList") or [])
    if not source_waypoints:
        emit(f"{log_prefix} FlightPath {artifacts.path_id} has no waypointList.")
        return None

    removed_waypoint_id: Optional[int] = None
    anchor_waypoint_id: Optional[int] = None
    trimmed_sweep_points = 0
    search_speed_scale = _coerce_float(detail.get("searchSpeedScale"))
    speed_adjustment_direction = str(detail.get("speedAdjustmentDirection") or "").strip() or None
    try:
        if is_quality_trigger:
            current_coordinate = _normalize_coordinate(
                detail.get("currentAircraftCoordinate")
                or detail.get("currentCoordinate")
                or detail.get("currentWaypointCoordinate")
            )
            if search_speed_scale is None or search_speed_scale <= 0.0:
                search_speed_scale = 1.0
            new_waypoints, removed_waypoint_id, anchor_waypoint_id, trimmed_sweep_points = _build_quality_replanned_waypoints(
                [deepcopy(item) for item in source_waypoints],
                artifacts=artifacts,
                current_coordinate=current_coordinate,
                search_speed_scale=float(search_speed_scale),
                emit=emit,
            )
            replaced_waypoint_id = int(removed_waypoint_id or current_waypoint_id)
            new_waypoint_id = int(anchor_waypoint_id or current_waypoint_id)
        else:
            new_waypoints, replaced_waypoint_id, new_waypoint_id = _build_replanned_waypoints(
                [deepcopy(item) for item in source_waypoints],
                current_waypoint_id=int(current_waypoint_id),
                replacement_waypoint_id=replacement_waypoint_id,
                emit=emit,
            )
    except Exception:
        return None

    new_fp_data = deepcopy(fp_data)
    new_fp_data["pathID"] = int(new_path_id)
    new_fp_data["timestamp"] = int(now_ms)
    new_fp_data["aircraftID"] = int(aircraft_id)
    new_fp_data["individualMissionID"] = int(new_individual_id)
    _set_source_field(new_fp_data, "MMR")
    new_fp_data["waypointList"] = new_waypoints
    if "lahWaypointList" in new_fp_data:
        new_fp_data["lahWaypointList"] = deepcopy(new_waypoints)

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    fp_dest = db_paths.get_db_subpath("FlightPath", f"{new_path_id}.json")
    for path in (plan_dest, imp_dest, fp_dest):
        path.parent.mkdir(parents=True, exist_ok=True)

    started_at = time.perf_counter()
    write_json(plan_dest, new_plan_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    write_json(imp_dest, new_imp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    write_json(fp_dest, new_fp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    elapsed_ms = (time.perf_counter() - started_at) * 1000.0
    emit(
        f"{log_prefix} stored replan artifacts -> "
        f"plan:{plan_dest.name}, imp:{imp_dest.name}, fp:{fp_dest.name} ({elapsed_ms:.1f} ms)"
    )

    log_dir = db_paths.get_db_subpath("DSS_Internal")
    log_dir.mkdir(parents=True, exist_ok=True)
    log_basename = "QualitySpeed" if is_quality_trigger else "ImagingSchedule"
    log_path = log_dir / f"{log_basename}_{aircraft_id}_{now_ms}.json"
    log_payload = {
        "timestamp": int(now_ms),
        "reason": str(reason or ""),
        "triggerType": str(trigger_type),
        "sourceMissionPlanID": int(source_plan_id),
        "aircraftID": int(aircraft_id),
        "sourceIndividualMissionPackageID": int(artifacts.individual_mission_package_id),
        "sourceIndividualMissionID": int(artifacts.individual_mission_id),
        "sourcePathID": int(artifacts.path_id),
        "currentWaypointID": int(current_waypoint_id),
        "replacedWaypointID": int(replaced_waypoint_id),
        "newWaypointID": int(new_waypoint_id),
        "removedWaypointID": int(removed_waypoint_id) if removed_waypoint_id is not None else None,
        "anchorWaypointID": int(anchor_waypoint_id) if anchor_waypoint_id is not None else None,
        "searchSpeedScale": float(search_speed_scale) if search_speed_scale is not None else None,
        "speedAdjustmentDirection": speed_adjustment_direction,
        "trimmedSweepPoints": int(trimmed_sweep_points),
        "generatedMissionPlanID": int(new_plan_id),
        "generatedIndividualMissionPackageID": int(new_imp_id),
        "generatedIndividualMissionID": int(new_individual_id),
        "generatedPathID": int(new_path_id),
        "logMessages": log_messages,
        "detail": dict(detail),
    }
    write_json(log_path, log_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
    emit(f"{log_prefix} log captured -> {log_path}")
    try:
        imaging_schedule_replan_store.save_event(
            "mission_pipeline_complete",
            {
                "triggerType": str(trigger_type),
                "generatedMissionPlanID": int(new_plan_id),
                "sourceMissionPlanID": int(source_plan_id),
                "aircraftID": int(aircraft_id),
                "replacedWaypointID": int(replaced_waypoint_id),
                "newWaypointID": int(new_waypoint_id),
                "removedWaypointID": int(removed_waypoint_id) if removed_waypoint_id is not None else None,
                "anchorWaypointID": int(anchor_waypoint_id) if anchor_waypoint_id is not None else None,
                "searchSpeedScale": float(search_speed_scale) if search_speed_scale is not None else None,
                "speedAdjustmentDirection": speed_adjustment_direction,
                "trimmedSweepPoints": int(trimmed_sweep_points),
                "logPath": str(log_path),
            },
        )
    except Exception:
        pass

    plan_meta_map = dict(ctx.get("_option_meta") or {})
    plan_meta_entry = plan_meta_map.setdefault(int(new_plan_id), {})
    plan_meta_entry.update(
        {
            "triggerType": str(trigger_type),
            "sourceMissionPlanID": int(source_plan_id),
            "aircraftID": int(aircraft_id),
            "individualMissionPackageID": int(new_imp_id),
            "individualMissionID": int(new_individual_id),
            "pathID": int(new_path_id),
            "replacedWaypointID": int(replaced_waypoint_id),
            "newWaypointID": int(new_waypoint_id),
            "removedWaypointID": int(removed_waypoint_id) if removed_waypoint_id is not None else None,
            "anchorWaypointID": int(anchor_waypoint_id) if anchor_waypoint_id is not None else None,
            "searchSpeedScale": float(search_speed_scale) if search_speed_scale is not None else None,
            "speedAdjustmentDirection": speed_adjustment_direction,
            "trimmedSweepPoints": int(trimmed_sweep_points),
            "logPath": str(log_path),
        }
    )

    return ImagingSchedulePipelineResult(
        plan_ids=[int(new_plan_id)],
        option_names=list(option_names),
        plan_meta_map=plan_meta_map,
        generated_imp_ids={int(new_imp_id)},
        generated_path_ids={int(new_path_id)},
        new_imp_id=int(new_imp_id),
        new_path_id=int(new_path_id),
        new_individual_id=int(new_individual_id),
        replaced_waypoint_id=int(replaced_waypoint_id),
        new_waypoint_id=int(new_waypoint_id),
        log_path=log_path,
        trigger_type=str(trigger_type),
        removed_waypoint_id=int(removed_waypoint_id) if removed_waypoint_id is not None else None,
        anchor_waypoint_id=int(anchor_waypoint_id) if anchor_waypoint_id is not None else None,
        search_speed_scale=float(search_speed_scale) if search_speed_scale is not None else None,
        speed_adjustment_direction=speed_adjustment_direction,
        trimmed_sweep_points=int(trimmed_sweep_points),
    )
