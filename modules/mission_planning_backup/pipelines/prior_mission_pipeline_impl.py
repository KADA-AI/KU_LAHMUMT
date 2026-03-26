from __future__ import annotations

import json
import math
import time
import importlib
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from modules.common import db_paths, agent_status_snapshot, prior_replan_store
from modules.mission_planning._paths import mission_planner_data_def_root
from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_prior_mission_profile
from modules.mission_planning.runtime.json_io import write_json
from modules.mission_planning.pipelines.mission_path_trim import (
    count_sweep_points_in_waypoints,
    load_sweep_progress,
    reassign_unique_waypoint_ids_inplace,
    scale_line_search_speed,
    sweep_cut_points,
    trim_waypoints_by_sweep_points,
    relink_waypoints,
)
import importlib.util
from types import ModuleType

_EPOCH_2000_MS = 946_684_800_000
_PRIOR_TRACKING_LOITER_SECONDS = 300
_PRIOR_DEFAULT_LOITER_SECONDS = 50
_PRIOR_APPROACH_BASE_OFFSET_M = 250.0  # was 100m
_PRIOR_APPROACH_FAR_OFFSET_M = 450.0   # was 300m
_RTB_FLIGHT_MODE = 5
_RESUME_SEARCH_SPEED_SCALE = 1.3
_ID_ALLOCATOR_MOD: Optional[ModuleType] = None
_MISSION_HELPERS_MOD: Optional[ModuleType] = None


def _load_id_allocator() -> ModuleType:
    global _ID_ALLOCATOR_MOD
    if _ID_ALLOCATOR_MOD is not None:
        return _ID_ALLOCATOR_MOD
    _ID_ALLOCATOR_MOD = importlib.import_module(
        "modules.mission_planning.MissionPlanner.data_def.id_allocator"
    )
    return _ID_ALLOCATOR_MOD


def _next_imp_id() -> int:
    return _load_id_allocator().next_imp_id()


def _next_individual_mission_id() -> int:
    return _load_id_allocator().next_individual_mission_id()


def _next_path_id(aircraft_id: int) -> int:
    return _load_id_allocator().next_path_id(aircraft_id)


def _reserve_imp_ids(count: int) -> List[int]:
    return [int(v) for v in _load_id_allocator().reserve_imp_ids(count)]


def _reserve_individual_mission_ids(count: int) -> List[int]:
    return [int(v) for v in _load_id_allocator().reserve_individual_mission_ids(count)]


def _reserve_path_ids(aircraft_id: int, count: int) -> List[int]:
    return [int(v) for v in _load_id_allocator().reserve_path_ids(aircraft_id, count)]


def _next_waypoint_id() -> int:
    return _load_id_allocator().next_waypoint_id()


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


def _orientation_altitude(lat: Optional[float], lon: Optional[float], *, fallback: int = 0) -> int:
    if lat is None or lon is None:
        return int(fallback)
    dem_alt = _sample_dem_altitude(float(lat), float(lon))
    dem_alt_int = _normalize_altitude_value(dem_alt)
    if dem_alt_int is None:
        return int(fallback)
    return dem_alt_int


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
    selected_agent_summary: Optional[AgentSnapshotSummary] = None
    selected_agent_distance_m: Optional[float] = None
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

        agent_snapshot_payload = agent_status_snapshot.load_agent_status_snapshot()
        agent_summaries = _summarize_agent_states(agent_snapshot_payload)
        _log_step1_agent_snapshot(emit, agent_snapshot_payload, agent_summaries)

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
                lat, lon, agent_summaries
            )
            _log_step3_nearest_agent(emit, selected_agent_summary, selected_agent_distance_m)

        if selected_agent_summary is None:
            emit("[PRIOR][STEP3] No eligible UAV found after excluding RTB aircraft.")
            return None

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
            plan_data = json.loads(plan_src.read_text(encoding="utf-8"))
            imp_data = json.loads(imp_src.read_text(encoding="utf-8"))
            fp_data = json.loads(fp_src.read_text(encoding="utf-8"))
        except Exception as exc:
            emit(f"[PRIOR] Failed to load source artifacts: {exc}")
            return None

        new_plan_id = plan_ids[0]
        [new_imp_id] = _reserve_imp_ids(1)
        prior_individual_id, resume_individual_id = _reserve_individual_mission_ids(2)
        done_path_id, prior_path_id, resume_path_id = _reserve_path_ids(aircraft_id, 3)
        prior_approach_wp_id = _next_waypoint_id()
        prior_target_wp_id = _next_waypoint_id()
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

        new_plan_data = deepcopy(plan_data)
        new_imp_data = deepcopy(imp_data)
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

        prior_rel_block = dict(base_rel_block)
        prior_rel_block["priorMissionID"] = prior_mission_id or 0
        prior_rel_block["relatedMissionType"] = 2
        if input_mission_id is not None:
            prior_rel_block["inputMissionID"] = input_mission_id

        resume_rel_block = dict(base_rel_block)
        resume_rel_block["priorMissionID"] = 0
        if input_mission_id is not None and "inputMissionID" not in resume_rel_block:
            resume_rel_block["inputMissionID"] = input_mission_id

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
        )
        if not resume_waypoints:
            emit("[PRIOR] FlightPath trimming produced an empty waypoint list.")
            return None

        has_done_segment = bool(done_waypoints)
        if not has_done_segment:
            done_path_id = None

        preserved_done_entry = None
        done_fp_data = None
        if has_done_segment and done_path_id is not None:
            preserved_done_entry = deepcopy(original_mission_template)
            preserved_done_entry["pathID"] = done_path_id
            preserved_done_entry["isDone"] = True

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
                _PRIOR_APPROACH_BASE_OFFSET_M,
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
                        _PRIOR_APPROACH_BASE_OFFSET_M,
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
        elif isinstance(agent_to_target_distance, (int, float)) and agent_to_target_distance > 400:
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
                    _PRIOR_APPROACH_FAR_OFFSET_M,
                )
                if approach_override:
                    approach_override["altitude"] = approach_alt
                    approach_coord = approach_override
                    emit(
                        "[PRIOR][STEP3] Approach waypoint adjusted: "
                        f"{agent_to_target_distance:.1f}m -> {_PRIOR_APPROACH_FAR_OFFSET_M:.0f}m ahead."
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
            100.0,
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

        approach_speed = 40.0
        target_speed = 30.0
        loiter_seconds = (
            _PRIOR_TRACKING_LOITER_SECONDS if mission_type == 2 else _PRIOR_DEFAULT_LOITER_SECONDS
        )
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
                "radius": prior_turn_radius_m,
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

        prefix_missions = deepcopy(mission_list[:target_index])
        suffix_missions = deepcopy(mission_list[target_index + 1 :])
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
        for summary in agent_summaries:
            aid = summary.aircraft_id
            if aid == aircraft_id:
                continue
            current_coord = None
            if summary.latitude is not None and summary.longitude is not None:
                current_coord = {
                    "latitude": summary.latitude,
                    "longitude": summary.longitude,
                }
                if summary.altitude is not None:
                    current_coord["altitude"] = summary.altitude
            update = _build_other_uav_resume_package(
                source_plan_id=source_plan_id,
                aircraft_id=aid,
                current_waypoint_id=summary.current_waypoint_id,
                current_coord=current_coord,
                emit=emit,
                now_ms=now_ms,
                sweep_progress=sweep_progress,
            )
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

        inserted_wp = target_wp
        inserted_wp_id = prior_target_wp_id

        plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
        imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
        done_fp_dest = (
            db_paths.get_db_subpath("FlightPath", f"{done_path_id}.json")
            if (done_path_id is not None and done_fp_data is not None)
            else None
        )
        prior_fp_dest = db_paths.get_db_subpath("FlightPath", f"{prior_path_id}.json")
        resume_fp_dest = db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json")
        write_targets = [plan_dest, imp_dest, prior_fp_dest, resume_fp_dest]
        if done_fp_dest is not None:
            write_targets.append(done_fp_dest)
        for path in write_targets:
            path.parent.mkdir(parents=True, exist_ok=True)
        write_json(plan_dest, new_plan_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        write_json(imp_dest, new_imp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        if done_fp_dest is not None and done_fp_data is not None:
            write_json(done_fp_dest, done_fp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        write_json(prior_fp_dest, prior_fp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        write_json(resume_fp_dest, resume_fp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        written_fp_names = [prior_fp_dest.name, resume_fp_dest.name]
        if done_fp_dest is not None:
            written_fp_names.insert(0, done_fp_dest.name)
        emit(
            "[PRIOR] Stored new artifacts -> "
            f"plan:{plan_dest.name}, imp:{imp_dest.name}, fp:{'/'.join(written_fp_names)}"
        )

        log_dir = db_paths.get_db_subpath("DSS_Internal")
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"PriorMission_{prior_mission_id or 0}_{now_ms}.json"
        log_payload = {
            "timestamp": now_ms,
            "reason": reason,
            "priorMissionID": prior_mission_id,
            "missionType": mission_type,
            "targetCoordinate": target_coord,
            "selectedAircraftID": aircraft_id,
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
            "logMessages": log_messages,
        }
        write_json(log_path, log_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
        emit(f"[PRIOR] Log captured -> {log_path}")

        plan_meta_map = dict(ctx.get("_option_meta") or {})
        plan_meta_entry = plan_meta_map.setdefault(new_plan_id, {})
        plan_meta_entry.update(
            {
                "priorMissionID": prior_mission_id,
                "sourceMissionPlanID": source_plan_id,
                "individualMissionPackageID": new_imp_id,
                "individualMissionID": prior_individual_id,
                "pathID": prior_path_id,
                "logPath": str(log_path),
                "removedWaypointID": removed_wp_id,
                "insertedWaypointID": prior_target_wp_id,
                "approachWaypointID": prior_approach_wp_id,
                "resumeIndividualMissionID": resume_individual_id,
                "resumePathID": resume_path_id,
                "targetCoordinate": target_coord,
            }
        )
        if done_path_id is not None:
            plan_meta_entry["donePathID"] = done_path_id

        success = True
        generated_path_ids: Set[int] = {prior_path_id, resume_path_id}.union(other_generated_path_ids)
        if done_path_id is not None:
            generated_path_ids.add(done_path_id)
        result = PriorMissionPipelineResult(
            plan_ids=plan_ids,
            option_names=option_names,
            plan_meta_map=plan_meta_map,
            generated_imp_ids={new_imp_id}.union(other_generated_imp_ids),
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


def _select_nearest_agent(
    target_lat: Optional[float],
    target_lon: Optional[float],
    summaries: List[AgentSnapshotSummary],
) -> Tuple[Optional[AgentSnapshotSummary], Optional[float]]:
    if target_lat is None or target_lon is None:
        return None, None
    best_agent: Optional[AgentSnapshotSummary] = None
    best_distance: Optional[float] = None
    for summary in summaries:
        if _is_rtb_agent(summary):
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
        "speed": 30.0,
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
            "radius": prior_turn_radius_m,
            "direction": 1,
            "time": 100,
            "speed": 30,
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
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], Optional[int]]:
    waypoints = list(resume_fp_data.get("waypointList") or [])
    done_waypoints: List[Dict[str, Any]] = []
    resume_waypoints: List[Dict[str, Any]] = []
    removed_wp_id: Optional[int] = None

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
            emit(f"[PRIOR][UAV] Resume trimmed by currentWP (lastRemovedWP={removed_wp_id}).")
    elif any(bool(wp.get("isDone")) for wp in waypoints):
        idx = 0
        while idx < len(waypoints) and bool(waypoints[idx].get("isDone")):
            idx += 1
        done_waypoints = deepcopy(waypoints[:idx]) if idx > 0 else []
        resume_waypoints = deepcopy(waypoints[idx:]) if idx > 0 else deepcopy(waypoints)
        if done_waypoints:
            removed_wp_id = _to_int(done_waypoints[-1].get("waypointID"))
        if removed_wp_id is not None:
            emit(f"[PRIOR][UAV] Resume trimmed by isDone (lastRemovedWP={removed_wp_id}).")
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
            "[PRIOR][UAV] Resume fallback applied "
            f"(forcedStartWP={_to_int((resume_waypoints[0] or {}).get('waypointID'))})."
        )

    done_sweep_points = count_sweep_points_in_waypoints(done_waypoints)

    # Append replan anchor waypoint to done path to preserve visualization continuity.
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
                emit(
                    "[PRIOR][UAV] Added replan anchor waypoint to done path "
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
                f"[PRIOR][UAV] Resume sweep trim applied "
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
        )
        scaled = scale_line_search_speed(resume_waypoints, _RESUME_SEARCH_SPEED_SCALE)
        if scaled > 0:
            emit(
                f"[PRIOR][UAV] Resume searchSpeed scaled "
                f"(factor={_RESUME_SEARCH_SPEED_SCALE:.2f}, waypoints={scaled})."
            )
        reassign_unique_waypoint_ids_inplace(resume_waypoints)
    resume_fp_data["waypointList"] = resume_waypoints
    return done_waypoints, resume_waypoints, removed_wp_id


def _clone_follow_up_replan_artifacts(
    *,
    missions: List[Dict[str, Any]],
    aircraft_id: int,
    now_ms: int,
    emit: Callable[[str], None],
    log_prefix: str,
    excluded_input_ids: Optional[Set[int]] = None,
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

    reserved_mission_ids = _reserve_individual_mission_ids(len(pending))
    reserved_path_ids = _reserve_path_ids(aircraft_id, len(pending))
    cloned_missions: List[Dict[str, Any]] = []
    cloned_paths: List[Tuple[Path, Dict[str, Any]]] = []
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
            fp_data = json.loads(src.read_text(encoding="utf-8"))
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

        fp_copy = deepcopy(fp_data)
        fp_copy["pathID"] = int(path_id)
        fp_copy["timestamp"] = now_ms
        fp_copy["Source"] = fp_copy.get("Source") or "MMR"
        fp_copy["aircraftID"] = aircraft_id
        fp_copy["individualMissionID"] = int(mission_id)
        for key in waypoint_keys:
            waypoints = fp_copy.get(key)
            if not isinstance(waypoints, list):
                continue
            copied_waypoints = deepcopy(waypoints)
            copied_wp_dicts = [wp for wp in copied_waypoints if isinstance(wp, dict)]
            for wp in copied_wp_dicts:
                wp["isDone"] = False
            if copied_waypoints:
                reassign_unique_waypoint_ids_inplace(copied_waypoints)
            fp_copy[key] = copied_waypoints

        dest = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        cloned_paths.append((dest, fp_copy))

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


def _load_done_input_ids_for_plan(source_plan_id: int) -> Set[int]:
    done_input_ids: Set[int] = set()
    try:
        plan_path = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = json.loads(plan_path.read_text(encoding="utf-8"))
        input_package_id = _to_int(plan_data.get("inputMissionPackageID"))
        if input_package_id is None:
            return done_input_ids
        input_path = db_paths.get_db_subpath("InputMissionPlan", f"{int(input_package_id)}.json")
        input_data = json.loads(input_path.read_text(encoding="utf-8"))
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
    allow_first_mission_fallback: bool = True,
) -> Optional[Dict[str, Any]]:
    artifacts = _resolve_plan_artifacts(
        source_plan_id=source_plan_id,
        aircraft_id=aircraft_id,
        current_waypoint_id=current_waypoint_id,
        emit=emit,
        allow_first_mission_fallback=allow_first_mission_fallback,
    )
    if artifacts is None:
        return None

    try:
        imp_src = db_paths.get_db_subpath(
            "IndividualMissionPlan", f"{artifacts.individual_mission_package_id}.json"
        )
        fp_src = db_paths.get_db_subpath("FlightPath", f"{artifacts.path_id}.json")
        imp_data = json.loads(imp_src.read_text(encoding="utf-8"))
        fp_data = json.loads(fp_src.read_text(encoding="utf-8"))
    except Exception as exc:
        emit(f"[PRIOR][UAV] Failed to load artifacts for aircraft {aircraft_id}: {exc}")
        return None

    [new_imp_id] = _reserve_imp_ids(1)
    done_path_id, resume_path_id = _reserve_path_ids(aircraft_id, 2)
    [resume_individual_id] = _reserve_individual_mission_ids(1)

    mission_list = imp_data.get("individualMissionList") or []
    target_index = None
    target_mission = None
    for idx, mission in enumerate(mission_list):
        if _to_int(mission.get("individualMissionID")) == artifacts.individual_mission_id:
            target_index = idx
            target_mission = mission
            break
    if target_mission is None:
        emit(
            f"[PRIOR][UAV] Individual mission {artifacts.individual_mission_id} "
            f"not found for aircraft {aircraft_id}."
        )
        return None

    follow_up_missions: List[Dict[str, Any]] = []
    follow_up_paths: List[Tuple[Path, Dict[str, Any]]] = []
    done_input_ids = _load_done_input_ids_for_plan(source_plan_id) if clone_follow_up_artifacts else set()
    if clone_follow_up_artifacts and target_index is not None:
        cloned_artifacts = _clone_follow_up_replan_artifacts(
            missions=mission_list[target_index + 1 :],
            aircraft_id=aircraft_id,
            now_ms=now_ms,
            emit=emit,
            log_prefix="[PRIOR][UAV]",
            excluded_input_ids=done_input_ids,
        )
        if cloned_artifacts is None:
            return None
        follow_up_missions, follow_up_paths = cloned_artifacts

    resume_mission = deepcopy(target_mission)
    resume_mission["individualMissionID"] = resume_individual_id
    resume_mission["pathID"] = resume_path_id
    resume_mission["isDone"] = False
    preserved_done_mission = deepcopy(target_mission)
    preserved_done_mission["pathID"] = done_path_id
    preserved_done_mission["isDone"] = True

    resume_fp_data = deepcopy(fp_data)
    done_waypoints, resume_waypoints, removed_wp_id = _apply_resume_path_trimming(
        resume_fp_data,
        artifacts=artifacts,
        sweep_progress=sweep_progress,
        emit=emit,
        current_coord=current_coord,
    )
    if not resume_waypoints:
        emit(f"[PRIOR][UAV] Resume path became empty for aircraft {aircraft_id}; skipping update.")
        return None

    done_fp_data = deepcopy(fp_data)
    done_fp_data["pathID"] = done_path_id
    done_fp_data["timestamp"] = now_ms
    done_fp_data["Source"] = done_fp_data.get("Source") or "MMR"
    done_fp_data["aircraftID"] = aircraft_id
    done_fp_data["individualMissionID"] = _to_int(target_mission.get("individualMissionID"))
    done_fp_data["waypointList"] = done_waypoints

    resume_fp_data["waypointList"] = resume_waypoints

    resume_fp_data["pathID"] = resume_path_id
    resume_fp_data["timestamp"] = now_ms
    resume_fp_data["Source"] = resume_fp_data.get("Source") or "MMR"
    resume_fp_data["aircraftID"] = aircraft_id
    resume_fp_data["individualMissionID"] = resume_individual_id

    imp_data["individualMissionPackageID"] = new_imp_id
    imp_data["timestamp"] = now_ms
    if 0 <= target_index < len(mission_list):
        if clone_follow_up_artifacts:
            prefix = deepcopy(mission_list[:target_index])
            rebuilt = prefix + [preserved_done_mission, resume_mission]
            rebuilt.extend(follow_up_missions)
            mission_list[:] = rebuilt
        else:
            mission_list[target_index] = preserved_done_mission
            mission_list.insert(target_index + 1, resume_mission)
    else:
        mission_list.insert(0, resume_mission)
        emit(
            f"[PRIOR][UAV] Target mission index invalid; appended resume at head (aircraft {aircraft_id})."
        )

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    done_fp_dest = db_paths.get_db_subpath("FlightPath", f"{done_path_id}.json")
    resume_fp_dest = db_paths.get_db_subpath("FlightPath", f"{resume_path_id}.json")
    for path in (imp_dest, done_fp_dest, resume_fp_dest, *(dest for dest, _ in follow_up_paths)):
        path.parent.mkdir(parents=True, exist_ok=True)
    write_json(imp_dest, imp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    write_json(done_fp_dest, done_fp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    write_json(resume_fp_dest, resume_fp_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    for dest, payload in follow_up_paths:
        write_json(dest, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)

    emit(
        f"[PRIOR][UAV] Generated done/resume mission -> "
        f"aircraft={aircraft_id} IMP:{imp_dest.name} PATHS:{done_fp_dest.name}/{resume_fp_dest.name}"
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
        "donePath": str(done_fp_dest),
        "resumePath": str(resume_fp_dest),
        "followUpMissionCount": len(follow_up_missions),
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
        write_json(log_path, data, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
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
        plan_data = json.loads(plan_path.read_text(encoding="utf-8"))
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
        imp_data = json.loads(imp_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        emit(f"[PRIOR] IndividualMissionPlan {package_id} not found.")
        return None
    except Exception as exc:
        emit(f"[PRIOR] IndividualMissionPlan {package_id} load failed: {exc}")
        return None

    missions = imp_data.get("individualMissionList") or []
    target_mission = None
    previous_wp = None
    resolved_current_wp = current_waypoint_id

    for mission in missions:
        path_id = mission.get("pathID")
        individual_mission_id = mission.get("individualMissionID")
        if path_id is None or individual_mission_id is None:
            continue
        waypoints = _load_waypoint_ids(path_id)
        if not waypoints:
            continue
        if current_waypoint_id in waypoints:
            idx = waypoints.index(current_waypoint_id)
            previous_wp = waypoints[idx - 1] if idx > 0 else None
            target_mission = (
                int(individual_mission_id),
                int(path_id),
            )
            break

    if target_mission is None and missions and allow_first_mission_fallback:
        fallback = missions[0]
        try:
            mission_id = int(fallback.get("individualMissionID"))
        except (TypeError, ValueError):
            mission_id = 0
        try:
            path_id = int(fallback.get("pathID"))
        except (TypeError, ValueError):
            path_id = 0
        waypoints = _load_waypoint_ids(path_id)
        if waypoints:
            resolved_current_wp = waypoints[0]
            previous_wp = None
        target_mission = (mission_id, path_id)
        emit(
            "[PRIOR] Falling back to first mission for aircraft "
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
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return []
    waypoints: List[int] = []
    for wp in data.get("waypointList", []):
        value = wp.get("waypointID")
        try:
            waypoints.append(int(value))
        except (TypeError, ValueError):
            continue
    return waypoints
