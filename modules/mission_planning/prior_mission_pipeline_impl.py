from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from modules.common import db_paths, agent_status_snapshot, prior_replan_store
import importlib.util
from types import ModuleType

_EPOCH_2000_MS = 946_684_800_000
_ID_ALLOCATOR_MOD: Optional[ModuleType] = None


def _load_id_allocator() -> ModuleType:
    global _ID_ALLOCATOR_MOD
    if _ID_ALLOCATOR_MOD is not None:
        return _ID_ALLOCATOR_MOD
    allocator_path = (
        Path(__file__).resolve().parent / "MissionPlanner" / "data_def" / "id_allocator.py"
    )
    spec = importlib.util.spec_from_file_location(
        "mission_planner_id_allocator", allocator_path
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load id_allocator from {allocator_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _ID_ALLOCATOR_MOD = module
    return module


def _next_imp_id() -> int:
    return _load_id_allocator().next_imp_id()


def _next_individual_mission_id() -> int:
    return _load_id_allocator().next_individual_mission_id()


def _next_path_id(aircraft_id: int) -> int:
    return _load_id_allocator().next_path_id(aircraft_id)


def _next_waypoint_id() -> int:
    return _load_id_allocator().next_waypoint_id()


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
    log_path: Path
    removed_waypoint_id: Optional[int]
    inserted_waypoint_id: int


@dataclass
class AgentSnapshotSummary:
    aircraft_id: int
    latitude: Optional[float]
    longitude: Optional[float]
    altitude: Optional[float]
    current_waypoint_id: Optional[int]


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
    new_path_id: Optional[int] = None
    new_individual_id: Optional[int] = None
    removed_wp_id: Optional[int] = None
    inserted_wp_id: Optional[int] = None
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
        aircraft_id = _to_int(detail.get("aircraftID"))
        path_id = _to_int(detail.get("pathID"))
        source_plan_id = _to_int(detail.get("sourceMissionPlanID"))
        imp_package_id = _to_int(detail.get("individualMissionPackageID"))
        individual_mission_id = _to_int(detail.get("individualMissionID"))
        current_waypoint_id = _to_int(detail.get("currentWaypointID"))
        previous_waypoint_id = _to_int(detail.get("previousWaypointID"))
        target_coord = dict(detail.get("targetCoordinate") or {})
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
                    target_coord["altitude"] = coord_block.get("altitude")
                    emit("[PRIOR][STEP2] Target coordinate 보강: PriorMissionInfo 최신 기록에서 좌표 복구.")

        target_tracking_entry = None
        if mission_type == 2:
            target_tracking_entry = _load_target_tracking_entry(target_id)
            if target_tracking_entry:
                coord_block = target_tracking_entry.get("coordinate") or {}
                if coord_block:
                    target_coord["latitude"] = coord_block.get("latitude")
                    target_coord["longitude"] = coord_block.get("longitude")
                    target_coord["altitude"] = coord_block.get("altitude")
        if target_coord.get("latitude") is None or target_coord.get("longitude") is None:
            fallback_coord = _load_prior_coordinate_from_db(prior_mission_id)
            if fallback_coord:
                target_coord["latitude"] = fallback_coord.get("latitude")
                target_coord["longitude"] = fallback_coord.get("longitude")
                target_coord["altitude"] = fallback_coord.get("altitude")
                emit(
                    f"[PRIOR][STEP2] Target coordinate 보강: PriorMissionInfo/{prior_mission_id}.json에서 좌표 복구."
                )
        lat = _to_float(target_coord.get("latitude"))
        lon = _to_float(target_coord.get("longitude"))
        _log_step2_target_coordinate(emit, lat, lon, target_coord.get("altitude"))
        if lat is None or lon is None:
            emit("[PRIOR] Target coordinate missing latitude/longitude.")
            return None
        if "altitude" not in target_coord:
            target_coord["altitude"] = None

        if mission_type == 2 and target_tracking_entry:
            watcher_id = _to_int(target_tracking_entry.get("watcherID"))
            if watcher_id is not None:
                selected_agent_summary = next(
                    (summary for summary in agent_summaries if summary.aircraft_id == watcher_id),
                    None,
                )
                selected_agent_distance_m = None
                if selected_agent_summary:
                    emit(
                        f"[PRIOR][STEP3] Target-tracking watcher UAV {watcher_id} selected (targetID={target_id})."
                    )
        if selected_agent_summary is None:
            selected_agent_summary, selected_agent_distance_m = _select_nearest_agent(
                lat, lon, agent_summaries
            )
            _log_step3_nearest_agent(emit, selected_agent_summary, selected_agent_distance_m)

        if selected_agent_summary is None:
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
        new_imp_id = _next_imp_id()
        new_path_id = _next_path_id(aircraft_id)
        new_individual_id = _next_individual_mission_id()
        new_waypoint_id = _next_waypoint_id()
        _log_step4_waypoint_allocation(
            emit,
            new_waypoint_id,
            selected_agent_summary,
            selected_agent_distance_m,
        )
        emit(
            f"[PRIOR] Allocated IDs -> plan:{new_plan_id} imp:{new_imp_id} path:{new_path_id} indiv:{new_individual_id} wp:{new_waypoint_id}"
        )

        new_plan_data = deepcopy(plan_data)
        new_imp_data = deepcopy(imp_data)
        new_fp_data = deepcopy(fp_data)

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
        new_plan_data["_priorMissionContext"] = detail

        target_mission_entry = None
        for mission in new_imp_data.get("individualMissionList", []):
            if _to_int(mission.get("individualMissionID")) == individual_mission_id:
                target_mission_entry = mission
                break
        if not target_mission_entry:
            emit(f"[PRIOR] Individual mission {individual_mission_id} not found in package {imp_package_id}.")
            return None

        target_mission_entry["individualMissionID"] = new_individual_id
        target_mission_entry["pathID"] = new_path_id
        rel_block = dict(target_mission_entry.get("relatedMission") or {})
        rel_block["priorMissionID"] = prior_mission_id or 0
        rel_block["relatedMissionType"] = 2
        input_mission_id = _to_int(detail.get("inputMissionID"))
        if input_mission_id is not None:
            rel_block["inputMissionID"] = input_mission_id
        target_mission_entry["relatedMission"] = rel_block
        target_mission_entry["isDone"] = False
        new_imp_data["individualMissionPackageID"] = new_imp_id

        target_tracking_payload = {"targetID": target_id} if mission_type == 2 and target_id is not None else None
        removed_wp_id, inserted_wp = _inject_prior_waypoint(
            new_fp_data,
            current_waypoint_id,
            previous_waypoint_id,
            target_coord,
            new_waypoint_id,
            mission_type=mission_type,
            target_tracking=target_tracking_payload,
        )
        inserted_wp_id = new_waypoint_id
        new_fp_data["pathID"] = new_path_id
        new_fp_data["timestamp"] = now_ms
        new_fp_data["Source"] = new_fp_data.get("Source") or "MMR"

        plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
        imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
        fp_dest = db_paths.get_db_subpath("FlightPath", f"{new_path_id}.json")
        for path in (plan_dest, imp_dest, fp_dest):
            path.parent.mkdir(parents=True, exist_ok=True)
        plan_dest.write_text(json.dumps(new_plan_data, ensure_ascii=False, indent=2), encoding="utf-8")
        imp_dest.write_text(json.dumps(new_imp_data, ensure_ascii=False, indent=2), encoding="utf-8")
        fp_dest.write_text(json.dumps(new_fp_data, ensure_ascii=False, indent=2), encoding="utf-8")
        emit(f"[PRIOR] Stored new artifacts -> plan:{plan_dest.name}, imp:{imp_dest.name}, fp:{fp_dest.name}")

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
            "generatedIndividualMissionID": new_individual_id,
            "generatedPathID": new_path_id,
            "logMessages": log_messages,
        }
        log_path.write_text(json.dumps(log_payload, ensure_ascii=False, indent=2), encoding="utf-8")
        emit(f"[PRIOR] Log captured -> {log_path}")

        plan_meta_map = dict(ctx.get("_option_meta") or {})
        plan_meta_entry = plan_meta_map.setdefault(new_plan_id, {})
        plan_meta_entry.update(
            {
                "priorMissionID": prior_mission_id,
                "sourceMissionPlanID": source_plan_id,
                "individualMissionPackageID": new_imp_id,
                "individualMissionID": new_individual_id,
                "pathID": new_path_id,
                "logPath": str(log_path),
                "removedWaypointID": removed_wp_id,
                "insertedWaypointID": new_waypoint_id,
                "targetCoordinate": target_coord,
            }
        )

        success = True
        result = PriorMissionPipelineResult(
            plan_ids=plan_ids,
            option_names=option_names,
            plan_meta_map=plan_meta_map,
            generated_imp_ids={new_imp_id},
            generated_path_ids={new_path_id},
            new_imp_id=new_imp_id,
            new_path_id=new_path_id,
            new_individual_id=new_individual_id,
            log_path=log_path,
            removed_waypoint_id=removed_wp_id,
            inserted_waypoint_id=new_waypoint_id,
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
            "newIndividualMissionID": new_individual_id,
            "newPathID": new_path_id,
            "removedWaypointID": removed_wp_id,
            "insertedWaypointID": inserted_wp_id,
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
        summaries.append(
            AgentSnapshotSummary(
                aircraft_id=aircraft_id,
                latitude=lat,
                longitude=lon,
                altitude=alt,
                current_waypoint_id=current_wp,
            )
        )
    return summaries


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
    target_coord: Dict[str, float],
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
    inherited_altitude: Optional[float] = None
    if current_index > 0:
        completed_segment = waypoint_list[:current_index]
        waypoint_list = waypoint_list[current_index:]
        current_index = 0
        last_completed = completed_segment[-1]
        removed_waypoint_id = _to_int(last_completed.get("waypointID"))
        inherited_altitude = _to_float((last_completed.get("coordinate") or {}).get("altitude"))
    elif previous_waypoint_id:
        # ensure previous pointer is cleared when explicit ID provided
        removed_waypoint_id = previous_waypoint_id

    preceding_index = current_index - 1
    altitude = target_coord.get("altitude")
    if altitude is None and inherited_altitude is not None:
        altitude = inherited_altitude
    if altitude is None:
        altitude = 700.0

    inserted_wp = {
        "waypointID": new_waypoint_id,
        "coordinate": {
            "latitude": target_coord["latitude"],
            "longitude": target_coord["longitude"],
            "altitude": altitude,
        },
        "speed": 30.0,
        "eta": 30,
        "ecf": 0.0,
        "nextWaypointID": current_waypoint_id,
        "waypointPassType": 2,
        "filmingProperty": {
            "fieldOfView": 5.0,
            "sensorType": 1,
            "operationMode": 1,
            "coordinateOrientation": {
                "coordinate": {
                    "latitude": target_coord["latitude"],
                    "longitude": target_coord["longitude"],
                    "altitude": target_coord["altitude"]
                    if target_coord.get("altitude") is not None
                    else 0.0,
                }
            },
        },
        "loiterProperty": {
            "radius": 400,
            "direction": 1,
            "time": 30,
            "speed": 30,
        },
    }

    if mission_type == 2:
        filming = inserted_wp.get("filmingProperty") or {}
        filming["operationMode"] = 3
        inserted_wp["filmingProperty"] = filming
        target_track_id = _to_int((target_tracking or {}).get("targetID"))
        if target_track_id is not None:
            inserted_wp["autoTracking"] = {"targetID": target_track_id}

    waypoint_list.insert(current_index, inserted_wp)
    if preceding_index >= 0:
        waypoint_list[preceding_index]["nextWaypointID"] = new_waypoint_id
    flight_path["waypointList"] = waypoint_list
    return removed_waypoint_id, inserted_wp


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
        log_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
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
    for entry in target_list.values():
        if not isinstance(entry, dict):
            continue
        entry_target_id = _to_int(entry.get("targetID"))
        if entry_target_id != target_id:
            continue
        return entry
    return None


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
    alt = _to_float(coordinate.get("altitude"))
    if lat is None or lon is None:
        return None
    result = {"latitude": lat, "longitude": lon}
    if alt is not None:
        result["altitude"] = alt
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

    if target_mission is None and missions:
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
