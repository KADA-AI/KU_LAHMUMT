from __future__ import annotations

import json
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set

from modules.common import db_paths, path_deviation_replan_store
from modules.mission_planning.runtime.json_io import write_json
from modules.mission_planning.pipelines.mission_path_trim import (
    reassign_unique_waypoint_ids_inplace,
)
from modules.mission_planning.pipelines.prior_mission_pipeline_impl import (
    _next_waypoint_id,
    _now_ms_since_2000,
    _normalize_altitude_value,
    _reserve_imp_ids,
    _reserve_individual_mission_ids,
    _reserve_path_ids,
    _resolve_plan_artifacts,
    _to_float,
    _to_int,
)


DEFAULT_OPTION_NAME = "비행/촬영"


@dataclass
class PathDeviationPipelineResult:
    plan_ids: List[int]
    option_names: List[str]
    plan_meta_map: Dict[int, Dict[str, Any]]
    generated_imp_ids: Set[int]
    generated_path_ids: Set[int]
    new_imp_id: int
    new_path_id: int
    new_individual_id: int
    removed_waypoint_id: int
    inserted_waypoint_id: int
    log_path: Path


def warm_path_deviation_replan_pipeline() -> Dict[str, Any]:
    return {"ready": True}


def _ensure_option_names(plan_ids: List[int], option_names: List[str] | None) -> List[str]:
    names = [str(name) for name in (option_names or []) if name is not None]
    if not names:
        names = [DEFAULT_OPTION_NAME]
    while len(names) < len(plan_ids):
        names.append(names[-1])
    return names[: len(plan_ids)]


def _extract_alt_coordinate(detail: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    payload = detail.get("alternateWaypointCoordinate")
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    alt = _normalize_altitude_value(payload.get("altitude"))
    if lat is None or lon is None:
        return None
    result: Dict[str, Any] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        result["altitude"] = int(alt)
    return result


def _set_source_field(payload: Dict[str, Any], source: str) -> None:
    if "Source" in payload or "source" not in payload:
        payload["Source"] = str(payload.get("Source") or payload.get("source") or source)
    else:
        payload["source"] = str(payload.get("source") or payload.get("Source") or source)


def _load_detail_from_store(plan_ids: List[int]) -> Optional[Dict[str, Any]]:
    for value in plan_ids:
        try:
            plan_id = int(value)
        except Exception:
            continue
        payload = path_deviation_replan_store.load_detail(plan_id)
        if payload:
            return payload
    return None


def _build_replanned_waypoints(
    waypoint_list: List[Dict[str, Any]],
    *,
    current_waypoint_id: int,
    alternate_waypoint_id: Optional[int],
    alternate_coordinate: Dict[str, Any],
    emit: Callable[[str], None],
) -> tuple[List[Dict[str, Any]], int, int]:
    current_index = None
    for idx, waypoint in enumerate(waypoint_list):
        if _to_int((waypoint or {}).get("waypointID")) == int(current_waypoint_id):
            current_index = idx
            break
    if current_index is None:
        emit(f"[PATHDEV] currentWaypointID {current_waypoint_id} not found in source FlightPath.")
        raise RuntimeError("current waypoint not found in source path")

    current_waypoint = deepcopy(waypoint_list[current_index])
    remaining_waypoints = [deepcopy(item) for item in waypoint_list[current_index + 1 :]]
    inserted_waypoint = deepcopy(current_waypoint)
    existing_ids = {
        int(wp_id)
        for wp_id in (_to_int((item or {}).get("waypointID")) for item in waypoint_list)
        if wp_id is not None and int(wp_id) > 0
    }
    candidate_waypoint_id = _to_int(alternate_waypoint_id)
    if (
        candidate_waypoint_id is None
        or candidate_waypoint_id <= 0
        or candidate_waypoint_id == int(current_waypoint_id)
        or candidate_waypoint_id in existing_ids
    ):
        candidate_waypoint_id = _next_waypoint_id()
    inserted_waypoint["waypointID"] = int(candidate_waypoint_id)

    coordinate = dict(inserted_waypoint.get("coordinate") or {})
    coordinate["latitude"] = float(alternate_coordinate["latitude"])
    coordinate["longitude"] = float(alternate_coordinate["longitude"])
    current_altitude = _normalize_altitude_value(coordinate.get("altitude"))
    alternate_altitude = _normalize_altitude_value(alternate_coordinate.get("altitude"))
    if alternate_altitude is not None:
        coordinate["altitude"] = int(alternate_altitude)
    elif current_altitude is not None:
        coordinate["altitude"] = int(current_altitude)
    else:
        coordinate["altitude"] = 0
    inserted_waypoint["coordinate"] = coordinate
    inserted_waypoint["isDone"] = False

    new_waypoints = [inserted_waypoint] + remaining_waypoints
    for waypoint in new_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    reassign_unique_waypoint_ids_inplace(new_waypoints)
    inserted_waypoint_id = _to_int(inserted_waypoint.get("waypointID")) or int(candidate_waypoint_id)
    return new_waypoints, int(current_waypoint_id), int(inserted_waypoint_id)


def run_path_deviation_replan_pipeline(
    ctx: Dict[str, Any],
    detail: Dict[str, Any],
    reason: str,
    *,
    log: Callable[[str], None],
) -> Optional[PathDeviationPipelineResult]:
    log_messages: List[str] = []

    def emit(message: str) -> None:
        log_messages.append(message)
        log(message)

    plan_ids_raw = list(ctx.get("plan_ids") or [])
    try:
        plan_ids = [int(value) for value in plan_ids_raw if value is not None]
    except Exception:
        emit(f"[PATHDEV] invalid plan_ids in context: {plan_ids_raw!r}")
        return None
    if len(plan_ids) != 1:
        emit(f"[PATHDEV] expected exactly one pending missionPlanID, got {len(plan_ids)}.")
        return None

    stored_detail = _load_detail_from_store(plan_ids)
    if not isinstance(detail, dict) or not detail:
        detail = stored_detail or {}
    elif stored_detail:
        merged_detail = dict(stored_detail)
        merged_detail.update(detail)
        detail = merged_detail
    if not isinstance(detail, dict) or not detail:
        emit("[PATHDEV] replanDetail missing and store lookup failed.")
        try:
            path_deviation_replan_store.save_event(
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
    if current_waypoint_id is None or current_waypoint_id <= 0:
        emit("[PATHDEV] currentWaypointID is 0/invalid -> not a replan target.")
        return None
    alternate_coordinate = _extract_alt_coordinate(detail)
    if alternate_coordinate is None:
        emit("[PATHDEV] alternateWaypointCoordinate missing latitude/longitude.")
        return None
    alternate_waypoint_id = _to_int(detail.get("alternateWaypointID"))

    if source_plan_id is None or source_plan_id <= 0 or aircraft_id is None or aircraft_id <= 0:
        emit(
            "[PATHDEV] required source identifiers missing "
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
        emit("[PATHDEV] failed to resolve source mission artifacts.")
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
        emit(f"[PATHDEV] failed to load source artifacts: {exc}")
        return None

    new_plan_id = int(plan_ids[0])
    [new_imp_id] = _reserve_imp_ids(1)
    [new_individual_id] = _reserve_individual_mission_ids(1)
    [new_path_id] = _reserve_path_ids(int(aircraft_id), 1)
    option_names = _ensure_option_names([new_plan_id], ctx.get("option_names"))
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
        emit(f"[PATHDEV] aircraft {aircraft_id} not found in source MissionPlan {source_plan_id}.")
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
            f"[PATHDEV] individualMissionID {artifacts.individual_mission_id} "
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
        emit(f"[PATHDEV] FlightPath {artifacts.path_id} has no waypointList.")
        return None

    try:
        new_waypoints, removed_waypoint_id, inserted_waypoint_id = _build_replanned_waypoints(
            [deepcopy(item) for item in source_waypoints],
            current_waypoint_id=int(current_waypoint_id),
            alternate_waypoint_id=alternate_waypoint_id,
            alternate_coordinate=alternate_coordinate,
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
        "[PATHDEV] stored path-deviation artifacts -> "
        f"plan:{plan_dest.name}, imp:{imp_dest.name}, fp:{fp_dest.name} ({elapsed_ms:.1f} ms)"
    )

    log_dir = db_paths.get_db_subpath("DSS_Internal")
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"PathDeviation_{aircraft_id}_{now_ms}.json"
    log_payload = {
        "timestamp": int(now_ms),
        "reason": str(reason or ""),
        "sourceMissionPlanID": int(source_plan_id),
        "aircraftID": int(aircraft_id),
        "sourceIndividualMissionPackageID": int(artifacts.individual_mission_package_id),
        "sourceIndividualMissionID": int(artifacts.individual_mission_id),
        "sourcePathID": int(artifacts.path_id),
        "currentWaypointID": int(current_waypoint_id),
        "removedWaypointID": int(removed_waypoint_id),
        "alternateWaypointID": int(inserted_waypoint_id),
        "alternateWaypointCoordinate": dict(alternate_coordinate),
        "generatedMissionPlanID": int(new_plan_id),
        "generatedIndividualMissionPackageID": int(new_imp_id),
        "generatedIndividualMissionID": int(new_individual_id),
        "generatedPathID": int(new_path_id),
        "logMessages": log_messages,
        "detail": dict(detail),
    }
    write_json(log_path, log_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
    emit(f"[PATHDEV] log captured -> {log_path}")
    try:
        path_deviation_replan_store.save_event(
            "mission_pipeline_complete",
            {
                "generatedMissionPlanID": int(new_plan_id),
                "sourceMissionPlanID": int(source_plan_id),
                "aircraftID": int(aircraft_id),
                "removedWaypointID": int(removed_waypoint_id),
                "insertedWaypointID": int(inserted_waypoint_id),
                "logPath": str(log_path),
            },
        )
    except Exception:
        pass

    plan_meta_map = dict(ctx.get("_option_meta") or {})
    plan_meta_entry = plan_meta_map.setdefault(int(new_plan_id), {})
    plan_meta_entry.update(
        {
            "triggerType": "pathDeviation",
            "sourceMissionPlanID": int(source_plan_id),
            "aircraftID": int(aircraft_id),
            "individualMissionPackageID": int(new_imp_id),
            "individualMissionID": int(new_individual_id),
            "pathID": int(new_path_id),
            "removedWaypointID": int(removed_waypoint_id),
            "insertedWaypointID": int(inserted_waypoint_id),
            "logPath": str(log_path),
            "alternateWaypointCoordinate": dict(alternate_coordinate),
        }
    )

    return PathDeviationPipelineResult(
        plan_ids=[int(new_plan_id)],
        option_names=list(option_names),
        plan_meta_map=plan_meta_map,
        generated_imp_ids={int(new_imp_id)},
        generated_path_ids={int(new_path_id)},
        new_imp_id=int(new_imp_id),
        new_path_id=int(new_path_id),
        new_individual_id=int(new_individual_id),
        removed_waypoint_id=int(removed_waypoint_id),
        inserted_waypoint_id=int(inserted_waypoint_id),
        log_path=log_path,
    )
