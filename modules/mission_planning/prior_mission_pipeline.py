from __future__ import annotations

import json
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set, Tuple

from modules.common import db_paths
import importlib.util
from types import ModuleType

_EPOCH_2000_MS = 946_684_800_000
_ID_ALLOCATOR_MOD: Optional[ModuleType] = None


def _load_id_allocator() -> ModuleType:
    global _ID_ALLOCATOR_MOD
    if _ID_ALLOCATOR_MOD is not None:
        return _ID_ALLOCATOR_MOD
    allocator_path = (
        Path(__file__).resolve().parents[1] / "MissionPlanner" / "data_def" / "id_allocator.py"
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

    aircraft_id = _to_int(detail.get("aircraftID"))
    path_id = _to_int(detail.get("pathID"))
    source_plan_id = _to_int(detail.get("sourceMissionPlanID"))
    imp_package_id = _to_int(detail.get("individualMissionPackageID"))
    individual_mission_id = _to_int(detail.get("individualMissionID"))
    current_waypoint_id = _to_int(detail.get("currentWaypointID"))
    previous_waypoint_id = _to_int(detail.get("previousWaypointID"))
    prior_mission_id = _to_int(detail.get("priorMissionID"))
    mission_type = _to_int(detail.get("missionType"))

    required = {
        "aircraftID": aircraft_id,
        "pathID": path_id,
        "sourceMissionPlanID": source_plan_id,
        "individualMissionPackageID": imp_package_id,
        "individualMissionID": individual_mission_id,
        "currentWaypointID": current_waypoint_id,
    }
    missing = [name for name, value in required.items() if value is None]
    if missing:
        emit(f"[PRIOR] Missing required detail fields: {', '.join(missing)}")
        return None

    target_coord = dict(detail.get("targetCoordinate") or {})
    lat = _to_float(target_coord.get("latitude"))
    lon = _to_float(target_coord.get("longitude"))
    if lat is None or lon is None:
        emit("[PRIOR] Target coordinate missing latitude/longitude.")
        return None
    if "altitude" not in target_coord:
        target_coord["altitude"] = None

    db_root = db_paths.get_active_db_root()
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

    removed_wp_id, inserted_wp = _inject_prior_waypoint(
        new_fp_data,
        current_waypoint_id,
        previous_waypoint_id,
        target_coord,
        new_waypoint_id,
    )
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

    return PriorMissionPipelineResult(
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


def _inject_prior_waypoint(
    flight_path: Dict[str, Any],
    current_waypoint_id: int,
    previous_waypoint_id: Optional[int],
    target_coord: Dict[str, float],
    new_waypoint_id: int,
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
    removal_index = current_index - 1
    if removal_index >= 0:
        removed_wp = waypoint_list.pop(removal_index)
        removed_waypoint_id = _to_int(removed_wp.get("waypointID"))
        current_index -= 1
    elif previous_waypoint_id:
        # ensure previous pointer is cleared when explicit ID provided
        removed_waypoint_id = previous_waypoint_id

    preceding_index = current_index - 1
    altitude = target_coord.get("altitude")
    if altitude is None and removal_index >= 0 and removed_waypoint_id is not None:
        altitude = _to_float(
            (waypoint_list[removal_index].get("coordinate") or {}).get("altitude")
        )
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

    waypoint_list.insert(current_index, inserted_wp)
    if preceding_index >= 0:
        waypoint_list[preceding_index]["nextWaypointID"] = new_waypoint_id
    flight_path["waypointList"] = waypoint_list
    return removed_waypoint_id, inserted_wp
