from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set

from modules.common import agent_status_snapshot, db_paths, mission_area_replan_store, path_deviation_replan_store
from modules.mission_planning.runtime.debug_artifacts import debug_artifact_mode, write_debug_json
from modules.mission_planning.runtime.json_io import write_json, write_json_batch
from modules.mission_planning.runtime.logging.pipeline_events import (
    PipelinePhaseTimer,
    new_replan_transaction_id,
)
from modules.mission_planning.runtime.ids.replan_reservation import ReplanIdReservation
from modules.mission_planning.runtime.validation.replan_payloads import (
    validate_generated_artifact_payloads,
    validate_replan_payloads,
)
from modules.mission_planning.MissionPlanner.runtime_settings import (
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
    load_sweep_progress,
    physical_sweep_cut_points,
    recompute_line_search_speed_from_geometry,
    relink_waypoints,
    reassign_unique_waypoint_ids_inplace,
    trim_waypoints_by_is_done_prefix,
    trim_waypoints_by_sweep_points,
)
from modules.mission_planning.replanning.triggers.prior.pipeline import (
    PlanMissionArtifacts,
    _apply_runtime_flyover_to_flight_path_payload,
    _prepare_uav_collaborative_resume_replan,
    _now_ms_since_2000,
    _normalize_altitude_value,
    _reserve_imp_ids,
    _reserve_individual_mission_ids,
    _reserve_path_ids,
    _reserve_waypoint_block,
    _resolve_plan_artifacts,
    _to_float,
    _to_int,
)
from modules.mission_planning.runtime.state.attack_tracking import (
    get_tracking_assignment,
    list_active_tracking_assignments,
    rebind_tracking_assignments_to_plan,
)


DEFAULT_OPTION_NAME = "비행/촬영"


_PATHDEV_COMPLETE_HOLD_SECONDS = 5
_PATHDEV_COMPLETE_HOLD_RADIUS_M = 180
_PATHDEV_COMPLETE_HOLD_SPEED_MPS = 30.0
_PATHDEV_ENTRY_WAYPOINT_RETAIN_RADIUS_M = 180.0
_PATHDEV_DEFAULT_CLIMB_RATE_MPS = 5.0
_PATHDEV_DEFAULT_WAYPOINT_SPEED_MPS = 38.89
_PATHDEV_FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M = 30.0


def _assign_waypoint_ids_inplace(
    waypoints: List[Dict[str, Any]],
    allocator: Callable[[], int] | None,
) -> List[int]:
    if allocator is None:
        return reassign_unique_waypoint_ids_inplace(waypoints)
    waypoint_dicts = [item for item in (waypoints or []) if isinstance(item, dict)]
    assigned: List[int] = []
    for waypoint in waypoint_dicts:
        waypoint["waypointID"] = int(allocator())
        assigned.append(int(waypoint["waypointID"]))
    for idx in range(len(waypoint_dicts) - 1):
        waypoint_dicts[idx]["nextWaypointID"] = int(waypoint_dicts[idx + 1].get("waypointID", 0) or 0)
    if waypoint_dicts:
        waypoint_dicts[-1]["nextWaypointID"] = 0
    return assigned


@dataclass
class PathDeviationPipelineResult:
    plan_ids: List[int]
    option_names: List[str]
    plan_meta_map: Dict[int, Dict[str, Any]]
    generated_imp_ids: Set[int]
    generated_path_ids: Set[int]
    preserved_manned_imp_ids: Set[int]
    preserved_manned_path_ids: Set[int]
    new_imp_id: int
    new_path_id: int
    new_individual_id: int
    removed_waypoint_id: int
    inserted_waypoint_id: int
    log_path: Path
    other_updates: List[Dict[str, Any]]


def warm_path_deviation_replan_pipeline() -> Dict[str, Any]:
    return {"ready": True}


def _ensure_option_names(plan_ids: List[int], option_names: List[str] | None) -> List[str]:
    names = [str(name) for name in (option_names or []) if name is not None]
    if not names:
        names = [DEFAULT_OPTION_NAME]
    while len(names) < len(plan_ids):
        names.append(names[-1])
    return names[: len(plan_ids)]


def _tracking_assignment_preserved_in_plan(plan_data: Dict[str, Any], assignment: Dict[str, Any]) -> bool:
    aircraft_id = _to_int(assignment.get("aircraft_id"))
    tracking_path_id = _to_int(assignment.get("tracking_path_id"))
    tracking_mission_id = _to_int(assignment.get("tracking_individual_mission_id"))
    if aircraft_id is None or tracking_path_id is None:
        return False
    imp_id: Optional[int] = None
    for aircraft_entry in plan_data.get("aircraftList") or []:
        if not isinstance(aircraft_entry, dict):
            continue
        if _to_int(aircraft_entry.get("aircraftID")) != int(aircraft_id):
            continue
        imp_id = _to_int(aircraft_entry.get("individualMissionPackageID"))
        break
    if imp_id is None or imp_id <= 0:
        return False
    try:
        imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(imp_id)}.json")
        imp_data = json.loads(imp_path.read_text(encoding="utf-8"))
    except Exception:
        return False
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        path_id = _to_int(mission.get("pathID") or mission.get("flightPathID"))
        if path_id != int(tracking_path_id):
            continue
        if tracking_mission_id is None:
            return True
        return _to_int(mission.get("individualMissionID")) == int(tracking_mission_id)
    return False


def _tracking_aircraft_ids_preserved_for_rebind(
    *,
    source_plan_id: int,
    plan_data: Dict[str, Any],
    emit: Callable[[str], None],
) -> Set[int]:
    preserved: Set[int] = set()
    for assignment in list_active_tracking_assignments():
        if not isinstance(assignment, dict):
            continue
        if _to_int(assignment.get("attack_plan_id")) != int(source_plan_id):
            continue
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        if aircraft_id is None:
            continue
        if _tracking_assignment_preserved_in_plan(plan_data, assignment):
            preserved.add(int(aircraft_id))
            continue
        emit(
            "[PATHDEV] active attack tracking assignment not rebound because tracking package is absent "
            f"(aircraft={int(aircraft_id)}, sourcePlan={int(source_plan_id)})."
        )
    return preserved


def _active_attack_tracking_aircraft_ids_for_plan(source_plan_id: int) -> Set[int]:
    aircraft_ids: Set[int] = set()
    for assignment in list_active_tracking_assignments():
        if not isinstance(assignment, dict) or not bool(assignment.get("active")):
            continue
        if _to_int(assignment.get("attack_plan_id")) != int(source_plan_id):
            continue
        aircraft_id = _to_int(assignment.get("aircraft_id"))
        if aircraft_id is not None and aircraft_id > 0:
            aircraft_ids.add(int(aircraft_id))
    return aircraft_ids


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


def _first_valid_waypoint_altitude(waypoints: List[Dict[str, Any]]) -> Optional[int]:
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        coordinate = waypoint.get("coordinate")
        if not isinstance(coordinate, dict):
            continue
        altitude = _normalize_altitude_value(coordinate.get("altitude"))
        if altitude is not None:
            return int(altitude)
    return None


def _runtime_pathdev_climb_rate_mps() -> float:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float

        value = float(get_runtime_float("uav_climb_rate_mps", _PATHDEV_DEFAULT_CLIMB_RATE_MPS))
    except Exception:
        value = float(_PATHDEV_DEFAULT_CLIMB_RATE_MPS)
    if not math.isfinite(value) or value <= 0.0:
        value = float(_PATHDEV_DEFAULT_CLIMB_RATE_MPS)
    return float(value)


def _coordinate_distance_m(left: Dict[str, Any], right: Dict[str, Any]) -> float:
    lat1 = _to_float((left or {}).get("latitude"))
    lon1 = _to_float((left or {}).get("longitude"))
    lat2 = _to_float((right or {}).get("latitude"))
    lon2 = _to_float((right or {}).get("longitude"))
    if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
        return 0.0
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    d_phi = math.radians(float(lat2) - float(lat1))
    d_lambda = math.radians(float(lon2) - float(lon1))
    a_val = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
    )
    return 6_371_000.0 * 2.0 * math.atan2(math.sqrt(a_val), math.sqrt(max(0.0, 1.0 - a_val)))


def _waypoint_speed_mps(*waypoints: Dict[str, Any] | None) -> float:
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        speed = _to_float(waypoint.get("speed"))
        if speed is not None and math.isfinite(float(speed)) and float(speed) > 0.0:
            return float(speed)
    return float(_PATHDEV_DEFAULT_WAYPOINT_SPEED_MPS)


def _filming_target_altitude_floor_m(waypoint: Dict[str, Any]) -> Optional[int]:
    filming = waypoint.get("filmingProperty") if isinstance(waypoint, dict) else None
    if not isinstance(filming, dict):
        return None
    samples: List[int] = []
    line_search = filming.get("lineSearch")
    if isinstance(line_search, dict):
        for item in line_search.get("coordinateList") or []:
            if not isinstance(item, dict):
                continue
            altitude = _normalize_altitude_value(item.get("altitude"))
            if altitude is not None:
                samples.append(int(altitude))
    coord_orientation = filming.get("coordinateOrientation")
    target_coord = coord_orientation.get("coordinate") if isinstance(coord_orientation, dict) else None
    if isinstance(target_coord, dict):
        altitude = _normalize_altitude_value(target_coord.get("altitude"))
        if altitude is not None:
            samples.append(int(altitude))
    if not samples:
        return None
    return int(math.ceil(float(max(samples)) + float(_PATHDEV_FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M)))


def _enforce_inserted_waypoint_filming_altitude_floor(
    waypoints: List[Dict[str, Any]],
    *,
    inserted_waypoint_id: int,
    emit: Callable[[str], None],
) -> None:
    if not isinstance(waypoints, list):
        return
    for waypoint in waypoints:
        if _to_int((waypoint or {}).get("waypointID")) != int(inserted_waypoint_id):
            continue
        coordinate = waypoint.get("coordinate") if isinstance(waypoint, dict) else None
        if not isinstance(coordinate, dict):
            return
        current_altitude = _normalize_altitude_value(coordinate.get("altitude"))
        minimum_altitude = _filming_target_altitude_floor_m(waypoint)
        if current_altitude is None or minimum_altitude is None or int(current_altitude) >= int(minimum_altitude):
            return
        coordinate["altitude"] = int(minimum_altitude)
        waypoint["coordinate"] = coordinate
        emit(
            "[PATHDEV] raised alternate waypoint altitude above filming target "
            f"(waypoint={inserted_waypoint_id}, previousAltitude={int(current_altitude)}, "
            f"minimumAltitude={int(minimum_altitude)})."
        )
        return


def _restore_inserted_waypoint_altitude_after_runtime_profile(
    waypoints: List[Dict[str, Any]],
    *,
    inserted_waypoint_id: int,
    desired_altitude: Any,
    emit: Callable[[str], None],
) -> None:
    desired = _normalize_altitude_value(desired_altitude)
    if desired is None or not isinstance(waypoints, list):
        return

    target_index = None
    for idx, waypoint in enumerate(waypoints):
        if _to_int((waypoint or {}).get("waypointID")) == int(inserted_waypoint_id):
            target_index = idx
            break
    if target_index is None:
        return

    waypoint = waypoints[target_index]
    if not isinstance(waypoint, dict):
        return
    coordinate = waypoint.get("coordinate")
    if not isinstance(coordinate, dict):
        return

    profile_altitude = _normalize_altitude_value(coordinate.get("altitude"))
    restored_altitude = int(desired)
    if target_index + 1 < len(waypoints):
        next_waypoint = waypoints[target_index + 1]
        next_coordinate = next_waypoint.get("coordinate") if isinstance(next_waypoint, dict) else None
        next_altitude = _normalize_altitude_value(
            next_coordinate.get("altitude") if isinstance(next_coordinate, dict) else None
        )
        if isinstance(next_coordinate, dict) and next_altitude is not None:
            segment_distance_m = _coordinate_distance_m(coordinate, next_coordinate)
            if segment_distance_m > 0.0:
                speed_mps = max(_waypoint_speed_mps(next_waypoint, waypoint), 1.0)
                allowed_climb_m = _runtime_pathdev_climb_rate_mps() * (segment_distance_m / speed_mps)
                min_previous_altitude = int(math.ceil(float(next_altitude) - float(allowed_climb_m)))
                if min_previous_altitude > restored_altitude:
                    restored_altitude = min_previous_altitude
            if restored_altitude > int(next_altitude):
                restored_altitude = int(next_altitude)

    if profile_altitude is not None and restored_altitude > int(profile_altitude):
        restored_altitude = int(profile_altitude)
    if profile_altitude is not None and int(profile_altitude) == int(restored_altitude):
        return

    coordinate["altitude"] = int(restored_altitude)
    waypoint["coordinate"] = coordinate
    emit(
        "[PATHDEV] restored alternate waypoint altitude after runtime profile "
        f"(waypoint={inserted_waypoint_id}, profileAltitude={profile_altitude}, "
        f"restoredAltitude={restored_altitude})."
    )


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


def _extract_snapshot_current_waypoints(snapshot_payload: Dict[str, Any] | None) -> Dict[int, int]:
    aircraft_states = _extract_snapshot_aircraft_states(snapshot_payload)
    result: Dict[int, int] = {}
    for aircraft_id, state in aircraft_states.items():
        waypoint_id = _to_int((state or {}).get("currentWaypointID"))
        if waypoint_id is not None and waypoint_id > 0:
            result[int(aircraft_id)] = int(waypoint_id)
    return result


def _extract_snapshot_aircraft_states(snapshot_payload: Dict[str, Any] | None) -> Dict[int, Dict[str, Any]]:
    states: Dict[int, Dict[str, Any]] = {}
    if not isinstance(snapshot_payload, dict):
        return states

    memory = snapshot_payload.get("last_nonzero_waypoint_by_aircraft")
    if isinstance(memory, dict):
        for key, value in memory.items():
            aircraft_id = _to_int(key)
            waypoint_id = _to_int(value)
            if aircraft_id in (4, 5, 6) and waypoint_id is not None and waypoint_id > 0:
                state = states.setdefault(int(aircraft_id), {})
                state["currentWaypointID"] = int(waypoint_id)

    for entry in snapshot_payload.get("agent_states") or []:
        if not isinstance(entry, dict):
            continue
        aircraft_id = _to_int(entry.get("aircraftID") or entry.get("aircraftId"))
        if aircraft_id not in (4, 5, 6):
            continue
        state = states.setdefault(int(aircraft_id), {})
        raw_waypoint_id = _extract_raw_waypoint_id(entry)
        if raw_waypoint_id is not None:
            state["rawCurrentWaypointID"] = int(raw_waypoint_id)
        wp_block = entry.get("currentWaypointID") or {}
        if not wp_block:
            wp_block = (entry.get("unmannedInfo") or {}).get("currentWaypointID") or {}
        waypoint_id = _to_int((wp_block or {}).get("waypointID"))
        if waypoint_id is not None and waypoint_id > 0:
            state["currentWaypointID"] = int(waypoint_id)
        flight_mode = _to_int((entry.get("unmannedInfo") or {}).get("flightMode") or entry.get("flightMode"))
        if flight_mode is not None:
            state["flightMode"] = int(flight_mode)
        on_mission = _to_int((entry.get("unmannedInfo") or {}).get("onMission") or entry.get("onMission"))
        if on_mission is not None:
            state["onMission"] = int(on_mission)
        coord = _normalize_coordinate(entry.get("coordinate"))
        if coord is None:
            coord = _normalize_coordinate((entry.get("unmannedInfo") or {}).get("coordinate"))
        if coord is not None:
            state["coordinate"] = coord
        loiter_coord = _normalize_coordinate((entry.get("unmannedInfo") or {}).get("loiterCoordinate"))
        if loiter_coord is not None:
            state["loiterCoordinate"] = loiter_coord
        velocity = entry.get("velocity") if isinstance(entry.get("velocity"), dict) else {}
        unmanned_info = entry.get("unmannedInfo") if isinstance(entry.get("unmannedInfo"), dict) else {}
        unmanned_velocity = (
            unmanned_info.get("velocity") if isinstance(unmanned_info.get("velocity"), dict) else {}
        )
        if velocity:
            state["velocity"] = deepcopy(velocity)
        elif unmanned_velocity:
            state["velocity"] = deepcopy(unmanned_velocity)

        heading = _to_float(
            entry.get("headingDeg")
            if entry.get("headingDeg") is not None
            else entry.get("heading")
        )
        if heading is None:
            heading = _to_float(entry.get("Heading"))
        if heading is None:
            heading = _to_float(velocity.get("heading") if velocity else None)
        if heading is None:
            heading = _to_float(unmanned_velocity.get("heading") if unmanned_velocity else None)
        if heading is not None:
            state["heading"] = float(heading) % 360.0
            state["headingDeg"] = float(heading) % 360.0

        speed = _to_float(entry.get("speedMps") if entry.get("speedMps") is not None else entry.get("speed"))
        if speed is None:
            speed = _to_float(velocity.get("speed") if velocity else None)
        if speed is None:
            speed = _to_float(unmanned_velocity.get("speed") if unmanned_velocity else None)
        if speed is not None and speed > 0.0:
            state["speedMps" if speed <= 70.0 else "speed"] = float(speed)

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
                state[key] = value
        if coord is not None:
            state["currentCoordinate"] = dict(coord)
    return states


def _normalize_coordinate(payload: Any) -> Optional[Dict[str, float]]:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    alt = _to_float(payload.get("altitude"))
    if lat is None or lon is None:
        return None
    result: Dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        result["altitude"] = float(alt)
    return result


def _extract_raw_waypoint_id(entry: Dict[str, Any]) -> Optional[int]:
    if not isinstance(entry, dict):
        return None
    wp_block = entry.get("currentWaypointID") or {}
    if not wp_block:
        wp_block = (entry.get("unmannedInfo") or {}).get("currentWaypointID") or {}
    if not isinstance(wp_block, dict):
        return None
    return _to_int((wp_block or {}).get("waypointID"))


def _coord_distance_m(coord_a: Dict[str, Any] | None, coord_b: Dict[str, Any] | None) -> Optional[float]:
    a = _normalize_coordinate(coord_a)
    b = _normalize_coordinate(coord_b)
    if a is None or b is None:
        return None
    lat1 = math.radians(float(a["latitude"]))
    lon1 = math.radians(float(a["longitude"]))
    lat2 = math.radians(float(b["latitude"]))
    lon2 = math.radians(float(b["longitude"]))
    x = (lon2 - lon1) * math.cos((lat1 + lat2) * 0.5)
    y = lat2 - lat1
    return float(6371000.0 * math.sqrt((x * x) + (y * y)))


def _line_search_coordinate_list(waypoint: Dict[str, Any] | None) -> List[Dict[str, Any]]:
    if not isinstance(waypoint, dict):
        return []
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return []
    line_search = filming.get("lineSearch")
    if not isinstance(line_search, dict):
        return []
    coords = line_search.get("coordinateList")
    if not isinstance(coords, list):
        return []
    return [coord for coord in coords if isinstance(coord, dict)]


def _should_retain_previous_entry_waypoint(
    source_waypoints: List[Dict[str, Any]],
    current_index: int,
    *,
    aircraft_coordinate: Dict[str, Any] | None,
) -> tuple[bool, Optional[float], Optional[int], Optional[int]]:
    if current_index <= 0 or current_index >= len(source_waypoints):
        return False, None, None, None
    previous_wp = source_waypoints[current_index - 1]
    current_wp = source_waypoints[current_index]
    if not isinstance(previous_wp, dict) or not isinstance(current_wp, dict):
        return False, None, None, None

    previous_coords = _line_search_coordinate_list(previous_wp)
    current_coords = _line_search_coordinate_list(current_wp)
    if previous_coords or not current_coords:
        return False, None, None, None

    previous_pass_type = _to_int(previous_wp.get("waypointPassType"))
    current_pass_type = _to_int(current_wp.get("waypointPassType"))
    current_filming = current_wp.get("filmingProperty") if isinstance(current_wp.get("filmingProperty"), dict) else {}
    current_operation_mode = _to_int(current_filming.get("operationMode"))
    if current_pass_type != 3 and current_operation_mode != 2:
        return False, None, None, None
    if previous_pass_type is not None and previous_pass_type not in (1, 4):
        return False, None, None, None

    distance_to_previous = _coord_distance_m(aircraft_coordinate, previous_wp.get("coordinate"))
    if distance_to_previous is None:
        return False, None, None, None
    previous_wp_id = _to_int(previous_wp.get("waypointID"))
    current_wp_id = _to_int(current_wp.get("waypointID"))
    if distance_to_previous > float(_PATHDEV_ENTRY_WAYPOINT_RETAIN_RADIUS_M):
        return False, distance_to_previous, previous_wp_id, current_wp_id
    return True, distance_to_previous, previous_wp_id, current_wp_id


def _match_waypoint_index_by_coordinate(
    waypoint_list: List[Dict[str, Any]],
    aircraft_coordinate: Dict[str, Any] | None,
    *,
    max_distance_m: float = 3000.0,
) -> tuple[Optional[int], Optional[int], Optional[float]]:
    aircraft_coord = _normalize_coordinate(aircraft_coordinate)
    if aircraft_coord is None or not waypoint_list:
        return None, None, None

    best_index: Optional[int] = None
    best_sweep_index: Optional[int] = None
    best_distance: Optional[float] = None

    for wp_index, waypoint in enumerate(waypoint_list):
        if not isinstance(waypoint, dict):
            continue
        direct_distance = _coord_distance_m(aircraft_coord, waypoint.get("coordinate"))
        if direct_distance is not None and (best_distance is None or direct_distance < best_distance):
            best_index = int(wp_index)
            best_sweep_index = None
            best_distance = float(direct_distance)

        line_search = ((waypoint.get("filmingProperty") or {}).get("lineSearch") or {})
        coordinate_list = line_search.get("coordinateList")
        if not isinstance(coordinate_list, list):
            continue
        for sweep_index, sweep_coord in enumerate(coordinate_list):
            sweep_distance = _coord_distance_m(aircraft_coord, sweep_coord)
            if sweep_distance is None:
                continue
            if best_distance is None or sweep_distance < best_distance:
                best_index = int(wp_index)
                best_sweep_index = int(sweep_index)
                best_distance = float(sweep_distance)

    if best_distance is None or best_distance > float(max_distance_m):
        return None, None, best_distance
    return best_index, best_sweep_index, best_distance


def _estimate_line_search_cut_points(
    waypoint: Dict[str, Any] | None,
    aircraft_coordinate: Dict[str, Any] | None,
    *,
    max_distance_m: float = 400.0,
) -> tuple[int, Optional[float]]:
    if not isinstance(waypoint, dict):
        return 0, None
    _, sweep_index, best_distance = _match_waypoint_index_by_coordinate(
        [waypoint],
        aircraft_coordinate,
        max_distance_m=max_distance_m,
    )
    if sweep_index is None or sweep_index <= 0:
        return 0, best_distance
    return int(sweep_index), best_distance


def _resolve_terminal_waypoint_index(
    waypoint_list: List[Dict[str, Any]],
    *,
    raw_current_waypoint_id: Optional[int],
    flight_mode: Optional[int],
    aircraft_coordinate: Dict[str, Any] | None,
    loiter_coordinate: Dict[str, Any] | None,
    max_distance_m: float = 1500.0,
) -> tuple[Optional[int], Optional[float]]:
    raw_wp = _to_int(raw_current_waypoint_id)
    if raw_wp is not None and raw_wp > 0:
        return None, None
    if _to_int(flight_mode) != 8 and _normalize_coordinate(loiter_coordinate) is None:
        return None, None
    if not waypoint_list:
        return None, None

    anchor_coord = _normalize_coordinate(loiter_coordinate) or _normalize_coordinate(aircraft_coordinate)
    if anchor_coord is None:
        return None, None

    last_index = max(0, len(waypoint_list) - 1)
    candidate_index, _, distance_m = _match_waypoint_index_by_coordinate(
        [waypoint_list[last_index]],
        anchor_coord,
        max_distance_m=max_distance_m,
    )
    if candidate_index is None:
        return None, distance_m
    return int(last_index), distance_m


def _load_flight_path_payload(path_id: int) -> Optional[Dict[str, Any]]:
    try:
        path = db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json")
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def _collect_preserved_manned_artifact_ids(
    plan_data: Dict[str, Any],
    *,
    emit: Callable[[str], None],
) -> tuple[Set[int], Set[int], List[int]]:
    imp_ids: Set[int] = set()
    path_ids: Set[int] = set()
    aircraft_ids: List[int] = []
    for aircraft_entry in plan_data.get("aircraftList") or []:
        if not isinstance(aircraft_entry, dict):
            continue
        aircraft_id = _to_int(aircraft_entry.get("aircraftID"))
        if aircraft_id is None or int(aircraft_id) not in (1, 2, 3):
            continue
        package_id = _to_int(
            aircraft_entry.get("individualMissionPackageID")
            or aircraft_entry.get("individualMissionPlanPackageID")
        )
        if package_id is None or int(package_id) <= 0:
            continue
        aircraft_ids.append(int(aircraft_id))
        imp_ids.add(int(package_id))
        try:
            imp_src = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(package_id)}.json")
            imp_data = json.loads(imp_src.read_text(encoding="utf-8"))
        except Exception as exc:
            emit(
                f"[PATHDEV][LAH] preserved package load failed "
                f"(aircraft={aircraft_id}, IMP={package_id}, error={exc})."
            )
            continue
        for mission in imp_data.get("individualMissionList") or []:
            if not isinstance(mission, dict):
                continue
            path_id = _to_int(mission.get("pathID"))
            if path_id is not None and int(path_id) > 0:
                path_ids.add(int(path_id))
    return imp_ids, path_ids, sorted(set(aircraft_ids))


def _resolve_other_uav_artifacts(
    *,
    source_plan_id: int,
    aircraft_id: int,
    current_waypoint_id: Optional[int],
    raw_current_waypoint_id: Optional[int],
    flight_mode: Optional[int],
    aircraft_coordinate: Dict[str, Any] | None,
    loiter_coordinate: Dict[str, Any] | None,
    emit: Callable[[str], None],
) -> Optional[PlanMissionArtifacts]:
    current_wp = _to_int(current_waypoint_id)
    raw_current_wp = _to_int(raw_current_waypoint_id)
    prefer_terminal_hold = (
        raw_current_wp is not None
        and raw_current_wp <= 0
        and (_to_int(flight_mode) == 8 or _normalize_coordinate(loiter_coordinate) is not None)
    )
    if not prefer_terminal_hold and current_wp is not None and current_wp > 0:
        artifacts = _resolve_plan_artifacts(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
            current_waypoint_id=int(current_wp),
            emit=emit,
            allow_first_mission_fallback=False,
        )
        if artifacts is not None:
            return artifacts
        emit(
            f"[PATHDEV][OTHER] aircraft {aircraft_id} currentWP={current_wp} not found in source plan; "
            "falling back to first active mission."
        )

    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = json.loads(plan_src.read_text(encoding="utf-8"))
    except Exception as exc:
        emit(f"[PATHDEV][OTHER] failed to load source MissionPlan {source_plan_id}: {exc}")
        return None

    package_id = None
    for aircraft_entry in plan_data.get("aircraftList") or []:
        if _to_int((aircraft_entry or {}).get("aircraftID")) == int(aircraft_id):
            package_id = _to_int((aircraft_entry or {}).get("individualMissionPackageID"))
            break
    if package_id is None or package_id <= 0:
        emit(f"[PATHDEV][OTHER] aircraft {aircraft_id} package not found in MissionPlan {source_plan_id}.")
        return None

    try:
        imp_src = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(package_id)}.json")
        imp_data = json.loads(imp_src.read_text(encoding="utf-8"))
    except Exception as exc:
        emit(f"[PATHDEV][OTHER] failed to load IMP {package_id} for aircraft {aircraft_id}: {exc}")
        return None

    missions = imp_data.get("individualMissionList") or []
    fallback_mission = None
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        mission_id = _to_int(mission.get("individualMissionID"))
        path_id = _to_int(mission.get("pathID"))
        if mission_id is None or mission_id <= 0 or path_id is None or path_id <= 0:
            continue
        if not bool(mission.get("isDone")):
            fallback_mission = mission
            break
    if fallback_mission is None:
        for mission in missions:
            if not isinstance(mission, dict):
                continue
            mission_id = _to_int(mission.get("individualMissionID"))
            path_id = _to_int(mission.get("pathID"))
            if mission_id is not None and mission_id > 0 and path_id is not None and path_id > 0:
                fallback_mission = mission
                break
    if fallback_mission is None:
        emit(f"[PATHDEV][OTHER] no active mission found for aircraft {aircraft_id}.")
        return None

    mission_id = _to_int(fallback_mission.get("individualMissionID")) or 0
    path_id = _to_int(fallback_mission.get("pathID")) or 0
    if mission_id <= 0 or path_id <= 0:
        emit(f"[PATHDEV][OTHER] fallback mission for aircraft {aircraft_id} is missing identifiers.")
        return None

    fp_data = _load_flight_path_payload(path_id)
    waypoint_list = list((fp_data or {}).get("waypointList") or (fp_data or {}).get("lahWaypointList") or [])
    resolved_current_wp: Optional[int] = None
    previous_wp: Optional[int] = None
    if waypoint_list:
        terminal_index, terminal_distance = _resolve_terminal_waypoint_index(
            waypoint_list,
            raw_current_waypoint_id=raw_current_waypoint_id,
            flight_mode=flight_mode,
            aircraft_coordinate=aircraft_coordinate,
            loiter_coordinate=loiter_coordinate,
        )
        if terminal_index is not None and 0 <= terminal_index < len(waypoint_list):
            resolved_current_wp = _to_int((waypoint_list[terminal_index] or {}).get("waypointID"))
            if terminal_index > 0:
                previous_wp = _to_int((waypoint_list[terminal_index - 1] or {}).get("waypointID"))
            emit(
                f"[PATHDEV][OTHER] aircraft {aircraft_id} raw currentWP=0 in loiter/terminal state -> "
                f"keeping last waypoint {resolved_current_wp} (idx={terminal_index}, distance={terminal_distance:.1f}m)."
            )
        if resolved_current_wp is None and current_wp is not None and current_wp > 0:
            for idx, waypoint in enumerate(waypoint_list):
                if _to_int((waypoint or {}).get("waypointID")) == int(current_wp):
                    resolved_current_wp = int(current_wp)
                    if idx > 0:
                        previous_wp = _to_int((waypoint_list[idx - 1] or {}).get("waypointID"))
                    break
        if resolved_current_wp is None:
            coord_index, _, coord_distance = _match_waypoint_index_by_coordinate(
                waypoint_list,
                aircraft_coordinate,
            )
            if coord_index is not None and 0 <= coord_index < len(waypoint_list):
                resolved_current_wp = _to_int((waypoint_list[coord_index] or {}).get("waypointID"))
                if coord_index > 0:
                    previous_wp = _to_int((waypoint_list[coord_index - 1] or {}).get("waypointID"))
                emit(
                    f"[PATHDEV][OTHER] aircraft {aircraft_id} coordinate fallback matched "
                    f"waypoint {resolved_current_wp} (idx={coord_index}, distance={coord_distance:.1f}m)."
                )
        if resolved_current_wp is None:
            first_active_idx = next(
                (idx for idx, waypoint in enumerate(waypoint_list) if not bool((waypoint or {}).get("isDone"))),
                None,
            )
            if first_active_idx is None:
                first_active_idx = 0
            if 0 <= first_active_idx < len(waypoint_list):
                resolved_current_wp = _to_int((waypoint_list[first_active_idx] or {}).get("waypointID"))
                if first_active_idx > 0:
                    previous_wp = _to_int((waypoint_list[first_active_idx - 1] or {}).get("waypointID"))

    return PlanMissionArtifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        individual_mission_package_id=int(package_id),
        individual_mission_id=int(mission_id),
        path_id=int(path_id),
        current_waypoint_id=resolved_current_wp,
        previous_waypoint_id=previous_wp,
    )


def _build_other_uav_resume_waypoints(
    waypoint_list: List[Dict[str, Any]],
    *,
    current_waypoint_id: Optional[int],
    previous_waypoint_id: Optional[int],
    raw_current_waypoint_id: Optional[int],
    flight_mode: Optional[int],
    aircraft_coordinate: Dict[str, Any] | None,
    loiter_coordinate: Dict[str, Any] | None,
    sweep_progress_entry: Dict[str, Any] | None,
    allow_line_search_point_trim: bool = True,
    emit: Callable[[str], None],
    log_prefix: str,
) -> tuple[List[Dict[str, Any]], Optional[int]]:
    source_waypoints = [deepcopy(item) for item in (waypoint_list or []) if isinstance(item, dict)]
    if not source_waypoints:
        return [], previous_waypoint_id

    removed_waypoint_id = _to_int(previous_waypoint_id)
    done_waypoints: List[Dict[str, Any]] = []
    resume_waypoints: List[Dict[str, Any]] = deepcopy(source_waypoints)
    current_index = None
    terminal_exception_applied = False
    current_wp = _to_int(current_waypoint_id)
    terminal_index, terminal_distance = _resolve_terminal_waypoint_index(
        source_waypoints,
        raw_current_waypoint_id=raw_current_waypoint_id,
        flight_mode=flight_mode,
        aircraft_coordinate=aircraft_coordinate,
        loiter_coordinate=loiter_coordinate,
    )
    if terminal_index is not None:
        current_index = int(terminal_index)
        current_wp = _to_int((source_waypoints[current_index] or {}).get("waypointID"))
        terminal_exception_applied = True
        emit(
            f"{log_prefix} terminal currentWP=0 exception applied "
            f"(keepLastWP={current_wp}, idx={current_index}, distance={terminal_distance:.1f}m)."
        )
    if current_wp is not None and current_wp > 0:
        if current_index is None:
            for idx, waypoint in enumerate(source_waypoints):
                if _to_int((waypoint or {}).get("waypointID")) == int(current_wp):
                    current_index = idx
                    break
    if current_index is None:
        coord_index, _, coord_distance = _match_waypoint_index_by_coordinate(
            source_waypoints,
            aircraft_coordinate,
        )
        if coord_index is not None:
            current_index = int(coord_index)
            current_wp = _to_int((source_waypoints[current_index] or {}).get("waypointID"))
            emit(
                f"{log_prefix} coordinate currentWP fallback applied "
                f"(matchedWP={current_wp}, idx={current_index}, distance={coord_distance:.1f}m)."
            )

    if current_index is not None:
        if not terminal_exception_applied:
            retain_previous, retain_distance, entry_wp_id, sweep_wp_id = _should_retain_previous_entry_waypoint(
                source_waypoints,
                int(current_index),
                aircraft_coordinate=aircraft_coordinate,
            )
            if retain_previous:
                current_index = int(current_index) - 1
                current_wp = _to_int((source_waypoints[current_index] or {}).get("waypointID"))
                emit(
                    f"{log_prefix} entry waypoint retained before first sweep "
                    f"(entryWP={entry_wp_id}, sweepWP={sweep_wp_id}, "
                    f"distance={retain_distance:.1f}m)."
                )
        done_waypoints = deepcopy(source_waypoints[:current_index]) if current_index > 0 else []
        resume_waypoints = deepcopy(source_waypoints[current_index:])
        if done_waypoints:
            removed_waypoint_id = _to_int((done_waypoints[-1] or {}).get("waypointID"))
        else:
            removed_waypoint_id = None
        emit(
            f"{log_prefix} currentWP trim applied "
            f"(currentWP={current_wp}, removedPrefix={len(done_waypoints)})."
        )
    else:
        trimmed_waypoints, prefix_removed_id = trim_waypoints_by_is_done_prefix(deepcopy(source_waypoints))
        if prefix_removed_id is not None:
            prefix_count = max(0, len(source_waypoints) - len(trimmed_waypoints))
            done_waypoints = deepcopy(source_waypoints[:prefix_count])
            resume_waypoints = trimmed_waypoints
            removed_waypoint_id = prefix_removed_id
            emit(
                f"{log_prefix} isDone prefix trim applied "
                f"(lastRemovedWP={removed_waypoint_id}, removedPrefix={prefix_count})."
            )

    done_sweep_points = count_sweep_points_in_waypoints(done_waypoints)
    raw_cut_points = (
        physical_sweep_cut_points(
            sweep_progress_entry,
            default_buffer_seconds=DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS,
        )
        if bool(allow_line_search_point_trim)
        else 0
    )
    cut_points = max(0, int(raw_cut_points) - int(done_sweep_points))
    if terminal_exception_applied:
        pass
    elif cut_points > 0 and resume_waypoints:
        resume_waypoints, removed_points = trim_waypoints_by_sweep_points(
            resume_waypoints,
            cut_points,
            preserve_waypoints=True,
        )
        if removed_points > 0:
            emit(
                f"{log_prefix} sweep trim applied "
                f"(cutPoints={removed_points}, rawCutPoints={raw_cut_points}, "
                f"doneSweepPoints={done_sweep_points})."
            )
            _sync_waypoint_coord_to_first_line_search_point(resume_waypoints[0] if resume_waypoints else None)
    elif bool(allow_line_search_point_trim) and resume_waypoints:
        estimated_cut_points, estimated_distance = _estimate_line_search_cut_points(
            resume_waypoints[0],
            aircraft_coordinate,
        )
        if estimated_cut_points > 0:
            resume_waypoints, removed_points = trim_waypoints_by_sweep_points(
                resume_waypoints,
                estimated_cut_points,
                preserve_waypoints=True,
            )
            if removed_points > 0:
                emit(
                    f"{log_prefix} coordinate sweep trim applied "
                    f"(cutPoints={removed_points}, distance={estimated_distance:.1f}m)."
                )
                _sync_waypoint_coord_to_first_line_search_point(resume_waypoints[0] if resume_waypoints else None)
    elif not bool(allow_line_search_point_trim) and resume_waypoints:
        emit(
            f"{log_prefix} line mission sweep-point trim skipped "
            "(preserving executable lineSearch geometry)."
        )

    if not resume_waypoints:
        fallback_index = None
        if current_index is not None and 0 <= current_index < len(source_waypoints):
            fallback_index = min(current_index, len(source_waypoints) - 1)
        if fallback_index is None:
            fallback_index = next(
                (idx for idx, waypoint in enumerate(source_waypoints) if not bool((waypoint or {}).get("isDone"))),
                None,
            )
        if fallback_index is None:
            fallback_index = max(0, len(source_waypoints) - 1)
        resume_waypoints = [deepcopy(source_waypoints[fallback_index])]
        if fallback_index > 0:
            removed_waypoint_id = _to_int((source_waypoints[fallback_index - 1] or {}).get("waypointID"))
        emit(
            f"{log_prefix} resume fallback applied "
            f"(fallbackWP={_to_int((resume_waypoints[0] or {}).get('waypointID'))})."
        )

    _align_first_resume_waypoint_altitude(resume_waypoints)
    reference_coord = aircraft_coordinate if isinstance(aircraft_coordinate, dict) else None
    search_speed_weight = get_runtime_float("search_speed_weight", 1.1)
    recomputed = recompute_line_search_speed_from_geometry(
        resume_waypoints,
        first_reference_coord=reference_coord,
        speed_scale=float(search_speed_weight),
        only_increase=True,
    )
    if recomputed > 0:
        emit(
            f"{log_prefix} resume searchSpeed geometry recomputed "
            f"(weight={float(search_speed_weight):.2f}, waypoints={recomputed})."
        )
    for waypoint in resume_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    relink_waypoints(resume_waypoints)
    return resume_waypoints, removed_waypoint_id


def _sync_waypoint_coord_to_first_line_search_point(waypoint: Dict[str, Any] | None) -> None:
    if not isinstance(waypoint, dict):
        return
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return
    line_search = filming.get("lineSearch")
    if not isinstance(line_search, dict):
        return
    coords = line_search.get("coordinateList")
    if not isinstance(coords, list) or not coords:
        return
    first = coords[0]
    if not isinstance(first, dict):
        return
    coord = waypoint.get("coordinate")
    if not isinstance(coord, dict):
        coord = {}
    preserved_altitude = _normalize_altitude_value(coord.get("altitude"))
    if "latitude" in first:
        coord["latitude"] = first.get("latitude")
    if "longitude" in first:
        coord["longitude"] = first.get("longitude")
    # Keep the mission waypoint altitude. The line-search coordinates carry
    # terrain-following sample altitudes, so copying that value into the
    # trimmed path's first anchor makes only the first WP collapse to a much
    # lower altitude than the rest of the resumed mission.
    if preserved_altitude is not None:
        coord["altitude"] = int(preserved_altitude)
    elif "altitude" in first:
        coord["altitude"] = first.get("altitude")
    waypoint["coordinate"] = coord


def _align_first_resume_waypoint_altitude(waypoints: List[Dict[str, Any]]) -> None:
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        return
    first = waypoints[0] if isinstance(waypoints[0], dict) else None
    if not isinstance(first, dict):
        return
    filming = first.get("filmingProperty")
    if not isinstance(filming, dict):
        return
    line_search = filming.get("lineSearch")
    if not isinstance(line_search, dict):
        return
    coord = first.get("coordinate")
    if not isinstance(coord, dict):
        return
    reference_altitudes: List[int] = []
    for waypoint in waypoints[1:4]:
        if not isinstance(waypoint, dict):
            continue
        next_coord = waypoint.get("coordinate")
        if not isinstance(next_coord, dict):
            continue
        altitude = _normalize_altitude_value(next_coord.get("altitude"))
        if altitude is None:
            continue
        reference_altitudes.append(int(altitude))
    if not reference_altitudes:
        return
    coord["altitude"] = int(round(sum(reference_altitudes) / len(reference_altitudes)))
    first["coordinate"] = coord


def _mission_has_imaging_geometry(
    mission: Dict[str, Any] | None,
    waypoint_list: List[Dict[str, Any]] | None,
) -> bool:
    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        info = {}
    mission_type = _to_int(info.get("individualMissionType"))
    if mission_type in (3, 6):
        return True
    if isinstance(info.get("lineList"), list) and info.get("lineList"):
        return True
    if isinstance(info.get("areaList"), list) and info.get("areaList"):
        return True
    return count_sweep_points_in_waypoints(list(waypoint_list or [])) > 0


def _waypoint_has_filming_property(waypoint: Dict[str, Any] | None) -> bool:
    if not isinstance(waypoint, dict):
        return False
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return False
    line_search = filming.get("lineSearch")
    if isinstance(line_search, dict) and len(line_search.get("coordinateList") or []) >= 2:
        return True
    if isinstance(filming.get("coordinateOrientation"), dict):
        return True
    operation_mode = _to_int(filming.get("operationMode"))
    return operation_mode is not None and int(operation_mode) > 0


def _current_waypoint_has_filming_property(
    waypoint_list: List[Dict[str, Any]] | None,
    current_waypoint_id: int | None,
) -> bool:
    current_wp = _to_int(current_waypoint_id)
    if current_wp is None or current_wp <= 0:
        return False
    for waypoint in waypoint_list or []:
        if not isinstance(waypoint, dict):
            continue
        if _to_int(waypoint.get("waypointID")) != int(current_wp):
            continue
        return _waypoint_has_filming_property(waypoint)
    return False


def _mission_is_line_geometry(mission: Dict[str, Any] | None) -> bool:
    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        info = {}
    mission_type = _to_int(info.get("individualMissionType"))
    if mission_type == 6:
        return True
    line_list = info.get("lineList")
    area_list = info.get("areaList")
    return bool(isinstance(line_list, list) and line_list) and not bool(
        isinstance(area_list, list) and area_list
    )


def _mission_related_input_id(mission: Dict[str, Any] | None, detail: Dict[str, Any] | None = None) -> Optional[int]:
    if isinstance(mission, dict):
        related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
        for value in (
            related.get("inputMissionID"),
            related.get("input_mission_id"),
            mission.get("inputMissionID"),
            mission.get("input_mission_id"),
        ):
            input_id = _to_int(value)
            if input_id is not None and input_id > 0:
                return int(input_id)
    if isinstance(detail, dict):
        for value in (
            detail.get("currentInputMissionID"),
            detail.get("targetInputMissionID"),
            detail.get("inputMissionID"),
        ):
            input_id = _to_int(value)
            if input_id is not None and input_id > 0:
                return int(input_id)
    return None


def _generated_current_mission_refs(
    aircraft_imp_ids: Dict[int, int],
    *,
    current_input_id: int,
) -> Dict[int, List[Dict[str, int]]]:
    refs: Dict[int, List[Dict[str, int]]] = {}
    for aircraft_id, imp_id in sorted((aircraft_imp_ids or {}).items()):
        try:
            imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(imp_id)}.json")
            imp_data = json.loads(imp_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        mission_refs: List[Dict[str, int]] = []
        for mission in imp_data.get("individualMissionList") or []:
            if not isinstance(mission, dict):
                continue
            if _mission_related_input_id(mission) != int(current_input_id):
                continue
            mission_id = _to_int(mission.get("individualMissionID"))
            path_id = _to_int(mission.get("pathID"))
            if mission_id is None or path_id is None or mission_id <= 0 or path_id <= 0:
                continue
            mission_refs.append(
                {
                    "individualMissionID": int(mission_id),
                    "pathID": int(path_id),
                }
            )
        if mission_refs:
            refs[int(aircraft_id)] = mission_refs
    return refs


def _first_waypoint_id_for_path(path_id: int) -> Optional[int]:
    try:
        fp_data = _load_flight_path_payload(int(path_id))
    except Exception:
        fp_data = None
    waypoints = (fp_data or {}).get("waypointList") or (fp_data or {}).get("lahWaypointList") or []
    for waypoint in waypoints:
        waypoint_id = _to_int((waypoint or {}).get("waypointID"))
        if waypoint_id is not None and waypoint_id > 0:
            return int(waypoint_id)
    return None


def _pathdev_collab_entry_context_override(
    *,
    aircraft_id: int,
    alternate_coordinate: Dict[str, Any],
    state: Dict[str, Any] | None,
) -> Dict[int, Dict[str, Any]]:
    state_row = dict(state or {})
    entry_coord = _pathdev_coordinate_for_output(alternate_coordinate)
    context_row: Dict[str, Any] = dict(state_row)
    context_row["aircraftID"] = int(aircraft_id)
    context_row["coordinate"] = deepcopy(entry_coord)
    current_coord = _normalize_coordinate(state_row.get("coordinate")) or _normalize_coordinate(
        state_row.get("currentCoordinate")
    )
    if current_coord is not None:
        context_row["currentCoordinate"] = deepcopy(current_coord)
    return {int(aircraft_id): context_row}


def _last_imaging_coordinate(
    waypoint_list: List[Dict[str, Any]] | None,
    *,
    fallback_coordinate: Dict[str, Any] | None = None,
) -> Optional[Dict[str, Any]]:
    for waypoint in reversed([item for item in (waypoint_list or []) if isinstance(item, dict)]):
        waypoint_coord = _normalize_coordinate(waypoint.get("coordinate"))
        altitude = _normalize_altitude_value((waypoint_coord or {}).get("altitude"))
        line_search = ((waypoint.get("filmingProperty") or {}).get("lineSearch") or {})
        coord_list = line_search.get("coordinateList")
        if isinstance(coord_list, list):
            for coord in reversed(coord_list):
                normalized = _normalize_coordinate(coord)
                if normalized is None:
                    continue
                if altitude is not None:
                    normalized["altitude"] = float(altitude)
                elif waypoint_coord is not None and waypoint_coord.get("altitude") is not None:
                    normalized["altitude"] = float(waypoint_coord["altitude"])
                return normalized
        if waypoint_coord is not None:
            return waypoint_coord
    return _normalize_coordinate(fallback_coordinate)


def _pathdev_coordinate_for_output(coord: Dict[str, Any]) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "latitude": float(coord["latitude"]),
        "longitude": float(coord["longitude"]),
    }
    altitude = _normalize_altitude_value(coord.get("altitude"))
    if altitude is not None:
        result["altitude"] = int(altitude)
    return result


def _ctx_source_plan_override(ctx: Dict[str, Any] | None) -> Optional[int]:
    if not isinstance(ctx, dict):
        return None
    for key in (
        "currentMissionPlanID",
        "sourceMissionPlanID",
        "currentPlanID",
        "sourcePlanID",
        "current_mission_plan_id",
        "source_mission_plan_id",
        "source_plan_id",
    ):
        value = _to_int(ctx.get(key))
        if value is not None and value > 0:
            return int(value)
    return None


def _resolve_plan_artifacts_from_explicit_detail(
    *,
    source_plan_id: int,
    aircraft_id: int,
    current_waypoint_id: int,
    detail: Dict[str, Any],
    emit: Callable[[str], None],
) -> Optional[PlanMissionArtifacts]:
    package_id = _to_int(
        detail.get("individualMissionPackageID")
        or detail.get("individualMissionPlanPackageID")
        or detail.get("individualMissionPackageId")
    )
    mission_id = _to_int(detail.get("individualMissionID") or detail.get("individualMissionId"))
    path_id = _to_int(detail.get("pathID") or detail.get("pathId"))
    if package_id is None or package_id <= 0 or path_id is None or path_id <= 0:
        return None

    try:
        imp_path = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(package_id)}.json")
        imp_data = json.loads(imp_path.read_text(encoding="utf-8"))
        fp_data = _load_flight_path_payload(int(path_id))
    except Exception as exc:
        emit(f"[PATHDEV] explicit detail artifact fallback load failed: {exc}")
        return None
    if not isinstance(fp_data, dict):
        return None

    waypoints = fp_data.get("waypointList") or fp_data.get("lahWaypointList") or fp_data.get("uavWaypointList") or []
    waypoint_ids: List[int] = []
    for waypoint in waypoints:
        waypoint_id = _to_int((waypoint or {}).get("waypointID"))
        if waypoint_id is not None and waypoint_id > 0:
            waypoint_ids.append(int(waypoint_id))
    previous_wp = None
    resolved_current_wp = int(current_waypoint_id)
    if int(current_waypoint_id) in waypoint_ids:
        idx = waypoint_ids.index(int(current_waypoint_id))
        previous_wp = waypoint_ids[idx - 1] if idx > 0 else None
    elif waypoint_ids:
        resolved_current_wp = int(waypoint_ids[0])
        emit(
            "[PATHDEV] explicit detail artifact fallback: current waypoint not found; "
            f"using first waypoint {resolved_current_wp} for path {int(path_id)}."
        )

    resolved_mission_id = mission_id
    for mission in imp_data.get("individualMissionList") or []:
        if not isinstance(mission, dict):
            continue
        candidate_path_id = _to_int(mission.get("pathID"))
        candidate_mission_id = _to_int(mission.get("individualMissionID"))
        if candidate_path_id == int(path_id) and candidate_mission_id is not None:
            resolved_mission_id = int(candidate_mission_id)
            break
    if resolved_mission_id is None or resolved_mission_id <= 0:
        return None
    emit(
        "[PATHDEV] source artifact resolved from explicit path-deviation detail "
        f"(sourcePlan={source_plan_id}, aircraft={aircraft_id}, "
        f"IMP={package_id}, mission={resolved_mission_id}, path={path_id})."
    )
    return PlanMissionArtifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        individual_mission_package_id=int(package_id),
        individual_mission_id=int(resolved_mission_id),
        path_id=int(path_id),
        current_waypoint_id=int(resolved_current_wp),
        previous_waypoint_id=previous_wp,
    )


def _build_completion_hold_waypoint(
    *,
    template_waypoint: Dict[str, Any] | None,
    coordinate: Dict[str, Any],
    waypoint_id: int,
) -> Dict[str, Any]:
    coord = _pathdev_coordinate_for_output(coordinate)
    waypoint = deepcopy(template_waypoint if isinstance(template_waypoint, dict) else {})
    waypoint["waypointID"] = int(waypoint_id)
    waypoint["coordinate"] = deepcopy(coord)
    waypoint["speed"] = float(_PATHDEV_COMPLETE_HOLD_SPEED_MPS)
    waypoint["eta"] = int(_PATHDEV_COMPLETE_HOLD_SECONDS)
    waypoint["ecf"] = float(waypoint.get("ecf") or 0.0)
    waypoint["nextWaypointID"] = 0
    waypoint["waypointPassType"] = 2
    filming = deepcopy(waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {})
    filming.pop("lineSearch", None)
    filming.pop("autoTracking", None)
    filming["fieldOfView"] = float(filming.get("fieldOfView") or 5.0)
    filming["sensorType"] = int(_to_int(filming.get("sensorType")) or 1)
    filming["operationMode"] = 1
    filming["coordinateOrientation"] = {"coordinate": deepcopy(coord)}
    waypoint["filmingProperty"] = filming
    normalize_filming_target_altitudes_in_waypoints([waypoint])
    waypoint["loiterProperty"] = {
        "radius": int(_PATHDEV_COMPLETE_HOLD_RADIUS_M),
        "direction": 1,
        "time": int(_PATHDEV_COMPLETE_HOLD_SECONDS),
        "speed": int(round(_PATHDEV_COMPLETE_HOLD_SPEED_MPS)),
    }
    waypoint["isDone"] = False
    return waypoint


def _build_completion_hold_mission(
    *,
    template_mission: Dict[str, Any],
    individual_mission_id: int,
    path_id: int,
    coordinate: Dict[str, Any],
) -> Dict[str, Any]:
    mission = deepcopy(template_mission)
    mission["individualMissionID"] = int(individual_mission_id)
    mission["pathID"] = int(path_id)
    mission["isDone"] = False
    coord = _pathdev_coordinate_for_output(coordinate)
    info = deepcopy(mission.get("individualMissionInfo") if isinstance(mission.get("individualMissionInfo"), dict) else {})
    info["individualMissionType"] = 7
    info["patternType"] = 10
    info["autoZoomIn"] = False
    info["targetID"] = None
    info["coordinateList"] = [deepcopy(coord)]
    info["lineList"] = []
    info["areaList"] = []
    info["SPEED"] = float(_PATHDEV_COMPLETE_HOLD_SPEED_MPS)
    mission["individualMissionInfo"] = info
    return mission


def _build_completion_hold_flight_path(
    *,
    template_path: Dict[str, Any],
    aircraft_id: int,
    individual_mission_id: int,
    path_id: int,
    coordinate: Dict[str, Any],
    template_waypoint: Dict[str, Any] | None,
    now_ms: int,
    waypoint_id: int,
) -> tuple[Dict[str, Any], int]:
    hold_waypoint = _build_completion_hold_waypoint(
        template_waypoint=template_waypoint,
        coordinate=coordinate,
        waypoint_id=int(waypoint_id),
    )
    payload = deepcopy(template_path if isinstance(template_path, dict) else {})
    payload["pathID"] = int(path_id)
    payload["timestamp"] = int(now_ms)
    payload["aircraftID"] = int(aircraft_id)
    payload["individualMissionID"] = int(individual_mission_id)
    _set_source_field(payload, "MMR")
    payload["waypointList"] = [hold_waypoint]
    if "lahWaypointList" in payload:
        payload["lahWaypointList"] = [deepcopy(hold_waypoint)]
    return payload, int(hold_waypoint["waypointID"])


def _build_other_uav_current_package(
    *,
    source_plan_id: int,
    aircraft_id: int,
    current_waypoint_id: Optional[int],
    raw_current_waypoint_id: Optional[int],
    flight_mode: Optional[int],
    on_mission: Optional[int],
    aircraft_coordinate: Dict[str, Any] | None,
    loiter_coordinate: Dict[str, Any] | None,
    sweep_progress: Dict[int, Dict[str, Any]] | None,
    now_ms: int,
    emit: Callable[[str], None],
) -> Optional[Dict[str, Any]]:
    log_prefix = f"[PATHDEV][OTHER][UAV{aircraft_id}]"
    artifacts = _resolve_other_uav_artifacts(
        source_plan_id=int(source_plan_id),
        aircraft_id=int(aircraft_id),
        current_waypoint_id=current_waypoint_id,
        raw_current_waypoint_id=raw_current_waypoint_id,
        flight_mode=flight_mode,
        aircraft_coordinate=aircraft_coordinate,
        loiter_coordinate=loiter_coordinate,
        emit=emit,
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
        emit(f"{log_prefix} failed to load source artifacts: {exc}")
        return None

    mission_list = imp_data.get("individualMissionList") or []
    target_index = None
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

    source_waypoints = list(fp_data.get("waypointList") or fp_data.get("lahWaypointList") or [])
    if not source_waypoints:
        emit(f"{log_prefix} FlightPath {artifacts.path_id} has no waypointList.")
        return None

    target_mission = deepcopy(mission_list[target_index])
    if _mission_has_imaging_geometry(target_mission, source_waypoints) or _current_waypoint_has_filming_property(
        source_waypoints,
        artifacts.current_waypoint_id,
    ):
        emit(
            f"{log_prefix} skipped: filming/sweep mission is protected from path-deviation other-UAV update "
            f"(mission={int(artifacts.individual_mission_id)}, path={int(artifacts.path_id)}, "
            f"currentWP={artifacts.current_waypoint_id or '-'})."
        )
        return None
    if _to_int(on_mission) == 2 and _mission_has_imaging_geometry(target_mission, source_waypoints):
        hold_coord = _last_imaging_coordinate(source_waypoints, fallback_coordinate=aircraft_coordinate)
        if hold_coord is None:
            emit(f"{log_prefix} onMission=2 completion guard could not resolve hold coordinate.")
            return None

        [new_imp_id] = _reserve_imp_ids(1)
        [new_individual_id] = _reserve_individual_mission_ids(1)
        [new_path_id] = _reserve_path_ids(int(aircraft_id), 1)
        hold_waypoint_id = int(_reserve_waypoint_block(1))

        updated_imp = deepcopy(imp_data)
        updated_imp["individualMissionPackageID"] = int(new_imp_id)
        updated_imp["timestamp"] = int(now_ms)
        _set_source_field(updated_imp, "MMR")
        updated_mission_list = updated_imp.get("individualMissionList") or []

        completed_mission = deepcopy(updated_mission_list[target_index])
        completed_mission["isDone"] = True
        updated_mission_list[target_index] = completed_mission
        hold_mission = _build_completion_hold_mission(
            template_mission=target_mission,
            individual_mission_id=int(new_individual_id),
            path_id=int(new_path_id),
            coordinate=hold_coord,
        )
        updated_mission_list.insert(target_index + 1, hold_mission)

        template_waypoint = source_waypoints[-1] if source_waypoints else None
        hold_fp, hold_waypoint_id = _build_completion_hold_flight_path(
            template_path=fp_data,
            aircraft_id=int(aircraft_id),
            individual_mission_id=int(new_individual_id),
            path_id=int(new_path_id),
            coordinate=hold_coord,
            template_waypoint=template_waypoint,
            now_ms=int(now_ms),
            waypoint_id=int(hold_waypoint_id),
        )
        _apply_runtime_flyover_to_flight_path_payload(hold_fp)
        sanitize_flight_path_payload_filming_altitudes(hold_fp)

        imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
        fp_dest = db_paths.get_db_subpath("FlightPath", f"{new_path_id}.json")
        for path in (imp_dest, fp_dest):
            path.parent.mkdir(parents=True, exist_ok=True)
        validate_generated_artifact_payloads(
            individual_mission_plans=[updated_imp],
            flight_paths=[hold_fp],
            scope=f"pathDeviation:otherUavHold:{new_imp_id}",
            allow_existing_db_artifacts=True,
            log=emit,
        )
        write_json(imp_dest, updated_imp, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        write_json(fp_dest, hold_fp, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
        removed_waypoint_id = _to_int((source_waypoints[-1] or {}).get("waypointID")) if source_waypoints else None
        emit(
            f"{log_prefix} onMission=2 imaging mission completed; inserted "
            f"{_PATHDEV_COMPLETE_HOLD_SECONDS}s hold instead of reviving sweep -> "
            f"IMP:{imp_dest.name}, FP:{fp_dest.name}, holdWP={hold_waypoint_id}"
        )
        return {
            "aircraftID": int(aircraft_id),
            "sourceIndividualMissionPackageID": int(artifacts.individual_mission_package_id),
            "sourceIndividualMissionID": int(artifacts.individual_mission_id),
            "sourcePathID": int(artifacts.path_id),
            "resolvedCurrentWaypointID": int(artifacts.current_waypoint_id)
            if artifacts.current_waypoint_id is not None
            else None,
            "generatedIndividualMissionPackageID": int(new_imp_id),
            "generatedIndividualMissionID": int(new_individual_id),
            "generatedPathID": int(new_path_id),
            "removedWaypointID": int(removed_waypoint_id) if removed_waypoint_id is not None else None,
            "completedByOnMission2": True,
            "holdWaypointID": int(hold_waypoint_id),
            "holdSeconds": int(_PATHDEV_COMPLETE_HOLD_SECONDS),
        }

    sweep_progress_entry = None
    if isinstance(sweep_progress, dict):
        sweep_progress_entry = sweep_progress.get(int(artifacts.path_id))
    resume_waypoints, removed_waypoint_id = _build_other_uav_resume_waypoints(
        source_waypoints,
        current_waypoint_id=artifacts.current_waypoint_id,
        previous_waypoint_id=artifacts.previous_waypoint_id,
        raw_current_waypoint_id=raw_current_waypoint_id,
        flight_mode=flight_mode,
        aircraft_coordinate=aircraft_coordinate,
        loiter_coordinate=loiter_coordinate,
        sweep_progress_entry=sweep_progress_entry,
        allow_line_search_point_trim=not _mission_is_line_geometry(target_mission),
        emit=emit,
        log_prefix=log_prefix,
    )
    if not resume_waypoints:
        emit(f"{log_prefix} resume waypoint list became empty; skipping update.")
        return None

    [new_imp_id] = _reserve_imp_ids(1)
    [new_individual_id] = _reserve_individual_mission_ids(1)
    [new_path_id] = _reserve_path_ids(int(aircraft_id), 1)

    updated_imp = deepcopy(imp_data)
    updated_imp["individualMissionPackageID"] = int(new_imp_id)
    updated_imp["timestamp"] = int(now_ms)
    _set_source_field(updated_imp, "MMR")
    updated_mission_list = updated_imp.get("individualMissionList") or []

    updated_mission = deepcopy(updated_mission_list[target_index])
    updated_mission["individualMissionID"] = int(new_individual_id)
    updated_mission["pathID"] = int(new_path_id)
    updated_mission["isDone"] = False
    updated_mission_list[target_index] = updated_mission

    updated_fp = deepcopy(fp_data)
    updated_fp["pathID"] = int(new_path_id)
    updated_fp["timestamp"] = int(now_ms)
    updated_fp["aircraftID"] = int(aircraft_id)
    updated_fp["individualMissionID"] = int(new_individual_id)
    _set_source_field(updated_fp, "MMR")
    updated_fp["waypointList"] = resume_waypoints
    if "lahWaypointList" in updated_fp:
        updated_fp["lahWaypointList"] = deepcopy(resume_waypoints)
    _apply_runtime_flyover_to_flight_path_payload(updated_fp)
    sanitize_flight_path_payload_filming_altitudes(updated_fp)

    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    fp_dest = db_paths.get_db_subpath("FlightPath", f"{new_path_id}.json")
    for path in (imp_dest, fp_dest):
        path.parent.mkdir(parents=True, exist_ok=True)
    validate_generated_artifact_payloads(
        individual_mission_plans=[updated_imp],
        flight_paths=[updated_fp],
        scope=f"pathDeviation:otherUavResume:{new_imp_id}",
        allow_existing_db_artifacts=True,
        log=emit,
    )
    write_json(imp_dest, updated_imp, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    write_json(fp_dest, updated_fp, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    emit(
        f"{log_prefix} stored trimmed current path -> "
        f"IMP:{imp_dest.name}, FP:{fp_dest.name}, currentWP={artifacts.current_waypoint_id or '-'}"
    )
    return {
        "aircraftID": int(aircraft_id),
        "sourceIndividualMissionPackageID": int(artifacts.individual_mission_package_id),
        "sourceIndividualMissionID": int(artifacts.individual_mission_id),
        "sourcePathID": int(artifacts.path_id),
        "resolvedCurrentWaypointID": int(artifacts.current_waypoint_id)
        if artifacts.current_waypoint_id is not None
        else None,
        "generatedIndividualMissionPackageID": int(new_imp_id),
        "generatedIndividualMissionID": int(new_individual_id),
        "generatedPathID": int(new_path_id),
        "removedWaypointID": int(removed_waypoint_id) if removed_waypoint_id is not None else None,
    }


def _build_replanned_waypoints(
    waypoint_list: List[Dict[str, Any]],
    *,
    current_waypoint_id: int,
    alternate_waypoint_id: Optional[int],
    alternate_coordinate: Dict[str, Any],
    fallback_aircraft_altitude: Optional[int],
    emit: Callable[[str], None],
    waypoint_allocator: Callable[[], int] | None = None,
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
        candidate_waypoint_id = (
            int(waypoint_allocator()) if waypoint_allocator is not None else _reserve_waypoint_block(1)
        )
    inserted_waypoint["waypointID"] = int(candidate_waypoint_id)

    coordinate = dict(inserted_waypoint.get("coordinate") or {})
    coordinate["latitude"] = float(alternate_coordinate["latitude"])
    coordinate["longitude"] = float(alternate_coordinate["longitude"])
    current_altitude = _normalize_altitude_value(coordinate.get("altitude"))
    alternate_altitude = _normalize_altitude_value(alternate_coordinate.get("altitude"))
    aircraft_altitude = _normalize_altitude_value(fallback_aircraft_altitude)
    fallback_altitude = aircraft_altitude
    if fallback_altitude is None:
        fallback_altitude = current_altitude
    if fallback_altitude is None:
        fallback_altitude = _first_valid_waypoint_altitude([current_waypoint, *remaining_waypoints])
    if alternate_altitude is not None:
        coordinate["altitude"] = int(alternate_altitude)
    elif aircraft_altitude is not None:
        coordinate["altitude"] = int(aircraft_altitude)
        emit(
            "[PATHDEV] alternate waypoint altitude missing -> "
            f"using current UAV altitude {int(aircraft_altitude)}m."
        )
    elif fallback_altitude is not None:
        coordinate["altitude"] = int(fallback_altitude)
    else:
        emit("[PATHDEV] alternate waypoint altitude unavailable; aborting to avoid altitude=0 waypoint.")
        raise RuntimeError("alternate waypoint altitude unavailable")
    inserted_waypoint["coordinate"] = coordinate
    inserted_waypoint["isDone"] = False

    new_waypoints = [inserted_waypoint] + remaining_waypoints
    search_speed_weight = get_runtime_float("search_speed_weight", 1.1)
    recomputed = recompute_line_search_speed_from_geometry(
        new_waypoints,
        first_reference_coord=coordinate,
        speed_scale=float(search_speed_weight),
        only_increase=True,
    )
    if recomputed > 0:
        emit(
            "[PATHDEV] current path searchSpeed geometry recomputed "
            f"(weight={float(search_speed_weight):.2f}, waypoints={recomputed})."
        )
    for waypoint in new_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    _assign_waypoint_ids_inplace(new_waypoints, waypoint_allocator)
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
    transaction_id = new_replan_transaction_id("path-deviation")
    phase_timer = PipelinePhaseTimer(
        pipeline="path_deviation",
        replan_transaction_id=transaction_id,
        emit_events=True,
    )

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
    ctx_source_override = _ctx_source_plan_override(ctx)
    if ctx_source_override is not None and ctx_source_override > 0:
        previous_source = _to_int(detail.get("sourceMissionPlanID"))
        previous_current = _to_int(detail.get("currentMissionPlanID"))
        if previous_source != int(ctx_source_override) or previous_current != int(ctx_source_override):
            emit(
                "[PATHDEV] source plan corrected from request context "
                f"(detail current/source={previous_current}/{previous_source}, "
                f"ctx={int(ctx_source_override)})."
            )
        detail = dict(detail)
        detail["currentMissionPlanID"] = int(ctx_source_override)
        detail["sourceMissionPlanID"] = int(ctx_source_override)
    phase_timer.mark("detail_store_load")

    current_plan_id = _to_int(detail.get("currentMissionPlanID"))
    source_plan_id = current_plan_id
    if source_plan_id is None or source_plan_id <= 0:
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

    active_tracking_aircraft_ids = _active_attack_tracking_aircraft_ids_for_plan(int(source_plan_id))
    if active_tracking_aircraft_ids:
        emit(
            "[PATHDEV] skipped: active attack tracking is protected from generic path-deviation replan "
            f"(sourcePlan={int(source_plan_id)}, trackingAircraft={sorted(active_tracking_aircraft_ids)}, "
            f"requestedAircraft={int(aircraft_id)})."
        )
        return None

    tracking_assignment = get_tracking_assignment(int(aircraft_id))
    tracking_attack_plan_id = (
        _to_int(tracking_assignment.get("attack_plan_id"))
        if isinstance(tracking_assignment, dict) and bool(tracking_assignment.get("active"))
        else None
    )
    if tracking_attack_plan_id is not None and int(tracking_attack_plan_id) == int(source_plan_id):
        emit(
            "[PATHDEV] skipped: active attack-tracking UAV is protected from generic path-deviation replan "
            f"(aircraft={int(aircraft_id)}, attackPlan={int(source_plan_id)}, "
            f"trackingPathID={_to_int(tracking_assignment.get('tracking_path_id'))}, "
            f"resumePathID={_to_int(tracking_assignment.get('resume_path_id'))})."
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
        emit(
            "[PATHDEV] strict source artifact lookup failed "
            f"(sourcePlan={source_plan_id}, aircraft={aircraft_id}, currentWP={current_waypoint_id}); "
            "retrying with current-mission fallback."
        )
        artifacts = _resolve_plan_artifacts(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
            current_waypoint_id=int(current_waypoint_id),
            emit=emit,
            allow_first_mission_fallback=True,
        )
    if artifacts is None:
        artifacts = _resolve_plan_artifacts_from_explicit_detail(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(aircraft_id),
            current_waypoint_id=int(current_waypoint_id),
            detail=detail,
            emit=emit,
        )
    if artifacts is None:
        emit("[PATHDEV] failed to resolve source mission artifacts.")
        return None
    if _to_int(artifacts.current_waypoint_id) is not None and int(artifacts.current_waypoint_id) != int(current_waypoint_id):
        emit(
            "[PATHDEV] current waypoint remapped by artifact fallback "
            f"({current_waypoint_id} -> {int(artifacts.current_waypoint_id)})."
        )
        current_waypoint_id = int(artifacts.current_waypoint_id)
    source_plan_id = int(artifacts.source_plan_id)

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
    phase_timer.mark("source_plan_imp_fp_resolve_load")

    new_plan_id = int(plan_ids[0])
    source_waypoints = list(fp_data.get("waypointList") or fp_data.get("lahWaypointList") or [])
    option_names = _ensure_option_names([new_plan_id], ctx.get("option_names"))
    now_ms = _now_ms_since_2000()

    target_index = None
    source_mission_list = imp_data.get("individualMissionList") or []
    for idx, mission in enumerate(source_mission_list):
        if _to_int((mission or {}).get("individualMissionID")) == int(artifacts.individual_mission_id):
            target_index = idx
            break
    if target_index is None:
        emit(
            f"[PATHDEV] individualMissionID {artifacts.individual_mission_id} "
            f"not found in package {artifacts.individual_mission_package_id}."
        )
        return None

    source_mission = deepcopy(source_mission_list[target_index])

    if not source_waypoints:
        emit(f"[PATHDEV] FlightPath {artifacts.path_id} has no waypointList.")
        return None

    if _mission_has_imaging_geometry(source_mission, source_waypoints) or _current_waypoint_has_filming_property(
        source_waypoints,
        current_waypoint_id,
    ):
        emit(
            "[PATHDEV] skipped: filming/sweep mission is protected from generic path-deviation replan "
            f"(aircraft={int(aircraft_id)}, sourcePlan={int(source_plan_id)}, "
            f"mission={int(artifacts.individual_mission_id)}, path={int(artifacts.path_id)}, "
            f"currentWP={int(current_waypoint_id)})."
        )
        return None

    sweep_progress = load_sweep_progress()
    snapshot_payload = agent_status_snapshot.load_agent_status_snapshot()
    snapshot_aircraft_states = _extract_snapshot_aircraft_states(snapshot_payload)
    trigger_aircraft_state = snapshot_aircraft_states.get(int(aircraft_id)) or {}
    fallback_aircraft_altitude = _normalize_altitude_value(
        (trigger_aircraft_state.get("coordinate") or {}).get("altitude")
    )

    current_input_id = _mission_related_input_id(source_mission, detail)
    collaborative_resume = None
    if current_input_id is not None and current_input_id > 0 and _mission_has_imaging_geometry(source_mission, source_waypoints):
        entry_coord = _normalize_coordinate(alternate_coordinate)
        if entry_coord is None:
            emit("[PATHDEV][CURRENT] entry coordinate unavailable; falling back to alternate-waypoint trim.")
            collaborative_resume = None
        else:
            entry_coord_override = {int(aircraft_id): _pathdev_coordinate_for_output(entry_coord)}
            heading_override: Dict[int, float] = {}
            heading = _to_float(trigger_aircraft_state.get("headingDeg") or trigger_aircraft_state.get("heading"))
            if heading is not None:
                heading_override[int(aircraft_id)] = float(heading) % 360.0
            agent_state_map = {
                int(aid): dict(state or {})
                for aid, state in snapshot_aircraft_states.items()
                if _to_int(aid) is not None and int(_to_int(aid) or 0) > 3
            }
            agent_state_map[int(aircraft_id)] = dict(trigger_aircraft_state)
            try:
                collaborative_resume = _prepare_uav_collaborative_resume_replan(
                    source_plan_id=int(source_plan_id),
                    current_input_id=int(current_input_id),
                    unavailable_aircraft_ids=set(),
                    agent_state_map=agent_state_map,
                    now_ms=int(now_ms),
                    emit=emit,
                    log_prefix="[PATHDEV][CURRENT]",
                    drop_prefix_missions=False,
                    entry_coord_map_override=entry_coord_override,
                    heading_map_override=heading_override,
                    entry_aircraft_context_map_override=_pathdev_collab_entry_context_override(
                        aircraft_id=int(aircraft_id),
                        alternate_coordinate=entry_coord,
                        state=trigger_aircraft_state,
                    ),
                    audit_context="path_deviation_current_mission_replan",
                )
                phase_timer.mark("current_mission_collab_prepare")
            except Exception as exc:
                emit(f"[PATHDEV][CURRENT] collaborative current mission replan failed: {exc}")
                collaborative_resume = None
        if collaborative_resume is not None and int(aircraft_id) not in collaborative_resume.aircraft_imp_ids:
            emit(
                "[PATHDEV][CURRENT] collaborative result did not include deviating UAV; "
                "falling back to alternate-waypoint trim."
            )
            collaborative_resume = None
    elif current_input_id is None or current_input_id <= 0:
        emit("[PATHDEV][CURRENT] current inputMissionID unavailable; falling back to alternate-waypoint trim.")
    else:
        emit("[PATHDEV][CURRENT] source mission has no imaging geometry; falling back to alternate-waypoint trim.")

    if collaborative_resume is not None:
        new_plan_data = deepcopy(plan_data)
        new_plan_data["missionPlanID"] = int(new_plan_id)
        new_plan_data["timestamp"] = int(now_ms)
        if "missionPlanTimestamp" in new_plan_data:
            new_plan_data["missionPlanTimestamp"] = int(now_ms)
        updated_aircraft_ids: Set[int] = set()
        for aircraft_entry in new_plan_data.get("aircraftList") or []:
            aid = _to_int((aircraft_entry or {}).get("aircraftID"))
            if aid is None:
                continue
            new_aircraft_imp_id = collaborative_resume.aircraft_imp_ids.get(int(aid))
            if new_aircraft_imp_id is None:
                continue
            aircraft_entry["individualMissionPackageID"] = int(new_aircraft_imp_id)
            updated_aircraft_ids.add(int(aid))
        if int(aircraft_id) not in updated_aircraft_ids:
            emit("[PATHDEV][CURRENT] deviating UAV was not linked into new MissionPlan; falling back to trim.")
            collaborative_resume = None
        else:
            current_refs = _generated_current_mission_refs(
                collaborative_resume.aircraft_imp_ids,
                current_input_id=int(current_input_id),
            )
            target_refs = current_refs.get(int(aircraft_id)) or []
            first_target_ref = target_refs[0] if target_refs else {}
            new_imp_id = int(collaborative_resume.aircraft_imp_ids[int(aircraft_id)])
            new_individual_id = int(first_target_ref.get("individualMissionID") or 0)
            new_path_id = int(first_target_ref.get("pathID") or 0)
            inserted_waypoint_id = _first_waypoint_id_for_path(new_path_id) if new_path_id > 0 else None
            removed_waypoint_id = int(current_waypoint_id)
            if new_individual_id <= 0 or new_path_id <= 0 or inserted_waypoint_id is None:
                emit("[PATHDEV][CURRENT] generated target mission refs unavailable; falling back to trim.")
                collaborative_resume = None
            else:
                preserved_manned_imp_ids, preserved_manned_path_ids, preserved_manned_aircraft_ids = (
                    _collect_preserved_manned_artifact_ids(new_plan_data, emit=emit)
                )
                if preserved_manned_imp_ids or preserved_manned_path_ids:
                    emit(
                        "[PATHDEV][LAH] preserved existing manned references "
                        f"(aircraft={preserved_manned_aircraft_ids}, "
                        f"IMP={sorted(preserved_manned_imp_ids)}, FP={sorted(preserved_manned_path_ids)})."
                    )
                phase_timer.mark("preserved_lah_artifact_handling")
                validation_summary = validate_replan_payloads(
                    mission_plan=new_plan_data,
                    individual_mission_plans=[],
                    flight_paths=[],
                    scope=f"pathDeviation:currentMission:{new_plan_id}",
                    allow_existing_db_artifacts=True,
                    log=emit,
                )
                phase_timer.mark("current_mission_collab_validation")
                plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
                started_at = time.perf_counter()
                write_results = write_json_batch(
                    ((plan_dest, new_plan_data),),
                    pretty=True,
                    ensure_ascii=False,
                    skip_if_unchanged=True,
                    log=emit,
                )
                carried_snapshot = mission_area_replan_store.carry_forward_snapshot(
                    int(source_plan_id),
                    int(new_plan_id),
                    reason="path_deviation_current_mission_replan",
                )
                if carried_snapshot is not None:
                    emit(
                        "[PATHDEV] carried area remaining snapshot -> "
                        f"{carried_snapshot.name} (sourcePlan={source_plan_id}, plan={new_plan_id})"
                    )
                elapsed_ms = (time.perf_counter() - started_at) * 1000.0
                phase_timer.mark("write_artifacts")
                write_count = sum(1 for row in write_results if row.get("written"))
                emit(
                    "[PATHDEV][CURRENT] stored current-mission replan artifacts -> "
                    f"plan:{plan_dest.name}, aircraft={sorted(updated_aircraft_ids)}, "
                    f"paths={sorted(collaborative_resume.generated_path_ids)} "
                    f"(written={write_count}/{len(write_results)}, {elapsed_ms:.1f} ms)"
                )
                preserved_tracking_aircraft_ids = _tracking_aircraft_ids_preserved_for_rebind(
                    source_plan_id=int(source_plan_id),
                    plan_data=new_plan_data,
                    emit=emit,
                )
                rebound_tracking_aircraft_ids = (
                    rebind_tracking_assignments_to_plan(
                        old_attack_plan_id=int(source_plan_id),
                        new_attack_plan_id=int(new_plan_id),
                        aircraft_ids=sorted(preserved_tracking_aircraft_ids),
                    )
                    if preserved_tracking_aircraft_ids
                    else []
                )
                if rebound_tracking_aircraft_ids:
                    emit(
                        "[PATHDEV] active attack tracking assignment rebound -> "
                        f"{int(source_plan_id)} -> {int(new_plan_id)} "
                        f"aircraft={sorted(rebound_tracking_aircraft_ids)}"
                    )
                for fov_adjust_message in pop_runtime_camera_fov_adjustment_logs():
                    emit(str(fov_adjust_message))

                other_updates = [
                    {
                        "aircraftID": int(aid),
                        "generatedIndividualMissionPackageID": int(imp_id),
                        "generatedPathIDs": [int(row["pathID"]) for row in current_refs.get(int(aid), [])],
                        "currentMissionReplanned": True,
                    }
                    for aid, imp_id in sorted(collaborative_resume.aircraft_imp_ids.items())
                    if int(aid) != int(aircraft_id)
                ]
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
                    "currentInputMissionID": int(current_input_id),
                    "currentMissionReplanned": True,
                    "plannerWorkflow": str(collaborative_resume.planner_workflow or ""),
                    "plannerResultText": str(collaborative_resume.planner_result_text or ""),
                    "generatedMissionPlanID": int(new_plan_id),
                    "generatedIndividualMissionPackageID": int(new_imp_id),
                    "generatedIndividualMissionID": int(new_individual_id),
                    "generatedPathID": int(new_path_id),
                    "generatedIndividualMissionPackageIDs": [
                        int(imp_id) for imp_id in sorted(collaborative_resume.aircraft_imp_ids.values())
                    ],
                    "generatedPathIDs": sorted(int(path_id) for path_id in collaborative_resume.generated_path_ids),
                    "preservedMannedAircraftIDs": sorted(preserved_manned_aircraft_ids),
                    "preservedMannedIndividualMissionPackageIDs": sorted(preserved_manned_imp_ids),
                    "preservedMannedPathIDs": sorted(preserved_manned_path_ids),
                    "reboundTrackingAircraftIDs": sorted(int(aid) for aid in rebound_tracking_aircraft_ids),
                    "otherAircraftUpdates": other_updates,
                    "logMessages": log_messages,
                    "detail": dict(detail),
                    "logArtifactMode": debug_artifact_mode(),
                    "replanTransactionId": transaction_id,
                    "reservedIds": dict(getattr(collaborative_resume, "id_reservation", {}) or {}),
                    "writeResults": write_results,
                    "validation": validation_summary,
                    "timingMs": phase_timer.snapshot(include_total=True),
                }
                log_written = write_debug_json(
                    log_path,
                    log_payload,
                    pretty=True,
                    ensure_ascii=False,
                    skip_if_unchanged=False,
                )
                log_payload["logArtifactWritten"] = bool(log_written)
                phase_timer.mark("log_artifact")
                if log_written:
                    emit(f"[PATHDEV] log captured -> {log_path}")
                else:
                    emit("[PATHDEV] log artifact skipped by runtime artifact mode.")
                try:
                    path_deviation_replan_store.save_event(
                        "mission_pipeline_complete",
                        {
                            "generatedMissionPlanID": int(new_plan_id),
                            "sourceMissionPlanID": int(source_plan_id),
                            "aircraftID": int(aircraft_id),
                            "currentInputMissionID": int(current_input_id),
                            "currentMissionReplanned": True,
                            "plannerWorkflow": str(collaborative_resume.planner_workflow or ""),
                            "generatedIndividualMissionPackageIDs": [
                                int(imp_id) for imp_id in sorted(collaborative_resume.aircraft_imp_ids.values())
                            ],
                            "generatedPathIDs": sorted(int(path_id) for path_id in collaborative_resume.generated_path_ids),
                            "preservedMannedAircraftIDs": sorted(preserved_manned_aircraft_ids),
                            "preservedMannedIndividualMissionPackageIDs": sorted(preserved_manned_imp_ids),
                            "preservedMannedPathIDs": sorted(preserved_manned_path_ids),
                            "reboundTrackingAircraftIDs": sorted(int(aid) for aid in rebound_tracking_aircraft_ids),
                            "otherAircraftUpdates": other_updates,
                            "removedWaypointID": int(removed_waypoint_id),
                            "insertedWaypointID": int(inserted_waypoint_id),
                            "logPath": str(log_path),
                            "logArtifactMode": debug_artifact_mode(),
                            "logArtifactWritten": bool(log_written),
                            "timingMs": dict(log_payload.get("timingMs") or {}),
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
                        "currentInputMissionID": int(current_input_id),
                        "currentMissionReplanned": True,
                        "plannerWorkflow": str(collaborative_resume.planner_workflow or ""),
                        "individualMissionPackageID": int(new_imp_id),
                        "individualMissionID": int(new_individual_id),
                        "pathID": int(new_path_id),
                        "removedWaypointID": int(removed_waypoint_id),
                        "insertedWaypointID": int(inserted_waypoint_id),
                        "logPath": str(log_path),
                        "logArtifactMode": debug_artifact_mode(),
                        "logArtifactWritten": bool(log_written),
                        "alternateWaypointCoordinate": dict(alternate_coordinate),
                        "reboundTrackingAircraftIDs": sorted(int(aid) for aid in rebound_tracking_aircraft_ids),
                        "otherAircraftUpdates": other_updates,
                        "preservedMannedAircraftIDs": sorted(preserved_manned_aircraft_ids),
                        "preservedMannedIndividualMissionPackageIDs": sorted(preserved_manned_imp_ids),
                        "preservedMannedPathIDs": sorted(preserved_manned_path_ids),
                        "timingMs": dict(log_payload.get("timingMs") or {}),
                        "replanTransactionId": transaction_id,
                    }
                )
                return PathDeviationPipelineResult(
                    plan_ids=[int(new_plan_id)],
                    option_names=list(option_names),
                    plan_meta_map=plan_meta_map,
                    generated_imp_ids=set(int(imp_id) for imp_id in collaborative_resume.aircraft_imp_ids.values()),
                    generated_path_ids=set(int(path_id) for path_id in collaborative_resume.generated_path_ids),
                    preserved_manned_imp_ids=set(preserved_manned_imp_ids),
                    preserved_manned_path_ids=set(preserved_manned_path_ids),
                    new_imp_id=int(new_imp_id),
                    new_path_id=int(new_path_id),
                    new_individual_id=int(new_individual_id),
                    removed_waypoint_id=int(removed_waypoint_id),
                    inserted_waypoint_id=int(inserted_waypoint_id),
                    log_path=log_path,
                    other_updates=other_updates,
                )

    reservation = ReplanIdReservation.reserve(
        imp_count=1,
        individual_count=1,
        path_count_by_aircraft={int(aircraft_id): 1},
        waypoint_count=max(1, len(source_waypoints) + 1),
    )
    new_imp_id = reservation.next_imp()
    new_individual_id = reservation.next_individual()
    new_path_id = reservation.next_path(int(aircraft_id))
    phase_timer.mark("id_allocation")

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

    mission_list = new_imp_data.get("individualMissionList") or []
    target_index = None
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

    completed_by_on_mission2 = (
        _to_int(trigger_aircraft_state.get("onMission")) == 2
        and _mission_has_imaging_geometry(source_mission, source_waypoints)
    )
    if completed_by_on_mission2:
        hold_coord = _last_imaging_coordinate(
            source_waypoints,
            fallback_coordinate=trigger_aircraft_state.get("coordinate"),
        )
        if hold_coord is None:
            emit("[PATHDEV] onMission=2 completion guard could not resolve hold coordinate.")
            return None
        completed_mission = deepcopy(source_mission)
        completed_mission["isDone"] = True
        mission_list[target_index] = completed_mission
        hold_mission = _build_completion_hold_mission(
            template_mission=source_mission,
            individual_mission_id=int(new_individual_id),
            path_id=int(new_path_id),
            coordinate=hold_coord,
        )
        mission_list.insert(target_index + 1, hold_mission)
        template_waypoint = source_waypoints[-1] if source_waypoints else None
        new_fp_data, inserted_waypoint_id = _build_completion_hold_flight_path(
            template_path=fp_data,
            aircraft_id=int(aircraft_id),
            individual_mission_id=int(new_individual_id),
            path_id=int(new_path_id),
            coordinate=hold_coord,
            template_waypoint=template_waypoint,
            now_ms=int(now_ms),
        )
        removed_waypoint_id = _to_int((source_waypoints[-1] or {}).get("waypointID")) if source_waypoints else int(current_waypoint_id)
        _apply_runtime_flyover_to_flight_path_payload(new_fp_data)
        sanitize_flight_path_payload_filming_altitudes(new_fp_data)
        emit(
            "[PATHDEV] onMission=2 imaging mission completed; inserted "
            f"{_PATHDEV_COMPLETE_HOLD_SECONDS}s hold instead of replanning sweep "
            f"(aircraft={aircraft_id}, holdWP={inserted_waypoint_id})."
        )
    else:
        source_mission["individualMissionID"] = int(new_individual_id)
        source_mission["pathID"] = int(new_path_id)
        source_mission["isDone"] = False
        mission_list[target_index] = source_mission

        try:
            new_waypoints, removed_waypoint_id, inserted_waypoint_id = _build_replanned_waypoints(
                [deepcopy(item) for item in source_waypoints],
                current_waypoint_id=int(current_waypoint_id),
                alternate_waypoint_id=alternate_waypoint_id,
                alternate_coordinate=alternate_coordinate,
                fallback_aircraft_altitude=fallback_aircraft_altitude,
                emit=emit,
                waypoint_allocator=reservation.next_waypoint,
            )
        except Exception:
            return None
        phase_timer.mark("currentWP_trim_alternate_insert")

        new_fp_data = deepcopy(fp_data)
        new_fp_data["pathID"] = int(new_path_id)
        new_fp_data["timestamp"] = int(now_ms)
        new_fp_data["aircraftID"] = int(aircraft_id)
        new_fp_data["individualMissionID"] = int(new_individual_id)
        _set_source_field(new_fp_data, "MMR")
        new_fp_data["waypointList"] = new_waypoints
        if "lahWaypointList" in new_fp_data:
            new_fp_data["lahWaypointList"] = deepcopy(new_waypoints)
        _apply_runtime_flyover_to_flight_path_payload(new_fp_data)
        desired_inserted_altitude = _normalize_altitude_value(alternate_coordinate.get("altitude"))
        if desired_inserted_altitude is None:
            desired_inserted_altitude = fallback_aircraft_altitude
        _restore_inserted_waypoint_altitude_after_runtime_profile(
            new_fp_data.get("waypointList") or [],
            inserted_waypoint_id=int(inserted_waypoint_id),
            desired_altitude=desired_inserted_altitude,
            emit=emit,
        )
        sanitize_flight_path_payload_filming_altitudes(new_fp_data)
        _enforce_inserted_waypoint_filming_altitude_floor(
            new_fp_data.get("waypointList") or [],
            inserted_waypoint_id=int(inserted_waypoint_id),
            emit=emit,
        )
        if "lahWaypointList" in new_fp_data:
            new_fp_data["lahWaypointList"] = deepcopy(new_fp_data.get("waypointList") or [])
        sanitize_flight_path_payload_filming_altitudes(new_fp_data)

    other_updates: List[Dict[str, Any]] = []
    other_generated_imp_ids: Set[int] = set()
    other_generated_path_ids: Set[int] = set()
    for aircraft_entry in new_plan_data.get("aircraftList") or []:
        other_aircraft_id = _to_int((aircraft_entry or {}).get("aircraftID"))
        if other_aircraft_id not in (4, 5, 6) or int(other_aircraft_id) == int(aircraft_id):
            continue
        other_tracking_assignment = get_tracking_assignment(int(other_aircraft_id))
        other_tracking_attack_plan_id = (
            _to_int(other_tracking_assignment.get("attack_plan_id"))
            if isinstance(other_tracking_assignment, dict) and bool(other_tracking_assignment.get("active"))
            else None
        )
        if other_tracking_attack_plan_id is not None and int(other_tracking_attack_plan_id) == int(source_plan_id):
            emit(
                "[PATHDEV][OTHER] skipped active attack-tracking UAV "
                f"(aircraft={int(other_aircraft_id)}, attackPlan={int(source_plan_id)})."
            )
            continue
        other_update = _build_other_uav_current_package(
            source_plan_id=int(source_plan_id),
            aircraft_id=int(other_aircraft_id),
            current_waypoint_id=_to_int(
                (snapshot_aircraft_states.get(int(other_aircraft_id)) or {}).get("currentWaypointID")
            ),
            raw_current_waypoint_id=_to_int(
                (snapshot_aircraft_states.get(int(other_aircraft_id)) or {}).get("rawCurrentWaypointID")
            ),
            flight_mode=_to_int((snapshot_aircraft_states.get(int(other_aircraft_id)) or {}).get("flightMode")),
            on_mission=_to_int((snapshot_aircraft_states.get(int(other_aircraft_id)) or {}).get("onMission")),
            aircraft_coordinate=(snapshot_aircraft_states.get(int(other_aircraft_id)) or {}).get("coordinate"),
            loiter_coordinate=(snapshot_aircraft_states.get(int(other_aircraft_id)) or {}).get("loiterCoordinate"),
            sweep_progress=sweep_progress,
            now_ms=int(now_ms),
            emit=emit,
        )
        if not isinstance(other_update, dict):
            continue
        generated_imp_id = _to_int(other_update.get("generatedIndividualMissionPackageID"))
        generated_path_id = _to_int(other_update.get("generatedPathID"))
        if generated_imp_id is not None and generated_imp_id > 0:
            aircraft_entry["individualMissionPackageID"] = int(generated_imp_id)
            other_generated_imp_ids.add(int(generated_imp_id))
        if generated_path_id is not None and generated_path_id > 0:
            other_generated_path_ids.add(int(generated_path_id))
        other_updates.append(other_update)
    phase_timer.mark("other_uav_resume_update")

    preserved_manned_imp_ids, preserved_manned_path_ids, preserved_manned_aircraft_ids = (
        _collect_preserved_manned_artifact_ids(new_plan_data, emit=emit)
    )
    if preserved_manned_imp_ids or preserved_manned_path_ids:
        emit(
            "[PATHDEV][LAH] preserved existing manned references "
            f"(aircraft={preserved_manned_aircraft_ids}, "
            f"IMP={sorted(preserved_manned_imp_ids)}, FP={sorted(preserved_manned_path_ids)})."
        )
    phase_timer.mark("preserved_lah_artifact_handling")

    validation_summary = validate_replan_payloads(
        mission_plan=new_plan_data,
        individual_mission_plans=[new_imp_data],
        flight_paths=[new_fp_data],
        scope=f"pathDeviation:{new_plan_id}",
        allow_existing_db_artifacts=True,
        log=emit,
    )
    phase_timer.mark("validation")

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{new_plan_id}.json")
    imp_dest = db_paths.get_db_subpath("IndividualMissionPlan", f"{new_imp_id}.json")
    fp_dest = db_paths.get_db_subpath("FlightPath", f"{new_path_id}.json")
    started_at = time.perf_counter()
    write_results = write_json_batch(
        (
            (plan_dest, new_plan_data),
            (imp_dest, new_imp_data),
            (fp_dest, new_fp_data),
        ),
        pretty=True,
        ensure_ascii=False,
        skip_if_unchanged=True,
        log=emit,
    )
    carried_snapshot = mission_area_replan_store.carry_forward_snapshot(
        int(source_plan_id),
        int(new_plan_id),
        reason="path_deviation",
    )
    if carried_snapshot is not None:
        emit(
            "[PATHDEV] carried area remaining snapshot -> "
            f"{carried_snapshot.name} (sourcePlan={source_plan_id}, plan={new_plan_id})"
        )
    elapsed_ms = (time.perf_counter() - started_at) * 1000.0
    phase_timer.mark("write_artifacts")
    write_count = sum(1 for row in write_results if row.get("written"))
    emit(
        "[PATHDEV] stored path-deviation artifacts -> "
        f"plan:{plan_dest.name}, imp:{imp_dest.name}, fp:{fp_dest.name} "
        f"(written={write_count}/{len(write_results)}, {elapsed_ms:.1f} ms)"
    )
    preserved_tracking_aircraft_ids = _tracking_aircraft_ids_preserved_for_rebind(
        source_plan_id=int(source_plan_id),
        plan_data=new_plan_data,
        emit=emit,
    )
    rebound_tracking_aircraft_ids = (
        rebind_tracking_assignments_to_plan(
            old_attack_plan_id=int(source_plan_id),
            new_attack_plan_id=int(new_plan_id),
            aircraft_ids=sorted(preserved_tracking_aircraft_ids),
        )
        if preserved_tracking_aircraft_ids
        else []
    )
    if rebound_tracking_aircraft_ids:
        emit(
            "[PATHDEV] active attack tracking assignment rebound -> "
            f"{int(source_plan_id)} -> {int(new_plan_id)} "
            f"aircraft={sorted(rebound_tracking_aircraft_ids)}"
        )
    for fov_adjust_message in pop_runtime_camera_fov_adjustment_logs():
        emit(str(fov_adjust_message))

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
        "completedByOnMission2": bool(completed_by_on_mission2),
        "generatedMissionPlanID": int(new_plan_id),
        "generatedIndividualMissionPackageID": int(new_imp_id),
        "generatedIndividualMissionID": int(new_individual_id),
        "generatedPathID": int(new_path_id),
        "generatedIndividualMissionPackageIDs": [int(new_imp_id), *sorted(other_generated_imp_ids)],
        "generatedPathIDs": [int(new_path_id), *sorted(other_generated_path_ids)],
        "preservedMannedAircraftIDs": sorted(preserved_manned_aircraft_ids),
        "preservedMannedIndividualMissionPackageIDs": sorted(preserved_manned_imp_ids),
        "preservedMannedPathIDs": sorted(preserved_manned_path_ids),
        "reboundTrackingAircraftIDs": sorted(int(aid) for aid in rebound_tracking_aircraft_ids),
        "otherAircraftUpdates": other_updates,
        "logMessages": log_messages,
        "detail": dict(detail),
        "logArtifactMode": debug_artifact_mode(),
        "replanTransactionId": transaction_id,
        "reservedIds": reservation.summary(),
        "writeResults": write_results,
        "validation": validation_summary,
        "timingMs": phase_timer.snapshot(include_total=True),
    }
    log_written = write_debug_json(log_path, log_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
    log_payload["logArtifactWritten"] = bool(log_written)
    phase_timer.mark("log_artifact")
    if log_written:
        emit(f"[PATHDEV] log captured -> {log_path}")
    else:
        emit("[PATHDEV] log artifact skipped by runtime artifact mode.")
    try:
        path_deviation_replan_store.save_event(
            "mission_pipeline_complete",
            {
                "generatedMissionPlanID": int(new_plan_id),
                "sourceMissionPlanID": int(source_plan_id),
                "aircraftID": int(aircraft_id),
                "generatedIndividualMissionPackageIDs": [int(new_imp_id), *sorted(other_generated_imp_ids)],
                "generatedPathIDs": [int(new_path_id), *sorted(other_generated_path_ids)],
                "preservedMannedAircraftIDs": sorted(preserved_manned_aircraft_ids),
                "preservedMannedIndividualMissionPackageIDs": sorted(preserved_manned_imp_ids),
                "preservedMannedPathIDs": sorted(preserved_manned_path_ids),
                "reboundTrackingAircraftIDs": sorted(int(aid) for aid in rebound_tracking_aircraft_ids),
                "otherAircraftUpdates": other_updates,
                "removedWaypointID": int(removed_waypoint_id),
                "insertedWaypointID": int(inserted_waypoint_id),
                "logPath": str(log_path),
                "logArtifactMode": debug_artifact_mode(),
                "logArtifactWritten": bool(log_written),
                "timingMs": dict(log_payload.get("timingMs") or {}),
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
            "logArtifactMode": debug_artifact_mode(),
            "logArtifactWritten": bool(log_written),
            "alternateWaypointCoordinate": dict(alternate_coordinate),
            "completedByOnMission2": bool(completed_by_on_mission2),
            "reboundTrackingAircraftIDs": sorted(int(aid) for aid in rebound_tracking_aircraft_ids),
            "otherAircraftUpdates": other_updates,
            "preservedMannedAircraftIDs": sorted(preserved_manned_aircraft_ids),
            "preservedMannedIndividualMissionPackageIDs": sorted(preserved_manned_imp_ids),
            "preservedMannedPathIDs": sorted(preserved_manned_path_ids),
            "timingMs": dict(log_payload.get("timingMs") or {}),
            "replanTransactionId": transaction_id,
        }
    )

    return PathDeviationPipelineResult(
        plan_ids=[int(new_plan_id)],
        option_names=list(option_names),
        plan_meta_map=plan_meta_map,
        generated_imp_ids={int(new_imp_id)}.union(other_generated_imp_ids),
        generated_path_ids={int(new_path_id)}.union(other_generated_path_ids),
        preserved_manned_imp_ids=set(preserved_manned_imp_ids),
        preserved_manned_path_ids=set(preserved_manned_path_ids),
        new_imp_id=int(new_imp_id),
        new_path_id=int(new_path_id),
        new_individual_id=int(new_individual_id),
        removed_waypoint_id=int(removed_waypoint_id),
        inserted_waypoint_id=int(inserted_waypoint_id),
        log_path=log_path,
        other_updates=other_updates,
    )
