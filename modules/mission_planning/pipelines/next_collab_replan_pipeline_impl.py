from __future__ import annotations

import json
import sys
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Set

from modules.common import db_paths, next_collab_replan_store
from modules.mission_planning.runtime.json_io import write_json
from modules.mission_planning._paths import mission_planner_root, mission_planning_root, project_root
from modules.mission_planning.pipelines.mission_path_trim import reassign_unique_waypoint_ids_inplace
from modules.mission_planning.pipelines.prior_mission_pipeline_impl import (
    _next_waypoint_id,
    _now_ms_since_2000,
    _normalize_altitude_value,
    _reserve_imp_ids,
    _reserve_individual_mission_ids,
    _reserve_path_ids,
    _to_float,
    _to_int,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.algo import split_algorithms as split_algorithms_module
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.area_review import review_assigned_areas_local
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
    _assign_group_by_takeover_distance,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0302 import (
    _piece_runtime_meta,
    _piece_to_mission_info,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0303_0304 import (
    _apply_runtime_params,
    _import_runtime_modules,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.models import (
    DirectionDebug,
    SplitPiece,
    SplitRunResult,
)
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        get_runtime_bool,
        get_runtime_area_review_max_segment_m,
        load_runtime_settings,
    )
except Exception:
    from MissionPlanner.runtime_settings import (  # type: ignore
        get_runtime_bool,
        get_runtime_area_review_max_segment_m,
        load_runtime_settings,
    )


DEFAULT_OPTION_NAME = "비행/촬영"
TRIGGER_TYPE = "nextCollaborativeMission"
REPLAN_FLOW_MODE = "next_collab_local_assigned"
ENTRY_FOV_DEG = 10.0
DEFAULT_AREA_REVIEW_MAX_SEGMENT_M = 3000.0


@dataclass
class NextCollabPipelineResult:
    plan_ids: List[int]
    option_names: List[str]
    plan_meta_map: Dict[int, Dict[str, Any]]
    generated_imp_ids: Set[int]
    generated_path_ids: Set[int]
    new_input_package_id: int
    log_path: Path


def warm_next_collab_replan_pipeline() -> Dict[str, Any]:
    try:
        _ensure_runtime_import_paths()
        d0303, _, search_speed, mp_config = _import_runtime_modules()
        cruise_speed, turn_step = _apply_runtime_params(d0303, search_speed, mp_config)
    except Exception as exc:
        return {"ready": False, "error": str(exc)}
    return {
        "ready": True,
        "cruiseSpeedMps": float(cruise_speed),
        "turnStepDeg": float(turn_step),
    }


def _ensure_runtime_import_paths() -> None:
    candidate_paths = (
        mission_planner_root(),
        mission_planning_root(),
        project_root() / "modules",
        project_root(),
    )
    for path in candidate_paths:
        path_str = str(path)
        if path.exists() and path_str not in sys.path:
            sys.path.insert(0, path_str)


def _ensure_option_names(plan_ids: List[int], option_names: List[str] | None) -> List[str]:
    names = [str(name) for name in (option_names or []) if name is not None]
    if not names:
        names = [DEFAULT_OPTION_NAME]
    while len(names) < len(plan_ids):
        names.append(names[-1])
    return names[: len(plan_ids)]


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
        payload = next_collab_replan_store.load_detail(plan_id)
        if payload:
            return payload
    return None


def _normalize_coordinate(payload: object | None) -> dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    alt = _normalize_altitude_value(payload.get("altitude"))
    if lat is None or lon is None:
        return None
    coord: dict[str, float] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    if alt is not None:
        coord["altitude"] = int(alt)
    return coord


def _centroid_coordinate(coords: List[dict[str, float]]) -> dict[str, float] | None:
    if not coords:
        return None
    lat_vals = [float(item["latitude"]) for item in coords if "latitude" in item]
    lon_vals = [float(item["longitude"]) for item in coords if "longitude" in item]
    if not lat_vals or not lon_vals:
        return None
    out: dict[str, float] = {
        "latitude": sum(lat_vals) / float(len(lat_vals)),
        "longitude": sum(lon_vals) / float(len(lon_vals)),
    }
    alt_vals = [float(item["altitude"]) for item in coords if "altitude" in item]
    if alt_vals:
        out["altitude"] = sum(alt_vals) / float(len(alt_vals))
    return out


def _extract_entry_coordinate_map(detail: Dict[str, Any]) -> Dict[int, dict[str, float]]:
    out: Dict[int, dict[str, float]] = {}
    for item in detail.get("entryAircraftList") or []:
        if not isinstance(item, dict):
            continue
        aid = _to_int(item.get("aircraftID"))
        coord = _normalize_coordinate(item.get("coordinate"))
        if aid is None or aid <= 0 or coord is None:
            continue
        out[int(aid)] = coord
    return out


def _build_takeover_info_list(entry_coord_map: Dict[int, dict[str, float]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for aircraft_id, coord in sorted(entry_coord_map.items()):
        rows.append(
            {
                "aircraftID": int(aircraft_id),
                "coordinate": {
                    "latitude": float(coord["latitude"]),
                    "longitude": float(coord["longitude"]),
                    "altitude": int(round(float(coord.get("altitude", 0.0) or 0.0))),
                },
            }
        )
    return rows


def _normalize_coord_list(payload: object | None) -> List[Dict[str, Any]]:
    coords = payload if isinstance(payload, list) else []
    out: List[Dict[str, Any]] = []
    for item in coords:
        coord = _normalize_coordinate(item)
        if coord is None:
            continue
        out.append(coord)
    return out


def _mission_entry_point(mission: Dict[str, Any]) -> Optional[Dict[str, float]]:
    if not isinstance(mission, dict):
        return None
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []

    if line_list:
        coords = _normalize_coord_list(line_list[0].get("coordinateList"))
        if coords:
            return dict(coords[0])
    if coord_list:
        coords = _normalize_coord_list(coord_list)
        if coords:
            return dict(coords[0])
    if area_list:
        all_centers: List[dict[str, float]] = []
        for area in area_list:
            if not isinstance(area, dict):
                continue
            coords = _normalize_coord_list(area.get("coordinateList"))
            if not coords:
                continue
            center = _centroid_coordinate(coords)
            if center is not None:
                all_centers.append(center)
        if all_centers:
            return _centroid_coordinate(all_centers)
    return None


def _find_input_mission(input_plan: Dict[str, Any], input_mission_id: int) -> Dict[str, Any] | None:
    for item in input_plan.get("inputMissionList") or []:
        if not isinstance(item, dict):
            continue
        if _to_int(item.get("inputMissionID")) == int(input_mission_id):
            return item
    return None


def _find_next_input_entry(input_plan: Dict[str, Any], input_mission_id: int) -> Dict[str, float] | None:
    found = False
    for item in input_plan.get("inputMissionList") or []:
        if not isinstance(item, dict):
            continue
        current_id = _to_int(item.get("inputMissionID"))
        if current_id == int(input_mission_id):
            found = True
            continue
        if not found:
            continue
        entry = _mission_entry_point(item)
        if entry is not None:
            return entry
    return None


def _next_input_package_id() -> int:
    folder = db_paths.get_db_subpath("InputMissionPlan")
    max_id = 0
    if folder.exists():
        for path in folder.glob("*.json"):
            try:
                max_id = max(max_id, int(path.stem))
            except Exception:
                continue
    return int(max_id + 1 if max_id > 0 else 1)


def _piece_entry_point(piece: SplitPiece) -> Optional[Dict[str, float]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    for key in ("Centerline", "coordinateList", "rawCoordinateList"):
        coords = _normalize_coord_list(data.get(key))
        if coords:
            return dict(coords[0])
    return None


def _bearing_from_coords(start: Dict[str, Any], end: Dict[str, Any]) -> float | None:
    lat1 = _to_float(start.get("latitude"))
    lon1 = _to_float(start.get("longitude"))
    lat2 = _to_float(end.get("latitude"))
    lon2 = _to_float(end.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    return float(split_algorithms_module._bearing_deg(start, end))


def _build_target_direction_debugs(
    mission: Dict[str, Any],
    *,
    prev_pt: Dict[str, Any] | None,
    next_pt: Dict[str, Any] | None,
) -> List[DirectionDebug]:
    mission_id = _to_int(mission.get("inputMissionID")) or 0
    mission_type = _to_int(mission.get("inputMissionType")) or 0
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    directions: List[DirectionDebug] = []

    if mission_type in (1, 4, 5, 7):
        debug = DirectionDebug(
            parent_order=1,
            mission_id=mission_id,
            mission_type=mission_type,
            source_area_index=None,
            prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
            next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
        )
        line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
        coord_list = detail.get("coordinateList") if isinstance(detail.get("coordinateList"), list) else []
        if line_list:
            coords = _normalize_coord_list(line_list[0].get("coordinateList"))
            if coords:
                debug.line_start = dict(coords[0])
                debug.line_end = dict(coords[-1])
        elif coord_list:
            coords = _normalize_coord_list(coord_list)
            if coords:
                debug.line_start = dict(coords[0])
                debug.line_end = dict(coords[-1])
        directions.append(debug)
        return directions

    if mission_type in (2, 3, 6):
        area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
        for area_idx, area in enumerate(area_list, start=1):
            if not isinstance(area, dict):
                continue
            coords = _normalize_coord_list(area.get("coordinateList"))
            if len(coords) < 3:
                continue
            center, bearing_move, bearing_in, bearing_out = split_algorithms_module._resolve_area_bearing(
                prev_pt,
                next_pt,
                coords,
            )
            debug = DirectionDebug(
                parent_order=1,
                mission_id=mission_id,
                mission_type=mission_type,
                source_area_index=int(area_idx),
                prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
                next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
            )
            if center is not None:
                debug.center_point = {
                    "latitude": float(center["latitude"]),
                    "longitude": float(center["longitude"]),
                    "altitude": float(center.get("altitude", 0.0) or 0.0),
                }
            debug.bearing_in_deg = float(bearing_in) if bearing_in is not None else None
            debug.bearing_out_deg = float(bearing_out) if bearing_out is not None else None
            debug.bearing_move_deg = float(bearing_move)
            debug.bearing_split_deg = float((bearing_move + 90.0) % 360.0)
            directions.append(debug)
        if directions:
            return directions

    return [
        DirectionDebug(
            parent_order=1,
            mission_id=mission_id,
            mission_type=mission_type,
            source_area_index=None,
            prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
            next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
        )
    ]


def _apply_piece_template_metadata(
    piece: SplitPiece,
    *,
    template_map: Dict[int, List[Dict[str, Any]]],
) -> None:
    aircraft_id = _to_int(piece.assigned_uav)
    if aircraft_id is None or aircraft_id <= 0:
        return
    templates = template_map.get(int(aircraft_id)) or []
    template = templates[0] if templates else None
    template_info = _template_mission_info(template)
    if not template_info:
        return
    if "individualMissionType" in template_info:
        piece.data["individualMissionType"] = template_info.get("individualMissionType")
    if "patternType" in template_info:
        piece.data["patternType"] = template_info.get("patternType")


def _run_area_review_for_target(
    *,
    pieces: List[SplitPiece],
    target_aircraft_ids: List[int],
    target_input_mission: Dict[str, Any],
    representative_entry: Dict[str, Any],
    next_entry: Dict[str, Any] | None,
    mrpk_data: Dict[str, Any],
    emit: Callable[[str], None],
) -> tuple[List[SplitPiece], Dict[str, Any]]:
    runtime_cfg = load_runtime_settings()
    review_enabled = bool(get_runtime_bool("enhanced_area_review_enabled", True, runtime_cfg))
    review_max_segment_m = float(
        get_runtime_area_review_max_segment_m(
            DEFAULT_AREA_REVIEW_MAX_SEGMENT_M,
            runtime_cfg,
        )
    )
    base_report: Dict[str, Any] = {
        "enabled": review_enabled,
        "mode": REPLAN_FLOW_MODE,
        "maxSegmentM": float(review_max_segment_m),
        "changed": False,
        "overflowRows": 0,
        "targets": 0,
        "localized": 0,
        "oldPieceCount": len(pieces),
        "newPieceCount": len(pieces),
        "details": [],
    }
    if not review_enabled:
        emit("[NEXTCOLLAB] area-review skipped by config.")
        return list(pieces), base_report

    split_result = SplitRunResult(
        uav_count=len(target_aircraft_ids),
        uav_ids=[int(aid) for aid in target_aircraft_ids],
        pieces=list(pieces),
    )
    review_report = dict(base_report)
    review_report.update(
        review_assigned_areas_local(
            split_result,
            mrpk_data if isinstance(mrpk_data, dict) else {},
            max_segment_m=review_max_segment_m,
        )
    )
    emit(
        "[NEXTCOLLAB] area-review(replan-local) done: "
        f"targets={int(review_report.get('targets', 0))} "
        f"localized={int(review_report.get('localized', 0))} "
        f"pieces={int(review_report.get('oldPieceCount', len(pieces)))}->"
        f"{int(review_report.get('newPieceCount', len(split_result.pieces)))} "
        f"maxSegmentM={float(review_max_segment_m):.1f}"
    )
    return list(split_result.pieces), review_report


def _apply_piece_entry_metadata(
    piece: SplitPiece,
    *,
    entry_coord: dict[str, float],
    next_coord: dict[str, float] | None,
) -> None:
    data = piece.data if isinstance(piece.data, dict) else {}
    data["prevPoint"] = {
        "latitude": float(entry_coord["latitude"]),
        "longitude": float(entry_coord["longitude"]),
        "altitude": int(round(float(entry_coord.get("altitude", 0.0) or 0.0))),
    }
    if next_coord is not None:
        data["nextPoint"] = {
            "latitude": float(next_coord["latitude"]),
            "longitude": float(next_coord["longitude"]),
            "altitude": int(round(float(next_coord.get("altitude", 0.0) or 0.0))),
        }

    coords = _normalize_coord_list(data.get("coordinateList"))
    if int(piece.mission_type) in (2, 3, 6) and len(coords) >= 3:
        center, bearing_move, bearing_in, bearing_out = split_algorithms_module._resolve_area_bearing(
            dict(data["prevPoint"]),
            dict(data["nextPoint"]) if "nextPoint" in data else None,
            coords,
        )
        bearing_entry = float(bearing_in) if bearing_in is not None else float(bearing_move)
        bearing_split = (bearing_entry + 90.0) % 360.0
        data["bearing_deg"] = float(bearing_entry)
        data["splitBearing_deg"] = float(bearing_entry)
        data["phaseMoveBearing_deg"] = float(bearing_entry)
        data["phaseSplitBearing_deg"] = float(bearing_entry)
        data["boundaryAxisBearing_deg"] = float(bearing_split)
        if bearing_in is not None:
            data["bearingIn_deg"] = float(bearing_in)
        if bearing_out is not None:
            data["bearingOut_deg"] = float(bearing_out)
        data.setdefault("sourceAreaIndex", 1)
        if center is not None and "center" not in data:
            data["center"] = center
    else:
        centerline = _normalize_coord_list(data.get("Centerline"))
        if not centerline:
            centerline = coords
        if centerline:
            bearing = _bearing_from_coords(data["prevPoint"], centerline[0])
            if bearing is not None:
                data["bearingFromPrev"] = float(bearing)


def _replace_geometry_from_piece(
    template_info: Dict[str, Any],
    generated_info: Dict[str, Any],
) -> Dict[str, Any]:
    info = deepcopy(template_info or {})
    if not info:
        return deepcopy(generated_info)

    for key in ("individualMissionType", "patternType", "autoZoomIn"):
        if key in generated_info:
            info[key] = deepcopy(generated_info[key])

    for key in ("lineList", "areaList", "coordinateList"):
        if key in generated_info:
            info[key] = deepcopy(generated_info[key])
        else:
            info.pop(key, None)

    for key in ("BEARING", "MOVE_BEARING"):
        if key in generated_info:
            info[key] = generated_info[key]

    for key in ("FOV", "SEP", "SPEED"):
        if key not in info and key in generated_info:
            info[key] = generated_info[key]

    if "targetID" in generated_info and "targetID" not in info:
        info["targetID"] = generated_info["targetID"]
    return info


def _template_mission_info(mission: Dict[str, Any] | None) -> Dict[str, Any]:
    if not isinstance(mission, dict):
        return {}
    info = mission.get("individualMissionInfo")
    return dict(info) if isinstance(info, dict) else {}


def _mission_input_id(mission: Dict[str, Any]) -> int | None:
    related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    return _to_int(related.get("inputMissionID"))


def _extract_target_templates(
    packages_by_aircraft: Dict[int, Dict[str, Any]],
    target_input_id: int,
) -> Dict[int, List[Dict[str, Any]]]:
    out: Dict[int, List[Dict[str, Any]]] = {}
    for aircraft_id, pkg in packages_by_aircraft.items():
        missions = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        for mission in missions:
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) != int(target_input_id):
                continue
            out.setdefault(int(aircraft_id), []).append(mission)
    return out


def _focus_coordinate_from_waypoint(waypoint: Dict[str, Any]) -> dict[str, float] | None:
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    coords = _normalize_coord_list(line_search.get("coordinateList"))
    if coords:
        return dict(coords[0])
    coord_orientation = filming.get("coordinateOrientation") if isinstance(filming.get("coordinateOrientation"), dict) else {}
    coord = _normalize_coordinate(coord_orientation.get("coordinate"))
    if coord is not None:
        return coord
    return _normalize_coordinate(waypoint.get("coordinate"))


def _build_entry_waypoint(
    *,
    entry_coord: dict[str, float],
    focus_coord: dict[str, float] | None,
    template_wp: dict[str, Any] | None,
) -> Dict[str, Any]:
    base_coord = _normalize_coordinate((template_wp or {}).get("coordinate")) or {}
    altitude = _normalize_altitude_value(entry_coord.get("altitude"))
    if altitude is None:
        altitude = _normalize_altitude_value(base_coord.get("altitude"))
    if altitude is None:
        altitude = 0
    speed = _to_float((template_wp or {}).get("speed"))
    if speed is None or speed <= 0.0:
        speed = 40.0
    filming = (template_wp or {}).get("filmingProperty") if isinstance((template_wp or {}).get("filmingProperty"), dict) else {}
    sensor_type = _to_int(filming.get("sensorType")) or 1
    fov = _to_float(filming.get("fieldOfView"))
    if fov is None or fov <= 0.0:
        fov = ENTRY_FOV_DEG
    entry_wp: Dict[str, Any] = {
        "waypointID": int(_next_waypoint_id()),
        "coordinate": {
            "latitude": float(entry_coord["latitude"]),
            "longitude": float(entry_coord["longitude"]),
            "altitude": int(altitude),
        },
        "speed": float(speed),
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": 1,
        "filmingProperty": {
            "fieldOfView": float(fov),
            "sensorType": int(sensor_type),
            "operationMode": 1,
        },
    }
    if focus_coord is not None:
        entry_wp["filmingProperty"]["coordinateOrientation"] = {
            "coordinate": {
                "latitude": float(focus_coord["latitude"]),
                "longitude": float(focus_coord["longitude"]),
                "altitude": int(round(float(focus_coord.get("altitude", 0.0) or 0.0))),
            }
        }
    return entry_wp


def _midpoint_of_sweep_coords(coords: List[Dict[str, Any]]) -> Dict[str, float] | None:
    if len(coords) < 2:
        return None
    first = coords[0]
    last = coords[-1]
    lat1 = _to_float(first.get("latitude"))
    lon1 = _to_float(first.get("longitude"))
    lat2 = _to_float(last.get("latitude"))
    lon2 = _to_float(last.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    alt1 = _to_float(first.get("altitude"))
    alt2 = _to_float(last.get("altitude"))
    out: Dict[str, float] = {
        "latitude": (float(lat1) + float(lat2)) / 2.0,
        "longitude": (float(lon1) + float(lon2)) / 2.0,
    }
    if alt1 is not None and alt2 is not None:
        out["altitude"] = (float(alt1) + float(alt2)) / 2.0
    elif alt1 is not None:
        out["altitude"] = float(alt1)
    elif alt2 is not None:
        out["altitude"] = float(alt2)
    return out


def _align_waypoint_to_line_search_center(waypoint: Dict[str, Any]) -> None:
    if not isinstance(waypoint, dict):
        return
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    coords = _normalize_coord_list(line_search.get("coordinateList"))
    if not coords:
        return
    center = _midpoint_of_sweep_coords(coords)
    if center is None:
        return
    base_coord = _normalize_coordinate(waypoint.get("coordinate")) or {}
    altitude = _normalize_altitude_value(base_coord.get("altitude"))
    if altitude is None:
        altitude = _normalize_altitude_value(center.get("altitude"))
    if altitude is None:
        altitude = 0
    waypoint["coordinate"] = {
        "latitude": float(center["latitude"]),
        "longitude": float(center["longitude"]),
        "altitude": int(altitude),
    }

def _prepend_entry_waypoint(
    flight_plan: Dict[str, Any],
    *,
    entry_coord: dict[str, float],
) -> None:
    waypoints = flight_plan.get("waypointList")
    if not isinstance(waypoints, list) or not waypoints:
        return

    first_wp = waypoints[0] if isinstance(waypoints[0], dict) else {}
    first_filming = first_wp.get("filmingProperty") if isinstance(first_wp.get("filmingProperty"), dict) else {}
    first_line_search = first_filming.get("lineSearch") if isinstance(first_filming.get("lineSearch"), dict) else {}
    if len(waypoints) >= 2 and not first_line_search and _to_int(first_filming.get("operationMode")) == 4:
        second_wp = waypoints[1] if isinstance(waypoints[1], dict) else {}
        second_filming = second_wp.get("filmingProperty") if isinstance(second_wp.get("filmingProperty"), dict) else {}
        second_line_search = second_filming.get("lineSearch") if isinstance(second_filming.get("lineSearch"), dict) else {}
        if second_line_search or _to_int(second_filming.get("operationMode")) == 2:
            waypoints.pop(0)

    if not waypoints:
        return
    _align_waypoint_to_line_search_center(waypoints[0] if isinstance(waypoints[0], dict) else {})
    focus_coord = _focus_coordinate_from_waypoint(waypoints[0])
    entry_wp = _build_entry_waypoint(
        entry_coord=entry_coord,
        focus_coord=focus_coord,
        template_wp=waypoints[0] if isinstance(waypoints[0], dict) else None,
    )
    waypoints.insert(0, entry_wp)
    for wp in waypoints:
        if isinstance(wp, dict):
            wp["isDone"] = False
    reassign_unique_waypoint_ids_inplace(waypoints)
    flight_plan["waypointList"] = waypoints
    if "lahWaypointList" in flight_plan:
        flight_plan["lahWaypointList"] = deepcopy(waypoints)


def run_next_collab_replan_pipeline(
    ctx: Dict[str, Any],
    detail: Dict[str, Any],
    reason: str,
    *,
    log: Callable[[str], None],
) -> Optional[NextCollabPipelineResult]:
    log_messages: List[str] = []

    def emit(message: str) -> None:
        log_messages.append(message)
        log(message)

    plan_ids_raw = list(ctx.get("plan_ids") or [])
    try:
        plan_ids = [int(value) for value in plan_ids_raw if value is not None]
    except Exception:
        emit(f"[NEXTCOLLAB] invalid plan_ids in context: {plan_ids_raw!r}")
        return None
    if len(plan_ids) != 1:
        emit(f"[NEXTCOLLAB] expected exactly one pending missionPlanID, got {len(plan_ids)}.")
        return None

    stored_detail = _load_detail_from_store(plan_ids)
    if not isinstance(detail, dict) or not detail:
        detail = stored_detail or {}
    elif stored_detail:
        merged_detail = dict(stored_detail)
        merged_detail.update(detail)
        detail = merged_detail
    if not isinstance(detail, dict) or not detail:
        emit("[NEXTCOLLAB] replanDetail missing and store lookup failed.")
        return None

    source_plan_id = _to_int(detail.get("sourceMissionPlanID"))
    current_input_id = _to_int(detail.get("currentInputMissionID"))
    target_input_id = _to_int(detail.get("targetInputMissionID"))
    if source_plan_id is None or source_plan_id <= 0:
        emit("[NEXTCOLLAB] sourceMissionPlanID missing.")
        return None
    if current_input_id is None or current_input_id <= 0:
        emit("[NEXTCOLLAB] currentInputMissionID missing.")
        return None
    if target_input_id is None or target_input_id <= 0:
        emit("[NEXTCOLLAB] targetInputMissionID missing.")
        return None

    entry_coord_map = _extract_entry_coordinate_map(detail)
    if not entry_coord_map:
        emit("[NEXTCOLLAB] entryAircraftList missing/empty.")
        return None

    new_plan_id = int(plan_ids[0])
    option_names = _ensure_option_names([new_plan_id], ctx.get("option_names"))
    now_ms = _now_ms_since_2000()

    try:
        plan_src = db_paths.get_db_subpath("MissionPlan", f"{int(source_plan_id)}.json")
        plan_data = json.loads(plan_src.read_text(encoding="utf-8"))
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to load source MissionPlan {source_plan_id}: {exc}")
        return None

    source_input_pkg_id = _to_int(
        plan_data.get("inputMissionPackageID")
        or plan_data.get("InputMissionPackageID")
        or plan_data.get("inputMissionPackageId")
    )
    if source_input_pkg_id is None or source_input_pkg_id <= 0:
        emit("[NEXTCOLLAB] source MissionPlan missing inputMissionPackageID.")
        return None
    try:
        input_src = db_paths.get_db_subpath("InputMissionPlan", f"{int(source_input_pkg_id)}.json")
        input_data = json.loads(input_src.read_text(encoding="utf-8"))
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to load InputMissionPlan {source_input_pkg_id}: {exc}")
        return None

    mrpk_id = _to_int(
        plan_data.get("missionReferencePackageID")
        or plan_data.get("MissionReferencePackageID")
        or plan_data.get("missionReferencePackageId")
    )
    mrpk_data: Dict[str, Any] = {}
    candidate_mrpk_ids: List[int] = []
    if mrpk_id is not None:
        candidate_mrpk_ids.append(int(mrpk_id))
    if 0 not in candidate_mrpk_ids:
        candidate_mrpk_ids.append(0)
    for candidate_id in candidate_mrpk_ids:
        try:
            mrpk_path = db_paths.get_db_subpath("MissionReferenceInfo", f"{int(candidate_id)}.json")
            if not mrpk_path.exists():
                continue
            mrpk_data = json.loads(mrpk_path.read_text(encoding="utf-8"))
            break
        except Exception:
            mrpk_data = {}

    target_input_mission = _find_input_mission(input_data, int(target_input_id))
    if not isinstance(target_input_mission, dict):
        emit(f"[NEXTCOLLAB] target input mission {target_input_id} not found in InputMissionPlan.")
        return None

    new_plan_data = deepcopy(plan_data)
    new_plan_data["missionPlanID"] = int(new_plan_id)
    new_plan_data["timestamp"] = int(now_ms)
    if "missionPlanTimestamp" in new_plan_data:
        new_plan_data["missionPlanTimestamp"] = int(now_ms)
    new_input_pkg_id = _next_input_package_id()
    new_plan_data["inputMissionPackageID"] = int(new_input_pkg_id)
    _set_source_field(new_plan_data, "MMR")

    new_input_data = deepcopy(input_data)
    new_input_data["inputMissionPackageID"] = int(new_input_pkg_id)
    new_input_data["timestamp"] = int(now_ms)
    _set_source_field(new_input_data, "MMR")
    for item in new_input_data.get("inputMissionList") or []:
        if not isinstance(item, dict):
            continue
        input_id = _to_int(item.get("inputMissionID"))
        if input_id == int(current_input_id):
            item["isDone"] = True
        elif input_id == int(target_input_id):
            item["isDone"] = False

    aircraft_entries = [entry for entry in new_plan_data.get("aircraftList") or [] if isinstance(entry, dict)]
    if not aircraft_entries:
        emit("[NEXTCOLLAB] source MissionPlan has no aircraftList.")
        return None

    new_imp_ids = _reserve_imp_ids(len(aircraft_entries))
    if len(new_imp_ids) != len(aircraft_entries):
        emit("[NEXTCOLLAB] failed to reserve IMP package IDs.")
        return None

    packages_by_aircraft: Dict[int, Dict[str, Any]] = {}
    generated_imp_ids: Set[int] = set()
    for idx, aircraft_entry in enumerate(aircraft_entries):
        aircraft_id = _to_int(aircraft_entry.get("aircraftID"))
        source_imp_id = _to_int(
            aircraft_entry.get("individualMissionPackageID")
            or aircraft_entry.get("individualMissionPlanPackageID")
            or aircraft_entry.get("individualMissionPackageId")
        )
        if aircraft_id is None or aircraft_id <= 0 or source_imp_id is None or source_imp_id <= 0:
            emit("[NEXTCOLLAB] MissionPlan aircraft entry missing aircraftID/individualMissionPackageID.")
            return None
        try:
            imp_src = db_paths.get_db_subpath("IndividualMissionPlan", f"{int(source_imp_id)}.json")
            imp_data = json.loads(imp_src.read_text(encoding="utf-8"))
        except Exception as exc:
            emit(f"[NEXTCOLLAB] failed to load IndividualMissionPlan {source_imp_id}: {exc}")
            return None
        new_imp_id = int(new_imp_ids[idx])
        new_imp_data = deepcopy(imp_data)
        new_imp_data["individualMissionPackageID"] = int(new_imp_id)
        new_imp_data["timestamp"] = int(now_ms)
        _set_source_field(new_imp_data, "MMR")
        aircraft_entry["individualMissionPackageID"] = int(new_imp_id)
        packages_by_aircraft[int(aircraft_id)] = new_imp_data
        generated_imp_ids.add(int(new_imp_id))

    for pkg in packages_by_aircraft.values():
        mission_list = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        for mission in mission_list:
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) == int(current_input_id):
                mission["isDone"] = True

    template_map = _extract_target_templates(packages_by_aircraft, int(target_input_id))
    if not template_map:
        emit(f"[NEXTCOLLAB] no target individual missions found for inputMissionID={target_input_id}.")
        return None

    takeover_list = mrpk_data.get("takeOverInfoList") if isinstance(mrpk_data.get("takeOverInfoList"), list) else []
    if not takeover_list:
        mrpk_data = deepcopy(mrpk_data) if isinstance(mrpk_data, dict) else {}
        mrpk_data["takeOverInfoList"] = _build_takeover_info_list(entry_coord_map)

    target_aircraft_ids = [aid for aid in entry_coord_map if aid in template_map]
    if not target_aircraft_ids:
        target_aircraft_ids = sorted(template_map.keys())
    target_aircraft_ids = [int(aid) for aid in target_aircraft_ids if int(aid) in template_map]
    if not target_aircraft_ids:
        emit("[NEXTCOLLAB] no target aircraft IDs resolved.")
        return None

    representative_entry = _normalize_coordinate(detail.get("representativeEntryCoordinate"))
    if representative_entry is None:
        representative_entry = _centroid_coordinate(list(entry_coord_map.values()))
    if representative_entry is None:
        emit("[NEXTCOLLAB] representative entry coordinate unavailable.")
        return None

    next_entry = _find_next_input_entry(input_data, int(target_input_id))
    subs = split_algorithms_module.split_mission_into_subareas(
        deepcopy(target_input_mission),
        len(target_aircraft_ids),
        deepcopy(representative_entry),
        deepcopy(next_entry),
    )
    if not subs:
        emit(f"[NEXTCOLLAB] split returned empty pieces for inputMissionID={target_input_id}.")
        return None

    pieces: List[SplitPiece] = []
    target_mission_type = _to_int(target_input_mission.get("inputMissionType")) or 0
    for idx, piece_payload in enumerate(subs, start=1):
        pieces.append(
            SplitPiece(
                parent_order=1,
                mission_id=int(target_input_id),
                mission_type=int(target_mission_type),
                piece_index=int(idx),
                data=dict(piece_payload or {}),
            )
        )

    assigned = _assign_group_by_takeover_distance(pieces, list(target_aircraft_ids), entry_coord_map)
    for idx, piece in enumerate(pieces):
        assigned_aid = assigned.get(int(idx))
        if assigned_aid is None and idx < len(target_aircraft_ids):
            assigned_aid = int(target_aircraft_ids[idx])
        if assigned_aid is None:
            assigned_aid = int(target_aircraft_ids[-1])
        piece.assigned_uav = int(assigned_aid)
        _apply_piece_template_metadata(piece, template_map=template_map)

    area_review_report: Dict[str, Any] = {
        "enabled": False,
        "maxSegmentM": float(DEFAULT_AREA_REVIEW_MAX_SEGMENT_M),
        "changed": False,
        "overflowRows": 0,
        "targets": 0,
        "oldPieceCount": len(pieces),
        "newPieceCount": len(pieces),
        "details": [],
    }
    try:
        pieces, area_review_report = _run_area_review_for_target(
            pieces=pieces,
            target_aircraft_ids=list(target_aircraft_ids),
            target_input_mission=target_input_mission,
            representative_entry=representative_entry,
            next_entry=next_entry,
            mrpk_data=mrpk_data,
            emit=emit,
        )
    except Exception as exc:
        area_review_report = {
            **area_review_report,
            "error": str(exc),
        }
        emit(f"[NEXTCOLLAB] area-review failed, using raw split pieces: {exc}")

    generated_path_ids: Set[int] = set()
    missions_for_0303: List[Dict[str, Any]] = []
    replacement_by_aircraft: Dict[int, List[Dict[str, Any]]] = {}

    individual_ids = _reserve_individual_mission_ids(len(pieces))
    if len(individual_ids) != len(pieces):
        emit("[NEXTCOLLAB] failed to reserve individualMissionIDs.")
        return None

    for idx, piece in enumerate(pieces):
        aircraft_id = int(piece.assigned_uav or 0)
        if aircraft_id <= 0:
            continue
        templates = template_map.get(int(aircraft_id)) or []
        template = deepcopy(templates[0]) if templates else {}
        template_info = _template_mission_info(template)
        _apply_piece_template_metadata(piece, template_map=template_map)
        entry_coord = entry_coord_map.get(int(aircraft_id)) or representative_entry
        _apply_piece_entry_metadata(
            piece,
            entry_coord=dict(entry_coord),
            next_coord=dict(next_entry) if isinstance(next_entry, dict) else None,
        )

        generated_info = _piece_to_mission_info(piece)
        mission_info = _replace_geometry_from_piece(template_info, generated_info)
        related_mission = deepcopy(template.get("relatedMission") or {})
        related_mission["relatedMissionType"] = _to_int(related_mission.get("relatedMissionType")) or 1
        related_mission["inputMissionID"] = int(target_input_id)
        related_mission["priorMissionID"] = _to_int(related_mission.get("priorMissionID")) or 0
        [new_path_id] = _reserve_path_ids(int(aircraft_id), 1)
        generated_path_ids.add(int(new_path_id))
        new_mission_entry: Dict[str, Any] = {
            "individualMissionID": int(individual_ids[idx]),
            "isDone": False,
            "relatedMission": related_mission,
            "individualMissionInfo": mission_info,
            "pathID": int(new_path_id),
        }
        runtime_meta = _piece_runtime_meta(piece)
        if runtime_meta:
            new_mission_entry.update(runtime_meta)
        replacement_by_aircraft.setdefault(int(aircraft_id), []).append(new_mission_entry)
        mission_for_0303 = deepcopy(new_mission_entry)
        mission_for_0303["aircraftID"] = int(aircraft_id)
        missions_for_0303.append(mission_for_0303)
    for aircraft_id, pkg in packages_by_aircraft.items():
        mission_list = pkg.get("individualMissionList") if isinstance(pkg.get("individualMissionList"), list) else []
        replacements = replacement_by_aircraft.get(int(aircraft_id)) or []
        if int(aircraft_id) in (1, 2, 3) and not replacements:
            pkg["individualMissionList"] = list(mission_list)
            continue

        filtered_missions: List[Dict[str, Any]] = []
        insert_index = None
        for mission in mission_list:
            if not isinstance(mission, dict):
                continue
            if _mission_input_id(mission) == int(target_input_id):
                if insert_index is None:
                    insert_index = len(filtered_missions)
                continue
            filtered_missions.append(mission)
        if insert_index is None:
            insert_index = len(filtered_missions)
        pkg["individualMissionList"] = (
            filtered_missions[:insert_index] + replacements + filtered_missions[insert_index:]
        )

    if not missions_for_0303:
        emit("[NEXTCOLLAB] no replacement missions prepared for 0303 generation.")
        return None

    try:
        _ensure_runtime_import_paths()
        d0303, _, search_speed, mp_config = _import_runtime_modules()
        cruise_speed_mps, turn_step_deg = _apply_runtime_params(d0303, search_speed, mp_config)
    except Exception as exc:
        emit(f"[NEXTCOLLAB] failed to load 0303 runtime: {exc}")
        return None

    try:
        wp_alloc = d0303._WPAllocator()
    except Exception:
        wp_alloc = None

    try:
        flight_plans_0303 = d0303.build_flight_plans(
            missions=missions_for_0303,
            wp_alloc=wp_alloc,
            cruise_speed=float(cruise_speed_mps),
            turn_step_deg=float(turn_step_deg),
            ref0203=mrpk_data if isinstance(mrpk_data, dict) else None,
        )
    except Exception as exc:
        emit(f"[NEXTCOLLAB] 0303 build_flight_plans failed: {exc}")
        return None

    generated_fp_by_path: Dict[int, Dict[str, Any]] = {}
    for fp in flight_plans_0303 or []:
        if not isinstance(fp, dict):
            continue
        path_id = _to_int(fp.get("pathID"))
        if path_id is None or path_id <= 0:
            continue
        new_fp_data = deepcopy(fp)
        new_fp_data["timestamp"] = int(now_ms)
        _set_source_field(new_fp_data, "MMR")
        generated_fp_by_path[int(path_id)] = new_fp_data

    missing_path_ids = sorted(int(pid) for pid in generated_path_ids if int(pid) not in generated_fp_by_path)
    if missing_path_ids:
        emit(f"[NEXTCOLLAB] 0303 output missing generated pathIDs: {missing_path_ids}")
        return None

    plan_dest = db_paths.get_db_subpath("MissionPlan", f"{int(new_plan_id)}.json")
    input_dest = db_paths.get_db_subpath("InputMissionPlan", f"{int(new_input_pkg_id)}.json")
    imp_rows: List[tuple[Path, Dict[str, Any]]] = []
    for aircraft_id, pkg in packages_by_aircraft.items():
        imp_id = _to_int(pkg.get("individualMissionPackageID"))
        if imp_id is None or imp_id <= 0:
            emit(f"[NEXTCOLLAB] cloned IMP missing individualMissionPackageID for aircraft {aircraft_id}.")
            return None
        imp_rows.append(
            (
                db_paths.get_db_subpath("IndividualMissionPlan", f"{int(imp_id)}.json"),
                pkg,
            )
        )

    fp_rows: List[tuple[Path, Dict[str, Any]]] = []
    for path_id in sorted(generated_path_ids):
        fp_rows.append(
            (
                db_paths.get_db_subpath("FlightPath", f"{int(path_id)}.json"),
                generated_fp_by_path[int(path_id)],
            )
        )

    for path in [plan_dest, input_dest] + [row[0] for row in imp_rows] + [row[0] for row in fp_rows]:
        path.parent.mkdir(parents=True, exist_ok=True)

    started_at = time.perf_counter()
    write_json(plan_dest, new_plan_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    write_json(input_dest, new_input_data, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    for path, payload in imp_rows:
        write_json(path, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    for path, payload in fp_rows:
        write_json(path, payload, pretty=True, ensure_ascii=False, skip_if_unchanged=True)
    elapsed_ms = (time.perf_counter() - started_at) * 1000.0
    emit(
        "[NEXTCOLLAB] stored replanned artifacts -> "
        f"plan:{plan_dest.name}, input:{input_dest.name}, imp:{len(imp_rows)}, fp:{len(fp_rows)} "
        f"({elapsed_ms:.1f} ms)"
    )

    log_dir = db_paths.get_db_subpath("DSS_Internal")
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"NextCollab_{int(target_input_id)}_{int(now_ms)}.json"
    log_payload = {
        "timestamp": int(now_ms),
        "reason": str(reason or ""),
        "sourceMissionPlanID": int(source_plan_id),
        "sourceInputMissionPackageID": int(source_input_pkg_id),
        "generatedMissionPlanID": int(new_plan_id),
        "generatedInputMissionPackageID": int(new_input_pkg_id),
        "currentInputMissionID": int(current_input_id),
        "targetInputMissionID": int(target_input_id),
        "targetAircraftIDs": [int(aid) for aid in target_aircraft_ids],
        "representativeEntryCoordinate": dict(representative_entry),
        "nextInputEntryCoordinate": dict(next_entry) if isinstance(next_entry, dict) else None,
        "entryAircraftList": [
            {
                "aircraftID": int(aid),
                "coordinate": dict(coord),
            }
            for aid, coord in sorted(entry_coord_map.items())
        ],
        "areaReview": dict(area_review_report),
        "generatedIndividualMissionPackageIDs": sorted(int(val) for val in generated_imp_ids),
        "generatedPathIDs": sorted(int(val) for val in generated_path_ids),
        "replanFlowMode": REPLAN_FLOW_MODE,
        "logMessages": list(log_messages),
        "detail": dict(detail),
    }
    write_json(log_path, log_payload, pretty=True, ensure_ascii=False, skip_if_unchanged=False)
    emit(f"[NEXTCOLLAB] log captured -> {log_path}")
    try:
        next_collab_replan_store.save_event(
            "mission_pipeline_complete",
            {
                "generatedMissionPlanID": int(new_plan_id),
                "generatedInputMissionPackageID": int(new_input_pkg_id),
                "sourceMissionPlanID": int(source_plan_id),
                "currentInputMissionID": int(current_input_id),
                "targetInputMissionID": int(target_input_id),
                "targetAircraftIDs": [int(aid) for aid in target_aircraft_ids],
                "generatedPathIDs": sorted(int(val) for val in generated_path_ids),
                "areaReview": dict(area_review_report),
                "replanFlowMode": REPLAN_FLOW_MODE,
                "logPath": str(log_path),
            },
        )
    except Exception:
        pass

    plan_meta_map = dict(ctx.get("_option_meta") or {})
    plan_meta_entry = plan_meta_map.setdefault(int(new_plan_id), {})
    plan_meta_entry.update(
        {
            "triggerType": TRIGGER_TYPE,
            "sourceMissionPlanID": int(source_plan_id),
            "currentInputMissionID": int(current_input_id),
            "targetInputMissionID": int(target_input_id),
            "inputMissionPackageID": int(new_input_pkg_id),
            "generatedIndividualMissionPackageIDs": sorted(int(val) for val in generated_imp_ids),
            "generatedPathIDs": sorted(int(val) for val in generated_path_ids),
            "areaReview": dict(area_review_report),
            "replanFlowMode": REPLAN_FLOW_MODE,
            "suppress0702Fallback": True,
            "logPath": str(log_path),
        }
    )

    return NextCollabPipelineResult(
        plan_ids=[int(new_plan_id)],
        option_names=list(option_names),
        plan_meta_map=plan_meta_map,
        generated_imp_ids=set(int(val) for val in generated_imp_ids),
        generated_path_ids=set(int(val) for val in generated_path_ids),
        new_input_package_id=int(new_input_pkg_id),
        log_path=log_path,
    )
