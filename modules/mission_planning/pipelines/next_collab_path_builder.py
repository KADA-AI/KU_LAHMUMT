from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from modules.mission_planning.pipelines.mission_path_trim import (
    reassign_unique_waypoint_ids_inplace,
)
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    normalize_filming_target_altitudes_in_waypoints,
)
from modules.mission_planning.MissionPlanner.data_def.mission_helpers import terrain_elev
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        apply_runtime_camera_adjusted_fov_deg,
        get_runtime_manual_fov_deg,
        get_runtime_manual_fov_sync_active,
        get_runtime_altitude_layers_m,
        get_runtime_bool,
        get_runtime_float,
        get_runtime_int,
        load_runtime_flyover,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        apply_runtime_camera_adjusted_fov_deg,
        get_runtime_manual_fov_deg,
        get_runtime_manual_fov_sync_active,
        get_runtime_altitude_layers_m,
        get_runtime_bool,
        get_runtime_float,
        get_runtime_int,
        load_runtime_flyover,
    )
from modules.mission_planning.planners.next_collab_division._geo_utils import (
    coord_to_xy,
    local_xy_to_llh,
)

_LEGACY_ALTITUDE_HELPERS: Dict[str, Any] | None = None
_LEGACY_ALTITUDE_HELPERS_LOADED = False
ALTITUDE_LAYERS_M = (1000.0, 1010.0, 1020.0)
UAV_CLIMB_RATE_MPS = 5.0
UAV_MIN_FORWARD_SPEED_MPS = 30.0
OPMODE_POINT = 1
PASS_FLYBY = 1
PASS_LOITER = 2
PASS_FLYOVER = 3
NEXT_COLLAB_SWEEP_POINTS_PER_LEG = 3
NEXT_COLLAB_LINE_ROUTE_WP_SPACING_M = 1200.0
NEXT_COLLAB_FIRST_LINE_FOV_SCALE = 1.35
NEXT_COLLAB_FIRST_LINE_FOV_MAX_DEG = 15.4
NEXT_COLLAB_LINE_INGRESS_ENTRY_TRIGGER_SCALE = 1.25
NEXT_COLLAB_LINE_INGRESS_ENTRY_MIN_TRIGGER_M = 250.0
ENTRY_ALTITUDE_MIN_M = 700.0
ENTRY_ALTITUDE_MAX_PRESERVE_DELTA_M = 600.0
FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M = 30.0


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _to_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _runtime_manual_fov_active() -> bool:
    try:
        return bool(get_runtime_manual_fov_sync_active())
    except Exception:
        return False


def _runtime_manual_fov_value(key: str, default: float) -> float:
    try:
        return float(get_runtime_manual_fov_deg(key, float(default)))
    except Exception:
        return float(default)


def _mission_search_speed_weight(mission_info: Dict[str, Any] | None) -> float:
    info = mission_info if isinstance(mission_info, dict) else {}
    key = "search_speed_weight" if bool(info.get("lineList")) and not bool(info.get("areaList")) else "area_search_speed_weight"
    try:
        value = float(get_runtime_float(key, 1.0))
    except Exception:
        value = 1.0
    return max(value, 0.1)


def _fov_key_for_mission_info(mission_info: Dict[str, Any] | None) -> str:
    info = mission_info if isinstance(mission_info, dict) else {}
    try:
        pattern_type = int(info.get("patternType", 0) or 0)
    except Exception:
        pattern_type = 0
    if bool(info.get("lineList")) and not bool(info.get("areaList")):
        return "line_custom_fov_deg"
    if pattern_type == 3:
        return "area_nadir_fov_deg"
    return "area_custom_fov_deg"


def _altitude_int(value: Any, default: int = 0) -> int:
    altitude = _to_float(value)
    if altitude is None:
        return int(default)
    return int(round(float(altitude)))


def _xy_to_coord(point_xy: Sequence[float], altitude: float = 0.0) -> Dict[str, float]:
    lat, lon = local_xy_to_llh(float(point_xy[0]), float(point_xy[1]))
    return {
        "latitude": float(lat),
        "longitude": float(lon),
        "altitude": _altitude_int(altitude),
    }


def _dem_alt(lat: float, lon: float) -> int:
    return int(round(terrain_elev(float(lat), float(lon))))


def _aircraft_alt_offset_m(aircraft_id: int) -> float:
    layers = get_runtime_altitude_layers_m()
    try:
        idx = (int(aircraft_id) - 1) % len(layers)
    except Exception:
        idx = 0
    return float(layers[idx])


def _collect_ref_points_from_info(info: Dict[str, Any]) -> List[Tuple[float, float]]:
    points: List[Tuple[float, float]] = []
    for coord in info.get("coordinateList") or []:
        lat = _to_float(coord.get("latitude")) if isinstance(coord, dict) else None
        lon = _to_float(coord.get("longitude")) if isinstance(coord, dict) else None
        if lat is not None and lon is not None:
            points.append((float(lat), float(lon)))
    for line in info.get("lineList") or []:
        if not isinstance(line, dict):
            continue
        for coord in line.get("coordinateList") or []:
            lat = _to_float(coord.get("latitude")) if isinstance(coord, dict) else None
            lon = _to_float(coord.get("longitude")) if isinstance(coord, dict) else None
            if lat is not None and lon is not None:
                points.append((float(lat), float(lon)))
    for area in info.get("areaList") or []:
        if not isinstance(area, dict):
            continue
        for coord in area.get("coordinateList") or []:
            lat = _to_float(coord.get("latitude")) if isinstance(coord, dict) else None
            lon = _to_float(coord.get("longitude")) if isinstance(coord, dict) else None
            if lat is not None and lon is not None:
                points.append((float(lat), float(lon)))
    return points


def _resolve_generated_mission_types(
    template_info: Dict[str, Any],
    *,
    geometry_kind: str,
) -> tuple[int, int]:
    current_type = _to_int(template_info.get("individualMissionType"))
    current_pattern = _to_int(template_info.get("patternType"))

    if geometry_kind == "area":
        mission_type = int(current_type) if current_type in (3, 4) else 3
        pattern_type = int(current_pattern) if current_type in (3, 4) and current_pattern is not None else 6
        return mission_type, pattern_type

    if geometry_kind == "line":
        if current_type == 7:
            mission_type = 7
            pattern_type = int(current_pattern) if current_pattern is not None else 9
            return mission_type, pattern_type
        mission_type = 6
        pattern_type = int(current_pattern) if current_type == 6 and current_pattern is not None else 8
        return mission_type, pattern_type

    return int(current_type or 0), int(current_pattern or 0)


def _median_ground_m(points: List[Tuple[float, float]]) -> float | None:
    if not points:
        return None
    samples: List[int] = []
    for lat, lon in points:
        try:
            samples.append(_dem_alt(float(lat), float(lon)))
        except Exception:
            continue
    if not samples:
        return None
    samples.sort()
    mid = len(samples) // 2
    if len(samples) % 2:
        return float(samples[mid])
    return (float(samples[mid - 1]) + float(samples[mid])) / 2.0


def _sample_ground_profile_along_coords(
    coords: List[Dict[str, Any]],
    *,
    sample_step_m: float = 120.0,
    max_samples_per_leg: int = 24,
) -> List[float]:
    usable: List[Tuple[float, float]] = []
    for coord in coords or []:
        if not isinstance(coord, dict):
            continue
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is not None and lon is not None:
            usable.append((float(lat), float(lon)))
    if not usable:
        return []

    samples: List[float] = [float(_dem_alt(usable[0][0], usable[0][1]))]
    for idx in range(1, len(usable)):
        prev_coord = {"latitude": usable[idx - 1][0], "longitude": usable[idx - 1][1]}
        curr_coord = {"latitude": usable[idx][0], "longitude": usable[idx][1]}
        prev_xy = coord_to_xy(prev_coord)
        curr_xy = coord_to_xy(curr_coord)
        if prev_xy is None or curr_xy is None:
            continue
        seg_dist = _distance_xy(prev_xy, curr_xy)
        if seg_dist <= 1e-6:
            continue
        step_m = max(float(sample_step_m), 1.0)
        subdivisions = max(1, int(math.ceil(seg_dist / step_m)))
        subdivisions = min(subdivisions, int(max_samples_per_leg))
        prev_lat, prev_lon = usable[idx - 1]
        curr_lat, curr_lon = usable[idx]
        for step_idx in range(1, subdivisions + 1):
            ratio = step_idx / subdivisions
            lat = prev_lat + ((curr_lat - prev_lat) * ratio)
            lon = prev_lon + ((curr_lon - prev_lon) * ratio)
            samples.append(float(_dem_alt(lat, lon)))
    return samples


def _ground_mid_m_from_coords(
    coords: List[Dict[str, Any]],
    *,
    fallback_coord: Dict[str, Any] | None = None,
    fallback_ground_ref_m: float | None = None,
) -> float | None:
    samples = _sample_ground_profile_along_coords(coords)
    if not samples and isinstance(fallback_coord, dict):
        lat = _to_float(fallback_coord.get("latitude"))
        lon = _to_float(fallback_coord.get("longitude"))
        if lat is not None and lon is not None:
            samples.append(float(_dem_alt(float(lat), float(lon))))
    if not samples:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None
    return (min(samples) + max(samples)) / 2.0


def _ground_required_m_from_coords(
    coords: List[Dict[str, Any]],
    *,
    fallback_coord: Dict[str, Any] | None = None,
    fallback_ground_ref_m: float | None = None,
) -> float | None:
    samples: List[float] = _sample_ground_profile_along_coords(coords)
    for coord in coords or []:
        if not isinstance(coord, dict):
            continue
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        try:
            samples.append(float(_dem_alt(float(lat), float(lon))))
        except Exception:
            continue
    if not samples and isinstance(fallback_coord, dict):
        lat = _to_float(fallback_coord.get("latitude"))
        lon = _to_float(fallback_coord.get("longitude"))
        if lat is not None and lon is not None:
            try:
                samples.append(float(_dem_alt(float(lat), float(lon))))
            except Exception:
                pass
    if not samples:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None
    return max(samples)


def _waypoint_required_ground_m(
    waypoint: Dict[str, Any],
    coord: Dict[str, Any],
    *,
    fallback_ground_ref_m: float | None = None,
) -> float | None:
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    line_coords = [item for item in (line_search.get("coordinateList") or []) if isinstance(item, dict)]
    if line_coords:
        return _ground_required_m_from_coords(
            line_coords,
            fallback_coord=coord,
            fallback_ground_ref_m=fallback_ground_ref_m,
        )

    orientation = filming.get("coordinateOrientation") if isinstance(filming.get("coordinateOrientation"), dict) else {}
    target_coord = orientation.get("coordinate") if isinstance(orientation.get("coordinate"), dict) else None
    ref_coord = target_coord if isinstance(target_coord, dict) else coord
    lat = _to_float(ref_coord.get("latitude")) if isinstance(ref_coord, dict) else None
    lon = _to_float(ref_coord.get("longitude")) if isinstance(ref_coord, dict) else None
    if lat is None or lon is None:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None
    try:
        return float(_dem_alt(float(lat), float(lon)))
    except Exception:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None


def _apply_segment_altitude_to_search_waypoints(
    waypoints: List[Dict[str, Any]],
    *,
    altitude_offset_m: float,
    fallback_ground_ref_m: float | None = None,
) -> None:
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else {}
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        ground_required_m = _waypoint_required_ground_m(
            waypoint,
            coord,
            fallback_ground_ref_m=fallback_ground_ref_m,
        )
        if ground_required_m is None:
            continue
        waypoint["coordinate"] = {
            "latitude": round(float(lat), 6),
            "longitude": round(float(lon), 6),
            "altitude": int(round(float(ground_required_m) + float(altitude_offset_m))),
        }


def _enforce_filming_target_altitude_floor_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    clearance_m: float = FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M,
) -> None:
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else None
        if not isinstance(coord, dict):
            continue
        current_altitude = _to_float(coord.get("altitude"))
        if current_altitude is None:
            continue
        ground_required_m = _waypoint_required_ground_m(waypoint, coord)
        if ground_required_m is None:
            continue
        minimum_altitude = int(math.ceil(float(ground_required_m) + max(float(clearance_m), 0.0)))
        if int(round(float(current_altitude))) >= minimum_altitude:
            continue
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        waypoint["coordinate"] = {
            "latitude": round(float(lat), 6),
            "longitude": round(float(lon), 6),
            "altitude": int(minimum_altitude),
        }


def _align_point_anchor_altitude_with_search_waypoints(
    waypoints: List[Dict[str, Any]],
) -> None:
    if not waypoints:
        return
    next_line_altitude_by_idx: Dict[int, int] = {}
    next_altitude: int | None = None
    for idx in range(len(waypoints) - 1, -1, -1):
        waypoint = waypoints[idx]
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        if line_search:
            coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else {}
            altitude = _to_float(coord.get("altitude"))
            if altitude is not None:
                next_altitude = int(round(float(altitude)))
        if next_altitude is not None:
            next_line_altitude_by_idx[idx] = int(next_altitude)
    for idx, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        if int(_to_float(filming.get("operationMode")) or 0) != OPMODE_POINT:
            continue
        if not isinstance(filming.get("coordinateOrientation"), dict):
            continue
        coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else {}
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        target_altitude = next_line_altitude_by_idx.get(idx)
        if lat is None or lon is None or target_altitude is None:
            continue
        waypoint["coordinate"] = {
            "latitude": round(float(lat), 6),
            "longitude": round(float(lon), 6),
            "altitude": int(target_altitude),
        }


def _runtime_uav_climb_rate_mps() -> float:
    try:
        value = float(get_runtime_float("uav_climb_rate_mps", UAV_CLIMB_RATE_MPS))
    except Exception:
        value = float(UAV_CLIMB_RATE_MPS)
    return max(value, 0.1)


def _runtime_uav_min_forward_speed_mps() -> float:
    for key in ("uav_min_forward_speed_mps", "uav_min_speed_mps"):
        try:
            value = float(get_runtime_float(key, UAV_MIN_FORWARD_SPEED_MPS))
        except Exception:
            continue
        if value > 0.0:
            return float(value)
    return float(UAV_MIN_FORWARD_SPEED_MPS)


def _distance_between_coords_m(left: Dict[str, Any], right: Dict[str, Any]) -> float:
    lat1 = _to_float(left.get("latitude"))
    lon1 = _to_float(left.get("longitude"))
    lat2 = _to_float(right.get("latitude"))
    lon2 = _to_float(right.get("longitude"))
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


def _enforce_waypoint_altitude_rate_limit_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    vertical_rate_mps: float | None = None,
    default_speed_mps: float = 50.0,
) -> None:
    if not waypoints:
        return
    rate_mps = _runtime_uav_climb_rate_mps() if vertical_rate_mps is None else max(float(vertical_rate_mps), 0.1)
    try:
        fallback_speed_mps = max(float(default_speed_mps), 1.0)
    except Exception:
        fallback_speed_mps = 50.0

    items: List[Tuple[Dict[str, Any], float, float, float]] = []
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else {}
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        alt = _to_float(coord.get("altitude"))
        if lat is None or lon is None or alt is None:
            continue
        items.append((waypoint, float(lat), float(lon), float(alt)))
    if len(items) < 2:
        return

    mutable_alts = [float(item[3]) for item in items]
    for pos in range(len(items) - 2, -1, -1):
        prev_wp, prev_lat, prev_lon, _ = items[pos]
        next_wp, next_lat, next_lon, _ = items[pos + 1]
        seg_dist_m = _distance_between_coords_m(
            {"latitude": prev_lat, "longitude": prev_lon},
            {"latitude": next_lat, "longitude": next_lon},
        )
        if seg_dist_m <= 0.0:
            continue
        speed_raw = _to_float(next_wp.get("speed"))
        speed_mps = max(float(speed_raw), 1.0) if speed_raw is not None else fallback_speed_mps
        allowed_delta_m = rate_mps * (float(seg_dist_m) / speed_mps)
        required_prev_alt = float(mutable_alts[pos + 1]) - float(allowed_delta_m)
        if float(mutable_alts[pos]) >= required_prev_alt:
            continue
        new_alt = int(math.ceil(required_prev_alt))
        mutable_alts[pos] = float(new_alt)
        prev_wp["coordinate"] = {
            "latitude": round(float(prev_lat), 6),
            "longitude": round(float(prev_lon), 6),
            "altitude": int(new_alt),
        }


def _stabilize_entry_transition_altitude_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    entry_coord: Dict[str, Any] | None,
    default_speed_mps: float = 50.0,
) -> None:
    if not waypoints or not isinstance(entry_coord, dict):
        return
    first_wp = waypoints[0] if isinstance(waypoints[0], dict) else None
    if not isinstance(first_wp, dict):
        return
    if not _is_point_hold_waypoint(first_wp):
        return
    first_coord = first_wp.get("coordinate") if isinstance(first_wp.get("coordinate"), dict) else None
    if not isinstance(first_coord, dict):
        return

    entry_alt = _to_float(entry_coord.get("altitude"))
    first_alt = _to_float(first_coord.get("altitude"))
    if entry_alt is None or first_alt is None:
        return

    rate_mps = _runtime_uav_climb_rate_mps()
    speed_mps = max(_to_float(first_wp.get("speed")) or float(default_speed_mps), 1.0)
    entry_distance_m = _distance_between_coords_m(entry_coord, first_coord)
    if entry_distance_m > 0.0 and float(first_alt) > float(entry_alt):
        max_first_alt = float(entry_alt) + (rate_mps * (entry_distance_m / speed_mps))
        if float(first_alt) > max_first_alt:
            first_alt = math.floor(max_first_alt)
            first_coord["altitude"] = int(first_alt)
            first_wp["coordinate"] = first_coord

    if len(waypoints) < 2:
        return
    next_wp = waypoints[1] if isinstance(waypoints[1], dict) else None
    if not isinstance(next_wp, dict):
        return
    next_coord = next_wp.get("coordinate") if isinstance(next_wp.get("coordinate"), dict) else None
    if not isinstance(next_coord, dict):
        return
    next_alt = _to_float(next_coord.get("altitude"))
    if next_alt is None or float(next_alt) <= float(first_alt):
        return

    segment_distance_m = _distance_between_coords_m(first_coord, next_coord)
    if segment_distance_m <= 0.0:
        return
    current_next_speed = max(
        _to_float(next_wp.get("speed")) or _to_float(first_wp.get("speed")) or float(default_speed_mps),
        1.0,
    )
    required_speed_mps = rate_mps * (segment_distance_m / max(float(next_alt) - float(first_alt), 1.0))
    if required_speed_mps <= 0.0 or required_speed_mps >= current_next_speed:
        return
    next_wp["speed"] = round(max(float(required_speed_mps), _runtime_uav_min_forward_speed_mps()), 2)


def _legacy_altitude_helpers() -> Dict[str, Any]:
    global _LEGACY_ALTITUDE_HELPERS
    global _LEGACY_ALTITUDE_HELPERS_LOADED
    if _LEGACY_ALTITUDE_HELPERS_LOADED:
        return _LEGACY_ALTITUDE_HELPERS or {}
    _LEGACY_ALTITUDE_HELPERS_LOADED = True
    _LEGACY_ALTITUDE_HELPERS = {
        "aircraft_alt_offset_m": _aircraft_alt_offset_m,
        "align_point_anchor_altitude_with_search_waypoints": _align_point_anchor_altitude_with_search_waypoints,
        "apply_segment_altitude_to_search_waypoints": _apply_segment_altitude_to_search_waypoints,
        "collect_ref_points_from_info": _collect_ref_points_from_info,
        "dem_alt": _dem_alt,
        "enforce_waypoint_altitude_rate_limit_inplace": _enforce_waypoint_altitude_rate_limit_inplace,
        "median_ground_m": _median_ground_m,
    }
    return _LEGACY_ALTITUDE_HELPERS or {}


def _mission_altitude_policy(
    *,
    aircraft_id: int,
    mission_info: Dict[str, Any] | None,
) -> Tuple[float, float | None, Callable[[float, float], int] | None]:
    helpers = _legacy_altitude_helpers()
    if not helpers:
        return 0.0, None, None
    try:
        altitude_offset_m = float(helpers["aircraft_alt_offset_m"](int(aircraft_id)))
    except Exception:
        altitude_offset_m = 0.0
    try:
        ref_points = helpers["collect_ref_points_from_info"](mission_info or {})
    except Exception:
        ref_points = []
    try:
        ground_ref_m = helpers["median_ground_m"](ref_points)
    except Exception:
        ground_ref_m = None
    dem_alt = helpers.get("dem_alt")

    def _altitude_fn(lat: float, lon: float) -> int:
        if ground_ref_m is None:
            try:
                ground_m = float(dem_alt(float(lat), float(lon))) if callable(dem_alt) else 0.0
            except Exception:
                ground_m = 0.0
        else:
            ground_m = float(ground_ref_m)
        return int(round(ground_m + altitude_offset_m))

    return float(altitude_offset_m), ground_ref_m, _altitude_fn


def _coord_with_altitude(
    coord: Dict[str, Any],
    altitude_fn: Callable[[float, float], int] | None,
) -> Dict[str, float]:
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return {
            "latitude": float(coord.get("latitude", 0.0) or 0.0),
            "longitude": float(coord.get("longitude", 0.0) or 0.0),
            "altitude": _altitude_int(coord.get("altitude", 0.0)),
        }
    altitude = _to_float(coord.get("altitude"))
    if altitude_fn is not None:
        try:
            altitude = float(altitude_fn(float(lat), float(lon)))
        except Exception:
            altitude = altitude
    if altitude is None:
        altitude = 0.0
    return {
        "latitude": float(lat),
        "longitude": float(lon),
        "altitude": _altitude_int(altitude),
    }


def _xy_to_coord_with_altitude(
    point_xy: Sequence[float],
    altitude_fn: Callable[[float, float], int] | None,
) -> Dict[str, float]:
    return _coord_with_altitude(_xy_to_coord(point_xy), altitude_fn)


def _coord_with_dem_altitude(coord: Dict[str, Any]) -> Dict[str, float]:
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return {
            "latitude": float(coord.get("latitude", 0.0) or 0.0),
            "longitude": float(coord.get("longitude", 0.0) or 0.0),
            "altitude": _altitude_int(coord.get("altitude", 0.0)),
        }
    return {
        "latitude": float(lat),
        "longitude": float(lon),
        "altitude": int(_dem_alt(float(lat), float(lon))),
    }


def _xy_to_coord_with_dem_altitude(
    point_xy: Sequence[float],
) -> Dict[str, float]:
    return _coord_with_dem_altitude(_xy_to_coord(point_xy))


def _apply_altitude_to_coord_list(
    coords: List[Dict[str, Any]],
    altitude_fn: Callable[[float, float], int] | None,
) -> List[Dict[str, float]]:
    return [
        _coord_with_altitude(coord, altitude_fn)
        for coord in coords
        if isinstance(coord, dict)
    ]


def _apply_altitude_to_mission_info_inplace(
    info: Dict[str, Any],
    *,
    aircraft_id: int,
) -> None:
    if not isinstance(info, dict):
        return
    _, _, altitude_fn = _mission_altitude_policy(
        aircraft_id=int(aircraft_id),
        mission_info=info,
    )
    if altitude_fn is None:
        return
    if isinstance(info.get("coordinateList"), list):
        info["coordinateList"] = _apply_altitude_to_coord_list(
            list(info.get("coordinateList") or []),
            altitude_fn,
        )
    for line in info.get("lineList") or []:
        if isinstance(line, dict) and isinstance(line.get("coordinateList"), list):
            line["coordinateList"] = _apply_altitude_to_coord_list(
                list(line.get("coordinateList") or []),
                altitude_fn,
            )
    for area in info.get("areaList") or []:
        if isinstance(area, dict) and isinstance(area.get("coordinateList"), list):
            area["coordinateList"] = _apply_altitude_to_coord_list(
                list(area.get("coordinateList") or []),
                altitude_fn,
            )
    if isinstance(info.get("sourceCoordinateList"), list):
        info["sourceCoordinateList"] = _apply_altitude_to_coord_list(
            list(info.get("sourceCoordinateList") or []),
            altitude_fn,
        )
    for line in info.get("sourceLineList") or []:
        if isinstance(line, dict) and isinstance(line.get("coordinateList"), list):
            line["coordinateList"] = _apply_altitude_to_coord_list(
                list(line.get("coordinateList") or []),
                altitude_fn,
            )


def _apply_legacy_altitude_profile_to_waypoints(
    waypoints: List[Dict[str, Any]],
    *,
    aircraft_id: int,
    mission_info: Dict[str, Any] | None,
) -> None:
    helpers = _legacy_altitude_helpers()
    if not helpers or not waypoints:
        return
    altitude_offset_m, ground_ref_m, _ = _mission_altitude_policy(
        aircraft_id=int(aircraft_id),
        mission_info=mission_info,
    )
    try:
        helpers["apply_segment_altitude_to_search_waypoints"](
            waypoints,
            altitude_offset_m=float(altitude_offset_m),
            fallback_ground_ref_m=ground_ref_m,
        )
        helpers["align_point_anchor_altitude_with_search_waypoints"](waypoints)
        enforce_fn = helpers.get("enforce_waypoint_altitude_rate_limit_inplace")
        if callable(enforce_fn):
            enforce_fn(waypoints)
    except Exception:
        return


def _preserve_first_waypoint_altitude_from_entry(
    waypoints: List[Dict[str, Any]],
    *,
    entry_coord: Dict[str, Any] | None,
) -> None:
    if not waypoints or not isinstance(entry_coord, dict):
        return
    entry_altitude = _to_float(entry_coord.get("altitude"))
    if entry_altitude is None:
        return
    first_waypoint = waypoints[0] if isinstance(waypoints[0], dict) else None
    if not isinstance(first_waypoint, dict):
        return
    if not _is_point_hold_waypoint(first_waypoint):
        return
    coordinate = first_waypoint.get("coordinate") if isinstance(first_waypoint.get("coordinate"), dict) else None
    if not isinstance(coordinate, dict):
        return
    current_altitude = _to_float(coordinate.get("altitude"))
    if (
        current_altitude is not None
        and (
            float(entry_altitude) < float(ENTRY_ALTITUDE_MIN_M)
            or (
                float(entry_altitude) > float(current_altitude)
                and abs(float(entry_altitude) - float(current_altitude)) > float(ENTRY_ALTITUDE_MAX_PRESERVE_DELTA_M)
            )
        )
    ):
        return
    lat = _to_float(coordinate.get("latitude"))
    lon = _to_float(coordinate.get("longitude"))
    if lat is None or lon is None:
        return
    first_waypoint["coordinate"] = {
        "latitude": round(float(lat), 6),
        "longitude": round(float(lon), 6),
        "altitude": int(round(float(entry_altitude))),
    }


def _xy_rows(value: Any) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    if not isinstance(value, list):
        return rows
    for item in value:
        if not (isinstance(item, (tuple, list)) and len(item) >= 2):
            continue
        rows.append((float(item[0]), float(item[1])))
    return rows


def _dedupe_xy_rows(rows: Iterable[Tuple[float, float]], *, eps_m: float = 1.0) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    eps2 = float(eps_m) * float(eps_m)
    for point_xy in rows:
        if out:
            dx = float(point_xy[0]) - float(out[-1][0])
            dy = float(point_xy[1]) - float(out[-1][1])
            if (dx * dx + dy * dy) <= eps2:
                continue
        out.append((float(point_xy[0]), float(point_xy[1])))
    return out


def _distance_xy(start_xy: Tuple[float, float], end_xy: Tuple[float, float]) -> float:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    return float((dx * dx + dy * dy) ** 0.5)


def _line_transition_backtracks(
    *,
    entry_xy: Tuple[float, float] | None,
    transition_points_xy: Sequence[Tuple[float, float]],
    first_sweep_point_xy: Tuple[float, float] | None,
) -> bool:
    if not transition_points_xy or first_sweep_point_xy is None:
        return False
    from_xy = transition_points_xy[-2] if len(transition_points_xy) >= 2 else entry_xy
    if from_xy is None:
        return False
    to_xy = transition_points_xy[-1]
    ingress_dx = float(to_xy[0]) - float(from_xy[0])
    ingress_dy = float(to_xy[1]) - float(from_xy[1])
    scan_dx = float(first_sweep_point_xy[0]) - float(to_xy[0])
    scan_dy = float(first_sweep_point_xy[1]) - float(to_xy[1])
    ingress_len = float((ingress_dx * ingress_dx + ingress_dy * ingress_dy) ** 0.5)
    scan_len = float((scan_dx * scan_dx + scan_dy * scan_dy) ** 0.5)
    if ingress_len <= 1e-6 or scan_len <= 1e-6:
        return False
    return float((ingress_dx * scan_dx) + (ingress_dy * scan_dy)) < 0.0


def _line_ingress_entry_offset_m(path_row: Dict[str, Any]) -> float:
    offset_m = _to_float(path_row.get("lineRouteOffsetM"))
    if offset_m is None or offset_m <= 0.0:
        sep_m = _to_float(path_row.get("dbSepM")) or _to_float(path_row.get("sepCandM")) or 0.0
        if sep_m > 0.0:
            try:
                route_scale = float(get_runtime_float("line_route_offset_scale", 1.0))
            except Exception:
                route_scale = 1.0
            offset_m = float(sep_m) * max(float(route_scale), 0.1)
    if offset_m is None or offset_m <= 0.0:
        offset_m = 300.0
    try:
        entry_scale = float(get_runtime_float("next_collab_line_ingress_entry_offset_scale", 1.0))
    except Exception:
        entry_scale = 1.0
    if entry_scale <= 0.0:
        entry_scale = 1.0
    return max(1.0, float(offset_m) * float(entry_scale))


def _closest_point_on_segment_xy(
    point_xy: Tuple[float, float],
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
) -> Tuple[float, float]:
    sx, sy = float(start_xy[0]), float(start_xy[1])
    ex, ey = float(end_xy[0]), float(end_xy[1])
    px, py = float(point_xy[0]), float(point_xy[1])
    dx = ex - sx
    dy = ey - sy
    denom = (dx * dx) + (dy * dy)
    if denom <= 1e-9:
        return (sx, sy)
    t = ((px - sx) * dx + (py - sy) * dy) / denom
    t = max(0.0, min(1.0, float(t)))
    return (sx + (dx * t), sy + (dy * t))


def _closest_point_on_polyline_xy(
    point_xy: Tuple[float, float],
    line_xy: Sequence[Tuple[float, float]],
) -> Tuple[float, float] | None:
    rows = [
        (float(row[0]), float(row[1]))
        for row in line_xy
        if isinstance(row, (tuple, list)) and len(row) >= 2
    ]
    if not rows:
        return None
    if len(rows) == 1:
        return rows[0]
    best_xy: Tuple[float, float] | None = None
    best_dist_m: float | None = None
    for idx in range(len(rows) - 1):
        candidate = _closest_point_on_segment_xy(point_xy, rows[idx], rows[idx + 1])
        dist_m = _distance_xy(point_xy, candidate)
        if best_dist_m is None or dist_m < best_dist_m:
            best_xy = candidate
            best_dist_m = float(dist_m)
    return best_xy


def _line_ingress_guard_xy(
    *,
    entry_xy: Tuple[float, float] | None,
    first_sweep_xy: Sequence[Tuple[float, float]],
    first_anchor_xy: Tuple[float, float] | None,
    path_row: Dict[str, Any],
) -> Tuple[float, float] | None:
    if entry_xy is None:
        return None
    reference_xy = _closest_point_on_polyline_xy(entry_xy, first_sweep_xy)
    if reference_xy is None:
        return None
    distance_m = _distance_xy(entry_xy, reference_xy)
    offset_m = _line_ingress_entry_offset_m(path_row)
    try:
        trigger_scale = float(
            get_runtime_float(
                "next_collab_line_ingress_entry_trigger_scale",
                NEXT_COLLAB_LINE_INGRESS_ENTRY_TRIGGER_SCALE,
            )
        )
    except Exception:
        trigger_scale = float(NEXT_COLLAB_LINE_INGRESS_ENTRY_TRIGGER_SCALE)
    if trigger_scale <= 0.0:
        trigger_scale = float(NEXT_COLLAB_LINE_INGRESS_ENTRY_TRIGGER_SCALE)
    try:
        min_trigger_m = float(
            get_runtime_float(
                "next_collab_line_ingress_entry_min_trigger_m",
                NEXT_COLLAB_LINE_INGRESS_ENTRY_MIN_TRIGGER_M,
            )
        )
    except Exception:
        min_trigger_m = float(NEXT_COLLAB_LINE_INGRESS_ENTRY_MIN_TRIGGER_M)
    trigger_m = max(float(min_trigger_m), float(offset_m) * float(trigger_scale))
    if distance_m <= trigger_m:
        return None
    if first_anchor_xy is not None and _distance_xy(entry_xy, first_anchor_xy) <= trigger_m:
        return None

    dx = float(entry_xy[0]) - float(reference_xy[0])
    dy = float(entry_xy[1]) - float(reference_xy[1])
    length_m = math.hypot(dx, dy)
    if length_m <= 1e-6:
        return None
    ux = dx / length_m
    uy = dy / length_m
    guard_xy = (
        float(reference_xy[0]) + (ux * float(offset_m)),
        float(reference_xy[1]) + (uy * float(offset_m)),
    )
    if first_anchor_xy is not None:
        anchor_dist_m = _distance_xy(entry_xy, first_anchor_xy)
        guard_dist_m = _distance_xy(entry_xy, guard_xy)
        if anchor_dist_m + 25.0 < guard_dist_m:
            return None
    return guard_xy


def _line_search_coordinate_list(waypoint: Dict[str, Any]) -> List[Dict[str, Any]]:
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    coords = line_search.get("coordinateList") if isinstance(line_search.get("coordinateList"), list) else []
    out: List[Dict[str, Any]] = []
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        row: Dict[str, Any] = {
            "latitude": float(lat),
            "longitude": float(lon),
        }
        alt = _to_float(coord.get("altitude"))
        if alt is not None:
            row["altitude"] = _altitude_int(alt)
        out.append(row)
    return out


def _merge_line_search_coordinate_lists(
    prefix_coords: Sequence[Dict[str, Any]],
    suffix_coords: Sequence[Dict[str, Any]],
) -> List[Dict[str, Any]]:
    merged: List[Dict[str, Any]] = []
    for coord in list(prefix_coords or []) + list(suffix_coords or []):
        if not isinstance(coord, dict):
            continue
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        normalized: Dict[str, Any] = {
            "latitude": float(lat),
            "longitude": float(lon),
        }
        alt = _to_float(coord.get("altitude"))
        if alt is not None:
            normalized["altitude"] = _altitude_int(alt)
        if merged:
            prev_xy = coord_to_xy(merged[-1])
            curr_xy = coord_to_xy(normalized)
            if prev_xy is not None and curr_xy is not None and _distance_xy(prev_xy, curr_xy) <= 1.0:
                continue
        merged.append(normalized)
    return merged


def _waypoint_coordinate_xy(waypoint: Dict[str, Any]) -> Tuple[float, float] | None:
    coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else None
    if not isinstance(coord, dict):
        return None
    point_xy = coord_to_xy(coord)
    if point_xy is None:
        return None
    return (float(point_xy[0]), float(point_xy[1]))


def _is_point_hold_waypoint(waypoint: Dict[str, Any]) -> bool:
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    return _to_int(filming.get("operationMode")) == OPMODE_POINT and not _line_search_coordinate_list(waypoint)


def _squash_leading_short_line_search_waypoints(waypoints: List[Dict[str, Any]]) -> None:
    short_sweep_points = max(2, int(_next_collab_sweep_points_per_leg()))
    while len(waypoints) >= 3:
        if len(waypoints) >= 4:
            first_wp = waypoints[0] if isinstance(waypoints[0], dict) else {}
            second_wp = waypoints[1] if isinstance(waypoints[1], dict) else {}
            short_wp = waypoints[2] if isinstance(waypoints[2], dict) else {}
            next_wp = waypoints[3] if isinstance(waypoints[3], dict) else {}
            short_coords = _line_search_coordinate_list(short_wp)
            next_coords = _line_search_coordinate_list(next_wp)
            if (
                _is_point_hold_waypoint(first_wp)
                and _is_point_hold_waypoint(second_wp)
                and short_coords
                and len(short_coords) <= short_sweep_points
                and len(next_coords) > len(short_coords)
            ):
                first_xy = _waypoint_coordinate_xy(first_wp)
                second_xy = _waypoint_coordinate_xy(second_wp)
                short_start_xy = coord_to_xy(short_coords[0]) if short_coords else None
                if first_xy is not None and second_xy is not None and short_start_xy is not None:
                    first_dist_m = _distance_xy(first_xy, (float(short_start_xy[0]), float(short_start_xy[1])))
                    second_dist_m = _distance_xy(second_xy, (float(short_start_xy[0]), float(short_start_xy[1])))
                    if second_dist_m + 3.0 < first_dist_m:
                        del waypoints[0]
                        break

        first_wp = waypoints[0] if isinstance(waypoints[0], dict) else {}
        second_wp = waypoints[1] if isinstance(waypoints[1], dict) else {}
        first_coords = _line_search_coordinate_list(first_wp)
        second_coords = _line_search_coordinate_list(second_wp)
        if (
            first_coords
            and len(first_coords) <= short_sweep_points
            and len(second_coords) > len(first_coords)
        ):
            second_filming = second_wp.get("filmingProperty") if isinstance(second_wp.get("filmingProperty"), dict) else None
            second_line_search = (
                second_filming.get("lineSearch")
                if isinstance(second_filming, dict) and isinstance(second_filming.get("lineSearch"), dict)
                else None
            )
            if second_line_search is None:
                break
            second_line_search["coordinateList"] = _merge_line_search_coordinate_lists(first_coords, second_coords)
            del waypoints[0]
            continue

        entry_wp = waypoints[0] if isinstance(waypoints[0], dict) else {}
        short_wp = waypoints[1] if isinstance(waypoints[1], dict) else {}
        next_wp = waypoints[2] if isinstance(waypoints[2], dict) else {}

        entry_filming = entry_wp.get("filmingProperty") if isinstance(entry_wp.get("filmingProperty"), dict) else {}
        entry_op_mode = _to_int(entry_filming.get("operationMode"))
        if entry_op_mode != 1 or _line_search_coordinate_list(entry_wp):
            break

        short_coords = _line_search_coordinate_list(short_wp)
        next_coords = _line_search_coordinate_list(next_wp)
        if not short_coords or len(short_coords) > short_sweep_points:
            break
        if len(next_coords) <= len(short_coords):
            break

        next_filming = next_wp.get("filmingProperty") if isinstance(next_wp.get("filmingProperty"), dict) else None
        next_line_search = (
            next_filming.get("lineSearch")
            if isinstance(next_filming, dict) and isinstance(next_filming.get("lineSearch"), dict)
            else None
        )
        if next_line_search is None:
            break

        entry_xy = _waypoint_coordinate_xy(entry_wp)
        short_xy = _waypoint_coordinate_xy(short_wp)
        if entry_xy is not None and short_xy is not None and _distance_xy(entry_xy, short_xy) <= 3.0:
            break

        next_line_search["coordinateList"] = _merge_line_search_coordinate_lists(short_coords, next_coords)
        del waypoints[1]
        continue


def _squash_trailing_short_line_search_waypoints(
    waypoints: List[Dict[str, Any]],
    *,
    spacing_m: float,
    transit_speed_mps: float,
    fallback_search_speed_mps: float,
    speed_scale: float,
) -> None:
    try:
        target_spacing_m = max(float(spacing_m), 1.0)
    except Exception:
        target_spacing_m = float(_next_collab_line_route_wp_spacing_m())

    while len(waypoints) >= 2:
        tail_idx = len(waypoints) - 1
        prev_idx = tail_idx - 1
        prev_wp = waypoints[prev_idx] if isinstance(waypoints[prev_idx], dict) else {}
        tail_wp = waypoints[tail_idx] if isinstance(waypoints[tail_idx], dict) else {}
        prev_coords = _line_search_coordinate_list(prev_wp)
        tail_coords = _line_search_coordinate_list(tail_wp)
        if not prev_coords or not tail_coords:
            break

        prev_xy = _waypoint_coordinate_xy(prev_wp)
        tail_xy = _waypoint_coordinate_xy(tail_wp)
        if prev_xy is None or tail_xy is None:
            break
        if _distance_xy(prev_xy, tail_xy) + 1e-6 >= target_spacing_m:
            break

        tail_filming = tail_wp.get("filmingProperty") if isinstance(tail_wp.get("filmingProperty"), dict) else None
        tail_line_search = (
            tail_filming.get("lineSearch")
            if isinstance(tail_filming, dict) and isinstance(tail_filming.get("lineSearch"), dict)
            else None
        )
        if tail_line_search is None:
            break

        first_line_idx = next(
            (
                idx
                for idx, waypoint in enumerate(waypoints)
                if isinstance(waypoint, dict) and _line_search_coordinate_list(waypoint)
            ),
            None,
        )
        if first_line_idx == prev_idx:
            prev_filming = prev_wp.get("filmingProperty") if isinstance(prev_wp.get("filmingProperty"), dict) else {}
            for key in ("fieldOfView", "sensorType"):
                if key in prev_filming:
                    tail_filming[key] = deepcopy(prev_filming[key])

        merged_coords = _merge_line_search_coordinate_lists(prev_coords, tail_coords)
        if not merged_coords:
            break

        previous_coord = None
        if prev_idx - 1 >= 0 and isinstance(waypoints[prev_idx - 1], dict):
            previous_coord = waypoints[prev_idx - 1].get("coordinate")
        line_search_speed_mps = _estimate_line_search_speed_mps(
            prev_coord=previous_coord if isinstance(previous_coord, dict) else None,
            anchor_coord=tail_wp.get("coordinate") if isinstance(tail_wp.get("coordinate"), dict) else None,
            sweep_coords=merged_coords,
            cruise_speed_mps=float(transit_speed_mps),
            fallback_search_speed_mps=float(fallback_search_speed_mps),
            speed_scale=float(speed_scale),
        )
        tail_line_search["coordinateList"] = merged_coords
        tail_line_search["searchSpeed"] = float(line_search_speed_mps)
        del waypoints[prev_idx]


def _line_length_xy(points_xy: List[Tuple[float, float]]) -> float:
    if len(points_xy) < 2:
        return 0.0
    total = 0.0
    for idx in range(1, len(points_xy)):
        total += _distance_xy(points_xy[idx - 1], points_xy[idx])
    return float(total)


def _midpoint_xy(points_xy: List[Tuple[float, float]]) -> Tuple[float, float] | None:
    if not points_xy:
        return None
    if len(points_xy) == 1:
        return (float(points_xy[0][0]), float(points_xy[0][1]))
    start_xy = points_xy[0]
    end_xy = points_xy[-1]
    return (
        (float(start_xy[0]) + float(end_xy[0])) * 0.5,
        (float(start_xy[1]) + float(end_xy[1])) * 0.5,
    )


def _next_collab_sweep_points_per_leg() -> int:
    value = int(get_runtime_int("next_collab_sweep_points_per_leg", NEXT_COLLAB_SWEEP_POINTS_PER_LEG))
    return max(2, min(9, int(value)))


def _next_collab_auto_sweep_points() -> bool:
    return bool(get_runtime_bool("next_collab_auto_sweep_points", False))


def _next_collab_sweep_points_for_line(points_xy: List[Tuple[float, float]]) -> int:
    if not _next_collab_auto_sweep_points():
        return _next_collab_sweep_points_per_leg()
    length_m = _line_length_xy(points_xy)
    if length_m <= 1.0:
        return 2
    spacing_m = max(float(_next_collab_line_route_wp_spacing_m()), 1.0)
    return max(2, min(9, int(math.ceil(float(length_m) / float(spacing_m))) + 1))


def _next_collab_line_route_wp_spacing_m() -> float:
    value = float(get_runtime_float("uav_wp_interval_m", NEXT_COLLAB_LINE_ROUTE_WP_SPACING_M))
    return max(1.0, float(value))


def _next_collab_area_route_wp_spacing_m() -> float:
    spacing_m = _next_collab_line_route_wp_spacing_m()
    scale = float(get_runtime_float("area_first_packet_sweep_group_scale", 1.5))
    if scale <= 0.0:
        scale = 1.0
    return max(1.0, float(spacing_m) * float(scale))


def _next_collab_first_line_search_fov_deg(base_fov_deg: float) -> float:
    base = max(0.1, float(base_fov_deg))
    scale = float(get_runtime_float("next_collab_first_line_fov_scale", NEXT_COLLAB_FIRST_LINE_FOV_SCALE))
    cap = float(get_runtime_float("next_collab_first_line_fov_max_deg", NEXT_COLLAB_FIRST_LINE_FOV_MAX_DEG))
    if scale <= 0.0:
        scale = float(NEXT_COLLAB_FIRST_LINE_FOV_SCALE)
    if cap <= 0.0:
        cap = float(NEXT_COLLAB_FIRST_LINE_FOV_MAX_DEG)
    boosted = max(base, min(float(cap), float(base) * float(scale)))
    return float(
        apply_runtime_camera_adjusted_fov_deg(
            float(boosted),
            context="NEXTCOLLAB FIRST_LINE",
        )
    )


def _runtime_flyover_options() -> Dict[str, bool]:
    try:
        flyover = load_runtime_flyover()
    except Exception:
        flyover = {}
    if not isinstance(flyover, dict):
        return {"entry_offset": False, "dubins_prefix": False, "last_point": False, "all_wps": False}
    return {
        "entry_offset": bool(flyover.get("entry_offset", False)),
        "dubins_prefix": bool(flyover.get("dubins_prefix", False)),
        "last_point": bool(flyover.get("last_point", False)),
        "all_wps": bool(flyover.get("all_wps", False)),
    }


def _clear_runtime_flyover_markers(waypoints: List[Dict[str, Any]]) -> None:
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        waypoint.pop("_flyover_dubins_prefix", None)


def _apply_runtime_flyover_to_waypoints(waypoints: List[Dict[str, Any]]) -> None:
    if not isinstance(waypoints, list) or not waypoints:
        return
    flyover = _runtime_flyover_options()
    if flyover.get("all_wps"):
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                continue
            if int(_to_float(waypoint.get("waypointPassType")) or 0) == PASS_LOITER:
                continue
            waypoint["waypointPassType"] = PASS_FLYOVER
        _clear_runtime_flyover_markers(waypoints)
        return
    if flyover.get("entry_offset"):
        # Match the legacy/general planner semantics: promote only the
        # first collaborative-mission start waypoint.
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                continue
            if int(_to_float(waypoint.get("waypointPassType")) or 0) == PASS_LOITER:
                continue
            waypoint["waypointPassType"] = PASS_FLYOVER
            break
    if flyover.get("dubins_prefix"):
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                continue
            if not waypoint.get("_flyover_dubins_prefix"):
                continue
            if int(_to_float(waypoint.get("waypointPassType")) or 0) == PASS_LOITER:
                continue
            waypoint["waypointPassType"] = PASS_FLYOVER
    if flyover.get("last_point"):
        for waypoint in reversed(waypoints):
            if not isinstance(waypoint, dict):
                continue
            if int(_to_float(waypoint.get("waypointPassType")) or 0) == PASS_LOITER:
                continue
            waypoint["waypointPassType"] = PASS_FLYOVER
            break
    _clear_runtime_flyover_markers(waypoints)


def _line_three_point_xy(points_xy: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    rows = _dedupe_xy_rows(points_xy, eps_m=0.5)
    if len(rows) < 2:
        return rows
    sample_count = _next_collab_sweep_points_for_line(rows)
    if sample_count <= 2:
        return [rows[0], rows[-1]]
    start_xy = rows[0]
    end_xy = rows[-1]
    out = []
    for idx in range(sample_count):
        ratio = float(idx) / float(sample_count - 1)
        out.append(
            (
                float(start_xy[0]) + ((float(end_xy[0]) - float(start_xy[0])) * ratio),
                float(start_xy[1]) + ((float(end_xy[1]) - float(start_xy[1])) * ratio),
            )
        )
    return _dedupe_xy_rows(out, eps_m=0.5)


def _path_row_has_turn_prefix(path_row: Dict[str, Any]) -> bool:
    horizon_sec = _to_float(path_row.get("horizonSec"))
    if horizon_sec is not None and horizon_sec > 1e-6:
        return True
    route_xy_raw = path_row.get("routeXY")
    tangent_xy_raw = path_row.get("tangentXY")
    if not (isinstance(route_xy_raw, list) and isinstance(tangent_xy_raw, (tuple, list)) and len(tangent_xy_raw) >= 2):
        return False
    tangent_xy = (float(tangent_xy_raw[0]), float(tangent_xy_raw[1]))
    unique_points: List[Tuple[float, float]] = []
    for raw_xy in route_xy_raw:
        if not (isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2):
            continue
        point_xy = (float(raw_xy[0]), float(raw_xy[1]))
        if unique_points and _distance_xy(unique_points[-1], point_xy) <= 1.0:
            continue
        unique_points.append(point_xy)
    for idx, point_xy in enumerate(unique_points):
        if _distance_xy(point_xy, tangent_xy) <= 3.0:
            return idx > 1
    return False


def _flatten_sweep_lines_xy(scan_lines_xy: List[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
    merged: List[Tuple[float, float]] = []
    for line_xy in scan_lines_xy:
        if len(line_xy) < 2:
            continue
        for point_xy in _line_three_point_xy(line_xy):
            merged.append((float(point_xy[0]), float(point_xy[1])))
    return _dedupe_xy_rows(merged, eps_m=0.5)


def _line_sweep_items_xy(path_row: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows = path_row.get("lineSweepItemsXY") if isinstance(path_row.get("lineSweepItemsXY"), list) else []
    out: List[Dict[str, Any]] = []
    for item in rows:
        if not isinstance(item, dict):
            continue
        anchor_raw = item.get("anchorXY")
        sweep_xy = _dedupe_xy_rows(_xy_rows(item.get("sweepXY")), eps_m=0.5)
        if not (isinstance(anchor_raw, (tuple, list)) and len(anchor_raw) >= 2):
            continue
        if len(sweep_xy) < 2:
            continue
        out.append(
            {
                "anchorXY": (float(anchor_raw[0]), float(anchor_raw[1])),
                "sweepXY": sweep_xy,
                "sweepIndex": int(_to_float(item.get("sweepIndex")) or 0),
            }
        )
    return out


def _xy_pair(value: Any) -> Tuple[float, float] | None:
    if not (isinstance(value, (tuple, list)) and len(value) >= 2):
        return None
    try:
        return (float(value[0]), float(value[1]))
    except Exception:
        return None


def _area_sweep_items_xy(path_row: Dict[str, Any], scan_lines_xy: List[List[Tuple[float, float]]]) -> List[Dict[str, Any]]:
    if not scan_lines_xy:
        return []
    start_xy = (
        _xy_pair(path_row.get("waypointStartXY"))
        or _xy_pair(path_row.get("entryTPrimeXY"))
        or _xy_pair(path_row.get("tangentXY"))
    )
    end_xy = _xy_pair(path_row.get("waypointEndXY"))
    if start_xy is None:
        for line_xy in scan_lines_xy:
            if line_xy:
                start_xy = (float(line_xy[0][0]), float(line_xy[0][1]))
                break
    if end_xy is None:
        for line_xy in reversed(scan_lines_xy):
            if line_xy:
                end_xy = (float(line_xy[-1][0]), float(line_xy[-1][1]))
                break
    if start_xy is None or end_xy is None:
        return []

    route_dx = float(end_xy[0]) - float(start_xy[0])
    route_dy = float(end_xy[1]) - float(start_xy[1])
    route_len_m = math.hypot(route_dx, route_dy)
    if route_len_m <= 1.0:
        return []
    ux = route_dx / route_len_m
    uy = route_dy / route_len_m

    items: List[Dict[str, Any]] = []
    previous_progress_m = 0.0
    for idx, line_xy in enumerate(scan_lines_xy):
        rows = _dedupe_xy_rows(_xy_rows(line_xy), eps_m=0.5)
        if len(rows) < 2:
            continue
        midpoint = _midpoint_xy(rows)
        if midpoint is None:
            continue
        progress_m = (
            (float(midpoint[0]) - float(start_xy[0])) * ux
            + (float(midpoint[1]) - float(start_xy[1])) * uy
        )
        progress_m = max(previous_progress_m, min(float(route_len_m), float(progress_m)))
        anchor_xy = (
            float(start_xy[0]) + (ux * progress_m),
            float(start_xy[1]) + (uy * progress_m),
        )
        items.append(
            {
                "anchorXY": anchor_xy,
                "sweepXY": rows,
                "sweepIndex": int(idx),
                "progressM": float(progress_m),
            }
        )
        previous_progress_m = float(progress_m)
    if items and end_xy is not None:
        last_item = items[-1]
        last_progress_m = _to_float(last_item.get("progressM"))
        route_tail_m = (
            max(0.0, float(route_len_m) - float(last_progress_m))
            if last_progress_m is not None
            else 0.0
        )
        try:
            route_spacing_m = max(float(_next_collab_area_route_wp_spacing_m()), 1.0)
        except Exception:
            route_spacing_m = float(NEXT_COLLAB_LINE_ROUTE_WP_SPACING_M)
        # The endpoint snap is only a small closure correction. If the last
        # sweep is still far from waypointEndXY, forcing the anchor to endXY
        # creates an isolated tail waypoint whose flight coordinate is nowhere
        # near the actual camera sweep line.
        end_snap_limit_m = max(100.0, min(500.0, route_spacing_m * 0.25))
        if route_tail_m <= end_snap_limit_m:
            last_item["anchorXY"] = (float(end_xy[0]), float(end_xy[1]))
            last_item["progressM"] = float(route_len_m)
    return items


def _group_area_sweep_items_by_spacing(
    items: List[Dict[str, Any]],
    *,
    spacing_m: float,
    merge_short_tail: bool = True,
) -> List[List[Dict[str, Any]]]:
    if not items:
        return []
    target_spacing_m = max(float(spacing_m), 1.0)
    groups: List[List[Dict[str, Any]]] = []
    current_group: List[Dict[str, Any]] = [items[0]]
    anchor_progress_m = 0.0

    for item in items[1:]:
        candidate_progress_m = float(item.get("progressM", anchor_progress_m) or anchor_progress_m)
        last_progress_m = float(current_group[-1].get("progressM", anchor_progress_m) or anchor_progress_m)
        if (
            current_group
            and (candidate_progress_m - anchor_progress_m) > target_spacing_m
            and (last_progress_m - anchor_progress_m) >= 1.0
        ):
            groups.append(current_group)
            anchor_progress_m = last_progress_m
            current_group = [item]
        else:
            current_group.append(item)
    if current_group:
        groups.append(current_group)

    if merge_short_tail:
        while len(groups) >= 2:
            prev_progress_m = float(groups[-2][-1].get("progressM", 0.0) or 0.0)
            tail_progress_m = float(groups[-1][-1].get("progressM", prev_progress_m) or prev_progress_m)
            if tail_progress_m - prev_progress_m + 1e-6 >= target_spacing_m:
                break
            groups[-1] = groups[-2] + groups[-1]
            groups.pop(-2)
    return groups


def _group_line_sweep_items_by_spacing(
    items: List[Dict[str, Any]],
    *,
    spacing_m: float | None = None,
    merge_short_tail: bool = True,
) -> List[List[Dict[str, Any]]]:
    if not items:
        return []
    try:
        target_spacing_m = max(
            float(_next_collab_line_route_wp_spacing_m() if spacing_m is None else spacing_m),
            1.0,
        )
    except Exception:
        target_spacing_m = float(_next_collab_line_route_wp_spacing_m())

    groups: List[List[Dict[str, Any]]] = []
    current_group: List[Dict[str, Any]] = [items[0]]
    anchor_xy = items[0].get("anchorXY")
    progressed_m = 0.0
    prev_anchor_xy = anchor_xy if isinstance(anchor_xy, tuple) else None

    for item in items[1:]:
        curr_anchor_xy = item.get("anchorXY")
        if not isinstance(curr_anchor_xy, tuple):
            current_group.append(item)
            continue
        step_m = (
            _distance_xy(prev_anchor_xy, curr_anchor_xy)
            if isinstance(prev_anchor_xy, tuple)
            else 0.0
        )
        projected_m = progressed_m + float(step_m)
        if current_group and projected_m > target_spacing_m and progressed_m >= 1.0:
            groups.append(current_group)
            current_group = [item]
            progressed_m = float(step_m)
        else:
            current_group.append(item)
            progressed_m = float(projected_m)
        prev_anchor_xy = curr_anchor_xy
    if current_group:
        groups.append(current_group)

    if merge_short_tail:
        while len(groups) >= 2:
            prev_tail = groups[-2][-1].get("anchorXY")
            tail = groups[-1][-1].get("anchorXY")
            if not (isinstance(prev_tail, tuple) and isinstance(tail, tuple)):
                break
            if _distance_xy(prev_tail, tail) + 1e-6 >= target_spacing_m:
                break
            tail_counts = _line_group_sweep_span_counts(groups)
            tail_span = tail_counts[-1] if tail_counts else None
            if tail_span is None or int(tail_span) > 2:
                break
            groups[-1] = groups[-2] + groups[-1]
            groups.pop(-2)
    return _rebalance_line_sweep_item_groups(groups)


def _line_group_sweep_span_counts(
    groups: List[List[Dict[str, Any]]],
) -> List[int | None]:
    counts: List[int | None] = []
    previous_rep_sweep_idx: int | None = None
    for group in groups:
        if not group:
            counts.append(None)
            continue
        rep_idx = _to_float((group[-1] or {}).get("sweepIndex"))
        if rep_idx is None:
            counts.append(None)
            continue
        rep_idx = int(rep_idx)
        if previous_rep_sweep_idx is None:
            start_idx = 0
        else:
            step_sign = 1 if int(rep_idx) >= int(previous_rep_sweep_idx) else -1
            start_idx = int(previous_rep_sweep_idx) + int(step_sign)
        counts.append(abs(int(rep_idx) - int(start_idx)) + 1)
        previous_rep_sweep_idx = int(rep_idx)
    return counts


def _rebalance_line_sweep_item_groups(
    groups: List[List[Dict[str, Any]]],
    *,
    tiny_span_threshold: int = 2,
    min_prev_span: int = 4,
) -> List[List[Dict[str, Any]]]:
    if len(groups) < 2:
        return groups

    out = [list(group) for group in groups if group]
    if len(out) < 2:
        return out

    while True:
        counts = _line_group_sweep_span_counts(out)
        changed = False
        for idx in range(1, len(out)):
            prev_count = counts[idx - 1]
            curr_count = counts[idx]
            if prev_count is None or curr_count is None:
                continue
            if int(curr_count) > int(tiny_span_threshold):
                continue
            if len(out[idx - 1]) <= 1:
                continue

            candidate = [list(group) for group in out]
            candidate[idx].insert(0, candidate[idx - 1].pop())
            candidate_counts = _line_group_sweep_span_counts(candidate)
            cand_prev = candidate_counts[idx - 1]
            cand_curr = candidate_counts[idx]
            if cand_prev is None or cand_curr is None:
                continue
            if int(cand_prev) < int(min_prev_span):
                continue
            if int(cand_curr) <= int(curr_count):
                continue
            if max(int(cand_prev), int(cand_curr)) >= max(int(prev_count), int(curr_count)):
                continue

            out = candidate
            changed = True
            break
        if not changed:
            return out


def _collect_group_sweep_coords(
    *,
    group: List[Dict[str, Any]],
    all_sweep_lines_xy: List[List[Tuple[float, float]]],
    previous_rep_sweep_idx: int | None = None,
) -> List[Dict[str, Any]]:
    if not group:
        return []
    indices = [
        int(_to_float(item.get("sweepIndex")) or 0)
        for item in group
        if isinstance(item, dict)
    ]
    if indices and all_sweep_lines_xy:
        rep_idx = int(indices[-1])
        if previous_rep_sweep_idx is None:
            # The planner already orients the route/sweep order toward the
            # current entry side. Starting the first merged group back at
            # sweep 0 reintroduces earlier sweeps behind the rejoin point and
            # makes the aircraft loop back before resuming the scan.
            start_idx = int(indices[0])
        else:
            step_sign = 1 if rep_idx >= int(previous_rep_sweep_idx) else -1
            start_idx = int(previous_rep_sweep_idx) + int(step_sign)
        end_idx = rep_idx
        max_idx = len(all_sweep_lines_xy) - 1
        start_idx = max(0, min(max_idx, int(start_idx)))
        end_idx = max(0, min(max_idx, int(end_idx)))
        merged_coords: List[Dict[str, Any]] = []
        if start_idx <= end_idx:
            sweep_indices = range(start_idx, end_idx + 1)
        else:
            sweep_indices = range(start_idx, end_idx - 1, -1)
        for sweep_idx in sweep_indices:
            sweep_rows = _line_three_point_xy(
                _dedupe_xy_rows(list(all_sweep_lines_xy[sweep_idx] or []), eps_m=0.5)
            )
            if len(sweep_rows) < 2:
                continue
            merged_coords.extend(
                _xy_to_coord_with_dem_altitude(point_xy) for point_xy in sweep_rows
            )
        return merged_coords

    merged_coords_fallback: List[Dict[str, Any]] = []
    for item in group:
        sweep_xy = item.get("sweepXY") if isinstance(item, dict) else []
        sweep_rows = _line_three_point_xy(_dedupe_xy_rows(list(sweep_xy or []), eps_m=0.5))
        if len(sweep_rows) < 2:
            continue
        merged_coords_fallback.extend(
            _xy_to_coord_with_dem_altitude(point_xy) for point_xy in sweep_rows
        )
    return merged_coords_fallback


def _collect_area_group_sweep_coords(
    *,
    group: List[Dict[str, Any]],
    all_sweep_lines_xy: List[List[Tuple[float, float]]],
    previous_rep_sweep_idx: int | None = None,
) -> List[Dict[str, Any]]:
    if not group:
        return []
    indices = [
        int(_to_float(item.get("sweepIndex")) or 0)
        for item in group
        if isinstance(item, dict)
    ]
    if indices and all_sweep_lines_xy:
        rep_idx = int(indices[-1])
        if previous_rep_sweep_idx is None:
            start_idx = int(indices[0])
        else:
            step_sign = 1 if rep_idx >= int(previous_rep_sweep_idx) else -1
            start_idx = int(previous_rep_sweep_idx) + int(step_sign)
        max_idx = len(all_sweep_lines_xy) - 1
        start_idx = max(0, min(max_idx, int(start_idx)))
        rep_idx = max(0, min(max_idx, int(rep_idx)))
        sweep_range = range(start_idx, rep_idx + 1) if start_idx <= rep_idx else range(start_idx, rep_idx - 1, -1)
        merged_coords: List[Dict[str, Any]] = []
        for sweep_idx in sweep_range:
            sweep_rows = _line_three_point_xy(
                _dedupe_xy_rows(list(all_sweep_lines_xy[sweep_idx] or []), eps_m=0.5)
            )
            if len(sweep_rows) < 2:
                continue
            merged_coords.extend(
                _xy_to_coord_with_dem_altitude(point_xy) for point_xy in sweep_rows
            )
        return merged_coords

    merged_coords_fallback: List[Dict[str, Any]] = []
    for item in group:
        sweep_xy = item.get("sweepXY") if isinstance(item, dict) else []
        sweep_rows = _line_three_point_xy(_dedupe_xy_rows(list(sweep_xy or []), eps_m=0.5))
        if len(sweep_rows) < 2:
            continue
        merged_coords_fallback.extend(
            _xy_to_coord_with_dem_altitude(point_xy) for point_xy in sweep_rows
        )
    return merged_coords_fallback


def _estimate_line_search_speed_mps(
    *,
    prev_coord: Dict[str, Any] | None,
    anchor_coord: Dict[str, Any] | None,
    sweep_coords: List[Dict[str, Any]],
    cruise_speed_mps: float,
    fallback_search_speed_mps: float,
    speed_scale: float = 1.0,
    reference_coord: Dict[str, Any] | None = None,
) -> float:
    fallback_speed = max(0.0, float(fallback_search_speed_mps))
    prev_xy = coord_to_xy(prev_coord) if isinstance(prev_coord, dict) else None
    reference_xy = coord_to_xy(reference_coord) if isinstance(reference_coord, dict) else None
    anchor_xy = coord_to_xy(anchor_coord) if isinstance(anchor_coord, dict) else None
    if prev_xy is None or anchor_xy is None or cruise_speed_mps <= 0.0:
        if reference_xy is not None and anchor_xy is not None and cruise_speed_mps > 0.0:
            prev_xy = reference_xy
        else:
            return fallback_speed

    sweep_xy = [
        point_xy
        for point_xy in (coord_to_xy(coord) for coord in sweep_coords)
        if point_xy is not None
    ]
    if not sweep_xy:
        return fallback_speed

    transit_len_m = _distance_xy(prev_xy, anchor_xy)
    if transit_len_m <= 1e-6 and reference_xy is not None and anchor_xy is not None:
        transit_len_m = _distance_xy(reference_xy, anchor_xy)
    if transit_len_m <= 1e-6 and reference_xy is not None:
        transit_len_m = _distance_xy(reference_xy, sweep_xy[0])
    sweep_len_m = _line_length_xy(sweep_xy)
    if transit_len_m <= 1e-6 or sweep_len_m <= 1e-6:
        return fallback_speed

    travel_time_s = float(transit_len_m) / float(cruise_speed_mps)
    if travel_time_s <= 1e-6:
        return fallback_speed
    try:
        effective_scale = max(float(speed_scale), 0.1)
    except Exception:
        effective_scale = 1.0
    return max(0.0, (float(sweep_len_m) / float(travel_time_s)) * float(effective_scale))


def _chunk_xy_path(
    points_xy: List[Tuple[float, float]],
    *,
    max_points: int = 80,
) -> List[List[Tuple[float, float]]]:
    rows = _dedupe_xy_rows(points_xy, eps_m=0.5)
    if len(rows) < 2:
        return []
    max_points = max(2, int(max_points))
    if len(rows) <= max_points:
        return [rows]
    chunks: List[List[Tuple[float, float]]] = []
    start = 0
    while start < len(rows) - 1:
        end = min(len(rows), start + max_points)
        chunk = rows[start:end]
        if len(chunk) >= 2:
            chunks.append(chunk)
        if end >= len(rows):
            break
        start = end - 1
    return chunks


def _sensor_type_from_template(template_path: Dict[str, Any] | None) -> int:
    if not isinstance(template_path, dict):
        return 1
    for waypoint in template_path.get("waypointList") or []:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        try:
            sensor = int(filming.get("sensorType"))
        except Exception:
            sensor = 0
        if sensor > 0:
            return int(sensor)
    return 1


def _template_speed_mps(template_path: Dict[str, Any] | None, default: float) -> float:
    speed = _template_speed_mps_value(template_path)
    if speed is not None:
        return float(speed)
    return float(default)


def _template_speed_mps_value(template_path: Dict[str, Any] | None) -> float | None:
    if not isinstance(template_path, dict):
        return None
    for waypoint in template_path.get("waypointList") or []:
        if not isinstance(waypoint, dict):
            continue
        speed = _to_float(waypoint.get("speed"))
        if speed is not None and speed > 0.0:
            return float(speed)
    return None


def _template_fov_deg(template_path: Dict[str, Any] | None, default: float) -> float:
    if not isinstance(template_path, dict):
        return float(default)
    for waypoint in template_path.get("waypointList") or []:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        fov = _to_float(filming.get("fieldOfView"))
        if fov is not None and fov > 0.0:
            return float(fov)
    return float(default)


def _search_speed_mps(path_row: Dict[str, Any], template_path: Dict[str, Any] | None) -> float:
    resolved_vel_kmh = _to_float(path_row.get("resolvedVelMps"))
    if resolved_vel_kmh is not None and resolved_vel_kmh > 0.0:
        return float(resolved_vel_kmh) / 3.6
    if isinstance(template_path, dict):
        for waypoint in template_path.get("waypointList") or []:
            if not isinstance(waypoint, dict):
                continue
            filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
            line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
            speed = _to_float(line_search.get("searchSpeed"))
            if speed is not None and speed > 0.0:
                return float(speed)
    return 30.0


def _make_hold_waypoint(
    *,
    coordinate: Dict[str, Any],
    speed_mps: float,
    sensor_type: int,
    field_of_view_deg: float,
    orientation_coordinate: Dict[str, Any] | None = None,
    waypoint_pass_type: int = 1,
    flyover_dubins_prefix: bool = False,
    include_filming: bool = True,
) -> Dict[str, Any]:
    waypoint = {
        "waypointID": 0,
        "coordinate": {
            "latitude": float(coordinate["latitude"]),
            "longitude": float(coordinate["longitude"]),
            "altitude": int(round(float(coordinate.get("altitude", 0.0) or 0.0))),
        },
        "speed": float(speed_mps),
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": int(waypoint_pass_type),
        "isDone": False,
    }
    if include_filming:
        filming: Dict[str, Any] = {
            "fieldOfView": float(field_of_view_deg),
            "sensorType": int(sensor_type),
            "operationMode": 1 if isinstance(orientation_coordinate, dict) else 4,
        }
        if isinstance(orientation_coordinate, dict):
            orientation_target = _coord_with_dem_altitude(orientation_coordinate)
            filming["coordinateOrientation"] = {
                "coordinate": {
                    "latitude": float(orientation_target["latitude"]),
                    "longitude": float(orientation_target["longitude"]),
                    "altitude": int(round(float(orientation_target.get("altitude", 0.0) or 0.0))),
                }
            }
        else:
            filming["aircraftFixed"] = {
                "gimbalPitch": -90.0,
                "gimbalYaw": 0.0,
            }
        waypoint["filmingProperty"] = filming
    if flyover_dubins_prefix:
        waypoint["_flyover_dubins_prefix"] = True
    return waypoint


def _strip_filming_properties(waypoints: List[Dict[str, Any]]) -> None:
    for waypoint in waypoints:
        if isinstance(waypoint, dict):
            waypoint.pop("filmingProperty", None)
            waypoint.pop("FilmingProperty", None)


def _make_line_search_waypoint(
    *,
    coordinate: Dict[str, Any],
    sweep_coords: List[Dict[str, Any]],
    transit_speed_mps: float,
    search_speed_mps: float,
    sensor_type: int,
    field_of_view_deg: float,
    waypoint_pass_type: int = 1,
) -> Dict[str, Any]:
    interpolation_points = _next_collab_sweep_points_per_leg()
    if _next_collab_auto_sweep_points():
        interpolation_points = max(2, min(9, len(sweep_coords)))
    return {
        "waypointID": 0,
        "coordinate": {
            "latitude": float(coordinate["latitude"]),
            "longitude": float(coordinate["longitude"]),
            "altitude": int(round(float(coordinate.get("altitude", 0.0) or 0.0))),
        },
        "speed": float(transit_speed_mps),
        "eta": 0,
        "ecf": 0.0,
        "nextWaypointID": 0,
        "waypointPassType": int(waypoint_pass_type),
        "filmingProperty": {
            "fieldOfView": float(field_of_view_deg),
            "sensorType": int(sensor_type),
            "operationMode": 2,
                "lineSearch": {
                    "coordinateList": list(sweep_coords),
                    "searchSpeed": float(search_speed_mps),
                    "interpolationPoints": int(interpolation_points),
                },
            },
        "isDone": False,
    }


def _set_source_field(payload: Dict[str, Any], source: str) -> None:
    if "Source" in payload or "source" not in payload:
        payload["Source"] = str(payload.get("Source") or payload.get("source") or source)
    else:
        payload["source"] = str(payload.get("source") or payload.get("Source") or source)


def _recompute_waypoint_timeline(
    waypoints: List[Dict[str, Any]],
    *,
    default_speed_mps: float,
) -> None:
    if not waypoints:
        return

    def _line_search_duration_s(waypoint: Dict[str, Any]) -> float:
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        sweep_xy = []
        for coord in line_search.get("coordinateList") or []:
            point_xy = coord_to_xy(coord)
            if point_xy is not None:
                sweep_xy.append(point_xy)
        search_speed = _to_float(line_search.get("searchSpeed")) or default_speed_mps
        if len(sweep_xy) < 2 or search_speed <= 0.0:
            return 0.0
        return float(_line_length_xy(sweep_xy)) / float(search_speed)

    def _loiter_duration_s(waypoint: Dict[str, Any]) -> float:
        if int(_to_float(waypoint.get("waypointPassType")) or 0) != PASS_LOITER:
            return 0.0
        loiter = waypoint.get("loiter") if isinstance(waypoint.get("loiter"), dict) else None
        if loiter is None:
            loiter = waypoint.get("loiterProperty") if isinstance(waypoint.get("loiterProperty"), dict) else None
        duration = _to_float(loiter.get("time")) if isinstance(loiter, dict) else None
        if duration is None or duration <= 0.0:
            duration = 30.0
        return float(duration)

    cumulative_sec = 0.0
    for idx, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            continue
        if idx > 0:
            prev_waypoint = waypoints[idx - 1] if isinstance(waypoints[idx - 1], dict) else {}
            prev_xy = coord_to_xy(prev_waypoint.get("coordinate"))
            current_xy = coord_to_xy(waypoint.get("coordinate"))
            leg_speed = _to_float(waypoint.get("speed")) or _to_float(prev_waypoint.get("speed")) or default_speed_mps
            if prev_xy is not None and current_xy is not None and leg_speed > 0.0:
                cumulative_sec += _distance_xy(prev_xy, current_xy) / float(leg_speed)
        cumulative_sec += _loiter_duration_s(waypoint)
        cumulative_sec += _line_search_duration_s(waypoint)
        waypoint["eta"] = int(round(cumulative_sec))
        waypoint["ecf"] = 0.0
        next_waypoint = waypoints[idx + 1] if idx < len(waypoints) - 1 and isinstance(waypoints[idx + 1], dict) else {}
        waypoint["nextWaypointID"] = int(next_waypoint.get("waypointID", 0) or 0)
    total_sec = max(cumulative_sec, 1.0)
    for idx, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            continue
        eta_sec = _to_float(waypoint.get("eta")) or 0.0
        if idx >= len(waypoints) - 1:
            waypoint["ecf"] = 1.0
        else:
            waypoint["ecf"] = max(0.0, min(1.0, float(eta_sec) / float(total_sec)))


def _prepend_entry_waypoint(
    flight_plan: Dict[str, Any],
    *,
    entry_coord: Dict[str, Any] | None,
    speed_mps: float,
    sensor_type: int,
    field_of_view_deg: float,
) -> None:
    if not isinstance(entry_coord, dict):
        return
    waypoints = flight_plan.get("waypointList")
    if not isinstance(waypoints, list) or not waypoints:
        return
    first_waypoint = waypoints[0] if isinstance(waypoints[0], dict) else {}
    first_filming = first_waypoint.get("filmingProperty") if isinstance(first_waypoint.get("filmingProperty"), dict) else {}
    line_search = first_filming.get("lineSearch") if isinstance(first_filming.get("lineSearch"), dict) else {}
    orientation_target = None
    if isinstance(line_search.get("coordinateList"), list) and line_search.get("coordinateList"):
        first_coord = line_search["coordinateList"][0]
        if isinstance(first_coord, dict):
            orientation_target = {
                "latitude": float(first_coord["latitude"]),
                "longitude": float(first_coord["longitude"]),
                "altitude": _altitude_int(first_coord.get("altitude", 0.0)),
            }
    if orientation_target is None:
        coord_orientation = (
            first_filming.get("coordinateOrientation")
            if isinstance(first_filming.get("coordinateOrientation"), dict)
            else {}
        )
        target_coord = (
            coord_orientation.get("coordinate")
            if isinstance(coord_orientation.get("coordinate"), dict)
            else None
        )
        if isinstance(target_coord, dict):
            orientation_target = {
                "latitude": float(target_coord["latitude"]),
                "longitude": float(target_coord["longitude"]),
                "altitude": _altitude_int(target_coord.get("altitude", 0.0)),
            }
    if orientation_target is None and isinstance(first_waypoint.get("coordinate"), dict):
        first_coord = first_waypoint["coordinate"]
        orientation_target = {
            "latitude": float(first_coord["latitude"]),
            "longitude": float(first_coord["longitude"]),
            "altitude": _altitude_int(first_coord.get("altitude", 0.0)),
        }
    entry_waypoint = _make_hold_waypoint(
        coordinate=entry_coord,
        speed_mps=float(speed_mps),
        sensor_type=int(sensor_type),
        field_of_view_deg=float(field_of_view_deg),
        orientation_coordinate=orientation_target,
        waypoint_pass_type=1,
    )
    waypoints.insert(0, entry_waypoint)


def build_mission_info_from_planned_row(
    path_row: Dict[str, Any],
    *,
    template_info: Dict[str, Any],
    fallback_polygon_coords: List[Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    info = deepcopy(template_info or {})
    aircraft_id = int(_to_float(path_row.get("aircraftID")) or 0)
    resolved_fov_deg = _to_float(path_row.get("resolvedFovDeg"))
    resolved_vel_kmh = _to_float(path_row.get("resolvedVelMps"))
    resolved_sep_m = _to_float(path_row.get("sepCandM")) or _to_float(path_row.get("dbSepM"))
    bearing_deg = _to_float(path_row.get("bearingDeg"))
    manual_fov_active = _runtime_manual_fov_active()
    if resolved_fov_deg is not None and resolved_fov_deg > 0.0 and not manual_fov_active:
        info["FOV"] = float(resolved_fov_deg)
    if resolved_vel_kmh is not None and resolved_vel_kmh > 0.0:
        info["SPEED"] = float(resolved_vel_kmh)
    if resolved_sep_m is not None and resolved_sep_m > 0.0:
        info["SEP"] = float(resolved_sep_m)
    if bearing_deg is not None:
        info["BEARING"] = float(bearing_deg)
        info.setdefault("MOVE_BEARING", float(bearing_deg))
    info.setdefault("autoZoomIn", True)

    polygon_xy = _xy_rows(path_row.get("partPolygonXY"))
    polygon_coords = [_xy_to_coord(point_xy) for point_xy in polygon_xy] if polygon_xy else list(fallback_polygon_coords or [])
    if polygon_coords:
        mission_type, pattern_type = _resolve_generated_mission_types(
            info,
            geometry_kind="area",
        )
        info["individualMissionType"] = int(mission_type)
        info["patternType"] = int(pattern_type)
        info["areaList"] = [{"isHole": False, "coordinateList": polygon_coords}]
        info["coordinateList"] = []
        info.pop("lineList", None)
        if manual_fov_active:
            fov_key = "area_nadir_fov_deg" if int(pattern_type) == 3 else "area_custom_fov_deg"
            info["FOV"] = _runtime_manual_fov_value(fov_key, float(resolved_fov_deg or 10.0))
        _apply_altitude_to_mission_info_inplace(info, aircraft_id=aircraft_id)
        return info

    center_line_xy = _dedupe_xy_rows(_xy_rows(path_row.get("centerLineXY")), eps_m=0.5)
    sweep_lines = [
        _xy_rows(points_xy)
        for points_xy in (path_row.get("sweepLineListXY") or [])
        if isinstance(points_xy, list)
    ]
    line_coords: List[Dict[str, Any]] = []
    if len(center_line_xy) >= 2:
        line_coords = [_xy_to_coord(point_xy) for point_xy in center_line_xy]
    elif sweep_lines:
        for line_xy in sweep_lines:
            if len(line_xy) < 2:
                continue
            line_coords.extend([_xy_to_coord(point_xy) for point_xy in _line_three_point_xy(line_xy)])
    else:
        start_xy = path_row.get("waypointStartXY") or path_row.get("targetXY")
        end_xy = path_row.get("waypointEndXY") or path_row.get("targetFaceXY")
        if isinstance(start_xy, (tuple, list)) and isinstance(end_xy, (tuple, list)):
            line_coords = [_xy_to_coord(point_xy) for point_xy in _line_three_point_xy([
                (float(start_xy[0]), float(start_xy[1])),
                (float(end_xy[0]), float(end_xy[1])),
            ])]

    if line_coords:
        mission_type, pattern_type = _resolve_generated_mission_types(
            info,
            geometry_kind="line",
        )
        info["individualMissionType"] = int(mission_type)
        info["patternType"] = int(pattern_type)
        width_m = _to_float(path_row.get("partWidthM")) or 1.0
        info["lineList"] = [{"width": float(width_m), "coordinateList": line_coords}]
        source_width_m = _to_float(path_row.get("sourceLineWidthM"))
        if source_width_m is not None and source_width_m > 0.0:
            info["sourceLineWidthM"] = float(source_width_m)
        source_coordinate_list = path_row.get("sourceCoordinateList")
        if isinstance(source_coordinate_list, list) and len(source_coordinate_list) >= 2:
            info["sourceCoordinateList"] = deepcopy(source_coordinate_list)
        if "coordinateList" in info or info.get("individualMissionType") in (5, 7):
            info["coordinateList"] = list(line_coords)
        if "areaList" in info and not polygon_coords:
            info.pop("areaList", None)
        if manual_fov_active:
            info["FOV"] = _runtime_manual_fov_value("line_custom_fov_deg", float(resolved_fov_deg or 10.0))
        _apply_altitude_to_mission_info_inplace(info, aircraft_id=aircraft_id)
    return info


def _formation_route_coords_from_mission_info(
    mission_info: Dict[str, Any] | None,
) -> List[Dict[str, Any]]:
    info = mission_info if isinstance(mission_info, dict) else {}
    coords = [
        coord
        for coord in (info.get("coordinateList") or [])
        if isinstance(coord, dict)
    ]
    if coords:
        return deepcopy(coords)
    for line in info.get("lineList") or []:
        if not isinstance(line, dict):
            continue
        line_coords = [
            coord
            for coord in (line.get("coordinateList") or [])
            if isinstance(coord, dict)
        ]
        if line_coords:
            return deepcopy(line_coords)
    return []


def build_formation_flight_path_from_template(
    *,
    template_path: Dict[str, Any] | None,
    mission_info: Dict[str, Any] | None,
    individual_mission_id: int,
    path_id: int,
    aircraft_id: int,
    leader_aircraft_id: int,
    entry_coord: Dict[str, Any] | None,
    timestamp_ms: int,
    source: str = "MMR",
) -> Dict[str, Any]:
    mission_info_dict = mission_info if isinstance(mission_info, dict) else {}
    default_speed_mps = 40.0
    mission_speed_kmh = _to_float(mission_info_dict.get("SPEED"))
    if mission_speed_kmh is not None and mission_speed_kmh > 0.0:
        default_speed_mps = max(1.0, float(mission_speed_kmh) / 3.6)
    speed_mps = _template_speed_mps(template_path, default_speed_mps)
    sensor_type = _sensor_type_from_template(template_path)
    mission_fov_deg = _to_float(mission_info_dict.get("FOV"))
    default_fov_deg = (
        float(mission_fov_deg)
        if mission_fov_deg is not None and mission_fov_deg > 0.0
        else _template_fov_deg(template_path, 10.0)
    )
    field_of_view_deg = _runtime_manual_fov_value(
        _fov_key_for_mission_info(mission_info_dict),
        float(default_fov_deg),
    )
    _, _, altitude_fn = _mission_altitude_policy(
        aircraft_id=int(aircraft_id),
        mission_info=mission_info_dict,
    )

    route_coords = _formation_route_coords_from_mission_info(mission_info_dict)
    waypoint_coords: List[Dict[str, Any]] = [dict(coord) for coord in route_coords]

    waypoints: List[Dict[str, Any]] = []
    for coord in waypoint_coords:
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        waypoints.append(
            _make_hold_waypoint(
                coordinate=_coord_with_altitude(coord, altitude_fn),
                speed_mps=float(speed_mps),
                sensor_type=int(sensor_type),
                field_of_view_deg=float(field_of_view_deg),
                orientation_coordinate=None,
                waypoint_pass_type=1,
                include_filming=False,
            )
        )

    flight_path = deepcopy(template_path) if isinstance(template_path, dict) else {}
    flight_path["timestamp"] = int(timestamp_ms)
    flight_path["pathID"] = int(path_id)
    flight_path["aircraftID"] = int(aircraft_id)
    flight_path["individualMissionID"] = int(individual_mission_id)
    flight_path["isFormationFlight"] = True
    formation_info = flight_path.get("formationInfo") if isinstance(flight_path.get("formationInfo"), dict) else {}
    if not formation_info:
        formation_info = {
            "leaderAircraftID": int(leader_aircraft_id),
            "formation": {
                "dX": 0,
                "dY": 0,
                "dZ": 0,
            },
        }
    else:
        formation_info = deepcopy(formation_info)
        formation_info["leaderAircraftID"] = int(
            _to_int(formation_info.get("leaderAircraftID")) or int(leader_aircraft_id)
        )
        if not isinstance(formation_info.get("formation"), dict):
            formation_info["formation"] = {
                "dX": 0,
                "dY": 0,
                "dZ": 0,
            }
    flight_path["formationInfo"] = formation_info
    _set_source_field(flight_path, str(source))
    flight_path["waypointList"] = waypoints

    final_waypoints = flight_path.get("waypointList") if isinstance(flight_path.get("waypointList"), list) else []
    for waypoint in final_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    reassign_unique_waypoint_ids_inplace(final_waypoints)
    _apply_legacy_altitude_profile_to_waypoints(
        final_waypoints,
        aircraft_id=int(aircraft_id),
        mission_info=mission_info_dict,
    )
    _enforce_waypoint_altitude_rate_limit_inplace(
        final_waypoints,
        default_speed_mps=float(speed_mps),
    )
    _stabilize_entry_transition_altitude_inplace(
        final_waypoints,
        entry_coord=entry_coord,
        default_speed_mps=float(speed_mps),
    )
    normalize_filming_target_altitudes_in_waypoints(final_waypoints)
    _enforce_filming_target_altitude_floor_inplace(final_waypoints)
    _enforce_waypoint_altitude_rate_limit_inplace(
        final_waypoints,
        default_speed_mps=float(speed_mps),
    )
    _stabilize_entry_transition_altitude_inplace(
        final_waypoints,
        entry_coord=entry_coord,
        default_speed_mps=float(speed_mps),
    )
    reassign_unique_waypoint_ids_inplace(final_waypoints)
    _recompute_waypoint_timeline(final_waypoints, default_speed_mps=float(speed_mps))
    _strip_filming_properties(final_waypoints)
    flight_path["waypointList"] = final_waypoints
    if "lahWaypointList" in flight_path:
        flight_path["lahWaypointList"] = deepcopy(final_waypoints)
    return flight_path


def build_flight_path_from_planned_row(
    path_row: Dict[str, Any],
    *,
    template_path: Dict[str, Any] | None,
    mission_info: Dict[str, Any] | None,
    individual_mission_id: int,
    path_id: int,
    aircraft_id: int,
    entry_coord: Dict[str, Any] | None,
    timestamp_ms: int,
    source: str = "MMR",
) -> Dict[str, Any]:
    search_speed_mps = max(1.0, _search_speed_mps(path_row, template_path))
    template_transit_speed_mps = _template_speed_mps_value(template_path)
    transit_speed_mps = max(
        1.0,
        float(template_transit_speed_mps) if template_transit_speed_mps is not None else float(search_speed_mps),
    )
    geometry_search_speed_scale = (
        _mission_search_speed_weight(mission_info)
        if template_transit_speed_mps is not None
        else 1.0
    )
    sensor_type = _sensor_type_from_template(template_path)
    _, _, altitude_fn = _mission_altitude_policy(
        aircraft_id=int(aircraft_id),
        mission_info=mission_info,
    )

    waypoints: List[Dict[str, Any]] = []
    mission_info_dict = mission_info if isinstance(mission_info, dict) else {}
    manual_fov_active = _runtime_manual_fov_active()
    first_line_fov_boost_active = not manual_fov_active
    mission_fov_deg = _to_float(mission_info_dict.get("FOV"))
    resolved_fov_deg = _to_float(path_row.get("resolvedFovDeg"))
    resolved_base_fov_deg = _to_float(path_row.get("resolvedBaseFovDeg"))
    default_fov_deg = (
        resolved_fov_deg
        or (float(mission_fov_deg) if mission_fov_deg is not None and mission_fov_deg > 0.0 else None)
        or _template_fov_deg(template_path, 10.0)
    )
    first_line_base_fov_deg = (
        float(resolved_base_fov_deg)
        if resolved_base_fov_deg is not None and resolved_base_fov_deg > 0.0
        else float(default_fov_deg)
    )
    field_of_view_deg = _runtime_manual_fov_value(
        _fov_key_for_mission_info(mission_info_dict),
        float(default_fov_deg),
    )
    scan_lines_xy = [
        _dedupe_xy_rows(_xy_rows(points_xy), eps_m=0.5)
        for points_xy in (path_row.get("sweepLineListXY") or [])
        if isinstance(points_xy, list)
    ]
    scan_lines_xy = [points_xy for points_xy in scan_lines_xy if len(points_xy) >= 2]

    is_line_mission = bool(mission_info_dict.get("lineList")) and not bool(mission_info_dict.get("areaList"))
    entry_xy: Tuple[float, float] | None = None
    if isinstance(entry_coord, dict):
        raw_entry_xy = coord_to_xy(entry_coord)
        if raw_entry_xy is not None:
            entry_xy = (float(raw_entry_xy[0]), float(raw_entry_xy[1]))
    transition_points_xy: List[Tuple[float, float]] = []
    if is_line_mission:
        # Use only the planner-computed T' as the line pre-search waypoint.
        # The tangent/current-entry points are not emitted as standalone WPs.
        path_row_transition_points = (
            (path_row.get("entryTPrimeXY"),)
            if isinstance(path_row.get("entryTPrimeXY"), (tuple, list))
            else ()
        )
    else:
        path_row_transition_points = (
            path_row.get("entryTPrimeXY"),
            path_row.get("waypointStartXY"),
        )
    # Line rejoin paths can use these transition waypoints when they feed
    # naturally into the first sweep. If they would immediately reverse into
    # the first sweep, we fall back to the older direct-entry behavior below.
    for raw_xy in path_row_transition_points:
        if not (isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2):
            continue
        point_xy = (float(raw_xy[0]), float(raw_xy[1]))
        if transition_points_xy and _distance_xy(transition_points_xy[-1], point_xy) <= 3.0:
            continue
        transition_points_xy.append(point_xy)

    line_sweep_items = _line_sweep_items_xy(path_row)
    flattened_sweep_xy = _flatten_sweep_lines_xy(scan_lines_xy)
    first_sweep_point_xy = None
    first_sweep_xy_for_guard: List[Tuple[float, float]] = []
    first_anchor_xy: Tuple[float, float] | None = None
    if line_sweep_items:
        first_item = line_sweep_items[0] if isinstance(line_sweep_items[0], dict) else {}
        first_anchor_raw = first_item.get("anchorXY")
        if isinstance(first_anchor_raw, (tuple, list)) and len(first_anchor_raw) >= 2:
            first_anchor_xy = (float(first_anchor_raw[0]), float(first_anchor_raw[1]))
        first_sweep_xy = first_item.get("sweepXY") if isinstance(first_item, dict) else []
        if isinstance(first_sweep_xy, list) and first_sweep_xy:
            first_sweep_xy_for_guard = [
                (float(row[0]), float(row[1]))
                for row in first_sweep_xy
                if isinstance(row, (tuple, list)) and len(row) >= 2
            ]
            first_sweep_point_xy = first_sweep_xy[0]
    if first_sweep_point_xy is None:
        first_sweep_point_xy = flattened_sweep_xy[0] if flattened_sweep_xy else None
    if not first_sweep_xy_for_guard and scan_lines_xy:
        first_sweep_xy_for_guard = list(scan_lines_xy[0])
    if is_line_mission and _line_transition_backtracks(
        entry_xy=entry_xy,
        transition_points_xy=transition_points_xy,
        first_sweep_point_xy=first_sweep_point_xy,
    ):
        # Backup behavior was better here: if the transition ingress would
        # immediately reverse into the first sweep, skip the extra line
        # transition waypoints instead of creating a loop-back entry.
        transition_points_xy = []
    elif (
        is_line_mission
        and len(transition_points_xy) == 1
        and entry_xy is not None
        and first_sweep_point_xy is not None
        and _distance_xy(transition_points_xy[0], entry_xy) <= 3.0
    ):
        # A lone entry hold waypoint only adds a redundant first WP like 331
        # before the actual line-search anchor. In that case, start directly
        # from the first search waypoint instead of emitting the extra entry WP.
        transition_points_xy = []
    sweep_orientation_coord = (
        _xy_to_coord_with_dem_altitude(first_sweep_point_xy)
        if first_sweep_point_xy is not None
        else None
    )
    # Keep post-attack/prior rejoin line plans in the older direct-to-search
    # shape: do not synthesize a separate ingress guard waypoint.
    line_ingress_guard_xy = None

    for point_xy in transition_points_xy:
        waypoints.append(
            _make_hold_waypoint(
                coordinate=_xy_to_coord_with_altitude(point_xy, altitude_fn),
                speed_mps=float(transit_speed_mps),
                sensor_type=int(sensor_type),
                field_of_view_deg=float(field_of_view_deg),
                orientation_coordinate=sweep_orientation_coord,
                waypoint_pass_type=int(PASS_FLYBY),
                flyover_dubins_prefix=True,
            )
        )

    if line_ingress_guard_xy is not None:
        should_add_guard = True
        if waypoints and isinstance(waypoints[-1], dict):
            last_xy = _waypoint_coordinate_xy(waypoints[-1])
            if last_xy is not None and _distance_xy(last_xy, line_ingress_guard_xy) <= 3.0:
                should_add_guard = False
        if should_add_guard:
            waypoints.append(
                _make_hold_waypoint(
                    coordinate=_xy_to_coord_with_altitude(line_ingress_guard_xy, altitude_fn),
                    speed_mps=float(transit_speed_mps),
                    sensor_type=int(sensor_type),
                    field_of_view_deg=float(field_of_view_deg),
                    orientation_coordinate=sweep_orientation_coord,
                    waypoint_pass_type=int(PASS_FLYBY),
                    flyover_dubins_prefix=True,
                )
            )

    end_xy = path_row.get("waypointEndXY")
    area_sweep_items: List[Dict[str, Any]] = []
    if not is_line_mission and scan_lines_xy:
        area_sweep_items = _area_sweep_items_xy(path_row, scan_lines_xy)
    if area_sweep_items:
        prev_coord = None
        prev_rep_sweep_idx: int | None = None
        if waypoints and isinstance(waypoints[-1], dict):
            prev_coord = waypoints[-1].get("coordinate")
        if not isinstance(prev_coord, dict):
            prev_coord = entry_coord
        grouped_area_sweeps = _group_area_sweep_items_by_spacing(
            area_sweep_items,
            spacing_m=float(_next_collab_area_route_wp_spacing_m()),
            merge_short_tail=True,
        )
        for group in grouped_area_sweeps:
            rep_item = group[-1] if group else {}
            anchor_xy = rep_item.get("anchorXY")
            if not (isinstance(anchor_xy, (tuple, list)) and len(anchor_xy) >= 2):
                continue
            merged_sweep_coords = _collect_area_group_sweep_coords(
                group=group,
                all_sweep_lines_xy=scan_lines_xy,
                previous_rep_sweep_idx=prev_rep_sweep_idx,
            )
            if len(merged_sweep_coords) < 2:
                continue
            anchor_coord = _xy_to_coord_with_altitude((float(anchor_xy[0]), float(anchor_xy[1])), altitude_fn)
            line_search_speed_mps = _estimate_line_search_speed_mps(
                prev_coord=prev_coord,
                anchor_coord=anchor_coord,
                sweep_coords=merged_sweep_coords,
                cruise_speed_mps=float(transit_speed_mps),
                fallback_search_speed_mps=float(search_speed_mps),
                speed_scale=float(geometry_search_speed_scale),
            )
            waypoints.append(
                _make_line_search_waypoint(
                    coordinate=anchor_coord,
                    sweep_coords=merged_sweep_coords,
                    transit_speed_mps=float(transit_speed_mps),
                    search_speed_mps=float(line_search_speed_mps),
                    sensor_type=int(sensor_type),
                    field_of_view_deg=float(field_of_view_deg),
                    waypoint_pass_type=3,
                )
            )
            prev_coord = anchor_coord
            rep_sweep_idx = _to_float(rep_item.get("sweepIndex"))
            if rep_sweep_idx is not None:
                prev_rep_sweep_idx = int(rep_sweep_idx)
    elif line_sweep_items:
        prev_coord = None
        prev_rep_sweep_idx: int | None = None
        line_search_wp_index = 0
        if waypoints and isinstance(waypoints[-1], dict):
            prev_coord = waypoints[-1].get("coordinate")
        if not isinstance(prev_coord, dict):
            prev_coord = entry_coord
        # The planner already down-selects lineSweepItemsXY using the route-WP spacing.
        # Grouping them again here can collapse additional sweeps for only some UAVs.
        grouped_line_sweeps = [[dict(item)] for item in line_sweep_items if isinstance(item, dict)]
        for group in grouped_line_sweeps:
            rep_item = group[-1] if group else {}
            anchor_xy = rep_item.get("anchorXY")
            if not (isinstance(anchor_xy, (tuple, list)) and len(anchor_xy) >= 2):
                continue
            if line_search_wp_index == 0 and line_ingress_guard_xy is not None:
                anchor_xy = line_ingress_guard_xy
            merged_sweep_coords = _collect_group_sweep_coords(
                group=group,
                all_sweep_lines_xy=scan_lines_xy,
                previous_rep_sweep_idx=prev_rep_sweep_idx,
            )
            if len(merged_sweep_coords) < 2:
                continue
            anchor_coord = _xy_to_coord_with_altitude((float(anchor_xy[0]), float(anchor_xy[1])), altitude_fn)
            speed_reference_coord = (
                entry_coord
                if is_line_mission and line_search_wp_index == 0 and isinstance(entry_coord, dict)
                else None
            )
            line_search_speed_mps = _estimate_line_search_speed_mps(
                prev_coord=speed_reference_coord if speed_reference_coord is not None else prev_coord,
                anchor_coord=anchor_coord,
                sweep_coords=merged_sweep_coords,
                cruise_speed_mps=float(transit_speed_mps),
                fallback_search_speed_mps=float(search_speed_mps),
                speed_scale=float(geometry_search_speed_scale),
                reference_coord=speed_reference_coord,
            )
            waypoint_fov_deg = (
                _next_collab_first_line_search_fov_deg(float(first_line_base_fov_deg))
                if is_line_mission and line_search_wp_index == 0 and first_line_fov_boost_active
                else float(field_of_view_deg)
            )
            waypoints.append(
                _make_line_search_waypoint(
                    coordinate=anchor_coord,
                    sweep_coords=merged_sweep_coords,
                    transit_speed_mps=float(transit_speed_mps),
                    search_speed_mps=float(line_search_speed_mps),
                    sensor_type=int(sensor_type),
                    field_of_view_deg=float(waypoint_fov_deg),
                    waypoint_pass_type=3,
                )
            )
            line_search_wp_index += 1
            prev_coord = anchor_coord
            rep_sweep_idx = _to_float(rep_item.get("sweepIndex"))
            if rep_sweep_idx is not None:
                prev_rep_sweep_idx = int(rep_sweep_idx)
    elif flattened_sweep_xy and isinstance(end_xy, (tuple, list)) and len(end_xy) >= 2:
        line_search_anchor_xy = line_ingress_guard_xy if line_ingress_guard_xy is not None else (float(end_xy[0]), float(end_xy[1]))
        line_search_anchor_coord = _xy_to_coord_with_altitude(line_search_anchor_xy, altitude_fn)
        sweep_coords = [_xy_to_coord_with_dem_altitude(point_xy) for point_xy in flattened_sweep_xy]
        prev_coord = None
        if waypoints and isinstance(waypoints[-1], dict):
            prev_coord = waypoints[-1].get("coordinate")
        if not isinstance(prev_coord, dict):
            prev_coord = entry_coord
        speed_reference_coord = entry_coord if is_line_mission and isinstance(entry_coord, dict) else None
        line_search_speed_mps = _estimate_line_search_speed_mps(
            prev_coord=speed_reference_coord if speed_reference_coord is not None else prev_coord,
            anchor_coord=line_search_anchor_coord,
            sweep_coords=sweep_coords,
            cruise_speed_mps=float(transit_speed_mps),
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
            reference_coord=speed_reference_coord,
        )
        waypoint_fov_deg = (
            _next_collab_first_line_search_fov_deg(float(first_line_base_fov_deg))
            if is_line_mission and first_line_fov_boost_active
            else float(field_of_view_deg)
        )
        waypoints.append(
            _make_line_search_waypoint(
                coordinate=line_search_anchor_coord,
                sweep_coords=sweep_coords,
                transit_speed_mps=float(transit_speed_mps),
                search_speed_mps=float(line_search_speed_mps),
                sensor_type=int(sensor_type),
                field_of_view_deg=float(waypoint_fov_deg),
                waypoint_pass_type=3,
            )
        )
    elif isinstance(end_xy, (tuple, list)) and len(end_xy) >= 2:
        hold_xy = line_ingress_guard_xy if line_ingress_guard_xy is not None else (float(end_xy[0]), float(end_xy[1]))
        waypoints.append(
            _make_hold_waypoint(
                coordinate=_xy_to_coord_with_altitude(hold_xy, altitude_fn),
                speed_mps=float(transit_speed_mps),
                sensor_type=int(sensor_type),
                field_of_view_deg=float(field_of_view_deg),
                orientation_coordinate=sweep_orientation_coord,
                waypoint_pass_type=3,
            )
        )

    if not waypoints and isinstance(path_row.get("targetXY"), (tuple, list)):
        waypoints.append(
            _make_hold_waypoint(
                coordinate=_xy_to_coord_with_altitude(path_row["targetXY"], altitude_fn),
                speed_mps=float(transit_speed_mps),
                sensor_type=int(sensor_type),
                field_of_view_deg=float(field_of_view_deg),
                orientation_coordinate=None,
                waypoint_pass_type=3,
            )
        )

    flight_path = deepcopy(template_path) if isinstance(template_path, dict) else {}
    flight_path["timestamp"] = int(timestamp_ms)
    flight_path["pathID"] = int(path_id)
    flight_path["aircraftID"] = int(aircraft_id)
    flight_path["individualMissionID"] = int(individual_mission_id)
    flight_path["isFormationFlight"] = bool(flight_path.get("isFormationFlight", False))
    _set_source_field(flight_path, str(source))
    flight_path["waypointList"] = waypoints

    final_waypoints = flight_path.get("waypointList") if isinstance(flight_path.get("waypointList"), list) else []
    if is_line_mission:
        _squash_leading_short_line_search_waypoints(final_waypoints)
        _squash_trailing_short_line_search_waypoints(
            final_waypoints,
            spacing_m=float(_to_float(path_row.get("lineRouteWpSpacingM")) or _next_collab_line_route_wp_spacing_m()),
            transit_speed_mps=float(transit_speed_mps),
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
        )
    for waypoint in final_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    reassign_unique_waypoint_ids_inplace(final_waypoints)
    _apply_legacy_altitude_profile_to_waypoints(
        final_waypoints,
        aircraft_id=int(aircraft_id),
        mission_info=mission_info,
    )
    _preserve_first_waypoint_altitude_from_entry(
        final_waypoints,
        entry_coord=entry_coord,
    )
    _enforce_waypoint_altitude_rate_limit_inplace(
        final_waypoints,
        default_speed_mps=float(transit_speed_mps),
    )
    _stabilize_entry_transition_altitude_inplace(
        final_waypoints,
        entry_coord=entry_coord,
        default_speed_mps=float(transit_speed_mps),
    )
    normalize_filming_target_altitudes_in_waypoints(final_waypoints)
    _enforce_filming_target_altitude_floor_inplace(final_waypoints)
    _enforce_waypoint_altitude_rate_limit_inplace(
        final_waypoints,
        default_speed_mps=float(transit_speed_mps),
    )
    _stabilize_entry_transition_altitude_inplace(
        final_waypoints,
        entry_coord=entry_coord,
        default_speed_mps=float(transit_speed_mps),
    )
    reassign_unique_waypoint_ids_inplace(final_waypoints)
    _recompute_waypoint_timeline(final_waypoints, default_speed_mps=float(transit_speed_mps))
    _apply_runtime_flyover_to_waypoints(final_waypoints)
    if bool(flight_path.get("isFormationFlight", False)):
        _strip_filming_properties(final_waypoints)
    flight_path["waypointList"] = final_waypoints
    if "lahWaypointList" in flight_path:
        flight_path["lahWaypointList"] = deepcopy(final_waypoints)
    return flight_path
