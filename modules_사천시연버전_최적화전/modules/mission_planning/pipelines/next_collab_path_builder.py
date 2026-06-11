from __future__ import annotations

import math
import threading
import time
from copy import deepcopy
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from modules.mission_planning.pipelines.mission_path_trim import (
    reassign_unique_waypoint_ids_inplace,
)
from modules.mission_planning.pipelines.line_search_speed_guard import (
    clamp_line_search_speed_mps,
    effective_line_search_transit_m,
)
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    normalize_filming_target_altitudes_in_waypoints,
)
from modules.mission_planning.MissionPlanner.data_def.mission_helpers import terrain_elev, terrain_elev_many
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        MIN_LINE_FOV_DEG,
        apply_runtime_camera_adjusted_fov_deg,
        get_runtime_manual_fov_deg,
        get_runtime_manual_fov_sync_active,
        get_runtime_altitude_layers_m,
        get_runtime_bool,
        get_runtime_float,
        get_runtime_int,
        load_fov_db_rows,
        load_runtime_flyover,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        MIN_LINE_FOV_DEG,
        apply_runtime_camera_adjusted_fov_deg,
        get_runtime_manual_fov_deg,
        get_runtime_manual_fov_sync_active,
        get_runtime_altitude_layers_m,
        get_runtime_bool,
        get_runtime_float,
        get_runtime_int,
        load_fov_db_rows,
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
_DEM_ALT_CACHE_MAX = 200_000
_DEM_ALT_CACHE_LOCK = threading.Lock()
_DEM_ALT_CACHE: Dict[Tuple[float, float], float] = {}
_GROUND_REQUIRED_COORDS_CACHE_MAX = 50_000
_GROUND_REQUIRED_COORDS_CACHE_LOCK = threading.Lock()
_GROUND_REQUIRED_COORDS_CACHE: Dict[Tuple[Tuple[float, float], ...], float] = {}


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


def _clamp_line_fov_deg(value: Any, default: float | None = None) -> float:
    fov_deg = _to_float(value)
    if fov_deg is None or fov_deg <= 0.0:
        fov_deg = _to_float(default)
    if fov_deg is None or fov_deg <= 0.0:
        fov_deg = float(MIN_LINE_FOV_DEG)
    return max(float(MIN_LINE_FOV_DEG), float(fov_deg))


def _fov_db_min_sep_for_fov(fov_deg: Any) -> float:
    target_fov = _to_float(fov_deg)
    if target_fov is None or target_fov <= 0.0:
        return 0.0
    try:
        rows = load_fov_db_rows()
    except Exception:
        rows = []
    if not rows:
        return 0.0

    matches: List[float] = []
    for row in rows:
        row_fov = _to_float(row.get("fov"))
        row_sep = _to_float(row.get("sep"))
        if row_fov is None or row_fov <= 0.0 or row_sep is None or row_sep <= 0.0:
            continue
        if abs(float(row_fov) - float(target_fov)) <= 0.05:
            matches.append(float(row_sep))

    if not matches:
        for row in rows:
            row_fov = _to_float(row.get("fov"))
            row_sep = _to_float(row.get("sep"))
            if row_fov is None or row_fov <= 0.0 or row_sep is None or row_sep <= 0.0:
                continue
            try:
                adjusted_fov = float(
                    apply_runtime_camera_adjusted_fov_deg(
                        float(row_fov),
                        minimum_fov_deg=MIN_LINE_FOV_DEG,
                        context="NEXTCOLLAB PATH OFFSET_DB",
                    )
                )
            except Exception:
                adjusted_fov = float(row_fov)
            if abs(float(adjusted_fov) - float(target_fov)) <= 0.05:
                matches.append(float(row_sep))

    return min(matches) if matches else 0.0


def _route_offset_sep_for_fov(fov_deg: Any, default_sep_m: Any) -> float:
    default_sep = _to_float(default_sep_m) or 0.0
    min_sep = _fov_db_min_sep_for_fov(fov_deg)
    if min_sep > 0.0:
        return float(min_sep)
    return max(float(default_sep), 0.0)


def _clamp_line_waypoint_fov_inplace(waypoints: List[Dict[str, Any]]) -> None:
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else None
        if not isinstance(filming, dict):
            continue
        filming["fieldOfView"] = _clamp_line_fov_deg(filming.get("fieldOfView"))


def _runtime_manual_fov_active() -> bool:
    try:
        return bool(get_runtime_manual_fov_sync_active())
    except Exception:
        return False


def _runtime_manual_fov_value(key: str, default: float) -> float:
    try:
        value = float(get_runtime_manual_fov_deg(key, float(default)))
    except Exception:
        value = float(default)
    if key in {"line_custom_fov_deg", "line_override_fov_deg"}:
        return _clamp_line_fov_deg(value, default)
    return float(value)


def _mission_search_speed_weight(mission_info: Dict[str, Any] | None) -> float:
    info = mission_info if isinstance(mission_info, dict) else {}
    key = "search_speed_weight" if bool(info.get("lineList")) and not bool(info.get("areaList")) else "area_search_speed_weight"
    try:
        value = float(get_runtime_float(key, 1.0))
    except Exception:
        value = 1.0
    return max(value, 0.1)


def _path_row_search_speed_scale_multiplier(path_row: Dict[str, Any] | None) -> float:
    row = path_row if isinstance(path_row, dict) else {}
    raw_value = (
        row.get("searchSpeedScaleMultiplier")
        if row.get("searchSpeedScaleMultiplier") is not None
        else row.get("_searchSpeedScaleMultiplier")
    )
    value = _to_float(raw_value)
    if value is None or value <= 0.0:
        return 1.0
    return max(float(value), 0.1)


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


def _dem_cache_key(lat: float, lon: float) -> Tuple[float, float]:
    # DEM tiles are meter-scale data; 7 decimal places makes near-identical
    # generated route coordinates miss the cache. 6 decimals keeps the terrain
    # sample effectively identical for this planner while avoiding repeated
    # terrain_elev_many calls during attack/next-collab replans.
    return (round(float(lat), 6), round(float(lon), 6))


def _finite_dem_value(value: Any, *, invalid_default: float | None = 0.0) -> float | None:
    try:
        numeric = float(value)
    except Exception:
        return invalid_default
    return float(numeric) if math.isfinite(numeric) else invalid_default


def _dem_altitudes_for_pairs_cached(
    pairs: Iterable[Tuple[float, float]],
    *,
    invalid_default: float | None = 0.0,
) -> List[float | None]:
    normalized_pairs: List[Tuple[float, float]] = [
        (float(lat), float(lon))
        for lat, lon in (pairs or [])
    ]
    if not normalized_pairs:
        return []

    keys: List[Tuple[float, float] | None] = [
        _dem_cache_key(lat, lon) if math.isfinite(lat) and math.isfinite(lon) else None
        for lat, lon in normalized_pairs
    ]
    results: List[float | None] = [None] * len(keys)
    missing_by_key: Dict[Tuple[float, float], List[int]] = {}
    with _DEM_ALT_CACHE_LOCK:
        for idx, key in enumerate(keys):
            if key is None:
                results[idx] = invalid_default
                continue
            cached = _DEM_ALT_CACHE.get(key)
            if cached is None:
                missing_by_key.setdefault(key, []).append(idx)
            else:
                results[idx] = float(cached)

    if missing_by_key:
        missing_keys = list(missing_by_key.keys())
        try:
            missing_values = terrain_elev_many(missing_keys)
            if len(missing_values) != len(missing_keys):
                raise RuntimeError("terrain_elev_many returned unexpected length")
        except Exception:
            missing_values = [
                terrain_elev(float(lat), float(lon))
                for lat, lon in missing_keys
            ]
        updates: Dict[Tuple[float, float], float] = {}
        invalid_updates: set[Tuple[float, float]] = set()
        for key, value in zip(missing_keys, missing_values):
            try:
                numeric_value = float(value)
            except Exception:
                invalid_updates.add(key)
                continue
            if not math.isfinite(numeric_value):
                invalid_updates.add(key)
                continue
            updates[key] = float(numeric_value)
        with _DEM_ALT_CACHE_LOCK:
            if updates:
                if len(_DEM_ALT_CACHE) + len(updates) > _DEM_ALT_CACHE_MAX:
                    _DEM_ALT_CACHE.clear()
                _DEM_ALT_CACHE.update(updates)
        for key, indices in missing_by_key.items():
            value = updates.get(key)
            if value is None and key in invalid_updates:
                for idx in indices:
                    results[idx] = invalid_default
                continue
            value = float(value if value is not None else 0.0)
            for idx in indices:
                results[idx] = value

    return [_finite_dem_value(value, invalid_default=invalid_default) for value in results]


def _xy_to_coord(point_xy: Sequence[float], altitude: float = 0.0) -> Dict[str, float]:
    lat, lon = local_xy_to_llh(float(point_xy[0]), float(point_xy[1]))
    return {
        "latitude": float(lat),
        "longitude": float(lon),
        "altitude": _altitude_int(altitude),
    }


def _dem_alt(lat: float, lon: float) -> int:
    values = _dem_altitudes_for_pairs_cached([(float(lat), float(lon))])
    return int(round(float(values[0] if values else 0.0)))


def prewarm_dem_altitudes_for_path_rows(path_rows: Iterable[Dict[str, Any]]) -> Dict[str, Any]:
    """Populate the DEM cache for generated next-collab path rows before worker threads run."""
    started = time.perf_counter()
    xy_points: List[Tuple[float, float]] = []

    def _collect_xy(value: Any, *, xy_context: bool = False) -> None:
        if isinstance(value, dict):
            for key, child in value.items():
                _collect_xy(child, xy_context=xy_context or str(key).endswith("XY"))
            return
        if not isinstance(value, (list, tuple)):
            return
        if (
            xy_context
            and len(value) >= 2
            and not isinstance(value[0], (list, tuple, dict))
            and not isinstance(value[1], (list, tuple, dict))
        ):
            try:
                xy_points.append((float(value[0]), float(value[1])))
                return
            except Exception:
                pass
        for child in value:
            _collect_xy(child, xy_context=xy_context)

    for row in path_rows or []:
        if isinstance(row, dict):
            _collect_xy(row)

    if not xy_points:
        return {"xyPoints": 0, "uniquePairs": 0, "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3)}

    pairs: List[Tuple[float, float]] = []
    seen: set[Tuple[float, float]] = set()
    for xy in xy_points:
        try:
            coord = _xy_to_coord(xy)
            lat = float(coord["latitude"])
            lon = float(coord["longitude"])
        except Exception:
            continue
        key = _dem_cache_key(lat, lon)
        if key in seen:
            continue
        seen.add(key)
        pairs.append(key)

    if pairs:
        _dem_altitudes_for_pairs_cached(pairs)
    return {
        "xyPoints": len(xy_points),
        "uniquePairs": len(pairs),
        "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
    }


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
    try:
        values = _dem_altitudes_for_pairs_cached(points, invalid_default=None)
    except Exception:
        values = []
    for value in values:
        try:
            numeric = float(value)
        except Exception:
            continue
        if math.isfinite(numeric):
            samples.append(int(round(numeric)))
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

    sample_pairs: List[Tuple[float, float]] = [usable[0]]
    for idx in range(1, len(usable)):
        prev_coord = {"latitude": usable[idx - 1][0], "longitude": usable[idx - 1][1]}
        curr_coord = {"latitude": usable[idx][0], "longitude": usable[idx][1]}
        prev_xy = coord_to_xy(prev_coord)
        curr_xy = coord_to_xy(curr_coord)
        if prev_xy is None or curr_xy is None:
            sample_pairs.append(usable[idx])
            continue
        seg_dist = _distance_xy(prev_xy, curr_xy)
        if seg_dist <= 1e-6:
            sample_pairs.append(usable[idx])
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
            sample_pairs.append((float(lat), float(lon)))
    return _dem_altitudes_for_pairs_cached(sample_pairs)


def _ground_required_coords_cache_key(
    coords: List[Dict[str, Any]],
) -> Tuple[Tuple[float, float], ...] | None:
    rows: List[Tuple[float, float]] = []
    for coord in coords or []:
        if not isinstance(coord, dict):
            return None
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None or not math.isfinite(float(lat)) or not math.isfinite(float(lon)):
            return None
        rows.append((round(float(lat), 6), round(float(lon), 6)))
    if not rows:
        return None
    return tuple(rows)


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
    cache_key = _ground_required_coords_cache_key(coords)
    if cache_key is not None:
        with _GROUND_REQUIRED_COORDS_CACHE_LOCK:
            cached = _GROUND_REQUIRED_COORDS_CACHE.get(cache_key)
        if cached is not None:
            return float(cached)
    samples: List[float] = _sample_ground_profile_along_coords(coords)
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
    result = max(samples)
    if cache_key is not None and math.isfinite(float(result)):
        with _GROUND_REQUIRED_COORDS_CACHE_LOCK:
            if len(_GROUND_REQUIRED_COORDS_CACHE) >= _GROUND_REQUIRED_COORDS_CACHE_MAX:
                _GROUND_REQUIRED_COORDS_CACHE.clear()
            _GROUND_REQUIRED_COORDS_CACHE[cache_key] = float(result)
    return float(result)


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
    if not _is_point_hold_waypoint(first_wp) and not _line_search_coordinate_list(first_wp):
        return
    first_coord = first_wp.get("coordinate") if isinstance(first_wp.get("coordinate"), dict) else None
    if not isinstance(first_coord, dict):
        return

    entry_alt = _to_float(entry_coord.get("altitude"))
    first_alt = _to_float(first_coord.get("altitude"))
    if entry_alt is None or first_alt is None:
        return

    rate_mps = _runtime_uav_climb_rate_mps()
    if float(first_alt) > float(entry_alt):
        first_alt = math.floor(float(entry_alt))
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


def _xy_rows_to_coords_with_dem_altitude(
    rows_xy: Iterable[Sequence[float]],
) -> List[Dict[str, float]]:
    coords = [_xy_to_coord(point_xy) for point_xy in rows_xy or []]
    pairs: List[Tuple[float, float]] = []
    for coord in coords:
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        if lat is None or lon is None:
            pairs.append((math.nan, math.nan))
        else:
            pairs.append((float(lat), float(lon)))
    altitudes = _dem_altitudes_for_pairs_cached(pairs)
    out: List[Dict[str, float]] = []
    for coord, altitude in zip(coords, altitudes):
        out.append(
            {
                "latitude": float(coord["latitude"]),
                "longitude": float(coord["longitude"]),
                "altitude": _altitude_int(altitude),
            }
        )
    return out


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
    is_line_search_waypoint = bool(_line_search_coordinate_list(first_waypoint))
    if not _is_point_hold_waypoint(first_waypoint) and not is_line_search_waypoint:
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


def _line_route_offset_m(path_row: Dict[str, Any]) -> float:
    offset_m = _to_float(path_row.get("lineRouteOffsetM"))
    sep_m = (
        _to_float(path_row.get("lineRouteOffsetSepM"))
        or _to_float(path_row.get("dbSepM"))
        or _to_float(path_row.get("sepCandM"))
        or 0.0
    )
    fov_ref = (
        _to_float(path_row.get("resolvedDbFovDeg"))
        or _to_float(path_row.get("resolvedDbFov"))
        or _to_float(path_row.get("resolvedBaseFovDeg"))
        or _to_float(path_row.get("resolvedFovDeg"))
        or _to_float(path_row.get("FOV"))
        or 0.0
    )
    sep_m = _route_offset_sep_for_fov(fov_ref, sep_m)
    computed_offset_m = 0.0
    if sep_m > 0.0:
        try:
            route_scale = float(get_runtime_float("line_route_offset_scale", 1.0))
        except Exception:
            route_scale = 1.0
        computed_offset_m = float(sep_m) * max(float(route_scale), 0.1)
    if offset_m is None or offset_m <= 0.0:
        offset_m = computed_offset_m
    elif computed_offset_m > 0.0:
        offset_m = min(float(offset_m), float(computed_offset_m))
    if offset_m is None or offset_m <= 0.0:
        offset_m = 300.0
    return max(1.0, float(offset_m))


def _line_ingress_entry_offset_m(path_row: Dict[str, Any]) -> float:
    offset_m = _line_route_offset_m(path_row)
    try:
        entry_scale = float(get_runtime_float("next_collab_line_ingress_entry_offset_scale", 1.0))
    except Exception:
        entry_scale = 1.0
    if entry_scale <= 0.0:
        entry_scale = 1.0
    return max(1.0, float(offset_m) * float(entry_scale))


def _line_anchor_xy_from_sweep_xy(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    offset_m: float,
    reference_xy: Tuple[float, float] | None = None,
) -> Tuple[float, float] | None:
    rows = _dedupe_xy_rows(_xy_rows(sweep_xy), eps_m=0.5)
    if len(rows) < 2:
        return None

    def _anchor_for(row_pair: Sequence[Tuple[float, float]]) -> Tuple[float, float] | None:
        (x1, y1), (x2, y2) = row_pair[0], row_pair[-1]
        mid_x = (float(x1) + float(x2)) * 0.5
        mid_y = (float(y1) + float(y2)) * 0.5
        dx = float(x2) - float(x1)
        dy = float(y2) - float(y1)
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            return (mid_x, mid_y)
        ux = dy / norm
        uy = -dx / norm
        return (
            mid_x + (ux * max(float(offset_m), 0.0)),
            mid_y + (uy * max(float(offset_m), 0.0)),
        )

    candidates = [
        anchor
        for anchor in (
            _anchor_for(rows),
            _anchor_for(list(reversed(rows))),
        )
        if anchor is not None
    ]
    if not candidates:
        return None
    if reference_xy is not None:
        return min(candidates, key=lambda anchor: _distance_xy(reference_xy, anchor))
    return candidates[0]


def _line_route_axis_xy(
    path_row: Dict[str, Any],
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None = None,
) -> Tuple[Tuple[float, float], Tuple[float, float]] | None:
    centerline_xy = _dedupe_xy_rows(_xy_rows(path_row.get("centerLineXY")), eps_m=0.5)
    if len(centerline_xy) >= 2:
        return (
            (float(centerline_xy[0][0]), float(centerline_xy[0][1])),
            (float(centerline_xy[-1][0]), float(centerline_xy[-1][1])),
        )
    start_xy = _xy_pair(path_row.get("waypointStartXY") or path_row.get("targetXY"))
    end_xy = _xy_pair(path_row.get("waypointEndXY") or path_row.get("targetFaceXY"))
    if start_xy is not None and end_xy is not None and _distance_xy(start_xy, end_xy) > 1.0:
        return start_xy, end_xy

    midpoints: List[Tuple[float, float]] = []
    for line_xy in scan_lines_xy or []:
        rows = _dedupe_xy_rows(_xy_rows(line_xy), eps_m=0.5)
        midpoint = _midpoint_xy(rows)
        if midpoint is not None:
            midpoints.append(midpoint)
    if len(midpoints) >= 2 and _distance_xy(midpoints[0], midpoints[-1]) > 1.0:
        return midpoints[0], midpoints[-1]
    return None


def _line_route_polyline_xy(
    path_row: Dict[str, Any],
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None = None,
) -> List[Tuple[float, float]]:
    centerline_xy = _dedupe_xy_rows(_xy_rows(path_row.get("centerLineXY")), eps_m=0.5)
    if len(centerline_xy) >= 2:
        return centerline_xy

    axis_xy = _line_route_axis_xy(path_row, scan_lines_xy)
    if axis_xy is not None:
        return [axis_xy[0], axis_xy[1]]
    return []


def _project_point_to_route_polyline_xy(
    point_xy: Tuple[float, float],
    route_line_xy: Sequence[Tuple[float, float]],
) -> tuple[Tuple[float, float], Tuple[float, float], float] | None:
    rows = _dedupe_xy_rows(_xy_rows(route_line_xy), eps_m=0.5)
    if len(rows) < 2:
        return None

    total_len_m = _line_length_xy(rows)
    if total_len_m <= 1e-6:
        return None

    px, py = float(point_xy[0]), float(point_xy[1])
    walked_m = 0.0
    best: tuple[float, Tuple[float, float], Tuple[float, float], float] | None = None
    for idx in range(len(rows) - 1):
        sx, sy = float(rows[idx][0]), float(rows[idx][1])
        ex, ey = float(rows[idx + 1][0]), float(rows[idx + 1][1])
        dx = ex - sx
        dy = ey - sy
        seg_len_m = math.hypot(dx, dy)
        if seg_len_m <= 1e-6:
            continue
        denom = (dx * dx) + (dy * dy)
        ratio = (((px - sx) * dx) + ((py - sy) * dy)) / max(denom, 1e-9)
        ratio = max(0.0, min(1.0, float(ratio)))
        center_xy = (sx + (dx * ratio), sy + (dy * ratio))
        dist_m = _distance_xy((px, py), center_xy)
        tangent_xy = (dx / seg_len_m, dy / seg_len_m)
        progress_ratio = max(0.0, min(1.0, (walked_m + (seg_len_m * ratio)) / total_len_m))
        if best is None or dist_m < best[0]:
            best = (float(dist_m), center_xy, tangent_xy, float(progress_ratio))
        walked_m += seg_len_m

    if best is None:
        return None
    return best[1], best[2], best[3]


def _line_anchor_xy_from_route_polyline(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    route_line_xy: Sequence[Tuple[float, float]],
    offset_m: float,
    reference_xy: Tuple[float, float] | None = None,
    start_side: float | None = None,
    end_side: float | None = None,
) -> Tuple[float, float] | None:
    rows = _dedupe_xy_rows(_xy_rows(sweep_xy), eps_m=0.5)
    midpoint_xy = _midpoint_xy(rows)
    if midpoint_xy is None:
        return None
    projected = _project_point_to_route_polyline_xy(midpoint_xy, route_line_xy)
    if projected is None:
        return None

    center_xy, tangent_xy, progress_ratio = projected
    normal_xy = (-float(tangent_xy[1]), float(tangent_xy[0]))
    offset_abs_m = max(float(offset_m), 0.0)
    if start_side is not None and end_side is not None:
        signed_side = float(start_side) + ((float(end_side) - float(start_side)) * float(progress_ratio))
        return (
            float(center_xy[0]) + (normal_xy[0] * offset_abs_m * signed_side),
            float(center_xy[1]) + (normal_xy[1] * offset_abs_m * signed_side),
        )

    candidates = [
        (
            float(center_xy[0]) + (normal_xy[0] * offset_abs_m),
            float(center_xy[1]) + (normal_xy[1] * offset_abs_m),
        ),
        (
            float(center_xy[0]) - (normal_xy[0] * offset_abs_m),
            float(center_xy[1]) - (normal_xy[1] * offset_abs_m),
        ),
    ]
    if reference_xy is not None:
        return min(candidates, key=lambda anchor: _distance_xy(reference_xy, anchor))
    return candidates[0]


def _line_anchor_xy_from_route_axis(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    route_start_xy: Tuple[float, float],
    route_end_xy: Tuple[float, float],
    offset_m: float,
    reference_xy: Tuple[float, float] | None = None,
    start_side: float | None = None,
    end_side: float | None = None,
) -> Tuple[float, float] | None:
    rows = _dedupe_xy_rows(_xy_rows(sweep_xy), eps_m=0.5)
    midpoint_xy = _midpoint_xy(rows)
    if midpoint_xy is None:
        return None
    sx, sy = float(route_start_xy[0]), float(route_start_xy[1])
    ex, ey = float(route_end_xy[0]), float(route_end_xy[1])
    dx = ex - sx
    dy = ey - sy
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return None
    ux = dx / norm
    uy = dy / norm
    denom = (dx * dx) + (dy * dy)
    t = (((float(midpoint_xy[0]) - sx) * dx) + ((float(midpoint_xy[1]) - sy) * dy)) / max(denom, 1e-9)
    t = max(0.0, min(1.0, float(t)))
    center_xy = (sx + (dx * t), sy + (dy * t))
    normal_xy = (-uy, ux)
    offset_abs_m = max(float(offset_m), 0.0)
    if start_side is not None and end_side is not None:
        signed_side = float(start_side) + ((float(end_side) - float(start_side)) * t)
        return (
            float(center_xy[0]) + (normal_xy[0] * offset_abs_m * signed_side),
            float(center_xy[1]) + (normal_xy[1] * offset_abs_m * signed_side),
        )
    candidates = [
        (
            float(center_xy[0]) + (normal_xy[0] * offset_abs_m),
            float(center_xy[1]) + (normal_xy[1] * offset_abs_m),
        ),
        (
            float(center_xy[0]) - (normal_xy[0] * offset_abs_m),
            float(center_xy[1]) - (normal_xy[1] * offset_abs_m),
        ),
    ]
    if reference_xy is not None:
        return min(candidates, key=lambda anchor: _distance_xy(reference_xy, anchor))
    return candidates[0]


def _line_route_offset_side_pair(path_row: Dict[str, Any]) -> tuple[float, float] | None:
    start_side = _to_float(path_row.get("lineRouteOffsetStartSide"))
    end_side = _to_float(path_row.get("lineRouteOffsetEndSide"))
    if start_side is None or end_side is None:
        return None
    if not math.isfinite(float(start_side)) or not math.isfinite(float(end_side)):
        return None
    return (
        max(-1.0, min(1.0, float(start_side))),
        max(-1.0, min(1.0, float(end_side))),
    )


def _line_anchor_xy_for_path_row(
    path_row: Dict[str, Any],
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    offset_m: float,
    reference_xy: Tuple[float, float] | None = None,
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None = None,
) -> Tuple[float, float] | None:
    side_pair = _line_route_offset_side_pair(path_row)
    route_line_xy = _line_route_polyline_xy(path_row, scan_lines_xy)
    if len(route_line_xy) >= 2:
        anchor_xy = _line_anchor_xy_from_route_polyline(
            sweep_xy,
            route_line_xy=route_line_xy,
            offset_m=float(offset_m),
            reference_xy=reference_xy,
            start_side=side_pair[0] if side_pair is not None else None,
            end_side=side_pair[1] if side_pair is not None else None,
        )
        if anchor_xy is not None:
            return anchor_xy

    axis_xy = _line_route_axis_xy(path_row, scan_lines_xy)
    if axis_xy is not None:
        anchor_xy = _line_anchor_xy_from_route_axis(
            sweep_xy,
            route_start_xy=axis_xy[0],
            route_end_xy=axis_xy[1],
            offset_m=float(offset_m),
            reference_xy=reference_xy,
            start_side=side_pair[0] if side_pair is not None else None,
            end_side=side_pair[1] if side_pair is not None else None,
        )
        if anchor_xy is not None:
            return anchor_xy
    return _line_anchor_xy_from_sweep_xy(
        sweep_xy,
        offset_m=float(offset_m),
        reference_xy=reference_xy,
    )


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


def _reanchor_line_search_waypoints_to_first_sweep(
    waypoints: List[Dict[str, Any]],
    *,
    offset_m: float,
    altitude_fn: Callable[[float, float], int] | None,
    reference_xy_for_offset: Tuple[float, float] | None = None,
    path_row: Dict[str, Any] | None = None,
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None = None,
) -> int:
    changed = 0
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
        coords = _line_search_coordinate_list(waypoint)
        if len(coords) < 2:
            continue
        interp_points = _to_int(line_search.get("interpolationPoints")) or _next_collab_sweep_points_per_leg()
        first_sweep_coords = coords[: max(2, min(len(coords), int(interp_points)))]
        first_sweep_xy = [
            xy
            for xy in (coord_to_xy(coord) for coord in first_sweep_coords)
            if xy is not None
        ]
        if len(first_sweep_xy) < 2:
            continue
        current_xy = _waypoint_coordinate_xy(waypoint)
        reference_xy = reference_xy_for_offset if reference_xy_for_offset is not None else current_xy
        anchor_xy = (
            _line_anchor_xy_for_path_row(
                path_row,
                first_sweep_xy,
                offset_m=float(offset_m),
                reference_xy=reference_xy,
                scan_lines_xy=scan_lines_xy,
            )
            if isinstance(path_row, dict)
            else _line_anchor_xy_from_sweep_xy(
                first_sweep_xy,
                offset_m=float(offset_m),
                reference_xy=reference_xy,
            )
        )
        if anchor_xy is None:
            continue
        if current_xy is not None and _distance_xy(current_xy, anchor_xy) <= max(25.0, abs(float(offset_m)) * 0.1):
            continue
        waypoint["coordinate"] = _xy_to_coord_with_altitude(anchor_xy, altitude_fn)
        changed += 1
    return changed


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

        short_filming = short_wp.get("filmingProperty") if isinstance(short_wp.get("filmingProperty"), dict) else {}
        for key in ("fieldOfView", "sensorType"):
            if key in short_filming:
                next_filming[key] = deepcopy(short_filming[key])
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
    base = _clamp_line_fov_deg(base_fov_deg)
    scale = float(get_runtime_float("next_collab_first_line_fov_scale", NEXT_COLLAB_FIRST_LINE_FOV_SCALE))
    cap = float(get_runtime_float("next_collab_first_line_fov_max_deg", NEXT_COLLAB_FIRST_LINE_FOV_MAX_DEG))
    if scale <= 0.0:
        scale = float(NEXT_COLLAB_FIRST_LINE_FOV_SCALE)
    if cap <= 0.0:
        cap = float(NEXT_COLLAB_FIRST_LINE_FOV_MAX_DEG)
    boosted = max(base, min(max(float(cap), float(MIN_LINE_FOV_DEG)), float(base) * float(scale)))
    return float(
        apply_runtime_camera_adjusted_fov_deg(
            float(boosted),
            minimum_fov_deg=MIN_LINE_FOV_DEG,
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


def _normalize_runtime_flyover_defaults(waypoints: List[Dict[str, Any]]) -> None:
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        if int(_to_float(waypoint.get("waypointPassType")) or 0) == PASS_FLYOVER:
            waypoint["waypointPassType"] = PASS_FLYBY


def _apply_runtime_flyover_to_waypoints(waypoints: List[Dict[str, Any]]) -> None:
    if not isinstance(waypoints, list) or not waypoints:
        return
    flyover = _runtime_flyover_options()
    _normalize_runtime_flyover_defaults(waypoints)
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


def _line_sweep_items_from_scan_lines(
    path_row: Dict[str, Any],
    scan_lines_xy: List[List[Tuple[float, float]]],
    *,
    reference_xy: Tuple[float, float] | None,
) -> List[Dict[str, Any]]:
    if not scan_lines_xy:
        return []
    route_offset_m = _line_route_offset_m(path_row)
    all_items: List[Dict[str, Any]] = []
    for idx, line_xy in enumerate(scan_lines_xy):
        sweep_xy = _dedupe_xy_rows(_xy_rows(line_xy), eps_m=0.5)
        if len(sweep_xy) < 2:
            continue
        anchor_xy = _line_anchor_xy_for_path_row(
            path_row,
            sweep_xy,
            offset_m=float(route_offset_m),
            reference_xy=reference_xy,
            scan_lines_xy=scan_lines_xy,
        )
        if anchor_xy is None:
            continue
        all_items.append(
            {
                "anchorXY": (float(anchor_xy[0]), float(anchor_xy[1])),
                "sweepXY": sweep_xy,
                "sweepIndex": int(idx),
            }
        )
    if len(all_items) >= 2 and reference_xy is not None:
        if _distance_xy(reference_xy, all_items[-1]["anchorXY"]) + 1e-6 < _distance_xy(reference_xy, all_items[0]["anchorXY"]):
            all_items.reverse()
    if len(all_items) <= 2:
        return all_items

    spacing_m = max(float(_to_float(path_row.get("lineRouteWpSpacingM")) or _next_collab_line_route_wp_spacing_m()), 1.0)
    progress_m: List[float] = [0.0]
    for idx in range(1, len(all_items)):
        prev_anchor = all_items[idx - 1].get("anchorXY")
        curr_anchor = all_items[idx].get("anchorXY")
        if not (isinstance(prev_anchor, tuple) and isinstance(curr_anchor, tuple)):
            progress_m.append(float(progress_m[-1]))
            continue
        progress_m.append(float(progress_m[-1]) + _distance_xy(prev_anchor, curr_anchor))
    total_progress_m = float(progress_m[-1]) if progress_m else 0.0
    if total_progress_m <= spacing_m * 0.25:
        return [all_items[0], all_items[-1]]

    selected: List[Dict[str, Any]] = [dict(all_items[0])]
    selected[-1]["anchorXY"] = tuple(all_items[0]["anchorXY"])
    last_selected_idx = 0
    target_m = float(spacing_m)
    while target_m < total_progress_m - 1e-6:
        idx = max(last_selected_idx + 1, 1)
        while idx < len(progress_m) and float(progress_m[idx]) + 1e-6 < target_m:
            idx += 1
        if idx >= len(all_items):
            break
        prev_idx = max(0, idx - 1)
        prev_anchor = all_items[prev_idx].get("anchorXY")
        curr_anchor = all_items[idx].get("anchorXY")
        if not (isinstance(prev_anchor, tuple) and isinstance(curr_anchor, tuple)):
            break
        span_m = max(1e-6, float(progress_m[idx]) - float(progress_m[prev_idx]))
        ratio = max(0.0, min(1.0, (float(target_m) - float(progress_m[prev_idx])) / span_m))
        anchor_xy = (
            float(prev_anchor[0]) + ((float(curr_anchor[0]) - float(prev_anchor[0])) * ratio),
            float(prev_anchor[1]) + ((float(curr_anchor[1]) - float(prev_anchor[1])) * ratio),
        )
        selected_item = dict(all_items[idx])
        selected_item["anchorXY"] = anchor_xy
        selected.append(selected_item)
        last_selected_idx = int(idx)
        target_m += float(spacing_m)

    tail = dict(all_items[-1])
    tail_idx = int(_to_float(tail.get("sweepIndex")) or 0)
    last_idx = int(_to_float(selected[-1].get("sweepIndex")) or -1) if selected else -1
    tail_anchor = tail.get("anchorXY")
    last_anchor = selected[-1].get("anchorXY") if selected else None
    if tail_idx != last_idx or not (
        isinstance(tail_anchor, tuple)
        and isinstance(last_anchor, tuple)
        and _distance_xy(last_anchor, tail_anchor) <= 1.0
    ):
        selected.append(tail)
    return selected


def _normalize_line_sweep_item_anchors_to_route_axis(
    path_row: Dict[str, Any],
    items: List[Dict[str, Any]],
    *,
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None,
    reference_xy: Tuple[float, float] | None,
) -> int:
    if not items:
        return 0
    route_offset_m = _line_route_offset_m(path_row)
    changed = 0
    for item in items:
        if not isinstance(item, dict):
            continue
        sweep_xy = _dedupe_xy_rows(_xy_rows(item.get("sweepXY")), eps_m=0.5)
        if len(sweep_xy) < 2:
            continue
        anchor_xy = _line_anchor_xy_for_path_row(
            path_row,
            sweep_xy,
            offset_m=float(route_offset_m),
            reference_xy=reference_xy,
            scan_lines_xy=scan_lines_xy,
        )
        if anchor_xy is None:
            continue
        current_anchor = item.get("anchorXY")
        if not (
            isinstance(current_anchor, (tuple, list))
            and len(current_anchor) >= 2
            and _distance_xy((float(current_anchor[0]), float(current_anchor[1])), anchor_xy) <= 1.0
        ):
            changed += 1
        item["anchorXY"] = (float(anchor_xy[0]), float(anchor_xy[1]))
    return changed


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


def _collect_group_sweep_rows_xy(
    *,
    group: List[Dict[str, Any]],
    all_sweep_lines_xy: List[List[Tuple[float, float]]],
    previous_rep_sweep_idx: int | None = None,
) -> List[Tuple[float, float]]:
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
        merged_rows_xy: List[Tuple[float, float]] = []
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
            merged_rows_xy.extend(sweep_rows)
        return merged_rows_xy

    merged_rows_fallback: List[Tuple[float, float]] = []
    for item in group:
        sweep_xy = item.get("sweepXY") if isinstance(item, dict) else []
        sweep_rows = _line_three_point_xy(_dedupe_xy_rows(list(sweep_xy or []), eps_m=0.5))
        if len(sweep_rows) < 2:
            continue
        merged_rows_fallback.extend(sweep_rows)
    return merged_rows_fallback


def _line_group_start_anchor_xy(
    *,
    path_row: Dict[str, Any],
    group: List[Dict[str, Any]],
    all_sweep_lines_xy: List[List[Tuple[float, float]]],
    previous_rep_sweep_idx: int | None = None,
    offset_m: float,
    reference_anchor_xy: Tuple[float, float] | None = None,
) -> Tuple[float, float] | None:
    if not group:
        return None
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
        anchor_xy = _line_anchor_xy_for_path_row(
            path_row,
            all_sweep_lines_xy[start_idx],
            offset_m=float(offset_m),
            reference_xy=reference_anchor_xy,
            scan_lines_xy=all_sweep_lines_xy,
        )
        if anchor_xy is not None:
            return anchor_xy

    first_sweep_xy = group[0].get("sweepXY") if isinstance(group[0], dict) else None
    if isinstance(first_sweep_xy, list) and len(first_sweep_xy) >= 2:
        anchor_xy = _line_anchor_xy_for_path_row(
            path_row,
            first_sweep_xy,
            offset_m=float(offset_m),
            reference_xy=reference_anchor_xy,
            scan_lines_xy=all_sweep_lines_xy,
        )
        if anchor_xy is not None:
            return anchor_xy

    first_anchor = group[0].get("anchorXY") if isinstance(group[0], dict) else None
    if isinstance(first_anchor, (tuple, list)) and len(first_anchor) >= 2:
        return (float(first_anchor[0]), float(first_anchor[1]))
    return None


def _collect_group_sweep_coords(
    *,
    group: List[Dict[str, Any]],
    all_sweep_lines_xy: List[List[Tuple[float, float]]],
    previous_rep_sweep_idx: int | None = None,
) -> List[Dict[str, Any]]:
    return _xy_rows_to_coords_with_dem_altitude(
        _collect_group_sweep_rows_xy(
            group=group,
            all_sweep_lines_xy=all_sweep_lines_xy,
            previous_rep_sweep_idx=previous_rep_sweep_idx,
        )
    )


def _collect_area_group_sweep_rows_xy(
    *,
    group: List[Dict[str, Any]],
    all_sweep_lines_xy: List[List[Tuple[float, float]]],
    previous_rep_sweep_idx: int | None = None,
) -> List[Tuple[float, float]]:
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
        merged_rows_xy: List[Tuple[float, float]] = []
        for sweep_idx in sweep_range:
            sweep_rows = _line_three_point_xy(
                _dedupe_xy_rows(list(all_sweep_lines_xy[sweep_idx] or []), eps_m=0.5)
            )
            if len(sweep_rows) < 2:
                continue
            merged_rows_xy.extend(sweep_rows)
        return merged_rows_xy

    merged_rows_fallback: List[Tuple[float, float]] = []
    for item in group:
        sweep_xy = item.get("sweepXY") if isinstance(item, dict) else []
        sweep_rows = _line_three_point_xy(_dedupe_xy_rows(list(sweep_xy or []), eps_m=0.5))
        if len(sweep_rows) < 2:
            continue
        merged_rows_fallback.extend(sweep_rows)
    return merged_rows_fallback


def _collect_area_group_sweep_coords(
    *,
    group: List[Dict[str, Any]],
    all_sweep_lines_xy: List[List[Tuple[float, float]]],
    previous_rep_sweep_idx: int | None = None,
) -> List[Dict[str, Any]]:
    return _xy_rows_to_coords_with_dem_altitude(
        _collect_area_group_sweep_rows_xy(
            group=group,
            all_sweep_lines_xy=all_sweep_lines_xy,
            previous_rep_sweep_idx=previous_rep_sweep_idx,
        )
    )


def _estimate_line_search_speed_xy_mps(
    *,
    prev_xy: Tuple[float, float] | None,
    anchor_xy: Tuple[float, float] | None,
    sweep_xy: List[Tuple[float, float]],
    cruise_speed_mps: float,
    fallback_search_speed_mps: float,
    speed_scale: float = 1.0,
    reference_xy: Tuple[float, float] | None = None,
) -> float:
    fallback_speed = max(0.0, float(fallback_search_speed_mps))
    if prev_xy is None or anchor_xy is None or cruise_speed_mps <= 0.0:
        if reference_xy is not None and anchor_xy is not None and cruise_speed_mps > 0.0:
            prev_xy = reference_xy
        else:
            return fallback_speed

    if not sweep_xy:
        return fallback_speed

    transit_len_m = _distance_xy(prev_xy, anchor_xy)
    origin_xy = reference_xy if reference_xy is not None else prev_xy
    if transit_len_m <= 1e-6 and reference_xy is not None and anchor_xy is not None:
        transit_len_m = _distance_xy(reference_xy, anchor_xy)
    if transit_len_m <= 1e-6 and origin_xy is not None:
        for candidate_xy in [anchor_xy] + list(sweep_xy):
            if candidate_xy is None:
                continue
            candidate_dist_m = _distance_xy(origin_xy, candidate_xy)
            if candidate_dist_m > 1.0:
                transit_len_m = candidate_dist_m
                break
    sweep_len_m = _line_length_xy(sweep_xy)
    if transit_len_m <= 1e-6 or sweep_len_m <= 1e-6:
        return fallback_speed

    effective_transit_len_m = effective_line_search_transit_m(transit_len_m)
    if effective_transit_len_m <= 1e-6:
        return fallback_speed
    travel_time_s = float(effective_transit_len_m) / float(cruise_speed_mps)
    if travel_time_s <= 1e-6:
        return fallback_speed
    try:
        effective_scale = max(float(speed_scale), 0.1)
    except Exception:
        effective_scale = 1.0
    estimated_speed_mps = (float(sweep_len_m) / float(travel_time_s)) * float(effective_scale)
    return clamp_line_search_speed_mps(
        estimated_speed_mps,
        cruise_speed_mps=float(cruise_speed_mps),
        speed_scale=float(effective_scale),
    )


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
    return _estimate_line_search_speed_xy_mps(
        prev_xy=prev_xy,
        anchor_xy=anchor_xy,
        sweep_xy=sweep_xy,
        cruise_speed_mps=float(cruise_speed_mps),
        fallback_search_speed_mps=float(fallback_search_speed_mps),
        speed_scale=float(speed_scale),
        reference_xy=reference_xy,
    )


def _recompute_first_line_search_speed_from_entry_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    entry_coord: Dict[str, Any] | None,
    transit_speed_mps: float,
    fallback_search_speed_mps: float,
    speed_scale: float,
) -> bool:
    first_line_idx = next(
        (
            idx
            for idx, waypoint in enumerate(waypoints)
            if isinstance(waypoint, dict) and _line_search_coordinate_list(waypoint)
        ),
        None,
    )
    if first_line_idx is None:
        return False

    waypoint = waypoints[first_line_idx] if isinstance(waypoints[first_line_idx], dict) else {}
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else None
    if line_search is None:
        return False

    previous_coord = None
    if first_line_idx > 0 and isinstance(waypoints[first_line_idx - 1], dict):
        previous_coord = waypoints[first_line_idx - 1].get("coordinate")
    if not isinstance(previous_coord, dict):
        previous_coord = entry_coord

    speed_mps = _estimate_line_search_speed_mps(
        prev_coord=previous_coord if isinstance(previous_coord, dict) else None,
        anchor_coord=waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else None,
        sweep_coords=_line_search_coordinate_list(waypoint),
        cruise_speed_mps=float(transit_speed_mps),
        fallback_search_speed_mps=float(fallback_search_speed_mps),
        speed_scale=float(speed_scale),
        reference_coord=entry_coord if isinstance(entry_coord, dict) else None,
    )
    if speed_mps <= 0.0:
        return False
    line_search["searchSpeed"] = float(speed_mps)
    return True


def _line_route_endpoint_anchor_xy(
    path_row: Dict[str, Any],
    *,
    tail_sweep_xy: Sequence[Tuple[float, float]],
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None,
    reference_anchor_xy: Tuple[float, float] | None,
) -> Tuple[float, float] | None:
    route_line_xy = _line_route_polyline_xy(path_row, scan_lines_xy)
    if len(route_line_xy) < 2:
        return None
    tail_points = _dedupe_xy_rows(_xy_rows(tail_sweep_xy), eps_m=0.5)
    if not tail_points:
        return None

    start_xy = (float(route_line_xy[0][0]), float(route_line_xy[0][1]))
    end_xy = (float(route_line_xy[-1][0]), float(route_line_xy[-1][1]))
    start_dist_m = min(_distance_xy(point_xy, start_xy) for point_xy in tail_points)
    end_dist_m = min(_distance_xy(point_xy, end_xy) for point_xy in tail_points)

    if end_dist_m <= start_dist_m:
        endpoint_xy = end_xy
        adjacent_xy = (float(route_line_xy[-2][0]), float(route_line_xy[-2][1]))
        side_idx = 1
        tangent_dx = endpoint_xy[0] - adjacent_xy[0]
        tangent_dy = endpoint_xy[1] - adjacent_xy[1]
    else:
        endpoint_xy = start_xy
        adjacent_xy = (float(route_line_xy[1][0]), float(route_line_xy[1][1]))
        side_idx = 0
        tangent_dx = adjacent_xy[0] - endpoint_xy[0]
        tangent_dy = adjacent_xy[1] - endpoint_xy[1]

    tangent_len_m = math.hypot(tangent_dx, tangent_dy)
    if tangent_len_m <= 1e-6:
        return None
    ux = float(tangent_dx) / float(tangent_len_m)
    uy = float(tangent_dy) / float(tangent_len_m)
    normal_xy = (-uy, ux)
    offset_m = max(float(_line_route_offset_m(path_row)), 0.0)
    side_pair = _line_route_offset_side_pair(path_row)
    if side_pair is not None:
        side = float(side_pair[side_idx])
        return (
            float(endpoint_xy[0]) + (float(normal_xy[0]) * offset_m * side),
            float(endpoint_xy[1]) + (float(normal_xy[1]) * offset_m * side),
        )

    candidates = [
        (
            float(endpoint_xy[0]) + (float(normal_xy[0]) * offset_m),
            float(endpoint_xy[1]) + (float(normal_xy[1]) * offset_m),
        ),
        (
            float(endpoint_xy[0]) - (float(normal_xy[0]) * offset_m),
            float(endpoint_xy[1]) - (float(normal_xy[1]) * offset_m),
        ),
    ]
    if reference_anchor_xy is not None:
        return min(candidates, key=lambda anchor: _distance_xy(anchor, reference_anchor_xy))
    return candidates[0]


def _snap_last_line_search_waypoint_to_route_endpoint(
    waypoints: List[Dict[str, Any]],
    *,
    path_row: Dict[str, Any],
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None,
    altitude_fn: Callable[[float, float], int] | None,
) -> int:
    line_indices = [
        idx
        for idx, waypoint in enumerate(waypoints or [])
        if isinstance(waypoint, dict) and _line_search_coordinate_list(waypoint)
    ]
    if not line_indices:
        return 0
    waypoint = waypoints[line_indices[-1]]
    filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
    line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else {}
    coords = _line_search_coordinate_list(waypoint)
    if len(coords) < 2:
        return 0

    interp_points = _to_int(line_search.get("interpolationPoints")) or _next_collab_sweep_points_per_leg()
    tail_count = max(2, min(len(coords), int(interp_points)))
    tail_sweep_xy = [
        xy
        for xy in (coord_to_xy(coord) for coord in coords[-tail_count:])
        if xy is not None
    ]
    if len(tail_sweep_xy) < 2:
        return 0

    current_xy = _waypoint_coordinate_xy(waypoint)
    anchor_xy = _line_route_endpoint_anchor_xy(
        path_row,
        tail_sweep_xy=tail_sweep_xy,
        scan_lines_xy=scan_lines_xy,
        reference_anchor_xy=current_xy,
    )
    if anchor_xy is None:
        return 0
    if current_xy is not None and _distance_xy(current_xy, anchor_xy) <= 10.0:
        return 0

    current_coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else {}
    snapped_coord = _xy_to_coord_with_altitude(anchor_xy, altitude_fn)
    if altitude_fn is None:
        current_alt = _to_float(current_coord.get("altitude")) if isinstance(current_coord, dict) else None
        if current_alt is not None:
            snapped_coord["altitude"] = _altitude_int(current_alt)
    waypoint["coordinate"] = snapped_coord
    return 1


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


def _path_row_transit_speed_mps(path_row: Dict[str, Any]) -> float | None:
    resolved_vel_kmh = _to_float(path_row.get("resolvedVelMps"))
    if resolved_vel_kmh is not None and resolved_vel_kmh > 0.0:
        return float(resolved_vel_kmh) / 3.6
    return None


def _average_sweep_length_m(path_row: Dict[str, Any]) -> float | None:
    lengths: List[float] = []
    for points_xy in (path_row.get("sweepLineListXY") or []):
        rows = _dedupe_xy_rows(_xy_rows(points_xy), eps_m=0.5)
        if len(rows) < 2:
            continue
        length_m = _line_length_xy(rows)
        if length_m > 1.0:
            lengths.append(float(length_m))
    if not lengths:
        for item in _line_sweep_items_xy(path_row):
            rows = _dedupe_xy_rows(_xy_rows(item.get("sweepXY")), eps_m=0.5)
            if len(rows) < 2:
                continue
            length_m = _line_length_xy(rows)
            if length_m > 1.0:
                lengths.append(float(length_m))
    if not lengths:
        return None
    return float(sum(lengths) / len(lengths))


def _spacing_based_search_speed_mps(
    path_row: Dict[str, Any],
    *,
    cruise_speed_mps: float,
    speed_scale: float,
) -> float | None:
    spacing_m = _to_float(path_row.get("lineSweepSpacingM"))
    if spacing_m is None or spacing_m <= 0.0:
        return None
    sweep_len_m = _average_sweep_length_m(path_row)
    if sweep_len_m is None or sweep_len_m <= 0.0 or cruise_speed_mps <= 0.0:
        return None
    try:
        effective_scale = max(float(speed_scale), 0.1)
    except Exception:
        effective_scale = 1.0
    estimated_speed_mps = (float(sweep_len_m) / float(spacing_m)) * float(cruise_speed_mps) * float(effective_scale)
    return clamp_line_search_speed_mps(
        estimated_speed_mps,
        cruise_speed_mps=float(cruise_speed_mps),
        speed_scale=float(effective_scale),
    )


def _search_speed_mps(
    path_row: Dict[str, Any],
    template_path: Dict[str, Any] | None,
    *,
    cruise_speed_mps: float | None = None,
    speed_scale: float = 1.0,
) -> float:
    if cruise_speed_mps is not None and cruise_speed_mps > 0.0:
        spacing_speed_mps = _spacing_based_search_speed_mps(
            path_row,
            cruise_speed_mps=float(cruise_speed_mps),
            speed_scale=float(speed_scale),
        )
        if spacing_speed_mps is not None and spacing_speed_mps > 0.0:
            return float(spacing_speed_mps)
    transit_speed_mps = _path_row_transit_speed_mps(path_row)
    if transit_speed_mps is not None and transit_speed_mps > 0.0:
        return float(transit_speed_mps)
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


def _simplify_line_waypoints_to_start_and_search(
    waypoints: List[Dict[str, Any]],
    *,
    speed_mps: float,
    sensor_type: int,
    field_of_view_deg: float,
) -> int:
    if not isinstance(waypoints, list) or not waypoints:
        return 0
    line_search_waypoints = [
        waypoint
        for waypoint in waypoints
        if isinstance(waypoint, dict) and _line_search_coordinate_list(waypoint)
    ]
    if not line_search_waypoints:
        return 0

    first_line_wp = line_search_waypoints[0]
    start_coord = first_line_wp.get("coordinate") if isinstance(first_line_wp.get("coordinate"), dict) else None
    if not isinstance(start_coord, dict):
        return 0

    first_filming = first_line_wp.get("filmingProperty") if isinstance(first_line_wp.get("filmingProperty"), dict) else {}
    first_line_search = first_filming.get("lineSearch") if isinstance(first_filming.get("lineSearch"), dict) else {}
    first_coords = first_line_search.get("coordinateList") if isinstance(first_line_search.get("coordinateList"), list) else []
    start_orientation_coord = first_coords[0] if first_coords and isinstance(first_coords[0], dict) else None

    if len(line_search_waypoints) >= 2:
        second_wp = line_search_waypoints[1]
        second_filming = second_wp.get("filmingProperty") if isinstance(second_wp.get("filmingProperty"), dict) else {}
        second_line_search = second_filming.get("lineSearch") if isinstance(second_filming.get("lineSearch"), dict) else {}
        second_coords = second_line_search.get("coordinateList") if isinstance(second_line_search.get("coordinateList"), list) else []
        if first_coords and isinstance(second_line_search, dict):
            second_line_search["coordinateList"] = _merge_line_search_coordinate_lists(first_coords, second_coords)
            second_filming["lineSearch"] = second_line_search
            second_wp["filmingProperty"] = second_filming
        line_search_waypoints = line_search_waypoints[1:]

    start_waypoint = _make_hold_waypoint(
        coordinate=start_coord,
        speed_mps=float(speed_mps),
        sensor_type=int(sensor_type),
        field_of_view_deg=float(field_of_view_deg),
        orientation_coordinate=start_orientation_coord,
        waypoint_pass_type=int(PASS_FLYBY),
        include_filming=True,
    )
    original_len = len(waypoints)
    waypoints[:] = [start_waypoint] + line_search_waypoints
    return max(0, int(original_len) - int(len(waypoints)))


def build_mission_info_from_planned_row(
    path_row: Dict[str, Any],
    *,
    template_info: Dict[str, Any],
    fallback_polygon_coords: List[Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    info = deepcopy(template_info or {})
    aircraft_id = int(_to_float(path_row.get("aircraftID")) or 0)
    resolved_db_fov_deg = _to_float(path_row.get("resolvedDbFovDeg")) or _to_float(path_row.get("resolvedDbFov"))
    resolved_base_fov_deg = _to_float(path_row.get("resolvedBaseFovDeg"))
    resolved_fov_deg = _to_float(path_row.get("resolvedFovDeg"))
    resolved_vel_kmh = _to_float(path_row.get("resolvedVelMps"))
    resolved_sep_m = _to_float(path_row.get("dbSepM")) or _to_float(path_row.get("sepCandM"))
    route_offset_sep_m = (
        _to_float(path_row.get("lineRouteOffsetSepM"))
        or _to_float(path_row.get("dbSepM"))
        or _to_float(path_row.get("sepCandM"))
    )
    route_offset_sep_m = _route_offset_sep_for_fov(
        resolved_db_fov_deg or resolved_base_fov_deg or resolved_fov_deg,
        route_offset_sep_m,
    )
    bearing_deg = _to_float(path_row.get("bearingDeg"))
    manual_fov_active = _runtime_manual_fov_active()
    if resolved_fov_deg is not None and resolved_fov_deg > 0.0 and not manual_fov_active:
        info["FOV"] = float(resolved_fov_deg)
    if resolved_vel_kmh is not None and resolved_vel_kmh > 0.0:
        info["SPEED"] = float(resolved_vel_kmh)
    if resolved_sep_m is not None and resolved_sep_m > 0.0:
        info["SEP"] = float(resolved_sep_m)
    if route_offset_sep_m is not None and route_offset_sep_m > 0.0:
        info["routeOffsetSepM"] = float(route_offset_sep_m)
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
            info["FOV"] = _clamp_line_fov_deg(
                _runtime_manual_fov_value("line_custom_fov_deg", float(resolved_fov_deg or 10.0)),
                resolved_fov_deg,
            )
        else:
            info["FOV"] = _clamp_line_fov_deg(info.get("FOV"), resolved_fov_deg or 10.0)
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
    waypoint_id_provider: Callable[[], int] | None = None,
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
    normalize_filming_target_altitudes_in_waypoints(
        final_waypoints,
        dem_lookup_many=_dem_altitudes_for_pairs_cached,
    )
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
    reassign_unique_waypoint_ids_inplace(
        final_waypoints,
        waypoint_id_provider=waypoint_id_provider,
    )
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
    waypoint_id_provider: Callable[[], int] | None = None,
    metrics_callback: Callable[[Dict[str, Any]], None] | None = None,
) -> Dict[str, Any]:
    build_started = time.perf_counter()
    metrics: Dict[str, Any] = {
        "pathID": int(path_id),
        "aircraftID": int(aircraft_id),
        "individualMissionID": int(individual_mission_id),
        "source": str(source),
    }

    def _elapsed_ms(started: float) -> float:
        return round((time.perf_counter() - started) * 1000.0, 3)

    def _metric_add(key: str, value: float) -> None:
        metrics[key] = round(float(metrics.get(key, 0.0) or 0.0) + float(value), 3)

    template_transit_speed_mps = _template_speed_mps_value(template_path)
    path_row_transit_speed_mps = _path_row_transit_speed_mps(path_row)
    transit_speed_mps = max(
        1.0,
        float(template_transit_speed_mps)
        if template_transit_speed_mps is not None
        else float(path_row_transit_speed_mps or 30.0),
    )
    search_speed_cruise_mps = max(
        float(transit_speed_mps),
        float(path_row_transit_speed_mps or 0.0),
    )
    base_geometry_search_speed_scale = (
        _mission_search_speed_weight(mission_info)
        if template_transit_speed_mps is not None
        else 1.0
    )
    search_speed_scale_multiplier = _path_row_search_speed_scale_multiplier(path_row)
    geometry_search_speed_scale = max(
        0.1,
        float(base_geometry_search_speed_scale) * float(search_speed_scale_multiplier),
    )
    metrics["searchSpeedBaseScale"] = round(float(base_geometry_search_speed_scale), 3)
    metrics["searchSpeedScaleMultiplier"] = round(float(search_speed_scale_multiplier), 3)
    metrics["searchSpeedScale"] = round(float(geometry_search_speed_scale), 3)
    metrics["transitSpeedMps"] = round(float(transit_speed_mps), 3)
    metrics["pathRowTransitSpeedMps"] = (
        round(float(path_row_transit_speed_mps), 3)
        if path_row_transit_speed_mps is not None
        else None
    )
    metrics["searchSpeedCruiseMps"] = round(float(search_speed_cruise_mps), 3)
    line_sweep_spacing_m = _to_float(path_row.get("lineSweepSpacingM"))
    if line_sweep_spacing_m is not None and line_sweep_spacing_m > 0.0:
        metrics["lineSweepSpacingM"] = round(float(line_sweep_spacing_m), 3)
    density_speed_scale = _to_float(path_row.get("areaDensitySpeedScale"))
    if density_speed_scale is not None and density_speed_scale > 0.0:
        metrics["areaDensitySpeedScale"] = round(float(density_speed_scale), 3)
    search_speed_mps = max(
        1.0,
        _search_speed_mps(
            path_row,
            template_path,
            cruise_speed_mps=float(search_speed_cruise_mps),
            speed_scale=float(geometry_search_speed_scale),
        ),
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
    scan_started = time.perf_counter()
    scan_lines_xy = [
        _dedupe_xy_rows(_xy_rows(points_xy), eps_m=0.5)
        for points_xy in (path_row.get("sweepLineListXY") or [])
        if isinstance(points_xy, list)
    ]
    scan_lines_xy = [points_xy for points_xy in scan_lines_xy if len(points_xy) >= 2]
    metrics["scanLinesMs"] = _elapsed_ms(scan_started)
    metrics["scanLines"] = len(scan_lines_xy)
    metrics["scanLinePoints"] = sum(len(points_xy) for points_xy in scan_lines_xy)

    is_line_mission = bool(mission_info_dict.get("lineList")) and not bool(mission_info_dict.get("areaList"))
    metrics["isLineMission"] = bool(is_line_mission)
    if is_line_mission:
        default_fov_deg = _clamp_line_fov_deg(default_fov_deg)
        first_line_base_fov_deg = _clamp_line_fov_deg(first_line_base_fov_deg, default_fov_deg)
        field_of_view_deg = _clamp_line_fov_deg(field_of_view_deg, default_fov_deg)
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

    line_items_started = time.perf_counter()
    line_sweep_items = _line_sweep_items_xy(path_row)
    line_sweep_items_regenerated_from_scan = False
    if is_line_mission and scan_lines_xy:
        regenerated_line_sweep_items = _line_sweep_items_from_scan_lines(
            path_row,
            scan_lines_xy,
            reference_xy=entry_xy,
        )
        if regenerated_line_sweep_items:
            line_sweep_items = regenerated_line_sweep_items
            line_sweep_items_regenerated_from_scan = True
    if is_line_mission and line_sweep_items and not line_sweep_items_regenerated_from_scan:
        metrics["lineSweepAnchorAxisFixups"] = _normalize_line_sweep_item_anchors_to_route_axis(
            path_row,
            line_sweep_items,
            scan_lines_xy=scan_lines_xy,
            reference_xy=entry_xy,
        )
    elif is_line_mission and line_sweep_items:
        metrics["lineSweepAnchorAxisFixups"] = 0
    if is_line_mission and len(line_sweep_items) >= 2 and entry_xy is not None:
        first_anchor = line_sweep_items[0].get("anchorXY") if isinstance(line_sweep_items[0], dict) else None
        last_anchor = line_sweep_items[-1].get("anchorXY") if isinstance(line_sweep_items[-1], dict) else None
        if isinstance(first_anchor, tuple) and isinstance(last_anchor, tuple):
            if _distance_xy(entry_xy, last_anchor) + 1e-6 < _distance_xy(entry_xy, first_anchor):
                line_sweep_items.reverse()
    metrics["lineSweepItemsMs"] = _elapsed_ms(line_items_started)
    metrics["lineSweepItems"] = len(line_sweep_items)
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
        area_items_started = time.perf_counter()
        area_sweep_items = _area_sweep_items_xy(path_row, scan_lines_xy)
        metrics["areaSweepItemsMs"] = _elapsed_ms(area_items_started)
        metrics["areaSweepItems"] = len(area_sweep_items)
    if area_sweep_items:
        prev_coord = None
        prev_rep_sweep_idx: int | None = None
        if waypoints and isinstance(waypoints[-1], dict):
            prev_coord = waypoints[-1].get("coordinate")
        if not isinstance(prev_coord, dict):
            prev_coord = entry_coord
        prev_xy = coord_to_xy(prev_coord) if isinstance(prev_coord, dict) else None
        area_group_started = time.perf_counter()
        grouped_area_sweeps = _group_area_sweep_items_by_spacing(
            area_sweep_items,
            spacing_m=float(_next_collab_area_route_wp_spacing_m()),
            merge_short_tail=True,
        )
        metrics["areaGroupMs"] = _elapsed_ms(area_group_started)
        metrics["areaGroups"] = len(grouped_area_sweeps)
        for group in grouped_area_sweeps:
            rep_item = group[-1] if group else {}
            anchor_xy = rep_item.get("anchorXY")
            if not (isinstance(anchor_xy, (tuple, list)) and len(anchor_xy) >= 2):
                continue
            anchor_xy_pair = (float(anchor_xy[0]), float(anchor_xy[1]))
            collect_started = time.perf_counter()
            merged_sweep_xy = _collect_area_group_sweep_rows_xy(
                group=group,
                all_sweep_lines_xy=scan_lines_xy,
                previous_rep_sweep_idx=prev_rep_sweep_idx,
            )
            _metric_add("areaCollectRowsMs", _elapsed_ms(collect_started))
            metrics["areaMergedRows"] = int(metrics.get("areaMergedRows", 0) or 0) + len(merged_sweep_xy)
            dem_started = time.perf_counter()
            merged_sweep_coords = _xy_rows_to_coords_with_dem_altitude(merged_sweep_xy)
            _metric_add("areaDemMs", _elapsed_ms(dem_started))
            metrics["areaMergedCoords"] = int(metrics.get("areaMergedCoords", 0) or 0) + len(merged_sweep_coords)
            if len(merged_sweep_coords) < 2:
                continue
            anchor_started = time.perf_counter()
            anchor_coord = _xy_to_coord_with_altitude(anchor_xy_pair, altitude_fn)
            _metric_add("areaAnchorAltitudeMs", _elapsed_ms(anchor_started))
            speed_started = time.perf_counter()
            line_search_speed_mps = _estimate_line_search_speed_xy_mps(
                prev_xy=prev_xy,
                anchor_xy=anchor_xy_pair,
                sweep_xy=merged_sweep_xy,
                cruise_speed_mps=float(search_speed_cruise_mps),
                fallback_search_speed_mps=float(search_speed_mps),
                speed_scale=float(geometry_search_speed_scale),
            )
            _metric_add("areaSpeedMs", _elapsed_ms(speed_started))
            append_started = time.perf_counter()
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
            _metric_add("areaWaypointAppendMs", _elapsed_ms(append_started))
            metrics["areaLineSearchWaypointsBuilt"] = int(
                metrics.get("areaLineSearchWaypointsBuilt", 0) or 0
            ) + 1
            prev_coord = anchor_coord
            prev_xy = anchor_xy_pair
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
        prev_xy = coord_to_xy(prev_coord) if isinstance(prev_coord, dict) else None
        route_offset_m = _line_route_offset_m(path_row)
        # The planner already down-selects lineSweepItemsXY using the route-WP spacing.
        # Grouping them again here can collapse additional sweeps for only some UAVs.
        grouped_line_sweeps = [[dict(item)] for item in line_sweep_items if isinstance(item, dict)]
        for group in grouped_line_sweeps:
            rep_item = group[-1] if group else {}
            anchor_xy = rep_item.get("anchorXY")
            merged_sweep_xy = _collect_group_sweep_rows_xy(
                group=group,
                all_sweep_lines_xy=scan_lines_xy,
                previous_rep_sweep_idx=prev_rep_sweep_idx,
            )
            merged_sweep_coords = _xy_rows_to_coords_with_dem_altitude(merged_sweep_xy)
            if len(merged_sweep_coords) < 2:
                continue
            anchor_xy_pair = (
                (float(anchor_xy[0]), float(anchor_xy[1]))
                if isinstance(anchor_xy, (tuple, list)) and len(anchor_xy) >= 2
                else None
            )
            if line_search_wp_index == 0 and line_ingress_guard_xy is not None:
                anchor_xy_pair = line_ingress_guard_xy
            elif not line_sweep_items_regenerated_from_scan:
                # A selected LINE item can represent the end of a packed sweep
                # segment. Anchor the WP at the first sweep actually emitted in
                # coordinateList so the route offset stays visually/physically
                # tied to that segment's start.
                segment_anchor_xy = _line_group_start_anchor_xy(
                    path_row=path_row,
                    group=group,
                    all_sweep_lines_xy=scan_lines_xy,
                    previous_rep_sweep_idx=prev_rep_sweep_idx,
                    offset_m=float(route_offset_m),
                    reference_anchor_xy=entry_xy if entry_xy is not None else anchor_xy_pair,
                )
                if segment_anchor_xy is not None:
                    anchor_xy_pair = segment_anchor_xy
            if anchor_xy_pair is None:
                continue
            anchor_coord = _xy_to_coord_with_altitude(anchor_xy_pair, altitude_fn)
            speed_reference_coord = (
                entry_coord
                if is_line_mission and line_search_wp_index == 0 and isinstance(entry_coord, dict)
                else None
            )
            reference_xy = coord_to_xy(speed_reference_coord) if isinstance(speed_reference_coord, dict) else None
            prev_xy_for_speed = (
                reference_xy
                if reference_xy is not None
                else prev_xy
            )
            line_search_speed_mps = _estimate_line_search_speed_xy_mps(
                prev_xy=prev_xy_for_speed,
                anchor_xy=anchor_xy_pair,
                sweep_xy=merged_sweep_xy,
                cruise_speed_mps=float(search_speed_cruise_mps),
                fallback_search_speed_mps=float(search_speed_mps),
                speed_scale=float(geometry_search_speed_scale),
                reference_xy=reference_xy,
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
            prev_xy = anchor_xy_pair
            rep_sweep_idx = _to_float(rep_item.get("sweepIndex"))
            if rep_sweep_idx is not None:
                prev_rep_sweep_idx = int(rep_sweep_idx)
    elif flattened_sweep_xy and isinstance(end_xy, (tuple, list)) and len(end_xy) >= 2:
        line_search_anchor_xy = line_ingress_guard_xy
        if line_search_anchor_xy is None and is_line_mission:
            route_offset_m = _line_route_offset_m(path_row)
            reference_xy = coord_to_xy(entry_coord) if isinstance(entry_coord, dict) else None
            anchor_sweep_xy: Sequence[Tuple[float, float]] = []
            if scan_lines_xy and len(scan_lines_xy[0]) >= 2:
                anchor_sweep_xy = scan_lines_xy[0]
            else:
                first_sweep_points = max(2, int(_next_collab_sweep_points_per_leg()))
                anchor_sweep_xy = flattened_sweep_xy[:first_sweep_points]
            line_search_anchor_xy = _line_anchor_xy_for_path_row(
                path_row,
                anchor_sweep_xy,
                offset_m=float(route_offset_m),
                reference_xy=reference_xy,
                scan_lines_xy=scan_lines_xy,
            )
        if line_search_anchor_xy is None:
            line_search_anchor_xy = (float(end_xy[0]), float(end_xy[1]))
        line_search_anchor_coord = _xy_to_coord_with_altitude(line_search_anchor_xy, altitude_fn)
        sweep_coords = _xy_rows_to_coords_with_dem_altitude(flattened_sweep_xy)
        prev_coord = None
        if waypoints and isinstance(waypoints[-1], dict):
            prev_coord = waypoints[-1].get("coordinate")
        if not isinstance(prev_coord, dict):
            prev_coord = entry_coord
        speed_reference_coord = entry_coord if is_line_mission and isinstance(entry_coord, dict) else None
        reference_xy = coord_to_xy(speed_reference_coord) if isinstance(speed_reference_coord, dict) else None
        prev_xy_for_speed = (
            reference_xy
            if reference_xy is not None
            else (coord_to_xy(prev_coord) if isinstance(prev_coord, dict) else None)
        )
        line_search_speed_mps = _estimate_line_search_speed_xy_mps(
            prev_xy=prev_xy_for_speed,
            anchor_xy=line_search_anchor_xy,
            sweep_xy=flattened_sweep_xy,
            cruise_speed_mps=float(search_speed_cruise_mps),
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
            reference_xy=reference_xy,
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
        line_squash_started = time.perf_counter()
        if line_sweep_items_regenerated_from_scan:
            line_reanchor_count = 0
        else:
            line_reanchor_count = _reanchor_line_search_waypoints_to_first_sweep(
                final_waypoints,
                offset_m=float(_line_route_offset_m(path_row)),
                altitude_fn=altitude_fn,
                reference_xy_for_offset=entry_xy,
                path_row=path_row,
                scan_lines_xy=scan_lines_xy,
            )
        metrics["lineReanchoredWaypoints"] = int(line_reanchor_count)
        pruned_line_waypoints = _simplify_line_waypoints_to_start_and_search(
            final_waypoints,
            speed_mps=float(transit_speed_mps),
            sensor_type=int(sensor_type),
            field_of_view_deg=float(field_of_view_deg),
        )
        metrics["linePrunedAnchorWaypoints"] = int(pruned_line_waypoints)
        snapped_endpoint_waypoints = _snap_last_line_search_waypoint_to_route_endpoint(
            final_waypoints,
            path_row=path_row,
            scan_lines_xy=scan_lines_xy,
            altitude_fn=altitude_fn,
        )
        metrics["lineEndpointAnchorsSnapped"] = int(snapped_endpoint_waypoints)
        _recompute_first_line_search_speed_from_entry_inplace(
            final_waypoints,
            entry_coord=entry_coord if isinstance(entry_coord, dict) else None,
            transit_speed_mps=float(search_speed_cruise_mps),
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
        )
        metrics["lineSquashMs"] = _elapsed_ms(line_squash_started)
        _clamp_line_waypoint_fov_inplace(final_waypoints)
    is_done_started = time.perf_counter()
    for waypoint in final_waypoints:
        if isinstance(waypoint, dict):
            waypoint["isDone"] = False
    metrics["postIsDoneMs"] = _elapsed_ms(is_done_started)
    altitude_post_started = time.perf_counter()
    step_started = time.perf_counter()
    _apply_legacy_altitude_profile_to_waypoints(
        final_waypoints,
        aircraft_id=int(aircraft_id),
        mission_info=mission_info,
    )
    metrics["legacyAltitudeMs"] = _elapsed_ms(step_started)
    step_started = time.perf_counter()
    _preserve_first_waypoint_altitude_from_entry(
        final_waypoints,
        entry_coord=entry_coord,
    )
    metrics["entryAltitudePreserveMs"] = _elapsed_ms(step_started)
    step_started = time.perf_counter()
    _enforce_waypoint_altitude_rate_limit_inplace(
        final_waypoints,
        default_speed_mps=float(transit_speed_mps),
    )
    metrics["rateLimitFirstMs"] = _elapsed_ms(step_started)
    step_started = time.perf_counter()
    _stabilize_entry_transition_altitude_inplace(
        final_waypoints,
        entry_coord=entry_coord,
        default_speed_mps=float(transit_speed_mps),
    )
    metrics["stabilizeFirstMs"] = _elapsed_ms(step_started)
    step_started = time.perf_counter()
    normalize_filming_target_altitudes_in_waypoints(
        final_waypoints,
        dem_lookup_many=_dem_altitudes_for_pairs_cached,
    )
    metrics["filmingTargetNormalizeMs"] = _elapsed_ms(step_started)
    step_started = time.perf_counter()
    _enforce_filming_target_altitude_floor_inplace(final_waypoints)
    metrics["filmingTargetFloorMs"] = _elapsed_ms(step_started)
    step_started = time.perf_counter()
    _enforce_waypoint_altitude_rate_limit_inplace(
        final_waypoints,
        default_speed_mps=float(transit_speed_mps),
    )
    metrics["rateLimitSecondMs"] = _elapsed_ms(step_started)
    step_started = time.perf_counter()
    _stabilize_entry_transition_altitude_inplace(
        final_waypoints,
        entry_coord=entry_coord,
        default_speed_mps=float(transit_speed_mps),
    )
    metrics["stabilizeSecondMs"] = _elapsed_ms(step_started)
    metrics["postAltitudeMs"] = _elapsed_ms(altitude_post_started)
    waypoint_id_started = time.perf_counter()
    reassign_unique_waypoint_ids_inplace(
        final_waypoints,
        waypoint_id_provider=waypoint_id_provider,
    )
    metrics["waypointIdMs"] = _elapsed_ms(waypoint_id_started)
    timeline_started = time.perf_counter()
    _recompute_waypoint_timeline(final_waypoints, default_speed_mps=float(transit_speed_mps))
    metrics["timelineMs"] = _elapsed_ms(timeline_started)
    flyover_started = time.perf_counter()
    _apply_runtime_flyover_to_waypoints(final_waypoints)
    metrics["runtimeFlyoverMs"] = _elapsed_ms(flyover_started)
    if bool(flight_path.get("isFormationFlight", False)):
        strip_started = time.perf_counter()
        _strip_filming_properties(final_waypoints)
        metrics["formationStripMs"] = _elapsed_ms(strip_started)
    flight_path["waypointList"] = final_waypoints
    if "lahWaypointList" in flight_path:
        lah_copy_started = time.perf_counter()
        flight_path["lahWaypointList"] = deepcopy(final_waypoints)
        metrics["lahCopyMs"] = _elapsed_ms(lah_copy_started)
    line_search_waypoints = 0
    line_search_coords = 0
    for waypoint in final_waypoints:
        if not isinstance(waypoint, dict):
            continue
        coords = _line_search_coordinate_list(waypoint)
        if coords:
            line_search_waypoints += 1
            line_search_coords += len(coords)
    metrics["waypoints"] = len(final_waypoints)
    metrics["lineSearchWaypoints"] = int(line_search_waypoints)
    metrics["lineSearchCoords"] = int(line_search_coords)
    metrics["buildTotalMs"] = _elapsed_ms(build_started)
    if metrics_callback is not None:
        try:
            metrics_callback(dict(metrics))
        except Exception:
            pass
    return flight_path
