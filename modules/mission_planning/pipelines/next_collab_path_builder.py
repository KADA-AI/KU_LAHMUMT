from __future__ import annotations

import hashlib
import math
import os
import threading
import time
from copy import deepcopy
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from shapely.geometry import GeometryCollection, LineString, MultiLineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union

from modules.mission_planning.pipelines.mission_path_trim import (
    reassign_unique_waypoint_ids_inplace,
)
from modules.mission_planning.pipelines.line_search_speed_guard import (
    clamp_line_search_speed_mps,
    effective_line_search_transit_m,
    line_search_speed_min_transit_m,
)
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    normalize_filming_target_altitudes_in_waypoints,
)
from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
    terrain_data_signature,
    terrain_elev,
    terrain_elev_many,
)
from modules.mission_planning.runtime.line_search_geometry_cache import (
    get_or_build_line_search_coords,
)
from modules.mission_planning.runtime.line_search_size_estimate import (
    estimate_from_metrics,
)
from modules.mission_planning.pipelines.handover_terminal import (
    CONTROL_TRANSFER_FOV_DEG,
    control_transfer_route_coordinates,
    is_control_transfer_direct_mission,
)
from modules.common.turn_dynamics import interpolate_reference_turn_radius
from modules.common.turn_profile import reference_turn_radius_scale_for_aircraft
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        MIN_LINE_FOV_DEG,
        apply_runtime_camera_adjusted_fov_deg,
        get_runtime_manual_fov_deg,
        get_runtime_manual_fov_sync_active,
        get_runtime_altitude_layers_m,
        get_runtime_bool,
        get_runtime_camera_adjust_fov_scale,
        get_runtime_fov_db_path,
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
        get_runtime_camera_adjust_fov_scale,
        get_runtime_fov_db_path,
        get_runtime_float,
        get_runtime_int,
        load_fov_db_rows,
        load_runtime_flyover,
    )
from modules.mission_planning.MissionPlanner import capture_physics
from modules.mission_planning.planners.next_collab_division._geo_utils import (
    coord_to_xy,
    local_xy_to_llh,
)

_LEGACY_ALTITUDE_HELPERS: Dict[str, Any] | None = None
_LEGACY_ALTITUDE_HELPERS_LOADED = False
ALTITUDE_LAYERS_M = (1000.0, 1010.0, 1020.0)
UAV_CLIMB_RATE_MPS = 5.0
UAV_DESCENT_RATE_MPS = 7.0
UAV_MIN_FORWARD_SPEED_MPS = 30.0
AREA_SEARCH_SPEED_MAX_MPS = 800.0
OPMODE_POINT = 1
PASS_FLYBY = 1
PASS_LOITER = 2
PASS_FLYOVER = 3
_HANDOVER_TERMINAL_WAYPOINT_MARKER = "_handoverTerminalWaypoint"
NEXT_COLLAB_SWEEP_POINTS_PER_LEG = 3
NEXT_COLLAB_LINE_ROUTE_WP_SPACING_M = 2000.0
NEXT_COLLAB_FIRST_LINE_FOV_SCALE = 1.35
NEXT_COLLAB_FIRST_LINE_FOV_MAX_DEG = 15.4
NEXT_COLLAB_LINE_INGRESS_ENTRY_TRIGGER_SCALE = 1.25
NEXT_COLLAB_LINE_INGRESS_ENTRY_MIN_TRIGGER_M = 250.0
ENTRY_ALTITUDE_MIN_M = 1.0
ENTRY_ALTITUDE_MAX_PRESERVE_DELTA_M = 600.0
FILMING_TARGET_ALTITUDE_FLOOR_CLEARANCE_M = 30.0
_DEM_ALT_CACHE_MAX = 200_000
_DEM_ALT_CACHE_LOCK = threading.Lock()
_DEM_ALT_CACHE: Dict[Tuple[float, float], float] = {}
_GROUND_REQUIRED_COORDS_CACHE_MAX = 50_000
_GROUND_REQUIRED_COORDS_CACHE_LOCK = threading.Lock()
_GROUND_REQUIRED_COORDS_CACHE: Dict[Tuple[Tuple[float, float], ...], float] = {}
_FOV_MIN_SEP_CACHE_LOCK = threading.Lock()
_FOV_MIN_SEP_CACHE: Dict[Tuple[Any, ...], float] = {}
_LINE_SEARCH_GEOMETRY_NAMESPACE_LOCK = threading.Lock()
_LINE_SEARCH_GEOMETRY_NAMESPACE: Tuple[Tuple[Tuple[str, int, int], ...], str] | None = None


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


def _line_search_geometry_cache_namespace() -> str:
    global _LINE_SEARCH_GEOMETRY_NAMESPACE
    try:
        terrain_sig = tuple(terrain_data_signature())
    except Exception:
        terrain_sig = tuple()
    with _LINE_SEARCH_GEOMETRY_NAMESPACE_LOCK:
        cached = _LINE_SEARCH_GEOMETRY_NAMESPACE
        if cached is not None and cached[0] == terrain_sig:
            return cached[1]
        digest = hashlib.sha256()
        for path, mtime_ns, size in terrain_sig:
            digest.update(str(path).encode("utf-8", errors="ignore"))
            digest.update(b":")
            digest.update(str(int(mtime_ns)).encode("ascii", errors="ignore"))
            digest.update(b":")
            digest.update(str(int(size)).encode("ascii", errors="ignore"))
            digest.update(b";")
        namespace = f"next_collab_path_builder.dem_altitude.v2:{digest.hexdigest()[:16]}"
        _LINE_SEARCH_GEOMETRY_NAMESPACE = (terrain_sig, namespace)
        return namespace


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
        camera_adjust_scale = float(get_runtime_camera_adjust_fov_scale())
    except Exception:
        camera_adjust_scale = 1.0

    cache_key: Tuple[Any, ...] | None = None
    try:
        db_path = Path(get_runtime_fov_db_path()).resolve()
        db_stat = db_path.stat()
        cache_key = (
            round(float(target_fov), 6),
            "file",
            str(db_path),
            int(db_stat.st_mtime_ns),
            int(db_stat.st_size),
            round(float(camera_adjust_scale), 6),
            float(MIN_LINE_FOV_DEG),
        )
    except Exception:
        cache_key = None

    if cache_key is not None:
        with _FOV_MIN_SEP_CACHE_LOCK:
            cached = _FOV_MIN_SEP_CACHE.get(cache_key)
        if cached is not None:
            return float(cached)

    try:
        rows = load_fov_db_rows()
    except Exception:
        rows = []
    if not rows:
        return 0.0

    if cache_key is None:
        row_digest = hashlib.sha256()
        for row in rows:
            row_digest.update(float(_to_float(row.get("fov")) or 0.0).hex().encode("ascii"))
            row_digest.update(b",")
            row_digest.update(float(_to_float(row.get("sep")) or 0.0).hex().encode("ascii"))
            row_digest.update(b";")
        cache_key = (
            round(float(target_fov), 6),
            "rows",
            row_digest.hexdigest(),
            round(float(camera_adjust_scale), 6),
            float(MIN_LINE_FOV_DEG),
        )
        with _FOV_MIN_SEP_CACHE_LOCK:
            cached = _FOV_MIN_SEP_CACHE.get(cache_key)
        if cached is not None:
            return float(cached)

    matches: List[float] = []
    for row in rows:
        row_fov = _to_float(row.get("fov"))
        row_sep = _to_float(row.get("sep"))
        if row_fov is None or row_fov <= 0.0 or row_sep is None or row_sep <= 0.0:
            continue
        if abs(float(row_fov) - float(target_fov)) <= 0.05:
            matches.append(float(row_sep))

    adjustment_can_change_match = (
        abs(float(camera_adjust_scale) - 1.0) > 1e-9
        or abs(float(target_fov) - float(MIN_LINE_FOV_DEG)) <= 0.05
    )
    if not matches and adjustment_can_change_match:
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

    result = min(matches) if matches else 0.0
    with _FOV_MIN_SEP_CACHE_LOCK:
        _FOV_MIN_SEP_CACHE[cache_key] = float(result)
        if len(_FOV_MIN_SEP_CACHE) > 256:
            for oldest_key in list(_FOV_MIN_SEP_CACHE.keys())[:64]:
                _FOV_MIN_SEP_CACHE.pop(oldest_key, None)
    return float(result)


def _route_offset_sep_for_fov(fov_deg: Any, default_sep_m: Any) -> float:
    default_sep = _to_float(default_sep_m) or 0.0
    # 물리 우선: 측정/기존 이격을 그대로 쓰되 선택 FOV의 GSD 한계로만 자른다.
    try:
        physics_sep = capture_physics.physics_route_offset_cap_m(
            _to_float(fov_deg) or 0.0, default_sep
        )
    except Exception:
        physics_sep = 0.0
    if physics_sep > 0.0:
        return float(physics_sep)
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


def _prewarm_dem_altitudes_for_path_rows_if_enabled(
    path_rows: Iterable[Dict[str, Any]],
) -> Dict[str, Any]:
    raw_enabled = os.environ.get("REPLAN_NEXT_COLLAB_DEM_PREWARM")
    if raw_enabled is not None and str(raw_enabled).strip().lower() in {"0", "false", "no", "off"}:
        return {"xyPoints": 0, "uniquePairs": 0, "elapsedMs": 0.0, "skipped": True}
    return prewarm_dem_altitudes_for_path_rows(path_rows)


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
        locked_turn_altitude_m = _to_float(
            waypoint.get("_locked_area_reciprocal_turn_altitude_m")
        )
        if locked_turn_altitude_m is not None:
            waypoint["coordinate"] = {
                "latitude": round(float(lat), 6),
                "longitude": round(float(lon), 6),
                "altitude": int(round(float(locked_turn_altitude_m))),
            }
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


def _enforce_area_reciprocal_turn_contract_inplace(
    waypoints: List[Dict[str, Any]],
) -> Tuple[int, float | None]:
    """Keep reciprocal turn gates camera-ready, fly-by, and level."""
    turn_waypoints = [
        waypoint
        for waypoint in waypoints or []
        if isinstance(waypoint, dict)
        and (
            waypoint.get("_locked_area_reciprocal_turn")
            or waypoint.get("areaTurnRole") == "reciprocal_turn"
        )
    ]
    if not turn_waypoints:
        return 0, None
    altitude_candidates: List[float] = []
    for waypoint in turn_waypoints:
        coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else {}
        altitude = _to_float(coord.get("altitude"))
        locked_altitude = _to_float(
            waypoint.get("_locked_area_reciprocal_turn_altitude_m")
        )
        if altitude is not None:
            altitude_candidates.append(float(altitude))
        if locked_altitude is not None:
            altitude_candidates.append(float(locked_altitude))
    if not altitude_candidates:
        return len(turn_waypoints), None
    corridor_altitude_m = float(max(altitude_candidates))
    for waypoint in turn_waypoints:
        coord = waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else None
        if isinstance(coord, dict):
            lat = _to_float(coord.get("latitude"))
            lon = _to_float(coord.get("longitude"))
            if lat is not None and lon is not None:
                waypoint["coordinate"] = {
                    "latitude": round(float(lat), 6),
                    "longitude": round(float(lon), 6),
                    "altitude": int(round(corridor_altitude_m)),
                }
        waypoint["waypointPassType"] = PASS_FLYBY
        locked_speed_mps = _to_float(
            waypoint.get("_locked_area_reciprocal_turn_speed_mps")
        )
        if locked_speed_mps is not None and locked_speed_mps > 0.0:
            waypoint["speed"] = round(float(locked_speed_mps), 2)
        waypoint["_locked_area_reciprocal_turn"] = True
        waypoint["_locked_area_reciprocal_turn_altitude_m"] = corridor_altitude_m
    return len(turn_waypoints), corridor_altitude_m


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


def _runtime_uav_descent_rate_mps() -> float:
    try:
        value = float(get_runtime_float("uav_descent_rate_mps", UAV_DESCENT_RATE_MPS))
    except Exception:
        value = float(UAV_DESCENT_RATE_MPS)
    return max(value, 0.1)


def _runtime_area_search_speed_max_mps() -> float:
    try:
        return float(get_runtime_float("area_search_speed_max_mps", AREA_SEARCH_SPEED_MAX_MPS))
    except Exception:
        return float(AREA_SEARCH_SPEED_MAX_MPS)


def _apply_area_scan_rate_slowdown(
    *,
    estimated_search_speed_mps: float,
    transit_speed_mps: float,
) -> Tuple[float, float]:
    """Slow the leg flight speed when the synced scan rate is above target.

    Wide area sections pack more strip per meter of route, which pushes the
    leg-synced scan rate up. Instead of scanning faster, fly slower (down to
    the forward-speed floor) and re-sync the scan rate to the slowed leg.
    The scan estimate is linear in cruise speed, so the re-synced rate is
    estimate * slowed / transit.  The caller already applies the Area 1.10
    completion margin before this function.  Keeping a second pre-slowdown
    floor made the camera finish early while the aircraft was still flying the
    capture leg, so the returned search rate is always synchronized to the
    commanded leg speed.  Returns (search_speed_mps, leg_speed_mps).
    """
    est = max(0.0, float(estimated_search_speed_mps))
    transit = max(1.0, float(transit_speed_mps))
    target = _runtime_area_search_speed_max_mps()
    if target <= 0.0 or est <= target:
        return est, transit
    slowed = transit * (float(target) / est)
    slowed = max(float(_runtime_uav_min_forward_speed_mps()), float(slowed))
    slowed = min(float(slowed), transit)
    if slowed >= transit - 1e-6:
        return est, transit
    return est * (float(slowed) / transit), float(slowed)


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


def _is_speed_locked_waypoint(waypoint: Dict[str, Any]) -> bool:
    """촬영 계약 속도를 갖는 WP — 상승률 확보 목적으로도 감속하면 안 된다.

    line/area 촬영 속도는 촬영 간격과 함께 capture 법칙으로 묶여 있어 여기서
    낮추면 촬영 간격이 틀어진다. 전이(transit) WP만 감속 대상으로 삼는다.
    """
    if not isinstance(waypoint, dict):
        return True
    if waypoint.get("_locked_area_reciprocal_turn"):
        return True
    if _has_line_search_coordinates(waypoint):
        return True
    return _is_point_hold_waypoint(waypoint)


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
    min_forward_speed_mps = max(1.0, float(_runtime_uav_min_forward_speed_mps()))

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

    def _leg_speed_mps(next_wp: Dict[str, Any]) -> float:
        speed_raw = _to_float(next_wp.get("speed"))
        return max(float(speed_raw), 1.0) if speed_raw is not None else fallback_speed_mps

    def _leg_floor_speed_mps(next_wp: Dict[str, Any]) -> float:
        """상승 용량 산정에 쓰는, 해당 레그가 감속할 수 있는 최저 속도.

        전이 WP는 최소 전진속도까지 감속해 레그당 상승 여유를 키울 수 있고,
        촬영 WP는 계약 속도를 유지해야 하므로 현재 속도가 곧 바닥이다.
        """
        current = _leg_speed_mps(next_wp)
        if _is_speed_locked_waypoint(next_wp):
            return current
        return min(current, min_forward_speed_mps)

    mutable_alts = [float(item[3]) for item in items]
    # The first waypoint is pinned to the aircraft's entry (0401) altitude by
    # the preserve/stabilize passes — never raise it here; its leg absorbs the
    # remaining altitude delta. 상승 용량은 감속 가능 바닥 속도 기준으로 계산해
    # 선행 WP를 과도하게 끌어올리지 않는다(잔여 상승이 첫 레그에 몰려 달성
    # 불가능한 급상승 구간이 생기던 문제 완화). 실제 감속은 아래 속도 패스가
    # 최종 고도차에 맞춰 반영한다.
    for pos in range(len(items) - 2, 0, -1):
        prev_wp, prev_lat, prev_lon, _ = items[pos]
        next_wp, next_lat, next_lon, _ = items[pos + 1]
        seg_dist_m = _distance_between_coords_m(
            {"latitude": prev_lat, "longitude": prev_lon},
            {"latitude": next_lat, "longitude": next_lon},
        )
        if seg_dist_m <= 0.0:
            continue
        allowed_delta_m = rate_mps * (float(seg_dist_m) / _leg_floor_speed_mps(next_wp))
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

    # Forward pass: cap descent transitions by raising the later waypoint.
    # Raise-only, so terrain/filming altitude floors set upstream stay intact.
    # Skip the first segment: the entry-pinned first waypoint must not push
    # mission altitudes upward.
    descent_rate_mps = _runtime_uav_descent_rate_mps() if vertical_rate_mps is None else max(float(vertical_rate_mps), 0.1)
    for pos in range(2, len(items)):
        _, prev_lat, prev_lon, _ = items[pos - 1]
        next_wp, next_lat, next_lon, _ = items[pos]
        seg_dist_m = _distance_between_coords_m(
            {"latitude": prev_lat, "longitude": prev_lon},
            {"latitude": next_lat, "longitude": next_lon},
        )
        if seg_dist_m <= 0.0:
            continue
        allowed_delta_m = descent_rate_mps * (float(seg_dist_m) / _leg_floor_speed_mps(next_wp))
        required_next_alt = float(mutable_alts[pos - 1]) - float(allowed_delta_m)
        if float(mutable_alts[pos]) >= required_next_alt:
            continue
        new_alt = int(math.ceil(required_next_alt))
        mutable_alts[pos] = float(new_alt)
        next_wp["coordinate"] = {
            "latitude": round(float(next_lat), 6),
            "longitude": round(float(next_lon), 6),
            "altitude": int(new_alt),
        }

    # 속도 패스: 최종 고도 프로파일 기준으로 상승/강하율이 초과되는 레그의
    # 전이 WP 속도를 실제로 낮춰(최소 전진속도 바닥) 요구 수직률을 기체
    # 상승률 이내로 맞춘다. 첫 레그(진입 고도 고정 → WP1)의 잔여 상승도
    # 여기서 최대한 흡수한다.
    for pos in range(1, len(items)):
        next_wp, next_lat, next_lon, _ = items[pos]
        _, prev_lat, prev_lon, _ = items[pos - 1]
        if _is_speed_locked_waypoint(next_wp):
            continue
        seg_dist_m = _distance_between_coords_m(
            {"latitude": prev_lat, "longitude": prev_lon},
            {"latitude": next_lat, "longitude": next_lon},
        )
        if seg_dist_m <= 0.0:
            continue
        delta_alt_m = float(mutable_alts[pos]) - float(mutable_alts[pos - 1])
        if abs(delta_alt_m) <= 1e-6:
            continue
        vertical_limit_mps = rate_mps if delta_alt_m > 0.0 else descent_rate_mps
        allowed_speed_mps = vertical_limit_mps * (float(seg_dist_m) / abs(delta_alt_m))
        current_speed_mps = _leg_speed_mps(next_wp)
        if allowed_speed_mps >= current_speed_mps - 1e-6:
            continue
        new_speed_mps = max(float(allowed_speed_mps), min(current_speed_mps, min_forward_speed_mps))
        if new_speed_mps >= current_speed_mps - 1e-6:
            continue
        next_wp["speed"] = round(float(new_speed_mps), 2)


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
    if not _is_point_hold_waypoint(first_wp) and not _has_line_search_coordinates(first_wp):
        return
    first_coord = first_wp.get("coordinate") if isinstance(first_wp.get("coordinate"), dict) else None
    if not isinstance(first_coord, dict):
        return

    entry_alt = _to_float(entry_coord.get("altitude"))
    first_alt = _to_float(first_coord.get("altitude"))
    if entry_alt is None or first_alt is None:
        return
    if float(entry_alt) < float(ENTRY_ALTITUDE_MIN_M):
        return

    rate_mps = _runtime_uav_climb_rate_mps()
    # Pin the first waypoint to the aircraft's entry (0401) altitude in both
    # directions so the mission starts where the aircraft actually is; the
    # first leg absorbs the transition toward the mission altitude. The delta
    # guard only rejects implausible telemetry.
    if (
        abs(float(first_alt) - float(entry_alt)) >= 1.0
        and abs(float(first_alt) - float(entry_alt)) <= float(ENTRY_ALTITUDE_MAX_PRESERVE_DELTA_M)
    ):
        first_alt = float(math.floor(float(entry_alt)))
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
    if next_alt is None:
        return

    segment_distance_m = _distance_between_coords_m(first_coord, next_coord)
    if segment_distance_m <= 0.0:
        return
    current_next_speed = max(
        _to_float(next_wp.get("speed")) or _to_float(first_wp.get("speed")) or float(default_speed_mps),
        1.0,
    )

    # Slow the next waypoint when the climb toward it is too steep.
    if float(next_alt) > float(first_alt):
        required_speed_mps = rate_mps * (segment_distance_m / max(float(next_alt) - float(first_alt), 1.0))
        if 0.0 < required_speed_mps < current_next_speed:
            next_wp["speed"] = round(max(float(required_speed_mps), _runtime_uav_min_forward_speed_mps()), 2)


def _drop_redundant_area_entry_waypoint_before_sweep(
    waypoints: List[Dict[str, Any]],
    grouped_area_sweeps: Sequence[Sequence[Dict[str, Any]]],
    *,
    max_distance_m: float = 25.0,
) -> int:
    if not waypoints or not grouped_area_sweeps:
        return 0
    last_wp = waypoints[-1] if isinstance(waypoints[-1], dict) else None
    if not isinstance(last_wp, dict):
        return 0
    if not bool(last_wp.get("_flyover_dubins_prefix")):
        return 0
    last_xy = _waypoint_coordinate_xy(last_wp)
    if last_xy is None:
        return 0
    first_group = grouped_area_sweeps[0] if grouped_area_sweeps else []
    first_item = first_group[-1] if first_group else {}
    first_anchor_raw = first_item.get("anchorXY") if isinstance(first_item, dict) else None
    if not (isinstance(first_anchor_raw, (tuple, list)) and len(first_anchor_raw) >= 2):
        return 0
    first_anchor_xy = (float(first_anchor_raw[0]), float(first_anchor_raw[1]))
    if _distance_xy(last_xy, first_anchor_xy) > float(max_distance_m):
        return 0
    waypoints.pop()
    return 1


def _legacy_altitude_helpers() -> Dict[str, Any]:
    global _LEGACY_ALTITUDE_HELPERS
    global _LEGACY_ALTITUDE_HELPERS_LOADED
    if _LEGACY_ALTITUDE_HELPERS_LOADED:
        return _LEGACY_ALTITUDE_HELPERS or {}
    helpers = {
        "aircraft_alt_offset_m": _aircraft_alt_offset_m,
        "align_point_anchor_altitude_with_search_waypoints": _align_point_anchor_altitude_with_search_waypoints,
        "apply_segment_altitude_to_search_waypoints": _apply_segment_altitude_to_search_waypoints,
        "collect_ref_points_from_info": _collect_ref_points_from_info,
        "dem_alt": _dem_alt,
        "enforce_waypoint_altitude_rate_limit_inplace": _enforce_waypoint_altitude_rate_limit_inplace,
        "median_ground_m": _median_ground_m,
    }
    # Publish the fully built mapping before the loaded flag.  LINE replacement
    # paths can enter here concurrently, and no worker may observe a transient
    # loaded=True/empty-mapping state.
    _LEGACY_ALTITUDE_HELPERS = helpers
    _LEGACY_ALTITUDE_HELPERS_LOADED = True
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
    if isinstance(info.get("lineDeploymentCoordinateList"), list):
        info["lineDeploymentCoordinateList"] = _apply_altitude_to_coord_list(
            list(info.get("lineDeploymentCoordinateList") or []),
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
    is_line_search_waypoint = _has_line_search_coordinates(first_waypoint)
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
            or abs(float(entry_altitude) - float(current_altitude)) > float(ENTRY_ALTITUDE_MAX_PRESERVE_DELTA_M)
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


def _line_route_projection_context(
    route_line_xy: Sequence[Tuple[float, float]],
) -> Dict[str, Any] | None:
    rows = _dedupe_xy_rows(_xy_rows(route_line_xy), eps_m=0.5)
    if len(rows) < 2:
        return None

    total_len_m = _line_length_xy(rows)
    if total_len_m <= 1e-6:
        return None

    segments: List[Tuple[float, float, float, float, float, float, float]] = []
    walked_m = 0.0
    for idx in range(len(rows) - 1):
        sx, sy = float(rows[idx][0]), float(rows[idx][1])
        ex, ey = float(rows[idx + 1][0]), float(rows[idx + 1][1])
        dx = ex - sx
        dy = ey - sy
        seg_len_m = math.hypot(dx, dy)
        if seg_len_m <= 1e-6:
            continue
        denom = (dx * dx) + (dy * dy)
        segments.append((sx, sy, dx, dy, seg_len_m, denom, walked_m))
        walked_m += seg_len_m

    if not segments:
        return None
    return {
        "rows": rows,
        "segments": segments,
        "totalLenM": float(total_len_m),
    }


def _project_point_to_route_projection_context_xy(
    point_xy: Tuple[float, float],
    projection_context: Dict[str, Any] | None,
) -> tuple[Tuple[float, float], Tuple[float, float], float] | None:
    if not isinstance(projection_context, dict):
        return None
    segments = projection_context.get("segments")
    total_len_m = _to_float(projection_context.get("totalLenM"))
    if not isinstance(segments, list) or not segments or total_len_m is None or total_len_m <= 1e-6:
        return None

    px, py = float(point_xy[0]), float(point_xy[1])
    best: tuple[float, Tuple[float, float], Tuple[float, float], float] | None = None
    for segment in segments:
        if not (isinstance(segment, tuple) and len(segment) >= 7):
            continue
        sx, sy, dx, dy, seg_len_m, denom, walked_m = segment[:7]
        if float(seg_len_m) <= 1e-6:
            continue
        ratio = (((px - float(sx)) * float(dx)) + ((py - float(sy)) * float(dy))) / max(float(denom), 1e-9)
        ratio = max(0.0, min(1.0, float(ratio)))
        center_xy = (float(sx) + (float(dx) * ratio), float(sy) + (float(dy) * ratio))
        dist_m = _distance_xy((px, py), center_xy)
        tangent_xy = (float(dx) / float(seg_len_m), float(dy) / float(seg_len_m))
        progress_ratio = max(0.0, min(1.0, (float(walked_m) + (float(seg_len_m) * ratio)) / float(total_len_m)))
        if best is None or dist_m < best[0]:
            best = (float(dist_m), center_xy, tangent_xy, float(progress_ratio))

    if best is None:
        return None
    return best[1], best[2], best[3]


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
    return _project_point_to_route_projection_context_xy(
        point_xy,
        _line_route_projection_context(route_line_xy),
    )


def _line_anchor_xy_from_route_polyline(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    route_line_xy: Sequence[Tuple[float, float]],
    offset_m: float,
    reference_xy: Tuple[float, float] | None = None,
    start_side: float | None = None,
    end_side: float | None = None,
    projection_context: Dict[str, Any] | None = None,
) -> Tuple[float, float] | None:
    rows = _dedupe_xy_rows(_xy_rows(sweep_xy), eps_m=0.5)
    midpoint_xy = _midpoint_xy(rows)
    if midpoint_xy is None:
        return None
    projected = (
        _project_point_to_route_projection_context_xy(midpoint_xy, projection_context)
        if projection_context is not None
        else _project_point_to_route_polyline_xy(midpoint_xy, route_line_xy)
    )
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


def _line_anchor_context_for_path_row(
    path_row: Dict[str, Any],
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None = None,
) -> Dict[str, Any]:
    route_line_xy = _line_route_polyline_xy(path_row, scan_lines_xy)
    return {
        "sidePair": _line_route_offset_side_pair(path_row),
        "routeLineXY": route_line_xy,
        "routeProjectionContext": _line_route_projection_context(route_line_xy),
        "axisXY": _line_route_axis_xy(path_row, scan_lines_xy),
    }


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
    anchor_context: Dict[str, Any] | None = None,
) -> Tuple[float, float] | None:
    if anchor_context is not None:
        raw_side_pair = anchor_context.get("sidePair")
        side_pair = raw_side_pair if isinstance(raw_side_pair, tuple) and len(raw_side_pair) >= 2 else None
        raw_route_line_xy = anchor_context.get("routeLineXY")
        route_line_xy = raw_route_line_xy if isinstance(raw_route_line_xy, list) else []
        route_projection_context = anchor_context.get("routeProjectionContext")
        raw_axis_xy = anchor_context.get("axisXY")
        axis_xy = raw_axis_xy if isinstance(raw_axis_xy, tuple) and len(raw_axis_xy) >= 2 else None
    else:
        side_pair = _line_route_offset_side_pair(path_row)
        route_line_xy = _line_route_polyline_xy(path_row, scan_lines_xy)
        route_projection_context = None
        axis_xy = _line_route_axis_xy(path_row, scan_lines_xy)
    if len(route_line_xy) >= 2:
        anchor_xy = _line_anchor_xy_from_route_polyline(
            sweep_xy,
            route_line_xy=route_line_xy,
            offset_m=float(offset_m),
            reference_xy=reference_xy,
            start_side=side_pair[0] if side_pair is not None else None,
            end_side=side_pair[1] if side_pair is not None else None,
            projection_context=route_projection_context if isinstance(route_projection_context, dict) else None,
        )
        if anchor_xy is not None:
            return anchor_xy

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


def _has_line_search_coordinates(waypoint: Dict[str, Any]) -> bool:
    """Whether the waypoint carries at least one usable scan coordinate.

    Equivalent to ``bool(_line_search_coordinate_list(waypoint))`` but stops at
    the first valid coordinate instead of validating and rebuilding the whole
    list.  A scan waypoint can carry hundreds of coordinates and this predicate
    is evaluated per waypoint on every altitude/speed pass, so building a list
    only to test it for emptiness was the dominant cost of those passes.
    """

    if not isinstance(waypoint, dict):
        return False
    filming = waypoint.get("filmingProperty")
    if not isinstance(filming, dict):
        return False
    line_search = filming.get("lineSearch")
    if not isinstance(line_search, dict):
        return False
    coords = line_search.get("coordinateList")
    if not isinstance(coords, list):
        return False
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        if _to_float(coord.get("latitude")) is None:
            continue
        if _to_float(coord.get("longitude")) is None:
            continue
        return True
    return False


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
    anchor_context: Dict[str, Any] | None = None,
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
                anchor_context=anchor_context,
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
    return _to_int(filming.get("operationMode")) == OPMODE_POINT and not _has_line_search_coordinates(waypoint)


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
        if entry_op_mode != 1 or _has_line_search_coordinates(entry_wp):
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
                if isinstance(waypoint, dict) and _has_line_search_coordinates(waypoint)
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
            multiplier_cap_enabled=False,
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


def _line_search_coordinate_length_3d_m(
    coordinates: Sequence[Dict[str, Any]],
) -> float:
    """Measure a camera sweep the same way the SIM advances lineSearch.

    The executor converts latitude/longitude/altitude to local XYZ and uses
    ``math.dist``. Planning previously used only XY, which made rugged LINE
    sweeps finish after their carrier waypoint had already been reached.
    """

    total_m = 0.0
    previous_xy: Tuple[float, float] | None = None
    previous_altitude_m: float | None = None
    for coordinate in coordinates or []:
        if not isinstance(coordinate, dict):
            continue
        point_xy = coord_to_xy(coordinate)
        altitude_m = _to_float(coordinate.get("altitude"))
        if point_xy is None:
            continue
        if altitude_m is None:
            altitude_m = 0.0
        if previous_xy is not None and previous_altitude_m is not None:
            horizontal_m = _distance_xy(previous_xy, point_xy)
            total_m += math.hypot(
                float(horizontal_m),
                float(altitude_m) - float(previous_altitude_m),
            )
        previous_xy = point_xy
        previous_altitude_m = float(altitude_m)
    return float(total_m)


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
    if not _line_sweep_interpolation_enabled():
        return 2
    value = int(get_runtime_int("next_collab_sweep_points_per_leg", NEXT_COLLAB_SWEEP_POINTS_PER_LEG))
    return max(2, min(9, int(value)))


def _line_sweep_interpolation_enabled() -> bool:
    return bool(get_runtime_bool("line_sweep_interpolation_enabled", False))


def _next_collab_auto_sweep_points() -> bool:
    if not _line_sweep_interpolation_enabled():
        return False
    return bool(get_runtime_bool("next_collab_auto_sweep_points", False))


def _collapse_linesearch_midpoints_inplace(waypoints: List[Dict[str, Any]]) -> int:
    """Collapse valid interpolated sweep strips to their two endpoints."""
    if _line_sweep_interpolation_enabled():
        return 0
    removed = 0
    for waypoint in waypoints or []:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty") if isinstance(waypoint.get("filmingProperty"), dict) else {}
        line_search = filming.get("lineSearch") if isinstance(filming.get("lineSearch"), dict) else None
        if not isinstance(line_search, dict):
            continue
        coords = line_search.get("coordinateList")
        if not isinstance(coords, list) or len(coords) < 2:
            continue
        points = _to_int(line_search.get("interpolationPoints")) or 2
        if points <= 2:
            line_search["interpolationPoints"] = 2
            continue
        if len(coords) % int(points) != 0:
            continue
        collapsed: List[Dict[str, Any]] = []
        valid = True
        for offset in range(0, len(coords), int(points)):
            strip = coords[offset:offset + int(points)]
            if len(strip) != int(points) or not isinstance(strip[0], dict) or not isinstance(strip[-1], dict):
                valid = False
                break
            collapsed.extend((deepcopy(strip[0]), deepcopy(strip[-1])))
        if not valid:
            continue
        removed += max(0, len(coords) - len(collapsed))
        line_search["coordinateList"] = collapsed
        line_search["interpolationPoints"] = 2
    return int(removed)


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
    spacing_m = float(get_runtime_float("area_wp_interval_m", 1000.0))
    scale = float(get_runtime_float("area_first_packet_sweep_group_scale", 1.0))
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
        waypoint.pop("_locked_line_turn_prefix", None)
        waypoint.pop("_locked_area_reciprocal_turn", None)
        waypoint.pop("_locked_sequential_area_entry_turn", None)
        waypoint.pop("_locked_area_reciprocal_turn_altitude_m", None)
        waypoint.pop("_locked_area_reciprocal_turn_speed_mps", None)


def _force_handover_terminal_flyover(waypoints: List[Dict[str, Any]]) -> None:
    for waypoint in waypoints:
        if isinstance(waypoint, dict) and waypoint.get(_HANDOVER_TERMINAL_WAYPOINT_MARKER):
            waypoint["waypointPassType"] = PASS_FLYOVER


def _clear_handover_terminal_markers(waypoints: List[Dict[str, Any]]) -> None:
    for waypoint in waypoints:
        if isinstance(waypoint, dict):
            waypoint.pop(_HANDOVER_TERMINAL_WAYPOINT_MARKER, None)


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
    # A confidence-locked Line ingress is a flight-safety constraint, not a
    # visualization/runtime preference.  Its arc points must be crossed even
    # when the generic Dubins-prefix flyover option is disabled.
    for waypoint in waypoints:
        if not isinstance(waypoint, dict) or not waypoint.get("_locked_line_turn_prefix"):
            continue
        if int(_to_float(waypoint.get("waypointPassType")) or 0) != PASS_LOITER:
            waypoint["waypointPassType"] = PASS_FLYOVER
    if flyover.get("all_wps"):
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                continue
            if (
                waypoint.get("_locked_area_reciprocal_turn")
                or waypoint.get("_locked_sequential_area_entry_turn")
                or waypoint.get("areaTurnRole") == "reciprocal_turn"
            ):
                waypoint["waypointPassType"] = PASS_FLYBY
                continue
            if int(_to_float(waypoint.get("waypointPassType")) or 0) == PASS_LOITER:
                continue
            waypoint["waypointPassType"] = PASS_FLYOVER
        _force_handover_terminal_flyover(waypoints)
        _clear_runtime_flyover_markers(waypoints)
        return
    if flyover.get("entry_offset"):
        # Match the legacy/general planner semantics: promote only the
        # first collaborative-mission start waypoint.
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                continue
            if (
                waypoint.get("_locked_area_reciprocal_turn")
                or waypoint.get("_locked_sequential_area_entry_turn")
                or waypoint.get("areaTurnRole") == "reciprocal_turn"
            ):
                continue
            if int(_to_float(waypoint.get("waypointPassType")) or 0) == PASS_LOITER:
                continue
            waypoint["waypointPassType"] = PASS_FLYOVER
            break
    if flyover.get("dubins_prefix"):
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                continue
            if waypoint.get("_locked_sequential_area_entry_turn"):
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
            if (
                waypoint.get("_locked_area_reciprocal_turn")
                or waypoint.get("_locked_sequential_area_entry_turn")
                or waypoint.get("areaTurnRole") == "reciprocal_turn"
            ):
                continue
            if int(_to_float(waypoint.get("waypointPassType")) or 0) == PASS_LOITER:
                continue
            waypoint["waypointPassType"] = PASS_FLYOVER
            break
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        if (
            waypoint.get("_locked_area_reciprocal_turn")
            or waypoint.get("_locked_sequential_area_entry_turn")
            or waypoint.get("areaTurnRole") == "reciprocal_turn"
        ):
            waypoint["waypointPassType"] = PASS_FLYBY
    _force_handover_terminal_flyover(waypoints)
    _clear_runtime_flyover_markers(waypoints)


def _line_three_point_xy(points_xy: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    rows = _dedupe_xy_rows(points_xy, eps_m=0.5)
    if len(rows) < 2:
        return rows
    sample_count = _next_collab_sweep_points_for_line(rows)
    return _line_three_point_xy_from_rows(rows, sample_count)


def _line_three_point_xy_from_rows(
    rows: List[Tuple[float, float]],
    sample_count: int,
) -> List[Tuple[float, float]]:
    if len(rows) < 2:
        return rows
    sample_count = max(2, int(sample_count))
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


def _line_three_point_xy_with_settings(
    points_xy: List[Tuple[float, float]],
    *,
    auto_sweep_points: bool,
    points_per_leg: int,
    spacing_m: float,
) -> List[Tuple[float, float]]:
    rows = _dedupe_xy_rows(points_xy, eps_m=0.5)
    if len(rows) < 2:
        return rows
    if auto_sweep_points:
        length_m = _line_length_xy(rows)
        if length_m <= 1.0:
            sample_count = 2
        else:
            sample_count = max(2, min(9, int(math.ceil(float(length_m) / max(float(spacing_m), 1.0))) + 1))
    else:
        sample_count = int(points_per_leg)
    return _line_three_point_xy_from_rows(rows, sample_count)


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


def _locked_line_turn_prefix_xy(
    path_row: Dict[str, Any],
    *,
    entry_xy: Tuple[float, float] | None,
) -> List[Tuple[float, float]]:
    """Return the planner's observed-direction turn arc for a Line entry."""

    if not bool(path_row.get("lineTurnDirectionLocked")):
        return []
    expected_sign = _to_int(path_row.get("lineTurnDirectionSign")) or 0
    heading_deg = _to_float(path_row.get("lineEntryHeadingDeg"))
    if expected_sign == 0 or heading_deg is None:
        return []
    actual_entry_raw = path_row.get("lineEntryCurrentXY")
    if isinstance(actual_entry_raw, (tuple, list)) and len(actual_entry_raw) >= 2:
        try:
            entry_xy = (float(actual_entry_raw[0]), float(actual_entry_raw[1]))
        except Exception:
            pass
    marker_rows = path_row.get("markerRows") if isinstance(path_row.get("markerRows"), list) else []
    rows: List[Tuple[float, float]] = []
    for marker in marker_rows:
        if not isinstance(marker, dict):
            continue
        kind = str(marker.get("kind") or "").strip().lower()
        if kind not in {"turn", "tangent"}:
            if rows:
                break
            continue
        raw_xy = marker.get("xy")
        if not (isinstance(raw_xy, (tuple, list)) and len(raw_xy) >= 2):
            continue
        point_xy = (float(raw_xy[0]), float(raw_xy[1]))
        if entry_xy is not None and not rows and _distance_xy(entry_xy, point_xy) <= 3.0:
            continue
        if rows and _distance_xy(rows[-1], point_xy) <= 3.0:
            continue
        rows.append(point_xy)
        if kind == "tangent":
            break
    if not rows or entry_xy is None:
        return []

    theta = math.radians(float(heading_deg) % 360.0)
    heading_xy = (math.sin(theta), math.cos(theta))
    first_vector = (
        float(rows[0][0]) - float(entry_xy[0]),
        float(rows[0][1]) - float(entry_xy[1]),
    )
    first_norm = math.hypot(first_vector[0], first_vector[1])
    if first_norm <= 1e-6:
        return []
    cross = (
        float(heading_xy[0]) * (float(first_vector[1]) / first_norm)
        - float(heading_xy[1]) * (float(first_vector[0]) / first_norm)
    )
    first_turn_sign = 1 if cross > 0.02 else -1 if cross < -0.02 else 0
    if first_turn_sign != int(expected_sign):
        return []

    # Keep ICD size bounded on unusually long tangent searches while retaining
    # both the first continuation point and the final tangent point.
    if len(rows) > 6:
        selected_indices = sorted(
            {
                0,
                len(rows) - 1,
                *[
                    int(round(idx * (len(rows) - 1) / 5.0))
                    for idx in range(1, 5)
                ],
            }
        )
        rows = [rows[idx] for idx in selected_indices]
    return rows


def _flatten_sweep_lines_xy(scan_lines_xy: List[List[Tuple[float, float]]]) -> List[Tuple[float, float]]:
    merged: List[Tuple[float, float]] = []
    for line_xy in scan_lines_xy:
        if len(line_xy) < 2:
            continue
        for point_xy in _line_three_point_xy(line_xy):
            merged.append((float(point_xy[0]), float(point_xy[1])))
    return _dedupe_xy_rows(merged, eps_m=0.5)


def _flatten_sweep_three_point_rows_xy(
    sweep_rows_xy: List[List[Tuple[float, float]]],
) -> List[Tuple[float, float]]:
    merged: List[Tuple[float, float]] = []
    for rows in sweep_rows_xy:
        if len(rows) < 2:
            continue
        for point_xy in rows:
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


def _select_line_sweep_items_by_spacing(
    all_items: List[Dict[str, Any]],
    path_row: Dict[str, Any],
    *,
    reference_xy: Tuple[float, float] | None,
) -> List[Dict[str, Any]]:
    direction_locked = bool(path_row.get("lineDeploymentDirectionLocked"))
    if len(all_items) >= 2 and reference_xy is not None and not direction_locked:
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

    # Keep the route grouped at `spacing_m`, but absorb a sub-spacing tail into
    # the previous segment.  For example, 4760 m at 2000 m spacing becomes
    # 0 -> 2000 -> 4760, not 0 -> 2000 -> 4000 -> 4760.  This mirrors the
    # runtime LINE resampler and avoids a short final waypoint group.
    n_full = int(total_progress_m // float(spacing_m))
    target_distances_m = [
        float(spacing_m) * float(idx)
        for idx in range(1, n_full + 1)
        if (float(spacing_m) * float(idx)) < total_progress_m - 1e-6
    ]
    remainder_m = total_progress_m - (float(n_full) * float(spacing_m))
    if remainder_m > 1e-6 and target_distances_m:
        target_distances_m.pop()

    selected: List[Dict[str, Any]] = [dict(all_items[0])]
    selected[-1]["anchorXY"] = tuple(all_items[0]["anchorXY"])
    last_selected_idx = 0
    for target_m in target_distances_m:
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


def _line_sweep_items_from_scan_lines(
    path_row: Dict[str, Any],
    scan_lines_xy: List[List[Tuple[float, float]]],
    *,
    reference_xy: Tuple[float, float] | None,
    anchor_context: Dict[str, Any] | None = None,
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
            anchor_context=anchor_context,
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
    return _select_line_sweep_items_by_spacing(all_items, path_row, reference_xy=reference_xy)


def _normalize_line_sweep_item_anchors_to_route_axis(
    path_row: Dict[str, Any],
    items: List[Dict[str, Any]],
    *,
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None,
    reference_xy: Tuple[float, float] | None,
    anchor_context: Dict[str, Any] | None = None,
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
            anchor_context=anchor_context,
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


def _normalize_area_coverage_pass(value: Any) -> str | None:
    text = str(value or "").strip().lower()
    return text if text in {"forward", "reverse"} else None


def _area_pass_contract(
    path_row: Dict[str, Any],
    mission_info: Dict[str, Any] | None,
) -> Dict[str, Any]:
    """AREA reciprocal coverage is retired; all routes are single capture."""

    return {"explicit": False, "passes": [], "obligations": {}}

    sources = [
        source
        for source in (mission_info, path_row)
        if isinstance(source, dict)
    ]
    source = next(
        (
            row
            for row in sources
            if row.get("areaCoveragePassContractVersion") is not None
            or isinstance(row.get("remainingCoveragePasses"), list)
            or isinstance(row.get("coveragePassObligations"), list)
        ),
        None,
    )
    if source is None:
        return {"explicit": False, "passes": [], "obligations": {}}

    obligations: Dict[str, Dict[str, Any]] = {}
    for raw in source.get("coveragePassObligations") or []:
        if not isinstance(raw, dict):
            continue
        pass_name = _normalize_area_coverage_pass(raw.get("coveragePass"))
        if pass_name is not None:
            obligations[pass_name] = deepcopy(raw)
    passes: List[str] = []
    for raw in source.get("remainingCoveragePasses") or []:
        pass_name = _normalize_area_coverage_pass(raw)
        if pass_name is not None and pass_name not in passes:
            passes.append(pass_name)
    if not passes:
        passes = [
            pass_name
            for pass_name in ("forward", "reverse")
            if pass_name in obligations
        ]
    return {
        "explicit": True,
        "passes": passes,
        "obligations": obligations,
        "phase": str(source.get("areaCoveragePhase") or "").strip().lower(),
        "assignmentMode": str(source.get("areaPassAssignmentMode") or "").strip().lower(),
        "assignedPass": _normalize_area_coverage_pass(
            source.get("areaAssignedCoveragePass")
        ),
    }


def _area_coverage_acquisition_id(
    path_row: Dict[str, Any],
    mission_info: Dict[str, Any] | None,
    *,
    aircraft_id: int,
    path_id: int,
    pass_name: str,
    timestamp_ms: int | None = None,
    individual_mission_id: int | None = None,
) -> str:
    """Return one stable acquisition ID for a route traversal.

    All lineSearch waypoints in the same traversal share the ID, so dense
    samples and a resumed path cannot be mistaken for independent captures.
    A different aircraft or the reciprocal traversal receives a distinct ID.
    """

    normalized_pass = _normalize_area_coverage_pass(pass_name) or "forward"
    force_new_acquisition = False
    preserved_acquisition_id = ""
    for source in (mission_info, path_row):
        if not isinstance(source, dict):
            continue
        for obligation in source.get("coveragePassObligations") or []:
            if not isinstance(obligation, dict):
                continue
            if _normalize_area_coverage_pass(obligation.get("coveragePass")) != normalized_pass:
                continue
            if bool(obligation.get("forceNewCoverageAcquisition")):
                force_new_acquisition = True
                preserved_acquisition_id = ""
                break
            explicit_id = str(
                obligation.get("coverageAcquisitionID")
                or obligation.get("coverage_acquisition_id")
                or ""
            ).strip()
            if explicit_id:
                preserved_acquisition_id = explicit_id
                break
        if force_new_acquisition:
            break
        if preserved_acquisition_id:
            return preserved_acquisition_id
        active_ids = source.get("activeCoverageAcquisitionIDs")
        if isinstance(active_ids, dict):
            active_id = str(active_ids.get(normalized_pass) or "").strip()
            if active_id:
                return active_id

    namespace = ""
    for source in (mission_info, path_row):
        if not isinstance(source, dict):
            continue
        namespace = str(source.get("coverageAcquisitionNamespace") or "").strip()
        if namespace:
            break
        for key in (
            "areaTakeoverSourceInputMissionID",
            "sourceInputMissionID",
            "inputMissionID",
        ):
            value = source.get(key)
            try:
                if value is not None and int(value) > 0:
                    namespace = f"inputMission:{int(value)}"
                    break
            except Exception:
                continue
        if namespace:
            break
    if not namespace:
        namespace = "areaMission"
    if force_new_acquisition:
        generation_parts = [
            f"ts:{int(timestamp_ms)}" if timestamp_ms is not None else "",
            (
                f"mission:{int(individual_mission_id)}"
                if individual_mission_id is not None
                else ""
            ),
            f"path:{int(path_id)}",
        ]
        external_generation_id = ""
        for source in (mission_info, path_row):
            if not isinstance(source, dict):
                continue
            for key in (
                "coverageAcquisitionGenerationToken",
                "replanTransactionID",
                "replanRequestID",
                "requestID",
                "requestId",
                "transactionID",
                "transactionId",
                "sourceMissionPlanID",
            ):
                value = str(source.get(key) or "").strip()
                if value:
                    external_generation_id = value
                    break
            if external_generation_id:
                break
        if external_generation_id:
            generation_parts.append(
                "request:"
                + hashlib.sha256(
                    external_generation_id.encode("utf-8", errors="ignore")
                ).hexdigest()[:12]
            )
        generation_token = ":".join(part for part in generation_parts if part)
        return (
            f"{namespace}:aircraft:{int(aircraft_id)}:"
            f"generation:{generation_token}:traversal:{normalized_pass}"
        )
    return (
        f"{namespace}:aircraft:{int(aircraft_id)}:"
        f"path:{int(path_id)}:traversal:{normalized_pass}"
    )


def _coord_ring_xy(value: Any) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    for coordinate in value or []:
        if not isinstance(coordinate, dict):
            continue
        point_xy = coord_to_xy(coordinate)
        if point_xy is None:
            continue
        point = (float(point_xy[0]), float(point_xy[1]))
        if not rows or _distance_xy(rows[-1], point) > 0.05:
            rows.append(point)
    if len(rows) >= 2 and _distance_xy(rows[0], rows[-1]) <= 0.05:
        rows.pop()
    return rows


def _safe_polygon_xy(value: Any) -> Polygon | MultiPolygon | None:
    rows = _coord_ring_xy(value)
    if len(rows) < 3:
        return None
    try:
        geometry = Polygon(rows)
        if not geometry.is_valid:
            geometry = geometry.buffer(0)
    except Exception:
        return None
    if isinstance(geometry, (Polygon, MultiPolygon)) and not geometry.is_empty:
        return geometry
    return None


def _remaining_detail_geometry_xy(detail: Any):
    """Build the pass-specific remaining geometry in the planner XY CRS."""

    if not isinstance(detail, dict):
        return None
    segment_rows = [
        row
        for row in (detail.get("areaSegmentList") or [])
        if isinstance(row, dict)
    ]
    if segment_rows:
        segment_polygons = [
            polygon
            for polygon in (
                _safe_polygon_xy(row.get("coordinateList"))
                for row in segment_rows
            )
            if polygon is not None
        ]
        if segment_polygons:
            return unary_union(segment_polygons)

    area_rows = [
        row
        for row in (detail.get("areaList") or [])
        if isinstance(row, dict)
    ]
    outers = []
    holes = []
    for row in area_rows:
        polygon = _safe_polygon_xy(row.get("coordinateList"))
        if polygon is None:
            continue
        (holes if bool(row.get("isHole")) else outers).append(polygon)
    if not outers:
        fallback = _safe_polygon_xy(detail.get("coordinateList"))
        if fallback is not None:
            outers.append(fallback)
    if not outers:
        return None
    geometry = unary_union(outers)
    if holes:
        try:
            geometry = geometry.difference(unary_union(holes))
        except Exception:
            pass
    return geometry if geometry is not None and not geometry.is_empty else None


def _line_components(value: Any) -> List[LineString]:
    if isinstance(value, LineString):
        return [value] if value.length > 0.5 else []
    if isinstance(value, MultiLineString):
        return [row for row in value.geoms if isinstance(row, LineString) and row.length > 0.5]
    if isinstance(value, GeometryCollection):
        rows: List[LineString] = []
        for child in value.geoms:
            rows.extend(_line_components(child))
        return rows
    return []


def _clip_area_scan_lines_to_remaining_detail(
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]],
    detail: Any,
    *,
    assigned_polygon_xy: Any = None,
) -> List[List[Tuple[float, float]]]:
    """Clip sweep rows to one pass obligation and keep their route order."""

    geometry = _remaining_detail_geometry_xy(detail)
    if geometry is None:
        return []
    assigned = None
    assigned_rows = _xy_rows(assigned_polygon_xy)
    if len(assigned_rows) >= 3:
        try:
            assigned = Polygon(assigned_rows)
            if not assigned.is_valid:
                assigned = assigned.buffer(0)
        except Exception:
            assigned = None
    if assigned is not None and not assigned.is_empty:
        try:
            geometry = geometry.intersection(assigned)
        except Exception:
            pass
    if geometry is None or geometry.is_empty:
        return []

    clipped: List[List[Tuple[float, float]]] = []
    for raw_line in scan_lines_xy:
        rows = _dedupe_xy_rows(_xy_rows(raw_line), eps_m=0.05)
        if len(rows) < 2:
            continue
        source_line = LineString(rows)
        try:
            components = _line_components(source_line.intersection(geometry))
        except Exception:
            components = []
        ordered: List[Tuple[float, LineString]] = []
        for component in components:
            coords = list(component.coords)
            if len(coords) < 2:
                continue
            first_projection = float(source_line.project(Point(coords[0])))
            last_projection = float(source_line.project(Point(coords[-1])))
            if last_projection < first_projection:
                coords.reverse()
                component = LineString(coords)
                first_projection, last_projection = last_projection, first_projection
            ordered.append((first_projection, component))
        for _projection, component in sorted(ordered, key=lambda item: item[0]):
            length_m = float(component.length)
            if length_m <= 0.5:
                continue
            # Public Area lineSearch uses a stable three-point row contract.
            clipped.append(
                [
                    (float(point.x), float(point.y))
                    for point in (
                        component.interpolate(0.0),
                        component.interpolate(length_m * 0.5),
                        component.interpolate(length_m),
                    )
                ]
            )
    return clipped


def _reverse_area_route_context(path_row: Dict[str, Any]) -> Dict[str, Any]:
    """Return an Area route context that flies the assigned axis inbound.

    Pass-specific remaining geometry is stored in the canonical outbound scan
    order.  A reverse-only replan must still enter at the outbound terminal and
    consume those rows in the opposite direction; merely relabelling an
    outbound route as ``reverse`` would send the aircraft back through the
    wrong side of the Area first.
    """

    reversed_row = deepcopy(path_row)
    for start_key, end_key in (
        ("waypointStartXY", "waypointEndXY"),
        ("areaMissionStartXY", "areaMissionEndXY"),
        ("areaSweepRouteStartXY", "areaSweepRouteEndXY"),
    ):
        start_value = deepcopy(path_row.get(start_key))
        end_value = deepcopy(path_row.get(end_key))
        if end_value is not None:
            reversed_row[start_key] = end_value
        if start_value is not None:
            reversed_row[end_key] = start_value

    centerline_xy = path_row.get("areaCenterLineXY")
    if isinstance(centerline_xy, list) and centerline_xy:
        reversed_row["areaCenterLineXY"] = list(reversed(deepcopy(centerline_xy)))

    # These helpers were solved for the outbound entry.  Reusing them after
    # swapping the route endpoints creates a detour back to the old start.
    reversed_row.pop("entryTPrimeXY", None)
    reversed_row.pop("areaStabilizationLeadXY", None)
    return reversed_row


def _orient_area_scan_endpoints_from_outer_side(
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]],
    path_row: Dict[str, Any],
) -> tuple[List[List[Tuple[float, float]]], bool]:
    """Start an edge owner's serpentine at the original AREA exterior.

    The split normal is ``(cos(move), -sin(move))``. Reversing every scan line
    together changes only the filming endpoint preference; it preserves row
    order, route-axis selection, and sequential AREA hand-over decisions.
    """

    rows = [list(line_xy) for line_xy in scan_lines_xy]
    if not bool(path_row.get("areaOuterFirstSweep")):
        return rows, False
    side = str(path_row.get("areaOuterSide") or "").strip().lower()
    if side not in {"min", "max"}:
        return rows, False
    bearing_deg = _to_float(path_row.get("bearingDeg"))
    if bearing_deg is None:
        return rows, False
    first_line = next((line_xy for line_xy in rows if len(line_xy) >= 2), None)
    if first_line is None:
        return rows, False

    bearing_rad = math.radians(float(bearing_deg))
    normal_x = math.cos(bearing_rad)
    normal_y = -math.sin(bearing_rad)
    start_projection = (
        normal_x * float(first_line[0][0])
        + normal_y * float(first_line[0][1])
    )
    end_projection = (
        normal_x * float(first_line[-1][0])
        + normal_y * float(first_line[-1][1])
    )
    should_reverse = bool(
        (side == "min" and start_projection > end_projection + 1e-6)
        or (side == "max" and start_projection < end_projection - 1e-6)
    )
    if not should_reverse:
        return rows, False
    return [list(reversed(line_xy)) for line_xy in rows], True


def _area_sweep_items_xy(
    path_row: Dict[str, Any],
    scan_lines_xy: List[List[Tuple[float, float]]],
    *,
    deduped_scan_lines_xy: List[List[Tuple[float, float]]] | None = None,
) -> List[Dict[str, Any]]:
    if not scan_lines_xy:
        return []
    start_xy = (
        _xy_pair(path_row.get("areaSweepRouteStartXY"))
        or _xy_pair(path_row.get("waypointStartXY"))
        or _xy_pair(path_row.get("entryTPrimeXY"))
        or _xy_pair(path_row.get("tangentXY"))
    )
    end_xy = (
        _xy_pair(path_row.get("areaSweepRouteEndXY"))
        or _xy_pair(path_row.get("waypointEndXY"))
    )
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
    rows_by_index = deduped_scan_lines_xy if deduped_scan_lines_xy is not None else scan_lines_xy
    for idx, line_xy in enumerate(rows_by_index):
        rows = (
            list(line_xy or [])
            if deduped_scan_lines_xy is not None
            else _dedupe_xy_rows(_xy_rows(line_xy), eps_m=0.5)
        )
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
    anchor_to_first_item: bool = False,
) -> List[List[Dict[str, Any]]]:
    if not items:
        return []
    target_spacing_m = max(float(spacing_m), 1.0)
    # AREA sweeps can have short route progress but many dense scan lines.
    # Keep a soft cap so one terminal WP does not inherit the whole sensor run.
    # 단, 밀집 스캔(피치 ~22m)에서 상한이 route 간격보다 먼저 걸리면 간격
    # 파라미터(uav_wp_interval_m×scale)가 무력화되고 어중간한 꼬리 WP가
    # 남으므로 상한은 좌표 폭주 방지 수준으로만 둔다(간격이 분할을 지배).
    max_items_per_group = 256
    groups: List[List[Dict[str, Any]]] = []
    current_group: List[Dict[str, Any]] = [items[0]]
    anchor_progress_m = (
        float(items[0].get("progressM", 0.0) or 0.0)
        if anchor_to_first_item
        else 0.0
    )

    for item in items[1:]:
        candidate_progress_m = float(item.get("progressM", anchor_progress_m) or anchor_progress_m)
        last_progress_m = float(current_group[-1].get("progressM", anchor_progress_m) or anchor_progress_m)
        current_span_m = float(last_progress_m) - float(anchor_progress_m)
        spacing_limit_reached = (
            (float(candidate_progress_m) - float(anchor_progress_m)) > target_spacing_m
            and current_span_m >= 1.0
        )
        density_limit_reached = len(current_group) >= max_items_per_group and current_span_m >= 1.0
        if current_group and (spacing_limit_reached or density_limit_reached):
            groups.append(current_group)
            anchor_progress_m = last_progress_m
            current_group = [item]
        else:
            current_group.append(item)
    if current_group:
        groups.append(current_group)

    if merge_short_tail:
        # 마지막 그룹이 목표 간격에 못 미치면 이전 그룹에 흡수한다 — 1-2가
        # 간격이고 2-3이 잔여라면 1-3 하나로 만든다(짧은 꼬리 WP 방지, 사용자
        # 확정 규칙). 좌표 총량은 densify/ICD 상한이 별도로 지키므로 여기서는
        # 아이템 수 안전 상한만 확인한다.
        while len(groups) >= 2:
            prev_progress_m = float(groups[-2][-1].get("progressM", 0.0) or 0.0)
            tail_progress_m = float(groups[-1][-1].get("progressM", prev_progress_m) or prev_progress_m)
            if tail_progress_m - prev_progress_m + 1e-6 >= target_spacing_m:
                break
            if len(groups[-2]) + len(groups[-1]) > max_items_per_group:
                break
            groups[-1] = groups[-2] + groups[-1]
            groups.pop(-2)
    return groups


def _percentile_float(values: Sequence[float], ratio: float) -> float:
    rows = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not rows:
        return 0.0
    if len(rows) == 1:
        return float(rows[0])
    position = max(0.0, min(1.0, float(ratio))) * float(len(rows) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return float(rows[lower])
    weight = float(position) - float(lower)
    return float(rows[lower]) + ((float(rows[upper]) - float(rows[lower])) * weight)


def _area_reciprocal_terrain_profile(
    path_row: Dict[str, Any],
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]],
) -> Dict[str, Any]:
    """Return the retired state for the former second AREA pass.

    This is deliberately classification-only.  It does not import the saved
    terrain-stabilisation route policy, so the existing first Area pass remains
    unchanged.  Rows made by that policy can still provide an explicit trusted
    ``areaTerrainStabilizationActive`` decision.
    """
    return {
        "active": False,
        "reason": "single_capture_policy",
        "sampleCount": 0,
    }

    if "areaTerrainStabilizationActive" in path_row:
        upstream_active = bool(path_row.get("areaTerrainStabilizationActive"))
        terrain_score_value = _to_float(path_row.get("areaTerrainScore"))
        terrain_relief_value = _to_float(path_row.get("areaTerrainReliefM"))
        robust_relief_value = _to_float(path_row.get("areaTerrainRobustReliefM"))
        grade_value = _to_float(path_row.get("areaTerrainGradeP90"))
        has_upstream_terrain_metrics = any(
            value is not None
            for value in (
                terrain_score_value,
                terrain_relief_value,
                robust_relief_value,
                grade_value,
            )
        )
        active = bool(upstream_active)
        if active and has_upstream_terrain_metrics:
            min_score = float(
                get_runtime_float("next_collab_area_reciprocal_min_terrain_score", 0.45)
            )
            min_relief_m = float(
                get_runtime_float("next_collab_area_reciprocal_min_relief_m", 100.0)
            )
            min_robust_relief_m = float(
                get_runtime_float(
                    "next_collab_area_reciprocal_min_robust_relief_m",
                    75.0,
                )
            )
            min_grade_p90 = float(
                get_runtime_float("next_collab_area_reciprocal_min_grade_p90", 0.08)
            )
            active = bool(
                float(terrain_score_value or 0.0) >= min_score
                and float(terrain_relief_value or 0.0) >= min_relief_m
                and (
                    float(robust_relief_value or 0.0) >= min_robust_relief_m
                    or float(grade_value or 0.0) >= min_grade_p90
                )
            )
        return {
            "active": active,
            "reason": "upstream_rugged" if active else "upstream_not_rugged",
            "sampleCount": int(_to_float(path_row.get("areaTerrainSampleCount")) or 0),
            "terrainScore": float(terrain_score_value or 0.0),
            "terrainReliefM": float(terrain_relief_value or 0.0),
            "terrainRobustReliefM": float(
                robust_relief_value or 0.0
            ),
            "terrainGradeP90": float(grade_value or 0.0),
            "terrainRoughness": float(_to_float(path_row.get("areaTerrainRoughness")) or 0.0),
        }

    sample_cap = max(
        24,
        min(
            768,
            int(get_runtime_int("next_collab_area_reciprocal_terrain_sample_cap", 192)),
        ),
    )
    valid_lines = [
        _dedupe_xy_rows(_xy_rows(line_xy), eps_m=0.5)
        for line_xy in scan_lines_xy
        if isinstance(line_xy, (tuple, list))
    ]
    valid_lines = [line_xy for line_xy in valid_lines if len(line_xy) >= 2]
    if len(valid_lines) < 2:
        return {"active": False, "reason": "scan_geometry_missing", "sampleCount": 0}

    max_lines = max(2, int(sample_cap) // 3)
    if len(valid_lines) <= max_lines:
        selected_indices = list(range(len(valid_lines)))
    else:
        selected_indices = sorted(
            {
                int(round(idx * (len(valid_lines) - 1) / float(max_lines - 1)))
                for idx in range(max_lines)
            }
        )

    sample_points: List[Tuple[float, float]] = []
    route_midpoints: List[Tuple[float, float]] = []
    widths_m: List[float] = []
    seen: set[Tuple[float, float]] = set()

    def _append_sample(point_xy: Tuple[float, float]) -> None:
        key = (round(float(point_xy[0]), 2), round(float(point_xy[1]), 2))
        if key in seen or len(sample_points) >= int(sample_cap):
            return
        seen.add(key)
        sample_points.append((float(point_xy[0]), float(point_xy[1])))

    for line_idx in selected_indices:
        line_xy = valid_lines[line_idx]
        midpoint_xy = _midpoint_xy(line_xy)
        if midpoint_xy is None:
            continue
        width_m = _line_length_xy(line_xy)
        if width_m > 1.0:
            widths_m.append(float(width_m))
        route_midpoints.append((float(midpoint_xy[0]), float(midpoint_xy[1])))
        _append_sample(line_xy[0])
        _append_sample((float(midpoint_xy[0]), float(midpoint_xy[1])))
        _append_sample(line_xy[-1])

    if len(sample_points) < 3 or len(route_midpoints) < 2:
        return {
            "active": False,
            "reason": "terrain_samples_missing",
            "sampleCount": len(sample_points),
        }

    coords = [_xy_to_coord(point_xy) for point_xy in sample_points]
    pairs = [
        (float(coord["latitude"]), float(coord["longitude"]))
        for coord in coords
    ]
    elevations_raw = _dem_altitudes_for_pairs_cached(pairs, invalid_default=None)
    elevation_by_xy: Dict[Tuple[float, float], float] = {}
    elevations_m: List[float] = []
    for point_xy, raw_elevation in zip(sample_points, elevations_raw):
        elevation = _to_float(raw_elevation)
        if elevation is None or not math.isfinite(float(elevation)):
            continue
        key = (round(float(point_xy[0]), 2), round(float(point_xy[1]), 2))
        elevation_by_xy[key] = float(elevation)
        elevations_m.append(float(elevation))
    if len(elevations_m) < 3:
        return {
            "active": False,
            "reason": "terrain_elevation_missing",
            "sampleCount": len(elevations_m),
        }

    route_elevations: List[Tuple[Tuple[float, float], float]] = []
    for point_xy in route_midpoints:
        elevation = elevation_by_xy.get((round(point_xy[0], 2), round(point_xy[1], 2)))
        if elevation is not None:
            route_elevations.append((point_xy, float(elevation)))
    grades: List[float] = []
    total_route_m = 0.0
    total_vertical_m = 0.0
    for (prev_xy, prev_alt), (curr_xy, curr_alt) in zip(
        route_elevations,
        route_elevations[1:],
    ):
        distance_m = _distance_xy(prev_xy, curr_xy)
        if distance_m < 10.0:
            continue
        vertical_m = abs(float(curr_alt) - float(prev_alt))
        grades.append(vertical_m / distance_m)
        total_route_m += distance_m
        total_vertical_m += vertical_m

    relief_m = max(elevations_m) - min(elevations_m)
    p05_ground_m = _percentile_float(elevations_m, 0.05)
    p95_ground_m = _percentile_float(elevations_m, 0.95)
    robust_relief_m = max(0.0, p95_ground_m - p05_ground_m)
    clipped = [min(p95_ground_m, max(p05_ground_m, value)) for value in elevations_m]
    mean_ground_m = sum(clipped) / max(len(clipped), 1)
    terrain_std_m = math.sqrt(
        sum((value - mean_ground_m) ** 2 for value in clipped) / max(len(clipped), 1)
    )
    grade_p90 = _percentile_float(grades, 0.90) if grades else 0.0
    roughness = total_vertical_m / total_route_m if total_route_m > 1.0 else 0.0
    width_median = _percentile_float(widths_m, 0.50) if widths_m else 0.0
    width_variation = (
        (_percentile_float(widths_m, 0.90) - _percentile_float(widths_m, 0.10))
        / max(width_median, 1.0)
        if widths_m
        else 0.0
    )
    terrain_score = min(
        1.0,
        max(
            robust_relief_m / 300.0,
            terrain_std_m / 100.0,
            grade_p90 / 0.20,
            0.75 * (roughness / 0.12),
            0.35 * (width_variation / 0.75),
        ),
    )
    min_score = max(
        0.0,
        min(
            1.0,
            float(get_runtime_float("next_collab_area_reciprocal_min_terrain_score", 0.45)),
        ),
    )
    min_relief_m = max(
        0.0,
        float(get_runtime_float("next_collab_area_reciprocal_min_relief_m", 100.0)),
    )
    min_robust_relief_m = max(
        0.0,
        float(get_runtime_float("next_collab_area_reciprocal_min_robust_relief_m", 75.0)),
    )
    min_grade_p90 = max(
        0.0,
        float(get_runtime_float("next_collab_area_reciprocal_min_grade_p90", 0.08)),
    )
    elevation_varies = bool(
        relief_m >= min_relief_m
        and (robust_relief_m >= min_robust_relief_m or grade_p90 >= min_grade_p90)
    )
    active = bool(terrain_score >= min_score and elevation_varies)
    return {
        "active": active,
        "reason": "rugged_terrain" if active else "below_rugged_threshold",
        "sampleCount": len(elevations_m),
        "terrainScore": float(terrain_score),
        "terrainReliefM": float(relief_m),
        "terrainRobustReliefM": float(robust_relief_m),
        "terrainStdM": float(terrain_std_m),
        "terrainGradeP90": float(grade_p90),
        "terrainRoughness": float(roughness),
        "shapeWidthVariation": float(width_variation),
    }


def _area_reciprocal_turn_radius_m(
    *,
    fallback_radius_m: float,
    terminal_speed_mps: float | None,
    aircraft_id: int | None,
) -> Tuple[float, str]:
    radius_m = float(fallback_radius_m)
    terminal_speed = _to_float(terminal_speed_mps)
    if terminal_speed is None or terminal_speed <= 0.0:
        return radius_m, "planned_fallback"
    try:
        speed_radius_m = float(interpolate_reference_turn_radius(float(terminal_speed)))
        speed_radius_m *= float(
            reference_turn_radius_scale_for_aircraft(
                int(aircraft_id) if aircraft_id is not None else None
            )
        )
    except Exception:
        return radius_m, "planned_fallback"
    if not math.isfinite(speed_radius_m) or speed_radius_m < 25.0:
        return radius_m, "planned_fallback"
    return float(speed_radius_m), "terminal_speed_profile"


def _compact_area_reciprocal_turn_geometry(
    *,
    terminal_anchor_xy: Tuple[float, float],
    exit_unit_xy: Tuple[float, float],
    turn_side_sign: float,
    radius_m: float,
) -> Dict[str, Any]:
    ux, uy = float(exit_unit_xy[0]), float(exit_unit_xy[1])
    unit_length = math.hypot(ux, uy)
    if unit_length <= 1.0e-6:
        raise ValueError("reciprocal exit axis missing")
    ux, uy = ux / unit_length, uy / unit_length
    nx, ny = -uy, ux
    desired_side = 1.0 if float(turn_side_sign) >= 0.0 else -1.0
    # The SIM begins a fly-by turn roughly 0.45 reference radii before a gate.
    # A 0.50 scale leaves a small control margin and yields the replay-proven
    # 1.5R forward / sqrt(3)/2 R lateral gates, without the former 10.392R
    # excursion.  The lower bound prevents a runtime typo from removing that
    # margin and collapsing the two outside gates.
    gate_scale = max(
        0.45,
        min(
            0.75,
            float(
                get_runtime_float(
                    "next_collab_area_reciprocal_compact_gate_scale",
                    0.50,
                )
            ),
        ),
    )
    forward_m = 3.0 * float(radius_m) * float(gate_scale)
    lateral_m = math.sqrt(3.0) * float(radius_m) * float(gate_scale)
    anchor_x, anchor_y = float(terminal_anchor_xy[0]), float(terminal_anchor_xy[1])
    first_turn_target_xy = (
        anchor_x + (ux * forward_m) + (nx * desired_side * lateral_m),
        anchor_y + (uy * forward_m) + (ny * desired_side * lateral_m),
    )
    second_turn_target_xy = (
        anchor_x + (ux * forward_m) - (nx * desired_side * lateral_m),
        anchor_y + (uy * forward_m) - (ny * desired_side * lateral_m),
    )
    return {
        "turnPathXY": [terminal_anchor_xy, first_turn_target_xy, second_turn_target_xy],
        "reentryXY": terminal_anchor_xy,
        "turnRadiusM": float(radius_m),
        "turnGateRadiusScale": float(gate_scale),
        "turnForwardM": float(forward_m),
        "turnLateralM": float(lateral_m),
        "turnSide": "left" if desired_side > 0.0 else "right",
    }


def _build_area_reciprocal_pass_plan(
    path_row: Dict[str, Any],
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]],
    forward_items: Sequence[Dict[str, Any]],
    *,
    turn_radius_m: float,
    terminal_speed_mps: float | None = None,
    aircraft_id: int | None = None,
    force_active: bool = False,
) -> Dict[str, Any]:
    """Build a fail-closed reciprocal pass with a compact three-WP return.

    ``turnPathXY`` begins at the final *actual sweep anchor*, never at polygon
    support or ``waypointEndXY``.  Two outside targets plus the same anchor as
    a non-filming re-entry are intentionally retained: one/two total gates
    cannot both make a stable fixed-wing reversal and start the reverse camera
    leg on the exact outbound route.  The geometric radius comes from the
    commanded terminal speed when it is available.
    """
    terrain_profile = _area_reciprocal_terrain_profile(path_row, scan_lines_xy)
    inactive: Dict[str, Any] = {
        "active": False,
        "reason": str(terrain_profile.get("reason") or "inactive"),
        "terrainProfile": terrain_profile,
        "reverseScanLinesXY": [],
        "reverseItems": [],
        "turnPathXY": [],
    }
    if str(terrain_profile.get("reason") or "") in {
        "disabled",
        "single_capture_policy",
    }:
        return inactive
    if not bool(terrain_profile.get("active")) and not bool(force_active):
        return inactive
    if bool(force_active) and not bool(terrain_profile.get("active")):
        terrain_profile = dict(terrain_profile)
        terrain_profile["active"] = True
        terrain_profile["reason"] = "pending_pass_contract"
    radius_m = float(turn_radius_m)
    if not math.isfinite(radius_m) or radius_m < 25.0:
        inactive["reason"] = "invalid_turn_radius"
        return inactive

    valid_items: List[Dict[str, Any]] = []
    for raw_item in forward_items:
        if not isinstance(raw_item, dict):
            continue
        anchor_xy = _xy_pair(raw_item.get("anchorXY"))
        sweep_xy = _dedupe_xy_rows(_xy_rows(raw_item.get("sweepXY")), eps_m=0.5)
        if anchor_xy is None or len(sweep_xy) < 2:
            continue
        item = deepcopy(raw_item)
        item["anchorXY"] = anchor_xy
        item["sweepXY"] = sweep_xy
        valid_items.append(item)
    if not valid_items:
        inactive["reason"] = "insufficient_sweep_items"
        return inactive

    last_item = valid_items[-1]
    last_anchor_xy = _xy_pair(last_item.get("anchorXY"))
    previous_anchor_xy = None
    for item in reversed(valid_items[:-1]):
        candidate_xy = _xy_pair(item.get("anchorXY"))
        if candidate_xy is not None and _distance_xy(candidate_xy, last_anchor_xy) > 10.0:
            previous_anchor_xy = candidate_xy
            break
    if last_anchor_xy is not None and previous_anchor_xy is None:
        # A narrow assigned piece can legitimately collapse to one grouped
        # capture leg.  Its route start still supplies the outbound tangent;
        # rejecting it here would silently drop the required reverse pass.
        for raw_candidate in (
            path_row.get("waypointStartXY"),
            path_row.get("areaMissionStartXY"),
        ):
            candidate_xy = _xy_pair(raw_candidate)
            if (
                candidate_xy is not None
                and _distance_xy(candidate_xy, last_anchor_xy) > 10.0
            ):
                previous_anchor_xy = candidate_xy
                break
    if last_anchor_xy is None or previous_anchor_xy is None:
        inactive["reason"] = "exit_axis_missing"
        return inactive
    dx = float(last_anchor_xy[0]) - float(previous_anchor_xy[0])
    dy = float(last_anchor_xy[1]) - float(previous_anchor_xy[1])
    axis_length_m = math.hypot(dx, dy)
    if axis_length_m <= 10.0:
        inactive["reason"] = "exit_axis_too_short"
        return inactive
    ux, uy = dx / axis_length_m, dy / axis_length_m

    boundary_xy = (float(last_anchor_xy[0]), float(last_anchor_xy[1]))
    terminal_speed = _to_float(terminal_speed_mps)
    radius_m, radius_source = _area_reciprocal_turn_radius_m(
        fallback_radius_m=float(radius_m),
        terminal_speed_mps=terminal_speed,
        aircraft_id=aircraft_id,
    )

    # Keep the first-turn side stable across replans.  P1/P2 and the terminal
    # form the compact triangular gate replayed against the SIM fly-by
    # controller.  It avoids the ambiguous single-target 180-degree reversal
    # while the 0.50 look-ahead scale removes the former full-fillet excursion.
    desired_side = (
        -1.0
        if int(
            _to_float(path_row.get("pieceIndex"))
            or _to_float(path_row.get("aircraftID"))
            or 0
        )
        % 2
        == 0
        else 1.0
    )
    turn_geometry = _compact_area_reciprocal_turn_geometry(
        terminal_anchor_xy=boundary_xy,
        exit_unit_xy=(ux, uy),
        turn_side_sign=desired_side,
        radius_m=float(radius_m),
    )
    turn_path_xy = list(turn_geometry["turnPathXY"])
    max_turn_points = max(
        0,
        int(get_runtime_int("next_collab_area_reciprocal_max_turn_waypoints", 3)),
    )
    if len(turn_path_xy) > max_turn_points:
        inactive["reason"] = "turn_waypoint_budget_exceeded"
        return inactive

    reverse_scan_lines_xy = [
        list(reversed(_dedupe_xy_rows(_xy_rows(line_xy), eps_m=0.5)))
        for line_xy in reversed(scan_lines_xy)
    ]
    reverse_scan_lines_xy = [line_xy for line_xy in reverse_scan_lines_xy if len(line_xy) >= 2]
    max_progress_m = max(
        (float(_to_float(item.get("progressM")) or 0.0) for item in valid_items),
        default=0.0,
    )
    reverse_items: List[Dict[str, Any]] = []
    for item in reversed(valid_items):
        reverse_item = deepcopy(item)
        reverse_item["sweepXY"] = list(reversed(_dedupe_xy_rows(_xy_rows(item.get("sweepXY")), eps_m=0.5)))
        progress_m = _to_float(item.get("progressM"))
        if progress_m is not None:
            reverse_item["progressM"] = max(0.0, max_progress_m - float(progress_m))
        reverse_items.append(reverse_item)

    return {
        "active": True,
        "reason": str(terrain_profile.get("reason") or "rugged_terrain"),
        "policy": "minimal_two_gate_reciprocal",
        "terrainProfile": terrain_profile,
        "reverseScanLinesXY": reverse_scan_lines_xy,
        "reverseItems": reverse_items,
        "turnPathXY": turn_path_xy,
        "reentryXY": boundary_xy,
        "turnType": "two_gate_flyby",
        "turnRadiusM": float(radius_m),
        "turnRadiusSource": radius_source,
        "turnTerminalSpeedMps": float(terminal_speed or 0.0),
        "turnGateRadiusScale": float(turn_geometry["turnGateRadiusScale"]),
        "turnForwardM": float(turn_geometry["turnForwardM"]),
        "turnLateralM": float(turn_geometry["turnLateralM"]),
        "turnSide": str(turn_geometry["turnSide"]),
        "terminalAnchorXY": boundary_xy,
        "exitUnitXY": (float(ux), float(uy)),
        "turnSideSign": float(desired_side),
    }


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
    sweep_three_point_rows_xy: List[List[Tuple[float, float]]] | None = None,
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
            if sweep_three_point_rows_xy is not None and sweep_idx < len(sweep_three_point_rows_xy):
                sweep_rows = sweep_three_point_rows_xy[sweep_idx]
            else:
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
    anchor_context: Dict[str, Any] | None = None,
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
            anchor_context=anchor_context,
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
            anchor_context=anchor_context,
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
    sweep_three_point_rows_xy: List[List[Tuple[float, float]]] | None = None,
) -> List[Dict[str, Any]]:
    return _xy_rows_to_coords_with_dem_altitude(
        _collect_group_sweep_rows_xy(
            group=group,
            all_sweep_lines_xy=all_sweep_lines_xy,
            previous_rep_sweep_idx=previous_rep_sweep_idx,
            sweep_three_point_rows_xy=sweep_three_point_rows_xy,
        )
    )


def _collect_area_group_sweep_rows_xy(
    *,
    group: List[Dict[str, Any]],
    all_sweep_lines_xy: List[List[Tuple[float, float]]],
    previous_rep_sweep_idx: int | None = None,
    sweep_three_point_rows_xy: List[List[Tuple[float, float]]] | None = None,
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
            if sweep_three_point_rows_xy is not None and sweep_idx < len(sweep_three_point_rows_xy):
                sweep_rows = sweep_three_point_rows_xy[sweep_idx]
            else:
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
    sweep_three_point_rows_xy: List[List[Tuple[float, float]]] | None = None,
) -> List[Dict[str, Any]]:
    return _xy_rows_to_coords_with_dem_altitude(
        _collect_area_group_sweep_rows_xy(
            group=group,
            all_sweep_lines_xy=all_sweep_lines_xy,
            previous_rep_sweep_idx=previous_rep_sweep_idx,
            sweep_three_point_rows_xy=sweep_three_point_rows_xy,
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
    multiplier_cap_enabled: bool = True,
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
        multiplier_cap_enabled=bool(multiplier_cap_enabled),
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
    multiplier_cap_enabled: bool = True,
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
    estimated_speed_mps = _estimate_line_search_speed_xy_mps(
        prev_xy=prev_xy,
        anchor_xy=anchor_xy,
        sweep_xy=sweep_xy,
        cruise_speed_mps=float(cruise_speed_mps),
        fallback_search_speed_mps=float(fallback_search_speed_mps),
        speed_scale=float(speed_scale),
        reference_xy=reference_xy,
        multiplier_cap_enabled=bool(multiplier_cap_enabled),
    )
    planar_length_m = _line_length_xy(sweep_xy)
    spatial_length_m = _line_search_coordinate_length_3d_m(sweep_coords)
    if planar_length_m > 1e-6 and spatial_length_m > planar_length_m:
        estimated_speed_mps *= float(spatial_length_m) / float(planar_length_m)
        estimated_speed_mps = clamp_line_search_speed_mps(
            estimated_speed_mps,
            cruise_speed_mps=float(cruise_speed_mps),
            speed_scale=float(speed_scale),
            multiplier_cap_enabled=bool(multiplier_cap_enabled),
        )
    return float(estimated_speed_mps)


def _activation_constrained_search_speed_mps(
    current_search_speed_mps: float,
    *,
    origin_coords: Sequence[Dict[str, Any] | None],
    anchor_coord: Dict[str, Any] | None,
    sweep_coords: Sequence[Dict[str, Any]],
    flight_speed_mps: float,
    speed_scale: float,
    activation_delay_s: float,
    multiplier_cap_enabled: bool,
) -> float:
    """Keep the first scan feasible even when a stale helper WP is skipped."""

    anchor_xy = coord_to_xy(anchor_coord) if isinstance(anchor_coord, dict) else None
    if anchor_xy is None or flight_speed_mps <= 0.0:
        return float(current_search_speed_mps)
    transit_distances_m: List[float] = []
    for origin_coord in origin_coords or []:
        origin_xy = coord_to_xy(origin_coord) if isinstance(origin_coord, dict) else None
        if origin_xy is None:
            continue
        distance_m = _distance_xy(origin_xy, anchor_xy)
        if distance_m > 1.0:
            transit_distances_m.append(float(distance_m))
    sweep_length_m = _line_search_coordinate_length_3d_m(sweep_coords)
    if not transit_distances_m or sweep_length_m <= 1e-6:
        return float(current_search_speed_mps)
    # The public entry helper may be skipped when the newly authorized path
    # starts behind the aircraft. Use the shortest plausible activation leg.
    transit_time_s = min(transit_distances_m) / float(flight_speed_mps)
    available_scan_time_s = float(transit_time_s) - max(
        0.0,
        float(activation_delay_s),
    )
    if available_scan_time_s <= 0.05:
        available_scan_time_s = max(0.05, float(transit_time_s) * 0.25)
    try:
        effective_scale = max(0.10, float(speed_scale))
    except Exception:
        effective_scale = 1.0
    required_speed_mps = (
        float(sweep_length_m)
        / float(available_scan_time_s)
        * float(effective_scale)
    )
    required_speed_mps = clamp_line_search_speed_mps(
        required_speed_mps,
        cruise_speed_mps=float(flight_speed_mps),
        speed_scale=float(effective_scale),
        multiplier_cap_enabled=bool(multiplier_cap_enabled),
    )
    return max(float(current_search_speed_mps), float(required_speed_mps))


def _recompute_first_line_search_speed_from_entry_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    entry_coord: Dict[str, Any] | None,
    transit_speed_mps: float,
    fallback_search_speed_mps: float,
    speed_scale: float,
    multiplier_cap_enabled: bool = True,
    activation_delay_s: float = 0.0,
) -> bool:
    first_line_idx = next(
        (
            idx
            for idx, waypoint in enumerate(waypoints)
            if isinstance(waypoint, dict) and _has_line_search_coordinates(waypoint)
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
        multiplier_cap_enabled=bool(multiplier_cap_enabled),
    )
    speed_mps = _activation_constrained_search_speed_mps(
        speed_mps,
        origin_coords=(previous_coord, entry_coord),
        anchor_coord=(
            waypoint.get("coordinate")
            if isinstance(waypoint.get("coordinate"), dict)
            else None
        ),
        sweep_coords=_line_search_coordinate_list(waypoint),
        flight_speed_mps=float(transit_speed_mps),
        speed_scale=float(speed_scale),
        activation_delay_s=float(activation_delay_s),
        multiplier_cap_enabled=bool(multiplier_cap_enabled),
    )
    if speed_mps <= 0.0:
        return False
    line_search["searchSpeed"] = float(speed_mps)
    return True


def _resynchronize_area_capture_speeds_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    fallback_search_speed_mps: float,
    speed_scale: float,
    default_transit_speed_mps: float,
    activation_entry_coord: Dict[str, Any] | None = None,
    activation_delay_s: float = 0.0,
) -> int:
    """Match every emitted AREA scan to its actual incoming public WP leg."""

    changed = 0
    first_capture_seen = False
    for index in range(len(waypoints)):
        waypoint = waypoints[index] if isinstance(waypoints[index], dict) else {}
        sweep_coords = _line_search_coordinate_list(waypoint)
        if not sweep_coords:
            continue
        previous = (
            waypoints[index - 1]
            if index > 0 and isinstance(waypoints[index - 1], dict)
            else {}
        )
        previous_coord = (
            previous.get("coordinate")
            if isinstance(previous.get("coordinate"), dict)
            else None
        )
        if not isinstance(previous_coord, dict) and isinstance(
            activation_entry_coord,
            dict,
        ):
            previous_coord = activation_entry_coord
        anchor_coord = (
            waypoint.get("coordinate")
            if isinstance(waypoint.get("coordinate"), dict)
            else None
        )
        if not isinstance(previous_coord, dict) or not isinstance(anchor_coord, dict):
            continue
        leg_speed_mps = _to_float(waypoint.get("speed"))
        if leg_speed_mps is None or leg_speed_mps <= 0.0:
            leg_speed_mps = max(1.0, float(default_transit_speed_mps))
        search_speed_mps = _estimate_line_search_speed_mps(
            prev_coord=previous_coord,
            anchor_coord=anchor_coord,
            sweep_coords=sweep_coords,
            cruise_speed_mps=float(leg_speed_mps),
            fallback_search_speed_mps=float(fallback_search_speed_mps),
            speed_scale=float(speed_scale),
            multiplier_cap_enabled=False,
        )
        if not first_capture_seen and isinstance(activation_entry_coord, dict):
            search_speed_mps = _activation_constrained_search_speed_mps(
                search_speed_mps,
                origin_coords=(previous_coord, activation_entry_coord),
                anchor_coord=anchor_coord,
                sweep_coords=sweep_coords,
                flight_speed_mps=float(leg_speed_mps),
                speed_scale=float(speed_scale),
                activation_delay_s=float(activation_delay_s),
                multiplier_cap_enabled=False,
            )
        search_speed_mps, leg_speed_mps = _apply_area_scan_rate_slowdown(
            estimated_search_speed_mps=float(search_speed_mps),
            transit_speed_mps=float(leg_speed_mps),
        )
        if search_speed_mps <= 0.0:
            continue
        filming = (
            waypoint.get("filmingProperty")
            if isinstance(waypoint.get("filmingProperty"), dict)
            else {}
        )
        line_search = (
            filming.get("lineSearch")
            if isinstance(filming.get("lineSearch"), dict)
            else None
        )
        if line_search is None:
            continue
        waypoint["speed"] = round(float(leg_speed_mps), 2)
        line_search["searchSpeed"] = float(search_speed_mps)
        changed += 1
        first_capture_seen = True
    return int(changed)


def _resynchronize_line_capture_speeds_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    fallback_search_speed_mps: float,
    speed_scale: float,
    default_transit_speed_mps: float,
    multiplier_cap_enabled: bool,
    activation_entry_coord: Dict[str, Any] | None = None,
    activation_delay_s: float = 0.0,
) -> int:
    """Recompute every LINE camera rate from the final public WP geometry."""

    changed = 0
    first_capture_seen = False
    for index in range(len(waypoints)):
        waypoint = waypoints[index] if isinstance(waypoints[index], dict) else {}
        sweep_coords = _line_search_coordinate_list(waypoint)
        if not sweep_coords:
            continue
        previous = (
            waypoints[index - 1]
            if index > 0 and isinstance(waypoints[index - 1], dict)
            else {}
        )
        previous_coord = (
            previous.get("coordinate")
            if isinstance(previous.get("coordinate"), dict)
            else None
        )
        if not isinstance(previous_coord, dict) and isinstance(
            activation_entry_coord,
            dict,
        ):
            previous_coord = activation_entry_coord
        anchor_coord = (
            waypoint.get("coordinate")
            if isinstance(waypoint.get("coordinate"), dict)
            else None
        )
        if not isinstance(previous_coord, dict) or not isinstance(anchor_coord, dict):
            continue
        leg_speed_mps = _to_float(waypoint.get("speed"))
        if leg_speed_mps is None or leg_speed_mps <= 0.0:
            leg_speed_mps = max(1.0, float(default_transit_speed_mps))
        search_speed_mps = _estimate_line_search_speed_mps(
            prev_coord=previous_coord,
            anchor_coord=anchor_coord,
            sweep_coords=sweep_coords,
            cruise_speed_mps=float(leg_speed_mps),
            fallback_search_speed_mps=float(fallback_search_speed_mps),
            speed_scale=float(speed_scale),
            multiplier_cap_enabled=bool(multiplier_cap_enabled),
        )
        if not first_capture_seen and isinstance(activation_entry_coord, dict):
            search_speed_mps = _activation_constrained_search_speed_mps(
                search_speed_mps,
                origin_coords=(previous_coord, activation_entry_coord),
                anchor_coord=anchor_coord,
                sweep_coords=sweep_coords,
                flight_speed_mps=float(leg_speed_mps),
                speed_scale=float(speed_scale),
                activation_delay_s=float(activation_delay_s),
                multiplier_cap_enabled=bool(multiplier_cap_enabled),
            )
        if search_speed_mps <= 0.0:
            continue
        filming = (
            waypoint.get("filmingProperty")
            if isinstance(waypoint.get("filmingProperty"), dict)
            else {}
        )
        line_search = (
            filming.get("lineSearch")
            if isinstance(filming.get("lineSearch"), dict)
            else None
        )
        if line_search is None:
            continue
        line_search["searchSpeed"] = float(search_speed_mps)
        changed += 1
        first_capture_seen = True
    return int(changed)


def _line_route_endpoint_anchor_xy(
    path_row: Dict[str, Any],
    *,
    tail_sweep_xy: Sequence[Tuple[float, float]],
    scan_lines_xy: Sequence[Sequence[Tuple[float, float]]] | None,
    reference_anchor_xy: Tuple[float, float] | None,
    anchor_context: Dict[str, Any] | None = None,
) -> Tuple[float, float] | None:
    raw_route_line_xy = anchor_context.get("routeLineXY") if isinstance(anchor_context, dict) else None
    route_line_xy = raw_route_line_xy if isinstance(raw_route_line_xy, list) else _line_route_polyline_xy(path_row, scan_lines_xy)
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
    anchor_context: Dict[str, Any] | None = None,
) -> int:
    line_indices = [
        idx
        for idx, waypoint in enumerate(waypoints or [])
        if isinstance(waypoint, dict) and _has_line_search_coordinates(waypoint)
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
        anchor_context=anchor_context,
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


def _build_control_transfer_direct_flight_path(
    *,
    template_path: Dict[str, Any] | None,
    mission_info: Dict[str, Any],
    individual_mission_id: int,
    path_id: int,
    aircraft_id: int,
    entry_coord: Dict[str, Any] | None,
    timestamp_ms: int,
    source: str,
    speed_mps: float,
    sensor_type: int,
    altitude_fn: Callable[[float, float], int],
    waypoint_id_provider: Callable[[], int] | None,
    assign_waypoint_ids: bool,
) -> Dict[str, Any] | None:
    """Build a non-search regionType=2 route through the 0201 final point."""

    route_coords = control_transfer_route_coordinates(mission_info)
    if not route_coords:
        return None

    waypoints: List[Dict[str, Any]] = []
    for route_idx, coordinate in enumerate(route_coords):
        lat = float(coordinate["latitude"])
        lon = float(coordinate["longitude"])
        waypoints.append(
            _make_hold_waypoint(
                coordinate={
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": int(altitude_fn(lat, lon)),
                },
                speed_mps=float(speed_mps),
                sensor_type=int(sensor_type),
                field_of_view_deg=float(CONTROL_TRANSFER_FOV_DEG),
                orientation_coordinate=None,
                waypoint_pass_type=(
                    PASS_FLYOVER if route_idx == len(route_coords) - 1 else PASS_FLYBY
                ),
            )
        )

    flight_path = deepcopy(template_path) if isinstance(template_path, dict) else {}
    flight_path["timestamp"] = int(timestamp_ms)
    flight_path["pathID"] = int(path_id)
    flight_path["aircraftID"] = int(aircraft_id)
    flight_path["individualMissionID"] = int(individual_mission_id)
    flight_path["isFormationFlight"] = False
    flight_path.pop("formationInfo", None)
    _set_source_field(flight_path, str(source))

    _apply_legacy_altitude_profile_to_waypoints(
        waypoints,
        aircraft_id=int(aircraft_id),
        mission_info=mission_info,
    )
    _preserve_first_waypoint_altitude_from_entry(waypoints, entry_coord=entry_coord)
    _enforce_waypoint_altitude_rate_limit_inplace(
        waypoints,
        default_speed_mps=float(speed_mps),
    )
    _stabilize_entry_transition_altitude_inplace(
        waypoints,
        entry_coord=entry_coord,
        default_speed_mps=float(speed_mps),
    )
    if assign_waypoint_ids:
        reassign_unique_waypoint_ids_inplace(
            waypoints,
            waypoint_id_provider=waypoint_id_provider,
        )
    _recompute_waypoint_timeline(waypoints, default_speed_mps=float(speed_mps))
    _apply_runtime_flyover_to_waypoints(waypoints)
    _clear_handover_terminal_markers(waypoints)
    flight_path["waypointList"] = waypoints
    if "lahWaypointList" in flight_path:
        flight_path["lahWaypointList"] = deepcopy(waypoints)
    return flight_path


def _append_handover_terminal_waypoint_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    handover_coord: Dict[str, Any] | None,
    altitude_fn: Callable[[float, float], int] | None,
    speed_mps: float,
    sensor_type: int,
    field_of_view_deg: float,
    tolerance_m: float = 1.0,
) -> bool:
    """Retained API shim: HO reference coordinates no longer extend a path."""

    del waypoints, handover_coord, altitude_fn, speed_mps, sensor_type
    del field_of_view_deg, tolerance_m
    return False


def _strip_filming_properties(waypoints: List[Dict[str, Any]]) -> None:
    for waypoint in waypoints:
        if isinstance(waypoint, dict):
            if waypoint.get(_HANDOVER_TERMINAL_WAYPOINT_MARKER):
                continue
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
        leg_duration_s = 0.0
        if idx > 0:
            prev_waypoint = waypoints[idx - 1] if isinstance(waypoints[idx - 1], dict) else {}
            prev_xy = coord_to_xy(prev_waypoint.get("coordinate"))
            current_xy = coord_to_xy(waypoint.get("coordinate"))
            leg_speed = _to_float(waypoint.get("speed")) or _to_float(prev_waypoint.get("speed")) or default_speed_mps
            if prev_xy is not None and current_xy is not None and leg_speed > 0.0:
                leg_duration_s = _distance_xy(prev_xy, current_xy) / float(leg_speed)
        # lineSearch belongs to the destination waypoint.  The SIM advances the
        # camera sweep while the aircraft flies the incoming leg, so these times
        # are concurrent rather than sequential.  A compact one-WP plan has no
        # incoming leg and therefore executes its sweep on its own.
        sweep_duration_s = _line_search_duration_s(waypoint)
        cumulative_sec += (
            max(float(leg_duration_s), float(sweep_duration_s))
            if idx > 0
            else float(sweep_duration_s)
        )
        cumulative_sec += _loiter_duration_s(waypoint)
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
    preserve_turn_prefix: bool = False,
) -> int:
    if not isinstance(waypoints, list) or not waypoints:
        return 0
    line_search_waypoints = [
        waypoint
        for waypoint in waypoints
        if isinstance(waypoint, dict) and _has_line_search_coordinates(waypoint)
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
    turn_prefix_waypoints = (
        [
            waypoint
            for waypoint in waypoints[: waypoints.index(first_line_wp)]
            if isinstance(waypoint, dict) and bool(waypoint.get("_flyover_dubins_prefix"))
        ]
        if preserve_turn_prefix
        else []
    )
    original_len = len(waypoints)
    waypoints[:] = turn_prefix_waypoints + [start_waypoint] + line_search_waypoints
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
        else:
            # AREA 재계획은 지금까지 이전 임무의 FOV를 이월만 했다.  행 최대
            # 현(스윕 방향)을 기하로 넣고 area 전용 마진으로 실시간 선택한다.
            # None이면(비활성/실패) 기존 이월 FOV 그대로.
            try:
                row_chord_m = float(
                    capture_physics.max_sweep_row_chord_m_xy(
                        polygon_xy, bearing_deg if bearing_deg is not None else 0.0
                    )
                    or 0.0
                )
            except Exception:
                row_chord_m = 0.0
            try:
                physics_area_fov = capture_physics.physics_area_fov_deg(
                    row_length_m=row_chord_m
                )
            except Exception:
                physics_area_fov = None
            if physics_area_fov is not None and float(physics_area_fov) > 0.0:
                info["FOV"] = round(float(physics_area_fov), 3)
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
        info["lineList"] = [{"width": max(0, min(50000, int(round(float(width_m))))), "coordinateList": line_coords}]
        source_width_m = _to_float(path_row.get("sourceLineWidthM"))
        if source_width_m is not None and source_width_m > 0.0:
            info["sourceLineWidthM"] = float(source_width_m)
        source_coordinate_list = path_row.get("sourceCoordinateList")
        if isinstance(source_coordinate_list, list) and len(source_coordinate_list) >= 2:
            info["sourceCoordinateList"] = deepcopy(source_coordinate_list)
        deployment_coordinate_list = path_row.get("lineDeploymentCoordinateList")
        if isinstance(deployment_coordinate_list, list) and len(deployment_coordinate_list) >= 2:
            info["lineDeploymentCoordinateList"] = deepcopy(deployment_coordinate_list)
            info["lineDeploymentDirectionLocked"] = True
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
    assign_waypoint_ids: bool = True,
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
    if assign_waypoint_ids:
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
    handover_coord: Dict[str, Any] | None = None,
    timestamp_ms: int,
    source: str = "MMR",
    waypoint_id_provider: Callable[[], int] | None = None,
    assign_waypoint_ids: bool = True,
    metrics_callback: Callable[[Dict[str, Any]], None] | None = None,
    line_search_multiplier_cap_enabled: bool = True,
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

    def _coords_with_cached_dem(rows_xy: Iterable[Sequence[float]]) -> List[Dict[str, float]]:
        cache_started = time.perf_counter()
        result = get_or_build_line_search_coords(
            rows_xy,
            _xy_rows_to_coords_with_dem_altitude,
            namespace=_line_search_geometry_cache_namespace(),
        )
        _metric_add("lineSearchGeometryCacheMs", _elapsed_ms(cache_started))
        if result.hit:
            metrics["lineSearchGeometryCacheHits"] = int(metrics.get("lineSearchGeometryCacheHits", 0) or 0) + 1
        elif result.skipped:
            metrics["lineSearchGeometryCacheSkips"] = int(metrics.get("lineSearchGeometryCacheSkips", 0) or 0) + 1
        else:
            metrics["lineSearchGeometryCacheMisses"] = int(metrics.get("lineSearchGeometryCacheMisses", 0) or 0) + 1
        metrics["lineSearchGeometryCacheRows"] = int(metrics.get("lineSearchGeometryCacheRows", 0) or 0) + int(result.rowCount)
        metrics["lineSearchGeometryCacheCoords"] = int(metrics.get("lineSearchGeometryCacheCoords", 0) or 0) + int(result.coordCount)
        return result.coords

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
    capture_completion_speed_scale = max(
        1.0,
        min(
            1.5,
            float(
                get_runtime_float(
                    "next_collab_capture_completion_speed_scale",
                    1.10,
                )
            ),
        ),
    )
    geometry_search_speed_scale = max(
        0.1,
        float(base_geometry_search_speed_scale)
        * float(search_speed_scale_multiplier)
        * float(capture_completion_speed_scale),
    )
    first_capture_activation_delay_s = max(
        0.0,
        min(
            10.0,
            float(
                get_runtime_float(
                    "next_collab_first_capture_activation_delay_s",
                    1.50,
                )
            ),
        ),
    )
    area_first_capture_stale_entry_guard_s = max(
        0.0,
        min(
            20.0,
            float(
                get_runtime_float(
                    "next_collab_area_first_capture_stale_entry_guard_s",
                    6.00,
                )
            ),
        ),
    )
    area_first_capture_activation_delay_s = (
        float(first_capture_activation_delay_s)
        + float(area_first_capture_stale_entry_guard_s)
    )
    metrics["searchSpeedBaseScale"] = round(float(base_geometry_search_speed_scale), 3)
    metrics["searchSpeedScaleMultiplier"] = round(float(search_speed_scale_multiplier), 3)
    metrics["captureCompletionSpeedScale"] = round(
        float(capture_completion_speed_scale),
        3,
    )
    metrics["searchSpeedScale"] = round(float(geometry_search_speed_scale), 3)
    # Keep the old metric name for log/dashboard readers while the active
    # setting is shared by LINE and AREA.
    metrics["areaScanCompletionSpeedScale"] = round(
        float(capture_completion_speed_scale),
        3,
    )
    metrics["firstCaptureActivationDelayS"] = round(
        float(first_capture_activation_delay_s),
        3,
    )
    metrics["areaFirstCaptureStaleEntryGuardS"] = round(
        float(area_first_capture_stale_entry_guard_s),
        3,
    )
    metrics["transitSpeedMps"] = round(float(transit_speed_mps), 3)
    metrics["pathRowTransitSpeedMps"] = (
        round(float(path_row_transit_speed_mps), 3)
        if path_row_transit_speed_mps is not None
        else None
    )
    metrics["searchSpeedCruiseMps"] = round(float(search_speed_cruise_mps), 3)
    metrics["areaScanCompletionReferenceSpeedMps"] = round(
        float(transit_speed_mps),
        3,
    )
    metrics["lineSearchMultiplierCapEnabled"] = bool(
        line_search_multiplier_cap_enabled
    )
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
    skip_entry_altitude_preserve = False
    mission_info_dict = mission_info if isinstance(mission_info, dict) else {}
    if is_control_transfer_direct_mission(mission_info_dict):
        direct_path = _build_control_transfer_direct_flight_path(
            template_path=template_path,
            mission_info=mission_info_dict,
            individual_mission_id=int(individual_mission_id),
            path_id=int(path_id),
            aircraft_id=int(aircraft_id),
            entry_coord=entry_coord,
            timestamp_ms=int(timestamp_ms),
            source=str(source),
            speed_mps=float(transit_speed_mps),
            sensor_type=int(sensor_type),
            altitude_fn=altitude_fn,
            waypoint_id_provider=waypoint_id_provider,
            assign_waypoint_ids=bool(assign_waypoint_ids),
        )
        if direct_path is not None:
            direct_waypoints = direct_path.get("waypointList") or []
            metrics["isLineMission"] = True
            metrics["controlTransferDirect"] = True
            metrics["waypoints"] = len(direct_waypoints)
            metrics["lineSearchWaypoints"] = 0
            metrics["lineSearchCoords"] = 0
            metrics["handoverTerminalWaypointCount"] = 0
            metrics["buildTotalMs"] = _elapsed_ms(build_started)
            if metrics_callback is not None:
                try:
                    metrics_callback(dict(metrics))
                except Exception:
                    pass
            return direct_path
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
    scan_three_point_started = time.perf_counter()
    scan_auto_sweep_points = _next_collab_auto_sweep_points()
    scan_points_per_leg = _next_collab_sweep_points_per_leg()
    scan_auto_spacing_m = _next_collab_line_route_wp_spacing_m() if scan_auto_sweep_points else 0.0
    scan_line_three_point_rows_xy = [
        _line_three_point_xy_with_settings(
            points_xy,
            auto_sweep_points=bool(scan_auto_sweep_points),
            points_per_leg=int(scan_points_per_leg),
            spacing_m=float(scan_auto_spacing_m),
        )
        for points_xy in scan_lines_xy
    ]
    metrics["scanLineThreePointMs"] = _elapsed_ms(scan_three_point_started)
    metrics["scanLineThreePointAuto"] = bool(scan_auto_sweep_points)
    metrics["scanLineThreePointSampleCount"] = int(scan_points_per_leg)

    is_line_mission = bool(mission_info_dict.get("lineList")) and not bool(mission_info_dict.get("areaList"))
    metrics["isLineMission"] = bool(is_line_mission)
    area_pass_contract = _area_pass_contract(path_row, mission_info_dict)
    area_pass_scan_lines: Dict[str, List[List[Tuple[float, float]]]] = {}
    area_effective_passes: List[str] = []
    requested_passes: List[str] = []
    area_contract_reverse_route_axis = False
    if not is_line_mission and bool(area_pass_contract.get("explicit")):
        for raw_pass_name in area_pass_contract.get("passes") or []:
            pass_name = _normalize_area_coverage_pass(raw_pass_name)
            if pass_name is not None and pass_name not in requested_passes:
                requested_passes.append(pass_name)
        obligations = area_pass_contract.get("obligations") or {}
        base_scan_lines_xy = [list(line_xy) for line_xy in scan_lines_xy]
        for pass_name in requested_passes:
            obligation = obligations.get(pass_name) if isinstance(obligations, dict) else None
            remaining_detail = (
                obligation.get("remainingDetail")
                if isinstance(obligation, dict)
                else None
            )
            clipped_lines = _clip_area_scan_lines_to_remaining_detail(
                base_scan_lines_xy,
                remaining_detail,
                assigned_polygon_xy=path_row.get("partPolygonXY"),
            )
            # Older pass contracts may carry only the workload label.  Full
            # obligations can safely use the assigned baseline in that case;
            # a partial obligation without geometry must remain empty rather
            # than silently regenerate already-covered ground.
            if (
                not clipped_lines
                and isinstance(obligation, dict)
                and str(obligation.get("obligationKind") or "").strip().lower() == "full"
                and _remaining_detail_geometry_xy(remaining_detail) is None
            ):
                clipped_lines = [list(line_xy) for line_xy in base_scan_lines_xy]
            area_pass_scan_lines[pass_name] = clipped_lines
            if clipped_lines:
                area_effective_passes.append(pass_name)
        if {"forward", "reverse"}.issubset(area_effective_passes):
            forward_lines = area_pass_scan_lines.get("forward") or []
            reverse_lines = area_pass_scan_lines.get("reverse") or []
            if forward_lines and reverse_lines:
                normal_join_m = _distance_xy(
                    _midpoint_xy(forward_lines[-1]) or forward_lines[-1][-1],
                    _midpoint_xy(reverse_lines[-1]) or reverse_lines[-1][-1],
                )
                flipped_join_m = _distance_xy(
                    _midpoint_xy(forward_lines[0]) or forward_lines[0][0],
                    _midpoint_xy(reverse_lines[0]) or reverse_lines[0][0],
                )
                if flipped_join_m + 5.0 < normal_join_m:
                    area_contract_reverse_route_axis = True
                    for pass_name in ("forward", "reverse"):
                        area_pass_scan_lines[pass_name] = [
                            list(reversed(line_xy))
                            for line_xy in reversed(
                                area_pass_scan_lines.get(pass_name) or []
                            )
                        ]
        metrics["areaCoveragePassContractExplicit"] = True
        metrics["areaCoveragePassesRequested"] = list(requested_passes)
        metrics["areaCoveragePassesPlanned"] = list(area_effective_passes)
        metrics["areaCoveragePassScanLines"] = {
            pass_name: len(lines)
            for pass_name, lines in area_pass_scan_lines.items()
        }
        metrics["areaCoveragePassContractReverseAxis"] = bool(
            area_contract_reverse_route_axis
        )
        scan_lines_xy = (
            area_pass_scan_lines.get("forward")
            or area_pass_scan_lines.get("reverse")
            or []
        )
        scan_line_three_point_rows_xy = [
            _line_three_point_xy_with_settings(
                points_xy,
                auto_sweep_points=bool(scan_auto_sweep_points),
                points_per_leg=int(scan_points_per_leg),
                spacing_m=float(scan_auto_spacing_m),
            )
            for points_xy in scan_lines_xy
        ]
        metrics["scanLines"] = len(scan_lines_xy)
        metrics["scanLinePoints"] = sum(len(points_xy) for points_xy in scan_lines_xy)
    else:
        metrics["areaCoveragePassContractExplicit"] = False
    area_contract_no_work = bool(
        not is_line_mission
        and area_pass_contract.get("explicit")
        and not area_effective_passes
    )
    area_primary_pass = (
        "forward"
        if "forward" in area_effective_passes
        else "reverse"
        if "reverse" in area_effective_passes
        else None
    )
    area_route_path_row = (
        _reverse_area_route_context(path_row)
        if area_contract_reverse_route_axis
        else path_row
    )
    if area_primary_pass == "reverse" and "forward" not in area_effective_passes:
        independent_return_preoriented = bool(
            str(area_pass_contract.get("assignmentMode") or "").strip().lower()
            == "independent_available_uav_division"
            and _normalize_area_coverage_pass(
                area_pass_contract.get("assignedPass")
                or path_row.get("areaAssignedCoveragePass")
            )
            == "reverse"
        )
        if independent_return_preoriented:
            # The independent RETURN division was planned again from the
            # predicted OUT exits.  Its row and sweep order already begin at
            # the outbound terminal (the upper/return-side edge). Reversing it
            # here a second time changes only the label to RETURN while the
            # actual lineSearch still flies bottom-to-top.
            scan_lines_xy = [
                list(line_xy)
                for line_xy in (area_pass_scan_lines.get("reverse") or [])
            ]
            area_route_path_row = path_row
            metrics["areaCoverageReversePreoriented"] = True
        else:
            # Persisted remaining-detail polygons retain canonical outbound
            # ordering. Reverse both sweep rows and route axis for true RET.
            scan_lines_xy = [
                list(reversed(line_xy))
                for line_xy in reversed(area_pass_scan_lines.get("reverse") or [])
            ]
            area_route_path_row = _reverse_area_route_context(path_row)
            metrics["areaCoverageReversePreoriented"] = False
        scan_line_three_point_rows_xy = [
            _line_three_point_xy_with_settings(
                points_xy,
                auto_sweep_points=bool(scan_auto_sweep_points),
                points_per_leg=int(scan_points_per_leg),
                spacing_m=float(scan_auto_spacing_m),
            )
            for points_xy in scan_lines_xy
        ]
        metrics["areaCoveragePrimaryDirection"] = "reverse"
    elif area_primary_pass is not None:
        metrics["areaCoveragePrimaryDirection"] = (
            "forward_reverse_axis"
            if area_contract_reverse_route_axis
            else str(area_primary_pass)
        )
    if is_line_mission:
        default_fov_deg = _clamp_line_fov_deg(default_fov_deg)
        first_line_base_fov_deg = _clamp_line_fov_deg(first_line_base_fov_deg, default_fov_deg)
        field_of_view_deg = _clamp_line_fov_deg(field_of_view_deg, default_fov_deg)
    entry_xy: Tuple[float, float] | None = None
    if isinstance(entry_coord, dict):
        raw_entry_xy = coord_to_xy(entry_coord)
        if raw_entry_xy is not None:
            entry_xy = (float(raw_entry_xy[0]), float(raw_entry_xy[1]))
    # AREA 진입점 앵커 방향: 진입이 서펜타인의 끝쪽이면 행 순서·행 방향을 통째로
    # 뒤집는다.  LINE 은 이미 같은 반전(진입-근접 앵커)을 하는데 AREA 는 플래너의
    # 정방향(canonical outbound) 순서를 그대로 렌더링해서, 한 기체가 연속으로 두
    # 조각을 받는 순차 분할에서 두 번째 조각도 첫 조각과 같은 방향으로 "올라가는"
    # 경로가 나왔다 — 끝점에서 이어받으면 내려오는 경로가 되어야 한다.
    # 명시적 패스 계약(out/turn/return)은 자체 방향 기계가 있으므로 건드리지 않는다.
    if (
        not is_line_mission
        and scan_lines_xy
        and entry_xy is not None
        and area_primary_pass is None
        and not area_contract_reverse_route_axis
        and _normalize_area_coverage_pass(
            path_row.get("areaAssignedCoveragePass") or path_row.get("activeCoveragePass")
        )
        is None
    ):
        route_start_xy = (
            _xy_pair(area_route_path_row.get("areaSweepRouteStartXY"))
            or _xy_pair(area_route_path_row.get("waypointStartXY"))
            or (
                (float(scan_lines_xy[0][0][0]), float(scan_lines_xy[0][0][1]))
                if scan_lines_xy[0]
                else None
            )
        )
        route_end_xy = (
            _xy_pair(area_route_path_row.get("areaSweepRouteEndXY"))
            or _xy_pair(area_route_path_row.get("waypointEndXY"))
            or (
                (float(scan_lines_xy[-1][-1][0]), float(scan_lines_xy[-1][-1][1]))
                if scan_lines_xy[-1]
                else None
            )
        )
        sequential_direction_lock = path_row.get(
            "areaSingleAircraftExecutionReversed"
        )
        should_reverse_for_entry = bool(
            route_start_xy is not None
            and route_end_xy is not None
            and _distance_xy(entry_xy, route_end_xy) + 1e-6
            < _distance_xy(entry_xy, route_start_xy)
        )
        if sequential_direction_lock is not None:
            should_reverse_for_entry = bool(sequential_direction_lock)
        if should_reverse_for_entry:
            scan_lines_xy = [
                list(reversed(line_xy)) for line_xy in reversed(scan_lines_xy)
            ]
            scan_line_three_point_rows_xy = [
                _line_three_point_xy_with_settings(
                    points_xy,
                    auto_sweep_points=bool(scan_auto_sweep_points),
                    points_per_leg=int(scan_points_per_leg),
                    spacing_m=float(scan_auto_spacing_m),
                )
                for points_xy in scan_lines_xy
            ]
            area_route_path_row = _reverse_area_route_context(path_row)
            metrics["areaEntryAnchoredReverse"] = True
    outer_endpoint_contract_allowed = bool(
        not is_line_mission
        and area_primary_pass is None
        and not area_contract_reverse_route_axis
        and _normalize_area_coverage_pass(
            path_row.get("areaAssignedCoveragePass")
            or path_row.get("activeCoveragePass")
        )
        is None
    )
    if outer_endpoint_contract_allowed:
        scan_lines_xy, outer_endpoint_reversed = (
            _orient_area_scan_endpoints_from_outer_side(
                scan_lines_xy,
                path_row,
            )
        )
        if bool(path_row.get("areaOuterFirstSweep")):
            metrics["areaOuterFirstSweep"] = True
            metrics["areaOuterSide"] = str(
                path_row.get("areaOuterSide") or ""
            ).strip().lower()
        if outer_endpoint_reversed:
            scan_line_three_point_rows_xy = [
                _line_three_point_xy_with_settings(
                    points_xy,
                    auto_sweep_points=bool(scan_auto_sweep_points),
                    points_per_leg=int(scan_points_per_leg),
                    spacing_m=float(scan_auto_spacing_m),
                )
                for points_xy in scan_lines_xy
            ]
            metrics["areaOuterEndpointReversed"] = True
    transition_points_xy: List[Tuple[float, float]] = []
    locked_line_turn_prefix_xy: List[Tuple[float, float]] = []
    if is_line_mission:
        locked_line_turn_prefix_xy = _locked_line_turn_prefix_xy(
            path_row,
            entry_xy=entry_xy,
        )
        metrics["lineTurnPrefixPoints"] = len(locked_line_turn_prefix_xy)
        metrics["lineTurnDirectionLocked"] = bool(locked_line_turn_prefix_xy)
        # A high-confidence live turn keeps its observed-direction arc through
        # the tangent.  Uncertain/straight entries retain the proven T'-only
        # behavior used before this Line-specific stabilization.
        path_row_transition_points = (
            (*locked_line_turn_prefix_xy, path_row.get("entryTPrimeXY"))
            if isinstance(path_row.get("entryTPrimeXY"), (tuple, list))
            else tuple(locked_line_turn_prefix_xy)
        )
    else:
        path_row_transition_points = (
            area_route_path_row.get("entryTPrimeXY"),
            area_route_path_row.get("waypointStartXY"),
        )
        if bool(path_row.get("areaSingleAircraftSequentialSplit")):
            # The planner still uses T'/start/out-leg geometry to choose the
            # route and its direction, but a sequential AREA pair is handed
            # over at the previous area's real capture exit. Command the UAV
            # directly to the first capture waypoint; helper ingress points
            # create the long offset detour this contract is meant to avoid.
            path_row_transition_points = ()
            metrics["areaSequentialIngressHelpersEmitted"] = False
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

    line_anchor_context: Dict[str, Any] | None = None
    if is_line_mission:
        line_anchor_context_started = time.perf_counter()
        line_anchor_context = _line_anchor_context_for_path_row(path_row, scan_lines_xy)
        metrics["lineAnchorContextMs"] = _elapsed_ms(line_anchor_context_started)
        route_line_xy = line_anchor_context.get("routeLineXY") if isinstance(line_anchor_context, dict) else []
        route_projection_context = (
            line_anchor_context.get("routeProjectionContext") if isinstance(line_anchor_context, dict) else None
        )
        route_segments = route_projection_context.get("segments") if isinstance(route_projection_context, dict) else []
        metrics["lineAnchorContextRoutePoints"] = len(route_line_xy) if isinstance(route_line_xy, list) else 0
        metrics["lineAnchorContextSegments"] = len(route_segments) if isinstance(route_segments, list) else 0

    line_items_started = time.perf_counter()
    line_sweep_items = _line_sweep_items_xy(path_row)
    line_sweep_items_regenerated_from_scan = False
    if is_line_mission and scan_lines_xy:
        regenerated_line_sweep_items = _line_sweep_items_from_scan_lines(
            path_row,
            scan_lines_xy,
            reference_xy=entry_xy,
            anchor_context=line_anchor_context,
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
            anchor_context=line_anchor_context,
        )
    elif is_line_mission and line_sweep_items:
        metrics["lineSweepAnchorAxisFixups"] = 0
    if (
        is_line_mission
        and len(line_sweep_items) >= 2
        and entry_xy is not None
        and not bool(path_row.get("lineDeploymentDirectionLocked"))
    ):
        first_anchor = line_sweep_items[0].get("anchorXY") if isinstance(line_sweep_items[0], dict) else None
        last_anchor = line_sweep_items[-1].get("anchorXY") if isinstance(line_sweep_items[-1], dict) else None
        if isinstance(first_anchor, tuple) and isinstance(last_anchor, tuple):
            if _distance_xy(entry_xy, last_anchor) + 1e-6 < _distance_xy(entry_xy, first_anchor):
                line_sweep_items.reverse()
    metrics["lineSweepItemsMs"] = _elapsed_ms(line_items_started)
    metrics["lineSweepItems"] = len(line_sweep_items)
    flattened_sweep_xy = _flatten_sweep_three_point_rows_xy(scan_line_three_point_rows_xy)
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
    if is_line_mission and not locked_line_turn_prefix_xy and _line_transition_backtracks(
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

    if area_contract_no_work:
        transition_points_xy = []

    for transition_idx, point_xy in enumerate(transition_points_xy):
        transition_waypoint = _make_hold_waypoint(
            coordinate=_xy_to_coord_with_altitude(point_xy, altitude_fn),
            speed_mps=float(transit_speed_mps),
            sensor_type=int(sensor_type),
            field_of_view_deg=float(field_of_view_deg),
            orientation_coordinate=sweep_orientation_coord if is_line_mission else None,
            waypoint_pass_type=int(PASS_FLYBY),
            flyover_dubins_prefix=True,
        )
        if is_line_mission and transition_idx < len(locked_line_turn_prefix_xy):
            transition_waypoint["_locked_line_turn_prefix"] = True
        waypoints.append(transition_waypoint)

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

    end_xy = area_route_path_row.get("waypointEndXY")
    area_sweep_items: List[Dict[str, Any]] = []
    if not is_line_mission and scan_lines_xy:
        area_items_started = time.perf_counter()
        area_sweep_items = _area_sweep_items_xy(
            area_route_path_row,
            scan_lines_xy,
            deduped_scan_lines_xy=scan_lines_xy,
        )
        metrics["areaSweepItemsMs"] = _elapsed_ms(area_items_started)
        metrics["areaSweepItems"] = len(area_sweep_items)
    area_reciprocal_plan: Dict[str, Any] = {
        "active": False,
        "reason": "not_area_or_no_sweeps",
    }
    metrics["areaReciprocalPassActive"] = False
    reciprocal_requested = False
    if area_sweep_items and reciprocal_requested and not bool(
        template_path.get("isFormationFlight", False)
        if isinstance(template_path, dict)
        else False
    ):
        reciprocal_started = time.perf_counter()
        try:
            reciprocal_turn_radius_m = float(
                _to_float(path_row.get("turnRadiusM"))
                or get_runtime_float("dubins_turn_radius_m", 500.0)
            )
            area_reciprocal_plan = _build_area_reciprocal_pass_plan(
                area_route_path_row,
                scan_lines_xy,
                area_sweep_items,
                turn_radius_m=float(reciprocal_turn_radius_m),
                terminal_speed_mps=float(transit_speed_mps),
                aircraft_id=int(aircraft_id),
                force_active=bool(area_pass_contract.get("explicit")),
            )
        except Exception as exc:
            area_reciprocal_plan = {
                "active": False,
                "reason": "reciprocal_plan_error",
                "error": str(exc),
            }
        metrics["areaReciprocalPlanMs"] = _elapsed_ms(reciprocal_started)
    elif area_sweep_items and not reciprocal_requested:
        area_reciprocal_plan = {
            "active": False,
            "reason": "single_pending_coverage_pass",
        }
    elif area_sweep_items:
        area_reciprocal_plan = {"active": False, "reason": "formation_flight"}
    metrics["areaReciprocalPassReason"] = str(
        area_reciprocal_plan.get("reason") or "inactive"
    )
    terrain_profile = area_reciprocal_plan.get("terrainProfile")
    if isinstance(terrain_profile, dict):
        metrics["areaReciprocalTerrainSamples"] = int(
            terrain_profile.get("sampleCount") or 0
        )
        for metric_key, profile_key in (
            ("areaReciprocalTerrainScore", "terrainScore"),
            ("areaReciprocalTerrainReliefM", "terrainReliefM"),
            ("areaReciprocalTerrainRobustReliefM", "terrainRobustReliefM"),
            ("areaReciprocalTerrainGradeP90", "terrainGradeP90"),
            ("areaReciprocalTerrainRoughness", "terrainRoughness"),
        ):
            value = _to_float(terrain_profile.get(profile_key))
            if value is not None:
                metrics[metric_key] = round(float(value), 4)

    def _area_segment_specs_for_scan_lines(
        pass_scan_lines_xy: Sequence[Sequence[Tuple[float, float]]],
        *,
        route_row: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        """Build grouped capture legs without appending public waypoints."""

        normalized_lines = [
            _dedupe_xy_rows(_xy_rows(line_xy), eps_m=0.5)
            for line_xy in pass_scan_lines_xy
        ]
        normalized_lines = [line_xy for line_xy in normalized_lines if len(line_xy) >= 2]
        if not normalized_lines:
            return []
        items = _area_sweep_items_xy(
            route_row,
            normalized_lines,
            deduped_scan_lines_xy=normalized_lines,
        )
        groups = _group_area_sweep_items_by_spacing(
            items,
            spacing_m=float(_next_collab_area_route_wp_spacing_m()),
            merge_short_tail=True,
        )
        three_point_rows = [
            _line_three_point_xy_with_settings(
                line_xy,
                auto_sweep_points=bool(scan_auto_sweep_points),
                points_per_leg=int(scan_points_per_leg),
                spacing_m=float(scan_auto_spacing_m),
            )
            for line_xy in normalized_lines
        ]
        current_xy = _xy_pair(route_row.get("waypointStartXY"))
        if current_xy is None:
            current_xy = _xy_pair(route_row.get("areaMissionStartXY"))
        if current_xy is None:
            return []
        specs: List[Dict[str, Any]] = []
        previous_rep_sweep_idx: int | None = None
        for group in groups:
            rep_item = group[-1] if group else {}
            anchor_xy = _xy_pair(rep_item.get("anchorXY"))
            if anchor_xy is None:
                continue
            merged_sweep_xy = _collect_area_group_sweep_rows_xy(
                group=group,
                all_sweep_lines_xy=normalized_lines,
                previous_rep_sweep_idx=previous_rep_sweep_idx,
                sweep_three_point_rows_xy=three_point_rows,
            )
            merged_sweep_coords = _coords_with_cached_dem(merged_sweep_xy)
            if len(merged_sweep_xy) < 2 or len(merged_sweep_coords) < 2:
                continue
            if specs and _distance_xy(current_xy, anchor_xy) <= 1.0:
                specs[-1]["sweepXY"] = list(specs[-1]["sweepXY"]) + list(
                    merged_sweep_xy
                )
                specs[-1]["sweepCoords"] = list(specs[-1]["sweepCoords"]) + list(
                    merged_sweep_coords
                )
            else:
                specs.append(
                    {
                        "originXY": (float(current_xy[0]), float(current_xy[1])),
                        "targetXY": (float(anchor_xy[0]), float(anchor_xy[1])),
                        "sweepXY": list(merged_sweep_xy),
                        "sweepCoords": list(merged_sweep_coords),
                    }
                )
                current_xy = anchor_xy
            rep_sweep_idx = _to_float(rep_item.get("sweepIndex"))
            if rep_sweep_idx is not None:
                previous_rep_sweep_idx = int(rep_sweep_idx)
        return specs

    if area_sweep_items:
        area_group_started = time.perf_counter()
        grouped_area_sweeps = _group_area_sweep_items_by_spacing(
            area_sweep_items,
            spacing_m=float(_next_collab_area_route_wp_spacing_m()),
            merge_short_tail=True,
            anchor_to_first_item=bool(
                path_row.get("areaSingleAircraftSequentialSplit")
            ),
        )
        metrics["areaGroupMs"] = _elapsed_ms(area_group_started)
        metrics["areaGroups"] = len(grouped_area_sweeps)
        area_group_item_counts = [len(group) for group in grouped_area_sweeps if group]
        if area_group_item_counts:
            metrics["areaGroupMinItems"] = min(area_group_item_counts)
            metrics["areaGroupMaxItems"] = max(area_group_item_counts)
            metrics["areaGroupItemCounts"] = area_group_item_counts[:16]
        dropped_entry_count = _drop_redundant_area_entry_waypoint_before_sweep(
            waypoints,
            grouped_area_sweeps,
            max_distance_m=max(
                25.0,
                min(300.0, float(line_sweep_spacing_m or 0.0) * 5.0),
            ),
        )
        if dropped_entry_count:
            metrics["areaRedundantEntryWaypointsDropped"] = int(dropped_entry_count)
            skip_entry_altitude_preserve = True
        prev_coord = None
        prev_rep_sweep_idx: int | None = None
        if waypoints and isinstance(waypoints[-1], dict):
            prev_coord = waypoints[-1].get("coordinate")
        if not isinstance(prev_coord, dict):
            prev_coord = entry_coord
        prev_xy = coord_to_xy(prev_coord) if isinstance(prev_coord, dict) else None
        # A density-capped split can produce consecutive groups anchored at
        # (nearly) the same route point; a zero-length leg never gets scan
        # time in the sim, so such groups are folded into the previous WP.
        area_group_merge_threshold_m = min(
            max(25.0, float(line_search_speed_min_transit_m())),
            0.25 * float(_next_collab_area_route_wp_spacing_m()),
        )
        last_leg_origin_xy: Tuple[float, float] | None = None
        last_emitted_sweep_xy: List[Tuple[float, float]] = []
        forward_area_segments: List[Dict[str, Any]] = []
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
                sweep_three_point_rows_xy=scan_line_three_point_rows_xy,
            )
            _metric_add("areaCollectRowsMs", _elapsed_ms(collect_started))
            metrics["areaMergedRows"] = int(metrics.get("areaMergedRows", 0) or 0) + len(merged_sweep_xy)
            dem_started = time.perf_counter()
            merged_sweep_coords = _coords_with_cached_dem(merged_sweep_xy)
            _metric_add("areaDemMs", _elapsed_ms(dem_started))
            metrics["areaMergedCoords"] = int(metrics.get("areaMergedCoords", 0) or 0) + len(merged_sweep_coords)
            if len(merged_sweep_coords) < 2:
                continue

            last_wp = waypoints[-1] if waypoints and isinstance(waypoints[-1], dict) else None
            last_wp_coords = _line_search_coordinate_list(last_wp) if last_wp is not None else None
            if (
                last_wp_coords
                and prev_xy is not None
                and _distance_xy(prev_xy, anchor_xy_pair) <= area_group_merge_threshold_m
            ):
                last_filming = last_wp.get("filmingProperty")
                last_line_search = last_filming.get("lineSearch")
                combined_coords = _merge_line_search_coordinate_lists(
                    list(last_wp_coords), merged_sweep_coords
                )
                last_emitted_sweep_xy = list(last_emitted_sweep_xy) + list(merged_sweep_xy)
                line_search_speed_mps = _estimate_line_search_speed_xy_mps(
                    prev_xy=last_leg_origin_xy,
                    anchor_xy=prev_xy,
                    sweep_xy=last_emitted_sweep_xy,
                    cruise_speed_mps=float(transit_speed_mps),
                    fallback_search_speed_mps=float(search_speed_mps),
                    speed_scale=float(geometry_search_speed_scale),
                    multiplier_cap_enabled=False,
                )
                line_search_speed_mps, merged_leg_speed_mps = _apply_area_scan_rate_slowdown(
                    estimated_search_speed_mps=float(line_search_speed_mps),
                    transit_speed_mps=float(transit_speed_mps),
                )
                if merged_leg_speed_mps < float(transit_speed_mps) - 1e-6:
                    last_wp["speed"] = round(float(merged_leg_speed_mps), 2)
                    metrics["areaScanSlowdownLegs"] = int(
                        metrics.get("areaScanSlowdownLegs", 0) or 0
                    ) + 1
                last_line_search["coordinateList"] = combined_coords
                last_line_search["searchSpeed"] = float(line_search_speed_mps)
                if forward_area_segments and forward_area_segments[-1].get("waypoint") is last_wp:
                    forward_area_segments[-1]["sweepXY"] = list(last_emitted_sweep_xy)
                    forward_area_segments[-1]["sweepCoords"] = list(combined_coords)
                metrics["areaCoincidentGroupsMerged"] = int(
                    metrics.get("areaCoincidentGroupsMerged", 0) or 0
                ) + 1
                rep_sweep_idx = _to_float(rep_item.get("sweepIndex"))
                if rep_sweep_idx is not None:
                    prev_rep_sweep_idx = int(rep_sweep_idx)
                continue

            anchor_started = time.perf_counter()
            anchor_coord = _xy_to_coord_with_altitude(anchor_xy_pair, altitude_fn)
            _metric_add("areaAnchorAltitudeMs", _elapsed_ms(anchor_started))
            speed_started = time.perf_counter()
            # Match strip scan time to the actual leg flight time: use the real
            # waypoint speed as cruise (resolvedVelMps is a scan-rate scaled
            # value, not a flight speed) and skip the multiplier cap so the
            # scan cannot outlive the leg on dense sweep groups.
            line_search_speed_mps = _estimate_line_search_speed_xy_mps(
                prev_xy=prev_xy,
                anchor_xy=anchor_xy_pair,
                sweep_xy=merged_sweep_xy,
                cruise_speed_mps=float(transit_speed_mps),
                fallback_search_speed_mps=float(search_speed_mps),
                speed_scale=float(geometry_search_speed_scale),
                multiplier_cap_enabled=False,
            )
            line_search_speed_mps, group_leg_speed_mps = _apply_area_scan_rate_slowdown(
                estimated_search_speed_mps=float(line_search_speed_mps),
                transit_speed_mps=float(transit_speed_mps),
            )
            if group_leg_speed_mps < float(transit_speed_mps) - 1e-6:
                metrics["areaScanSlowdownLegs"] = int(
                    metrics.get("areaScanSlowdownLegs", 0) or 0
                ) + 1
            _metric_add("areaSpeedMs", _elapsed_ms(speed_started))
            append_started = time.perf_counter()
            area_search_waypoint = _make_line_search_waypoint(
                coordinate=anchor_coord,
                sweep_coords=merged_sweep_coords,
                transit_speed_mps=float(group_leg_speed_mps),
                search_speed_mps=float(line_search_speed_mps),
                sensor_type=int(sensor_type),
                field_of_view_deg=float(field_of_view_deg),
                waypoint_pass_type=3,
            )
            waypoints.append(area_search_waypoint)
            if prev_xy is not None:
                forward_area_segments.append(
                    {
                        "waypoint": area_search_waypoint,
                        "originXY": (float(prev_xy[0]), float(prev_xy[1])),
                        "targetXY": anchor_xy_pair,
                        "sweepXY": list(merged_sweep_xy),
                        "sweepCoords": list(merged_sweep_coords),
                    }
                )
            _metric_add("areaWaypointAppendMs", _elapsed_ms(append_started))
            metrics["areaLineSearchWaypointsBuilt"] = int(
                metrics.get("areaLineSearchWaypointsBuilt", 0) or 0
            ) + 1
            last_leg_origin_xy = prev_xy
            last_emitted_sweep_xy = list(merged_sweep_xy)
            prev_coord = anchor_coord
            prev_xy = anchor_xy_pair
            rep_sweep_idx = _to_float(rep_item.get("sweepIndex"))
            if rep_sweep_idx is not None:
                prev_rep_sweep_idx = int(rep_sweep_idx)

        if bool(area_reciprocal_plan.get("active")) and forward_area_segments:
            reciprocal_append_started = time.perf_counter()
            try:
                terminal_segment = forward_area_segments[-1]
                terminal_waypoint = terminal_segment.get("waypoint")
                terminal_origin_xy = _xy_pair(terminal_segment.get("originXY"))
                terminal_anchor_xy = _xy_pair(terminal_segment.get("targetXY"))
                terminal_sweep_xy = _xy_rows(terminal_segment.get("sweepXY"))
                terminal_sweep_coords = list(terminal_segment.get("sweepCoords") or [])
                if (
                    not isinstance(terminal_waypoint, dict)
                    or terminal_origin_xy is None
                    or terminal_anchor_xy is None
                    or len(terminal_sweep_xy) < 2
                    or len(terminal_sweep_coords) < 2
                    or _distance_xy(terminal_origin_xy, terminal_anchor_xy) <= 1.0
                ):
                    raise ValueError("invalid reciprocal forward capture terminal")
                terminal_filming = terminal_waypoint.get("filmingProperty")
                terminal_line_search = (
                    terminal_filming.get("lineSearch")
                    if isinstance(terminal_filming, dict)
                    else None
                )
                if not isinstance(terminal_line_search, dict):
                    raise ValueError("reciprocal forward terminal has no lineSearch")
                forward_terminal_leg_speed_mps = max(
                    1.0,
                    float(_to_float(terminal_waypoint.get("speed")) or transit_speed_mps),
                )
                forward_terminal_search_speed_mps = max(
                    0.0,
                    float(_to_float(terminal_line_search.get("searchSpeed")) or 0.0),
                )
                exit_unit_xy = _xy_pair(area_reciprocal_plan.get("exitUnitXY"))
                turn_side_sign = _to_float(area_reciprocal_plan.get("turnSideSign"))
                if exit_unit_xy is None or turn_side_sign is None:
                    raise ValueError("reciprocal terminal turn axis missing")
                effective_turn_radius_m, radius_source = _area_reciprocal_turn_radius_m(
                    fallback_radius_m=float(
                        _to_float(path_row.get("turnRadiusM"))
                        or _to_float(area_reciprocal_plan.get("turnRadiusM"))
                        or 500.0
                    ),
                    terminal_speed_mps=float(forward_terminal_leg_speed_mps),
                    aircraft_id=int(aircraft_id),
                )
                turn_geometry = _compact_area_reciprocal_turn_geometry(
                    terminal_anchor_xy=terminal_anchor_xy,
                    exit_unit_xy=exit_unit_xy,
                    turn_side_sign=float(turn_side_sign),
                    radius_m=float(effective_turn_radius_m),
                )
                area_reciprocal_plan.update(turn_geometry)
                area_reciprocal_plan["turnRadiusSource"] = str(radius_source)
                area_reciprocal_plan["turnTerminalSpeedMps"] = float(
                    forward_terminal_leg_speed_mps
                )
                turn_path_xy = [
                    point_xy
                    for point_xy in (
                        _xy_pair(raw_point)
                        for raw_point in (turn_geometry.get("turnPathXY") or [])
                    )
                    if point_xy is not None
                ]
                reentry_xy = _xy_pair(turn_geometry.get("reentryXY"))
                if (
                    len(turn_path_xy) != 3
                    or reentry_xy is None
                    or _distance_xy(turn_path_xy[0], terminal_anchor_xy) > 1.0
                    or _distance_xy(reentry_xy, terminal_anchor_xy) > 1.0
                ):
                    raise ValueError("reciprocal turn does not join the final sweep anchor")

                # The forward camera leg ends at its actual final sweep anchor.
                # No polygon-support/waypointEnd extension is allowed here.
                last_forward_xy = terminal_anchor_xy
                forward_capture_extension_m = 0.0
                forward_capture_leg_m = round(
                    float(_distance_xy(terminal_origin_xy, terminal_anchor_xy)),
                    3,
                )
                forward_capture_search_speed_mps = round(
                    float(forward_terminal_search_speed_mps),
                    3,
                )
                staged_forward_segments = [
                    dict(segment)
                    for segment in forward_area_segments
                    if isinstance(segment, dict)
                ]
                if len(staged_forward_segments) != len(forward_area_segments):
                    raise ValueError("invalid reciprocal forward segment list")

                reverse_segment_specs = staged_forward_segments
                if bool(area_pass_contract.get("explicit")):
                    reverse_segment_specs = _area_segment_specs_for_scan_lines(
                        area_pass_scan_lines.get("reverse") or [],
                        route_row=area_route_path_row,
                    )
                    if not reverse_segment_specs:
                        raise ValueError("reverse pass obligation produced no capture legs")
                reverse_terminal_xy = _xy_pair(
                    reverse_segment_specs[-1].get("targetXY")
                    if reverse_segment_specs
                    else None
                )
                if (
                    reverse_terminal_xy is None
                    or _distance_xy(reverse_terminal_xy, terminal_anchor_xy) > 5.0
                ):
                    raise ValueError(
                        "forward remainder and reverse obligation do not share a terminal"
                    )

                if _distance_xy(turn_path_xy[0], last_forward_xy) > 5.0:
                    raise ValueError("reciprocal turn does not join the forward pass")

                # turnPathXY[0] is already the completed forward capture WP.
                # The remaining two points are the outside-Area turn gates.
                # They are followed by one camera-ready re-entry/alignment gate
                # on the Area boundary; reverse lineSearch starts only after it.
                emitted_turn_xy = [*turn_path_xy[1:], reentry_xy]
                if len(emitted_turn_xy) != 3:
                    raise ValueError("reciprocal turn waypoint budget exceeded")
                if any(
                    _distance_xy(left_xy, right_xy) <= 1.0
                    for left_xy, right_xy in zip(emitted_turn_xy, emitted_turn_xy[1:])
                ):
                    raise ValueError("reciprocal turn contains consecutive duplicate gates")
                turn_route_xy = [last_forward_xy, *emitted_turn_xy]
                turn_leg_lengths_m = [
                    _distance_xy(left_xy, right_xy)
                    for left_xy, right_xy in zip(turn_route_xy, turn_route_xy[1:])
                ]

                turn_coords = [_xy_to_coord(point_xy) for point_xy in emitted_turn_xy]
                turn_altitude_candidates: List[float] = []
                for coordinate in turn_coords:
                    if altitude_fn is None:
                        continue
                    try:
                        turn_altitude_candidates.append(
                            float(
                                altitude_fn(
                                    float(coordinate["latitude"]),
                                    float(coordinate["longitude"]),
                                )
                            )
                        )
                    except Exception:
                        continue
                try:
                    ground_rows = _dem_altitudes_for_pairs_cached(
                        [
                            (float(coord["latitude"]), float(coord["longitude"]))
                            for coord in turn_coords
                        ],
                        invalid_default=None,
                    )
                except Exception:
                    ground_rows = []
                finite_ground_rows = [
                    float(value)
                    for value in ground_rows
                    if value is not None
                    and _to_float(value) is not None
                    and math.isfinite(float(value))
                ]
                if finite_ground_rows:
                    turn_altitude_candidates.append(
                        max(finite_ground_rows) + float(_aircraft_alt_offset_m(int(aircraft_id)))
                    )
                terminal_coord = (
                    terminal_waypoint.get("coordinate")
                    if isinstance(terminal_waypoint.get("coordinate"), dict)
                    else {}
                )
                terminal_altitude_m = _to_float(terminal_coord.get("altitude"))
                if terminal_altitude_m is not None:
                    turn_altitude_candidates.append(float(terminal_altitude_m))
                turn_corridor_altitude_m = max(turn_altitude_candidates or [0.0])

                turn_waypoints: List[Dict[str, Any]] = []
                # Keep the sensor prepared throughout the non-capturing turn.
                # This follows the same contract as ingress/lead waypoints:
                # coordinate-fixed mode looks at the first ground coordinate
                # that the next (reverse) lineSearch will consume.
                first_reverse_segment = reverse_segment_specs[-1]
                first_reverse_sweep_coords = list(
                    first_reverse_segment.get("sweepCoords") or []
                )
                reverse_capture_start_coord = (
                    deepcopy(first_reverse_sweep_coords[-1])
                    if first_reverse_sweep_coords
                    and isinstance(first_reverse_sweep_coords[-1], dict)
                    else _xy_to_coord(reentry_xy)
                )
                for turn_idx, (point_xy, coordinate) in enumerate(
                    zip(emitted_turn_xy, turn_coords)
                ):
                    coordinate = dict(coordinate)
                    coordinate["altitude"] = int(round(float(turn_corridor_altitude_m)))
                    phase = (
                        "turn_entry"
                        if turn_idx == 0
                        else "reentry"
                        if turn_idx == len(emitted_turn_xy) - 1
                        else "turn_exit"
                    )
                    waypoint = _make_hold_waypoint(
                        coordinate=coordinate,
                        speed_mps=float(forward_terminal_leg_speed_mps),
                        sensor_type=int(sensor_type),
                        field_of_view_deg=float(field_of_view_deg),
                        orientation_coordinate=reverse_capture_start_coord,
                        waypoint_pass_type=int(PASS_FLYBY),
                    )
                    waypoint["areaTurnRole"] = "reciprocal_turn"
                    waypoint["areaTurnPhase"] = phase
                    waypoint["_locked_area_reciprocal_turn"] = True
                    waypoint["_locked_area_reciprocal_turn_altitude_m"] = float(
                        turn_corridor_altitude_m
                    )
                    waypoint["_locked_area_reciprocal_turn_speed_mps"] = float(
                        forward_terminal_leg_speed_mps
                    )
                    turn_waypoints.append(waypoint)

                reverse_waypoints: List[Dict[str, Any]] = []
                reverse_current_xy = reentry_xy
                reverse_coord_count = 0
                for segment in reversed(reverse_segment_specs):
                    reverse_target_xy = _xy_pair(segment.get("originXY"))
                    reverse_sweep_xy = list(
                        reversed(_dedupe_xy_rows(_xy_rows(segment.get("sweepXY")), eps_m=0.5))
                    )
                    reverse_sweep_coords = list(
                        reversed(deepcopy(segment.get("sweepCoords") or []))
                    )
                    if (
                        reverse_target_xy is None
                        or len(reverse_sweep_xy) < 2
                        or len(reverse_sweep_coords) < 2
                        or _distance_xy(reverse_current_xy, reverse_target_xy) <= 1.0
                    ):
                        raise ValueError("invalid reciprocal capture leg")
                    source_waypoint = segment.get("waypoint")
                    source_filming = (
                        source_waypoint.get("filmingProperty")
                        if isinstance(source_waypoint, dict)
                        else None
                    )
                    source_line_search = (
                        source_filming.get("lineSearch")
                        if isinstance(source_filming, dict)
                        else None
                    )
                    source_leg_speed_mps = (
                        _to_float(source_waypoint.get("speed"))
                        if isinstance(source_waypoint, dict)
                        else None
                    )
                    source_search_speed_mps = (
                        _to_float(source_line_search.get("searchSpeed"))
                        if isinstance(source_line_search, dict)
                        else None
                    )
                    if (
                        source_leg_speed_mps is not None
                        and source_search_speed_mps is not None
                    ):
                        # Ordinary reciprocal plans remain exact mirrors.
                        reverse_leg_speed_mps = float(source_leg_speed_mps)
                        reverse_search_speed_mps = float(source_search_speed_mps)
                    else:
                        reverse_search_speed_mps = _estimate_line_search_speed_xy_mps(
                            prev_xy=reverse_current_xy,
                            anchor_xy=reverse_target_xy,
                            sweep_xy=reverse_sweep_xy,
                            cruise_speed_mps=float(transit_speed_mps),
                            fallback_search_speed_mps=float(search_speed_mps),
                            speed_scale=float(geometry_search_speed_scale),
                            multiplier_cap_enabled=False,
                        )
                        (
                            reverse_search_speed_mps,
                            reverse_leg_speed_mps,
                        ) = _apply_area_scan_rate_slowdown(
                            estimated_search_speed_mps=float(reverse_search_speed_mps),
                            transit_speed_mps=float(transit_speed_mps),
                        )
                    reverse_waypoint = _make_line_search_waypoint(
                        coordinate=_xy_to_coord_with_altitude(reverse_target_xy, altitude_fn),
                        sweep_coords=reverse_sweep_coords,
                        transit_speed_mps=float(reverse_leg_speed_mps),
                        search_speed_mps=float(reverse_search_speed_mps),
                        sensor_type=int(sensor_type),
                        field_of_view_deg=float(field_of_view_deg),
                        waypoint_pass_type=3,
                    )
                    reverse_waypoint["areaCoveragePass"] = "reverse"
                    reverse_waypoints.append(reverse_waypoint)
                    reverse_coord_count += len(reverse_sweep_coords)
                    reverse_current_xy = reverse_target_xy

                max_reverse_coords = max(
                    1000,
                    int(
                        get_runtime_int(
                            "next_collab_area_reciprocal_max_extra_search_coords",
                            50_000,
                        )
                    ),
                )
                if reverse_coord_count > max_reverse_coords:
                    raise ValueError("reciprocal lineSearch payload budget exceeded")
                if len(reverse_waypoints) != len(reverse_segment_specs):
                    raise ValueError("reciprocal coverage waypoint count mismatch")

                metrics["areaReciprocalForwardCaptureExtensionM"] = (
                    forward_capture_extension_m
                )
                metrics["areaReciprocalForwardCaptureLegM"] = forward_capture_leg_m
                metrics["areaReciprocalForwardCaptureSearchSpeedMps"] = (
                    forward_capture_search_speed_mps
                )
                for segment in forward_area_segments:
                    waypoint = segment.get("waypoint")
                    if isinstance(waypoint, dict):
                        waypoint["areaCoveragePass"] = "forward"
                waypoints.extend(turn_waypoints)
                waypoints.extend(reverse_waypoints)
                metrics["areaReciprocalPassActive"] = True
                metrics["areaReciprocalPassReason"] = str(
                    area_reciprocal_plan.get("reason") or "rugged_terrain"
                )
                metrics["areaReciprocalPassCount"] = 2
                metrics["areaReciprocalPolicy"] = str(
                    area_reciprocal_plan.get("policy") or "minimal_two_gate_reciprocal"
                )
                metrics["areaReciprocalForwardWaypoints"] = len(forward_area_segments)
                metrics["areaReciprocalReverseWaypoints"] = len(reverse_waypoints)
                metrics["areaReciprocalTurnWaypoints"] = len(turn_waypoints)
                metrics["areaReciprocalReverseCoords"] = int(reverse_coord_count)
                metrics["areaReciprocalTurnType"] = str(
                    area_reciprocal_plan.get("turnType") or ""
                )
                metrics["areaReciprocalTurnSide"] = str(
                    area_reciprocal_plan.get("turnSide") or ""
                )
                metrics["areaReciprocalTurnRadiusM"] = round(
                    float(area_reciprocal_plan.get("turnRadiusM") or 0.0),
                    3,
                )
                metrics["areaReciprocalTurnGateRadiusScale"] = round(
                    float(area_reciprocal_plan.get("turnGateRadiusScale") or 0.0),
                    3,
                )
                metrics["areaReciprocalTurnForwardM"] = round(
                    float(area_reciprocal_plan.get("turnForwardM") or 0.0),
                    3,
                )
                metrics["areaReciprocalTurnLateralM"] = round(
                    float(area_reciprocal_plan.get("turnLateralM") or 0.0),
                    3,
                )
                metrics["areaReciprocalTurnCorridorAltitudeM"] = round(
                    float(turn_corridor_altitude_m),
                    1,
                )
                metrics["areaReciprocalTurnRadiusSource"] = str(
                    area_reciprocal_plan.get("turnRadiusSource") or ""
                )
                metrics["areaReciprocalTurnTerminalSpeedMps"] = round(
                    float(area_reciprocal_plan.get("turnTerminalSpeedMps") or 0.0),
                    3,
                )
                metrics["areaReciprocalTurnLegLengthsM"] = [
                    round(float(value), 3) for value in turn_leg_lengths_m
                ]
                metrics["areaReciprocalTurnTotalM"] = round(
                    float(sum(turn_leg_lengths_m)),
                    3,
                )
            except Exception as exc:
                # Optional terrain-triggered reciprocity keeps the proven
                # legacy single pass on failure.  An explicit OUT contract is
                # different: publishing only its forward remainder would mark
                # an incomplete mission as valid, so abort that replacement.
                metrics["areaReciprocalPassActive"] = False
                metrics["areaReciprocalPassReason"] = "reciprocal_append_failed"
                metrics["areaReciprocalPassError"] = str(exc)
                if bool(area_pass_contract.get("explicit")):
                    raise ValueError(
                        f"explicit reciprocal Area contract could not be built: {exc}"
                    ) from exc
            metrics["areaReciprocalAppendMs"] = _elapsed_ms(reciprocal_append_started)
        if bool(area_pass_contract.get("explicit")) and area_primary_pass is not None:
            # A TURN/RET replan intentionally has one capture direction and no
            # reciprocal append.  Keep the same public pass marker used by the
            # two-pass route so monitoring and SIM visualization do not have to
            # infer ownership from waypoint order.
            for segment in forward_area_segments:
                waypoint = segment.get("waypoint") if isinstance(segment, dict) else None
                if (
                    isinstance(waypoint, dict)
                    and not waypoint.get("areaCoveragePass")
                ):
                    waypoint["areaCoveragePass"] = str(area_primary_pass)
            if not bool(metrics.get("areaReciprocalPassActive")):
                metrics["areaReciprocalPassCount"] = 1
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
                sweep_three_point_rows_xy=scan_line_three_point_rows_xy,
            )
            dem_started = time.perf_counter()
            merged_sweep_coords = _coords_with_cached_dem(merged_sweep_xy)
            _metric_add("lineDemMs", _elapsed_ms(dem_started))
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
                    anchor_context=line_anchor_context,
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
                multiplier_cap_enabled=bool(line_search_multiplier_cap_enabled),
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
                anchor_context=line_anchor_context,
            )
        if line_search_anchor_xy is None:
            line_search_anchor_xy = (float(end_xy[0]), float(end_xy[1]))
        line_search_anchor_coord = _xy_to_coord_with_altitude(line_search_anchor_xy, altitude_fn)
        dem_started = time.perf_counter()
        sweep_coords = _coords_with_cached_dem(flattened_sweep_xy)
        _metric_add("lineDemMs", _elapsed_ms(dem_started))
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
        fallback_speed_scale = float(geometry_search_speed_scale)
        line_search_speed_mps = _estimate_line_search_speed_xy_mps(
            prev_xy=prev_xy_for_speed,
            anchor_xy=line_search_anchor_xy,
            sweep_xy=flattened_sweep_xy,
            cruise_speed_mps=float(
                search_speed_cruise_mps if is_line_mission else transit_speed_mps
            ),
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(fallback_speed_scale),
            reference_xy=reference_xy,
            multiplier_cap_enabled=(
                bool(line_search_multiplier_cap_enabled)
                if is_line_mission
                else False
            ),
        )
        fallback_leg_speed_mps = float(transit_speed_mps)
        if not is_line_mission:
            line_search_speed_mps, fallback_leg_speed_mps = _apply_area_scan_rate_slowdown(
                estimated_search_speed_mps=float(line_search_speed_mps),
                transit_speed_mps=float(transit_speed_mps),
            )
            if fallback_leg_speed_mps < float(transit_speed_mps) - 1e-6:
                metrics["areaScanSlowdownLegs"] = int(
                    metrics.get("areaScanSlowdownLegs", 0) or 0
                ) + 1
        waypoint_fov_deg = (
            _next_collab_first_line_search_fov_deg(float(first_line_base_fov_deg))
            if is_line_mission and first_line_fov_boost_active
            else float(field_of_view_deg)
        )
        waypoints.append(
            _make_line_search_waypoint(
                coordinate=line_search_anchor_coord,
                sweep_coords=sweep_coords,
                transit_speed_mps=float(fallback_leg_speed_mps),
                search_speed_mps=float(line_search_speed_mps),
                sensor_type=int(sensor_type),
                field_of_view_deg=float(waypoint_fov_deg),
                waypoint_pass_type=3,
            )
        )
    elif (
        not area_contract_no_work
        and isinstance(end_xy, (tuple, list))
        and len(end_xy) >= 2
    ):
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

    if (
        not area_contract_no_work
        and not waypoints
        and isinstance(path_row.get("targetXY"), (tuple, list))
    ):
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

    if (
        not is_line_mission
        and bool(area_pass_contract.get("explicit"))
        and area_primary_pass is not None
    ):
        for waypoint in waypoints:
            filming = waypoint.get("filmingProperty") if isinstance(waypoint, dict) else None
            line_search = filming.get("lineSearch") if isinstance(filming, dict) else None
            if isinstance(line_search, dict) and line_search:
                waypoint.setdefault("areaCoveragePass", str(area_primary_pass))
        emitted_passes = {
            str(waypoint.get("areaCoveragePass"))
            for waypoint in waypoints
            if isinstance(waypoint, dict)
            and _has_line_search_coordinates(waypoint)
            and waypoint.get("areaCoveragePass")
        }
        missing_passes = set(area_effective_passes) - emitted_passes
        if missing_passes:
            raise ValueError(
                "explicit Area coverage passes were not emitted: "
                + ",".join(sorted(missing_passes))
            )
        metrics["areaCoveragePassesEmitted"] = sorted(emitted_passes)
        acquisition_ids: Dict[str, str] = {}
        for waypoint in waypoints:
            if not isinstance(waypoint, dict) or not _has_line_search_coordinates(waypoint):
                continue
            pass_name = _normalize_area_coverage_pass(waypoint.get("areaCoveragePass"))
            if pass_name is None:
                continue
            acquisition_id = acquisition_ids.setdefault(
                pass_name,
                _area_coverage_acquisition_id(
                    path_row,
                    mission_info_dict,
                    aircraft_id=int(aircraft_id),
                    path_id=int(path_id),
                    pass_name=pass_name,
                    timestamp_ms=int(timestamp_ms),
                    individual_mission_id=int(individual_mission_id),
                ),
            )
            waypoint["coverageAcquisitionID"] = str(acquisition_id)
            filming = waypoint.get("filmingProperty")
            line_search = filming.get("lineSearch") if isinstance(filming, dict) else None
            if isinstance(line_search, dict):
                line_search["coverageAcquisitionID"] = str(acquisition_id)
        metrics["areaCoverageAcquisitionIDs"] = dict(acquisition_ids)

    if not is_line_mission and not bool(area_pass_contract.get("explicit")):
        acquisition_ids: Dict[str, str] = {}
        for waypoint in waypoints:
            if not isinstance(waypoint, dict) or not _has_line_search_coordinates(waypoint):
                continue
            waypoint.pop("areaCoveragePass", None)
            pass_name = "forward"
            acquisition_id = acquisition_ids.setdefault(
                "single",
                _area_coverage_acquisition_id(
                    path_row,
                    mission_info_dict,
                    aircraft_id=int(aircraft_id),
                    path_id=int(path_id),
                    pass_name=pass_name,
                    timestamp_ms=int(timestamp_ms),
                    individual_mission_id=int(individual_mission_id),
                ),
            )
            waypoint["coverageAcquisitionID"] = str(acquisition_id)
            filming = waypoint.get("filmingProperty")
            line_search = filming.get("lineSearch") if isinstance(filming, dict) else None
            if isinstance(line_search, dict):
                line_search["coverageAcquisitionID"] = str(acquisition_id)
        if acquisition_ids:
            metrics["areaCoverageAcquisitionIDs"] = dict(acquisition_ids)

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
            metrics["lineSquashReanchorMs"] = 0.0
        else:
            step_started = time.perf_counter()
            line_reanchor_count = _reanchor_line_search_waypoints_to_first_sweep(
                final_waypoints,
                offset_m=float(_line_route_offset_m(path_row)),
                altitude_fn=altitude_fn,
                reference_xy_for_offset=entry_xy,
                path_row=path_row,
                scan_lines_xy=scan_lines_xy,
                anchor_context=line_anchor_context,
            )
            metrics["lineSquashReanchorMs"] = _elapsed_ms(step_started)
        metrics["lineReanchoredWaypoints"] = int(line_reanchor_count)
        step_started = time.perf_counter()
        pruned_line_waypoints = _simplify_line_waypoints_to_start_and_search(
            final_waypoints,
            speed_mps=float(transit_speed_mps),
            sensor_type=int(sensor_type),
            field_of_view_deg=float(field_of_view_deg),
            # The observed turn arc remains part of assignment/orientation
            # planning, but it must not become a tight set of commanded WPs.
            # Start the transmitted next-collab Line path at the real search
            # entry instead (also removes a coincident T' helper waypoint).
            preserve_turn_prefix=False,
        )
        metrics["lineTurnPrefixEmitted"] = False
        metrics["lineSquashSimplifyMs"] = _elapsed_ms(step_started)
        metrics["linePrunedAnchorWaypoints"] = int(pruned_line_waypoints)
        step_started = time.perf_counter()
        snapped_endpoint_waypoints = _snap_last_line_search_waypoint_to_route_endpoint(
            final_waypoints,
            path_row=path_row,
            scan_lines_xy=scan_lines_xy,
            altitude_fn=altitude_fn,
            anchor_context=line_anchor_context,
        )
        metrics["lineSquashSnapMs"] = _elapsed_ms(step_started)
        metrics["lineEndpointAnchorsSnapped"] = int(snapped_endpoint_waypoints)
        step_started = time.perf_counter()
        trailing_before = len(final_waypoints)
        _squash_trailing_short_line_search_waypoints(
            final_waypoints,
            spacing_m=max(
                float(line_search_speed_min_transit_m()),
                0.25 * float(_next_collab_line_route_wp_spacing_m()),
            ),
            transit_speed_mps=float(transit_speed_mps),
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
        )
        metrics["lineTrailingSquashMs"] = _elapsed_ms(step_started)
        metrics["lineTrailingWaypointsMerged"] = int(trailing_before - len(final_waypoints))
        step_started = time.perf_counter()
        _recompute_first_line_search_speed_from_entry_inplace(
            final_waypoints,
            entry_coord=entry_coord if isinstance(entry_coord, dict) else None,
            transit_speed_mps=float(search_speed_cruise_mps),
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
            multiplier_cap_enabled=bool(line_search_multiplier_cap_enabled),
            activation_delay_s=float(first_capture_activation_delay_s),
        )
        resynchronized_line_capture_count = (
            _resynchronize_line_capture_speeds_inplace(
                final_waypoints,
                fallback_search_speed_mps=float(search_speed_mps),
                speed_scale=float(geometry_search_speed_scale),
                default_transit_speed_mps=float(transit_speed_mps),
                multiplier_cap_enabled=bool(line_search_multiplier_cap_enabled),
                activation_entry_coord=(
                    entry_coord if isinstance(entry_coord, dict) else None
                ),
                activation_delay_s=float(first_capture_activation_delay_s),
            )
        )
        metrics["lineCaptureSpeedsResynced3D"] = int(
            resynchronized_line_capture_count
        )
        metrics["lineSquashSpeedMs"] = _elapsed_ms(step_started)
        metrics["lineSquashMs"] = _elapsed_ms(line_squash_started)
        step_started = time.perf_counter()
        _clamp_line_waypoint_fov_inplace(final_waypoints)
        metrics["lineSquashClampFovMs"] = _elapsed_ms(step_started)
    elif bool(path_row.get("areaSingleAircraftSequentialSplit")):
        # Keep the natural distance-based capture groups. WP1 is the first real
        # sweep-axis anchor (not the first group's terminal anchor), followed by
        # every distance-spaced capture WP. A short Area therefore remains one
        # full start->end leg instead of being force-split into two half-legs.
        capture_waypoints = [
            waypoint
            for waypoint in final_waypoints
            if isinstance(waypoint, dict)
            and _has_line_search_coordinates(waypoint)
        ]
        removed_helpers = len(final_waypoints) - len(capture_waypoints)
        if final_waypoints and not capture_waypoints:
            raise ValueError(
                "sequential AREA path produced no capture waypoint"
            )
        first_sweep_anchor_xy = (
            _xy_pair(area_sweep_items[0].get("anchorXY"))
            if area_sweep_items and isinstance(area_sweep_items[0], dict)
            else None
        )
        if first_sweep_anchor_xy is None:
            raise ValueError(
                "sequential AREA path produced no first sweep-axis anchor"
            )
        first_capture_coords = _line_search_coordinate_list(capture_waypoints[0])
        orientation_coordinate = (
            first_capture_coords[0] if first_capture_coords else None
        )
        start_waypoint = _make_hold_waypoint(
            coordinate=_xy_to_coord_with_altitude(
                first_sweep_anchor_xy,
                altitude_fn,
            ),
            speed_mps=float(transit_speed_mps),
            sensor_type=int(sensor_type),
            field_of_view_deg=float(field_of_view_deg),
            orientation_coordinate=orientation_coordinate,
            waypoint_pass_type=int(PASS_FLYBY),
            include_filming=True,
        )
        final_waypoints[:] = [start_waypoint] + capture_waypoints
        sequential_sequence = int(
            _to_float(path_row.get("areaSingleAircraftSequence")) or 0
        )
        resynchronized_capture_count = _resynchronize_area_capture_speeds_inplace(
            final_waypoints,
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
            default_transit_speed_mps=float(transit_speed_mps),
            activation_entry_coord=(
                entry_coord if isinstance(entry_coord, dict) else None
            ),
            activation_delay_s=float(area_first_capture_activation_delay_s),
        )
        entry_waypoint_reused = bool(
            len(final_waypoints) >= 2
            and not _line_search_coordinate_list(final_waypoints[0])
            and all(
                _line_search_coordinate_list(waypoint)
                for waypoint in final_waypoints[1:]
                if isinstance(waypoint, dict)
            )
        )
        if not entry_waypoint_reused:
            raise ValueError(
                "sequential AREA path could not emit first point plus captures"
            )
        if sequential_sequence >= 2:
            # The first capture-axis point may be approached from the preceding
            # Area, but it must not become a mandatory fly-over corner.
            final_waypoints[0]["_locked_sequential_area_entry_turn"] = True
        metrics["areaSequentialCaptureOnly"] = False
        metrics["areaSequentialEntryWaypointAdded"] = True
        metrics["areaSequentialFirstCaptureReusedAsEntry"] = False
        metrics["areaSequentialEntryWaypointSource"] = "first_sweep_axis_anchor"
        metrics["areaSequentialEntryTurnFlybyLocked"] = bool(
            sequential_sequence >= 2
        )
        metrics["areaSequentialCaptureSpeedsResynced"] = int(
            resynchronized_capture_count
        )
        metrics["areaSequentialActivationEntryConstrained"] = bool(
            isinstance(entry_coord, dict)
        )
        metrics["areaSequentialActivationDelayS"] = round(
            float(area_first_capture_activation_delay_s),
            3,
        )
        metrics["areaSequentialHelperWaypointsRemoved"] = int(
            removed_helpers
        )
    # Hand-over remains 0203 reference data only. Do not extend the final
    # InputMissionPlan route with an implicit terminal waypoint.
    metrics["handoverTerminalWaypointCount"] = 0
    metrics["handoverTerminalMs"] = 0.0
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
    if skip_entry_altitude_preserve:
        metrics["entryAltitudePreserveSkipped"] = "area_redundant_entry_removed"
    else:
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
    if not skip_entry_altitude_preserve:
        _stabilize_entry_transition_altitude_inplace(
            final_waypoints,
            entry_coord=entry_coord,
            default_speed_mps=float(transit_speed_mps),
        )
    metrics["stabilizeFirstMs"] = _elapsed_ms(step_started)
    step_started = time.perf_counter()
    removed_interpolation_coords = _collapse_linesearch_midpoints_inplace(final_waypoints)
    metrics["lineSweepInterpolationEnabled"] = _line_sweep_interpolation_enabled()
    metrics["lineSweepInterpolationRemovedCoords"] = int(removed_interpolation_coords)
    metrics["lineSweepInterpolationNormalizeMs"] = _elapsed_ms(step_started)
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
    if not skip_entry_altitude_preserve:
        _stabilize_entry_transition_altitude_inplace(
            final_waypoints,
            entry_coord=entry_coord,
            default_speed_mps=float(transit_speed_mps),
        )
    metrics["stabilizeSecondMs"] = _elapsed_ms(step_started)
    turn_contract_started = time.perf_counter()
    reciprocal_turn_count, reciprocal_turn_altitude_m = (
        _enforce_area_reciprocal_turn_contract_inplace(final_waypoints)
    )
    if reciprocal_turn_count:
        # Leveling the whole turn can move the required climb/descent envelope
        # to its neighbours.  Two raise-only propagation passes settle that
        # envelope without ever lowering the terrain-safe corridor altitude.
        for _ in range(2):
            _enforce_waypoint_altitude_rate_limit_inplace(
                final_waypoints,
                default_speed_mps=float(transit_speed_mps),
            )
            reciprocal_turn_count, reciprocal_turn_altitude_m = (
                _enforce_area_reciprocal_turn_contract_inplace(final_waypoints)
            )
        metrics["areaReciprocalTurnContractWaypoints"] = int(reciprocal_turn_count)
        metrics["areaReciprocalTurnFinalAltitudeM"] = (
            round(float(reciprocal_turn_altitude_m), 1)
            if reciprocal_turn_altitude_m is not None
            else None
        )
    metrics["areaReciprocalTurnContractMs"] = _elapsed_ms(turn_contract_started)
    metrics["postAltitudeMs"] = _elapsed_ms(altitude_post_started)
    # 경로·고도·촬영점(DEM 정규화)이 전부 확정된 시점에서, 실제 WP↔촬영점
    # 기하로 요구공간해상도를 전수 검증하고 초과 시 임무 FOV를 내린다.
    step_started = time.perf_counter()
    try:
        gsd_summary = capture_physics.certify_waypoint_gsd_inplace(
            final_waypoints,
            mission_info if isinstance(mission_info, dict) else None,
        )
    except Exception:
        gsd_summary = {"error": "certify_failed"}
    metrics["gsdCertifyMs"] = _elapsed_ms(step_started)
    metrics["gsdCertify"] = {
        key: gsd_summary.get(key)
        for key in (
            "checked",
            "worstSlantM",
            "worstAglM",
            "worstAreaM2",
            "requiredAreaM2",
            "fovBeforeDeg",
            "fovAfterDeg",
            "clamped",
            "unreachable",
            "skipped",
            "error",
        )
        if key in gsd_summary
    }
    # Altitude-rate limiting and filming-target DEM normalization above can
    # change both the final 3D sweep length and the real incoming WP leg after
    # the earlier geometry pass. Recompute from the payload that will actually
    # be emitted so neither LINE nor an AREA fallback loses its timing margin.
    final_capture_sync_started = time.perf_counter()
    if is_line_mission:
        final_capture_sync_count = _resynchronize_line_capture_speeds_inplace(
            final_waypoints,
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
            default_transit_speed_mps=float(transit_speed_mps),
            multiplier_cap_enabled=bool(line_search_multiplier_cap_enabled),
            activation_entry_coord=(
                entry_coord if isinstance(entry_coord, dict) else None
            ),
            activation_delay_s=float(first_capture_activation_delay_s),
        )
        metrics["lineCaptureSpeedsResynced3D"] = int(
            final_capture_sync_count
        )
    else:
        final_capture_sync_count = _resynchronize_area_capture_speeds_inplace(
            final_waypoints,
            fallback_search_speed_mps=float(search_speed_mps),
            speed_scale=float(geometry_search_speed_scale),
            default_transit_speed_mps=float(transit_speed_mps),
            activation_entry_coord=(
                entry_coord if isinstance(entry_coord, dict) else None
            ),
            activation_delay_s=float(area_first_capture_activation_delay_s),
        )
        metrics["areaCaptureSpeedsResynced3D"] = int(
            final_capture_sync_count
        )
        if bool(path_row.get("areaSingleAircraftSequentialSplit")):
            metrics["areaSequentialCaptureSpeedsResynced"] = int(
                final_capture_sync_count
            )
    metrics["finalCaptureSpeedSyncMs"] = _elapsed_ms(
        final_capture_sync_started
    )
    waypoint_id_started = time.perf_counter()
    if assign_waypoint_ids:
        reassign_unique_waypoint_ids_inplace(
            final_waypoints,
            waypoint_id_provider=waypoint_id_provider,
        )
    else:
        metrics["waypointIdsDeferred"] = True
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
    _clear_handover_terminal_markers(final_waypoints)
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
    try:
        size_estimate = estimate_from_metrics(metrics)
        metrics["lineSearchEstimateWarn"] = bool(size_estimate.isWarn)
        metrics["lineSearchEstimateHeavy"] = bool(size_estimate.isHeavy)
        metrics["lineSearchEstimatedCriticalPathMs"] = size_estimate.estimatedCriticalPathMs
        metrics["lineSearchEstimateReason"] = size_estimate.reason
    except Exception:
        pass
    metrics["buildTotalMs"] = _elapsed_ms(build_started)
    if metrics_callback is not None:
        try:
            metrics_callback(dict(metrics))
        except Exception:
            pass
    return flight_path
